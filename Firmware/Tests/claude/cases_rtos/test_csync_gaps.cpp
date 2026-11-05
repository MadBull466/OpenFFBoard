/*
 * Remaining gaps of cpp_freertos::Mutex* (FFBoard/Src/cmutex.cpp, FFBoard/Inc/mutex.hpp) and cpp_freertos::*Semaphore
 * (FFBoard/Src/csemaphore.cpp, FFBoard/Inc/semaphore.hpp) on the fake FreeRTOS API F11 (support/HostRtosExt.h); tier A
 * vendor code: suspected defects are reported, never fixed. The basic lock / unlock / take / give behaviour is tested
 * in claude/cases/test_rtos_wrappers.cpp (ffb_unit_tests); this file adds create failures, timeouts against simulated time,
 * contention with resumable tasks, mutex ownership and the ISR variants.
 *
 * Host vs target: the create failures throw on the host; the F407 build defines CPP_FREERTOS_NO_EXCEPTIONS and stops
 * in configASSERT instead (NEEDS REVIEW 142). Vendor V1 and V3 below are about these exception paths.
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"

#include "mutex.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

#include <cstdint>
#include <cstring>
#include <functional>
#include <new>
#include <string>

using namespace cpp_freertos;
using HostRtos::RtosObject;

namespace {

/** Thread running a function set by the test (resumable: continues after a blocking call) */
class SyncTask : public Thread {
public:
	explicit SyncTask(std::function<void()> body, const std::string &name, UBaseType_t priority = 1)
		: Thread(name, 128, priority), body(std::move(body)) {}

	TaskHandle_t startResumable() {
		REQUIRE(Start());
		HostRtos::makeResumable(GetHandle());
		return GetHandle();
	}

protected:
	void Run() override { body(); }

private:
	std::function<void()> body;
};

/**
 * Storage for constructing a CountingSemaphore with a known content of the members the constructor leaves
 * uninitialised (vendor V1): placement new into a buffer filled with 0xA5.
 */
struct PatternStorage {
	alignas(CountingSemaphore) unsigned char bytes[sizeof(CountingSemaphore)];
	PatternStorage() { std::memset(bytes, 0xA5, sizeof(bytes)); }
};

} // namespace

TEST_SUITE("Mutex") {

TEST_CASE_FIXTURE(RtosFixture, "a standard mutex whose kernel mutex can not be created throws MutexCreateException") {
	HostRtos::failCreate(RtosObject::Mutex);
	CHECK_THROWS_WITH_AS(MutexStandard(), "Mutex Constructor Failed", MutexCreateException);
	(void)expectKernelFault("vSemaphoreDelete(NULL)"); // vendor V3, see the KNOWN_ISSUE below
}

TEST_CASE_FIXTURE(RtosFixture, "a recursive mutex whose kernel mutex can not be created throws MutexCreateException") {
	HostRtos::failCreate(RtosObject::RecursiveMutex);
	CHECK_THROWS_WITH_AS(MutexRecursive(), "Mutex Constructor Failed", MutexCreateException);
	CHECK(HostRtos::createCalls(RtosObject::RecursiveMutex) == 1);
	(void)expectKernelFault("vSemaphoreDelete(NULL)"); // vendor V3, see the KNOWN_ISSUE below
}

TEST_CASE_FIXTURE(RtosFixture, "a task locking a standard mutex with a timeout gets false when the holder keeps it") {
	MutexStandard mutex;
	REQUIRE(mutex.Lock(0));
	bool returned = false;
	bool locked = true;
	SyncTask task([&] {
		locked = mutex.Lock(5);
		returned = true;
	}, "MTX");
	task.startResumable();
	runReady();
	advance(4);
	CHECK_FALSE(returned);
	advance(1);
	CHECK(returned);
	CHECK_FALSE(locked);
	CHECK(mutex.Unlock());
}

TEST_CASE_FIXTURE(RtosFixture, "a task locking a standard mutex with a timeout gets it when it is unlocked in time") {
	MutexStandard mutex;
	REQUIRE(mutex.Lock(0));
	bool locked = false;
	SyncTask task([&] {
		locked = mutex.Lock(5);
		if (locked) {
			mutex.Unlock();
		}
	}, "MTX");
	task.startResumable();
	runReady();
	advance(2);
	CHECK(mutex.Unlock());
	runReady();
	CHECK(locked);
}

TEST_CASE_FIXTURE(RtosFixture, "unlocking a standard mutex held by another task stops in configASSERT") {
	MutexStandard mutex;
	BinarySemaphore gate;
	SyncTask task([&] {
		mutex.Lock();
		gate.Take();
		mutex.Unlock();
	}, "MTX");
	task.startResumable();
	runReady();
	(void)mutex.Unlock(); // Not the holder (queue.c xQueueGenericSend -> xTaskPriorityDisinherit asserts)
	CHECK(expectKernelFault("does not hold"));
	gate.Give();
	runReady();
}

TEST_CASE_FIXTURE(RtosFixture, "a recursive mutex held by a task can not be locked by another caller within the timeout") {
	MutexRecursive mutex;
	BinarySemaphore gate;
	SyncTask task([&] {
		mutex.Lock();
		gate.Take();
		mutex.Unlock();
	}, "MTX");
	task.startResumable();
	runReady();
	CHECK_FALSE(mutex.Lock(3)); // The test thread is another caller
	CHECK(HostPlatform::nowUs() == 3000);
	gate.Give();
	runReady();
	CHECK(mutex.Lock(0));
	CHECK(mutex.Unlock());
}

TEST_CASE_FIXTURE(RtosFixture, "a recursive mutex held by a task can not be unlocked by another caller") {
	MutexRecursive mutex;
	BinarySemaphore gate;
	SyncTask task([&] {
		mutex.Lock();
		gate.Take();
		mutex.Unlock();
	}, "MTX");
	task.startResumable();
	runReady();
	CHECK_FALSE(mutex.Unlock());
	CHECK_FALSE(mutex.Lock(0)); // Still held by the task
	gate.Give();
	runReady();
}

TEST_CASE_FIXTURE(RtosFixture, "a task waiting for a recursive mutex continues when the holder unlocked it as often as it locked it") {
	MutexRecursive mutex;
	REQUIRE(mutex.Lock(0));
	REQUIRE(mutex.Lock(0));
	bool locked = false;
	SyncTask task([&] {
		locked = mutex.Lock();
		mutex.Unlock();
	}, "MTX");
	TaskHandle_t handle = task.startResumable();
	runReady();
	CHECK(mutex.Unlock());
	runReady();
	CHECK_FALSE(locked);
	CHECK(HostRtos::isWaiting(handle));
	CHECK(mutex.Unlock());
	runReady();
	CHECK(locked);
}

TEST_CASE_FIXTURE(RtosFixture, "a recursive mutex locked several times by a task is free after as many unlocks in that task") {
	MutexRecursive mutex;
	bool done = false;
	SyncTask task([&] {
		mutex.Lock();
		mutex.Lock(0);
		{
			LockGuard guard(mutex);
		}
		mutex.Unlock();
		mutex.Unlock();
		done = true;
	}, "MTX");
	task.startResumable();
	runReady();
	REQUIRE(done);
	CHECK(mutex.Lock(0));
	CHECK(mutex.Unlock());
}

// ---------------- Suspected vendor defects (tier A: reported, never fixed) ----------------

TEST_CASE_FIXTURE(RtosFixture, "a standard mutex whose creation fails does not delete a NULL handle"
				  * KNOWN_ISSUE("vendor V3: MutexStandard throws (cmutex.cpp:61-63) with handle NULL; the base destructor "
								"~Mutex (cmutex.cpp:53) then calls vSemaphoreDelete(NULL), on which FreeRTOS stops in "
								"configASSERT (queue.c:1979). The first failing assertion is CHECK_FALSE(deletedNull)")) {
	HostRtos::failCreate(RtosObject::Mutex);
	CHECK_THROWS_AS(MutexStandard(), MutexCreateException);
	const bool deletedNull = expectKernelFault("vSemaphoreDelete(NULL)");
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK_FALSE(deletedNull);
}

TEST_CASE_FIXTURE(RtosFixture, "a recursive mutex whose creation fails does not delete a NULL handle"
				  * KNOWN_ISSUE("vendor V3: MutexRecursive throws (cmutex.cpp:91-93) with handle NULL; the base destructor "
								"~Mutex (cmutex.cpp:53) then calls vSemaphoreDelete(NULL) (configASSERT, queue.c:1979). The "
								"first failing assertion is CHECK_FALSE(deletedNull)")) {
	HostRtos::failCreate(RtosObject::RecursiveMutex);
	CHECK_THROWS_AS(MutexRecursive(), MutexCreateException);
	const bool deletedNull = expectKernelFault("vSemaphoreDelete(NULL)");
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK_FALSE(deletedNull);
}

}

TEST_SUITE("Semaphore") {

TEST_CASE_FIXTURE(RtosFixture, "a binary semaphore whose kernel semaphore can not be created throws SemaphoreCreateException") {
	HostRtos::failCreate(RtosObject::BinarySemaphore);
	CHECK_THROWS_WITH_AS(BinarySemaphore(true), "Semaphore Constructor Failed", SemaphoreCreateException);
	(void)expectKernelFault("vSemaphoreDelete(NULL)"); // vendor V3, see the KNOWN_ISSUE below
}

TEST_CASE_FIXTURE(RtosFixture, "a counting semaphore whose kernel semaphore can not be created throws SemaphoreCreateException") {
	HostRtos::failCreate(RtosObject::CountingSemaphore);
	CHECK_THROWS_WITH_AS(CountingSemaphore(3, 1), "Semaphore Constructor Failed", SemaphoreCreateException);
	CHECK(HostRtos::createCalls(RtosObject::CountingSemaphore) == 1);
	(void)expectKernelFault("vSemaphoreDelete(NULL)"); // vendor V3, see the KNOWN_ISSUE below
}

TEST_CASE_FIXTURE(RtosFixture, "taking an empty semaphore on the test thread with a timeout returns false after the timeout of simulated time") {
	BinarySemaphore sem;
	CHECK_FALSE(sem.Take(7));
	CHECK(HostPlatform::nowUs() == 7000);
}

TEST_CASE_FIXTURE(RtosFixture, "a task taking an empty semaphore with a timeout gets false when nobody gives it in time") {
	CountingSemaphore sem(2, 0);
	bool returned = false;
	bool taken = true;
	SyncTask task([&] {
		taken = sem.Take(5);
		returned = true;
	}, "SEM");
	task.startResumable();
	runReady();
	advance(4);
	CHECK_FALSE(returned);
	advance(1);
	CHECK(returned);
	CHECK_FALSE(taken);
}

TEST_CASE_FIXTURE(RtosFixture, "a task taking a semaphore with a timeout gets it when it is given before the timeout") {
	BinarySemaphore sem;
	bool taken = false;
	SyncTask task([&] { taken = sem.Take(5); }, "SEM");
	task.startResumable();
	runReady();
	advance(3);
	CHECK(sem.Give());
	runReady();
	CHECK(taken);
	CHECK(HostPlatform::nowUs() == 3000);
}

TEST_CASE_FIXTURE(RtosFixture, "GiveFromISR reports a higher priority task woken when it releases a waiting task of higher priority") {
	BinarySemaphore sem;
	bool taken = false;
	SyncTask task([&] { taken = sem.Take(); }, "SEM", 3);
	task.startResumable();
	runReady();
	BaseType_t woken = pdFALSE;
	{
		HostPlatform::IsrContext isr;
		CHECK(sem.GiveFromISR(&woken));
	}
	CHECK(woken == pdTRUE);
	runReady();
	CHECK(taken);
}

TEST_CASE_FIXTURE(RtosFixture, "GiveFromISR fails on a full counting semaphore and succeeds again after a take") {
	CountingSemaphore sem(2, 2);
	BaseType_t woken = pdFALSE;
	HostPlatform::IsrContext isr;
	CHECK_FALSE(sem.GiveFromISR(&woken));
	CHECK(sem.TakeFromISR(&woken));
	CHECK(sem.GiveFromISR(&woken));
}

TEST_CASE_FIXTURE(RtosFixture, "TakeFromISR takes a counting semaphore once per count") {
	CountingSemaphore sem(3, 2);
	BaseType_t woken = pdFALSE;
	HostPlatform::IsrContext isr;
	CHECK(sem.TakeFromISR(&woken));
	CHECK(sem.TakeFromISR(&woken));
	CHECK_FALSE(sem.TakeFromISR(&woken));
}

// ---------------- Suspected vendor defects (tier A: reported, never fixed) ----------------

TEST_CASE_FIXTURE(RtosFixture, "a binary semaphore whose creation fails does not delete a NULL handle"
				  * KNOWN_ISSUE("vendor V3: BinarySemaphore throws (csemaphore.cpp:101-103) with handle NULL; the base "
								"destructor ~Semaphore (csemaphore.cpp:93) then calls vSemaphoreDelete(NULL) (configASSERT, "
								"queue.c:1979). The first failing assertion is CHECK_FALSE(deletedNull)")) {
	HostRtos::failCreate(RtosObject::BinarySemaphore);
	CHECK_THROWS_AS(BinarySemaphore(), SemaphoreCreateException);
	const bool deletedNull = expectKernelFault("vSemaphoreDelete(NULL)");
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK_FALSE(deletedNull);
}

TEST_CASE_FIXTURE(RtosFixture, "a counting semaphore whose creation fails does not delete a NULL handle"
				  * KNOWN_ISSUE("vendor V3: CountingSemaphore throws (csemaphore.cpp:135-137) with handle NULL; the base "
								"destructor ~Semaphore (csemaphore.cpp:93) then calls vSemaphoreDelete(NULL) (configASSERT, "
								"queue.c:1979). The first failing assertion is CHECK_FALSE(deletedNull)")) {
	HostRtos::failCreate(RtosObject::CountingSemaphore);
	CHECK_THROWS_AS(CountingSemaphore(3, 1), SemaphoreCreateException);
	const bool deletedNull = expectKernelFault("vSemaphoreDelete(NULL)");
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK_FALSE(deletedNull);
}

TEST_CASE_FIXTURE(RtosFixture, "a counting semaphore with a maximum count of 0 is rejected without deleting an unrelated handle"
				  * KNOWN_ISSUE_UB("vendor V1: CountingSemaphore throws for maxCount 0 (csemaphore.cpp:117-119) before "
								   "`handle` is assigned; ~Semaphore (csemaphore.cpp:93) then passes the uninitialised handle "
								   "to vSemaphoreDelete (UB; the fake reports a deleted semaphore for the 0xA5 pattern). The "
								   "vSemaphoreDelete(NULL) of V3 is ignored here. The first failing assertion is "
								   "CHECK_FALSE(deletedUnrelated)")) {
	PatternStorage storage;
	CHECK_THROWS_WITH_AS(new (storage.bytes) CountingSemaphore(0, 0), "Semaphore Constructor Failed bad maxCount",
						 SemaphoreCreateException);
	CHECK(HostRtos::createCalls(RtosObject::CountingSemaphore) == 0); // Rejected before the kernel call
	(void)expectKernelFault("vSemaphoreDelete(NULL)");				 // V3 (a fix of V1 alone may leave the handle NULL)
	const bool deletedUnrelated = expectKernelFault("vSemaphoreDelete: handle of a deleted semaphore");
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK_FALSE(deletedUnrelated);
}

TEST_CASE_FIXTURE(RtosFixture, "a counting semaphore with an initial count above its maximum is rejected without deleting an unrelated handle"
				  * KNOWN_ISSUE_UB("vendor V1: CountingSemaphore throws for initialCount > maxCount (csemaphore.cpp:125-127) "
								   "before `handle` is assigned; ~Semaphore (csemaphore.cpp:93) then passes the uninitialised "
								   "handle to vSemaphoreDelete (UB). The first failing assertion is CHECK_FALSE(deletedUnrelated)")) {
	PatternStorage storage;
	CHECK_THROWS_WITH_AS(new (storage.bytes) CountingSemaphore(2, 3), "Semaphore Constructor Failed bad initialCount",
						 SemaphoreCreateException);
	CHECK(HostRtos::createCalls(RtosObject::CountingSemaphore) == 0);
	(void)expectKernelFault("vSemaphoreDelete(NULL)");
	const bool deletedUnrelated = expectKernelFault("vSemaphoreDelete: handle of a deleted semaphore");
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK_FALSE(deletedUnrelated);
}

}
