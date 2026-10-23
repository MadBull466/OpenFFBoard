/*
 * FreeRTOS C++ wrappers (vendor code, tier A): cthread.cpp, cmutex.cpp, csemaphore.cpp
 *
 * Tested on the non-preemptive fake kernel (platform/src/host_freertos.cpp). Only what such a fake
 * can express is tested: results of take/give/lock/unlock, blocking of a thread that waits forever,
 * delays against simulated time and task creation/deletion. Priority inheritance, ownership of
 * recursive mutexes by another task and real timeouts can not be expressed and are not tested.
 *
 * Host vs target: the host build uses the exception variant of the wrappers. The target defines
 * CPP_FREERTOS_NO_EXCEPTIONS (Targets/F407VG/Core/Inc/FreeRTOSConfig.h) and halts in configASSERT
 * instead of throwing, so the exception paths are host only and are not tested here.
 */
#include "doctest.h"
#include "HostPlatform.h"
#include "HostRtos.h"
#include "thread.hpp"
#include "mutex.hpp"
#include "semaphore.hpp"

#include <functional>
#include <vector>

using namespace cpp_freertos;

namespace {

struct RtosFixture {
	RtosFixture() {
		HostPlatform::reset();
		HostRtos::reset();
	}
	~RtosFixture() {
		HostRtos::reset();
		HostPlatform::reset();
	}
};

/** Thread running a function set by the test. Gives the test access to the protected delay functions */
class FunctionThread : public Thread {
public:
	explicit FunctionThread(std::function<void(FunctionThread &)> body, const std::string &name = "FN")
		: Thread(name, 64, 2), body(std::move(body)) {}
	FunctionThread(std::function<void(FunctionThread &)> body, uint16_t stack, UBaseType_t prio)
		: Thread(stack, prio), body(std::move(body)) {}

	void delayUntil(TickType_t period) { DelayUntil(period); }
	void resetDelayUntil() { ResetDelayUntil(); }

	std::function<void(FunctionThread &)> body;

protected:
	void Run() override { body(*this); }
};

/** Thread overriding Cleanup */
class CleanupThread : public Thread {
public:
	CleanupThread() : Thread("CLEAN", 64, 1) {}
	uint32_t runs = 0, cleanups = 0;
	size_t tasksDuringCleanup = 0;

protected:
	void Run() override { runs++; }
	void Cleanup() override {
		cleanups++;
		tasksDuringCleanup = HostRtos::taskCount(); // Task is deleted after Cleanup
	}
};

/** Runs Thread::StartScheduler for the lifetime of the object and always ends it again */
struct SchedulerRunning {
	SchedulerRunning() { Thread::StartScheduler(); }
	~SchedulerRunning() { Thread::EndScheduler(); }
};

} // namespace

TEST_SUITE("cpp_freertos") {

// ---------------- Thread ----------------

TEST_CASE_FIXTURE(RtosFixture, "thread created without a name is called Default") {
	FunctionThread thread([](FunctionThread &) {}, 128, 3);
	CHECK(thread.GetName() == "Default");
	REQUIRE(thread.Start());
	CHECK(std::string(pcTaskGetName(thread.GetHandle())) == "Default");
	CHECK(thread.GetPriority() == 3);
}

TEST_CASE_FIXTURE(RtosFixture, "named thread registers its task with name and priority") {
	FunctionThread thread([](FunctionThread &) {}, "NAMED");
	const size_t before = HostRtos::taskCount();
	REQUIRE(thread.Start());
	CHECK(HostRtos::taskCount() == before + 1);
	CHECK(std::string(pcTaskGetName(thread.GetHandle())) == "NAMED");
	CHECK(thread.GetPriority() == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "a thread can only be started once") {
	FunctionThread thread([](FunctionThread &) {});
	const size_t before = HostRtos::taskCount();
	CHECK(thread.Start());
	CHECK_FALSE(thread.Start());
	CHECK(HostRtos::taskCount() == before + 1); // No second task
}

TEST_CASE_FIXTURE(RtosFixture, "a thread started while the scheduler runs can only be started once") {
	FunctionThread thread([](FunctionThread &) {});
	const size_t before = HostRtos::taskCount();
	{
		SchedulerRunning scheduler;
		CHECK(thread.Start());
		CHECK_FALSE(thread.Start());
	}
	CHECK(HostRtos::taskCount() == before + 1);
}

TEST_CASE_FIXTURE(RtosFixture, "the start guard lock is released after starting a thread while the scheduler runs") {
	// Thread::Start locks the static StartGuardLock with a LockGuard. If it was not released a second
	// thread started from a thread context would block forever on the lock
	FunctionThread first([](FunctionThread &) {}, "FIRST");
	bool secondStarted = false;
	FunctionThread second([](FunctionThread &) {}, "SECOND");
	FunctionThread starter([&](FunctionThread &) { secondStarted = second.Start(); }, "STARTER");
	SchedulerRunning scheduler;
	REQUIRE(first.Start());
	REQUIRE(starter.Start());
	HostRtos::runUntilBlocked(starter);
	CHECK(secondStarted);
}

// Vendor issue (tier A, report only, PROGRESS V4): after Run returns, TaskFunctionAdapter deletes the task
// (cthread.cpp:211), but Thread::handle keeps the old value. ~Thread (cthread.cpp:186) then calls
// vTaskDelete(handle) a second time on the already deleted task: double delete / use after free on the
// target. The fake only compares the pointer with its task list and ignores it, so the two tests below
// pass on the host. Related (V2): Thread::handle (thread.hpp:412) is uninitialised until Start(), so
// destroying a thread that was never started passes an indeterminate handle to vTaskDelete. No test
// destroys an unstarted thread for that reason.
TEST_CASE_FIXTURE(RtosFixture, "a thread whose Run returns calls Cleanup and deletes its task") {
	CleanupThread thread;
	REQUIRE(thread.Start());
	const size_t withTask = HostRtos::taskCount();
	HostRtos::runUntilBlocked(thread.GetHandle());
	CHECK(thread.runs == 1);
	CHECK(thread.cleanups == 1);
	CHECK(thread.tasksDuringCleanup == withTask);
	CHECK(HostRtos::taskCount() == withTask - 1);
}

TEST_CASE_FIXTURE(RtosFixture, "a thread with the default Cleanup deletes its task when Run returns") {
	uint32_t runs = 0;
	FunctionThread thread([&](FunctionThread &) { runs++; });
	REQUIRE(thread.Start());
	const size_t withTask = HostRtos::taskCount();
	HostRtos::runUntilBlocked(thread);
	CHECK(runs == 1);
	CHECK(HostRtos::taskCount() == withTask - 1);
}

TEST_CASE_FIXTURE(RtosFixture, "destroying a started thread deletes its task") {
	const size_t before = HostRtos::taskCount();
	{
		FunctionThread thread([](FunctionThread &) {});
		REQUIRE(thread.Start());
		CHECK(HostRtos::taskCount() == before + 1);
	}
	CHECK(HostRtos::taskCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "DelayUntil keeps a fixed period independent of the work time") {
	std::vector<TickType_t> wakeups;
	FunctionThread thread([&](FunctionThread &self) {
		while (true) {
			wakeups.push_back(xTaskGetTickCount());
			HostPlatform::advanceMs(3); // Work takes 3 ms
			self.delayUntil(10);
		}
	});
	REQUIRE(thread.Start());
	HostPlatform::setTimeMs(100);
	HostRtos::setDelayBudgetMs(30); // Delays of 10, 7 and 7 ms fit, the 4th delay (7 ms) exceeds the remaining 6 ms and blocks

	HostRtos::runUntilBlocked(thread);
	// The first DelayUntil takes its reference time when it is called (103), then wakes every 10 ms
	CHECK(wakeups == std::vector<TickType_t>{100, 113, 123, 133});
}

// This and the ResetDelayUntil test rely on the fake restarting Run() from the beginning on every
// runUntilBlocked (a blocked delay leaves Run by an exception). The thread keeps its DelayUntil state
// between activations like a real task keeps it across a delay, so the observed wake up times are the
// ones a real kernel would produce for the same tick sequence.
TEST_CASE_FIXTURE(RtosFixture, "DelayUntil does not wait if the wake up time already passed") {
	std::vector<TickType_t> wakeups;
	FunctionThread thread([&](FunctionThread &self) {
		while (true) {
			wakeups.push_back(xTaskGetTickCount());
			self.delayUntil(10);
		}
	});
	REQUIRE(thread.Start());
	HostRtos::setDelayBudgetMs(15);
	HostRtos::runUntilBlocked(thread); // wakes at 0, 10, blocked in the delay to 20
	REQUIRE(wakeups == std::vector<TickType_t>{0, 10});
	CHECK(xTaskGetTickCount() == 20);

	// The thread is late by 25 ms. The missed periods 30 and 40 are run immediately to catch up
	HostPlatform::setTimeMs(45);
	wakeups.clear();
	HostRtos::setDelayBudgetMs(6); // allow the delay to 50 (5 ms) but block at the next one
	HostRtos::runUntilBlocked(thread);
	CHECK(wakeups == std::vector<TickType_t>{45, 45, 45, 50});
}

TEST_CASE_FIXTURE(RtosFixture, "ResetDelayUntil makes the next DelayUntil start a new period from the current time") {
	std::vector<TickType_t> wakeups;
	FunctionThread thread([&](FunctionThread &self) {
		self.resetDelayUntil(); // Every activation starts a new period
		while (true) {
			wakeups.push_back(xTaskGetTickCount());
			self.delayUntil(10);
		}
	});
	REQUIRE(thread.Start());
	HostRtos::setDelayBudgetMs(15);
	HostRtos::runUntilBlocked(thread);
	REQUIRE(wakeups == std::vector<TickType_t>{0, 10});

	HostPlatform::setTimeMs(45);
	wakeups.clear();
	HostRtos::runUntilBlocked(thread);
	CHECK(wakeups == std::vector<TickType_t>{45, 55}); // No catching up of missed periods
	CHECK(xTaskGetTickCount() == 65);
}

// ---------------- Mutex ----------------

TEST_CASE_FIXTURE(RtosFixture, "standard mutex can be locked once and unlocked once") {
	MutexStandard mutex;
	CHECK(mutex.Lock());
	CHECK_FALSE(mutex.Lock(0)); // Already locked
	CHECK(mutex.Unlock());
	CHECK_FALSE(mutex.Unlock()); // Not locked
	CHECK(mutex.Lock(0));
	CHECK(mutex.Unlock());
}

TEST_CASE_FIXTURE(RtosFixture, "locking a held standard mutex with a timeout fails") {
	MutexStandard mutex;
	REQUIRE(mutex.Lock());
	CHECK_FALSE(mutex.Lock(10));
	CHECK(mutex.Unlock());
}

TEST_CASE_FIXTURE(RtosFixture, "a thread locking a held mutex without timeout blocks until it is unlocked") {
	MutexStandard mutex;
	bool gotLock = false;
	FunctionThread thread([&](FunctionThread &) {
		gotLock = mutex.Lock(); // portMAX_DELAY
		mutex.Unlock();
	});
	REQUIRE(thread.Start());
	REQUIRE(mutex.Lock());

	HostRtos::runUntilBlocked(thread);
	CHECK_FALSE(gotLock); // Waits for the mutex

	REQUIRE(mutex.Unlock());
	HostRtos::runUntilBlocked(thread);
	CHECK(gotLock);
	CHECK(mutex.Lock(0)); // Released again by the thread
}

TEST_CASE_FIXTURE(RtosFixture, "recursive mutex needs as many unlocks as locks") {
	MutexRecursive mutex;
	CHECK(mutex.Lock());
	CHECK(mutex.Lock());
	CHECK(mutex.Lock(0));
	CHECK(mutex.Unlock());
	CHECK(mutex.Unlock());
	CHECK(mutex.Unlock());
	CHECK_FALSE(mutex.Unlock()); // Not held anymore
}

TEST_CASE_FIXTURE(RtosFixture, "unlocking a recursive mutex that is not locked fails") {
	MutexRecursive mutex;
	CHECK_FALSE(mutex.Unlock());
}

TEST_CASE_FIXTURE(RtosFixture, "lock guard holds the mutex for its scope") {
	MutexStandard mutex;
	{
		LockGuard guard(mutex);
		CHECK_FALSE(mutex.Lock(0));
	}
	CHECK(mutex.Lock(0));
	CHECK(mutex.Unlock());
}

TEST_CASE_FIXTURE(RtosFixture, "lock guards can be nested on a recursive mutex") {
	MutexRecursive mutex;
	{
		LockGuard outer(mutex);
		{
			LockGuard inner(mutex);
		}
		CHECK(mutex.Unlock()); // Still held once by the outer guard
		CHECK(mutex.Lock());   // Give it back to the outer guard
	}
	CHECK_FALSE(mutex.Unlock()); // Released by the outer guard
}

TEST_CASE_FIXTURE(RtosFixture, "mutexes create a kernel semaphore and delete it when destroyed") {
	const size_t before = HostRtos::semaphoreCount();
	{
		MutexStandard standard;
		CHECK(HostRtos::semaphoreCount() == before + 1);
		MutexRecursive recursive;
		CHECK(HostRtos::semaphoreCount() == before + 2);
	}
	CHECK(HostRtos::semaphoreCount() == before);
}

// ---------------- Semaphore ----------------

TEST_CASE_FIXTURE(RtosFixture, "binary semaphore can be created given or empty") {
	BinarySemaphore given(true);
	CHECK(given.Take(0));
	CHECK_FALSE(given.Take(0));
	BinarySemaphore empty;
	CHECK_FALSE(empty.Take(0));
}

TEST_CASE_FIXTURE(RtosFixture, "TakeFromISR takes an available semaphore once") {
	BinarySemaphore sem(true);
	BaseType_t woken = pdTRUE;
	{
		HostPlatform::IsrContext isr;
		CHECK(sem.TakeFromISR(&woken));
		// Only shows that the wrapper forwards the pointer to xSemaphoreTakeFromISR (the fake writes pdFALSE
		// into it); the value itself says nothing about a real kernel
		CHECK(woken == pdFALSE);
		CHECK_FALSE(sem.TakeFromISR(&woken));
	}
	CHECK_FALSE(sem.Take(0));
}

TEST_CASE_FIXTURE(RtosFixture, "a semaphore given from an ISR can be taken from an ISR") {
	BinarySemaphore sem;
	BaseType_t woken = pdFALSE;
	HostPlatform::IsrContext isr;
	CHECK(sem.GiveFromISR(&woken));
	CHECK_FALSE(sem.GiveFromISR(&woken)); // Binary: already given
	CHECK(sem.TakeFromISR(&woken));
}

// Not tested: the argument checks of CountingSemaphore (maxCount == 0, initialCount > maxCount,
// csemaphore.cpp:117-131). In this exception build they throw before handle is assigned, and the base
// destructor ~Semaphore then calls vSemaphoreDelete on the uninitialised handle (vendor issue V1,
// undefined behaviour; it crashed the run in the mutation check). On the target (CPP_FREERTOS_NO_EXCEPTIONS)
// they halt in configASSERT instead.
TEST_CASE_FIXTURE(RtosFixture, "counting semaphore starts with the initial count") {
	CountingSemaphore sem(5, 2);
	CHECK(sem.Take(0));
	CHECK(sem.Take(0));
	CHECK_FALSE(sem.Take(0));
}

TEST_CASE_FIXTURE(RtosFixture, "counting semaphore can be given up to its maximum count") {
	CountingSemaphore sem(3, 0);
	CHECK_FALSE(sem.Take(0));
	CHECK(sem.Give());
	CHECK(sem.Give());
	CHECK(sem.Give());
	CHECK_FALSE(sem.Give()); // Full
	CHECK(sem.Take(0));
	CHECK(sem.Take(0));
	CHECK(sem.Take(0));
	CHECK_FALSE(sem.Take(0));
}

TEST_CASE_FIXTURE(RtosFixture, "counting semaphore may start full") {
	CountingSemaphore sem(2, 2);
	CHECK_FALSE(sem.Give());
	CHECK(sem.Take(0));
	CHECK(sem.Take(0));
	CHECK_FALSE(sem.Take(0));
}

TEST_CASE_FIXTURE(RtosFixture, "counting semaphore with a maximum count of 1 behaves like a binary semaphore") {
	CountingSemaphore sem(1, 0);
	CHECK(sem.Give());
	CHECK_FALSE(sem.Give());
	CHECK(sem.Take(0));
}

TEST_CASE_FIXTURE(RtosFixture, "a thread taking an empty semaphore without timeout blocks until it is given") {
	CountingSemaphore sem(2, 0);
	uint32_t taken = 0;
	FunctionThread thread([&](FunctionThread &) {
		while (true) {
			sem.Take(); // portMAX_DELAY
			taken++;
		}
	});
	REQUIRE(thread.Start());
	HostRtos::runUntilBlocked(thread);
	CHECK(taken == 0);
	sem.Give();
	sem.Give();
	HostRtos::runUntilBlocked(thread);
	CHECK(taken == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "semaphores create a kernel semaphore and delete it when destroyed") {
	const size_t before = HostRtos::semaphoreCount();
	{
		BinarySemaphore binary;
		CHECK(HostRtos::semaphoreCount() == before + 1);
		CountingSemaphore counting(4, 1);
		CHECK(HostRtos::semaphoreCount() == before + 2);
	}
	CHECK(HostRtos::semaphoreCount() == before);
}

} // TEST_SUITE
