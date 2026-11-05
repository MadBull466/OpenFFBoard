/*
 * Tests of cpp_freertos::ConditionVariable (FFBoard/Src/ccondition_variable.cpp, FFBoard/Inc/condition_variable.hpp)
 * together with Thread::Wait (FFBoard/Src/cthread.cpp:244-267), which is the waiting side of the condition variable;
 * tier A vendor code: suspected defects are reported, never fixed. Fake FreeRTOS API F11 (support/HostRtosExt.h).
 *
 * CPP_FREERTOS_CONDITION_VARIABLES is defined for the F407 target and in the fake FreeRTOSConfig.h. No shipped firmware
 * uses condition variables.
 *
 * Vendor V7 (a timed out Wait stays in the wait list; lost wakeup) is tested in claude/cases_rtos/test_rtos_setup.cpp with
 * Signal; the Broadcast variant (a later Wait of the timed out thread returns at once without a signal) is here.
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"

#include "condition_variable.hpp"
#include "mutex.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

using namespace cpp_freertos;

namespace {

/** Thread running a function set by the test; gives the test access to the protected Thread::Wait */
class CvTask : public Thread {
public:
	explicit CvTask(std::function<void(CvTask &)> body, const std::string &name, UBaseType_t priority = 1)
		: Thread(name, 128, priority), body(std::move(body)) {}

	bool wait(ConditionVariable &cv, Mutex &lock, TickType_t timeout = portMAX_DELAY) { return Wait(cv, lock, timeout); }

	TaskHandle_t startResumable() {
		REQUIRE(Start());
		HostRtos::makeResumable(GetHandle());
		return GetHandle();
	}

protected:
	void Run() override { body(*this); }

private:
	std::function<void(CvTask &)> body;
};

/** Result of one waiter: whether Wait returned and its result */
struct WaitResult {
	bool returned = false;
	bool signalled = false;
};

/** A waiter: lock, Wait(timeout), record, unlock */
std::function<void(CvTask &)> waiter(ConditionVariable &cv, MutexStandard &lock, WaitResult &result, TickType_t timeout = portMAX_DELAY) {
	return [&cv, &lock, &result, timeout](CvTask &self) {
		lock.Lock();
		result.signalled = self.wait(cv, lock, timeout);
		result.returned = true;
		lock.Unlock();
	};
}

} // namespace

TEST_SUITE("ConditionVariable") {

TEST_CASE_FIXTURE(RtosFixture, "a condition variable owns one internal mutex for its lifetime") {
	const size_t before = HostRtos::semaphoreCount();
	auto *cv = new ConditionVariable();
	CHECK(HostRtos::semaphoreCount() == before + 1);
	delete cv;
	CHECK(HostRtos::semaphoreCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "a signal without a waiting thread is lost") {
	ConditionVariable cv;
	MutexStandard lock;
	WaitResult result;
	cv.Signal();
	CvTask task(waiter(cv, lock, result), "CVW");
	TaskHandle_t handle = task.startResumable();
	runReady();
	CHECK_FALSE(result.returned);
	CHECK(HostRtos::isWaiting(handle));
	cv.Signal(); // Let the waiter end
	runReady();
	CHECK(result.signalled);
}

TEST_CASE_FIXTURE(RtosFixture, "a broadcast without a waiting thread is lost") {
	ConditionVariable cv;
	MutexStandard lock;
	WaitResult result;
	cv.Broadcast();
	CvTask task(waiter(cv, lock, result), "CVW");
	task.startResumable();
	runReady();
	CHECK_FALSE(result.returned);
	cv.Signal(); // Let the waiter end
	runReady();
	CHECK(result.signalled);
}

TEST_CASE_FIXTURE(RtosFixture, "Wait releases the lock while the thread waits") {
	ConditionVariable cv;
	MutexStandard lock;
	WaitResult result;
	CvTask task(waiter(cv, lock, result), "CVW");
	task.startResumable();
	runReady();
	REQUIRE_FALSE(result.returned);
	CHECK(lock.Lock(0));
	CHECK(lock.Unlock());
	cv.Signal();
	runReady();
	CHECK(result.signalled);
}

TEST_CASE_FIXTURE(RtosFixture, "a signalled thread holds the lock again when Wait returns") {
	ConditionVariable cv;
	MutexStandard lock;
	BinarySemaphore gate;
	bool signalled = false;
	CvTask task([&](CvTask &self) {
		lock.Lock();
		signalled = self.wait(cv, lock);
		gate.Take(); // Still inside the locked region
		lock.Unlock();
	}, "CVW");
	task.startResumable();
	runReady();
	cv.Signal();
	runReady();
	REQUIRE(signalled);
	CHECK_FALSE(lock.Lock(0));
	gate.Give();
	runReady();
	CHECK(lock.Lock(0));
	CHECK(lock.Unlock());
}

TEST_CASE_FIXTURE(RtosFixture, "a thread signalled while the signaller holds the lock continues once the lock is released") {
	ConditionVariable cv;
	MutexStandard lock;
	WaitResult result;
	CvTask task(waiter(cv, lock, result), "CVW");
	task.startResumable();
	runReady();
	REQUIRE(lock.Lock(0));
	cv.Signal();
	runReady();
	CHECK_FALSE(result.returned); // Woken, but waits for the lock
	CHECK(lock.Unlock());
	runReady();
	CHECK(result.returned);
	CHECK(result.signalled);
}

TEST_CASE_FIXTURE(RtosFixture, "a signal wakes only the thread that waited first") {
	ConditionVariable cv;
	MutexStandard lock;
	WaitResult first;
	WaitResult second;
	CvTask b(waiter(cv, lock, second), "CVB"); // Created first, but waits second
	CvTask a(waiter(cv, lock, first), "CVA");
	a.startResumable();
	runReady();
	b.startResumable();
	runReady();
	cv.Signal();
	runReady();
	CHECK(first.signalled);
	CHECK_FALSE(second.returned);
	cv.Signal();
	runReady();
	CHECK(second.signalled);
}

TEST_CASE_FIXTURE(RtosFixture, "a broadcast wakes every waiting thread") {
	ConditionVariable cv;
	MutexStandard lock;
	WaitResult results[3];
	CvTask a(waiter(cv, lock, results[0]), "CVA");
	CvTask b(waiter(cv, lock, results[1]), "CVB");
	CvTask c(waiter(cv, lock, results[2]), "CVC");
	a.startResumable();
	b.startResumable();
	c.startResumable();
	runReady();
	cv.Broadcast();
	runReady();
	for (const WaitResult &result : results) {
		CHECK(result.signalled);
	}
	cv.Signal(); // Takes the internal lock: a lock left held by Broadcast would block forever (kernel fault)
}

TEST_CASE_FIXTURE(RtosFixture, "a broadcast from a task wakes the threads waiting at that time") {
	ConditionVariable cv;
	MutexStandard lock;
	WaitResult first;
	WaitResult second;
	CvTask a(waiter(cv, lock, first), "CVA");
	CvTask b(waiter(cv, lock, second), "CVB");
	CvTask signaller([&](CvTask &) {
		lock.Lock();
		cv.Broadcast();
		lock.Unlock();
	}, "CVS");
	a.startResumable();
	b.startResumable();
	runReady();
	signaller.startResumable();
	runReady();
	CHECK(first.signalled);
	CHECK(second.signalled);
	cv.Broadcast(); // Takes the internal lock: a lock left held by the task's Broadcast would block forever (kernel fault)
}

TEST_CASE_FIXTURE(RtosFixture, "a wait with a timeout returns false after the timeout of simulated time") {
	ConditionVariable cv;
	MutexStandard lock;
	WaitResult result;
	CvTask task(waiter(cv, lock, result, 5), "CVW");
	task.startResumable();
	runReady();
	advance(4);
	CHECK_FALSE(result.returned);
	advance(1);
	CHECK(result.returned);
	CHECK_FALSE(result.signalled);
}

TEST_CASE_FIXTURE(RtosFixture, "a wait with a timeout returns true when signalled before the timeout") {
	ConditionVariable cv;
	MutexStandard lock;
	WaitResult result;
	CvTask task(waiter(cv, lock, result, 5), "CVW");
	task.startResumable();
	runReady();
	advance(2);
	cv.Signal();
	runReady();
	CHECK(result.returned);
	CHECK(result.signalled);
}

TEST_CASE_FIXTURE(RtosFixture, "characterization: a Wait with a recursive mutex locked twice keeps the mutex held while the thread waits") {
	// NEEDS REVIEW 151: Thread::Wait (cthread.cpp:253) calls CvLock.Unlock() once, which releases only one level of a
	// MutexRecursive; the waiting thread still holds it, so a signaller that takes the lock first (the usual pattern)
	// deadlocks. Should Wait release all levels, or are recursive mutexes not allowed as CvLock (undocumented)?
	ConditionVariable cv;
	MutexRecursive lock;
	bool signalled = false;
	CvTask task([&](CvTask &self) {
		lock.Lock();
		lock.Lock();
		signalled = self.wait(cv, lock);
		lock.Unlock();
		lock.Unlock();
	}, "CVW");
	task.startResumable();
	runReady();
	CHECK_FALSE(lock.Lock(5)); // A signaller can not get the lock while the thread waits
	CHECK(HostPlatform::nowUs() == 5000);
	cv.Signal(); // Without the lock
	runReady();
	CHECK(signalled);
	CHECK(lock.Lock(0)); // Fully released after the waiter's two unlocks
	CHECK(lock.Unlock());
}

// ---------------- Suspected vendor defects (tier A: reported, never fixed) ----------------

TEST_CASE_FIXTURE(RtosFixture, "a wait after a timed out wait and a broadcast blocks until the next signal"
				  * KNOWN_ISSUE("vendor V7 (Broadcast variant): Thread::Wait (cthread.cpp:244-262) leaves the timed out "
								"thread in the wait list; Broadcast (ccondition_variable.cpp:105-110) pops it and gives its "
								"wait semaphore, so its next Wait returns true at once without a signal (thread.hpp:391: true "
								"means signalled). The first failing assertion is CHECK_FALSE(second.returned)")) {
	ConditionVariable cv;
	MutexStandard lock;
	BinarySemaphore gate;
	WaitResult first;
	WaitResult second;
	CvTask task([&](CvTask &self) {
		lock.Lock();
		first.signalled = self.wait(cv, lock, 5);
		first.returned = true;
		lock.Unlock();
		gate.Take();
		lock.Lock();
		second.signalled = self.wait(cv, lock);
		second.returned = true;
		lock.Unlock();
	}, "CVW");
	task.startResumable();
	runReady();
	advance(5);
	REQUIRE(first.returned);
	REQUIRE_FALSE(first.signalled); // Timed out
	cv.Broadcast(); // Nobody waits now
	gate.Give();	// The thread waits again
	runReady();
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK_FALSE(second.returned);
}

}
