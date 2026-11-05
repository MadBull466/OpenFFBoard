/*
 * Tests of the cpp_freertos software timer wrapper Timer (FFBoard/Src/ctimer.cpp, FFBoard/Inc/timer.hpp; vendor code,
 * tier A: suspected defects are reported, never fixed) on the fake FreeRTOS API F11 (support/HostRtosExt.h).
 *
 * The timer commands are asynchronous like on the target: Start/Stop/Reset/SetPeriod only queue a command for the
 * timer service task ("Tmr Svc", priority 2); it is executed when the service runs (runReady(), advance()).
 *
 * Vendor V9 (ctimer.cpp:89): ~Timer only queues the delete. If the timer expired or a start command is pending, the
 * timer service would call Run() of the destroyed object. The timers of these tests are therefore destroyed in a task
 * (TimerInTask, RtosFixture::runInTask at priority 3) after the timer service has processed everything due at the
 * current tick, or on the test thread without running the service afterwards (the fixture discards pending commands);
 * the KNOWN_ISSUE case at the end shows the defect through HostRtos::timerIdsInService().
 * Fix experiments: a synchronous fix (~Timer waits until the service processed the delete, "V9b") cannot complete on
 * the test thread: no task runs while the test thread waits, so it is recorded as "would block forever" (fake
 * limitation, HostRtosExt.h). Timers that are created by value in the CHECK_THROWS cases or destroyed on the test
 * thread on purpose are the remaining cases that such a fix makes fail through that limitation.
 *
 * No shipped firmware uses this class (latent code). Timer::GetTimerDaemonHandle is compiled only with
 * INCLUDE_xTimerGetTimerDaemonTaskHandle (not set for the F407 nor here).
 * Host vs target: exception variant on the host (NEEDS REVIEW 142).
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"

#include "thread.hpp"
#include "timer.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

using namespace cpp_freertos;

namespace {

/** Timer recording its runs (tick count and task of every run) */
class RecordingTimer : public Timer {
public:
	RecordingTimer(const char *name, TickType_t period, bool periodic) : Timer(name, period, periodic) {}
	RecordingTimer(TickType_t period, bool periodic) : Timer(period, periodic) {}
	explicit RecordingTimer(TickType_t period) : Timer(period) {}					// Periodic by default
	RecordingTimer(const char *name, TickType_t period) : Timer(name, period) {}	// Periodic by default

	std::vector<TickType_t> runTicks;
	TaskHandle_t ranIn = nullptr;
	size_t runs() const { return runTicks.size(); }

protected:
	void Run() override {
		runTicks.push_back(xTaskGetTickCount());
		ranIn = xTaskGetCurrentTaskHandle();
	}
};

/**
 * Owns a timer and destroys it in a task of priority 3 (RtosFixture::runInTask), so a destructor that waits for the
 * timer service (a synchronous fix of V9) can complete. The timer service has processed everything due at the current
 * tick before the task runs, so it never calls Run() of the destroyed timer (V9).
 */
template <class T>
class TimerInTask {
public:
	template <class... Args>
	explicit TimerInTask(Args &&...args) : timer(new T(std::forward<Args>(args)...)) {}
	~TimerInTask() noexcept(false) {
		T *owned = timer;
		RtosFixture::runInTask([owned] { delete owned; }, 3);
	}
	TimerInTask(const TimerInTask &) = delete;
	TimerInTask &operator=(const TimerInTask &) = delete;
	T &operator*() { return *timer; }

private:
	T *timer;
};

/** Thread running a function set by the test, priority 3 (above the timer service, like a task the service can not preempt) */
class TimerTestThread : public Thread {
public:
	explicit TimerTestThread(std::function<void()> body) : Thread("TT", 128, 3), body(std::move(body)) {}

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

void noop(void *, uint32_t) {}

/** Fills the command queue of the timer service with pended no-op calls */
void fillTimerCommandQueue() {
	for (int i = 0; i < configTIMER_QUEUE_LENGTH; i++) {
		REQUIRE(xTimerPendFunctionCall(noop, nullptr, 0, 0) == pdPASS);
	}
	REQUIRE(HostRtos::timerCommandsPending() == configTIMER_QUEUE_LENGTH);
}

/** Runs a timer command in an interrupt; returns its result */
template <class Command>
bool fromIsr(Command command, BaseType_t *woken) {
	HostPlatform::IsrContext isr;
	return command(woken);
}

} // namespace

TEST_SUITE("Timer") {

// ---------------- Construction ----------------

TEST_CASE_FIXTURE(RtosFixture, "a new timer is dormant") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	CHECK_FALSE(timer.IsActive());
	advance(30);
	CHECK(timer.runs() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a timer created without a name runs once per period") {
	TimerInTask<RecordingTimer> timerOwner(5, true);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	advance(15);
	CHECK(timer.runTicks == std::vector<TickType_t>{5, 10, 15});
}

TEST_CASE_FIXTURE(RtosFixture, "a named timer is periodic by default") {
	TimerInTask<RecordingTimer> timerOwner("T", 4);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	advance(12);
	CHECK(timer.runs() == 3);
	CHECK(timer.IsActive());
}

TEST_CASE_FIXTURE(RtosFixture, "a timer created without a name is periodic by default") {
	TimerInTask<RecordingTimer> timerOwner(4);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	advance(12);
	CHECK(timer.runs() == 3);
	CHECK(timer.IsActive());
}

TEST_CASE_FIXTURE(RtosFixture, "a named one-shot timer runs once and becomes dormant") {
	TimerInTask<RecordingTimer> timerOwner("T", 3, false);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	advance(20);
	CHECK(timer.runTicks == std::vector<TickType_t>{3});
	CHECK_FALSE(timer.IsActive());
}

TEST_CASE_FIXTURE(RtosFixture, "a one-shot timer created without a name runs once and becomes dormant") {
	TimerInTask<RecordingTimer> timerOwner(3, false);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	advance(20);
	CHECK(timer.runTicks == std::vector<TickType_t>{3});
	CHECK_FALSE(timer.IsActive());
}

TEST_CASE_FIXTURE(RtosFixture, "a timer whose kernel timer can not be created throws TimerCreateException") {
	HostRtos::failCreate(HostRtos::RtosObject::Timer);
	const size_t before = HostRtos::timerCount();
	CHECK_THROWS_AS(RecordingTimer("T", 10, true), TimerCreateException);
	CHECK(HostRtos::timerCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "a timer without a name whose kernel timer can not be created throws TimerCreateException") {
	HostRtos::failCreate(HostRtos::RtosObject::Timer);
	const size_t before = HostRtos::timerCount();
	CHECK_THROWS_AS(RecordingTimer(10, true), TimerCreateException);
	CHECK(HostRtos::timerCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "the exception of a failed timer creation names the failure") {
	HostRtos::failCreate(HostRtos::RtosObject::Timer);
	std::string what;
	try {
		RecordingTimer timer(10, true);
	} catch (const TimerCreateException &e) {
		what = e.what();
	}
	CHECK(what == "Timer Constructor Failed");
}

TEST_CASE_FIXTURE(RtosFixture, "deleting a dormant timer deletes its kernel timer once the timer service ran") {
	const size_t before = HostRtos::timerCount();
	RecordingTimer *timer = new RecordingTimer("T", 10, true);
	CHECK(HostRtos::timerCount() == before + 1);
	runInTask([timer] { delete timer; }, 3); // Dormant: the service can not call Run() of the deleted object (V9)
	runReady();
	CHECK(HostRtos::timerCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "deleting a timer on the test thread queues a command for the timer service") {
	RecordingTimer *timer = new RecordingTimer("T", 10, true);
	runReady();
	REQUIRE(HostRtos::timerCommandsPending() == 0);
	delete timer; // Dormant (V9); on the test thread: see the header about a synchronous fix
	CHECK(HostRtos::timerCommandsPending() >= 1);
	runReady();
	CHECK(HostRtos::timerCommandsPending() == 0);
}

// ---------------- Start / IsActive / Run ----------------

TEST_CASE_FIXTURE(RtosFixture, "a started timer is active once the timer service processed the start command") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	CHECK_FALSE(timer.IsActive()); // Only queued
	runReady();
	CHECK(timer.IsActive());
}

TEST_CASE_FIXTURE(RtosFixture, "a start without a timeout argument queues the start command") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	CHECK(timer.Start());
	runReady();
	CHECK(timer.IsActive());
}

TEST_CASE_FIXTURE(RtosFixture, "the first run of a timer is one period after the start command") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	advance(5);
	REQUIRE(timer.Start(0)); // Command time 5; the service processes it at the next tick
	advance(9);
	CHECK(timer.runs() == 0);
	advance(1);
	CHECK(timer.runTicks == std::vector<TickType_t>{15});
}

TEST_CASE_FIXTURE(RtosFixture, "a timer runs in the timer service task") {
	TimerInTask<RecordingTimer> timerOwner("T", 2, false);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	advance(2);
	REQUIRE(timer.runs() == 1);
	CHECK(timer.ranIn == HostRtos::timerServiceTask());
}

TEST_CASE_FIXTURE(RtosFixture, "two timers run their own Run function") {
	TimerInTask<RecordingTimer> fastOwner("F", 2, true);
	RecordingTimer &fast = *fastOwner;
	TimerInTask<RecordingTimer> slowOwner("S", 5, true);
	RecordingTimer &slow = *slowOwner;
	REQUIRE(fast.Start(0));
	REQUIRE(slow.Start(0));
	advance(10);
	CHECK(fast.runs() == 5);
	CHECK(slow.runs() == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "a start command fails when the timer command queue is full and no timeout is given") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	fillTimerCommandQueue();
	CHECK_FALSE(timer.Start(0));
	runReady();
	CHECK_FALSE(timer.IsActive());
}

TEST_CASE_FIXTURE(RtosFixture, "a start command on the test thread fails after its timeout when the command queue stays full") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	fillTimerCommandQueue();
	CHECK_FALSE(timer.Start(4)); // The queue stays full only because no task (timer service) runs while the test thread waits
	CHECK(HostPlatform::nowUs() == 4000);
	runReady(); // Room for the delete command of ~Timer
}

TEST_CASE_FIXTURE(RtosFixture, "a start command from a task waits for room in the timer command queue") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	fillTimerCommandQueue();
	bool result = false;
	TimerTestThread task([&] { result = timer.Start(); });
	TaskHandle_t handle = task.startResumable();
	HostRtos::runUntilBlocked(handle); // Before the timer service: the queue is still full
	REQUIRE(HostRtos::isWaiting(handle));
	runReady(); // The service empties the queue, the task sends its command, the service starts the timer
	CHECK(result);
	CHECK(timer.IsActive());
}

// ---------------- Stop ----------------

TEST_CASE_FIXTURE(RtosFixture, "a stopped timer does not run and is dormant") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	advance(5);
	CHECK(timer.Stop(0));
	runReady();
	CHECK_FALSE(timer.IsActive());
	advance(30);
	CHECK(timer.runs() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a stop without a timeout argument queues the stop command") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	runReady();
	CHECK(timer.Stop());
	runReady();
	CHECK_FALSE(timer.IsActive());
}

TEST_CASE_FIXTURE(RtosFixture, "a stop command fails when the timer command queue is full and no timeout is given") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	runReady();
	fillTimerCommandQueue();
	CHECK_FALSE(timer.Stop(0));
	runReady();
	CHECK(timer.IsActive());
	REQUIRE(timer.Stop(0));
	runReady(); // Dormant before it is destroyed (V9)
}

// ---------------- Reset ----------------

TEST_CASE_FIXTURE(RtosFixture, "a reset restarts the period from the reset command") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	advance(6);
	CHECK(timer.Reset(0));
	advance(9);
	CHECK(timer.runs() == 0);
	advance(1);
	CHECK(timer.runTicks == std::vector<TickType_t>{16});
}

TEST_CASE_FIXTURE(RtosFixture, "a reset starts a dormant timer") {
	TimerInTask<RecordingTimer> timerOwner("T", 5, false);
	RecordingTimer &timer = *timerOwner;
	CHECK(timer.Reset());
	runReady();
	CHECK(timer.IsActive());
	advance(5);
	CHECK(timer.runTicks == std::vector<TickType_t>{5});
}

TEST_CASE_FIXTURE(RtosFixture, "a reset command fails when the timer command queue is full and no timeout is given") {
	TimerInTask<RecordingTimer> timerOwner("T", 5, false);
	RecordingTimer &timer = *timerOwner;
	fillTimerCommandQueue();
	CHECK_FALSE(timer.Reset(0));
	runReady();
	CHECK_FALSE(timer.IsActive());
}

// ---------------- SetPeriod ----------------

TEST_CASE_FIXTURE(RtosFixture, "changing the period of a dormant timer starts it with the new period") {
	TimerInTask<RecordingTimer> timerOwner("T", 50, false);
	RecordingTimer &timer = *timerOwner;
	CHECK(timer.SetPeriod(5));
	runReady();
	CHECK(timer.IsActive());
	advance(5);
	CHECK(timer.runTicks == std::vector<TickType_t>{5});
}

TEST_CASE_FIXTURE(RtosFixture, "a new period of an active timer counts from the change") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	advance(4);
	CHECK(timer.SetPeriod(3, 0));
	runReady(); // The service applies the new period at time 4
	advance(2);
	CHECK(timer.runs() == 0);
	advance(4);
	CHECK(timer.runTicks == std::vector<TickType_t>{7, 10});
	REQUIRE(timer.Stop(0));
	runReady();
}

TEST_CASE_FIXTURE(RtosFixture, "a period change fails when the timer command queue is full and no timeout is given") {
	TimerInTask<RecordingTimer> timerOwner("T", 50, false);
	RecordingTimer &timer = *timerOwner;
	fillTimerCommandQueue();
	CHECK_FALSE(timer.SetPeriod(5, 0));
	runReady();
	CHECK_FALSE(timer.IsActive());
	advance(10);
	CHECK(timer.runs() == 0);
}

// ---------------- Interrupt variants ----------------

TEST_CASE_FIXTURE(RtosFixture, "a timer started from an interrupt runs after its period") {
	TimerInTask<RecordingTimer> timerOwner("T", 4, false);
	RecordingTimer &timer = *timerOwner;
	BaseType_t woken = pdFALSE;
	CHECK(fromIsr([&](BaseType_t *w) { return timer.StartFromISR(w); }, &woken));
	runReady();
	CHECK(timer.IsActive());
	advance(4);
	CHECK(timer.runTicks == std::vector<TickType_t>{4});
}

TEST_CASE_FIXTURE(RtosFixture, "a timer command from an interrupt reports that the waiting timer service task was woken") {
	TimerInTask<RecordingTimer> timerOwner("T", 4, false);
	RecordingTimer &timer = *timerOwner;
	runReady(); // The service task waits for commands
	REQUIRE(HostRtos::isWaiting(HostRtos::timerServiceTask()));
	BaseType_t woken = pdFALSE;
	REQUIRE(fromIsr([&](BaseType_t *w) { return timer.StartFromISR(w); }, &woken));
	CHECK(woken == pdTRUE);
	REQUIRE(timer.Stop(0));
	runReady();
}

TEST_CASE_FIXTURE(RtosFixture, "a start from an interrupt fails when the timer command queue is full") {
	TimerInTask<RecordingTimer> timerOwner("T", 4, false);
	RecordingTimer &timer = *timerOwner;
	fillTimerCommandQueue();
	BaseType_t woken = pdFALSE;
	CHECK_FALSE(fromIsr([&](BaseType_t *w) { return timer.StartFromISR(w); }, &woken));
	runReady();
	CHECK_FALSE(timer.IsActive());
}

TEST_CASE_FIXTURE(RtosFixture, "a timer stopped from an interrupt does not run") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	runReady();
	BaseType_t woken = pdFALSE;
	CHECK(fromIsr([&](BaseType_t *w) { return timer.StopFromISR(w); }, &woken));
	runReady();
	CHECK_FALSE(timer.IsActive());
	advance(20);
	CHECK(timer.runs() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a stop from an interrupt fails when the timer command queue is full") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, true);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	runReady();
	fillTimerCommandQueue();
	BaseType_t woken = pdFALSE;
	CHECK_FALSE(fromIsr([&](BaseType_t *w) { return timer.StopFromISR(w); }, &woken));
	runReady();
	CHECK(timer.IsActive());
	REQUIRE(timer.Stop(0));
	runReady();
}

TEST_CASE_FIXTURE(RtosFixture, "a reset from an interrupt restarts the period") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, false);
	RecordingTimer &timer = *timerOwner;
	REQUIRE(timer.Start(0));
	advance(7);
	BaseType_t woken = pdFALSE;
	CHECK(fromIsr([&](BaseType_t *w) { return timer.ResetFromISR(w); }, &woken));
	advance(9);
	CHECK(timer.runs() == 0);
	advance(1);
	CHECK(timer.runTicks == std::vector<TickType_t>{17});
}

TEST_CASE_FIXTURE(RtosFixture, "a reset from an interrupt fails when the timer command queue is full") {
	TimerInTask<RecordingTimer> timerOwner("T", 10, false);
	RecordingTimer &timer = *timerOwner;
	fillTimerCommandQueue();
	BaseType_t woken = pdFALSE;
	CHECK_FALSE(fromIsr([&](BaseType_t *w) { return timer.ResetFromISR(w); }, &woken));
	runReady();
	CHECK_FALSE(timer.IsActive());
}

TEST_CASE_FIXTURE(RtosFixture, "a period change from an interrupt applies the new period") {
	TimerInTask<RecordingTimer> timerOwner("T", 50, false);
	RecordingTimer &timer = *timerOwner;
	BaseType_t woken = pdFALSE;
	CHECK(fromIsr([&](BaseType_t *w) { return timer.SetPeriodFromISR(6, w); }, &woken));
	runReady();
	CHECK(timer.IsActive());
	advance(6);
	CHECK(timer.runTicks == std::vector<TickType_t>{6});
}

TEST_CASE_FIXTURE(RtosFixture, "a period change from an interrupt fails when the timer command queue is full") {
	TimerInTask<RecordingTimer> timerOwner("T", 50, false);
	RecordingTimer &timer = *timerOwner;
	fillTimerCommandQueue();
	BaseType_t woken = pdFALSE;
	CHECK_FALSE(fromIsr([&](BaseType_t *w) { return timer.SetPeriodFromISR(6, w); }, &woken));
	runReady();
	CHECK_FALSE(timer.IsActive());
}

// ---------------- Suspected vendor defects (tier A: reported, never fixed) ----------------

TEST_CASE_FIXTURE(RtosFixture, "the timer service can no longer call a destroyed active timer"
				  * KNOWN_ISSUE("vendor V9: ~Timer (ctimer.cpp:89) only queues xTimerDelete and returns; the timer keeps "
								"the destroyed object as its ID and stays in the active list until the timer service processes "
								"the delete, and the service handles an expired timer (and queued starts) before the delete "
								"(timers.c:548-560), so it would call Run() of the destroyed object. Reachable when the "
								"deleting task has priority >= 2 (configTIMER_TASK_PRIORITY): here 3. A synchronous fix (~Timer "
								"waits for the service) also makes timers destroyed on the test thread fail through the fake's "
								"'would block forever' (see the file header). The first failing assertion is "
								"CHECK_FALSE(stillCallable)")) {
	RecordingTimer *timer = new RecordingTimer("T", 10, true);
	REQUIRE(timer->Start(0));
	runReady();
	void *const id = timer; // TimerCallbackFunctionAdapter calls Run() of the timer ID (ctimer.cpp:167-168)
	bool stillCallable = true;
	// Deleted by a task above the timer service (on the target the service can not run in between). The time does not
	// advance, so the timer does not expire and the fake service never calls Run() of the destroyed object here.
	runInTask([&] {
		delete timer;
		const std::vector<void *> ids = HostRtos::timerIdsInService();
		stillCallable = std::find(ids.begin(), ids.end(), id) != ids.end();
	}, 3);
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK_FALSE(stillCallable);
}

}
