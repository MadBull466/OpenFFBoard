/*
 * Tests of the cpp_freertos Tasklet wrapper (FFBoard/Src/ctasklet.cpp, FFBoard/Inc/tasklet.hpp; vendor code, tier A:
 * suspected defects are reported, never fixed) on the fake FreeRTOS API F11 (support/HostRtosExt.h).
 *
 * A tasklet is a function call pended to the timer service task (xTimerPendFunctionCall). Its binary semaphore DtorLock
 * is taken while a call is pending and given back after Run(), so a second Schedule waits for the first run and
 * CheckForSafeDelete (to be called by the destructor of every derived class, tasklet.hpp) waits until no call is pending.
 *
 * Vendor V8 (ctasklet.cpp:124, ScheduleFromISR gives DtorLock with the non-ISR API when the pend fails) is a KNOWN_ISSUE
 * in cases_rtos/test_rtos_setup.cpp; the passing ScheduleFromISR paths are tested here.
 * No shipped firmware uses this class (latent code). Host vs target: exception variant on the host (NEEDS REVIEW 142).
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"

#include "tasklet.hpp"
#include "thread.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

using namespace cpp_freertos;

namespace {

/** Tasklet recording its runs; `events` (optional) gets "run <parameter>" and "destroyed" */
class LoggingTasklet : public Tasklet {
public:
	explicit LoggingTasklet(std::vector<std::string> *events = nullptr) : events(events) {}
	~LoggingTasklet() override {
		CheckForSafeDelete(); // Required by the Tasklet design (tasklet.hpp)
		if (events != nullptr) {
			events->push_back("destroyed");
		}
	}

	std::vector<uint32_t> parameters;
	TaskHandle_t ranIn = nullptr;
	size_t runs() const { return parameters.size(); }

protected:
	void Run(uint32_t parameter) override {
		parameters.push_back(parameter);
		ranIn = xTaskGetCurrentTaskHandle();
		if (events != nullptr) {
			events->push_back("run " + std::to_string(parameter));
		}
	}

private:
	std::vector<std::string> *events;
};

/** Thread running a function set by the test */
class TaskletTestThread : public Thread {
public:
	explicit TaskletTestThread(std::function<void()> body) : Thread("TL", 128, 1), body(std::move(body)) {}

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
}

bool scheduleFromIsr(Tasklet &tasklet, uint32_t parameter, BaseType_t *woken) {
	HostPlatform::IsrContext isr;
	return tasklet.ScheduleFromISR(parameter, woken);
}

} // namespace

TEST_SUITE("Tasklet") {

// ---------------- Construction and deletion ----------------

TEST_CASE_FIXTURE(RtosFixture, "a tasklet does not run before it is scheduled") {
	LoggingTasklet tasklet;
	advance(10);
	CHECK(tasklet.runs() == 0);
	CHECK(HostRtos::timerCommandsPending() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a tasklet owns one semaphore, deleted by CheckForSafeDelete") {
	const size_t before = HostRtos::semaphoreCount();
	LoggingTasklet *tasklet = new LoggingTasklet();
	CHECK(HostRtos::semaphoreCount() == before + 1);
	delete tasklet;
	CHECK(HostRtos::semaphoreCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "a tasklet whose lock can not be created throws TaskletCreateException") {
	const size_t before = HostRtos::semaphoreCount();
	HostRtos::failCreate(HostRtos::RtosObject::BinarySemaphore);
	std::string what;
	try {
		LoggingTasklet tasklet;
	} catch (const TaskletCreateException &e) {
		what = e.what();
	}
	CHECK(what == "Tasklet Constructor Failed Create DtorLock Failed");
	CHECK(HostRtos::semaphoreCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "a tasklet create exception without details names the failure") {
	TaskletCreateException e;
	CHECK(std::string(e.what()) == "Tasklet Constructor Failed");
}

TEST_CASE_FIXTURE(RtosFixture, "deleting a scheduled tasklet in a task waits until its run is complete") {
	std::vector<std::string> events;
	LoggingTasklet *tasklet = new LoggingTasklet(&events);
	REQUIRE(tasklet->Schedule(5, 0));
	bool deleted = false;
	TaskletTestThread task([&] {
		delete tasklet; // CheckForSafeDelete waits for the pending run
		deleted = true;
	});
	TaskHandle_t handle = task.startResumable();
	HostRtos::runUntilBlocked(handle); // Before the timer service
	CHECK_FALSE(deleted);
	CHECK(HostRtos::isWaiting(handle));
	runReady();
	CHECK(deleted);
	CHECK(events == std::vector<std::string>{"run 5", "destroyed"});
}

// ---------------- Schedule ----------------

TEST_CASE_FIXTURE(RtosFixture, "a tasklet runs once per schedule with the parameter of that schedule") {
	LoggingTasklet tasklet;
	REQUIRE(tasklet.Schedule(1, 0));
	runReady();
	REQUIRE(tasklet.Schedule(2));
	runReady();
	CHECK(tasklet.parameters == std::vector<uint32_t>{1, 2});
	CHECK(tasklet.ranIn == HostRtos::timerServiceTask());
}

TEST_CASE_FIXTURE(RtosFixture, "a second schedule in a task waits until the first scheduled run is complete") {
	LoggingTasklet tasklet;
	bool first = false;
	bool second = false;
	TaskletTestThread task([&] {
		first = tasklet.Schedule(1);
		second = tasklet.Schedule(2);
	});
	TaskHandle_t handle = task.startResumable();
	HostRtos::runUntilBlocked(handle); // Before the timer service
	CHECK(first);
	CHECK_FALSE(second);
	CHECK(HostRtos::isWaiting(handle));
	CHECK(HostRtos::timerCommandsPending() == 1);
	runReady();
	CHECK(second);
	CHECK(tasklet.parameters == std::vector<uint32_t>{1, 2});
}

TEST_CASE_FIXTURE(RtosFixture, "a schedule fails when the timer command queue is full and no timeout is given") {
	LoggingTasklet tasklet;
	fillTimerCommandQueue();
	CHECK_FALSE(tasklet.Schedule(1, 0));
	runReady();
	CHECK(tasklet.runs() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a failed schedule releases the tasklet for the next schedule") {
	LoggingTasklet tasklet;
	fillTimerCommandQueue();
	REQUIRE_FALSE(tasklet.Schedule(1, 0));
	runReady();
	CHECK(tasklet.Schedule(2, 0)); // Would wait forever (recorded) if the lock was still taken
	runReady();
	CHECK(tasklet.parameters == std::vector<uint32_t>{2});
}

TEST_CASE_FIXTURE(RtosFixture, "a schedule on the test thread fails after its timeout when the command queue stays full") {
	LoggingTasklet tasklet;
	fillTimerCommandQueue();
	CHECK_FALSE(tasklet.Schedule(1, 6));
	CHECK(HostPlatform::nowUs() == 6000);
	runReady();
	CHECK(tasklet.runs() == 0);
}

// ---------------- ScheduleFromISR ----------------

TEST_CASE_FIXTURE(RtosFixture, "a tasklet scheduled from an interrupt runs in the timer service task with its parameter") {
	LoggingTasklet tasklet;
	BaseType_t woken = pdFALSE;
	CHECK(scheduleFromIsr(tasklet, 9, &woken));
	CHECK(tasklet.runs() == 0);
	runReady();
	CHECK(tasklet.parameters == std::vector<uint32_t>{9});
	CHECK(tasklet.ranIn == HostRtos::timerServiceTask());
}

TEST_CASE_FIXTURE(RtosFixture, "a schedule from an interrupt reports that the waiting timer service task was woken") {
	LoggingTasklet tasklet;
	REQUIRE(tasklet.Schedule(1, 0));
	runReady(); // The service ran the first schedule and waits for commands
	REQUIRE(HostRtos::isWaiting(HostRtos::timerServiceTask()));
	BaseType_t woken = pdFALSE;
	REQUIRE(scheduleFromIsr(tasklet, 2, &woken));
	CHECK(woken == pdTRUE);
	runReady();
}

TEST_CASE_FIXTURE(RtosFixture, "a schedule from an interrupt fails while the tasklet is still scheduled") {
	LoggingTasklet tasklet;
	REQUIRE(tasklet.Schedule(1, 0));
	BaseType_t woken = pdFALSE;
	CHECK_FALSE(scheduleFromIsr(tasklet, 2, &woken));
	CHECK(HostRtos::timerCommandsPending() == 1);
	runReady();
	CHECK(tasklet.parameters == std::vector<uint32_t>{1});
}

TEST_CASE_FIXTURE(RtosFixture, "a tasklet can be scheduled from an interrupt again after its run") {
	LoggingTasklet tasklet;
	BaseType_t woken = pdFALSE;
	REQUIRE(scheduleFromIsr(tasklet, 1, &woken));
	runReady();
	CHECK(scheduleFromIsr(tasklet, 2, &woken));
	runReady();
	CHECK(tasklet.parameters == std::vector<uint32_t>{1, 2});
}

}
