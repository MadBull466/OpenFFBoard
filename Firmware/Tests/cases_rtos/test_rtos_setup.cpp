/*
 * Smoke tests of ffb_rtos_tests (P8 batch 0): one behaviour per cpp_freertos wrapper family on the fake FreeRTOS API
 * F11 (support/HostRtosExt.h). The wrappers are vendor code (tier A): suspected defects are reported, never fixed.
 *
 * Host vs target: the host build uses the exception variant of the wrappers (the F407 defines
 * CPP_FREERTOS_NO_EXCEPTIONS and stops in configASSERT instead), and the tick hook wrapper is compiled although the
 * F407 has configUSE_TICK_HOOK 0.
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"
#include "cmsis_compiler.h"

#include "condition_variable.hpp"
#include "event_groups.hpp"
#include "mem_pool.hpp"
#include "mutex.hpp"
#include "queue.hpp"
#include "read_write_lock.hpp"
#include "tasklet.hpp"
#include "thread.hpp"
#include "tickhook.hpp"
#include "timer.hpp"
#include "workqueue.hpp"

#include <cstdint>
#include <functional>
#include <set>
#include <string>
#include <vector>

using namespace cpp_freertos;

namespace {

/** Thread running a function set by the test; gives the test access to the protected Thread::Wait */
class FunctionThread : public Thread {
public:
	explicit FunctionThread(std::function<void(FunctionThread &)> body, const std::string &name = "FN", UBaseType_t priority = 2)
		: Thread(name, 128, priority), body(std::move(body)) {}

	bool wait(ConditionVariable &cv, Mutex &lock, TickType_t timeout = portMAX_DELAY) { return Wait(cv, lock, timeout); }

	/** Starts the task and makes it resumable (it continues after a blocking call, see HostRtos::makeResumable) */
	TaskHandle_t startResumable() {
		REQUIRE(Start());
		HostRtos::makeResumable(GetHandle());
		return GetHandle();
	}

protected:
	void Run() override { body(*this); }

private:
	std::function<void(FunctionThread &)> body;
};

/** Runs `fn` in a resumable task until it is done (e.g. a destructor that waits for another task) */
void runInTask(std::function<void()> fn) {
	bool done = false;
	FunctionThread task([&](FunctionThread &) {
		fn();
		done = true;
	}, "RUNNER", 1);
	task.startResumable();
	HostRtos::runUntilIdle();
	REQUIRE(done);
}

class CountingTimer : public Timer {
public:
	CountingTimer(TickType_t period, bool periodic) : Timer("CNT", period, periodic) {}
	uint32_t runs = 0;

protected:
	void Run() override { runs++; }
};

class RecordingTasklet : public Tasklet {
public:
	~RecordingTasklet() override { CheckForSafeDelete(); } // Required by the Tasklet design (tasklet.hpp)
	uint32_t runs = 0;
	uint32_t lastParameter = 0;
	TaskHandle_t ranIn = nullptr;

protected:
	void Run(uint32_t parameter) override {
		runs++;
		lastParameter = parameter;
		ranIn = xTaskGetCurrentTaskHandle();
	}
};

class CountingWorkItem : public WorkItem {
public:
	uint32_t runs = 0;
	void Run() override { runs++; }
};

class CountingTickHook : public TickHook {
public:
	uint32_t runs = 0;
	uint32_t runsInInterrupt = 0;

protected:
	void Run() override {
		runs++;
		if (__get_IPSR() != 0) {
			runsInInterrupt++;
		}
	}
};

} // namespace

TEST_SUITE("RtosSetup") {

TEST_CASE_FIXTURE(RtosFixture, "an item sent to a queue is received unchanged") {
	Queue queue(3, sizeof(uint32_t));
	uint32_t value = 0xA5A5F00D;
	REQUIRE(queue.Enqueue(&value, 0));
	uint32_t received = 0;
	CHECK(queue.Dequeue(&received, 0));
	CHECK(received == 0xA5A5F00D);
	CHECK(queue.IsEmpty());
}

TEST_CASE_FIXTURE(RtosFixture, "a dequeue from an empty queue on the test thread times out after its timeout of simulated time") {
	Queue queue(1, sizeof(uint32_t));
	uint32_t received = 0;
	CHECK_FALSE(queue.Dequeue(&received, 5));
	CHECK(HostPlatform::nowUs() == 5000);
}

TEST_CASE_FIXTURE(RtosFixture, "an infinite wait outside a task is recorded as a kernel fault") {
	Queue queue(1, sizeof(uint32_t));
	uint32_t received = 0;
	CHECK_FALSE(queue.Dequeue(&received)); // portMAX_DELAY: would block forever on the target
	CHECK(expectKernelFault("would block forever"));
}

TEST_CASE_FIXTURE(RtosFixture, "a periodic timer runs once per period of simulated time") {
	CountingTimer timer(10, true);
	REQUIRE(timer.Start(0));
	advance(9);
	CHECK(timer.runs == 0);
	advance(1);
	CHECK(timer.runs == 1);
	advance(20);
	CHECK(timer.runs == 3);
}

TEST_CASE_FIXTURE(RtosFixture, "a task waiting for event bits continues when they are set") {
	EventGroup group;
	EventBits_t result = 0;
	FunctionThread task([&](FunctionThread &) { result = group.WaitBits(0x3, true, true, portMAX_DELAY); });
	TaskHandle_t handle = task.startResumable();
	runReady();
	REQUIRE(HostRtos::isWaiting(handle));
	group.SetBits(0x3);
	runReady();
	CHECK(result == 0x3);
	CHECK(group.GetBits() == 0); // cleared on exit
}

TEST_CASE_FIXTURE(RtosFixture, "a scheduled tasklet runs in the timer service task with its parameter") {
	RecordingTasklet tasklet;
	REQUIRE(tasklet.Schedule(42, 0));
	CHECK(tasklet.runs == 0);
	runReady();
	CHECK(tasklet.runs == 1);
	CHECK(tasklet.lastParameter == 42);
	CHECK(tasklet.ranIn == HostRtos::timerServiceTask());
}

TEST_CASE_FIXTURE(RtosFixture, "a work item queued to a work queue is run by its worker thread") {
	WorkQueue *queue = new WorkQueue("WQ");
	CountingWorkItem item;
	REQUIRE(queue->QueueWork(&item));
	runReady();
	CHECK(item.runs == 1);
	runInTask([queue] { delete queue; }); // ~WorkQueue waits for the worker thread to end
	CHECK(HostRtos::findTask("WQ") == nullptr);
}

TEST_CASE_FIXTURE(RtosFixture, "a registered tick hook runs once per simulated tick in interrupt context") {
	CountingTickHook hook;
	hook.Register();
	HostPlatform::advanceMs(5);
	CHECK(hook.runs == 5);
	CHECK(hook.runsInInterrupt == 5);
}

TEST_CASE_FIXTURE(RtosFixture, "a memory pool hands out distinct aligned items until it is exhausted") {
	MemoryPool *pool = keepAlive(new MemoryPool(10, 3, 8)); // ~MemoryPool is deleted: never destroyed
	std::set<void *> items;
	for (int i = 0; i < 3; i++) {
		void *item = pool->Allocate();
		REQUIRE(item != nullptr);
		CHECK(reinterpret_cast<uintptr_t>(item) % 8 == 0);
		items.insert(item);
	}
	CHECK(items.size() == 3);
	CHECK(pool->Allocate() == nullptr);
}

TEST_CASE_FIXTURE(RtosFixture, "a writer waits until the last reader has unlocked") {
	ReadWriteLockPreferReader lock;
	bool writing = false;
	FunctionThread writer([&](FunctionThread &) {
		lock.WriterLock();
		writing = true;
		lock.WriterUnlock();
	});
	lock.ReaderLock();
	writer.startResumable();
	runReady();
	CHECK_FALSE(writing);
	lock.ReaderUnlock();
	runReady();
	CHECK(writing);
}

TEST_CASE_FIXTURE(RtosFixture, "a condition variable signal wakes the waiting thread") {
	ConditionVariable cv;
	MutexStandard cvLock;
	bool signalled = false;
	FunctionThread waiter([&](FunctionThread &self) {
		cvLock.Lock();
		signalled = self.wait(cv, cvLock);
		cvLock.Unlock();
	});
	waiter.startResumable();
	runReady();
	REQUIRE_FALSE(signalled);
	cv.Signal();
	runReady();
	CHECK(signalled);
}

TEST_CASE_FIXTURE(RtosFixture, "a queue whose kernel queue can not be created throws QueueCreateException") {
	HostRtos::failCreate(HostRtos::RtosObject::Queue);
	CHECK_THROWS_AS(Queue(2, sizeof(uint32_t)), QueueCreateException);
	CHECK(HostRtos::queueCount() == 0);
}

// ---------------- Suspected vendor defects (tier A: reported, never fixed) ----------------

TEST_CASE_FIXTURE(RtosFixture, "a condition variable signal after a timed out wait wakes the thread that still waits"
				  * KNOWN_ISSUE("vendor V7: Thread::Wait (cthread.cpp:244-262) leaves a timed out thread in the wait list of the "
								"ConditionVariable; Signal (ccondition_variable.cpp:86-88) pops that stale entry and gives its "
								"semaphore, so the waiting thread misses the signal. The first failing assertion is CHECK(signalled)")) {
	ConditionVariable cv;
	MutexStandard cvLock;
	bool timedOutWaitResult = true;
	bool signalled = false;
	FunctionThread early([&](FunctionThread &self) {
		cvLock.Lock();
		timedOutWaitResult = self.wait(cv, cvLock, 5);
		cvLock.Unlock();
	}, "EARLY");
	FunctionThread waiter([&](FunctionThread &self) {
		cvLock.Lock();
		signalled = self.wait(cv, cvLock);
		cvLock.Unlock();
	}, "WAITER");
	early.startResumable();
	runReady();
	advance(5);
	REQUIRE_FALSE(timedOutWaitResult); // The first wait timed out
	waiter.startResumable();
	runReady();
	cv.Signal();
	runReady();
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK(signalled);
}

TEST_CASE_FIXTURE(RtosFixture, "a tasklet that can not be scheduled from an interrupt releases its lock with the interrupt API"
				  * KNOWN_ISSUE("vendor V8: Tasklet::ScheduleFromISR (ctasklet.cpp:124) gives DtorLock back with xSemaphoreGive "
								"instead of xSemaphoreGiveFromISR when xTimerPendFunctionCallFromISR fails (timer service queue "
								"full); in an interrupt FreeRTOS stops in configASSERT (port.c:415). The first failing assertion "
								"is CHECK(faults.empty())")) {
	class IdleTasklet : public Tasklet {
	public:
		~IdleTasklet() override { CheckForSafeDelete(); }

	protected:
		void Run(uint32_t) override {}
	} tasklet;
	uint32_t pended = 0;
	auto store = [](void *target, uint32_t value) { *static_cast<uint32_t *>(target) = value; };
	for (int i = 0; i < configTIMER_QUEUE_LENGTH; i++) {
		REQUIRE(xTimerPendFunctionCall(store, &pended, 1, 0) == pdPASS); // Fill the timer service queue
	}
	BaseType_t woken = pdFALSE;
	bool scheduled = true;
	{
		HostPlatform::IsrContext isr;
		scheduled = tasklet.ScheduleFromISR(7, &woken);
	}
	REQUIRE_FALSE(scheduled);
	const std::vector<std::string> faults = HostRtos::takeKernelFaults();
	for (const std::string &fault : faults) {
		INFO(fault);
		REQUIRE(fault.find("xSemaphoreGive called in interrupt context") != std::string::npos); // No other fault
	}
	CHECK(faults.empty());
	runReady();
}

}
