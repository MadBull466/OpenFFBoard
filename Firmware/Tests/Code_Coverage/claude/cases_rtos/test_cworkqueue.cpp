/*
 * Tests of the cpp_freertos WorkQueue / WorkItem wrappers (FFBoard/Src/cworkqueue.cpp, FFBoard/Inc/workqueue.hpp;
 * vendor code, tier A: suspected defects are reported, never fixed) on the fake FreeRTOS API F11
 * (support/HostRtosExt.h).
 *
 * A WorkQueue owns a Queue of WorkItem pointers, a BinarySemaphore ThreadComplete and a worker Thread (which itself
 * creates the binary semaphore ThreadWaitSem). ~WorkQueue queues a NULL item and waits on ThreadComplete until the
 * worker has ended, so it is deleted inside a resumable task (runInTask); the worker uses the restart model.
 *
 * Vendor V4 also applies here: when CWorkerThread::Run returns, TaskFunctionAdapter deletes the task, and ~WorkQueue
 * deletes WorkerThread, whose ~Thread calls vTaskDelete on the freed handle again (the fake ignores the unknown handle).
 * Vendor V10 (cworkqueue.cpp:78/96, the result of WorkerThread->Start() is ignored) is the KNOWN_ISSUE at the end.
 * Vendor V11 (cworkqueue.cpp:72-74 leaks the queue / semaphore when a later object of the constructor throws) is the
 * KNOWN_ISSUE_UB case at the end (skipped in sanitizer builds: the leak sanitizer reports the leaked Queue object).
 * No shipped firmware uses these classes (latent code). Host vs target: exception variant on the host
 * (NEEDS REVIEW 142); the stack depth is not recorded by the fake (not observable).
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"

#include "thread.hpp"
#include "workqueue.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <string>
#include <vector>

using namespace cpp_freertos;

namespace {

/** Work item appending its name to a log */
class LoggingWorkItem : public WorkItem {
public:
	LoggingWorkItem(std::vector<std::string> &log, std::string name, bool freeAfterComplete = false, bool *destroyed = nullptr)
		: WorkItem(freeAfterComplete), log(log), name(std::move(name)), destroyed(destroyed) {}
	~LoggingWorkItem() override {
		if (destroyed != nullptr) {
			*destroyed = true;
		}
	}
	TaskHandle_t ranIn = nullptr;
	void Run() override {
		log.push_back(name);
		ranIn = xTaskGetCurrentTaskHandle();
	}

private:
	std::vector<std::string> &log;
	std::string name;
	bool *destroyed;
};

/** Thread running a function set by the test */
class WorkTestThread : public Thread {
public:
	explicit WorkTestThread(std::function<void()> body, const std::string &name = "WT") : Thread(name, 128, 1), body(std::move(body)) {}

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

/** Runs `fn` in a resumable task until it is done (~WorkQueue waits for the worker thread) */
void runInTask(const std::function<void()> &fn) {
	bool done = false;
	WorkTestThread task([&] {
		fn();
		done = true;
	}, "RUNNER");
	task.startResumable();
	HostRtos::runUntilIdle();
	REQUIRE(done);
}

void deleteInTask(WorkQueue *queue) {
	runInTask([queue] { delete queue; });
}

} // namespace

TEST_SUITE("WorkQueue") {

// ---------------- WorkItem ----------------

TEST_CASE_FIXTURE(RtosFixture, "a work item is kept after its run by default") {
	std::vector<std::string> log;
	LoggingWorkItem item(log, "A");
	CHECK_FALSE(item.FreeAfterRun());
}

TEST_CASE_FIXTURE(RtosFixture, "a work item created to be freed reports it") {
	std::vector<std::string> log;
	LoggingWorkItem item(log, "A", true);
	CHECK(item.FreeAfterRun());
}

// ---------------- Worker thread ----------------

TEST_CASE_FIXTURE(RtosFixture, "a named work queue starts its worker thread with that name and priority") {
	WorkQueue *queue = new WorkQueue("WQX", 256, 3);
	TaskHandle_t worker = HostRtos::findTask("WQX");
	REQUIRE(worker != nullptr);
	CHECK(uxTaskPriorityGet(worker) == 3);
	deleteInTask(queue);
}

TEST_CASE_FIXTURE(RtosFixture, "a named work queue has the default worker priority") {
	WorkQueue *queue = new WorkQueue("WQX");
	TaskHandle_t worker = HostRtos::findTask("WQX");
	REQUIRE(worker != nullptr);
	CHECK(uxTaskPriorityGet(worker) == tskIDLE_PRIORITY + 1);
	deleteInTask(queue);
}

TEST_CASE_FIXTURE(RtosFixture, "a work queue without a name starts its worker thread as Default with the default priority") {
	REQUIRE(HostRtos::findTask("Default") == nullptr);
	WorkQueue *queue = new WorkQueue();
	TaskHandle_t worker = HostRtos::findTask("Default");
	REQUIRE(worker != nullptr);
	CHECK(uxTaskPriorityGet(worker) == tskIDLE_PRIORITY + 1);
	deleteInTask(queue);
}

TEST_CASE_FIXTURE(RtosFixture, "a work queue without a name starts its worker thread with the given priority") {
	WorkQueue *queue = new WorkQueue(256, 4);
	TaskHandle_t worker = HostRtos::findTask("Default");
	REQUIRE(worker != nullptr);
	CHECK(uxTaskPriorityGet(worker) == 4);
	deleteInTask(queue);
}

TEST_CASE_FIXTURE(RtosFixture, "work queued to a work queue without a name runs in its worker thread") {
	std::vector<std::string> log;
	LoggingWorkItem item(log, "A");
	WorkQueue *queue = new WorkQueue(256, 2, 3);
	REQUIRE(queue->QueueWork(&item));
	runReady();
	CHECK(log == std::vector<std::string>{"A"});
	CHECK(item.ranIn == HostRtos::findTask("Default"));
	deleteInTask(queue);
}

// ---------------- QueueWork ----------------

TEST_CASE_FIXTURE(RtosFixture, "work items run in the worker thread in the order they were queued") {
	std::vector<std::string> log;
	LoggingWorkItem a(log, "A"), b(log, "B"), c(log, "C");
	WorkQueue *queue = new WorkQueue("WQ");
	REQUIRE(queue->QueueWork(&a));
	REQUIRE(queue->QueueWork(&b));
	REQUIRE(queue->QueueWork(&c));
	CHECK(log.empty()); // Nothing runs before the worker thread
	runReady();
	CHECK(log == std::vector<std::string>{"A", "B", "C"});
	CHECK(a.ranIn == HostRtos::findTask("WQ"));
	deleteInTask(queue);
}

TEST_CASE_FIXTURE(RtosFixture, "a work item can be queued again after it ran") {
	std::vector<std::string> log;
	LoggingWorkItem a(log, "A");
	WorkQueue *queue = new WorkQueue("WQ");
	REQUIRE(queue->QueueWork(&a));
	runReady();
	REQUIRE(queue->QueueWork(&a));
	runReady();
	CHECK(log == std::vector<std::string>{"A", "A"});
	deleteInTask(queue);
}

TEST_CASE_FIXTURE(RtosFixture, "a work item to be freed is deleted after its run") {
	std::vector<std::string> log;
	bool destroyed = false;
	WorkQueue *queue = new WorkQueue("WQ");
	REQUIRE(queue->QueueWork(new LoggingWorkItem(log, "A", true, &destroyed)));
	runReady();
	CHECK(log == std::vector<std::string>{"A"});
	CHECK(destroyed);
	deleteInTask(queue);
}

TEST_CASE_FIXTURE(RtosFixture, "a work item not to be freed is kept after its run") {
	std::vector<std::string> log;
	bool destroyed = false;
	LoggingWorkItem item(log, "A", false, &destroyed);
	WorkQueue *queue = new WorkQueue("WQ");
	REQUIRE(queue->QueueWork(&item));
	runReady();
	CHECK(log == std::vector<std::string>{"A"});
	CHECK_FALSE(destroyed);
	deleteInTask(queue);
}

TEST_CASE_FIXTURE(RtosFixture, "queueing work to a full work queue waits in a task until the worker took an item") {
	std::vector<std::string> log;
	LoggingWorkItem a(log, "A"), b(log, "B");
	WorkQueue *queue = new WorkQueue("WQ", 256, 1, 1);
	runReady(); // The worker waits for work
	bool queuedB = false;
	WorkTestThread producer([&] {
		queue->QueueWork(&a);
		queuedB = queue->QueueWork(&b);
	});
	TaskHandle_t handle = producer.startResumable();
	HostRtos::runUntilBlocked(handle);
	CHECK_FALSE(queuedB);
	CHECK(HostRtos::isWaiting(handle)); // The queue of 1 item is full
	runReady();
	CHECK(queuedB);
	CHECK(log == std::vector<std::string>{"A", "B"});
	deleteInTask(queue);
}

TEST_CASE_FIXTURE(RtosFixture, "a work queue without a name holds at most its maximum number of items") {
	std::vector<std::string> log;
	LoggingWorkItem a(log, "A"), b(log, "B");
	WorkQueue *queue = new WorkQueue(256, 1, 1);
	runReady(); // The worker waits for work
	bool queuedB = false;
	WorkTestThread producer([&] {
		queue->QueueWork(&a);
		queuedB = queue->QueueWork(&b);
	});
	TaskHandle_t handle = producer.startResumable();
	HostRtos::runUntilBlocked(handle);
	CHECK_FALSE(queuedB);
	CHECK(HostRtos::isWaiting(handle)); // The queue of 1 item is full
	runReady();
	CHECK(queuedB);
	CHECK(log == std::vector<std::string>{"A", "B"});
	deleteInTask(queue);
}

// ---------------- Deletion ----------------

TEST_CASE_FIXTURE(RtosFixture, "deleting a work queue whose item queue is full waits for room for its stop item") {
	std::vector<std::string> log;
	LoggingWorkItem a(log, "A");
	WorkQueue *queue = new WorkQueue("WQ", 256, 1, 1);
	REQUIRE(queue->QueueWork(&a)); // Full; the worker did not run yet
	deleteInTask(queue);
	CHECK(log == std::vector<std::string>{"A"});
	CHECK(HostRtos::findTask("WQ") == nullptr);
}

TEST_CASE_FIXTURE(RtosFixture, "deleting a work queue runs the work queued before and ends the worker thread") {
	std::vector<std::string> log;
	LoggingWorkItem a(log, "A"), b(log, "B");
	WorkQueue *queue = new WorkQueue("WQ");
	REQUIRE(queue->QueueWork(&a));
	REQUIRE(queue->QueueWork(&b));
	bool deleted = false;
	WorkTestThread deleter([&] {
		delete queue;
		deleted = true;
	});
	deleter.startResumable();
	HostRtos::runUntilBlocked(deleter.GetHandle()); // Before the worker took any item
	CHECK(log.empty());
	runReady();
	REQUIRE(deleted);
	CHECK(log == std::vector<std::string>{"A", "B"});
	CHECK(HostRtos::findTask("WQ") == nullptr);
}

TEST_CASE_FIXTURE(RtosFixture, "deleting a work queue waits in a task until the worker thread has ended") {
	WorkQueue *queue = new WorkQueue("WQ");
	bool deleted = false;
	WorkTestThread task([&] {
		delete queue;
		deleted = true;
	});
	TaskHandle_t handle = task.startResumable();
	HostRtos::runUntilBlocked(handle); // Before the worker thread ran
	CHECK_FALSE(deleted);
	CHECK(HostRtos::isWaiting(handle));
	CHECK(HostRtos::findTask("WQ") != nullptr);
	runReady();
	CHECK(deleted);
	CHECK(HostRtos::findTask("WQ") == nullptr);
}

TEST_CASE_FIXTURE(RtosFixture, "deleting a work queue deletes its kernel queue and semaphores") {
	const size_t queuesBefore = HostRtos::queueCount();
	const size_t semaphoresBefore = HostRtos::semaphoreCount();
	WorkQueue *queue = new WorkQueue("WQ");
	CHECK(HostRtos::queueCount() == queuesBefore + 1);
	CHECK(HostRtos::semaphoreCount() == semaphoresBefore + 2); // ThreadComplete and the ThreadWaitSem of the worker
	deleteInTask(queue);
	CHECK(HostRtos::queueCount() == queuesBefore);
	CHECK(HostRtos::semaphoreCount() == semaphoresBefore);
}

// ---------------- Create failures ----------------

TEST_CASE_FIXTURE(RtosFixture, "a work queue whose item queue can not be created throws QueueCreateException and leaves nothing behind") {
	const size_t queuesBefore = HostRtos::queueCount();
	const size_t semaphoresBefore = HostRtos::semaphoreCount();
	const size_t tasksBefore = HostRtos::taskCount();
	HostRtos::failCreate(HostRtos::RtosObject::Queue);
	CHECK_THROWS_AS(new WorkQueue("WQ"), QueueCreateException);
	CHECK(HostRtos::queueCount() == queuesBefore);
	CHECK(HostRtos::semaphoreCount() == semaphoresBefore);
	CHECK(HostRtos::taskCount() == tasksBefore);
}

// ---------------- Suspected vendor defects (tier A: reported, never fixed) ----------------

TEST_CASE_FIXTURE(RtosFixture, "a work queue whose worker thread can not be created reports the failure"
				  * KNOWN_ISSUE("vendor V10: WorkQueue::WorkQueue (cworkqueue.cpp:78, :96) ignores the result of "
								"WorkerThread->Start(): the work queue is created without a worker, accepts work that never runs "
								"and ~WorkQueue waits forever on ThreadComplete. The first failing assertion is "
								"CHECK(failureReported)")) {
	HostRtos::failCreate(HostRtos::RtosObject::Task);
	std::vector<std::string> log;
	LoggingWorkItem item(log, "A");
	WorkQueue *queue = nullptr;
	bool failureReported = false;
	try {
		queue = new WorkQueue("WQ");
	} catch (const std::exception &) {
		failureReported = true; // Intended in the exception variant: the constructor throws
	}
	if (queue != nullptr) {
		failureReported = !queue->QueueWork(&item); // Or: the work queue refuses work it can never run
	}
	runReady();
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK(failureReported);
	CHECK(log.empty());
	if (queue != nullptr) {
		// Never deleted: without a worker ~WorkQueue would wait forever on ThreadComplete, and ~Thread would call
		// vTaskDelete with the handle xTaskCreate never wrote (vendor V2, undefined behaviour). Kept reachable for the
		// leak sanitizer; its queue and its two semaphores (ThreadComplete, ThreadWaitSem of the worker) stay.
		keepAlive(queue);
	}
	allowLeak(Leak::Queues, 1);
	allowLeak(Leak::Semaphores, 2);
}

TEST_CASE_FIXTURE(RtosFixture, "a work queue whose completion semaphore can not be created leaves no kernel queue behind"
				  * KNOWN_ISSUE_UB("vendor V11: WorkQueue::WorkQueue (cworkqueue.cpp:72-74, :90-92) creates the item Queue, "
								   "then ThreadComplete and the worker thread with new and does not free the earlier objects "
								   "when a later one throws (exception builds only; the F407 stops in configASSERT): the Queue "
								   "object and its kernel queue leak (also ThreadComplete when the worker's ThreadWaitSem "
								   "fails). Skipped in sanitizer builds (the leak sanitizer reports the Queue object). The "
								   "first failing assertion is CHECK(HostRtos::objectCounts().queues == atStart.queues)")) {
	HostRtos::failCreate(HostRtos::RtosObject::BinarySemaphore); // ThreadComplete
	bool threw = false;
	try {
		(void)new WorkQueue("WQ");
	} catch (const SemaphoreCreateException &) {
		threw = true;
	}
	REQUIRE(threw);
	(void)expectKernelFault("vSemaphoreDelete(NULL)"); // Vendor V3 (~Semaphore of the failed BinarySemaphore), test_csync_gaps.cpp
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK(HostRtos::objectCounts().queues == atStart.queues);
	CHECK(HostRtos::objectCounts().tasks == atStart.tasks);
}

}
