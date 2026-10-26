/*
 * host_freertos.cpp
 *
 * Fake FreeRTOS kernel for the host unit tests. See support/HostRtos.h for the concept.
 *
 * Resumable tasks (HostRtos::makeResumable, opt-in per task): the task function runs on its own host thread. Only one
 * thread runs at a time: the test thread hands over to the task in runUntilBlocked and waits until the task parks in a
 * blocking kernel call (instead of leaving its function with TaskBlocked); the next activation continues there, like
 * the task on the target continues after the blocking call returns. Tasks that are not resumable keep the restart
 * semantics. The hand-over (mutex + condition variable) keeps the execution sequential and deterministic.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "HostRtos.h"
#include "HostPlatform.h"
#include "cmsis_compiler.h"

#include <algorithm>
#include <condition_variable>
#include <cstdlib>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

/** Host thread of a resumable task (HostRtos::makeResumable) and the hand-over state between it and the test thread */
struct ResumableContext {
	std::thread thread;
	std::mutex mutex;
	std::condition_variable cv;
	bool taskTurn = false;		 //!< true while the task thread runs, false while the thread that activated it runs
	bool started = false;		 //!< The host thread exists (first activation done)
	bool finished = false;		 //!< The task function was left (returned, or by an exception)
	bool restart = false;		 //!< Left by HostRtos::TaskBlocked: the next activation starts the function again
	bool kill = false;			 //!< vTaskDelete of the parked task: unwind its thread
	std::function<bool()> ready; //!< While parked: true if the condition the task waits for is met (runUntilIdle)
	std::exception_ptr error;	 //!< Exception (other than the unwinding on delete) that left the task function
};

struct HostTask {
	TaskFunction_t fn = nullptr;
	void *param = nullptr;
	std::string name;
	UBaseType_t priority = 0;
	uint32_t notifications = 0;
	bool suspended = false;
	std::unique_ptr<ResumableContext> resumable; //!< Set by HostRtos::makeResumable
	bool deleted = false; //!< A resumable task deleted itself on its host thread: freed when the activation ends
};

struct HostSemaphore {
	UBaseType_t count = 0;
	UBaseType_t maxCount = 1;
	uint64_t generation = 0; //!< Unique per created semaphore: a parked task's wait survives a reused address
};

namespace {
std::vector<HostTask *> &tasks() {
	static std::vector<HostTask *> list;
	return list;
}
HostTask *currentTask = nullptr;
uint32_t delayBudgetMs = 1000;
uint32_t delayBudgetRemaining = 0;
size_t liveSemaphores = 0; //!< Created and not deleted semaphores (HostRtos::semaphoreCount)
HostRtos::ResumeStats resumeCalls; //!< Calls of the task resume functions (HostRtos::resumeStats)
HostRtos::HeapStats heapCalls; //!< Calls of the heap functions (HostRtos::heapStats)
std::map<const HostSemaphore *, uint64_t> &semaphores() { //!< Created and not deleted semaphores with their generation (a parked task may wait for a deleted one)
	static std::map<const HostSemaphore *, uint64_t> live;
	return live;
}
uint64_t semaphoreGenerations = 0;
/** True if `sem` is the live semaphore of this generation (not deleted, address not reused by a newer one) */
bool semaphoreLive(const HostSemaphore *sem, uint64_t generation) {
	auto it = semaphores().find(sem);
	return it != semaphores().end() && it->second == generation;
}

std::vector<std::string> kernelFaultList; //!< HostRtos::kernelFaults
HostRtos::FaultReporter faultReporter = nullptr;
/** Records a kernel call the fake can not complete (HostRtos::kernelFaults) and passes it to the reporter */
void kernelFault(const std::string &what) {
	kernelFaultList.push_back(what);
	if (faultReporter != nullptr) {
		faultReporter(what);
	}
}

bool isRegistered(HostTask *task) {
	return std::find(tasks().begin(), tasks().end(), task) != tasks().end();
}

/** Thrown on the host thread of a parked resumable task to unwind it when the task is deleted. Never leaves that thread */
struct ResumableTaskKilled {};

/** The resumable task running on the calling thread, nullptr if the caller is not a resumable task */
HostTask *resumableCurrent() {
	if (currentTask == nullptr || !currentTask->resumable || std::this_thread::get_id() != currentTask->resumable->thread.get_id()) {
		return nullptr;
	}
	return currentTask;
}

/** Test side of the hand-over: lets the task thread run until it parks or leaves its function */
void handToTask(ResumableContext &ctx) {
	std::unique_lock<std::mutex> lock(ctx.mutex);
	ctx.taskTurn = true;
	ctx.cv.notify_all();
	ctx.cv.wait(lock, [&ctx] { return !ctx.taskTurn; });
}

/**
 * Task side: parks the calling resumable task until its next activation (the thread that activated it continues).
 * `ready` tells runUntilIdle whether the task can continue. Throws ResumableTaskKilled if the task was deleted meanwhile.
 * Once the task is being killed it never parks again (killResumable waits for the thread to end): code running during
 * the unwind (a destructor taking a semaphore, a catch(...) that continues) gets ResumableTaskKilled again, or, while an
 * exception is already propagating (throwing would terminate), `false`: the caller ends its wait without the resource.
 * @return true after a normal activation
 */
bool parkCurrentTask(std::function<bool()> ready) {
	HostTask *self = currentTask;
	ResumableContext &ctx = *self->resumable;
	if (ctx.kill) {
		if (std::uncaught_exceptions() > 0) {
			return false;
		}
		throw ResumableTaskKilled();
	}
	ctx.ready = std::move(ready);
	{
		std::unique_lock<std::mutex> lock(ctx.mutex);
		ctx.taskTurn = false;
		ctx.cv.notify_all();
		ctx.cv.wait(lock, [&ctx] { return ctx.taskTurn; });
	}
	ctx.ready = nullptr;
	if (ctx.kill) {
		if (std::uncaught_exceptions() > 0) {
			return false;
		}
		throw ResumableTaskKilled();
	}
	return true;
}

/** Body of the host thread of a resumable task */
void resumableThreadMain(TaskFunction_t fn, void *param, ResumableContext *ctx) {
	{
		std::unique_lock<std::mutex> lock(ctx->mutex);
		ctx->cv.wait(lock, [ctx] { return ctx->taskTurn; });
	}
	if (!ctx->kill) {
		try {
			fn(param);
		} catch (const ResumableTaskKilled &) {
		} catch (const HostRtos::TaskBlocked &) {
			ctx->restart = true; // Left like a task that is not resumable: start again at the next activation
		} catch (...) {
			ctx->error = std::current_exception();
		}
	}
	std::lock_guard<std::mutex> lock(ctx->mutex);
	ctx->finished = true;
	ctx->taskTurn = false;
	ctx->cv.notify_all();
}

/** Activation of a resumable task (runUntilBlocked). The caller checked that no task is running */
bool runResumable(HostTask *task) {
	ResumableContext &ctx = *task->resumable;
	if (ctx.finished) {
		return false; // The task function returned (not allowed on FreeRTOS); nothing left to run
	}
	currentTask = task;
	delayBudgetRemaining = delayBudgetMs;
	if (!ctx.started) {
		ctx.started = true;
		ctx.thread = std::thread(resumableThreadMain, task->fn, task->param, &ctx);
	}
	handToTask(ctx);
	if (task->deleted && !ctx.finished) {
		ctx.kill = true; // Deleted itself and parked again: unwind it
		handToTask(ctx);
	}
	currentTask = nullptr;
	std::exception_ptr error = ctx.error;
	ctx.error = nullptr;
	if (ctx.finished) {
		ctx.thread.join();
		if (ctx.restart && !task->deleted) {
			task->resumable = std::make_unique<ResumableContext>(); // Fresh thread at the next activation
		}
	}
	if (task->deleted) {
		delete task; // Deleted itself (vTaskDelete on its own thread, e.g. cpp_freertos TaskFunctionAdapter after Run)
	}
	if (error) {
		std::rethrow_exception(error);
	}
	return true;
}

/**
 * Deletes a resumable task from another thread (test or other task): a parked task is unwound on its host thread
 * (ResumableTaskKilled), then the thread is joined.
 */
void killResumable(HostTask *task) {
	ResumableContext &ctx = *task->resumable;
	if (ctx.started && !ctx.finished) {
		HostTask *caller = currentTask;
		currentTask = task;
		ctx.kill = true;
		handToTask(ctx);
		currentTask = caller;
	}
	if (ctx.thread.joinable()) {
		ctx.thread.join();
	}
	if (ctx.error) {
		// Called from vTaskDelete, usually in ~cpp_freertos::Thread (noexcept): rethrowing would terminate. Recorded instead
		std::string what = "exception left the function of a resumable task while it was deleted";
		try {
			std::rethrow_exception(ctx.error);
		} catch (const std::exception &e) {
			what += std::string(": ") + e.what();
		} catch (...) {
		}
		ctx.error = nullptr;
		kernelFault(what);
	}
}
} // namespace

// ---------------- Test control ----------------
namespace HostRtos {

void reset() {
	for (HostTask *task : tasks()) {
		task->notifications = 0;
	}
	delayBudgetMs = 1000;
	currentTask = nullptr;
	resumeCalls = ResumeStats();
	heapCalls = HeapStats();
	kernelFaultList.clear();
}

const std::vector<std::string> &kernelFaults() { return kernelFaultList; }
void setFaultReporter(FaultReporter reporter) { faultReporter = reporter; }

bool runUntilBlocked(TaskHandle_t task) {
	if (!isRegistered(task) || currentTask != nullptr) {
		return false;
	}
	if (task->resumable) {
		return runResumable(task);
	}
	currentTask = task;
	delayBudgetRemaining = delayBudgetMs;
	try {
		task->fn(task->param);
	} catch (const TaskBlocked &) {
	}
	currentTask = nullptr;
	return true;
}

/**
 * True if runUntilIdle activates the task: a notification is pending; for a parked resumable task instead: the
 * condition it waits for is met (notification, semaphore available, resumed after a suspend)
 */
static bool isReady(HostTask *task) {
	if (task->resumable && task->resumable->started) {
		return !task->resumable->finished && task->resumable->ready && task->resumable->ready();
	}
	return task->notifications > 0;
}

uint32_t runUntilIdle(uint32_t maxActivations) {
	uint32_t activations = 0;
	bool ranAny = true;
	while (ranAny && activations < maxActivations) {
		ranAny = false;
		// Tasks may be created or deleted while running. Iterate over a copy
		std::vector<HostTask *> snapshot = tasks();
		for (HostTask *task : snapshot) {
			if (isRegistered(task) && !task->suspended && isReady(task)) {
				runUntilBlocked(task);
				activations++;
				ranAny = true;
			}
		}
	}
	return activations;
}

uint32_t pendingNotifications(TaskHandle_t task) {
	return isRegistered(task) ? task->notifications : 0;
}

size_t taskCount() {
	return tasks().size();
}

TaskHandle_t findTask(const char *name) {
	for (HostTask *task : tasks()) {
		if (name != nullptr && task->name == name) {
			return task;
		}
	}
	return nullptr;
}

bool isSuspended(TaskHandle_t task) {
	return isRegistered(task) && task->suspended;
}

void makeResumable(TaskHandle_t task) {
	if (isRegistered(task) && !task->resumable) {
		task->resumable = std::make_unique<ResumableContext>();
	}
}

TaskHandle_t makeResumable(const char *name) {
	TaskHandle_t task = findTask(name);
	makeResumable(task);
	return task;
}

bool isResumable(TaskHandle_t task) { return isRegistered(task) && task->resumable != nullptr; }

void setDelayBudgetMs(uint32_t ms) {
	delayBudgetMs = ms;
}

size_t semaphoreCount() {
	return liveSemaphores;
}

ResumeStats &resumeStats() {
	return resumeCalls;
}

HeapStats &heapStats() {
	return heapCalls;
}

} // namespace HostRtos

// ---------------- Kernel API ----------------
extern "C" {

void *pvPortMalloc(size_t size) {
	void *p = malloc(size);
	heapCalls.mallocCalls++;
	heapCalls.lastMallocSize = size;
	heapCalls.lastMallocResult = p;
	return p;
}
void vPortFree(void *ptr) {
	heapCalls.freeCalls++;
	heapCalls.lastFreed = ptr;
	free(ptr);
}
size_t xPortGetFreeHeapSize(void) { return 0x10000; }
size_t xPortGetMinimumEverFreeHeapSize(void) { return 0x8000; }

BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t, void *param, UBaseType_t prio, TaskHandle_t *handle) {
	HostTask *task = new HostTask();
	task->fn = fn;
	task->param = param;
	task->name = name ? name : "";
	task->priority = prio;
	tasks().push_back(task);
	if (handle) {
		*handle = task;
	}
	return pdPASS;
}

void vTaskDelete(TaskHandle_t task) {
	if (task == nullptr) {
		task = currentTask;
	}
	auto it = std::find(tasks().begin(), tasks().end(), task);
	if (it != tasks().end()) {
		tasks().erase(it);
		if (task->resumable && resumableCurrent() == task) {
			// A resumable task deletes itself: its host thread continues until the task function is left (like the
			// non-resumable fake, vTaskDelete returns); freed by runUntilBlocked when the activation ends
			task->deleted = true;
			return;
		}
		if (task->resumable) {
			killResumable(task);
		}
		if (currentTask == task) {
			currentTask = nullptr;
		}
		delete task;
	}
}

void vTaskDelay(TickType_t ticks) {
	HostPlatform::advanceMs(ticks);
	if (currentTask != nullptr) {
		if (ticks >= delayBudgetRemaining) {
			if (resumableCurrent() != nullptr) {
				parkCurrentTask([] { return false; }); // Delay budget used up: continues after the delay at the next runUntilBlocked
				return;
			}
			throw HostRtos::TaskBlocked();
		}
		delayBudgetRemaining -= ticks;
	}
}

void vTaskDelayUntil(TickType_t *previousWakeTime, TickType_t increment) {
	TickType_t now = xTaskGetTickCount();
	TickType_t wake = *previousWakeTime + increment;
	*previousWakeTime = wake;
	if (wake > now) {
		vTaskDelay(wake - now);
	}
}

TickType_t xTaskGetTickCount(void) { return (TickType_t)(HostPlatform::nowUs() / 1000); }
TickType_t xTaskGetTickCountFromISR(void) { return xTaskGetTickCount(); }
void vTaskStartScheduler(void) {}
void vTaskEndScheduler(void) {}
void vTaskSuspend(TaskHandle_t task) {
	if (task == nullptr) {
		task = currentTask; // NULL suspends the calling task
	}
	if (isRegistered(task)) task->suspended = true;
	if (task != nullptr && task == currentTask) {
		// A task that suspends itself blocks until another task resumes it
		if (resumableCurrent() == task) {
			while (task->suspended && parkCurrentTask([task] { return !task->suspended; })) {
			}
			return;
		}
		throw HostRtos::TaskBlocked();
	}
}
void vTaskResume(TaskHandle_t task) {
	resumeCalls.resume++;
	if (__get_IPSR() != 0U) {
		resumeCalls.resumeInIsr++; // Not allowed on the target. Recorded, execution continues
	}
	if (isRegistered(task)) task->suspended = false;
}
BaseType_t xTaskResumeFromISR(TaskHandle_t task) {
	resumeCalls.resumeFromIsr++;
	if (isRegistered(task)) task->suspended = false;
	return pdFALSE;
}
void vTaskSuspendAll(void) {}
BaseType_t xTaskResumeAll(void) { return pdFALSE; }
UBaseType_t uxTaskPriorityGet(TaskHandle_t task) { return isRegistered(task) ? task->priority : 0; }
UBaseType_t uxTaskPriorityGetFromISR(TaskHandle_t task) { return uxTaskPriorityGet(task); }
void vTaskPrioritySet(TaskHandle_t task, UBaseType_t prio) {
	if (isRegistered(task)) task->priority = prio;
}
char *pcTaskGetName(TaskHandle_t task) {
	static char empty[1] = {0};
	return isRegistered(task) ? task->name.data() : empty;
}
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return currentTask; }
BaseType_t xTaskGetSchedulerState(void) { return taskSCHEDULER_RUNNING; }
UBaseType_t uxTaskGetNumberOfTasks(void) { return tasks().size(); }
UBaseType_t uxTaskGetSystemState(TaskStatus_t *, UBaseType_t, uint32_t *) { return 0; }
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t) { return 0; }
void vTaskList(char *buf) {
	if (buf) *buf = 0;
}
void vTaskGetRunTimeStats(char *buf) {
	if (buf) *buf = 0;
}

BaseType_t xTaskNotifyGive(TaskHandle_t task) {
	if (isRegistered(task)) {
		task->notifications++;
	}
	return pdPASS;
}

void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *higherPriorityTaskWoken) {
	xTaskNotifyGive(task);
	if (higherPriorityTaskWoken) {
		*higherPriorityTaskWoken = pdFALSE;
	}
}

uint32_t ulTaskNotifyTake(BaseType_t clearOnExit, TickType_t ticksToWait) {
	if (HostTask *self = resumableCurrent()) {
		// Resumable task: waits (parks) until a notification is pending. With a finite timeout it parks once: an
		// activation without a pending notification ends the wait as a timeout (returns 0). No simulated time passes
		if (self->notifications == 0 && ticksToWait != 0) {
			while (parkCurrentTask([self] { return self->notifications > 0; }) && self->notifications == 0 && ticksToWait == portMAX_DELAY) {
			}
		}
		uint32_t count = self->notifications;
		if (count > 0) {
			self->notifications = clearOnExit ? 0 : count - 1;
		}
		return count;
	}
	if (currentTask == nullptr || currentTask->notifications == 0) {
		// Nothing pending. The task would block here
		throw HostRtos::TaskBlocked();
	}
	uint32_t count = currentTask->notifications;
	currentTask->notifications = clearOnExit ? 0 : count - 1;
	return count;
}

// ---------------- Semaphores ----------------
static SemaphoreHandle_t createSemaphore(UBaseType_t count, UBaseType_t maxCount) {
	liveSemaphores++;
	HostSemaphore *sem = new HostSemaphore{count, maxCount, ++semaphoreGenerations};
	semaphores()[sem] = sem->generation;
	return sem;
}
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return createSemaphore(0, 1); }
SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t maxCount, UBaseType_t initialCount) { return createSemaphore(initialCount, maxCount); }
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return createSemaphore(1, 1); }
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void) { return createSemaphore(0xffff, 0xffff); }
void vSemaphoreDelete(SemaphoreHandle_t sem) {
	if (sem != nullptr) {
		liveSemaphores--;
		semaphores().erase(sem);
	}
	delete sem;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticksToWait) {
	if (sem->count == 0 && ticksToWait != 0 && resumableCurrent() != nullptr) {
		// Resumable task: waits (parks) until the semaphore is available. With a finite timeout it parks once: an
		// activation while the semaphore is still taken ends the wait as a timeout. No simulated time passes
		const uint64_t generation = sem->generation;
		bool parked = true;
		while (parked) {
			parked = parkCurrentTask([sem, generation] { return semaphoreLive(sem, generation) && sem->count > 0; });
			if (!parked) {
				return pdFALSE; // Killed while an exception propagates (see parkCurrentTask)
			}
			if (sem->count > 0 || ticksToWait != portMAX_DELAY) {
				break;
			}
		}
	}
	if (sem->count > 0) {
		sem->count--;
		return pdTRUE;
	}
	if (currentTask != nullptr && ticksToWait == portMAX_DELAY) {
		throw HostRtos::TaskBlocked(); // Would wait forever
	}
	if (currentTask == nullptr && ticksToWait == portMAX_DELAY) {
		// Test thread (no task): the calling task would block forever on the target. Returns pdFALSE, recorded
		kernelFault("would block forever: xSemaphoreTake(portMAX_DELAY) of an unavailable semaphore outside a task");
	}
	return pdFALSE;
}
BaseType_t xSemaphoreTakeFromISR(SemaphoreHandle_t sem, BaseType_t *higherPriorityTaskWoken) {
	if (higherPriorityTaskWoken) *higherPriorityTaskWoken = pdFALSE;
	if (sem->count > 0) {
		sem->count--;
		return pdTRUE;
	}
	return pdFALSE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) {
	if (sem->count < sem->maxCount) {
		sem->count++;
		return pdTRUE;
	}
	return pdFALSE;
}
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t sem, BaseType_t *higherPriorityTaskWoken) {
	if (higherPriorityTaskWoken) *higherPriorityTaskWoken = pdFALSE;
	return xSemaphoreGive(sem);
}
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t sem, TickType_t ticksToWait) { return xSemaphoreTake(sem, ticksToWait); }
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t sem) { return xSemaphoreGive(sem); }
UBaseType_t uxSemaphoreGetCount(SemaphoreHandle_t sem) { return sem->count; }

} // extern "C"
