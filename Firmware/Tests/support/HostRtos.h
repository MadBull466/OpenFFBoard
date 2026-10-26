/*
 * HostRtos.h
 *
 * Test control interface of the fake FreeRTOS kernel (platform/src/host_freertos.cpp).
 *
 * There is no scheduler and no preemption in the host tests. Firmware threads
 * (cpp_freertos::Thread) are registered when started but never run on their own.
 * A test runs a thread explicitly until the thread would block:
 *
 *   itf.addBuf(...);                      // wakes up the command thread
 *   HostRtos::runUntilBlocked(cmdThread); // executes Run() until it waits again
 *
 * or lets all woken threads run until the system is idle:
 *
 *   HostRtos::runUntilIdle();
 *
 * A thread "blocks" when it waits for a notification that is not pending, takes an
 * unavailable semaphore forever or used up its delay budget. The thread function is
 * then left by an exception and is restarted from the beginning of Run() the next time.
 * Threads following the usual `while(true){ wait; work; }` pattern behave identical to the target.
 * Threads that wait in several places per loop need a resumable task (makeResumable, opt-in per task).
 */
#ifndef HOSTTEST_HOSTRTOS_H_
#define HOSTTEST_HOSTRTOS_H_

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include "FreeRTOS.h"
#include "task.h"

namespace HostRtos {

/** Thrown by blocking kernel calls to leave a thread function. Never catch this in tests. */
struct TaskBlocked {};

/** Deletes pending notifications of all tasks and restores default settings. Does not delete tasks. */
void reset();

/**
 * Runs the function of a task until it blocks.
 * @return true if the task had a pending notification or did any delay, false if handle is invalid
 */
bool runUntilBlocked(TaskHandle_t task);

/** Runs a firmware thread until it blocks */
template <class T>
bool runUntilBlocked(T &thread) { return runUntilBlocked(thread.GetHandle()); }

/**
 * Runs all tasks with pending notifications until no task has a notification pending anymore.
 * @return amount of task activations
 */
uint32_t runUntilIdle(uint32_t maxActivations = 1000);

/** Pending notification count of a task */
uint32_t pendingNotifications(TaskHandle_t task);
template <class T>
uint32_t pendingNotifications(T &thread) { return pendingNotifications(thread.GetHandle()); }

/** Amount of currently registered (started and not deleted) tasks */
size_t taskCount();

/** First registered task with this name, nullptr if there is none. For threads whose handle is not accessible */
TaskHandle_t findTask(const char *name);

/**
 * True while a task is suspended (vTaskSuspend) and not resumed (vTaskResume/xTaskResumeFromISR).
 * A task that suspends itself while running blocks (leaves its function like any other blocking call).
 */
bool isSuspended(TaskHandle_t task);

/**
 * Makes a task resumable (opt-in, ffb_target_tests P6 batch 0; existing tests keep the restart semantics).
 * A resumable task does not leave its function when it blocks: its function runs on its own host thread, which parks
 * in the blocking kernel call, and the next activation (runUntilBlocked / runUntilIdle) continues after that call,
 * like on the target. Needed for threads that block in more than one place per loop, e.g.
 *   requestSem.Take(); port.receive_DMA(...); WaitForNotification(); process();   (EncoderBissC, MtEncoderSPI)
 * With the restart semantics such a thread would start again at Take() after the DMA completion and never process.
 *
 * Execution stays sequential and deterministic: the activating thread waits until the task parks again.
 * Blocking calls of a resumable task:
 *  - ulTaskNotifyTake / WaitForNotification, xSemaphoreTake (also Take of cpp_freertos semaphores and mutexes) with
 *    portMAX_DELAY: park until a notification is pending / the semaphore is available (an activation while the
 *    condition is not met parks again at once). With a finite timeout: park once; an activation while the condition
 *    is still not met ends the wait as a timeout (0 / pdFALSE). No simulated time passes in either case. A timeout of 0
 *    does not park.
 *  - vTaskSuspend(self): park until resumed. vTaskDelay beyond the delay budget: park (continues after the delay at
 *    the next runUntilBlocked; runUntilIdle does not activate it).
 * runUntilIdle activates a parked resumable task when the condition it waits for is met (not only on notifications).
 * A resumable task that was never activated is started by runUntilIdle only if a notification is pending (like the
 * restart model); a first wait on a semaphore needs an explicit runUntilBlocked. A wait for a semaphore that is
 * deleted (or whose address is reused by a newer semaphore: checked with a creation generation) never becomes ready.
 * A task being deleted never parks again: blocking calls made while its function unwinds (destructors, catch(...))
 * leave immediately (ResumableTaskKilled, or "not available / timeout" while an exception propagates). An exception
 * that leaves the function during the delete can not be rethrown (vTaskDelete runs in ~Thread): it is recorded in
 * kernelFaults().
 * Deleting the task (vTaskDelete, e.g. ~cpp_freertos::Thread when the firmware object is deleted) unwinds the parked
 * function on its host thread and joins the thread. Exceptions leaving the task function are rethrown by
 * runUntilBlocked; HostRtos::TaskBlocked leaving it means "start again at the next activation".
 * Call it before the first activation of the task (e.g. right after the firmware object started its thread).
 */
void makeResumable(TaskHandle_t task);
/** makeResumable(findTask(name)). @return the task, nullptr if there is no task with this name */
TaskHandle_t makeResumable(const char *name);
/** True if the task is registered and resumable */
bool isResumable(TaskHandle_t task);

/**
 * Maximum time in ms a task may spend in vTaskDelay during one activation before
 * it is treated as blocked. Prevents endless polling loops. Default 1000.
 */
void setDelayBudgetMs(uint32_t ms);

/**
 * Kernel calls the fake can not complete, cleared by reset():
 *  - "would block forever: xSemaphoreTake(portMAX_DELAY) ... outside a task": an infinite take of an unavailable
 *    semaphore from the test thread (no current task). On the target the calling task would block forever; the fake
 *    returns pdFALSE. (ulTaskNotifyTake outside a task throws TaskBlocked into the test, as before.)
 *  - an exception that left a resumable task's function while the task was deleted.
 * ffb_target_tests passes every entry to HostTarget::state().unmodelledHalCalls (setFaultReporter in host_target.cpp),
 * so REQUIRE_ALL_HAL_MODELLED() and the TargetFixture check fail on it. ffb_unit_tests / ffb_flash_tests only record.
 */
const std::vector<std::string> &kernelFaults();
using FaultReporter = void (*)(const std::string &what);
/** Called for every new kernelFaults() entry (not reset by reset()). nullptr: only recorded */
void setFaultReporter(FaultReporter reporter);

/** Amount of currently existing kernel semaphores and mutexes (created and not deleted with vSemaphoreDelete) */
size_t semaphoreCount();

/** Calls of the task resume functions since the last reset() */
struct ResumeStats {
	uint32_t resume = 0;		//!< vTaskResume calls
	uint32_t resumeFromIsr = 0; //!< xTaskResumeFromISR calls
	uint32_t resumeInIsr = 0;	//!< vTaskResume calls in (simulated) interrupt context: not allowed on the target
};
ResumeStats &resumeStats();

/** Calls of the RTOS heap functions since the last reset() (the fake heap forwards to the C library allocator) */
struct HeapStats {
	uint32_t mallocCalls = 0; //!< pvPortMalloc calls
	uint32_t freeCalls = 0;	  //!< vPortFree calls
	size_t lastMallocSize = 0; //!< Size of the last pvPortMalloc request
	void *lastMallocResult = nullptr; //!< Pointer returned by the last pvPortMalloc
	void *lastFreed = nullptr;		  //!< Pointer passed to the last vPortFree
};
HeapStats &heapStats();

} // namespace HostRtos

#endif
