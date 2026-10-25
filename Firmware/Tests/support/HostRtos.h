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
 */
#ifndef HOSTTEST_HOSTRTOS_H_
#define HOSTTEST_HOSTRTOS_H_

#include <cstdint>
#include <cstddef>
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
 * Maximum time in ms a task may spend in vTaskDelay during one activation before
 * it is treated as blocked. Prevents endless polling loops. Default 1000.
 */
void setDelayBudgetMs(uint32_t ms);

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
