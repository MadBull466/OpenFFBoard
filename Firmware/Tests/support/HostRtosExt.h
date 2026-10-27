/*
 * HostRtosExt.h
 *
 * Test control of the fake FreeRTOS API F11 (platform/src/host_freertos_ext.inc), ffb_rtos_tests only
 * (compiled with HOSTTEST_RTOS_EXT). Extends HostRtos.h; everything there still applies, with these differences:
 *
 * Model (FreeRTOS V10.3.1 of the F407 target, Targets/F407VG/Middlewares/Third_Party/FreeRTOS/Source):
 *  - Queues, software timers with the timer service task ("Tmr Svc", priority 2, command queue of 10), event groups,
 *    mutex ownership, recursive mutexes, counting semaphores, critical sections / BASEPRI, scheduler suspension, the
 *    tick hook and the RTOS heap (pvPortMalloc / vPortFree, heap_4: pvPortMalloc(0) returns NULL).
 *  - A new task is ready: runUntilIdle starts every task that was never activated (on the target a created task is
 *    in the Ready state). A task blocked in a modelled wait (queue, semaphore, mutex, event group, notification, timer
 *    service) becomes ready again when its condition holds.
 *  - Blocking with a timeout, against simulated time:
 *      resumable task (HostRtos::makeResumable): parks; continues when the condition holds or ticksToWait ticks of
 *        simulated time have passed since the call (an activation before that parks again);
 *      other task (restart model): leaves its function (HostRtos::TaskBlocked) and is started again by runUntilIdle
 *        once the condition holds; a finite timeout can not expire (the call is made again from the start);
 *      test thread (no task): portMAX_DELAY on an unavailable object is recorded ("would block forever: ...") and
 *        fails; a finite timeout advances the simulated time tick by tick (tick hooks run) until the condition holds
 *        or the timeout has passed. No task runs during that time.
 *    A timeout of 0 never blocks.
 *  - The simulated time advances through HostPlatform::advanceMs / advanceUs / HAL_Delay / vTaskDelay: every 1 ms
 *    boundary is one SysTick: vApplicationTickHook() (ctickhook.cpp) runs in interrupt context (IPSR 15). setTimeUs
 *    and HostPlatform::reset() jump without ticks.
 *  - Software timers follow timers.c: xTimerStart & co. only queue a command; the timer service task executes it when
 *    it runs (runUntilIdle, RtosFixture::advance). Expired timers are processed before the queued commands; an
 *    auto-reload timer whose service task ran late catches up (one callback per missed period). Timer callbacks and
 *    pended functions must not block: a blocking call inside one is recorded as a kernel fault.
 *  - Everything FreeRTOS would stop in configASSERT, and every call the fake does not model, is recorded in
 *    HostRtos::kernelFaults() ("configASSERT: ...", "unmodelled: ...", "would block forever: ...") and the call
 *    returns its failure value instead of hanging. RtosFixture fails a test with recorded faults.
 *  - Deleted queues, semaphores, event groups and timers are detected by handle (registry with creation generation):
 *    a later use is recorded instead of touching freed memory. A timer deleted by the timer service stays allocated
 *    (zombie) until HostRtos::reset().
 * Not modelled: priorities between tasks (no preemption; pxHigherPriorityTaskWoken is computed from the priorities),
 * priority inheritance, the tick count overflow (recorded), queue sets, static creation other than event groups,
 * stream / message buffers, task notifications other than give/take.
 */
#ifndef HOSTTEST_HOSTRTOSEXT_H_
#define HOSTTEST_HOSTRTOSEXT_H_

#include "HostRtos.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace HostRtos {

/** Kernel objects whose creation can be made to fail (failCreate) */
enum class RtosObject {
	Task,			   //!< xTaskCreate returns errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY (Thread::Start returns false)
	Queue,			   //!< xQueueCreate returns NULL
	BinarySemaphore,   //!< xSemaphoreCreateBinary
	CountingSemaphore, //!< xSemaphoreCreateCounting
	Mutex,			   //!< xSemaphoreCreateMutex
	RecursiveMutex,	   //!< xSemaphoreCreateRecursiveMutex
	Timer,			   //!< xTimerCreate
	EventGroup,		   //!< xEventGroupCreate
	StaticEventGroup,  //!< xEventGroupCreateStatic
	Heap,			   //!< pvPortMalloc returns NULL (the kernel objects above do not use the heap in the fake)
	Count
};

/**
 * Makes creation calls of `kind` fail: the calls number after+1 .. after+count from now (0 = the next call).
 * Cleared by reset(). Internal objects of the fake (timer service task and queue) are never affected.
 */
void failCreate(RtosObject kind, uint32_t after = 0, uint32_t count = 1);
/** Creation calls of `kind` since the last reset() (failed ones included) */
uint32_t createCalls(RtosObject kind);

/** Returns and clears kernelFaults() */
std::vector<std::string> takeKernelFaults();
/** Removes the first kernel fault containing `part`. @return true if there was one */
bool takeKernelFault(const std::string &part);

/** Critical sections and interrupt masking (task.h macros of the Cortex-M4F port) since the last reset() */
struct CriticalStats {
	uint32_t enter = 0;				//!< taskENTER_CRITICAL (vPortEnterCritical)
	uint32_t exit = 0;				//!< taskEXIT_CRITICAL (vPortExitCritical)
	uint32_t nesting = 0;			//!< Current nesting (uxCriticalNesting)
	uint32_t maxNesting = 0;		//!< Highest nesting
	uint32_t raiseBasepri = 0;		//!< taskENTER_CRITICAL_FROM_ISR (ulPortRaiseBASEPRI)
	uint32_t disableInterrupts = 0; //!< taskDISABLE_INTERRUPTS (vPortRaiseBASEPRI)
	uint32_t setBasepri = 0;		//!< taskEXIT_CRITICAL_FROM_ISR / taskENABLE_INTERRUPTS (vPortSetBASEPRI)
	uint32_t basepri = 0;			//!< Current BASEPRI value (0x50 = configMAX_SYSCALL_INTERRUPT_PRIORITY: masked)
};
const CriticalStats &criticalStats();
/** Current vTaskSuspendAll nesting (xTaskGetSchedulerState returns taskSCHEDULER_SUSPENDED while > 0) */
uint32_t schedulerSuspendNesting();
/** SysTicks since the last reset() (= vApplicationTickHook calls) */
uint32_t tickCount();

/** The timer service task, nullptr until the first timer / pended function call after reset() */
TaskHandle_t timerServiceTask();
/** Commands (and pended function calls) queued for the timer service task */
size_t timerCommandsPending();
/** Timers in the active list of the timer service */
size_t activeTimerCount();
struct TimerServiceStats {
	uint32_t timerCallbacks = 0; //!< Timer callbacks executed
	uint32_t pendedCalls = 0;	 //!< Pended functions executed (xTimerPendFunctionCall, ...FromISR)
	uint32_t commands = 0;		 //!< Timer commands processed
};
const TimerServiceStats &timerServiceStats();

/** Existing (created and not deleted) objects. Timers count until the service processed their delete command */
size_t queueCount(); //!< Without the queue of the timer service
size_t timerCount();
size_t eventGroupCount();

/** Blocks returned by pvPortMalloc and not freed (all time, not reset), and pvPortMalloc calls that returned NULL */
size_t heapLiveBlocks();
uint32_t heapFailedCalls();

/** True while the task waits in a modelled blocking call (queue, semaphore, event group, notification, timer service) */
bool isWaiting(TaskHandle_t task);

} // namespace HostRtos

#endif
