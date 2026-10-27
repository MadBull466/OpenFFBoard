/*
 * Host test replacement for the target FreeRTOSConfig.h
 * Only the switches evaluated by the firmware sources and the cpp_freertos wrappers are defined.
 */
#ifndef HOSTTEST_FREERTOSCONFIG_H_
#define HOSTTEST_FREERTOSCONFIG_H_

#define configUSE_TASK_NOTIFICATIONS 1
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_TIMERS 1
#define configTICK_RATE_HZ 1000
#define configMINIMAL_STACK_SIZE 128
#define configMAX_PRIORITIES 56
#define configMAX_TASK_NAME_LEN 16

#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_xTaskResumeFromISR 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskPrioritySet 1

#ifdef HOSTTEST_RTOS_EXT
/*
 * ffb_rtos_tests (fake FreeRTOS API F11, platform/src/host_freertos_ext.inc): the switches of the F407 target
 * (Targets/F407VG/Core/Inc/FreeRTOSConfig.h) that the cpp_freertos wrappers evaluate, plus the tick hook.
 */
#define configSUPPORT_STATIC_ALLOCATION 1	// F407 FreeRTOSConfig.h:67 (EventGroup(StaticEventGroup_t *))
#define configSUPPORT_DYNAMIC_ALLOCATION 1	// F407 FreeRTOSConfig.h:68
#define configUSE_TRACE_FACILITY 1			// F407 FreeRTOSConfig.h:78 (EventGroup::SetBitsFromISR)
#define configTIMER_TASK_PRIORITY 2			// F407 FreeRTOSConfig.h:101
#define configTIMER_QUEUE_LENGTH 10			// F407 FreeRTOSConfig.h:102
#define INCLUDE_xTimerPendFunctionCall 1	// F407 FreeRTOSConfig.h:126 (Tasklet, EventGroup::SetBitsFromISR)
#define INCLUDE_xTaskGetSchedulerState 1	// F407 FreeRTOSConfig.h:125
#define CPP_FREERTOS_CONDITION_VARIABLES	// F407 FreeRTOSConfig.h:187 (ConditionVariable, Thread::Wait)
/*
 * Host-vs-target: the F407 target has configUSE_TICK_HOOK 0 (FreeRTOSConfig.h:70), so ctickhook.cpp compiles to
 * nothing there. It is on here so that the TickHook wrapper (tier A) can be tested at all.
 */
#define configUSE_TICK_HOOK 1
#endif

#endif
