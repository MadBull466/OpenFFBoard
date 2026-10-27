/*
 * Host test replacement for FreeRTOS event_groups.h.
 * ffb_unit_tests, ffb_target_tests and ffb_flash_tests do not use event groups (only the types).
 * ffb_rtos_tests (HOSTTEST_RTOS_EXT) gets the event group API of FreeRTOS V10.3.1, implemented by the fake kernel
 * (platform/src/host_freertos_ext.inc). Macros and values as in
 * Targets/F407VG/Middlewares/Third_Party/FreeRTOS/Source/include/event_groups.h (line numbers cited).
 */
#ifndef HOSTTEST_EVENT_GROUPS_H_
#define HOSTTEST_EVENT_GROUPS_H_
#include "FreeRTOS.h"

#ifndef HOSTTEST_RTOS_EXT
typedef void *EventGroupHandle_t;
typedef uint32_t EventBits_t;
#else
#include "timers.h"

#ifdef __cplusplus
extern "C" {
#endif

struct HostEventGroup; // defined in host_freertos_ext.inc
typedef struct HostEventGroup *EventGroupHandle_t;
typedef TickType_t EventBits_t; // event_groups.h:92 (configUSE_16_BIT_TICKS 0: 24 event bits, 8 control bits)

EventGroupHandle_t xEventGroupCreate(void);
EventGroupHandle_t xEventGroupCreateStatic(StaticEventGroup_t *pxEventGroupBuffer);
EventBits_t xEventGroupWaitBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xClearOnExit, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait);
EventBits_t xEventGroupClearBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear);
EventBits_t xEventGroupSetBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet);
EventBits_t xEventGroupSync(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet, const EventBits_t uxBitsToWaitFor, TickType_t xTicksToWait);
EventBits_t xEventGroupGetBitsFromISR(EventGroupHandle_t xEventGroup);
void vEventGroupDelete(EventGroupHandle_t xEventGroup);
/* For internal use: executed by the timer service task for the FromISR macros below (event_groups.c:657-669) */
void vEventGroupSetBitsCallback(void *pvEventGroup, const uint32_t ulBitsToSet);
void vEventGroupClearBitsCallback(void *pvEventGroup, const uint32_t ulBitsToClear);

/* configUSE_TRACE_FACILITY is 1 on the F407 target, so the FromISR functions are the pend macros (event_groups.h:410, 562) */
#define xEventGroupClearBitsFromISR(xEventGroup, uxBitsToClear) xTimerPendFunctionCallFromISR(vEventGroupClearBitsCallback, (void *)xEventGroup, (uint32_t)uxBitsToClear, NULL)
#define xEventGroupSetBitsFromISR(xEventGroup, uxBitsToSet, pxHigherPriorityTaskWoken) xTimerPendFunctionCallFromISR(vEventGroupSetBitsCallback, (void *)xEventGroup, (uint32_t)uxBitsToSet, pxHigherPriorityTaskWoken)
#define xEventGroupGetBits(xEventGroup) xEventGroupClearBits(xEventGroup, 0) // event_groups.h:708

#ifdef __cplusplus
}
#endif

#endif // HOSTTEST_RTOS_EXT
#endif
