/*
 * Host test replacement for FreeRTOS timers.h.
 * ffb_unit_tests, ffb_target_tests and ffb_flash_tests do not use software timers (only the handle type).
 * ffb_rtos_tests (HOSTTEST_RTOS_EXT) gets the software timer API of FreeRTOS V10.3.1 with the timer service task,
 * implemented by the fake kernel (platform/src/host_freertos_ext.inc). Macros and values as in
 * Targets/F407VG/Middlewares/Third_Party/FreeRTOS/Source/include/timers.h (line numbers cited).
 */
#ifndef HOSTTEST_TIMERS_H_
#define HOSTTEST_TIMERS_H_
#include "FreeRTOS.h"

#ifndef HOSTTEST_RTOS_EXT
typedef void *TimerHandle_t;
#else
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Commands sent to the timer service task (timers.h:54-67) */
#define tmrCOMMAND_EXECUTE_CALLBACK_FROM_ISR ((BaseType_t)-2)
#define tmrCOMMAND_EXECUTE_CALLBACK ((BaseType_t)-1)
#define tmrCOMMAND_START_DONT_TRACE ((BaseType_t)0)
#define tmrCOMMAND_START ((BaseType_t)1)
#define tmrCOMMAND_RESET ((BaseType_t)2)
#define tmrCOMMAND_STOP ((BaseType_t)3)
#define tmrCOMMAND_CHANGE_PERIOD ((BaseType_t)4)
#define tmrCOMMAND_DELETE ((BaseType_t)5)
#define tmrFIRST_FROM_ISR_COMMAND ((BaseType_t)6)
#define tmrCOMMAND_START_FROM_ISR ((BaseType_t)6)
#define tmrCOMMAND_RESET_FROM_ISR ((BaseType_t)7)
#define tmrCOMMAND_STOP_FROM_ISR ((BaseType_t)8)
#define tmrCOMMAND_CHANGE_PERIOD_FROM_ISR ((BaseType_t)9)

struct HostTimer; // defined in host_freertos_ext.inc
typedef struct HostTimer *TimerHandle_t;
typedef void (*TimerCallbackFunction_t)(TimerHandle_t xTimer); // timers.h:82
typedef void (*PendedFunction_t)(void *, uint32_t);			   // timers.h:88

TimerHandle_t xTimerCreate(const char *const pcTimerName, const TickType_t xTimerPeriodInTicks, const UBaseType_t uxAutoReload, void *const pvTimerID, TimerCallbackFunction_t pxCallbackFunction);
void *pvTimerGetTimerID(const TimerHandle_t xTimer);
void vTimerSetTimerID(TimerHandle_t xTimer, void *pvNewID);
BaseType_t xTimerIsTimerActive(TimerHandle_t xTimer);
TaskHandle_t xTimerGetTimerDaemonTaskHandle(void);
BaseType_t xTimerPendFunctionCallFromISR(PendedFunction_t xFunctionToPend, void *pvParameter1, uint32_t ulParameter2, BaseType_t *pxHigherPriorityTaskWoken);
BaseType_t xTimerPendFunctionCall(PendedFunction_t xFunctionToPend, void *pvParameter1, uint32_t ulParameter2, TickType_t xTicksToWait);
const char *pcTimerGetName(TimerHandle_t xTimer);
void vTimerSetReloadMode(TimerHandle_t xTimer, const UBaseType_t uxAutoReload);
UBaseType_t uxTimerGetReloadMode(TimerHandle_t xTimer);
TickType_t xTimerGetPeriod(TimerHandle_t xTimer);
TickType_t xTimerGetExpiryTime(TimerHandle_t xTimer);
BaseType_t xTimerGenericCommand(TimerHandle_t xTimer, const BaseType_t xCommandID, const TickType_t xOptionalValue, BaseType_t *const pxHigherPriorityTaskWoken, const TickType_t xTicksToWait);

#define xTimerStart(xTimer, xTicksToWait) xTimerGenericCommand((xTimer), tmrCOMMAND_START, (xTaskGetTickCount()), NULL, (xTicksToWait))						  // timers.h:504
#define xTimerStop(xTimer, xTicksToWait) xTimerGenericCommand((xTimer), tmrCOMMAND_STOP, 0U, NULL, (xTicksToWait))										  // timers.h:546
#define xTimerChangePeriod(xTimer, xNewPeriod, xTicksToWait) xTimerGenericCommand((xTimer), tmrCOMMAND_CHANGE_PERIOD, (xNewPeriod), NULL, (xTicksToWait))	  // timers.h:626
#define xTimerDelete(xTimer, xTicksToWait) xTimerGenericCommand((xTimer), tmrCOMMAND_DELETE, 0U, NULL, (xTicksToWait))									  // timers.h:664
#define xTimerReset(xTimer, xTicksToWait) xTimerGenericCommand((xTimer), tmrCOMMAND_RESET, (xTaskGetTickCount()), NULL, (xTicksToWait))						  // timers.h:788
#define xTimerStartFromISR(xTimer, pxHigherPriorityTaskWoken) xTimerGenericCommand((xTimer), tmrCOMMAND_START_FROM_ISR, (xTaskGetTickCountFromISR()), (pxHigherPriorityTaskWoken), 0U) // timers.h:874
#define xTimerStopFromISR(xTimer, pxHigherPriorityTaskWoken) xTimerGenericCommand((xTimer), tmrCOMMAND_STOP_FROM_ISR, 0, (pxHigherPriorityTaskWoken), 0U)						  // timers.h:937
#define xTimerChangePeriodFromISR(xTimer, xNewPeriod, pxHigherPriorityTaskWoken) xTimerGenericCommand((xTimer), tmrCOMMAND_CHANGE_PERIOD_FROM_ISR, (xNewPeriod), (pxHigherPriorityTaskWoken), 0U) // timers.h:1010
#define xTimerResetFromISR(xTimer, pxHigherPriorityTaskWoken) xTimerGenericCommand((xTimer), tmrCOMMAND_RESET_FROM_ISR, (xTaskGetTickCountFromISR()), (pxHigherPriorityTaskWoken), 0U) // timers.h:1096

#ifdef __cplusplus
}
#endif

#endif // HOSTTEST_RTOS_EXT
#endif
