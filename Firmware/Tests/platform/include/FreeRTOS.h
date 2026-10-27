/*
 * Host test replacement for the FreeRTOS kernel API.
 *
 * There is no scheduler on the host. Tasks are only registered and are executed
 * explicitly and deterministically by the tests (see support/HostRtos.h).
 * The implementation is in platform/src/host_freertos.cpp
 */
#ifndef HOSTTEST_FREERTOS_H_
#define HOSTTEST_FREERTOS_H_

#include <stdint.h>
#include <stddef.h>
#include "FreeRTOSConfig.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef long BaseType_t;
typedef unsigned long UBaseType_t;
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;

#define pdFALSE ((BaseType_t)0)
#define pdTRUE ((BaseType_t)1)
#define pdPASS (pdTRUE)
#define pdFAIL (pdFALSE)
#define portMAX_DELAY ((TickType_t)0xffffffffUL)
#define portTICK_PERIOD_MS ((TickType_t)1)
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portYIELD_FROM_ISR(x) ((void)(x))
#define portEND_SWITCHING_ISR(x) ((void)(x))
#define configASSERT(x) ((void)(x))

void *pvPortMalloc(size_t size);
void vPortFree(void *ptr);
size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);

#ifdef HOSTTEST_RTOS_EXT
/* ffb_rtos_tests (fake FreeRTOS API F11): values of FreeRTOS V10.3.1 (Targets/F407VG/Middlewares/Third_Party/FreeRTOS) */
#define errQUEUE_EMPTY ((BaseType_t)0)						 // projdefs.h:49
#define errQUEUE_FULL ((BaseType_t)0)						 // projdefs.h:50
#define errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY (-1)			 // projdefs.h:53
/** FreeRTOS.h:1220: storage of a statically allocated event group. Opaque; the fake keeps its state outside it */
typedef struct xSTATIC_EVENT_GROUP {
	TickType_t xDummy1;
	void *pvDummy2[5];
	UBaseType_t uxDummy3;
	uint8_t ucDummy4;
} StaticEventGroup_t;
#endif

#ifdef __cplusplus
}
#endif

#endif
