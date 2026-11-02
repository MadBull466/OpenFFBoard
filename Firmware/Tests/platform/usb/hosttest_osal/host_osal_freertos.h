/*
 * host_osal_freertos.h (ffb_usb_tests only)
 *
 * What TinyUSB's osal_freertos.h needs on top of the fake FreeRTOS API F11 (platform/src/host_freertos_ext.inc,
 * HOSTTEST_RTOS_EXT). Implemented in platform/src/host_usb_osal.cpp. TinyUSB itself is not changed.
 *
 *  - Static creation (configSUPPORT_STATIC_ALLOCATION 1 like the F407, FreeRTOSConfig.h:67):
 *    xSemaphoreCreateBinaryStatic, xSemaphoreCreateMutexStatic, xQueueCreateStatic and the storage types
 *    StaticSemaphore_t / StaticQueue_t (FreeRTOS V10.3.1 semphr.h:180, :735, queue.h:216; FreeRTOS.h:1191, :1202).
 *    The fake has no static objects: the functions create the object dynamically (the storage passed in is not used)
 *    and record a NULL storage pointer as unmodelled (configASSERT in queue.c:301 / :408 on the target).
 *    Host-vs-target: HostRtos::failCreate(Mutex / Queue) also makes these fail; a static create cannot fail for lack of
 *    memory on the target.
 *  - xQueueReset on a semaphore: osal_semaphore_reset (osal_freertos.h:148) passes a SemaphoreHandle_t to
 *    xQueueReset. In FreeRTOS a semaphore is a queue (semphr.h:39 typedef QueueHandle_t SemaphoreHandle_t); in the fake
 *    the two handle types differ. xQueueReset is redefined here: a semaphore handle goes to
 *    hosttest_xSemaphoreQueueReset (FreeRTOS xQueueGenericReset on a binary semaphore sets uxMessagesWaiting to 0,
 *    queue.c:255-268: the semaphore is taken; modelled with non-blocking takes until it is empty), a queue handle to
 *    xQueueGenericReset as before. C: _Generic; C++: overloads. TinyUSB's device code never calls osal_semaphore_reset
 *    (it is a static inline function that only has to compile).
 */
#ifndef HOSTTEST_HOST_OSAL_FREERTOS_H_
#define HOSTTEST_HOST_OSAL_FREERTOS_H_

#include "../../include/FreeRTOS.h"
#include "../../include/queue.h"
#include "../../include/semphr.h"
#include "../../include/task.h"

#ifndef HOSTTEST_RTOS_EXT
#error "ffb_usb_tests needs the fake FreeRTOS API F11 (HOSTTEST_RTOS_EXT)"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** FreeRTOS.h:1191 StaticQueue_t / :1202 StaticSemaphore_t: storage of a statically created queue / semaphore. Opaque */
typedef struct xSTATIC_QUEUE {
	void *pvDummy1[3];
	UBaseType_t uxDummy4[3];
	uint8_t ucDummy5[2];
	uint8_t ucDummy6;
	UBaseType_t uxDummy8;
	uint8_t ucDummy9;
} StaticQueue_t;
typedef StaticQueue_t StaticSemaphore_t;

SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *pxSemaphoreBuffer);
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *pxMutexBuffer);
QueueHandle_t xQueueCreateStatic(UBaseType_t uxQueueLength, UBaseType_t uxItemSize, uint8_t *pucQueueStorage, StaticQueue_t *pxQueueBuffer);

/** xQueueReset(semaphore): takes the semaphore until it is empty (see above). Returns pdPASS like xQueueGenericReset */
BaseType_t hosttest_xSemaphoreQueueReset(SemaphoreHandle_t xSemaphore);

#ifdef __cplusplus
}
#endif

#undef xQueueReset
#ifdef __cplusplus
// extern "C++": osal_freertos.h is reached from inside extern "C" blocks of TinyUSB headers (overloads need C++ linkage)
extern "C++" {
inline BaseType_t hosttest_xQueueReset(QueueHandle_t xQueue) { return xQueueGenericReset(xQueue, pdFALSE); }
inline BaseType_t hosttest_xQueueReset(SemaphoreHandle_t xSemaphore) { return hosttest_xSemaphoreQueueReset(xSemaphore); }
}
#define xQueueReset(xQueue) hosttest_xQueueReset(xQueue)
#else
static inline BaseType_t hosttest_xQueueResetQueue(QueueHandle_t xQueue) { return xQueueGenericReset(xQueue, pdFALSE); }
#define xQueueReset(xQueue) _Generic((xQueue), SemaphoreHandle_t: hosttest_xSemaphoreQueueReset, default: hosttest_xQueueResetQueue)(xQueue)
#endif

#endif
