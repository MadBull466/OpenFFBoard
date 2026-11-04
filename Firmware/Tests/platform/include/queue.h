/*
 * Host test replacement for FreeRTOS queue.h.
 * ffb_unit_tests, ffb_target_tests and ffb_flash_tests do not use queues (only the handle type).
 * ffb_rtos_tests (HOSTTEST_RTOS_EXT) gets the queue API of FreeRTOS V10.3.1, implemented by the fake kernel
 * (platform/src/host_freertos_ext.inc). Macros and values as in
 * Targets/F407VG/Middlewares/Third_Party/FreeRTOS/Source/include/queue.h (line numbers cited).
 */
#ifndef HOSTTEST_QUEUE_H_
#define HOSTTEST_QUEUE_H_
#include "FreeRTOS.h"

#ifndef HOSTTEST_RTOS_EXT
typedef void *QueueHandle_t;
#else

#ifdef __cplusplus
extern "C" {
#endif

struct HostQueue; // defined in host_freertos_ext.inc
typedef struct HostQueue *QueueHandle_t;

#define queueSEND_TO_BACK ((BaseType_t)0)  // queue.h:65
#define queueSEND_TO_FRONT ((BaseType_t)1) // queue.h:66
#define queueOVERWRITE ((BaseType_t)2)	   // queue.h:67

#define queueQUEUE_TYPE_BASE ((uint8_t)0U)				  // queue.h:70
#define queueQUEUE_TYPE_MUTEX ((uint8_t)1U)				  // queue.h:72
#define queueQUEUE_TYPE_COUNTING_SEMAPHORE ((uint8_t)2U) // queue.h:73
#define queueQUEUE_TYPE_BINARY_SEMAPHORE ((uint8_t)3U)	  // queue.h:74
#define queueQUEUE_TYPE_RECURSIVE_MUTEX ((uint8_t)4U)	  // queue.h:75

QueueHandle_t xQueueGenericCreate(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, const uint8_t ucQueueType);
#define xQueueCreate(uxQueueLength, uxItemSize) xQueueGenericCreate((uxQueueLength), (uxItemSize), (queueQUEUE_TYPE_BASE)) // queue.h:146

BaseType_t xQueueGenericSend(QueueHandle_t xQueue, const void *const pvItemToQueue, TickType_t xTicksToWait, const BaseType_t xCopyPosition);
#define xQueueSendToFront(xQueue, pvItemToQueue, xTicksToWait) xQueueGenericSend((xQueue), (pvItemToQueue), (xTicksToWait), queueSEND_TO_FRONT) // queue.h:313
#define xQueueSendToBack(xQueue, pvItemToQueue, xTicksToWait) xQueueGenericSend((xQueue), (pvItemToQueue), (xTicksToWait), queueSEND_TO_BACK)	  // queue.h:395
#define xQueueSend(xQueue, pvItemToQueue, xTicksToWait) xQueueGenericSend((xQueue), (pvItemToQueue), (xTicksToWait), queueSEND_TO_BACK)		  // queue.h:479
#define xQueueOverwrite(xQueue, pvItemToQueue) xQueueGenericSend((xQueue), (pvItemToQueue), 0, queueOVERWRITE)								  // queue.h:562

BaseType_t xQueueGenericSendFromISR(QueueHandle_t xQueue, const void *const pvItemToQueue, BaseType_t *const pxHigherPriorityTaskWoken, const BaseType_t xCopyPosition);
#define xQueueSendToFrontFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken) xQueueGenericSendFromISR((xQueue), (pvItemToQueue), (pxHigherPriorityTaskWoken), queueSEND_TO_FRONT) // queue.h:984
#define xQueueSendToBackFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken) xQueueGenericSendFromISR((xQueue), (pvItemToQueue), (pxHigherPriorityTaskWoken), queueSEND_TO_BACK)	  // queue.h:1055
#define xQueueOverwriteFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken) xQueueGenericSendFromISR((xQueue), (pvItemToQueue), (pxHigherPriorityTaskWoken), queueOVERWRITE)	  // queue.h:1142
#define xQueueSendFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken) xQueueGenericSendFromISR((xQueue), (pvItemToQueue), (pxHigherPriorityTaskWoken), queueSEND_TO_BACK)		  // queue.h:1216

BaseType_t xQueueReceive(QueueHandle_t xQueue, void *const pvBuffer, TickType_t xTicksToWait);
BaseType_t xQueuePeek(QueueHandle_t xQueue, void *const pvBuffer, TickType_t xTicksToWait);
BaseType_t xQueueReceiveFromISR(QueueHandle_t xQueue, void *const pvBuffer, BaseType_t *const pxHigherPriorityTaskWoken);
BaseType_t xQueuePeekFromISR(QueueHandle_t xQueue, void *const pvBuffer);

UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t xQueue);
UBaseType_t uxQueueSpacesAvailable(const QueueHandle_t xQueue);
UBaseType_t uxQueueMessagesWaitingFromISR(const QueueHandle_t xQueue);
BaseType_t xQueueIsQueueEmptyFromISR(const QueueHandle_t xQueue);
BaseType_t xQueueIsQueueFullFromISR(const QueueHandle_t xQueue);

BaseType_t xQueueGenericReset(QueueHandle_t xQueue, BaseType_t xNewQueue);
#define xQueueReset(xQueue) xQueueGenericReset(xQueue, pdFALSE) // queue.h:1433

void vQueueDelete(QueueHandle_t xQueue);

#ifdef __cplusplus
}
#endif

#endif // HOSTTEST_RTOS_EXT
#endif
