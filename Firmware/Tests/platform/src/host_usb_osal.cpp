/*
 * host_usb_osal.cpp (ffb_usb_tests only)
 *
 * The FreeRTOS functions TinyUSB's osal_freertos.h needs on top of the fake FreeRTOS API F11. Declarations and the
 * model: platform/usb/hosttest_osal/host_osal_freertos.h.
 */
#include "hosttest_osal/host_osal_freertos.h"

#include "HostDcd.h"

namespace {
/** configASSERT of FreeRTOS on a NULL storage pointer (queue.c:301, :408): recorded with the DCD faults */
bool storageGiven(const void *storage, const char *api) {
	if (storage == nullptr) {
		HostDcd::recordFault(std::string(api) + " with NULL storage (configASSERT on the target)");
		return false;
	}
	return true;
}
} // namespace

extern "C" {

// semphr.h:180 xSemaphoreCreateBinaryStatic -> xQueueGenericCreateStatic(1, 0, NULL, buffer, BINARY_SEMAPHORE)
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *pxSemaphoreBuffer) {
	return storageGiven(pxSemaphoreBuffer, "xSemaphoreCreateBinaryStatic") ? xSemaphoreCreateBinary() : nullptr;
}

// semphr.h:735 xSemaphoreCreateMutexStatic -> xQueueCreateMutexStatic(MUTEX, buffer)
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *pxMutexBuffer) {
	return storageGiven(pxMutexBuffer, "xSemaphoreCreateMutexStatic") ? xSemaphoreCreateMutex() : nullptr;
}

// queue.h:216 xQueueCreateStatic -> xQueueGenericCreateStatic(length, size, storage, buffer, BASE)
QueueHandle_t xQueueCreateStatic(UBaseType_t uxQueueLength, UBaseType_t uxItemSize, uint8_t *pucQueueStorage, StaticQueue_t *pxQueueBuffer) {
	if (!storageGiven(pxQueueBuffer, "xQueueCreateStatic") || (uxItemSize > 0 && !storageGiven(pucQueueStorage, "xQueueCreateStatic"))) {
		return nullptr;
	}
	return xQueueCreate(uxQueueLength, uxItemSize);
}

// xQueueGenericReset (queue.c:255-268) on a binary semaphore: uxMessagesWaiting = 0, i.e. the semaphore is taken
BaseType_t hosttest_xSemaphoreQueueReset(SemaphoreHandle_t xSemaphore) {
	while (xSemaphoreTake(xSemaphore, 0) == pdTRUE) {
	}
	return pdPASS;
}

} // extern "C"
