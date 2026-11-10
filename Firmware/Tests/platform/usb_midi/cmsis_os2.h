/*
 * cmsis_os2.h (ffb_usb_tests only, P17)
 *
 * Host test replacement for the CMSIS-RTOS2 API header that MidiMain.cpp includes (`#include "cmsis_os2.h"`, MidiMain.cpp:15)
 * for one call: osDelay(500) in MidiMain::update().
 *
 * Models:  osDelay(ticks) as the CMSIS-RTOS2 wrapper of the target (cmsis_os2.c: `osDelay` returns osOK after
 *          vTaskDelay(ticks); with ticks == 0 it only yields (osKernelGetState check omitted)). The tick rate of the F407
 *          FreeRTOSConfig.h is 1000 Hz, so one tick is one millisecond of simulated time; vTaskDelay is the fake
 *          kernel call of platform/src/host_freertos.cpp (simulated time, cooperative scheduling).
 * Does not model: the error returns (osErrorISR when called from an interrupt, osErrorParameter, osKernelGetState check),
 *          the other 100 functions of cmsis_os2.h (none is used by the sources of this executable).
 * Source:  CMSIS-RTOS2 cmsis_os2.h / cmsis_os2.c (osDelay), FreeRTOS tasks.c (vTaskDelay).
 * Confidence: High for the used call, nothing else exists.
 */
#ifndef HOSTTEST_USB_MIDI_CMSIS_OS2_H_
#define HOSTTEST_USB_MIDI_CMSIS_OS2_H_
#include "cmsis_os.h"
#include "task.h"

#include <stdint.h>

typedef int32_t osStatus_t;
#define osOK 0

#ifdef __cplusplus
extern "C" {
#endif
static inline osStatus_t osDelay(uint32_t ticks) {
	vTaskDelay(ticks);
	return osOK;
}
#ifdef __cplusplus
}
#endif
#endif
