/*
 * TinyUSB configuration of ffb_usb_tests (P9): the real tusb_config.h of the F407 target, plus the include path of
 * the FreeRTOS headers used by osal_freertos.h.
 *
 * platform/usb is the first include directory of ffb_usb_tests only, so this file shadows the generic test
 * configuration (platform/include/tusb_config.h, OPT_OS_NONE) of the other executables, which is not changed.
 *
 * CFG_TUSB_OS_INC_PATH (tusb_option.h:403, used by osal_freertos.h:31-34 through TU_INCLUDE_PATH) is the TinyUSB hook
 * for the location of the FreeRTOS headers. It points at platform/usb/hosttest_osal/, whose headers include the fake
 * FreeRTOS headers of platform/include and add what osal_freertos.h needs on top of the fake API F11 (see
 * hosttest_osal/host_osal_freertos.h). Nothing in TinyUSB is changed.
 *
 * FFB_F407_TUSB_CONFIG_H is defined by CMakeLists.txt: the absolute path of Targets/F407VG/Core/Inc/tusb_config.h
 * (OPT_MCU_STM32F4, OPT_OS_FREERTOS, full speed, EP0 64 bytes, CDC + HID + MIDI class drivers, CDC FIFOs 512 / 1024).
 */
#ifndef HOSTTEST_USB_TUSB_CONFIG_H_
#define HOSTTEST_USB_TUSB_CONFIG_H_

#define CFG_TUSB_OS_INC_PATH hosttest_osal/

#include FFB_F407_TUSB_CONFIG_H

#endif
