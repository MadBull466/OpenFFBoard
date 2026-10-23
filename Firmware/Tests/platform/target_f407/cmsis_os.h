/*
 * Host test replacement for cmsis_os.h (CMSIS-RTOS2 wrapper) of the F407 target.
 * Extends the generic fake (platform/include/cmsis_os.h) with the CMSIS-RTOS2 types used by
 * the sources of ffb_target_tests.
 */
#ifndef HOSTTEST_TARGET_CMSIS_OS_H_
#define HOSTTEST_TARGET_CMSIS_OS_H_
#include "../include/cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef void *osThreadId_t; // cmsis_os2.h: thread ID identifies the thread
#ifdef __cplusplus
}
#endif
#endif
