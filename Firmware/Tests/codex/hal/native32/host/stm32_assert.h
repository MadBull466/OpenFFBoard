#pragma once
#include <stdint.h>
/* The optional LL full-assert source expects this application-owned header.
 * Match the supplied HAL configuration's assert_failed boundary exactly. */
#ifdef __cplusplus
extern "C" {
#endif
void assert_failed(uint8_t *file,uint32_t line);
#ifdef __cplusplus
}
#endif
#ifndef assert_param
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#endif
