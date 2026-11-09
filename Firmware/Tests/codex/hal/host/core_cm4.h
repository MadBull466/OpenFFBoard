#ifndef CODEX_P12_HOST_CORE_CM4_H
#define CODEX_P12_HOST_CORE_CM4_H
/* Narrow CMSIS boundary: storage qualifiers only, no CPU emulation.
 * Models: volatile register storage used by the real ST device header.
 * Source: checked-in CMSIS core_cm4.h access qualifier definitions.
 * Does not model: CPU, NVIC, barriers, exclusive accesses, interrupt timing.
 * Confidence: high for plain storage qualifiers; no hardware-effect claim.
 * No intrinsics are stubbed to succeed. A new intrinsic dependency must be
 * explicitly assessed before adding a declaration/implementation here.
 */
#include <stdint.h>
#define __I volatile const
#define __O volatile
#define __IO volatile
#endif
