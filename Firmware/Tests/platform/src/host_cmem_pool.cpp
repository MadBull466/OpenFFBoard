/*
 * host_cmem_pool.cpp
 *
 * Compiles the unmodified FreeRTOS C++ wrapper FFBoard/Src/cmem_pool.cpp (MemoryPool, tier A vendor code) for
 * ffb_rtos_tests, with its malloc() calls going to the RTOS heap like on the target: there cppmain.cpp:138-141 defines
 * malloc() as pvPortMalloc() for the whole firmware (newlib's malloc is replaced). On the host the C library malloc
 * would be called; the fake pvPortMalloc (host_freertos_ext.inc) adds the heap_4 behaviour (pvPortMalloc(0) returns
 * NULL) and failure injection (HostRtos::failCreate(RtosObject::Heap)), so the MemoryPoolMallocException paths can be
 * tested.
 *
 * Same technique as host_cppmain.cpp: every header cmem_pool.cpp includes is included here first (include guards make
 * its own #include lines no-ops), then malloc is redefined, so the macro only reaches the code of cmem_pool.cpp.
 * CMakeLists.txt checks at configure time that the pre-include list contains every #include of cmem_pool.cpp.
 * gcov reports the lines under cmem_pool.cpp.
 */
#include "FreeRTOS.h"
#include <stdlib.h>
#include "mem_pool.hpp"
// End of the pre-include list (checked by CMakeLists.txt)

#define malloc(size) pvPortMalloc(size)
#include "../../../FFBoard/Src/cmem_pool.cpp"
#undef malloc
