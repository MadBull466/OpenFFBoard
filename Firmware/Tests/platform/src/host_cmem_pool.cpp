/*
 * host_cmem_pool.cpp
 *
 * Compiles the unmodified FreeRTOS C++ wrapper FFBoard/Src/cmem_pool.cpp (MemoryPool, tier A vendor code) for
 * ffb_rtos_tests, with its malloc() and free() calls going to the RTOS heap like on the target: there
 * cppmain.cpp:136-144 defines malloc() as pvPortMalloc() and free() as vPortFree() for the whole firmware (newlib's
 * allocator is replaced). On the host the C library functions would be called; the fake pvPortMalloc / vPortFree
 * (host_freertos_ext.inc) add the heap_4 behaviour (pvPortMalloc(0) returns NULL), failure injection
 * (HostRtos::failCreate(RtosObject::Heap)) and the live block count (HostRtos::heapLiveBlocks()), so the
 * MemoryPoolMallocException paths can be tested and a free() of pool memory (none today; e.g. a fix of a leak) is seen
 * by the heap model instead of going to the C library with a block it did not allocate.
 *
 * Same technique as host_cppmain.cpp: every header cmem_pool.cpp includes is included here first (include guards make
 * its own #include lines no-ops), then malloc and free are redefined, so the macros only reach the code of cmem_pool.cpp.
 * CMakeLists.txt checks at configure time that the pre-include list contains every #include of cmem_pool.cpp.
 * gcov reports the lines under cmem_pool.cpp.
 */
#include "FreeRTOS.h"
#include <stdlib.h>
#include "mem_pool.hpp"
// End of the pre-include list (checked by CMakeLists.txt)

#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)
#include "../../../FFBoard/Src/cmem_pool.cpp"
#undef free
#undef malloc
