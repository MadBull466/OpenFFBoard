/*
 * MinGW and macOS only: stub of mallinfo() (declared in platform/mingw/malloc.h). The host heap has no newlib statistics;
 * all fields are 0. Unverified: no MinGW or macOS toolchain was available.
 */
#include <malloc.h>

extern "C" struct mallinfo mallinfo(void) { return {}; }
