/*
 * MinGW and macOS only (ffb_target_tests, added to the include path under if(MINGW OR APPLE) in CMakeLists.txt; never
 * used on Linux). SystemCommands.cpp (mallinfo command) uses struct mallinfo and mallinfo() from <malloc.h> (newlib on
 * the target, glibc on Linux). mingw-w64's <malloc.h> does not declare them and the macOS SDK has no <malloc.h> at all,
 * so they are added here; the definition (all fields 0) is in platform/mingw/host_mallinfo.cpp.
 * Unverified: no MinGW or macOS toolchain was available.
 */
#ifndef HOSTTEST_MINGW_MALLOC_H_
#define HOSTTEST_MINGW_MALLOC_H_

#if defined(__has_include_next)
#if __has_include_next(<malloc.h>)
#include_next <malloc.h>
#else
#include <stdlib.h> /* macOS: malloc / free (glibc and mingw-w64 declare them in <malloc.h>) */
#endif
#else
#include_next <malloc.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif
/** Same fields as newlib / glibc struct mallinfo */
struct mallinfo {
	int arena;
	int ordblks;
	int smblks;
	int hblks;
	int hblkhd;
	int usmblks;
	int fsmblks;
	int uordblks;
	int fordblks;
	int keepcost;
};
struct mallinfo mallinfo(void);
#ifdef __cplusplus
}
#endif
#endif
