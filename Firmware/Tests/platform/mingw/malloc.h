/*
 * MinGW only (ffb_target_tests, added to the include path under if(MINGW) in CMakeLists.txt; never used on
 * Linux/macOS). SystemCommands.cpp (mallinfo command) uses struct mallinfo and mallinfo() from <malloc.h> (newlib on
 * the target, glibc on Linux). mingw-w64's <malloc.h> does not declare them, so they are added here; the definition
 * (all fields 0) is in platform/mingw/host_mallinfo.cpp. Unverified: no MinGW toolchain was available.
 */
#ifndef HOSTTEST_MINGW_MALLOC_H_
#define HOSTTEST_MINGW_MALLOC_H_
#include_next <malloc.h>

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
