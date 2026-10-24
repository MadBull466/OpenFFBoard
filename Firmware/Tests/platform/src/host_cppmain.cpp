/*
 * host_cppmain.cpp
 *
 * Compiles the unmodified firmware source FFBoard/Src/cppmain.cpp for ffb_target_tests, with one change made from
 * outside: its definitions of malloc() and free() (forwarding to pvPortMalloc / vPortFree, which replace newlib's
 * allocator on the target) are renamed to hosttest_cppmain_malloc / hosttest_cppmain_free. On the host they would
 * replace the C library allocator of the whole executable (and the sanitizer's), and the fake pvPortMalloc calls
 * malloc: endless recursion.
 *
 * A -D option on cppmain.cpp does not work: <cstdlib> #undefs malloc and free. So every header cppmain.cpp includes
 * is included here first (include guards make cppmain.cpp's own #include lines no-ops), then the two names are
 * redefined, so they only apply to the code of cppmain.cpp itself. gcov reports the lines under cppmain.cpp.
 * Tests reach the renamed functions through support/HostTarget.h. On macOS the section name of `mainclass` is adapted
 * as well (see below).
 */
#include "main.h"
#include "cppmain.h"
#include "mainclass_chooser.h"
#include "flash_helpers.h"
#include "global_callbacks.h"
#include "cpp_target_config.h"
#include "cmsis_os.h"
#include "tusb.h"

#include <cstdlib>

// `mainclass` is placed in CCRAM_SEC (".ccmram", target_constants.h). Mach-O section names need a segment
// ("segment,section"); ELF and PE accept the name as it is. Not verified on macOS (no macOS host available).
#ifdef __APPLE__
#undef CCRAM_SEC
#define CCRAM_SEC "__DATA,__ccmram"
#endif

#define malloc hosttest_cppmain_malloc
#define free hosttest_cppmain_free
#include "../../../FFBoard/Src/cppmain.cpp"
#undef malloc
#undef free
