# OpenFFBoard firmware host unit tests
#
# Builds the hardware independent firmware sources with the native compiler of the
# development machine and links them against a fake platform layer (HAL, FreeRTOS, TinyUSB).
#
#   cmake -S Firmware/Tests -B Firmware/Tests/build
#   cmake --build Firmware/Tests/build
#   ctest --test-dir Firmware/Tests/build --output-on-failure
#
# See README.md in this directory.
cmake_minimum_required(VERSION 3.16)
project(OpenFFBoardHostTests LANGUAGES C CXX)

# The hardware targets are built with USE_DSP_FUNCTIONS. Default: ON if the CMSIS-DSP submodule is checked out
set(FFB_TESTS_USE_DSP "AUTO" CACHE STRING "Build the firmware with USE_DSP_FUNCTIONS and CMSIS-DSP like the hardware targets (ON, OFF, AUTO)")
set_property(CACHE FFB_TESTS_USE_DSP PROPERTY STRINGS AUTO ON OFF)
option(FFB_TESTS_SANITIZE "Build with address and undefined behaviour sanitizers (gcc/clang on linux/macOS)" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON) # gnu++20 like the firmware build
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Debug)
endif()

set(FW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(FFBOARD_DIR ${FW_DIR}/FFBoard)

# ---------------------------------------------------------------------------
# Firmware sources under test. These are the unmodified firmware files.
# Add a file here to make it available to the tests. If it needs something from
# the hardware extend the fake platform in platform/ instead of changing the source.
# ---------------------------------------------------------------------------
set(FIRMWARE_SOURCES
	# Command system
	${FFBOARD_DIR}/Src/CmdParser.cpp
	${FFBOARD_DIR}/Src/CommandHandler.cpp
	${FFBOARD_DIR}/Src/CommandInterface.cpp
	${FFBOARD_DIR}/Src/HidCommandInterface.cpp
	${FFBOARD_DIR}/Src/CanCommandInterface.cpp
	${FFBOARD_DIR}/Src/FFBoardMainCommandThread.cpp
	${FFBOARD_DIR}/Src/CDCcomm.cpp
	${FFBOARD_DIR}/Src/UART.cpp
	${FFBOARD_DIR}/Src/UartHandler.cpp
	${FFBOARD_DIR}/Src/CAN.cpp
	${FFBOARD_DIR}/Src/CanHandler.cpp
	# FFB
	${FFBOARD_DIR}/Src/EffectsCalculator.cpp
	${FFBOARD_DIR}/Src/HidFFB.cpp
	${FFBOARD_DIR}/Src/UsbHidHandler.cpp
	${FFBOARD_DIR}/Src/Filters.cpp
	# Device class interfaces
	${FFBOARD_DIR}/Src/Encoder.cpp
	${FFBOARD_DIR}/Src/MotorDriver.cpp
	# Infrastructure
	${FFBOARD_DIR}/Src/ErrorHandler.cpp
	${FFBOARD_DIR}/Src/PersistentStorage.cpp
	${FFBOARD_DIR}/Src/ringbuffer.c
	${FFBOARD_DIR}/Src/ringbufferwrapper.cpp
	${FFBOARD_DIR}/Src/cthread.cpp
	${FFBOARD_DIR}/Src/csemaphore.cpp
	${FFBOARD_DIR}/Src/cmutex.cpp
)

set(PLATFORM_SOURCES
	platform/src/host_freertos.cpp
	platform/src/host_platform.cpp
	platform/src/host_usb.cpp
	platform/src/host_stubs.cpp
)

# Every test_*.cpp in cases/ is picked up automatically
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cases/test_*.cpp)

add_executable(ffb_unit_tests
	${FIRMWARE_SOURCES}
	${PLATFORM_SOURCES}
	${TEST_SOURCES}
	support/test_main.cpp
)

# Order matters: test doubles and the fake platform shadow firmware and target headers.
target_include_directories(ffb_unit_tests PRIVATE
	fakes
	platform/include
	support
	mocks
	third_party
	${FFBOARD_DIR}/Inc
	${FFBOARD_DIR}/UserExtensions/Inc
	${FFBOARD_DIR}/USB
	${FFBOARD_DIR}/USB/common
	${FFBOARD_DIR}/USB/device
	${FFBOARD_DIR}/USB/class/hid
	${FFBOARD_DIR}/USB/class/cdc
	${FFBOARD_DIR}/USB/class/midi
)

target_compile_definitions(ffb_unit_tests PRIVATE HOSTTEST)

set(DSP_DIR ${FW_DIR}/Libraries/CMSIS-DSP)
set(USE_DSP ${FFB_TESTS_USE_DSP})
if(FFB_TESTS_USE_DSP STREQUAL "AUTO")
	if(EXISTS ${DSP_DIR}/Include/arm_math_types.h)
		set(USE_DSP ON)
	else()
		set(USE_DSP OFF)
	endif()
endif()
message(STATUS "OpenFFBoard tests: USE_DSP_FUNCTIONS=${USE_DSP}")

if(USE_DSP)
	# CMSIS-DSP has a portable C implementation which is used on the host.
	# Only the functions used by the tested firmware sources are compiled.
	if(NOT EXISTS ${DSP_DIR}/Include/arm_math_types.h)
		message(FATAL_ERROR "CMSIS-DSP not found. Run: git submodule update --init Firmware/Libraries/CMSIS-DSP")
	endif()
	target_sources(ffb_unit_tests PRIVATE
		${DSP_DIR}/Source/CommonTables/arm_common_tables.c
		${DSP_DIR}/Source/FastMathFunctions/arm_sin_f32.c
		${DSP_DIR}/Source/FastMathFunctions/arm_cos_f32.c
		${DSP_DIR}/Source/FilteringFunctions/arm_biquad_cascade_df1_f32.c
		${DSP_DIR}/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c
		${DSP_DIR}/Source/InterpolationFunctions/arm_spline_interp_f32.c
		${DSP_DIR}/Source/InterpolationFunctions/arm_spline_interp_init_f32.c
	)
	target_include_directories(ffb_unit_tests PRIVATE ${DSP_DIR}/Include ${DSP_DIR}/PrivateInclude)
	# __GNUC_PYTHON__ is the CMSIS-DSP switch for host builds without CMSIS core headers
	target_compile_definitions(ffb_unit_tests PRIVATE USE_DSP_FUNCTIONS ARM_MATH_LOOPUNROLL __GNUC_PYTHON__)
endif()

find_package(Threads REQUIRED) # Only used by tests which guard against endless loops
target_link_libraries(ffb_unit_tests PRIVATE Threads::Threads)

# Firmware sources are built with the warning settings of the firmware makefile.
target_compile_options(ffb_unit_tests PRIVATE -Wall $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>)
if(MINGW)
	# Packed HID report structs must use the gcc layout like on the target
	target_compile_options(ffb_unit_tests PRIVATE -mno-ms-bitfields)
	target_link_options(ffb_unit_tests PRIVATE -static)
endif()

if(FFB_TESTS_SANITIZE)
	target_compile_definitions(ffb_unit_tests PRIVATE HOSTTEST_SANITIZERS)
	target_compile_options(ffb_unit_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
	target_link_options(ffb_unit_tests PRIVATE -fsanitize=address,undefined)
endif()

# Test cases excluded from ctest runs (doctest --test-case-exclude). Each entry is a suspected defect
# that aborts the run instead of failing a single test. Keep this list in sync with Tests/PROGRESS.md.
#  - effectsForces reads current[MAX_AXIS] out of bounds. libstdc++ assertions (GCC 16, -O0) abort the run.
set(FFB_TESTS_EXCLUDE
	"requesting the forces of a not existing axis is rejected"
	CACHE STRING "Test cases excluded from ctest (list)")
#  - CMSIS-DSP arm_spline_f32 reads one element past its input. Only reported by the address sanitizer,
#    so these are only excluded in sanitizer builds with CMSIS-DSP.
#    NOTE: "reconstruction of a step ..." currently passes without a sanitizer report (GCC 16 and clang 23,
#    checked 2026-11-02). It stays excluded on purpose (my decision); tools/run_all_tests.py highlights it under
#    ATTENTION in the report of sanitizer builds.
set(FFB_TESTS_EXCLUDE_SANITIZE
	"reconstruction of a constant signal is constant"
	"reconstruction of a step never leaves the range of the input values and reaches the target"
	CACHE STRING "Additional test cases excluded from ctest in sanitizer builds with CMSIS-DSP (list)")

set(FFB_TESTS_EXCLUDED ${FFB_TESTS_EXCLUDE})
if(FFB_TESTS_SANITIZE AND USE_DSP)
	list(APPEND FFB_TESTS_EXCLUDED ${FFB_TESTS_EXCLUDE_SANITIZE})
endif()
list(JOIN FFB_TESTS_EXCLUDED "," FFB_TESTS_EXCLUDED_ARG)
# Read by tools/check_baseline.py --build-dir
set(FFB_TESTS_EXCLUDED_ARG "${FFB_TESTS_EXCLUDED_ARG}" CACHE INTERNAL "Test cases excluded from ctest (doctest filter)")
message(STATUS "OpenFFBoard tests: excluded from ctest: ${FFB_TESTS_EXCLUDED_ARG}")

enable_testing()
if(FFB_TESTS_EXCLUDED_ARG)
	add_test(NAME ffb_unit_tests COMMAND ffb_unit_tests "--test-case-exclude=${FFB_TESTS_EXCLUDED_ARG}")
else()
	add_test(NAME ffb_unit_tests COMMAND ffb_unit_tests)
endif()

# Every baseline test case (baseline/test_cases.txt) must still exist with its expected result
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
	if(FFB_TESTS_SANITIZE)
		set(FFB_TESTS_BASELINE_SANITIZE on)
	else()
		set(FFB_TESTS_BASELINE_SANITIZE off)
	endif()
	add_test(NAME ffb_unit_tests_baseline
		COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/check_baseline.py
			--exe $<TARGET_FILE:ffb_unit_tests>
			"--exclude-list=${FFB_TESTS_EXCLUDED_ARG}"
			--sanitize=${FFB_TESTS_BASELINE_SANITIZE})
else()
	message(STATUS "OpenFFBoard tests: Python 3 not found, baseline check not added to ctest")
endif()

# ===========================================================================
# ffb_target_tests: F407 feature set without the motor drivers and MIDI, with the real Axis.h.
# Axis, AxesManager and the main classes (P2), later system, inputs, encoders, buses and CAN.
#
# ffb_unit_tests above is not changed by this section: its sources, include order, defines and
# options stay exactly the same. The platform fakes are compiled again for this executable because
# they include main.h / target_constants.h and therefore depend on the feature header.
# ===========================================================================
set(TARGET_FIRMWARE_SOURCES
	${FIRMWARE_SOURCES}
	# Axis and main classes (P2)
	${FFBOARD_DIR}/Src/Axis.cpp
	${FFBOARD_DIR}/Src/AxesManager.cpp
	${FFBOARD_DIR}/UserExtensions/Src/FFBoardMain.cpp
	${FFBOARD_DIR}/UserExtensions/Src/FFBHIDMain.cpp
	${FFBOARD_DIR}/UserExtensions/Src/FFBHIDMain_commmands.cpp
	${FFBOARD_DIR}/UserExtensions/Src/FFBWheel.cpp
	${FFBOARD_DIR}/UserExtensions/Src/FFBJoystick.cpp
	${FFBOARD_DIR}/UserExtensions/Src/FFBHIDExt.cpp
	${FFBOARD_DIR}/UserExtensions/Src/CustomMain.cpp
	${FFBOARD_DIR}/UserExtensions/Src/mainclass_chooser.cpp
	${FFBOARD_DIR}/UserExtensions/Src/CanBridge.cpp
	${FFBOARD_DIR}/UserExtensions/Src/CanInputMain.cpp
	# Needed by the main classes to link (class registries, inputs, USB device). Tested in later phases
	${FFBOARD_DIR}/Src/SystemCommands.cpp
	${FFBOARD_DIR}/Src/SelectableInputs.cpp
	${FFBOARD_DIR}/Src/ButtonSource.cpp
	${FFBOARD_DIR}/Src/AnalogSource.cpp
	${FFBOARD_DIR}/Src/AnalogAxisProcessing.cpp
	${FFBOARD_DIR}/Src/AdcHandler.cpp
	${FFBOARD_DIR}/Src/SerialFFB.cpp
	${FFBOARD_DIR}/Src/TimerHandler.cpp
	${FFBOARD_DIR}/Src/ExtiHandler.cpp
	${FFBOARD_DIR}/Src/USBdevice.cpp
	${FFBOARD_DIR}/Src/SPI.cpp
	${FFBOARD_DIR}/Src/SpiHandler.cpp
	${FFBOARD_DIR}/Src/I2C.cpp
	${FFBOARD_DIR}/Src/I2CHandler.cpp
	${FFBOARD_DIR}/Src/CRC.cpp
	${FFBOARD_DIR}/UserExtensions/Src/ButtonSources.cpp
	${FFBOARD_DIR}/UserExtensions/Src/AnalogSources.cpp
	${FFBOARD_DIR}/UserExtensions/Src/EncoderSources.cpp
	${FFBOARD_DIR}/UserExtensions/Src/LocalButtons.cpp
	${FFBOARD_DIR}/UserExtensions/Src/SPIButtons.cpp
	${FFBOARD_DIR}/UserExtensions/Src/ShifterAnalog.cpp
	${FFBOARD_DIR}/UserExtensions/Src/PCF8574.cpp
	${FFBOARD_DIR}/UserExtensions/Src/CanButtons.cpp
	${FFBOARD_DIR}/UserExtensions/Src/LocalAnalog.cpp
	${FFBOARD_DIR}/UserExtensions/Src/CanAnalog.cpp
	${FFBOARD_DIR}/UserExtensions/Src/ADS111X.cpp
	${FFBOARD_DIR}/UserExtensions/Src/EncoderLocal.cpp
	${FFBOARD_DIR}/UserExtensions/Src/MtEncoderSPI.cpp
	${FFBOARD_DIR}/UserExtensions/Src/EncoderBissC.cpp
	${FFBOARD_DIR}/UserExtensions/Src/EncoderSSI.cpp
	${FFBOARD_DIR}/UserExtensions/Src/usb_descriptors.cpp
	${FFBOARD_DIR}/UserExtensions/Src/usb_hid_1ffb_desc.c
	${FFBOARD_DIR}/UserExtensions/Src/usb_hid_2ffb_desc.c
	${FFBOARD_DIR}/UserExtensions/Src/usb_hid_gamepad.c
	# System: startup, interrupt callbacks, voltage sensing, status LEDs (P4).
	# cppmain.cpp is compiled through platform/src/host_cppmain.cpp (TARGET_PLATFORM_SOURCES): it defines malloc() and
	# free(), which the wrapper renames (see there).
	${FFBOARD_DIR}/Src/global_callbacks.cpp
	${FFBOARD_DIR}/Src/voltagesense.cpp
	${FFBOARD_DIR}/Src/ledEffects.cpp
	# CAN port of the F407 (P7): the global canport is the real CANPort_2B on the fake HAL CAN (host_target_can.cpp)
	${FFBOARD_DIR}/Src/CANPort2B.cpp
)

# Compiled again for this executable (they include main.h / target_constants.h). host_stubs.cpp is replaced by
# the real SystemCommands.cpp. host_target_system.cpp: fake HAL ADC (F6), IWDG, the TIM7 HAL tick and
# the main.c callbacks. host_target_bus.cpp: fake HAL SPI (F4) and I2C (F5). host_target_can.cpp: fake HAL CAN (F8). Every HAL function the compiled sources
# call has a fake; paths a fake does not model are recorded in HostTarget::state().unmodelledHalCalls.
set(TARGET_PLATFORM_SOURCES
	platform/src/host_freertos.cpp
	platform/src/host_platform.cpp
	platform/src/host_usb.cpp
	platform/src/host_target.cpp
	platform/src/host_target_config.cpp
	platform/src/host_target_system.cpp
	platform/src/host_target_bus.cpp
	platform/src/host_target_can.cpp
	platform/src/host_cppmain.cpp
)

# Configure-time guard for platform/src/host_cppmain.cpp: every #include of cppmain.cpp must be in the wrapper's
# pre-include list (the lines before "End of the pre-include list"). Otherwise the malloc/free rename of the wrapper
# would apply to a newly included header.
file(STRINGS ${FFBOARD_DIR}/Src/cppmain.cpp FFB_CPPMAIN_INCLUDES REGEX "^[ \t]*#[ \t]*include")
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/platform/src/host_cppmain.cpp FFB_CPPMAIN_WRAPPER)
string(FIND "${FFB_CPPMAIN_WRAPPER}" "// End of the pre-include list" FFB_CPPMAIN_WRAPPER_END)
if(FFB_CPPMAIN_WRAPPER_END LESS 0)
	message(FATAL_ERROR "platform/src/host_cppmain.cpp: marker '// End of the pre-include list' not found")
endif()
string(SUBSTRING "${FFB_CPPMAIN_WRAPPER}" 0 ${FFB_CPPMAIN_WRAPPER_END} FFB_CPPMAIN_WRAPPER_HEAD)
string(REGEX MATCHALL "#[ \t]*include[ \t]*[\"<][^\">]+[\">]" FFB_CPPMAIN_WRAPPER_INCLUDES "${FFB_CPPMAIN_WRAPPER_HEAD}")
string(REGEX REPLACE "[ \t]" "" FFB_CPPMAIN_WRAPPER_INCLUDES "${FFB_CPPMAIN_WRAPPER_INCLUDES}")
foreach(inc IN LISTS FFB_CPPMAIN_INCLUDES)
	string(REGEX MATCH "#[ \t]*include[ \t]*[\"<][^\">]+[\">]" inc "${inc}")
	string(REGEX REPLACE "[ \t]" "" inc "${inc}")
	if(NOT inc IN_LIST FFB_CPPMAIN_WRAPPER_INCLUDES)
		message(FATAL_ERROR "FFBoard/Src/cppmain.cpp has '${inc}', which is missing in the pre-include list of "
			"Firmware/Tests/platform/src/host_cppmain.cpp. Add it there (before the malloc/free rename).")
	endif()
endforeach()

# ledEffects.cpp is real code in ffb_target_tests (HOSTTEST_REAL_LED_EFFECTS below). The existing tests count the
# calls of the pulse and set functions (HostPlatform::leds()); these six functions are renamed for this source file
# only and host_platform.cpp defines the public names as counting wrappers that call the real functions.
set_property(SOURCE ${FFBOARD_DIR}/Src/ledEffects.cpp APPEND PROPERTY COMPILE_DEFINITIONS
	pulseSysLed=hosttest_real_pulseSysLed pulseErrLed=hosttest_real_pulseErrLed pulseClipLed=hosttest_real_pulseClipLed
	setSysLed=hosttest_real_setSysLed setErrLed=hosttest_real_setErrLed setClipLed=hosttest_real_setClipLed)

# Axis.cpp: target include order and compile workarounds (no firmware change allowed, see Tests/PROGRESS.md F14):
#  - -include math.h (both DSP variants): on the shipped targets (arm-none-eabi-g++, TMC4671DRIVER on) Axis.cpp gets
#    <math.h> transitively via TMC4671.h -> SPI.h:17. The libstdc++ math.h wrapper does `using std::abs;`, so
#    abs(float) in Axis.cpp (setExpo, updateHandsOffState) calls the float overload. ffb_target_tests has the
#    drivers off and would only see ::abs(int). Including math.h first restores the target's overload set.
#    It also provides fabsf, powf, sqrtf, sinf and M_PI, which Axis.cpp uses without including <cmath> (with
#    CMSIS-DSP the DSP headers include math.h; without them Axis.cpp would not compile: F14).
#  - setEqGain calls clip<int8_t>(gain, -120, 120): the braced return in clip() (cppmain.h) narrows int to
#    int8_t. GCC warns, clang rejects it (-Wc++11-narrowing is an error by default). The value is in range.
set_property(SOURCE ${FFBOARD_DIR}/Src/Axis.cpp APPEND PROPERTY COMPILE_OPTIONS -include math.h)
set_property(SOURCE ${FFBOARD_DIR}/Src/Axis.cpp APPEND PROPERTY COMPILE_OPTIONS $<$<CXX_COMPILER_ID:Clang,AppleClang>:-Wno-c++11-narrowing>)

file(GLOB TARGET_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cases_target/test_*.cpp)

add_executable(ffb_target_tests
	${TARGET_FIRMWARE_SOURCES}
	${TARGET_PLATFORM_SOURCES}
	${TARGET_TEST_SOURCES}
	support/test_main.cpp
)

# Order matters: the F407 feature header (platform/target_f407) shadows the one of ffb_unit_tests in
# platform/include. No fakes/ directory: the real Axis.h is used.
target_include_directories(ffb_target_tests PRIVATE
	platform/target_f407
	platform/include
	support
	mocks
	third_party
	${FFBOARD_DIR}/Inc
	${FFBOARD_DIR}/UserExtensions/Inc
	${FFBOARD_DIR}/USB
	${FFBOARD_DIR}/USB/common
	${FFBOARD_DIR}/USB/device
	${FFBOARD_DIR}/USB/class/hid
	${FFBOARD_DIR}/USB/class/cdc
	${FFBOARD_DIR}/USB/class/midi
)

# HOSTTEST_REAL_CPPMAIN: host_platform.cpp leaves out its micros() / refreshWatchdog() / getRunTimeCounterValue() /
#   Error_Handler() fakes (real cppmain.cpp, main.c model in host_target_system.cpp) and keeps the TIM7 counter of the
#   HAL tick in step with the simulated time; platform/include/task.h routes taskYIELD() to a hook (cppmain loop).
# HOSTTEST_REAL_LED_EFFECTS: host_platform.cpp leaves out its LED fake (real ledEffects.cpp, see above).
# HOSTTEST_TARGET_OTP: host_platform.cpp leaves out its OTP_Write / OTP_Read stubs; host_target_system.cpp models them
#   like flash_helpers.cpp on an in-memory OTP area (SystemCommands "signature").
target_compile_definitions(ffb_target_tests PRIVATE HOSTTEST HOSTTEST_REAL_CPPMAIN HOSTTEST_REAL_LED_EFFECTS HOSTTEST_TARGET_OTP)

if(USE_DSP)
	target_sources(ffb_target_tests PRIVATE
		${DSP_DIR}/Source/CommonTables/arm_common_tables.c
		${DSP_DIR}/Source/FastMathFunctions/arm_sin_f32.c
		${DSP_DIR}/Source/FastMathFunctions/arm_cos_f32.c
		${DSP_DIR}/Source/FilteringFunctions/arm_biquad_cascade_df1_f32.c
		${DSP_DIR}/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c
		${DSP_DIR}/Source/InterpolationFunctions/arm_spline_interp_f32.c
		${DSP_DIR}/Source/InterpolationFunctions/arm_spline_interp_init_f32.c
		# Axis.cpp: speed limiter PID and hands-off detection
		${DSP_DIR}/Source/ControllerFunctions/arm_pid_init_f32.c
		${DSP_DIR}/Source/ControllerFunctions/arm_pid_reset_f32.c
		${DSP_DIR}/Source/StatisticsFunctions/arm_std_f32.c
		${DSP_DIR}/Source/StatisticsFunctions/arm_var_f32.c
	)
	target_include_directories(ffb_target_tests PRIVATE ${DSP_DIR}/Include ${DSP_DIR}/PrivateInclude)
	target_compile_definitions(ffb_target_tests PRIVATE USE_DSP_FUNCTIONS ARM_MATH_LOOPUNROLL __GNUC_PYTHON__)
endif()

target_link_libraries(ffb_target_tests PRIVATE Threads::Threads)

# -funsigned-char: char is unsigned on the ARM target (AAPCS). Without it CanBridge.h does not compile
# (narrowing 0xF1 to char) and char arithmetic would differ from the target.
target_compile_options(ffb_target_tests PRIVATE -Wall $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile> -funsigned-char)
if(MINGW)
	target_compile_options(ffb_target_tests PRIVATE -mno-ms-bitfields)
	target_link_options(ffb_target_tests PRIVATE -static)
endif()
if(MINGW OR APPLE)
	# SystemCommands.cpp uses struct mallinfo / mallinfo() from <malloc.h>, which mingw-w64 does not declare and the
	# macOS SDK does not have: shim header (#include_next if a <malloc.h> exists) and a stub returning zeros.
	# Unverified (no MinGW or macOS toolchain available). Not used on Linux (glibc declares both).
	target_include_directories(ffb_target_tests BEFORE PRIVATE platform/mingw)
	target_sources(ffb_target_tests PRIVATE platform/mingw/host_mallinfo.cpp)
endif()

if(FFB_TESTS_SANITIZE)
	target_compile_definitions(ffb_target_tests PRIVATE HOSTTEST_SANITIZERS)
	target_compile_options(ffb_target_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
	target_link_options(ffb_target_tests PRIVATE -fsanitize=address,undefined)
endif()

# Test cases of ffb_target_tests excluded from ctest (doctest --test-case-exclude). Same rule as FFB_TESTS_EXCLUDE:
# only cases that abort the whole run because of a suspected defect belong here.
set(FFB_TARGET_TESTS_EXCLUDE "" CACHE STRING "Test cases of ffb_target_tests excluded from ctest (list)")
# Additionally excluded in sanitizer builds (like FFB_TESTS_EXCLUDE_SANITIZE). Cases deleting an FFBHIDMain skip
# themselves with DELETES_FFBHIDMAIN and need no entry here.
set(FFB_TARGET_TESTS_EXCLUDE_SANITIZE "" CACHE STRING "Test cases of ffb_target_tests additionally excluded from ctest in sanitizer builds (list)")
set(FFB_TARGET_TESTS_EXCLUDED ${FFB_TARGET_TESTS_EXCLUDE})
if(FFB_TESTS_SANITIZE)
	list(APPEND FFB_TARGET_TESTS_EXCLUDED ${FFB_TARGET_TESTS_EXCLUDE_SANITIZE})
endif()
list(JOIN FFB_TARGET_TESTS_EXCLUDED "," FFB_TARGET_TESTS_EXCLUDED_ARG)
message(STATUS "OpenFFBoard tests: ffb_target_tests excluded from ctest: ${FFB_TARGET_TESTS_EXCLUDED_ARG}")
if(FFB_TARGET_TESTS_EXCLUDED_ARG)
	add_test(NAME ffb_target_tests COMMAND ffb_target_tests "--test-case-exclude=${FFB_TARGET_TESTS_EXCLUDED_ARG}")
else()
	add_test(NAME ffb_target_tests COMMAND ffb_target_tests)
endif()

# ===========================================================================
# ffb_flash_tests: the real flash layer (flash_helpers.cpp, eeprom.c = ST AN3969 EEPROM emulation, vendor tier A,
# eeprom_addresses.c) and PersistentStorage on a fake HAL FLASH with NOR semantics (P3).
#
# ffb_unit_tests and ffb_target_tests above are not changed by this section. Feature set: the F407 test header
# (platform/target_f407/target_constants.h: USE_EEPROM_EMULATION, EEPROM_START_ADDRESS 0x08004000).
#  - HOSTTEST_FLASH_HAL: platform/target_f407/main.h includes host_flash_hal.h (FLASH HAL declarations, FLASH_OTP_BASE,
#    so constants.h defines OTPMEMORY like on the F407). Implementation: platform/src/host_flash.cpp.
#  - HOSTTEST_REAL_FLASH_HELPERS: host_platform.cpp leaves out its map-backed Flash_* replacement.
# The fake flash memory is mapped at the target addresses (0x08000000 and 0x1FFF0000, 64 KiB each) when the
# executable starts, see host_flash.cpp.
# Not built on macOS: the default 4 GiB __PAGEZERO of macOS executables covers these addresses (decided 2026-10-24).
# ===========================================================================
if(APPLE)
message(STATUS "OpenFFBoard tests: ffb_flash_tests is not built on macOS (fixed-address flash memory below 4 GiB)")
else() # NOT APPLE: everything up to the matching endif() at the end of this section
set(FLASH_FIRMWARE_SOURCES
	# Under test (P3)
	${FFBOARD_DIR}/Src/flash_helpers.cpp
	${FFBOARD_DIR}/Src/eeprom.c
	${FFBOARD_DIR}/UserExtensions/Src/eeprom_addresses.c
	${FFBOARD_DIR}/Src/PersistentStorage.cpp
	# Needed to link: flashMutex in flash_helpers.cpp, external_uart in host_platform.cpp
	${FFBOARD_DIR}/Src/cmutex.cpp
	${FFBOARD_DIR}/Src/csemaphore.cpp
	${FFBOARD_DIR}/Src/UART.cpp
	${FFBOARD_DIR}/Src/UartHandler.cpp
)

set(FLASH_PLATFORM_SOURCES
	platform/src/host_freertos.cpp
	platform/src/host_platform.cpp
	platform/src/host_flash.cpp
)

file(GLOB FLASH_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cases_flash/test_*.cpp)

add_executable(ffb_flash_tests
	${FLASH_FIRMWARE_SOURCES}
	${FLASH_PLATFORM_SOURCES}
	${FLASH_TEST_SOURCES}
	support/test_main.cpp
)

target_include_directories(ffb_flash_tests PRIVATE
	platform/target_f407
	platform/include
	support
	mocks
	third_party
	${FFBOARD_DIR}/Inc
	${FFBOARD_DIR}/UserExtensions/Inc
	${FFBOARD_DIR}/USB
	${FFBOARD_DIR}/USB/common
	${FFBOARD_DIR}/USB/device
	${FFBOARD_DIR}/USB/class/hid
	${FFBOARD_DIR}/USB/class/cdc
	${FFBOARD_DIR}/USB/class/midi
)

target_compile_definitions(ffb_flash_tests PRIVATE HOSTTEST HOSTTEST_FLASH_HAL HOSTTEST_REAL_FLASH_HELPERS)
# Header parsed at run time by cases_flash/test_eeprom_addresses.cpp (consistency of VirtAddVarTab with the ADR_ constants)
get_filename_component(FFB_EEPROM_ADDRESSES_HEADER ${FFBOARD_DIR}/UserExtensions/Inc/eeprom_addresses.h ABSOLUTE)
target_compile_definitions(ffb_flash_tests PRIVATE FFB_EEPROM_ADDRESSES_H="${FFB_EEPROM_ADDRESSES_HEADER}")

target_link_libraries(ffb_flash_tests PRIVATE Threads::Threads)

# -funsigned-char: char is unsigned on the ARM target (like ffb_target_tests)
target_compile_options(ffb_flash_tests PRIVATE -Wall $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile> -funsigned-char)
if(MINGW)
	target_compile_options(ffb_flash_tests PRIVATE -mno-ms-bitfields)
	target_link_options(ffb_flash_tests PRIVATE -static)
endif()

if(FFB_TESTS_SANITIZE)
	target_compile_definitions(ffb_flash_tests PRIVATE HOSTTEST_SANITIZERS)
	target_compile_options(ffb_flash_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
	target_link_options(ffb_flash_tests PRIVATE -fsanitize=address,undefined)
endif()

# Test cases of ffb_flash_tests excluded from ctest (doctest --test-case-exclude). Same rule as FFB_TESTS_EXCLUDE:
# only cases that abort the whole run because of a suspected defect belong here.
set(FFB_FLASH_TESTS_EXCLUDE "" CACHE STRING "Test cases of ffb_flash_tests excluded from ctest (list)")
set(FFB_FLASH_TESTS_EXCLUDE_SANITIZE "" CACHE STRING "Test cases of ffb_flash_tests additionally excluded from ctest in sanitizer builds (list)")
set(FFB_FLASH_TESTS_EXCLUDED ${FFB_FLASH_TESTS_EXCLUDE})
if(FFB_TESTS_SANITIZE)
	list(APPEND FFB_FLASH_TESTS_EXCLUDED ${FFB_FLASH_TESTS_EXCLUDE_SANITIZE})
endif()
list(JOIN FFB_FLASH_TESTS_EXCLUDED "," FFB_FLASH_TESTS_EXCLUDED_ARG)
message(STATUS "OpenFFBoard tests: ffb_flash_tests excluded from ctest: ${FFB_FLASH_TESTS_EXCLUDED_ARG}")
if(FFB_FLASH_TESTS_EXCLUDED_ARG)
	add_test(NAME ffb_flash_tests COMMAND ffb_flash_tests "--test-case-exclude=${FFB_FLASH_TESTS_EXCLUDED_ARG}")
else()
	add_test(NAME ffb_flash_tests COMMAND ffb_flash_tests)
endif()
endif() # NOT APPLE (ffb_flash_tests)

# ===========================================================================
# ffb_rtos_tests: the FreeRTOS C++ wrappers (cpp_freertos, FFBoard/Src/c*.cpp, vendor tier A) on the fake FreeRTOS API
# F11 (P8): queues, software timers with the timer service task, event groups, mutex ownership, recursive mutexes,
# counting semaphores, critical sections, scheduler suspension, tick hook, RTOS heap, create-failure injection.
#
# ffb_unit_tests, ffb_target_tests and ffb_flash_tests above are not changed by this section.
#  - HOSTTEST_RTOS_EXT: host_freertos.cpp includes platform/src/host_freertos_ext.inc (the F11 fakes, replacing its
#    semaphore / notification / heap / task creation functions), host_platform.cpp runs one SysTick per simulated ms,
#    the fake FreeRTOS headers in platform/include declare the queue / timer / event group API and the F407 switches
#    the wrappers evaluate (FreeRTOSConfig.h: static allocation, trace facility, xTimerPendFunctionCall, condition
#    variables; the tick hook is on although the F407 has configUSE_TICK_HOOK 0).
#  - cmem_pool.cpp is compiled through platform/src/host_cmem_pool.cpp (its malloc() goes to pvPortMalloc like on the
#    target, cppmain.cpp:138).
# Feature header: the generic test header of platform/include (the wrappers use no feature macro). No CMSIS-DSP.
# Built on every host (no fixed addresses, unlike ffb_flash_tests).
# ===========================================================================
set(RTOS_FIRMWARE_SOURCES
	# Under test (P8): the cpp_freertos wrappers. cmem_pool.cpp: see RTOS_PLATFORM_SOURCES
	${FFBOARD_DIR}/Src/cthread.cpp
	${FFBOARD_DIR}/Src/cmutex.cpp
	${FFBOARD_DIR}/Src/csemaphore.cpp
	${FFBOARD_DIR}/Src/cqueue.cpp
	${FFBOARD_DIR}/Src/ctimer.cpp
	${FFBOARD_DIR}/Src/ctasklet.cpp
	${FFBOARD_DIR}/Src/ctickhook.cpp
	${FFBOARD_DIR}/Src/cevent_groups.cpp
	${FFBOARD_DIR}/Src/cworkqueue.cpp
	${FFBOARD_DIR}/Src/cread_write_lock.cpp
	${FFBOARD_DIR}/Src/ccondition_variable.cpp
	# Needed to link: external_uart in host_platform.cpp
	${FFBOARD_DIR}/Src/UART.cpp
	${FFBOARD_DIR}/Src/UartHandler.cpp
)

set(RTOS_PLATFORM_SOURCES
	platform/src/host_freertos.cpp
	platform/src/host_platform.cpp
	platform/src/host_cmem_pool.cpp
)

# Configure-time guard for platform/src/host_cmem_pool.cpp (like host_cppmain.cpp above): every #include of
# cmem_pool.cpp must be in the wrapper's pre-include list, otherwise the malloc macro would reach a header.
file(STRINGS ${FFBOARD_DIR}/Src/cmem_pool.cpp FFB_CMEM_POOL_INCLUDES REGEX "^[ \t]*#[ \t]*include")
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/platform/src/host_cmem_pool.cpp FFB_CMEM_POOL_WRAPPER)
string(FIND "${FFB_CMEM_POOL_WRAPPER}" "// End of the pre-include list" FFB_CMEM_POOL_WRAPPER_END)
if(FFB_CMEM_POOL_WRAPPER_END LESS 0)
	message(FATAL_ERROR "platform/src/host_cmem_pool.cpp: marker '// End of the pre-include list' not found")
endif()
string(SUBSTRING "${FFB_CMEM_POOL_WRAPPER}" 0 ${FFB_CMEM_POOL_WRAPPER_END} FFB_CMEM_POOL_WRAPPER_HEAD)
string(REGEX MATCHALL "#[ \t]*include[ \t]*[\"<][^\">]+[\">]" FFB_CMEM_POOL_WRAPPER_INCLUDES "${FFB_CMEM_POOL_WRAPPER_HEAD}")
string(REGEX REPLACE "[ \t]" "" FFB_CMEM_POOL_WRAPPER_INCLUDES "${FFB_CMEM_POOL_WRAPPER_INCLUDES}")
foreach(inc IN LISTS FFB_CMEM_POOL_INCLUDES)
	string(REGEX MATCH "#[ \t]*include[ \t]*[\"<][^\">]+[\">]" inc "${inc}")
	string(REGEX REPLACE "[ \t]" "" inc "${inc}")
	if(NOT inc IN_LIST FFB_CMEM_POOL_WRAPPER_INCLUDES)
		message(FATAL_ERROR "FFBoard/Src/cmem_pool.cpp has '${inc}', which is missing in the pre-include list of "
			"Firmware/Tests/platform/src/host_cmem_pool.cpp. Add it there (before the malloc macro).")
	endif()
endforeach()

file(GLOB RTOS_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cases_rtos/test_*.cpp)

add_executable(ffb_rtos_tests
	${RTOS_FIRMWARE_SOURCES}
	${RTOS_PLATFORM_SOURCES}
	${RTOS_TEST_SOURCES}
	support/test_main.cpp
)

# The USB directories only for the headers host_platform.cpp / UART.cpp include (no USB code is compiled)
target_include_directories(ffb_rtos_tests PRIVATE
	platform/include
	support
	mocks
	third_party
	${FFBOARD_DIR}/Inc
	${FFBOARD_DIR}/UserExtensions/Inc
	${FFBOARD_DIR}/USB
	${FFBOARD_DIR}/USB/common
	${FFBOARD_DIR}/USB/device
	${FFBOARD_DIR}/USB/class/hid
	${FFBOARD_DIR}/USB/class/cdc
	${FFBOARD_DIR}/USB/class/midi
)

target_compile_definitions(ffb_rtos_tests PRIVATE HOSTTEST HOSTTEST_RTOS_EXT)

target_link_libraries(ffb_rtos_tests PRIVATE Threads::Threads)

# -funsigned-char: char is unsigned on the ARM target (like ffb_target_tests)
target_compile_options(ffb_rtos_tests PRIVATE -Wall $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile> -funsigned-char)
if(MINGW)
	target_compile_options(ffb_rtos_tests PRIVATE -mno-ms-bitfields)
	target_link_options(ffb_rtos_tests PRIVATE -static)
endif()

if(FFB_TESTS_SANITIZE)
	target_compile_definitions(ffb_rtos_tests PRIVATE HOSTTEST_SANITIZERS)
	target_compile_options(ffb_rtos_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
	target_link_options(ffb_rtos_tests PRIVATE -fsanitize=address,undefined)
endif()

# Test cases of ffb_rtos_tests excluded from ctest (doctest --test-case-exclude). Same rule as FFB_TESTS_EXCLUDE:
# only cases that abort the whole run because of a suspected defect belong here.
set(FFB_RTOS_TESTS_EXCLUDE "" CACHE STRING "Test cases of ffb_rtos_tests excluded from ctest (list)")
set(FFB_RTOS_TESTS_EXCLUDE_SANITIZE "" CACHE STRING "Test cases of ffb_rtos_tests additionally excluded from ctest in sanitizer builds (list)")
set(FFB_RTOS_TESTS_EXCLUDED ${FFB_RTOS_TESTS_EXCLUDE})
if(FFB_TESTS_SANITIZE)
	list(APPEND FFB_RTOS_TESTS_EXCLUDED ${FFB_RTOS_TESTS_EXCLUDE_SANITIZE})
endif()
list(JOIN FFB_RTOS_TESTS_EXCLUDED "," FFB_RTOS_TESTS_EXCLUDED_ARG)
message(STATUS "OpenFFBoard tests: ffb_rtos_tests excluded from ctest: ${FFB_RTOS_TESTS_EXCLUDED_ARG}")
if(FFB_RTOS_TESTS_EXCLUDED_ARG)
	add_test(NAME ffb_rtos_tests COMMAND ffb_rtos_tests "--test-case-exclude=${FFB_RTOS_TESTS_EXCLUDED_ARG}")
else()
	add_test(NAME ffb_rtos_tests COMMAND ffb_rtos_tests)
endif()

# ===========================================================================
# ffb_usb_tests: the TinyUSB device stack (FFBoard/USB, vendor tier B: tested, never fixed) on a fake USB device
# controller (F15, P9): usbd.c, usbd_control.c, tusb.c, tusb_fifo.c and the CDC / HID / MIDI class drivers with the
# F407 tusb_config.h (OPT_MCU_STM32F4, OPT_OS_FREERTOS), and the firmware's USB glue (USBdevice.cpp, CDCcomm.cpp,
# UsbHidHandler.cpp, the TinyUSB callbacks of global_callbacks.cpp, the descriptors).
#
# The executables above are not changed by this section (their compile commands are identical).
#  - Firmware: the sources of ffb_target_tests (F407 feature set without drivers and MIDI; global_callbacks.cpp needs
#    them) plus ctickhook.cpp (vApplicationTickHook, called by the SysTick of the fake API F11) and
#    ccondition_variable.cpp (Thread::Wait with the F407 switches of the fake API).
#  - Platform: the platform sources of ffb_target_tests without host_usb.cpp (the recording TinyUSB fake), plus
#    host_dcd.cpp (fake device controller, support/HostDcd.h) and host_usb_osal.cpp.
#  - platform/usb is the first include directory: its tusb_config.h includes the real F407 tusb_config.h
#    (FFB_F407_TUSB_CONFIG_H) and points CFG_TUSB_OS_INC_PATH at platform/usb/hosttest_osal/ (what osal_freertos.h
#    needs on top of the fake FreeRTOS API: static creation, xQueueReset on a semaphore).
#  - HOSTTEST_RTOS_EXT: the fake FreeRTOS API F11 (queues for TinyUSB's event queue, see ffb_rtos_tests).
#  - HOSTTEST_REAL_TINYUSB: host_target.cpp leaves out its models of tusb_rhport_init / tud_task_ext / tud_speed_get.
#  - C sources with -fexceptions: a resumable task parked in TinyUSB (e.g. the USBdevice thread) is unwound through the
#    C frames of usbd.c when it is deleted.
# Built on every host (no fixed addresses). MIDI: CFG_TUD_MIDI is 1 in the F407 tusb_config.h, so midi_device.c is
# compiled (the usbd.c driver table needs it); the firmware's MIDI feature (MidiMain, MidiHandler) stays off.
# ===========================================================================
set(USB_DIR ${FFBOARD_DIR}/USB)
set(USB_TINYUSB_SOURCES
	${USB_DIR}/tusb.c
	${USB_DIR}/common/tusb_fifo.c
	${USB_DIR}/device/usbd.c
	${USB_DIR}/device/usbd_control.c
	${USB_DIR}/class/cdc/cdc_device.c
	${USB_DIR}/class/hid/hid_device.c
	${USB_DIR}/class/midi/midi_device.c
)

set(USB_FIRMWARE_SOURCES
	${TARGET_FIRMWARE_SOURCES}
	${FFBOARD_DIR}/Src/ctickhook.cpp
	${FFBOARD_DIR}/Src/ccondition_variable.cpp # Thread::Wait (CPP_FREERTOS_CONDITION_VARIABLES, F407 FreeRTOSConfig.h:187)
)

set(USB_PLATFORM_SOURCES ${TARGET_PLATFORM_SOURCES})
list(REMOVE_ITEM USB_PLATFORM_SOURCES platform/src/host_usb.cpp)
list(APPEND USB_PLATFORM_SOURCES
	platform/src/host_dcd.cpp
	platform/src/host_usb_osal.cpp
)

file(GLOB USB_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cases_usb/test_*.cpp)

add_executable(ffb_usb_tests
	${USB_TINYUSB_SOURCES}
	${USB_FIRMWARE_SOURCES}
	${USB_PLATFORM_SOURCES}
	${USB_TEST_SOURCES}
	support/test_main.cpp
)

target_include_directories(ffb_usb_tests PRIVATE
	platform/usb
	platform/target_f407
	platform/include
	support
	mocks
	third_party
	${FFBOARD_DIR}/Inc
	${FFBOARD_DIR}/UserExtensions/Inc
	${USB_DIR}
	${USB_DIR}/common
	${USB_DIR}/device
	${USB_DIR}/class/hid
	${USB_DIR}/class/cdc
	${USB_DIR}/class/midi
)

target_compile_definitions(ffb_usb_tests PRIVATE HOSTTEST HOSTTEST_REAL_CPPMAIN HOSTTEST_REAL_LED_EFFECTS HOSTTEST_TARGET_OTP
	HOSTTEST_RTOS_EXT HOSTTEST_REAL_TINYUSB)
get_filename_component(FFB_F407_TUSB_CONFIG ${FW_DIR}/Targets/F407VG/Core/Inc/tusb_config.h ABSOLUTE)
target_compile_definitions(ffb_usb_tests PRIVATE FFB_F407_TUSB_CONFIG_H="${FFB_F407_TUSB_CONFIG}")

if(USE_DSP)
	target_sources(ffb_usb_tests PRIVATE
		${DSP_DIR}/Source/CommonTables/arm_common_tables.c
		${DSP_DIR}/Source/FastMathFunctions/arm_sin_f32.c
		${DSP_DIR}/Source/FastMathFunctions/arm_cos_f32.c
		${DSP_DIR}/Source/FilteringFunctions/arm_biquad_cascade_df1_f32.c
		${DSP_DIR}/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c
		${DSP_DIR}/Source/InterpolationFunctions/arm_spline_interp_f32.c
		${DSP_DIR}/Source/InterpolationFunctions/arm_spline_interp_init_f32.c
		${DSP_DIR}/Source/ControllerFunctions/arm_pid_init_f32.c
		${DSP_DIR}/Source/ControllerFunctions/arm_pid_reset_f32.c
		${DSP_DIR}/Source/StatisticsFunctions/arm_std_f32.c
		${DSP_DIR}/Source/StatisticsFunctions/arm_var_f32.c
	)
	target_include_directories(ffb_usb_tests PRIVATE ${DSP_DIR}/Include ${DSP_DIR}/PrivateInclude)
	target_compile_definitions(ffb_usb_tests PRIVATE USE_DSP_FUNCTIONS ARM_MATH_LOOPUNROLL __GNUC_PYTHON__)
endif()

target_link_libraries(ffb_usb_tests PRIVATE Threads::Threads)

# -funsigned-char: char is unsigned on the ARM target (like ffb_target_tests)
target_compile_options(ffb_usb_tests PRIVATE -Wall $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile> $<$<COMPILE_LANGUAGE:C>:-fexceptions> -funsigned-char)
if(MINGW)
	target_compile_options(ffb_usb_tests PRIVATE -mno-ms-bitfields)
	target_link_options(ffb_usb_tests PRIVATE -static)
	target_include_directories(ffb_usb_tests BEFORE PRIVATE platform/mingw)
	target_sources(ffb_usb_tests PRIVATE platform/mingw/host_mallinfo.cpp)
endif()

if(FFB_TESTS_SANITIZE)
	target_compile_definitions(ffb_usb_tests PRIVATE HOSTTEST_SANITIZERS)
	target_compile_options(ffb_usb_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
	target_link_options(ffb_usb_tests PRIVATE -fsanitize=address,undefined)
endif()

# Test cases of ffb_usb_tests excluded from ctest (doctest --test-case-exclude). Same rule as FFB_TESTS_EXCLUDE:
# only cases that abort the whole run because of a suspected defect belong here.
set(FFB_USB_TESTS_EXCLUDE "" CACHE STRING "Test cases of ffb_usb_tests excluded from ctest (list)")
set(FFB_USB_TESTS_EXCLUDE_SANITIZE "" CACHE STRING "Test cases of ffb_usb_tests additionally excluded from ctest in sanitizer builds (list)")
set(FFB_USB_TESTS_EXCLUDED ${FFB_USB_TESTS_EXCLUDE})
if(FFB_TESTS_SANITIZE)
	list(APPEND FFB_USB_TESTS_EXCLUDED ${FFB_USB_TESTS_EXCLUDE_SANITIZE})
endif()
list(JOIN FFB_USB_TESTS_EXCLUDED "," FFB_USB_TESTS_EXCLUDED_ARG)
message(STATUS "OpenFFBoard tests: ffb_usb_tests excluded from ctest: ${FFB_USB_TESTS_EXCLUDED_ARG}")
if(FFB_USB_TESTS_EXCLUDED_ARG)
	add_test(NAME ffb_usb_tests COMMAND ffb_usb_tests "--test-case-exclude=${FFB_USB_TESTS_EXCLUDED_ARG}")
else()
	add_test(NAME ffb_usb_tests COMMAND ffb_usb_tests)
endif()

# ===========================================================================
# ffb_driver_tests (driver track, Tests/DRIVER_TEST_PLAN.md): the real TMC4671, ODrive and VESC drivers on the
# fake HAL SPI / CAN of ffb_target_tests. Feature header platform/target_f407_drivers/target_constants.h = the
# ffb_target_tests header + TMC4671DRIVER, ODRIVE, VESC (MIDI, TMCDEBUG, RMDCAN, PWMDRIVER, SIMPLEMOTION stay off).
#
# The executables above are not changed by this section (their compile commands are identical). The firmware and
# platform sources of ffb_target_tests are compiled again for this executable (they include target_constants.h).
# platform/target_f407_drivers is the first include directory: it shadows only target_constants.h (main.h,
# cmsis_os.h etc. still come from platform/target_f407). platform/src/host_driver_timers.cpp: fake TIM6 (htim6).
# Test cases: cases/driver/test_*.cpp (not picked up by ffb_unit_tests, whose glob is not recursive).
# ===========================================================================
set(DRIVER_FIRMWARE_SOURCES
	${TARGET_FIRMWARE_SOURCES}
	${FFBOARD_DIR}/UserExtensions/Src/TMC4671.cpp
	${FFBOARD_DIR}/UserExtensions/Src/TMC4671_configs.cpp
	${FFBOARD_DIR}/UserExtensions/Src/ODriveCAN.cpp
	${FFBOARD_DIR}/UserExtensions/Src/VescCAN.cpp
)
# TMC4671.cpp:2467 calls clip(maxcnt, 255, 4095) with an uint16_t value: the braced return in clip() (cppmain.h:67) narrows
# int to unsigned short. GCC only warns, clang rejects it (-Wc++11-narrowing is an error by default). Same workaround
# as Axis.cpp above (F14); the value is in range, the firmware source is not changed.
set_property(SOURCE ${FFBOARD_DIR}/UserExtensions/Src/TMC4671.cpp APPEND PROPERTY COMPILE_OPTIONS
	$<$<CXX_COMPILER_ID:Clang,AppleClang>:-Wno-c++11-narrowing>)
set(DRIVER_PLATFORM_SOURCES ${TARGET_PLATFORM_SOURCES} platform/src/host_driver_timers.cpp fakes/FakeTmc4671.cpp fakes/FakeOdrivePeer.cpp fakes/FakeVescPeer.cpp)
# FFB_DRIVER_TEST_FILTER (developer aid for parallel test writing): glob of the driver test files to compile, e.g.
# "test_tmc4671_init*.cpp". The setup and fake self-tests are always compiled. Default: all driver tests.
set(FFB_DRIVER_TEST_FILTER "test_*.cpp" CACHE STRING "Glob of the cases/driver test files compiled into ffb_driver_tests")
file(GLOB DRIVER_TEST_SOURCES CONFIGURE_DEPENDS
	${CMAKE_CURRENT_SOURCE_DIR}/cases/driver/${FFB_DRIVER_TEST_FILTER}
	${CMAKE_CURRENT_SOURCE_DIR}/cases/driver/test_driver_setup.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/cases/driver/test_tmc4671_fake.cpp)
list(REMOVE_DUPLICATES DRIVER_TEST_SOURCES)

add_executable(ffb_driver_tests
	${DRIVER_FIRMWARE_SOURCES}
	${DRIVER_PLATFORM_SOURCES}
	${DRIVER_TEST_SOURCES}
	support/test_main.cpp
)

target_include_directories(ffb_driver_tests PRIVATE
	platform/target_f407_drivers
	platform/target_f407
	platform/include
	support
	mocks
	third_party
	${FFBOARD_DIR}/Inc
	${FFBOARD_DIR}/UserExtensions/Inc
	${FFBOARD_DIR}/USB
	${FFBOARD_DIR}/USB/common
	${FFBOARD_DIR}/USB/device
	${FFBOARD_DIR}/USB/class/hid
	${FFBOARD_DIR}/USB/class/cdc
	${FFBOARD_DIR}/USB/class/midi
)

# Same definitions as ffb_target_tests (see there)
target_compile_definitions(ffb_driver_tests PRIVATE HOSTTEST HOSTTEST_REAL_CPPMAIN HOSTTEST_REAL_LED_EFFECTS HOSTTEST_TARGET_OTP)

if(USE_DSP)
	target_sources(ffb_driver_tests PRIVATE
		${DSP_DIR}/Source/CommonTables/arm_common_tables.c
		${DSP_DIR}/Source/FastMathFunctions/arm_sin_f32.c
		${DSP_DIR}/Source/FastMathFunctions/arm_cos_f32.c
		${DSP_DIR}/Source/FilteringFunctions/arm_biquad_cascade_df1_f32.c
		${DSP_DIR}/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c
		${DSP_DIR}/Source/InterpolationFunctions/arm_spline_interp_f32.c
		${DSP_DIR}/Source/InterpolationFunctions/arm_spline_interp_init_f32.c
		${DSP_DIR}/Source/ControllerFunctions/arm_pid_init_f32.c
		${DSP_DIR}/Source/ControllerFunctions/arm_pid_reset_f32.c
		${DSP_DIR}/Source/StatisticsFunctions/arm_std_f32.c
		${DSP_DIR}/Source/StatisticsFunctions/arm_var_f32.c
	)
	target_include_directories(ffb_driver_tests PRIVATE ${DSP_DIR}/Include ${DSP_DIR}/PrivateInclude)
	target_compile_definitions(ffb_driver_tests PRIVATE USE_DSP_FUNCTIONS ARM_MATH_LOOPUNROLL __GNUC_PYTHON__)
endif()

target_link_libraries(ffb_driver_tests PRIVATE Threads::Threads)

# -funsigned-char: char is unsigned on the ARM target (like ffb_target_tests)
target_compile_options(ffb_driver_tests PRIVATE -Wall $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile> -funsigned-char)
if(MINGW)
	target_compile_options(ffb_driver_tests PRIVATE -mno-ms-bitfields)
	target_link_options(ffb_driver_tests PRIVATE -static)
endif()
if(MINGW OR APPLE)
	# See ffb_target_tests: mallinfo shim. Unverified (no MinGW or macOS toolchain available).
	target_include_directories(ffb_driver_tests BEFORE PRIVATE platform/mingw)
	target_sources(ffb_driver_tests PRIVATE platform/mingw/host_mallinfo.cpp)
endif()

if(FFB_TESTS_SANITIZE)
	target_compile_definitions(ffb_driver_tests PRIVATE HOSTTEST_SANITIZERS)
	target_compile_options(ffb_driver_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
	target_link_options(ffb_driver_tests PRIVATE -fsanitize=address,undefined)
endif()

# Test cases of ffb_driver_tests excluded from ctest (doctest --test-case-exclude). Same rule as FFB_TESTS_EXCLUDE:
# only cases that abort the whole run because of a suspected defect belong here.
set(FFB_DRIVER_TESTS_EXCLUDE "" CACHE STRING "Test cases of ffb_driver_tests excluded from ctest (list)")
set(FFB_DRIVER_TESTS_EXCLUDE_SANITIZE "" CACHE STRING "Test cases of ffb_driver_tests additionally excluded from ctest in sanitizer builds (list)")
set(FFB_DRIVER_TESTS_EXCLUDED ${FFB_DRIVER_TESTS_EXCLUDE})
if(FFB_TESTS_SANITIZE)
	list(APPEND FFB_DRIVER_TESTS_EXCLUDED ${FFB_DRIVER_TESTS_EXCLUDE_SANITIZE})
endif()
list(JOIN FFB_DRIVER_TESTS_EXCLUDED "," FFB_DRIVER_TESTS_EXCLUDED_ARG)
message(STATUS "OpenFFBoard tests: ffb_driver_tests excluded from ctest: ${FFB_DRIVER_TESTS_EXCLUDED_ARG}")
if(FFB_DRIVER_TESTS_EXCLUDED_ARG)
	add_test(NAME ffb_driver_tests COMMAND ffb_driver_tests "--test-case-exclude=${FFB_DRIVER_TESTS_EXCLUDED_ARG}")
else()
	add_test(NAME ffb_driver_tests COMMAND ffb_driver_tests)
endif()

# ===========================================================================
# python_helper_tests: stdlib unittest smoke tests of the Python helper scripts (P9): Firmware/scripts/generate_memory.py
# and FFBoard/USB/portable/synopsys/dwc2/dwc2_info.py, in python/test_*.py. Only when Python 3 is found (like the
# baseline check). -B and PYTHONDONTWRITEBYTECODE: no __pycache__ is written into the repository; the scripts' outputs
# go to temporary directories created by the tests. Runs in the build directory.
# ===========================================================================
if(Python3_Interpreter_FOUND)
	add_test(NAME python_helper_tests
		COMMAND ${Python3_EXECUTABLE} -B -m unittest discover -v
			-s ${CMAKE_CURRENT_SOURCE_DIR}/python -t ${CMAKE_CURRENT_SOURCE_DIR}/python -p "test_*.py"
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
	set_tests_properties(python_helper_tests PROPERTIES ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
else()
	message(STATUS "OpenFFBoard tests: Python 3 not found, python_helper_tests not added to ctest")
endif()

# ===========================================================================
# test_report: full report of every test case (tools/run_all_tests.py). Not part of ctest. Runs every test case of
# every test executable of this build directory and the Python helper tests, nothing skipped (skip-decorated cases run
# one by one with --no-skip, crashing cases are isolated), and writes test_report.log (passed / failed / bugged) into
# the build directory. A failed test is reported as failed and the run carries on with the next test case; the target
# itself always succeeds (--exit-zero). Only when Python 3 is found (like the baseline check). Builds the executables first; their
# compile commands are not changed.
#   cmake --build <build-dir> --target test_report
# ===========================================================================
if(Python3_Interpreter_FOUND)
	add_custom_target(test_report
		COMMAND ${CMAKE_COMMAND} -E env PYTHONDONTWRITEBYTECODE=1
			${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_all_tests.py
			--build-dir ${CMAKE_CURRENT_BINARY_DIR} --out ${CMAKE_CURRENT_BINARY_DIR}/test_report.log --exit-zero
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Running every test case (report: ${CMAKE_CURRENT_BINARY_DIR}/test_report.log)"
		USES_TERMINAL
		VERBATIM)
	foreach(ffb_report_exe ffb_unit_tests ffb_target_tests ffb_flash_tests ffb_rtos_tests ffb_usb_tests ffb_driver_tests)
		if(TARGET ${ffb_report_exe})
			add_dependencies(test_report ${ffb_report_exe})
		endif()
	endforeach()
else()
	message(STATUS "OpenFFBoard tests: Python 3 not found, test_report target not added")
endif()
