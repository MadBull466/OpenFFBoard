cmake_minimum_required(VERSION 3.16)
project(CodexRealStHalTests LANGUAGES C CXX)

get_filename_component(FW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(P12_DEVICE "F407VG" CACHE STRING "Checked-in HAL/device configuration")
set_property(CACHE P12_DEVICE PROPERTY STRINGS F407VG F407VG_DISCO F411RE)
if(NOT P12_DEVICE MATCHES "^(F407VG|F407VG_DISCO|F411RE)$")
    message(FATAL_ERROR "Unsupported P12_DEVICE")
endif()

set(DRIVERS "${FW_DIR}/Targets/${P12_DEVICE}/Drivers")
set(P12_GPIO_SOURCE "${DRIVERS}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
    CACHE FILEPATH "Real GPIO translation unit; override only for scratch mutation checks")
option(P12_FULL_ASSERT "Enable vendor parameter assertions" OFF)
option(P12_SANITIZE "Enable address/undefined sanitizers" OFF)

add_executable(p12_gpio_tests
    "${P12_GPIO_SOURCE}" host/boundary.c test_main.cpp cases/test_gpio_io.cpp)
target_compile_features(p12_gpio_tests PRIVATE c_std_11 cxx_std_20)
target_include_directories(p12_gpio_tests PRIVATE
    host
    "${DRIVERS}/STM32F4xx_HAL_Driver/Inc"
    "${DRIVERS}/CMSIS/Device/ST/STM32F4xx/Include")
target_include_directories(p12_gpio_tests SYSTEM PRIVATE "${FW_DIR}/Tests/third_party")
if(P12_DEVICE STREQUAL "F411RE")
    target_compile_definitions(p12_gpio_tests PRIVATE STM32F411xE)
else()
    target_compile_definitions(p12_gpio_tests PRIVATE STM32F407xx)
endif()
if(P12_FULL_ASSERT)
    target_compile_definitions(p12_gpio_tests PRIVATE USE_FULL_ASSERT)
endif()
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(p12_gpio_tests PRIVATE -Wall -Wextra -Werror)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # doctest 2.4.11 explicitly includes ciso646, which libstdc++16 diagnoses
    # under C++20. Keep the dependency warning visible, without turning that
    # preprocessor warning into an error. No runtime/sanitizer suppression.
    target_compile_options(p12_gpio_tests PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-Wno-error=\#warnings>")
endif()
if(P12_SANITIZE)
    if(NOT CMAKE_C_COMPILER_ID MATCHES "GNU|Clang" OR WIN32)
        message(FATAL_ERROR "P12_SANITIZE requires GCC/Clang on a supported host")
    endif()
    target_compile_options(p12_gpio_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(p12_gpio_tests PRIVATE -fsanitize=address,undefined)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
enable_testing()
add_test(NAME p12_gpio_io COMMAND p12_gpio_tests --no-colors)
set_tests_properties(p12_gpio_io PROPERTIES TIMEOUT 10)

# Separate observation target: the supplied C source is included unchanged in
# C++ so register operators can observe EVERY word access. The real-C target
# above remains the independent production-language storage cross-check.
# RM0090/ES0182 support F407 only; do not claim F411 model fidelity.
if(NOT P12_DEVICE STREQUAL "F411RE")
    set(P12_MODEL_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/model_host/model.cpp" CACHE FILEPATH
        "Register model implementation; override only for scratch model discrimination")
    add_executable(p12_gpio_model_tests model_host/real_gpio.cpp "${P12_MODEL_SOURCE}"
        test_main.cpp cases/test_gpio_model.cpp)
    target_compile_features(p12_gpio_model_tests PRIVATE cxx_std_20)
    target_include_directories(p12_gpio_model_tests PRIVATE model_host host
        "${DRIVERS}/STM32F4xx_HAL_Driver/Inc"
        "${DRIVERS}/CMSIS/Device/ST/STM32F4xx/Include")
    target_include_directories(p12_gpio_model_tests SYSTEM PRIVATE "${FW_DIR}/Tests/third_party")
    target_compile_definitions(p12_gpio_model_tests PRIVATE STM32F407xx USE_FULL_ASSERT
        P12_GPIO_SOURCE_PATH="${P12_GPIO_SOURCE}")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(p12_gpio_model_tests PRIVATE -Wall -Wextra -Werror)
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(p12_gpio_model_tests PRIVATE "-Wno-error=\#warnings")
    endif()
    if(P12_SANITIZE)
        target_compile_options(p12_gpio_model_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(p12_gpio_model_tests PRIVATE -fsanitize=address,undefined)
    endif()
    add_test(NAME p12_gpio_register_model COMMAND p12_gpio_model_tests --no-colors)
    set_tests_properties(p12_gpio_register_model PROPERTIES TIMEOUT 10)
endif()

# Tick and DMA software-state prerequisites compile genuine vendor C. SysTick
# configuration/NVIC calls are lower CPU-boundary recorders, not core tests.
set(P12_HAL_SOURCE "${DRIVERS}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c" CACHE FILEPATH "Core HAL source; scratch overrides only")
set(P12_DMA_SOURCE "${DRIVERS}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c" CACHE FILEPATH "DMA source; scratch overrides only")
function(p12_time_target name)
    add_executable(${name} "${P12_HAL_SOURCE}" tick_host/tick_boundary.c test_main.cpp ${ARGN})
    target_compile_features(${name} PRIVATE c_std_11 cxx_std_20)
    target_include_directories(${name} PRIVATE tick_host host
        "${DRIVERS}/STM32F4xx_HAL_Driver/Inc"
        "${DRIVERS}/CMSIS/Device/ST/STM32F4xx/Include"
        "${FW_DIR}/Tests/support")
    target_include_directories(${name} SYSTEM PRIVATE "${FW_DIR}/Tests/third_party")
    if(P12_DEVICE STREQUAL "F411RE")
        target_compile_definitions(${name} PRIVATE STM32F411xE)
    else()
        target_compile_definitions(${name} PRIVATE STM32F407xx)
    endif()
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${name} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-Wno-error=\#warnings>")
    endif()
    if(P12_SANITIZE)
        target_compile_options(${name} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(${name} PRIVATE -fsanitize=address,undefined)
    endif()
    add_test(NAME ${name} COMMAND ${name} --no-colors)
    set_tests_properties(${name} PROPERTIES TIMEOUT 10)
endfunction()
p12_time_target(p12_tick_tests cases/test_tick.cpp)
p12_time_target(p12_delay_tests tick_host/scripted_get_tick.c cases/test_delay.cpp)
p12_time_target(p12_dma_state_tests "${P12_DMA_SOURCE}" cases/test_dma_state.cpp)
# Real DMA register paths contain target-32-bit pointer conversions. Keep
# warnings visible and never execute those address-derived paths in this slice.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(p12_dma_state_tests PRIVATE
        "$<$<COMPILE_LANGUAGE:C>:-Wno-error=pointer-to-int-cast>"
        "$<$<COMPILE_LANGUAGE:C>:-Wno-error=int-to-pointer-cast>")
endif()
