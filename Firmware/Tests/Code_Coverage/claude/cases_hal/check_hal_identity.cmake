# P12 identity check (ctest hal_identity_f407vg_disco, only with FFB_TESTS_REAL_HAL).
#
# ffb_hal_tests (F407VG) and ffb_hal_tests_disco (F407VG_DISCO) run the same test files against each target's own HAL
# copy. That is only valid while both targets compile the same HAL: this script fails as soon as the two targets'
# HAL driver files, CMSIS headers or stm32f4xx_hal_conf.h differ (an added, missing or changed file).
#
#   cmake -DFW_DIR=<Firmware folder> -P check_hal_identity.cmake
if(NOT FW_DIR)
	message(FATAL_ERROR "check_hal_identity.cmake: pass -DFW_DIR=<Firmware folder>")
endif()
set(A "${FW_DIR}/Targets/F407VG")
set(B "${FW_DIR}/Targets/F407VG_DISCO")
set(failures 0)
set(compared 0)

foreach(sub Drivers/STM32F4xx_HAL_Driver Drivers/CMSIS)
	file(GLOB_RECURSE filesA RELATIVE "${A}/${sub}" "${A}/${sub}/*")
	file(GLOB_RECURSE filesB RELATIVE "${B}/${sub}" "${B}/${sub}/*")
	list(SORT filesA)
	list(SORT filesB)
	if(NOT filesA STREQUAL filesB)
		set(onlyA ${filesA})
		list(REMOVE_ITEM onlyA ${filesB})
		set(onlyB ${filesB})
		list(REMOVE_ITEM onlyB ${filesA})
		message(SEND_ERROR "${sub}: file lists differ. Only in F407VG: ${onlyA}. Only in F407VG_DISCO: ${onlyB}")
		math(EXPR failures "${failures} + 1")
	endif()
	foreach(f IN LISTS filesA)
		if(EXISTS "${B}/${sub}/${f}")
			execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files "${A}/${sub}/${f}" "${B}/${sub}/${f}" RESULT_VARIABLE rc)
			math(EXPR compared "${compared} + 1")
			if(NOT rc EQUAL 0)
				message(SEND_ERROR "${sub}/${f} differs between F407VG and F407VG_DISCO")
				math(EXPR failures "${failures} + 1")
			endif()
		endif()
	endforeach()
endforeach()

execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files "${A}/Core/Inc/stm32f4xx_hal_conf.h" "${B}/Core/Inc/stm32f4xx_hal_conf.h" RESULT_VARIABLE rc)
math(EXPR compared "${compared} + 1")
if(NOT rc EQUAL 0)
	message(SEND_ERROR "Core/Inc/stm32f4xx_hal_conf.h differs between F407VG and F407VG_DISCO")
	math(EXPR failures "${failures} + 1")
endif()

if(failures GREATER 0)
	message(FATAL_ERROR "F407VG and F407VG_DISCO no longer compile the same HAL (${failures} difference(s)); ffb_hal_tests_disco needs its own review")
endif()
message(STATUS "F407VG and F407VG_DISCO HAL identical: ${compared} files compared")
