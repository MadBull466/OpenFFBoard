/*
 * target_constants.h
 *
 *  Created on: 06.11.2020
 *      Author: Yannick
 *
 * Host test copy of Targets/F407VG/Core/Inc/target_constants.h for ffb_driver_tests (driver track,
 * Tests/claude/plans/DRIVER_TEST_PLAN.md). It is platform/target_f407/target_constants.h (the F407VG header without
 * the drivers and MIDI) with exactly eight feature macros added back:
 *   TMC4671DRIVER  (F407VG target_constants.h:46)
 *   ODRIVE         (F407VG target_constants.h:50)
 *   VESC           (F407VG target_constants.h:51)
 *   TMCDEBUG       (P13, TMCDebugBridge in scope since 2026-11-10; Tests/claude/plans/P13_TMCDEBUGBRIDGE_PLAN.md).
 *                  On in ffb_driver_tests only: the other executables keep their own target_constants.h.
 *   PWMDRIVER      (P14, MotorPWM in scope since 2026-11-10; F407VG target_constants.h:47; Tests/claude/plans/P14_P17_REMAINING_DRIVERS_PLAN.md).
 *                  On in ffb_driver_tests only. TIM_PWM htim1 is the fake TIM1 of platform/src/host_driver_timers.cpp.
 *   SIMPLEMOTION   (P15, MotorSimplemotion in scope since 2026-11-10; F407VG target_constants.h:49; Tests/claude/plans/P14_P17_REMAINING_DRIVERS_PLAN.md).
 *                  On in ffb_driver_tests only. gpMotor is defined in platform/src/host_driver_timers.cpp.
 *   RMDCAN        (P16, RmdMotorCAN in scope since 2026-11-10; F407VG target_constants.h:48; Tests/claude/plans/P14_P17_REMAINING_DRIVERS_PLAN.md).
 *                  On in ffb_driver_tests only.
 * MIDI stays off until its phase (P17). HSPIDRV (hspi1), CANBUS and
 * TIM_TMC htim6 are as in the F407VG header (FFBoard/Inc/constants.h:23-27, 52-54 drop the drivers without them).
 * htim6 is declared extern below (the real main.h declares it; the host main.h only declares htim7).
 */

#ifndef INC_TARGET_CONSTANTS_H_
#define INC_TARGET_CONSTANTS_H_

/*
 * Add settings and peripheral maps for this specific target here
 */

// Hardware name string
#define HW_TYPE "F407VG"
#define HW_TYPE_INT 2
#define FW_DEVID 0x413 // F407

#include "main.h"

// Enabled features
#define DEFAULTMAIN 1 // FFBWheel
// Main classes
#define FFBWHEEL
#define FFBJOYSTICK
#define CANBRIDGE
#define FFBHIDEXT
#define CANINPUTMAIN

// Motor drivers of the driver track (the only differences to platform/target_f407/target_constants.h)
#define TMC4671DRIVER // F407VG target_constants.h:46
#define ODRIVE // F407VG target_constants.h:50
#define VESC // F407VG target_constants.h:51
extern TIM_HandleTypeDef htim1; // defined by platform/src/host_driver_timers.cpp (fake TIM1, TIM_PWM)
#define PWMDRIVER // P14: MotorPWM (adds MotorPWM to MotorDriver.cpp / Axis.cpp driver lists as id 4)
#ifndef HOSTTEST_NO_SIMPLEMOTION // set by CMake with clang: MotorSimplemotion.h uses GCC-only variable length array initialisers
#define SIMPLEMOTION // P15: MotorSimplemotion (adds ids 9 and 10 to MotorDriver.cpp / Axis.cpp driver lists, GPIO_MOTOR via constants.h; fake motor UART huart3 with FakeSmDevice)
#endif
#define RMDCAN // P16: RmdMotorCAN (adds ids 11 and 12 to MotorDriver.cpp / Axis.cpp driver lists; CAN port canport with the fake HAL CAN, FakeRmdPeer)
#define TMCDEBUG // P13: TMCDebugBridge (adds TMCDebugBridge to mainclass_chooser.cpp and `friend class TMCDebugBridge` to TMC4671.h)

/*
 * FFBWheel uses 2 FFB axis descriptor instead of 1 axis.
 * Might improve compatibility with direct input but will report a 2 axis ffb compatible device
 */
//#define FFBWHEEL_USE_1AXIS_DESC

// Extra features
#define LOCALBUTTONS
#define SPIBUTTONS
#define SHIFTERBUTTONS
#define PCF8574BUTTONS // Requires I2C
#define ANALOGAXES
#define LOCALENCODER
#define CANBUS
#define MTENCODERSPI // requires SPI3
#define CANBUTTONS // Requires CAN
#define CANANALOG // Requires CAN
#define BISSENCODER // Requires SPI3
#define SSIENCODER // Requires SPI3
#define ADS111XANALOG // Requires I2C
#define UARTCOMMANDS
#define CANCOMMANDS

//----------------------
#define BTNFAILSAFE // Use user button to force board into failsafe mainclass

#define TIM_ENC htim3
// Timer 3 is used by the encoder.
#define TIM_PWM htim1

#define TIM_MICROS_HALTICK htim7 // Micros timer MUST be reset by hal tick timer or isr to count microseconds since last tick
#define TIM_USER htim9 // Timer with full core clock speed available for the mainclass
extern TIM_HandleTypeDef htim6; // defined by platform/src/host_driver_timers.cpp
#define TIM_TMC htim6 // Timer running at half clock speed
#define TIM_TMC_BCLK SystemCoreClock / 2
#define TIM_TMC_ARR 250 // 4khz
#define TIM_FFB htim13

extern UART_HandleTypeDef huart1;
#define UART_PORT_EXT huart1 // main uart port

extern UART_HandleTypeDef huart3;
#define UART_PORT_MOTOR huart3 // motor uart port

#define UART_BUF_SIZE 1 // How many bytes to expect via DMA

extern I2C_HandleTypeDef hi2c1;
#define I2C_PORT hi2c1



// ADC Channels
#define ADC1_CHANNELS 8 	// how many analog input values to be read by dma
#define ADC2_CHANNELS 2		// VSENSE

extern ADC_HandleTypeDef hadc2;
#define VSENSE_HADC hadc2
#define ADC_CHAN_VINT 1	// adc buffer index of internal voltage sense
#define ADC_CHAN_VEXT 0 // adc buffer index of supply voltage sense
extern volatile uint32_t ADC2_BUF[ADC2_CHANNELS]; // Buffer
#define VSENSE_ADC_BUF ADC2_BUF

extern volatile uint32_t ADC1_BUF[ADC1_CHANNELS]; // Buffer
#define TEMPSENSOR_ADC_VAL ADC1_BUF[6] // ADC1 ch 7
#define ADC_INTREF_VAL ADC1_BUF[7] // ADC1 ch 8.

extern ADC_HandleTypeDef hadc1;
#define AIN_HADC hadc1	// main adc for analog pins
#define ADC_PINS 6	// Amount of analog channel pins
#define ADC_CHAN_FPIN 0 // First analog channel pin. last channel = fpin+ADC_PINS-1
//#define VOLTAGE_MULT_DEFAULT 30.12 // mV adc * scaler = voltage

#define BUTTON_PINS 8

extern SPI_HandleTypeDef hspi1;
#define HSPIDRV hspi1
extern SPI_HandleTypeDef hspi2;
#define HSPI2 hspi2
extern SPI_HandleTypeDef hspi3;
#define EXT3_SPI_PORT hspi3

// CAN
#ifdef CANBUS
extern CAN_HandleTypeDef hcan1;
#define CANPORT hcan1
#endif

#define DEBUGPIN // GP1 pin. see cpp target constants

//Flash. 2 pages used
/* EEPROM start address in Flash
 * PAGE_ID sectors 1 and 2!
 * */
#define USE_EEPROM_EMULATION
#define PAGE0_ID               FLASH_SECTOR_1
#define PAGE1_ID               FLASH_SECTOR_2
#define EEPROM_START_ADDRESS  ((uint32_t)0x08004000) /* EEPROM emulation start address: from sector1*/
#define PAGE_SIZE             (uint32_t)0x4000  /* Page size = 16KByte */


// System
// BKPSRAM positions
#define DFU_JUMP_MAGIC_ADR BKPSRAM_BASE + 0

#define CCRAM_SEC ".ccmram"

#define SIGBNATUREBASEADR 0 // First block in OTP

#endif /* INC_TARGET_CONSTANTS_H_ */
