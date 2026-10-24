/*
 * HostTarget.h
 *
 * Test control interface of the F407 part of the fake platform (ffb_target_tests only):
 * timers (fake HAL TIM), external interrupts (EXTI), GPIO pins, chip ids, reset requests and the
 * peripheral objects of the target configuration (platform/src/host_target.cpp, host_target_config.cpp),
 * the fake HAL ADC (F6), IWDG, the CAN receive FIFOs, the TIM7 HAL tick, the main.c callbacks and the start-up
 * code of cppmain.cpp (platform/src/host_target_system.cpp).
 *
 * Reset with HostTarget::reset() (done by AxisFixture / TargetFixture).
 */
#ifndef HOSTTEST_HOSTTARGET_H_
#define HOSTTEST_HOSTTARGET_H_

#include "main.h"
#include "target_constants.h"
#include "FreeRTOS.h"
#include "task.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class MockCANPort;
class FFBoardMain;

// Peripheral handles of the target (normally defined by the CubeMX code in main.c)
extern TIM_HandleTypeDef htim3;	 // TIM_ENC: local quadrature encoder
extern TIM_HandleTypeDef htim13; // TIM_FFB: FFB update rate timer
extern ADC_HandleTypeDef hadc1; // AIN_HADC: analog inputs (ADC1_BUF[0..5]), chip temperature [6], VREFINT [7]
extern ADC_HandleTypeDef hadc2; // VSENSE_HADC: supply voltage VEXT (ADC2_BUF[0]) and internal voltage VINT [1]
extern volatile uint32_t ADC1_BUF[ADC1_CHANNELS]; // DMA buffers, defined by global_callbacks.cpp
extern volatile uint32_t ADC2_BUF[ADC2_CHANNELS];

// cppmain.cpp defines malloc() / free(); they are renamed for the host (platform/src/host_cppmain.cpp) and reachable as these
void *hosttest_cppmain_malloc(size_t size);
void hosttest_cppmain_free(void *p);

namespace HostTarget {

struct State {
	// --- Recorded ---
	uint32_t systemResets = 0;		  //!< NVIC_SystemReset calls, including those of RebootDFU / ExitDFU (the fake returns, the target would reset)
	uint32_t dfuReboots = 0;		  //!< RebootDFU calls (each also counts as a system reset, like main.c)
	uint32_t dfuExits = 0;			  //!< ExitDFU calls (each also counts as a system reset, like main.c)
	uint32_t usbInits = 0;			  //!< tusb_rhport_init calls (tusb_init)
	uint32_t usbEventsProcessed = 0;  //!< tud_task_ext calls that returned after consuming a queued USB event
	uint32_t halIncTicks = 0;		  //!< HAL_IncTick calls (TIM7 update interrupts through main.c's HAL_TIM_PeriodElapsedCallback)
	uint32_t watchdogRefreshes = 0;	  //!< HAL_IWDG_Refresh calls (refreshWatchdog of cppmain.cpp)
	uint32_t errorHandlerCalls = 0;	  //!< Error_Handler calls (main.c: error LED off, returns)
	uint32_t taskYields = 0;		  //!< taskYIELD() calls (cppmain loop, Thread::Yield)
	uint32_t canRxOverruns[2] = {0, 0}; //!< Frames that overwrote the newest frame of a full CAN receive FIFO (canQueueRx)
	uint32_t canRxStalls = 0;		  //!< canRxInterrupt stopped because a callback did not read the FIFO (the target would re-enter the interrupt forever)
	std::vector<std::string> unmodelledHalCalls; //!< HAL functions without a fake that were called (see host_stubs_target.cpp), triggerExti on a pin without EXTI line, and HAL calls the fake can not complete (e.g. an endless wait)

	// --- Controlled by the test ---
	uint32_t uid[3] = {0x00410032, 0x34385111, 0x30383632}; //!< HAL_GetUIDw0/1/2 (96 bit unique id)
	uint32_t devId = 0x413;									//!< HAL_GetDEVID: STM32F405/407
	uint32_t revId = 0x1007;								//!< HAL_GetREVID: revision 1
	uint8_t usbSpeed = 0; //!< tud_speed_get() result (tusb_speed_t). 0 = TUSB_SPEED_FULL
	uint32_t errorHandlerLimit = 0; //!< > 0: the Error_Handler call with this number throws ErrorHandlerLimitReached (ends an endless Error_Handler loop, e.g. cppmain's wrong chip id loop). 0 = Error_Handler always returns
};

/** Thrown by Error_Handler when state().errorHandlerLimit calls are reached (see State::errorHandlerLimit) */
struct ErrorHandlerLimitReached {};
State &state();

/**
 * Restores the state after the CubeMX initialisation (MX_GPIO_Init, MX_TIMx_Init in main.c): timers initialised
 * and stopped (HAL_TIM_Base_Init: State READY, ARR = Period, PSC = Prescaler, CNT 0), inputs with pull-up high
 * (DIN0-7, E_STOP released), all other inputs low, outputs at their initial level (SPI1_SS1 set by MX_GPIO_Init;
 * LED_SYS, LED_CLIP and LED_ERR set by main() before the scheduler starts), chip ids default.
 * Also resets the target configuration (resetTargetConfig) and the system part (resetSystem).
 */
void reset();

/**
 * Resets the objects of the host target configuration (host_target_config.cpp), called by reset():
 *  - the global `canport` (MockCANPort::resetForTest: records, speed, start/stop, port users, semaphores)
 *  - the HAL handles hspi1-3, hi2c1, huart3, hadc1/2 (Init and State as after the CubeMX init)
 *  - the transfer semaphores of motor_uart
 * NOT reset (private firmware state, no public reset): the SPI ports external_spi / motor_spi / ext3_spi
 * (freePins, current_device, takenExclusive, isTakenFlag), i2cport (port users, config) and the device pointer of
 * motor_uart. A test that takes a CS pin or a port must release it again (freeCsPin / freePort). To be replaced by
 * a full reset when the fake HAL SPI (F4) and I2C (F5) arrive in P5/P6.
 */
void resetTargetConfig();

/**
 * Resets the system part (host_target_system.cpp), called by reset():
 *  - fake ADC: DMA transfers stopped, inputs 0, calibration values default (setAdcCalibration); ADC1_BUF / ADC2_BUF 0
 *  - TIM7 (HAL tick) as after HAL_InitTick: 1 MHz, Period 999, running with update interrupt, CNT = time % 1000
 *  - IWDG handle, CAN receive FIFOs empty
 *  - firmware globals to their initial values: voltagesense.cpp (brake thresholds and state, vSenseMult, chip
 *    temperature average), ledEffects.cpp (sysled, errled, clipled), cppmain.cpp (running, mainclassChosen,
 *    forceErase, main_id) and the cppmain loop budget (runCppmain)
 * Not reset here: `mainclass` / `usb_device` (TargetFixture), a main class created by runCppmain (deleteCppmainMainclass).
 */
void resetSystem();

// --- Timers ---
/**
 * Simulates an update event (counter overflow) of a timer. Models HAL_TIM_IRQHandler for the update
 * interrupt: only if the counter is enabled (CR1.CEN) and the update interrupt is enabled (DIER.UIE)
 * HAL_TIM_PeriodElapsedCallback(htim) is called in interrupt context.
 * @return true if the callback was called
 */
bool timerUpdate(TIM_HandleTypeDef *htim);
/** True while the counter is enabled (CR1.CEN) */
bool timerRunning(const TIM_HandleTypeDef *htim);
/** True while the update interrupt is enabled (DIER.UIE) */
bool timerUpdateIrqEnabled(const TIM_HandleTypeDef *htim);

// --- GPIO and external interrupts ---
/** Sets the input level of a pin (IDR) */
void setPin(GPIO_TypeDef *port, uint16_t pin, bool high);
/** Output level of a pin (ODR) as written with HAL_GPIO_WritePin / TogglePin */
bool outputPin(const GPIO_TypeDef *port, uint16_t pin);
/**
 * Simulates an EXTI interrupt on a pin. Models HAL_GPIO_EXTI_IRQHandler: calls HAL_GPIO_EXTI_Callback(pin)
 * in interrupt context. Set the new pin level with setPin before.
 * The interrupt is raised regardless of the edge and of the pin level, like a contact bounce or glitch would.
 * On the target the lines are (MX_GPIO_Init): BUTTON_A rising, FLAG rising, E_STOP both edges, ENCODER_Z falling.
 * A test that models a real edge must set the level accordingly. Pins without EXTI line are rejected: no callback,
 * recorded in unmodelledHalCalls (the fixture check fails).
 */
void triggerExti(uint16_t pin);

// --- USB device task ---
/**
 * Queues one USB event for the TinyUSB task (USBdevice thread): notifies the task. The next tud_task_ext call of
 * that task returns once (usbEventsProcessed + 1) instead of blocking. Run it with HostRtos::runUntilBlocked.
 * Note: the host RTOS restarts USBdevice::Run from the start, so tusb_init (usbInits) is called on every activation.
 */
void queueUsbEvent(TaskHandle_t usbTask);

// --- ADC (fake HAL ADC F6, host_target_system.cpp) ---
/**
 * Sets the analog input of a rank of the regular sequence (0 = first conversion = first DMA buffer word). Persistent
 * like a voltage on the pin; converted at the next conversion (adcConversionComplete / adcSetConversionResult).
 * Values above the full scale of the resolution (4095 for 12 bit) are converted as full scale (input above Vref+).
 */
void adcSetInput(ADC_HandleTypeDef *hadc, uint32_t rank, uint32_t value);
/** Sets the inputs of ranks 0..values.size()-1 (see adcSetInput) */
void adcSetInputs(ADC_HandleTypeDef *hadc, const std::vector<uint32_t> &values);
/**
 * Simulates the end of a regular sequence converted with DMA (circular DMA of the F407 configuration): the DMA writes
 * the converted inputs of all ranks (Length of HAL_ADC_Start_DMA) into the buffer armed by HAL_ADC_Start_DMA, DR holds
 * the last one, then ADC_DMAConvCplt (stm32f4xx_hal_adc.c) runs in interrupt context: State |= REG_EOC and
 * HAL_ADC_ConvCpltCallback(hadc) (global_callbacks.cpp). The DMA keeps running (circular).
 * Nothing happens if the ADC is not running with DMA (HAL_ADC_Start_DMA not called, or stopped).
 * @return true if the callback was called
 */
bool adcConversionComplete(ADC_HandleTypeDef *hadc);
/** adcSetInputs(hadc, values) followed by adcConversionComplete(hadc) */
bool adcConvert(ADC_HandleTypeDef *hadc, const std::vector<uint32_t> &values);
/** True while a DMA transfer started by HAL_ADC_Start_DMA is running */
bool adcDmaRunning(const ADC_HandleTypeDef *hadc);
/** Buffer and length of the running DMA transfer (nullptr / 0 if none) */
uint32_t *adcDmaBuffer(const ADC_HandleTypeDef *hadc);
uint32_t adcDmaLength(const ADC_HandleTypeDef *hadc);
/**
 * Single conversion without DMA (for HAL_ADC_PollForConversion / HAL_ADC_GetValue): if the ADC is enabled (ADON),
 * DR = value and the EOC and STRT flags are set. @return false if the ADC is disabled (no conversion)
 */
bool adcSetConversionResult(ADC_HandleTypeDef *hadc, uint32_t value);
/**
 * Factory calibration values read by the LL ADC macros (VREFINT_CAL, TS_CAL1 at 30 C, TS_CAL2 at 110 C; 12 bit at
 * 3.3 V). Defaults after reset: 1500, 950, 1200 (example values in the range of the F407 datasheet, not of a real chip).
 */
void setAdcCalibration(uint16_t vrefintCal, uint16_t tsCal1, uint16_t tsCal2);

// --- HAL tick (TIM7, TIM_MICROS_HALTICK) ---
/**
 * HAL_GetTick() is the simulated time in ms; the TIM7 counter (read by the real micros() of cppmain.cpp) is kept at
 * the microseconds since the last tick (time % 1000) whenever the simulated time changes. A TIM7 update interrupt
 * can be raised with timerUpdate(&htim7): main.c's HAL_TIM_PeriodElapsedCallback calls HAL_IncTick (counted in
 * state().halIncTicks, the time does not change).
 */
uint32_t tickTimerCounter();

// --- CAN receive FIFOs of hcan1 (fake HAL CAN, receive path only; the rest of F8 follows in P7) ---
/**
 * Puts a received frame into receive FIFO 0 or 1 of hcan1 (3 frames deep). The acceptance filters are not modelled:
 * the frame counts as accepted (FilterMatchIndex as given). A full FIFO (not locked, MX_CAN1_Init
 * ReceiveFifoLocked = DISABLE) overwrites its newest frame (RM0090 32.7.5, counted in state().canRxOverruns).
 * Fields are stored like the mailbox registers: StdId 11 bit, ExtId 29 bit, DLC 4 bit, Timestamp 16 bit.
 */
void canQueueRx(uint32_t fifo, const CAN_RxHeaderTypeDef &header, const uint8_t data[8]);
/** Frames waiting in a receive FIFO of hcan1 */
size_t canRxPending(uint32_t fifo);
/**
 * Raises the "message pending" interrupt of a receive FIFO of hcan1 in interrupt context (HAL_CAN_IRQHandler:
 * HAL_CAN_RxFifo0MsgPendingCallback / ..Fifo1..). The interrupt is level triggered: it is raised again while frames
 * are pending. Stops (state().canRxStalls) if a callback reads nothing. The notifications are assumed active (like
 * CANPort_2B::start). @return number of callback calls
 */
uint32_t canRxInterrupt(uint32_t fifo);

// --- Start-up code (cppmain.cpp) ---
/**
 * Runs the real cppmain() like the default task of main.c: chip id check, flash init and version check, startADC,
 * main class selection (BUTTON_A pressed = failsafe class 0, else ADR_CURRENT_CONFIG, default FFBWheel), delayed
 * flash restore, usbInit, then `loops` passes of the main loop (mainclass->update(), updateLeds(), refreshWatchdog(),
 * taskYIELD()). The loop ends after the loops-th taskYIELD (the hook sets cppmain's `running` to false); loops = 0
 * skips the loop.
 * The created main class is owned by the fake: TargetFixture deletes it at the end (deleteCppmainMainclass). Deleting
 * an FFBHIDMain class (FFBWheel, FFBJoystick, FFBHIDExt) is undefined behaviour (bug 24): such tests need
 * DELETES_FFBHIDMAIN. Select the failsafe class (setPin(BUTTON_A..., true) or flash ADR_CURRENT_CONFIG = 0) otherwise.
 * With a wrong chip id (state().devId != FW_DEVID) cppmain() loops forever calling Error_Handler (as on the target):
 * set state().errorHandlerLimit before, the limit-th Error_Handler call throws ErrorHandlerLimitReached out of
 * runCppmain (nothing is created at that point).
 */
void runCppmain(uint32_t loops);
/** Main class created by the last runCppmain (nullptr if none or already deleted) */
FFBoardMain *cppmainMainclass();
/** Deletes the main class created by runCppmain, if any, and clears `mainclass` if it pointed to it */
void deleteCppmainMainclass();

// --- Target configuration (host replacement of cpp_target_config.cpp) ---
/**
 * The global `canport` of the firmware. A MockCANPort until the fake HAL CAN exists (P7). Its handle is &hcan1 like
 * CANPort_2B, so CANPort::handleToPort(&hcan1) and the CAN callbacks of global_callbacks.cpp find it.
 */
MockCANPort &canPort();

} // namespace HostTarget

#endif
