/*
 * HostTarget.h
 *
 * Test control interface of the F407 part of the fake platform (ffb_target_tests only):
 * timers (fake HAL TIM), external interrupts (EXTI), GPIO pins, chip ids, reset requests and the
 * peripheral objects of the target configuration (platform/src/host_target.cpp, host_target_config.cpp),
 * the fake HAL ADC (F6), IWDG, the CAN receive FIFOs, the TIM7 HAL tick, the main.c callbacks and the start-up
 * code of cppmain.cpp (platform/src/host_target_system.cpp), the fake HAL SPI (F4) and I2C (F5)
 * (platform/src/host_target_bus.cpp), the fake HAL CAN (F8, platform/src/host_target_can.cpp).
 *
 * Reset with HostTarget::reset() (done by AxisFixture / TargetFixture).
 */
#ifndef HOSTTEST_HOSTTARGET_H_
#define HOSTTEST_HOSTTARGET_H_

#include "main.h"
#include "target_constants.h"
#include "FreeRTOS.h"
#include "task.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

class MockCANPort;
class CANPort_2B;
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
	uint32_t canRxStalls = 0;		  //!< canRxInterrupt or a raised CAN interrupt (fake HAL CAN) stopped because the handler changed nothing, e.g. a callback did not read the FIFO (the target would re-enter the interrupt forever)
	struct OtpCall {
		bool write = false;	  //!< OTP_Write (true) or OTP_Read (false)
		uint16_t offset = 0;  //!< adroffset argument (8 byte slots)
		uint64_t value = 0;	  //!< Written value, or the value read (only if result is true)
		bool result = false;  //!< Return value
	};
	std::vector<OtpCall> otpCalls;	//!< OTP_Write / OTP_Read calls in order
	uint32_t otpPrograms = 0;		//!< 32 bit programming steps of OTP_Write (HAL_FLASH_Program, FLASH_TYPEPROGRAM_WORD), including failed ones
	std::vector<std::string> unmodelledHalCalls; //!< HAL calls the fakes do not model (e.g. SPI slave mode or I2C 10 bit addressing, see host_target_bus.cpp), triggerExti on a pin without EXTI line, and HAL calls the fake can not complete (e.g. an endless wait)

	// --- Controlled by the test ---
	uint32_t uid[3] = {0x00410032, 0x34385111, 0x30383632}; //!< HAL_GetUIDw0/1/2 (96 bit unique id)
	uint32_t devId = 0x413;									//!< HAL_GetDEVID: STM32F405/407
	uint32_t revId = 0x1007;								//!< HAL_GetREVID: revision 1
	uint8_t usbSpeed = 0; //!< tud_speed_get() result (tusb_speed_t). 0 = TUSB_SPEED_FULL
	uint32_t otpFailProgram = 0; //!< > 0: the OTP programming step with this number (counted in otpPrograms) fails like HAL_FLASH_Program returning HAL_ERROR (nothing programmed). 0 = no failure
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
 *  - the fake HAL CAN (resetCan: CAN1 and hcan1 as after MX_CAN1_Init, no frames, no records)
 *  - the global `canport` (the real CANPort_2B) to its state after construction: speed preset 4 (500k), no filters, no
 *    transmit mailboxes, silent, not active, lastSentTime 0, nextLen 8, no port users, not waiting, semaphores
 *    available, delayed flash restore pending; its view canPort() (MockCANPort::resetForTest: records, start/stop
 *    counters, filters)
 *  - the HAL handles hspi1-3, hi2c1, huart3, hadc1/2 (Init, State, ErrorCode, Lock as after MX_SPIx_Init /
 *    MX_I2C1_Init / MX_ADCx_Init of main.c) and the fake HAL SPI / I2C (resetBus: scripts, devices, records)
 *  - the transfer semaphores of motor_uart
 *  - the SPI ports external_spi / motor_spi / ext3_spi to their state after construction: all CS pins free (pin list
 *    as constructed), no current device, not taken exclusively, semaphore available
 *  - i2cport to its state after construction: no port users, no current device, speed preset 0, semaphore available,
 *    delayed flash restore pending (restoreFlashDelayed in the constructor)
 *  - the SPI and I2C handler lists (SpiHandler::getSPIHandlers, I2CHandler::getI2CHandlers) to the handlers registered
 *    during the static initialisation (the three SPI ports, i2cport). Needed because ~I2CHandler does not unregister
 *    (I2CHandler.cpp: it calls addCallbackHandler; suspected bug), so a destroyed test handler would stay in the list
 *  - the static "in use" flags of the encoders EncoderLocal, EncoderSSI, EncoderBissC, MtEncoderSPI (false)
 * The private members of SPIPort / I2CPort are reset through member pointers obtained by explicit template
 * instantiation (host_target_config.cpp); tests never access them.
 * NOT reset: the device pointer of motor_uart (UARTPort, out of scope until P7). Firmware objects that still use a port
 * when the fixture resets (a device a test did not delete) keep dangling state: delete devices inside the test.
 */
void resetTargetConfig();

/**
 * Resets the fake HAL SPI and I2C (host_target_bus.cpp), called by resetTargetConfig(): no pending transfers, no
 * records, no scripted responses or injected failures, CS pins of hspi1-3 as in cpp_target_config.cpp, no I2C devices,
 * bus idle, hi2c1 without DMA (like stm32f4xx_hal_msp.c).
 */
void resetBus();

/**
 * Resets the system part (host_target_system.cpp), called by reset():
 *  - fake ADC: DMA transfers stopped, inputs 0, calibration values default (setAdcCalibration); ADC1_BUF / ADC2_BUF 0
 *  - TIM7 (HAL tick) as after HAL_InitTick: 1 MHz, Period 999, running with update interrupt, CNT = time % 1000
 *  - IWDG handle (the CAN receive FIFOs are reset by resetTargetConfig -> resetCan)
 *  - firmware globals to their initial values: voltagesense.cpp (brake thresholds and state, vSenseMult, chip
 *    temperature average), ledEffects.cpp (sysled, errled, clipled), cppmain.cpp (running, mainclassChosen,
 *    forceErase, main_id) and the cppmain loop budget (runCppmain)
 * Not reset here: `mainclass` / `usb_device` (TargetFixture), a main class created by runCppmain (deleteCppmainMainclass).
 * Note: on the target `mainclass` lives in the NOLOAD .ccmram section (STM32F407VGTX_FLASH.ld:150) and is not zeroed
 * by the startup code (only .bss is); on the host it starts as nullptr and TargetFixture sets it to nullptr.
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

// --- Quadrature encoder on a timer in encoder interface mode (TIM_ENC = htim3, P6) ---
/**
 * Moves the counter of a timer in encoder interface mode by `counts` (> 0 up, < 0 down), like quadrature edges on
 * TI1/TI2 (MX_TIM3_Init: TIM_ENCODERMODE_TI12, every edge of both inputs counts: 4 counts per encoder line).
 * Models RM0090 "General-purpose timers (TIM2 to TIM5)", encoder interface mode: the counter only counts while it is
 * enabled (CR1.CEN); counting up from ARR wraps to 0 and counting down from 0 wraps to ARR, each wrap is an update
 * event (overflow / underflow): SR.UIF is set and, if the update interrupt is enabled (DIER.UIE), HAL_TIM_IRQHandler's
 * update branch (stm32f4xx_hal_tim.c) runs in interrupt context: UIF cleared, HAL_TIM_PeriodElapsedCallback(htim)
 * (main.c -> HAL_TIM_PeriodElapsedCallback_CPP -> TimerHandler::timerElapsed). The interrupt is raised right at the
 * wrap (CNT = 0 after an overflow, ARR after an underflow), i.e. with no interrupt latency, before the counter moves on.
 * MX_TIM3_Init configures centre-aligned mode 1 (CMS) together with encoder mode; EncoderLocal's constructor clears it
 * with CR1 = 1. Only the encoder mode behaviour is modelled: one update event per overflow / underflow (RM0090).
 * Not modelled (recorded in state().unmodelledHalCalls, nothing counted): a counter above ARR (16 bit counter of TIM3
 * counting beyond ARR), a pending UIF raising the interrupt when UIE is enabled later.
 * @return number of update events (wraps)
 */
uint32_t encoderTimerCount(TIM_HandleTypeDef *htim, int32_t counts);
/** Counter register (CNT) */
uint32_t timerCounter(const TIM_HandleTypeDef *htim);
/** Writes the counter register directly (no update event), like a debugger or a write by the firmware */
void timerSetCounter(TIM_HandleTypeDef *htim, uint32_t count);
/**
 * Index pulse of the local encoder (ENCODER_Z, MX_GPIO_Init: EXTI falling edge, no pull): the pin goes high (idle),
 * then low (the falling edge raises the EXTI interrupt while the pin reads low: HAL_GPIO_EXTI_Callback(ENCODER_Z_Pin)
 * via triggerExti), then high again (end of the pulse; the rising edge raises no interrupt).
 */
void encoderIndexPulse();

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

// --- bxCAN CAN1 / hcan1 (fake HAL CAN F8, host_target_can.cpp) ---
/**
 * The fake models stm32f4xx_hal_can.c (every HAL CAN function the firmware calls, plus Init/DeInit, sleep/wake-up,
 * DeactivateNotification, GetState/GetError) on the CAN1 registers (`*CAN1`, readable by tests: BTR, IER, TSR, ESR,
 * FA1R, sFilterRegister ...). The peripheral side follows RM0090 chapter 32:
 *  - After reset (resetCan) CAN1 is as after MX_CAN1_Init (main.c:486-511, HAL_CAN_Init): State READY, initialisation
 *    mode, BTR 0x001A0005 (500 kbit/s), automatic retransmission, no automatic bus-off recovery, FIFOs not locked.
 *    Mode changes (start, stop, init) are acknowledged at once; canFailNextModeChange injects the HAL timeout.
 *  - Transmit: HAL_CAN_AddTxMessage puts the frame into the lowest empty of the 3 mailboxes (recorded in canTxFrames)
 *    and it stays pending (a bus without another node never acknowledges) until the test completes it (canTxComplete,
 *    canTxCompleteAll, canSetTxAutoComplete) or reports a failed attempt (canTxError). Only in normal mode (started,
 *    not sleeping, not bus-off). HAL_CAN_AbortTxRequest empties a pending mailbox at once.
 *    Host-vs-target: the fake aborts a mailbox at once even if its frame is being transmitted; on the target (nothing
 *    acknowledges, automatic retransmission) a mailbox in the middle of an attempt is aborted only after that attempt.
 *  - Receive: canReceive passes a frame through the filter banks into the FIFO of the matching filter (3 frames deep).
 *  - Interrupts: HAL_CAN_IRQHandler (modelled step by step) runs in interrupt context while an event is pending whose
 *    interrupt is enabled in IER and whose NVIC line is enabled. As in HAL_CAN_MspInit (stm32f4xx_hal_msp.c:291-294)
 *    only CAN1_TX and CAN1_RX0 are enabled: FIFO 1 frames and the status change / error events (bus-off, error
 *    warning / passive, last error code) raise no interrupt of their own on the F407 (canSetIrqLineEnabled to model
 *    other boards). HAL_CAN_IRQHandler services every pending flag whichever line raised it, so they are handled at the
 *    next CAN1_TX or CAN1_RX0 interrupt (a transmit completion / abort, a FIFO 0 frame), like on the target.
 *    Transmit, abort and FIFO 0 interrupts are raised right after the event (inside the fake call that caused it).
 * Not modelled: bit timing / bus speed, time-triggered mode, the error counters TEC/REC, automatic bus-off recovery
 * (ABOM) and wake-up (AWUM), loop back / silent mode (BTR LBKM / SILM: recorded in state().unmodelledHalCalls at
 * HAL_CAN_Start), a frame that is being sent while it is aborted, the ownership of the filter banks by CAN1 / CAN2
 * (FMR CAN2SB is written like the HAL, all 28 banks filter CAN1), CAN2. HAL calls with another handle than hcan1 and
 * filter banks above 27 are recorded as unmodelled.
 */
/** Resets the fake HAL CAN: CAN1 / hcan1 as after MX_CAN1_Init, no frames, no records, NVIC lines as HAL_CAN_MspInit, no injections. Called by resetTargetConfig */
void resetCan();

enum class CanTxResult : uint8_t { Pending, Sent, Aborted, Failed };
/** A frame put into a transmit mailbox by HAL_CAN_AddTxMessage. id / extId / rtr / dlc / data are decoded from the mailbox registers (what goes on the bus) */
struct CanTxFrame {
	CAN_TxHeaderTypeDef header{};	//!< Header as passed to HAL_CAN_AddTxMessage
	uint32_t mailbox = 0;			//!< 0..2 (*pTxMailbox = 1 << mailbox)
	uint32_t id = 0;				//!< Identifier in the mailbox (11 bit standard / 29 bit extended; the HAL does not mask StdId / ExtId: higher bits are lost)
	bool extId = false;				//!< IDE
	bool rtr = false;				//!< Remote frame
	uint8_t dlc = 0;				//!< DLC field (4 bit, the HAL writes header.DLC unmasked; 9..15 means 8 data bytes on the bus)
	std::array<uint8_t, 8> data{};	//!< The 8 data bytes of the mailbox (the HAL always copies 8)
	uint32_t time = 0;				//!< HAL_GetTick() at HAL_CAN_AddTxMessage
	CanTxResult result = CanTxResult::Pending;
	/** The first min(dlc, 8) data bytes */
	std::vector<uint8_t> payload() const { return std::vector<uint8_t>(data.begin(), data.begin() + (dlc > 8 ? 8 : dlc)); }
	/** The 8 data bytes as little endian 64 bit value (byte 0 = bits 7:0) */
	uint64_t value() const {
		uint64_t v = 0;
		for (int i = 7; i >= 0; i--) {
			v = (v << 8) | data[i];
		}
		return v;
	}
};
/** Every frame accepted by HAL_CAN_AddTxMessage since the reset, in order (also aborted ones, see result) */
const std::vector<CanTxFrame> &canTxFrames();
void canClearTxFrames();
/** Mailboxes with a pending transmit request: bit m = mailbox m (TME clear) */
uint32_t canTxPending();
/**
 * The pending frame of a mailbox (0..2) was sent and acknowledged: TSR RQCP, TXOK, TME set, then the transmit
 * interrupt (if TMEIE and the NVIC line): HAL_CAN_TxMailboxXCompleteCallback (global_callbacks.cpp -> every
 * CanHandler::canTxCpltCallback with mailbox index X). @return false if the mailbox is not pending or the peripheral
 * can not transmit (not started, sleeping, bus-off)
 */
bool canTxComplete(uint32_t mailbox);
/** canTxComplete for every pending mailbox (0, 1, 2), one interrupt for all. @return number of completed frames */
uint32_t canTxCompleteAll();
/**
 * A transmission attempt of the pending frame failed (TERR, or ALST with arbitrationLost). With automatic
 * retransmission (MX_CAN1_Init) the request stays pending and no interrupt is raised; an abort then reports the error
 * (HAL_CAN_ErrorCallback with HAL_CAN_ERROR_TX_TERRx / ALSTx instead of the abort callback). With MCR NART set the
 * request completes with that error. @return false like canTxComplete
 */
bool canTxError(uint32_t mailbox, bool arbitrationLost = false);
/**
 * Auto-complete: another node acknowledges every frame. Pending frames are completed (canTxCompleteAll) at the start
 * of the next HAL CAN call and of canIrq, i.e. some time after the firmware queued them, never inside the call that
 * queued them. Off after reset.
 */
void canSetTxAutoComplete(bool on);

/** Error state of the node (RM0090 32.7.6): sets CAN_ESR EWGF / EPVF / BOFF (cumulative), ERRI if the matching interrupt (CAN_IT_ERROR_WARNING / _PASSIVE / _BUSOFF) is enabled, then raises the status change interrupt (CAN1_SCE: disabled in the F407 NVIC). Bus-off stops transmission and reception until the peripheral leaves the initialisation mode again (HAL_CAN_Stop + HAL_CAN_Start, recovery assumed at once) */
enum class CanErrorState : uint8_t { Active, Warning, Passive, BusOff };
void canSetErrorState(CanErrorState errorState);
/** Last error code CAN_ESR LEC (1 stuff, 2 form, 3 ack, 4 bit recessive, 5 bit dominant, 6 CRC), ERRI if CAN_IT_LAST_ERROR_CODE is enabled, raises CAN1_SCE */
void canSetLastErrorCode(uint8_t lec);
/** NVIC lines of CAN1. After reset: Tx and Rx0 enabled, Rx1 and Sce disabled (HAL_CAN_MspInit, stm32f4xx_hal_msp.c:291-294) */
enum class CanIrqLine : uint8_t { Tx, Rx0, Rx1, Sce };
void canSetIrqLineEnabled(CanIrqLine line, bool enabled);
bool canIrqLineEnabled(CanIrqLine line);
/** Raises the CAN interrupt for every pending enabled event (after auto-complete). @return number of HAL_CAN_IRQHandler runs */
uint32_t canIrq();
/** The next mode change (HAL_CAN_Init / _Start / _Stop) is not acknowledged: HAL_ERROR, ErrorCode |= TIMEOUT, State ERROR (no time passes) */
void canFailNextModeChange();

/** Whether the active filter banks of CAN1 accept a frame (see canReceive), with the FIFO and filter match index */
bool canAccepts(uint32_t id, bool extId = false, bool rtr = false, uint32_t *fifo = nullptr, uint32_t *filterMatchIndex = nullptr);
/** Number of active filter banks of CAN1 (bits set in CAN1->FA1R): the hardware state, unlike canPort().activeFilters() */
uint32_t canActiveFilterBanks();
/** Every HAL_CAN_ConfigFilter argument since the reset (also rejected calls) */
const std::vector<CAN_FilterTypeDef> &canFilterConfigs();
/** A HAL CAN call: function name, argument (ConfigFilter: FilterBank, AbortTxRequest / IsTxMessagePending: mailboxes, (De)ActivateNotification: interrupts, AddTxMessage: *pTxMailbox) and result (status or returned value) */
struct CanHalCall {
	std::string function;
	uint32_t arg = 0;
	uint32_t result = 0;
};
/** HAL CAN calls of the firmware since the reset in order (not recorded: GetRxFifoFillLevel, GetRxMessage) */
const std::vector<CanHalCall> &canHalCalls();
void canClearHalCalls();

/**
 * Puts a received frame into receive FIFO 0 or 1 of hcan1 (3 frames deep) without filters and without raising an
 * interrupt (FilterMatchIndex as given). The third frame sets FULL; a full FIFO (not locked, MX_CAN1_Init
 * ReceiveFifoLocked = DISABLE) overwrites its newest frame and sets FOVR (RM0090 32.7.5, counted in
 * state().canRxOverruns). Fields are stored like the mailbox registers: StdId 11 bit, ExtId 29 bit, DLC 4 bit, Timestamp 16 bit.
 */
void canQueueRx(uint32_t fifo, const CAN_RxHeaderTypeDef &header, const uint8_t data[8]);
/** Frames waiting in a receive FIFO of hcan1 */
size_t canRxPending(uint32_t fifo);
/**
 * Calls HAL_CAN_RxFifo0MsgPendingCallback / ..Fifo1.. directly in interrupt context while frames are pending in the
 * FIFO, regardless of IER and the NVIC (a forced "message pending" interrupt, P4). Stops (state().canRxStalls) if a
 * callback reads nothing. HAL calls of the callbacks raise no further interrupt. @return number of callback calls
 */
uint32_t canRxInterrupt(uint32_t fifo);
/**
 * Delivers a data frame from the bus the way the hardware does it: only in normal mode (HAL_CAN_Start called, not
 * sleeping, not bus-off), the active filter banks of CAN1 decide (RM0090 32.7.4: 32 / 16 bit, mask / list, priority
 * 32 bit > 16 bit > list > mask > lower bank; filter match index per FIFO). The frame goes into the FIFO of the matching
 * bank and the receive interrupt is raised if enabled (CANPort_2B::start enables it): HAL_CAN_IRQHandler ->
 * HAL_CAN_RxFifo0MsgPendingCallback of global_callbacks.cpp reads the FIFO and calls every CanHandler. A FIFO 1 frame
 * raises no interrupt on the F407 (CAN1_RX1 is not enabled in the NVIC) and stays in the FIFO until the next CAN1_TX
 * or CAN1_RX0 interrupt (HAL_CAN_IRQHandler then also services FIFO 1); canReceive counts the message pending
 * callbacks of both FIFOs.
 * With the filters built by CANPort_2B::addCanFilter (CANPort2B.cpp:272-287) a standard filter compares STID /
 * EXID[28:18] only, so it also accepts extended frames whose upper 11 id bits match; an extended filter rejects
 * standard frames. (MockCANPort::passesFilter keeps its generic mock semantics.)
 * The DLC is data.size() (at most 8 bytes are used); extId selects a 29 bit id; rtr a remote frame.
 * @return number of message pending callbacks raised (0 = frame not accepted or not delivered)
 */
uint32_t canReceive(uint32_t id, const std::vector<uint8_t> &data, bool extId = false, bool rtr = false);

// --- SPI (fake HAL SPI F4, host_target_bus.cpp) ---
/**
 * The fake models stm32f4xx_hal_spi.c for the F407 configuration: master, 2 lines, 8 bit, no CRC (USE_SPI_CRC 0),
 * DMA streams linked for SPI1-3 in normal (not circular) mode (stm32f4xx_hal_msp.c). Other configurations (slave,
 * 1 line, rx only, 16 bit) are not modelled: such a call is recorded in state().unmodelledHalCalls and returns HAL_ERROR.
 *
 *  - Blocking calls (HAL_SPI_Transmit / _Receive / _TransmitReceive) complete before they return.
 *  - IT and DMA calls (_IT / _DMA) start the transfer and return; the transfer stays pending until the test calls
 *    spiComplete (end of transfer: data delivered, State READY, the HAL callback in interrupt context) or spiFail.
 *    While a transfer is pending State is BUSY_TX / BUSY_RX / BUSY_TX_RX and every new call returns HAL_BUSY.
 *  - Received bytes (MISO) of a transfer, in this order of priority: the next response queued with spiQueueRx, the
 *    responder function (spiSetResponder), the idle response (spiSetIdleRx). A response shorter than the transfer is
 *    padded with 0x00 (MISO driven low); a longer one is cut.
 *  - HAL_SPI_Receive(_IT/_DMA) as master in 2 line mode transmits the receive buffer as dummy bytes (the HAL calls
 *    TransmitReceive(pData, pData)); the recorded tx bytes are the buffer content at the start.
 */
enum class BusMode : uint8_t { Blocking, Interrupt, Dma };
enum class SpiCall : uint8_t { Transmit, Receive, TransmitReceive };
struct SpiTransfer {
	SpiCall call = SpiCall::Transmit;
	BusMode mode = BusMode::Blocking;
	uint16_t size = 0;
	uint32_t timeout = 0;		   //!< Timeout argument (blocking calls)
	std::vector<uint8_t> tx;	   //!< Bytes clocked out on MOSI. Blocking: read during the call; IT/DMA: read from the tx buffer when the transfer ends (spiComplete / spiFail), like the DMA / TXE interrupt reading it during the transfer; while pending: the buffer content at the start
	std::vector<uint8_t> rx;	   //!< Bytes clocked in on MISO. Written into the rx buffer (Transmit: discarded by the HAL)
	SPI_InitTypeDef init{};		   //!< Port configuration (hspi->Init) when the transfer started
	std::vector<bool> csAtStart;   //!< Output levels of the CS pins of the port (spiCsPins) when the transfer started
	std::vector<bool> csAtEnd;	   //!< ... when it ended (blocking: before the call returns; IT/DMA: before the callback)
	HAL_StatusTypeDef result = HAL_OK; //!< Return value of the HAL call that started the transfer
	bool completed = false;		   //!< Data delivered and completion callback raised (blocking: returned HAL_OK)
	uint32_t errorCode = 0;		   //!< ErrorCode of the handle at the end (spiFail, injected failures)
	/** Index of the only CS pin at its active level when the transfer started, -1 if none or several */
	int selectedCs(bool activeLow = true) const;
};
/** Responder: returns the MISO bytes for a transfer from its MOSI bytes (tx) and the port state (e.g. a register read) */
using SpiResponder = std::function<std::vector<uint8_t>(const SpiTransfer &)>;

void spiQueueRx(SPI_HandleTypeDef *hspi, const std::vector<uint8_t> &bytes); //!< MISO bytes of the next transfer (FIFO)
void spiSetIdleRx(SPI_HandleTypeDef *hspi, const std::vector<uint8_t> &bytes); //!< MISO bytes of every transfer without queued response
void spiSetResponder(SPI_HandleTypeDef *hspi, SpiResponder responder);			//!< nullptr removes it
size_t spiQueuedRx(const SPI_HandleTypeDef *hspi);								//!< Responses still queued
/** All transfers started on the port since the last reset (including rejected calls: result != HAL_OK, no data) */
const std::deque<SpiTransfer> &spiTransfers(const SPI_HandleTypeDef *hspi);
void spiClearTransfers(const SPI_HandleTypeDef *hspi);
/** True while an IT or DMA transfer is pending (started, not yet completed or failed) */
bool spiTransferPending(const SPI_HandleTypeDef *hspi);
/**
 * Ends the pending IT/DMA transfer like the end of transfer interrupt (SPI_CloseTx_ISR / SPI_CloseRx_ISR /
 * SPI_CloseRxTx_ISR) or the DMA transfer complete interrupt (SPI_DMATransmitCplt / SPI_DMAReceiveCplt /
 * SPI_DMATransmitReceiveCplt): the received bytes are written into the rx buffer, State READY, then in interrupt
 * context: for DMA transfers the half transfer callback (HAL_SPI_TxHalfCpltCallback / RxHalfCplt / TxRxHalfCplt, the
 * HAL enables the DMA half transfer interrupt; assumed also for 1 byte transfers) and then HAL_SPI_TxCpltCallback
 * (transmit), HAL_SPI_RxCpltCallback (receive) or HAL_SPI_TxRxCpltCallback (transmit/receive).
 * @return false if no transfer was pending
 */
bool spiComplete(SPI_HandleTypeDef *hspi);
/**
 * Ends the pending IT/DMA transfer with an error like HAL_SPI_IRQHandler's error branch (OVR, MODF, FRE) or
 * SPI_DMAError (HAL_SPI_ERROR_DMA): nothing is written into the rx buffer, ErrorCode |= errorCode, State READY,
 * HAL_SPI_ErrorCallback in interrupt context. @return false if no transfer was pending
 */
bool spiFail(SPI_HandleTypeDef *hspi, uint32_t errorCode = HAL_SPI_ERROR_OVR);
/**
 * The next blocking call fails: HAL_TIMEOUT (TXE/RXNE never set: nothing transferred, State READY, like the timeout
 * branches of HAL_SPI_TransmitReceive) or HAL_ERROR (the transfer happens but SPI_EndRxTxTransaction times out:
 * ErrorCode HAL_SPI_ERROR_FLAG).
 */
void spiFailNextBlocking(SPI_HandleTypeDef *hspi, HAL_StatusTypeDef status);
/**
 * The next DMA call fails to start its DMA stream (HAL_DMA_Start_IT != HAL_OK): like the HAL, ErrorCode |=
 * HAL_SPI_ERROR_DMA, the handle is unlocked, HAL_ERROR is returned and State stays BUSY_xx (the HAL does not restore
 * it, so every later call returns HAL_BUSY until HAL_SPI_Init).
 */
void spiFailNextDmaStart(SPI_HandleTypeDef *hspi);
/** HAL_SPI_Init calls since the reset; the configuration of the last one is hspi->Init */
uint32_t spiInitCount(const SPI_HandleTypeDef *hspi);
/** The CS pins of a port (hspi1-3: the pin lists of cpp_target_config.cpp) and their current output levels */
void spiSetCsPins(SPI_HandleTypeDef *hspi, const std::vector<std::pair<GPIO_TypeDef *, uint16_t>> &pins);
std::vector<bool> spiCsLevels(const SPI_HandleTypeDef *hspi);
/**
 * MISO bytes of a frame of up to 64 bits in transmission order with SPI_FIRSTBIT_MSB (the configuration of all
 * firmware SPI devices): the first bit on the line is bit (8 * bytes - 1) of `frame`, i.e. `bytes` bytes, most
 * significant first. E.g. spiFrameBytes(0x1234, 2) = {0x12, 0x34}. Use it to script 16, 24, 32 or 64 bit encoder
 * frames (8 bit SPI transfers of 2..8 bytes). bytes must be 1..8.
 */
std::vector<uint8_t> spiFrameBytes(uint64_t frame, uint8_t bytes);

// --- I2C (fake HAL I2C F5, host_target_bus.cpp) ---
/**
 * The fake models stm32f4xx_hal_i2c.c (master and memory transfers, 7 bit addressing) on a bus with scripted devices.
 * Addresses: the HAL functions take the 8 bit form (7 bit address << 1); devices are registered by their 7 bit address.
 *
 *  - Blocking calls complete before they return. A missing device or a device that does not acknowledge its address
 *    (NACK) gives HAL_ERROR with ErrorCode HAL_I2C_ERROR_AF (I2C_WaitOnMasterAddressFlagUntilTimeout), a NACKed data
 *    byte HAL_ERROR / AF (I2C_IsAcknowledgeFailed); State READY, Mode NONE.
 *  - IT calls (and DMA calls on a handle with DMA, see i2cSetDmaLinked) start the transfer and return HAL_OK; the
 *    transfer stays pending until i2cComplete (the device is accessed then; completion callback, or on a NACK
 *    HAL_I2C_ErrorCallback with AF like HAL_I2C_ER_IRQHandler) or i2cFail. hi2c1 has no DMA stream
 *    (stm32f4xx_hal_msp.c): its _DMA calls return HAL_ERROR with HAL_I2C_ERROR_DMA_PARAM like the HAL.
 *  - Any call while State is not READY returns HAL_BUSY. A busy bus (i2cSetBusBusy) gives HAL_BUSY with ErrorCode
 *    HAL_I2C_ERROR_TIMEOUT (the BUSY flag wait; no simulated time passes).
 *  - HAL_I2C_Init / HAL_I2C_DeInit drop a pending transfer (the peripheral is reset; no callback).
 */
enum class I2cCall : uint8_t { MasterTransmit, MasterReceive, MemWrite, MemRead, IsDeviceReady };
struct I2cTransfer {
	I2cCall call = I2cCall::MasterTransmit;
	BusMode mode = BusMode::Blocking;
	uint16_t devAddress = 0;  //!< DevAddress argument (8 bit form)
	uint8_t address = 0;	  //!< 7 bit address (devAddress >> 1)
	uint16_t memAddress = 0;  //!< MemAddress argument (Mem_Write / Mem_Read)
	uint16_t memAddSize = 0;  //!< Memory address size in bytes (1 or 2; I2C_MEMADD_SIZE_8BIT / _16BIT)
	uint16_t size = 0;		  //!< Size argument
	uint32_t timeout = 0;	  //!< Timeout argument (blocking calls)
	uint32_t trials = 0;	  //!< IsDeviceReady: Trials argument
	std::vector<uint8_t> data; //!< Written bytes that the device acknowledged (transmit / mem write; IT/DMA: read from the buffer when the transfer ends; while pending: the buffer content at the start), bytes read (receive / mem read)
	bool acknowledged = false; //!< The device acknowledged its address (IsDeviceReady: in one of the trials)
	HAL_StatusTypeDef result = HAL_OK; //!< Return value of the HAL call that started the transfer
	bool completed = false;	  //!< Blocking: returned HAL_OK; IT/DMA: completion callback raised
	uint32_t errorCode = 0;	  //!< ErrorCode of the handle at the end of the transfer
};
/**
 * Scripted I2C device. Register model (pointerRegister = true): a register pointer (pointerSize bytes) is set by the
 * memory address of Mem_Write / Mem_Read or by the first byte(s) of a Master_Transmit; data bytes are written to /
 * read from the register at the pointer. A register holds registerWidth bytes (bus order, e.g. big endian for the
 * ADS111X); after its last byte the pointer moves to the next register (autoIncrement, EEPROM like) or wraps to the
 * first byte of the same register (ADS111X). Registers never written read as 0xFF.
 * Plain model (pointerRegister = false, e.g. PCF8574): Master_Receive returns the bytes of registers[0] (repeated), a
 * Master_Transmit is only recorded.
 * readQueue: scripted read responses, consumed one per read transfer before the register model (padded with 0xFF).
 */
struct I2cDevice {
	bool pointerRegister = true;
	uint8_t pointerSize = 1;	//!< Bytes of the register pointer sent by Master_Transmit (Mem_* use MemAddSize)
	uint8_t registerWidth = 1;	//!< Bytes per register
	bool autoIncrement = true;
	uint16_t pointer = 0;		//!< Current register pointer
	std::map<uint16_t, std::vector<uint8_t>> registers;
	std::deque<std::vector<uint8_t>> readQueue;
	bool nackAddress = false;	 //!< Never acknowledges its address (absent / defective)
	uint32_t nackAddressCount = 0; //!< Does not acknowledge the next n address phases (busy, e.g. EEPROM write cycle)
	int32_t nackWriteByte = -1;	 //!< Index of the written byte (counted from the first byte after the address, memory address included) that is not acknowledged; -1 = none
	/** Called after the device took part in a transfer (also NACKed ones), before the HAL returns / the callback runs */
	std::function<void(I2cDevice &, const I2cTransfer &)> onTransfer;

	void setRegister(uint16_t reg, const std::vector<uint8_t> &bytes) { registers[reg] = bytes; }
	void setRegister16(uint16_t reg, uint16_t value) { registers[reg] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)}; } //!< Big endian
	uint16_t register16(uint16_t reg) const; //!< Big endian value of a 2 byte register (0xFFFF if never written)
	std::vector<uint8_t> registerBytes(uint16_t reg) const; //!< registerWidth bytes (0xFF if never written)
};
/** The device with this 7 bit address on the bus of hi2c (created on first use: acknowledges, register model) */
I2cDevice &i2cDevice(I2C_HandleTypeDef *hi2c, uint8_t address);
void i2cRemoveDevice(I2C_HandleTypeDef *hi2c, uint8_t address);
/** All transfers on the bus since the last reset (including rejected calls: result != HAL_OK) */
const std::deque<I2cTransfer> &i2cTransfers(const I2C_HandleTypeDef *hi2c);
void i2cClearTransfers(const I2C_HandleTypeDef *hi2c);
bool i2cTransferPending(const I2C_HandleTypeDef *hi2c);
/**
 * Ends the pending IT/DMA transfer: the device is accessed (address and data acknowledge, read data written into the
 * buffer), State READY, Mode NONE, then in interrupt context HAL_I2C_MasterTxCpltCallback / MasterRxCplt /
 * MemTxCplt / MemRxCplt (I2C_MasterTransmit_BTF, I2C_MasterReceive_RXNE/BTF, I2C_MemoryTransmit_TXE_BTF,
 * I2C_DMAXferCplt), or on a NACK ErrorCode |= AF and HAL_I2C_ErrorCallback (HAL_I2C_ER_IRQHandler -> I2C_ITError).
 * @return false if no transfer was pending
 */
bool i2cComplete(I2C_HandleTypeDef *hi2c);
/**
 * Ends the pending IT/DMA transfer with a bus error (HAL_I2C_ER_IRQHandler: BERR, ARLO, AF, OVR; I2C_DMAError: DMA):
 * no data, ErrorCode |= errorCode, State READY, Mode NONE, HAL_I2C_ErrorCallback in interrupt context.
 */
bool i2cFail(I2C_HandleTypeDef *hi2c, uint32_t errorCode = HAL_I2C_ERROR_BERR);
/** BUSY flag stuck (another master, SDA held low): every call returns HAL_BUSY with ErrorCode |= TIMEOUT */
void i2cSetBusBusy(I2C_HandleTypeDef *hi2c, bool busy);
/**
 * The next blocking call fails without data: returns `status` (HAL_ERROR or HAL_TIMEOUT), ErrorCode |= errorCode,
 * State READY, Mode NONE, unlocked (the timeout branches of I2C_WaitOnFlagUntilTimeout & co.).
 */
void i2cFailNextBlocking(I2C_HandleTypeDef *hi2c, HAL_StatusTypeDef status, uint32_t errorCode = HAL_I2C_ERROR_TIMEOUT);
/** Whether the handle has DMA streams linked (hdmatx / hdmarx). hi2c1: false (stm32f4xx_hal_msp.c) */
void i2cSetDmaLinked(I2C_HandleTypeDef *hi2c, bool linked);
uint32_t i2cInitCount(const I2C_HandleTypeDef *hi2c);   //!< HAL_I2C_Init calls since the reset
uint32_t i2cDeInitCount(const I2C_HandleTypeDef *hi2c); //!< HAL_I2C_DeInit calls since the reset

// --- OTP area (fake of OTP_Write / OTP_Read of flash_helpers.cpp, host_target_system.cpp) ---
/**
 * F407 OTP area (RM0090 3.7): 16 blocks of 32 data bytes (offsets 0..511) followed by 16 lock bytes (512..527);
 * erased = 0xFF, a block whose lock byte is 0x00 can not be programmed. Offsets are byte offsets from FLASH_OTP_BASE.
 * OTP_Write / OTP_Read reproduce flash_helpers.cpp:298-326 (the firmware source is not compiled in this
 * executable): slot address FLASH_OTP_BASE + adroffset * 8, rejected only if beyond FLASH_OTP_END (so offsets 64 and
 * 65 reach the lock bytes: bug 41), write only if all 8 bytes are 0xFF, two 32 bit programming steps (low word, then
 * high word; the second runs even if the first failed), programming clears bits only (old & new). Unlock / lock of
 * the flash are not modelled (always succeed). Calls recorded in state().otpCalls, failures with state().otpFailProgram.
 */
constexpr uint32_t OTP_DATA_SIZE = 512;
constexpr uint32_t OTP_SIZE = 528; // data + 16 lock bytes
uint8_t otpByte(uint32_t offset);
void setOtpByte(uint32_t offset, uint8_t value); //!< Raw write (no OTP rules), e.g. to prepare a programmed or locked area
uint64_t otpSlot(uint16_t adroffset);			  //!< Raw little endian 64 bit value of a slot (adroffset 0..65)
void setOtpSlot(uint16_t adroffset, uint64_t value); //!< Raw write of a slot

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
 * The global `canport` of the firmware is the real CANPort_2B on hcan1 (F407 presets 50k..1000k, silent pin CAN_S),
 * like cpp_target_config.cpp:51-56. canPort2B() returns it.
 * canPort() returns a view of it for the P5 tests: a MockCANPort whose fields record what the firmware did with
 * canport (started, startCalls, stopCalls, filters = the CAN_filter values passed to addCanFilter, active cleared by
 * removeCanFilter) and whose CANPort functions forward to canport. Its `sent`, `speed`, `silent` and `sendResult` are
 * not used. New tests use canPort2B() / canport and the fake HAL CAN (canTxFrames, canReceive, CAN1 registers).
 * The view records the calls, not the hardware state: `started` is set even when start() reports a failure, and
 * `filters` / activeFilters() count every filter passed to addCanFilter, also one that overwrote another bank or was
 * rejected. Tests about the hardware state use canActiveFilterBanks(), CAN1->FA1R, canAccepts and hcan1.State.
 */
MockCANPort &canPort();
CANPort_2B &canPort2B();

} // namespace HostTarget

#endif
