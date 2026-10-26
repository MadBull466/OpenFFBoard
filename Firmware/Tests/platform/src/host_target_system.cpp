/*
 * host_target_system.cpp
 *
 * System part of the fake F407 platform for ffb_target_tests (declarations in platform/target_f407/main.h, test
 * control in support/HostTarget.h):
 *  - fake HAL ADC (F6): HAL_ADC_Start/_Stop/_PollForConversion/_GetValue/_Start_DMA/_Stop_DMA and the DMA conversion
 *    complete interrupt (ADC_DMAConvCplt -> HAL_ADC_ConvCpltCallback); LL ADC calibration values
 *  - IWDG (HAL_IWDG_Refresh), the HAL time base TIM7 (HAL_InitTick, HAL_IncTick). The receive FIFOs of bxCAN moved to
 *    the fake HAL CAN in host_target_can.cpp (P7)
 *  - the callbacks main.c defines: HAL_TIM_PeriodElapsedCallback (TIM7 -> HAL_IncTick) and Error_Handler
 *  - control of the real start-up code (cppmain.cpp): taskYIELD hook, runCppmain, reset of its globals and of the
 *    globals of voltagesense.cpp and ledEffects.cpp
 *
 * Every fake models the documented behaviour of the real function. The reference is the ST HAL source in
 * Targets/F407VG/Drivers/STM32F4xx_HAL_Driver (read only) and Targets/F407VG/Core/Src (main.c,
 * stm32f4xx_hal_timebase_tim.c, stm32f4xx_hal_msp.c), cited per function.
 */
#include "HostTarget.h"
#include "HostPlatform.h"

#include "main.h"
#include "constants.h"
#include "FastAvg.h"
#include "FFBoardMain.h"
#include "global_callbacks.h"
#include "ledEffects.h"
#include "flash_helpers.h"

#include <algorithm>
#include <array>
#include <deque>
#include <map>

// ---------------- Firmware globals reset by resetSystem (all have external linkage in the firmware) ----------------
// cppmain.cpp
extern bool running;
extern bool mainclassChosen;
extern volatile bool forceErase;
extern uint16_t main_id;
extern FFBoardMain *mainclass;
// ledEffects.cpp
extern Ledstruct_t sysled, errled, clipled;
// voltagesense.cpp
extern bool braking_flag;
extern int32_t maxVoltage;
extern int32_t voltageDiffActivate;
extern int32_t voltageDiffDeactivate;
extern float vSenseMult;
extern bool brake_failure;
extern int32_t minVoltage;
extern uint32_t brakeActiveTime;
extern FastMovingAverage<int32_t> chipTempAvg;

// LL ADC calibration values (main.h: VREFINT_CAL_ADDR, TEMPSENSOR_CAL1_ADDR, TEMPSENSOR_CAL2_ADDR)
uint16_t hosttest_adc_vrefint_cal = 1500;
uint16_t hosttest_adc_ts_cal1 = 950;
uint16_t hosttest_adc_ts_cal2 = 1200;

// TIM7: HAL time base (stm32f4xx_hal_timebase_tim.c defines htim7 on the target)
TIM_TypeDef hosttest_tim7 = {};
TIM_HandleTypeDef htim7 = {};

// IWDG (MX_IWDG_Init in main.c defines hiwdg)
namespace {
IWDG_TypeDef iwdgRegs = {};
}
IWDG_HandleTypeDef hiwdg = {};

namespace {

/**
 * Initial values of the firmware globals. Taken during the dynamic initialisation of this file: all of them are
 * constant-initialised (literals and addresses), so their initial values are already in place.
 */
struct FirmwareInitialValues {
	bool running = ::running;
	bool mainclassChosen = ::mainclassChosen;
	bool forceErase = ::forceErase;
	uint16_t mainId = ::main_id;
	// The LEDs are written literally like ledEffects.cpp:10-18: command handlers constructed during the dynamic
	// initialisation of other files call pulseSysLed() (CommandHandler.cpp:505), possibly before this snapshot is taken.
	Ledstruct_t sysled{0, 0, 0, LED_SYS_GPIO_Port, LED_SYS_Pin, 0};
	Ledstruct_t errled{0, 0, 0, LED_ERR_GPIO_Port, LED_ERR_Pin, 0};
	Ledstruct_t clipled{0, 0, 0, LED_CLIP_GPIO_Port, LED_CLIP_Pin, 0};
	bool brakingFlag = ::braking_flag;
	int32_t maxVoltage = ::maxVoltage;
	int32_t voltageDiffActivate = ::voltageDiffActivate;
	int32_t voltageDiffDeactivate = ::voltageDiffDeactivate;
	float vSenseMult = ::vSenseMult;
	bool brakeFailure = ::brake_failure;
	int32_t minVoltage = ::minVoltage;
	uint32_t brakeActiveTime = ::brakeActiveTime;
};
const FirmwareInitialValues initialValues;

// ---- ADC ----
struct AdcFake {
	uint32_t *dmaBuffer = nullptr; //!< pData of HAL_ADC_Start_DMA
	uint32_t dmaLength = 0;		   //!< Length of HAL_ADC_Start_DMA
	bool dmaRunning = false;	   //!< DMA stream enabled (HAL_DMA_Start_IT .. HAL_DMA_Abort)
	std::vector<uint32_t> inputs;  //!< Analog input per rank (adcSetInput)
};
std::map<const ADC_HandleTypeDef *, AdcFake> adcs;

// ---- OTP (HostTarget::otpByte ...) ----
std::array<uint8_t, HostTarget::OTP_SIZE> otp;
constexpr uint32_t OTP_BASE = 0x1FFF7800UL; // FLASH_OTP_BASE (stm32f407xx.h)
constexpr uint32_t OTP_END = 0x1FFF7A0FUL;	// FLASH_OTP_END (stm32f407xx.h): last lock byte
constexpr uint32_t OTP_BLOCK_SIZE = 32;

/**
 * One FLASH_TYPEPROGRAM_WORD step of OTP_Write (HAL_FLASH_Program, stm32f4xx_hal_flash.c): fails (HAL_ERROR, nothing
 * programmed) when injected (state().otpFailProgram) or when the word is in a block whose lock byte is 0x00
 * (RM0090 3.7, WRPERR). Programming clears bits only.
 */
bool otpProgramWord(uint32_t offset, uint32_t word) {
	HostTarget::State &st = HostTarget::state();
	st.otpPrograms++;
	if (st.otpFailProgram != 0 && st.otpPrograms == st.otpFailProgram) {
		return false;
	}
	if (offset < HostTarget::OTP_DATA_SIZE && otp[HostTarget::OTP_DATA_SIZE + offset / OTP_BLOCK_SIZE] == 0x00) {
		return false;
	}
	for (uint32_t i = 0; i < 4; i++) {
		otp[offset + i] &= (uint8_t)(word >> (8 * i));
	}
	return true;
}

// ---- cppmain ----
uint32_t cppmainLoopsLeft = 0;
FFBoardMain *cppmainCreated = nullptr;

/** ADC_IT_OVR (stm32f4xx_hal_adc.h) = ADC_CR1_OVRIE */
constexpr uint32_t ADC_IT_OVR = 0x04000000U;

/**
 * __HAL_LOCK (stm32f4xx_hal_def.h): returns true (= HAL_BUSY) if the handle is already locked, otherwise locks it.
 */
bool halLock(ADC_HandleTypeDef *hadc) {
	if (hadc->Lock == HAL_LOCKED) {
		return true;
	}
	hadc->Lock = HAL_LOCKED;
	return false;
}

/** Full scale of the ADC resolution (__LL_ADC_DIGITAL_SCALE) */
uint32_t adcFullScale(const ADC_HandleTypeDef *hadc) { return __LL_ADC_DIGITAL_SCALE(hadc->Init.Resolution); }

uint32_t adcInput(const ADC_HandleTypeDef *hadc, uint32_t rank) {
	auto it = adcs.find(hadc);
	if (it == adcs.end() || rank >= it->second.inputs.size()) {
		return 0;
	}
	return std::min(it->second.inputs[rank], adcFullScale(hadc)); // Input above Vref+ converts to full scale
}

/**
 * Common part of HAL_ADC_Start and HAL_ADC_Start_DMA (stm32f4xx_hal_adc.c) up to the DMA set-up: enable the ADC
 * (ADON; the stabilisation delay is a busy loop without time), state REG_BUSY (INJ_BUSY with JAUTO), error code
 * cleared, unlocked. ADON can always be set on the host, so the HAL_ADC_ERROR_INTERNAL branch is not reachable.
 */
void adcEnableAndSetBusy(ADC_HandleTypeDef *hadc) {
	if ((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON) {
		hadc->Instance->CR2 |= ADC_CR2_ADON; // __HAL_ADC_ENABLE
	}
	hadc->State = (hadc->State & ~(HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR)) | HAL_ADC_STATE_REG_BUSY;
	if ((hadc->Instance->CR1 & ADC_CR1_JAUTO) != 0U) {
		hadc->State = (hadc->State & ~HAL_ADC_STATE_INJ_EOC) | HAL_ADC_STATE_INJ_BUSY;
	}
	if ((hadc->State & HAL_ADC_STATE_INJ_BUSY) != 0U) {
		hadc->ErrorCode &= ~(HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA);
	} else {
		hadc->ErrorCode = HAL_ADC_ERROR_NONE;
	}
	hadc->Lock = HAL_UNLOCKED;
}

/**
 * Software start of HAL_ADC_Start(_DMA): single ADC mode (ADC common CCR MULTI = 0 on the F407 configuration), so
 * SWSTART is set if no external trigger is configured (EXTEN = 0). The F407 ADCs use the TIM4 CC4 trigger.
 */
void adcSoftwareStart(ADC_HandleTypeDef *hadc) {
	if ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == 0U) {
		hadc->Instance->CR2 |= ADC_CR2_SWSTART;
	}
}

/** ADC_IS_SOFTWARE_START_REGULAR (stm32f4xx_hal_adc.h) */
bool adcSoftwareStartRegular(const ADC_HandleTypeDef *hadc) { return (hadc->Instance->CR2 & ADC_CR2_EXTEN) == 0U; }

/** End of the regular group in ADC_DMAConvCplt / HAL_ADC_PollForConversion: no further conversion upcoming -> READY */
void adcEndOfRegularGroup(ADC_HandleTypeDef *hadc) {
	if (adcSoftwareStartRegular(hadc) && hadc->Init.ContinuousConvMode == DISABLE &&
		((hadc->Instance->SQR1 & ADC_SQR1_L) == 0U || (hadc->Instance->CR2 & ADC_CR2_EOCS) == 0U)) {
		hadc->State &= ~HAL_ADC_STATE_REG_BUSY;
		if ((hadc->State & HAL_ADC_STATE_INJ_BUSY) == 0U) {
			hadc->State |= HAL_ADC_STATE_READY;
		}
	}
}

/** Models HAL_InitTick (stm32f4xx_hal_timebase_tim.c): TIM7 at 1 MHz (APB1 timer clock 84 MHz, PSC 83), Period 999, started with interrupt */
void initTickTimer() {
	hosttest_tim7 = TIM_TypeDef();
	htim7 = TIM_HandleTypeDef();
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 84 - 1;
	htim7.Init.Period = (1000000U / 1000U) - 1U;
	hosttest_tim7.PSC = htim7.Init.Prescaler;
	hosttest_tim7.ARR = htim7.Init.Period;
	htim7.State = HAL_TIM_STATE_BUSY; // HAL_TIM_Base_Start_IT
	hosttest_tim7.DIER |= TIM_DIER_UIE;
	hosttest_tim7.CR1 |= TIM_CR1_CEN;
	hosttest_tim7.CNT = (uint32_t)(HostPlatform::nowUs() % 1000U);
}

} // namespace

// ---------------- Hooks called by the shared fake platform ----------------

/** host_platform.cpp (HOSTTEST_REAL_CPPMAIN): the simulated time changed. TIM7 counts the microseconds since the last tick */
void hosttest_timeChanged(uint64_t us) {
	if ((hosttest_tim7.CR1 & TIM_CR1_CEN) != 0U) {
		hosttest_tim7.CNT = (uint32_t)(us % 1000U);
	}
}

/** platform/include/task.h (HOSTTEST_REAL_CPPMAIN): taskYIELD(). Ends the cppmain loop after the requested passes */
extern "C" void hosttest_taskYield(void) {
	HostTarget::state().taskYields++;
	if (cppmainLoopsLeft > 0 && --cppmainLoopsLeft == 0) {
		running = false; // while(running) of cppmain() ends after this pass
	}
}

namespace HostTarget {

void resetSystem() {
	// ADC
	adcs.clear();
	hosttest_adc_vrefint_cal = 1500;
	hosttest_adc_ts_cal1 = 950;
	hosttest_adc_ts_cal2 = 1200;
	for (auto &v : ADC1_BUF) {
		v = 0;
	}
	for (auto &v : ADC2_BUF) {
		v = 0;
	}
	// HAL tick, watchdog (the CAN receive FIFOs are reset with the fake HAL CAN by resetTargetConfig -> resetCan)
	initTickTimer();
	iwdgRegs = IWDG_TypeDef();
	hiwdg = IWDG_HandleTypeDef();
	hiwdg.Instance = &iwdgRegs;
	hiwdg.Init.Prescaler = 0x00000002U; // MX_IWDG_Init (main.c:569): IWDG_PRESCALER_16 = IWDG_PR_PR_1
	hiwdg.Init.Reload = 4095;			// MX_IWDG_Init
	// Firmware globals
	running = initialValues.running;
	mainclassChosen = initialValues.mainclassChosen;
	forceErase = initialValues.forceErase;
	main_id = initialValues.mainId;
	cppmainLoopsLeft = 0;
	sysled = initialValues.sysled;
	errled = initialValues.errled;
	clipled = initialValues.clipled;
	braking_flag = initialValues.brakingFlag;
	maxVoltage = initialValues.maxVoltage;
	voltageDiffActivate = initialValues.voltageDiffActivate;
	voltageDiffDeactivate = initialValues.voltageDiffDeactivate;
	vSenseMult = initialValues.vSenseMult;
	brake_failure = initialValues.brakeFailure;
	minVoltage = initialValues.minVoltage;
	brakeActiveTime = initialValues.brakeActiveTime;
	chipTempAvg.clear();
	otp.fill(0xFF); // OTP area erased
}

// ---- OTP ----
uint8_t otpByte(uint32_t offset) { return otp.at(offset); }
void setOtpByte(uint32_t offset, uint8_t value) { otp.at(offset) = value; }
uint64_t otpSlot(uint16_t adroffset) {
	uint64_t v = 0;
	for (uint32_t i = 0; i < 8; i++) {
		v |= (uint64_t)otp.at(adroffset * 8U + i) << (8 * i);
	}
	return v;
}
void setOtpSlot(uint16_t adroffset, uint64_t value) {
	for (uint32_t i = 0; i < 8; i++) {
		otp.at(adroffset * 8U + i) = (uint8_t)(value >> (8 * i));
	}
}

// ---- ADC ----
void adcSetInput(ADC_HandleTypeDef *hadc, uint32_t rank, uint32_t value) {
	std::vector<uint32_t> &inputs = adcs[hadc].inputs;
	if (inputs.size() <= rank) {
		inputs.resize(rank + 1, 0);
	}
	inputs[rank] = value;
}

void adcSetInputs(ADC_HandleTypeDef *hadc, const std::vector<uint32_t> &values) {
	for (uint32_t i = 0; i < values.size(); i++) {
		adcSetInput(hadc, i, values[i]);
	}
}

bool adcConversionComplete(ADC_HandleTypeDef *hadc) {
	auto it = adcs.find(hadc);
	if (it == adcs.end() || !it->second.dmaRunning || (hadc->Instance->CR2 & (ADC_CR2_ADON | ADC_CR2_DMA)) != (ADC_CR2_ADON | ADC_CR2_DMA)) {
		return false; // No conversion with DMA running
	}
	AdcFake &adc = it->second;
	// DMA transfer of the regular sequence (one word per rank, DMA_MDATAALIGN_WORD, circular: stm32f4xx_hal_msp.c)
	for (uint32_t i = 0; i < adc.dmaLength; i++) {
		uint32_t value = adcInput(hadc, i);
		hadc->Instance->DR = value;
		adc.dmaBuffer[i] = value;
	}
	// ADC_DMAConvCplt (stm32f4xx_hal_adc.c)
	if ((hadc->State & (HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA)) != 0U) {
		return false; // Error branch (HAL_ADC_ErrorCallback / DMA error callback): not reachable with this fake
	}
	hadc->State |= HAL_ADC_STATE_REG_EOC;
	adcEndOfRegularGroup(hadc);
	HostPlatform::IsrContext isr;
	HAL_ADC_ConvCpltCallback(hadc);
	return true;
}

bool adcConvert(ADC_HandleTypeDef *hadc, const std::vector<uint32_t> &values) {
	adcSetInputs(hadc, values);
	return adcConversionComplete(hadc);
}

bool adcDmaRunning(const ADC_HandleTypeDef *hadc) {
	auto it = adcs.find(hadc);
	return it != adcs.end() && it->second.dmaRunning;
}

uint32_t *adcDmaBuffer(const ADC_HandleTypeDef *hadc) { return adcDmaRunning(hadc) ? adcs[hadc].dmaBuffer : nullptr; }
uint32_t adcDmaLength(const ADC_HandleTypeDef *hadc) { return adcDmaRunning(hadc) ? adcs[hadc].dmaLength : 0; }

bool adcSetConversionResult(ADC_HandleTypeDef *hadc, uint32_t value) {
	if ((hadc->Instance->CR2 & ADC_CR2_ADON) == 0U) {
		return false;
	}
	hadc->Instance->DR = std::min(value, adcFullScale(hadc));
	hadc->Instance->SR |= ADC_SR_EOC | ADC_SR_STRT;
	return true;
}

void setAdcCalibration(uint16_t vrefintCal, uint16_t tsCal1, uint16_t tsCal2) {
	hosttest_adc_vrefint_cal = vrefintCal;
	hosttest_adc_ts_cal1 = tsCal1;
	hosttest_adc_ts_cal2 = tsCal2;
}

// ---- HAL tick ----
uint32_t tickTimerCounter() { return hosttest_tim7.CNT; }

// ---- cppmain ----
void runCppmain(uint32_t loops) {
	running = loops > 0;
	cppmainLoopsLeft = loops;
	cppmain();
	cppmainLoopsLeft = 0;
	cppmainCreated = mainclass;
}

FFBoardMain *cppmainMainclass() { return cppmainCreated; }

void deleteCppmainMainclass() {
	if (cppmainCreated == nullptr) {
		return;
	}
	if (mainclass == cppmainCreated) {
		mainclass = nullptr;
	}
	delete cppmainCreated;
	cppmainCreated = nullptr;
}

} // namespace HostTarget

extern "C" {

// ---------------- main.c (Targets/F407VG/Core/Src/main.c) ----------------

/** main.c HAL_TIM_PeriodElapsedCallback: TIM7 (HAL time base) -> HAL_IncTick, every other timer -> HAL_TIM_PeriodElapsedCallback_CPP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM7) {
		HAL_IncTick();
	} else {
		HAL_TIM_PeriodElapsedCallback_CPP(htim);
	}
}

/**
 * main.c Error_Handler: switches the error LED off (LED_ERR reset) and returns. Test extension: the
 * state().errorHandlerLimit-th call throws HostTarget::ErrorHandlerLimitReached, so a firmware loop that calls
 * Error_Handler forever (cppmain's wrong chip id check) can be left. Only C++ callers are expected while it is set.
 */
void Error_Handler(void) {
	HostTarget::state().errorHandlerCalls++;
	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
	if (HostTarget::state().errorHandlerLimit != 0 && HostTarget::state().errorHandlerCalls >= HostTarget::state().errorHandlerLimit) {
		throw HostTarget::ErrorHandlerLimitReached();
	}
}

// ---------------- HAL tick (stm32f4xx_hal.c) ----------------

/** Models HAL_IncTick: uwTick += uwTickFreq. Counted only, HAL_GetTick() follows the simulated time */
void HAL_IncTick(void) { HostTarget::state().halIncTicks++; }

// ---------------- IWDG (stm32f4xx_hal_iwdg.c) ----------------

/** Models HAL_IWDG_Refresh: __HAL_IWDG_RELOAD_COUNTER (KR = IWDG_KEY_RELOAD), HAL_OK */
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg) {
	hiwdg->Instance->KR = IWDG_KEY_RELOAD;
	HostTarget::state().watchdogRefreshes++;
	return HAL_OK;
}

// ---------------- ADC (stm32f4xx_hal_adc.c) ----------------

/**
 * Models HAL_ADC_Start: __HAL_LOCK (HAL_BUSY if locked), ADC enabled (ADON), state REG_BUSY, error code cleared,
 * unlocked, EOC/OVR flags cleared, SWSTART if no external trigger. Always HAL_OK otherwise.
 */
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc) {
	if (halLock(hadc)) {
		return HAL_BUSY;
	}
	adcEnableAndSetBusy(hadc);
	hadc->Instance->SR &= ~(ADC_FLAG_EOC | ADC_FLAG_OVR);
	adcSoftwareStart(hadc);
	return HAL_OK;
}

/** Models HAL_ADC_Stop: __HAL_LOCK, ADC disabled (ADON 0), state REG_BUSY/INJ_BUSY -> READY, unlocked, HAL_OK */
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc) {
	if (halLock(hadc)) {
		return HAL_BUSY;
	}
	hadc->Instance->CR2 &= ~ADC_CR2_ADON;
	hadc->State = (hadc->State & ~(HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY)) | HAL_ADC_STATE_READY;
	hadc->Lock = HAL_UNLOCKED;
	return HAL_OK;
}

/**
 * Models HAL_ADC_PollForConversion: HAL_ERROR (state ERROR_CONFIG) if EOCS and DMA are both set (DMA mode with end
 * of each conversion, like the F407 configuration after HAL_ADC_Start_DMA). Otherwise waits for EOC: the wait takes
 * simulated time (Timeout + 1 ms) and ends with HAL_TIMEOUT (state TIMEOUT) if no conversion result is pending
 * (HostTarget::adcSetConversionResult). An endless wait (HAL_MAX_DELAY) is recorded as unmodelled and returns
 * HAL_TIMEOUT. With EOC: STRT/EOC cleared, state REG_EOC, READY if no further conversion follows, HAL_OK.
 */
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout) {
	if ((hadc->Instance->CR2 & ADC_CR2_EOCS) != 0U && (hadc->Instance->CR2 & ADC_CR2_DMA) != 0U) {
		hadc->State |= HAL_ADC_STATE_ERROR_CONFIG;
		hadc->Lock = HAL_UNLOCKED;
		return HAL_ERROR;
	}
	if ((hadc->Instance->SR & ADC_FLAG_EOC) == 0U) {
		if (Timeout == HAL_MAX_DELAY) {
			HostTarget::state().unmodelledHalCalls.emplace_back("HAL_ADC_PollForConversion without a conversion and HAL_MAX_DELAY (waits forever)");
		} else if (Timeout > 0U) {
			HostPlatform::advanceMs((uint64_t)Timeout + 1U); // The busy wait until HAL_GetTick() - tickstart > Timeout
		}
		hadc->State |= HAL_ADC_STATE_TIMEOUT;
		hadc->Lock = HAL_UNLOCKED;
		return HAL_TIMEOUT;
	}
	hadc->Instance->SR &= ~(ADC_FLAG_STRT | ADC_FLAG_EOC);
	hadc->State |= HAL_ADC_STATE_REG_EOC;
	adcEndOfRegularGroup(hadc);
	return HAL_OK;
}

/**
 * Models HAL_ADC_Start_DMA: __HAL_LOCK, ADC enabled, DMA bit cleared, state REG_BUSY, error code cleared, unlocked,
 * EOC/OVR cleared, overrun interrupt enabled, DMA bit set, HAL_DMA_Start_IT(pData, Length) and SWSTART if no external
 * trigger. HAL_DMA_Start_IT returns HAL_BUSY without reconfiguring if the stream already runs (stm32f4xx_hal_dma.c);
 * HAL_ADC_Start_DMA ignores that result, so a second start keeps the first buffer. Always HAL_OK (unless locked).
 */
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length) {
	if (halLock(hadc)) {
		return HAL_BUSY;
	}
	if ((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON) {
		hadc->Instance->CR2 |= ADC_CR2_ADON;
	}
	hadc->Instance->CR2 &= ~ADC_CR2_DMA;
	adcEnableAndSetBusy(hadc);
	hadc->Instance->SR &= ~(ADC_FLAG_EOC | ADC_FLAG_OVR);
	hadc->Instance->CR1 |= ADC_IT_OVR;
	hadc->Instance->CR2 |= ADC_CR2_DMA;
	AdcFake &adc = adcs[hadc];
	if (!adc.dmaRunning) {
		adc.dmaBuffer = pData;
		adc.dmaLength = Length;
		adc.dmaRunning = true;
	}
	adcSoftwareStart(hadc);
	return HAL_OK;
}

/**
 * Models HAL_ADC_Stop_DMA: __HAL_LOCK, ADC disabled, DMA bit cleared, running DMA aborted (HAL_DMA_Abort, HAL_OK),
 * overrun interrupt disabled, state REG_BUSY/INJ_BUSY -> READY, unlocked, HAL_OK.
 */
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc) {
	if (halLock(hadc)) {
		return HAL_BUSY;
	}
	hadc->Instance->CR2 &= ~ADC_CR2_ADON;
	hadc->Instance->CR2 &= ~ADC_CR2_DMA;
	auto it = adcs.find(hadc);
	if (it != adcs.end() && it->second.dmaRunning) {
		it->second.dmaRunning = false;
		it->second.dmaBuffer = nullptr;
		it->second.dmaLength = 0;
	}
	hadc->Instance->CR1 &= ~ADC_IT_OVR;
	hadc->State = (hadc->State & ~(HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY)) | HAL_ADC_STATE_READY;
	hadc->Lock = HAL_UNLOCKED;
	return HAL_OK;
}

/** Models HAL_ADC_GetValue: returns DR. Reading DR clears the EOC flag in hardware (RM0090 13.13.1) */
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc) {
	hadc->Instance->SR &= ~ADC_SR_EOC;
	return hadc->Instance->DR;
}

} // extern "C"

// ---------------- OTP (flash_helpers.cpp:298-326, F407: FLASH_OTP_BASE / FLASH_OTP_END defined) ----------------
// flash_helpers.cpp is not compiled in ffb_target_tests (map-backed Flash_* of host_platform.cpp); these reproduce
// its OTP functions on the in-memory OTP area (see HostTarget.h). host_platform.cpp leaves out its stubs
// (HOSTTEST_TARGET_OTP).

/** Like OTP_Write of flash_helpers.cpp: bound check as coded (bug 41), only an all-ones slot, low word then high word */
bool OTP_Write(uint16_t adroffset, uint64_t dat) {
	HostTarget::State::OtpCall call;
	call.write = true;
	call.offset = adroffset;
	call.value = dat;
	uint32_t adr = (OTP_BASE + adroffset * sizeof(uint64_t));
	bool success = false;
	if (adr <= OTP_END && HostTarget::otpSlot(adroffset) == 0xffffffffffffffffULL) {
		uint32_t offset = adr - OTP_BASE;
		success = otpProgramWord(offset, (uint32_t)(dat));
		success = otpProgramWord(offset + 4, (uint32_t)(dat >> 32)) && success;
	}
	call.result = success;
	HostTarget::state().otpCalls.push_back(call);
	return success;
}

/** Like OTP_Read of flash_helpers.cpp: bound check as coded (bug 41), reads the 8 bytes of the slot */
bool OTP_Read(uint16_t adroffset, uint64_t *dat) {
	HostTarget::State::OtpCall call;
	call.offset = adroffset;
	uint32_t adr = (OTP_BASE + adroffset * sizeof(uint64_t));
	if (adr > OTP_END) {
		HostTarget::state().otpCalls.push_back(call);
		return false;
	}
	*dat = HostTarget::otpSlot(adroffset);
	call.value = *dat;
	call.result = true;
	HostTarget::state().otpCalls.push_back(call);
	return true;
}
