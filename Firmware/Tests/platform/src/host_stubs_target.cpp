/*
 * host_stubs_target.cpp
 *
 * Link stubs for ffb_target_tests: definitions of firmware symbols whose source file is not compiled yet,
 * and HAL functions that do not have a fake yet. Anything defined here is NOT tested.
 *
 * Remove an entry as soon as the real source (or a fake) is compiled:
 *  - cppmain.cpp (P4): mainclass, mainchooser
 *  - global_callbacks.cpp (P4): usb_device, getAnalogBuffer, ADC1_BUF/ADC2_BUF, HAL_TIM_PeriodElapsedCallback,
 *    HAL_GPIO_EXTI_Callback
 *  - voltagesense.cpp (P4, needs the fake ADC F6): getIntV, getExtV, getChipTemp
 *  - fake HAL SPI (F4, P5/P6) and fake HAL I2C (F5, P5): HAL_SPI_*, HAL_I2C_*
 */
#include "HostTarget.h"

#include "main.h"
#include "cppmain.h"
#include "mainclass_chooser.h"
#include "TimerHandler.h"
#include "ExtiHandler.h"
#include "USBdevice.h"
#include "voltagesense.h"
#include "global_callbacks.h"

// ---------------- cppmain.cpp ----------------
FFBoardMain *mainclass = nullptr; // The active main class. Set by tests that need it (e.g. SystemCommands)
ClassChooser<FFBoardMain> mainchooser(class_registry);

// ---------------- global_callbacks.cpp ----------------
USBdevice *usb_device = nullptr; // Set by USBdevice::registerUsb()
volatile uint32_t ADC1_BUF[ADC1_CHANNELS] = {0};
volatile uint32_t ADC2_BUF[ADC2_CHANNELS] = {0};

/** Same as getAnalogBuffer in global_callbacks.cpp: the DMA buffer and channel count of an ADC */
volatile uint32_t *getAnalogBuffer(ADC_HandleTypeDef *hadc, uint8_t *chans) {
	if (hadc == &hadc1) {
		*chans = ADC1_CHANNELS;
		return ADC1_BUF;
	}
	if (hadc == &hadc2) {
		*chans = ADC2_CHANNELS;
		return ADC2_BUF;
	}
	return NULL;
}

namespace HostTarget {
void resetStubs() {
	for (auto &v : ADC1_BUF) {
		v = 0;
	}
	for (auto &v : ADC2_BUF) {
		v = 0;
	}
}
} // namespace HostTarget

/**
 * Same as HAL_TIM_PeriodElapsedCallback / _CPP in global_callbacks.cpp: dispatch to all TimerHandlers.
 * Not modelled: the F407 main.c overrides HAL_TIM_PeriodElapsedCallback and handles TIM7 (the HAL tick timer,
 * HAL_IncTick) itself; only other timers reach HAL_TIM_PeriodElapsedCallback_CPP. The host has no TIM7 handle
 * (the tick is the simulated time of HostPlatform).
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	for (TimerHandler *c : TimerHandler::timerHandlers) {
		c->timerElapsed(htim);
	}
}

/** Same as HAL_GPIO_EXTI_Callback in global_callbacks.cpp: dispatch to all ExtiHandlers */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	for (ExtiHandler *c : ExtiHandler::extiHandlers) {
		c->exti(GPIO_Pin);
	}
}

// ---------------- voltagesense.cpp ----------------
// Stubs returning 0: no test may assert these values (vint, vext, temp commands of SystemCommands) before
// voltagesense.cpp is compiled with the fake ADC in P4.
int32_t getIntV() { return 0; }
int32_t getExtV() { return 0; }
int32_t getChipTemp() { return 0; }

// ---------------- HAL functions without a fake ----------------
// Recorded in HostTarget::state().unmodelledHalCalls and answered with HAL_ERROR. A test that reaches one of
// these needs the fake first; the fixtures check that the list is empty.
namespace {
HAL_StatusTypeDef unmodelled(const char *name) {
	HostTarget::state().unmodelledHalCalls.emplace_back(name);
	return HAL_ERROR;
}
} // namespace

extern "C" {
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *) { return unmodelled("HAL_SPI_Init"); }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_SPI_Transmit"); }
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_SPI_Receive"); }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_SPI_TransmitReceive"); }
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_Transmit_IT"); }
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_Receive_IT"); }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_TransmitReceive_IT"); }
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_Transmit_DMA"); }
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_Receive_DMA"); }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t) { return unmodelled("HAL_SPI_TransmitReceive_DMA"); }

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *) { return unmodelled("HAL_I2C_Init"); }
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *) { return unmodelled("HAL_I2C_DeInit"); }
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_I2C_Master_Transmit"); }
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_I2C_Master_Receive"); }
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Master_Transmit_IT"); }
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Master_Receive_IT"); }
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Master_Transmit_DMA"); }
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Master_Receive_DMA"); }
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_I2C_Mem_Write"); }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t) { return unmodelled("HAL_I2C_Mem_Read"); }
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Mem_Write_IT"); }
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t) { return unmodelled("HAL_I2C_Mem_Read_IT"); }
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *, uint16_t, uint32_t, uint32_t) { return unmodelled("HAL_I2C_IsDeviceReady"); }
} // extern "C"
