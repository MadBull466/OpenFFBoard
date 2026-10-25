/*
 * host_platform.cpp
 *
 * Fake hardware layer for the host unit tests: HAL, time base, flash storage and LEDs.
 * Replaces the target specific code (Targets/<target>/Core) and the hardware bound
 * helpers (flash_helpers.cpp, ledEffects.cpp, parts of cppmain.cpp). ffb_flash_tests keeps the real
 * flash_helpers.cpp (HOSTTEST_REAL_FLASH_HELPERS, see the end of this file).
 * ffb_target_tests compiles the real cppmain.cpp (HOSTTEST_REAL_CPPMAIN: micros(), refreshWatchdog(),
 * getRunTimeCounterValue() and Error_Handler() are left out here) and the real ledEffects.cpp
 * (HOSTTEST_REAL_LED_EFFECTS: the LED functions here only count the calls and forward them to the real code).
 * See support/HostPlatform.h for the test control interface.
 */
#include "HostPlatform.h"

#include "main.h"
#include "cmsis_compiler.h"
#include "cppmain.h"
#include "flash_helpers.h"
#include "ledEffects.h"
#include "UART.h"

uint32_t hosttest_ipsr = 0;

UART_HandleTypeDef huart1 = {};
CAN_HandleTypeDef hcan1 = {};
UARTPort external_uart{UART_PORT_EXT}; // Normally defined in cpp_target_config.cpp

namespace {
uint64_t timeUs = 0;
HostPlatform::LedState ledState;
HostPlatform::UartState uartState;
uint8_t *uartRxBuffer = nullptr; // Buffer armed by HAL_UART_Receive_IT/_DMA
UART_HandleTypeDef *uartRxHandle = nullptr;
uint16_t uartRxSize = 0;  // Armed transfer size. Completion after this many bytes
uint16_t uartRxCount = 0; // Bytes stored in the armed buffer so far
} // namespace

#ifdef HOSTTEST_REAL_CPPMAIN
// host_target_system.cpp: keeps the counter of the HAL tick timer (TIM7, read by the real micros()) in step with the
// simulated time. Called after every change of the time.
void hosttest_timeChanged(uint64_t us);
#endif

namespace HostPlatform {

void reset() {
	timeUs = 0;
#ifdef HOSTTEST_REAL_CPPMAIN
	hosttest_timeChanged(timeUs);
#endif
	hosttest_ipsr = 0;
	flash().clear();
	ledState = LedState();
	uartState = UartState();
	uartRxBuffer = nullptr;
	uartRxHandle = nullptr;
	uartRxSize = 0;
	uartRxCount = 0;
	huart1 = {};
	// Release the port in case a test left a transfer unfinished
	external_uart.giveSemaphore(true);
	external_uart.giveSemaphore(false);
}

void uartCompleteTx() {
	IsrContext isr;
	for (UartHandler *handler : UartHandler::getUARTHandlers()) {
		handler->uartTxComplete(&huart1);
	}
}

void uartReceive(const std::string &data) {
	for (char c : data) {
		if (uartRxBuffer == nullptr) {
			uartState.rxLost++;
			continue;
		}
		uartRxBuffer[uartRxCount++] = (uint8_t)c;
		if (uartRxCount < uartRxSize) {
			continue; // Like the HAL: the complete callback follows after `size` bytes
		}
		UART_HandleTypeDef *huart = uartRxHandle;
		uartRxBuffer = nullptr; // Single shot. Must be rearmed by the firmware
		uartRxCount = 0;
		IsrContext isr;
		for (UartHandler *handler : UartHandler::getUARTHandlers()) {
			handler->uartRxComplete(huart);
		}
	}
}

uint64_t nowUs() { return timeUs; }
void setTimeUs(uint64_t us) {
	timeUs = us;
#ifdef HOSTTEST_REAL_CPPMAIN
	hosttest_timeChanged(timeUs);
#endif
}
void advanceUs(uint64_t us) {
	timeUs += us;
#ifdef HOSTTEST_REAL_CPPMAIN
	hosttest_timeChanged(timeUs);
#endif
}
void setInIsr(bool isr) { hosttest_ipsr = isr ? 1 : 0; }

std::map<uint16_t, uint16_t> &flash() {
	static std::map<uint16_t, uint16_t> storage;
	return storage;
}

LedState &leds() { return ledState; }
UartState &uart() { return uartState; }

} // namespace HostPlatform

// ---------------- Time base ----------------
#ifndef HOSTTEST_REAL_CPPMAIN
uint32_t micros() { return (uint32_t)timeUs; }
void refreshWatchdog() {}
unsigned long getRunTimeCounterValue(void) { return (unsigned long)timeUs; }
#endif

extern "C" {

uint32_t HAL_GetTick(void) { return (uint32_t)(timeUs / 1000); }
void HAL_Delay(uint32_t ms) {
	timeUs += (uint64_t)ms * 1000;
#ifdef HOSTTEST_REAL_CPPMAIN
	hosttest_timeChanged(timeUs);
#endif
}
void HAL_NVIC_SystemReset(void) {}
#ifndef HOSTTEST_REAL_CPPMAIN
void Error_Handler(void) {}
#endif

// ---------------- GPIO ----------------
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *) {}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
	if (port) {
		port->ODR = state == GPIO_PIN_SET ? (port->ODR | pin) : (port->ODR & ~pin);
	}
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
	return (port && (port->IDR & pin)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin) {
	if (port) {
		port->ODR ^= pin;
	}
}

// ---------------- UART ----------------
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *) {
	uartState.initCount++;
	return (HAL_StatusTypeDef)uartState.initResult;
}
static HAL_StatusTypeDef recordUartTx(const uint8_t *data, uint16_t size) {
	if (uartState.txResult != HAL_OK) {
		return (HAL_StatusTypeDef)uartState.txResult;
	}
	uartState.tx.append((const char *)data, size);
	uartState.txTransfers++;
	return HAL_OK;
}
static HAL_StatusTypeDef armUartRx(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size) {
	uartState.lastRxSize = size;
	if (uartState.rxArmResult != HAL_OK) {
		return (HAL_StatusTypeDef)uartState.rxArmResult;
	}
	uartRxBuffer = data;
	uartRxHandle = huart;
	uartRxSize = size == 0 ? 1 : size;
	uartRxCount = 0;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *data, uint16_t size, uint32_t timeout) {
	uartState.txBlockingCount++;
	uartState.lastTimeout = timeout;
	return recordUartTx(data, size);
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *data, uint16_t size) {
	uartState.txItCount++;
	return recordUartTx(data, size);
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, const uint8_t *data, uint16_t size) {
	uartState.txDmaCount++;
	return recordUartTx(data, size);
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *, uint8_t *data, uint16_t size, uint32_t timeout) {
	uartState.rxBlockingCount++;
	uartState.lastTimeout = timeout;
	uartState.lastRxSize = size;
	if (uartState.rxData.size() < size) {
		return HAL_TIMEOUT;
	}
	for (uint16_t i = 0; i < size; i++) {
		data[i] = (uint8_t)uartState.rxData[i];
	}
	uartState.rxData.erase(0, size);
	return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size) {
	uartState.rxArmedCount++;
	return armUartRx(huart, data, size);
}
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size) {
	uartState.rxDmaArmedCount++;
	return armUartRx(huart, data, size);
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *) {
	uartState.abortRxCount++;
	if (uartState.abortResult == HAL_OK) {
		uartRxBuffer = nullptr; // Disarmed
		uartRxHandle = nullptr;
		uartRxCount = 0;
	}
	return (HAL_StatusTypeDef)uartState.abortResult;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *) {
	uartState.abortTxCount++;
	return (HAL_StatusTypeDef)uartState.abortResult;
}
uint32_t HAL_UART_GetError(UART_HandleTypeDef *huart) { return huart->ErrorCode; }

} // extern "C"

// ---------------- LEDs (ledEffects.h) ----------------
#ifndef HOSTTEST_REAL_LED_EFFECTS
void blinkLed(Ledstruct_t *, uint16_t, uint16_t) {}
void pulseSysLed() { ledState.sysPulses++; }
void pulseErrLed() { ledState.errPulses++; }
void pulseClipLed() { ledState.clipPulses++; }
void blinkSysLed(uint16_t, uint16_t) {}
void blinkErrLed(uint16_t, uint16_t) {}
void blinkClipLed(uint16_t, uint16_t) {}
void updateLed(Ledstruct_t *) {}
void updateLeds() {}
void setLed(Ledstruct_t *, uint8_t) {}
void setClipLed(uint8_t on) { ledState.clipLed = on; }
void setErrLed(uint8_t on) { ledState.errLed = on; }
void setSysLed(uint8_t on) { ledState.sysLed = on; }
#else
// ffb_target_tests: real ledEffects.cpp, compiled with these six functions renamed (see CMakeLists.txt). The public
// names count the calls (HostPlatform::leds(), used by the existing tests) and call the real functions.
void hosttest_real_pulseSysLed();
void hosttest_real_pulseErrLed();
void hosttest_real_pulseClipLed();
void hosttest_real_setSysLed(uint8_t on);
void hosttest_real_setErrLed(uint8_t on);
void hosttest_real_setClipLed(uint8_t on);
void pulseSysLed() {
	ledState.sysPulses++;
	hosttest_real_pulseSysLed();
}
void pulseErrLed() {
	ledState.errPulses++;
	hosttest_real_pulseErrLed();
}
void pulseClipLed() {
	ledState.clipPulses++;
	hosttest_real_pulseClipLed();
}
void setClipLed(uint8_t on) {
	ledState.clipLed = on;
	hosttest_real_setClipLed(on);
}
void setErrLed(uint8_t on) {
	ledState.errLed = on;
	hosttest_real_setErrLed(on);
}
void setSysLed(uint8_t on) {
	ledState.sysLed = on;
	hosttest_real_setSysLed(on);
}
#endif // HOSTTEST_REAL_LED_EFFECTS

// ---------------- Persistent storage (flash_helpers.h) ----------------
// Map-backed replacement of flash_helpers.cpp. ffb_flash_tests compiles the real flash_helpers.cpp and eeprom.c on
// the fake HAL FLASH (host_flash.cpp) and defines HOSTTEST_REAL_FLASH_HELPERS to leave this section out.
#ifndef HOSTTEST_REAL_FLASH_HELPERS
bool Flash_Init() { return true; }

bool Flash_Write(uint16_t adr, uint16_t dat) {
	HostPlatform::flash()[adr] = dat;
	return true;
}

bool Flash_Read(uint16_t adr, uint16_t *buf, bool) {
	auto it = HostPlatform::flash().find(adr);
	if (it == HostPlatform::flash().end()) {
		return false;
	}
	*buf = it->second;
	return true;
}

bool Flash_ReadWriteDefault(uint16_t adr, uint16_t *buf, uint16_t def) {
	if (!Flash_Read(adr, buf)) {
		Flash_Write(adr, def);
		*buf = def;
		return false;
	}
	return true;
}

void Flash_Dump(std::vector<std::tuple<uint16_t, uint16_t>> *result, bool) {
	for (const auto &entry : HostPlatform::flash()) {
		result->emplace_back(entry.first, entry.second);
	}
}

bool Flash_Format() {
	HostPlatform::flash().clear();
	return true;
}

void Flash_Write_Defaults() {}
#ifndef HOSTTEST_TARGET_OTP
bool OTP_Write(uint16_t, uint64_t) { return false; }
bool OTP_Read(uint16_t, uint64_t *) { return false; }
#endif // HOSTTEST_TARGET_OTP: ffb_target_tests models the OTP area (host_target_system.cpp)
#endif // HOSTTEST_REAL_FLASH_HELPERS
