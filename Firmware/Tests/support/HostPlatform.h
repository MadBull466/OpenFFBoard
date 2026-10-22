/*
 * HostPlatform.h
 *
 * Test control interface of the fake hardware layer (platform/src/host_platform.cpp):
 * simulated time, interrupt context, flash storage, status LEDs and the external UART.
 */
#ifndef HOSTTEST_HOSTPLATFORM_H_
#define HOSTTEST_HOSTPLATFORM_H_

#include <cstdint>
#include <map>
#include <string>

namespace HostPlatform {

/** Resets time to 0, leaves interrupt context, clears the flash, LED counters and UART buffers */
void reset();

// --- Simulated time. HAL_GetTick(), micros() and the RTOS tick are derived from it ---
uint64_t nowUs();
void setTimeUs(uint64_t us);
void advanceUs(uint64_t us);
inline void setTimeMs(uint64_t ms) { setTimeUs(ms * 1000); }
inline void advanceMs(uint64_t ms) { advanceUs(ms * 1000); }

/** Simulates interrupt context. inIsr() returns true while set */
void setInIsr(bool isr);

/** RAII helper to execute a block in simulated interrupt context */
struct IsrContext {
	IsrContext() { setInIsr(true); }
	~IsrContext() { setInIsr(false); }
};

/** Emulated EEPROM content used by Flash_Read/Flash_Write. address -> value */
std::map<uint16_t, uint16_t> &flash();

struct LedState {
	uint32_t sysPulses = 0;
	uint32_t errPulses = 0;
	uint32_t clipPulses = 0;
	bool clipLed = false;
	bool errLed = false;
	bool sysLed = false;
};
LedState &leds();

struct UartState {
	std::string tx;			   //!< All bytes transmitted via the HAL uart functions
	uint32_t txTransfers = 0;  //!< Amount of transmit calls
	uint32_t rxArmedCount = 0; //!< Amount of HAL_UART_Receive_IT calls
	uint32_t initCount = 0;	   //!< Amount of HAL_UART_Init calls
	uint32_t rxLost = 0;	   //!< Bytes received while no receive transfer was armed

	// --- Per function details (added for the UART port tests) ---
	uint32_t txBlockingCount = 0; //!< Amount of HAL_UART_Transmit calls
	uint32_t txItCount = 0;		  //!< Amount of HAL_UART_Transmit_IT calls
	uint32_t txDmaCount = 0;	  //!< Amount of HAL_UART_Transmit_DMA calls
	uint32_t rxBlockingCount = 0; //!< Amount of HAL_UART_Receive calls
	uint32_t rxDmaArmedCount = 0; //!< Amount of HAL_UART_Receive_DMA calls
	uint32_t abortRxCount = 0;	  //!< Amount of HAL_UART_AbortReceive calls
	uint32_t abortTxCount = 0;	  //!< Amount of HAL_UART_AbortTransmit calls
	uint32_t lastTimeout = 0;	  //!< Timeout of the last blocking transmit or receive
	uint16_t lastRxSize = 0;	  //!< Requested size of the last receive call (any mode)
	std::string rxData;			  //!< Bytes returned by the blocking HAL_UART_Receive (consumed). Not enough data: HAL_TIMEOUT

	// --- Injected return values (HAL_StatusTypeDef values, 0 = HAL_OK). Defaults keep the original behaviour ---
	uint32_t initResult = 0;	//!< HAL_UART_Init
	uint32_t txResult = 0;		//!< HAL_UART_Transmit, _IT and _DMA. Data is only recorded if HAL_OK
	uint32_t rxArmResult = 0;	//!< HAL_UART_Receive_IT and _DMA. The buffer is only armed if HAL_OK
	uint32_t abortResult = 0;	//!< HAL_UART_AbortReceive and HAL_UART_AbortTransmit
};
UartState &uart();

/**
 * Simulates bytes received on the uart whose receive transfer is armed.
 * Every byte is stored in the buffer armed by HAL_UART_Receive_IT (or _DMA) and the receive
 * complete callback of all UartHandlers is called in interrupt context like on the target.
 * Bytes are lost if the firmware did not rearm the receive interrupt.
 */
void uartReceive(const std::string &data);

/** Simulates the transmit complete interrupt of the external uart. Releases the port for the next transfer */
void uartCompleteTx();

} // namespace HostPlatform

#endif
