/*
 * host_target_bus.cpp
 *
 * Fake HAL SPI (F4) and fake HAL I2C (F5) of the F407 target for ffb_target_tests (declarations in
 * platform/target_f407/main.h, test control in support/HostTarget.h).
 *
 * Every fake models the documented behaviour of the real function. The reference is the ST HAL source in
 * Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c and stm32f4xx_hal_i2c.c (read only), with the
 * configuration of Targets/F407VG/Core (main.c MX_SPIx_Init / MX_I2C1_Init, stm32f4xx_hal_msp.c DMA links,
 * stm32f4xx_hal_conf.h: USE_SPI_CRC 0, no register callbacks), cited per function.
 *
 * Not modelled (recorded in HostTarget::state().unmodelledHalCalls, the call returns HAL_ERROR): SPI slave mode,
 * 1 line and rx only directions, 16 bit frames; I2C 10 bit addressing, transfers of 0 bytes; HAL_SPI_Abort / DMA
 * pause, I2C sequential and slave transfers (not declared: the firmware does not use them).
 * Timing is not modelled: blocking calls take no simulated time, timeouts are injected (spiFailNextBlocking,
 * i2cFailNextBlocking, i2cSetBusBusy).
 */
#include "HostTarget.h"
#include "HostPlatform.h"

#include "main.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <optional>
#include <string>

namespace {

void unmodelled(const std::string &what) { HostTarget::state().unmodelledHalCalls.push_back(what); }

/** __HAL_LOCK (stm32f4xx_hal_def.h): true (= return HAL_BUSY) if the handle is already locked, otherwise locks it */
template <class H>
bool halLock(H *h) {
	if (h->Lock == HAL_LOCKED) {
		return true;
	}
	h->Lock = HAL_LOCKED;
	return false;
}
template <class H>
void halUnlock(H *h) {
	h->Lock = HAL_UNLOCKED;
}

// =====================================================================================================================
// SPI
// =====================================================================================================================

/** Completion callback of a pending transfer (decided by the HAL when the transfer starts) */
enum class SpiCallback { Tx, Rx, TxRx };

struct SpiPending {
	size_t record = 0;			//!< Index in SpiFake::transfers
	const uint8_t *txBuffer = nullptr; //!< pTxBuffPtr: read when the transfer ends (the DMA / TXE ISR reads it during the transfer)
	uint8_t *rxBuffer = nullptr; //!< pRxBuffPtr (nullptr for transmit)
	SpiCallback callback = SpiCallback::Tx;
};

struct SpiFake {
	std::deque<std::vector<uint8_t>> rxQueue;
	std::vector<uint8_t> idleRx;
	HostTarget::SpiResponder responder;
	std::deque<HostTarget::SpiTransfer> transfers; //!< deque: references stay valid when a callback or hook starts another transfer
	std::optional<SpiPending> pending;
	std::optional<HAL_StatusTypeDef> failNextBlocking;
	bool failNextDmaStart = false;
	uint32_t initCount = 0;
	std::vector<std::pair<GPIO_TypeDef *, uint16_t>> csPins;
};
std::map<const SPI_HandleTypeDef *, SpiFake> spis;

SpiFake &spiFake(const SPI_HandleTypeDef *hspi) { return spis[hspi]; }

std::vector<bool> csLevels(const SpiFake &fake) {
	std::vector<bool> levels;
	for (const auto &pin : fake.csPins) {
		levels.push_back(HostTarget::outputPin(pin.first, pin.second));
	}
	return levels;
}

/** Bytes on MISO for a transfer: queued response, responder, idle response; padded with 0x00, cut to the size */
std::vector<uint8_t> spiResponse(SpiFake &fake, const HostTarget::SpiTransfer &t) {
	std::vector<uint8_t> rx;
	if (!fake.rxQueue.empty()) {
		rx = fake.rxQueue.front();
		fake.rxQueue.pop_front();
	} else if (fake.responder) {
		rx = fake.responder(t);
	} else {
		rx = fake.idleRx;
	}
	rx.resize(t.size, 0x00);
	return rx;
}

/** The configurations the fake models (see file comment). Records the call otherwise */
bool spiModelled(const SPI_HandleTypeDef *hspi, const char *function) {
	if (hspi->Init.Mode != SPI_MODE_MASTER || hspi->Init.Direction != SPI_DIRECTION_2LINES || hspi->Init.DataSize != SPI_DATASIZE_8BIT) {
		unmodelled(std::string(function) + ": only master, 2 lines, 8 bit is modelled");
		return false;
	}
	return true;
}

/** New record of a call (result filled in by the caller) */
HostTarget::SpiTransfer &spiRecord(SPI_HandleTypeDef *hspi, HostTarget::SpiCall call, HostTarget::BusMode mode, uint16_t size, uint32_t timeout) {
	SpiFake &fake = spiFake(hspi);
	HostTarget::SpiTransfer t;
	t.call = call;
	t.mode = mode;
	t.size = size;
	t.timeout = timeout;
	t.init = hspi->Init;
	t.csAtStart = csLevels(fake);
	fake.transfers.push_back(t);
	return fake.transfers.back();
}

/** A call rejected before anything happens (HAL_BUSY / HAL_ERROR checks at the start of the HAL functions) */
HAL_StatusTypeDef spiReject(SPI_HandleTypeDef *hspi, HostTarget::SpiCall call, HostTarget::BusMode mode, uint16_t size, uint32_t timeout, HAL_StatusTypeDef status) {
	HostTarget::SpiTransfer &t = spiRecord(hspi, call, mode, size, timeout);
	t.result = status;
	t.csAtEnd = t.csAtStart;
	t.errorCode = hspi->ErrorCode;
	return status;
}

/**
 * The data phase of a blocking transfer (HAL_SPI_Transmit / HAL_SPI_TransmitReceive after the checks and __HAL_LOCK):
 * State and ErrorCode set by the caller. Injected failures, data, State READY, unlock.
 */
HAL_StatusTypeDef spiBlocking(SPI_HandleTypeDef *hspi, HostTarget::SpiCall call, const uint8_t *txData, uint8_t *rxData, uint16_t size, uint32_t timeout) {
	SpiFake &fake = spiFake(hspi);
	HostTarget::SpiTransfer &t = spiRecord(hspi, call, HostTarget::BusMode::Blocking, size, timeout);
	std::optional<HAL_StatusTypeDef> fail = fake.failNextBlocking;
	fake.failNextBlocking.reset();
	if (fail == HAL_TIMEOUT) {
		// Timeout while waiting for TXE / RXNE: State READY, unlocked, HAL_TIMEOUT; no data recorded
		hspi->State = HAL_SPI_STATE_READY;
		halUnlock(hspi);
		t.result = HAL_TIMEOUT;
		t.csAtEnd = csLevels(fake);
		t.errorCode = hspi->ErrorCode;
		return HAL_TIMEOUT;
	}
	t.tx.assign(txData, txData + size); // Read before the rx data is written: Receive uses the same buffer
	t.rx = spiResponse(fake, t);
	if (rxData != nullptr) {
		std::memcpy(rxData, t.rx.data(), size);
	}
	if (fail == HAL_ERROR) {
		hspi->ErrorCode = HAL_SPI_ERROR_FLAG; // SPI_EndRxTxTransaction timed out
	}
	hspi->State = HAL_SPI_STATE_READY;
	halUnlock(hspi);
	t.csAtEnd = csLevels(fake);
	t.errorCode = hspi->ErrorCode;
	t.result = hspi->ErrorCode != HAL_SPI_ERROR_NONE ? HAL_ERROR : HAL_OK;
	t.completed = t.result == HAL_OK;
	return t.result;
}

/**
 * Start of an IT/DMA transfer after the checks, __HAL_LOCK and the State update of the HAL function: records the tx
 * bytes, arms the pending transfer, unlocks. DMA start failure injection like HAL_SPI_Transmit_DMA & co.
 */
HAL_StatusTypeDef spiStartAsync(SPI_HandleTypeDef *hspi, HostTarget::SpiCall call, HostTarget::BusMode mode, const uint8_t *txData, uint8_t *rxData, uint16_t size,
								SpiCallback callback) {
	SpiFake &fake = spiFake(hspi);
	HostTarget::SpiTransfer &t = spiRecord(hspi, call, mode, size, 0);
	hspi->ErrorCode = HAL_SPI_ERROR_NONE;
	if (mode == HostTarget::BusMode::Dma && fake.failNextDmaStart) {
		fake.failNextDmaStart = false;
		hspi->ErrorCode |= HAL_SPI_ERROR_DMA; // HAL_DMA_Start_IT failed. State is not restored by the HAL
		halUnlock(hspi);
		t.result = HAL_ERROR;
		t.csAtEnd = t.csAtStart;
		t.errorCode = hspi->ErrorCode;
		return HAL_ERROR;
	}
	// The tx bytes that count are read from the buffer when the transfer ends (spiComplete / spiFail): the DMA stream
	// or the TXE interrupt reads pTxBuffPtr during the transfer, so a buffer changed before the end goes out changed.
	// Until then t.tx shows the buffer content at the start (preview of the pending transfer)
	if (txData != nullptr) {
		t.tx.assign(txData, txData + size);
	}
	fake.pending = SpiPending{fake.transfers.size() - 1, txData, rxData, callback};
	halUnlock(hspi);
	t.result = HAL_OK;
	return HAL_OK;
}

} // namespace

extern "C" {

/**
 * Models HAL_SPI_Init (stm32f4xx_hal_spi.c): HAL_ERROR for a null handle; State RESET -> Lock UNLOCKED (MspInit);
 * State BUSY, the peripheral is disabled (a pending transfer stops: it never completes) and configured, CRC forced off
 * (USE_SPI_CRC 0), ErrorCode NONE, State READY, HAL_OK. Counted (spiInitCount).
 */
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi) {
	if (hspi == nullptr) {
		return HAL_ERROR;
	}
	SpiFake &fake = spiFake(hspi);
	fake.initCount++;
	hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	if (hspi->State == HAL_SPI_STATE_RESET) {
		hspi->Lock = HAL_UNLOCKED;
	}
	hspi->State = HAL_SPI_STATE_BUSY;
	fake.pending.reset(); // __HAL_SPI_DISABLE: SPE cleared, an ongoing transfer stops
	hspi->ErrorCode = HAL_SPI_ERROR_NONE;
	hspi->State = HAL_SPI_STATE_READY;
	return HAL_OK;
}

/**
 * Models HAL_SPI_Transmit: HAL_BUSY unless State READY, HAL_ERROR for no data, __HAL_LOCK, State BUSY_TX, bytes sent
 * (received bytes discarded, OVR flag cleared), State READY, unlock, HAL_OK (HAL_ERROR if ErrorCode is set).
 */
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	using HostTarget::BusMode;
	using HostTarget::SpiCall;
	if (hspi->State != HAL_SPI_STATE_READY) {
		return spiReject(hspi, SpiCall::Transmit, BusMode::Blocking, Size, Timeout, HAL_BUSY);
	}
	if (pData == nullptr || Size == 0U) {
		return spiReject(hspi, SpiCall::Transmit, BusMode::Blocking, Size, Timeout, HAL_ERROR);
	}
	if (!spiModelled(hspi, "HAL_SPI_Transmit")) {
		return HAL_ERROR;
	}
	if (halLock(hspi)) {
		return spiReject(hspi, SpiCall::Transmit, BusMode::Blocking, Size, Timeout, HAL_BUSY);
	}
	hspi->State = HAL_SPI_STATE_BUSY_TX;
	hspi->ErrorCode = HAL_SPI_ERROR_NONE;
	return spiBlocking(hspi, SpiCall::Transmit, pData, nullptr, Size, Timeout);
}

/**
 * Models HAL_SPI_TransmitReceive: HAL_BUSY unless State READY (or BUSY_RX when called by HAL_SPI_Receive as master
 * in 2 line mode), HAL_ERROR for missing buffers, __HAL_LOCK, State BUSY_TX_RX (BUSY_RX kept), full duplex transfer,
 * State READY, unlock.
 */
static HAL_StatusTypeDef spiTransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout, HostTarget::SpiCall call) {
	using HostTarget::BusMode;
	const bool fromReceive = hspi->State == HAL_SPI_STATE_BUSY_RX && hspi->Init.Mode == SPI_MODE_MASTER && hspi->Init.Direction == SPI_DIRECTION_2LINES;
	if (hspi->State != HAL_SPI_STATE_READY && !fromReceive) {
		return spiReject(hspi, call, BusMode::Blocking, Size, Timeout, HAL_BUSY);
	}
	if (pTxData == nullptr || pRxData == nullptr || Size == 0U) {
		return spiReject(hspi, call, BusMode::Blocking, Size, Timeout, HAL_ERROR); // State stays BUSY_RX if set by HAL_SPI_Receive (HAL)
	}
	if (!spiModelled(hspi, "HAL_SPI_TransmitReceive")) {
		return HAL_ERROR;
	}
	if (halLock(hspi)) {
		return spiReject(hspi, call, BusMode::Blocking, Size, Timeout, HAL_BUSY);
	}
	if (hspi->State != HAL_SPI_STATE_BUSY_RX) {
		hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
	}
	hspi->ErrorCode = HAL_SPI_ERROR_NONE;
	return spiBlocking(hspi, call, pTxData, pRxData, Size, Timeout);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
	return spiTransmitReceive(hspi, pTxData, pRxData, Size, Timeout, HostTarget::SpiCall::TransmitReceive);
}

/**
 * Models HAL_SPI_Receive: HAL_BUSY unless State READY; as master in 2 line mode State BUSY_RX and
 * HAL_SPI_TransmitReceive(pData, pData): the buffer content is clocked out as dummy bytes.
 */
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	if (hspi->State != HAL_SPI_STATE_READY) {
		return spiReject(hspi, HostTarget::SpiCall::Receive, HostTarget::BusMode::Blocking, Size, Timeout, HAL_BUSY);
	}
	if (!spiModelled(hspi, "HAL_SPI_Receive")) {
		return HAL_ERROR;
	}
	hspi->State = HAL_SPI_STATE_BUSY_RX;
	return spiTransmitReceive(hspi, pData, pData, Size, Timeout, HostTarget::SpiCall::Receive);
}

/** Models HAL_SPI_Transmit_IT: HAL_ERROR for no data (checked first), HAL_BUSY unless READY, lock, State BUSY_TX, TXE/ERR interrupts on */
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size) {
	using HostTarget::BusMode;
	using HostTarget::SpiCall;
	if (pData == nullptr || Size == 0U) {
		return spiReject(hspi, SpiCall::Transmit, BusMode::Interrupt, Size, 0, HAL_ERROR);
	}
	if (hspi->State != HAL_SPI_STATE_READY) {
		return spiReject(hspi, SpiCall::Transmit, BusMode::Interrupt, Size, 0, HAL_BUSY);
	}
	if (!spiModelled(hspi, "HAL_SPI_Transmit_IT")) {
		return HAL_ERROR;
	}
	if (halLock(hspi)) {
		return spiReject(hspi, SpiCall::Transmit, BusMode::Interrupt, Size, 0, HAL_BUSY);
	}
	hspi->State = HAL_SPI_STATE_BUSY_TX;
	return spiStartAsync(hspi, SpiCall::Transmit, BusMode::Interrupt, pData, nullptr, Size, SpiCallback::Tx);
}

/**
 * Models HAL_SPI_TransmitReceive_IT / _DMA: HAL_BUSY unless READY (or BUSY_RX from HAL_SPI_Receive_IT/_DMA as master
 * in 2 line mode), HAL_ERROR for missing buffers, lock, State BUSY_TX_RX (BUSY_RX kept). The completion callback is
 * HAL_SPI_RxCpltCallback when started as a receive (SPI_CloseRxTx_ISR checks BUSY_RX; HAL_SPI_TransmitReceive_DMA
 * selects SPI_DMAReceiveCplt), HAL_SPI_TxRxCpltCallback otherwise.
 */
static HAL_StatusTypeDef spiTransmitReceiveAsync(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, HostTarget::BusMode mode, HostTarget::SpiCall call,
												 const char *function) {
	const bool fromReceive = hspi->State == HAL_SPI_STATE_BUSY_RX && hspi->Init.Mode == SPI_MODE_MASTER && hspi->Init.Direction == SPI_DIRECTION_2LINES;
	if (hspi->State != HAL_SPI_STATE_READY && !fromReceive) {
		return spiReject(hspi, call, mode, Size, 0, HAL_BUSY);
	}
	if (pTxData == nullptr || pRxData == nullptr || Size == 0U) {
		return spiReject(hspi, call, mode, Size, 0, HAL_ERROR); // State stays BUSY_RX if set by the receive function (HAL)
	}
	if (!spiModelled(hspi, function)) {
		return HAL_ERROR;
	}
	if (halLock(hspi)) {
		return spiReject(hspi, call, mode, Size, 0, HAL_BUSY);
	}
	if (hspi->State != HAL_SPI_STATE_BUSY_RX) {
		hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
	}
	return spiStartAsync(hspi, call, mode, pTxData, pRxData, Size, hspi->State == HAL_SPI_STATE_BUSY_RX ? SpiCallback::Rx : SpiCallback::TxRx);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size) {
	return spiTransmitReceiveAsync(hspi, pTxData, pRxData, Size, HostTarget::BusMode::Interrupt, HostTarget::SpiCall::TransmitReceive, "HAL_SPI_TransmitReceive_IT");
}

/** Models HAL_SPI_Receive_IT: HAL_BUSY unless READY; as master in 2 line mode State BUSY_RX and HAL_SPI_TransmitReceive_IT(pData, pData) */
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size) {
	if (hspi->State != HAL_SPI_STATE_READY) {
		return spiReject(hspi, HostTarget::SpiCall::Receive, HostTarget::BusMode::Interrupt, Size, 0, HAL_BUSY);
	}
	if (!spiModelled(hspi, "HAL_SPI_Receive_IT")) {
		return HAL_ERROR;
	}
	hspi->State = HAL_SPI_STATE_BUSY_RX;
	return spiTransmitReceiveAsync(hspi, pData, pData, Size, HostTarget::BusMode::Interrupt, HostTarget::SpiCall::Receive, "HAL_SPI_Receive_IT");
}

/** Models HAL_SPI_Transmit_DMA: HAL_BUSY unless READY, HAL_ERROR for no data, lock, State BUSY_TX, tx DMA started (completion: SPI_DMATransmitCplt) */
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size) {
	using HostTarget::BusMode;
	using HostTarget::SpiCall;
	if (hspi->State != HAL_SPI_STATE_READY) {
		return spiReject(hspi, SpiCall::Transmit, BusMode::Dma, Size, 0, HAL_BUSY);
	}
	if (pData == nullptr || Size == 0U) {
		return spiReject(hspi, SpiCall::Transmit, BusMode::Dma, Size, 0, HAL_ERROR);
	}
	if (!spiModelled(hspi, "HAL_SPI_Transmit_DMA")) {
		return HAL_ERROR;
	}
	if (halLock(hspi)) {
		return spiReject(hspi, SpiCall::Transmit, BusMode::Dma, Size, 0, HAL_BUSY);
	}
	hspi->State = HAL_SPI_STATE_BUSY_TX;
	return spiStartAsync(hspi, SpiCall::Transmit, BusMode::Dma, pData, nullptr, Size, SpiCallback::Tx);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size) {
	return spiTransmitReceiveAsync(hspi, pTxData, pRxData, Size, HostTarget::BusMode::Dma, HostTarget::SpiCall::TransmitReceive, "HAL_SPI_TransmitReceive_DMA");
}

/** Models HAL_SPI_Receive_DMA: HAL_BUSY unless READY; as master in 2 line mode State BUSY_RX and HAL_SPI_TransmitReceive_DMA(pData, pData) */
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size) {
	if (hspi->State != HAL_SPI_STATE_READY) {
		return spiReject(hspi, HostTarget::SpiCall::Receive, HostTarget::BusMode::Dma, Size, 0, HAL_BUSY);
	}
	if (!spiModelled(hspi, "HAL_SPI_Receive_DMA")) {
		return HAL_ERROR;
	}
	hspi->State = HAL_SPI_STATE_BUSY_RX;
	return spiTransmitReceiveAsync(hspi, pData, pData, Size, HostTarget::BusMode::Dma, HostTarget::SpiCall::Receive, "HAL_SPI_Receive_DMA");
}

/** Models HAL_SPI_GetState / HAL_SPI_GetError */
HAL_SPI_StateTypeDef HAL_SPI_GetState(const SPI_HandleTypeDef *hspi) { return hspi->State; }
uint32_t HAL_SPI_GetError(const SPI_HandleTypeDef *hspi) { return hspi->ErrorCode; }

} // extern "C"

// =====================================================================================================================
// I2C
// =====================================================================================================================
namespace {

struct I2cPending {
	size_t record = 0;
	const uint8_t *writeData = nullptr; //!< pBuffPtr of a write: read when the transfer ends (the ISR / DMA reads it during the transfer)
	uint8_t *buffer = nullptr;			//!< pBuffPtr of a read
};

struct I2cFake {
	std::map<uint8_t, HostTarget::I2cDevice> devices;
	std::deque<HostTarget::I2cTransfer> transfers;
	std::optional<I2cPending> pending;
	bool busBusy = false;
	bool dmaLinked = false;
	std::optional<std::pair<HAL_StatusTypeDef, uint32_t>> failNextBlocking;
	uint32_t initCount = 0;
	uint32_t deInitCount = 0;
};
std::map<const I2C_HandleTypeDef *, I2cFake> i2cs;

I2cFake &i2cFake(const I2C_HandleTypeDef *hi2c) { return i2cs[hi2c]; }

/** The device at the address, nullptr if none (no device acknowledges) */
HostTarget::I2cDevice *i2cFind(I2cFake &fake, uint8_t address) {
	auto it = fake.devices.find(address);
	return it == fake.devices.end() ? nullptr : &it->second;
}

/** Address phase: true if a device acknowledges (consumes one busy NACK of nackAddressCount) */
bool i2cAddressAck(HostTarget::I2cDevice *dev) {
	if (dev == nullptr || dev->nackAddress) {
		return false;
	}
	if (dev->nackAddressCount > 0) {
		dev->nackAddressCount--;
		return false;
	}
	return true;
}

/** Register byte access of the register model: byte `offset` of the register at the pointer */
struct RegisterCursor {
	HostTarget::I2cDevice &dev;
	uint16_t offset = 0;
	void advance() {
		if (++offset >= std::max<uint8_t>(dev.registerWidth, 1)) {
			offset = 0;
			if (dev.autoIncrement) {
				dev.pointer++;
			}
		}
	}
	uint8_t read() {
		auto it = dev.registers.find(dev.pointer);
		uint8_t value = 0xFF;
		if (it != dev.registers.end() && offset < it->second.size()) {
			value = it->second[offset];
		}
		advance();
		return value;
	}
	void write(uint8_t value) {
		std::vector<uint8_t> &reg = dev.registers[dev.pointer];
		if (reg.size() < std::max<uint8_t>(dev.registerWidth, 1)) {
			reg.resize(std::max<uint8_t>(dev.registerWidth, 1), 0xFF);
		}
		reg[offset] = value;
		advance();
	}
};

/** Reads `size` bytes from the device (scripted response first, then the register / plain model) */
std::vector<uint8_t> i2cDeviceRead(HostTarget::I2cDevice &dev, uint16_t size) {
	std::vector<uint8_t> data;
	if (!dev.readQueue.empty()) {
		data = dev.readQueue.front();
		dev.readQueue.pop_front();
		data.resize(size, 0xFF);
		return data;
	}
	if (!dev.pointerRegister) {
		std::vector<uint8_t> pins = dev.registers.count(0) ? dev.registers[0] : std::vector<uint8_t>{0xFF};
		if (pins.empty()) {
			pins.push_back(0xFF);
		}
		for (uint16_t i = 0; i < size; i++) {
			data.push_back(pins[i % pins.size()]);
		}
		return data;
	}
	RegisterCursor cursor{dev};
	for (uint16_t i = 0; i < size; i++) {
		data.push_back(cursor.read());
	}
	return data;
}

/**
 * Write phase after an acknowledged address: `pointerBytes` bytes set the register pointer (Mem_* memory address or
 * the first bytes of a Master_Transmit in the register model), the rest are data. Stops at the NACKed byte
 * (nackWriteByte). @return false if a byte was not acknowledged. `acked` receives the acknowledged data bytes.
 */
bool i2cDeviceWrite(HostTarget::I2cDevice &dev, const std::vector<uint8_t> &bytes, size_t pointerBytes, std::vector<uint8_t> &acked) {
	uint16_t pointer = 0;
	RegisterCursor cursor{dev};
	for (size_t i = 0; i < bytes.size(); i++) {
		if (dev.nackWriteByte >= 0 && static_cast<size_t>(dev.nackWriteByte) == i) {
			return false;
		}
		if (i < pointerBytes) {
			pointer = static_cast<uint16_t>((pointer << 8) | bytes[i]);
			if (i + 1 == pointerBytes) {
				dev.pointer = pointer;
				cursor.offset = 0;
			}
			continue;
		}
		acked.push_back(bytes[i]);
		if (dev.pointerRegister) {
			cursor.write(bytes[i]);
		}
	}
	return true;
}

/** Bytes of a memory address in bus order (I2C_RequestMemoryWrite / _Read: MSB first for 16 bit) */
std::vector<uint8_t> memAddressBytes(uint16_t memAddress, uint16_t memAddSize) {
	if (memAddSize == 2) {
		return {static_cast<uint8_t>(memAddress >> 8), static_cast<uint8_t>(memAddress)};
	}
	return {static_cast<uint8_t>(memAddress)};
}

uint16_t memAddBytes(uint16_t memAddSize) { return memAddSize == I2C_MEMADD_SIZE_16BIT ? 2 : 1; }

/**
 * Executes a transfer on the bus (blocking call or completion of an IT/DMA transfer): address phase, data phase,
 * the device hook. Fills t.data / t.acknowledged. @return HAL_I2C_ERROR_NONE or HAL_I2C_ERROR_AF
 */
uint32_t i2cExecute(I2cFake &fake, HostTarget::I2cTransfer &t, const uint8_t *writeData, uint8_t *readBuffer) {
	using HostTarget::I2cCall;
	HostTarget::I2cDevice *dev = i2cFind(fake, t.address);
	uint32_t error = HAL_I2C_ERROR_NONE;
	t.acknowledged = i2cAddressAck(dev);
	if (!t.acknowledged) {
		error = HAL_I2C_ERROR_AF;
	} else {
		switch (t.call) {
		case I2cCall::MasterTransmit: {
			std::vector<uint8_t> bytes(writeData, writeData + t.size);
			if (!i2cDeviceWrite(*dev, bytes, dev->pointerRegister ? dev->pointerSize : 0, t.data)) {
				error = HAL_I2C_ERROR_AF;
			}
			break;
		}
		case I2cCall::MemWrite: {
			std::vector<uint8_t> bytes = memAddressBytes(t.memAddress, t.memAddSize);
			bytes.insert(bytes.end(), writeData, writeData + t.size);
			if (!i2cDeviceWrite(*dev, bytes, t.memAddSize, t.data)) {
				error = HAL_I2C_ERROR_AF;
			}
			break;
		}
		case I2cCall::MasterReceive:
			t.data = i2cDeviceRead(*dev, t.size);
			break;
		case I2cCall::MemRead: {
			std::vector<uint8_t> ignored;
			if (!i2cDeviceWrite(*dev, memAddressBytes(t.memAddress, t.memAddSize), t.memAddSize, ignored)) {
				error = HAL_I2C_ERROR_AF; // Memory address not acknowledged: no repeated start, no data
				break;
			}
			// Repeated start with the read address: acknowledged by the same device
			t.data = i2cDeviceRead(*dev, t.size);
			break;
		}
		case I2cCall::IsDeviceReady:
			break;
		}
	}
	if (readBuffer != nullptr && !t.data.empty() && (t.call == I2cCall::MasterReceive || t.call == I2cCall::MemRead)) {
		std::memcpy(readBuffer, t.data.data(), t.data.size());
	}
	if (dev != nullptr && dev->onTransfer) {
		dev->onTransfer(*dev, t);
	}
	return error;
}

HostTarget::I2cTransfer &i2cRecord(I2C_HandleTypeDef *hi2c, HostTarget::I2cCall call, HostTarget::BusMode mode, uint16_t devAddress, uint16_t size, uint32_t timeout) {
	I2cFake &fake = i2cFake(hi2c);
	HostTarget::I2cTransfer t;
	t.call = call;
	t.mode = mode;
	t.devAddress = devAddress;
	t.address = static_cast<uint8_t>(devAddress >> 1); // I2C_7BIT_ADD_WRITE / _READ use bits 7..1
	t.size = size;
	t.timeout = timeout;
	fake.transfers.push_back(t);
	return fake.transfers.back();
}

/**
 * Checks at the start of every master / memory function: State READY (else HAL_BUSY), BUSY flag (i2cSetBusBusy:
 * HAL_BUSY, ErrorCode |= TIMEOUT, State READY, Mode NONE), modelled configuration, __HAL_LOCK (HAL_BUSY).
 * @return HAL_OK to continue, else the status to return (the call is recorded)
 */
HAL_StatusTypeDef i2cBegin(I2C_HandleTypeDef *hi2c, HostTarget::I2cTransfer &t, const char *function) {
	I2cFake &fake = i2cFake(hi2c);
	if (hi2c->State != HAL_I2C_STATE_READY) {
		t.result = HAL_BUSY;
	} else if (fake.busBusy) {
		hi2c->State = HAL_I2C_STATE_READY;
		hi2c->Mode = HAL_I2C_MODE_NONE;
		hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
		t.result = HAL_BUSY;
	} else if (hi2c->Init.AddressingMode != I2C_ADDRESSINGMODE_7BIT || (t.size == 0 && t.call != HostTarget::I2cCall::IsDeviceReady)) {
		unmodelled(std::string(function) + ": only 7 bit addressing and transfers of at least 1 byte are modelled");
		t.result = HAL_ERROR;
	} else if (halLock(hi2c)) {
		t.result = HAL_BUSY;
	} else {
		t.result = HAL_OK;
	}
	t.errorCode = hi2c->ErrorCode;
	return t.result;
}

/** End of a failed transfer (address / data NACK, injected timeout): State READY, Mode NONE, ErrorCode |= error, unlock */
void i2cEndWithError(I2C_HandleTypeDef *hi2c, uint32_t error) {
	hi2c->State = HAL_I2C_STATE_READY;
	hi2c->Mode = HAL_I2C_MODE_NONE;
	hi2c->ErrorCode |= error;
	halUnlock(hi2c);
}

/**
 * Blocking master / memory transfer after i2cBegin (HAL_I2C_Master_Transmit, _Receive, HAL_I2C_Mem_Write, _Read):
 * State BUSY_TX / BUSY_RX, Mode MASTER / MEM, ErrorCode NONE; injected failure, bus transfer; on a NACK HAL_ERROR with
 * AF (and STOP); State READY, Mode NONE, unlock, HAL_OK.
 */
HAL_StatusTypeDef i2cBlocking(I2C_HandleTypeDef *hi2c, HostTarget::I2cTransfer &t, const uint8_t *writeData, uint8_t *readBuffer, HAL_I2C_StateTypeDef busyState, HAL_I2C_ModeTypeDef mode) {
	I2cFake &fake = i2cFake(hi2c);
	hi2c->State = busyState;
	hi2c->Mode = mode;
	hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
	if (fake.failNextBlocking) {
		auto [status, error] = *fake.failNextBlocking;
		fake.failNextBlocking.reset();
		i2cEndWithError(hi2c, error);
		t.result = status;
		t.errorCode = hi2c->ErrorCode;
		return status;
	}
	uint32_t error = i2cExecute(fake, t, writeData, readBuffer);
	if (error != HAL_I2C_ERROR_NONE) {
		i2cEndWithError(hi2c, error);
		t.result = HAL_ERROR;
		t.errorCode = hi2c->ErrorCode;
		return HAL_ERROR;
	}
	hi2c->State = HAL_I2C_STATE_READY;
	hi2c->Mode = HAL_I2C_MODE_NONE;
	halUnlock(hi2c);
	t.result = HAL_OK;
	t.completed = true;
	t.errorCode = hi2c->ErrorCode;
	return HAL_OK;
}

/**
 * Start of an IT / DMA transfer after i2cBegin (HAL_I2C_Master_Transmit_IT ... HAL_I2C_Mem_Read_IT, the _DMA
 * variants): DMA without linked stream -> HAL_ERROR, DMA_PARAM, State READY (HAL_I2C_Master_Transmit_DMA: hdmatx ==
 * NULL branch); else State BUSY_TX / BUSY_RX, Mode MASTER / MEM, ErrorCode NONE, unlock, the transfer is pending.
 */
HAL_StatusTypeDef i2cStartAsync(I2C_HandleTypeDef *hi2c, HostTarget::I2cTransfer &t, const uint8_t *writeData, uint8_t *buffer, HAL_I2C_StateTypeDef busyState, HAL_I2C_ModeTypeDef mode) {
	I2cFake &fake = i2cFake(hi2c);
	hi2c->State = busyState;
	hi2c->Mode = mode;
	hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
	if (t.mode == HostTarget::BusMode::Dma && !fake.dmaLinked) {
		i2cEndWithError(hi2c, HAL_I2C_ERROR_DMA_PARAM);
		t.result = HAL_ERROR;
		t.errorCode = hi2c->ErrorCode;
		return HAL_ERROR;
	}
	// The written bytes that count are read from the buffer when the transfer ends (i2cComplete): the TXE interrupt / DMA
	// reads pBuffPtr byte by byte during the transfer, so a buffer changed before the end goes out changed. Until then
	// t.data shows the buffer content at the start (preview of the pending transfer)
	if (writeData != nullptr) {
		t.data.assign(writeData, writeData + t.size);
	}
	fake.pending = I2cPending{fake.transfers.size() - 1, writeData, buffer};
	halUnlock(hi2c);
	t.result = HAL_OK;
	t.errorCode = hi2c->ErrorCode;
	return HAL_OK;
}

HAL_StatusTypeDef i2cMaster(I2C_HandleTypeDef *hi2c, HostTarget::I2cCall call, HostTarget::BusMode mode, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout,
							const char *function) {
	HostTarget::I2cTransfer &t = i2cRecord(hi2c, call, mode, DevAddress, Size, Timeout);
	if (i2cBegin(hi2c, t, function) != HAL_OK) {
		return t.result;
	}
	const bool tx = call == HostTarget::I2cCall::MasterTransmit;
	HAL_I2C_StateTypeDef busy = tx ? HAL_I2C_STATE_BUSY_TX : HAL_I2C_STATE_BUSY_RX;
	if (mode == HostTarget::BusMode::Blocking) {
		return i2cBlocking(hi2c, t, tx ? pData : nullptr, tx ? nullptr : pData, busy, HAL_I2C_MODE_MASTER);
	}
	return i2cStartAsync(hi2c, t, tx ? pData : nullptr, tx ? nullptr : pData, busy, HAL_I2C_MODE_MASTER);
}

HAL_StatusTypeDef i2cMem(I2C_HandleTypeDef *hi2c, HostTarget::I2cCall call, HostTarget::BusMode mode, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData,
						 uint16_t Size, uint32_t Timeout, const char *function) {
	HostTarget::I2cTransfer &t = i2cRecord(hi2c, call, mode, DevAddress, Size, Timeout);
	t.memAddress = MemAddress;
	t.memAddSize = memAddBytes(MemAddSize);
	if (MemAddSize != I2C_MEMADD_SIZE_8BIT && MemAddSize != I2C_MEMADD_SIZE_16BIT) {
		unmodelled(std::string(function) + ": MemAddSize must be I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT");
		t.result = HAL_ERROR;
		return HAL_ERROR;
	}
	if (i2cBegin(hi2c, t, function) != HAL_OK) {
		return t.result;
	}
	const bool write = call == HostTarget::I2cCall::MemWrite;
	HAL_I2C_StateTypeDef busy = write ? HAL_I2C_STATE_BUSY_TX : HAL_I2C_STATE_BUSY_RX;
	if (mode == HostTarget::BusMode::Blocking) {
		return i2cBlocking(hi2c, t, write ? pData : nullptr, write ? nullptr : pData, busy, HAL_I2C_MODE_MEM);
	}
	return i2cStartAsync(hi2c, t, write ? pData : nullptr, write ? nullptr : pData, busy, HAL_I2C_MODE_MEM);
}

} // namespace

extern "C" {

/**
 * Models HAL_I2C_Init (stm32f4xx_hal_i2c.c): HAL_ERROR for a null handle; State RESET -> Lock UNLOCKED (MspInit);
 * State BUSY, peripheral reset (SWRST: a pending transfer is dropped) and configured (PCLK1 42 MHz passes
 * I2C_MIN_PCLK_FREQ), ErrorCode NONE, State READY, Mode NONE, HAL_OK. Counted (i2cInitCount).
 */
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c) {
	if (hi2c == nullptr) {
		return HAL_ERROR;
	}
	I2cFake &fake = i2cFake(hi2c);
	fake.initCount++;
	if (hi2c->State == HAL_I2C_STATE_RESET) {
		hi2c->Lock = HAL_UNLOCKED;
	}
	hi2c->State = HAL_I2C_STATE_BUSY;
	fake.pending.reset();
	hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
	hi2c->State = HAL_I2C_STATE_READY;
	hi2c->Mode = HAL_I2C_MODE_NONE;
	return HAL_OK;
}

/**
 * Models HAL_I2C_DeInit: HAL_ERROR for a null handle; State BUSY, peripheral disabled (a pending transfer is dropped),
 * MspDeInit, ErrorCode NONE, State RESET, Mode NONE, unlock, HAL_OK. Counted (i2cDeInitCount). Every transfer call
 * returns HAL_BUSY until the next HAL_I2C_Init (State is not READY).
 */
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c) {
	if (hi2c == nullptr) {
		return HAL_ERROR;
	}
	I2cFake &fake = i2cFake(hi2c);
	fake.deInitCount++;
	hi2c->State = HAL_I2C_STATE_BUSY;
	fake.pending.reset();
	hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
	hi2c->State = HAL_I2C_STATE_RESET;
	hi2c->Mode = HAL_I2C_MODE_NONE;
	halUnlock(hi2c);
	return HAL_OK;
}

/** Models HAL_I2C_Master_Transmit: see i2cBegin / i2cBlocking (address NACK and data NACK: HAL_ERROR, AF) */
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	return i2cMaster(hi2c, HostTarget::I2cCall::MasterTransmit, HostTarget::BusMode::Blocking, DevAddress, pData, Size, Timeout, "HAL_I2C_Master_Transmit");
}
/** Models HAL_I2C_Master_Receive: see i2cBegin / i2cBlocking (address NACK: HAL_ERROR, AF) */
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	return i2cMaster(hi2c, HostTarget::I2cCall::MasterReceive, HostTarget::BusMode::Blocking, DevAddress, pData, Size, Timeout, "HAL_I2C_Master_Receive");
}
/** Models HAL_I2C_Master_Transmit_IT: State BUSY_TX, Mode MASTER, pending until i2cComplete (HAL_I2C_MasterTxCpltCallback) */
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size) {
	return i2cMaster(hi2c, HostTarget::I2cCall::MasterTransmit, HostTarget::BusMode::Interrupt, DevAddress, pData, Size, 0, "HAL_I2C_Master_Transmit_IT");
}
/** Models HAL_I2C_Master_Receive_IT: State BUSY_RX, Mode MASTER, pending until i2cComplete (HAL_I2C_MasterRxCpltCallback) */
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size) {
	return i2cMaster(hi2c, HostTarget::I2cCall::MasterReceive, HostTarget::BusMode::Interrupt, DevAddress, pData, Size, 0, "HAL_I2C_Master_Receive_IT");
}
/** Models HAL_I2C_Master_Transmit_DMA: like _IT if a DMA stream is linked, else HAL_ERROR with HAL_I2C_ERROR_DMA_PARAM */
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size) {
	return i2cMaster(hi2c, HostTarget::I2cCall::MasterTransmit, HostTarget::BusMode::Dma, DevAddress, pData, Size, 0, "HAL_I2C_Master_Transmit_DMA");
}
/** Models HAL_I2C_Master_Receive_DMA: like _IT if a DMA stream is linked, else HAL_ERROR with HAL_I2C_ERROR_DMA_PARAM */
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size) {
	return i2cMaster(hi2c, HostTarget::I2cCall::MasterReceive, HostTarget::BusMode::Dma, DevAddress, pData, Size, 0, "HAL_I2C_Master_Receive_DMA");
}
/** Models HAL_I2C_Mem_Write: address, memory address (I2C_RequestMemoryWrite, MSB first), data; NACK: HAL_ERROR, AF */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	return i2cMem(hi2c, HostTarget::I2cCall::MemWrite, HostTarget::BusMode::Blocking, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout, "HAL_I2C_Mem_Write");
}
/** Models HAL_I2C_Mem_Read: address, memory address (I2C_RequestMemoryRead), repeated start, data; NACK: HAL_ERROR, AF */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	return i2cMem(hi2c, HostTarget::I2cCall::MemRead, HostTarget::BusMode::Blocking, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout, "HAL_I2C_Mem_Read");
}
/** Models HAL_I2C_Mem_Write_IT: State BUSY_TX, Mode MEM, pending until i2cComplete (HAL_I2C_MemTxCpltCallback) */
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {
	return i2cMem(hi2c, HostTarget::I2cCall::MemWrite, HostTarget::BusMode::Interrupt, DevAddress, MemAddress, MemAddSize, pData, Size, 0, "HAL_I2C_Mem_Write_IT");
}
/** Models HAL_I2C_Mem_Read_IT: State BUSY_RX, Mode MEM, pending until i2cComplete (HAL_I2C_MemRxCpltCallback) */
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {
	return i2cMem(hi2c, HostTarget::I2cCall::MemRead, HostTarget::BusMode::Interrupt, DevAddress, MemAddress, MemAddSize, pData, Size, 0, "HAL_I2C_Mem_Read_IT");
}

/**
 * Models HAL_I2C_IsDeviceReady: HAL_BUSY unless READY (or bus busy), lock, State BUSY; up to Trials address phases:
 * the first acknowledged one gives HAL_OK; all NACKed: HAL_ERROR (ErrorCode stays NONE, the AF flag is cleared);
 * State READY, unlock.
 */
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout) {
	HostTarget::I2cTransfer &t = i2cRecord(hi2c, HostTarget::I2cCall::IsDeviceReady, HostTarget::BusMode::Blocking, DevAddress, 0, Timeout);
	t.trials = Trials;
	if (i2cBegin(hi2c, t, "HAL_I2C_IsDeviceReady") != HAL_OK) {
		return t.result;
	}
	I2cFake &fake = i2cFake(hi2c);
	hi2c->State = HAL_I2C_STATE_BUSY;
	hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
	if (fake.failNextBlocking) {
		auto [status, error] = *fake.failNextBlocking;
		fake.failNextBlocking.reset();
		i2cEndWithError(hi2c, error);
		t.result = status;
		t.errorCode = hi2c->ErrorCode;
		return status;
	}
	HostTarget::I2cDevice *dev = i2cFind(fake, t.address);
	uint32_t trial = 0;
	do {
		if (i2cAddressAck(dev)) {
			t.acknowledged = true;
			break;
		}
		trial++;
	} while (trial < Trials);
	if (dev != nullptr && dev->onTransfer) {
		dev->onTransfer(*dev, t);
	}
	hi2c->State = HAL_I2C_STATE_READY;
	halUnlock(hi2c);
	t.result = t.acknowledged ? HAL_OK : HAL_ERROR;
	t.completed = t.acknowledged;
	t.errorCode = hi2c->ErrorCode;
	return t.result;
}

/** Models HAL_I2C_GetState / HAL_I2C_GetMode / HAL_I2C_GetError */
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c) { return hi2c->State; }
HAL_I2C_ModeTypeDef HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c) { return hi2c->Mode; }
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *hi2c) { return hi2c->ErrorCode; }

} // extern "C"

// =====================================================================================================================
// Test control
// =====================================================================================================================
namespace HostTarget {

int SpiTransfer::selectedCs(bool activeLow) const {
	int selected = -1;
	for (size_t i = 0; i < csAtStart.size(); i++) {
		if (csAtStart[i] != activeLow) {
			if (selected != -1) {
				return -1;
			}
			selected = static_cast<int>(i);
		}
	}
	return selected;
}

void resetBus() {
	spis.clear();
	i2cs.clear();
}

void spiQueueRx(SPI_HandleTypeDef *hspi, const std::vector<uint8_t> &bytes) { spiFake(hspi).rxQueue.push_back(bytes); }
void spiSetIdleRx(SPI_HandleTypeDef *hspi, const std::vector<uint8_t> &bytes) { spiFake(hspi).idleRx = bytes; }
void spiSetResponder(SPI_HandleTypeDef *hspi, SpiResponder responder) { spiFake(hspi).responder = std::move(responder); }
size_t spiQueuedRx(const SPI_HandleTypeDef *hspi) { return spiFake(hspi).rxQueue.size(); }
const std::deque<SpiTransfer> &spiTransfers(const SPI_HandleTypeDef *hspi) { return spiFake(hspi).transfers; }
void spiClearTransfers(const SPI_HandleTypeDef *hspi) {
	SpiFake &fake = spiFake(hspi);
	if (fake.pending) {
		fake.transfers.erase(fake.transfers.begin(), fake.transfers.begin() + fake.pending->record);
		fake.pending->record = 0;
	} else {
		fake.transfers.clear();
	}
}
bool spiTransferPending(const SPI_HandleTypeDef *hspi) { return spiFake(hspi).pending.has_value(); }

bool spiComplete(SPI_HandleTypeDef *hspi) {
	SpiFake &fake = spiFake(hspi);
	if (!fake.pending) {
		return false;
	}
	SpiPending pending = *fake.pending;
	fake.pending.reset();
	SpiTransfer &t = fake.transfers[pending.record];
	t.tx.assign(pending.txBuffer, pending.txBuffer + t.size); // Before the rx bytes: Receive uses the same buffer
	t.rx = spiResponse(fake, t);
	if (pending.rxBuffer != nullptr) {
		std::memcpy(pending.rxBuffer, t.rx.data(), t.size);
	}
	t.csAtEnd = csLevels(fake);
	t.completed = true;
	t.errorCode = hspi->ErrorCode;
	hspi->State = HAL_SPI_STATE_READY; // SPI_Close*_ISR / SPI_DMA*Cplt: State READY before the callback
	HostPlatform::IsrContext isr;
	if (t.mode == BusMode::Dma) {
		switch (pending.callback) {
		case SpiCallback::Tx: HAL_SPI_TxHalfCpltCallback(hspi); break;
		case SpiCallback::Rx: HAL_SPI_RxHalfCpltCallback(hspi); break;
		case SpiCallback::TxRx: HAL_SPI_TxRxHalfCpltCallback(hspi); break;
		}
	}
	switch (pending.callback) {
	case SpiCallback::Tx: HAL_SPI_TxCpltCallback(hspi); break;
	case SpiCallback::Rx: HAL_SPI_RxCpltCallback(hspi); break;
	case SpiCallback::TxRx: HAL_SPI_TxRxCpltCallback(hspi); break;
	}
	return true;
}

bool spiFail(SPI_HandleTypeDef *hspi, uint32_t errorCode) {
	SpiFake &fake = spiFake(hspi);
	if (!fake.pending) {
		return false;
	}
	SpiPending pending = *fake.pending;
	fake.pending.reset();
	SpiTransfer &t = fake.transfers[pending.record];
	t.tx.assign(pending.txBuffer, pending.txBuffer + t.size); // Buffer content at the end (how much was sent is not modelled)
	hspi->ErrorCode |= errorCode;
	hspi->State = HAL_SPI_STATE_READY;
	t.csAtEnd = csLevels(fake);
	t.errorCode = hspi->ErrorCode;
	HostPlatform::IsrContext isr;
	HAL_SPI_ErrorCallback(hspi);
	return true;
}

void spiFailNextBlocking(SPI_HandleTypeDef *hspi, HAL_StatusTypeDef status) { spiFake(hspi).failNextBlocking = status; }
void spiFailNextDmaStart(SPI_HandleTypeDef *hspi) { spiFake(hspi).failNextDmaStart = true; }
uint32_t spiInitCount(const SPI_HandleTypeDef *hspi) { return spiFake(hspi).initCount; }
void spiSetCsPins(SPI_HandleTypeDef *hspi, const std::vector<std::pair<GPIO_TypeDef *, uint16_t>> &pins) { spiFake(hspi).csPins = pins; }
std::vector<bool> spiCsLevels(const SPI_HandleTypeDef *hspi) { return csLevels(spiFake(hspi)); }

// ---- I2C ----
uint16_t I2cDevice::register16(uint16_t reg) const {
	auto it = registers.find(reg);
	if (it == registers.end() || it->second.size() < 2) {
		return 0xFFFF;
	}
	return static_cast<uint16_t>((it->second[0] << 8) | it->second[1]);
}

std::vector<uint8_t> I2cDevice::registerBytes(uint16_t reg) const {
	std::vector<uint8_t> bytes(std::max<uint8_t>(registerWidth, 1), 0xFF);
	auto it = registers.find(reg);
	if (it != registers.end()) {
		for (size_t i = 0; i < bytes.size() && i < it->second.size(); i++) {
			bytes[i] = it->second[i];
		}
	}
	return bytes;
}

I2cDevice &i2cDevice(I2C_HandleTypeDef *hi2c, uint8_t address) { return i2cFake(hi2c).devices[address]; }
void i2cRemoveDevice(I2C_HandleTypeDef *hi2c, uint8_t address) { i2cFake(hi2c).devices.erase(address); }
const std::deque<I2cTransfer> &i2cTransfers(const I2C_HandleTypeDef *hi2c) { return i2cFake(hi2c).transfers; }
void i2cClearTransfers(const I2C_HandleTypeDef *hi2c) {
	I2cFake &fake = i2cFake(hi2c);
	if (fake.pending) {
		fake.transfers.erase(fake.transfers.begin(), fake.transfers.begin() + fake.pending->record);
		fake.pending->record = 0;
	} else {
		fake.transfers.clear();
	}
}
bool i2cTransferPending(const I2C_HandleTypeDef *hi2c) { return i2cFake(hi2c).pending.has_value(); }

bool i2cComplete(I2C_HandleTypeDef *hi2c) {
	I2cFake &fake = i2cFake(hi2c);
	if (!fake.pending) {
		return false;
	}
	I2cPending pending = *fake.pending;
	fake.pending.reset();
	I2cTransfer &t = fake.transfers[pending.record];
	t.data.clear();
	uint32_t error = i2cExecute(fake, t, pending.writeData, pending.buffer);
	const HAL_I2C_ModeTypeDef mode = hi2c->Mode;
	hi2c->State = HAL_I2C_STATE_READY;
	hi2c->Mode = HAL_I2C_MODE_NONE;
	HostPlatform::IsrContext isr;
	if (error != HAL_I2C_ERROR_NONE) {
		hi2c->ErrorCode |= error; // HAL_I2C_ER_IRQHandler (AF, STOP generated) -> I2C_ITError
		t.errorCode = hi2c->ErrorCode;
		HAL_I2C_ErrorCallback(hi2c);
		return true;
	}
	t.completed = true;
	t.errorCode = hi2c->ErrorCode;
	const bool tx = t.call == I2cCall::MasterTransmit || t.call == I2cCall::MemWrite;
	if (mode == HAL_I2C_MODE_MEM) {
		tx ? HAL_I2C_MemTxCpltCallback(hi2c) : HAL_I2C_MemRxCpltCallback(hi2c);
	} else {
		tx ? HAL_I2C_MasterTxCpltCallback(hi2c) : HAL_I2C_MasterRxCpltCallback(hi2c);
	}
	return true;
}

bool i2cFail(I2C_HandleTypeDef *hi2c, uint32_t errorCode) {
	I2cFake &fake = i2cFake(hi2c);
	if (!fake.pending) {
		return false;
	}
	I2cPending pending = *fake.pending;
	fake.pending.reset();
	I2cTransfer &t = fake.transfers[pending.record];
	t.data.clear();
	hi2c->ErrorCode |= errorCode;
	hi2c->State = HAL_I2C_STATE_READY;
	hi2c->Mode = HAL_I2C_MODE_NONE;
	t.errorCode = hi2c->ErrorCode;
	HostPlatform::IsrContext isr;
	HAL_I2C_ErrorCallback(hi2c);
	return true;
}

void i2cSetBusBusy(I2C_HandleTypeDef *hi2c, bool busy) { i2cFake(hi2c).busBusy = busy; }
void i2cFailNextBlocking(I2C_HandleTypeDef *hi2c, HAL_StatusTypeDef status, uint32_t errorCode) { i2cFake(hi2c).failNextBlocking = std::make_pair(status, errorCode); }
void i2cSetDmaLinked(I2C_HandleTypeDef *hi2c, bool linked) { i2cFake(hi2c).dmaLinked = linked; }
uint32_t i2cInitCount(const I2C_HandleTypeDef *hi2c) { return i2cFake(hi2c).initCount; }
uint32_t i2cDeInitCount(const I2C_HandleTypeDef *hi2c) { return i2cFake(hi2c).deInitCount; }

} // namespace HostTarget
