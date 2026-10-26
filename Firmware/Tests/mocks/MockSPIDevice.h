/*
 * MockSPIDevice.h
 *
 * SPI device implementing the SPIDevice interface of SPI.h without hardware (ffb_target_tests, P6). For tests of the
 * port side of SPI.cpp (SPIPort: device arbitration, CS pins, reconfiguration, completion dispatch): the mock records
 * every call the port makes on its device, in order, with the port it came from.
 *
 * beginSpiTransfer / endSpiTransfer call the SPIDevice implementation (take / give the port semaphore, assert / clear
 * the CS pin) unless callBase is false. The completion callbacks (spiTxCompleted ...) only record, like the empty base
 * implementations. The output level of the device's CS pin (ODR) at the time of a call is recorded too.
 * No static state: nothing to reset.
 */
#ifndef HOSTTEST_MOCKSPIDEVICE_H_
#define HOSTTEST_MOCKSPIDEVICE_H_

#include "SPI.h"

#include <cstdint>
#include <vector>

class MockSPIDevice : public SPIDevice {
public:
	enum class Call : uint8_t { BeginTransfer, EndTransfer, TxCompleted, RxCompleted, TxRxCompleted, RequestError };
	struct Event {
		Call call;
		SPIPort *port;		//!< Port argument of the call
		bool csHigh;		//!< Output level of the device's CS pin when the call was made (before begin/end changed it)
		bool portTaken;		//!< SPIPort::isTaken() when the call was made
	};

	/** SPIDevice(port, csPin): copies the port's current configuration (hspi->Init), reserves the CS pin, CS active low */
	MockSPIDevice(SPIPort &port, OutputPin csPin) : SPIDevice(port, csPin) {}
	/** SPIDevice(port, config): the given CS pin and polarity, peripheral settings replaced by the port's hspi->Init */
	MockSPIDevice(SPIPort &port, SPIConfig &config) : SPIDevice(port, config) {}

	void beginSpiTransfer(SPIPort *port) override {
		record(Call::BeginTransfer, port);
		if (callBase) {
			SPIDevice::beginSpiTransfer(port);
		}
	}
	void endSpiTransfer(SPIPort *port) override {
		record(Call::EndTransfer, port);
		if (callBase) {
			SPIDevice::endSpiTransfer(port);
		}
	}
	void spiTxCompleted(SPIPort *port) override { record(Call::TxCompleted, port); }
	void spiRxCompleted(SPIPort *port) override { record(Call::RxCompleted, port); }
	void spiTxRxCompleted(SPIPort *port) override { record(Call::TxRxCompleted, port); }
	void spiRequestError(SPIPort *port) override { record(Call::RequestError, port); }

	// --- Test helpers ---
	/** Replaces the device configuration (SPIDevice::setSpiConfig is protected), e.g. CPOL/CPHA/prescaler or cspol */
	void setConfig(const SPIConfig &config) { setSpiConfig(config); }
	SPIConfig &config() { return spiConfig; }
	SPIPort &port() { return spiPort; }
	/** The recorded calls only (without port and levels) */
	std::vector<Call> calls() const {
		std::vector<Call> list;
		for (const Event &e : events) {
			list.push_back(e.call);
		}
		return list;
	}
	/** Number of recorded calls of one kind */
	size_t count(Call call) const {
		size_t n = 0;
		for (const Event &e : events) {
			n += e.call == call ? 1 : 0;
		}
		return n;
	}

	// --- Controlled by the test ---
	bool callBase = true; //!< begin/endSpiTransfer call the SPIDevice implementation (semaphore and CS pin)

	// --- Recorded ---
	std::vector<Event> events;

private:
	void record(Call call, SPIPort *p) {
		const OutputPin &cs = spiConfig.cs;
		bool high = (cs.getPort()->ODR & cs.getPin()) != 0;
		events.push_back(Event{call, p, high, p != nullptr && p->isTaken()});
	}
};

#endif
