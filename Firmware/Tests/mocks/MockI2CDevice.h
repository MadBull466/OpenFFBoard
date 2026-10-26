/*
 * MockI2CDevice.h
 *
 * I2C device implementing the I2CDevice interface of I2C.h without hardware (ffb_target_tests, P6). For tests of the
 * port side of I2C.cpp (I2CPort: port users, semaphore, current device, completion and error dispatch): the mock
 * records every call the port makes on its device, in order, with the port it came from.
 *
 * startI2CTransfer / endI2CTransfer / i2cError call the I2CDevice implementation (take / give the port semaphore;
 * i2cError resets the port: I2CPort::resetPort) unless callBase is false. i2cTxCompleted / i2cRxCompleted only record,
 * like the empty base implementations. No static state: nothing to reset.
 */
#ifndef HOSTTEST_MOCKI2CDEVICE_H_
#define HOSTTEST_MOCKI2CDEVICE_H_

#include "I2C.h"

#include <cstdint>
#include <vector>

class MockI2CDevice : public I2CDevice {
public:
	enum class Call : uint8_t { StartTransfer, EndTransfer, TxCompleted, RxCompleted, Error };
	struct Event {
		Call call;
		I2CPort *port; //!< Port argument of the call
		uint32_t state; //!< HAL state of the port's handle (I2C_HandleTypeDef::State) when the call was made
	};

	explicit MockI2CDevice(I2C_HandleTypeDef *handle = nullptr) : handle(handle) {}

	void startI2CTransfer(I2CPort *port) override {
		record(Call::StartTransfer, port);
		if (callBase) {
			I2CDevice::startI2CTransfer(port);
		}
	}
	void endI2CTransfer(I2CPort *port) override {
		record(Call::EndTransfer, port);
		if (callBase) {
			I2CDevice::endI2CTransfer(port);
		}
	}
	void i2cTxCompleted(I2CPort *port) override { record(Call::TxCompleted, port); }
	void i2cRxCompleted(I2CPort *port) override { record(Call::RxCompleted, port); }
	void i2cError(I2CPort *port) override {
		record(Call::Error, port);
		if (callBase) {
			I2CDevice::i2cError(port);
		}
	}

	// --- Test helpers ---
	/** The recorded calls only */
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
	bool callBase = true;					//!< start/end/error call the I2CDevice implementation
	I2C_HandleTypeDef *handle = nullptr;	//!< Handle whose State is recorded (optional, e.g. &hi2c1)

	// --- Recorded ---
	std::vector<Event> events;

private:
	void record(Call call, I2CPort *p) { events.push_back(Event{call, p, handle != nullptr ? static_cast<uint32_t>(handle->State) : 0U}); }
};

#endif
