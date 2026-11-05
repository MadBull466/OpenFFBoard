/*
 * Default implementations of the Encoder and MotorDriver base classes (Encoder.cpp, MotorDriver.cpp)
 * that are not covered by test_device_mocks.cpp: Encoder::getPosAbs, Encoder::setPos and
 * MotorDriver::setupDriver.
 *
 * MockEncoder and MockMotorDriver override exactly these functions, so the tests use small
 * test-local subclasses that only override what is needed to observe the base class behaviour.
 */
#include "doctest.h"
#include "Encoder.h"
#include "MotorDriver.h"

#include <cstdint>
#include <memory>

using doctest::Approx;

namespace {

/** Incremental encoder that only provides a position and cpr. Everything else is the base class */
class PositionOnlyEncoder : public Encoder {
public:
	explicit PositionOnlyEncoder(uint32_t cpr) { this->cpr = cpr; }
	int32_t getPos() override {
		posReads++;
		return position;
	}
	int32_t position = 0;
	uint32_t posReads = 0;
};

/** Motor driver recording calls of the motor control functions. setupDriver is NOT overridden */
class RecordingDriver : public MotorDriver {
public:
	void turn(int16_t power) override { turnCalls++; }
	void startMotor() override { startCalls++; }
	void stopMotor() override { stopCalls++; }
	void emergencyStop(bool reset) override { emergencyCalls++; }
	void setPowerLimit(uint16_t power) override { powerLimitCalls++; }
	uint32_t turnCalls = 0, startCalls = 0, stopCalls = 0, emergencyCalls = 0, powerLimitCalls = 0;
};

} // namespace

TEST_SUITE("Encoder") {

TEST_CASE("base getPosAbs returns the position of getPos") {
	PositionOnlyEncoder encoder(4096);
	Encoder &base = encoder;
	for (int32_t pos : {0, 1, -1, 1234, -98765, INT32_MAX, INT32_MIN}) {
		CAPTURE(pos);
		encoder.position = pos;
		const uint32_t readsBefore = encoder.posReads;
		CHECK(base.getPosAbs() == pos);
		CHECK(encoder.posReads == readsBefore + 1); // Delegates to the virtual getPos
	}
}

TEST_CASE("base getPosAbs of the dummy encoder is 0") {
	Encoder encoder;
	CHECK(encoder.getPosAbs() == 0);
}

TEST_CASE("base getPosAbs_f scales the position of getPos to rotations") {
	PositionOnlyEncoder encoder(4096);
	encoder.position = -2048;
	CHECK(encoder.getPosAbs_f() == Approx(-0.5));
	encoder.position = 3 * 4096 + 1024;
	CHECK(encoder.getPosAbs_f() == Approx(3.25));
}

TEST_CASE("base setPos does not change the position") {
	PositionOnlyEncoder encoder(1000);
	Encoder &base = encoder;
	encoder.position = 250;
	base.setPos(0);
	CHECK(base.getPos() == 250);
	CHECK(base.getPosAbs() == 250);
	base.setPos(-12345);
	CHECK(base.getPos() == 250);
	CHECK(base.getPos_f() == Approx(0.25));
}

TEST_CASE("base setPos of the dummy encoder keeps position 0") {
	Encoder encoder;
	encoder.setPos(500);
	CHECK(encoder.getPos() == 0);
	CHECK(encoder.getPosAbs() == 0);
}

} // TEST_SUITE

TEST_SUITE("MotorDriver") {

TEST_CASE("base setupDriver does not command the motor") {
	RecordingDriver driver;
	MotorDriver &base = driver;
	Encoder *encoderBefore = base.getEncoder();

	base.setupDriver();

	CHECK(driver.turnCalls == 0);
	CHECK(driver.startCalls == 0);
	CHECK(driver.stopCalls == 0);
	CHECK(driver.emergencyCalls == 0);
	CHECK(driver.powerLimitCalls == 0);
	CHECK(base.getEncoder() == encoderBefore); // Keeps the assigned encoder
	CHECK(base.motorReady());
}

TEST_CASE("base setupDriver keeps an assigned external encoder") {
	RecordingDriver driver;
	std::shared_ptr<Encoder> encoder = std::make_shared<PositionOnlyEncoder>(2000);
	driver.setEncoder(encoder);
	driver.setupDriver();
	CHECK(driver.getEncoder() == encoder.get());
}

} // TEST_SUITE
