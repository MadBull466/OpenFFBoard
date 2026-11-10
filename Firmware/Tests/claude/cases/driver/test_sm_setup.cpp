/*
 * SimpleMotion driver, phase P15 T1 skeleton: MotorSimplemotion builds into ffb_driver_tests (SIMPLEMOTION on in this
 * executable only) and the fake SimpleMotion device behaves like the SM library. Charter:
 * Tests/claude/charters/driver/SIMPLEMOTION_REQUIREMENTS.md (R1.2, R2.1, R3.1, R4.1 smoke and the fake section); the full test
 * groups follow in T2.
 *
 * Oracles: SM (SimpleMotion V2 library, Tests/shared/specs/SimpleMotionV2-master, never copied) for frame ids, CRCs and
 * subpacket sizes; HD (hand derivations in the comments); the fake is checked against the library values by script.
 */
#include "doctest.h"
#include "SmFixture.h"

#include "MotorDriver.h"
#include "constants.h"

#include <string>

TEST_SUITE("SM setup") {

TEST_CASE("the fake device's CRC-8 gives the library value for the fast update header {0x10, 0x01, 0, 0, 0, 0}") {
	// HD: CRC-8 poly 0x07, init 0x52, over 10 01 00 00 00 00 = 0x46 (charter R3.1, checked against sm_consts.c table_crc8 by script)
	const uint8_t b[6] = {0x10, 0x01, 0, 0, 0, 0};
	CHECK(FakeSmDevice::crc8(b, 6, 0x52) == 0x46);
}

TEST_CASE("the fake device's CRC-16 of the check string 123456789 is the CRC-16/ARC value 0xBB3D (library value byte swapped: 0x3DBB)") {
	const uint8_t s[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
	CHECK(FakeSmDevice::crc16arc(s, 9) == 0xBB3D);
}

TEST_CASE_FIXTURE(SmFixture, "a MotorSimplemotion1 can be constructed and destroyed and the motor UART is reserved while it exists") {
	CHECK(MotorSimplemotion1::isCreatable());
	{
		auto sm = makeSm(1);
		REQUIRE(static_cast<bool>(sm));
		CHECK_FALSE(MotorSimplemotion1::isCreatable());
		CHECK(MotorSimplemotion2::isCreatable());
	}
	CHECK(MotorSimplemotion1::isCreatable());
}

TEST_CASE_FIXTURE(SmFixture, "the constructor configures the motor UART to 460800 baud 8N1 with TX and RX and oversampling 8, write enable low") {
	auto sm = makeSm(1);
	CHECK(huart3.Init.BaudRate == 460800);
	CHECK(huart3.Init.WordLength == UART_WORDLENGTH_8B);
	CHECK(huart3.Init.StopBits == UART_STOPBITS_1);
	CHECK(huart3.Init.Parity == UART_PARITY_NONE);
	CHECK(huart3.Init.Mode == UART_MODE_TX_RX);
	CHECK(huart3.Init.OverSampling == UART_OVERSAMPLING_8);
	CHECK((GPIOE->ODR & DRV_GP1_Pin) == 0);
}

TEST_CASE_FIXTURE(SmFixture, "the driver lists contain the SimpleMotion drivers 9 and 10 once each") {
	int one = 0, two = 0;
	for (const auto &c : MotorDriver::all_drivers) {
		one += c.selectionId == 9 && c.info.id == CLSID_MOT_SM1;
		two += c.selectionId == 10 && c.info.id == CLSID_MOT_SM2;
	}
	CHECK(one == 1);
	CHECK(two == 1);
}

TEST_CASE_FIXTURE(SmFixture, "on an initialized driver turn(0) sends the 7 byte fast update {0x10, address, 0, 0, 0, 0, crc8} with a valid CRC-8 in one transfer") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	const uint32_t transfers = HostPlatform::uart().txTransfers;
	sm->turn(0);
	REQUIRE(dev.frameCount() == 1);
	const auto &f = dev.frames()[0];
	CHECK(f.kind == FakeSmDevice::Kind::Fast);
	REQUIRE(f.bytes.size() == 7);
	CHECK(f.bytes[0] == 0x10);
	CHECK(f.bytes[2] == 0);
	CHECK(f.bytes[3] == 0);
	CHECK(f.bytes[4] == 0);
	CHECK(f.bytes[5] == 0);
	CHECK(f.bytes[6] == FakeSmDevice::crc8(f.bytes.data(), 6, 0x52)); // the fake also checked it: no violation
	CHECK(HostPlatform::uart().txTransfers == transfers + 1);
}

TEST_CASE_FIXTURE(SmFixture, "the settings sequence on a healthy device ends initialized and the exchanges run without a peer violation") {
	auto sm = makeSm(1);
	CHECK_FALSE(sm->motorReady());
	REQUIRE(initialize(*sm));
	CHECK(sm->motorReady());
	// devtype, faults, status, FBR, FBD, ControlMode: six instant commands
	CHECK(dev.frameCount() == 6);
	CHECK(dev.violations().empty());
}

TEST_CASE_FIXTURE(SmFixture, "with a silent peer every parameter request times out after 10 ms of simulated time and the driver stays uninitialized") {
	dev.silent = true;
	auto sm = makeSm(1);
	const uint64_t before = HostPlatform::nowUs();
	CHECK_FALSE(initialize(*sm));
	CHECK_FALSE(sm->motorReady());
	CHECK(HostPlatform::nowUs() > before + 150000); // the 150 ms interval plus at least one 10 ms wait
}

TEST_CASE_FIXTURE(SmFixture, "a fast update reply delivered by the fake updates the position and releases the port for the next frame") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.position16 = 0;
	sm->turn(100);
	process(); // transmit complete interrupt, then the reply bytes
	CHECK(dev.bytesNotAccepted() == 0);
	CHECK(sm->getPos() == 0); // first report 0 = the zero point (charter R6.1)
	dev.clearFrames();
	sm->turn(200);
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.setpoints().back() == 200);
}

TEST_CASE_FIXTURE(SmFixture, "the write enable pin is high after the transmit and low after the transmit complete interrupt") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	sm->turn(0);
	CHECK((GPIOE->ODR & DRV_GP1_Pin) != 0); // set by startUartTransfer(transmit)
	process();
	CHECK((GPIOE->ODR & DRV_GP1_Pin) == 0); // reset by endUartTransfer(transmit) in the tx complete callback
}

}
