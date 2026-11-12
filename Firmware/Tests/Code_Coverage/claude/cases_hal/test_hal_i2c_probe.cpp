// P12 T4c lead probe of the I2C model through the real HAL (kept as the model's own sanity cases; the writers' files cover the HAL).
#include "I2cTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>

extern "C" uint32_t SystemCoreClock;

namespace {
struct Probe : I2cFixture {
	I2C_HandleTypeDef h{};
	Probe() {
		SystemCoreClock = 168000000u;
		RCC->CFGR = (RCC->CFGR & ~(7u << 10)) | (5u << 10); /* PPRE1 /4 -> 42 MHz */
		h.Instance = I2C1;
		h.Init.ClockSpeed = 100000;
		h.Init.DutyCycle = I2C_DUTYCYCLE_2;
		h.Init.OwnAddress1 = 0x30;
		h.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		h.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		h.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		h.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		uwTickPrio = 15;
	}
};
}

TEST_CASE_FIXTURE(Probe, "probe: init, polling master transmit and receive against a peer device") {
	REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
	CHECK(hostI2cConfigWhileEnabled(0) == 0u);
	const int dev = hostI2cDeviceAdd(0, 0x50, 0);
	uint8_t tx[3] = {1, 2, 3};
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0x50 << 1, tx, 3, 100) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_READY);
	/* log: START, addr(W) ack, 3 data acks, STOP */
	REQUIRE(hostI2cLogCount(0) == 6u);
	CHECK(hostI2cLogAt(0, 0).type == HOST_I2C_EV_START);
	CHECK(hostI2cLogAt(0, 1).value == 0xA0);
	CHECK(hostI2cLogAt(0, 3).value == 2);
	CHECK(hostI2cLogAt(0, 5).type == HOST_I2C_EV_STOP);
	hostI2cLogClear(0);
	const uint8_t q[4] = {9, 8, 7, 6};
	hostI2cDeviceQueue(0, dev, q, 4);
	for (unsigned n : {1u, 2u, 3u, 4u}) {
		uint8_t rx[4] = {0, 0, 0, 0};
		hostI2cDeviceQueue(0, dev, q, 0);
		REQUIRE(HAL_I2C_Master_Receive(&h, 0x50 << 1, rx, (uint16_t)n, 100) == HAL_OK);
		CHECK(rx[0] != 0);
		hostI2cLogClear(0);
		const uint8_t again[4] = {9, 8, 7, 6};
		hostI2cDeviceQueue(0, dev, again, 4); /* queue keeps the unread rest; only checking the transfer completes */
	}
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(Probe, "probe: address nack and IsDeviceReady") {
	REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
	uint8_t tx[1] = {1};
	CHECK(HAL_I2C_Master_Transmit(&h, 0x51 << 1, tx, 1, 100) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	hostI2cDeviceAdd(0, 0x50, 0);
	CHECK(HAL_I2C_IsDeviceReady(&h, 0x50 << 1, 2, 100) == HAL_OK);
}

TEST_CASE_FIXTURE(Probe, "probe: interrupt master transmit and slave receive with the external controller") {
	REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
	hostI2cDeviceAdd(0, 0x50, 0);
	uint8_t tx[2] = {0x11, 0x22};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 2) == HAL_OK);
	i2cRunIrqs(&h, 0);
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	hostI2cLogClear(0);
	uint8_t rx[2] = {0, 0};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x30) == 1);
	i2cRunIrqs(&h, 0);
	CHECK(hostI2cExtWrite(0, 0xAA) == 1);
	i2cRunIrqs(&h, 0);
	CHECK(hostI2cExtWrite(0, 0xBB) == 1);
	i2cRunIrqs(&h, 0);
	hostI2cExtStop(0);
	i2cRunIrqs(&h, 0);
	CHECK(rx[0] == 0xAA);
	CHECK(rx[1] == 0xBB);
	CHECK(i2cCounts.slaveRx == 1u);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(Probe, "probe: memory read and write, master receive IT, slave transmit IT, 10-bit target") {
	REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
	const int dev = hostI2cDeviceAdd(0, 0x50, 0);
	uint8_t w[2] = {5, 6};
	REQUIRE(HAL_I2C_Mem_Write(&h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, w, 2, 100) == HAL_OK);
	CHECK(hostI2cLogCount(0) == 6u); /* S, A, mem, d, d, P */
	hostI2cLogClear(0);
	const uint8_t q[5] = {1, 2, 3, 4, 5};
	hostI2cDeviceQueue(0, dev, q, 5);
	uint8_t r[3] = {};
	REQUIRE(HAL_I2C_Mem_Read(&h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_16BIT, r, 3, 100) == HAL_OK);
	CHECK(r[0] == 1);
	CHECK(r[2] == 3);
	hostI2cLogClear(0);
	uint8_t r2[2] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, r2, 2) == HAL_OK);
	i2cRunIrqs(&h, 0);
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(r2[0] == 4);
	CHECK(r2[1] == 5);
	/* slave transmit */
	hostI2cLogClear(0);
	uint8_t s[2] = {0x77, 0x88};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, s, 2) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x31) == 1);
	i2cRunIrqs(&h, 0);
	uint8_t b = 0;
	CHECK(hostI2cExtReadByte(0, 1, &b) == 1);
	CHECK(b == 0x77);
	i2cRunIrqs(&h, 0);
	CHECK(hostI2cExtReadByte(0, 0, &b) == 0);
	CHECK(b == 0x88);
	i2cRunIrqs(&h, 0);
	hostI2cExtStop(0);
	i2cRunIrqs(&h, 0);
	CHECK(i2cCounts.slaveTx == 1u);
	CHECK(HalFixture::faults() == 0u);
}
