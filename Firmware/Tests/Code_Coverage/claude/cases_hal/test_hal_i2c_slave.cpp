// P12 T4c group I2S (IS1-IS5): the I2C target mode of the real ST HAL, blocking calls (HAL_I2C_Slave_Transmit / Receive), listen mode
// (HAL_I2C_EnableListen_IT / DisableListen_IT) and the address and listen paths of the event / error interrupt handlers.
// Oracles: UM1725 chapter "HAL I2C Generic Driver", RM0090 chapter 27 (27.3.2 target, 27.3.4 errors, 27.6 registers). The
// external controller is the model's hostI2cExt* API; the blocking calls wait inside the HAL, so it is driven by a tick script.
// Hand derivation of the script timing: the HAL's first HAL_GetTick() (tickstart) is tick base + 1; every further call is one tick, so
// the wait loop of the HAL meets action "tick k" (relative to base) at its k-th call. The model delivers a byte at once.
#include "I2cTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>

extern "C" uint32_t SystemCoreClock;

namespace {

enum { OP_START = 1, OP_RESTART, OP_WRITE, OP_READ, OP_STOP, OP_RAISE, OP_SNAP };

struct Step {
	int op;
	uint8_t val;
	int ack;
	uint32_t mask;
	int ret;
	uint8_t out;
	uint32_t cr1, sr1, sr2;
	int state, mode;
	uint32_t err;
	unsigned runs;
};

I2C_HandleTypeDef *gsH;
unsigned gsIdx;

void gsRun(void *p) {
	Step *s = static_cast<Step *>(p);
	++s->runs;
	switch (s->op) {
	case OP_START: s->ret = hostI2cExtStart(gsIdx, s->val); break;
	case OP_RESTART: s->ret = hostI2cExtRestart(gsIdx, s->val); break;
	case OP_WRITE: s->ret = hostI2cExtWrite(gsIdx, s->val); break;
	case OP_READ: s->ret = hostI2cExtReadByte(gsIdx, s->ack, &s->out); break;
	case OP_STOP: hostI2cExtStop(gsIdx); break;
	case OP_RAISE: hostI2cRaiseSr1(gsIdx, s->mask); break;
	case OP_SNAP:
		s->cr1 = HalFixture::peek(&gsH->Instance->CR1);
		s->sr1 = HalFixture::peek(&gsH->Instance->SR1);
		s->sr2 = HalFixture::peek(&gsH->Instance->SR2);
		s->state = (int)gsH->State;
		s->mode = (int)gsH->Mode;
		s->err = gsH->ErrorCode;
		break;
	}
}

struct Script {
	Step steps[16];
	I2cScriptAction acts[16];
	unsigned n = 0;
	Script() { std::memset(steps, 0, sizeof steps); }
	Step &at(uint32_t rel, int op, uint8_t val = 0, int ack = 1, uint32_t mask = 0) {
		Step &s = steps[n];
		s.op = op;
		s.val = val;
		s.ack = ack;
		s.mask = mask;
		s.ret = -99;
		acts[n].atTick = rel;
		acts[n].fn = gsRun;
		acts[n].arg = &s;
		++n;
		return s;
	}
	void go() {
		for (unsigned k = 0; k < n; ++k) acts[k].atTick += uwTick;
		i2cScriptStart(acts, n);
	}
};

const uint32_t kNoOption = 0xFFFF0000u; /* I2C_NO_OPTION_FRAME (UM1725: no sequential option) */
const uint8_t kData[5] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5};

struct Slave : I2cFixture {
	I2C_HandleTypeDef h{};
	Slave() {
		SystemCoreClock = 168000000u;
		RCC->CFGR = (RCC->CFGR & ~(7u << 10)) | (5u << 10); /* PPRE1 /4 -> 42 MHz */
		uwTickPrio = 15;
		gsH = &h;
		gsIdx = 0;
		std::memset(&i2cCounts, 0, sizeof i2cCounts); // see report: i2cSupportReset leaves orderCount uninitialised
	}
	~Slave() { i2cScriptStop(); }
	void cfg(I2C_HandleTypeDef &x, unsigned inst, uint32_t own1 = 0x30) {
		std::memset(&x, 0, sizeof x);
		x.Instance = inst == 0 ? I2C1 : (inst == 1 ? I2C2 : I2C3);
		x.Init.ClockSpeed = 100000;
		x.Init.DutyCycle = I2C_DUTYCYCLE_2;
		x.Init.OwnAddress1 = own1;
		x.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		x.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		x.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		x.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	}
	void bring(unsigned inst = 0, uint32_t own1 = 0x30) {
		cfg(h, inst, own1);
		gsIdx = inst;
		gsH = &h;
	}
	bool initOk() { return HAL_I2C_Init(&h) == HAL_OK; }
	uint32_t cr1() { return peek(&h.Instance->CR1); }
	uint32_t sr1() { return peek(&h.Instance->SR1); }
	uint32_t sr2() { return peek(&h.Instance->SR2); }
	uint32_t cr2() { return peek(&h.Instance->CR2); }
	// n data bytes written by the controller at ticks 4.., after the address at tick 3 and followed by a stop
	void rxScript(Script &s, uint8_t addrByte, unsigned n, const uint8_t *d, bool stop = true) {
		s.at(3, OP_START, addrByte);
		for (unsigned k = 0; k < n; ++k) s.at(4 + k, OP_WRITE, d[k]);
		if (stop) s.at(4 + n, OP_STOP);
	}
	// n bytes read by the controller at ticks 4..; the last one is not acknowledged
	void txScript(Script &s, uint8_t addrByte, unsigned n) {
		s.at(3, OP_START, addrByte);
		for (unsigned k = 0; k < n; ++k) s.at(4 + k, OP_READ, 0, k + 1 < n ? 1 : 0);
	}
};

} // namespace

// ------------------------------------------------------------------------------------------------ IS1 Slave_Receive
TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive delivers 1, 2, 3 and 5 bytes in the order the controller sent them") {
	for (unsigned n : {1u, 2u, 3u, 5u}) {
		bring();
		h.State = HAL_I2C_STATE_RESET;
		REQUIRE(initOk());
		uint8_t rx[6];
		std::memset(rx, 0, sizeof rx);
		Script s;
		rxScript(s, 0x30, n, kData);
		s.go();
		INFO("n = " << n);
		CHECK(HAL_I2C_Slave_Receive(&h, rx, (uint16_t)n, 100) == HAL_OK);
		for (unsigned k = 0; k < n; ++k) CHECK(rx[k] == kData[k]);
		for (unsigned k = n; k < 6; ++k) CHECK(rx[k] == 0);
		CHECK(s.steps[0].ret == 1);
		i2cScriptStop();
		HAL_I2C_DeInit(&h);
	}
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive works on I2C1, I2C2 and I2C3") {
	for (unsigned inst = 0; inst < 3; ++inst) {
		bring(inst);
		REQUIRE(initOk());
		uint8_t rx[2] = {0, 0};
		Script s;
		rxScript(s, 0x30, 2, kData);
		s.go();
		INFO("instance " << inst);
		CHECK(HAL_I2C_Slave_Receive(&h, rx, 2, 100) == HAL_OK);
		CHECK(rx[0] == kData[0]);
		CHECK(rx[1] == kData[1]);
		CHECK(h.State == HAL_I2C_STATE_READY);
		i2cScriptStop();
	}
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive reads two bytes that arrive back to back in the same tick") {
	bring();
	REQUIRE(initOk());
	uint8_t rx[3] = {0, 0, 0};
	Script s;
	s.at(3, OP_START, 0x30);
	s.at(4, OP_WRITE, 0x11);
	s.at(4, OP_WRITE, 0x22); // RXNE still set: the byte waits in the shift register (BTF, RM0090 27.3.3)
	s.at(5, OP_WRITE, 0x33);
	s.at(6, OP_STOP);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 3, 100) == HAL_OK);
	CHECK(rx[0] == 0x11);
	CHECK(rx[1] == 0x22);
	CHECK(rx[2] == 0x33);
	CHECK(s.steps[1].ret == 1);
	CHECK(s.steps[2].ret == 1);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive enables the acknowledge, sets BUSY_RX and slave mode while it waits for the address") {
	bring();
	REQUIRE(initOk());
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
	uint8_t rx[1] = {0};
	Script s;
	s.at(2, OP_SNAP);
	rxScript(s, 0x30, 1, kData);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 100) == HAL_OK);
	CHECK((s.steps[0].cr1 & I2C_CR1_ACK) == I2C_CR1_ACK);
	CHECK(s.steps[0].state == (int)HAL_I2C_STATE_BUSY_RX);
	CHECK(s.steps[0].mode == (int)HAL_I2C_MODE_SLAVE);
	CHECK(s.steps[0].err == HAL_I2C_ERROR_NONE);
	CHECK(s.steps[1].ret == 1); // the own address was acknowledged
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive ends with state READY, mode NONE, ErrorCode NONE, acknowledge off, STOPF cleared, unlocked") {
	bring();
	REQUIRE(initOk());
	h.ErrorCode = HAL_I2C_ERROR_AF | HAL_I2C_ERROR_OVR; // left from an earlier transfer: cleared at the start of the call
	uint8_t rx[2] = {0, 0};
	Script s;
	rxScript(s, 0x30, 2, kData);
	s.go();
	REQUIRE(HAL_I2C_Slave_Receive(&h, rx, 2, 100) == HAL_OK);
	CHECK(h.pBuffPtr == rx + 2); // advanced by the number of bytes received
	CHECK(h.XferCount == 0u);
	CHECK(h.XferSize == 0u);
	CHECK((cr2() & (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN)) == 0u); // polling: no interrupt enabled
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
	CHECK((sr1() & I2C_SR1_STOPF) == 0u);
	CHECK((cr1() & I2C_CR1_PE) == I2C_CR1_PE);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive can be called again after a finished transfer") {
	bring();
	REQUIRE(initOk());
	uint8_t rx[1] = {0};
	for (unsigned k = 0; k < 2; ++k) {
		Script s;
		rxScript(s, 0x30, 1, &kData[k]);
		s.go();
		CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 100) == HAL_OK);
		CHECK(rx[0] == kData[k]);
		i2cScriptStop();
	}
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive enables the peripheral when PE is 0 and clears POS") {
	bring();
	REQUIRE(initOk());
	h.Instance->CR1 &= ~I2C_CR1_PE;
	h.Instance->CR1 |= I2C_CR1_POS;
	REQUIRE((cr1() & I2C_CR1_PE) == 0u);
	uint8_t rx[1] = {0};
	Script s;
	s.at(2, OP_SNAP);
	s.at(3, OP_START, 0x30);
	s.at(4, OP_WRITE, 0x5A);
	s.at(5, OP_STOP);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 100) == HAL_OK);
	CHECK((s.steps[0].cr1 & I2C_CR1_PE) == I2C_CR1_PE);
	CHECK((s.steps[0].cr1 & I2C_CR1_POS) == 0u);
	CHECK(rx[0] == 0x5A);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive times out when nobody addresses it") {
	bring();
	REQUIRE(initOk());
	uint8_t rx[1] = {0};
	const uint32_t t0 = uwTick;
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 5) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(uwTick - t0 > 5u); // an error only after more than Timeout ticks (UM1725: Timeout duration)
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive with timeout 0 fails at once when no address is pending") {
	bring();
	REQUIRE(initOk());
	uint8_t rx[1] = {0};
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 0) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive times out without data after the address and switches the acknowledge off") {
	bring();
	REQUIRE(initOk());
	uint8_t rx[2] = {0, 0};
	Script s;
	s.at(3, OP_START, 0x30);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 2, 10) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive times out when the stop never comes after the last byte") {
	bring();
	REQUIRE(initOk());
	uint8_t rx[1] = {0};
	Script s;
	rxScript(s, 0x30, 1, kData, false);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 10) == HAL_ERROR);
	CHECK(rx[0] == kData[0]);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive reports an acknowledge failure seen while it waits for the stop") {
	bring();
	REQUIRE(initOk());
	uint8_t rx[1] = {0};
	Script s;
	rxScript(s, 0x30, 1, kData, false);
	s.at(6, OP_RAISE, 0, 0, I2C_SR1_AF);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 100) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((sr1() & I2C_SR1_AF) == 0u); // AF cleared by the HAL
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(Slave, "characterization: blocking slave receive returns HAL_ERROR with ErrorCode NONE when a stop ends the transfer early") {
	// NEEDS REVIEW 322: UM1725 does not say what a stop before Size bytes returns. The HAL returns HAL_ERROR, state READY, STOPF cleared and
	// leaves ErrorCode at NONE (it ORs HAL_I2C_ERROR_NONE, a no-op).
	bring();
	REQUIRE(initOk());
	uint8_t rx[3] = {0, 0, 0};
	Script s;
	rxScript(s, 0x30, 1, kData);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 3, 100) == HAL_ERROR);
	CHECK(rx[0] == kData[0]);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((sr1() & I2C_SR1_STOPF) == 0u);
}

TEST_CASE_FIXTURE(Slave, "characterization: blocking slave receive leaves the acknowledge on after an address timeout") {
	// NEEDS REVIEW 322: on the address-wait timeout the HAL returns without clearing CR1.ACK (it does on the data and stop failures), so the
	// peripheral keeps acknowledging its address while the handle is READY. UM1725 says nothing.
	bring();
	REQUIRE(initOk());
	uint8_t rx[1] = {0};
	REQUIRE(HAL_I2C_Slave_Receive(&h, rx, 1, 3) == HAL_ERROR);
	CHECK((cr1() & I2C_CR1_ACK) == I2C_CR1_ACK);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive switches the acknowledge off when a stop ends the transfer early") {
	bring();
	REQUIRE(initOk());
	uint8_t rx[3] = {0, 0, 0};
	Script s;
	rxScript(s, 0x30, 1, kData);
	s.go();
	REQUIRE(HAL_I2C_Slave_Receive(&h, rx, 3, 100) == HAL_ERROR);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive returns HAL_BUSY and touches nothing when the state is not READY") {
	const HAL_I2C_StateTypeDef states[] = {HAL_I2C_STATE_RESET,        HAL_I2C_STATE_BUSY,          HAL_I2C_STATE_BUSY_TX,
	                                       HAL_I2C_STATE_BUSY_RX,      HAL_I2C_STATE_LISTEN,        HAL_I2C_STATE_BUSY_TX_LISTEN,
	                                       HAL_I2C_STATE_BUSY_RX_LISTEN, HAL_I2C_STATE_ABORT,         HAL_I2C_STATE_TIMEOUT,
	                                       HAL_I2C_STATE_ERROR};
	bring();
	REQUIRE(initOk());
	uint8_t rx[1] = {0};
	for (HAL_I2C_StateTypeDef st : states) {
		h.State = st;
		h.Mode = HAL_I2C_MODE_MASTER;
		h.ErrorCode = HAL_I2C_ERROR_BERR;
		const uint32_t cr1Before = cr1();
		INFO("state " << (unsigned)st);
		CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 100) == HAL_BUSY);
		CHECK(h.State == st);
		CHECK(h.Mode == HAL_I2C_MODE_MASTER);
		CHECK(h.ErrorCode == HAL_I2C_ERROR_BERR);
		CHECK(h.Lock == HAL_UNLOCKED);
		CHECK(cr1() == cr1Before);
	}
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive rejects size 0 and a null buffer without changing the handle") {
	bring();
	REQUIRE(initOk());
	uint8_t rx[1] = {0};
	const uint32_t cr1Before = cr1();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 0, 100) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Receive(&h, nullptr, 1, 100) == HAL_ERROR);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(cr1() == cr1Before);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive with a 10-bit own address needs the header and the second address byte") {
	// own address 0x2A5: bits 9:8 = 10b, header 11110 10 0 = 0xF4 (write), low byte 0xA5 (RM0090 27.3.2, 10-bit slave).
	bring(0, 0x2A5);
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	REQUIRE(initOk());
	uint8_t rx[2] = {0, 0};
	Script s;
	s.at(3, OP_START, 0xF4);
	s.at(4, OP_WRITE, 0xA5);
	s.at(5, OP_WRITE, 0x77);
	s.at(6, OP_WRITE, 0x88);
	s.at(7, OP_STOP);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 2, 100) == HAL_OK);
	CHECK(s.steps[0].ret == 1);
	CHECK(s.steps[1].ret == 1);
	CHECK(rx[0] == 0x77);
	CHECK(rx[1] == 0x88);
}

// ------------------------------------------------------------------------------------------------ IS2 Slave_Transmit
TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit sends 1, 2, 3 and 5 bytes in order and ends on the nack of the controller") {
	for (unsigned n : {1u, 2u, 3u, 5u}) {
		bring();
		h.State = HAL_I2C_STATE_RESET;
		REQUIRE(initOk());
		uint8_t tx[5];
		std::memcpy(tx, kData, 5);
		Script s;
		txScript(s, 0x31, n);
		s.go();
		INFO("n = " << n);
		hostI2cLogClear(0);
		CHECK(HAL_I2C_Slave_Transmit(&h, tx, (uint16_t)n, 100) == HAL_OK);
		CHECK(s.steps[0].ret == 1);
		for (unsigned k = 0; k < n; ++k) {
			CHECK(s.steps[1 + k].ret == (k + 1 < n ? 1 : 0));
			CHECK(s.steps[1 + k].out == kData[k]);
		}
		CHECK(hostI2cLogCount(0) >= n);
		i2cScriptStop();
		HAL_I2C_DeInit(&h);
	}
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit works on I2C1, I2C2 and I2C3") {
	for (unsigned inst = 0; inst < 3; ++inst) {
		bring(inst);
		REQUIRE(initOk());
		uint8_t tx[2] = {0x44, 0x55};
		Script s;
		txScript(s, 0x31, 2);
		s.go();
		INFO("instance " << inst);
		CHECK(HAL_I2C_Slave_Transmit(&h, tx, 2, 100) == HAL_OK);
		CHECK(s.steps[1].out == 0x44);
		CHECK(s.steps[2].out == 0x55);
		i2cScriptStop();
	}
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit sets BUSY_TX, slave mode and the acknowledge while it waits for the address") {
	bring();
	REQUIRE(initOk());
	uint8_t tx[1] = {0x12};
	Script s;
	s.at(2, OP_SNAP);
	txScript(s, 0x31, 1);
	s.go();
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 1, 100) == HAL_OK);
	CHECK(s.steps[0].state == (int)HAL_I2C_STATE_BUSY_TX);
	CHECK(s.steps[0].mode == (int)HAL_I2C_MODE_SLAVE);
	CHECK((s.steps[0].cr1 & I2C_CR1_ACK) == I2C_CR1_ACK);
	CHECK(s.steps[0].err == HAL_I2C_ERROR_NONE);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit ends with state READY, mode NONE, ErrorCode NONE, AF cleared, acknowledge off, unlocked") {
	bring();
	REQUIRE(initOk());
	h.ErrorCode = HAL_I2C_ERROR_OVR;
	uint8_t tx[2] = {0x01, 0x02};
	Script s;
	txScript(s, 0x31, 2);
	s.go();
	REQUIRE(HAL_I2C_Slave_Transmit(&h, tx, 2, 100) == HAL_OK);
	CHECK(h.pBuffPtr == tx + 2);
	CHECK(h.XferCount == 0u);
	CHECK(h.XferSize == 0u);
	CHECK((cr2() & (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN)) == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((sr1() & I2C_SR1_AF) == 0u);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
	CHECK((cr1() & I2C_CR1_PE) == I2C_CR1_PE);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit enables the peripheral when PE is 0 and clears POS") {
	bring();
	REQUIRE(initOk());
	h.Instance->CR1 &= ~I2C_CR1_PE;
	h.Instance->CR1 |= I2C_CR1_POS;
	uint8_t tx[1] = {0x12};
	Script s;
	s.at(2, OP_SNAP);
	txScript(s, 0x31, 1);
	s.go();
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 1, 100) == HAL_OK);
	CHECK((s.steps[0].cr1 & I2C_CR1_PE) == I2C_CR1_PE);
	CHECK((s.steps[0].cr1 & I2C_CR1_POS) == 0u);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit sends the same data again in a second call") {
	bring();
	REQUIRE(initOk());
	uint8_t tx[2] = {0x61, 0x62};
	for (unsigned k = 0; k < 2; ++k) {
		Script s;
		txScript(s, 0x31, 2);
		s.go();
		CHECK(HAL_I2C_Slave_Transmit(&h, tx, 2, 100) == HAL_OK);
		CHECK(s.steps[1].out == 0x61);
		CHECK(s.steps[2].out == 0x62);
		i2cScriptStop();
	}
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit times out when nobody addresses it") {
	bring();
	REQUIRE(initOk());
	uint8_t tx[1] = {1};
	const uint32_t t0 = uwTick;
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 1, 5) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(uwTick - t0 > 5u);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit with timeout 0 fails at once when no address is pending") {
	bring();
	REQUIRE(initOk());
	uint8_t tx[1] = {1};
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 1, 0) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit times out when the controller never reads the bytes it was given") {
	// Size 2: both bytes are loaded (shift register and DR), then the HAL waits for AF (nack of the controller).
	bring();
	REQUIRE(initOk());
	uint8_t tx[2] = {1, 2};
	Script s;
	s.at(3, OP_START, 0x31);
	s.go();
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 2, 10) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit times out waiting for TXE when the controller reads nothing and more bytes remain") {
	// Size 3: shift register and DR are full after two bytes, TXE stays 0, no byte is taken (RM0090 27.6.6 TxE).
	bring();
	REQUIRE(initOk());
	uint8_t tx[3] = {1, 2, 3};
	Script s;
	s.at(3, OP_START, 0x31);
	s.go();
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 3, 10) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit stalls with BTF set when the controller asks for more bytes than it was given") {
	// RM0090 27.3.2: with nothing to send (BTF = 1, TxE = 1) SCL stays low. The HAL waits for AF and times out.
	bring();
	REQUIRE(initOk());
	uint8_t tx[1] = {0x9C};
	Script s;
	s.at(3, OP_START, 0x31);
	s.at(4, OP_READ, 0, 1);
	s.at(5, OP_READ, 0, 1);
	s.go();
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 1, 10) == HAL_ERROR);
	CHECK(s.steps[1].ret == 1);
	CHECK(s.steps[1].out == 0x9C);
	CHECK(s.steps[2].ret == -1); // stretched
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK((sr1() & I2C_SR1_BTF) == I2C_SR1_BTF);
}

TEST_CASE_FIXTURE(Slave, "characterization: blocking slave transmit returns HAL_ERROR with AF when the controller stops reading before all bytes were given") {
	// NEEDS REVIEW 322: UM1725 does not say. Size 4, the controller nacks the first byte: the TXE wait sees AF and fails.
	bring();
	REQUIRE(initOk());
	uint8_t tx[4] = {1, 2, 3, 4};
	Script s;
	s.at(3, OP_START, 0x31);
	s.at(4, OP_READ, 0, 0);
	s.go();
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 4, 100) == HAL_ERROR);
	CHECK(s.steps[1].out == 1);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((sr1() & I2C_SR1_AF) == 0u);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit returns HAL_BUSY and touches nothing when the state is not READY") {
	const HAL_I2C_StateTypeDef states[] = {HAL_I2C_STATE_RESET,        HAL_I2C_STATE_BUSY,          HAL_I2C_STATE_BUSY_TX,
	                                       HAL_I2C_STATE_BUSY_RX,      HAL_I2C_STATE_LISTEN,        HAL_I2C_STATE_BUSY_TX_LISTEN,
	                                       HAL_I2C_STATE_BUSY_RX_LISTEN, HAL_I2C_STATE_ABORT,         HAL_I2C_STATE_TIMEOUT,
	                                       HAL_I2C_STATE_ERROR};
	bring();
	REQUIRE(initOk());
	uint8_t tx[1] = {1};
	for (HAL_I2C_StateTypeDef st : states) {
		h.State = st;
		h.Mode = HAL_I2C_MODE_MASTER;
		h.ErrorCode = HAL_I2C_ERROR_BERR;
		const uint32_t cr1Before = cr1();
		INFO("state " << (unsigned)st);
		CHECK(HAL_I2C_Slave_Transmit(&h, tx, 1, 100) == HAL_BUSY);
		CHECK(h.State == st);
		CHECK(h.Mode == HAL_I2C_MODE_MASTER);
		CHECK(h.ErrorCode == HAL_I2C_ERROR_BERR);
		CHECK(h.Lock == HAL_UNLOCKED);
		CHECK(cr1() == cr1Before);
	}
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit rejects size 0 and a null buffer without changing the handle") {
	bring();
	REQUIRE(initOk());
	uint8_t tx[1] = {1};
	const uint32_t cr1Before = cr1();
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 0, 100) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Transmit(&h, nullptr, 1, 100) == HAL_ERROR);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(cr1() == cr1Before);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit with a 10-bit own address needs the write header, the address byte and the read header") {
	// own address 0x2A5: write header 0xF4, address byte 0xA5, after the repeated start the read header 0xF5 (RM0090 27.3.2).
	bring(0, 0x2A5);
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	REQUIRE(initOk());
	uint8_t tx[2] = {0x3C, 0x4D};
	Script s;
	s.at(3, OP_START, 0xF4);
	s.at(4, OP_WRITE, 0xA5);
	s.at(5, OP_RESTART, 0xF5);
	s.at(6, OP_READ, 0, 1);
	s.at(7, OP_READ, 0, 0);
	s.go();
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 2, 100) == HAL_OK);
	CHECK(s.steps[0].ret == 1);
	CHECK(s.steps[1].ret == 1);
	CHECK(s.steps[2].ret == 1);
	CHECK(s.steps[3].out == 0x3C);
	CHECK(s.steps[4].out == 0x4D);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit with a 10-bit own address times out when the read header never follows") {
	bring(0, 0x2A5);
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	REQUIRE(initOk());
	uint8_t tx[1] = {0x3C};
	Script s;
	s.at(3, OP_START, 0xF4);
	s.at(4, OP_WRITE, 0xA5);
	s.go();
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 1, 10) == HAL_ERROR);
	CHECK(s.steps[1].ret == 1);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

// ------------------------------------------------------------------------------------------------ IS3 address match, listen mode
namespace {
struct Listen : Slave {
	void startListen(uint32_t own1 = 0x30) {
		bring(0, own1);
	}
	// the ADDR event: what the event interrupt does, then the application's part (clear ADDR)
	void serveAddr() {
		HAL_I2C_EV_IRQHandler(&h);
		__HAL_I2C_CLEAR_ADDRFLAG(&h);
	}
};
} // namespace

TEST_CASE_FIXTURE(Listen, "IS3 listen mode reports a write to the own 7-bit address as transmit direction with the own address as code") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x30) == 1);
	CHECK((sr1() & I2C_SR1_ADDR) == I2C_SR1_ADDR);
	HAL_I2C_EV_IRQHandler(&h);
	REQUIRE(i2cCounts.addr == 1u);
	// UM1725: TransferDirection is the direction the master requests (Write / Read); the controller writes: I2C_DIRECTION_TRANSMIT
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_TRANSMIT);
	CHECK(i2cCounts.addrMatchCode == 0x30u);
}

TEST_CASE_FIXTURE(Listen, "IS3 listen mode reports a read from the own 7-bit address as receive direction") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x31) == 1);
	HAL_I2C_EV_IRQHandler(&h);
	REQUIRE(i2cCounts.addr == 1u);
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_RECEIVE);
	CHECK(i2cCounts.addrMatchCode == 0x30u);
}

TEST_CASE_FIXTURE(Listen, "IS3 listen mode works on I2C1, I2C2 and I2C3") {
	for (unsigned inst = 0; inst < 3; ++inst) {
		i2cSupportReset();
		bring(inst);
		REQUIRE(initOk());
		REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
		INFO("instance " << inst);
		CHECK(hostI2cExtStart(inst, 0x30) == 1);
		HAL_I2C_EV_IRQHandler(&h);
		CHECK(i2cCounts.addr == 1u);
		CHECK(i2cCounts.last == &h);
		__HAL_I2C_CLEAR_ADDRFLAG(&h);
		hostI2cExtStop(inst);
		HAL_I2C_EV_IRQHandler(&h);
		CHECK(i2cCounts.listen == 1u);
	}
}

TEST_CASE_FIXTURE(Listen, "IS3 addresses that are not the own address are not acknowledged and raise nothing") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	for (uint8_t a : {0x32, 0x2E, 0x2F, 0x60, 0x20, 0xFE, 0x01, 0x00}) {
		INFO("address byte " << (unsigned)a);
		CHECK(hostI2cExtStart(0, a) == 0);
	}
	CHECK((sr1() & I2C_SR1_ADDR) == 0u);
	CHECK(hostI2cEventPending(0) == 0);
	CHECK(i2cCounts.addr == 0u);
}

TEST_CASE_FIXTURE(Listen, "IS3 the second own address with dual addressing reports DUALF and OwnAddress2 as code") {
	startListen();
	h.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
	h.Init.OwnAddress2 = 0x40;
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x40) == 1);
	CHECK((sr2() & I2C_SR2_DUALF) == I2C_SR2_DUALF);
	HAL_I2C_EV_IRQHandler(&h);
	REQUIRE(i2cCounts.addr == 1u);
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_TRANSMIT);
	CHECK(i2cCounts.addrMatchCode == 0x40u);
}

TEST_CASE_FIXTURE(Listen, "IS3 the first own address still reports OwnAddress1 and DUALF 0 when dual addressing is on") {
	startListen();
	h.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
	h.Init.OwnAddress2 = 0x40;
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x31) == 1);
	CHECK((sr2() & I2C_SR2_DUALF) == 0u);
	HAL_I2C_EV_IRQHandler(&h);
	REQUIRE(i2cCounts.addr == 1u);
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_RECEIVE);
	CHECK(i2cCounts.addrMatchCode == 0x30u);
}

TEST_CASE_FIXTURE(Listen, "IS3 a read from the second own address reports the receive direction") {
	startListen();
	h.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
	h.Init.OwnAddress2 = 0x40;
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x41) == 1);
	HAL_I2C_EV_IRQHandler(&h);
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_RECEIVE);
	CHECK(i2cCounts.addrMatchCode == 0x40u);
}

TEST_CASE_FIXTURE(Listen, "IS3 the second own address is ignored when dual addressing is off") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x40) == 0);
	CHECK((sr1() & I2C_SR1_ADDR) == 0u);
}

TEST_CASE_FIXTURE(Listen, "IS3 the general call is answered when ENGC is set and sets GENCALL") {
	startListen();
	h.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x00) == 1);
	CHECK((sr2() & I2C_SR2_GENCALL) == I2C_SR2_GENCALL);
	CHECK((sr2() & I2C_SR2_TRA) == 0u); // the general call is a write
	HAL_I2C_EV_IRQHandler(&h);
	REQUIRE(i2cCounts.addr == 1u);
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_TRANSMIT);
}

TEST_CASE_FIXTURE(Listen, "characterization: the match code of a general call is OwnAddress1") {
	// NEEDS REVIEW 322: UM1725 does not define AddrMatchCode for a general call; the HAL reports OwnAddress1 (DUALF = 0).
	startListen();
	h.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x00) == 1);
	HAL_I2C_EV_IRQHandler(&h);
	CHECK(i2cCounts.addrMatchCode == 0x30u);
}

TEST_CASE_FIXTURE(Listen, "IS3 the general call is ignored when ENGC is off") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x00) == 0);
	CHECK((sr1() & I2C_SR1_ADDR) == 0u);
	CHECK((sr2() & I2C_SR2_GENCALL) == 0u);
}

TEST_CASE_FIXTURE(Listen, "IS3 listen mode with a 10-bit own address reports the write after the second address byte and the read after the repeated start") {
	// own address 0x2A5: header 0xF4 + address byte 0xA5 = write (target receives); restart with 0xF5 = read (target transmits).
	startListen(0x2A5);
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0xF4) == 1);
	CHECK((sr1() & I2C_SR1_ADDR) == 0u); // the header alone does not complete the address
	CHECK(hostI2cExtWrite(0, 0xA5) == 1);
	CHECK((sr1() & I2C_SR1_ADDR) == I2C_SR1_ADDR);
	serveAddr();
	REQUIRE(i2cCounts.addr == 1u);
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_TRANSMIT);
	CHECK(i2cCounts.addrMatchCode == 0x2A5u);
	CHECK(hostI2cExtRestart(0, 0xF5) == 1);
	serveAddr();
	REQUIRE(i2cCounts.addr == 2u);
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_RECEIVE);
	CHECK(i2cCounts.addrMatchCode == 0x2A5u);
}

TEST_CASE_FIXTURE(Listen, "IS3 a 10-bit header with other address bits 9:8 is ignored and a wrong second byte is not acknowledged") {
	startListen(0x2A5);
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0xF2) == 0); // bits 9:8 = 01b
	CHECK(hostI2cExtStart(0, 0xF6) == 0); // 11b
	CHECK(hostI2cExtStart(0, 0xF0) == 0); // 00b
	CHECK(hostI2cExtStart(0, 0x30) == 0); // a 7-bit address is not the own address in 10-bit mode
	CHECK(hostI2cExtStart(0, 0xF4) == 1);
	CHECK(hostI2cExtWrite(0, 0xA4) == 0); // wrong low byte
	CHECK((sr1() & I2C_SR1_ADDR) == 0u);
	CHECK(i2cCounts.addr == 0u);
}

TEST_CASE_FIXTURE(Listen, "IS3 an address event while the handle is not listening is cleared and raises no callback") {
	// Slave mode, state READY, the event interrupt enabled by the application: I2C_Slave_ADDR clears ADDR and unlocks.
	startListen();
	REQUIRE(initOk());
	h.Instance->CR1 |= I2C_CR1_ACK;
	h.Instance->CR2 |= I2C_CR2_ITEVTEN;
	h.Lock = HAL_LOCKED;
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	REQUIRE((sr1() & I2C_SR1_ADDR) == I2C_SR1_ADDR);
	HAL_I2C_EV_IRQHandler(&h);
	CHECK((sr1() & I2C_SR1_ADDR) == 0u);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(i2cCounts.addr == 0u);
}

TEST_CASE_FIXTURE(Listen, "IS3 the address callback runs with the process unlocked and the BUF interrupt off") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	h.Instance->CR2 |= I2C_CR2_ITBUFEN;
	h.Lock = HAL_LOCKED;
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	HAL_I2C_EV_IRQHandler(&h);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr2() & I2C_CR2_ITBUFEN) == 0u);
	CHECK(i2cCounts.stateAtLast == HAL_I2C_STATE_LISTEN);
}

// ------------------------------------------------------------------------------------------------ IS4 listen
TEST_CASE_FIXTURE(Listen, "IS4 EnableListen from READY sets state LISTEN, PE, ACK and the event and error interrupts") {
	startListen();
	REQUIRE(initOk());
	h.Instance->CR1 &= ~I2C_CR1_PE;
	CHECK(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
	CHECK((cr1() & I2C_CR1_PE) == I2C_CR1_PE);
	CHECK((cr1() & I2C_CR1_ACK) == I2C_CR1_ACK);
	CHECK((cr2() & I2C_CR2_ITEVTEN) == I2C_CR2_ITEVTEN);
	CHECK((cr2() & I2C_CR2_ITERREN) == I2C_CR2_ITERREN);
	CHECK((cr2() & I2C_CR2_ITBUFEN) == 0u);
}

TEST_CASE_FIXTURE(Listen, "IS4 EnableListen returns HAL_BUSY and changes nothing when the state is not READY") {
	const HAL_I2C_StateTypeDef states[] = {HAL_I2C_STATE_RESET,        HAL_I2C_STATE_BUSY,          HAL_I2C_STATE_BUSY_TX,
	                                       HAL_I2C_STATE_BUSY_RX,      HAL_I2C_STATE_LISTEN,        HAL_I2C_STATE_BUSY_TX_LISTEN,
	                                       HAL_I2C_STATE_BUSY_RX_LISTEN, HAL_I2C_STATE_ABORT,         HAL_I2C_STATE_TIMEOUT,
	                                       HAL_I2C_STATE_ERROR};
	startListen();
	REQUIRE(initOk());
	const uint32_t cr1Before = cr1();
	const uint32_t cr2Before = cr2();
	for (HAL_I2C_StateTypeDef st : states) {
		h.State = st;
		INFO("state " << (unsigned)st);
		CHECK(HAL_I2C_EnableListen_IT(&h) == HAL_BUSY);
		CHECK(h.State == st);
		CHECK(cr1() == cr1Before);
		CHECK(cr2() == cr2Before);
	}
}

TEST_CASE_FIXTURE(Listen, "IS4 EnableListen on an already enabled peripheral keeps PE and works") {
	startListen();
	REQUIRE(initOk());
	REQUIRE((cr1() & I2C_CR1_PE) == I2C_CR1_PE);
	CHECK(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK((cr1() & I2C_CR1_PE) == I2C_CR1_PE);
}

TEST_CASE_FIXTURE(Listen, "IS4 DisableListen from LISTEN returns to READY, mode NONE, ACK off and the interrupts off") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	h.Mode = HAL_I2C_MODE_SLAVE;
	CHECK(HAL_I2C_DisableListen_IT(&h) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
	CHECK((cr2() & I2C_CR2_ITEVTEN) == 0u);
	CHECK((cr2() & I2C_CR2_ITERREN) == 0u);
}

TEST_CASE_FIXTURE(Listen, "IS4 DisableListen returns HAL_BUSY and changes nothing in any state but LISTEN") {
	const HAL_I2C_StateTypeDef states[] = {HAL_I2C_STATE_RESET,        HAL_I2C_STATE_READY,         HAL_I2C_STATE_BUSY,
	                                       HAL_I2C_STATE_BUSY_TX,      HAL_I2C_STATE_BUSY_RX,       HAL_I2C_STATE_BUSY_TX_LISTEN,
	                                       HAL_I2C_STATE_BUSY_RX_LISTEN, HAL_I2C_STATE_ABORT,         HAL_I2C_STATE_TIMEOUT,
	                                       HAL_I2C_STATE_ERROR};
	startListen();
	REQUIRE(initOk());
	h.Instance->CR1 |= I2C_CR1_ACK;
	h.Instance->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
	const uint32_t cr1Before = cr1();
	const uint32_t cr2Before = cr2();
	for (HAL_I2C_StateTypeDef st : states) {
		h.State = st;
		h.Mode = HAL_I2C_MODE_SLAVE;
		INFO("state " << (unsigned)st);
		CHECK(HAL_I2C_DisableListen_IT(&h) == HAL_BUSY);
		CHECK(h.State == st);
		CHECK(h.Mode == HAL_I2C_MODE_SLAVE);
		CHECK(cr1() == cr1Before);
		CHECK(cr2() == cr2Before);
	}
}

TEST_CASE_FIXTURE(Listen, "IS4 Enable and Disable listen can be repeated") {
	startListen();
	REQUIRE(initOk());
	for (unsigned k = 0; k < 3; ++k) {
		CHECK(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
		CHECK(h.State == HAL_I2C_STATE_LISTEN);
		CHECK(HAL_I2C_DisableListen_IT(&h) == HAL_OK);
		CHECK(h.State == HAL_I2C_STATE_READY);
	}
}

TEST_CASE_FIXTURE(Listen, "IS4 a stop after an address match ends the listen: ListenCplt, state READY, ACK and interrupts off") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	serveAddr();
	hostI2cExtStop(0);
	REQUIRE((sr1() & I2C_SR1_STOPF) == I2C_SR1_STOPF);
	HAL_I2C_EV_IRQHandler(&h);
	CHECK(i2cCounts.listen == 1u);
	CHECK(i2cCounts.stateAtLast == HAL_I2C_STATE_READY);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK((sr1() & I2C_SR1_STOPF) == 0u);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
	CHECK((cr2() & (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN)) == 0u);
	REQUIRE(i2cCounts.orderCount == 2u);
	CHECK(i2cCounts.order[0] == (unsigned)I2C_CB_ADDR);
	CHECK(i2cCounts.order[1] == (unsigned)I2C_CB_LISTEN);
}

TEST_CASE_FIXTURE(Listen, "IS4 an unread byte at the stop is stored but does not fail the listen") {
	// no transfer is armed (XferCount 0): the byte stays in DR, ListenCplt still comes
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	serveAddr();
	CHECK(hostI2cExtWrite(0, 0x42) == 1);
	hostI2cExtStop(0);
	HAL_I2C_EV_IRQHandler(&h);
	CHECK(i2cCounts.listen == 1u);
	CHECK(i2cCounts.error == 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
}

TEST_CASE_FIXTURE(Listen, "IS4 a listen that is never addressed raises no callback and stays LISTEN") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x50) == 0);
	CHECK(i2cRunIrqs(&h, 0) == 0u);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
	CHECK(i2cCounts.orderCount == 0u);
}

TEST_CASE_FIXTURE(Listen, "characterization: a not acknowledged read in listen mode ends the listen with an error callback and ListenCplt") {
	// RM0090 27.3.2 / 27.3.4: the controller's nack on a transmitted byte sets AF (no STOPF). The HAL (state LISTEN, no own transfer)
	// takes the generic AF path: ErrorCode AF, ErrorCallback, then ListenCplt.
	// NEEDS REVIEW 322: UM1725 does not say that the error callback comes first and that it comes at all for a nack in listen mode.
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	serveAddr();
	h.Instance->DR = 0x5B; // the application loads a byte
	uint8_t b = 0;
	CHECK(hostI2cExtReadByte(0, 0, &b) == 0);
	CHECK(b == 0x5B);
	REQUIRE((sr1() & I2C_SR1_AF) == I2C_SR1_AF);
	CHECK(i2cRunIrqs(&h, 0) >= 1u);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.errorCodeAtError == HAL_I2C_ERROR_AF);
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.XferOptions == kNoOption);
	CHECK(h.PreviousState == 0u); // I2C_STATE_NONE
	CHECK((sr1() & I2C_SR1_AF) == 0u);
	REQUIRE(i2cCounts.orderCount == 3u);
	CHECK(i2cCounts.order[1] == (unsigned)I2C_CB_ERROR);
	CHECK(i2cCounts.order[2] == (unsigned)I2C_CB_LISTEN);
}

TEST_CASE_FIXTURE(Listen, "IS4 starting an own interrupt transfer while listening is refused with HAL_BUSY") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	uint8_t buf[2] = {0, 0};
	CHECK(HAL_I2C_Slave_Receive_IT(&h, buf, 2) == HAL_BUSY);
	CHECK(HAL_I2C_Slave_Transmit_IT(&h, buf, 2) == HAL_BUSY);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
}

TEST_CASE_FIXTURE(Listen, "IS4 blocking slave calls are refused with HAL_BUSY while listening") {
	startListen();
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	uint8_t buf[2] = {0, 0};
	CHECK(HAL_I2C_Slave_Receive(&h, buf, 2, 10) == HAL_BUSY);
	CHECK(HAL_I2C_Slave_Transmit(&h, buf, 2, 10) == HAL_BUSY);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
}

TEST_CASE_FIXTURE(Listen, "IS4 a nack at the end of an interrupt slave transmit completes it with the slave transmit callback") {
	// Slave_Transmit_IT of 1 byte, all bytes given, the controller nacks the (last) byte: AF with XferCount 0 is the normal end
	// (RM0090 27.3.2 slave transmitter: the nack ends the transfer).
	startListen();
	REQUIRE(initOk());
	uint8_t tx[1] = {0x6D};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 1) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	i2cRunIrqs(&h, 0);
	uint8_t b = 0;
	CHECK(hostI2cExtReadByte(0, 0, &b) == 0);
	CHECK(b == 0x6D);
	i2cRunIrqs(&h, 0);
	CHECK(i2cCounts.slaveTx == 1u);
	CHECK(i2cCounts.error == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK((sr1() & I2C_SR1_AF) == 0u);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(Listen, "characterization: a stop with the interrupt slave receive unfinished reports an error through ErrorCode AF") {
	// Slave_Receive_IT of 3 bytes, the controller sends 1 byte and stops: not all data came, HAL sets AF (UM1725 silent) and
	// the error callback runs.
	startListen();
	REQUIRE(initOk());
	uint8_t rx[3] = {0, 0, 0};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	i2cRunIrqs(&h, 0);
	REQUIRE(hostI2cExtWrite(0, 0x21) == 1);
	i2cRunIrqs(&h, 0);
	hostI2cExtStop(0);
	i2cRunIrqs(&h, 0);
	CHECK(rx[0] == 0x21);
	CHECK(i2cCounts.error == 1u);
	CHECK((i2cCounts.errorCodeAtError & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(Listen, "IS4 an interrupt slave receive that got all its bytes completes with the slave receive callback at the stop") {
	startListen();
	REQUIRE(initOk());
	uint8_t rx[2] = {0, 0};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	i2cRunIrqs(&h, 0);
	REQUIRE(hostI2cExtWrite(0, 0x21) == 1);
	i2cRunIrqs(&h, 0);
	REQUIRE(hostI2cExtWrite(0, 0x22) == 1);
	i2cRunIrqs(&h, 0);
	hostI2cExtStop(0);
	i2cRunIrqs(&h, 0);
	CHECK(rx[0] == 0x21);
	CHECK(rx[1] == 0x22);
	CHECK(i2cCounts.slaveRx == 1u);
	CHECK(i2cCounts.error == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(Listen, "characterization: the stop handler stores the bytes still waiting in DR and in the shift register before it reports the shortfall") {
	// Slave_Receive_IT of 4 bytes; the controller sends 2 bytes (one in DR, one waiting: RXNE and BTF) and stops before the
	// interrupt ran. The HAL stores both bytes; 2 bytes are missing: ErrorCode AF (UM1725 silent, NEEDS REVIEW 322).
	startListen();
	REQUIRE(initOk());
	uint8_t rx[4] = {0, 0, 0, 0};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 4) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	i2cRunIrqs(&h, 0);
	REQUIRE(hostI2cExtWrite(0, 0x31) == 1);
	REQUIRE(hostI2cExtWrite(0, 0x32) == 1);
	REQUIRE((sr1() & I2C_SR1_BTF) == I2C_SR1_BTF);
	hostI2cExtStop(0);
	HAL_I2C_EV_IRQHandler(&h);
	CHECK(rx[0] == 0x31);
	CHECK(rx[1] == 0x32);
	CHECK(rx[2] == 0);
	CHECK(i2cCounts.error == 1u);
	CHECK((i2cCounts.errorCodeAtError & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF);
}

TEST_CASE_FIXTURE(Listen, "IS4 the stop handler stores a single byte waiting in DR") {
	startListen();
	REQUIRE(initOk());
	uint8_t rx[3] = {0, 0, 0};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	i2cRunIrqs(&h, 0);
	REQUIRE(hostI2cExtWrite(0, 0x41) == 1);
	REQUIRE((sr1() & I2C_SR1_RXNE) == I2C_SR1_RXNE);
	hostI2cExtStop(0);
	HAL_I2C_EV_IRQHandler(&h);
	CHECK(rx[0] == 0x41);
	CHECK(i2cCounts.error == 1u);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive stores no more than Size bytes when the controller sends more") {
	// buffer of 2 (guard byte behind it), the controller sends 3 bytes: the third stays in DR, the buffer behind Size is untouched
	bring();
	REQUIRE(initOk());
	uint8_t rx[3] = {0, 0, 0xEE};
	Script s;
	rxScript(s, 0x30, 3, kData);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 2, 100) == HAL_OK);
	CHECK(rx[0] == kData[0]);
	CHECK(rx[1] == kData[1]);
	CHECK(rx[2] == 0xEE);
	CHECK(h.pBuffPtr == rx + 2);
	CHECK(h.XferCount == 0u);
}

TEST_CASE_FIXTURE(Slave, "IS1 blocking slave receive with two bytes in flight stops after Size even when a byte waits in the shift register") {
	// bytes 1 and 2 arrive in the same tick (RXNE and BTF), Size is 1: only the first is stored
	bring();
	REQUIRE(initOk());
	uint8_t rx[2] = {0, 0xEE};
	Script s;
	s.at(3, OP_START, 0x30);
	s.at(4, OP_WRITE, 0x11);
	s.at(4, OP_WRITE, 0x22);
	s.at(5, OP_STOP);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 100) == HAL_OK);
	CHECK(rx[0] == 0x11);
	CHECK(rx[1] == 0xEE);
	CHECK(h.pBuffPtr == rx + 1);
}

TEST_CASE_FIXTURE(Slave, "IS2 blocking slave transmit loads exactly Size bytes into DR") {
	bring();
	REQUIRE(initOk());
	uint8_t tx[3] = {0x01, 0x02, 0x03};
	Script s;
	txScript(s, 0x31, 2); // the controller stops after 2 reads (the last one nacked)
	s.go();
	// Size 2: the third byte is never written, so the controller's second read ends the transfer
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 2, 100) == HAL_OK);
	CHECK(h.pBuffPtr == tx + 2);
	CHECK(h.XferCount == 0u);
	CHECK(s.steps[2].out == 0x02);
}

TEST_CASE_FIXTURE(Listen, "IS4 an interrupt slave transmit that ends with the nack of the controller leaves TXE clear (flush)") {
	// RM0090 27.6.6: TxE is not set after a nack. I2C_Flush_DR (called from the AF handler) must leave TXE clear and write nothing.
	startListen();
	REQUIRE(initOk());
	uint8_t tx[1] = {0x6D};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 1) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	i2cRunIrqs(&h, 0);
	uint8_t b = 0;
	CHECK(hostI2cExtReadByte(0, 0, &b) == 0);
	i2cRunIrqs(&h, 0);
	REQUIRE(i2cCounts.slaveTx == 1u);
	CHECK((sr1() & I2C_SR1_TXE) == 0u);
	CHECK(hostI2cStrayDrWrites(0) == 0u);
	CHECK(h.XferOptions == kNoOption);
}

TEST_CASE_FIXTURE(Listen, "IS4 the interrupt slave calls enable the event, buffer and error interrupts, and set XferOptions to no option") {
	startListen();
	REQUIRE(initOk());
	uint8_t buf[2] = {0, 0};
	h.XferOptions = 0x12345678u;
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, buf, 2) == HAL_OK);
	CHECK(h.XferOptions == kNoOption);
	CHECK((cr2() & (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN)) == (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN));
	CHECK(h.pBuffPtr == buf);
	CHECK(h.XferCount == 2u);
	HAL_I2C_DeInit(&h);
	cfg(h, 0);
	REQUIRE(initOk());
	h.XferOptions = 0x12345678u;
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, buf, 2) == HAL_OK);
	CHECK(h.XferOptions == kNoOption);
}

TEST_CASE_FIXTURE(Listen, "characterization: an own address with ACK off sets ADDR and is not acknowledged, a later blocking receive sees ADDR at once") {
	// NEEDS REVIEW 322: RM0090 27.3.2 says ADDR is set when the address matched; whether it is also set without ACK is not stated for the
	// not-acknowledged case, the model sets it. ExtStart returns 0 (nack).
	startListen();
	REQUIRE(initOk());
	REQUIRE((cr1() & I2C_CR1_ACK) == 0u);
	CHECK(hostI2cExtStart(0, 0x30) == 0);
	CHECK((sr1() & I2C_SR1_ADDR) == I2C_SR1_ADDR);
	uint8_t rx[1] = {0};
	Script s;
	s.at(4, OP_WRITE, 0x5A);
	s.at(5, OP_STOP);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 100) == HAL_OK);
	CHECK(rx[0] == 0x5A);
}

// ------------------------------------------------------------------------------------------------ IS5 NOSTRETCH
TEST_CASE_FIXTURE(Slave, "IS5 the blocking slave calls keep the NOSTRETCH bit set by init") {
	bring();
	h.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	REQUIRE(initOk());
	REQUIRE((cr1() & I2C_CR1_NOSTRETCH) == I2C_CR1_NOSTRETCH);
	uint8_t rx[1] = {0};
	Script s;
	rxScript(s, 0x30, 1, kData);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 100) == HAL_OK);
	CHECK((cr1() & I2C_CR1_NOSTRETCH) == I2C_CR1_NOSTRETCH);
	CHECK(rx[0] == kData[0]);
}

TEST_CASE_FIXTURE(Slave, "IS5 a byte that arrives before the previous one was read is lost and sets OVR with NOSTRETCH") {
	// RM0090 27.3.4: overrun in a slave receiver with clock stretching disabled: the received byte is discarded, OVR is set.
	bring();
	h.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	REQUIRE(initOk());
	uint8_t rx[1] = {0};
	Script s;
	s.at(3, OP_START, 0x30);
	s.at(4, OP_WRITE, 0x11);
	s.at(4, OP_WRITE, 0x22);
	s.at(5, OP_STOP);
	s.go();
	CHECK(HAL_I2C_Slave_Receive(&h, rx, 1, 100) == HAL_OK);
	CHECK(rx[0] == 0x11);
	CHECK((sr1() & I2C_SR1_OVR) == I2C_SR1_OVR);
}

TEST_CASE_FIXTURE(Slave, "characterization: the blocking slave receive neither reports nor clears OVR") {
	// NEEDS REVIEW 322: the polling function never looks at OVR; the flag stays set and ErrorCode stays NONE.
	bring();
	h.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	REQUIRE(initOk());
	uint8_t rx[1] = {0};
	Script s;
	s.at(3, OP_START, 0x30);
	s.at(4, OP_WRITE, 0x11);
	s.at(4, OP_WRITE, 0x22);
	s.at(5, OP_STOP);
	s.go();
	REQUIRE(HAL_I2C_Slave_Receive(&h, rx, 1, 100) == HAL_OK);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK((sr1() & I2C_SR1_OVR) == I2C_SR1_OVR);
}

TEST_CASE_FIXTURE(Slave, "IS5 with NOSTRETCH an underrun resends the last byte and sets OVR") {
	// RM0090 27.3.4: underrun in a slave transmitter with clock stretching disabled: the same byte is sent again, OVR is set.
	bring();
	h.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	REQUIRE(initOk());
	uint8_t tx[1] = {0x7E};
	Script s;
	s.at(3, OP_START, 0x31);
	s.at(4, OP_READ, 0, 1);
	s.at(5, OP_READ, 0, 0);
	s.go();
	CHECK(HAL_I2C_Slave_Transmit(&h, tx, 1, 100) == HAL_OK);
	CHECK(s.steps[1].out == 0x7E);
	CHECK(s.steps[2].out == 0x7E);
	CHECK(s.steps[2].ret == 0);
	CHECK((sr1() & I2C_SR1_OVR) == I2C_SR1_OVR);
}

TEST_CASE_FIXTURE(Listen, "IS5 an overrun in listen mode is reported by the error interrupt as ErrorCode OVR and the error callback") {
	startListen();
	h.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	REQUIRE(initOk());
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	serveAddr();
	CHECK(hostI2cExtWrite(0, 0x11) == 1);
	CHECK(hostI2cExtWrite(0, 0x22) == 1); // lost
	REQUIRE((sr1() & I2C_SR1_OVR) == I2C_SR1_OVR);
	REQUIRE(hostI2cErrorPending(0) == 1);
	// the application empties DR first: with no transfer armed the HAL's error path would store a pending byte through a null buffer
	// pointer (reported as a lead decision, not exercised)
	CHECK(h.Instance->DR == 0x11u);
	HAL_I2C_ER_IRQHandler(&h);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.errorCodeAtError == HAL_I2C_ERROR_OVR);
	CHECK((sr1() & I2C_SR1_OVR) == 0u);
	CHECK(h.State == HAL_I2C_STATE_LISTEN); // an error other than AF keeps the listen going
}
