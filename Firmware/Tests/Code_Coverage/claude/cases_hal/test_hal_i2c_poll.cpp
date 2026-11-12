/*
 * P12 T4c group I2P: the blocking controller functions of the real ST HAL I2C driver (stm32f4xx_hal_i2c.c: HAL_I2C_Master_Transmit,
 * Master_Receive, Mem_Write, Mem_Read, IsDeviceReady and the static helpers they use) against the I2C model (fakes/stm32_hal/HalI2c.h).
 * Requirements IP1-IP7 of Tests/Code_Coverage/claude/charters/hal/P12_T4C_REQUIREMENTS.md; compiled into ffb_hal_tests and
 * ffb_hal_tests_disco.
 *
 * Oracles: RM0090 Rev 22 chapter 27 (27.3.3 controller: Figure 243 transmitter, Figure 244 receiver procedures for 1, 2 and N bytes
 * incl. the 10-bit sequence, 27.3.4 errors: AF "software must generate a stop", ARLO, BERR; 27.6 registers) and UM1725 Rev 8 chapter
 * 36 (the functions' purpose, HAL_I2C_ERROR_* names, state names). The bus log (hostI2cLogAt) is the oracle of what went on the bus.
 * Hand derived log of a transfer to the device with the 7 bit address 0x50 (address byte 0xA0 for a write, 0xA1 for a read):
 *   write of n bytes: S, A0(ack), d1(ack) ... dn(ack), P
 *   read of n bytes:  S, A1(ack), d1(ack) ... d(n-1)(ack), dn(NACK), P   (RM0090 27.3.3: the controller NACKs the last byte)
 *   memory write:     S, A0, mem (MSB first for 16 bit), d1 ... dn, P
 *   memory read:      S, A0, mem, Sr (repeated start), A1, d1 ... dn(NACK), P
 *   10 bit address 0x2A5 (xx = 10b): header 11110xx0 = 0xF4, second byte 0xA5; a read goes on with Sr, header 11110xx1 = 0xF5.
 * The model's timing: a byte (address or data) takes `byteTicks` HAL ticks; one HAL tick passes per HAL_GetTick call.
 *
 * NEEDS REVIEW proposals (characterization cases, the lead assigns the numbers, proposals use NEEDS REVIEW 321):
 *  NEEDS REVIEW 321 characterization: a transmit / receive / memory call with Size 0 is accepted: address phase, STOP, HAL_OK (UM1725 is silent on Size 0).
 *  NEEDS REVIEW 321 characterization: a timeout in a blocking call returns HAL_ERROR (HAL_TIMEOUT only for IsDeviceReady's START wait) with
 *       ErrorCode HAL_I2C_ERROR_TIMEOUT; the START wait overrides it with HAL_I2C_WRONG_START (UM1725 names the timeout error code but not the
 *       return value; WRONG_START is not in UM1725).
 *  NEEDS REVIEW 321 characterization: the 25 ms BUSY flag wait ends with HAL_BUSY, State READY, ErrorCode TIMEOUT left set (UM1725 names HAL_BUSY only generally).
 *  NEEDS REVIEW 321 characterization: ARLO / BERR are not looked at by the blocking functions; the call ends with a timeout and the flag stays set.
 *  NEEDS REVIEW 321 characterization: IsDeviceReady with Trials 0 tries once (do-while).
 *  NEEDS REVIEW 321 characterization: MemAddSize that is neither 8 nor 16 bit is sent as 16 bit (assert_param off).
 *  NEEDS REVIEW 321 characterization: State RESET and a locked handle give HAL_BUSY.
 *  NEEDS REVIEW 321 characterization: a pending STOPF in the RXNE wait ends the receive with HAL_ERROR, no error code, State READY.
 *  NEEDS REVIEW 321 characterization: Mem_Read, 16 bit address on a slow bus, LSB nacked: the HAL sends the repeated start and the AF of the LSB ends the call
 *       (HAL_ERROR, AF) in the address wait of the read phase.
 *  NEEDS REVIEW 321 characterization: Mem_Read with a nacked memory address byte (8 bit address, or 16 bit with byteTicks 0) returns HAL_OK, sends the
 *       repeated start and reads, AF stays set, ErrorCode NONE (the HAL waits for TXE, which stays 1 after the first DR write, RM0090 27.6.6).
 */
#include "HalFixture.h"
#include "HalI2c.h"
#include "I2cTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>
#include <vector>

extern "C" uint32_t SystemCoreClock;

namespace {
typedef HostI2cEvent Ev;
Ev evS() { return Ev{HOST_I2C_EV_START, 0, 0}; }
Ev evSr() { return Ev{HOST_I2C_EV_RESTART, 0, 0}; }
Ev evP() { return Ev{HOST_I2C_EV_STOP, 0, 0}; }
Ev evO(uint8_t v, uint8_t ack = 1) { return Ev{HOST_I2C_EV_BYTE_OUT, v, ack}; }
Ev evI(uint8_t v, uint8_t ack = 1) { return Ev{HOST_I2C_EV_BYTE_IN, v, ack}; }

typedef std::vector<Ev> Log;

void expectLog(unsigned idx, const Log &want) {
	CHECK(hostI2cLogCount(idx) == want.size());
	for (size_t k = 0; k < want.size(); ++k) {
		INFO("log entry " << k);
		const Ev got = hostI2cLogAt(idx, (unsigned)k);
		CHECK(got.type == want[k].type);
		CHECK(got.value == want[k].value);
		CHECK(got.ack == want[k].ack);
	}
}

/* byte pattern: distinct for 256 values (7 is odd) and never the peer's 0xFF filler for n <= 255 starting at k = 0 */
uint8_t pat(unsigned k) { return (uint8_t)((k * 7u + 3u) & 0xFFu); }

Log txLog(uint8_t addrByte, const uint8_t *d, unsigned n) {
	Log l{evS(), evO(addrByte)};
	for (unsigned k = 0; k < n; ++k) l.push_back(evO(d[k]));
	l.push_back(evP());
	return l;
}
Log rxLog(uint8_t addrByte, const uint8_t *d, unsigned n) {
	Log l{evS(), evO(addrByte)};
	for (unsigned k = 0; k < n; ++k) l.push_back(evI(d[k], (uint8_t)(k + 1 < n)));
	l.push_back(evP());
	return l;
}

/* external event: when the bus log has reached a length, run fn once at the next HAL_GetTick (needs byteTicks > 0 so that a wait loop runs) */
struct Trig {
	unsigned idx, logAtLeast;
	void (*fn)(unsigned);
	bool fired;
};
Trig gTrig;
void trigStep() {
	HAL_IncTick();
	if (!gTrig.fired && hostI2cLogCount(gTrig.idx) >= gTrig.logAtLeast) {
		gTrig.fired = true;
		gTrig.fn(gTrig.idx);
	}
}
void trigArm(unsigned idx, unsigned logAtLeast, void (*fn)(unsigned)) {
	gTrig = Trig{idx, logAtLeast, fn, false};
	hostTickSetStep(trigStep);
}

struct Snap {
	I2C_HandleTypeDef *h;
	HAL_I2C_StateTypeDef state;
	HAL_I2C_ModeTypeDef mode;
	HAL_LockTypeDef lock;
	bool taken;
};
void snapFn(void *p) {
	Snap *s = static_cast<Snap *>(p);
	s->state = s->h->State;
	s->mode = s->h->Mode;
	s->lock = s->h->Lock;
	s->taken = true;
}

enum Fn { F_TX, F_RX, F_MW, F_MR };
const char *fnName(Fn f) { return f == F_TX ? "Master_Transmit" : f == F_RX ? "Master_Receive" : f == F_MW ? "Mem_Write" : "Mem_Read"; }
const Fn kAllFns[] = {F_TX, F_RX, F_MW, F_MR};

struct Rig : I2cFixture {
	I2C_HandleTypeDef h{};
	unsigned idx = 0;
	int dev = -1;
	Rig() { uwTickPrio = 15; }
	~Rig() { i2cScriptStop(); }

	void up(I2C_TypeDef *inst = I2C1, uint32_t mode = I2C_ADDRESSINGMODE_7BIT, uint16_t devAddr = 0x50, bool ten = false, uint32_t speed = 100000) {
		hostI2cResetState(); /* fresh peer devices, queues, log, byte time, fault hooks */
		h = I2C_HandleTypeDef{};
		SystemCoreClock = 168000000u;
		RCC->CFGR = (RCC->CFGR & ~(7u << 10)) | (5u << 10); /* PPRE1 /4 -> 42 MHz */
		h.Instance = inst;
		h.Init.ClockSpeed = speed;
		h.Init.DutyCycle = I2C_DUTYCYCLE_2;
		h.Init.OwnAddress1 = 0x30;
		h.Init.AddressingMode = mode;
		h.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		h.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		h.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
		idx = i2cIndexOf(inst);
		dev = hostI2cDeviceAdd(idx, devAddr, ten ? 1 : 0);
		hostI2cLogClear(idx);
	}
	void queue(unsigned n) {
		uint8_t d[256];
		for (unsigned k = 0; k < n && k < 256; ++k) d[k] = pat(k);
		hostI2cDeviceQueue(idx, dev, d, n > 256 ? 256 : n);
	}
	HAL_StatusTypeDef run(Fn f, uint8_t *buf, uint16_t n, uint32_t to, uint16_t dev16 = 0xA0, uint16_t mem = 0x12, uint32_t memSize = I2C_MEMADD_SIZE_8BIT) {
		switch (f) {
		case F_TX: return HAL_I2C_Master_Transmit(&h, dev16, buf, n, to);
		case F_RX: return HAL_I2C_Master_Receive(&h, dev16, buf, n, to);
		case F_MW: return HAL_I2C_Mem_Write(&h, dev16, mem, (uint16_t)memSize, buf, n, to);
		default: return HAL_I2C_Mem_Read(&h, dev16, mem, (uint16_t)memSize, buf, n, to);
		}
	}
	uint32_t cr1() const { return peek(&h.Instance->CR1); }
	uint32_t sr1() const { return peek(&h.Instance->SR1); }
	uint32_t sr2() const { return peek(&h.Instance->SR2); }
	/* what the HAL must leave behind after a transfer that ended in an error (RM0090 27.3.4: the stop was sent, UM1725: the handle is usable) */
	void expectReleased() {
		CHECK(h.State == HAL_I2C_STATE_READY);
		CHECK(h.Mode == HAL_I2C_MODE_NONE);
		CHECK(h.Lock == HAL_UNLOCKED);
		CHECK((sr1() & I2C_SR1_AF) == 0u);
		CHECK((sr2() & I2C_SR2_BUSY) == 0u);
		CHECK((sr2() & I2C_SR2_MSL) == 0u);
	}
};

const unsigned kSizes[] = {1, 2, 3, 4, 5, 7, 255, 256};
} // namespace

/* ======================================================================== IP1 transmit */
TEST_CASE_FIXTURE(Rig, "IP1 Master_Transmit of 1 to 256 bytes puts S, address|W, every byte acked, and P on the bus") {
	up();
	for (unsigned n : kSizes) {
		INFO("size " << n);
		uint8_t d[256];
		for (unsigned k = 0; k < 256; ++k) d[k] = pat(k);
		hostI2cLogClear(0);
		REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, (uint16_t)n, 100) == HAL_OK);
		expectLog(0, txLog(0xA0, d, n));
		CHECK(h.State == HAL_I2C_STATE_READY);
	}
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(Rig, "IP1 Master_Transmit on a slow bus (2 ticks per byte) sends the same bytes") {
	up();
	hostI2cSetByteTicks(2);
	for (unsigned n : {1u, 2u, 3u, 5u, 9u}) {
		INFO("size " << n);
		uint8_t d[16];
		for (unsigned k = 0; k < 16; ++k) d[k] = pat(k);
		hostI2cLogClear(0);
		REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, (uint16_t)n, 1000) == HAL_OK);
		expectLog(0, txLog(0xA0, d, n));
	}
}

TEST_CASE_FIXTURE(Rig, "IP1 Master_Transmit works on I2C1, I2C2 and I2C3 alike") {
	I2C_TypeDef *const inst[3] = {I2C1, I2C2, I2C3};
	for (unsigned i = 0; i < 3; ++i) {
		INFO("instance " << i);
		h = I2C_HandleTypeDef{};
		up(inst[i]);
		REQUIRE(idx == i);
		uint8_t d[3] = {0x11, 0x22, 0x33};
		REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, 3, 100) == HAL_OK);
		expectLog(i, txLog(0xA0, d, 3));
		CHECK(h.State == HAL_I2C_STATE_READY);
	}
}

TEST_CASE_FIXTURE(Rig, "IP1 characterization: Master_Transmit of 0 bytes addresses the device and stops") {
	up();
	uint8_t d[1] = {0x55};
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, 0, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA0), evP()});
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(Rig, "IP1 Master_Transmit is in BUSY_TX state, master mode and locked while the address is on the bus") {
	up();
	hostI2cSetByteTicks(5);
	Snap s{&h, HAL_I2C_STATE_RESET, HAL_I2C_MODE_NONE, HAL_UNLOCKED, false};
	const I2cScriptAction a[1] = {{uwTick + 2u, snapFn, &s}};
	i2cScriptStart(a, 1);
	uint8_t d[2] = {1, 2};
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, 2, 1000) == HAL_OK);
	REQUIRE(s.taken);
	CHECK(s.state == HAL_I2C_STATE_BUSY_TX);
	CHECK(s.mode == HAL_I2C_MODE_MASTER);
	CHECK(s.lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(Rig, "IP1 Master_Transmit leaves READY, mode NONE, ErrorCode NONE, unlocked and the STOP bit consumed") {
	up();
	h.ErrorCode = 0x7Fu; /* an old error must be cleared at the start of the call */
	uint8_t d[2] = {1, 2};
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, 2, 100) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr1() & I2C_CR1_STOP) == 0u);
	CHECK((sr2() & I2C_SR2_BUSY) == 0u);
	CHECK(HAL_I2C_GetError(&h) == HAL_I2C_ERROR_NONE);
}

TEST_CASE_FIXTURE(Rig, "IP1 Master_Transmit ignores bit 0 of DevAddress (address byte is always |W)") {
	up();
	uint8_t d[1] = {9};
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA1, d, 1, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA0), evO(9), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP1 Master_Transmit enables the peripheral when PE is 0") {
	up();
	h.Instance->CR1 &= ~I2C_CR1_PE;
	REQUIRE((cr1() & I2C_CR1_PE) == 0u);
	uint8_t d[1] = {7};
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, 1, 100) == HAL_OK);
	CHECK((cr1() & I2C_CR1_PE) != 0u);
	expectLog(0, Log{evS(), evO(0xA0), evO(7), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP1 Master_Transmit clears POS before it starts") {
	up();
	h.Instance->CR1 |= I2C_CR1_POS;
	uint8_t d[1] = {7};
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, 1, 100) == HAL_OK);
	CHECK((cr1() & I2C_CR1_POS) == 0u);
}

TEST_CASE_FIXTURE(Rig, "IP1 Master_Transmit with HAL_MAX_DELAY completes on a working bus") {
	up();
	uint8_t d[3] = {1, 2, 3};
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, 3, HAL_MAX_DELAY) == HAL_OK);
	expectLog(0, txLog(0xA0, d, 3));
}

TEST_CASE_FIXTURE(Rig, "IP1 Master_Transmit with Timeout 0 completes when the bus answers at once") {
	up();
	uint8_t d[3] = {1, 2, 3};
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, 3, 0) == HAL_OK);
	expectLog(0, txLog(0xA0, d, 3));
}

TEST_CASE_FIXTURE(Rig, "IP1 every blocking function returns HAL_BUSY without touching the bus when the state is not READY") {
	up();
	const HAL_I2C_StateTypeDef states[] = {HAL_I2C_STATE_BUSY, HAL_I2C_STATE_BUSY_TX, HAL_I2C_STATE_BUSY_RX, HAL_I2C_STATE_LISTEN, HAL_I2C_STATE_BUSY_TX_LISTEN,
	                                       HAL_I2C_STATE_BUSY_RX_LISTEN, HAL_I2C_STATE_ABORT, HAL_I2C_STATE_TIMEOUT, HAL_I2C_STATE_ERROR};
	for (HAL_I2C_StateTypeDef st : states) {
		for (Fn f : kAllFns) {
			INFO("state " << (unsigned)st << " " << fnName(f));
			h.State = st;
			uint8_t d[2] = {1, 2};
			CHECK(run(f, d, 2, 100) == HAL_BUSY);
			CHECK(h.State == st);
			CHECK(h.Lock == HAL_UNLOCKED);
			CHECK(hostI2cLogCount(0) == 0u);
		}
	}
	h.State = HAL_I2C_STATE_READY;
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(Rig, "IP1 IsDeviceReady returns HAL_BUSY without touching the bus when the state is not READY") {
	up();
	h.State = HAL_I2C_STATE_BUSY_TX;
	CHECK(HAL_I2C_IsDeviceReady(&h, 0xA0, 1, 100) == HAL_BUSY);
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(hostI2cLogCount(0) == 0u);
	h.State = HAL_I2C_STATE_READY;
}

TEST_CASE_FIXTURE(Rig, "IP1 characterization: State RESET and an already locked handle give HAL_BUSY from every blocking function") {
	up();
	for (Fn f : kAllFns) {
		INFO(fnName(f));
		uint8_t d[2] = {1, 2};
		h.State = HAL_I2C_STATE_RESET;
		CHECK(run(f, d, 2, 100) == HAL_BUSY);
		h.State = HAL_I2C_STATE_READY;
		h.Lock = HAL_LOCKED;
		CHECK(run(f, d, 2, 100) == HAL_BUSY);
		CHECK(h.Lock == HAL_LOCKED);
		h.Lock = HAL_UNLOCKED;
		CHECK(hostI2cLogCount(0) == 0u);
	}
	CHECK(HAL_I2C_IsDeviceReady(&h, 0xA0, 1, 100) == HAL_OK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_I2C_IsDeviceReady(&h, 0xA0, 1, 100) == HAL_BUSY);
	h.Lock = HAL_UNLOCKED;
}

/* ======================================================================== IP2 receive */
TEST_CASE_FIXTURE(Rig, "IP2 Master_Receive of 1 to 256 bytes delivers the bytes in order with a NACK on the last byte only") {
	for (unsigned n : kSizes) {
		INFO("size " << n);
		up();
		queue(256);
		uint8_t want[256], got[257];
		for (unsigned k = 0; k < 256; ++k) want[k] = pat(k);
		std::memset(got, 0xEE, sizeof got);
		REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, (uint16_t)n, 100) == HAL_OK);
		CHECK(std::memcmp(got, want, n) == 0);
		CHECK(got[n] == 0xEE); /* nothing written past the size */
		expectLog(0, rxLog(0xA1, want, n));
		CHECK(h.State == HAL_I2C_STATE_READY);
	}
}

TEST_CASE_FIXTURE(Rig, "IP2 Master_Receive on a slow bus (2 ticks per byte) delivers the same bytes") {
	for (unsigned n : {1u, 2u, 3u, 4u, 5u, 9u}) {
		INFO("size " << n);
		up();
		hostI2cSetByteTicks(2);
		queue(16);
		uint8_t want[16], got[16] = {};
		for (unsigned k = 0; k < 16; ++k) want[k] = pat(k);
		REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, (uint16_t)n, 1000) == HAL_OK);
		CHECK(std::memcmp(got, want, n) == 0);
		expectLog(0, rxLog(0xA1, want, n));
	}
}

TEST_CASE_FIXTURE(Rig, "IP2 Master_Receive works on I2C1, I2C2 and I2C3 alike") {
	I2C_TypeDef *const inst[3] = {I2C1, I2C2, I2C3};
	for (unsigned i = 0; i < 3; ++i) {
		INFO("instance " << i);
		h = I2C_HandleTypeDef{};
		up(inst[i]);
		queue(3);
		uint8_t want[3] = {pat(0), pat(1), pat(2)}, got[3] = {};
		REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 3, 100) == HAL_OK);
		CHECK(std::memcmp(got, want, 3) == 0);
		expectLog(i, rxLog(0xA1, want, 3));
	}
}

TEST_CASE_FIXTURE(Rig, "IP2 characterization: Master_Receive of 0 bytes addresses the device, stops and writes nothing") {
	up();
	uint8_t got[2] = {0xEE, 0xEE};
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 0, 100) == HAL_OK);
	CHECK(got[0] == 0xEE);
	CHECK(got[1] == 0xEE);
	REQUIRE(hostI2cLogCount(0) >= 3u);
	CHECK(hostI2cLogAt(0, 0).type == HOST_I2C_EV_START);
	CHECK(hostI2cLogAt(0, 1).value == 0xA1);
	CHECK(hostI2cLogAt(0, hostI2cLogCount(0) - 1).type == HOST_I2C_EV_STOP);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(Rig, "IP2 Master_Receive is in BUSY_RX state, master mode and locked while the address is on the bus") {
	up();
	queue(2);
	hostI2cSetByteTicks(5);
	Snap s{&h, HAL_I2C_STATE_RESET, HAL_I2C_MODE_NONE, HAL_UNLOCKED, false};
	const I2cScriptAction a[1] = {{uwTick + 2u, snapFn, &s}};
	i2cScriptStart(a, 1);
	uint8_t got[2];
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 2, 1000) == HAL_OK);
	REQUIRE(s.taken);
	CHECK(s.state == HAL_I2C_STATE_BUSY_RX);
	CHECK(s.mode == HAL_I2C_MODE_MASTER);
	CHECK(s.lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(Rig, "IP2 Master_Receive leaves READY, mode NONE, ErrorCode NONE, unlocked, ACK cleared and the STOP bit consumed") {
	for (unsigned n : {1u, 2u, 3u, 4u}) {
		INFO("size " << n);
		up();
		queue(8);
		h.ErrorCode = 0x7Fu;
		uint8_t got[8];
		REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, (uint16_t)n, 100) == HAL_OK);
		CHECK(h.State == HAL_I2C_STATE_READY);
		CHECK(h.Mode == HAL_I2C_MODE_NONE);
		CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
		CHECK(h.Lock == HAL_UNLOCKED);
		CHECK((cr1() & I2C_CR1_ACK) == 0u); /* the last byte was NACKed: ACK is off when the call ends */
		CHECK((cr1() & I2C_CR1_STOP) == 0u);
		CHECK((sr2() & I2C_SR2_BUSY) == 0u);
	}
}

TEST_CASE_FIXTURE(Rig, "IP2 Master_Receive sets bit 0 of the address byte whatever DevAddress bit 0 is") {
	up();
	queue(1);
	uint8_t got[1];
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 1, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA1), evI(pat(0), 0), evP()});
	hostI2cLogClear(0);
	queue(1);
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA1, got, 1, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA1), evI(pat(0), 0), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP2 receives of 1, 2, 3, 4, 5 and 1 bytes in a row each re-enable ACK, deliver the next queue bytes and end with exactly one NACK") {
	up();
	queue(16);
	const unsigned seq[] = {1, 2, 3, 4, 5, 1};
	unsigned off = 0;
	for (unsigned n : seq) {
		INFO("size " << n << " offset " << off);
		uint8_t got[8];
		hostI2cLogClear(0);
		REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, (uint16_t)n, 100) == HAL_OK);
		for (unsigned k = 0; k < n; ++k) CHECK(got[k] == pat(off + k));
		unsigned nacks = 0;
		for (unsigned k = 0; k < hostI2cLogCount(0); ++k) {
			const Ev e = hostI2cLogAt(0, k);
			if (e.type == HOST_I2C_EV_BYTE_IN && e.ack == 0) ++nacks;
		}
		CHECK(nacks == 1u);
		off += n;
	}
}

TEST_CASE_FIXTURE(Rig, "IP2 a receive of 2 bytes sets POS, a following receive of 3 bytes still ACKs bytes 1 and 2 (POS cleared at the start)") {
	up();
	queue(2);
	uint8_t got[3];
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 2, 100) == HAL_OK);
	/* the hand derived log of the second call: 3 bytes, NACK on the third only */
	hostI2cLogClear(0);
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 3, 100) == HAL_OK);
	REQUIRE(hostI2cLogCount(0) == 6u);
	CHECK(hostI2cLogAt(0, 2).ack == 1);
	CHECK(hostI2cLogAt(0, 3).ack == 1);
	CHECK(hostI2cLogAt(0, 4).ack == 0);
	CHECK((cr1() & I2C_CR1_POS) == 0u);
}

TEST_CASE_FIXTURE(Rig, "IP2 Master_Receive with HAL_MAX_DELAY and with Timeout 0 completes when the bus answers at once") {
	up();
	queue(4);
	uint8_t want[4] = {pat(0), pat(1), pat(2), pat(3)}, got[4];
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 4, HAL_MAX_DELAY) == HAL_OK);
	CHECK(std::memcmp(got, want, 4) == 0);
	up();
	queue(4);
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 4, 0) == HAL_OK);
	CHECK(std::memcmp(got, want, 4) == 0);
}

TEST_CASE_FIXTURE(Rig, "IP2 Master_Receive enables the peripheral when PE is 0") {
	up();
	queue(1);
	h.Instance->CR1 &= ~I2C_CR1_PE;
	uint8_t got[1];
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 1, 100) == HAL_OK);
	CHECK((cr1() & I2C_CR1_PE) != 0u);
	CHECK(got[0] == pat(0));
}

/* ======================================================================== IP7 register use (ordering is observed through the bus log) */
TEST_CASE_FIXTURE(Rig, "IP7 a 1 byte receive clears ACK before the ADDR flag is cleared (the byte on the bus is NACKed) and sets STOP") {
	/* RM0090 27.3.3 (Figure 244, N = 1): ACK must be cleared before clearing ADDR, otherwise the single byte would be acknowledged */
	up();
	queue(1);
	uint8_t got[1];
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 1, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA1), evI(pat(0), 0), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP7 a 2 byte receive sets POS and clears ACK before ADDR is cleared (byte 1 ACK, byte 2 NACK)") {
	/* RM0090 27.3.3 (N = 2): ACK = 0 and POS = 1 are set before ADDR is cleared; the NACK then goes out on the second byte */
	up();
	queue(2);
	uint8_t got[2];
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 2, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA1), evI(pat(0), 1), evI(pat(1), 0), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP7 a 2 byte Mem_Read sets POS and clears ACK before ADDR is cleared too") {
	up();
	queue(2);
	uint8_t got[2];
	REQUIRE(HAL_I2C_Mem_Read(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, got, 2, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA0), evO(0x10), evSr(), evO(0xA1), evI(pat(0), 1), evI(pat(1), 0), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP7 a 3 byte receive NACKs byte 3 only (ACK cleared after BTF of byte 1, STOP after BTF of byte 2)") {
	up();
	queue(3);
	uint8_t got[3];
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 3, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA1), evI(pat(0), 1), evI(pat(1), 1), evI(pat(2), 0), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP7 an N byte receive (N = 6) switches ACK off two bytes before the end and sends STOP after the NACK") {
	up();
	hostI2cSetByteTicks(2); /* slow bus: BTF is not set when the byte is read, the ACK is cleared in the 3 last bytes branch */
	queue(6);
	uint8_t got[6];
	REQUIRE(HAL_I2C_Master_Receive(&h, 0xA0, got, 6, 1000) == HAL_OK);
	Log want{evS(), evO(0xA1)};
	for (unsigned k = 0; k < 6; ++k) want.push_back(evI(pat(k), (uint8_t)(k < 5)));
	want.push_back(evP());
	expectLog(0, want);
}

TEST_CASE_FIXTURE(Rig, "IP7 Mem_Read enables ACK before it starts the transfer (N = 4 bytes: ACK, ACK, ACK, NACK)") {
	up();
	queue(4);
	h.Instance->CR1 &= ~I2C_CR1_ACK;
	uint8_t got[4];
	REQUIRE(HAL_I2C_Mem_Read(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, got, 4, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA0), evO(0x10), evSr(), evO(0xA1), evI(pat(0)), evI(pat(1)), evI(pat(2)), evI(pat(3), 0), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP7 Master_Transmit generates the STOP only after the last byte went out (BTF), never in the middle") {
	up();
	hostI2cSetByteTicks(2);
	uint8_t d[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, 4, 1000) == HAL_OK);
	const unsigned n = hostI2cLogCount(0);
	REQUIRE(n == 7u);
	for (unsigned k = 0; k + 1 < n; ++k) CHECK(hostI2cLogAt(0, k).type != HOST_I2C_EV_STOP);
	CHECK(hostI2cLogAt(0, n - 1).type == HOST_I2C_EV_STOP);
	CHECK(hostI2cLogAt(0, n - 2).value == 4);
}

/* ======================================================================== IP3 errors */
TEST_CASE_FIXTURE(Rig, "IP3 an address nack (no device) ends every blocking function with HAL_ERROR, ErrorCode AF, STOP sent and the handle released") {
	for (Fn f : kAllFns) {
		INFO(fnName(f));
		up();
		uint8_t d[2] = {1, 2};
		CHECK(run(f, d, 2, 100, 0xA4) == HAL_ERROR); /* nobody at 0x52 */
		CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
		CHECK(HAL_I2C_GetError(&h) == HAL_I2C_ERROR_AF);
		const uint8_t addr = (f == F_RX) ? 0xA5 : 0xA4; /* Mem_Read also addresses with W first */
		expectLog(0, Log{evS(), evO(addr, 0), evP()});
		expectReleased();
		h = I2C_HandleTypeDef{};
		hostI2cResetState();
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 a device that nacks its address is handled like a missing device") {
	up();
	hostI2cDeviceNackAddress(0, dev, 1);
	uint8_t d[2] = {1, 2};
	CHECK(HAL_I2C_Master_Transmit(&h, 0xA0, d, 2, 100) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	expectLog(0, Log{evS(), evO(0xA0, 0), evP()});
	expectReleased();
}

TEST_CASE_FIXTURE(Rig, "IP3 after an address nack the next transfer works and its ErrorCode is NONE") {
	up();
	uint8_t d[1] = {5};
	CHECK(HAL_I2C_Master_Transmit(&h, 0xA4, d, 1, 100) == HAL_ERROR);
	hostI2cLogClear(0);
	REQUIRE(HAL_I2C_Master_Transmit(&h, 0xA0, d, 1, 100) == HAL_OK);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	expectLog(0, Log{evS(), evO(0xA0), evO(5), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP3 Master_Receive address nack: HAL_ERROR, ErrorCode AF, STOP, handle released") {
	up();
	hostI2cDeviceNackAddress(0, dev, 1);
	uint8_t got[3] = {0xEE, 0xEE, 0xEE};
	CHECK(HAL_I2C_Master_Receive(&h, 0xA0, got, 3, 100) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	expectLog(0, Log{evS(), evO(0xA1, 0), evP()});
	CHECK(got[0] == 0xEE);
	expectReleased();
}

TEST_CASE_FIXTURE(Rig, "IP3 a data nack in Master_Transmit stops the transfer: HAL_ERROR, ErrorCode AF, STOP, nothing sent after the nacked byte") {
	for (int at : {0, 1, 2, 3}) {
		INFO("nack at data index " << at);
		up();
		hostI2cDeviceNackDataAt(0, dev, at);
		uint8_t d[4] = {0x11, 0x22, 0x33, 0x44};
		CHECK(HAL_I2C_Master_Transmit(&h, 0xA0, d, 4, 100) == HAL_ERROR);
		CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
		Log want{evS(), evO(0xA0)};
		for (int k = 0; k <= at; ++k) want.push_back(evO(d[k], (uint8_t)(k != at)));
		want.push_back(evP());
		expectLog(0, want);
		expectReleased();
		h = I2C_HandleTypeDef{};
		hostI2cResetState();
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 a data nack of the only byte on a slow bus ends in AF as well") {
	up();
	hostI2cSetByteTicks(2);
	hostI2cDeviceNackDataAt(0, dev, 0);
	uint8_t d[1] = {0x11};
	CHECK(HAL_I2C_Master_Transmit(&h, 0xA0, d, 1, 1000) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	expectLog(0, Log{evS(), evO(0xA0), evO(0x11, 0), evP()});
	expectReleased();
}

TEST_CASE_FIXTURE(Rig, "IP3 Mem_Write: a nack of the data byte ends with HAL_ERROR, AF and STOP") {
	for (uint32_t msz : {(uint32_t)I2C_MEMADD_SIZE_8BIT, (uint32_t)I2C_MEMADD_SIZE_16BIT}) {
		INFO("mem size " << msz);
		up();
		const int memBytes = msz == I2C_MEMADD_SIZE_8BIT ? 1 : 2;
		hostI2cDeviceNackDataAt(0, dev, memBytes); /* first data byte after the memory address */
		uint8_t d[3] = {0x11, 0x22, 0x33};
		CHECK(HAL_I2C_Mem_Write(&h, 0xA0, 0x1234, (uint16_t)msz, d, 3, 100) == HAL_ERROR);
		CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
		REQUIRE(hostI2cLogCount(0) >= 4u);
		CHECK(hostI2cLogAt(0, hostI2cLogCount(0) - 1).type == HOST_I2C_EV_STOP);
		CHECK(hostI2cLogAt(0, hostI2cLogCount(0) - 2).ack == 0);
		expectReleased();
		h = I2C_HandleTypeDef{};
		hostI2cResetState();
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 Mem_Write: a nack of the memory address byte ends with HAL_ERROR, AF and STOP") {
	for (uint32_t msz : {(uint32_t)I2C_MEMADD_SIZE_8BIT, (uint32_t)I2C_MEMADD_SIZE_16BIT}) {
		for (unsigned ticks : {0u, 2u}) {
			INFO("mem size " << msz << " byteTicks " << ticks);
			up();
			hostI2cSetByteTicks(ticks);
			hostI2cDeviceNackDataAt(0, dev, 0);
			uint8_t d[2] = {0x11, 0x22};
			CHECK(HAL_I2C_Mem_Write(&h, 0xA0, 0x1234, (uint16_t)msz, d, 2, 1000) == HAL_ERROR);
			CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
			REQUIRE(hostI2cLogCount(0) >= 4u);
			CHECK(hostI2cLogAt(0, 2).ack == 0);
			CHECK(hostI2cLogAt(0, hostI2cLogCount(0) - 1).type == HOST_I2C_EV_STOP);
			expectReleased();
			h = I2C_HandleTypeDef{};
			hostI2cResetState();
		}
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: Mem_Read with a nacked 8 bit memory address byte goes on with the repeated start and returns HAL_OK") {
	up();
	queue(2);
	hostI2cDeviceNackDataAt(0, dev, 0);
	uint8_t got[2] = {};
	CHECK(HAL_I2C_Mem_Read(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, got, 2, 100) == HAL_OK);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.State == HAL_I2C_STATE_READY);
	REQUIRE(hostI2cLogCount(0) >= 4u);
	CHECK(hostI2cLogAt(0, 2).ack == 0);
	CHECK(hostI2cLogAt(0, 3).type == HOST_I2C_EV_RESTART);
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: the BUSY flag wait gives up after 25 ms with HAL_BUSY and leaves the handle READY and unlocked") {
	for (Fn f : kAllFns) {
		INFO(fnName(f));
		up();
		hostI2cBusBusy(0, 1);
		const uint32_t t0 = uwTick;
		uint8_t d[2] = {1, 2};
		CHECK(run(f, d, 2, 100) == HAL_BUSY);
		const uint32_t dt = uwTick - t0;
		CHECK(dt > 25u);  /* the HAL waits while (tick - start) <= 25 */
		CHECK(dt < 40u);
		CHECK(h.State == HAL_I2C_STATE_READY);
		CHECK(h.Mode == HAL_I2C_MODE_NONE);
		CHECK(h.Lock == HAL_UNLOCKED);
		CHECK(hostI2cLogCount(0) == 0u); /* no START was generated */
		h = I2C_HandleTypeDef{};
		hostI2cResetState();
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: the BUSY timeout leaves ErrorCode TIMEOUT") {
	up();
	hostI2cBusBusy(0, 1);
	uint8_t d[1] = {1};
	CHECK(HAL_I2C_Master_Transmit(&h, 0xA0, d, 1, 100) == HAL_BUSY);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: the BUSY wait is 25 ms whatever the Timeout argument is (0 and HAL_MAX_DELAY)") {
	for (uint32_t to : {0u, 1u, (uint32_t)HAL_MAX_DELAY}) {
		INFO("timeout " << to);
		up();
		hostI2cBusBusy(0, 1);
		const uint32_t t0 = uwTick;
		uint8_t d[1] = {1};
		CHECK(HAL_I2C_Master_Transmit(&h, 0xA0, d, 1, to) == HAL_BUSY);
		CHECK(uwTick - t0 > 25u);
		CHECK(uwTick - t0 < 40u);
		h = I2C_HandleTypeDef{};
		hostI2cResetState();
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 a transfer starts once the other controller releases the bus before the 25 ms are over") {
	up();
	hostI2cBusBusy(0, 1);
	struct R {
		static void release(void *) { hostI2cBusBusy(0, 0); }
	};
	const I2cScriptAction a[1] = {{uwTick + 10u, R::release, nullptr}};
	i2cScriptStart(a, 1);
	uint8_t d[1] = {1};
	CHECK(HAL_I2C_Master_Transmit(&h, 0xA0, d, 1, 100) == HAL_OK);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: START that never completes (stuck bus) ends with HAL_ERROR and ErrorCode WRONG_START after Timeout ms") {
	for (Fn f : kAllFns) {
		for (uint32_t to : {0u, 1u, 7u}) {
			INFO(fnName(f) << " timeout " << to);
			up();
			hostI2cStuck(0, 1);
			const uint32_t t0 = uwTick;
			uint8_t d[2] = {1, 2};
			CHECK(run(f, d, 2, to) == HAL_ERROR);
			const uint32_t dt = uwTick - t0;
			CHECK(dt >= to); /* Tickstart is the first HAL_GetTick of the call: the wait lasts more than `to` ticks (0: at the first check) */
			CHECK(dt <= to + 4u);
			CHECK(h.ErrorCode == HAL_I2C_WRONG_START);
			CHECK(h.State == HAL_I2C_STATE_READY);
			CHECK(h.Lock == HAL_UNLOCKED);
			h = I2C_HandleTypeDef{};
			hostI2cResetState();
		}
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 a stuck bus makes IsDeviceReady return HAL_TIMEOUT, state READY, unlocked") {
	up();
	hostI2cStuck(0, 1);
	const uint32_t t0 = uwTick;
	CHECK(HAL_I2C_IsDeviceReady(&h, 0xA0, 3, 5) == HAL_TIMEOUT);
	CHECK(uwTick - t0 >= 5u);
	CHECK(uwTick - t0 <= 10u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(h.ErrorCode == HAL_I2C_WRONG_START);
	CHECK(hostI2cLogCount(0) == 0u);
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: a timeout in the address phase (address takes longer than Timeout) ends with HAL_ERROR, ErrorCode TIMEOUT, READY") {
	for (Fn f : kAllFns) {
		INFO(fnName(f));
		up();
		hostI2cSetByteTicks(30);
		uint8_t d[2] = {1, 2};
		CHECK(run(f, d, 2, 10) == HAL_ERROR);
		CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
		CHECK(h.State == HAL_I2C_STATE_READY);
		CHECK(h.Mode == HAL_I2C_MODE_NONE);
		CHECK(h.Lock == HAL_UNLOCKED);
		h = I2C_HandleTypeDef{};
		hostI2cResetState();
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: a timeout in the data phase of Master_Transmit (byte slower than Timeout) ends with HAL_ERROR, TIMEOUT, READY") {
	up();
	hostI2cSetByteTicks(30);
	uint8_t d[2] = {1, 2};
	CHECK(HAL_I2C_Master_Transmit(&h, 0xA0, d, 2, 40) == HAL_ERROR); /* address 30 ticks, first byte ends at 60 > 40 */
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: a timeout waiting for the single byte of Master_Receive ends with HAL_ERROR, TIMEOUT, READY") {
	up();
	queue(1);
	hostI2cSetByteTicks(30);
	uint8_t got[1];
	CHECK(HAL_I2C_Master_Receive(&h, 0xA0, got, 1, 40) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: a timeout waiting for BTF in the 2, 3 and N byte branches of Master_Receive ends with HAL_ERROR, TIMEOUT, READY") {
	for (unsigned n : {2u, 3u, 4u, 6u}) {
		for (uint32_t to : {40u, 70u, 100u, 140u}) {
			INFO("size " << n << " timeout " << to);
			up();
			queue(8);
			hostI2cSetByteTicks(30);
			uint8_t got[8];
			const HAL_StatusTypeDef r = HAL_I2C_Master_Receive(&h, 0xA0, got, (uint16_t)n, to);
			if (r == HAL_ERROR) {
				CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
				CHECK(h.State == HAL_I2C_STATE_READY);
				CHECK(h.Lock == HAL_UNLOCKED);
			} else {
				CHECK(r == HAL_OK);
			}
			h = I2C_HandleTypeDef{};
			hostI2cResetState();
		}
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: a timeout while the last byte of Mem_Write is on the bus ends with HAL_ERROR, TIMEOUT, READY") {
	for (uint32_t msz : {(uint32_t)I2C_MEMADD_SIZE_8BIT, (uint32_t)I2C_MEMADD_SIZE_16BIT}) {
		/* on the bus: address, 1 or 2 memory address bytes, 2 data bytes = 4 or 5 bytes of 30 ticks = 120 / 150 ticks: every Timeout below that runs out */
		for (uint32_t to : {40u, 70u, 100u}) {
			INFO("mem size " << msz << " timeout " << to);
			up();
			hostI2cSetByteTicks(30);
			uint8_t d[2] = {1, 2};
			const HAL_StatusTypeDef r = HAL_I2C_Mem_Write(&h, 0xA0, 0x1234, (uint16_t)msz, d, 2, to);
			CHECK(r == HAL_ERROR);
			CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
			CHECK(h.State == HAL_I2C_STATE_READY);
			CHECK(h.Lock == HAL_UNLOCKED);
			h = I2C_HandleTypeDef{};
			hostI2cResetState();
		}
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: a timeout while the memory address of Mem_Read is on the bus ends with HAL_ERROR, TIMEOUT, READY") {
	for (uint32_t msz : {(uint32_t)I2C_MEMADD_SIZE_8BIT, (uint32_t)I2C_MEMADD_SIZE_16BIT}) {
		/* on the bus before the data: address, 1 or 2 memory bytes, restart, address = 4 or 5 bytes of 30 ticks (the restart waits for the byte in flight) */
		for (uint32_t to : {40u, 70u, 100u}) {
			INFO("mem size " << msz << " timeout " << to);
			up();
			queue(4);
			hostI2cSetByteTicks(30);
			uint8_t got[4];
			const HAL_StatusTypeDef r = HAL_I2C_Mem_Read(&h, 0xA0, 0x1234, (uint16_t)msz, got, 4, to);
			CHECK(r == HAL_ERROR);
			/* TIMEOUT, or WRONG_START when the restart was requested but its START condition did not appear in time (START bit still set) */
			CHECK((h.ErrorCode == HAL_I2C_ERROR_TIMEOUT || h.ErrorCode == HAL_I2C_WRONG_START));
			CHECK(h.State == HAL_I2C_STATE_READY);
			CHECK(h.Lock == HAL_UNLOCKED);
			h = I2C_HandleTypeDef{};
			hostI2cResetState();
		}
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: arbitration lost during Master_Transmit is not looked at by the HAL: the call ends with a timeout and ARLO stays set") {
	up();
	hostI2cSetByteTicks(2);
	trigArm(0, 3, [](unsigned i) { hostI2cArbitrationLost(i); }); /* after S, address, first byte */
	uint8_t d[4] = {1, 2, 3, 4};
	const HAL_StatusTypeDef r = HAL_I2C_Master_Transmit(&h, 0xA0, d, 4, 30);
	CHECK(r == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK((sr1() & I2C_SR1_ARLO) != 0u);
	CHECK((sr2() & I2C_SR2_MSL) == 0u); /* RM0090 27.3.4: the interface went back to target mode */
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: a bus error during Master_Transmit is not looked at by the HAL: BERR stays set, the transfer completes") {
	up();
	hostI2cSetByteTicks(2);
	trigArm(0, 3, [](unsigned i) { hostI2cRaiseSr1(i, I2C_SR1_BERR); });
	uint8_t d[4] = {1, 2, 3, 4};
	const HAL_StatusTypeDef r = HAL_I2C_Master_Transmit(&h, 0xA0, d, 4, 100);
	CHECK(r == HAL_OK);
	CHECK((sr1() & I2C_SR1_BERR) != 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: arbitration lost during Master_Receive ends with a timeout and ARLO stays set") {
	up();
	queue(6);
	hostI2cSetByteTicks(2);
	trigArm(0, 3, [](unsigned i) { hostI2cArbitrationLost(i); });
	uint8_t got[6];
	const HAL_StatusTypeDef r = HAL_I2C_Master_Receive(&h, 0xA0, got, 6, 30);
	CHECK(r == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK((sr1() & I2C_SR1_ARLO) != 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: a STOPF seen while Master_Receive waits for RXNE ends the call with HAL_ERROR and no error code") {
	up();
	queue(1);
	hostI2cSetByteTicks(5);
	/* the model has no hook that sets STOPF in controller mode: the test sets it in the register file (raw poke) while the byte is on the bus */
	trigArm(0, 2, [](unsigned) { HalFixture::poke(&I2C1->SR1, HalFixture::peek(&I2C1->SR1) | I2C_SR1_STOPF); });
	uint8_t got[1];
	CHECK(HAL_I2C_Master_Receive(&h, 0xA0, got, 1, 1000) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

/* ======================================================================== IP4 memory access */
TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write with an 8 bit memory address sends S, A|W, the low byte of the address, the data, P") {
	for (unsigned n : {1u, 2u, 3u, 4u, 5u, 9u}) {
		INFO("size " << n);
		up();
		uint8_t d[16];
		for (unsigned k = 0; k < 16; ++k) d[k] = pat(k);
		REQUIRE(HAL_I2C_Mem_Write(&h, 0xA0, 0x1234, I2C_MEMADD_SIZE_8BIT, d, (uint16_t)n, 100) == HAL_OK);
		Log want{evS(), evO(0xA0), evO(0x34)};
		for (unsigned k = 0; k < n; ++k) want.push_back(evO(d[k]));
		want.push_back(evP());
		expectLog(0, want);
		CHECK(h.State == HAL_I2C_STATE_READY);
		CHECK(h.Mode == HAL_I2C_MODE_NONE);
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write with a 16 bit memory address sends the high byte first") {
	for (unsigned n : {1u, 2u, 3u, 5u}) {
		INFO("size " << n);
		up();
		uint8_t d[8];
		for (unsigned k = 0; k < 8; ++k) d[k] = pat(k);
		REQUIRE(HAL_I2C_Mem_Write(&h, 0xA0, 0x1234, I2C_MEMADD_SIZE_16BIT, d, (uint16_t)n, 100) == HAL_OK);
		Log want{evS(), evO(0xA0), evO(0x12), evO(0x34)};
		for (unsigned k = 0; k < n; ++k) want.push_back(evO(d[k]));
		want.push_back(evP());
		expectLog(0, want);
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write on a slow bus (2 ticks per byte) sends the same bytes (8 and 16 bit address)") {
	for (uint32_t msz : {(uint32_t)I2C_MEMADD_SIZE_8BIT, (uint32_t)I2C_MEMADD_SIZE_16BIT}) {
		for (unsigned n : {1u, 2u, 3u, 4u}) {
			INFO("mem size " << msz << " size " << n);
			up();
			hostI2cSetByteTicks(2);
			uint8_t d[4] = {0x41, 0x42, 0x43, 0x44};
			REQUIRE(HAL_I2C_Mem_Write(&h, 0xA0, 0xABCD, (uint16_t)msz, d, (uint16_t)n, 1000) == HAL_OK);
			Log want{evS(), evO(0xA0)};
			if (msz == I2C_MEMADD_SIZE_16BIT) want.push_back(evO(0xAB));
			want.push_back(evO(0xCD));
			for (unsigned k = 0; k < n; ++k) want.push_back(evO(d[k]));
			want.push_back(evP());
			expectLog(0, want);
		}
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write works on I2C1, I2C2 and I2C3 alike") {
	I2C_TypeDef *const inst[3] = {I2C1, I2C2, I2C3};
	for (unsigned i = 0; i < 3; ++i) {
		INFO("instance " << i);
		h = I2C_HandleTypeDef{};
		up(inst[i]);
		uint8_t d[2] = {0x21, 0x22};
		REQUIRE(HAL_I2C_Mem_Write(&h, 0xA0, 0x0102, I2C_MEMADD_SIZE_16BIT, d, 2, 100) == HAL_OK);
		expectLog(i, Log{evS(), evO(0xA0), evO(0x01), evO(0x02), evO(0x21), evO(0x22), evP()});
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 characterization: Mem_Write of 0 bytes sends the address phase and the memory address only") {
	up();
	uint8_t d[1] = {1};
	const HAL_StatusTypeDef r = HAL_I2C_Mem_Write(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, d, 0, 100);
	CHECK(r == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA0), evO(0x10), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write is in BUSY_TX state and MEM mode, locked, while the address is on the bus") {
	up();
	hostI2cSetByteTicks(5);
	Snap s{&h, HAL_I2C_STATE_RESET, HAL_I2C_MODE_NONE, HAL_UNLOCKED, false};
	const I2cScriptAction a[1] = {{uwTick + 2u, snapFn, &s}};
	i2cScriptStart(a, 1);
	uint8_t d[1] = {1};
	REQUIRE(HAL_I2C_Mem_Write(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, d, 1, 1000) == HAL_OK);
	REQUIRE(s.taken);
	CHECK(s.state == HAL_I2C_STATE_BUSY_TX);
	CHECK(s.mode == HAL_I2C_MODE_MEM);
	CHECK(s.lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Read is in BUSY_RX state and MEM mode, locked, while the address is on the bus") {
	up();
	queue(1);
	hostI2cSetByteTicks(5);
	Snap s{&h, HAL_I2C_STATE_RESET, HAL_I2C_MODE_NONE, HAL_UNLOCKED, false};
	const I2cScriptAction a[1] = {{uwTick + 2u, snapFn, &s}};
	i2cScriptStart(a, 1);
	uint8_t got[1];
	REQUIRE(HAL_I2C_Mem_Read(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, got, 1, 1000) == HAL_OK);
	REQUIRE(s.taken);
	CHECK(s.state == HAL_I2C_STATE_BUSY_RX);
	CHECK(s.mode == HAL_I2C_MODE_MEM);
	CHECK(s.lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write and Mem_Read leave READY, mode NONE, ErrorCode NONE, unlocked") {
	up();
	queue(2);
	h.ErrorCode = 0x7F;
	uint8_t d[2] = {1, 2};
	REQUIRE(HAL_I2C_Mem_Write(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, d, 2, 100) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	h.ErrorCode = 0x7F;
	REQUIRE(HAL_I2C_Mem_Read(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, d, 2, 100) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Read sends S, A|W, the memory address, a repeated start, A|R and receives the bytes with a NACK on the last (8 bit address)") {
	for (unsigned n : {1u, 2u, 3u, 4u, 5u, 9u}) {
		INFO("size " << n);
		up();
		queue(16);
		uint8_t want[16], got[16] = {};
		for (unsigned k = 0; k < 16; ++k) want[k] = pat(k);
		REQUIRE(HAL_I2C_Mem_Read(&h, 0xA0, 0x1234, I2C_MEMADD_SIZE_8BIT, got, (uint16_t)n, 100) == HAL_OK);
		CHECK(std::memcmp(got, want, n) == 0);
		Log l{evS(), evO(0xA0), evO(0x34), evSr(), evO(0xA1)};
		for (unsigned k = 0; k < n; ++k) l.push_back(evI(want[k], (uint8_t)(k + 1 < n)));
		l.push_back(evP());
		expectLog(0, l);
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Read with a 16 bit memory address sends the high byte first") {
	for (unsigned n : {1u, 2u, 3u, 4u}) {
		INFO("size " << n);
		up();
		queue(8);
		uint8_t want[8], got[8] = {};
		for (unsigned k = 0; k < 8; ++k) want[k] = pat(k);
		REQUIRE(HAL_I2C_Mem_Read(&h, 0xA0, 0x1234, I2C_MEMADD_SIZE_16BIT, got, (uint16_t)n, 100) == HAL_OK);
		CHECK(std::memcmp(got, want, n) == 0);
		Log l{evS(), evO(0xA0), evO(0x12), evO(0x34), evSr(), evO(0xA1)};
		for (unsigned k = 0; k < n; ++k) l.push_back(evI(want[k], (uint8_t)(k + 1 < n)));
		l.push_back(evP());
		expectLog(0, l);
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Read on a slow bus (2 ticks per byte) delivers the same bytes (8 and 16 bit address)") {
	for (uint32_t msz : {(uint32_t)I2C_MEMADD_SIZE_8BIT, (uint32_t)I2C_MEMADD_SIZE_16BIT}) {
		for (unsigned n : {1u, 2u, 3u, 4u, 6u}) {
			INFO("mem size " << msz << " size " << n);
			up();
			hostI2cSetByteTicks(2);
			queue(8);
			uint8_t want[8], got[8] = {};
			for (unsigned k = 0; k < 8; ++k) want[k] = pat(k);
			REQUIRE(HAL_I2C_Mem_Read(&h, 0xA0, 0xABCD, (uint16_t)msz, got, (uint16_t)n, 1000) == HAL_OK);
			CHECK(std::memcmp(got, want, n) == 0);
			Log l{evS(), evO(0xA0)};
			if (msz == I2C_MEMADD_SIZE_16BIT) l.push_back(evO(0xAB));
			l.push_back(evO(0xCD));
			l.push_back(evSr());
			l.push_back(evO(0xA1));
			for (unsigned k = 0; k < n; ++k) l.push_back(evI(want[k], (uint8_t)(k + 1 < n)));
			l.push_back(evP());
			expectLog(0, l);
		}
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Read works on I2C1, I2C2 and I2C3 alike") {
	I2C_TypeDef *const inst[3] = {I2C1, I2C2, I2C3};
	for (unsigned i = 0; i < 3; ++i) {
		INFO("instance " << i);
		h = I2C_HandleTypeDef{};
		up(inst[i]);
		queue(2);
		uint8_t got[2] = {};
		REQUIRE(HAL_I2C_Mem_Read(&h, 0xA0, 0x07, I2C_MEMADD_SIZE_8BIT, got, 2, 100) == HAL_OK);
		CHECK(got[0] == pat(0));
		CHECK(got[1] == pat(1));
		expectLog(i, Log{evS(), evO(0xA0), evO(0x07), evSr(), evO(0xA1), evI(pat(0), 1), evI(pat(1), 0), evP()});
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 characterization: Mem_Read of 0 bytes does the address phases and stops") {
	up();
	uint8_t got[1] = {0xEE};
	const HAL_StatusTypeDef r = HAL_I2C_Mem_Read(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, got, 0, 100);
	CHECK(r == HAL_OK);
	CHECK(got[0] == 0xEE);
	REQUIRE(hostI2cLogCount(0) >= 4u);
	CHECK(hostI2cLogAt(0, 3).type == HOST_I2C_EV_RESTART);
	CHECK(hostI2cLogAt(0, hostI2cLogCount(0) - 1).type == HOST_I2C_EV_STOP);
}

TEST_CASE_FIXTURE(Rig, "IP4 characterization: a MemAddSize that is neither 8 nor 16 bit is sent as 16 bit") {
	up();
	uint8_t d[1] = {0x77};
	REQUIRE(HAL_I2C_Mem_Write(&h, 0xA0, 0x1234, 0x55, d, 1, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA0), evO(0x12), evO(0x34), evO(0x77), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write and Mem_Read ignore bit 0 of DevAddress for the write phase and use |R for the read phase") {
	up();
	queue(1);
	uint8_t d[1] = {0x77};
	REQUIRE(HAL_I2C_Mem_Write(&h, 0xA1, 0x10, I2C_MEMADD_SIZE_8BIT, d, 1, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA0), evO(0x10), evO(0x77), evP()});
	hostI2cLogClear(0);
	REQUIRE(HAL_I2C_Mem_Read(&h, 0xA1, 0x10, I2C_MEMADD_SIZE_8BIT, d, 1, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA0), evO(0x10), evSr(), evO(0xA1), evI(pat(0), 0), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Read with a nack of the read address ends in AF with a STOP") {
	up();
	hostI2cSetByteTicks(2);
	trigArm(0, 4, [](unsigned i) { hostI2cDeviceNackAddress(i, 0, 1); }); /* after S, A0, mem, Sr: the read address byte is nacked */
	uint8_t got[2] = {0xEE, 0xEE};
	const HAL_StatusTypeDef r = HAL_I2C_Mem_Read(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, got, 2, 1000);
	CHECK(r == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(hostI2cLogAt(0, hostI2cLogCount(0) - 1).type == HOST_I2C_EV_STOP);
	expectReleased();
}

/* ======================================================================== IP5 IsDeviceReady */
TEST_CASE_FIXTURE(Rig, "IP5 IsDeviceReady with a present device returns HAL_OK on the first trial: S, A|W acked, P") {
	for (uint32_t trials : {1u, 3u}) {
		INFO("trials " << trials);
		up();
		REQUIRE(HAL_I2C_IsDeviceReady(&h, 0xA0, trials, 100) == HAL_OK);
		expectLog(0, Log{evS(), evO(0xA0), evP()});
		CHECK(h.State == HAL_I2C_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
		CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
		CHECK((sr2() & I2C_SR2_BUSY) == 0u);
	}
}

TEST_CASE_FIXTURE(Rig, "IP5 IsDeviceReady works on I2C1, I2C2 and I2C3 alike") {
	I2C_TypeDef *const inst[3] = {I2C1, I2C2, I2C3};
	for (unsigned i = 0; i < 3; ++i) {
		INFO("instance " << i);
		h = I2C_HandleTypeDef{};
		up(inst[i]);
		REQUIRE(HAL_I2C_IsDeviceReady(&h, 0xA0, 2, 100) == HAL_OK);
		expectLog(i, Log{evS(), evO(0xA0), evP()});
	}
}

TEST_CASE_FIXTURE(Rig, "IP5 IsDeviceReady sends the address with R/W = 0 whatever bit 0 of DevAddress is") {
	up();
	REQUIRE(HAL_I2C_IsDeviceReady(&h, 0xA1, 1, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA0), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP5 IsDeviceReady with a missing device tries Trials times, each trial S, address nacked, P, and returns HAL_ERROR") {
	for (uint32_t trials : {1u, 2u, 5u}) {
		INFO("trials " << trials);
		up();
		CHECK(HAL_I2C_IsDeviceReady(&h, 0xA4, trials, 100) == HAL_ERROR);
		Log want;
		for (uint32_t k = 0; k < trials; ++k) {
			want.push_back(evS());
			want.push_back(evO(0xA4, 0));
			want.push_back(evP());
		}
		expectLog(0, want);
		CHECK(h.State == HAL_I2C_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
		CHECK((sr1() & I2C_SR1_AF) == 0u); /* AF is cleared after every failed trial */
		CHECK((sr2() & I2C_SR2_BUSY) == 0u);
		h = I2C_HandleTypeDef{};
		hostI2cResetState();
	}
}

TEST_CASE_FIXTURE(Rig, "IP5 characterization: IsDeviceReady with Trials 0 tries once") {
	up();
	CHECK(HAL_I2C_IsDeviceReady(&h, 0xA4, 0, 100) == HAL_ERROR);
	expectLog(0, Log{evS(), evO(0xA4, 0), evP()});
	hostI2cLogClear(0);
	CHECK(HAL_I2C_IsDeviceReady(&h, 0xA0, 0, 100) == HAL_OK);
	expectLog(0, Log{evS(), evO(0xA0), evP()});
}

TEST_CASE_FIXTURE(Rig, "IP5 IsDeviceReady returns HAL_OK on a later trial when the device starts answering (nack, then ack)") {
	up();
	hostI2cSetByteTicks(2);
	hostI2cDeviceNackAddress(0, dev, 1);
	struct R {
		static void answer(void *) { hostI2cDeviceNackAddress(0, 0, 0); }
	};
	const I2cScriptAction a[1] = {{uwTick + 7u, R::answer, nullptr}};
	i2cScriptStart(a, 1);
	REQUIRE(HAL_I2C_IsDeviceReady(&h, 0xA0, 10, 1000) == HAL_OK);
	REQUIRE(i2cScriptDone() == 1u);
	const unsigned n = hostI2cLogCount(0);
	REQUIRE(n >= 6u); /* at least two trials */
	/* all trials but the last: S, nacked address, P; the last: S, acked address, P */
	REQUIRE(n % 3u == 0u);
	for (unsigned t = 0; t + 1 < n / 3u; ++t) {
		CHECK(hostI2cLogAt(0, 3 * t).type == HOST_I2C_EV_START);
		CHECK(hostI2cLogAt(0, 3 * t + 1).ack == 0);
		CHECK(hostI2cLogAt(0, 3 * t + 2).type == HOST_I2C_EV_STOP);
	}
	CHECK(hostI2cLogAt(0, n - 2).value == 0xA0);
	CHECK(hostI2cLogAt(0, n - 2).ack == 1);
	CHECK(hostI2cLogAt(0, n - 1).type == HOST_I2C_EV_STOP);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK((sr1() & I2C_SR1_AF) == 0u);
}

TEST_CASE_FIXTURE(Rig, "IP5 IsDeviceReady with a missing device and a slow bus uses time: more trials take more ticks") {
	up();
	hostI2cSetByteTicks(3);
	const uint32_t t0 = uwTick;
	CHECK(HAL_I2C_IsDeviceReady(&h, 0xA4, 2, 1000) == HAL_ERROR);
	const uint32_t two = uwTick - t0;
	up();
	hostI2cSetByteTicks(3);
	const uint32_t t1 = uwTick;
	CHECK(HAL_I2C_IsDeviceReady(&h, 0xA4, 5, 1000) == HAL_ERROR);
	const uint32_t five = uwTick - t1;
	CHECK(five > two);
	CHECK(two >= 6u); /* each trial waits for the 3 tick address byte */
}

TEST_CASE_FIXTURE(Rig, "IP5 characterization: IsDeviceReady whose ADDR wait runs into the Timeout treats the trial as not answered") {
	up();
	hostI2cSetByteTicks(20);
	/* Timeout 5 ms: the address byte takes 20 ticks, the wait loop gives up first. */
	const HAL_StatusTypeDef r = HAL_I2C_IsDeviceReady(&h, 0xA0, 1, 5);
	CHECK(r == HAL_ERROR);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(Rig, "IP5 characterization: IsDeviceReady with a bus held busy by another controller returns HAL_BUSY after 25 ms") {
	up();
	hostI2cBusBusy(0, 1);
	const uint32_t t0 = uwTick;
	CHECK(HAL_I2C_IsDeviceReady(&h, 0xA0, 1, 100) == HAL_BUSY);
	CHECK(uwTick - t0 > 25u);
	CHECK(uwTick - t0 < 40u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(hostI2cLogCount(0) == 0u);
}

TEST_CASE_FIXTURE(Rig, "IP5 IsDeviceReady is in the BUSY state while the address is on the bus") {
	up();
	hostI2cSetByteTicks(5);
	Snap s{&h, HAL_I2C_STATE_RESET, HAL_I2C_MODE_NONE, HAL_UNLOCKED, false};
	const I2cScriptAction a[1] = {{uwTick + 2u, snapFn, &s}};
	i2cScriptStart(a, 1);
	REQUIRE(HAL_I2C_IsDeviceReady(&h, 0xA0, 1, 1000) == HAL_OK);
	REQUIRE(s.taken);
	CHECK(s.state == HAL_I2C_STATE_BUSY);
	CHECK(s.lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(Rig, "IP5 IsDeviceReady enables PE when it is 0 and clears POS") {
	up();
	h.Instance->CR1 &= ~I2C_CR1_PE;
	REQUIRE(HAL_I2C_IsDeviceReady(&h, 0xA0, 1, 100) == HAL_OK);
	CHECK((cr1() & I2C_CR1_PE) != 0u);
	h.Instance->CR1 |= I2C_CR1_POS;
	REQUIRE(HAL_I2C_IsDeviceReady(&h, 0xA0, 1, 100) == HAL_OK);
	CHECK((cr1() & I2C_CR1_POS) == 0u);
}

TEST_CASE_FIXTURE(Rig, "IP5 IsDeviceReady whose BUSY wait after the stop never ends returns HAL_ERROR") {
	up();
	hostI2cSetByteTicks(2);
	trigArm(0, 1, [](unsigned i) { hostI2cBusBusy(i, 1); }); /* another controller holds the bus from the START on: BUSY stays 1 after the HAL's STOP */
	const HAL_StatusTypeDef r = HAL_I2C_IsDeviceReady(&h, 0xA0, 1, 1000);
	CHECK(r == HAL_ERROR);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(Rig, "IP5 IsDeviceReady with a missing device whose BUSY wait after the stop never ends returns HAL_ERROR") {
	up();
	hostI2cSetByteTicks(2);
	trigArm(0, 1, [](unsigned i) { hostI2cBusBusy(i, 1); });
	const HAL_StatusTypeDef r = HAL_I2C_IsDeviceReady(&h, 0xA4, 3, 1000);
	CHECK(r == HAL_ERROR);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
}

/* ======================================================================== IP6 10 bit addressing */
TEST_CASE_FIXTURE(Rig, "IP6 10 bit Master_Transmit sends S, header 11110xx0, the second address byte, the data, P") {
	for (unsigned n : {1u, 2u, 3u}) {
		INFO("size " << n);
		up(I2C1, I2C_ADDRESSINGMODE_10BIT, 0x2A5, true);
		uint8_t d[4] = {0x11, 0x22, 0x33, 0x44};
		REQUIRE(HAL_I2C_Master_Transmit(&h, 0x2A5, d, (uint16_t)n, 100) == HAL_OK);
		Log want{evS(), evO(0xF4), evO(0xA5)};
		for (unsigned k = 0; k < n; ++k) want.push_back(evO(d[k]));
		want.push_back(evP());
		expectLog(0, want);
		CHECK(h.State == HAL_I2C_STATE_READY);
		h = I2C_HandleTypeDef{};
		hostI2cResetState();
	}
}

TEST_CASE_FIXTURE(Rig, "IP6 10 bit header carries address bits 9:8 (0x000 -> F0, 0x1FF -> F2, 0x2A5 -> F4, 0x3FF -> F6)") {
	const struct {
		uint16_t addr;
		uint8_t header, low;
	} cases[] = {{0x000, 0xF0, 0x00}, {0x1FF, 0xF2, 0xFF}, {0x2A5, 0xF4, 0xA5}, {0x3FF, 0xF6, 0xFF}};
	for (const auto &c : cases) {
		INFO("address " << c.addr);
		up(I2C1, I2C_ADDRESSINGMODE_10BIT, c.addr, true);
		uint8_t d[1] = {0x5A};
		REQUIRE(HAL_I2C_Master_Transmit(&h, c.addr, d, 1, 100) == HAL_OK);
		expectLog(0, Log{evS(), evO(c.header), evO(c.low), evO(0x5A), evP()});
		h = I2C_HandleTypeDef{};
		hostI2cResetState();
	}
}

TEST_CASE_FIXTURE(Rig, "IP6 10 bit Master_Receive sends the header and the address as for a write, then Sr, header 11110xx1 and receives the data") {
	for (unsigned n : {1u, 2u, 3u, 4u}) {
		INFO("size " << n);
		up(I2C1, I2C_ADDRESSINGMODE_10BIT, 0x2A5, true);
		queue(8);
		uint8_t got[8] = {};
		REQUIRE(HAL_I2C_Master_Receive(&h, 0x2A5, got, (uint16_t)n, 100) == HAL_OK);
		Log want{evS(), evO(0xF4), evO(0xA5), evSr(), evO(0xF5)};
		for (unsigned k = 0; k < n; ++k) want.push_back(evI(pat(k), (uint8_t)(k + 1 < n)));
		want.push_back(evP());
		expectLog(0, want);
		for (unsigned k = 0; k < n; ++k) CHECK(got[k] == pat(k));
		h = I2C_HandleTypeDef{};
		hostI2cResetState();
	}
}

TEST_CASE_FIXTURE(Rig, "IP6 10 bit transmit and receive work on I2C2 and I2C3 too") {
	for (I2C_TypeDef *inst : {I2C2, I2C3}) {
		h = I2C_HandleTypeDef{};
		up(inst, I2C_ADDRESSINGMODE_10BIT, 0x1B2, true);
		INFO("instance " << idx);
		queue(2);
		uint8_t d[1] = {0x66};
		REQUIRE(HAL_I2C_Master_Transmit(&h, 0x1B2, d, 1, 100) == HAL_OK);
		expectLog(idx, Log{evS(), evO(0xF2), evO(0xB2), evO(0x66), evP()});
		hostI2cLogClear(idx);
		uint8_t got[2] = {};
		REQUIRE(HAL_I2C_Master_Receive(&h, 0x1B2, got, 2, 100) == HAL_OK);
		expectLog(idx, Log{evS(), evO(0xF2), evO(0xB2), evSr(), evO(0xF3), evI(pat(0), 1), evI(pat(1), 0), evP()});
	}
}

TEST_CASE_FIXTURE(Rig, "IP6 10 bit Master_Transmit to a missing device: the header is nacked, HAL_ERROR, ErrorCode AF, STOP") {
	up(I2C1, I2C_ADDRESSINGMODE_10BIT, 0x2A5, true);
	uint8_t d[1] = {1};
	CHECK(HAL_I2C_Master_Transmit(&h, 0x0A5, d, 1, 100) == HAL_ERROR); /* header F0: nobody */
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	expectLog(0, Log{evS(), evO(0xF0, 0), evP()});
	expectReleased();
}

TEST_CASE_FIXTURE(Rig, "IP6 10 bit Master_Transmit: header acked, second address byte not acked: HAL_ERROR, ErrorCode AF, STOP") {
	up(I2C1, I2C_ADDRESSINGMODE_10BIT, 0x2A5, true);
	uint8_t d[1] = {1};
	CHECK(HAL_I2C_Master_Transmit(&h, 0x2A6, d, 1, 100) == HAL_ERROR); /* same header 0xF4, other low byte */
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	expectLog(0, Log{evS(), evO(0xF4), evO(0xA6, 0), evP()});
	expectReleased();
}

TEST_CASE_FIXTURE(Rig, "IP6 10 bit Master_Receive to a missing device and to a wrong low byte ends in AF with a STOP") {
	up(I2C1, I2C_ADDRESSINGMODE_10BIT, 0x2A5, true);
	uint8_t got[2];
	CHECK(HAL_I2C_Master_Receive(&h, 0x0A5, got, 2, 100) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	expectLog(0, Log{evS(), evO(0xF0, 0), evP()});
	expectReleased();
	hostI2cLogClear(0);
	CHECK(HAL_I2C_Master_Receive(&h, 0x2A6, got, 2, 100) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	expectLog(0, Log{evS(), evO(0xF4), evO(0xA6, 0), evP()});
	expectReleased();
}

TEST_CASE_FIXTURE(Rig, "IP6 10 bit transfers on a stuck bus end with a timeout (START wait) and on a slow bus in the header wait") {
	up(I2C1, I2C_ADDRESSINGMODE_10BIT, 0x2A5, true);
	hostI2cStuck(0, 1);
	uint8_t d[1] = {1};
	CHECK(HAL_I2C_Master_Transmit(&h, 0x2A5, d, 1, 3) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_WRONG_START);
	CHECK(HAL_I2C_Master_Receive(&h, 0x2A5, d, 1, 3) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_WRONG_START);
	hostI2cStuck(0, 0);
	hostI2cResetState();
	h = I2C_HandleTypeDef{};
	up(I2C1, I2C_ADDRESSINGMODE_10BIT, 0x2A5, true);
	hostI2cSetByteTicks(30);
	CHECK(HAL_I2C_Master_Transmit(&h, 0x2A5, d, 1, 10) == HAL_ERROR); /* ADD10 wait times out */
	CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(Rig, "IP6 10 bit Master_Transmit and Master_Receive time out in the second byte, in the restart and in the read header phase") {
	for (uint32_t to : {40u, 70u, 100u, 130u, 160u}) {
		for (int rx = 0; rx < 2; ++rx) {
			INFO("timeout " << to << " rx " << rx);
			up(I2C1, I2C_ADDRESSINGMODE_10BIT, 0x2A5, true);
			queue(2);
			hostI2cSetByteTicks(30);
			uint8_t d[2] = {1, 2};
			const HAL_StatusTypeDef r = rx ? HAL_I2C_Master_Receive(&h, 0x2A5, d, 2, to) : HAL_I2C_Master_Transmit(&h, 0x2A5, d, 2, to);
			if (r == HAL_ERROR) {
				CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
				CHECK(h.State == HAL_I2C_STATE_READY);
				CHECK(h.Lock == HAL_UNLOCKED);
			}
			h = I2C_HandleTypeDef{};
			hostI2cResetState();
		}
	}
}

/* the 10 bit read tail (clear ADDR after the second byte, repeated start, SB wait, read header) is blocked by LD-1 (see the report) */

/* ======================================================================== more IP3 / IP4 */
TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write and Mem_Read enable the peripheral when PE is 0") {
	up();
	queue(1);
	h.Instance->CR1 &= ~I2C_CR1_PE;
	uint8_t d[1] = {0x42};
	REQUIRE(HAL_I2C_Mem_Write(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, d, 1, 100) == HAL_OK);
	CHECK((cr1() & I2C_CR1_PE) != 0u);
	expectLog(0, Log{evS(), evO(0xA0), evO(0x10), evO(0x42), evP()});
	hostI2cLogClear(0);
	h.Instance->CR1 &= ~I2C_CR1_PE;
	REQUIRE(HAL_I2C_Mem_Read(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, d, 1, 100) == HAL_OK);
	CHECK((cr1() & I2C_CR1_PE) != 0u);
	CHECK(d[0] == pat(0));
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write and Mem_Read clear POS before they start") {
	up();
	queue(1);
	uint8_t d[1] = {0x42};
	h.Instance->CR1 |= I2C_CR1_POS;
	REQUIRE(HAL_I2C_Mem_Write(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, d, 1, 100) == HAL_OK);
	CHECK((cr1() & I2C_CR1_POS) == 0u);
	h.Instance->CR1 |= I2C_CR1_POS;
	REQUIRE(HAL_I2C_Mem_Read(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, d, 1, 100) == HAL_OK);
	CHECK((cr1() & I2C_CR1_POS) == 0u);
	CHECK(d[0] == pat(0));
}

TEST_CASE_FIXTURE(Rig, "IP3 Mem_Read with a 16 bit address on a slow bus: a nack of the first address byte ends in HAL_ERROR, AF, STOP, no repeated start") {
	up();
	hostI2cSetByteTicks(2);
	hostI2cDeviceNackDataAt(0, dev, 0);
	uint8_t got[2] = {0xEE, 0xEE};
	CHECK(HAL_I2C_Mem_Read(&h, 0xA0, 0x1234, I2C_MEMADD_SIZE_16BIT, got, 2, 1000) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	for (unsigned k = 0; k < hostI2cLogCount(0); ++k) CHECK(hostI2cLogAt(0, k).type != HOST_I2C_EV_RESTART);
	CHECK(hostI2cLogAt(0, hostI2cLogCount(0) - 1).type == HOST_I2C_EV_STOP);
	CHECK(got[0] == 0xEE);
	expectReleased();
}

/* Timing derivation (model: a byte takes 30 ticks, one tick per HAL_GetTick call, Tickstart = the first call of the HAL function = tick 1):
 * the address byte is written at tick 1 and ends at tick 31. HAL wait loop: GetTick, then `(tick - 1) > Timeout` and, if so, the flag is read once more.
 * Timeout T times out at the address phase iff the flag is still 0 at the first tick with tick - 1 > T, i.e. tick T + 2 < 31, T < 29.
 * So T = 29 completes the address phase (tick 31 re-reads the flag after the model advanced), T = 28 times out. A transmit of 0 bytes ends there.
 * A 1 byte receive starts its data byte when ADDR is cleared at tick 31 and it ends at tick 61: T = 59 completes, T = 58 times out. */
TEST_CASE_FIXTURE(Rig, "IP3 characterization: Master_Transmit of 0 bytes completes with Timeout 29 and times out with Timeout 28 (30 tick address byte)") {
	for (uint32_t to : {29u, 28u}) {
		INFO("timeout " << to);
		up();
		hostI2cSetByteTicks(30);
		uint8_t d[1] = {1};
		const HAL_StatusTypeDef r = HAL_I2C_Master_Transmit(&h, 0xA0, d, 0, to);
		if (to == 29u) {
			CHECK(r == HAL_OK);
			CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
		} else {
			CHECK(r == HAL_ERROR);
			CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
			CHECK(h.State == HAL_I2C_STATE_READY);
			CHECK(h.Lock == HAL_UNLOCKED);
		}
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: Master_Receive of 1 byte completes with Timeout 59 and times out with Timeout 58 (30 tick bytes)") {
	for (uint32_t to : {59u, 58u}) {
		INFO("timeout " << to);
		up();
		queue(1);
		hostI2cSetByteTicks(30);
		uint8_t got[1] = {};
		const HAL_StatusTypeDef r = HAL_I2C_Master_Receive(&h, 0xA0, got, 1, to);
		if (to == 59u) {
			CHECK(r == HAL_OK);
			CHECK(got[0] == pat(0));
		} else {
			CHECK(r == HAL_ERROR);
			CHECK(h.ErrorCode == HAL_I2C_ERROR_TIMEOUT);
			CHECK(h.State == HAL_I2C_STATE_READY);
			CHECK(h.Lock == HAL_UNLOCKED);
		}
	}
}

/* Sweeps over Timeout. Deterministic regions: T < 29 (the address byte alone outlasts it): HAL_ERROR, TIMEOUT or WRONG_START, READY, unlocked;
 * T >= 30 * (bytes on the bus) + 30: HAL_OK. In between only the clean-ending invariant is asserted. */
namespace {
void sweepCheck(Rig &t, HAL_StatusTypeDef r, uint32_t to, unsigned busBytes) {
	if (to < 29u) {
		CHECK(r == HAL_ERROR);
	} else if (to >= 30u * busBytes + 30u) {
		CHECK(r == HAL_OK);
	}
	if (r != HAL_OK) {
		CHECK(r == HAL_ERROR);
		CHECK((t.h.ErrorCode == HAL_I2C_ERROR_TIMEOUT || t.h.ErrorCode == HAL_I2C_WRONG_START));
		CHECK(t.h.Lock == HAL_UNLOCKED);
	}
	CHECK(t.h.State == HAL_I2C_STATE_READY);
	CHECK(t.h.Mode == HAL_I2C_MODE_NONE);
}
const uint32_t kSweepTimeouts[] = {1, 10, 28, 35, 40, 55, 70, 85, 100, 115, 130, 145, 160, 175, 190, 205, 220, 235, 250, 265, 280, 295, 310, 340, 370, 400, 450, 500};
} // namespace

TEST_CASE_FIXTURE(Rig, "IP3 characterization: Mem_Read and Master_Receive over a range of Timeouts: short ones fail, long ones complete, all end cleanly") {
	for (int mem = 0; mem < 3; ++mem) { /* 0: Master_Receive, 1: Mem_Read 8 bit, 2: Mem_Read 16 bit */
		for (unsigned n : {1u, 2u, 3u, 4u, 6u}) {
			for (uint32_t to : kSweepTimeouts) {
				INFO("mode " << mem << " size " << n << " timeout " << to);
				up();
				queue(8);
				hostI2cSetByteTicks(30);
				uint8_t got[8] = {};
				HAL_StatusTypeDef r;
				unsigned busBytes;
				if (mem == 0) {
					r = HAL_I2C_Master_Receive(&h, 0xA0, got, (uint16_t)n, to);
					busBytes = 1 + n;
				} else {
					r = HAL_I2C_Mem_Read(&h, 0xA0, 0x1234, mem == 1 ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT, got, (uint16_t)n, to);
					busBytes = 2 + (unsigned)mem + n;
				}
				if (r == HAL_OK)
					for (unsigned k = 0; k < n; ++k) CHECK(got[k] == pat(k));
				sweepCheck(*this, r, to, busBytes);
			}
		}
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 characterization: Master_Transmit and Mem_Write over a range of Timeouts: short ones fail, long ones complete, all end cleanly") {
	for (int mem = 0; mem < 3; ++mem) {
		for (unsigned n : {1u, 2u, 3u, 4u}) {
			for (uint32_t to : kSweepTimeouts) {
				INFO("mode " << mem << " size " << n << " timeout " << to);
				up();
				hostI2cSetByteTicks(30);
				uint8_t d[4] = {1, 2, 3, 4};
				HAL_StatusTypeDef r;
				unsigned busBytes;
				if (mem == 0) {
					r = HAL_I2C_Master_Transmit(&h, 0xA0, d, (uint16_t)n, to);
					busBytes = 1 + n;
				} else {
					r = HAL_I2C_Mem_Write(&h, 0xA0, 0x1234, mem == 1 ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT, d, (uint16_t)n, to);
					busBytes = 1 + (unsigned)mem + n;
				}
				sweepCheck(*this, r, to, busBytes);
			}
		}
	}
}

/* ======================================================================== memory address byte nacks */
TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write: a nack of the MSB or the LSB of a 16 bit memory address ends in HAL_ERROR, AF, STOP, READY, unlocked") {
	for (int at : {0, 1}) {
		for (unsigned ticks : {0u, 2u}) {
			INFO("nack at memory address byte " << at << " byteTicks " << ticks);
			up();
			hostI2cSetByteTicks(ticks);
			hostI2cDeviceNackDataAt(0, dev, at);
			uint8_t d[2] = {0x11, 0x22};
			CHECK(HAL_I2C_Mem_Write(&h, 0xA0, 0x1234, I2C_MEMADD_SIZE_16BIT, d, 2, 1000) == HAL_ERROR);
			CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
			CHECK(hostI2cLogAt(0, 2 + at).ack == 0);
			CHECK(hostI2cLogAt(0, 2 + at).value == (at == 0 ? 0x12 : 0x34));
			CHECK(hostI2cLogAt(0, hostI2cLogCount(0) - 1).type == HOST_I2C_EV_STOP);
			CHECK((cr1() & I2C_CR1_PE) != 0u);
			expectReleased();
		}
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Write: a nack of the 8 bit memory address byte ends in HAL_ERROR, AF, STOP and no data byte is sent after it") {
	for (unsigned ticks : {0u, 2u}) {
		INFO("byteTicks " << ticks);
		up();
		hostI2cSetByteTicks(ticks);
		hostI2cDeviceNackDataAt(0, dev, 0);
		uint8_t d[2] = {0x11, 0x22};
		CHECK(HAL_I2C_Mem_Write(&h, 0xA0, 0x34, I2C_MEMADD_SIZE_8BIT, d, 2, 1000) == HAL_ERROR);
		CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
		expectLog(0, Log{evS(), evO(0xA0), evO(0x34, 0), evP()});
		expectReleased();
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 Mem_Read: a nack of the MSB of a 16 bit memory address on a slow bus ends in HAL_ERROR, AF, STOP, no repeated start") {
	for (int at : {0}) {
		INFO("nack at memory address byte " << at);
		up();
		hostI2cSetByteTicks(2);
		hostI2cDeviceNackDataAt(0, dev, at);
		uint8_t got[2] = {0xEE, 0xEE};
		CHECK(HAL_I2C_Mem_Read(&h, 0xA0, 0x1234, I2C_MEMADD_SIZE_16BIT, got, 2, 1000) == HAL_ERROR);
		CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
		for (unsigned k = 0; k < hostI2cLogCount(0); ++k) CHECK(hostI2cLogAt(0, k).type != HOST_I2C_EV_RESTART);
		CHECK(hostI2cLogAt(0, hostI2cLogCount(0) - 1).type == HOST_I2C_EV_STOP);
		CHECK(got[0] == 0xEE);
		CHECK((cr1() & I2C_CR1_PE) != 0u);
		expectReleased();
	}
}

TEST_CASE_FIXTURE(Rig, "IP3 IsDeviceReady clears an old ErrorCode at the start of the call") {
	up();
	h.ErrorCode = 0x7Fu;
	REQUIRE(HAL_I2C_IsDeviceReady(&h, 0xA0, 1, 100) == HAL_OK);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
}

TEST_CASE_FIXTURE(Rig, "IP7 no data byte is on the bus while ADDR is still set, and ADDR is cleared when the call ends") {
	/* RM0090 27.3.3: the first data byte goes out only after ADDR was cleared (SR1 then SR2 read). The step checks the log at every tick. */
	struct V {
		static bool &bad() { static bool b; return b; }
		static void step() {
			HAL_IncTick();
			if ((HalFixture::peek(&I2C1->SR1) & I2C_SR1_ADDR) != 0u && hostI2cLogCount(0) > 2u) bad() = true;
		}
	};
	for (int f = 0; f < 3; ++f) {
		INFO("function " << f);
		up();
		queue(2);
		hostI2cSetByteTicks(3);
		V::bad() = false;
		hostTickSetStep(V::step);
		uint8_t d[2] = {1, 2};
		HAL_StatusTypeDef r;
		if (f == 0) r = HAL_I2C_Master_Transmit(&h, 0xA0, d, 2, 1000);
		else if (f == 1) r = HAL_I2C_Mem_Write(&h, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, d, 2, 1000);
		else r = HAL_I2C_Mem_Write(&h, 0xA0, 0x1010, I2C_MEMADD_SIZE_16BIT, d, 2, 1000);
		CHECK(r == HAL_OK);
		CHECK(!V::bad());
		CHECK((sr1() & I2C_SR1_ADDR) == 0u);
		i2cScriptStop();
	}
}

TEST_CASE_FIXTURE(Rig, "IP4 characterization: Mem_Read with a nacked LSB of a 16 bit memory address (slow bus) sends the repeated start, then the stale AF ends the call with HAL_ERROR (NEEDS REVIEW 321)") {
	up();
	queue(2);
	hostI2cSetByteTicks(2);
	hostI2cDeviceNackDataAt(0, dev, 1);
	uint8_t got[2] = {0xEE, 0xEE};
	const HAL_StatusTypeDef r = HAL_I2C_Mem_Read(&h, 0xA0, 0x1234, I2C_MEMADD_SIZE_16BIT, got, 2, 1000);
	CHECK(r == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(got[0] == 0xEE);
	REQUIRE(hostI2cLogCount(0) >= 5u);
	CHECK(hostI2cLogAt(0, 3).ack == 0);
	CHECK(hostI2cLogAt(0, 4).type == HOST_I2C_EV_RESTART);
}
