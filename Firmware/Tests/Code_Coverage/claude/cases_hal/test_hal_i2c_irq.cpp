/*
 * P12 T4c group I2T: I2C interrupt mode (HAL_I2C_Master_Transmit_IT / Master_Receive_IT / Mem_Write_IT / Mem_Read_IT /
 * Slave_Transmit_IT / Slave_Receive_IT / Master_Abort_IT, HAL_I2C_EV_IRQHandler, HAL_I2C_ER_IRQHandler and the error
 * treatment behind them) of stm32f4xx_hal_i2c.c against the I2C register and bus model (fakes/stm32_hal/HalI2c.h).
 * Requirements IT1-IT5 of Tests/Code_Coverage/claude/charters/hal/P12_T4C_REQUIREMENTS.md; compiled into ffb_hal_tests and
 * ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 chapter 36 (the IT functions return after starting the transfer, the end of the transfer is
 * indicated through the I2C IRQ and the Tx / Rx complete callbacks, HAL_I2C_ErrorCallback in case of a transfer error,
 * HAL_I2C_AbortCpltCallback at the end of an abort; AddrCallback: "Master request Transfer Direction (Write/Read)") and
 * RM0090 Rev 22 chapter 27 (27.3.3 controller transmitter / receiver sequences incl. the 1 / 2 / N byte receive procedures,
 * 27.3.4 error conditions, 27.4 Table 124 interrupt requests, 27.6 registers, bit names ITEVTEN / ITBUFEN / ITERREN), ES0182
 * 2.10.1 (a spurious bus error in controller mode has no effect on the transfer; clear the flag, nothing else).
 *
 * Interrupts are not delivered by the model: the tests call the HAL handlers (i2cRunIrqs) while the request line of
 * RM0090 Table 124 is pending, or call one handler by hand to observe one step. The bus log of the model (S, address byte,
 * data bytes with ack, P) is the oracle for what happened on the wire.
 *
 * NEEDS REVIEW proposals (characterization cases; the lead assigns the numbers):
 *  NEEDS REVIEW 323  Master_Transmit_IT / Master_Receive_IT / Mem_*_IT / Slave_*_IT return HAL_BUSY when the handle is locked although
 *            the state is READY (UM1725 only says "HAL: status").
 *  NEEDS REVIEW 323  A bus-busy timeout of an IT start function returns HAL_BUSY, sets HAL_I2C_ERROR_TIMEOUT and leaves state READY.
 *  NEEDS REVIEW 323  Master_Transmit_IT / Mem_Write_IT with Size 0: address (and STOP) only, completion callback, no memory address.
 *  NEEDS REVIEW 323  Master_Receive_IT with Size 0 reads one byte into the buffer, wraps XferCount and completes with MasterRx.
 *  NEEDS REVIEW 323  Mem_Read_IT with Size 0 enables no interrupt (nothing happens after the START).
 *  NEEDS REVIEW 323  After a 2 byte master receive POS stays 1 until the next start function clears it.
 *  NEEDS REVIEW 323  Master_Abort_IT ignores its DevAddress argument.
 *  NEEDS REVIEW 323  After an abort the peripheral is disabled (PE = 0) and ErrorCode is NONE.
 *  NEEDS REVIEW 323  Slave_Receive_IT with a STOP before Size bytes: ErrorCallback with HAL_I2C_ERROR_AF (data left over).
 *  NEEDS REVIEW 323 ER handler with AF while a slave receives: no STOP, AF flag cleared, ErrorCallback.
 *  NEEDS REVIEW 323 ErrorCode accumulates all simultaneous error bits and a single ErrorCallback is made.
 *  NEEDS REVIEW 323 After an error the three interrupt enables are cleared (the handle is READY, mode NONE).
 *  NEEDS REVIEW 323 Error in the listen state keeps the state LISTEN; AF additionally ends the listen (ListenCplt, READY).
 *  NEEDS REVIEW 323 After Master_Abort_IT the state is READY but the mode stays MASTER / MEM.
 *  NEEDS REVIEW 323 A 2 byte master receive that reaches BTF without an RXNE interrupt leaves ITBUFEN set.
 */
#include "HalFixture.h"
#include "HalI2c.h"
#include "I2cTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

extern "C" uint32_t SystemCoreClock;

namespace {
constexpr uint32_t kNoOption = 0xFFFF0000u; /* I2C_NO_OPTION_FRAME */
constexpr uint32_t kIts = I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;

/* The bus log as text: S, R (repeated start), P, "A0+" = byte sent by the controller and acked, "r5A-" = byte sent by the
 * target (read by the controller) and nacked by the controller. */
std::string itLog(unsigned idx) {
	std::string s;
	for (unsigned k = 0; k < hostI2cLogCount(idx); ++k) {
		const HostI2cEvent e = hostI2cLogAt(idx, k);
		char b[16];
		switch (e.type) {
		case HOST_I2C_EV_START: s += "S "; break;
		case HOST_I2C_EV_RESTART: s += "R "; break;
		case HOST_I2C_EV_STOP: s += "P "; break;
		case HOST_I2C_EV_BYTE_OUT:
			std::snprintf(b, sizeof b, "%02X%c ", e.value, e.ack ? '+' : '-');
			s += b;
			break;
		case HOST_I2C_EV_BYTE_IN:
			std::snprintf(b, sizeof b, "r%02X%c ", e.value, e.ack ? '+' : '-');
			s += b;
			break;
		default: s += "? "; break;
		}
	}
	if (!s.empty()) s.pop_back();
	return s;
}

std::string itHex(uint8_t v, const char *prefix = "", char ack = '+') {
	char b[16];
	std::snprintf(b, sizeof b, "%s%02X%c", prefix, v, ack);
	return b;
}

/* Expected log of a master write of n bytes: S, address (W), data acked, P. */
std::string itWriteLog(uint8_t addrByte, const uint8_t *d, unsigned n, bool stop = true) {
	std::string s = "S " + itHex(addrByte);
	for (unsigned k = 0; k < n; ++k) s += " " + itHex(d[k]);
	if (stop) s += " P";
	return s;
}

/* Expected log of a master read of n bytes: every byte acked except the last (RM0090 27.3.3). */
std::string itReadLog(uint8_t addrByte, const uint8_t *d, unsigned n) {
	std::string s = "S " + itHex(addrByte);
	for (unsigned k = 0; k < n; ++k) s += " " + itHex(d[k], "r", (k + 1 < n) ? '+' : '-');
	s += " P";
	return s;
}

struct It : I2cFixture {
	I2C_HandleTypeDef h{};
	I2C_TypeDef *inst;
	unsigned idx;
	explicit It(I2C_TypeDef *i = I2C1, unsigned ix = 0) : inst(i), idx(ix) {
		SystemCoreClock = 168000000u;
		RCC->CFGR = (RCC->CFGR & ~(7u << 10)) | (5u << 10); /* PPRE1 /4 -> 42 MHz */
		h.Instance = i;
		h.Init.ClockSpeed = 100000;
		h.Init.DutyCycle = I2C_DUTYCYCLE_2;
		h.Init.OwnAddress1 = 0x30;
		h.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		h.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		h.Init.OwnAddress2 = 0;
		h.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		h.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		uwTickPrio = 15;
	}
	void up() { REQUIRE(HAL_I2C_Init(&h) == HAL_OK); }
	unsigned run() { return i2cRunIrqs(&h, idx); }
	int peer(uint16_t a = 0x50) { return hostI2cDeviceAdd(idx, a, 0); }
	void queue(int dev, const uint8_t *d, unsigned n) { hostI2cDeviceQueue(idx, dev, d, n); }
	uint32_t cr1() const { return inst->CR1; }
	uint32_t cr2() const { return inst->CR2; }
	std::string log() const { return itLog(idx); }
	/* Sets the HAL tick to t and lets the model's engine run (it evaluates on a register access; a CR1 write of the same value is
	 * harmless). The handlers then see the flags that are due at time t. */
	void at(uint32_t t) {
		hostTickSet(t);
		hostI2cPoll(idx);
	}
	void ev() { HAL_I2C_EV_IRQHandler(&h); }
	void er() { HAL_I2C_ER_IRQHandler(&h); }
};
struct It2 : It {
	It2() : It(I2C2, 1) {}
};
struct It3 : It {
	It3() : It(I2C3, 2) {}
};

const uint8_t kData[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xF0, 0x0F};

/* ---- bodies shared by the three instances ---- */
template <class F> void itMasterTxBody(F &f, unsigned n) {
	f.up();
	f.peer();
	uint8_t tx[16];
	std::memcpy(tx, kData, sizeof tx);
	REQUIRE(HAL_I2C_Master_Transmit_IT(&f.h, 0x50 << 1, tx, (uint16_t)n) == HAL_OK);
	f.run();
	CHECK(f.log() == itWriteLog(0xA0, kData, n));
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(i2cCounts.error == 0u);
	CHECK(f.h.State == HAL_I2C_STATE_READY);
	CHECK(f.h.Mode == HAL_I2C_MODE_NONE);
	CHECK(f.h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(i2cCounts.stateAtLast == HAL_I2C_STATE_READY);
	CHECK(f.h.XferCount == 0u);
	CHECK(f.h.pBuffPtr == tx + n);
	CHECK((f.cr2() & kIts) == 0u);
	CHECK(HalFixture::faults() == 0u);
}

template <class F> void itMasterRxBody(F &f, unsigned n) {
	f.up();
	const int d = f.peer();
	f.queue(d, kData, n);
	uint8_t rx[16] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&f.h, 0x50 << 1, rx, (uint16_t)n) == HAL_OK);
	f.run();
	CHECK(f.log() == itReadLog(0xA1, kData, n));
	CHECK(std::memcmp(rx, kData, n) == 0);
	CHECK(f.h.XferCount == 0u);
	CHECK(f.h.pBuffPtr == rx + n);
	CHECK((f.cr1() & I2C_CR1_ACK) == 0u);
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(i2cCounts.error == 0u);
	CHECK(f.h.State == HAL_I2C_STATE_READY);
	CHECK(f.h.Mode == HAL_I2C_MODE_NONE);
	CHECK(f.h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK((f.cr2() & (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN)) == 0u);
	CHECK(HalFixture::faults() == 0u);
}

/* Starts one of the six IT start functions (kind 0..5) on a READY handle. */
HAL_StatusTypeDef itStart(It &f, int kind, uint8_t *buf, uint16_t n) {
	switch (kind) {
	case 0: return HAL_I2C_Master_Transmit_IT(&f.h, 0x50 << 1, buf, n);
	case 1: return HAL_I2C_Master_Receive_IT(&f.h, 0x50 << 1, buf, n);
	case 2: return HAL_I2C_Mem_Write_IT(&f.h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, buf, n);
	case 3: return HAL_I2C_Mem_Read_IT(&f.h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, buf, n);
	case 4: return HAL_I2C_Slave_Transmit_IT(&f.h, buf, n);
	default: return HAL_I2C_Slave_Receive_IT(&f.h, buf, n);
	}
}
} // namespace

/* ================= IT1: start functions ================= */

TEST_CASE_FIXTURE(It, "IT1 Master_Transmit_IT starts: HAL_OK, BUSY_TX, mode MASTER, unlocked, three interrupt enables set") {
	up();
	peer();
	uint8_t tx[2] = {1, 2};
	CHECK(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 2) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(h.Mode == HAL_I2C_MODE_MASTER);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr2() & kIts) == kIts);
	CHECK((cr1() & I2C_CR1_PE) == I2C_CR1_PE);
	CHECK(i2cCounts.masterTx == 0u);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(It, "IT1 Master_Receive_IT starts: HAL_OK, BUSY_RX, mode MASTER, ACK set, three interrupt enables set") {
	up();
	peer();
	uint8_t rx[2];
	CHECK(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 2) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_RX);
	CHECK(h.Mode == HAL_I2C_MODE_MASTER);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr2() & kIts) == kIts);
	CHECK((cr1() & I2C_CR1_ACK) == I2C_CR1_ACK);
	CHECK(i2cCounts.masterRx == 0u);
}

TEST_CASE_FIXTURE(It, "IT1 Mem_Write_IT starts: HAL_OK, BUSY_TX, mode MEM, three interrupt enables set") {
	up();
	peer();
	uint8_t tx[2] = {1, 2};
	CHECK(HAL_I2C_Mem_Write_IT(&h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, tx, 2) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(h.Mode == HAL_I2C_MODE_MEM);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr2() & kIts) == kIts);
}

TEST_CASE_FIXTURE(It, "IT1 Mem_Read_IT starts: HAL_OK, BUSY_RX, mode MEM, ACK set, three interrupt enables set") {
	up();
	peer();
	uint8_t rx[2];
	CHECK(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, rx, 2) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_RX);
	CHECK(h.Mode == HAL_I2C_MODE_MEM);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr2() & kIts) == kIts);
	CHECK((cr1() & I2C_CR1_ACK) == I2C_CR1_ACK);
}

TEST_CASE_FIXTURE(It, "IT1 Slave_Transmit_IT starts: HAL_OK, BUSY_TX, mode SLAVE, ACK set, nothing on the bus") {
	up();
	uint8_t tx[2] = {1, 2};
	CHECK(HAL_I2C_Slave_Transmit_IT(&h, tx, 2) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(h.Mode == HAL_I2C_MODE_SLAVE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr2() & kIts) == kIts);
	CHECK((cr1() & I2C_CR1_ACK) == I2C_CR1_ACK);
	CHECK(hostI2cLogCount(0) == 0u);
}

TEST_CASE_FIXTURE(It, "IT1 Slave_Receive_IT starts: HAL_OK, BUSY_RX, mode SLAVE, ACK set, nothing on the bus") {
	up();
	uint8_t rx[2];
	CHECK(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_RX);
	CHECK(h.Mode == HAL_I2C_MODE_SLAVE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr2() & kIts) == kIts);
	CHECK((cr1() & I2C_CR1_ACK) == I2C_CR1_ACK);
	CHECK(hostI2cLogCount(0) == 0u);
}

TEST_CASE_FIXTURE(It, "IT1 every IT start function returns HAL_BUSY when the state is not READY and changes nothing") {
	up();
	peer();
	uint8_t buf[4] = {};
	for (int kind = 0; kind < 6; ++kind) {
		SUBCASE(("function " + std::to_string(kind)).c_str()) {
			INFO("function " << kind);
			h.State = HAL_I2C_STATE_BUSY_TX;
			h.Mode = HAL_I2C_MODE_MASTER;
			const uint32_t cr2Before = cr2();
			CHECK(itStart(*this, kind, buf, 2) == HAL_BUSY);
			CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
			CHECK(h.Mode == HAL_I2C_MODE_MASTER);
			CHECK(cr2() == cr2Before);
			CHECK(hostI2cLogCount(0) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(It, "IT1 every IT start function returns HAL_BUSY when the state is LISTEN") {
	up();
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	uint8_t buf[4] = {};
	for (int kind = 0; kind < 6; ++kind) {
		SUBCASE(("function " + std::to_string(kind)).c_str()) {
			INFO("function " << kind);
			CHECK(itStart(*this, kind, buf, 2) == HAL_BUSY);
			CHECK(h.State == HAL_I2C_STATE_LISTEN);
		}
	}
}

TEST_CASE_FIXTURE(It, "IT1 characterization: NEEDS REVIEW 323 every IT start function returns HAL_BUSY when the handle is locked in state READY") {
	up();
	peer();
	uint8_t buf[4] = {};
	for (int kind = 0; kind < 6; ++kind) {
		SUBCASE(("function " + std::to_string(kind)).c_str()) {
			INFO("function " << kind);
			h.Lock = HAL_LOCKED;
			CHECK(itStart(*this, kind, buf, 2) == HAL_BUSY);
			CHECK(h.State == HAL_I2C_STATE_READY);
			CHECK(h.Mode == HAL_I2C_MODE_NONE);
			CHECK(hostI2cLogCount(0) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(It, "IT1 characterization: NEEDS REVIEW 323 a busy bus makes a master IT start function return HAL_BUSY with ERROR TIMEOUT and state READY") {
	up();
	peer();
	uint8_t buf[4] = {};
	for (int kind = 0; kind < 4; ++kind) {
		SUBCASE(("function " + std::to_string(kind)).c_str()) {
			INFO("function " << kind);
			hostI2cBusBusy(0, 1);
			CHECK(itStart(*this, kind, buf, 2) == HAL_BUSY);
			CHECK((h.ErrorCode & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT);
			CHECK(h.State == HAL_I2C_STATE_READY);
			CHECK(h.Mode == HAL_I2C_MODE_NONE);
			CHECK(h.PreviousState == 0u);
			CHECK((cr1() & I2C_CR1_START) == 0u);
			CHECK(hostI2cLogCount(0) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(It, "IT1 every IT start function enables PE when it was clear") {
	up();
	peer();
	uint8_t buf[4] = {};
	for (int kind = 0; kind < 6; ++kind) {
		SUBCASE(("function " + std::to_string(kind)).c_str()) {
			INFO("function " << kind);
			inst->CR1 &= ~I2C_CR1_PE;
			REQUIRE(itStart(*this, kind, buf, 2) == HAL_OK);
			CHECK((cr1() & I2C_CR1_PE) == I2C_CR1_PE);
		}
	}
}

TEST_CASE_FIXTURE(It, "IT1 every IT start function clears POS") {
	up();
	peer();
	uint8_t buf[4] = {};
	for (int kind = 0; kind < 6; ++kind) {
		SUBCASE(("function " + std::to_string(kind)).c_str()) {
			INFO("function " << kind);
			inst->CR1 |= I2C_CR1_POS;
			REQUIRE(itStart(*this, kind, buf, 2) == HAL_OK);
			CHECK((cr1() & I2C_CR1_POS) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(It, "IT1 every IT start function sets XferOptions to I2C_NO_OPTION_FRAME") {
	up();
	peer();
	uint8_t buf[4] = {};
	for (int kind = 0; kind < 6; ++kind) {
		SUBCASE(("function " + std::to_string(kind)).c_str()) {
			h.XferOptions = 0;
			REQUIRE(itStart(*this, kind, buf, 2) == HAL_OK);
			CHECK(h.XferOptions == kNoOption);
			CHECK(h.pBuffPtr == buf);
			CHECK(h.XferCount == 2u);
		}
	}
}

TEST_CASE_FIXTURE(It, "IT1 every IT start function clears an old ErrorCode") {
	up();
	peer();
	uint8_t buf[4] = {};
	for (int kind = 0; kind < 6; ++kind) {
		SUBCASE(("function " + std::to_string(kind)).c_str()) {
			INFO("function " << kind);
			h.ErrorCode = HAL_I2C_ERROR_AF | HAL_I2C_ERROR_TIMEOUT;
			REQUIRE(itStart(*this, kind, buf, 2) == HAL_OK);
			CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
		}
	}
}

TEST_CASE_FIXTURE(It, "IT1 Slave_Transmit_IT and Slave_Receive_IT reject a NULL buffer and Size 0 with HAL_ERROR, handle stays READY and unlocked") {
	up();
	uint8_t buf[2] = {};
	CHECK(HAL_I2C_Slave_Transmit_IT(&h, nullptr, 2) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Transmit_IT(&h, buf, 0) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Receive_IT(&h, nullptr, 2) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Receive_IT(&h, buf, 0) == HAL_ERROR);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr2() & kIts) == 0u);
}

TEST_CASE_FIXTURE(It, "IT1 Master_Transmit_IT of 0 bytes completes with address and stop only") {
	itMasterTxBody(*this, 0);
}
TEST_CASE_FIXTURE(It, "IT1 Master_Transmit_IT of 1 byte completes: log S A0 11 P, one callback") {
	itMasterTxBody(*this, 1);
}
TEST_CASE_FIXTURE(It, "IT1 Master_Transmit_IT of 2 bytes completes: log in order, one callback") {
	itMasterTxBody(*this, 2);
}
TEST_CASE_FIXTURE(It, "IT1 Master_Transmit_IT of 3 bytes completes: log in order, one callback") {
	itMasterTxBody(*this, 3);
}
TEST_CASE_FIXTURE(It, "IT1 Master_Transmit_IT of 16 bytes completes: log in order, one callback") {
	itMasterTxBody(*this, 16);
}
TEST_CASE_FIXTURE(It2, "IT1 Master_Transmit_IT of 3 bytes completes on I2C2") { itMasterTxBody(*this, 3); }
TEST_CASE_FIXTURE(It3, "IT1 Master_Transmit_IT of 3 bytes completes on I2C3") { itMasterTxBody(*this, 3); }

TEST_CASE_FIXTURE(It, "IT1 Master_Receive_IT of 1 byte completes: single byte nacked, stop, one callback") {
	itMasterRxBody(*this, 1);
}
TEST_CASE_FIXTURE(It, "IT1 Master_Receive_IT of 2 bytes completes: last byte nacked, data in order") { itMasterRxBody(*this, 2); }
TEST_CASE_FIXTURE(It, "IT1 Master_Receive_IT of 3 bytes completes: last byte nacked, data in order") { itMasterRxBody(*this, 3); }
TEST_CASE_FIXTURE(It, "IT1 Master_Receive_IT of 4 bytes completes: last byte nacked, data in order") { itMasterRxBody(*this, 4); }
TEST_CASE_FIXTURE(It, "IT1 Master_Receive_IT of 5 bytes completes: last byte nacked, data in order") { itMasterRxBody(*this, 5); }
TEST_CASE_FIXTURE(It, "IT1 Master_Receive_IT of 16 bytes completes: last byte nacked, data in order") { itMasterRxBody(*this, 16); }
TEST_CASE_FIXTURE(It2, "IT1 Master_Receive_IT of 4 bytes completes on I2C2") { itMasterRxBody(*this, 4); }
TEST_CASE_FIXTURE(It3, "IT1 Master_Receive_IT of 4 bytes completes on I2C3") { itMasterRxBody(*this, 4); }
TEST_CASE_FIXTURE(It2, "IT1 Master_Receive_IT of 2 bytes completes on I2C2") { itMasterRxBody(*this, 2); }
TEST_CASE_FIXTURE(It3, "IT1 Master_Receive_IT of 1 byte completes on I2C3") { itMasterRxBody(*this, 1); }

TEST_CASE_FIXTURE(It, "IT1 Master_Receive_IT: ACK is clear after a completed 1 byte and N byte receive") {
	up();
	const int d = peer();
	queue(d, kData, 5);
	uint8_t rx[5];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 5) == HAL_OK);
	run();
	REQUIRE(i2cCounts.masterRx == 1u);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(It, "IT1 characterization: NEEDS REVIEW 323 a 2 byte receive that reaches BTF without an RXNE interrupt leaves ITBUFEN set") {
	up();
	const int d = peer();
	queue(d, kData, 2);
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 2) == HAL_OK);
	run();
	REQUIRE(i2cCounts.masterRx == 1u);
	CHECK((cr2() & I2C_CR2_ITBUFEN) == I2C_CR2_ITBUFEN);
}

TEST_CASE_FIXTURE(It, "IT1 Master_Receive_IT of 2 bytes with an RXNE interrupt before BTF clears all three interrupt enables") {
	up();
	const int d = peer();
	queue(d, kData, 2);
	hostI2cSetByteTicks(3);
	uint8_t rx[2] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 2) == HAL_OK);
	for (unsigned t = 0; t < 60 && i2cCounts.masterRx == 0u; ++t) {
		at(t);
		run();
	}
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(std::memcmp(rx, kData, 2) == 0);
	CHECK((cr2() & kIts) == 0u);
}

TEST_CASE_FIXTURE(It, "IT1 characterization: NEEDS REVIEW 323 after a 2 byte master receive POS stays set, the next start function clears it") {
	up();
	const int d = peer();
	queue(d, kData, 2);
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 2) == HAL_OK);
	run();
	REQUIRE(i2cCounts.masterRx == 1u);
	CHECK((cr1() & I2C_CR1_POS) == I2C_CR1_POS);
	uint8_t tx[1] = {1};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 1) == HAL_OK);
	CHECK((cr1() & I2C_CR1_POS) == 0u);
}

TEST_CASE_FIXTURE(It, "IT1 two master transmissions in a row both complete, one callback each") {
	up();
	peer();
	uint8_t tx[2] = {1, 2};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 2) == HAL_OK);
	run();
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 2) == HAL_OK);
	run();
	CHECK(i2cCounts.masterTx == 2u);
	CHECK(log() == "S A0+ 01+ 02+ P S A0+ 01+ 02+ P");
}

TEST_CASE_FIXTURE(It, "IT1 a master receive after a master transmit uses its own address byte with the read bit") {
	up();
	const int d = peer();
	uint8_t tx[1] = {7};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 1) == HAL_OK);
	run();
	queue(d, kData, 1);
	uint8_t rx[1];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 1) == HAL_OK);
	run();
	CHECK(log() == "S A0+ 07+ P S A1+ r11- P");
	CHECK(rx[0] == 0x11);
}

/* Memory mode */
TEST_CASE_FIXTURE(It, "IT1 Mem_Write_IT with an 8 bit memory address of 3 bytes: S A0 mem d d d P, MemTx callback once") {
	up();
	peer();
	uint8_t tx[3] = {0x11, 0x22, 0x33};
	REQUIRE(HAL_I2C_Mem_Write_IT(&h, 0x50 << 1, 0x1234, I2C_MEMADD_SIZE_8BIT, tx, 3) == HAL_OK);
	run();
	CHECK(log() == "S A0+ 34+ 11+ 22+ 33+ P");
	CHECK(i2cCounts.memTx == 1u);
	CHECK(i2cCounts.masterTx == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK((cr2() & kIts) == 0u);
}

TEST_CASE_FIXTURE(It, "IT1 Mem_Write_IT with a 16 bit memory address sends high byte then low byte before the data when the handler is slower than a byte" * KNOWN_ISSUE("V107 (arguable: depends on interrupt latency > one byte time) Mem_Write_IT: with BTF set after the high address byte the BTF path writes the first data byte instead of the low address byte (state BUSY_TX makes the handler call I2C_MasterTransmit_BTF, line 4836), so the low byte of a 16 bit memory address is never sent")) {
	up();
	peer();
	uint8_t tx[2] = {0x11, 0x22};
	REQUIRE(HAL_I2C_Mem_Write_IT(&h, 0x50 << 1, 0x1234, I2C_MEMADD_SIZE_16BIT, tx, 2) == HAL_OK);
	run();
	CHECK(log() == "S A0+ 12+ 34+ 11+ 22+ P");
	CHECK(i2cCounts.memTx == 1u);
}

TEST_CASE_FIXTURE(It, "IT1 Mem_Write_IT of 1 byte completes") {
	up();
	peer();
	uint8_t tx[1] = {0x55};
	REQUIRE(HAL_I2C_Mem_Write_IT(&h, 0x50 << 1, 0x20, I2C_MEMADD_SIZE_8BIT, tx, 1) == HAL_OK);
	run();
	CHECK(log() == "S A0+ 20+ 55+ P");
	CHECK(i2cCounts.memTx == 1u);
}

TEST_CASE_FIXTURE(It3, "IT1 Mem_Write_IT of 2 bytes with a 16 bit address completes on I2C3 when the handler is faster than a byte") {
	up();
	peer();
	hostI2cSetByteTicks(3);
	uint8_t tx[2] = {0x55, 0x66};
	REQUIRE(HAL_I2C_Mem_Write_IT(&h, 0x50 << 1, 0xBEEF, I2C_MEMADD_SIZE_16BIT, tx, 2) == HAL_OK);
	for (unsigned t = 0; t < 100 && i2cCounts.memTx == 0u; ++t) {
		at(t);
		run();
	}
	CHECK(log() == "S A0+ BE+ EF+ 55+ 66+ P");
	CHECK(i2cCounts.memTx == 1u);
}

TEST_CASE_FIXTURE(It, "IT1 characterization: NEEDS REVIEW 323 Mem_Write_IT of 0 bytes ends after the address byte, without a memory address") {
	up();
	peer();
	uint8_t tx[1] = {};
	REQUIRE(HAL_I2C_Mem_Write_IT(&h, 0x50 << 1, 0x20, I2C_MEMADD_SIZE_8BIT, tx, 0) == HAL_OK);
	run();
	CHECK(log() == "S A0+ P");
	CHECK(i2cCounts.memTx == 1u);
}

TEST_CASE_FIXTURE(It, "IT1 Mem_Read_IT with an 8 bit memory address of 3 bytes: S A0 mem Sr A1 reads P") {
	up();
	const int d = peer();
	queue(d, kData, 3);
	uint8_t rx[3] = {};
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x1234, I2C_MEMADD_SIZE_8BIT, rx, 3) == HAL_OK);
	run();
	CHECK(log() == "S A0+ 34+ R A1+ r11+ r22+ r33- P");
	CHECK(i2cCounts.memRx == 1u);
	CHECK(i2cCounts.masterRx == 0u);
	CHECK(std::memcmp(rx, kData, 3) == 0);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(It, "IT1 Mem_Read_IT with a 16 bit memory address sends the high byte first") {
	up();
	const int d = peer();
	queue(d, kData, 2);
	uint8_t rx[2] = {};
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x1234, I2C_MEMADD_SIZE_16BIT, rx, 2) == HAL_OK);
	run();
	CHECK(log() == "S A0+ 12+ 34+ R A1+ r11+ r22- P");
	CHECK(i2cCounts.memRx == 1u);
	CHECK(std::memcmp(rx, kData, 2) == 0);
}

TEST_CASE_FIXTURE(It, "IT1 Mem_Read_IT of 1 byte: the single byte is nacked") {
	up();
	const int d = peer();
	queue(d, kData, 1);
	uint8_t rx[1] = {};
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x20, I2C_MEMADD_SIZE_8BIT, rx, 1) == HAL_OK);
	run();
	CHECK(log() == "S A0+ 20+ R A1+ r11- P");
	CHECK(i2cCounts.memRx == 1u);
	CHECK(rx[0] == 0x11);
}

TEST_CASE_FIXTURE(It, "IT1 Mem_Read_IT of 2 bytes: last byte nacked") {
	up();
	const int d = peer();
	queue(d, kData, 2);
	uint8_t rx[2] = {};
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x20, I2C_MEMADD_SIZE_8BIT, rx, 2) == HAL_OK);
	run();
	CHECK(log() == "S A0+ 20+ R A1+ r11+ r22- P");
	CHECK(i2cCounts.memRx == 1u);
}

TEST_CASE_FIXTURE(It, "IT1 Mem_Read_IT of 6 bytes: last byte nacked, data in order") {
	up();
	const int d = peer();
	queue(d, kData, 6);
	uint8_t rx[6] = {};
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x20, I2C_MEMADD_SIZE_8BIT, rx, 6) == HAL_OK);
	run();
	CHECK(log() == "S A0+ 20+ R A1+ r11+ r22+ r33+ r44+ r55+ r66- P");
	CHECK(i2cCounts.memRx == 1u);
	CHECK(std::memcmp(rx, kData, 6) == 0);
}

TEST_CASE_FIXTURE(It2, "IT1 Mem_Read_IT of 4 bytes completes on I2C2") {
	up();
	const int d = peer();
	queue(d, kData, 4);
	uint8_t rx[4] = {};
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x20, I2C_MEMADD_SIZE_8BIT, rx, 4) == HAL_OK);
	run();
	CHECK(i2cCounts.memRx == 1u);
	CHECK(std::memcmp(rx, kData, 4) == 0);
}

TEST_CASE_FIXTURE(It, "IT1 characterization: NEEDS REVIEW 323 Mem_Read_IT of 0 bytes enables no interrupt and nothing is sent after the START") {
	up();
	peer();
	uint8_t rx[1] = {};
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x20, I2C_MEMADD_SIZE_8BIT, rx, 0) == HAL_OK);
	CHECK((cr2() & kIts) == 0u);
	CHECK(run() == 0u);
	CHECK(i2cCounts.memRx == 0u);
	CHECK(h.State == HAL_I2C_STATE_BUSY_RX);
}

TEST_CASE_FIXTURE(It, "IT1 characterization: NEEDS REVIEW 323 Master_Receive_IT of 0 bytes still reads a byte into the buffer and completes") {
	up();
	peer();
	uint8_t rx[2] = {0xEE, 0xEE};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 0) == HAL_OK);
	run();
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(rx[0] == 0xFF); /* the peer sends 0xFF after its queue */
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.XferCount == 0xFFFFu);
}

/* 10 bit addressing (RM0090 27.3.3 figures 243 / 244: header 11110xx0, second address byte, repeated start, header 11110xx1) */
TEST_CASE_FIXTURE(It, "IT1 Master_Transmit_IT with a 10 bit address sends header F4 and the second address byte A5") {
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	h.Init.OwnAddress1 = 0x155;
	up();
	hostI2cDeviceAdd(0, 0x2A5, 1);
	uint8_t tx[2] = {0x11, 0x22};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x2A5, tx, 2) == HAL_OK);
	run();
	/* 10 bit address 0x2A5 = 10 1010 0101: header 11110 (bits 9:8 = 10) 0 = 1111 0100 = F4, second byte A5 */
	CHECK(log() == "S F4+ A5+ 11+ 22+ P");
	CHECK(i2cCounts.masterTx == 1u);
}

TEST_CASE_FIXTURE(It, "IT1 Master_Receive_IT with a 10 bit address: F4 A5, repeated start, header F5, then the data") {
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	h.Init.OwnAddress1 = 0x155;
	up();
	const int d = hostI2cDeviceAdd(0, 0x2A5, 1);
	queue(d, kData, 3);
	uint8_t rx[3] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x2A5, rx, 3) == HAL_OK);
	run();
	CHECK(log() == "S F4+ A5+ R F5+ r11+ r22+ r33- P");
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(std::memcmp(rx, kData, 3) == 0);
}

/* ================= IT2: event handler, one step at a time ================= */

TEST_CASE_FIXTURE(It, "IT2 SB in master transmit: the handler sends the address with the write bit") {
	up();
	peer();
	uint8_t tx[2] = {1, 2};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 2) == HAL_OK);
	REQUIRE(hostI2cEventPending(0) == 1);
	ev();
	CHECK(log() == "S A0+");
	CHECK((inst->SR1 & I2C_SR1_SB) == 0u);
}

TEST_CASE_FIXTURE(It, "IT2 SB in master receive: the handler sends the address with the read bit") {
	up();
	peer();
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 2) == HAL_OK);
	ev();
	CHECK(log() == "S A1+");
}

TEST_CASE_FIXTURE(It, "IT2 SB in memory mode: first the write address, after the restart the read address") {
	up();
	const int d = peer();
	queue(d, kData, 2);
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, rx, 2) == HAL_OK);
	run();
	CHECK(log() == "S A0+ 10+ R A1+ r11+ r22- P");
}

TEST_CASE_FIXTURE(It, "IT2 SB is ignored while ITEVTEN is clear") {
	up();
	peer();
	uint8_t tx[2] = {1, 2};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 2) == HAL_OK);
	inst->CR2 &= ~I2C_CR2_ITEVTEN;
	ev();
	CHECK(log() == "S");
	CHECK((inst->SR1 & I2C_SR1_SB) == I2C_SR1_SB);
}

TEST_CASE_FIXTURE(It, "IT2 ADDR in master transmit: the handler clears ADDR and nothing else is sent") {
	up();
	peer();
	uint8_t tx[2] = {1, 2};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 2) == HAL_OK);
	ev(); /* SB */
	REQUIRE(hostI2cEventPending(0) == 1);
	ev(); /* ADDR */
	CHECK((inst->SR1 & I2C_SR1_ADDR) == 0u);
	CHECK(log() == "S A0+");
}

TEST_CASE_FIXTURE(It, "IT2 TXE in master transmit: the handler writes the next byte, one byte per call") {
	up();
	peer();
	uint8_t tx[3] = {1, 2, 3};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 3) == HAL_OK);
	ev(); /* SB */
	ev(); /* ADDR */
	ev(); /* TXE */
	CHECK(log() == "S A0+ 01+");
	CHECK(h.XferCount == 2u);
}

TEST_CASE_FIXTURE(It, "IT2 TXE is ignored while ITBUFEN is clear") {
	up();
	peer();
	uint8_t tx[3] = {1, 2, 3};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 3) == HAL_OK);
	ev();
	ev();
	inst->CR2 &= ~I2C_CR2_ITBUFEN;
	ev();
	CHECK(log() == "S A0+");
	CHECK(h.XferCount == 3u);
}

TEST_CASE_FIXTURE(It, "IT2 ADDR in a master receiver of 1 byte: ACK is cleared when ADDR is cleared") {
	up();
	const int d = peer();
	queue(d, kData, 1);
	uint8_t rx[1];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 1) == HAL_OK);
	ev(); /* SB */
	ev(); /* ADDR */
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
	CHECK((inst->SR1 & I2C_SR1_ADDR) == 0u);
}

TEST_CASE_FIXTURE(It, "IT2 ADDR in a master receiver of 2 bytes: ACK cleared and POS set (RM0090 27.3.3 two byte reception)") {
	up();
	const int d = peer();
	queue(d, kData, 2);
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 2) == HAL_OK);
	ev();
	ev();
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
	CHECK((cr1() & I2C_CR1_POS) == I2C_CR1_POS);
}

TEST_CASE_FIXTURE(It, "IT2 ADDR in a master receiver of 3 bytes: ACK stays set, POS clear") {
	up();
	const int d = peer();
	queue(d, kData, 3);
	uint8_t rx[3];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 3) == HAL_OK);
	ev();
	ev();
	CHECK((cr1() & I2C_CR1_ACK) == I2C_CR1_ACK);
	CHECK((cr1() & I2C_CR1_POS) == 0u);
}

TEST_CASE_FIXTURE(It, "IT2 RXNE in a master receiver of 5 bytes: the handler reads one byte per call") {
	up();
	const int d = peer();
	queue(d, kData, 5);
	hostI2cSetByteTicks(4);
	uint8_t rx[5] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 5) == HAL_OK);
	ev(); /* SB: address on the wire */
	at(4);
	REQUIRE(hostI2cEventPending(0) == 1);
	ev(); /* ADDR */
	at(8);
	REQUIRE(hostI2cEventPending(0) == 1);
	ev(); /* RXNE */
	CHECK(rx[0] == 0x11);
	CHECK(h.XferCount == 4u);
	CHECK(h.pBuffPtr == rx + 1);
	CHECK((cr2() & I2C_CR2_ITBUFEN) == I2C_CR2_ITBUFEN);
	at(12);
	ev(); /* RXNE of byte 2: three bytes are left, the BUF interrupt is disabled */
	CHECK(rx[1] == 0x22);
	CHECK(h.XferCount == 3u);
	CHECK(h.pBuffPtr == rx + 2);
	CHECK((cr2() & I2C_CR2_ITBUFEN) == 0u);
	CHECK((cr2() & I2C_CR2_ITEVTEN) == I2C_CR2_ITEVTEN);
}

TEST_CASE_FIXTURE(It, "IT2 RXNE is ignored while ITBUFEN is clear") {
	up();
	const int d = peer();
	queue(d, kData, 5);
	hostI2cSetByteTicks(4);
	uint8_t rx[5] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 5) == HAL_OK);
	ev();
	at(4);
	ev(); /* ADDR */
	at(8);
	inst->CR2 &= ~I2C_CR2_ITBUFEN;
	ev();
	CHECK(h.XferCount == 5u);
	CHECK(rx[0] == 0);
}

TEST_CASE_FIXTURE(It, "IT2 ADD10 in a 10 bit master transmit: the handler sends the second address byte") {
	h.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
	h.Init.OwnAddress1 = 0x155;
	up();
	hostI2cDeviceAdd(0, 0x2A5, 1);
	uint8_t tx[1] = {0x11};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x2A5, tx, 1) == HAL_OK);
	ev(); /* SB: header */
	CHECK((inst->SR1 & I2C_SR1_ADD10) == I2C_SR1_ADD10);
	ev(); /* ADD10: second byte */
	CHECK(log() == "S F4+ A5+");
	CHECK((inst->SR1 & I2C_SR1_ADD10) == 0u);
}

TEST_CASE_FIXTURE(It, "IT2 flags of a master transfer are not handled by the slave path: ADDR while the handle is master is cleared") {
	up();
	peer();
	uint8_t tx[1] = {1};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 1) == HAL_OK);
	ev();
	CHECK(h.Mode == HAL_I2C_MODE_MASTER);
	CHECK(i2cCounts.addr == 0u);
}

TEST_CASE_FIXTURE(It, "IT2 BTF in master transmit: the last byte ends with STOP and one MasterTx callback") {
	up();
	peer();
	hostI2cSetByteTicks(3);
	uint8_t tx[2] = {1, 2};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 2) == HAL_OK);
	for (unsigned t = 0; t < 40 && i2cCounts.masterTx == 0u; ++t) {
		at(t);
		run();
	}
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(log() == "S A0+ 01+ 02+ P");
}

TEST_CASE_FIXTURE(It, "IT2 BTF in a master receiver: with byte times the 4 byte reception delivers the data in order") {
	up();
	const int d = peer();
	queue(d, kData, 4);
	hostI2cSetByteTicks(3);
	uint8_t rx[4] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 4) == HAL_OK);
	for (unsigned t = 0; t < 80 && i2cCounts.masterRx == 0u; ++t) {
		at(t);
		run();
	}
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(std::memcmp(rx, kData, 4) == 0);
}

TEST_CASE_FIXTURE(It, "IT2 BTF in a master receiver with byte times: 6 bytes delivered in order") {
	up();
	const int d = peer();
	queue(d, kData, 6);
	hostI2cSetByteTicks(2);
	uint8_t rx[6] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 6) == HAL_OK);
	for (unsigned t = 0; t < 120 && i2cCounts.masterRx == 0u; ++t) {
		at(t);
		run();
	}
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(std::memcmp(rx, kData, 6) == 0);
}

TEST_CASE_FIXTURE(It, "IT2 BTF in a master receiver with byte times: 3 bytes delivered in order") {
	up();
	const int d = peer();
	queue(d, kData, 3);
	hostI2cSetByteTicks(2);
	uint8_t rx[3] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 3) == HAL_OK);
	for (unsigned t = 0; t < 120 && i2cCounts.masterRx == 0u; ++t) {
		at(t);
		run();
	}
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(std::memcmp(rx, kData, 3) == 0);
}

TEST_CASE_FIXTURE(It, "IT2 BTF in a master receiver with byte times: 2 bytes delivered in order") {
	up();
	const int d = peer();
	queue(d, kData, 2);
	hostI2cSetByteTicks(2);
	uint8_t rx[2] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 2) == HAL_OK);
	for (unsigned t = 0; t < 120 && i2cCounts.masterRx == 0u; ++t) {
		at(t);
		run();
	}
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(std::memcmp(rx, kData, 2) == 0);
}

TEST_CASE_FIXTURE(It, "IT2 memory write with byte times completes with one MemTx callback") {
	up();
	peer();
	hostI2cSetByteTicks(3);
	uint8_t tx[2] = {0x11, 0x22};
	REQUIRE(HAL_I2C_Mem_Write_IT(&h, 0x50 << 1, 0x1234, I2C_MEMADD_SIZE_16BIT, tx, 2) == HAL_OK);
	for (unsigned t = 0; t < 100 && i2cCounts.memTx == 0u; ++t) {
		at(t);
		run();
	}
	CHECK(i2cCounts.memTx == 1u);
	CHECK(log() == "S A0+ 12+ 34+ 11+ 22+ P");
}

/* Model question (lead decision L3): with byte times the HAL requests the repeated START on a TXE interrupt while the low address
 * byte is still on the wire, and then writes dummy bytes (I2C_Flush_DR) on every further TXE; the model sends the byte waiting in DR
 * before it performs the repeated start, so the START never comes. RM0090 27.6.1 START only says "after the current byte". Skipped. */
TEST_CASE_FIXTURE(It, "IT2 memory read with byte times and a 16 bit address delivers the data in order") {
	up();
	const int d = peer();
	queue(d, kData, 4);
	hostI2cSetByteTicks(3);
	uint8_t rx[4] = {};
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x1234, I2C_MEMADD_SIZE_16BIT, rx, 4) == HAL_OK);
	for (unsigned t = 0; t < 200 && i2cCounts.memRx == 0u; ++t) {
		at(t);
		run();
	}
	CHECK(i2cCounts.memRx == 1u);
	CHECK(std::memcmp(rx, kData, 4) == 0);
}

TEST_CASE_FIXTURE(It, "IT2 a handler call with nothing pending changes nothing in master transmit idle state") {
	up();
	peer();
	uint8_t tx[1] = {1};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 1) == HAL_OK);
	run();
	const std::string before = log();
	ev();
	CHECK(log() == before);
	CHECK(i2cCounts.masterTx == 1u);
}

/* ================= IT3: error handler ================= */

namespace {
/* A master transmit of 3 bytes brought to the point after the address (ADDR cleared). */
void itMasterTxToAddr(It &f, uint8_t *tx) {
	f.up();
	f.peer();
	REQUIRE(HAL_I2C_Master_Transmit_IT(&f.h, 0x50 << 1, tx, 3) == HAL_OK);
	f.ev();
	f.ev();
}
} // namespace

TEST_CASE_FIXTURE(It, "IT3 BERR in a master transmit: ErrorCode BERR, flag cleared, one ErrorCallback, state READY") {
	uint8_t tx[3] = {1, 2, 3};
	itMasterTxToAddr(*this, tx);
	hostI2cRaiseSr1(0, I2C_SR1_BERR);
	REQUIRE(hostI2cErrorPending(0) == 1);
	er();
	CHECK((inst->SR1 & I2C_SR1_BERR) == 0u);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.errorCodeAtError == HAL_I2C_ERROR_BERR);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_BERR);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(It, "IT3 ARLO in a master transmit: ErrorCode ARLO, flag cleared, one ErrorCallback, state READY") {
	uint8_t tx[3] = {1, 2, 3};
	itMasterTxToAddr(*this, tx);
	hostI2cArbitrationLost(0);
	REQUIRE(hostI2cErrorPending(0) == 1);
	er();
	CHECK((inst->SR1 & I2C_SR1_ARLO) == 0u);
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_ARLO);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(It, "IT3 ARLO in a master: no STOP is generated by the handler") {
	uint8_t tx[3] = {1, 2, 3};
	itMasterTxToAddr(*this, tx);
	hostI2cArbitrationLost(0);
	er();
	CHECK((cr1() & I2C_CR1_STOP) == 0u);
	CHECK(log().find('P') == std::string::npos);
}

TEST_CASE_FIXTURE(It, "IT3 OVR in a master receive: ErrorCode OVR, flag cleared, one ErrorCallback") {
	up();
	const int d = peer();
	queue(d, kData, 3);
	uint8_t rx[3];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 3) == HAL_OK);
	ev();
	ev();
	hostI2cRaiseSr1(0, I2C_SR1_OVR);
	er();
	CHECK((inst->SR1 & I2C_SR1_OVR) == 0u);
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_OVR);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT3 AF in a master transmit with data pending: ErrorCode AF, STOP generated, one ErrorCallback") {
	up();
	const int d = peer();
	hostI2cDeviceNackDataAt(0, d, 1); /* second data byte nacked */
	uint8_t tx[3] = {1, 2, 3};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 3) == HAL_OK);
	run();
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.masterTx == 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(log() == "S A0+ 01+ 02- P");
	CHECK((inst->SR1 & I2C_SR1_AF) == 0u);
}

TEST_CASE_FIXTURE(It, "IT3 AF after the address byte (no device): ErrorCode AF, STOP on the bus, one ErrorCallback") {
	up();
	uint8_t tx[2] = {1, 2};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x51 << 1, tx, 2) == HAL_OK);
	run();
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(log() == "S A2- P");
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT3 AF after the address byte of a master receive: ErrorCode AF, STOP, one ErrorCallback") {
	up();
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x51 << 1, rx, 2) == HAL_OK);
	run();
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.masterRx == 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(log() == "S A3- P");
}

TEST_CASE_FIXTURE(It, "IT3 AF in a memory write: ErrorCode AF, STOP, one ErrorCallback") {
	up();
	const int d = peer();
	hostI2cDeviceNackDataAt(0, d, 0); /* the memory address byte is nacked */
	uint8_t tx[2] = {1, 2};
	REQUIRE(HAL_I2C_Mem_Write_IT(&h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, tx, 2) == HAL_OK);
	run();
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.memTx == 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(log() == "S A0+ 10- P");
}

TEST_CASE_FIXTURE(It, "IT3 AF in a memory read: ErrorCode AF, STOP, one ErrorCallback") {
	up();
	hostI2cDeviceAdd(0, 0x50, 0);
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x51 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, rx, 2) == HAL_OK);
	run();
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT3 characterization: NEEDS REVIEW 323 BERR, ARLO, AF and OVR at once: all four ErrorCode bits, one ErrorCallback") {
	uint8_t tx[3] = {1, 2, 3};
	itMasterTxToAddr(*this, tx);
	hostI2cRaiseSr1(0, I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR);
	er();
	CHECK(h.ErrorCode == (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_AF | HAL_I2C_ERROR_OVR));
	CHECK(i2cCounts.error == 1u);
	CHECK((inst->SR1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR)) == 0u);
}

TEST_CASE_FIXTURE(It, "IT3 BERR and OVR at once: both ErrorCode bits") {
	uint8_t tx[3] = {1, 2, 3};
	itMasterTxToAddr(*this, tx);
	hostI2cRaiseSr1(0, I2C_SR1_BERR | I2C_SR1_OVR);
	er();
	CHECK(h.ErrorCode == (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_OVR));
	CHECK(i2cCounts.error == 1u);
}

TEST_CASE_FIXTURE(It, "IT3 characterization: NEEDS REVIEW 323 after an error all three interrupt enables are cleared") {
	uint8_t tx[3] = {1, 2, 3};
	itMasterTxToAddr(*this, tx);
	hostI2cRaiseSr1(0, I2C_SR1_BERR);
	er();
	CHECK((cr2() & kIts) == 0u);
}

TEST_CASE_FIXTURE(It, "IT3 error flags are ignored while ITERREN is clear") {
	uint8_t tx[3] = {1, 2, 3};
	itMasterTxToAddr(*this, tx);
	inst->CR2 &= ~I2C_CR2_ITERREN;
	hostI2cRaiseSr1(0, I2C_SR1_BERR | I2C_SR1_AF | I2C_SR1_OVR);
	CHECK(hostI2cErrorPending(0) == 0);
	er();
	CHECK(i2cCounts.error == 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK((inst->SR1 & I2C_SR1_BERR) == I2C_SR1_BERR);
}

TEST_CASE_FIXTURE(It, "IT3 the error handler with no flag set changes nothing") {
	uint8_t tx[3] = {1, 2, 3};
	itMasterTxToAddr(*this, tx);
	er();
	CHECK(i2cCounts.error == 0u);
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
}

TEST_CASE_FIXTURE(It, "IT3 a spurious BERR in a master transmit does not stop the transfer" * KNOWN_ISSUE("V106 BERR in controller mode aborts the interrupt transfer although ES0182 2.10.1 says it continues normally")) {
	/* ES0182 2.10.1: "Detection of bus error has no effect on the I2C-bus transfer in controller mode and any such transfer
	 * continues normally. Workaround: ... the BERR flag must be cleared by software. No other action is required." The HAL
	 * handler clears BERR and then runs I2C_ITError (ErrorCallback, state READY, interrupts off). */
	uint8_t tx[3] = {1, 2, 3};
	itMasterTxToAddr(*this, tx);
	hostI2cRaiseSr1(0, I2C_SR1_BERR);
	run();
	CHECK(i2cCounts.error == 0u);
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(log() == "S A0+ 01+ 02+ 03+ P");
}

/* Slave side of the error handler */
TEST_CASE_FIXTURE(It, "IT3 AF in slave transmit with all bytes sent (controller nack) ends with SlaveTxCplt, no error") {
	up();
	uint8_t tx[2] = {0x77, 0x88};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 2) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	run();
	uint8_t b = 0;
	REQUIRE(hostI2cExtReadByte(0, 1, &b) == 1);
	run();
	REQUIRE(hostI2cExtReadByte(0, 0, &b) == 0);
	run();
	CHECK(b == 0x88);
	CHECK(i2cCounts.slaveTx == 1u);
	CHECK(i2cCounts.error == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK((inst->SR1 & I2C_SR1_AF) == 0u);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
	CHECK((cr2() & kIts) == 0u);
}

TEST_CASE_FIXTURE(It, "IT3 AF in slave transmit with bytes left over is an error: ErrorCode AF, ErrorCallback, no SlaveTxCplt") {
	up();
	uint8_t tx[3] = {0x77, 0x88, 0x99};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	run();
	uint8_t b = 0;
	REQUIRE(hostI2cExtReadByte(0, 0, &b) == 0); /* nack after the first byte */
	run();
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.slaveTx == 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT3 characterization: NEEDS REVIEW 323 AF in slave receive: AF cleared, no STOP request, ErrorCallback with AF") {
	up();
	uint8_t rx[3];
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	hostI2cRaiseSr1(0, I2C_SR1_AF);
	er();
	CHECK((inst->SR1 & I2C_SR1_AF) == 0u);
	CHECK((cr1() & I2C_CR1_STOP) == 0u);
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
}

TEST_CASE_FIXTURE(It, "IT3 BERR in a slave receive: ErrorCode BERR, one ErrorCallback, state READY") {
	up();
	uint8_t rx[3];
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	hostI2cRaiseSr1(0, I2C_SR1_BERR);
	er();
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_BERR);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(It, "IT3 OVR in a slave transmit: ErrorCode OVR, one ErrorCallback") {
	up();
	uint8_t tx[3] = {1, 2, 3};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 3) == HAL_OK);
	hostI2cRaiseSr1(0, I2C_SR1_OVR);
	er();
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_OVR);
}

TEST_CASE_FIXTURE(It, "IT3 ARLO in a slave receive: ErrorCode ARLO, one ErrorCallback") {
	up();
	uint8_t rx[3];
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	hostI2cRaiseSr1(0, I2C_SR1_ARLO);
	er();
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_ARLO);
}

TEST_CASE_FIXTURE(It, "IT3 an error in a master receive of 2 bytes clears POS") {
	up();
	const int d = peer();
	queue(d, kData, 2);
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 2) == HAL_OK);
	ev();
	ev();
	REQUIRE((cr1() & I2C_CR1_POS) == I2C_CR1_POS);
	hostI2cRaiseSr1(0, I2C_SR1_BERR);
	er();
	CHECK((cr1() & I2C_CR1_POS) == 0u);
	CHECK(i2cCounts.error == 1u);
}

TEST_CASE_FIXTURE(It, "IT3 an error in a memory read of 2 bytes clears POS") {
	up();
	const int d = peer();
	queue(d, kData, 2);
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, rx, 2) == HAL_OK);
	inst->CR1 |= I2C_CR1_POS;
	hostI2cRaiseSr1(0, I2C_SR1_OVR);
	er();
	CHECK((cr1() & I2C_CR1_POS) == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT3 an error with a received byte in DR stores the byte in the buffer") {
	up();
	const int d = peer();
	queue(d, kData, 3);
	hostI2cSetByteTicks(4);
	uint8_t rx[3] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 3) == HAL_OK);
	ev();
	at(4);
	ev(); /* ADDR */
	at(8); /* first byte received */
	hostI2cRaiseSr1(0, I2C_SR1_OVR);
	er();
	CHECK(rx[0] == 0x11);
	CHECK(h.pBuffPtr == rx + 1);
}

/* Listen state */
TEST_CASE_FIXTURE(It, "IT3 characterization: NEEDS REVIEW 323 BERR in the listen state keeps LISTEN and reports the error") {
	up();
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	hostI2cRaiseSr1(0, I2C_SR1_BERR);
	er();
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.listen == 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_BERR);
}

TEST_CASE_FIXTURE(It, "IT3 characterization: NEEDS REVIEW 323 AF in the listen state ends the listen: ErrorCallback, then ListenCplt, state READY") {
	up();
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	hostI2cRaiseSr1(0, I2C_SR1_AF);
	er();
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(i2cCounts.order[0] == (unsigned)I2C_CB_ERROR);
	CHECK(i2cCounts.order[1] == (unsigned)I2C_CB_LISTEN);
}

TEST_CASE_FIXTURE(It, "IT3 characterization: AF in LISTEN after a slave transmit (previous state slave tx) only clears the flag") {
	up();
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	h.PreviousState = 0x21u | 0x20u; /* I2C_STATE_SLAVE_BUSY_TX: BUSY_TX bits (0x01) | MODE_SLAVE (0x20) */
	h.Mode = HAL_I2C_MODE_SLAVE;
	h.XferCount = 0;
	hostI2cRaiseSr1(0, I2C_SR1_AF);
	er();
	CHECK((inst->SR1 & I2C_SR1_AF) == 0u);
	CHECK(i2cCounts.error == 0u);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
}

/* ================= IT4: Master_Abort_IT ================= */

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT with no transfer returns HAL_ERROR and changes nothing") {
	up();
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_ERROR);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(i2cCounts.abortCplt == 0u);
	CHECK(hostI2cLogCount(0) == 0u);
}

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT returns HAL_ERROR in slave mode") {
	up();
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x30) == HAL_ERROR);
	CHECK(h.State == HAL_I2C_STATE_BUSY_RX);
	CHECK(i2cCounts.abortCplt == 0u);
}

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT during a master transmit: HAL_OK, one AbortCplt, no other callback, state READY") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	ev();
	ev();
	ev(); /* first data byte */
	h.PreviousState = 0x21u; /* some old value */
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(h.PreviousState == 0u); /* I2C_STATE_NONE */
	CHECK(h.XferOptions == kNoOption);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.Mode == HAL_I2C_MODE_MASTER); /* NEEDS REVIEW 323 */
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
	CHECK(i2cCounts.abortCplt == 1u);
	CHECK(i2cCounts.error == 0u);
	CHECK(i2cCounts.masterTx == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(h.XferCount == 0u);
	CHECK((cr2() & kIts) == 0u);
}

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT generates a STOP condition") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	ev();
	ev();
	ev();
	REQUIRE(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(log().back() == 'P');
}

TEST_CASE_FIXTURE(It, "IT4 characterization: NEEDS REVIEW 323 after an abort Mode stays MASTER although the state is READY") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	ev();
	REQUIRE(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(HAL_I2C_GetMode(&h) == HAL_I2C_MODE_MASTER);
}

TEST_CASE_FIXTURE(It, "IT4 characterization: NEEDS REVIEW 323 after an abort PE is clear and ErrorCode is NONE") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	ev();
	REQUIRE(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK((cr1() & I2C_CR1_PE) == 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
}

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT right after the START (before the address): AbortCplt, state READY") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(i2cCounts.abortCplt == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT after the address (ADDR pending): AbortCplt, state READY") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	ev();
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(i2cCounts.abortCplt == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT during a master receive: AbortCplt once, no MasterRx callback") {
	up();
	const int d = peer();
	queue(d, kData, 5);
	hostI2cSetByteTicks(4);
	uint8_t rx[5] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 5) == HAL_OK);
	ev();
	at(4);
	ev();
	at(8);
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(i2cCounts.abortCplt == 1u);
	CHECK(i2cCounts.masterRx == 0u);
	CHECK(i2cCounts.error == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT during a receive stores the byte waiting in DR") {
	up();
	const int d = peer();
	queue(d, kData, 5);
	hostI2cSetByteTicks(4);
	uint8_t rx[5] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 5) == HAL_OK);
	ev();
	at(4);
	ev();
	at(8); /* first byte in DR */
	REQUIRE(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(rx[0] == 0x11);
}

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT during a memory write: AbortCplt once, no MemTx callback") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Mem_Write_IT(&h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, tx, 4) == HAL_OK);
	ev();
	ev();
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(i2cCounts.abortCplt == 1u);
	CHECK(i2cCounts.memTx == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT during a memory read: AbortCplt once, no MemRx callback") {
	up();
	const int d = peer();
	queue(d, kData, 4);
	uint8_t rx[4] = {};
	REQUIRE(HAL_I2C_Mem_Read_IT(&h, 0x50 << 1, 0x10, I2C_MEMADD_SIZE_8BIT, rx, 4) == HAL_OK);
	ev();
	ev();
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(i2cCounts.abortCplt == 1u);
	CHECK(i2cCounts.memRx == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT4 characterization: NEEDS REVIEW 323 Master_Abort_IT ignores its DevAddress argument") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	ev();
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x7E << 1) == HAL_OK);
	CHECK(i2cCounts.abortCplt == 1u);
}

TEST_CASE_FIXTURE(It, "IT4 Master_Abort_IT returns HAL_BUSY when the handle is locked") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	ev();
	h.Lock = HAL_LOCKED;
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_BUSY);
	CHECK(i2cCounts.abortCplt == 0u);
}

TEST_CASE_FIXTURE(It, "IT4 a new transfer can start after an abort and completes") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	ev();
	REQUIRE(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	hostI2cLogClear(0);
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 2) == HAL_OK);
	run();
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(log() == "S A0+ 01+ 02+ P");
}

TEST_CASE_FIXTURE(It2, "IT4 Master_Abort_IT works on I2C2") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	ev();
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(i2cCounts.abortCplt == 1u);
}

TEST_CASE_FIXTURE(It3, "IT4 Master_Abort_IT works on I2C3") {
	up();
	peer();
	uint8_t tx[4] = {1, 2, 3, 4};
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 4) == HAL_OK);
	ev();
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0x50 << 1) == HAL_OK);
	CHECK(i2cCounts.abortCplt == 1u);
}

/* ================= IT5: slave interrupt mode ================= */

TEST_CASE_FIXTURE(It, "IT5 Slave_Receive_IT of 3 bytes: bytes in order, stop ends it with one SlaveRx callback") {
	up();
	uint8_t rx[3] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	for (uint8_t b : {0xAA, 0xBB, 0xCC}) {
		REQUIRE(hostI2cExtWrite(0, b) == 1);
		run();
	}
	hostI2cExtStop(0);
	run();
	CHECK(rx[0] == 0xAA);
	CHECK(rx[1] == 0xBB);
	CHECK(rx[2] == 0xCC);
	CHECK(i2cCounts.slaveRx == 1u);
	CHECK(i2cCounts.error == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK((cr1() & I2C_CR1_ACK) == 0u);
	CHECK((cr2() & kIts) == 0u);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(It, "IT5 Slave_Receive_IT: the address match does not call AddrCallback outside the listen state") {
	up();
	uint8_t rx[2] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	CHECK(i2cCounts.addr == 0u);
	CHECK((inst->SR1 & I2C_SR1_ADDR) == 0u);
}

TEST_CASE_FIXTURE(It2, "IT5 Slave_Receive_IT of 2 bytes on I2C2 completes") {
	up();
	uint8_t rx[2] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	REQUIRE(hostI2cExtStart(1, 0x30) == 1);
	run();
	REQUIRE(hostI2cExtWrite(1, 0x12) == 1);
	run();
	REQUIRE(hostI2cExtWrite(1, 0x34) == 1);
	run();
	hostI2cExtStop(1);
	run();
	CHECK(rx[0] == 0x12);
	CHECK(rx[1] == 0x34);
	CHECK(i2cCounts.slaveRx == 1u);
}

TEST_CASE_FIXTURE(It3, "IT5 Slave_Transmit_IT of 2 bytes on I2C3 completes with the controller nack") {
	up();
	uint8_t tx[2] = {0x77, 0x88};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 2) == HAL_OK);
	REQUIRE(hostI2cExtStart(2, 0x31) == 1);
	run();
	uint8_t b = 0;
	REQUIRE(hostI2cExtReadByte(2, 1, &b) == 1);
	CHECK(b == 0x77);
	run();
	REQUIRE(hostI2cExtReadByte(2, 0, &b) == 0);
	CHECK(b == 0x88);
	run();
	CHECK(i2cCounts.slaveTx == 1u);
}

TEST_CASE_FIXTURE(It, "IT5 Slave_Transmit_IT of 1 byte: the byte goes out and the controller nack completes the transfer") {
	up();
	uint8_t tx[1] = {0x5A};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 1) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	run();
	uint8_t b = 0;
	REQUIRE(hostI2cExtReadByte(0, 0, &b) == 0);
	run();
	CHECK(b == 0x5A);
	CHECK(i2cCounts.slaveTx == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT5 Slave_Transmit_IT: the controller's stop after the nack leaves the handle READY with one callback") {
	up();
	uint8_t tx[2] = {0x77, 0x88};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 2) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	run();
	uint8_t b = 0;
	REQUIRE(hostI2cExtReadByte(0, 1, &b) == 1);
	run();
	REQUIRE(hostI2cExtReadByte(0, 0, &b) == 0);
	run();
	hostI2cExtStop(0);
	run();
	CHECK(i2cCounts.slaveTx == 1u);
	CHECK(i2cCounts.error == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT5 characterization: NEEDS REVIEW 323 Slave_Receive_IT with a stop before the size is reached ends with ErrorCallback and AF") {
	up();
	uint8_t rx[3] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	REQUIRE(hostI2cExtWrite(0, 0xAA) == 1);
	run();
	REQUIRE(hostI2cExtWrite(0, 0xBB) == 1);
	run();
	hostI2cExtStop(0);
	run();
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.slaveRx == 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(rx[0] == 0xAA);
	CHECK(rx[1] == 0xBB);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It, "IT5 characterization: NEEDS REVIEW 323 Slave_Receive_IT with a stop and no data at all: ErrorCallback with AF, nothing stored") {
	up();
	uint8_t rx[2] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	hostI2cExtStop(0);
	run();
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(rx[0] == 0);
}

TEST_CASE_FIXTURE(It, "IT5 Slave_Receive_IT of 1 byte completes after the stop") {
	up();
	uint8_t rx[1] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 1) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	REQUIRE(hostI2cExtWrite(0, 0xA5) == 1);
	run();
	hostI2cExtStop(0);
	run();
	CHECK(rx[0] == 0xA5);
	CHECK(i2cCounts.slaveRx == 1u);
	CHECK(i2cCounts.error == 0u);
}

TEST_CASE_FIXTURE(It, "IT5 Slave_Receive_IT: more bytes than Size are not stored beyond the buffer end") {
	up();
	uint8_t rx[3] = {0, 0, 0xEE}; /* rx[2] is the guard behind the 2 byte buffer */
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	REQUIRE(hostI2cExtWrite(0, 0x01) == 1);
	run();
	REQUIRE(hostI2cExtWrite(0, 0x02) == 1);
	run();
	hostI2cExtWrite(0, 0x03); /* the third byte: the HAL has no room left */
	run();
	hostI2cExtStop(0);
	run();
	CHECK(rx[0] == 0x01);
	CHECK(rx[1] == 0x02);
	CHECK(rx[2] == 0xEE);
}

TEST_CASE_FIXTURE(It, "IT5 the address of another device is ignored by the slave: nothing happens") {
	up();
	uint8_t rx[2] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	CHECK(hostI2cExtStart(0, 0x40) == 0);
	CHECK(run() == 0u);
	CHECK(h.State == HAL_I2C_STATE_BUSY_RX);
	CHECK(i2cCounts.slaveRx == 0u);
}

TEST_CASE_FIXTURE(It, "IT5 listen: the address match calls AddrCallback with the direction and the own address") {
	up();
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	/* controller writes (address byte 0x30, R/W = 0): UM1725 "Master request Transfer Direction (Write/Read)" = write = I2C_DIRECTION_TRANSMIT */
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	CHECK(i2cCounts.addr == 1u);
	CHECK(i2cCounts.addrDirection == (uint8_t)I2C_DIRECTION_TRANSMIT);
	CHECK(i2cCounts.addrMatchCode == 0x30u);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
}

TEST_CASE_FIXTURE(It, "IT5 listen: a read request calls AddrCallback with I2C_DIRECTION_RECEIVE") {
	up();
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	run();
	CHECK(i2cCounts.addr == 1u);
	CHECK(i2cCounts.addrDirection == (uint8_t)I2C_DIRECTION_RECEIVE);
	CHECK(i2cCounts.addrMatchCode == 0x30u);
}

TEST_CASE_FIXTURE(It, "IT5 listen: AddrCallback leaves BUF interrupt disabled and unlocks the handle") {
	up();
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	h.Lock = HAL_LOCKED;
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((cr2() & I2C_CR2_ITBUFEN) == 0u);
}

TEST_CASE_FIXTURE(It, "IT5 listen: a stop after the address match ends the listen with one ListenCplt") {
	up();
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	i2cCounts.addr = 0;
	hostI2cExtStop(0);
	run();
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(It, "IT5 an error code left in the handle is not cleared by the slave address handling") {
	up();
	uint8_t rx[2] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	h.ErrorCode = HAL_I2C_ERROR_BERR;
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	ev();
	CHECK((inst->SR1 & I2C_SR1_ADDR) == 0u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_BERR);
}

TEST_CASE_FIXTURE(It, "IT5 slave handler: flags are ignored while ITEVTEN is clear") {
	up();
	uint8_t rx[2] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 2) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	inst->CR2 &= ~I2C_CR2_ITEVTEN;
	ev();
	CHECK((inst->SR1 & I2C_SR1_ADDR) == I2C_SR1_ADDR);
}

TEST_CASE_FIXTURE(It, "IT5 slave receive: RXNE is ignored while ITBUFEN is clear") {
	up();
	uint8_t rx[3] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	run();
	REQUIRE(hostI2cExtWrite(0, 0xAA) == 1);
	inst->CR2 &= ~I2C_CR2_ITBUFEN;
	ev();
	CHECK(h.XferCount == 3u);
	CHECK(rx[0] == 0);
}

TEST_CASE_FIXTURE(It, "IT5 slave transmit: TXE is ignored while ITBUFEN is clear") {
	up();
	uint8_t tx[3] = {1, 2, 3};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	ev(); /* ADDR */
	inst->CR2 &= ~I2C_CR2_ITBUFEN;
	ev();
	CHECK(h.XferCount == 3u);
}

TEST_CASE_FIXTURE(It, "IT5 slave transmit: the handler writes one byte per TXE") {
	up();
	uint8_t tx[3] = {1, 2, 3};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	ev(); /* ADDR */
	ev(); /* TXE */
	CHECK(h.XferCount == 2u);
}

/* ================= additional paths: slave BTF, data left at STOPF, dual address, stuck STOP, other instances ================= */

TEST_CASE_FIXTURE(It, "IT2 BTF in slave receive: the handler reads the byte in DR, one byte per call") {
	up();
	uint8_t rx[3] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	ev(); /* ADDR */
	REQUIRE(hostI2cExtWrite(0, 0xAA) == 1);
	hostI2cExtWrite(0, 0xBB); /* RXNE is still set: the byte waits in the shift register, BTF set */
	REQUIRE((hostRegsPeek((uintptr_t)inst + 0x14u) & I2C_SR1_BTF) == I2C_SR1_BTF);
	ev();
	CHECK(rx[0] == 0xAA);
	CHECK(h.XferCount == 2u);
}

TEST_CASE_FIXTURE(It, "IT2 BTF in slave transmit: the handler writes the next byte when TXE and BTF are both set") {
	up();
	uint8_t tx[3] = {1, 2, 3};
	REQUIRE(HAL_I2C_Slave_Transmit_IT(&h, tx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	ev(); /* ADDR */
	ev(); /* TXE: byte 1 */
	uint8_t b = 0;
	REQUIRE(hostI2cExtReadByte(0, 1, &b) == 1);
	CHECK(b == 1);
	hostI2cExtReadByte(0, 1, &b); /* nothing to send: BTF set, stretched */
	REQUIRE((hostRegsPeek((uintptr_t)inst + 0x14u) & I2C_SR1_BTF) == I2C_SR1_BTF);
	ev();
	CHECK(h.XferCount == 1u);
}

TEST_CASE_FIXTURE(It, "IT5 characterization: NEEDS REVIEW 323 STOPF with a received byte still in DR stores it, then reports AF for the missing bytes") {
	up();
	uint8_t rx[3] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	ev(); /* ADDR */
	REQUIRE(hostI2cExtWrite(0, 0xAA) == 1); /* RXNE set, handler not run */
	hostI2cExtStop(0);
	ev();
	CHECK(rx[0] == 0xAA);
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
}

TEST_CASE_FIXTURE(It, "IT5 characterization: NEEDS REVIEW 323 STOPF with two bytes waiting (BTF and RXNE) stores both, then reports AF for the third") {
	up();
	uint8_t rx[3] = {};
	REQUIRE(HAL_I2C_Slave_Receive_IT(&h, rx, 3) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x30) == 1);
	ev(); /* ADDR */
	REQUIRE(hostI2cExtWrite(0, 0xAA) == 1);
	hostI2cExtWrite(0, 0xBB);
	hostI2cExtStop(0);
	ev();
	CHECK(rx[0] == 0xAA);
	CHECK(rx[1] == 0xBB);
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
}

TEST_CASE_FIXTURE(It, "IT5 listen with a second own address: AddrCallback reports OwnAddress2 as the match code") {
	h.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
	h.Init.OwnAddress2 = 0x40;
	up();
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x40) == 1);
	run();
	CHECK(i2cCounts.addr == 1u);
	CHECK(i2cCounts.addrMatchCode == 0x40u);
	CHECK(i2cCounts.addrDirection == (uint8_t)I2C_DIRECTION_TRANSMIT);
}

TEST_CASE_FIXTURE(It, "IT5 listen with a second own address: the first own address still reports OwnAddress1") {
	h.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
	h.Init.OwnAddress2 = 0x40;
	up();
	REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK);
	REQUIRE(hostI2cExtStart(0, 0x31) == 1);
	run();
	CHECK(i2cCounts.addrMatchCode == 0x30u);
	CHECK(i2cCounts.addrDirection == (uint8_t)I2C_DIRECTION_RECEIVE);
}

TEST_CASE_FIXTURE(It, "IT2 characterization: NEEDS REVIEW 323 a 1 byte master receive whose STOP never completes ends with ErrorCallback and TIMEOUT") {
	up();
	const int d = peer();
	queue(d, kData, 1);
	hostI2cSetByteTicks(4);
	uint8_t rx[1] = {};
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x50 << 1, rx, 1) == HAL_OK);
	ev(); /* SB */
	at(4);
	ev(); /* ADDR: ACK off, STOP requested */
	REQUIRE((cr1() & I2C_CR1_STOP) == I2C_CR1_STOP);
	hostI2cStuck(0, 1); /* SCL held low: the STOP does not happen */
	hostRegsPoke((uintptr_t)inst + 0x14u, hostRegsPeek((uintptr_t)inst + 0x14u) | I2C_SR1_RXNE);
	ev();
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.masterRx == 0u);
	CHECK((h.ErrorCode & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK((cr2() & kIts) == 0u);
}

TEST_CASE_FIXTURE(It2, "IT3 BERR in a master transmit on I2C2: ErrorCode BERR, ErrorCallback, state READY") {
	uint8_t tx[3] = {1, 2, 3};
	up();
	peer();
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 3) == HAL_OK);
	ev();
	hostI2cRaiseSr1(1, I2C_SR1_BERR);
	er();
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_BERR);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(It3, "IT3 AF after the address of a master receive on I2C3: ErrorCode AF and STOP") {
	up();
	uint8_t rx[2];
	REQUIRE(HAL_I2C_Master_Receive_IT(&h, 0x51 << 1, rx, 2) == HAL_OK);
	run();
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(log() == "S A3- P");
}

TEST_CASE_FIXTURE(It3, "IT3 ARLO in a master transmit on I2C3: ErrorCode ARLO, ErrorCallback") {
	uint8_t tx[3] = {1, 2, 3};
	up();
	peer();
	REQUIRE(HAL_I2C_Master_Transmit_IT(&h, 0x50 << 1, tx, 3) == HAL_OK);
	ev();
	hostI2cArbitrationLost(2);
	er();
	CHECK(i2cCounts.error == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_ARLO);
}
