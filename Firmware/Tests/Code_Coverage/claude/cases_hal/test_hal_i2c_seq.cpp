/*
 * P12 T4c group I2Q: the sequential transfer functions of stm32f4xx_hal_i2c.c (HAL_I2C_Master_Seq_Transmit_IT,
 * HAL_I2C_Master_Seq_Receive_IT, HAL_I2C_Slave_Seq_Transmit_IT, HAL_I2C_Slave_Seq_Receive_IT), the XferOptions
 * (I2C_FIRST_FRAME ... I2C_OTHER_AND_LAST_FRAME), I2C_ConvertOtherXferOptions and the option dependent branches of the shared
 * interrupt handlers. Requirements IQ1-IQ4 of Tests/Code_Coverage/claude/charters/hal/P12_T4C_REQUIREMENTS.md; compiled into
 * ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 chapter 36 (the XferOptions descriptions: FIRST_FRAME = start, address and data without a final stop;
 * FIRST_AND_NEXT_FRAME = the same, and the same interface may be called again; NEXT_FRAME = restart with address if the
 * direction changes, only data if not, no final stop; LAST_FRAME = the same with a final stop; LAST_FRAME_NO_STOP = ends a
 * FIRST_AND_NEXT / NEXT series so that the opposite interface follows with a restart; OTHER_FRAME = restart with address after
 * every call; OTHER_AND_LAST_FRAME = the same, ending with a stop; FIRST_AND_LAST_FRAME = like the non sequential interface;
 * the completion callback of the current frame; ErrorCallback on a transfer error) and RM0090 Rev 22 chapter 27 (27.3.3
 * controller procedures: repeated start, ACK / POS handling of the last bytes, the NACK of the last received byte; 27.3.2
 * target sequencing; 27.4 Table 124; SR1 AF / BERR / ARLO).
 *
 * Interrupts are not delivered by the model: i2cRunIrqs calls the HAL handlers while a request line is pending. The bus log is
 * rendered as text: S = start, R = repeated start, P = stop, Oxx = byte the controller sent (an address byte included),
 * Ixx = byte the target sent, a trailing ! = not acknowledged. Extra bytes after the requested ones in a receive frame that keeps
 * ACK on (FIRST_AND_NEXT_FRAME, NEXT_FRAME) are the byte pipeline of RM0090 27.3.3 (the controller keeps clocking while ACK = 1),
 * so these cases check the delivered data and the absence of a stop, not the number of clocked bytes.
 *
 * NEEDS REVIEW proposals (characterization cases, the lead assigns the numbers):
 *  NEEDS REVIEW 324: a NEXT_FRAME / LAST_FRAME as the first call of a sequence (nothing was sent before) generates a start condition
 *             (UM1725 describes NEXT_FRAME only as a continuation).
 *  NEEDS REVIEW 324: after a transfer error the next NEXT_FRAME call starts with a start condition, not a repeated start (the
 *             previous state is reset by the error); UM1725 does not say.
 *  NEEDS REVIEW 324: SEQ_NO_OPTION_FRAME (not a sequential option, UM1725 does not list it for the Seq functions) behaves like
 *             FIRST_AND_LAST_FRAME (stop at the end).
 *  NEEDS REVIEW 324: a master receive of Size 0 (FIRST_AND_LAST_FRAME) reads one byte and stores it in the buffer, then stops.
 *  NEEDS REVIEW 324: a slave receive frame cut short by a stop (fewer bytes than Size) ends with HAL_I2C_ERROR_AF in ErrorCode and one
 *             ErrorCallback (UM1725: "transfer error" is not defined for this case), the listen phase is then ended.
 *  NEEDS REVIEW 324: an OTHER_FRAME / OTHER_AND_LAST_FRAME call rewrites hi2c->XferOptions to FIRST_FRAME / FIRST_AND_LAST_FRAME at the
 *             start condition (the documented purpose of I2C_ConvertOtherXferOptions, but the field value is not in UM1725).
 *  NEEDS REVIEW 324: a LAST_FRAME call on a held bus (BUSY = 1 from another controller) does not check BUSY (only FIRST_FRAME,
 *             FIRST_AND_LAST_FRAME or a pending STOP do).
 *  NEEDS REVIEW 324: bytes still in DR and the shift register when a stop ends a short slave receive frame are stored, the frame ends with AF.
 *  NEEDS REVIEW 324: (first proposed as a vendor bug, withdrawn: see the case) a 1 byte FIRST_AND_NEXT_FRAME receive never completes, UM1725 does not promise it.
 *  NEEDS REVIEW 324: after a frame that completes in the TXE handler / the BTF handler of a one-shot receive / Abort_IT / a slave nack, hi2c->PreviousState
 *             has the HAL private values (I2C_STATE_MASTER_BUSY_TX 0x11, MASTER_BUSY_RX 0x12, SLAVE_BUSY_TX 0x21, NONE 0), hand derived from the macros.
 *  NEEDS REVIEW 324: the RXNE handler of the last byte of a 1 byte receive reports HAL_I2C_ERROR_TIMEOUT (ErrorCallback, state READY) when
 *             CR1.STOP does not clear within its 5 ms wait.
 *
 * KNOWN_ISSUE proposals: V108 (a 1 byte LAST receive frame after a FIRST_AND_NEXT frame never completes), V109 (a 1 byte slave transmit frame
 * that starts while the controller waits is sent through the BTF handler and never completes), see the cases.
 */
#include "I2cTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

extern "C" uint32_t SystemCoreClock;

namespace {

std::string seqLog(unsigned i) {
	std::string s;
	char b[16];
	for (unsigned k = 0; k < hostI2cLogCount(i); ++k) {
		const HostI2cEvent e = hostI2cLogAt(i, k);
		if (!s.empty()) s += ' ';
		switch (e.type) {
		case HOST_I2C_EV_START: s += "S"; break;
		case HOST_I2C_EV_RESTART: s += "R"; break;
		case HOST_I2C_EV_STOP: s += "P"; break;
		case HOST_I2C_EV_BYTE_OUT: std::snprintf(b, sizeof b, "O%02X%s", e.value, e.ack ? "" : "!"); s += b; break;
		default: std::snprintf(b, sizeof b, "I%02X%s", e.value, e.ack ? "" : "!"); s += b; break;
		}
	}
	return s;
}

bool seqHasStop(unsigned i) {
	for (unsigned k = 0; k < hostI2cLogCount(i); ++k)
		if (hostI2cLogAt(i, k).type == HOST_I2C_EV_STOP) return true;
	return false;
}

I2C_TypeDef *seqInstance(unsigned i) { return i == 0 ? I2C1 : (i == 1 ? I2C2 : I2C3); }

const uint32_t SEQ_NO_OPTION_FRAME = 0xFFFF0000U; /* I2C_NO_OPTION_FRAME, private to the HAL source */
const uint32_t SEQ_EVT_BUF_ERR = I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;

/* The controller of the cases: handle on an instance, a peer at 7 bit address 0x50 (bus address bytes 0xA0 / 0xA1), our own
 * address 0x30 (bus bytes 0x30 / 0x31). The receive buffer is oversized so that a runaway receive (a defect case) cannot leave it. */
struct SeqRig : I2cFixture {
	I2C_HandleTypeDef h{};
	unsigned ix = 0;
	int dev = -1;
	uint8_t txb[16];
	uint8_t rx[48];

	SeqRig() {
		SystemCoreClock = 168000000u;
		RCC->CFGR = (RCC->CFGR & ~(7u << 10)) | (5u << 10); /* PPRE1 /4: 42 MHz */
		uwTickPrio = 15;
		for (unsigned k = 0; k < sizeof txb; ++k) txb[k] = (uint8_t)(0xA1 + k);
		std::memset(rx, 0, sizeof rx);
		setup(0);
	}

	void setup(unsigned i) {
		ix = i;
		std::memset(&h, 0, sizeof h);
		h.Instance = seqInstance(i);
		h.Init.ClockSpeed = 100000;
		h.Init.DutyCycle = I2C_DUTYCYCLE_2;
		h.Init.OwnAddress1 = 0x30;
		h.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		h.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		h.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		h.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		REQUIRE(HAL_I2C_Init(&h) == HAL_OK);
		dev = hostI2cDeviceAdd(i, 0x50, 0);
	}

	/* A fresh start on another instance (or the same): the model and the handle are reset. */
	void reinit(unsigned i) {
		hostI2cResetState();
		i2cSupportReset();
		std::memset(rx, 0, sizeof rx);
		setup(i);
	}

	void feed(unsigned n) { /* the peer will send 1, 2, ..., n */
		uint8_t q[40];
		for (unsigned k = 0; k < n; ++k) q[k] = (uint8_t)(1 + k);
		hostI2cDeviceQueue(ix, dev, q, n);
	}

	unsigned pump(unsigned limit = 12) { return i2cRunIrqs(&h, ix, limit); }

	HAL_StatusTypeDef tx(uint32_t opt, uint16_t n, const uint8_t *p = nullptr, unsigned limit = 12) {
		const HAL_StatusTypeDef r = HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, const_cast<uint8_t *>(p ? p : txb), n, opt);
		pump(limit);
		return r;
	}
	HAL_StatusTypeDef rxf(uint32_t opt, uint16_t n, uint8_t *p = nullptr, unsigned limit = 12) {
		const HAL_StatusTypeDef r = HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, p ? p : rx, n, opt);
		pump(limit);
		return r;
	}
	/* With a byte time (hostI2cSetByteTicks) the model needs time: advance the HAL tick, let the model evaluate the bus
	 * (hostI2cPoll; a plain SR1 read would arm the SR1-then-SR2 clear sequence of ADDR), then serve the interrupt requests, until
	 * the frame is over (state back to READY) or the tick budget is used. */
	void slowRun(unsigned budget = 300) {
		for (unsigned k = 0; k < budget && h.State != HAL_I2C_STATE_READY; ++k) {
			++slowTick;
			hostTickSet(slowTick);
			hostI2cPoll(ix);
			pump(4);
		}
		/* let the model finish what the last register access started (a requested stop, the end of the last byte) */
		slowTick += 10;
		hostTickSet(slowTick);
		hostI2cPoll(ix);
	}
	uint32_t slowTick = 1000;
	std::string log() const { return seqLog(ix); }
	void clearLog() { hostI2cLogClear(ix); }
	I2C_TypeDef *inst() const { return h.Instance; }

	/* target side */
	void listen() { REQUIRE(HAL_I2C_EnableListen_IT(&h) == HAL_OK); }
	/* the external controller addresses us (bus byte 0x30 = write, 0x31 = read): the address interrupt runs once, the ADDR flag
	 * stays set until the sequential call clears it */
	bool addressed(uint8_t byte) {
		const int r = hostI2cExtStart(ix, byte);
		pump(1);
		return r == 1;
	}
	int extWrite(uint8_t b) {
		const int r = hostI2cExtWrite(ix, b);
		pump(6);
		return r;
	}
	int extRead(int masterAck, uint8_t *b) {
		const int r = hostI2cExtReadByte(ix, masterAck, b);
		pump(6);
		return r;
	}
	void extStop() {
		hostI2cExtStop(ix);
		pump(6);
	}

	~SeqRig() {
		CHECK(hostI2cSequenceBreaks(ix) == 0u);
		CHECK(HalFixture::faults() == 0u);
	}
};

/* expected log of a receive frame of n bytes starting with byte `from`: Ixx..., the last one not acknowledged */
std::string seqRxBytes(unsigned from, unsigned n, bool nackLast) {
	std::string s;
	char b[16];
	for (unsigned k = 0; k < n; ++k) {
		if (!s.empty()) s += ' ';
		std::snprintf(b, sizeof b, "I%02X%s", from + k, (nackLast && k + 1 == n) ? "!" : "");
		s += b;
	}
	return s;
}

std::string seqTail(const std::string &s, unsigned n) { return s.size() >= n ? s.substr(s.size() - n) : s; }

} // namespace

/* ---------------------------------------------------------------- IQ1: each option, master transmit */

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit I2C_FIRST_FRAME sends start, address and data and no stop") {
	CHECK(tx(I2C_FIRST_FRAME, 2) == HAL_OK);
	/* UM1725: FIRST_FRAME = start condition, address and data without a final stop condition. */
	CHECK(log() == "S OA0 OA1 OA2");
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit I2C_FIRST_AND_NEXT_FRAME sends start, address and data and no stop") {
	CHECK(tx(I2C_FIRST_AND_NEXT_FRAME, 3) == HAL_OK);
	CHECK(log() == "S OA0 OA1 OA2 OA3");
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK((I2C1->CR1 & I2C_CR1_STOP) == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit I2C_FIRST_AND_LAST_FRAME is start, address, data, stop") {
	CHECK(tx(I2C_FIRST_AND_LAST_FRAME, 2) == HAL_OK);
	/* UM1725: "No sequential usage, functional is same as associated interfaces in no sequential mode". */
	CHECK(log() == "S OA0 OA1 OA2 P");
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit I2C_LAST_FRAME ends the sequence with a stop") {
	CHECK(tx(I2C_FIRST_FRAME, 1) == HAL_OK);
	clearLog();
	CHECK(tx(I2C_LAST_FRAME, 2) == HAL_OK);
	/* same direction: only the new data, then the final stop */
	CHECK(log() == "OA1 OA2 P");
	CHECK(i2cCounts.masterTx == 2u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit I2C_LAST_FRAME_NO_STOP ends the series without a stop") {
	CHECK(tx(I2C_FIRST_AND_NEXT_FRAME, 1) == HAL_OK);
	clearLog();
	CHECK(tx(I2C_LAST_FRAME_NO_STOP, 2) == HAL_OK);
	CHECK(log() == "OA1 OA2");
	CHECK(i2cCounts.masterTx == 2u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit I2C_NEXT_FRAME continues the same direction with data only") {
	CHECK(tx(I2C_FIRST_AND_NEXT_FRAME, 1) == HAL_OK);
	clearLog();
	CHECK(tx(I2C_NEXT_FRAME, 3) == HAL_OK);
	CHECK(log() == "OA1 OA2 OA3");
	CHECK_FALSE(seqHasStop(0));
	CHECK(i2cCounts.masterTx == 2u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit sizes of the I2C_FIRST_AND_LAST_FRAME frame: 0 1 2 3 5") {
	const char *expect[] = {"S OA0 P", "S OA0 OA1 P", "S OA0 OA1 OA2 P", "S OA0 OA1 OA2 OA3 P", "S OA0 OA1 OA2 OA3 OA4 OA5 P"};
	const unsigned sizes[] = {0, 1, 2, 3, 5};
	for (unsigned k = 0; k < 5; ++k) {
		reinit(0);
		INFO("size " << sizes[k]);
		CHECK(tx(I2C_FIRST_AND_LAST_FRAME, (uint16_t)sizes[k]) == HAL_OK);
		CHECK(log() == expect[k]);
		CHECK(i2cCounts.masterTx == 1u);
		CHECK(h.State == HAL_I2C_STATE_READY);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit of size 0 with I2C_FIRST_FRAME sends the address only and no stop") {
	CHECK(tx(I2C_FIRST_FRAME, 0) == HAL_OK);
	CHECK(log() == "S OA0");
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit call starts the transfer: busy transmit state, master mode, start generated, interrupts on") {
	const HAL_StatusTypeDef r = HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 2, I2C_FIRST_FRAME);
	CHECK(r == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(h.Mode == HAL_I2C_MODE_MASTER);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.XferOptions == I2C_FIRST_FRAME);
	CHECK(h.XferSize == 2u);
	CHECK(h.XferCount == 2u);
	CHECK(h.Devaddress == 0xA0u);
	CHECK(h.Lock == HAL_UNLOCKED);
	/* the start condition was generated: BUSY, MSL (RM0090 27.3.3 EV5) */
	CHECK((I2C1->SR2 & (I2C_SR2_BUSY | I2C_SR2_MSL)) == (I2C_SR2_BUSY | I2C_SR2_MSL));
	CHECK((I2C1->CR2 & SEQ_EVT_BUF_ERR) == SEQ_EVT_BUF_ERR);
	pump();
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit enables the peripheral if it was disabled and clears POS") {
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_POS;
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 1, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	CHECK((I2C1->CR1 & I2C_CR1_PE) != 0u);
	CHECK((I2C1->CR1 & I2C_CR1_POS) == 0u);
	pump();
	CHECK(log() == "S OA0 OA1 P");
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive enables the peripheral if it was disabled") {
	feed(4);
	I2C1->CR1 &= ~I2C_CR1_PE;
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 1, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	CHECK((I2C1->CR1 & I2C_CR1_PE) != 0u);
	pump();
	CHECK(log() == "S OA1 I01! P");
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit frame end disables all three interrupt enables and calls back once") {
	CHECK(tx(I2C_FIRST_FRAME, 2) == HAL_OK);
	CHECK((I2C1->CR2 & SEQ_EVT_BUF_ERR) == 0u);
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(i2cCounts.stateAtLast == HAL_I2C_STATE_READY);
	CHECK(i2cCounts.modeAtLast == HAL_I2C_MODE_NONE);
	CHECK(i2cCounts.last == &h);
	CHECK(i2cCounts.error == 0u);
	CHECK(i2cCounts.orderCount == 1u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit works on I2C1, I2C2 and I2C3") {
	for (unsigned i = 0; i < 3; ++i) {
		reinit(i);
		INFO("instance " << i);
		CHECK(tx(I2C_FIRST_AND_LAST_FRAME, 2) == HAL_OK);
		CHECK(log() == "S OA0 OA1 OA2 P");
		CHECK(i2cCounts.masterTx == 1u);
		CHECK(h.State == HAL_I2C_STATE_READY);
	}
}

/* ---------------------------------------------------------------- IQ1: each option, master receive */

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive I2C_FIRST_AND_LAST_FRAME of 1 2 3 4 5 bytes: data in order, nack on the last byte only, then stop") {
	for (unsigned n = 1; n <= 5; ++n) {
		reinit(0);
		feed(10);
		INFO("size " << n);
		CHECK(rxf(I2C_FIRST_AND_LAST_FRAME, (uint16_t)n) == HAL_OK);
		/* RM0090 27.3.3: the controller NACKs the last received byte, then generates the stop */
		CHECK(log() == "S OA1 " + seqRxBytes(1, n, true) + " P");
		for (unsigned k = 0; k < n; ++k) CHECK(rx[k] == k + 1);
		CHECK(rx[n] == 0);
		CHECK(i2cCounts.masterRx == 1u);
		CHECK(h.State == HAL_I2C_STATE_READY);
		CHECK(h.Mode == HAL_I2C_MODE_NONE);
		CHECK(h.XferCount == 0u);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive I2C_FIRST_FRAME of 1 2 3 4 5 bytes: nack on the last byte and no stop") {
	for (unsigned n = 1; n <= 5; ++n) {
		reinit(0);
		feed(10);
		INFO("size " << n);
		CHECK(rxf(I2C_FIRST_FRAME, (uint16_t)n) == HAL_OK);
		/* UM1725: FIRST_FRAME = start, address and data without a final stop */
		CHECK(log() == "S OA1 " + seqRxBytes(1, n, true));
		for (unsigned k = 0; k < n; ++k) CHECK(rx[k] == k + 1);
		CHECK(i2cCounts.masterRx == 1u);
		CHECK(h.State == HAL_I2C_STATE_READY);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive I2C_LAST_FRAME as a first call behaves like a single frame with stop") {
	/* characterization: NEEDS REVIEW 324 (the first call of a sequence with a LAST_FRAME option) */
	for (unsigned n = 1; n <= 5; ++n) {
		reinit(0);
		feed(10);
		INFO("size " << n);
		CHECK(rxf(I2C_LAST_FRAME, (uint16_t)n) == HAL_OK);
		CHECK(log() == "S OA1 " + seqRxBytes(1, n, true) + " P");
		CHECK(i2cCounts.masterRx == 1u);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive I2C_FIRST_AND_NEXT_FRAME of 2 3 4 5 bytes: data in order, ack kept on, no stop") {
	for (unsigned n = 2; n <= 5; ++n) {
		reinit(0);
		feed(16);
		INFO("size " << n);
		CHECK(rxf(I2C_FIRST_AND_NEXT_FRAME, (uint16_t)n) == HAL_OK);
		for (unsigned k = 0; k < n; ++k) CHECK(rx[k] == k + 1);
		CHECK(rx[n] == 0);
		CHECK_FALSE(seqHasStop(0));
		/* ACK stays on so that the next frame continues without a restart: no byte of the first frame is NACKed */
		CHECK(log().find('!') == std::string::npos);
		CHECK((I2C1->CR1 & I2C_CR1_ACK) != 0u);
		CHECK(i2cCounts.masterRx == 1u);
		CHECK(h.State == HAL_I2C_STATE_READY);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive call starts the transfer: busy receive state, ack on, start generated") {
	feed(4);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 3, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_RX);
	CHECK(h.Mode == HAL_I2C_MODE_MASTER);
	CHECK(h.XferOptions == I2C_FIRST_AND_LAST_FRAME);
	CHECK(h.XferCount == 3u);
	CHECK((I2C1->CR1 & I2C_CR1_ACK) != 0u);
	CHECK((I2C1->CR1 & I2C_CR1_POS) == 0u);
	CHECK((I2C1->SR2 & I2C_SR2_BUSY) != 0u);
	CHECK((I2C1->CR2 & SEQ_EVT_BUF_ERR) == SEQ_EVT_BUF_ERR);
	pump();
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive works on I2C1, I2C2 and I2C3") {
	for (unsigned i = 0; i < 3; ++i) {
		reinit(i);
		feed(4);
		INFO("instance " << i);
		CHECK(rxf(I2C_FIRST_AND_LAST_FRAME, 3) == HAL_OK);
		CHECK(log() == "S OA1 I01 I02 I03! P");
		CHECK(rx[0] == 1);
		CHECK(rx[2] == 3);
		CHECK(i2cCounts.masterRx == 1u);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive frame end disables the event and error interrupts") {
	feed(4);
	CHECK(rxf(I2C_FIRST_FRAME, 2) == HAL_OK);
	CHECK((I2C1->CR2 & (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN)) == 0u);
	CHECK(i2cCounts.stateAtLast == HAL_I2C_STATE_READY);
	CHECK(i2cCounts.modeAtLast == HAL_I2C_MODE_NONE);
	CHECK(i2cCounts.orderCount == 1u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 characterization: a 1 byte receive with I2C_FIRST_AND_NEXT_FRAME does not complete and keeps reading") {
	/* characterization: NEEDS REVIEW 324 (first proposed as a vendor bug, withdrawn). UM1725 only says "At reception end of current frame transfer,
	 * HAL_I2C_MasterRxCpltCallback() is executed" for the interface and describes FIRST_AND_NEXT_FRAME as "transfer without a
	 * final stop condition ... permit a call the same master sequential interface several times"; neither sentence promises a
	 * 1 byte frame with ACK kept on (the controller keeps clocking while ACK = 1, RM0090 27.3.3), so the observed behaviour
	 * is recorded, not judged: no callback, state BUSY_RX, and bytes beyond Size are stored (stm32f4xx_hal_i2c.c line 5548 and
	 * the BTF handler else branch line 5753). */
	feed(16);
	CHECK(rxf(I2C_FIRST_AND_NEXT_FRAME, 1) == HAL_OK);
	CHECK(i2cCounts.masterRx == 0u);
	CHECK(h.State == HAL_I2C_STATE_BUSY_RX);
	CHECK(rx[1] == 2);
}

/* ---------------------------------------------------------------- IQ1: OTHER options and I2C_ConvertOtherXferOptions */

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit I2C_OTHER_FRAME after a frame restarts with the address and no stop") {
	CHECK(tx(I2C_FIRST_FRAME, 1) == HAL_OK);
	clearLog();
	CHECK(tx(I2C_OTHER_FRAME, 1, txb + 1) == HAL_OK);
	/* UM1725: OTHER_FRAME = a restart condition after each call of the same interface */
	CHECK(log() == "R OA0 OA2");
	CHECK(i2cCounts.masterTx == 2u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit I2C_OTHER_AND_LAST_FRAME restarts and ends with a stop") {
	CHECK(tx(I2C_FIRST_FRAME, 1) == HAL_OK);
	clearLog();
	CHECK(tx(I2C_OTHER_AND_LAST_FRAME, 1, txb + 1) == HAL_OK);
	CHECK(log() == "R OA0 OA2 P");
	CHECK(i2cCounts.masterTx == 2u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive I2C_OTHER_FRAME restarts with the read address and no stop") {
	feed(8);
	CHECK(rxf(I2C_FIRST_FRAME, 1) == HAL_OK);
	clearLog();
	CHECK(rxf(I2C_OTHER_FRAME, 1, rx + 1) == HAL_OK);
	CHECK(log() == "R OA1 I02!");
	CHECK(rx[0] == 1);
	CHECK(rx[1] == 2);
	CHECK(i2cCounts.masterRx == 2u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive I2C_OTHER_AND_LAST_FRAME of 1 and of 2 bytes restarts and ends with a stop") {
	const unsigned sizes[] = {1, 2};
	for (unsigned n : sizes) {
		reinit(0);
		feed(8);
		INFO("size " << n);
		CHECK(rxf(I2C_FIRST_FRAME, 1) == HAL_OK);
		clearLog();
		CHECK(rxf(I2C_OTHER_AND_LAST_FRAME, (uint16_t)n, rx + 1) == HAL_OK);
		CHECK(log() == "R OA1 " + seqRxBytes(2, n, true) + " P");
		CHECK(rx[1] == 2);
		CHECK(i2cCounts.masterRx == 2u);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 characterization: the OTHER options are converted to FIRST_FRAME and FIRST_AND_LAST_FRAME at the start condition") {
	/* characterization: NEEDS REVIEW 324. I2C_ConvertOtherXferOptions: OTHER_FRAME -> FIRST_FRAME, OTHER_AND_LAST_FRAME ->
	 * FIRST_AND_LAST_FRAME (the comments of the function, not UM1725, state it). */
	CHECK(tx(I2C_FIRST_FRAME, 1) == HAL_OK);
	CHECK(h.XferOptions == I2C_FIRST_FRAME);
	CHECK(tx(I2C_OTHER_FRAME, 1) == HAL_OK);
	CHECK(h.XferOptions == I2C_FIRST_FRAME);
	CHECK(tx(I2C_OTHER_AND_LAST_FRAME, 1) == HAL_OK);
	CHECK(h.XferOptions == I2C_FIRST_AND_LAST_FRAME);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 the options that are not OTHER options are left unchanged by the start handling") {
	const uint32_t opts[] = {I2C_FIRST_FRAME, I2C_FIRST_AND_NEXT_FRAME, I2C_FIRST_AND_LAST_FRAME};
	for (uint32_t o : opts) {
		reinit(0);
		INFO("option " << o);
		CHECK(tx(o, 1) == HAL_OK);
		CHECK(h.XferOptions == o);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 an OTHER_FRAME call waits for the start condition before the interrupt handler acts") {
	/* the event handler returns until SB is set when an OTHER option is requested (stm32f4xx_hal_i2c.c EV handler): with the
	 * bus stuck the start condition does not complete, so a handler call changes nothing; once the bus runs the frame goes on */
	CHECK(tx(I2C_FIRST_FRAME, 1) == HAL_OK);
	clearLog();
	hostI2cStuck(0, 1);
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 1, I2C_OTHER_FRAME) == HAL_OK);
	HAL_I2C_EV_IRQHandler(&h);
	HAL_I2C_EV_IRQHandler(&h);
	CHECK(i2cCounts.masterTx == 1u); /* still only the first frame */
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(h.XferOptions == I2C_OTHER_FRAME); /* not converted before SB */
	CHECK(h.XferCount == 1u);
	hostI2cStuck(0, 0);
	pump();
	CHECK(i2cCounts.masterTx == 2u);
	CHECK(log() == "R OA0 OA1");
}

/* ---------------------------------------------------------------- IQ2: sequences */

TEST_CASE_FIXTURE(SeqRig, "IQ2 transmit FIRST then NEXT then LAST: one start, no repeated start, one stop at the end") {
	CHECK(tx(I2C_FIRST_FRAME, 2) == HAL_OK);
	CHECK(tx(I2C_NEXT_FRAME, 2, txb + 2) == HAL_OK);
	CHECK(tx(I2C_LAST_FRAME, 1, txb + 4) == HAL_OK);
	CHECK(log() == "S OA0 OA1 OA2 OA3 OA4 OA5 P");
	CHECK(i2cCounts.masterTx == 3u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 transmit FIRST_AND_NEXT, NEXT, LAST_FRAME_NO_STOP keeps the bus: no stop in the log") {
	CHECK(tx(I2C_FIRST_AND_NEXT_FRAME, 1) == HAL_OK);
	CHECK(tx(I2C_NEXT_FRAME, 1, txb + 1) == HAL_OK);
	CHECK(tx(I2C_LAST_FRAME_NO_STOP, 1, txb + 2) == HAL_OK);
	CHECK(log() == "S OA0 OA1 OA2 OA3");
	CHECK_FALSE(seqHasStop(0));
	CHECK((I2C1->SR2 & I2C_SR2_BUSY) != 0u); /* the bus is still ours */
	CHECK(i2cCounts.masterTx == 3u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 after LAST_FRAME_NO_STOP the opposite interface continues with a restart and the read address") {
	/* UM1725: LAST_FRAME_NO_STOP permits calling the opposite interface without stopping, generating a restart */
	feed(8);
	CHECK(tx(I2C_FIRST_AND_NEXT_FRAME, 1) == HAL_OK);
	CHECK(tx(I2C_NEXT_FRAME, 1, txb + 1) == HAL_OK);
	CHECK(tx(I2C_LAST_FRAME_NO_STOP, 1, txb + 2) == HAL_OK);
	clearLog();
	CHECK(rxf(I2C_LAST_FRAME, 1) == HAL_OK);
	CHECK(log() == "R OA1 I01! P");
	CHECK(rx[0] == 1);
	CHECK(i2cCounts.masterRx == 1u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 transmit FIRST then receive LAST: the direction change restarts with the read address and ends with a stop") {
	feed(8);
	CHECK(tx(I2C_FIRST_FRAME, 1) == HAL_OK);
	clearLog();
	CHECK(rxf(I2C_LAST_FRAME, 2) == HAL_OK);
	CHECK(log() == "R OA1 I01 I02! P");
	CHECK(rx[0] == 1);
	CHECK(rx[1] == 2);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 receive FIRST then transmit LAST: the direction change restarts with the write address and ends with a stop") {
	feed(8);
	CHECK(rxf(I2C_FIRST_FRAME, 2) == HAL_OK);
	clearLog();
	CHECK(tx(I2C_LAST_FRAME, 2) == HAL_OK);
	CHECK(log() == "R OA0 OA1 OA2 P");
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(i2cCounts.masterRx == 1u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 receive FIRST_AND_NEXT, NEXT, LAST of 2 bytes each: data 1..6 in order, one start, one stop") {
	feed(16);
	CHECK(rxf(I2C_FIRST_AND_NEXT_FRAME, 2) == HAL_OK);
	CHECK(rxf(I2C_NEXT_FRAME, 2, rx + 2) == HAL_OK);
	CHECK(rxf(I2C_LAST_FRAME, 2, rx + 4) == HAL_OK);
	for (unsigned k = 0; k < 6; ++k) CHECK(rx[k] == k + 1);
	CHECK(i2cCounts.masterRx == 3u);
	const std::string l = log();
	CHECK(l.rfind("S OA1 I01 I02", 0) == 0);
	CHECK(l.find('R') == std::string::npos);
	CHECK(seqTail(l, 2) == " P"); /* the last two bytes were clocked in while ACK was on (byte pipeline), see lead decisions */
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 receive FIRST_AND_NEXT, NEXT, LAST of 3 bytes each: data 1..9 in order") {
	feed(16);
	CHECK(rxf(I2C_FIRST_AND_NEXT_FRAME, 3) == HAL_OK);
	CHECK(rxf(I2C_NEXT_FRAME, 3, rx + 3) == HAL_OK);
	CHECK(rxf(I2C_LAST_FRAME, 3, rx + 6) == HAL_OK);
	for (unsigned k = 0; k < 9; ++k) CHECK(rx[k] == k + 1);
	CHECK(i2cCounts.masterRx == 3u);
	CHECK(seqTail(log(), 6) == "I09! P");
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 receive FIRST_AND_NEXT of 5 then LAST of 5: data 1..10 in order") {
	feed(16);
	CHECK(rxf(I2C_FIRST_AND_NEXT_FRAME, 5) == HAL_OK);
	CHECK(rxf(I2C_LAST_FRAME, 5, rx + 5) == HAL_OK);
	for (unsigned k = 0; k < 10; ++k) CHECK(rx[k] == k + 1);
	CHECK(i2cCounts.masterRx == 2u);
	CHECK(seqTail(log(), 6) == "I0A! P");
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 receive FIRST_AND_NEXT of 2 then LAST of 3 and then LAST of 4 bytes: data in order") {
	const unsigned lastSizes[] = {3, 4};
	for (unsigned ls : lastSizes) {
		reinit(0);
		feed(16);
		INFO("last frame " << ls);
		CHECK(rxf(I2C_FIRST_AND_NEXT_FRAME, 2) == HAL_OK);
		CHECK(rxf(I2C_LAST_FRAME, (uint16_t)ls, rx + 2) == HAL_OK);
		for (unsigned k = 0; k < 2 + ls; ++k) CHECK(rx[k] == k + 1);
		CHECK(i2cCounts.masterRx == 2u);
		CHECK(seqHasStop(0));
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 receive FIRST_AND_NEXT of 4 then LAST_FRAME_NO_STOP of 2 then transmit LAST: restart, address, stop") {
	feed(16);
	CHECK(rxf(I2C_FIRST_AND_NEXT_FRAME, 4) == HAL_OK);
	CHECK(rxf(I2C_LAST_FRAME_NO_STOP, 2, rx + 4) == HAL_OK);
	for (unsigned k = 0; k < 6; ++k) CHECK(rx[k] == k + 1);
	CHECK_FALSE(seqHasStop(0));
	clearLog();
	CHECK(tx(I2C_LAST_FRAME, 1) == HAL_OK);
	CHECK(log() == "R OA0 OA1 P");
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 receive FIRST then OTHER then OTHER_AND_LAST of 1 byte: a restart with address for every byte") {
	feed(8);
	CHECK(rxf(I2C_FIRST_FRAME, 1) == HAL_OK);
	CHECK(rxf(I2C_OTHER_FRAME, 1, rx + 1) == HAL_OK);
	CHECK(rxf(I2C_OTHER_AND_LAST_FRAME, 1, rx + 2) == HAL_OK);
	CHECK(log() == "S OA1 I01! R OA1 I02! R OA1 I03! P");
	CHECK(rx[0] == 1);
	CHECK(rx[1] == 2);
	CHECK(rx[2] == 3);
	CHECK(i2cCounts.masterRx == 3u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 receive FIRST then OTHER then OTHER_AND_LAST of 2 bytes: a restart with address for every frame") {
	feed(8);
	CHECK(rxf(I2C_FIRST_FRAME, 2) == HAL_OK);
	CHECK(rxf(I2C_OTHER_FRAME, 2, rx + 2) == HAL_OK);
	CHECK(rxf(I2C_OTHER_AND_LAST_FRAME, 2, rx + 4) == HAL_OK);
	CHECK(log() == "S OA1 I01 I02! R OA1 I03 I04! R OA1 I05 I06! P");
	for (unsigned k = 0; k < 6; ++k) CHECK(rx[k] == k + 1);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 transmit FIRST then OTHER then OTHER_AND_LAST: a restart with address for every frame, one stop") {
	CHECK(tx(I2C_FIRST_FRAME, 1) == HAL_OK);
	CHECK(tx(I2C_OTHER_FRAME, 1, txb + 1) == HAL_OK);
	CHECK(tx(I2C_OTHER_AND_LAST_FRAME, 1, txb + 2) == HAL_OK);
	CHECK(log() == "S OA0 OA1 R OA0 OA2 R OA0 OA3 P");
	CHECK(i2cCounts.masterTx == 3u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 a new sequence after a LAST_FRAME starts with a start condition, not a restart") {
	CHECK(tx(I2C_FIRST_FRAME, 1) == HAL_OK);
	CHECK(tx(I2C_LAST_FRAME, 1, txb + 1) == HAL_OK);
	clearLog();
	CHECK(tx(I2C_FIRST_AND_LAST_FRAME, 1, txb + 2) == HAL_OK);
	CHECK(log() == "S OA0 OA3 P");
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 a receive LAST_FRAME of 2 bytes after same direction frames ends with a stop and ACK off") {
	feed(16);
	CHECK(rxf(I2C_FIRST_AND_NEXT_FRAME, 2) == HAL_OK);
	CHECK(rxf(I2C_NEXT_FRAME, 2, rx + 2) == HAL_OK);
	CHECK(rxf(I2C_LAST_FRAME, 2, rx + 4) == HAL_OK);
	/* RM0090 27.3.3 two byte reception: ACK off (and POS on) before the last two bytes; the frame ends with the stop */
	CHECK(seqHasStop(0));
	CHECK((I2C1->CR1 & I2C_CR1_ACK) == 0u);
	for (unsigned k = 0; k < 6; ++k) CHECK(rx[k] == k + 1);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 sequences work on I2C2 and I2C3") {
	for (unsigned i = 1; i < 3; ++i) {
		reinit(i);
		feed(8);
		INFO("instance " << i);
		CHECK(tx(I2C_FIRST_FRAME, 1) == HAL_OK);
		CHECK(rxf(I2C_LAST_FRAME, 2) == HAL_OK);
		CHECK(log() == "S OA0 OA1 R OA1 I01 I02! P");
		CHECK(rx[1] == 2);
	}
}

/* ---------------------------------------------------------------- IQ3: slave sequential */

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave receive FIRST then LAST frame delivers the bytes and ends the listen phase at the stop") {
	listen();
	REQUIRE(addressed(0x30));
	CHECK(i2cCounts.addr == 1u);
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 2, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(extWrite(0x11) == 1);
	CHECK(extWrite(0x22) == 1);
	CHECK(i2cCounts.slaveRx == 1u);
	/* after a frame the handle listens again (UM1725: the listen phase continues until the stop) */
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
	CHECK(i2cCounts.listen == 0u);
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx + 2, 2, I2C_LAST_FRAME) == HAL_OK);
	CHECK(extWrite(0x33) == 1);
	CHECK(extWrite(0x44) == 1);
	CHECK(i2cCounts.slaveRx == 2u);
	extStop();
	CHECK(rx[0] == 0x11);
	CHECK(rx[1] == 0x22);
	CHECK(rx[2] == 0x33);
	CHECK(rx[3] == 0x44);
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave receive call enters the busy listen state, stores the option and clears the ADDR flag") {
	listen();
	REQUIRE(addressed(0x30));
	CHECK((I2C1->SR1 & I2C_SR1_ADDR) != 0u);
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 3, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_RX_LISTEN);
	CHECK(h.Mode == HAL_I2C_MODE_SLAVE);
	CHECK(h.XferOptions == I2C_FIRST_FRAME);
	CHECK(h.XferSize == 3u);
	CHECK(h.XferCount == 3u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	/* the ADDR flag is cleared by the call (RM0090: read SR1 then SR2) and the interrupts are on */
	CHECK((I2C1->SR1 & I2C_SR1_ADDR) == 0u);
	CHECK((I2C1->CR2 & SEQ_EVT_BUF_ERR) == SEQ_EVT_BUF_ERR);
	CHECK((I2C1->CR1 & I2C_CR1_POS) == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave receive keeps ACK on between frames and clears it at the stop") {
	listen();
	REQUIRE(addressed(0x30));
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 1, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(extWrite(0x61) == 1);
	CHECK(i2cCounts.slaveRx == 1u);
	CHECK((I2C1->CR1 & I2C_CR1_ACK) != 0u);
	extStop();
	CHECK((I2C1->CR1 & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave receive FIRST_AND_LAST frame completes, the stop then ends the listen phase") {
	listen();
	REQUIRE(addressed(0x30));
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 2, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	CHECK(extWrite(0x55) == 1);
	CHECK(extWrite(0x56) == 1);
	CHECK(i2cCounts.slaveRx == 1u);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
	extStop();
	CHECK(i2cCounts.slaveRx == 1u);
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.XferOptions == SEQ_NO_OPTION_FRAME);
	CHECK(rx[0] == 0x55);
	CHECK(rx[1] == 0x56);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 a byte sent between two receive frames is delivered to the next frame") {
	listen();
	REQUIRE(addressed(0x30));
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 1, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(extWrite(0x71) == 1);
	CHECK(i2cCounts.slaveRx == 1u);
	/* no frame active: the controller's next byte is accepted into DR and held */
	CHECK(hostI2cExtWrite(0, 0x72) == 1);
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx + 1, 1, I2C_LAST_FRAME) == HAL_OK);
	pump(6);
	CHECK(i2cCounts.slaveRx == 2u);
	CHECK(rx[0] == 0x71);
	CHECK(rx[1] == 0x72);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 characterization: a stop before the receive frame is full ends the frame with an acknowledge failure error") {
	/* characterization: NEEDS REVIEW 324 */
	listen();
	REQUIRE(addressed(0x30));
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 2, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	CHECK(extWrite(0x55) == 1);
	extStop();
	CHECK(rx[0] == 0x55);
	CHECK((h.ErrorCode & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.slaveRx == 0u);
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave transmit FIRST then LAST frame sends the bytes and a nack of the last byte ends the listen phase") {
	listen();
	REQUIRE(addressed(0x31));
	/* the controller reads: I2C_DIRECTION_TRANSMIT (1) is "master transmits", so a read request is I2C_DIRECTION_RECEIVE (0) */
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_RECEIVE);
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 2, I2C_FIRST_FRAME) == HAL_OK);
	pump(6);
	uint8_t b = 0;
	CHECK(extRead(1, &b) == 1);
	CHECK(b == 0xA1);
	CHECK(extRead(1, &b) == 1);
	CHECK(b == 0xA2);
	CHECK(i2cCounts.slaveTx == 1u);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb + 2, 1, I2C_LAST_FRAME) == HAL_OK);
	pump(6);
	CHECK(extRead(0, &b) == 0); /* nack from the controller */
	CHECK(b == 0xA3);
	CHECK(i2cCounts.slaveTx == 2u);
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(i2cCounts.error == 0u);
	CHECK((I2C1->CR1 & I2C_CR1_ACK) == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave transmit FIRST_AND_LAST frame of 1 byte: the nack ends the listen phase") {
	listen();
	REQUIRE(addressed(0x31));
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 1, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	pump(6);
	uint8_t b = 0;
	CHECK(extRead(0, &b) == 0);
	CHECK(b == 0xA1);
	CHECK(i2cCounts.slaveTx == 1u);
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.XferOptions == SEQ_NO_OPTION_FRAME);
	CHECK(i2cCounts.error == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave transmit FIRST frame: a nack after the frame does not end the listen phase") {
	listen();
	REQUIRE(addressed(0x31));
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 1, I2C_FIRST_FRAME) == HAL_OK);
	pump(6);
	uint8_t b = 0;
	CHECK(extRead(0, &b) == 0);
	CHECK(b == 0xA1);
	CHECK(i2cCounts.slaveTx == 1u);
	/* UM1725: FIRST_FRAME is a sequence without final stop: the listen phase goes on */
	CHECK(i2cCounts.listen == 0u);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
	CHECK((I2C1->SR1 & I2C_SR1_AF) == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave transmit call enters the busy transmit listen state and stores the option") {
	listen();
	REQUIRE(addressed(0x31));
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 3, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX_LISTEN);
	CHECK(h.Mode == HAL_I2C_MODE_SLAVE);
	CHECK(h.XferOptions == I2C_FIRST_FRAME);
	CHECK(h.XferSize == 3u);
	CHECK(h.XferCount == 3u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((I2C1->SR1 & I2C_SR1_ADDR) == 0u);
	CHECK((I2C1->CR2 & SEQ_EVT_BUF_ERR) == SEQ_EVT_BUF_ERR);
	pump(6);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave transmit sends three bytes in order and the stop afterwards ends the listen phase") {
	listen();
	REQUIRE(addressed(0x31));
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 3, I2C_FIRST_FRAME) == HAL_OK);
	pump(6);
	uint8_t b[3] = {};
	CHECK(extRead(1, &b[0]) == 1);
	CHECK(extRead(1, &b[1]) == 1);
	CHECK(extRead(1, &b[2]) == 1);
	CHECK(b[0] == 0xA1);
	CHECK(b[1] == 0xA2);
	CHECK(b[2] == 0xA3);
	CHECK(i2cCounts.slaveTx == 1u);
	extStop();
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 the address callback runs again for a restart: write frame, then read frame, direction and match code per phase") {
	listen();
	REQUIRE(addressed(0x30));
	CHECK(i2cCounts.addr == 1u);
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_TRANSMIT); /* the controller writes */
	CHECK(i2cCounts.addrMatchCode == 0x30u);
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 1, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(extWrite(0x81) == 1);
	CHECK(i2cCounts.slaveRx == 1u);
	/* repeated start with the read address */
	CHECK(hostI2cExtRestart(0, 0x31) == 1);
	pump(1);
	CHECK(i2cCounts.addr == 2u);
	CHECK(i2cCounts.addrDirection == I2C_DIRECTION_RECEIVE); /* the controller reads */
	CHECK(i2cCounts.addrMatchCode == 0x30u);
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 1, I2C_LAST_FRAME) == HAL_OK);
	pump(6);
	uint8_t b = 0;
	CHECK(extRead(0, &b) == 0);
	CHECK(b == 0xA1);
	CHECK(rx[0] == 0x81);
	CHECK(i2cCounts.slaveTx == 1u);
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 the sequential slave functions return HAL_BUSY when the handle is not listening") {
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 1, I2C_FIRST_FRAME) == HAL_BUSY);
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 1, I2C_FIRST_FRAME) == HAL_BUSY);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 the sequential slave functions return HAL_ERROR for a null buffer or size 0 and change nothing") {
	listen();
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, nullptr, 1, I2C_FIRST_FRAME) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 0, I2C_FIRST_FRAME) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, nullptr, 1, I2C_FIRST_FRAME) == HAL_ERROR);
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 0, I2C_FIRST_FRAME) == HAL_ERROR);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 the slave sequential receive enables the peripheral when it was disabled") {
	listen();
	REQUIRE(addressed(0x30));
	I2C1->CR1 &= ~I2C_CR1_PE;
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 1, I2C_FIRST_FRAME) == HAL_OK);
	CHECK((I2C1->CR1 & I2C_CR1_PE) != 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 the slave sequential transmit enables the peripheral when it was disabled") {
	listen();
	REQUIRE(addressed(0x31));
	I2C1->CR1 &= ~I2C_CR1_PE;
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 1, I2C_FIRST_FRAME) == HAL_OK);
	CHECK((I2C1->CR1 & I2C_CR1_PE) != 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave sequences work on I2C2 and I2C3") {
	for (unsigned i = 1; i < 3; ++i) {
		reinit(i);
		INFO("instance " << i);
		listen();
		REQUIRE(addressed(0x30));
		CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 2, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
		CHECK(extWrite(0x91) == 1);
		CHECK(extWrite(0x92) == 1);
		extStop();
		CHECK(rx[0] == 0x91);
		CHECK(rx[1] == 0x92);
		CHECK(i2cCounts.slaveRx == 1u);
		CHECK(i2cCounts.listen == 1u);
	}
}

/* ---------------------------------------------------------------- IQ4: errors */

TEST_CASE_FIXTURE(SeqRig, "IQ4 master transmit to an address that does not answer reports an acknowledge failure and stops") {
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA2, txb, 2, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	pump();
	CHECK(log() == "S OA2! P");
	CHECK((h.ErrorCode & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.masterTx == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(i2cCounts.errorCodeAtError == HAL_I2C_ERROR_AF);
	CHECK((I2C1->CR2 & SEQ_EVT_BUF_ERR) == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 master receive from an address that does not answer reports an acknowledge failure and stops") {
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA2, rx, 2, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	pump();
	CHECK(log() == "S OA3! P");
	CHECK((h.ErrorCode & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.masterRx == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK((I2C1->CR1 & I2C_CR1_POS) == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 a data byte that is not acknowledged ends a FIRST frame with an error and a stop") {
	hostI2cDeviceNackDataAt(0, dev, 1);
	CHECK(tx(I2C_FIRST_FRAME, 3) == HAL_OK);
	CHECK(log() == "S OA0 OA1 OA2! P");
	CHECK(h.ErrorCode == HAL_I2C_ERROR_AF);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.masterTx == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 after an error a new FIRST frame starts a fresh sequence") {
	hostI2cDeviceNackDataAt(0, dev, 0);
	CHECK(tx(I2C_FIRST_FRAME, 2) == HAL_OK);
	CHECK(i2cCounts.error == 1u);
	hostI2cDeviceNackDataAt(0, dev, -1);
	clearLog();
	CHECK(tx(I2C_FIRST_FRAME, 1) == HAL_OK);
	CHECK(log() == "S OA0 OA1");
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 characterization: a NEXT_FRAME after an error starts with a start condition") {
	/* characterization: NEEDS REVIEW 324 */
	hostI2cDeviceNackDataAt(0, dev, 0);
	CHECK(tx(I2C_FIRST_FRAME, 2) == HAL_OK);
	CHECK(i2cCounts.error == 1u);
	hostI2cDeviceNackDataAt(0, dev, -1);
	clearLog();
	CHECK(tx(I2C_NEXT_FRAME, 1) == HAL_OK);
	CHECK(log() == "S OA0 OA1");
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 characterization: a NEXT_FRAME as the first call of a sequence starts with a start condition") {
	/* characterization: NEEDS REVIEW 324 */
	CHECK(tx(I2C_NEXT_FRAME, 1) == HAL_OK);
	CHECK(log() == "S OA0 OA1");
	CHECK(i2cCounts.masterTx == 1u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 characterization: a LAST_FRAME as the first call of a transmit sequence is start, address, data, stop") {
	/* characterization: NEEDS REVIEW 324 */
	CHECK(tx(I2C_LAST_FRAME, 2) == HAL_OK);
	CHECK(log() == "S OA0 OA1 OA2 P");
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 a call while a frame is running returns HAL_BUSY and changes nothing") {
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 2, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 2, I2C_NEXT_FRAME) == HAL_BUSY);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 2, I2C_NEXT_FRAME) == HAL_BUSY);
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(h.XferOptions == I2C_FIRST_FRAME);
	CHECK(h.ErrorCode == HAL_I2C_ERROR_NONE);
	pump();
	CHECK(i2cCounts.masterTx == 1u);
	CHECK(log() == "S OA0 OA1 OA2");
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 a call while a receive frame is running returns HAL_BUSY") {
	feed(8);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 3, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx + 8, 3, I2C_NEXT_FRAME) == HAL_BUSY);
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 1, I2C_FIRST_FRAME) == HAL_BUSY);
	pump();
	CHECK(i2cCounts.masterRx == 1u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 a first frame on a bus held by another controller returns HAL_BUSY with a timeout error code") {
	hostI2cBusBusy(0, 1);
	const uint32_t firsts[] = {I2C_FIRST_FRAME, I2C_FIRST_AND_LAST_FRAME};
	for (uint32_t o : firsts) {
		h.ErrorCode = HAL_I2C_ERROR_NONE;
		INFO("option " << o);
		CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 1, o) == HAL_BUSY);
		CHECK((h.ErrorCode & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT);
		CHECK(h.State == HAL_I2C_STATE_READY);
		CHECK(h.Mode == HAL_I2C_MODE_NONE);
		h.ErrorCode = HAL_I2C_ERROR_NONE;
		CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 1, o) == HAL_BUSY);
		CHECK((h.ErrorCode & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT);
		CHECK(h.State == HAL_I2C_STATE_READY);
		CHECK(h.Mode == HAL_I2C_MODE_NONE);
	}
	CHECK(i2cCounts.masterTx == 0u);
	CHECK(i2cCounts.masterRx == 0u);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 a pending stop request makes even a LAST_FRAME wait for the bus and return HAL_BUSY") {
	/* the HAL waits for BUSY = 0 when CR1.STOP is still set (a stop that has not completed) or for a first frame */
	hostI2cBusBusy(0, 1);
	HalFixture::poke(&I2C1->CR1, HalFixture::peek(&I2C1->CR1) | I2C_CR1_STOP);
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 1, I2C_LAST_FRAME) == HAL_BUSY);
	CHECK((h.ErrorCode & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	h.ErrorCode = HAL_I2C_ERROR_NONE;
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 2, I2C_LAST_FRAME) == HAL_BUSY);
	CHECK((h.ErrorCode & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	HalFixture::poke(&I2C1->CR1, HalFixture::peek(&I2C1->CR1) & ~I2C_CR1_STOP);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 characterization: a LAST_FRAME call does not look at the BUSY flag") {
	/* characterization: NEEDS REVIEW 324. Only FIRST_FRAME, FIRST_AND_LAST_FRAME or a pending STOP make the call wait. */
	hostI2cBusBusy(0, 1);
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 1, I2C_LAST_FRAME) == HAL_OK);
	CHECK(h.State == HAL_I2C_STATE_BUSY_TX);
	CHECK(h.Mode == HAL_I2C_MODE_MASTER);
	hostI2cBusBusy(0, 0);
	pump();
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 LAST_FRAME_NO_STOP followed by FIRST_FRAME returns HAL_BUSY because the bus was not released") {
	CHECK(tx(I2C_FIRST_AND_NEXT_FRAME, 1) == HAL_OK);
	CHECK(tx(I2C_LAST_FRAME_NO_STOP, 1, txb + 1) == HAL_OK);
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 1, I2C_FIRST_FRAME) == HAL_BUSY);
	CHECK((h.ErrorCode & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(i2cCounts.masterTx == 2u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 a bus error during a FIRST frame ends it with HAL_I2C_ERROR_BERR and the error callback") {
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 4, I2C_FIRST_FRAME) == HAL_OK);
	pump(2);
	hostI2cRaiseSr1(0, I2C_SR1_BERR);
	pump();
	CHECK((h.ErrorCode & HAL_I2C_ERROR_BERR) == HAL_I2C_ERROR_BERR);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.masterTx == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK((I2C1->SR1 & I2C_SR1_BERR) == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 lost arbitration during a receive frame ends it with HAL_I2C_ERROR_ARLO and the error callback") {
	feed(8);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 3, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	pump(1);
	hostI2cArbitrationLost(0);
	pump();
	CHECK((h.ErrorCode & HAL_I2C_ERROR_ARLO) == HAL_I2C_ERROR_ARLO);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.masterRx == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK((I2C1->CR1 & I2C_CR1_POS) == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 characterization: SEQ_NO_OPTION_FRAME as a transmit option ends the frame with a stop") {
	/* characterization: NEEDS REVIEW 324 */
	CHECK(tx(SEQ_NO_OPTION_FRAME, 2) == HAL_OK);
	CHECK(log() == "S OA0 OA1 OA2 P");
	CHECK(i2cCounts.masterTx == 1u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 characterization: SEQ_NO_OPTION_FRAME as a receive option of 1 byte nacks the byte and ends with a stop") {
	/* characterization: NEEDS REVIEW 324 */
	feed(4);
	CHECK(rxf(SEQ_NO_OPTION_FRAME, 1) == HAL_OK);
	CHECK(log() == "S OA1 I01! P");
	CHECK(rx[0] == 1);
	CHECK(i2cCounts.masterRx == 1u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 characterization: a receive of size 0 reads one byte and ends with a stop") {
	/* characterization: NEEDS REVIEW 324 */
	feed(4);
	CHECK(rxf(I2C_FIRST_AND_LAST_FRAME, 0) == HAL_OK);
	CHECK(seqHasStop(0));
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 a receive FIRST_FRAME of 1 byte leaves ACK off and POS clear") {
	feed(4);
	CHECK(rxf(I2C_FIRST_FRAME, 1) == HAL_OK);
	CHECK((I2C1->CR1 & I2C_CR1_ACK) == 0u);
	CHECK((I2C1->CR1 & I2C_CR1_POS) == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 KNOWN_ISSUE a 1 byte LAST_FRAME after a 2 byte FIRST_AND_NEXT_FRAME completes the sequence" * KNOWN_ISSUE("V108 HAL_I2C_Master_Seq_Receive_IT: a Size 1 LAST_FRAME that follows a same direction FIRST_AND_NEXT_FRAME never completes: UM1725 option text 'I2C_LAST_FRAME: ... manage only the new data to transfer if no direction change and with a final stop condition' and 'At reception end of current frame transfer, HAL_I2C_MasterRxCpltCallback() is executed'")) {
	feed(16);
	CHECK(rxf(I2C_FIRST_AND_NEXT_FRAME, 2) == HAL_OK);
	CHECK(rxf(I2C_LAST_FRAME, 1, rx + 2) == HAL_OK);
	/* first failing assertion: no Rx callback for the second frame (BTF handler else branch, stm32f4xx_hal_i2c.c line 5753,
	 * keeps reading) */
	CHECK(i2cCounts.masterRx == 2u);
	CHECK(rx[2] == 3);
	CHECK(rx[3] == 0);
	CHECK(seqHasStop(0));
}


/* ---------------------------------------------------------------- IQ1/IQ2 with a byte time on the bus (TXE / RXNE visible before BTF) */

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit with a byte time: TXE then BTF carry the frame, one callback, same bus log") {
	const unsigned sizes[] = {1, 2, 3, 5};
	for (unsigned n : sizes) {
		reinit(0);
		INFO("size " << n);
		hostI2cSetByteTicks(3);
		CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, (uint16_t)n, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
		slowRun();
		std::string expect = "S OA0";
		for (unsigned k = 0; k < n; ++k) {
			char b[8];
			std::snprintf(b, sizeof b, " O%02X", 0xA1 + k);
			expect += b;
		}
		CHECK(log() == expect + " P");
		CHECK(i2cCounts.masterTx == 1u);
		CHECK(h.State == HAL_I2C_STATE_READY);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master transmit I2C_FIRST_FRAME with a byte time ends without a stop") {
	hostI2cSetByteTicks(3);
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 3, I2C_FIRST_FRAME) == HAL_OK);
	slowRun();
	CHECK(log() == "S OA0 OA1 OA2 OA3");
	CHECK(i2cCounts.masterTx == 1u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive with a byte time of 1 to 6 bytes: data in order, nack on the last byte only, then stop") {
	for (unsigned n = 1; n <= 6; ++n) {
		reinit(0);
		feed(10);
		INFO("size " << n);
		hostI2cSetByteTicks(3);
		CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, (uint16_t)n, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
		slowRun();
		CHECK(log() == "S OA1 " + seqRxBytes(1, n, true) + " P");
		for (unsigned k = 0; k < n; ++k) CHECK(rx[k] == k + 1);
		CHECK(i2cCounts.masterRx == 1u);
		CHECK(h.State == HAL_I2C_STATE_READY);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master receive I2C_FIRST_FRAME with a byte time of 1 to 6 bytes: nack on the last byte and no stop") {
	for (unsigned n = 1; n <= 6; ++n) {
		reinit(0);
		feed(10);
		INFO("size " << n);
		hostI2cSetByteTicks(3);
		CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, (uint16_t)n, I2C_FIRST_FRAME) == HAL_OK);
		slowRun();
		CHECK(log() == "S OA1 " + seqRxBytes(1, n, true));
		for (unsigned k = 0; k < n; ++k) CHECK(rx[k] == k + 1);
		CHECK(i2cCounts.masterRx == 1u);
	}
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 receive FIRST_AND_NEXT of 5 and LAST of 5 with a byte time: data 1..10 in order, one stop") {
	feed(16);
	hostI2cSetByteTicks(3);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 5, I2C_FIRST_AND_NEXT_FRAME) == HAL_OK);
	slowRun();
	CHECK(i2cCounts.masterRx == 1u);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx + 5, 5, I2C_LAST_FRAME) == HAL_OK);
	slowRun();
	CHECK(i2cCounts.masterRx == 2u);
	for (unsigned k = 0; k < 10; ++k) CHECK(rx[k] == k + 1);
	CHECK(seqHasStop(0));
	CHECK(log().find('R') == std::string::npos);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 transmit FIRST then LAST with a byte time: one start, data in order, one stop") {
	hostI2cSetByteTicks(3);
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 2, I2C_FIRST_FRAME) == HAL_OK);
	slowRun();
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb + 2, 2, I2C_LAST_FRAME) == HAL_OK);
	slowRun();
	CHECK(log() == "S OA0 OA1 OA2 OA3 OA4 P");
	CHECK(i2cCounts.masterTx == 2u);
}

/* ---------------------------------------------------------------- IQ3: slave frames that meet a waiting controller */

TEST_CASE_FIXTURE(SeqRig, "IQ3 KNOWN_ISSUE a 1 byte slave transmit frame that starts while the controller waits for data reports its completion" * KNOWN_ISSUE("V109 HAL_I2C_Slave_Seq_Transmit_IT: a Size 1 frame started while the controller already waits for the next byte (BTF set, clock stretched) is sent by I2C_SlaveTransmit_BTF, which has no completion handling: no SlaveTxCpltCallback, the state stays BUSY_TX_LISTEN (UM1725, sequential interfaces list for HAL_I2C_Slave_Seq_Transmit_IT: 'At transmission end of current frame transfer, HAL_I2C_SlaveTxCpltCallback() is executed')")) {
	listen();
	REQUIRE(addressed(0x31));
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 1, I2C_FIRST_FRAME) == HAL_OK);
	pump(6);
	uint8_t b = 0;
	CHECK(extRead(1, &b) == 1);
	CHECK(i2cCounts.slaveTx == 1u);
	CHECK(hostI2cExtReadByte(0, 1, &b) == -1); /* nothing to send: stretched */
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb + 1, 1, I2C_LAST_FRAME) == HAL_OK);
	pump(6);
	CHECK(extRead(0, &b) == 0);
	CHECK(b == 0xA2); /* the byte of the second frame went out */
	/* first failing assertion: the frame never completes (stm32f4xx_hal_i2c.c I2C_SlaveTransmit_BTF line 6038) */
	CHECK(i2cCounts.slaveTx == 2u);
	CHECK(h.State != HAL_I2C_STATE_BUSY_TX_LISTEN);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 a multi byte slave transmit frame that starts while the controller waits sends all its bytes in order") {
	listen();
	REQUIRE(addressed(0x31));
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 1, I2C_FIRST_FRAME) == HAL_OK);
	pump(6);
	uint8_t b = 0;
	CHECK(extRead(1, &b) == 1);
	CHECK(hostI2cExtReadByte(0, 1, &b) == -1);
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb + 1, 3, I2C_FIRST_FRAME) == HAL_OK);
	pump(6);
	uint8_t got[3] = {};
	CHECK(extRead(1, &got[0]) == 1);
	CHECK(extRead(1, &got[1]) == 1);
	CHECK(extRead(1, &got[2]) == 1);
	CHECK(got[0] == 0xA2);
	CHECK(got[1] == 0xA3);
	CHECK(got[2] == 0xA4);
	CHECK(i2cCounts.slaveTx == 2u);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 characterization: bytes still in the data register at the stop are stored, then the short frame ends with an acknowledge failure") {
	/* characterization: NEEDS REVIEW 324. Two bytes were acknowledged (RM0090 27.3.2: a byte the target acked was received) and are
	 * waiting in DR and the shift register when the stop arrives with the frame not full (Size 3). */
	listen();
	REQUIRE(addressed(0x30));
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 3, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	CHECK(hostI2cExtWrite(0, 0x41) == 1);
	CHECK(hostI2cExtWrite(0, 0x42) == 1);
	CHECK(hostI2cExtWrite(0, 0x43) == -1); /* DR and shift register are full: the controller is stretched */
	hostI2cExtStop(0);
	pump(6);
	CHECK(rx[0] == 0x41);
	CHECK(rx[1] == 0x42);
	CHECK(rx[2] == 0);
	CHECK(h.XferCount == 1u);
	CHECK((h.ErrorCode & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.slaveRx == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 two bytes held between two receive frames are delivered to the next frame") {
	listen();
	REQUIRE(addressed(0x30));
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx, 1, I2C_FIRST_FRAME) == HAL_OK);
	CHECK(extWrite(0x71) == 1);
	CHECK(i2cCounts.slaveRx == 1u);
	/* no frame active: one byte waits in DR, the next one in the shift register (RM0090 27.3.2: BTF, clock stretched) */
	CHECK(hostI2cExtWrite(0, 0x72) == 1);
	CHECK(hostI2cExtWrite(0, 0x73) == 1);
	CHECK(hostI2cExtWrite(0, 0x74) == -1);
	CHECK(HAL_I2C_Slave_Seq_Receive_IT(&h, rx + 1, 2, I2C_LAST_FRAME) == HAL_OK);
	pump(6);
	CHECK(i2cCounts.slaveRx == 2u);
	CHECK(rx[1] == 0x72);
	CHECK(rx[2] == 0x73);
	CHECK(rx[3] == 0);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
}

TEST_CASE_FIXTURE(SeqRig, "IQ4 characterization: a stop request that does not complete ends a 1 byte receive with a timeout error") {
	/* characterization: NEEDS REVIEW 324. UM1725 names HAL_I2C_ERROR_TIMEOUT but not this path: the RXNE handler of the last byte waits
	 * for the hardware to clear CR1.STOP (5 ms); the stop request is kept set with a raw poke to model a stop that never ends. */
	feed(4);
	hostI2cSetByteTicks(3);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 1, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	for (unsigned guard = 0; guard < 200; ++guard) {
		++slowTick;
		hostTickSet(slowTick);
		hostI2cPoll(0);
		if ((HalFixture::peek(&I2C1->SR1) & I2C_SR1_RXNE) != 0u) break;
		if (hostI2cEventPending(0)) HAL_I2C_EV_IRQHandler(&h);
	}
	REQUIRE((HalFixture::peek(&I2C1->SR1) & I2C_SR1_RXNE) != 0u);
	HalFixture::poke(&I2C1->CR1, HalFixture::peek(&I2C1->CR1) | I2C_CR1_STOP);
	HAL_I2C_EV_IRQHandler(&h);
	CHECK((h.ErrorCode & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT);
	CHECK(i2cCounts.error == 1u);
	CHECK(i2cCounts.masterRx == 0u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.Mode == HAL_I2C_MODE_NONE);
	CHECK(rx[0] == 1); /* the received byte is still stored */
	HalFixture::poke(&I2C1->CR1, HalFixture::peek(&I2C1->CR1) & ~I2C_CR1_STOP);
}

/* ---------------------------------------------------------------- PreviousState: does the next NEXT_FRAME / LAST_FRAME restart? */
/* I2C_STATE_* private values from the macros: MSK = (0x21 | 0x22) & ~0x20 = 0x03; MASTER_BUSY_TX = 0x21 & 3 | MODE_MASTER (0x10) = 0x11;
 * MASTER_BUSY_RX = 0x12; SLAVE_BUSY_TX = 0x21 & 3 | MODE_SLAVE (0x20) = 0x21; NONE = 0. */
namespace {
const uint32_t SEQ_PS_NONE = 0x00u, SEQ_PS_MASTER_TX = 0x11u, SEQ_PS_MASTER_RX = 0x12u, SEQ_PS_SLAVE_TX = 0x21u;
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 a FIRST_FRAME of size 0 that ends in the TXE handler lets the next NEXT_FRAME continue without a start") {
	CHECK(tx(I2C_FIRST_FRAME, 0) == HAL_OK);
	CHECK(log() == "S OA0");
	clearLog();
	CHECK(tx(I2C_NEXT_FRAME, 2) == HAL_OK);
	CHECK(log() == "OA1 OA2");
	CHECK(i2cCounts.masterTx == 2u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 characterization: PreviousState after a TXE handler completion is master busy transmit") {
	/* characterization: NEEDS REVIEW 324 */
	CHECK(tx(I2C_FIRST_FRAME, 0) == HAL_OK);
	CHECK(h.PreviousState == SEQ_PS_MASTER_TX);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 a FIRST_AND_LAST_FRAME of 0 bytes that ends in the TXE handler makes the next NEXT_FRAME start again") {
	CHECK(tx(I2C_FIRST_AND_LAST_FRAME, 0) == HAL_OK);
	CHECK(h.PreviousState == SEQ_PS_NONE); /* NEEDS REVIEW 324 */
	clearLog();
	CHECK(tx(I2C_NEXT_FRAME, 1) == HAL_OK);
	CHECK(log() == "S OA0 OA1");
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 after Master_Abort_IT the next NEXT_FRAME starts a new transfer with a start condition") {
	CHECK(HAL_I2C_Master_Seq_Transmit_IT(&h, 0xA0, txb, 4, I2C_FIRST_FRAME) == HAL_OK);
	pump(2);
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0xA0) == HAL_OK);
	pump();
	CHECK(i2cCounts.abortCplt == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
	CHECK(h.PreviousState == SEQ_PS_NONE); /* NEEDS REVIEW 324 */
	CHECK(seqHasStop(0));
	clearLog();
	CHECK(tx(I2C_NEXT_FRAME, 1, txb + 4) == HAL_OK);
	CHECK(log() == "S OA0 OA5");
	CHECK(i2cCounts.masterTx == 1u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 after Master_Abort_IT the next LAST_FRAME receive starts with a start condition") {
	feed(8);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 4, I2C_FIRST_AND_NEXT_FRAME) == HAL_OK);
	pump(2);
	CHECK(HAL_I2C_Master_Abort_IT(&h, 0xA0) == HAL_OK);
	pump();
	CHECK(h.State == HAL_I2C_STATE_READY);
	hostI2cResetState();
	hostI2cDeviceAdd(0, 0x50, 0);
	feed(8);
	CHECK(rxf(I2C_LAST_FRAME, 2) == HAL_OK);
	CHECK(log() == "S OA1 I01 I02! P");
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 a one-shot FIRST_AND_LAST_FRAME receive of 2 and of 3 bytes makes the next NEXT_FRAME and LAST_FRAME start again") {
	const unsigned sizes[] = {2, 3};
	const uint32_t nexts[] = {I2C_NEXT_FRAME, I2C_LAST_FRAME};
	for (unsigned n : sizes)
		for (uint32_t o : nexts) {
			reinit(0);
			feed(12);
			INFO("size " << n << " next option " << o);
			CHECK(rxf(I2C_FIRST_AND_LAST_FRAME, (uint16_t)n) == HAL_OK); /* ends in the BTF handler (line 5738) */
			CHECK(h.PreviousState == SEQ_PS_NONE); /* NEEDS REVIEW 324 */
			clearLog();
			CHECK(rxf(o, 2, rx + 4) == HAL_OK);
			CHECK(log().rfind("S OA1", 0) == 0);
		}
}

TEST_CASE_FIXTURE(SeqRig, "IQ2 a FIRST_AND_NEXT_FRAME receive of 2 bytes that ends in the BTF handler lets the next frame continue without a start") {
	feed(12);
	CHECK(rxf(I2C_FIRST_AND_NEXT_FRAME, 2) == HAL_OK);
	CHECK(h.PreviousState == SEQ_PS_MASTER_RX); /* NEEDS REVIEW 324 */
	clearLog();
	CHECK(rxf(I2C_NEXT_FRAME, 2, rx + 2) == HAL_OK);
	CHECK(log().find('S') == std::string::npos);
	CHECK(log().find('R') == std::string::npos);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave transmit: a nack after a FIRST_AND_LAST frame ends the listen phase and PreviousState is cleared") {
	listen();
	REQUIRE(addressed(0x31));
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 1, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	pump(6);
	CHECK(h.PreviousState == SEQ_PS_SLAVE_TX); /* after the frame, before the nack: NEEDS REVIEW 324 */
	uint8_t b = 0;
	CHECK(extRead(0, &b) == 0);
	CHECK(h.PreviousState == SEQ_PS_NONE); /* NEEDS REVIEW 324 */
	CHECK(i2cCounts.listen == 1u);
	CHECK(h.State == HAL_I2C_STATE_READY);
}

TEST_CASE_FIXTURE(SeqRig, "IQ3 slave transmit: a nack after a FIRST_FRAME keeps the listen phase and the slave transmit history") {
	listen();
	REQUIRE(addressed(0x31));
	CHECK(HAL_I2C_Slave_Seq_Transmit_IT(&h, txb, 1, I2C_FIRST_FRAME) == HAL_OK);
	pump(6);
	uint8_t b = 0;
	CHECK(extRead(0, &b) == 0);
	CHECK(h.State == HAL_I2C_STATE_LISTEN);
	CHECK(h.PreviousState == SEQ_PS_SLAVE_TX); /* NEEDS REVIEW 324 */
	CHECK(i2cCounts.listen == 0u);
}

TEST_CASE_FIXTURE(SeqRig, "IQ1 master Seq receive of 1 and 2 bytes stores the frame size in XferSize") {
	feed(8);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 1, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	CHECK(h.XferSize == 1u);
	CHECK(h.XferCount == 1u);
	pump();
	reinit(0);
	feed(8);
	CHECK(HAL_I2C_Master_Seq_Receive_IT(&h, 0xA0, rx, 2, I2C_FIRST_AND_LAST_FRAME) == HAL_OK);
	CHECK(h.XferSize == 2u);
	CHECK(h.XferCount == 2u);
	pump();
}
