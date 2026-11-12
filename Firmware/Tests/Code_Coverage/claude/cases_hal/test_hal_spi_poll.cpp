/*
 * P12 T4a group SP: HAL_SPI_Transmit, HAL_SPI_Receive, HAL_SPI_TransmitReceive (polling, stm32f4xx_hal_spi.c) against the
 * SPI register model (fakes/stm32_hal/HalSpi.h). Requirements SP1-SP9 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T4A_REQUIREMENTS.md; compiled into ffb_hal_tests (F407VG) and
 * ffb_hal_tests_disco (F407VG_DISCO).
 *
 * Oracles: UM1725 Rev 8 chapter SPI (the API only names the functions, parameters and the HAL status; it gives no return code
 * for a failing call and no timeout behaviour: those cases are `characterization:`) and RM0090 Rev 22 chapter 28:
 *  - 28.3.5: full duplex master: SPE = 1, write DR, per item wait TXE / RXNE; transmit-only: after the last write wait TXE then
 *    BSY = 0; "After transmitting two data items in transmit-only mode, the OVR flag is set ... since the received data are never
 *    read" (so a HAL that returns must have cleared it, 28.3.10: read DR then read SR); receive-only master (RXONLY = 1, or
 *    BIDIMODE = 1 and BIDIOE = 0): the clock runs as soon as SPE = 1 until SPE = 0; bidirectional transmit needs BIDIMODE and
 *    BIDIOE set before the SPI is enabled, bidirectional receive BIDIOE clear.
 *  - 28.3.8: do not disable the SPI while BSY = 1 (model counter hostSpiDisabledBusy), except the receive-only master sequence
 *    (disable after the second to last RXNE, the last frame still completes).
 *  - 28.5.1: DFF bit 11 = 1 selects 16 bit frames; DFF, RXONLY, BR, MSTR, CPOL, CPHA ... must not change while SPE = 1
 *    (model counter hostSpiConfigWhileEnabled). 28.3.10: MODF, OVR sequences.
 * Time is the HAL tick (every HAL_GetTick() call advances it by 1, HalHostSystem.h); a frame lasts one tick by default.
 * Slave tests drive the master through the tick hook (hostTickSetStep).
 * Every file-local symbol is in the anonymous namespace or starts with spiPoll.
 *
 * Proposed NEEDS REVIEW entries are in the comments of the `characterization:` cases.
 */
#include "HalFixture.h"
#include "HalHostSystem.h"
#include "HalSpi.h"
#include "SpiTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <vector>

namespace {

constexpr uint32_t spiPollCR1_SPE = 1u << 6;
constexpr uint32_t spiPollCR1_DFF = 1u << 11;
constexpr uint32_t spiPollCR1_BIDIOE = 1u << 14;
constexpr uint32_t spiPollCR1_BIDIMODE = 1u << 15;
constexpr uint32_t spiPollCR1_MSTR = 1u << 2;
constexpr uint32_t spiPollSR_RXNE = 1u << 0;
constexpr uint32_t spiPollSR_TXE = 1u << 1;
constexpr uint32_t spiPollSR_MODF = 1u << 5;
constexpr uint32_t spiPollSR_OVR = 1u << 6;
constexpr uint32_t spiPollSR_BSY = 1u << 7;

using spiPollBytes = std::vector<uint8_t>;
using spiPollWordsT = std::vector<uint16_t>;

SPI_TypeDef *spiPollBase(unsigned i) {
	static SPI_TypeDef *const t[3] = {SPI1, SPI2, SPI3};
	return t[i];
}

/* A real read of SR: evaluates the frames that ended. A pending OVR is only cleared by the model if DR was read before (28.3.10),
 * the value returned is the one before the clearing. */
uint32_t spiPollSr(unsigned i) { return spiPollBase(i)->SR; }
uint32_t spiPollCr1(unsigned i) { return HalFixture::peek(&spiPollBase(i)->CR1); }

struct spiPollSize {
	bool wide;
	unsigned n;
};
const spiPollSize spiPollSizes[] = {{false, 1}, {false, 2}, {false, 3}, {false, 16}, {true, 1}, {true, 5}};

spiPollWordsT spiPollWords(unsigned n, bool wide, unsigned seed) {
	spiPollWordsT w;
	for (unsigned k = 0; k < n; ++k) {
		w.push_back(wide ? (uint16_t)(0x1234u + seed * 0x0300u + 0x0421u * k) : (uint16_t)((0x31u + seed * 0x40u + 17u * k) & 0xFFu));
	}
	return w;
}
spiPollBytes spiPollPack(const spiPollWordsT &w, bool wide) {
	spiPollBytes b;
	for (uint16_t x : w) {
		b.push_back((uint8_t)(x & 0xFFu));
		if (wide) b.push_back((uint8_t)(x >> 8));
	}
	return b;
}
spiPollWordsT spiPollUnpack(const spiPollBytes &b, bool wide) {
	spiPollWordsT w;
	for (std::size_t k = 0; k < b.size(); k += wide ? 2 : 1) w.push_back(wide ? (uint16_t)(b[k] | (b[k + 1] << 8)) : (uint16_t)b[k]);
	return w;
}

void spiPollInit(SPI_HandleTypeDef &h, unsigned i, uint32_t mode, uint32_t dir, bool wide) {
	h = SPI_HandleTypeDef();
	h.Instance = spiPollBase(i);
	h.Init.Mode = mode;
	h.Init.Direction = dir;
	h.Init.DataSize = wide ? SPI_DATASIZE_16BIT : SPI_DATASIZE_8BIT;
	h.Init.CLKPolarity = SPI_POLARITY_LOW;
	h.Init.CLKPhase = SPI_PHASE_1EDGE;
	h.Init.NSS = SPI_NSS_SOFT;
	h.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	h.Init.FirstBit = SPI_FIRSTBIT_MSB;
	h.Init.TIMode = SPI_TIMODE_DISABLE;
	h.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	h.Init.CRCPolynomial = 7;
	REQUIRE(HAL_SPI_Init(&h) == HAL_OK);
}

/* Back to the power-on state of the model and the registers between two iterations of a loop in one test case. */
void spiPollFresh(unsigned frameTicks = 1) {
	hostRegsDisarm();
	hostPeriphResetState();
	hostRegsReset();
	hostRegsArm();
	hostSpiSetFrameTicks(frameTicks);
}

void spiPollPeer(unsigned i, const spiPollWordsT &w) {
	for (uint16_t x : w) hostSpiPeerPush(i, x);
}

/* SPE-enabling write of CR1 in the access log: the value written (0 and a failed check if there was none). */
uint32_t spiPollCr1AtEnable(unsigned i) {
	const uintptr_t a = reinterpret_cast<uintptr_t>(&spiPollBase(i)->CR1);
	for (std::size_t k = 0; k < hostRegsLogCount(); ++k) {
		const HostRegAccess *e = hostRegsLogAt(k);
		if (e->write && e->addr == a && (e->written & spiPollCR1_SPE) != 0u) return e->written;
	}
	FAIL_CHECK("the HAL never set SPE");
	return 0;
}

HAL_StatusTypeDef spiPollTransmit(SPI_HandleTypeDef &h, const spiPollWordsT &w, bool wide, uint32_t timeout = 100) {
	spiPollBytes b = spiPollPack(w, wide);
	return HAL_SPI_Transmit(&h, b.data(), (uint16_t)w.size(), timeout);
}
HAL_StatusTypeDef spiPollReceive(SPI_HandleTypeDef &h, spiPollWordsT &out, unsigned n, bool wide, uint32_t timeout = 100) {
	spiPollBytes b(n * (wide ? 2u : 1u), 0xEE);
	const HAL_StatusTypeDef st = HAL_SPI_Receive(&h, b.data(), (uint16_t)n, timeout);
	out = spiPollUnpack(b, wide);
	return st;
}
HAL_StatusTypeDef spiPollTxRx(SPI_HandleTypeDef &h, const spiPollWordsT &w, spiPollWordsT &out, bool wide, uint32_t timeout = 100) {
	spiPollBytes t = spiPollPack(w, wide);
	spiPollBytes r(t.size(), 0xEE);
	const HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&h, t.data(), r.data(), (uint16_t)w.size(), timeout);
	out = spiPollUnpack(r, wide);
	return st;
}

#define SPIPOLL_INSTANCES        \
	unsigned inst = 0;           \
	SUBCASE("SPI1") { inst = 0; } \
	SUBCASE("SPI2") { inst = 1; } \
	SUBCASE("SPI3") { inst = 2; }

/* ----- slave / fault master driven from the tick hook ----- */
struct spiPollHook {
	bool active = false;
	unsigned inst = 0;
	bool needTxPending = false; /* deliver a frame only while the slave has a word to send (TXE = 0) */
	bool needRxEmpty = false;   /* deliver a frame only while RXNE = 0 */
	unsigned perTick = 1;       /* frames per tick (2: a missed frame) */
	spiPollWordsT master;       /* the words the master sends; then it stops clocking */
	std::size_t next = 0;
	unsigned calls = 0;
	unsigned modfAtCall = 0; /* 0 = never */
	unsigned framesDone = 0;
};
spiPollHook spiPollH;

void spiPollStep() {
	HAL_IncTick();
	if (!spiPollH.active) return;
	++spiPollH.calls;
	if (spiPollH.modfAtCall != 0u && spiPollH.calls == spiPollH.modfAtCall) {
		hostSpiRaiseModf(spiPollH.inst);
		return;
	}
	if (spiPollH.modfAtCall != 0u) return; /* MODF mode: no slave frames */
	if ((HalFixture::peek(&spiPollBase(spiPollH.inst)->CR1) & spiPollCR1_SPE) == 0u) return; /* the master only clocks an enabled slave */
	for (unsigned f = 0; f < spiPollH.perTick; ++f) {
		const uint32_t sr = HalFixture::peek(&spiPollBase(spiPollH.inst)->SR);
		if (spiPollH.needTxPending && (sr & spiPollSR_TXE) != 0u) return;
		if (spiPollH.needRxEmpty && (sr & spiPollSR_RXNE) != 0u) return;
		if (spiPollH.next >= spiPollH.master.size()) return; /* the master has clocked all its words */
		const uint16_t w = spiPollH.master[spiPollH.next];
		++spiPollH.next;
		++spiPollH.framesDone;
		hostSpiSlaveFrame(spiPollH.inst, w);
	}
}
void spiPollArm(unsigned inst, const spiPollWordsT &master) {
	spiPollH = spiPollHook();
	spiPollH.inst = inst;
	spiPollH.master = master;
	spiPollH.active = true;
	hostTickSetStep(spiPollStep);
}

struct spiPollStepGuard {
	~spiPollStepGuard() {
		hostTickSetStep(nullptr);
		spiPollH = spiPollHook();
	}
};

} // namespace

/* =============================================================================================================== SP1 */

TEST_CASE_FIXTURE(SpiFixture, "SP1 master 2 lines Transmit puts the bytes on the bus in order and returns HAL_OK") {
	SPIPOLL_INSTANCES
	for (unsigned n : {1u, 2u, 3u, 16u}) {
		spiPollFresh();
		CAPTURE(n);
		hostSpiTxClear(inst);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
		const spiPollWordsT w = spiPollWords(n, false, 0);
		CHECK(spiPollTransmit(h, w, false) == HAL_OK);
		REQUIRE(hostSpiTxCount(inst) == n);
		for (unsigned k = 0; k < n; ++k) {
			CAPTURE(k);
			CHECK(hostSpiTxAt(inst, k) == w[k]);
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP1 master 2 lines Transmit leaves the state READY and the handle unlocked") {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	REQUIRE(spiPollTransmit(h, spiPollWords(3, false, 0), false) == HAL_OK);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(HAL_SPI_GetError(&h) == HAL_SPI_ERROR_NONE);
}

TEST_CASE_FIXTURE(SpiFixture, "SP1 master 2 lines Transmit returns with BSY clear") {
	SPIPOLL_INSTANCES
	const unsigned spiPollFt = 3; // RM0090 28.3.8: the HAL must wait for the last frame, not only for TXE
	for (unsigned n : {1u, 2u, 3u, 16u}) {
		spiPollFresh(spiPollFt);
		CAPTURE(n);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
		REQUIRE(spiPollTransmit(h, spiPollWords(n, false, 0), false, 1000) == HAL_OK);
		CHECK((spiPollSr(inst) & spiPollSR_BSY) == 0u);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP1 master 2 lines Transmit returns with OVR clear") {
	SPIPOLL_INSTANCES
	// RM0090 28.3.5: two frames sent in transmit-only mode set OVR because the received data are never read; 28.3.10: it is
	// cleared by reading DR then SR.
	for (unsigned n : {2u, 3u, 16u}) {
		spiPollFresh();
		CAPTURE(n);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
		REQUIRE(spiPollTransmit(h, spiPollWords(n, false, 0), false) == HAL_OK);
		CHECK((spiPollSr(inst) & spiPollSR_OVR) == 0u);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP1 master 2 lines Transmit never disables the SPI while BSY is set") {
	SPIPOLL_INSTANCES
	const unsigned spiPollFt = 2;
	for (unsigned n : {1u, 3u}) {
		spiPollFresh(spiPollFt);
		CAPTURE(n);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
		REQUIRE(spiPollTransmit(h, spiPollWords(n, false, 0), false, 1000) == HAL_OK);
		CHECK(hostSpiDisabledBusy(inst) == 0u); // RM0090 28.3.8
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP1 master 2 lines Transmit does not change the configuration while SPE is set") {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	REQUIRE(spiPollTransmit(h, spiPollWords(3, false, 0), false) == HAL_OK);
	CHECK(hostSpiConfigWhileEnabled(inst) == 0u); // RM0090 28.5.1
}

TEST_CASE_FIXTURE(SpiFixture, "SP1 master 2 lines Transmit leaves the other two instances untouched") {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	REQUIRE(spiPollTransmit(h, spiPollWords(3, false, 0), false) == HAL_OK);
	for (unsigned o = 0; o < 3; ++o) {
		if (o == inst) continue;
		CAPTURE(o);
		CHECK(hostSpiTxCount(o) == 0u);
		CHECK(spiPollCr1(o) == 0u);
	}
}

/* =============================================================================================================== SP2 */

TEST_CASE_FIXTURE(SpiFixture, "SP2 master 2 lines Transmit of 16 bit frames sends Size frames with the 16 bit values") {
	SPIPOLL_INSTANCES
	for (unsigned n : {1u, 5u}) {
		spiPollFresh();
		CAPTURE(n);
		hostSpiTxClear(inst);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, true);
		const spiPollWordsT w = spiPollWords(n, true, 0);
		CHECK(spiPollTransmit(h, w, true) == HAL_OK);
		REQUIRE(hostSpiTxCount(inst) == n); // Size counts frames, not bytes
		for (unsigned k = 0; k < n; ++k) {
			CAPTURE(k);
			CHECK(hostSpiTxAt(inst, k) == w[k]);
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP2 16 bit Transmit has DFF set in CR1 at the write that enables the SPI") {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, true);
	REQUIRE(spiPollTransmit(h, spiPollWords(2, true, 0), true) == HAL_OK);
	CHECK((spiPollCr1AtEnable(inst) & spiPollCR1_DFF) != 0u); // RM0090 28.5.1: DFF = 1 is 16 bit
}

TEST_CASE_FIXTURE(SpiFixture, "SP2 8 bit Transmit has DFF clear in CR1 at the write that enables the SPI") {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	REQUIRE(spiPollTransmit(h, spiPollWords(2, false, 0), false) == HAL_OK);
	CHECK((spiPollCr1AtEnable(inst) & spiPollCR1_DFF) == 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP2 16 bit Transmit returns with the state READY, BSY clear and OVR clear") {
	SPIPOLL_INSTANCES
	hostSpiSetFrameTicks(2);
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, true);
	REQUIRE(spiPollTransmit(h, spiPollWords(5, true, 0), true, 1000) == HAL_OK);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	const uint32_t sr = spiPollSr(inst);
	CHECK((sr & spiPollSR_BSY) == 0u);
	CHECK((sr & spiPollSR_OVR) == 0u);
}

/* =============================================================================================================== SP3 */

namespace {
struct spiPollDir {
	uint32_t dir;
	bool freeRunning;
	const char *name;
};
const spiPollDir spiPollRxDirs[] = {{SPI_DIRECTION_2LINES, false, "2 lines"},
                                    {SPI_DIRECTION_2LINES_RXONLY, true, "2 lines RX only"},
                                    {SPI_DIRECTION_1LINE, true, "1 line"}};
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SP3 master Receive returns the words the peer sent and HAL_OK") {
	SPIPOLL_INSTANCES
	for (const spiPollDir &d : spiPollRxDirs) {
		spiPollFresh();
		for (const spiPollSize &s : spiPollSizes) {
			spiPollFresh();
			CAPTURE(d.name);
			CAPTURE(s.wide);
			CAPTURE(s.n);
			SPI_HandleTypeDef h;
			spiPollInit(h, inst, SPI_MODE_MASTER, d.dir, s.wide);
			const spiPollWordsT peer = spiPollWords(s.n, s.wide, 1);
			spiPollPeer(inst, peer);
			spiPollWordsT got;
			CHECK(spiPollReceive(h, got, s.n, s.wide) == HAL_OK);
			CHECK(got == peer);
		}
	}
}

// Note for the receive-only master cases below: the model counter hostSpiDisabledBusy and RM0090 28.3.8 (SPE is cleared during the
// last frame of a receive-only master) are in tension, so those cases assert only what the manual states (SPE clear, no OVR, no BSY).
TEST_CASE_FIXTURE(SpiFixture, "SP3 characterization: master RX only and 1 line Receive stop the clock, SPE is clear on return") {
	// NEEDS REVIEW (proposed): RM0090 28.3.8 clears SPE during the last frame of a receive-only master; the extra free-running frame
	// depends on the frame timing of the model, so only SPE clear on return is recorded (hostSpiDisabledBusy is not asserted here).
	SPIPOLL_INSTANCES
	for (const spiPollDir &d : spiPollRxDirs) {
		spiPollFresh();
		if (!d.freeRunning) continue;
		for (const spiPollSize &s : spiPollSizes) {
			spiPollFresh();
			CAPTURE(d.name);
			CAPTURE(s.n);
			SPI_HandleTypeDef h;
			spiPollInit(h, inst, SPI_MODE_MASTER, d.dir, s.wide);
			spiPollPeer(inst, spiPollWords(s.n, s.wide, 1));
			spiPollWordsT got;
			REQUIRE(spiPollReceive(h, got, s.n, s.wide) == HAL_OK);
			CHECK((spiPollCr1(inst) & spiPollCR1_SPE) == 0u); // RM0090 28.3.5: the clock runs until SPE = 0
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP3 characterization: master RX only and 1 line Receive leave no OVR and no BSY") {
	// NEEDS REVIEW (proposed): the HAL stops the free-running clock after the last frame; whether an extra frame sets OVR depends on
	// the frame timing of the model; observed: OVR and BSY clear on return.
	SPIPOLL_INSTANCES
	for (const spiPollDir &d : spiPollRxDirs) {
		spiPollFresh();
		for (const spiPollSize &s : spiPollSizes) {
			spiPollFresh();
			CAPTURE(d.name);
			CAPTURE(s.n);
			SPI_HandleTypeDef h;
			spiPollInit(h, inst, SPI_MODE_MASTER, d.dir, s.wide);
			spiPollPeer(inst, spiPollWords(s.n, s.wide, 1));
			spiPollWordsT got;
			REQUIRE(spiPollReceive(h, got, s.n, s.wide) == HAL_OK);
			const uint32_t sr = spiPollSr(inst);
			CHECK((sr & spiPollSR_OVR) == 0u);
			CHECK((sr & spiPollSR_BSY) == 0u);
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP3 master Receive leaves the state READY and the handle unlocked") {
	SPIPOLL_INSTANCES
	for (const spiPollDir &d : spiPollRxDirs) {
		spiPollFresh();
		CAPTURE(d.name);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, d.dir, false);
		spiPollWordsT got;
		REQUIRE(spiPollReceive(h, got, 3, false) == HAL_OK);
		CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP3 master Receive does not disable the SPI while BSY is set except in the receive-only sequence") {
	SPIPOLL_INSTANCES
	// RM0090 28.3.8: the receive-only master is stopped after the second to last RXNE, so the last frame is still running when
	// SPE is cleared; for the 2 lines mode the HAL must wait for BSY = 0 first. Only the 2 lines mode is checked.
	hostSpiSetFrameTicks(2);
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	spiPollWordsT got;
	REQUIRE(spiPollReceive(h, got, 3, false, 1000) == HAL_OK);
	CHECK(hostSpiDisabledBusy(inst) == 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP3 master Receive does not change the configuration while SPE is set") {
	SPIPOLL_INSTANCES
	for (const spiPollDir &d : spiPollRxDirs) {
		spiPollFresh();
		CAPTURE(d.name);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, d.dir, false);
		spiPollWordsT got;
		REQUIRE(spiPollReceive(h, got, 3, false) == HAL_OK);
		CHECK(hostSpiConfigWhileEnabled(inst) == 0u); // RM0090 28.5.1
	}
}

/* =============================================================================================================== SP4 */

TEST_CASE_FIXTURE(SpiFixture, "SP4 master 2 lines TransmitReceive returns the peer words") {
	SPIPOLL_INSTANCES
	for (const spiPollSize &s : spiPollSizes) {
		spiPollFresh();
		CAPTURE(s.wide);
		CAPTURE(s.n);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, s.wide);
		const spiPollWordsT peer = spiPollWords(s.n, s.wide, 2);
		spiPollPeer(inst, peer);
		spiPollWordsT got;
		CHECK(spiPollTxRx(h, spiPollWords(s.n, s.wide, 0), got, s.wide) == HAL_OK);
		CHECK(got == peer);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP4 master 2 lines TransmitReceive puts the tx buffer on the bus in order") {
	SPIPOLL_INSTANCES
	for (const spiPollSize &s : spiPollSizes) {
		spiPollFresh();
		CAPTURE(s.wide);
		CAPTURE(s.n);
		hostSpiTxClear(inst);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, s.wide);
		const spiPollWordsT w = spiPollWords(s.n, s.wide, 0);
		spiPollWordsT got;
		REQUIRE(spiPollTxRx(h, w, got, s.wide) == HAL_OK);
		REQUIRE(hostSpiTxCount(inst) == s.n);
		for (unsigned k = 0; k < s.n; ++k) {
			CAPTURE(k);
			CHECK(hostSpiTxAt(inst, k) == w[k]);
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP4 master 2 lines TransmitReceive in loopback returns what was sent") {
	SPIPOLL_INSTANCES
	for (const spiPollSize &s : spiPollSizes) {
		spiPollFresh();
		CAPTURE(s.wide);
		CAPTURE(s.n);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, s.wide);
		hostSpiPeerLoopback(inst, 1);
		const spiPollWordsT w = spiPollWords(s.n, s.wide, 3);
		spiPollWordsT got;
		CHECK(spiPollTxRx(h, w, got, s.wide) == HAL_OK);
		CHECK(got == w);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP4 master 2 lines TransmitReceive leaves READY, unlocked, BSY clear, OVR clear") {
	SPIPOLL_INSTANCES
	hostSpiSetFrameTicks(2);
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	spiPollWordsT got;
	REQUIRE(spiPollTxRx(h, spiPollWords(16, false, 0), got, false, 1000) == HAL_OK);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	const uint32_t sr = spiPollSr(inst);
	CHECK((sr & spiPollSR_BSY) == 0u);
	CHECK((sr & spiPollSR_OVR) == 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP4 master 2 lines TransmitReceive neither disables while BSY nor changes the configuration while enabled") {
	SPIPOLL_INSTANCES
	hostSpiSetFrameTicks(2);
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	spiPollWordsT got;
	REQUIRE(spiPollTxRx(h, spiPollWords(3, false, 0), got, false, 1000) == HAL_OK);
	CHECK(hostSpiDisabledBusy(inst) == 0u);
	CHECK(hostSpiConfigWhileEnabled(inst) == 0u);
}

/* =============================================================================================================== SP5 */

TEST_CASE_FIXTURE(SpiFixture, "SP5 1 line Transmit has BIDIMODE and BIDIOE set at the write that enables the SPI") {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_1LINE, false);
	REQUIRE(spiPollTransmit(h, spiPollWords(3, false, 0), false) == HAL_OK);
	const uint32_t cr1 = spiPollCr1AtEnable(inst);
	CHECK((cr1 & spiPollCR1_BIDIMODE) != 0u); // RM0090 28.3.5: both set before the SPI is enabled
	CHECK((cr1 & spiPollCR1_BIDIOE) != 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP5 1 line Transmit sends the bytes in order") {
	SPIPOLL_INSTANCES
	for (const spiPollSize &s : spiPollSizes) {
		spiPollFresh();
		CAPTURE(s.wide);
		CAPTURE(s.n);
		hostSpiTxClear(inst);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_1LINE, s.wide);
		const spiPollWordsT w = spiPollWords(s.n, s.wide, 0);
		CHECK(spiPollTransmit(h, w, s.wide) == HAL_OK);
		REQUIRE(hostSpiTxCount(inst) == s.n);
		for (unsigned k = 0; k < s.n; ++k) CHECK(hostSpiTxAt(inst, k) == w[k]);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP5 1 line Receive has BIDIMODE set and BIDIOE clear at the write that enables the SPI") {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_1LINE, false);
	HalFixture::poke(&spiPollBase(inst)->CR1, spiPollCr1(inst) | spiPollCR1_BIDIOE); // left over from an earlier Transmit
	spiPollPeer(inst, spiPollWords(3, false, 1));
	spiPollWordsT got;
	REQUIRE(spiPollReceive(h, got, 3, false) == HAL_OK);
	const uint32_t cr1 = spiPollCr1AtEnable(inst);
	CHECK((cr1 & spiPollCR1_BIDIMODE) != 0u);
	CHECK((cr1 & spiPollCR1_BIDIOE) == 0u); // RM0090 28.3.5: bidirectional receive needs BIDIOE = 0
}

TEST_CASE_FIXTURE(SpiFixture, "SP5 1 line Transmit after a Receive sets BIDIOE again") {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_1LINE, false);
	spiPollWordsT got;
	REQUIRE(spiPollReceive(h, got, 2, false) == HAL_OK);
	hostSpiTxClear(inst);
	REQUIRE(spiPollTransmit(h, spiPollWords(2, false, 0), false) == HAL_OK);
	CHECK((spiPollCr1(inst) & spiPollCR1_BIDIOE) != 0u);
	CHECK(hostSpiTxCount(inst) == 2u);
}

/* =============================================================================================================== SP6 */

namespace {
enum spiPollFn { spiPollFnTx, spiPollFnRx, spiPollFnTxRx };
/* Runs the function on a silent bus; returns the status and the elapsed ticks. */
HAL_StatusTypeDef spiPollSilent(SPI_HandleTypeDef &h, unsigned inst, spiPollFn fn, uint32_t timeout, uint32_t &elapsed) {
	hostSpiPeerSilent(inst, 1);
	const uint32_t t0 = hostTickPeek();
	HAL_StatusTypeDef st;
	spiPollWordsT out;
	switch (fn) {
	case spiPollFnTx: st = spiPollTransmit(h, spiPollWords(3, false, 0), false, timeout); break;
	case spiPollFnRx: st = spiPollReceive(h, out, 3, false, timeout); break;
	default: st = spiPollTxRx(h, spiPollWords(3, false, 0), out, false, timeout); break;
	}
	elapsed = hostTickPeek() - t0;
	return st;
}
const uint32_t spiPollSlack = 3; // ticks the loop may need after the timeout has expired
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SP6 a silent bus makes Transmit, Receive and TransmitReceive return HAL_TIMEOUT after about Timeout ticks") {
	SPIPOLL_INSTANCES
	for (spiPollFn fn : {spiPollFnTx, spiPollFnRx, spiPollFnTxRx}) {
		spiPollFresh();
		for (uint32_t timeout : {5u, 20u}) {
			spiPollFresh();
			CAPTURE((int)fn);
			CAPTURE(timeout);
			SPI_HandleTypeDef h;
			spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
			uint32_t elapsed = 0;
			CHECK(spiPollSilent(h, inst, fn, timeout, elapsed) == HAL_TIMEOUT);
			CHECK(elapsed >= timeout);
			CHECK(elapsed <= timeout + spiPollSlack);
		}
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP6 after a timeout the state is READY and the handle is unlocked") {
	SPIPOLL_INSTANCES
	for (spiPollFn fn : {spiPollFnTx, spiPollFnRx, spiPollFnTxRx}) {
		spiPollFresh();
		CAPTURE((int)fn);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
		uint32_t elapsed = 0;
		REQUIRE(spiPollSilent(h, inst, fn, 5, elapsed) == HAL_TIMEOUT);
		CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP6 a receive-only master and a 1 line master time out on a silent bus too") {
	SPIPOLL_INSTANCES
	for (uint32_t dir : {(uint32_t)SPI_DIRECTION_2LINES_RXONLY, (uint32_t)SPI_DIRECTION_1LINE}) {
		spiPollFresh();
		CAPTURE(dir);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, dir, false);
		uint32_t elapsed = 0;
		CHECK(spiPollSilent(h, inst, spiPollFnRx, 10, elapsed) == HAL_TIMEOUT);
		CHECK(elapsed >= 10u);
		CHECK(elapsed <= 10u + spiPollSlack);
		CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP6 a frame longer than Timeout makes Transmit return HAL_TIMEOUT") {
	SPIPOLL_INSTANCES
	hostSpiSetFrameTicks(50);
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	const uint32_t t0 = hostTickPeek();
	CHECK(spiPollTransmit(h, spiPollWords(3, false, 0), false, 5) == HAL_TIMEOUT);
	const uint32_t elapsed = hostTickPeek() - t0;
	CHECK(elapsed >= 5u);
	CHECK(elapsed <= 5u + spiPollSlack);
}

TEST_CASE_FIXTURE(SpiFixture, "SP6 HAL_MAX_DELAY waits for slow frames and returns HAL_OK") {
	SPIPOLL_INSTANCES
	hostSpiSetFrameTicks(200);
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	spiPollPeer(inst, spiPollWords(2, false, 1));
	spiPollWordsT got;
	CHECK(spiPollTxRx(h, spiPollWords(2, false, 0), got, false, HAL_MAX_DELAY) == HAL_OK);
	CHECK(got == spiPollWords(2, false, 1));
}

TEST_CASE_FIXTURE(SpiFixture, "SP6 characterization: Timeout 0 on a silent bus returns HAL_TIMEOUT at once") {
	// NEEDS REVIEW (proposed): UM1725 gives no meaning for Timeout = 0. Observed: HAL_TIMEOUT within a few ticks, state READY,
	// unlocked. No spec says whether 0 means "do not wait" or "wait forever".
	SPIPOLL_INSTANCES
	for (spiPollFn fn : {spiPollFnTx, spiPollFnRx, spiPollFnTxRx}) {
		spiPollFresh();
		CAPTURE((int)fn);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
		uint32_t elapsed = 0;
		CHECK(spiPollSilent(h, inst, fn, 0, elapsed) == HAL_TIMEOUT);
		CHECK(elapsed <= spiPollSlack);
		CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP6 characterization: the error code after a timeout") {
	// NEEDS REVIEW (proposed): UM1725 names HAL_SPI_ERROR_FLAG ("Flag: RXNE, TXE, BSY") but does not say which code a timeout of
	// a polling call sets. Observed: the call returns HAL_TIMEOUT and the error code stays HAL_SPI_ERROR_NONE.
	SPIPOLL_INSTANCES
	for (spiPollFn fn : {spiPollFnTx, spiPollFnRx, spiPollFnTxRx}) {
		spiPollFresh();
		CAPTURE((int)fn);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
		uint32_t elapsed = 0;
		REQUIRE(spiPollSilent(h, inst, fn, 5, elapsed) == HAL_TIMEOUT);
		CHECK(HAL_SPI_GetError(&h) == HAL_SPI_ERROR_NONE);
	}
}

/* =============================================================================================================== SP7 */

namespace {
enum spiPollBad { spiPollBadNull, spiPollBadSize0 };
/* One rejected call on a handle that was just initialised, so that only the argument check can make it return: it must return at
 * once (0 or 1 tick) and the SPI must not have been enabled. Direction and function: the 2 lines master Transmit and
 * TransmitReceive, the receive-only and 1 line master Receive (the 2 lines master Receive has its own KNOWN_ISSUE cases). */
void spiPollRejected(unsigned inst, uint32_t dir, spiPollFn fn, spiPollBad bad, HAL_StatusTypeDef &ret, uint32_t &elapsed, SPI_HandleTypeDef &h) {
	spiPollFresh();
	spiPollInit(h, inst, SPI_MODE_MASTER, dir, false);
	uint8_t buf[4] = {1, 2, 3, 4};
	uint8_t *p = bad == spiPollBadNull ? nullptr : buf;
	const uint16_t n = bad == spiPollBadNull ? 3 : 0;
	const uint32_t t0 = hostTickPeek();
	switch (fn) {
	case spiPollFnTx: ret = HAL_SPI_Transmit(&h, p, n, 100); break;
	case spiPollFnRx: ret = HAL_SPI_Receive(&h, p, n, 100); break;
	default: ret = HAL_SPI_TransmitReceive(&h, p, buf, n, 100); break;
	}
	elapsed = hostTickPeek() - t0;
}
struct spiPollRejectCase {
	uint32_t dir;
	spiPollFn fn;
	const char *name;
};
const spiPollRejectCase spiPollRejectCases[] = {{SPI_DIRECTION_2LINES, spiPollFnTx, "Transmit 2 lines"},
                                                {SPI_DIRECTION_2LINES, spiPollFnTxRx, "TransmitReceive 2 lines"},
                                                {SPI_DIRECTION_2LINES_RXONLY, spiPollFnRx, "Receive RX only"},
                                                {SPI_DIRECTION_1LINE, spiPollFnRx, "Receive 1 line"},
                                                {SPI_DIRECTION_1LINE, spiPollFnTx, "Transmit 1 line"}};
void spiPollCheckRejected(unsigned inst, spiPollBad bad) {
	for (const spiPollRejectCase &c : spiPollRejectCases) {
		CAPTURE(c.name);
		SPI_HandleTypeDef h;
		HAL_StatusTypeDef ret = HAL_OK;
		uint32_t elapsed = 99;
		spiPollRejected(inst, c.dir, c.fn, bad, ret, elapsed, h);
		CHECK(ret == HAL_ERROR);
		CHECK(elapsed <= 1u); // returned at once: the argument check, not a timeout
		CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
		CHECK(hostSpiTxCount(inst) == 0u);
		CHECK((spiPollCr1(inst) & spiPollCR1_SPE) == 0u);
	}
}
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SP7 characterization: a NULL data pointer returns HAL_ERROR at once and starts nothing") {
	// NEEDS REVIEW (proposed): UM1725 does not give the return code for a NULL buffer. Observed: HAL_ERROR within 1 tick,
	// state READY, handle unlocked, SPE clear, nothing on the bus (each call on a freshly initialised handle).
	SPIPOLL_INSTANCES
	spiPollCheckRejected(inst, spiPollBadNull);
}

TEST_CASE_FIXTURE(SpiFixture, "SP7 characterization: Size 0 returns HAL_ERROR at once and starts nothing") {
	// NEEDS REVIEW (proposed): as above for Size = 0.
	SPIPOLL_INSTANCES
	spiPollCheckRejected(inst, spiPollBadSize0);
}

TEST_CASE_FIXTURE(SpiFixture, "SP7 characterization: a NULL receive buffer of TransmitReceive returns HAL_ERROR at once") {
	// NEEDS REVIEW (proposed): as above, for pRxData = NULL.
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t buf[4] = {1, 2, 3, 4};
	const uint32_t t0 = hostTickPeek();
	CHECK(HAL_SPI_TransmitReceive(&h, buf, nullptr, 3, 100) == HAL_ERROR);
	CHECK(hostTickPeek() - t0 <= 1u);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
	CHECK(hostSpiTxCount(inst) == 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP7 master 2 lines Receive with a NULL pointer returns HAL_ERROR and leaves the state READY" * KNOWN_ISSUE("candidate vendor bug V97: HAL_SPI_Receive of a 2 lines master sets the state BUSY_RX and then calls HAL_SPI_TransmitReceive, which rejects the arguments and returns HAL_ERROR without restoring the state; the handle stays BUSY_RX and every later Transmit / Receive returns HAL_BUSY (UM1725: a failed call must not start a transfer)")) {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	REQUIRE(HAL_SPI_Receive(&h, nullptr, 3, 10) == HAL_ERROR);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiFixture, "SP7 master 2 lines Receive with Size 0 returns HAL_ERROR and leaves the state READY" * KNOWN_ISSUE("candidate vendor bug V97: as for the NULL pointer, HAL_SPI_Receive of a 2 lines master leaves the state BUSY_RX after HAL_ERROR for Size 0")) {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	uint8_t buf[4] = {1, 2, 3, 4};
	REQUIRE(HAL_SPI_Receive(&h, buf, 0, 10) == HAL_ERROR);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiFixture, "SP7 master 2 lines Receive stays usable after a rejected call" * KNOWN_ISSUE("candidate vendor bug V97: after the rejected HAL_SPI_Receive the state is BUSY_RX and the next Receive returns HAL_BUSY")) {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	REQUIRE(HAL_SPI_Receive(&h, nullptr, 3, 10) == HAL_ERROR);
	spiPollWordsT got;
	CHECK(spiPollReceive(h, got, 2, false) == HAL_OK);
}

TEST_CASE_FIXTURE(SpiFixture, "SP7 characterization: a state that is not READY returns HAL_BUSY and leaves the state alone") {
	// NEEDS REVIEW (proposed): UM1725 does not give the return code when a transfer is running. Observed: HAL_BUSY, state
	// unchanged, nothing on the bus.
	SPIPOLL_INSTANCES
	uint8_t buf[4] = {1, 2, 3, 4};
	for (HAL_SPI_StateTypeDef st : {HAL_SPI_STATE_BUSY_TX, HAL_SPI_STATE_BUSY_TX_RX, HAL_SPI_STATE_ABORT, HAL_SPI_STATE_ERROR}) {
		CAPTURE((int)st);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
		h.State = st;
		CHECK(HAL_SPI_Transmit(&h, buf, 2, 10) == HAL_BUSY);
		CHECK(HAL_SPI_Receive(&h, buf, 2, 10) == HAL_BUSY);
		CHECK(HAL_SPI_TransmitReceive(&h, buf, buf, 2, 10) == HAL_BUSY);
		CHECK(h.State == st);
		CHECK(hostSpiTxCount(inst) == 0u);
		CHECK((spiPollCr1(inst) & spiPollCR1_SPE) == 0u);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP7 characterization: a locked handle returns HAL_BUSY and starts nothing") {
	// NEEDS REVIEW (proposed): UM1725 does not describe the lock. Observed: HAL_BUSY, handle stays locked, state READY,
	// nothing on the bus (Receive of a 2 lines master: see the KNOWN_ISSUE case below).
	SPIPOLL_INSTANCES
	uint8_t buf[4] = {1, 2, 3, 4};
	SPI_HandleTypeDef h, hr;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	spiPollInit(hr, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES_RXONLY, false);
	h.Lock = HAL_LOCKED;
	hr.Lock = HAL_LOCKED;
	CHECK(HAL_SPI_Transmit(&h, buf, 2, 10) == HAL_BUSY);
	CHECK(HAL_SPI_TransmitReceive(&h, buf, buf, 2, 10) == HAL_BUSY);
	CHECK(HAL_SPI_Receive(&hr, buf, 2, 10) == HAL_BUSY);
	CHECK(h.Lock == HAL_LOCKED);
	CHECK(hr.Lock == HAL_LOCKED);
	CHECK(h.State == HAL_SPI_STATE_READY);
	CHECK(hr.State == HAL_SPI_STATE_READY);
	CHECK(hostSpiTxCount(inst) == 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP7 master 2 lines Receive on a locked handle returns HAL_BUSY and leaves the state READY" * KNOWN_ISSUE("candidate vendor bug V97: HAL_SPI_Receive of a 2 lines master sets the state BUSY_RX before the lock of HAL_SPI_TransmitReceive refuses the call, so a refused call (HAL_BUSY) changes the state of a handle that is not running a transfer")) {
	SPIPOLL_INSTANCES
	uint8_t buf[4] = {1, 2, 3, 4};
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	h.Lock = HAL_LOCKED;
	REQUIRE(HAL_SPI_Receive(&h, buf, 2, 10) == HAL_BUSY);
	CHECK(h.State == HAL_SPI_STATE_READY);
}

TEST_CASE_FIXTURE(SpiFixture, "SP7 a successful transfer can be followed by another one on the same handle") {
	SPIPOLL_INSTANCES
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	REQUIRE(spiPollTransmit(h, spiPollWords(2, false, 0), false) == HAL_OK);
	spiPollPeer(inst, spiPollWords(2, false, 2));
	spiPollWordsT got;
	CHECK(spiPollTxRx(h, spiPollWords(2, false, 0), got, false) == HAL_OK);
	CHECK(got == spiPollWords(2, false, 2));
}

/* =============================================================================================================== SP8 */

TEST_CASE_FIXTURE(SpiFixture, "SP8 slave Transmit sends the bytes the master clocks out") {
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	for (const spiPollSize &s : spiPollSizes) {
		spiPollFresh();
		CAPTURE(s.wide);
		CAPTURE(s.n);
		hostSpiTxClear(inst);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, s.wide);
		spiPollArm(inst, spiPollWords(s.n + 4, s.wide, 2));
		spiPollH.needTxPending = true;
		const spiPollWordsT w = spiPollWords(s.n, s.wide, 0);
		CHECK(spiPollTransmit(h, w, s.wide, 1000) == HAL_OK);
		REQUIRE(hostSpiTxCount(inst) == s.n);
		for (unsigned k = 0; k < s.n; ++k) CHECK(hostSpiTxAt(inst, k) == w[k]);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP8 slave Transmit returns with the state READY, unlocked and OVR clear") {
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false);
	spiPollArm(inst, spiPollWords(8, false, 2));
	spiPollH.needTxPending = true; // the master clocks frames and never reads: RM0090 28.3.5 sets OVR after two frames
	REQUIRE(spiPollTransmit(h, spiPollWords(3, false, 0), false, 1000) == HAL_OK);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((spiPollSr(inst) & spiPollSR_OVR) == 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP8 slave Receive returns the words the master sent") {
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	for (const spiPollSize &s : spiPollSizes) {
		spiPollFresh();
		CAPTURE(s.wide);
		CAPTURE(s.n);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, s.wide);
		const spiPollWordsT m = spiPollWords(s.n, s.wide, 2);
		spiPollArm(inst, m);
		spiPollH.needRxEmpty = true;
		spiPollWordsT got;
		CHECK(spiPollReceive(h, got, s.n, s.wide, 1000) == HAL_OK);
		CHECK(got == m);
		CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP8 slave TransmitReceive moves data both ways") {
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	for (const spiPollSize &s : spiPollSizes) {
		spiPollFresh();
		CAPTURE(s.wide);
		CAPTURE(s.n);
		hostSpiTxClear(inst);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, s.wide);
		const spiPollWordsT m = spiPollWords(s.n, s.wide, 2);
		const spiPollWordsT w = spiPollWords(s.n, s.wide, 0);
		spiPollArm(inst, m);
		spiPollH.needTxPending = true;
		spiPollH.needRxEmpty = true;
		spiPollWordsT got;
		CHECK(spiPollTxRx(h, w, got, s.wide, 1000) == HAL_OK);
		CHECK(got == m);
		REQUIRE(hostSpiTxCount(inst) == s.n);
		for (unsigned k = 0; k < s.n; ++k) CHECK(hostSpiTxAt(inst, k) == w[k]);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP8 slave modes configure SPE without MSTR and do not disable while BSY") {
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	SPI_HandleTypeDef h;
	spiPollInit(h, inst, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false);
	spiPollArm(inst, spiPollWords(3, false, 2));
	spiPollH.needRxEmpty = true;
	spiPollWordsT got;
	REQUIRE(spiPollReceive(h, got, 3, false, 1000) == HAL_OK);
	CHECK((spiPollCr1AtEnable(inst) & spiPollCR1_MSTR) == 0u);
	CHECK(hostSpiDisabledBusy(inst) == 0u);
	CHECK(hostSpiConfigWhileEnabled(inst) == 0u);
}

namespace {
/* Slave Receive of 3 frames while the master clocks 2 frames per tick: every second frame is missed (RXNE still set). The master
 * sends 3 words in all, so only the first reaches the buffer. */
HAL_StatusTypeDef spiPollMissedFrame(SPI_HandleTypeDef &h, unsigned inst, spiPollWordsT &got) {
	spiPollInit(h, inst, SPI_MODE_SLAVE, SPI_DIRECTION_2LINES, false);
	spiPollArm(inst, spiPollWords(3, false, 2));
	spiPollH.perTick = 2;
	return spiPollReceive(h, got, 3, false, 50);
}
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SP8 slave Receive with a missed frame keeps the older byte in the receive buffer") {
	// RM0090 28.3.10 overrun: "the receiver buffer contents are not updated with the newly received data ... A read from the
	// SPI_DR register returns this byte". First word of seed 2: 0x31 + 0x80 = 0xB1.
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	SPI_HandleTypeDef h;
	spiPollWordsT got;
	spiPollMissedFrame(h, inst, got);
	CHECK(got[0] == 0xB1u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP8 characterization: slave Receive with a missed frame never sets HAL_SPI_ERROR_OVR") {
	// NEEDS REVIEW (proposed): a slave Receive that loses frames to an overrun times out and never sets HAL_SPI_ERROR_OVR;
	// RM0090 28.3.10 sets the OVR flag and UM1725 lists HAL_SPI_ERROR_OVR as the overrun error, but neither says what a polling
	// call reports. Observed: error code NONE.
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	SPI_HandleTypeDef h;
	spiPollWordsT got;
	spiPollMissedFrame(h, inst, got);
	CHECK((HAL_SPI_GetError(&h) & HAL_SPI_ERROR_OVR) == 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP8 characterization: slave Receive with a missed frame returns HAL_TIMEOUT with the state READY") {
	// NEEDS REVIEW (proposed): UM1725 gives no result for a polling Receive that lost frames. Observed: the lost frame is never
	// received, the call waits for the missing data until the Timeout and returns HAL_TIMEOUT, error code NONE, state READY,
	// handle unlocked, OVR clear on return.
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	SPI_HandleTypeDef h;
	spiPollWordsT got;
	CHECK(spiPollMissedFrame(h, inst, got) == HAL_TIMEOUT);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((spiPollSr(inst) & spiPollSR_OVR) == 0u);
}

/* =============================================================================================================== SP9 */

namespace {
/* Master Transmit of 16 bytes on which a mode fault hits at the 4th tick (SPE and MSTR are cleared by the model, RM0090 28.3.10). */
HAL_StatusTypeDef spiPollModfTransmit(SPI_HandleTypeDef &h, unsigned inst) {
	spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
	spiPollArm(inst, {});
	spiPollH.modfAtCall = 4;
	return spiPollTransmit(h, spiPollWords(16, false, 0), false, 20);
}
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SP9 a mode fault during Transmit sets the error code MODF" * KNOWN_ISSUE("candidate vendor bug V100: the polling HAL_SPI_Transmit never looks at MODF; after the mode fault cleared SPE and MSTR it keeps waiting for TXE until the timeout and HAL_SPI_ERROR_MODF is never set (arguable: UM1725 does not say that the polling functions report MODF; the oracle is the UM1725 error code list and RM0090 28.3.10, which expect the mode fault to end the transfer with the MODF error)")) {
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	SPI_HandleTypeDef h;
	spiPollModfTransmit(h, inst);
	CHECK((HAL_SPI_GetError(&h) & HAL_SPI_ERROR_MODF) != 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP9 a mode fault during Transmit does not return HAL_OK and clears the MODF flag" * KNOWN_ISSUE("candidate vendor bug V100: the mode fault of a polling transfer is not cleared by the HAL: the MODF flag stays set (RM0090 28.3.10: read SR then write CR1 clears it)")) {
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	SPI_HandleTypeDef h;
	CHECK(spiPollModfTransmit(h, inst) != HAL_OK);
	CHECK((HalFixture::peek(&spiPollBase(inst)->SR) & spiPollSR_MODF) == 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP9 characterization: a mode fault during Transmit makes the call time out") {
	// NEEDS REVIEW (proposed): UM1725 does not describe MODF for the polling functions. Observed: HAL_TIMEOUT after the
	// Timeout (the transmit loop waits for TXE that never comes), error code NONE, state READY, handle unlocked, SPE and MSTR
	// stay cleared (hardware), the MODF flag stays set.
	SPIPOLL_INSTANCES
	spiPollStepGuard guard;
	SPI_HandleTypeDef h;
	const uint32_t t0 = hostTickPeek();
	CHECK(spiPollModfTransmit(h, inst) == HAL_TIMEOUT);
	CHECK(hostTickPeek() - t0 >= 20u);
	CHECK(hostTickPeek() - t0 <= 20u + spiPollSlack);
	CHECK(HAL_SPI_GetError(&h) == HAL_SPI_ERROR_NONE);
	CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK((spiPollCr1(inst) & (spiPollCR1_SPE | spiPollCR1_MSTR)) == 0u);
	CHECK((HalFixture::peek(&spiPollBase(inst)->SR) & spiPollSR_MODF) != 0u);
}

/* ============================================================================================== SP6 (more timeouts) */

TEST_CASE_FIXTURE(SpiFixture, "SP6 a silent bus makes a 16 bit Transmit and TransmitReceive return HAL_TIMEOUT after about Timeout ticks") {
	SPIPOLL_INSTANCES
	for (bool txrx : {false, true}) {
		spiPollFresh();
		CAPTURE(txrx);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, true);
		hostSpiPeerSilent(inst, 1);
		spiPollWordsT out;
		const uint32_t t0 = hostTickPeek();
		const HAL_StatusTypeDef st = txrx ? spiPollTxRx(h, spiPollWords(3, true, 0), out, true, 10) : spiPollTransmit(h, spiPollWords(3, true, 0), true, 10);
		const uint32_t elapsed = hostTickPeek() - t0;
		CHECK(st == HAL_TIMEOUT);
		CHECK(elapsed >= 10u);
		CHECK(elapsed <= 10u + spiPollSlack);
		CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP6 characterization: a one frame Transmit and TransmitReceive on a silent bus") {
	// NEEDS REVIEW (proposed): with Size = 1 the only wait is the end-of-transaction wait for BSY = 0. UM1725 gives neither the
	// return code nor the error code of that timeout. Observed: Transmit returns HAL_ERROR with HAL_SPI_ERROR_FLAG (the
	// end-of-transaction wait failed), TransmitReceive returns HAL_TIMEOUT with error code NONE: the two functions report the same
	// condition differently.
	SPIPOLL_INSTANCES
	for (spiPollFn fn : {spiPollFnTx, spiPollFnTxRx}) {
		spiPollFresh();
		CAPTURE((int)fn);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
		hostSpiPeerSilent(inst, 1);
		spiPollWordsT out;
		const uint32_t t0 = hostTickPeek();
		const HAL_StatusTypeDef st = fn == spiPollFnTx ? spiPollTransmit(h, spiPollWords(1, false, 0), false, 10) : spiPollTxRx(h, spiPollWords(1, false, 0), out, false, 10);
		const uint32_t elapsed = hostTickPeek() - t0;
		CHECK(st == (fn == spiPollFnTx ? HAL_ERROR : HAL_TIMEOUT));
		CHECK(HAL_SPI_GetError(&h) == (fn == spiPollFnTx ? HAL_SPI_ERROR_FLAG : HAL_SPI_ERROR_NONE));
		CHECK(elapsed >= 10u);
		CHECK(elapsed <= 10u + spiPollSlack);
		CHECK(HAL_SPI_GetState(&h) == HAL_SPI_STATE_READY);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

/* ============================================================================================== SP1 (CR2 enables) */

TEST_CASE_FIXTURE(SpiFixture, "SP1 characterization: polling transfers leave TXEIE, RXNEIE and ERRIE clear in CR2") {
	// NEEDS REVIEW (proposed): RM0090 28.5.2: TXEIE bit 7, RXNEIE bit 6, ERRIE bit 5. Polling mode needs no interrupts; UM1725
	// does not say whether the polling functions leave ERRIE alone. Observed: all three clear after completion.
	SPIPOLL_INSTANCES
	constexpr uint32_t enables = (1u << 7) | (1u << 6) | (1u << 5);
	for (spiPollFn fn : {spiPollFnTx, spiPollFnRx, spiPollFnTxRx}) {
		spiPollFresh();
		CAPTURE((int)fn);
		SPI_HandleTypeDef h;
		spiPollInit(h, inst, SPI_MODE_MASTER, SPI_DIRECTION_2LINES, false);
		spiPollWordsT out;
		switch (fn) {
		case spiPollFnTx: REQUIRE(spiPollTransmit(h, spiPollWords(3, false, 0), false) == HAL_OK); break;
		case spiPollFnRx: REQUIRE(spiPollReceive(h, out, 3, false) == HAL_OK); break;
		default: REQUIRE(spiPollTxRx(h, spiPollWords(3, false, 0), out, false) == HAL_OK); break;
		}
		CHECK((HalFixture::peek(&spiPollBase(inst)->CR2) & enables) == 0u);
	}
}

/* ============================================================================================== SP-model self-tests */

namespace {
constexpr uint32_t spiPollCR1_CPHA = 1u << 0;
constexpr uint32_t spiPollCR1_BR0 = 1u << 3;
constexpr uint32_t spiPollCR1_SSI = 1u << 8;
constexpr uint32_t spiPollCR1_CRCNEXT = 1u << 12;
} // namespace

TEST_CASE_FIXTURE(SpiFixture, "SP-model changing DFF, BR or CPHA while SPE is set is counted") {
	// RM0090 28.5.1: these bits must not change while SPE = 1 (the model counts it).
	SPIPOLL_INSTANCES
	for (uint32_t bit : {spiPollCR1_DFF, spiPollCR1_BR0, spiPollCR1_CPHA}) {
		spiPollFresh();
		CAPTURE(bit);
		spiPollBase(inst)->CR1 = spiPollCR1_MSTR | spiPollCR1_SPE;
		REQUIRE(hostSpiConfigWhileEnabled(inst) == 0u);
		spiPollBase(inst)->CR1 = spiPollCR1_MSTR | spiPollCR1_SPE | bit;
		CHECK(hostSpiConfigWhileEnabled(inst) == 1u);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP-model changing BIDIOE, SSI or CRCNEXT while SPE is set is not counted") {
	// RM0090 28.5.1: these bits may change while SPE = 1.
	SPIPOLL_INSTANCES
	for (uint32_t bit : {spiPollCR1_BIDIOE, spiPollCR1_SSI, spiPollCR1_CRCNEXT}) {
		spiPollFresh();
		CAPTURE(bit);
		spiPollBase(inst)->CR1 = spiPollCR1_MSTR | spiPollCR1_SPE;
		spiPollBase(inst)->CR1 = spiPollCR1_MSTR | spiPollCR1_SPE | bit;
		CHECK(hostSpiConfigWhileEnabled(inst) == 0u);
	}
}

TEST_CASE_FIXTURE(SpiFixture, "SP-model changing the configuration with SPE clear is not counted") {
	SPIPOLL_INSTANCES
	spiPollBase(inst)->CR1 = spiPollCR1_MSTR | spiPollCR1_DFF | spiPollCR1_BR0;
	spiPollBase(inst)->CR1 = spiPollCR1_MSTR;
	CHECK(hostSpiConfigWhileEnabled(inst) == 0u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP-model a 16 bit word written to DR with DFF = 0 goes out as 8 bit") {
	// RM0090 28.5.1 DFF = 0: 8 bit frames; 28.5.4: only the low byte is used.
	SPIPOLL_INSTANCES
	spiPollBase(inst)->CR1 = spiPollCR1_MSTR | spiPollCR1_SPE;
	spiPollBase(inst)->DR = 0xABCDu;
	REQUIRE(hostSpiTxCount(inst) == 1u);
	CHECK(hostSpiTxAt(inst, 0) == 0xCDu);
}

TEST_CASE_FIXTURE(SpiFixture, "SP-model a 16 bit word written to DR with DFF = 1 goes out whole") {
	SPIPOLL_INSTANCES
	spiPollBase(inst)->CR1 = spiPollCR1_MSTR | spiPollCR1_DFF | spiPollCR1_SPE;
	spiPollBase(inst)->DR = 0xABCDu;
	REQUIRE(hostSpiTxCount(inst) == 1u);
	CHECK(hostSpiTxAt(inst, 0) == 0xABCDu);
}

TEST_CASE_FIXTURE(SpiFixture, "SP-model CR2 keeps only bits 7:4 and 2:0") {
	// RM0090 28.5.2: bit 3 and bits 15:8 are reserved and read 0.
	SPIPOLL_INSTANCES
	spiPollBase(inst)->CR2 = 0xFFFFu;
	CHECK(spiPollBase(inst)->CR2 == 0x00F7u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP-model I2SCFGR keeps its defined bits only") {
	// RM0090 28.5.8: bits 11:7 and 5:0 (bit 6 and bits 15:12 reserved): mask 0x0FBF.
	SPIPOLL_INSTANCES
	spiPollBase(inst)->I2SCFGR = 0xFFFFu;
	CHECK(spiPollBase(inst)->I2SCFGR == 0x0FBFu);
}

TEST_CASE_FIXTURE(SpiFixture, "SP-model I2SPR keeps bits 9:0 and resets to 0x0002") {
	// RM0090 28.5.9: I2SDIV 7:0, ODD 8, MCKOE 9; reset value 0000 0010.
	SPIPOLL_INSTANCES
	CHECK(spiPollBase(inst)->I2SPR == 0x0002u);
	spiPollBase(inst)->I2SPR = 0xFFFFu;
	CHECK(spiPollBase(inst)->I2SPR == 0x03FFu);
}

TEST_CASE_FIXTURE(SpiFixture, "SP-model clearing SPE while BSY is counted") {
	// RM0090 28.3.8: the disable procedure requires BSY = 0.
	SPIPOLL_INSTANCES
	hostSpiSetFrameTicks(5);
	spiPollBase(inst)->CR1 = spiPollCR1_MSTR | spiPollCR1_SPE;
	spiPollBase(inst)->DR = 0x55u; // the frame starts, BSY = 1 (no tick passes)
	REQUIRE(hostSpiDisabledBusy(inst) == 0u);
	spiPollBase(inst)->CR1 = spiPollCR1_MSTR;
	CHECK(hostSpiDisabledBusy(inst) == 1u);
}

TEST_CASE_FIXTURE(SpiFixture, "SP-model clearing SPE with BSY clear is not counted") {
	SPIPOLL_INSTANCES
	spiPollBase(inst)->CR1 = spiPollCR1_MSTR | spiPollCR1_SPE;
	spiPollBase(inst)->CR1 = spiPollCR1_MSTR;
	CHECK(hostSpiDisabledBusy(inst) == 0u);
}
