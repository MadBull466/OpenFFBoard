/*
 * P12 T8 group PQ: HAL_PCD_IRQHandler (device mode, USB_OTG_FS) with the static PCD_WriteEmptyTxFifo / PCD_EP_OutXfrComplete_int /
 * PCD_EP_OutSetupPacket_int behind it, HAL_PCD_WKUP_IRQHandler, and the LL status reads USB_Read* / USB_ClearInterrupts / USB_GetMode
 * of stm32f4xx_hal_pcd.c / stm32f4xx_ll_usb.c against the OTG_FS model (fakes/stm32_hal/HalOtg.h). Requirements PQ1-PQ5 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T8_REQUIREMENTS.md.
 *
 * Oracles: RM0090 Rev 22 chapter 34 (34.16 registers: GINTSTS / GINTMSK, GRXSTSP packet status values PKTSTS 2 / 3 / 4 / 6 (0x1 global OUT NAK),
 * DAINT / DAINTMSK, DIEPINTx / DOEPINTx bit lists, DIEPEMPMSK, DSTS SUSPSTS / ENUMSPD / FNSOF, GUSBCFG.TRDT Table 203; 34.17.5 "Endpoint
 * initialization on USB reset", SETUP transactions: the SETUP data words, the Setup stage done word and the STUP interrupt on its pop,
 * STUPCNT = 3 and XFRSIZ = 3 x 8 bytes) and UM1725 Rev 8 chapter "HAL PCD Generic Driver" (callback names and meaning). The USB 2.0 rule
 * that a device uses address 0 after reset.
 *
 * Hand derivations used below (little endian, RM0090 34.12: a FIFO word carries 4 bytes, the first byte in bits 7:0):
 *   SETUP 80 06 00 01 00 00 12 00: word0 = 0x01000680, word1 = 0x00120000.
 *   DOEPTSIZ0 after USB_EP0_OutStart: STUPCNT (bits 30:29) = 3 -> 0x60000000, PKTCNT (bit 19) = 1 -> 0x00080000, XFRSIZ = 3 x 8 = 24 = 0x18
 *     -> 0x60080018.
 *   Words of a packet of n bytes: (n + 3) / 4: 5 bytes -> 2, 64 bytes -> 16, 200 bytes as 64 + 64 + 64 + 8 -> 16 + 16 + 16 + 2 = 50.
 *   DIEPMSK after the USB reset handler (RM0090 34.17.5: XFRC, TOC; plus EPDM): XFRCM bit 0, EPDM bit 1, TOM bit 3 -> 0x0B.
 *   DOEPMSK (STUPM bit 3, XFRCM bit 0, plus EPDM bit 1): 0x0B at least; the HAL adds STSPHSRXM (bit 5) and NAKMSK (bit 13) -> 0x202B.
 *   GINTSTS 0xA5A5A5A5 & GINTMSK 0x0F0FF0F0 = 0x0505A0A0 (A5 & 0F = 05, A5 & F0 = A0).
 *   DAINT 0x000A0005 & DAINTMSK 0x000C0003 = 0x00080001: IN endpoints 0x0001, OUT endpoints 0x0008.
 *
 * NEEDS REVIEW proposals (characterization cases; the lead numbers them):
 *  NR347_a: a SETUP data entry adds its BCNT (8) to the EP0 xfer_count (the HAL does it, UM1725 / RM0090 are silent).
 *  NR347_b: IN endpoint 0 XFRC does not call USB_EP0_OutStart without DMA (the HAL gates it on dma_enable, UM1725 / RM0090 silent).
 *  NR347_c: EP0 IN transmit longer than the maximum packet size is cut to one packet by USB_EPStartXfer (xfer_len = maxpacket).
 *  NR347_d: TXFE with xfer_count > xfer_len writes nothing and leaves DIEPEMPMSK set (defensive HAL guard).
 *  NR347_e: the USB reset handler unmasks STSPHSRXM / NAKMSK in DOEPMSK in addition to the RM0090 list; dedicated EP1 mode writes
 *            the OTG_HS only registers DINEP1MSK / DOUTEP1MSK and leaves DIEPMSK / DOEPMSK alone.
 *  NR347_f: the order of the events when several flags are pending in one call.
 *  NR347_g: DISCINT has no handler branch (device mode): no callback, flag stays set.
 *  NR347_h (now V125, arguable KNOWN_ISSUE): the incomplete isochronous OUT test at stm32f4xx_hal_pcd.c:1479 compares DOEPCTL bit 16 in place with frame bit 0, so it only matches EONUM 0 on an even frame; RM0090 34.17.5 says EONUM = SOFFN[0].
 *            Characterization NR347_j: with two matching isochronous OUT endpoints one PXFR interrupt flags only the lowest.
 *  NR347_i: OTGINT is not derived from GOTGINT by the model: the cases set GINTSTS.OTGINT by poke (lead decision).
 */
#include "OtgTestSupport.h"

#include "HalPeripherals.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>

extern "C" uint32_t SystemCoreClock;

namespace {

constexpr uint32_t pqBase = HOST_OTG_BASE;
constexpr uint32_t R_GOTGINT = 0x004, R_GAHBCFG = 0x008, R_GUSBCFG = 0x00C, R_GINTSTS = 0x014, R_GINTMSK = 0x018, R_GRXSTSP = 0x020;
constexpr uint32_t R_DCFG = 0x800, R_DCTL = 0x804, R_DSTS = 0x808, R_DIEPMSK = 0x810, R_DOEPMSK = 0x814, R_DAINT = 0x818,
                   R_DAINTMSK = 0x81C, R_DIEPEMPMSK = 0x834, R_PCGCCTL = 0xE00;

uint32_t pqRd(uint32_t off) { return hostRegsPeek(pqBase + off); }
void pqWr(uint32_t off, uint32_t v) { hostRegsPoke(pqBase + off, v); }
void pqOr(uint32_t off, uint32_t b) { pqWr(off, pqRd(off) | b); }
void pqStore(uint32_t off, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(static_cast<std::uintptr_t>(pqBase + off)) = v; }
// Makes the model recompute DAINT / GINTSTS after a poke.
void pqTouch() { hostOtgRefresh(); }

uint32_t inCtl(unsigned n) { return 0x900u + 0x20u * n; }
uint32_t inInt(unsigned n) { return inCtl(n) + 8u; }
uint32_t outCtl(unsigned n) { return 0xB00u + 0x20u * n; }
uint32_t outInt(unsigned n) { return outCtl(n) + 8u; }
uint32_t outTsiz(unsigned n) { return outCtl(n) + 0x10u; }

constexpr uint32_t GI_SOF = 1u << 3, GI_USBSUSP = 1u << 11, GI_USBRST = 1u << 12, GI_ENUMDNE = 1u << 13, GI_OTGINT = 1u << 2,
                   GI_IISOIXFR = 1u << 20, GI_PXFR = 1u << 21, GI_DISCINT = 1u << 29, GI_SRQINT = 1u << 30, GI_WKUPINT = 1u << 31,
                   GI_MMIS = 1u << 1, GI_GONAKEFF = 1u << 7, GI_RXFLVL = 1u << 4;
constexpr uint32_t D_XFRC = 1u << 0, D_EPDISD = 1u << 1, D_STUP = 1u << 3, D_TOC = 1u << 3, D_OTEPDIS = 1u << 4, D_ITTXFE = 1u << 4,
                   D_STSPHSRX = 1u << 5, D_INEPNE = 1u << 6, D_NAK = 1u << 13;
constexpr uint32_t C_EPENA = 1u << 31, C_STALL = 1u << 21, C_NAKSTS = 1u << 17;

struct PqRig : OtgFixture {
	uint8_t rx[4][600];
	uint8_t tx[600];  // V118: USB_WritePacket over-reads up to 3 bytes: the source is a 600 byte buffer, transmitted lengths stay below 524
	PqRig() {
		std::memset(rx, 0xEE, sizeof rx);
		std::memset(tx, 0, sizeof tx);
		for (unsigned i = 0; i < 520; ++i) tx[i] = static_cast<uint8_t>(i * 3u + 1u);
	}
	// Firmware bring-up, USB reset and enumeration done, then all four endpoints open (mps 64). Callback counters are cleared at the end.
	void up(unsigned tx1Words = 0x40) {
		REQUIRE(init() == HAL_OK);
		REQUIRE(HAL_PCDEx_SetRxFiFo(&h, 0x40) == HAL_OK);
		REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 0, 0x20) == HAL_OK);
		REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 1, static_cast<uint16_t>(tx1Words)) == HAL_OK);
		REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 2, 0x40) == HAL_OK);
		REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 3, 0x40) == HAL_OK);
		REQUIRE(HAL_PCD_Start(&h) == HAL_OK);
		hostOtgUsbReset();
		drain();
		hostOtgEnumDone();
		drain();
		REQUIRE(HAL_PCD_EP_Open(&h, 0x80, 64, EP_TYPE_CTRL) == HAL_OK);
		REQUIRE(HAL_PCD_EP_Open(&h, 0x00, 64, EP_TYPE_CTRL) == HAL_OK);
		for (unsigned e = 1; e <= 3; ++e) {
			REQUIRE(HAL_PCD_EP_Open(&h, static_cast<uint8_t>(0x80u | e), 64, EP_TYPE_BULK) == HAL_OK);
			REQUIRE(HAL_PCD_EP_Open(&h, static_cast<uint8_t>(e), 64, EP_TYPE_BULK) == HAL_OK);
		}
		otgSupportReset();
	}
	void irq() { HAL_PCD_IRQHandler(&h); }
	// Host sends one OUT packet of len bytes of the test pattern (offset 0).
	HostOtgHandshake hostOut(unsigned ep, unsigned len, int pid1 = 0) { return hostOtgHostOut(ep, len ? tx : nullptr, len, pid1); }
	// The host reads IN packets until it is NAKed; returns the number of bytes, packets in pkts.
	unsigned hostReadAll(unsigned ep, uint8_t *out, unsigned *pkts) {
		unsigned total = 0, n = 0;
		*pkts = 0;
		while (hostOtgHostIn(ep, out + total, 64, &n) == HOSTOTG_ACK) {
			total += n;
			++*pkts;
		}
		return total;
	}
};

}  // namespace

/* ------------------------------------------------------------------------------------------------ PQ1 */

TEST_CASE_FIXTURE(PqRig, "PQ1 USB_GetMode reads device mode after bring-up and host mode when GINTSTS.CMOD is set") {
	up();
	CHECK(USB_GetMode(h.Instance) == USB_OTG_MODE_DEVICE);
	pqOr(R_GINTSTS, 1u);
	CHECK(USB_GetMode(h.Instance) == USB_OTG_MODE_HOST);
}

TEST_CASE_FIXTURE(PqRig, "PQ1 the handler does nothing in host mode and leaves the flags pending") {
	up();
	pqOr(R_GINTSTS, 1u);  // CMOD = 1: host mode
	hostOtgUsbReset();
	uint32_t before = pqRd(R_GINTSTS);
	irq();
	CHECK(otgCounts.orderCount == 0);
	CHECK((pqRd(R_GINTSTS) & GI_USBRST) == GI_USBRST);
	CHECK(pqRd(R_GINTSTS) == before);
}

TEST_CASE_FIXTURE(PqRig, "PQ1 in host mode the receive FIFO status entries are not popped") {
	up();
	const uint8_t setup[8] = {0x80, 6, 0, 1, 0, 0, 18, 0};
	REQUIRE(hostOtgHostSetup(setup) == HOSTOTG_ACK);
	unsigned entries = hostOtgRxEntries();
	pqOr(R_GINTSTS, 1u);
	irq();
	CHECK(hostOtgRxEntries() == entries);
	CHECK(otgCounts.setup == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ1 a pending mode mismatch interrupt that is unmasked is acknowledged") {
	up();
	pqOr(R_GINTMSK, GI_MMIS);
	pqOr(R_GINTSTS, GI_MMIS);
	irq();
	CHECK((pqRd(R_GINTSTS) & GI_MMIS) == 0);
	CHECK(otgCounts.orderCount == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ1 a mode mismatch flag whose mask bit is clear is left alone (spurious interrupt)") {
	up();
	pqOr(R_GINTSTS, GI_MMIS);
	irq();
	CHECK((pqRd(R_GINTSTS) & GI_MMIS) == GI_MMIS);
}

TEST_CASE_FIXTURE(PqRig, "PQ1 a spurious interrupt does not update FrameNumber") {
	up();
	h.FrameNumber = 0x1234;
	hostOtgSof(7);  // SOF is not unmasked (Sof_enable off): no valid interrupt
	irq();
	CHECK(h.FrameNumber == 0x1234);
	CHECK(otgCounts.sof == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ1 a valid interrupt stores DSTS.FNSOF (bits 21:8) in FrameNumber") {
	up();
	hostOtgSof(0x2A5);
	hostOtgUsbReset();
	irq();
	CHECK(h.FrameNumber == 0x2A5u);
}

/* ------------------------------------------------------------------------------------------------ PQ2 */

#define PQ_LENS_EP(ep, fn) \
	SUBCASE("len 1") fn(ep, 1); \
	SUBCASE("len 2") fn(ep, 2); \
	SUBCASE("len 3") fn(ep, 3); \
	SUBCASE("len 4") fn(ep, 4); \
	SUBCASE("len 5") fn(ep, 5); \
	SUBCASE("len 63") fn(ep, 63); \
	SUBCASE("len 64") fn(ep, 64);

TEST_CASE_FIXTURE(PqRig, "PQ2 an OUT data packet is stored in the buffer, counted, and the buffer pointer advances (all endpoints and tail lengths)") {
	auto run = [&](unsigned ep, unsigned len) {
		up();
		REQUIRE(HAL_PCD_EP_Receive(&h, static_cast<uint8_t>(ep), rx[ep], 64) == HAL_OK);
		REQUIRE(hostOut(ep, len) == HOSTOTG_ACK);
		drain();
		CHECK(HAL_PCD_EP_GetRxCount(&h, static_cast<uint8_t>(ep)) == len);
		CHECK(h.OUT_ep[ep].xfer_buff == rx[ep] + len);
		CHECK(std::memcmp(rx[ep], tx, len) == 0);
		CHECK(rx[ep][len] == 0xEE);  // bytes beyond BCNT are not written
		CHECK(otgCounts.dataOut == 1);
		CHECK(otgCounts.lastEp == ep);
	};
	SUBCASE("ep 1") { PQ_LENS_EP(1, run) }
	SUBCASE("ep 2") { PQ_LENS_EP(2, run) }
	SUBCASE("ep 3") { PQ_LENS_EP(3, run) }
}

TEST_CASE_FIXTURE(PqRig, "PQ2 an OUT transfer of 65 bytes arrives as 64 + 1: the count accumulates and the callback runs once at the end") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx[1], 65) == HAL_OK);
	REQUIRE(hostOut(1, 64) == HOSTOTG_ACK);
	drain();
	CHECK(HAL_PCD_EP_GetRxCount(&h, 1) == 64);
	CHECK(otgCounts.dataOut == 0);
	REQUIRE(hostOtgHostOut(1, tx + 64, 1, 1) == HOSTOTG_ACK);
	drain();
	CHECK(HAL_PCD_EP_GetRxCount(&h, 1) == 65);
	CHECK(h.OUT_ep[1].xfer_buff == rx[1] + 65);
	CHECK(std::memcmp(rx[1], tx, 65) == 0);
	CHECK(otgCounts.dataOut == 1);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 an OUT transfer of 130 bytes arrives as 64 + 64 + 2 with one callback") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 2, rx[2], 130) == HAL_OK);
	REQUIRE(hostOut(2, 64) == HOSTOTG_ACK);
	drain();
	REQUIRE(hostOtgHostOut(2, tx + 64, 64, 1) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 0);
	REQUIRE(hostOtgHostOut(2, tx + 128, 2, 0) == HOSTOTG_ACK);
	drain();
	CHECK(HAL_PCD_EP_GetRxCount(&h, 2) == 130);
	CHECK(std::memcmp(rx[2], tx, 130) == 0);
	CHECK(otgCounts.dataOut == 1);
	CHECK(otgCounts.lastEp == 2);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 an OUT transfer of exactly two full packets completes after the second one") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 3, rx[3], 128) == HAL_OK);
	REQUIRE(hostOut(3, 64) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 0);
	REQUIRE(hostOtgHostOut(3, tx + 64, 64, 1) == HOSTOTG_ACK);
	drain();
	CHECK(HAL_PCD_EP_GetRxCount(&h, 3) == 128);
	CHECK(otgCounts.dataOut == 1);
	CHECK(std::memcmp(rx[3], tx, 128) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 a zero length OUT packet reads no data and completes with a callback") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx[1], 0) == HAL_OK);
	REQUIRE(hostOtgHostOut(1, nullptr, 0, 0) == HOSTOTG_ACK);
	drain();
	CHECK(HAL_PCD_EP_GetRxCount(&h, 1) == 0);
	CHECK(otgCounts.dataOut == 1);
	CHECK(rx[1][0] == 0xEE);
	CHECK(hostOtgFifoUnderflows() == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 RXFLVL is masked before the status entry is popped and unmasked after the data is read") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx[1], 64) == HAL_OK);
	REQUIRE(hostOut(1, 5) == HOSTOTG_ACK);
	hostRegsLogClear();
	irq();
	long firstMaskWrite = -1, pop = -1, lastMaskWrite = -1;
	for (std::size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *a = hostRegsLogAt(i);
		if (a->addr == pqBase + R_GINTMSK && a->write) {
			if (firstMaskWrite < 0) firstMaskWrite = static_cast<long>(i);
			lastMaskWrite = static_cast<long>(i);
		}
		if (a->addr == pqBase + R_GRXSTSP && !a->write && pop < 0) pop = static_cast<long>(i);
	}
	REQUIRE(firstMaskWrite >= 0);
	REQUIRE(pop >= 0);
	CHECK(firstMaskWrite < pop);
	CHECK(pop < lastMaskWrite);
	CHECK((hostRegsLogAt(static_cast<std::size_t>(firstMaskWrite))->value & GI_RXFLVL) == 0);
	CHECK((hostRegsLogAt(static_cast<std::size_t>(lastMaskWrite))->value & GI_RXFLVL) == GI_RXFLVL);
	CHECK((pqRd(R_GINTMSK) & GI_RXFLVL) == GI_RXFLVL);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 one status entry is popped per handler call") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx[1], 64) == HAL_OK);
	REQUIRE(hostOut(1, 5) == HOSTOTG_ACK);
	REQUIRE(hostOtgRxEntries() == 2);  // the data packet entry and the transfer completed entry
	irq();
	CHECK(hostOtgRxEntries() == 1);
	CHECK(hostOtgRxWordsUsed() == 1);  // data words were read, the completed entry is still queued
	irq();
	CHECK(hostOtgRxEntries() == 0);
	CHECK(hostOtgRxWordsUsed() == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 the OUT transfer completed entry raises XFRC when popped and the callback runs in that call") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx[1], 64) == HAL_OK);
	REQUIRE(hostOut(1, 5) == HOSTOTG_ACK);
	irq();
	CHECK(otgCounts.dataOut == 0);
	CHECK((pqRd(outInt(1)) & D_XFRC) == 0);
	irq();
	CHECK(otgCounts.dataOut == 1);
	CHECK((pqRd(outInt(1)) & D_XFRC) == 0);  // the handler cleared it
	CHECK(otgCounts.lastEp == 1);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 a SETUP packet is copied into Setup[] and the setup callback runs after the completed entry") {
	up();
	const uint8_t setup[8] = {0x80, 6, 0, 1, 0, 0, 0x12, 0};
	REQUIRE(hostOtgHostSetup(setup) == HOSTOTG_ACK);
	irq();  // pops the SETUP data entry and reads the two words
	CHECK(h.Setup[0] == 0x01000680u);
	CHECK(h.Setup[1] == 0x00120000u);
	CHECK(otgCounts.setup == 0);
	irq();  // pops the Setup stage done word: STUP
	CHECK(otgCounts.setup == 1);
	CHECK(otgCounts.setupWords[0] == 0x01000680u);
	CHECK(otgCounts.setupWords[1] == 0x00120000u);
	CHECK((pqRd(outInt(0)) & D_STUP) == 0);
	CHECK(hostOtgRxWordsUsed() == 0);
}

// NEEDS REVIEW 417
TEST_CASE_FIXTURE(PqRig, "characterization: PQ2 SETUP data adds its byte count (8) to the EP0 xfer_count") {
	up();
	const uint8_t setup[8] = {0x00, 5, 7, 0, 0, 0, 0, 0};
	REQUIRE(hostOtgHostSetup(setup) == HOSTOTG_ACK);
	drain();
	CHECK(h.OUT_ep[0].xfer_count == 8);
	REQUIRE(hostOtgHostSetup(setup) == HOSTOTG_ACK);
	drain();
	CHECK(h.OUT_ep[0].xfer_count == 16);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 the status OUT stage (zero length on EP0) restarts the SETUP reception with STUPCNT 3, PKTCNT 1, XFRSIZ 24") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 0, rx[0], 0) == HAL_OK);
	REQUIRE(hostOtgHostOut(0, nullptr, 0, 1) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 1);
	CHECK(otgCounts.lastEp == 0);
	CHECK(pqRd(outTsiz(0)) == 0x60080018u);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 an EP0 OUT data stage with data does not restart the SETUP reception") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 0, rx[0], 8) == HAL_OK);
	REQUIRE(hostOtgHostOut(0, tx, 8, 1) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 1);
	CHECK(otgCounts.lastEp == 0);
	CHECK(std::memcmp(rx[0], tx, 8) == 0);
	CHECK(((pqRd(outTsiz(0)) >> 19) & 1u) == 0);  // PKTCNT used up by the packet: USB_EP0_OutStart would have set it to 1
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0) == 8);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 status entries without an action (global OUT NAK, reserved values) pop without reading data or calling back") {
	auto run = [&](unsigned p) {
		up();
		REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx[1], 64) == HAL_OK);
		hostOtgRxFill((p << 17) | 1u, 1);
		irq();
		CHECK(hostOtgRxWordsUsed() == 0);
		CHECK(hostOtgFifoUnderflows() == 0);
		CHECK(otgCounts.orderCount == 0);
		CHECK(h.OUT_ep[1].xfer_count == 0);
		CHECK((pqRd(R_GINTMSK) & GI_RXFLVL) == GI_RXFLVL);
	};
	SUBCASE("pktsts 0") run(0);
	SUBCASE("pktsts 1") run(1);
	SUBCASE("pktsts 5") run(5);
	SUBCASE("pktsts 7") run(7);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 an OUT complete or SETUP complete entry on its own (no XFRC / STUP raised) causes no callback") {
	up();
	hostOtgRxFill((3u << 17) | 1u, 1);  // PKTSTS 3: OUT transfer completed, EP1
	hostOtgRxFill((4u << 17) | 0u, 1);  // PKTSTS 4: SETUP transaction completed, EP0
	drain();
	CHECK(otgCounts.orderCount == 0);
	CHECK(hostOtgRxWordsUsed() == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 an OUT data entry with a byte count of 0 reads nothing and counts nothing") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx[1], 64) == HAL_OK);
	hostOtgRxFill((2u << 17) | (0u << 4) | 1u, 1);
	irq();
	CHECK(h.OUT_ep[1].xfer_count == 0);
	CHECK(h.OUT_ep[1].xfer_buff == rx[1]);
	CHECK(hostOtgFifoUnderflows() == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ2 an OUT data entry is routed to the endpoint named in EPNUM") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 3, rx[3], 64) == HAL_OK);
	hostOtgRxFill((2u << 17) | (3u << 4) | 3u, 1);
	hostOtgRxFill(0x00030201u, 1);
	irq();
	CHECK(h.OUT_ep[3].xfer_count == 3);
	CHECK(rx[3][0] == 1);
	CHECK(rx[3][1] == 2);
	CHECK(rx[3][2] == 3);
	CHECK(rx[3][3] == 0xEE);
	CHECK(h.OUT_ep[1].xfer_count == 0);
}

/* ------------------------------------------------------------------------------------------------ PQ3 OUT */

TEST_CASE_FIXTURE(PqRig, "PQ3 OUT XFRC on each endpoint gives the data out callback with that endpoint number and is cleared") {
	auto run = [&](unsigned ep) {
		up();
		pqOr(outInt(ep), D_XFRC);
		pqTouch();
		REQUIRE(hostOtgIrqPending());
		CHECK(drain() == 1);
		CHECK(otgCounts.dataOut == 1);
		CHECK(otgCounts.lastEp == ep);
		CHECK((pqRd(outInt(ep)) & D_XFRC) == 0);
	};
	SUBCASE("ep 0") run(0);
	SUBCASE("ep 1") run(1);
	SUBCASE("ep 2") run(2);
	SUBCASE("ep 3") run(3);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 OUT interrupt bits of several endpoints are served in one call lowest endpoint first") {
	up();
	pqOr(outInt(3), D_XFRC);
	pqOr(outInt(1), D_XFRC);
	pqOr(outInt(2), D_XFRC);
	pqTouch();
	CHECK(drain() == 1);
	REQUIRE(otgCounts.orderCount == 3);
	CHECK(otgCounts.epOrder[0] == 1);
	CHECK(otgCounts.epOrder[1] == 2);
	CHECK(otgCounts.epOrder[2] == 3);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 OUT STUP gives the setup callback and is cleared") {
	up();
	pqOr(outInt(0), D_STUP);
	pqTouch();
	drain();
	CHECK(otgCounts.setup == 1);
	CHECK((pqRd(outInt(0)) & D_STUP) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 OUT XFRC and STUP together run the data out and the setup callbacks in that order") {
	up();
	pqOr(outInt(0), D_XFRC | D_STUP);
	pqTouch();
	drain();
	REQUIRE(otgCounts.orderCount == 2);
	CHECK(otgCounts.order[0] == OTG_CB_DATA_OUT);
	CHECK(otgCounts.order[1] == OTG_CB_SETUP);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 OUT bits OTEPDIS, STSPHSRX and NAK are cleared without a callback when they are unmasked") {
	up();
	pqOr(R_DOEPMSK, D_OTEPDIS);
	pqOr(outInt(1), D_OTEPDIS | D_STSPHSRX | D_NAK);
	pqTouch();
	REQUIRE(hostOtgIrqPending());
	drain();
	CHECK((pqRd(outInt(1)) & (D_OTEPDIS | D_STSPHSRX | D_NAK)) == 0);
	CHECK(otgCounts.orderCount == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 an OUT interrupt bit whose DOEPMSK bit is clear raises no interrupt and stays set") {
	up();
	pqOr(outInt(1), D_OTEPDIS);  // OTEPDM is not unmasked by the HAL
	pqTouch();
	CHECK(hostOtgIrqPending() == 0);
	irq();
	CHECK((pqRd(outInt(1)) & D_OTEPDIS) == D_OTEPDIS);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 OUT EPDISD is cleared with no callback for a non isochronous endpoint") {
	up();
	pqOr(outInt(2), D_EPDISD);
	pqTouch();
	drain();
	CHECK((pqRd(outInt(2)) & D_EPDISD) == 0);
	CHECK(otgCounts.orderCount == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 OUT EPDISD with global OUT NAK effective clears it with CGONAK") {
	up();
	pqStore(R_DCTL, pqRd(R_DCTL) | (1u << 9));  // SGONAK
	REQUIRE((pqRd(R_GINTSTS) & GI_GONAKEFF) == GI_GONAKEFF);
	pqOr(outInt(1), D_EPDISD);
	pqTouch();
	drain();
	CHECK((pqRd(R_GINTSTS) & GI_GONAKEFF) == 0);
	CHECK((pqRd(R_DCTL) & (1u << 3)) == 0);  // GONSTS
	CHECK((pqRd(outInt(1)) & D_EPDISD) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 OUT EPDISD of an endpoint flagged iso incomplete gives the ISO OUT incomplete callback once") {
	up();
	h.OUT_ep[2].is_iso_incomplete = 1;
	pqOr(outInt(2), D_EPDISD);
	pqTouch();
	drain();
	CHECK(otgCounts.isoOut == 1);
	CHECK(otgCounts.lastEp == 2);
	CHECK(h.OUT_ep[2].is_iso_incomplete == 0);
	pqOr(outInt(2), D_EPDISD);
	pqTouch();
	drain();
	CHECK(otgCounts.isoOut == 1);
}

/* ------------------------------------------------------------------------------------------------ PQ3 IN */

TEST_CASE_FIXTURE(PqRig, "PQ3 IN XFRC on each endpoint gives the data in callback with that endpoint number and is cleared") {
	auto run = [&](unsigned ep) {
		up();
		pqOr(inInt(ep), D_XFRC);
		pqTouch();
		REQUIRE(hostOtgIrqPending());
		CHECK(drain() == 1);
		CHECK(otgCounts.dataIn == 1);
		CHECK(otgCounts.lastEp == ep);
		CHECK((pqRd(inInt(ep)) & D_XFRC) == 0);
	};
	SUBCASE("ep 0") run(0);
	SUBCASE("ep 1") run(1);
	SUBCASE("ep 2") run(2);
	SUBCASE("ep 3") run(3);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 IN interrupts of several endpoints are served in one call lowest endpoint first") {
	up();
	pqOr(inInt(3), D_XFRC);
	pqOr(inInt(0), D_XFRC);
	pqOr(inInt(2), D_XFRC);
	pqTouch();
	CHECK(drain() == 1);
	REQUIRE(otgCounts.orderCount == 3);
	CHECK(otgCounts.epOrder[0] == 0);
	CHECK(otgCounts.epOrder[1] == 2);
	CHECK(otgCounts.epOrder[2] == 3);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 an IN interrupt of an endpoint masked in DAINTMSK is not served") {
	up();
	pqWr(R_DAINTMSK, pqRd(R_DAINTMSK) & ~2u);
	pqOr(inInt(1), D_XFRC);
	pqTouch();
	CHECK(hostOtgIrqPending() == 0);
	irq();
	CHECK(otgCounts.dataIn == 0);
	CHECK((pqRd(inInt(1)) & D_XFRC) == D_XFRC);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 IN XFRC clears the endpoint's DIEPEMPMSK bit") {
	up();
	pqOr(R_DIEPEMPMSK, 1u << 2);
	pqOr(inInt(2), D_XFRC);
	pqTouch();
	drain();
	CHECK((pqRd(R_DIEPEMPMSK) & (1u << 2)) == 0);
	CHECK(otgCounts.dataIn == 1);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 IN TOC, ITTXFE and INEPNE are cleared with no callback") {
	up();
	pqOr(R_DIEPMSK, D_ITTXFE | D_INEPNE);
	pqOr(inInt(1), D_TOC | D_ITTXFE | D_INEPNE);
	pqTouch();
	REQUIRE(hostOtgIrqPending());
	drain();
	CHECK((pqRd(inInt(1)) & (D_TOC | D_ITTXFE | D_INEPNE)) == 0);
	CHECK(otgCounts.orderCount == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 IN ITTXFE and INEPNE whose DIEPMSK bits are clear raise no interrupt and stay set") {
	up();
	pqOr(inInt(1), D_ITTXFE | D_INEPNE);
	pqTouch();
	CHECK(hostOtgIrqPending() == 0);
	irq();
	CHECK((pqRd(inInt(1)) & (D_ITTXFE | D_INEPNE)) == (D_ITTXFE | D_INEPNE));
}

TEST_CASE_FIXTURE(PqRig, "PQ3 IN EPDISD flushes only that endpoint's TxFIFO and is cleared") {
	up();
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, tx, 5) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x82, tx, 8) == HAL_OK);
	drain();
	REQUIRE(hostOtgTxWordsUsed(1) == 2);
	REQUIRE(hostOtgTxWordsUsed(2) == 2);
	pqOr(inInt(1), D_EPDISD);
	pqTouch();
	drain();
	CHECK(hostOtgTxWordsUsed(1) == 0);
	CHECK(hostOtgTxWordsUsed(2) == 2);
	CHECK((pqRd(inInt(1)) & D_EPDISD) == 0);
	CHECK(otgCounts.orderCount == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 IN EPDISD of an endpoint flagged iso incomplete gives the ISO IN incomplete callback once") {
	up();
	h.IN_ep[3].is_iso_incomplete = 1;
	pqOr(inInt(3), D_EPDISD);
	pqTouch();
	drain();
	CHECK(otgCounts.isoIn == 1);
	CHECK(otgCounts.lastEp == 3);
	CHECK(h.IN_ep[3].is_iso_incomplete == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 a bulk IN transfer of 5 bytes: the FIFO is filled by the TXFE interrupt, the data in callback follows the host's read") {
	up();
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, tx, 5) == HAL_OK);
	REQUIRE(hostOtgIrqPending());
	drain();
	CHECK(hostOtgTxWordsUsed(1) == 2);
	CHECK(h.IN_ep[1].xfer_count == 5);
	CHECK((pqRd(R_DIEPEMPMSK) & 2u) == 0);
	CHECK(otgCounts.dataIn == 0);
	uint8_t buf[64];
	unsigned n = 0;
	REQUIRE(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_ACK);
	CHECK(n == 5);
	CHECK(std::memcmp(buf, tx, 5) == 0);
	drain();
	CHECK(otgCounts.dataIn == 1);
	CHECK(otgCounts.lastEp == 1);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 TXFE writes the first packet words little endian and leaves DIEPEMPMSK clear when the data is complete") {
	up();
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, tx, 4) == HAL_OK);
	drain();
	REQUIRE(hostOtgTxWordsUsed(1) == 1);
	const uint32_t expect = static_cast<uint32_t>(tx[0]) | (static_cast<uint32_t>(tx[1]) << 8) | (static_cast<uint32_t>(tx[2]) << 16) |
	                        (static_cast<uint32_t>(tx[3]) << 24);
	CHECK(hostOtgTxWords(1)[0] == expect);
	CHECK((pqRd(R_DIEPEMPMSK) & 2u) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 a transfer of 200 bytes (64 + 64 + 64 + 8) is written in one TXFE call: 50 words, count and pointer advanced") {
	up();
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, tx, 200) == HAL_OK);
	CHECK(drain() == 1);
	CHECK(hostOtgTxWordsUsed(1) == 50);
	CHECK(h.IN_ep[1].xfer_count == 200);
	CHECK(h.IN_ep[1].xfer_buff == tx + 200);
	CHECK((pqRd(R_DIEPEMPMSK) & 2u) == 0);
	uint8_t out[256];
	unsigned pkts = 0;
	CHECK(hostReadAll(1, out, &pkts) == 200);
	CHECK(pkts == 4);
	CHECK(std::memcmp(out, tx, 200) == 0);
	drain();
	CHECK(otgCounts.dataIn == 1);
}

#define PQ_LENS_IN(ep, fn) \
	SUBCASE("len 1") fn(ep, 1); \
	SUBCASE("len 63") fn(ep, 63); \
	SUBCASE("len 64") fn(ep, 64); \
	SUBCASE("len 65") fn(ep, 65);

TEST_CASE_FIXTURE(PqRig, "PQ3 bulk IN transfers of 1, 63, 64 and 65 bytes on EP1-3 deliver every byte and one data in callback") {
	auto run = [&](unsigned ep, unsigned len) {
		up();
		REQUIRE(HAL_PCD_EP_Transmit(&h, static_cast<uint8_t>(0x80u | ep), tx, len) == HAL_OK);
		drain();
		CHECK(h.IN_ep[ep].xfer_count == len);
		uint8_t out[200];
		unsigned pkts = 0;
		CHECK(hostReadAll(ep, out, &pkts) == len);
		CHECK(pkts == (len + 63) / 64);
		CHECK(std::memcmp(out, tx, len) == 0);
		drain();
		CHECK(otgCounts.dataIn == 1);
		CHECK(otgCounts.lastEp == ep);
	};
	SUBCASE("ep 1") { PQ_LENS_IN(1, run) }
	SUBCASE("ep 2") { PQ_LENS_IN(2, run) }
	SUBCASE("ep 3") { PQ_LENS_IN(3, run) }
}

TEST_CASE_FIXTURE(PqRig, "PQ3 a short TxFIFO (16 words) takes one 64 byte packet per TXFE call: partial writes keep the count and the mask") {
	up(0x10);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, tx, 130) == HAL_OK);
	drain();
	CHECK(hostOtgTxWordsUsed(1) == 16);
	CHECK(h.IN_ep[1].xfer_count == 64);
	CHECK((pqRd(R_DIEPEMPMSK) & 2u) == 2u);
	uint8_t buf[64];
	unsigned n = 0;
	REQUIRE(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_ACK);
	CHECK(n == 64);
	CHECK(std::memcmp(buf, tx, 64) == 0);
	drain();
	CHECK(hostOtgTxWordsUsed(1) == 16);
	CHECK(h.IN_ep[1].xfer_count == 128);
	CHECK((pqRd(R_DIEPEMPMSK) & 2u) == 2u);
	REQUIRE(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_ACK);
	CHECK(std::memcmp(buf, tx + 64, 64) == 0);
	drain();
	CHECK(hostOtgTxWordsUsed(1) == 1);
	CHECK(h.IN_ep[1].xfer_count == 130);
	CHECK((pqRd(R_DIEPEMPMSK) & 2u) == 0);
	REQUIRE(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_ACK);
	CHECK(n == 2);
	CHECK(std::memcmp(buf, tx + 128, 2) == 0);
	CHECK(otgCounts.dataIn == 0);
	drain();
	CHECK(otgCounts.dataIn == 1);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 a TxFIFO smaller than one packet (8 words for a 64 byte packet) gets no write and keeps DIEPEMPMSK set") {
	up(8);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, tx, 64) == HAL_OK);
	irq();
	CHECK(hostOtgTxWordsUsed(1) == 0);
	CHECK(h.IN_ep[1].xfer_count == 0);
	CHECK((pqRd(R_DIEPEMPMSK) & 2u) == 2u);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 a FIFO with room for a short last packet but not a full one writes the short packet only") {
	up(8);  // 8 words = 32 bytes
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, tx, 30) == HAL_OK);  // one 30 byte packet = 8 words
	irq();
	CHECK(hostOtgTxWordsUsed(1) == 8);
	CHECK(h.IN_ep[1].xfer_count == 30);
	CHECK((pqRd(R_DIEPEMPMSK) & 2u) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 odd length transfers write (len + 3) / 4 words per packet") {
	auto run = [&](unsigned len) {
		up();
		REQUIRE(HAL_PCD_EP_Transmit(&h, 0x82, tx, len) == HAL_OK);
		drain();
		CHECK(hostOtgTxWordsUsed(2) == (len + 3) / 4);
		CHECK(h.IN_ep[2].xfer_count == len);
	};
	SUBCASE("len 1") run(1);
	SUBCASE("len 2") run(2);
	SUBCASE("len 3") run(3);
	SUBCASE("len 5") run(5);
	SUBCASE("len 6") run(6);
	SUBCASE("len 7") run(7);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 EP0 IN data of 18 bytes at maximum packet size 64 is written as one packet of 5 words") {
	up();
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x80, tx, 18) == HAL_OK);
	drain();
	CHECK(hostOtgTxWordsUsed(0) == 5);
	CHECK(h.IN_ep[0].xfer_count == 18);
	CHECK((pqRd(R_DIEPEMPMSK) & 1u) == 0);
	uint8_t buf[64];
	unsigned n = 0;
	REQUIRE(hostOtgHostIn(0, buf, 64, &n) == HOSTOTG_ACK);
	CHECK(n == 18);
	CHECK(std::memcmp(buf, tx, 18) == 0);
	drain();
	CHECK(otgCounts.dataIn == 1);
	CHECK(otgCounts.lastEp == 0);
}

// NEEDS REVIEW 417
TEST_CASE_FIXTURE(PqRig, "characterization: PQ3 EP0 IN data longer than the maximum packet size (8) is cut to one packet") {
	up();
	REQUIRE(HAL_PCD_EP_Open(&h, 0x80, 8, EP_TYPE_CTRL) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x80, tx, 18) == HAL_OK);
	drain();
	CHECK(h.IN_ep[0].xfer_len == 8);
	CHECK(h.IN_ep[0].xfer_count == 8);
	CHECK(hostOtgTxWordsUsed(0) == 2);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 TXFE for an endpoint with no transfer (xfer_len 0) writes nothing and clears its DIEPEMPMSK bit") {
	up();
	pqOr(R_DIEPEMPMSK, 1u << 2);
	pqTouch();
	REQUIRE(hostOtgIrqPending());
	drain();
	CHECK(hostOtgTxWordsUsed(2) == 0);
	CHECK((pqRd(R_DIEPEMPMSK) & (1u << 2)) == 0);
}

// NEEDS REVIEW 347
TEST_CASE_FIXTURE(PqRig, "characterization: PQ3 TXFE with xfer_count beyond xfer_len writes nothing and leaves DIEPEMPMSK set") {
	up();
	h.IN_ep[1].xfer_len = 5;
	h.IN_ep[1].xfer_count = 10;
	pqOr(R_DIEPEMPMSK, 2u);
	pqTouch();
	irq();
	CHECK(hostOtgTxWordsUsed(1) == 0);
	CHECK((pqRd(R_DIEPEMPMSK) & 2u) == 2u);
	CHECK(h.IN_ep[1].xfer_count == 10);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 the ISO IN incomplete flag aborts an enabled isochronous IN endpoint and reports it once through the callback") {
	up();
	h.IN_ep[2].type = EP_TYPE_ISOC;
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x82, tx, 10) == HAL_OK);
	REQUIRE((pqRd(inCtl(2)) & C_EPENA) == C_EPENA);
	pqOr(R_GINTSTS, GI_IISOIXFR);
	drain();
	CHECK((pqRd(R_GINTSTS) & GI_IISOIXFR) == 0);
	CHECK((pqRd(inCtl(2)) & C_EPENA) == 0);
	CHECK(otgCounts.isoIn == 1);
	CHECK(otgCounts.lastEp == 2);
	CHECK(h.IN_ep[2].is_iso_incomplete == 0);
	CHECK(hostOtgTxWordsUsed(2) == 0);  // flushed by the EPDISD branch
}

TEST_CASE_FIXTURE(PqRig, "PQ3 the ISO IN incomplete flag leaves non isochronous and disabled endpoints alone") {
	up();
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, tx, 200) == HAL_OK);  // bulk, enabled
	h.IN_ep[3].type = EP_TYPE_ISOC;                             // isochronous but not enabled
	pqOr(R_GINTSTS, GI_IISOIXFR);
	drain();
	CHECK(h.IN_ep[1].is_iso_incomplete == 0);
	CHECK(h.IN_ep[3].is_iso_incomplete == 0);
	CHECK((pqRd(inCtl(1)) & C_EPENA) == C_EPENA);
	CHECK(otgCounts.isoIn == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 an incomplete ISO OUT with EONUM 0 on an even frame runs the global OUT NAK / abort / callback chain") {
	up();
	h.OUT_ep[2].type = EP_TYPE_ISOC;
	REQUIRE(HAL_PCD_EP_Receive(&h, 2, rx[2], 64) == HAL_OK);
	REQUIRE((pqRd(outCtl(2)) & C_EPENA) == C_EPENA);
	hostOtgSof(4);  // frame 4: even
	pqOr(R_GINTSTS, GI_PXFR);
	drain();
	CHECK((pqRd(R_GINTSTS) & GI_PXFR) == 0);
	CHECK(otgCounts.isoOut == 1);
	CHECK(otgCounts.lastEp == 2);
	CHECK(h.OUT_ep[2].is_iso_incomplete == 0);
	CHECK((pqRd(outCtl(2)) & C_EPENA) == 0);
	CHECK((pqRd(R_GINTSTS) & GI_GONAKEFF) == 0);  // CGONAK written by the EPDISD branch
	CHECK((pqRd(R_GINTMSK) & GI_GONAKEFF) == 0);  // unmasked by the GONAKEFF branch itself
}

// RM0090 34.17.5 (isochronous OUT, application programming sequence step 3): "An endpoint transfer is incomplete if both the following
// conditions are met: EONUM bit (in OTG_FS_DOEPCTLx) = SOFFN[0] (in OTG_FS_DSTS); EPENA = 1 (in OTG_FS_DOEPCTLx)". The HAL
// (stm32f4xx_hal_pcd.c:1479) compares (RegVal & (1 << 16)) with (FrameNumber & 1): 0x10000 never equals 1, so an odd frame with EONUM = 1 is missed.
TEST_CASE_FIXTURE(PqRig, "PQ3 an incomplete ISO OUT with EONUM 1 on an odd frame is detected as incomplete" * KNOWN_ISSUE("V125 (arguable: the HAL's intent is the RM0090 test EONUM = SOFFN[0] and EPENA; the unshifted compare at stm32f4xx_hal_pcd.c:1479 can only match EONUM 0 on an even frame): an enabled isochronous OUT endpoint with EONUM 1 in an odd frame is not reported through HAL_PCD_ISOOUTIncompleteCallback")) {
	up();
	h.OUT_ep[2].type = EP_TYPE_ISOC;
	REQUIRE(HAL_PCD_EP_Receive(&h, 2, rx[2], 64) == HAL_OK);
	pqOr(outCtl(2), 1u << 16);  // EONUM = 1
	hostOtgSof(5);              // SOFFN[0] = 1
	pqOr(R_GINTSTS, GI_PXFR);
	drain();
	CHECK(otgCounts.isoOut == 1);
	CHECK((pqRd(outCtl(2)) & C_EPENA) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 an incomplete ISO OUT with EONUM 0 on an odd frame is not incomplete (EONUM differs from SOFFN[0])") {
	up();
	h.OUT_ep[2].type = EP_TYPE_ISOC;
	REQUIRE(HAL_PCD_EP_Receive(&h, 2, rx[2], 64) == HAL_OK);
	hostOtgSof(5);
	pqOr(R_GINTSTS, GI_PXFR);
	drain();
	CHECK(h.OUT_ep[2].is_iso_incomplete == 0);
	CHECK(otgCounts.isoOut == 0);
	CHECK((pqRd(outCtl(2)) & C_EPENA) == C_EPENA);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 an incomplete ISO OUT with EONUM 1 on an even frame is not incomplete (EONUM differs from SOFFN[0])") {
	up();
	h.OUT_ep[2].type = EP_TYPE_ISOC;
	REQUIRE(HAL_PCD_EP_Receive(&h, 2, rx[2], 64) == HAL_OK);
	pqOr(outCtl(2), 1u << 16);
	hostOtgSof(4);
	pqOr(R_GINTSTS, GI_PXFR);
	drain();
	CHECK(h.OUT_ep[2].is_iso_incomplete == 0);
	CHECK(otgCounts.isoOut == 0);
}

// NEEDS REVIEW 417
TEST_CASE_FIXTURE(PqRig, "characterization: PQ3 with two matching isochronous OUT endpoints only the lowest is flagged by one PXFR interrupt") {
	up();
	h.OUT_ep[1].type = EP_TYPE_ISOC;
	h.OUT_ep[3].type = EP_TYPE_ISOC;
	REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx[1], 64) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Receive(&h, 3, rx[3], 64) == HAL_OK);
	hostOtgSof(4);
	pqOr(R_GINTSTS, GI_PXFR);
	irq();
	CHECK(h.OUT_ep[1].is_iso_incomplete == 1);
	CHECK(h.OUT_ep[3].is_iso_incomplete == 0);  // the HAL breaks out of the loop after setting global OUT NAK
}

TEST_CASE_FIXTURE(PqRig, "PQ3 the incomplete ISO OUT flag leaves a bulk OUT endpoint alone") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx[1], 64) == HAL_OK);
	hostOtgSof(4);
	pqOr(R_GINTSTS, GI_PXFR);
	drain();
	CHECK(h.OUT_ep[1].is_iso_incomplete == 0);
	CHECK((pqRd(R_DCTL) & (1u << 3)) == 0);
	CHECK((pqRd(outCtl(1)) & C_EPENA) == C_EPENA);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 the global OUT NAK effective branch masks itself and aborts only endpoints flagged iso incomplete") {
	up();
	REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx[1], 64) == HAL_OK);
	pqOr(R_GINTMSK, GI_GONAKEFF);
	pqStore(R_DCTL, pqRd(R_DCTL) | (1u << 9));  // SGONAK: GINTSTS.GONAKEFF
	REQUIRE(hostOtgIrqPending());
	irq();
	CHECK((pqRd(R_GINTMSK) & GI_GONAKEFF) == 0);
	CHECK((pqRd(outCtl(1)) & C_EPENA) == C_EPENA);
}

/* ------------------------------------------------------------------------------------------------ PQ4 */

TEST_CASE_FIXTURE(PqRig, "PQ4 USBRST clears the remote wake-up signalling bit") {
	up();
	pqStore(R_DCTL, pqRd(R_DCTL) | 1u);
	REQUIRE((pqRd(R_DCTL) & 1u) == 1u);
	hostOtgUsbReset();
	drain();
	CHECK((pqRd(R_DCTL) & 1u) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 USBRST flushes every TxFIFO") {
	up();
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, tx, 20) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x83, tx, 8) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x80, tx, 4) == HAL_OK);
	drain();
	REQUIRE(hostOtgTxWordsUsed(0) == 1);
	REQUIRE(hostOtgTxWordsUsed(1) == 5);
	REQUIRE(hostOtgTxWordsUsed(3) == 2);
	hostOtgUsbReset();
	drain();
	for (unsigned f = 0; f < 4; ++f) CHECK(hostOtgTxWordsUsed(f) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 USBRST clears the interrupt flags and the stall bit of every endpoint and sets NAK on the OUT endpoints") {
	up();
	pqWr(R_DIEPMSK, 0);  // keep the flags from being served by the endpoint branches first
	pqWr(R_DOEPMSK, 0);
	for (unsigned e = 0; e < 4; ++e) {
		pqOr(inInt(e), D_XFRC | D_EPDISD | D_TOC);
		pqOr(outInt(e), D_XFRC | D_EPDISD | D_STUP);
		pqOr(inCtl(e), C_STALL);
		pqOr(outCtl(e), C_STALL);
	}
	pqTouch();
	hostOtgUsbReset();
	irq();  // one call: the flags are cleared by the USBRST branch itself, not served as endpoint interrupts
	CHECK(otgCounts.dataIn == 0);
	CHECK(otgCounts.dataOut == 0);
	for (unsigned e = 0; e < 4; ++e) {
		CHECK((pqRd(inInt(e)) & (D_XFRC | D_EPDISD | D_TOC)) == 0);
		CHECK((pqRd(outInt(e)) & (D_XFRC | D_EPDISD | D_STUP)) == 0);
		CHECK((pqRd(inCtl(e)) & C_STALL) == 0);
		CHECK((pqRd(outCtl(e)) & C_STALL) == 0);
		CHECK((pqRd(outCtl(e)) & C_NAKSTS) == C_NAKSTS);
	}
}

TEST_CASE_FIXTURE(PqRig, "PQ4 USBRST unmasks control endpoint 0 in DAINTMSK and the XFRC / TOC / STUP interrupts") {
	up();
	pqWr(R_DAINTMSK, 0);
	pqWr(R_DIEPMSK, 0);
	pqWr(R_DOEPMSK, 0);
	hostOtgUsbReset();
	drain();
	CHECK(pqRd(R_DAINTMSK) == 0x00010001u);
	CHECK((pqRd(R_DIEPMSK) & (D_XFRC | D_TOC)) == (D_XFRC | D_TOC));
	CHECK((pqRd(R_DOEPMSK) & (D_XFRC | D_STUP)) == (D_XFRC | D_STUP));
	CHECK((pqRd(R_DIEPMSK) & D_EPDISD) == D_EPDISD);
	CHECK((pqRd(R_DOEPMSK) & D_EPDISD) == D_EPDISD);
}

// NEEDS REVIEW 347
TEST_CASE_FIXTURE(PqRig, "characterization: PQ4 USBRST sets DIEPMSK 0x0B and DOEPMSK 0x202B (STSPHSRXM and NAKMSK in addition to the RM0090 list)") {
	up();
	pqWr(R_DIEPMSK, 0);
	pqWr(R_DOEPMSK, 0);
	hostOtgUsbReset();
	drain();
	CHECK(pqRd(R_DIEPMSK) == 0x0000000Bu);
	CHECK(pqRd(R_DOEPMSK) == 0x0000202Bu);
}

// NEEDS REVIEW 349
TEST_CASE_FIXTURE(PqRig, "characterization: PQ4 USBRST with use_dedicated_ep1 leaves DIEPMSK / DOEPMSK alone and touches the OTG_HS only registers") {
	up();
	h.Init.use_dedicated_ep1 = ENABLE;
	pqWr(R_DIEPMSK, 0);
	pqWr(R_DOEPMSK, 0);
	uint32_t before = hostOtgHsOnlyAccesses();
	hostOtgUsbReset();
	drain();
	CHECK(pqRd(R_DIEPMSK) == 0);
	CHECK(pqRd(R_DOEPMSK) == 0);
	CHECK(hostOtgHsOnlyAccesses() > before);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 USBRST sets the device address to 0 and keeps the other DCFG bits") {
	up();
	REQUIRE(HAL_PCD_SetAddress(&h, 0x2A) == HAL_OK);
	uint32_t dcfg = pqRd(R_DCFG);
	REQUIRE(((dcfg >> 4) & 0x7Fu) == 0x2Au);
	hostOtgUsbReset();
	drain();
	CHECK(((pqRd(R_DCFG) >> 4) & 0x7Fu) == 0);
	CHECK((pqRd(R_DCFG) & ~(0x7Fu << 4)) == (dcfg & ~(0x7Fu << 4)));
}

TEST_CASE_FIXTURE(PqRig, "PQ4 USBRST restarts the SETUP reception: DOEPTSIZ0 = STUPCNT 3, PKTCNT 1, XFRSIZ 24") {
	up();
	pqWr(outTsiz(0), 0);
	hostOtgUsbReset();
	drain();
	CHECK(pqRd(outTsiz(0)) == 0x60080018u);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 USBRST clears its flag and gives no callback by itself") {
	up();
	hostOtgUsbReset();
	drain();
	CHECK((pqRd(R_GINTSTS) & GI_USBRST) == 0);
	CHECK(otgCounts.orderCount == 0);
	CHECK(otgCounts.reset == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 ENUMDNE sets the maximum packet size of EP0 to 64 and clears the global IN NAK") {
	up();
	pqOr(inCtl(0), 3u);  // MPSIZ 8 bytes
	pqStore(R_DCTL, pqRd(R_DCTL) | (1u << 7));  // SGINAK
	REQUIRE((pqRd(R_GINTSTS) & (1u << 6)) != 0);
	hostOtgEnumDone();
	drain();
	CHECK((pqRd(inCtl(0)) & 3u) == 0);  // 00 = 64 bytes (RM0090 DIEPCTL0)
	CHECK((pqRd(R_GINTSTS) & (1u << 6)) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 ENUMDNE reads the enumerated speed (DSTS.ENUMSPD = full speed) into Init.speed") {
	up();
	h.Init.speed = 0;
	hostOtgEnumDone();
	drain();
	CHECK(h.Init.speed == PCD_SPEED_FULL);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 ENUMDNE sets TRDT = 6 for HCLK 168 MHz (RM0090 Table 203: 32 MHz and above)") {
	up();
	SystemCoreClock = 168000000u;
	hostOtgEnumDone();
	drain();
	CHECK(((pqRd(R_GUSBCFG) >> 10) & 0xFu) == 6u);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 ENUMDNE sets TRDT from HCLK: 25 MHz gives 8 and 16 MHz gives 0xD (RM0090 Table 203)") {
	up();
	SystemCoreClock = 25000000u;
	hostOtgEnumDone();
	drain();
	CHECK(((pqRd(R_GUSBCFG) >> 10) & 0xFu) == 8u);
	SystemCoreClock = 16000000u;
	hostOtgEnumDone();
	drain();
	CHECK(((pqRd(R_GUSBCFG) >> 10) & 0xFu) == 0xDu);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 ENUMDNE below the lowest AHB frequency of Table 203 leaves TRDT unchanged and still runs the reset callback") {
	up();
	SystemCoreClock = 168000000u;
	hostOtgEnumDone();
	drain();
	otgSupportReset();
	uint32_t trdt = (pqRd(R_GUSBCFG) >> 10) & 0xFu;
	SystemCoreClock = 10000000u;
	hostOtgEnumDone();
	drain();
	CHECK(((pqRd(R_GUSBCFG) >> 10) & 0xFu) == trdt);
	CHECK(otgCounts.reset == 1);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 ENUMDNE runs the reset callback once and clears its flag") {
	up();
	hostOtgEnumDone();
	drain();
	CHECK(otgCounts.reset == 1);
	CHECK((pqRd(R_GINTSTS) & GI_ENUMDNE) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 with USBRST and ENUMDNE pending the reset callback runs after the USBRST effects") {
	up();
	REQUIRE(HAL_PCD_SetAddress(&h, 0x15) == HAL_OK);
	hostOtgUsbReset();
	hostOtgEnumDone();
	irq();
	CHECK(otgCounts.reset == 1);
	CHECK(otgCounts.orderCount == 1);
	CHECK((otgCounts.statusAtCallback[0] & GI_USBRST) == 0);
	CHECK(((pqRd(R_DCFG) >> 4) & 0x7Fu) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 USBSUSP with DSTS.SUSPSTS set runs the suspend callback once and clears its flag") {
	up();
	hostOtgSuspend();
	drain();
	CHECK(otgCounts.suspend == 1);
	CHECK((pqRd(R_GINTSTS) & GI_USBSUSP) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 USBSUSP while DSTS.SUSPSTS is clear gives no callback but clears the flag") {
	up();
	pqOr(R_GINTSTS, GI_USBSUSP);
	drain();
	CHECK(otgCounts.suspend == 0);
	CHECK((pqRd(R_GINTSTS) & GI_USBSUSP) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 WKUPINT runs the resume callback, clears RWUSIG and its flag") {
	up();
	pqStore(R_DCTL, pqRd(R_DCTL) | 1u);
	uint32_t pcg = pqRd(R_PCGCCTL);
	hostOtgSuspend();
	drain();
	otgSupportReset();
	hostOtgResume();
	drain();
	CHECK(otgCounts.resume == 1);
	CHECK((pqRd(R_DCTL) & 1u) == 0);
	CHECK((pqRd(R_GINTSTS) & GI_WKUPINT) == 0);
	CHECK(pqRd(R_PCGCCTL) == pcg);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 WKUPINT in LPM state L1 reports L0 active through the LPM callback instead of the resume callback") {
	up();
	h.LPM_State = LPM_L1;
	hostOtgResume();
	drain();
	CHECK(otgCounts.resume == 0);
	CHECK(otgCounts.lpm == 1);
	CHECK(otgCounts.lastMsg == static_cast<unsigned>(PCD_LPM_L0_ACTIVE));
	CHECK(h.LPM_State == LPM_L0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 SOF runs the SOF callback and clears its flag when Sof_enable is on") {
	h.Init.Sof_enable = ENABLE;
	up();
	hostOtgSof(0x155);
	drain();
	CHECK(otgCounts.sof == 1);
	CHECK((pqRd(R_GINTSTS) & GI_SOF) == 0);
	CHECK(h.FrameNumber == 0x155u);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 SOF gives no callback when Sof_enable is off (the interrupt is not unmasked)") {
	up();
	hostOtgSof(3);
	drain();
	CHECK(otgCounts.sof == 0);
	CHECK((pqRd(R_GINTSTS) & GI_SOF) == GI_SOF);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 SRQINT runs the connect callback and clears its flag") {
	up();
	hostOtgVbus(1);
	drain();
	CHECK(otgCounts.connect == 1);
	CHECK((pqRd(R_GINTSTS) & GI_SRQINT) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 SRQINT gives no callback when VBUS sensing is off (the interrupt is not unmasked)") {
	h.Init.vbus_sensing_enable = DISABLE;
	up();
	hostOtgVbus(1);
	drain();
	CHECK(otgCounts.connect == 0);
	CHECK((pqRd(R_GINTSTS) & GI_SRQINT) == GI_SRQINT);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 OTGINT with the session end detected bit runs the disconnect callback and clears GOTGINT") {
	up();
	hostOtgVbus(0);  // GOTGINT.SEDET (bit 2)
	REQUIRE((pqRd(R_GOTGINT) & (1u << 2)) != 0);
	pqOr(R_GINTSTS, GI_OTGINT);
	irq();
	CHECK(otgCounts.disconnect == 1);
	CHECK(pqRd(R_GOTGINT) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 OTGINT without SEDET gives no disconnect callback but clears the GOTGINT bits") {
	up();
	pqWr(R_GOTGINT, 1u << 8);  // SRSSCHG
	pqOr(R_GINTSTS, GI_OTGINT);
	irq();
	CHECK(otgCounts.disconnect == 0);
	CHECK(pqRd(R_GOTGINT) == 0);
}

// NEEDS REVIEW 347
TEST_CASE_FIXTURE(PqRig, "characterization: PQ4 DISCINT has no handler in device mode: no callback, flag stays set") {
	up();
	pqOr(R_GINTMSK, GI_DISCINT);
	hostOtgDisconnectEvent();
	irq();
	CHECK(otgCounts.orderCount == 0);
	CHECK((pqRd(R_GINTSTS) & GI_DISCINT) == GI_DISCINT);
}

// NEEDS REVIEW 347
TEST_CASE_FIXTURE(PqRig, "characterization: PQ4 with several events pending the callbacks run resume, suspend, reset, SOF, connect, disconnect") {
	h.Init.Sof_enable = ENABLE;
	up();
	hostOtgSuspend();
	hostOtgResume();
	hostOtgEnumDone();
	hostOtgSof(9);
	hostOtgVbus(1);
	hostOtgVbus(0);
	pqOr(R_GINTSTS, GI_OTGINT);
	// DSTS.SUSPSTS was cleared by hostOtgResume: set it again so that the suspend callback runs
	pqOr(R_DSTS, 1u);
	irq();
	REQUIRE(otgCounts.orderCount == 6);
	CHECK(otgCounts.order[0] == OTG_CB_RESUME);
	CHECK(otgCounts.order[1] == OTG_CB_SUSPEND);
	CHECK(otgCounts.order[2] == OTG_CB_RESET);
	CHECK(otgCounts.order[3] == OTG_CB_SOF);
	CHECK(otgCounts.order[4] == OTG_CB_CONNECT);
	CHECK(otgCounts.order[5] == OTG_CB_DISCONNECT);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 the endpoint branches run before the USB reset branch in the same call") {
	up();
	pqOr(inInt(1), D_XFRC);
	pqTouch();
	hostOtgUsbReset();
	irq();
	CHECK(otgCounts.dataIn == 1);
	CHECK((pqRd(R_GINTSTS) & GI_USBRST) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 HAL_PCD_WKUP_IRQHandler clears the EXTI line 18 pending bit of the FS instance") {
	up();
	EXTI->IMR |= 1u << 18;
	EXTI->SWIER = 1u << 18;
	REQUIRE((EXTI->PR & (1u << 18)) != 0);
	HAL_PCD_WKUP_IRQHandler(&h);
	CHECK((EXTI->PR & (1u << 18)) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 HAL_PCD_WKUP_IRQHandler clears the EXTI line 20 pending bit for the HS instance") {
	PCD_HandleTypeDef hs = h;
	hs.Instance = USB_OTG_HS;
	EXTI->IMR |= 1u << 20;
	EXTI->SWIER = 1u << 20;
	REQUIRE((EXTI->PR & (1u << 20)) != 0);
	HAL_PCD_WKUP_IRQHandler(&hs);
	CHECK((EXTI->PR & (1u << 20)) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 IN XFRC clears DIEPEMPMSK in one call while the TxFIFO is more than half full (TXFE low)") {
	auto run = [&](unsigned ep, unsigned words) {
		up();
		REQUIRE(USB_WritePacket(h.Instance, tx, static_cast<uint8_t>(ep), static_cast<uint16_t>(words * 4), 0) == HAL_OK);
		pqOr(R_DIEPEMPMSK, 1u << ep);
		pqOr(inInt(ep), D_XFRC);
		pqTouch();
		REQUIRE((pqRd(inInt(ep)) & (1u << 7)) == 0);  // TXFE low: the XFRC branch alone must clear the mask
		irq();
		CHECK((pqRd(R_DIEPEMPMSK) & (1u << ep)) == 0);
		CHECK(otgCounts.dataIn == 1);
		CHECK(otgCounts.lastEp == ep);
	};
	SUBCASE("ep 0") run(0, 20);  // depth 32 words, more than half
	SUBCASE("ep 1") run(1, 40);  // depth 64 words
	SUBCASE("ep 2") run(2, 40);
}

TEST_CASE_FIXTURE(PqRig, "PQ3 a TxFIFO of 7 words gets no write for a 30 byte packet (8 words) and no overflow") {
	up(7);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, tx, 30) == HAL_OK);
	irq();
	CHECK(hostOtgTxWordsUsed(1) == 0);
	CHECK(h.IN_ep[1].xfer_count == 0);
	CHECK((pqRd(R_DIEPEMPMSK) & 2u) == 2u);
	CHECK(hostOtgFifoOverflows() == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ4 ENUMDNE takes the turnaround time from HCLK, not from PCLK1") {
	up();
	HalFixture::poke(&RCC->CFGR, 6u << 10);  // PPRE1 = /8: PCLK1 = 21 MHz, whose turnaround time (0xA) differs from the 0x6 of HCLK = 168 MHz
	pqWr(R_GUSBCFG, (pqRd(R_GUSBCFG) & ~(0xFu << 10)) | (0xFu << 10));
	SystemCoreClock = 168000000u;
	hostOtgEnumDone();
	drain();
	CHECK(((pqRd(R_GUSBCFG) >> 10) & 0xFu) == 6u);
}

/* ------------------------------------------------------------------------------------------------ PQ5 */

TEST_CASE_FIXTURE(PqRig, "PQ5 USB_ReadInterrupts returns GINTSTS & GINTMSK") {
	up();
	pqWr(R_GINTSTS, 0xA5A5A5A5u);
	pqWr(R_GINTMSK, 0x0F0FF0F0u);
	CHECK(USB_ReadInterrupts(h.Instance) == 0x0505A0A0u);
	pqWr(R_GINTMSK, 0);
	CHECK(USB_ReadInterrupts(h.Instance) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ5 USB_ReadDevAllInEpInterrupt and USB_ReadDevAllOutEpInterrupt split DAINT & DAINTMSK into the low and high half") {
	up();
	pqWr(R_DAINT, 0x000A0005u);
	pqWr(R_DAINTMSK, 0x000C0003u);
	CHECK(USB_ReadDevAllInEpInterrupt(h.Instance) == 0x0001u);
	CHECK(USB_ReadDevAllOutEpInterrupt(h.Instance) == 0x0008u);
	pqWr(R_DAINT, 0xFFFFFFFFu);
	pqWr(R_DAINTMSK, 0xFFFFFFFFu);
	CHECK(USB_ReadDevAllInEpInterrupt(h.Instance) == 0xFFFFu);
	CHECK(USB_ReadDevAllOutEpInterrupt(h.Instance) == 0xFFFFu);
}

TEST_CASE_FIXTURE(PqRig, "PQ5 USB_ReadDevOutEPInterrupt returns DOEPINTx & DOEPMSK of the selected endpoint") {
	up();
	pqWr(outInt(0), 0x01u);
	pqWr(outInt(1), 0x02u);
	pqWr(outInt(2), 0x28u);
	pqWr(outInt(3), 0x10u);
	pqWr(R_DOEPMSK, 0x1Bu);  // bits 0, 1, 3, 4
	CHECK(USB_ReadDevOutEPInterrupt(h.Instance, 0) == 0x01u);
	CHECK(USB_ReadDevOutEPInterrupt(h.Instance, 1) == 0x02u);
	CHECK(USB_ReadDevOutEPInterrupt(h.Instance, 2) == 0x08u);  // 0x28 & 0x1B
	CHECK(USB_ReadDevOutEPInterrupt(h.Instance, 3) == 0x10u);
	pqWr(R_DOEPMSK, 0);
	CHECK(USB_ReadDevOutEPInterrupt(h.Instance, 1) == 0);
}

TEST_CASE_FIXTURE(PqRig, "PQ5 USB_ReadDevInEPInterrupt returns DIEPINTx & (DIEPMSK | TXFE if DIEPEMPMSK has the endpoint's bit)") {
	up();
	for (unsigned e = 0; e < 4; ++e) pqWr(inInt(e), 0xFFu);
	pqWr(R_DIEPMSK, 0x09u);
	pqWr(R_DIEPEMPMSK, 0x2u);
	CHECK(USB_ReadDevInEPInterrupt(h.Instance, 0) == 0x09u);
	CHECK(USB_ReadDevInEPInterrupt(h.Instance, 1) == 0x89u);  // TXFE (bit 7) added for endpoint 1
	CHECK(USB_ReadDevInEPInterrupt(h.Instance, 2) == 0x09u);
	CHECK(USB_ReadDevInEPInterrupt(h.Instance, 3) == 0x09u);
	pqWr(R_DIEPEMPMSK, 0x8u);
	CHECK(USB_ReadDevInEPInterrupt(h.Instance, 1) == 0x09u);
	CHECK(USB_ReadDevInEPInterrupt(h.Instance, 3) == 0x89u);
}

TEST_CASE_FIXTURE(PqRig, "PQ5 USB_ClearInterrupts clears exactly the flags named in the argument (rc_w1)") {
	up();
	pqOr(R_GINTSTS, GI_SOF | GI_USBRST | GI_WKUPINT);
	USB_ClearInterrupts(h.Instance, GI_SOF);
	CHECK((pqRd(R_GINTSTS) & (GI_SOF | GI_USBRST | GI_WKUPINT)) == (GI_USBRST | GI_WKUPINT));
	USB_ClearInterrupts(h.Instance, GI_ENUMDNE);  // not set: no effect on the others
	CHECK((pqRd(R_GINTSTS) & (GI_USBRST | GI_WKUPINT)) == (GI_USBRST | GI_WKUPINT));
	USB_ClearInterrupts(h.Instance, 0);
	CHECK((pqRd(R_GINTSTS) & (GI_USBRST | GI_WKUPINT)) == (GI_USBRST | GI_WKUPINT));
	USB_ClearInterrupts(h.Instance, GI_USBRST | GI_WKUPINT);
	CHECK((pqRd(R_GINTSTS) & (GI_USBRST | GI_WKUPINT)) == 0);
}
