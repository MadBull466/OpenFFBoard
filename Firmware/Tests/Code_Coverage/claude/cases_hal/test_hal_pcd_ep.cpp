/*
 * P12 T8 group PE: endpoints and data movement of stm32f4xx_hal_pcd.c / stm32f4xx_ll_usb.c (device mode, USB_OTG_FS) against the OTG_FS
 * model (fakes/stm32_hal/HalOtg.h). HAL_PCD_EP_Open / Close / Receive / Transmit / GetRxCount / SetStall / ClrStall / Abort / Flush and the LL
 * functions USB_ActivateEndpoint, USB_DeactivateEndpoint, USB_ActivateDedicatedEndpoint, USB_DeactivateDedicatedEndpoint, USB_EPStartXfer,
 * USB_EPStopXfer, USB_WritePacket, USB_ReadPacket, USB_EPSetStall, USB_EPClearStall, USB_FlushTxFifo, USB_FlushRxFifo, USB_ActivateSetup,
 * USB_EP0_OutStart. Requirements PE1-PE4 of Tests/Code_Coverage/claude/charters/hal/P12_T8_REQUIREMENTS.md.
 *
 * Oracles: RM0090 Rev 22 chapter 34 (34.16 register descriptions; 34.17.5 device programming model: endpoint activation / deactivation,
 * 34.17.6 operational model: SETUP and OUT transfers, generic non isochronous OUT and IN transfers, IN endpoint disable, stalling endpoints,
 * disabling an OUT endpoint) and UM1725 Rev 8 (HAL PCD generic driver: the HAL_PCD_EP_* functions are described by one line each).
 *
 * Register fields (RM0090 34.16.4): DIEPCTLx / DOEPCTLx EPENA 31, EPDIS 30, SODDFRM 29, SD0PID (SEVNFRM) 28, SNAK 27, CNAK 26, TXFNUM 25:22
 * (IN), STALL 21, EPTYP 19:18, NAKSTS 17, USBAEP 15, MPSIZ 10:0 (endpoint 0: 1:0, 0 = 64 / 1 = 32 / 2 = 16 / 3 = 8 bytes). DIEPTSIZx /
 * DOEPTSIZx PKTCNT 28:19 (endpoint 0: bits 20:19 IN, bit 19 OUT), XFRSIZ 18:0 (endpoint 0: 6:0), STUPCNT 30:29 (OUT endpoint 0).
 * EPTYP: 0 control, 1 isochronous, 2 bulk, 3 interrupt. SNAK / CNAK / SD0PID / SODDFRM are write only strobes: their writes are read back
 * with HalFixture::lastWritten (the word the CPU wrote). GRSTCTL: TXFFLSH bit 5, TXFNUM 10:6, RXFFLSH bit 4. DCTL: GINSTS bit 2, GONSTS bit 3,
 * SGINAK 7, CGINAK 8, SGONAK 9, CGONAK 10. DSTS bit 8 is SOFFN[0]. Hand derivations:
 *   IN transfer of L bytes, maximum packet M: PKTCNT = ceil(L / M) (L = 0: 1), XFRSIZ = L (RM0090 "Generic non-periodic IN data transfers" 2).
 *   OUT transfer of L bytes: PKTCNT = n = ceil(L / M) (n >= 1), XFRSIZ = n * M for M = 64 (RM0090 "Generic non-isochronous OUT" 2).
 *   USB_EP0_OutStart: DOEPTSIZ0 = PKTCNT 1 (bit 19) | XFRSIZ 24 | STUPCNT 3 (bits 30:29) = 0x00080000 | 0x18 | 0x60000000 = 0x60080018
 *   (RM0090 34.17.5 step 4: STUPCNT = 3; 24 = 3 SETUP packets of 8 bytes).
 *
 * Not tested (reported): DMA mode (dma_enable = 1: the FS core has no DMA address registers), the early return of USB_EP0_OutStart for a core
 * newer than 0x4F54300A (the model's GSNPSID reads 0), the HAL_USB_TIMEOUT (251658240 iterations) paths of USB_FlushTxFifo / USB_FlushRxFifo
 * and the 10000 iteration timeout of USB_EPStopXfer (the model disables an endpoint at once, so EPENA never stays set).
 *
 * NEEDS REVIEW proposals (characterization cases, the lead assigns the numbers):
 *  NR346_a: HAL_PCD_EP_Open / Close on a locked handle return HAL_BUSY without touching the registers (UM1725 only names HAL_BUSY generically).
 *  NR346_b: endpoint numbers above the four endpoints of the firmware (5, 15) are not validated by Open / Close: they return HAL_OK and only touch reserved register offsets.
 *  NR346_c: Close clears MPSIZ, TXFNUM, SD0PID and EPTYP together with USBAEP (RM0090 only asks for USBAEP).
 *  NR346_d: Close of an endpoint writes DEACHMSK (offset 0x83C, no register of the OTG_FS core): counted by the model, not a defect.
 *  NR346_e: the HAL programs the even / odd frame bit of an isochronous transfer from DSTS bit 8 (current frame even: SODDFRM, odd: SD0PID).
 *  NR346_f: transfers longer than the 10 bit PKTCNT is silently truncated.
 *  NR346_g: endpoint 0 transfers are cut to one packet (xfer_len = maxpacket) by the HAL.
 *  NR346_h: USB_ActivateDedicatedEndpoint / DeactivateDedicatedEndpoint (HS only, not called by the HAL FS flow) behaviour.
 *  NR346_i: Abort does not flush the TxFIFO.
 *  NR346_j: the isochronous IN open writes the SEVNFRM bit (same bit as SD0PID).
 *  NR346_k: USB_WritePacket with dma = 1 writes nothing.
 *  NR346_l: bulk Open sets data_pid_start = 0, interrupt Open leaves it.
 *  NR346_m: an isochronous IN transfer writes the HS only MCNT bits 30:29 of DIEPTSIZ (RM0090 silent for the FS core).
 *  NR346_n: SetStall on an armed IN endpoint leaves it enabled (no EPDIS); RM0090 says STALL takes precedence over NAK.
 *  NR346_o: an Open maximum packet size above 11 bits is masked with 0x7FF.
 *  NR346_p: Close of an endpoint whose disable does not finish still returns HAL_OK.
 * Known issues (arguable): V120 EP0 Open with 8 bytes, V121 re-Open of an active endpoint, V122 Close / Abort of an OUT endpoint without
 * global OUT NAK, V123 SetStall / ClrStall accept the endpoint number equal to dev_endpoints, V124 Flush hides a flush timeout.
 */
#include "OtgTestSupport.h"
#include "TestHelpers.h"

#if HAL_USB_TIMEOUT > 100000
#error "the PE timeout cases need a small HAL_USB_TIMEOUT (CMake sets 2000U)"
#endif

#include <cstdint>
#include <cstring>

namespace {

constexpr std::uint32_t kB = HOST_OTG_BASE;

constexpr std::uint32_t kEPENA = 1u << 31, kEPDIS = 1u << 30, kSODDFRM = 1u << 29, kSD0PID = 1u << 28, kSNAK = 1u << 27, kCNAK = 1u << 26,
                        kSTALL = 1u << 21, kNAKSTS = 1u << 17, kUSBAEP = 1u << 15;
constexpr std::uint32_t kINT_XFRC = 1u << 0, kINT_EPDISD = 1u << 1, kINT_INEPNE = 1u << 6;

constexpr std::uint32_t kGRSTCTL = kB + 0x010u, kDCTL = kB + 0x804u, kDSTS = kB + 0x808u, kDAINT = kB + 0x818u, kDAINTMSK = kB + 0x81Cu,
                        kDIEPEMPMSK = kB + 0x834u, kGINTSTS = kB + 0x014u;

std::uint32_t inCtl(unsigned n) { return kB + 0x900u + 0x20u * n; }
std::uint32_t inInt(unsigned n) { return inCtl(n) + 0x08u; }
std::uint32_t inTsiz(unsigned n) { return inCtl(n) + 0x10u; }
std::uint32_t outCtl(unsigned n) { return kB + 0xB00u + 0x20u * n; }
std::uint32_t outInt(unsigned n) { return outCtl(n) + 0x08u; }
std::uint32_t outTsiz(unsigned n) { return outCtl(n) + 0x10u; }

std::uint32_t rd(std::uint32_t a) { return hostRegsPeek(a); }
void wr(std::uint32_t a, std::uint32_t v) { hostRegsPoke(a, v); }
std::uint32_t lw(std::uint32_t a) { return HalFixture::lastWritten(reinterpret_cast<const volatile void *>(static_cast<std::uintptr_t>(a))); }

/* True if any CPU write to the register since the log was cleared had all bits of 'bits' set. */
bool wroteBits(std::uint32_t a, std::uint32_t bits) {
	for (std::size_t i = 0; i < hostRegsLogCount(); ++i) {
		const HostRegAccess *e = hostRegsLogAt(i);
		if (e != nullptr && e->write && e->addr == a && (e->written & bits) == bits) return true;
	}
	return false;
}

std::uint32_t mpsizOf(std::uint32_t ctl) { return ctl & 0x7FFu; }
std::uint32_t eptypOf(std::uint32_t ctl) { return (ctl >> 18) & 3u; }
std::uint32_t txfnumOf(std::uint32_t ctl) { return (ctl >> 22) & 0xFu; }
std::uint32_t pktcntOf(std::uint32_t tsiz) { return (tsiz >> 19) & 0x3FFu; }
std::uint32_t xfrsizOf(std::uint32_t tsiz) { return tsiz & 0x7FFFFu; }

struct PeFixture : OtgFixture {
	std::uint8_t rx[600];
	std::uint8_t tx[600];
	PeFixture() {
		bringUp();
		hostOtgResetState();
		hostRegsLogClear();
		std::memset(rx, 0xAA, sizeof rx);
		for (unsigned i = 0; i < sizeof tx; ++i) tx[i] = static_cast<std::uint8_t>(i * 7u + 1u);
	}
	void open(std::uint8_t addr, std::uint16_t mps, std::uint8_t type) { REQUIRE(HAL_PCD_EP_Open(&h, addr, mps, type) == HAL_OK); }
	void arm(unsigned ep, std::uint32_t len) { REQUIRE(HAL_PCD_EP_Receive(&h, static_cast<std::uint8_t>(ep), rx, len) == HAL_OK); }
	void send(unsigned ep, std::uint32_t len) { REQUIRE(HAL_PCD_EP_Transmit(&h, static_cast<std::uint8_t>(0x80u | ep), tx, len) == HAL_OK); }
	/* Number of HS only offsets 'off' written by the CPU since the counter was at 'from'. */
	static unsigned hsOnlyWrites(unsigned from, std::uint32_t off) {
		unsigned n = 0;
		for (unsigned i = from; i < hostOtgHsOnlyAccesses(); ++i)
			if (hostOtgHsOnlyOffset(i) == off && hostOtgHsOnlyWasWrite(i)) ++n;
		return n;
	}
};

constexpr unsigned kLens[] = {0, 1, 63, 64, 65, 128, 130};
constexpr unsigned kPktsIn[] = {1, 1, 1, 1, 2, 2, 3};
constexpr unsigned kPktsOut[] = {1, 1, 1, 1, 2, 2, 3};
constexpr unsigned kXfrOut[] = {64, 64, 64, 64, 128, 128, 192};

} // namespace

// ================================================================================================= PE1 open / close

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an IN endpoint fills its endpoint structure") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(0x80u | ep), 64, EP_TYPE_BULK);
		CHECK(h.IN_ep[ep].num == ep);
		CHECK(h.IN_ep[ep].is_in == 1);
		CHECK(h.IN_ep[ep].maxpacket == 64u);
		CHECK(h.IN_ep[ep].type == EP_TYPE_BULK);
		CHECK(h.IN_ep[ep].tx_fifo_num == ep);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an OUT endpoint fills its endpoint structure and assigns no TxFIFO") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(ep), 32, EP_TYPE_INTR);
		CHECK(h.OUT_ep[ep].num == ep);
		CHECK(h.OUT_ep[ep].is_in == 0);
		CHECK(h.OUT_ep[ep].maxpacket == 32u);
		CHECK(h.OUT_ep[ep].type == EP_TYPE_INTR);
		CHECK(h.OUT_ep[ep].tx_fifo_num == 0);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an IN endpoint activates it with the maximum packet size in DIEPCTL") {
	const unsigned mpss[] = {8, 16, 32, 64};
	for (unsigned ep = 1; ep <= 3; ++ep) {
		for (unsigned mps : mpss) {
			CAPTURE(ep);
			CAPTURE(mps);
			open(static_cast<std::uint8_t>(0x80u | ep), static_cast<std::uint16_t>(mps), EP_TYPE_BULK);
			CHECK((rd(inCtl(ep)) & kUSBAEP) != 0u);
			CHECK(mpsizOf(rd(inCtl(ep))) == mps);
			REQUIRE(HAL_PCD_EP_Close(&h, static_cast<std::uint8_t>(0x80u | ep)) == HAL_OK);
		}
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an OUT endpoint activates it with the maximum packet size in DOEPCTL") {
	const unsigned mpss[] = {8, 16, 32, 64};
	for (unsigned ep = 1; ep <= 3; ++ep) {
		for (unsigned mps : mpss) {
			CAPTURE(ep);
			CAPTURE(mps);
			open(static_cast<std::uint8_t>(ep), static_cast<std::uint16_t>(mps), EP_TYPE_BULK);
			CHECK((rd(outCtl(ep)) & kUSBAEP) != 0u);
			CHECK(mpsizOf(rd(outCtl(ep))) == mps);
			REQUIRE(HAL_PCD_EP_Close(&h, static_cast<std::uint8_t>(ep)) == HAL_OK);
		}
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an IN endpoint writes the endpoint type for every type") {
	// RM0090 EPTYP: 00 control, 01 isochronous, 10 bulk, 11 interrupt.
	const std::uint8_t types[] = {EP_TYPE_CTRL, EP_TYPE_ISOC, EP_TYPE_BULK, EP_TYPE_INTR};
	const unsigned codes[] = {0, 1, 2, 3};
	for (unsigned t = 0; t < 4; ++t) {
		CAPTURE(t);
		open(0x81, 64, types[t]);
		CHECK(eptypOf(rd(inCtl(1))) == codes[t]);
		REQUIRE(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an OUT endpoint writes the endpoint type for every type") {
	const std::uint8_t types[] = {EP_TYPE_CTRL, EP_TYPE_ISOC, EP_TYPE_BULK, EP_TYPE_INTR};
	const unsigned codes[] = {0, 1, 2, 3};
	for (unsigned t = 0; t < 4; ++t) {
		CAPTURE(t);
		open(0x02, 64, types[t]);
		CHECK(eptypOf(rd(outCtl(2))) == codes[t]);
		REQUIRE(HAL_PCD_EP_Close(&h, 0x02) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an IN endpoint assigns the TxFIFO with its own number") {
	// RM0090 endpoint activation: TXFNUM is programmed at activation; the firmware uses FIFO n for IN endpoint n.
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(0x80u | ep), 64, EP_TYPE_BULK);
		CHECK(txfnumOf(rd(inCtl(ep))) == ep);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an OUT endpoint leaves the TXFNUM bits of the register at 0") {
	open(0x01, 64, EP_TYPE_BULK);
	CHECK(txfnumOf(rd(outCtl(1))) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening a bulk IN endpoint starts the data toggle at DATA0") {
	// RM0090 endpoint activation: "Endpoint start data toggle (for interrupt and bulk endpoints)"; SD0PID sets DATA0.
	open(0x81, 64, EP_TYPE_BULK);
	CHECK((lw(inCtl(1)) & kSD0PID) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening a bulk OUT endpoint starts the data toggle at DATA0") {
	open(0x01, 64, EP_TYPE_BULK);
	CHECK((lw(outCtl(1)) & kSD0PID) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an interrupt IN endpoint starts the data toggle at DATA0") {
	open(0x83, 8, EP_TYPE_INTR);
	CHECK((lw(inCtl(3)) & kSD0PID) != 0u);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 opening an isochronous IN endpoint writes the SEVNFRM bit (NR346_j)") {
	// The bit 28 is SD0PID for bulk / interrupt and SEVNFRM for isochronous endpoints (RM0090 34.16.4); RM0090 does not say which frame an open endpoint starts in.
	open(0x82, 64, EP_TYPE_ISOC);
	CHECK((lw(inCtl(2)) & kSD0PID) != 0u);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 opening a bulk endpoint resets data_pid_start and an interrupt endpoint leaves it (NR346_l)") {
	h.IN_ep[1].data_pid_start = 1;
	h.IN_ep[2].data_pid_start = 1;
	open(0x81, 64, EP_TYPE_BULK);
	open(0x82, 8, EP_TYPE_INTR);
	CHECK(h.IN_ep[1].data_pid_start == 0);
	CHECK(h.IN_ep[2].data_pid_start == 1);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an IN endpoint unmasks its interrupt in DAINTMSK") {
	// RM0090 SetConfiguration step 4: unmask the interrupt of each active endpoint (bits 3:0 IN, 19:16 OUT).
	const std::uint32_t before = rd(kDAINTMSK);
	open(0x82, 64, EP_TYPE_BULK);
	CHECK(rd(kDAINTMSK) == (before | (1u << 2)));
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an OUT endpoint unmasks its interrupt in DAINTMSK") {
	const std::uint32_t before = rd(kDAINTMSK);
	open(0x03, 64, EP_TYPE_BULK);
	CHECK(rd(kDAINTMSK) == (before | (1u << 19)));
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening endpoint 0 unmasks both endpoint 0 interrupts and leaves the control registers alone") {
	const std::uint32_t inBefore = rd(inCtl(0)), outBefore = rd(outCtl(0));
	open(0x80, 64, EP_TYPE_CTRL);
	open(0x00, 64, EP_TYPE_CTRL);
	CHECK((rd(kDAINTMSK) & 0x00010001u) == 0x00010001u);
	// RM0090 DIEPCTL0: USBAEP is always 1 and MPSIZ = 0 means 64 bytes, so an open of a 64 byte control endpoint changes nothing.
	CHECK(rd(inCtl(0)) == inBefore);
	CHECK(rd(outCtl(0)) == outBefore);
	CHECK(h.IN_ep[0].is_in == 1);
	CHECK(h.OUT_ep[0].is_in == 0);
	CHECK(h.IN_ep[0].maxpacket == 64u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening endpoint 0 with 8 bytes programs the MPSIZ code of 8 bytes" * KNOWN_ISSUE("V120 (arguable: UM1725 only says 'Open and configure an endpoint'): RM0090 DIEPCTL0 MPSIZ code 11 means 8 bytes, but USB_ActivateEndpoint (stm32f4xx_ll_usb.c lines 606-612) only writes MPSIZ when USBAEP is 0 and for endpoint 0 USBAEP always reads 1, so a 8, 16 or 32 byte control endpoint keeps the 64 byte code while the endpoint structure says 8")) {
	open(0x80, 8, EP_TYPE_CTRL);
	CHECK((rd(inCtl(0)) & 3u) == 3u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an endpoint that is already active with the same parameters changes no register") {
	open(0x81, 64, EP_TYPE_BULK);
	const std::uint32_t ctl = rd(inCtl(1)), msk = rd(kDAINTMSK);
	open(0x81, 64, EP_TYPE_BULK);
	CHECK(rd(inCtl(1)) == ctl);
	CHECK(rd(kDAINTMSK) == msk);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an active endpoint again with another type and size programs the new type and size" * KNOWN_ISSUE("V121 (arguable: the application can close first): RM0090 'Endpoint activation': the sequence also 'configure[s] an existing device endpoint to a new type', but USB_ActivateEndpoint (stm32f4xx_ll_usb.c lines 606-612 and 619-625) skips the DIEPCTL write while USBAEP is set, so an Open of an active bulk 64 endpoint as interrupt 32 keeps EPTYP 2 and MPSIZ 64 while the structure says 32")) {
	open(0x81, 64, EP_TYPE_BULK);
	open(0x81, 32, EP_TYPE_INTR);
	CHECK(eptypOf(rd(inCtl(1))) == 3u);
	CHECK(mpsizOf(rd(inCtl(1))) == 32u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening an endpoint reports HAL_OK and leaves the handle unlocked") {
	CHECK(HAL_PCD_EP_Open(&h, 0x81, 64, EP_TYPE_BULK) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(HAL_PCD_EP_Open(&h, 0x01, 64, EP_TYPE_BULK) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 opening an endpoint on a locked handle returns HAL_BUSY and writes no register (NR346_a)") {
	h.Lock = HAL_LOCKED;
	const std::uint32_t msk = rd(kDAINTMSK), ctl = rd(inCtl(1));
	CHECK(HAL_PCD_EP_Open(&h, 0x81, 64, EP_TYPE_BULK) == HAL_BUSY);
	CHECK(h.Lock == HAL_LOCKED);
	CHECK(rd(kDAINTMSK) == msk);
	CHECK(rd(inCtl(1)) == ctl);
}

// NEEDS REVIEW 349
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 opening an endpoint above the four endpoints touches only reserved registers (NR346_b)") {
	const std::uint32_t ctl1 = rd(inCtl(1)), ctl2 = rd(outCtl(2)), msk = rd(kDAINTMSK);
	const std::uint32_t reserved = hostOtgReservedAccesses();
	CHECK(HAL_PCD_EP_Open(&h, 0x85, 64, EP_TYPE_BULK) == HAL_OK);
	CHECK(HAL_PCD_EP_Open(&h, 0x0F, 64, EP_TYPE_BULK) == HAL_OK);
	CHECK(hostOtgReservedAccesses() > reserved);
	CHECK(rd(inCtl(1)) == ctl1);
	CHECK(rd(outCtl(2)) == ctl2);
	CHECK(rd(kDAINTMSK) == msk);
	clearFaults();
}

TEST_CASE_FIXTURE(PeFixture, "PE1 closing an IN endpoint deactivates it and masks its interrupt") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(0x80u | ep), 64, EP_TYPE_BULK);
		REQUIRE((rd(kDAINTMSK) & (1u << ep)) != 0u);
		CHECK(HAL_PCD_EP_Close(&h, static_cast<std::uint8_t>(0x80u | ep)) == HAL_OK);
		CHECK((rd(inCtl(ep)) & kUSBAEP) == 0u);
		CHECK((rd(kDAINTMSK) & (1u << ep)) == 0u);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE1 closing an OUT endpoint deactivates it and masks its interrupt") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(ep), 64, EP_TYPE_BULK);
		REQUIRE((rd(kDAINTMSK) & (1u << (16 + ep))) != 0u);
		CHECK(HAL_PCD_EP_Close(&h, static_cast<std::uint8_t>(ep)) == HAL_OK);
		CHECK((rd(outCtl(ep)) & kUSBAEP) == 0u);
		CHECK((rd(kDAINTMSK) & (1u << (16 + ep))) == 0u);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE1 closing an endpoint keeps the interrupt masks of the other endpoints") {
	open(0x81, 64, EP_TYPE_BULK);
	open(0x82, 64, EP_TYPE_BULK);
	open(0x02, 64, EP_TYPE_BULK);
	const std::uint32_t before = rd(kDAINTMSK);
	CHECK(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
	CHECK(rd(kDAINTMSK) == (before & ~(1u << 1)));
}

TEST_CASE_FIXTURE(PeFixture, "PE1 closing endpoint 0 masks both endpoint 0 interrupts") {
	open(0x80, 64, EP_TYPE_CTRL);
	open(0x00, 64, EP_TYPE_CTRL);
	REQUIRE((rd(kDAINTMSK) & 0x00010001u) == 0x00010001u);
	CHECK(HAL_PCD_EP_Close(&h, 0x80) == HAL_OK);
	CHECK(HAL_PCD_EP_Close(&h, 0x00) == HAL_OK);
	CHECK((rd(kDAINTMSK) & 0x00010001u) == 0u);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 closing an endpoint clears the size, type, TxFIFO and toggle fields with USBAEP (NR346_c)") {
	open(0x81, 32, EP_TYPE_INTR);
	open(0x01, 32, EP_TYPE_INTR);
	REQUIRE(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Close(&h, 0x01) == HAL_OK);
	CHECK(mpsizOf(rd(inCtl(1))) == 0u);
	CHECK(eptypOf(rd(inCtl(1))) == 0u);
	CHECK(txfnumOf(rd(inCtl(1))) == 0u);
	CHECK(mpsizOf(rd(outCtl(1))) == 0u);
	CHECK(eptypOf(rd(outCtl(1))) == 0u);
	CHECK((lw(inCtl(1)) & kSD0PID) == 0u);
	CHECK((lw(outCtl(1)) & kSD0PID) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 closing an armed IN endpoint disables it with NAK") {
	// RM0090 IN endpoint disable: SNAK and EPDIS; EPDISD asserts and EPENA clears.
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 10);
	REQUIRE((rd(inCtl(1)) & kEPENA) != 0u);
	CHECK(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
	CHECK((rd(inCtl(1)) & kEPENA) == 0u);
	CHECK((rd(inCtl(1)) & kNAKSTS) != 0u);
	CHECK((rd(inInt(1)) & kINT_EPDISD) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 closing an armed OUT endpoint disables it with NAK") {
	open(0x01, 64, EP_TYPE_BULK);
	arm(1, 64);
	REQUIRE((rd(outCtl(1)) & kEPENA) != 0u);
	CHECK(HAL_PCD_EP_Close(&h, 0x01) == HAL_OK);
	CHECK((rd(outCtl(1)) & kEPENA) == 0u);
	CHECK((rd(outCtl(1)) & kNAKSTS) != 0u);
	CHECK((rd(outInt(1)) & kINT_EPDISD) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 closing an armed OUT endpoint first sets the global OUT NAK" * KNOWN_ISSUE("V122 (arguable: the application can set it; the Abort case is the same defect at USB_EPStopXfer lines 967-968): RM0090 'Disabling an OUT endpoint' step 1: 'Before disabling any OUT endpoint, the application must enable Global OUT NAK mode in the core (SGONAK = 1 in DCTL)', but USB_DeactivateEndpoint (stm32f4xx_ll_usb.c lines 699-702) sets SNAK and EPDIS without SGONAK, so a packet being received for the endpoint can be lost")) {
	open(0x01, 64, EP_TYPE_BULK);
	arm(1, 64);
	HAL_PCD_EP_Close(&h, 0x01);
	CHECK((rd(kDCTL) & (1u << 3)) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 closing an endpoint that is not armed does not disable it") {
	open(0x81, 64, EP_TYPE_BULK);
	const std::size_t writes = hostRegsWritesTo(inCtl(1));
	CHECK(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
	CHECK((rd(inInt(1)) & kINT_EPDISD) == 0u);
	CHECK((rd(inCtl(1)) & kNAKSTS) == 0u);
	CHECK(hostRegsWritesTo(inCtl(1)) == writes + 1u); // only the write that clears USBAEP and the fields
}

TEST_CASE_FIXTURE(PeFixture, "PE1 closing an endpoint that was never opened returns HAL_OK") {
	CHECK(HAL_PCD_EP_Close(&h, 0x82) == HAL_OK);
	CHECK(HAL_PCD_EP_Close(&h, 0x03) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 closing an endpoint writes the DEACHMSK offset that the OTG_FS core does not have (NR346_d)") {
	open(0x81, 64, EP_TYPE_BULK);
	const unsigned from = hostOtgHsOnlyAccesses();
	REQUIRE(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
	CHECK(hsOnlyWrites(from, 0x83Cu) == 1u);
	CHECK(hostOtgReservedAccesses() == 0u);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 closing an endpoint on a locked handle returns HAL_BUSY and writes no register (NR346_a)") {
	open(0x81, 64, EP_TYPE_BULK);
	h.Lock = HAL_LOCKED;
	const std::uint32_t ctl = rd(inCtl(1)), msk = rd(kDAINTMSK);
	CHECK(HAL_PCD_EP_Close(&h, 0x81) == HAL_BUSY);
	CHECK(rd(inCtl(1)) == ctl);
	CHECK(rd(kDAINTMSK) == msk);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 an endpoint answers the host while open and is ignored by the core after Close") {
	// RM0090 endpoint deactivation: "the core ignores tokens addressed to that endpoint, which results in a timeout".
	std::uint8_t buf[8];
	unsigned n = 0;
	open(0x81, 64, EP_TYPE_BULK);
	open(0x01, 64, EP_TYPE_BULK);
	CHECK(hostOtgHostIn(1, buf, sizeof buf, &n) == HOSTOTG_NAK);
	CHECK(hostOtgHostOut(1, buf, 4, 0) != HOSTOTG_IGNORED);
	REQUIRE(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Close(&h, 0x01) == HAL_OK);
	CHECK(hostOtgHostIn(1, buf, sizeof buf, &n) == HOSTOTG_IGNORED);
	CHECK(hostOtgHostOut(1, buf, 4, 0) == HOSTOTG_IGNORED);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 an endpoint closed and opened again takes the new size") {
	open(0x81, 64, EP_TYPE_BULK);
	REQUIRE(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
	open(0x81, 16, EP_TYPE_INTR);
	CHECK((rd(inCtl(1)) & kUSBAEP) != 0u);
	CHECK(mpsizOf(rd(inCtl(1))) == 16u);
	CHECK(eptypOf(rd(inCtl(1))) == 3u);
	CHECK((rd(kDAINTMSK) & (1u << 1)) != 0u);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 USB_ActivateDedicatedEndpoint activates an IN endpoint and writes DEACHMSK (NR346_h)") {
	USB_OTG_EPTypeDef ep;
	std::memset(&ep, 0, sizeof ep);
	ep.num = 2;
	ep.is_in = 1;
	ep.maxpacket = 64;
	ep.type = EP_TYPE_BULK;
	const unsigned from = hostOtgHsOnlyAccesses();
	const std::uint32_t msk = rd(kDAINTMSK);
	CHECK(USB_ActivateDedicatedEndpoint(USB_OTG_FS, &ep) == HAL_OK);
	CHECK((rd(inCtl(2)) & kUSBAEP) != 0u);
	CHECK(mpsizOf(rd(inCtl(2))) == 64u);
	CHECK(eptypOf(rd(inCtl(2))) == 2u);
	CHECK(txfnumOf(rd(inCtl(2))) == 2u);
	CHECK(hsOnlyWrites(from, 0x83Cu) == 1u);
	CHECK(rd(kDAINTMSK) == msk);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 USB_ActivateDedicatedEndpoint activates an OUT endpoint and writes DEACHMSK (NR346_h)") {
	USB_OTG_EPTypeDef ep;
	std::memset(&ep, 0, sizeof ep);
	ep.num = 3;
	ep.is_in = 0;
	ep.maxpacket = 32;
	ep.type = EP_TYPE_INTR;
	const unsigned from = hostOtgHsOnlyAccesses();
	CHECK(USB_ActivateDedicatedEndpoint(USB_OTG_FS, &ep) == HAL_OK);
	CHECK((rd(outCtl(3)) & kUSBAEP) != 0u);
	CHECK(mpsizOf(rd(outCtl(3))) == 32u);
	CHECK(eptypOf(rd(outCtl(3))) == 3u);
	CHECK(hsOnlyWrites(from, 0x83Cu) == 1u);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 USB_ActivateDedicatedEndpoint keeps the registers of an endpoint that is already active (NR346_h)") {
	USB_OTG_EPTypeDef ep;
	std::memset(&ep, 0, sizeof ep);
	ep.num = 1;
	ep.is_in = 1;
	ep.maxpacket = 64;
	ep.type = EP_TYPE_BULK;
	open(0x81, 16, EP_TYPE_INTR);
	const std::uint32_t ctl = rd(inCtl(1));
	CHECK(USB_ActivateDedicatedEndpoint(USB_OTG_FS, &ep) == HAL_OK);
	CHECK(rd(inCtl(1)) == ctl);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 USB_DeactivateDedicatedEndpoint of an armed IN endpoint disables it and masks its interrupt (NR346_h)") {
	USB_OTG_EPTypeDef ep;
	std::memset(&ep, 0, sizeof ep);
	ep.num = 1;
	ep.is_in = 1;
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 10);
	REQUIRE((rd(inCtl(1)) & kEPENA) != 0u);
	CHECK(USB_DeactivateDedicatedEndpoint(USB_OTG_FS, &ep) == HAL_OK);
	CHECK((rd(inCtl(1)) & kEPENA) == 0u);
	CHECK((rd(inInt(1)) & kINT_EPDISD) != 0u);
	CHECK((rd(inCtl(1)) & kUSBAEP) == 0u);
	CHECK((rd(kDAINTMSK) & (1u << 1)) == 0u);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 USB_DeactivateDedicatedEndpoint of an armed OUT endpoint disables it and masks its interrupt (NR346_h)") {
	USB_OTG_EPTypeDef ep;
	std::memset(&ep, 0, sizeof ep);
	ep.num = 2;
	ep.is_in = 0;
	open(0x02, 64, EP_TYPE_BULK);
	arm(2, 64);
	REQUIRE((rd(outCtl(2)) & kEPENA) != 0u);
	CHECK(USB_DeactivateDedicatedEndpoint(USB_OTG_FS, &ep) == HAL_OK);
	CHECK((rd(outCtl(2)) & kEPENA) == 0u);
	CHECK((rd(outInt(2)) & kINT_EPDISD) != 0u);
	CHECK((rd(outCtl(2)) & kUSBAEP) == 0u);
	CHECK((rd(kDAINTMSK) & (1u << 18)) == 0u);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 USB_DeactivateDedicatedEndpoint of endpoints that are not armed only clears USBAEP and the mask (NR346_h)") {
	USB_OTG_EPTypeDef epIn, epOut;
	std::memset(&epIn, 0, sizeof epIn);
	std::memset(&epOut, 0, sizeof epOut);
	epIn.num = 3;
	epIn.is_in = 1;
	epOut.num = 3;
	epOut.is_in = 0;
	open(0x83, 64, EP_TYPE_BULK);
	open(0x03, 64, EP_TYPE_BULK);
	CHECK(USB_DeactivateDedicatedEndpoint(USB_OTG_FS, &epIn) == HAL_OK);
	CHECK(USB_DeactivateDedicatedEndpoint(USB_OTG_FS, &epOut) == HAL_OK);
	CHECK((rd(inInt(3)) & kINT_EPDISD) == 0u);
	CHECK((rd(outInt(3)) & kINT_EPDISD) == 0u);
	CHECK((rd(inCtl(3)) & kUSBAEP) == 0u);
	CHECK((rd(outCtl(3)) & kUSBAEP) == 0u);
}

// ================================================================================================= PE2 receive / transmit

TEST_CASE_FIXTURE(PeFixture, "PE2 receive records the buffer, the length and clears the count of the OUT endpoint") {
	open(0x01, 64, EP_TYPE_BULK);
	h.OUT_ep[1].xfer_count = 77;
	CHECK(HAL_PCD_EP_Receive(&h, 0x01, rx, 130) == HAL_OK);
	CHECK(h.OUT_ep[1].xfer_buff == rx);
	CHECK(h.OUT_ep[1].xfer_len == 130u);
	CHECK(h.OUT_ep[1].xfer_count == 0u);
	CHECK(h.OUT_ep[1].is_in == 0);
	CHECK(h.OUT_ep[1].num == 1);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit records the buffer, the length and clears the count of the IN endpoint") {
	open(0x82, 64, EP_TYPE_BULK);
	h.IN_ep[2].xfer_count = 77;
	CHECK(HAL_PCD_EP_Transmit(&h, 0x82, tx, 130) == HAL_OK);
	CHECK(h.IN_ep[2].xfer_buff == tx);
	CHECK(h.IN_ep[2].xfer_len == 130u);
	CHECK(h.IN_ep[2].xfer_count == 0u);
	CHECK(h.IN_ep[2].is_in == 1);
	CHECK(h.IN_ep[2].num == 2);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive programs the packet count of the OUT transfer size register for every length") {
	// RM0090 generic OUT transfer: PKTCNT = n, n = ceil(length / 64), at least 1.
	open(0x01, 64, EP_TYPE_BULK);
	for (unsigned i = 0; i < sizeof kLens / sizeof kLens[0]; ++i) {
		CAPTURE(kLens[i]);
		arm(1, kLens[i]);
		CHECK(pktcntOf(rd(outTsiz(1))) == kPktsOut[i]);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive programs the transfer size of the OUT transfer size register for every length") {
	// RM0090 generic OUT transfer: XFRSIZ = n * 64 for the 64 byte endpoint.
	open(0x01, 64, EP_TYPE_BULK);
	for (unsigned i = 0; i < sizeof kLens / sizeof kLens[0]; ++i) {
		CAPTURE(kLens[i]);
		arm(1, kLens[i]);
		CHECK(xfrsizOf(rd(outTsiz(1))) == kXfrOut[i]);
		if (kLens[i] != 0u) CHECK(h.OUT_ep[1].xfer_size == kXfrOut[i]);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive works on every OUT endpoint 1-3") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(ep), 64, EP_TYPE_BULK);
		arm(ep, 65);
		CHECK(pktcntOf(rd(outTsiz(ep))) == 2u);
		CHECK(xfrsizOf(rd(outTsiz(ep))) == 128u);
		CHECK((rd(outCtl(ep)) & kEPENA) != 0u);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive with an 8 byte endpoint rounds the packet count up to whole packets") {
	// 17 bytes with 8 byte packets: ceil(17 / 8) = 3 packets, XFRSIZ = 3 * 8 = 24 (8 is a multiple of 4).
	open(0x02, 8, EP_TYPE_BULK);
	arm(2, 17);
	CHECK(pktcntOf(rd(outTsiz(2))) == 3u);
	CHECK(xfrsizOf(rd(outTsiz(2))) == 24u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive with a 32 byte endpoint of 100 bytes programs 4 packets of 32 bytes") {
	open(0x03, 32, EP_TYPE_INTR);
	arm(3, 100);
	CHECK(pktcntOf(rd(outTsiz(3))) == 4u);
	CHECK(xfrsizOf(rd(outTsiz(3))) == 128u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive clears the count and size fields of the previous transfer") {
	open(0x01, 64, EP_TYPE_BULK);
	arm(1, 130);
	arm(1, 10);
	CHECK(pktcntOf(rd(outTsiz(1))) == 1u);
	CHECK(xfrsizOf(rd(outTsiz(1))) == 64u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive enables the endpoint and clears its NAK") {
	// RM0090 OUT programming sequence step 2: EPENA = 1 and CNAK = 1.
	open(0x01, 64, EP_TYPE_BULK);
	wr(outCtl(1), rd(outCtl(1)) | kNAKSTS); // NAKSTS is read only: the test sets it directly in the register
	REQUIRE((rd(outCtl(1)) & kNAKSTS) != 0u);
	arm(1, 64);
	CHECK((lw(outCtl(1)) & kEPENA) != 0u);
	CHECK((lw(outCtl(1)) & kCNAK) != 0u);
	CHECK((rd(outCtl(1)) & kEPENA) != 0u);
	CHECK((rd(outCtl(1)) & kNAKSTS) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 an armed OUT endpoint accepts a host packet and an endpoint that is not armed does not") {
	open(0x01, 64, EP_TYPE_BULK);
	const std::uint8_t d[4] = {1, 2, 3, 4};
	CHECK(hostOtgHostOut(1, d, 4, 0) == HOSTOTG_NAK);
	arm(1, 64);
	CHECK(hostOtgHostOut(1, d, 4, 0) == HOSTOTG_ACK);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive on endpoint 0 programs one 64 byte packet in the endpoint 0 transfer size register") {
	open(0x80, 64, EP_TYPE_CTRL);
	open(0x00, 64, EP_TYPE_CTRL);
	arm(0, 64);
	// RM0090 DOEPTSIZ0: PKTCNT is bit 19 only, XFRSIZ 7 bits.
	CHECK(pktcntOf(rd(outTsiz(0))) == 1u);
	CHECK(xfrsizOf(rd(outTsiz(0))) == 64u);
	CHECK((rd(outCtl(0)) & kEPENA) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive on endpoint 0 of length 0 still programs one packet of the maximum size") {
	// A zero length status OUT stage needs one packet of room: RM0090 'n > 0'.
	open(0x00, 64, EP_TYPE_CTRL);
	arm(0, 0);
	CHECK(pktcntOf(rd(outTsiz(0))) == 1u);
	CHECK(xfrsizOf(rd(outTsiz(0))) == 64u);
	CHECK(h.OUT_ep[0].xfer_len == 0u);
}

// NEEDS REVIEW 349
TEST_CASE_FIXTURE(PeFixture, "characterization: PE2 receive on endpoint 0 of more than one packet is cut to one packet (NR346_g)") {
	open(0x00, 64, EP_TYPE_CTRL);
	arm(0, 200);
	CHECK(h.OUT_ep[0].xfer_len == 64u);
	CHECK(h.OUT_ep[0].xfer_size == 64u);
	CHECK(pktcntOf(rd(outTsiz(0))) == 1u);
	CHECK(xfrsizOf(rd(outTsiz(0))) == 64u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive on endpoint 0 keeps the SETUP packet counter of the transfer size register") {
	// RM0090 34.17.5: STUPCNT = 3 is programmed once at the USB reset; a data stage must not lose it.
	open(0x00, 64, EP_TYPE_CTRL);
	REQUIRE(USB_EP0_OutStart(USB_OTG_FS, 0, reinterpret_cast<const std::uint8_t *>(h.Setup)) == HAL_OK);
	arm(0, 64);
	CHECK(((rd(outTsiz(0)) >> 29) & 3u) == 3u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit programs the packet count of the IN transfer size register for every length") {
	// RM0090 generic IN transfer: PKTCNT = x + 1 for a short packet, x otherwise; zero length: 1.
	open(0x81, 64, EP_TYPE_BULK);
	for (unsigned i = 0; i < sizeof kLens / sizeof kLens[0]; ++i) {
		CAPTURE(kLens[i]);
		send(1, kLens[i]);
		CHECK(pktcntOf(rd(inTsiz(1))) == kPktsIn[i]);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit programs the transfer size of the IN transfer size register for every length") {
	open(0x81, 64, EP_TYPE_BULK);
	for (unsigned i = 0; i < sizeof kLens / sizeof kLens[0]; ++i) {
		CAPTURE(kLens[i]);
		send(1, kLens[i]);
		CHECK(xfrsizOf(rd(inTsiz(1))) == kLens[i]);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit works on every IN endpoint 1-3") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(0x80u | ep), 64, EP_TYPE_BULK);
		send(ep, 65);
		CHECK(pktcntOf(rd(inTsiz(ep))) == 2u);
		CHECK(xfrsizOf(rd(inTsiz(ep))) == 65u);
		CHECK((rd(inCtl(ep)) & kEPENA) != 0u);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit with an 8 byte endpoint of 17 bytes programs 3 packets") {
	open(0x81, 8, EP_TYPE_INTR);
	send(1, 17);
	CHECK(pktcntOf(rd(inTsiz(1))) == 3u);
	CHECK(xfrsizOf(rd(inTsiz(1))) == 17u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit clears the count and size fields of the previous transfer") {
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 130);
	send(1, 5);
	CHECK(pktcntOf(rd(inTsiz(1))) == 1u);
	CHECK(xfrsizOf(rd(inTsiz(1))) == 5u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit enables the endpoint and clears its NAK") {
	// RM0090 IN programming sequence step 2: set CNAK and EPENA.
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 8);
	CHECK((lw(inCtl(1)) & kEPENA) != 0u);
	CHECK((lw(inCtl(1)) & kCNAK) != 0u);
	CHECK((rd(inCtl(1)) & kNAKSTS) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit of data unmasks the TxFIFO empty interrupt of its endpoint only") {
	open(0x81, 64, EP_TYPE_BULK);
	open(0x82, 64, EP_TYPE_BULK);
	send(2, 8);
	CHECK(rd(kDIEPEMPMSK) == (1u << 2));
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit of zero length does not unmask the TxFIFO empty interrupt") {
	// RM0090: "To write zero length packet, the application must not look at the FIFO space."
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 0);
	CHECK(rd(kDIEPEMPMSK) == 0u);
	CHECK(pktcntOf(rd(inTsiz(1))) == 1u);
	CHECK(xfrsizOf(rd(inTsiz(1))) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 an armed IN transfer of data raises the interrupt line through the TxFIFO empty flag") {
	open(0x81, 64, EP_TYPE_BULK);
	CHECK((rd(kDAINT) & (1u << 1)) == 0u);
	send(1, 8);
	CHECK((rd(kDAINT) & (1u << 1)) != 0u);
	CHECK((rd(kGINTSTS) & (1u << 18)) != 0u); // IEPINT
	CHECK(hostOtgIrqPending() != 0);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 an armed zero length IN transfer is answered by an empty packet") {
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 0);
	std::uint8_t buf[4];
	unsigned n = 77;
	CHECK(hostOtgHostIn(1, buf, sizeof buf, &n) == HOSTOTG_ACK);
	CHECK(n == 0u);
	CHECK((rd(inInt(1)) & kINT_XFRC) != 0u);
	CHECK((rd(inCtl(1)) & kEPENA) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 a transmitted packet reaches the host on every IN endpoint") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(0x80u | ep), 64, EP_TYPE_BULK);
		send(ep, 64);
		drain();
		std::uint8_t got[64];
		unsigned n = 0;
		CHECK(hostOtgHostIn(ep, got, sizeof got, &n) == HOSTOTG_ACK);
		REQUIRE(n == 64u);
		CHECK(std::memcmp(got, tx, 64) == 0);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit on endpoint 0 programs one packet with the length in the endpoint 0 transfer size register") {
	open(0x80, 64, EP_TYPE_CTRL);
	send(0, 18);
	CHECK(pktcntOf(rd(inTsiz(0))) == 1u);
	CHECK(xfrsizOf(rd(inTsiz(0))) == 18u);
	CHECK(h.IN_ep[0].xfer_len == 18u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit on endpoint 0 of length 0 programs a zero length packet") {
	open(0x80, 64, EP_TYPE_CTRL);
	send(0, 0);
	CHECK(pktcntOf(rd(inTsiz(0))) == 1u);
	CHECK(xfrsizOf(rd(inTsiz(0))) == 0u);
	CHECK(rd(kDIEPEMPMSK) == 0u);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE2 transmit on endpoint 0 of more than one packet is cut to one packet (NR346_g)") {
	open(0x80, 64, EP_TYPE_CTRL);
	send(0, 100);
	CHECK(h.IN_ep[0].xfer_len == 64u);
	CHECK(xfrsizOf(rd(inTsiz(0))) == 64u);
	CHECK(pktcntOf(rd(inTsiz(0))) == 1u);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE2 transmit on endpoint 0 with an endpoint structure size of 8 bytes is cut to 8 bytes (NR346_g)") {
	open(0x80, 8, EP_TYPE_CTRL);
	send(0, 18);
	CHECK(h.IN_ep[0].xfer_len == 8u);
	CHECK(xfrsizOf(rd(inTsiz(0))) == 8u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit on endpoint 0 raises the TxFIFO empty interrupt for endpoint 0") {
	open(0x80, 64, EP_TYPE_CTRL);
	send(0, 18);
	CHECK(rd(kDIEPEMPMSK) == 1u);
	CHECK((rd(kDAINT) & 1u) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit on an isochronous endpoint writes the packet to the TxFIFO at once") {
	// RM0090 periodic IN: the whole payload of the frame is in the FIFO before the IN token; the HAL writes it in the start call.
	open(0x81, 64, EP_TYPE_ISOC);
	send(1, 40);
	CHECK(hostOtgTxWordsUsed(1) == 10u);
	CHECK(rd(kDIEPEMPMSK) == 0u);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE2 transmit on an isochronous endpoint in an even frame sets the odd frame bit (NR346_e)") {
	open(0x81, 64, EP_TYPE_ISOC);
	hostOtgSof(4);
	send(1, 8);
	CHECK((lw(inCtl(1)) & kSODDFRM) != 0u);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE2 transmit on an isochronous endpoint in an odd frame sets the even frame bit (NR346_e)") {
	open(0x81, 64, EP_TYPE_ISOC);
	hostOtgSof(5);
	send(1, 8);
	CHECK((lw(inCtl(1)) & kSODDFRM) == 0u);
	CHECK((lw(inCtl(1)) & kSD0PID) != 0u);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE2 receive on an isochronous endpoint in an even frame sets the odd frame bit (NR346_e)") {
	open(0x01, 64, EP_TYPE_ISOC);
	hostOtgSof(4);
	arm(1, 64);
	CHECK(wroteBits(outCtl(1), kSODDFRM));
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE2 receive on an isochronous endpoint in an odd frame sets the even frame bit (NR346_e)") {
	open(0x01, 64, EP_TYPE_ISOC);
	hostOtgSof(5);
	arm(1, 64);
	CHECK(!wroteBits(outCtl(1), kSODDFRM));
	CHECK(wroteBits(outCtl(1), kSD0PID));
}

TEST_CASE_FIXTURE(PeFixture, "PE2 transmit on a non isochronous endpoint does not write a frame parity bit") {
	open(0x81, 64, EP_TYPE_BULK);
	hostOtgSof(4);
	send(1, 8);
	CHECK((lw(inCtl(1)) & kSODDFRM) == 0u);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE2 an isochronous IN transfer of two packets writes the HS only MCNT bits 30:29 of the transfer size register (NR346_m)") {
	// RM0090 marks DIEPTSIZx bits 31:29 of the OTG_FS core reserved and says nothing about writing them; the HAL writes MULCNT = PKTCNT = 2 there.
	open(0x81, 64, EP_TYPE_ISOC);
	send(1, 100);
	CHECK(wroteBits(inTsiz(1), 2u << 29));
	CHECK(pktcntOf(rd(inTsiz(1))) == 2u);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE2 transmit of more packets than the 10 bit packet count holds is truncated (NR346_f)") {
	// 70000 bytes / 64 = 1094 packets; PKTCNT is 10 bits (RM0090 DIEPTSIZx 28:19), 1094 mod 1024 = 70. XFRSIZ (19 bits) holds 70000.
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 70000);
	CHECK(pktcntOf(rd(inTsiz(1))) == 70u);
	CHECK(xfrsizOf(rd(inTsiz(1))) == 70000u);
}

// NEEDS REVIEW 415
TEST_CASE_FIXTURE(PeFixture, "characterization: PE2 receive of more packets than the 10 bit packet count holds is truncated (NR346_f)") {
	open(0x01, 64, EP_TYPE_BULK);
	arm(1, 70000);
	CHECK(pktcntOf(rd(outTsiz(1))) == 70u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 receive and transmit do not touch the registers of other endpoints") {
	open(0x01, 64, EP_TYPE_BULK);
	open(0x81, 64, EP_TYPE_BULK);
	open(0x82, 64, EP_TYPE_BULK);
	open(0x03, 64, EP_TYPE_BULK);
	const std::uint32_t out3 = rd(outCtl(3)), out3s = rd(outTsiz(3)), in2 = rd(inCtl(2)), in2s = rd(inTsiz(2));
	arm(1, 64);
	send(1, 8);
	CHECK(rd(outCtl(3)) == out3);
	CHECK(rd(outTsiz(3)) == out3s);
	CHECK(rd(inCtl(2)) == in2);
	CHECK(rd(inTsiz(2)) == in2s);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 the received byte count of an OUT endpoint is 0 before data arrive") {
	open(0x01, 64, EP_TYPE_BULK);
	arm(1, 64);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x01) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 the received byte count is the length of the packet the host sent") {
	const unsigned sizes[] = {0, 1, 5, 63, 64};
	open(0x01, 64, EP_TYPE_BULK);
	for (unsigned size : sizes) {
		CAPTURE(size);
		arm(1, 64);
		CHECK(hostOtgHostOut(1, tx, size, 0) == HOSTOTG_ACK);
		drain();
		CHECK(HAL_PCD_EP_GetRxCount(&h, 0x01) == size);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE2 the received byte count of a transfer of two packets is the sum") {
	open(0x01, 64, EP_TYPE_BULK);
	arm(1, 128);
	CHECK(hostOtgHostOut(1, tx, 64, 0) == HOSTOTG_ACK);
	CHECK(hostOtgHostOut(1, tx + 64, 1, 1) == HOSTOTG_ACK);
	drain();
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x01) == 65u);
	CHECK(std::memcmp(rx, tx, 65) == 0);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 the received byte count is kept per endpoint") {
	open(0x01, 64, EP_TYPE_BULK);
	open(0x02, 64, EP_TYPE_BULK);
	arm(1, 64);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x02, rx + 100, 64) == HAL_OK);
	CHECK(hostOtgHostOut(1, tx, 3, 0) == HOSTOTG_ACK);
	CHECK(hostOtgHostOut(2, tx, 9, 0) == HOSTOTG_ACK);
	drain();
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x01) == 3u);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x02) == 9u);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x03) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE2 the received byte count is the one set by the structure field") {
	h.OUT_ep[2].xfer_count = 1234;
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x02) == 1234u);
}

// ================================================================================================= PE3 write / read packet

TEST_CASE_FIXTURE(PeFixture, "PE3 writing a packet puts one word per 4 bytes into the TxFIFO of the endpoint") {
	const unsigned lens[] = {0, 1, 2, 3, 4, 5, 8, 63, 64, 65};
	std::uint8_t src[72];
	std::memset(src, 0x5A, sizeof src);
	for (unsigned len : lens) {
		CAPTURE(len);
		REQUIRE(USB_FlushTxFifo(USB_OTG_FS, 1) == HAL_OK);
		CHECK(USB_WritePacket(USB_OTG_FS, src, 1, static_cast<std::uint16_t>(len), 0) == HAL_OK);
		CHECK(hostOtgTxWordsUsed(1) == (len + 3u) / 4u);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE3 writing a packet packs the bytes little endian") {
	// bytes 01 02 03 04 05 06 07 08 -> words 0x04030201, 0x08070605 (RM0090 FIFO word access: first byte in the low byte).
	const std::uint8_t src[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	REQUIRE(USB_WritePacket(USB_OTG_FS, const_cast<std::uint8_t *>(src), 2, 8, 0) == HAL_OK);
	REQUIRE(hostOtgTxWordsUsed(2) == 2u);
	CHECK(hostOtgTxWords(2)[0] == 0x04030201u);
	CHECK(hostOtgTxWords(2)[1] == 0x08070605u);
}

TEST_CASE_FIXTURE(PeFixture, "PE3 writing a packet with an odd tail keeps the tail bytes in the low bytes of the last word") {
	for (unsigned tail = 1; tail <= 3; ++tail) {
		CAPTURE(tail);
		const std::uint8_t full[8] = {0x11, 0x22, 0x33, 0x44, 0xA1, 0xA2, 0xA3, 0xA4};
		std::uint8_t buf[8];
		std::memcpy(buf, full, 8);
		REQUIRE(USB_FlushTxFifo(USB_OTG_FS, 3) == HAL_OK);
		REQUIRE(USB_WritePacket(USB_OTG_FS, buf, 3, static_cast<std::uint16_t>(4 + tail), 0) == HAL_OK);
		REQUIRE(hostOtgTxWordsUsed(3) == 2u);
		const std::uint32_t mask = tail == 1 ? 0xFFu : tail == 2 ? 0xFFFFu : 0xFFFFFFu;
		const std::uint32_t expect = 0xA4A3A2A1u & mask; // bytes A1.. little endian
		CHECK((hostOtgTxWords(3)[1] & mask) == expect);
		CHECK(hostOtgTxWords(3)[0] == 0x44332211u);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE3 writing a packet from an unaligned address gives the same words") {
	std::uint8_t buf[16];
	for (unsigned i = 0; i < sizeof buf; ++i) buf[i] = static_cast<std::uint8_t>(0x10u + i);
	for (unsigned off = 1; off <= 3; ++off) {
		CAPTURE(off);
		REQUIRE(USB_FlushTxFifo(USB_OTG_FS, 0) == HAL_OK);
		REQUIRE(USB_WritePacket(USB_OTG_FS, buf + off, 0, 8, 0) == HAL_OK);
		REQUIRE(hostOtgTxWordsUsed(0) == 2u);
		const std::uint8_t *e = buf + off;
		CHECK(hostOtgTxWords(0)[0] == (static_cast<std::uint32_t>(e[0]) | static_cast<std::uint32_t>(e[1]) << 8 | static_cast<std::uint32_t>(e[2]) << 16 | static_cast<std::uint32_t>(e[3]) << 24));
		CHECK(hostOtgTxWords(0)[1] == (static_cast<std::uint32_t>(e[4]) | static_cast<std::uint32_t>(e[5]) << 8 | static_cast<std::uint32_t>(e[6]) << 16 | static_cast<std::uint32_t>(e[7]) << 24));
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE3 writing a packet uses the FIFO window of its endpoint only") {
	std::uint8_t src[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	for (unsigned ep = 0; ep <= 3; ++ep) {
		CAPTURE(ep);
		REQUIRE(USB_WritePacket(USB_OTG_FS, src, static_cast<std::uint8_t>(ep), 8, 0) == HAL_OK);
		for (unsigned f = 0; f <= 3; ++f) CHECK(hostOtgTxWordsUsed(f) == (f <= ep ? 2u : 0u));
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE3 writing a packet of zero length writes no word") {
	std::uint8_t src[4] = {9, 9, 9, 9};
	CHECK(USB_WritePacket(USB_OTG_FS, src, 1, 0, 0) == HAL_OK);
	CHECK(hostOtgTxWordsUsed(1) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE3 writing a packet as large as the TxFIFO fills it without an overflow") {
	// Bring-up gives TxFIFO 1 a depth of 0x40 words = 256 bytes (RM0090 TX FIFO depth in words).
	REQUIRE(USB_WritePacket(USB_OTG_FS, tx, 1, 256, 0) == HAL_OK);
	CHECK(hostOtgTxWordsUsed(1) == 64u);
	CHECK(hostOtgFifoOverflows() == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE3 writing more than the depth of the TxFIFO overflows it and the LL does not check") {
	hostOtgStrictFifo(0);
	CHECK(USB_WritePacket(USB_OTG_FS, tx, 1, 260, 0) == HAL_OK);
	CHECK(hostOtgTxWordsUsed(1) == 64u);
	CHECK(hostOtgFifoOverflows() == 1u); // 65 words into a 64 word FIFO: one word lost
}

TEST_CASE_FIXTURE(PeFixture, "PE3 writing more than the depth of the TxFIFO is a bus fault in the strict model") {
	REQUIRE(USB_WritePacket(USB_OTG_FS, tx, 2, 260, 0) == HAL_OK);
	CHECK(faults() == 1u);
	clearFaults();
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE3 writing a packet with dma 1 writes nothing (NR346_k)") {
	CHECK(USB_WritePacket(USB_OTG_FS, tx, 1, 16, 1) == HAL_OK);
	CHECK(hostOtgTxWordsUsed(1) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE3 reading a packet copies exactly the byte count and returns the end pointer") {
	const unsigned lens[] = {1, 2, 3, 4, 5, 7, 8, 9, 63, 64};
	open(0x01, 64, EP_TYPE_BULK);
	for (unsigned len : lens) {
		CAPTURE(len);
		arm(1, 64);
		REQUIRE(hostOtgHostOut(1, tx, len, 0) == HOSTOTG_ACK);
		const std::uint32_t status = USB_OTG_FS->GRXSTSP;
		REQUIRE(((status >> 4) & 0x7FFu) == len);
		std::uint8_t dest[80];
		std::memset(dest, 0xCC, sizeof dest);
		void *end = USB_ReadPacket(USB_OTG_FS, dest, static_cast<std::uint16_t>(len));
		CHECK(end == dest + len);
		CHECK(std::memcmp(dest, tx, len) == 0);
		CHECK(dest[len] == 0xCC);
		CHECK(hostOtgRxWordsUsed() == 1u); // only the transfer completed entry is left
		(void)USB_OTG_FS->GRXSTSP; // pops it so that the next round starts clean
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE3 reading a packet of 65 bytes reads 17 words") {
	// 65 bytes: 16 whole words and a 1 byte tail = 17 words.
	hostOtgRxFill(0x44332211u, 17);
	std::uint8_t dest[80];
	std::memset(dest, 0xCC, sizeof dest);
	void *end = USB_ReadPacket(USB_OTG_FS, dest, 65);
	CHECK(end == dest + 65);
	CHECK(hostOtgRxWordsUsed() == 0u);
	for (unsigned i = 0; i < 65; ++i) {
		CAPTURE(i);
		CHECK(dest[i] == static_cast<std::uint8_t>(0x11u * ((i % 4u) + 1u)));
	}
	CHECK(dest[65] == 0xCC);
}

TEST_CASE_FIXTURE(PeFixture, "PE3 reading a packet of zero length reads no word") {
	hostOtgRxFill(0x01020304u, 1);
	std::uint8_t dest[4] = {0xCC, 0xCC, 0xCC, 0xCC};
	void *end = USB_ReadPacket(USB_OTG_FS, dest, 0);
	CHECK(end == dest);
	CHECK(hostOtgRxWordsUsed() == 1u);
	CHECK(dest[0] == 0xCC);
	CHECK(hostOtgFifoUnderflows() == 0u);
	clearFaults();
	USB_ReadPacket(USB_OTG_FS, dest, 4); // empty the model so that nothing is left
}

TEST_CASE_FIXTURE(PeFixture, "PE3 reading a packet to an unaligned address stores the bytes in order") {
	for (unsigned off = 1; off <= 3; ++off) {
		CAPTURE(off);
		hostOtgRxFill(0x04030201u, 2);
		std::uint8_t dest[16];
		std::memset(dest, 0xCC, sizeof dest);
		void *end = USB_ReadPacket(USB_OTG_FS, dest + off, 7);
		CHECK(end == dest + off + 7);
		const std::uint8_t expect[7] = {1, 2, 3, 4, 1, 2, 3};
		CHECK(std::memcmp(dest + off, expect, 7) == 0);
		CHECK(dest[off - 1] == 0xCC);
		CHECK(dest[off + 7] == 0xCC);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE3 reading a packet from an empty receive FIFO is a bus fault in the strict model") {
	std::uint8_t dest[4];
	USB_ReadPacket(USB_OTG_FS, dest, 4);
	CHECK(hostOtgFifoUnderflows() == 1u);
	CHECK(faults() == 1u);
	clearFaults();
}

TEST_CASE_FIXTURE(PeFixture, "PE3 a packet written to a TxFIFO is the packet the host reads") {
	// 7 bytes: one full word and a 3 byte tail; the host reads exactly the 7 bytes.
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 7);
	REQUIRE(USB_WritePacket(USB_OTG_FS, tx, 1, 7, 0) == HAL_OK);
	std::uint8_t got[16];
	unsigned n = 0;
	CHECK(hostOtgHostIn(1, got, sizeof got, &n) == HOSTOTG_ACK);
	CHECK(n == 7u);
	CHECK(std::memcmp(got, tx, 7) == 0);
}

// ================================================================================================= PE4 stall, abort, flush, EP0 start

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall on an IN endpoint sets STALL in DIEPCTL and the host sees STALL") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(0x80u | ep), 64, EP_TYPE_BULK);
		CHECK(HAL_PCD_EP_SetStall(&h, static_cast<std::uint8_t>(0x80u | ep)) == HAL_OK);
		CHECK((rd(inCtl(ep)) & kSTALL) != 0u);
		std::uint8_t buf[4];
		unsigned n = 0;
		CHECK(hostOtgHostIn(ep, buf, sizeof buf, &n) == HOSTOTG_STALL);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall on an OUT endpoint sets STALL in DOEPCTL and the host sees STALL") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(ep), 64, EP_TYPE_BULK);
		CHECK(HAL_PCD_EP_SetStall(&h, static_cast<std::uint8_t>(ep)) == HAL_OK);
		CHECK((rd(outCtl(ep)) & kSTALL) != 0u);
		const std::uint8_t d[2] = {1, 2};
		CHECK(hostOtgHostOut(ep, d, 2, 0) == HOSTOTG_STALL);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall records the endpoint structure and leaves the handle unlocked") {
	open(0x82, 64, EP_TYPE_BULK);
	open(0x03, 64, EP_TYPE_BULK);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x82) == HAL_OK);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x03) == HAL_OK);
	CHECK(h.IN_ep[2].is_stall == 1);
	CHECK(h.IN_ep[2].is_in == 1);
	CHECK(h.IN_ep[2].num == 2);
	CHECK(h.OUT_ep[3].is_stall == 1);
	CHECK(h.OUT_ep[3].is_in == 0);
	CHECK(h.OUT_ep[3].num == 3);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall on endpoint 0 restarts the SETUP reception") {
	// RM0090 34.17.5 step 4: STUPCNT = 3 in DOEPTSIZ0, PKTCNT 1, XFRSIZ 24.
	open(0x80, 64, EP_TYPE_CTRL);
	open(0x00, 64, EP_TYPE_CTRL);
	CHECK(HAL_PCD_EP_SetStall(&h, 0x80) == HAL_OK);
	CHECK((rd(inCtl(0)) & kSTALL) != 0u);
	CHECK(rd(outTsiz(0)) == 0x60080018u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall on the OUT endpoint 0 sets STALL and restarts the SETUP reception") {
	open(0x00, 64, EP_TYPE_CTRL);
	CHECK(HAL_PCD_EP_SetStall(&h, 0x00) == HAL_OK);
	CHECK((rd(outCtl(0)) & kSTALL) != 0u);
	CHECK(rd(outTsiz(0)) == 0x60080018u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall on a non control endpoint leaves the endpoint 0 transfer size register") {
	open(0x81, 64, EP_TYPE_BULK);
	wr(outTsiz(0), 0x00000040u);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_OK);
	CHECK(rd(outTsiz(0)) == 0x00000040u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall on an IN endpoint 0 is seen by the host as STALL") {
	open(0x80, 64, EP_TYPE_CTRL);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x80) == HAL_OK);
	std::uint8_t buf[4];
	unsigned n = 0;
	CHECK(hostOtgHostIn(0, buf, sizeof buf, &n) == HOSTOTG_STALL);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall on an endpoint number above the endpoint count fails") {
	CHECK(HAL_PCD_EP_SetStall(&h, 0x85) == HAL_ERROR);
	CHECK(HAL_PCD_EP_SetStall(&h, 0x05) == HAL_ERROR);
	CHECK(HAL_PCD_EP_SetStall(&h, 0x8F) == HAL_ERROR);
	CHECK(hostOtgReservedAccesses() == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall on the endpoint number equal to the endpoint count fails" * KNOWN_ISSUE("V123 (arguable: UM1725 describes dev_endpoints only as the 'Device Endpoints number'): with dev_endpoints = 4 the endpoints are 0-3 (HAL_PCD_Init initialises IN_ep[0..3]) and the OTG_FS core has registers for four endpoints, but HAL_PCD_EP_SetStall (stm32f4xx_hal_pcd.c line 1965) rejects only numbers GREATER than dev_endpoints, so endpoint 4 is accepted and the reserved registers of endpoint 4 are written")) {
	CHECK(HAL_PCD_EP_SetStall(&h, 0x84) == HAL_ERROR);
	clearFaults();
}

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall on a locked handle returns HAL_BUSY") {
	open(0x81, 64, EP_TYPE_BULK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_BUSY);
	CHECK((rd(inCtl(1)) & kSTALL) == 0u);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE4 setting a stall on an armed IN endpoint leaves it enabled with STALL set (NR346_n)") {
	// RM0090 'Stalling non-isochronous IN endpoints' lists EPDIS = 1 for an enabled endpoint but also says STALL always takes precedence over NAK;
	// USB_EPSetStall never sets EPDIS: the armed endpoint stays enabled and answers STALL.
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 10);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_OK);
	CHECK((rd(inCtl(1)) & kEPENA) != 0u);
	CHECK((rd(inCtl(1)) & kSTALL) != 0u);
	CHECK((rd(inInt(1)) & kINT_EPDISD) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 setting a stall on an armed IN endpoint keeps the stall bit set") {
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 10);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_OK);
	CHECK((rd(inCtl(1)) & kSTALL) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall on an IN endpoint clears STALL and the host gets a NAK") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(0x80u | ep), 64, EP_TYPE_BULK);
		REQUIRE(HAL_PCD_EP_SetStall(&h, static_cast<std::uint8_t>(0x80u | ep)) == HAL_OK);
		CHECK(HAL_PCD_EP_ClrStall(&h, static_cast<std::uint8_t>(0x80u | ep)) == HAL_OK);
		CHECK((rd(inCtl(ep)) & kSTALL) == 0u);
		std::uint8_t buf[4];
		unsigned n = 0;
		CHECK(hostOtgHostIn(ep, buf, sizeof buf, &n) == HOSTOTG_NAK);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall on an OUT endpoint clears STALL") {
	for (unsigned ep = 1; ep <= 3; ++ep) {
		CAPTURE(ep);
		open(static_cast<std::uint8_t>(ep), 64, EP_TYPE_BULK);
		REQUIRE(HAL_PCD_EP_SetStall(&h, static_cast<std::uint8_t>(ep)) == HAL_OK);
		CHECK(HAL_PCD_EP_ClrStall(&h, static_cast<std::uint8_t>(ep)) == HAL_OK);
		CHECK((rd(outCtl(ep)) & kSTALL) == 0u);
		const std::uint8_t d[1] = {1};
		CHECK(hostOtgHostOut(ep, d, 1, 0) != HOSTOTG_STALL);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall resets the data toggle of a bulk or interrupt endpoint to DATA0") {
	// RM0090 34.17.6: a ClearFeature(ENDPOINT_HALT) restarts the toggle at DATA0 (SD0PID).
	const std::uint8_t types[] = {EP_TYPE_BULK, EP_TYPE_INTR};
	for (std::uint8_t type : types) {
		CAPTURE(type);
		open(0x81, 64, type);
		open(0x01, 64, type);
		REQUIRE(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_OK);
		REQUIRE(HAL_PCD_EP_SetStall(&h, 0x01) == HAL_OK);
		REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x81) == HAL_OK);
		CHECK((lw(inCtl(1)) & kSD0PID) != 0u);
		REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x01) == HAL_OK);
		CHECK((lw(outCtl(1)) & kSD0PID) != 0u);
		REQUIRE(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
		REQUIRE(HAL_PCD_EP_Close(&h, 0x01) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall of a control or isochronous endpoint does not write the data toggle bit") {
	// The toggle of an isochronous endpoint is not a data PID: SD0PID / SEVNFRM stays out of the write.
	open(0x81, 64, EP_TYPE_ISOC);
	open(0x02, 64, EP_TYPE_CTRL);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x02) == HAL_OK);
	REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x81) == HAL_OK);
	CHECK((lw(inCtl(1)) & kSD0PID) == 0u);
	REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x02) == HAL_OK);
	CHECK((lw(outCtl(2)) & kSD0PID) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall keeps an armed IN endpoint enabled") {
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 10);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x81) == HAL_OK);
	CHECK((rd(inCtl(1)) & kEPENA) != 0u);
	CHECK((rd(inCtl(1)) & kSTALL) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall keeps an armed OUT endpoint enabled") {
	open(0x01, 64, EP_TYPE_BULK);
	arm(1, 64);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x01) == HAL_OK);
	REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x01) == HAL_OK);
	CHECK((rd(outCtl(1)) & kEPENA) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall records the endpoint structure and leaves the handle unlocked") {
	open(0x82, 64, EP_TYPE_BULK);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x82) == HAL_OK);
	CHECK(HAL_PCD_EP_ClrStall(&h, 0x82) == HAL_OK);
	CHECK(h.IN_ep[2].is_stall == 0);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall on endpoint 0 clears STALL of both directions") {
	open(0x80, 64, EP_TYPE_CTRL);
	open(0x00, 64, EP_TYPE_CTRL);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x80) == HAL_OK);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x00) == HAL_OK);
	CHECK(HAL_PCD_EP_ClrStall(&h, 0x80) == HAL_OK);
	CHECK(HAL_PCD_EP_ClrStall(&h, 0x00) == HAL_OK);
	CHECK((rd(inCtl(0)) & kSTALL) == 0u);
	CHECK((rd(outCtl(0)) & kSTALL) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall on an endpoint number above the endpoint count fails") {
	CHECK(HAL_PCD_EP_ClrStall(&h, 0x85) == HAL_ERROR);
	CHECK(HAL_PCD_EP_ClrStall(&h, 0x05) == HAL_ERROR);
	CHECK(HAL_PCD_EP_ClrStall(&h, 0x8F) == HAL_ERROR);
	CHECK(hostOtgReservedAccesses() == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall on the endpoint number equal to the endpoint count fails" * KNOWN_ISSUE("V123 (arguable, same as the SetStall case): HAL_PCD_EP_ClrStall (stm32f4xx_hal_pcd.c line 2008) rejects only numbers GREATER than dev_endpoints, so endpoint 4 of a 4 endpoint device (0-3) is accepted and its reserved registers are written")) {
	CHECK(HAL_PCD_EP_ClrStall(&h, 0x84) == HAL_ERROR);
	clearFaults();
}

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall on a locked handle returns HAL_BUSY") {
	open(0x81, 64, EP_TYPE_BULK);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_OK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_PCD_EP_ClrStall(&h, 0x81) == HAL_BUSY);
	CHECK((rd(inCtl(1)) & kSTALL) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 aborting an armed IN endpoint disables it with NAK and reports HAL_OK") {
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 10);
	REQUIRE((rd(inCtl(1)) & kEPENA) != 0u);
	CHECK(HAL_PCD_EP_Abort(&h, 0x81) == HAL_OK);
	CHECK((rd(inCtl(1)) & kEPENA) == 0u);
	CHECK((rd(inCtl(1)) & kNAKSTS) != 0u);
	CHECK((rd(inInt(1)) & kINT_EPDISD) != 0u);
	CHECK((rd(inInt(1)) & kINT_INEPNE) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 aborting an armed OUT endpoint disables it with NAK and reports HAL_OK") {
	open(0x02, 64, EP_TYPE_BULK);
	arm(2, 64);
	REQUIRE((rd(outCtl(2)) & kEPENA) != 0u);
	CHECK(HAL_PCD_EP_Abort(&h, 0x02) == HAL_OK);
	CHECK((rd(outCtl(2)) & kEPENA) == 0u);
	CHECK((rd(outCtl(2)) & kNAKSTS) != 0u);
	CHECK((rd(outInt(2)) & kINT_EPDISD) != 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 aborting an endpoint that is not armed writes nothing") {
	open(0x81, 64, EP_TYPE_BULK);
	open(0x01, 64, EP_TYPE_BULK);
	const std::size_t wi = hostRegsWritesTo(inCtl(1)), wo = hostRegsWritesTo(outCtl(1));
	CHECK(HAL_PCD_EP_Abort(&h, 0x81) == HAL_OK);
	CHECK(HAL_PCD_EP_Abort(&h, 0x01) == HAL_OK);
	CHECK(hostRegsWritesTo(inCtl(1)) == wi);
	CHECK(hostRegsWritesTo(outCtl(1)) == wo);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 aborting an armed endpoint 0 disables it") {
	open(0x80, 64, EP_TYPE_CTRL);
	open(0x00, 64, EP_TYPE_CTRL);
	send(0, 10);
	arm(0, 64);
	CHECK(HAL_PCD_EP_Abort(&h, 0x80) == HAL_OK);
	CHECK(HAL_PCD_EP_Abort(&h, 0x00) == HAL_OK);
	CHECK((rd(inCtl(0)) & kEPENA) == 0u);
	CHECK((rd(outCtl(0)) & kEPENA) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 an aborted endpoint can start a new transfer") {
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 10);
	REQUIRE(HAL_PCD_EP_Abort(&h, 0x81) == HAL_OK);
	send(1, 20);
	CHECK((rd(inCtl(1)) & kEPENA) != 0u);
	CHECK((rd(inCtl(1)) & kNAKSTS) == 0u);
	CHECK(xfrsizOf(rd(inTsiz(1))) == 20u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 aborting an armed OUT endpoint first sets the global OUT NAK" * KNOWN_ISSUE("V122 (arguable, same as the Close case): RM0090 'Disabling an OUT endpoint' step 1: SGONAK = 1 in DCTL before disabling an OUT endpoint, but USB_EPStopXfer (stm32f4xx_ll_usb.c lines 967-968) sets SNAK and EPDIS without it")) {
	open(0x02, 64, EP_TYPE_BULK);
	arm(2, 64);
	HAL_PCD_EP_Abort(&h, 0x02);
	CHECK((rd(kDCTL) & (1u << 3)) != 0u);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE4 aborting an armed IN endpoint leaves the data in its TxFIFO (NR346_i)") {
	// RM0090 IN endpoint disable step 7: the application flushes the FIFO itself; HAL_PCD_EP_Abort does not.
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 8);
	REQUIRE(USB_WritePacket(USB_OTG_FS, tx, 1, 8, 0) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Abort(&h, 0x81) == HAL_OK);
	CHECK(hostOtgTxWordsUsed(1) == 2u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 flushing an IN endpoint empties the TxFIFO of that endpoint only") {
	for (unsigned ep = 0; ep <= 3; ++ep) {
		CAPTURE(ep);
		for (unsigned f = 0; f <= 3; ++f) {
			REQUIRE(USB_FlushTxFifo(USB_OTG_FS, f) == HAL_OK);
			REQUIRE(USB_WritePacket(USB_OTG_FS, tx, static_cast<std::uint8_t>(f), 12, 0) == HAL_OK);
		}
		CHECK(HAL_PCD_EP_Flush(&h, static_cast<std::uint8_t>(0x80u | ep)) == HAL_OK);
		for (unsigned f = 0; f <= 3; ++f) CHECK(hostOtgTxWordsUsed(f) == (f == ep ? 0u : 3u));
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE4 flushing an IN endpoint writes TXFFLSH with the FIFO number in TXFNUM") {
	// RM0090 GRSTCTL: TXFFLSH bit 5, TXFNUM bits 10:6. FIFO 2: 0x20 | (2 << 6) = 0xA0.
	CHECK(HAL_PCD_EP_Flush(&h, 0x82) == HAL_OK);
	CHECK(lw(kGRSTCTL) == 0xA0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 flushing an OUT endpoint empties the receive FIFO and no TxFIFO") {
	hostOtgRxFill(0x12345678u, 5);
	REQUIRE(USB_WritePacket(USB_OTG_FS, tx, 1, 8, 0) == HAL_OK);
	CHECK(HAL_PCD_EP_Flush(&h, 0x01) == HAL_OK);
	CHECK(hostOtgRxWordsUsed() == 0u);
	CHECK(hostOtgTxWordsUsed(1) == 2u);
	CHECK(lw(kGRSTCTL) == 0x10u); // RXFFLSH, bit 4
}

TEST_CASE_FIXTURE(PeFixture, "PE4 flushing the receive FIFO drops the pending packet status entries") {
	open(0x01, 64, EP_TYPE_BULK);
	arm(1, 64);
	REQUIRE(hostOtgHostOut(1, tx, 10, 0) == HOSTOTG_ACK);
	REQUIRE(hostOtgRxEntries() > 0u);
	CHECK(HAL_PCD_EP_Flush(&h, 0x00) == HAL_OK);
	CHECK(hostOtgRxEntries() == 0u);
	CHECK((rd(kGINTSTS) & (1u << 4)) == 0u); // RXFLVL
}

TEST_CASE_FIXTURE(PeFixture, "PE4 flushing an endpoint on a locked handle returns HAL_BUSY and flushes nothing") {
	REQUIRE(USB_WritePacket(USB_OTG_FS, tx, 1, 8, 0) == HAL_OK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_PCD_EP_Flush(&h, 0x81) == HAL_BUSY);
	CHECK(hostOtgTxWordsUsed(1) == 2u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 flushing leaves the handle unlocked") {
	CHECK(HAL_PCD_EP_Flush(&h, 0x81) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
	CHECK(HAL_PCD_EP_Flush(&h, 0x01) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_FlushTxFifo empties the selected TxFIFO and reports HAL_OK") {
	REQUIRE(USB_WritePacket(USB_OTG_FS, tx, 3, 16, 0) == HAL_OK);
	CHECK(USB_FlushTxFifo(USB_OTG_FS, 3) == HAL_OK);
	CHECK(hostOtgTxWordsUsed(3) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_FlushTxFifo with the all FIFOs number empties every TxFIFO") {
	// RM0090 TXFNUM 10000: flush all the transmit FIFOs.
	for (unsigned f = 0; f <= 3; ++f) REQUIRE(USB_WritePacket(USB_OTG_FS, tx, static_cast<std::uint8_t>(f), 8, 0) == HAL_OK);
	CHECK(USB_FlushTxFifo(USB_OTG_FS, 16) == HAL_OK);
	for (unsigned f = 0; f <= 3; ++f) CHECK(hostOtgTxWordsUsed(f) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_FlushTxFifo that needs several reads of GRSTCTL still ends with HAL_OK") {
	// The flush bit clears itself at the 3rd read: the HAL loop of USB_FlushTxFifo polls until then (RM0090 34.16.2: "The application must poll until the bit is cleared").
	hostOtgSetPollReads(3);
	REQUIRE(USB_WritePacket(USB_OTG_FS, tx, 1, 8, 0) == HAL_OK);
	CHECK(USB_FlushTxFifo(USB_OTG_FS, 1) == HAL_OK);
	CHECK(hostOtgTxWordsUsed(1) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_FlushRxFifo empties the receive FIFO and reports HAL_OK") {
	hostOtgRxFill(0x01u, 4);
	CHECK(USB_FlushRxFifo(USB_OTG_FS) == HAL_OK);
	CHECK(hostOtgRxWordsUsed() == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_FlushRxFifo that needs several reads of GRSTCTL still ends with HAL_OK") {
	hostOtgSetPollReads(4);
	hostOtgRxFill(0x01u, 4);
	CHECK(USB_FlushRxFifo(USB_OTG_FS) == HAL_OK);
	CHECK(hostOtgRxWordsUsed() == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_ActivateSetup selects the 64 byte packet size of IN endpoint 0") {
	// RM0090 DIEPCTL0 MPSIZ code 00 = 64 bytes.
	wr(inCtl(0), rd(inCtl(0)) | 3u);
	REQUIRE(mpsizOf(rd(inCtl(0))) == 3u);
	CHECK(USB_ActivateSetup(USB_OTG_FS) == HAL_OK);
	CHECK(mpsizOf(rd(inCtl(0))) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_ActivateSetup clears the global IN NAK") {
	// RM0090 DCTL: CGINAK (bit 8) clears the global IN NAK; GINSTS (bit 2) reads the state.
	*reinterpret_cast<volatile std::uint32_t *>(static_cast<std::uintptr_t>(kDCTL)) = 1u << 7; // SGINAK through a bus write
	REQUIRE((rd(kDCTL) & (1u << 2)) != 0u);
	CHECK(USB_ActivateSetup(USB_OTG_FS) == HAL_OK);
	CHECK((lw(kDCTL) & (1u << 8)) != 0u);
	CHECK((rd(kDCTL) & (1u << 2)) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_EP0_OutStart programs one packet, 24 bytes and 3 SETUP packets") {
	CHECK(USB_EP0_OutStart(USB_OTG_FS, 0, reinterpret_cast<const std::uint8_t *>(h.Setup)) == HAL_OK);
	CHECK(rd(outTsiz(0)) == 0x60080018u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_EP0_OutStart replaces the previous contents of the transfer size register") {
	wr(outTsiz(0), 0x0000007Fu);
	CHECK(USB_EP0_OutStart(USB_OTG_FS, 0, reinterpret_cast<const std::uint8_t *>(h.Setup)) == HAL_OK);
	CHECK(rd(outTsiz(0)) == 0x60080018u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_EP0_OutStart in slave mode does not enable the OUT endpoint 0") {
	// RM0090 34.17.5: only STUPCNT is programmed; the core receives SETUP packets irrespective of EPENA.
	const std::uint32_t before = rd(outCtl(0));
	REQUIRE(USB_EP0_OutStart(USB_OTG_FS, 0, reinterpret_cast<const std::uint8_t *>(h.Setup)) == HAL_OK);
	CHECK(rd(outCtl(0)) == before);
	CHECK((rd(outCtl(0)) & kEPENA) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 after USB_EP0_OutStart a SETUP packet is accepted and decrements the SETUP counter") {
	open(0x00, 64, EP_TYPE_CTRL);
	REQUIRE(USB_EP0_OutStart(USB_OTG_FS, 0, reinterpret_cast<const std::uint8_t *>(h.Setup)) == HAL_OK);
	const std::uint8_t setup[8] = {0x80, 6, 0, 1, 0, 0, 18, 0};
	CHECK(hostOtgHostSetup(setup) == HOSTOTG_ACK);
	CHECK(((rd(outTsiz(0)) >> 29) & 3u) == 2u); // RM0090: STUPCNT is decremented for every SETUP packet
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_EPSetStall and USB_EPClearStall act on the register of the endpoint given") {
	USB_OTG_EPTypeDef epIn, epOut;
	std::memset(&epIn, 0, sizeof epIn);
	std::memset(&epOut, 0, sizeof epOut);
	epIn.num = 2;
	epIn.is_in = 1;
	epIn.type = EP_TYPE_BULK;
	epOut.num = 2;
	epOut.is_in = 0;
	epOut.type = EP_TYPE_BULK;
	open(0x82, 64, EP_TYPE_BULK);
	open(0x02, 64, EP_TYPE_BULK);
	CHECK(USB_EPSetStall(USB_OTG_FS, &epIn) == HAL_OK);
	CHECK((rd(inCtl(2)) & kSTALL) != 0u);
	CHECK((rd(outCtl(2)) & kSTALL) == 0u);
	CHECK(USB_EPSetStall(USB_OTG_FS, &epOut) == HAL_OK);
	CHECK((rd(outCtl(2)) & kSTALL) != 0u);
	CHECK(USB_EPClearStall(USB_OTG_FS, &epIn) == HAL_OK);
	CHECK((rd(inCtl(2)) & kSTALL) == 0u);
	CHECK((rd(outCtl(2)) & kSTALL) != 0u);
	CHECK(USB_EPClearStall(USB_OTG_FS, &epOut) == HAL_OK);
	CHECK((rd(outCtl(2)) & kSTALL) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_EPStopXfer of an armed endpoint reports HAL_OK and of an idle one does too") {
	USB_OTG_EPTypeDef ep;
	std::memset(&ep, 0, sizeof ep);
	ep.num = 1;
	ep.is_in = 1;
	open(0x81, 64, EP_TYPE_BULK);
	CHECK(USB_EPStopXfer(USB_OTG_FS, &ep) == HAL_OK);
	send(1, 8);
	CHECK(USB_EPStopXfer(USB_OTG_FS, &ep) == HAL_OK);
	CHECK((rd(inCtl(1)) & kEPENA) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 the model reports no reserved register access during endpoint setup, transfers, stall and flush") {
	open(0x80, 64, EP_TYPE_CTRL);
	open(0x00, 64, EP_TYPE_CTRL);
	open(0x81, 64, EP_TYPE_BULK);
	open(0x01, 64, EP_TYPE_BULK);
	send(1, 70);
	arm(1, 70);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Abort(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Flush(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Close(&h, 0x01) == HAL_OK);
	CHECK(hostOtgReservedAccesses() == 0u);
	CHECK(faults() == 0u);
}

// ================================================================================================= fix pass additions

TEST_CASE_FIXTURE(PeFixture, "PE4 clearing a stall of an IN control endpoint and of an OUT isochronous endpoint does not write the data toggle bit") {
	open(0x81, 64, EP_TYPE_CTRL);
	open(0x02, 64, EP_TYPE_ISOC);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x02) == HAL_OK);
	hostRegsLogClear();
	REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x81) == HAL_OK);
	REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x02) == HAL_OK);
	CHECK(!wroteBits(inCtl(1), kSD0PID));
	CHECK(!wroteBits(outCtl(2), kSD0PID));
	CHECK((rd(inCtl(1)) & kSTALL) == 0u);
	CHECK((rd(outCtl(2)) & kSTALL) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening with the largest 11 bit maximum packet size keeps all bits") {
	open(0x81, 0x7FF, EP_TYPE_BULK);
	open(0x01, 0x7FF, EP_TYPE_BULK);
	CHECK(mpsizOf(rd(inCtl(1))) == 0x7FFu);
	CHECK(mpsizOf(rd(outCtl(1))) == 0x7FFu);
	CHECK(h.IN_ep[1].maxpacket == 0x7FFu);
}

TEST_CASE_FIXTURE(PeFixture, "PE1 opening with a maximum packet size of 0x400 keeps bit 10") {
	open(0x82, 0x400, EP_TYPE_BULK);
	open(0x02, 0x400, EP_TYPE_BULK);
	CHECK(mpsizOf(rd(inCtl(2))) == 0x400u);
	CHECK(mpsizOf(rd(outCtl(2))) == 0x400u);
	CHECK(h.OUT_ep[2].maxpacket == 0x400u);
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 opening with a maximum packet size above 11 bits drops the bit 11 (NR346_o)") {
	// MPSIZ is 11 bits (RM0090); the HAL masks the uint16 argument with 0x7FF: 0x840 gives 0x40.
	open(0x83, 0x840, EP_TYPE_BULK);
	CHECK(h.IN_ep[3].maxpacket == 0x40u);
	CHECK(mpsizOf(rd(inCtl(3))) == 0x40u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_FlushTxFifo returns HAL_TIMEOUT when the flush bit never clears") {
	hostOtgSetPollReads(0);
	CHECK(USB_FlushTxFifo(USB_OTG_FS, 1) == HAL_TIMEOUT);
	CHECK(lw(kGRSTCTL) == 0x60u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_FlushRxFifo returns HAL_TIMEOUT when the flush bit never clears") {
	hostOtgSetPollReads(0);
	CHECK(USB_FlushRxFifo(USB_OTG_FS) == HAL_TIMEOUT);
	CHECK(lw(kGRSTCTL) == 0x10u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_FlushTxFifo waits for the AHB master idle bit before it flushes") {
	// RM0090 GRSTCTL AHBIDL: the application polls it before a flush; with AHBIDL = 0 for ever no flush is started.
	wr(kGRSTCTL, 0u);
	const std::size_t writes = hostRegsWritesTo(kGRSTCTL);
	REQUIRE(USB_WritePacket(USB_OTG_FS, tx, 1, 8, 0) == HAL_OK);
	CHECK(USB_FlushTxFifo(USB_OTG_FS, 1) == HAL_TIMEOUT);
	CHECK(hostRegsWritesTo(kGRSTCTL) == writes);
	CHECK(hostOtgTxWordsUsed(1) == 2u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_FlushRxFifo waits for the AHB master idle bit before it flushes") {
	wr(kGRSTCTL, 0u);
	const std::size_t writes = hostRegsWritesTo(kGRSTCTL);
	hostOtgRxFill(1u, 3);
	CHECK(USB_FlushRxFifo(USB_OTG_FS) == HAL_TIMEOUT);
	CHECK(hostRegsWritesTo(kGRSTCTL) == writes);
	CHECK(hostOtgRxWordsUsed() == 3u);
	CHECK(USB_ReadPacket(USB_OTG_FS, tx, 12) != nullptr); // empties the model
}

TEST_CASE_FIXTURE(PeFixture, "PE4 flushing an IN endpoint writes the FIFO number of every endpoint into TXFNUM") {
	for (unsigned ep = 0; ep <= 3; ++ep) {
		CAPTURE(ep);
		REQUIRE(HAL_PCD_EP_Flush(&h, static_cast<std::uint8_t>(0x80u | ep)) == HAL_OK);
		CHECK(lw(kGRSTCTL) == (0x20u | (ep << 6)));
	}
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_EPStopXfer of an IN endpoint that never finishes disabling returns HAL_ERROR") {
	USB_OTG_EPTypeDef ep;
	std::memset(&ep, 0, sizeof ep);
	ep.num = 1;
	ep.is_in = 1;
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 8);
	hostOtgHoldEndpointDisable(1);
	CHECK(USB_EPStopXfer(USB_OTG_FS, &ep) == HAL_ERROR);
	CHECK((rd(inCtl(1)) & kEPENA) != 0u);
	hostOtgReleaseDisable();
	CHECK((rd(inCtl(1)) & kEPENA) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 USB_EPStopXfer of an OUT endpoint that never finishes disabling returns HAL_ERROR") {
	USB_OTG_EPTypeDef ep;
	std::memset(&ep, 0, sizeof ep);
	ep.num = 2;
	ep.is_in = 0;
	open(0x02, 64, EP_TYPE_BULK);
	arm(2, 64);
	hostOtgHoldEndpointDisable(1);
	CHECK(USB_EPStopXfer(USB_OTG_FS, &ep) == HAL_ERROR);
	CHECK((rd(outCtl(2)) & kEPENA) != 0u);
	hostOtgReleaseDisable();
	CHECK((rd(outCtl(2)) & kEPENA) == 0u);
}

TEST_CASE_FIXTURE(PeFixture, "PE4 aborting an IN endpoint whose disable does not finish returns HAL_ERROR") {
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 8);
	hostOtgHoldEndpointDisable(1);
	CHECK(HAL_PCD_EP_Abort(&h, 0x81) == HAL_ERROR);
	hostOtgReleaseDisable();
}

TEST_CASE_FIXTURE(PeFixture, "PE4 aborting an OUT endpoint whose disable does not finish returns HAL_ERROR") {
	open(0x01, 64, EP_TYPE_BULK);
	arm(1, 64);
	hostOtgHoldEndpointDisable(1);
	CHECK(HAL_PCD_EP_Abort(&h, 0x01) == HAL_ERROR);
	hostOtgReleaseDisable();
}

// NEEDS REVIEW 346
TEST_CASE_FIXTURE(PeFixture, "characterization: PE1 closing an endpoint whose disable does not finish still returns HAL_OK and deactivates it (NR346_p)") {
	// HAL_PCD_EP_Close does not wait for EPDISD (the Close paths of USB_DeactivateEndpoint never poll).
	open(0x81, 64, EP_TYPE_BULK);
	send(1, 8);
	hostOtgHoldEndpointDisable(1);
	CHECK(HAL_PCD_EP_Close(&h, 0x81) == HAL_OK);
	CHECK((rd(inCtl(1)) & kUSBAEP) == 0u);
	hostOtgReleaseDisable();
}

TEST_CASE_FIXTURE(PeFixture, "PE4 flushing an endpoint whose flush times out reports the timeout" * KNOWN_ISSUE("V124 (arguable: UM1725 only says 'Return values: HAL status'): HAL_PCD_EP_Flush (stm32f4xx_hal_pcd.c lines 2093-2103) casts the result of USB_FlushTxFifo / USB_FlushRxFifo to void and returns HAL_OK, so a flush that times out is reported as success")) {
	hostOtgSetPollReads(0);
	CHECK(HAL_PCD_EP_Flush(&h, 0x81) != HAL_OK);
	CHECK(HAL_PCD_EP_Flush(&h, 0x01) != HAL_OK);
}
