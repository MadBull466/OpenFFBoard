// P12 T8 group PI: HAL_PCD_Init / DeInit / Start / Stop / DevConnect / DevDisconnect / SetAddress / GetState / SetTestMode / ActivateRemoteWakeup /
// DeActivateRemoteWakeup / WKUP_IRQHandler, HAL_PCDEx_SetRxFiFo / SetTxFiFo, and the LL USB core functions USB_CoreInit (with the static USB_CoreReset),
// USB_SetCurrentMode, USB_GetMode, USB_DevInit, USB_SetDevSpeed, USB_GetDevSpeed, USB_SetTurnaroundTime, USB_Enable / DisableGlobalInt, USB_DevConnect /
// DevDisconnect / SetDevAddress / StopDevice, USB_Activate / DeActivateRemoteWakeup. Real ST HAL, OTG_FS device mode, F407VG and F407VG_DISCO.
// Oracles: UM1725 Rev 8 "HAL PCD Generic Driver" / "HAL PCD Extension Driver"; RM0090 Rev 22 chapter 34 OTG_FS: 34.13.1 FIFO RAM allocation, 34.16 registers
// (GAHBCFG, GUSBCFG Table 203 TRDT values, GRSTCTL, GINTMSK, GCCFG, DCFG, DCTL, DSTS, PCGCCTL), 34.17.1 core initialization, 34.17.3 device initialization.
// Charter: P12_T8_REQUIREMENTS.md (group PI). The USB HAL's LPM / BCD functions (HAL_PCDEx_ActivateLPM ... BCD_VBUSDetect) are compiled only for the
// F446 / F469 / F479 / F412 / F413 / F423 devices (stm32f4xx_hal_pcd_ex.c): they do not exist in the F407VG build and are not tested.
// Characterization cases carry NR345_<n> (1-22, local numbers, the lead renumbers): 1 GINTMSK mask, 2 DIEPTXF4-15 writes, 3 dev_endpoints 5, 4 ULPI, 5 DeInit flush failure,
// 6 DeInit keeps SDIS, 7 SetAddress above 127, 8 remote wakeup needs suspend, 9 TxFIFO order, 10 FIFO depth limits, 11 FIFO number 4, 12 CMOD timeout, 13 invalid mode,
// 14 SetDevSpeed ORs, 15 reserved ENUMSPD, 16-18 TRDT edges / HS code, 19 SRQINT kept, 20 StopDevice endpoints 4-14, 21 SetTestMode ORs, 22 IN EP0 stays enabled.
// KNOWN_ISSUE: V119 (SetCurrentMode last poll).
#include "OtgTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>

// The HAL polling loops of the LL FIFO flush and core reset give up after HAL_USB_TIMEOUT iterations; the HAL build sets it to 2000 (CMake), the timeout cases
// below need it small: refuse to build otherwise so they cannot be silently lost.
#if HAL_USB_TIMEOUT > 100000
#error "the PI timeout cases need HAL_USB_TIMEOUT <= 100000 (set in CMake for the HAL builds)"
#endif

namespace {

constexpr uint32_t kBase = HOST_OTG_BASE;

// Register offsets (RM0090 Tables 198-201).
constexpr uint32_t oGAHBCFG = 0x008, oGUSBCFG = 0x00C, oGRSTCTL = 0x010, oGINTSTS = 0x014, oGINTMSK = 0x018, oGRXFSIZ = 0x024, oDIEPTXF0 = 0x028, oGCCFG = 0x038;
constexpr uint32_t oDIEPTXF1 = 0x104;
constexpr uint32_t oDCFG = 0x800, oDCTL = 0x804, oDSTS = 0x808, oDIEPMSK = 0x810, oDOEPMSK = 0x814, oDAINT = 0x818, oDAINTMSK = 0x81C, oPCGCCTL = 0xE00;
constexpr uint32_t oEXTI_PR = 0x40013C14;

constexpr uint32_t inCtl(unsigned n) { return 0x900u + 0x20u * n; }
constexpr uint32_t inInt(unsigned n) { return inCtl(n) + 0x8u; }
constexpr uint32_t inSiz(unsigned n) { return inCtl(n) + 0x10u; }
constexpr uint32_t outCtl(unsigned n) { return 0xB00u + 0x20u * n; }
constexpr uint32_t outInt(unsigned n) { return outCtl(n) + 0x8u; }
constexpr uint32_t outSiz(unsigned n) { return outCtl(n) + 0x10u; }

// Bits.
constexpr uint32_t GINT = 1u << 0;
constexpr uint32_t M_OTGINT = 1u << 2, M_SOF = 1u << 3, M_RXFLVL = 1u << 4, M_ESUSP = 1u << 10, M_USBSUSP = 1u << 11, M_USBRST = 1u << 12, M_ENUMDNE = 1u << 13;
constexpr uint32_t M_IEPINT = 1u << 18, M_OEPINT = 1u << 19, M_IISOIXFR = 1u << 20, M_PXFR = 1u << 21, M_LPM = 1u << 27, M_SRQ = 1u << 30, M_WUI = 1u << 31, M_MMIS = 1u << 1;
constexpr uint32_t G_PHYSEL = 1u << 6, G_FHMOD = 1u << 29, G_FDMOD = 1u << 30;
constexpr uint32_t CC_PWRDWN = 1u << 16, CC_VBUSASEN = 1u << 18, CC_VBUSBSEN = 1u << 19, CC_SOFOUTEN = 1u << 20, CC_NOVBUSSENS = 1u << 21;
constexpr uint32_t DCTL_RWUSIG = 1u << 0, DCTL_SDIS = 1u << 1;
constexpr uint32_t DSTS_SUSPSTS = 1u << 0;
constexpr uint32_t C_EPENA = 1u << 31, C_EPDIS = 1u << 30, C_SNAK = 1u << 27, C_STALL = 1u << 21, C_NAKSTS = 1u << 17, C_USBAEP = 1u << 15;
constexpr uint32_t I_XFRC = 1u << 0, I_EPDISD = 1u << 1, I_STUP_OR_AHBERR = 1u << 3, I_TOC_OTEPDIS = 1u << 4, I_INEPNE = 1u << 6;
constexpr uint32_t kMspMarkerPcgc = 0x3u;  // PCGCCTL STPPCLK | GATEHCLK (bits 1:0, RM0090 OTG_FS_PCGCCTL)

uint32_t rd(uint32_t off) { return hostRegsPeek(kBase + off); }
void wr(uint32_t off, uint32_t v) { hostRegsPoke(kBase + off, v); }
uint32_t tctl() { return (rd(oDCTL) >> 4) & 7u; }
uint32_t dad() { return (rd(oDCFG) >> 4) & 0x7Fu; }
uint32_t dspd() { return rd(oDCFG) & 3u; }

// A CPU write into the data FIFO window of an IN endpoint (goes through the model like on the target).
void fifoWrite(unsigned ep, uint32_t word) { *reinterpret_cast<volatile uint32_t *>(kBase + 0x1000u * (ep + 1u)) = word; }

// Hooks run by HAL_GetTick (once per call): they let a register change while the HAL waits in HAL_Delay.
PCD_HandleTypeDef *piHandle = nullptr;
unsigned piStateCalls = 0, piStateBusyCalls = 0;
uint32_t piCmodThreshold = 0;   // CMOD (GINTSTS bit 0) follows the tick: 1 before this tick, 0 from it on (device mode wanted)
uint32_t piHostCmodAt = 0;      // CMOD = 1 from this tick on (host mode wanted)

void piStepObserveState() {
	HAL_IncTick();
	++piStateCalls;
	if (piHandle != nullptr && piHandle->State == HAL_PCD_STATE_BUSY) ++piStateBusyCalls;
}
void piStepHostForever() {  // the core stays in host mode
	HAL_IncTick();
	wr(oGINTSTS, rd(oGINTSTS) | 1u);
}
void piStepDeviceAtThreshold() {
	HAL_IncTick();
	wr(oGINTSTS, hostTickPeek() >= piCmodThreshold ? (rd(oGINTSTS) & ~1u) : (rd(oGINTSTS) | 1u));
}
void piStepHostAt() {
	HAL_IncTick();
	if (hostTickPeek() >= piHostCmodAt) wr(oGINTSTS, rd(oGINTSTS) | 1u);
}
void piStepFlushNeverCompletes() {  // after CSRST has completed: the flush handshakes never finish
	HAL_IncTick();
	hostOtgSetPollReads(0);
}

class PiFx : public OtgFixture {
public:
	PiFx() {
		piHandle = &h;
		piStateCalls = piStateBusyCalls = 0;
		piCmodThreshold = piHostCmodAt = 0;
	}
	~PiFx() { piHandle = nullptr; }
	// a handle poisoned so that "untouched" fields are visible
	void poison() { std::memset(&h.IN_ep, 0x5A, sizeof h.IN_ep); std::memset(&h.OUT_ep, 0x5A, sizeof h.OUT_ep); }
};

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------------------------------
// PI1 HAL_PCD_Init / HAL_PCD_DeInit / HAL_PCD_GetState
// ---------------------------------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init of a NULL handle returns HAL_ERROR and touches nothing") {
	CHECK(HAL_PCD_Init(nullptr) == HAL_ERROR);
	CHECK(otgCounts.mspInit == 0);
	CHECK(hostRegsLogCount() == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init of a fresh handle calls MspInit once and ends in state READY with HAL_OK") {
	CHECK(h.State == HAL_PCD_STATE_RESET);
	CHECK(HAL_PCD_Init(&h) == HAL_OK);
	CHECK(otgCounts.mspInit == 1);
	CHECK(otgLastHandle == &h);
	CHECK(h.State == HAL_PCD_STATE_READY);
	CHECK(HAL_PCD_GetState(&h) == HAL_PCD_STATE_READY);
	CHECK(faults() == 0);
	CHECK(hostOtgReservedAccesses() == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init leaves the handle unlocked even if it was locked while in state RESET") {
	h.Lock = HAL_LOCKED;  // UM1725: the lock resource is initialised when the state is RESET
	CHECK(HAL_PCD_Init(&h) == HAL_OK);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init holds the state BUSY while it configures the core") {
	hostTickSetStep(piStepObserveState);  // HAL_Delay of USB_SetCurrentMode runs inside HAL_PCD_Init
	CHECK(HAL_PCD_Init(&h) == HAL_OK);
	CHECK(piStateCalls > 0);
	CHECK(piStateBusyCalls == piStateCalls);
	CHECK(h.State == HAL_PCD_STATE_READY);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init of an already READY handle does not call MspInit again") {
	REQUIRE(init() == HAL_OK);
	CHECK(init() == HAL_OK);
	CHECK(otgCounts.mspInit == 1);
	CHECK(h.State == HAL_PCD_STATE_READY);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init of a handle in state RESET after DeInit calls MspInit again") {
	REQUIRE(init() == HAL_OK);
	REQUIRE(HAL_PCD_DeInit(&h) == HAL_OK);
	CHECK(init() == HAL_OK);
	CHECK(otgCounts.mspInit == 2);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init selects the full speed transceiver and forces device mode") {
	REQUIRE(init() == HAL_OK);
	// RM0090 GUSBCFG: PHYSEL 1 = full speed serial transceiver, FDMOD forces device mode, FHMOD (forced host) stays 0
	CHECK((rd(oGUSBCFG) & G_PHYSEL) == G_PHYSEL);
	CHECK((rd(oGUSBCFG) & G_FDMOD) == G_FDMOD);
	CHECK((rd(oGUSBCFG) & G_FHMOD) == 0);
	CHECK(USB_GetMode(USB_OTG_FS) == (uint32_t)USB_DEVICE_MODE);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init with VBUS sensing sets VBUSBSEN, clears NOVBUSSENS and activates the transceiver") {
	h.Init.vbus_sensing_enable = ENABLE;
	REQUIRE(init() == HAL_OK);
	// RM0090 GCCFG: VBUSBSEN 1 = VBUS sensing B enabled, NOVBUSSENS 0 = VBUS sensing by hardware, PWRDWN 1 = transceiver active
	CHECK(rd(oGCCFG) == (CC_VBUSBSEN | CC_PWRDWN));
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init without VBUS sensing sets NOVBUSSENS, clears VBUSBSEN and VBUSASEN and keeps the transceiver active") {
	h.Init.vbus_sensing_enable = DISABLE;
	REQUIRE(init() == HAL_OK);
	CHECK(rd(oGCCFG) == (CC_NOVBUSSENS | CC_PWRDWN));
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init with VBUS sensing unmasks the session request and OTG interrupts") {
	REQUIRE(init() == HAL_OK);
	CHECK((rd(oGINTMSK) & (M_SRQ | M_OTGINT)) == (M_SRQ | M_OTGINT));
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init without VBUS sensing leaves the session request and OTG interrupts masked") {
	h.Init.vbus_sensing_enable = DISABLE;
	REQUIRE(init() == HAL_OK);
	CHECK((rd(oGINTMSK) & (M_SRQ | M_OTGINT)) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init unmasks the interrupts a device needs: receive FIFO, reset, enumeration done, suspend, IN and OUT endpoints") {
	REQUIRE(init() == HAL_OK);
	// RM0090 34.17.3 step 2 (USB reset, enumeration done, USB suspend) and the transfer flows of 34.17.5 (RXFLVL, IEPINT, OEPINT)
	const uint32_t need = M_RXFLVL | M_USBRST | M_ENUMDNE | M_USBSUSP | M_IEPINT | M_OEPINT;
	CHECK((rd(oGINTMSK) & need) == need);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init with Sof_enable unmasks the SOF interrupt and without it keeps it masked") {
	h.Init.Sof_enable = ENABLE;
	REQUIRE(init() == HAL_OK);
	CHECK((rd(oGINTMSK) & M_SOF) == M_SOF);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init without Sof_enable keeps the SOF interrupt masked") {
	h.Init.Sof_enable = DISABLE;
	REQUIRE(init() == HAL_OK);
	CHECK((rd(oGINTMSK) & M_SOF) == 0);
}

// NR345_1: RM0090 34.17.3 step 2 says to unmask early suspend and SOF; the HAL leaves ESUSPM masked always and SOFM unless Sof_enable.
// 34.17.1 step 3 says to unmask the mode mismatch interrupt: MMISM stays masked. The exact mask the HAL programs is recorded here:
// RXFLVL, USBSUSP, USBRST, ENUMDNE, IEPINT, OEPINT, IISOIXFR, PXFR_IISOOXFR (bits 4, 11, 12, 13, 18, 19, 20, 21), WUI (31) and with VBUS sensing SRQ (30) and OTGINT (2).
// Hand sum: 0x10 + 0x800 + 0x1000 + 0x2000 + 0x40000 + 0x80000 + 0x100000 + 0x200000 + 0x80000000 + 0x40000000 + 0x4 = 0xC03C3814.
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI1 GINTMSK after HAL_PCD_Init of the firmware init has early suspend, SOF and mode mismatch masked and wakeup unmasked") {
	REQUIRE(init() == HAL_OK);
	CHECK(rd(oGINTMSK) == 0xC03C3814u);
	CHECK((rd(oGINTMSK) & (M_ESUSP | M_SOF | M_MMIS)) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init leaves the global interrupt disabled, so no interrupt reaches the CPU before Start") {
	REQUIRE(init() == HAL_OK);
	CHECK((rd(oGAHBCFG) & GINT) == 0);
	hostOtgVbus(1);  // a pending, unmasked source
	CHECK(hostOtgIrqPending() == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init programs the device speed field for full speed and clears the device address") {
	REQUIRE(init() == HAL_OK);
	CHECK(dspd() == 3u);  // RM0090 DCFG DSPD 11 = full speed
	CHECK(dad() == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init leaves the device in soft disconnect") {
	REQUIRE(init() == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);  // RM0090 DCTL SDIS 1 = the core generates a device disconnect event to the host
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init restarts the PHY clock: PCGCCTL cleared") {
	wr(oPCGCCTL, kMspMarkerPcgc);
	REQUIRE(init() == HAL_OK);  // the core soft reset keeps PCGCCTL, USB_DevInit clears it
	CHECK(rd(oPCGCCTL) == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init zeroes the IN endpoint FIFO size registers DIEPTXF1 to DIEPTXF3") {
	REQUIRE(init() == HAL_OK);
	for (unsigned i = 0; i < 3; ++i) CHECK(rd(oDIEPTXF1 + 4u * i) == 0u);
}

// NR345_2: USB_DevInit clears DIEPTXF[0..14], the OTG_FS core has three (0x104-0x10C). The twelve writes to 0x110-0x13C are accesses to registers that do not exist on this core.
// NEEDS REVIEW 349
TEST_CASE_FIXTURE(PiFx, "characterization: PI1 HAL_PCD_Init writes twelve IN FIFO size registers DIEPTXF4 to DIEPTXF15 that the OTG_FS core does not have") {
	REQUIRE(init() == HAL_OK);
	REQUIRE(hostOtgHsOnlyAccesses() == 12u);
	for (unsigned i = 0; i < 12; ++i) {
		CHECK(hostOtgHsOnlyOffset(i) == 0x110u + 4u * i);
		CHECK(hostOtgHsOnlyWasWrite(i) == 1);
	}
	CHECK(hostOtgReservedAccesses() == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init clears the device interrupt masks") {
	REQUIRE(init() == HAL_OK);
	CHECK(rd(oDIEPMSK) == 0u);
	CHECK(rd(oDOEPMSK) == 0u);
	CHECK(rd(oDAINTMSK) == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init leaves no transfer pending on the four endpoints") {
	REQUIRE(init() == HAL_OK);
	for (unsigned n = 0; n < 4; ++n) {
		CAPTURE(n);
		CHECK((rd(inCtl(n)) & C_EPENA) == 0);
		CHECK((rd(outCtl(n)) & C_EPENA) == 0);
		CHECK(rd(inSiz(n)) == 0u);
		CHECK(rd(outSiz(n)) == 0u);
		CHECK((rd(inInt(n)) & (I_XFRC | I_EPDISD | I_TOC_OTEPDIS | I_INEPNE)) == 0);
		CHECK((rd(outInt(n)) & (I_XFRC | I_EPDISD | I_STUP_OR_AHBERR | I_TOC_OTEPDIS)) == 0);
	}
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init initialises the IN endpoint structures of all four endpoints") {
	poison();
	REQUIRE(init() == HAL_OK);
	for (unsigned i = 0; i < 4; ++i) {
		CAPTURE(i);
		CHECK(h.IN_ep[i].is_in == 1);
		CHECK(h.IN_ep[i].num == i);
		CHECK(h.IN_ep[i].tx_fifo_num == i);  // the IN endpoint n uses TxFIFO n
		CHECK(h.IN_ep[i].type == EP_TYPE_CTRL);
		CHECK(h.IN_ep[i].maxpacket == 0);
		CHECK(h.IN_ep[i].xfer_buff == nullptr);
		CHECK(h.IN_ep[i].xfer_len == 0);
	}
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init initialises the OUT endpoint structures of all four endpoints") {
	poison();
	REQUIRE(init() == HAL_OK);
	for (unsigned i = 0; i < 4; ++i) {
		CAPTURE(i);
		CHECK(h.OUT_ep[i].is_in == 0);
		CHECK(h.OUT_ep[i].num == i);
		CHECK(h.OUT_ep[i].type == EP_TYPE_CTRL);
		CHECK(h.OUT_ep[i].maxpacket == 0);
		CHECK(h.OUT_ep[i].xfer_buff == nullptr);
		CHECK(h.OUT_ep[i].xfer_len == 0);
	}
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init with dev_endpoints 1 initialises only endpoint 0 of both directions") {
	poison();
	h.Init.dev_endpoints = 1;
	REQUIRE(init() == HAL_OK);
	CHECK(h.IN_ep[0].maxpacket == 0);
	CHECK(h.OUT_ep[0].maxpacket == 0);
	CHECK(h.IN_ep[1].num == 0x5A);  // untouched
	CHECK(h.OUT_ep[1].num == 0x5A);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init with dev_endpoints 2 initialises endpoints 0 and 1 only") {
	poison();
	h.Init.dev_endpoints = 2;
	REQUIRE(init() == HAL_OK);
	CHECK(h.IN_ep[1].num == 1);
	CHECK(h.OUT_ep[1].num == 1);
	CHECK(h.IN_ep[2].num == 0x5A);
	CHECK(h.OUT_ep[2].num == 0x5A);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init with dev_endpoints 3 initialises endpoints 0 to 2 only") {
	poison();
	h.Init.dev_endpoints = 3;
	REQUIRE(init() == HAL_OK);
	CHECK(h.IN_ep[2].num == 2);
	CHECK(h.OUT_ep[2].num == 2);
	CHECK(h.IN_ep[3].num == 0x5A);
	CHECK(h.OUT_ep[3].num == 0x5A);
}

// NR345_3: UM1725 allows dev_endpoints 1..15 for the generic driver; the OTG_FS core has 4 endpoints (RM0090 34.2). The HAL does not reject a larger number and
// USB_DevInit then writes the control registers of endpoints that do not exist (reserved offsets).
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI1 HAL_PCD_Init with dev_endpoints 5 accepts it and touches the registers of a fifth endpoint") {
	h.Init.dev_endpoints = 5;
	CHECK(init() == HAL_OK);
	CHECK(hostOtgReservedAccesses() > 0u);
	clearFaults();
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init clears USB_Address") {
	h.USB_Address = 0x55;
	REQUIRE(init() == HAL_OK);
	CHECK(h.USB_Address == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init forces dma_enable to 0 for the OTG_FS instance, which has no DMA") {
	h.Init.dma_enable = ENABLE;
	REQUIRE(init() == HAL_OK);
	CHECK(h.Init.dma_enable == 0);
	CHECK((rd(oGAHBCFG) & (1u << 5)) == 0);  // RM0090 GAHBCFG bit 5 is reserved on this core (DMAEN only in OTG_HS)
	CHECK((rd(oGINTMSK) & M_RXFLVL) == M_RXFLVL);  // slave mode: the receive FIFO interrupt is used
}

// NR345_4: the OTG_FS core has no ULPI; RM0090 does not describe the ULPI selection of the HAL. With phy_itface ULPI the HAL clears PWRDWN and the ULPI select bits and
// then sets DCFG.DSPD to 0 (high speed) or 1 (high speed core in full speed mode); the registers of this core ignore the bits it does not implement. Recorded as is.
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI1 HAL_PCD_Init with the ULPI interface and high speed sets DSPD to 00 and clears PWRDWN") {
	h.Init.phy_itface = PCD_PHY_ULPI;
	h.Init.speed = PCD_SPEED_HIGH;
	REQUIRE(init() == HAL_OK);
	CHECK(dspd() == 0u);
	CHECK((rd(oGCCFG) & CC_PWRDWN) == 0);
}

// NEEDS REVIEW 416
TEST_CASE_FIXTURE(PiFx, "characterization: PI1 HAL_PCD_Init with the ULPI interface and full speed sets DSPD to 01") {
	h.Init.phy_itface = PCD_PHY_ULPI;
	h.Init.speed = PCD_SPEED_FULL;
	REQUIRE(init() == HAL_OK);
	CHECK(dspd() == 1u);
}

// NEEDS REVIEW 416
TEST_CASE_FIXTURE(PiFx, "characterization: PI1 HAL_PCD_Init with the ULPI interface and external VBUS drive still ends READY") {
	h.Init.phy_itface = PCD_PHY_ULPI;
	h.Init.use_external_vbus = 1;
	CHECK(init() == HAL_OK);
	CHECK(h.State == HAL_PCD_STATE_READY);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init ignores the speed field for the embedded transceiver and programs full speed") {
	h.Init.speed = PCD_SPEED_HIGH;  // not available with the embedded full speed PHY
	REQUIRE(init() == HAL_OK);
	CHECK(dspd() == 3u);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init with lpm_enable does not touch any register the OTG_FS core lacks for LPM") {
	h.Init.lpm_enable = ENABLE;  // RM0090 chapter 34 has no LPM support and no GLPMCFG register
	REQUIRE(init() == HAL_OK);
	CHECK((rd(oGINTMSK) & M_LPM) == 0);
	for (unsigned i = 0; i < 32; ++i) CHECK(hostOtgHsOnlyOffset(i) != 0x54u);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init with battery_charging_enable deactivates the transceiver until Start") {
	h.Init.battery_charging_enable = ENABLE;
	REQUIRE(init() == HAL_OK);
	CHECK((rd(oGCCFG) & CC_PWRDWN) == 0);  // RM0090 PWRDWN 0 = power down active
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init returns HAL_ERROR and state ERROR when the core soft reset never completes") {
	hostOtgSetPollReads(0);  // CSRST never clears: USB_CoreReset gives HAL_TIMEOUT, which Init maps to HAL_ERROR
	CHECK(HAL_PCD_Init(&h) == HAL_ERROR);
	CHECK(h.State == HAL_PCD_STATE_ERROR);
	CHECK(HAL_PCD_GetState(&h) == HAL_PCD_STATE_ERROR);
	CHECK(otgCounts.mspInit == 1);
	CHECK(hostOtgHsOnlyAccesses() == 0u);  // USB_DevInit was not reached
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init returns HAL_ERROR and state ERROR when the core does not enter device mode") {
	hostTickSetStep(piStepHostForever);  // CMOD stays 1 (host) during the 200 ms the HAL waits
	CHECK(HAL_PCD_Init(&h) == HAL_ERROR);
	CHECK(h.State == HAL_PCD_STATE_ERROR);
	CHECK(hostTickPeek() >= 200u);
	CHECK(hostOtgHsOnlyAccesses() == 0u);  // USB_DevInit was not reached
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_Init returns HAL_ERROR and state ERROR when a FIFO flush in USB_DevInit never completes") {
	hostTickSetStep(piStepFlushNeverCompletes);  // CSRST is done by then; the TxFIFO and RxFIFO flush handshakes hang
	CHECK(HAL_PCD_Init(&h) == HAL_ERROR);
	CHECK(h.State == HAL_PCD_STATE_ERROR);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_DeInit of a NULL handle returns HAL_ERROR") {
	CHECK(HAL_PCD_DeInit(nullptr) == HAL_ERROR);
	CHECK(otgCounts.mspDeInit == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_DeInit calls MspDeInit once, sets the state RESET and returns HAL_OK") {
	REQUIRE(init() == HAL_OK);
	CHECK(HAL_PCD_DeInit(&h) == HAL_OK);
	CHECK(otgCounts.mspDeInit == 1);
	CHECK(otgLastHandle == &h);
	CHECK(h.State == HAL_PCD_STATE_RESET);
	CHECK(HAL_PCD_GetState(&h) == HAL_PCD_STATE_RESET);
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_DeInit clears the pending endpoint flags, the interrupt masks and the FIFOs") {
	bringUp();
	wr(oDIEPMSK, 0x1);
	wr(oDOEPMSK, 0x1);
	wr(oDAINTMSK, 0x10001);
	wr(inInt(1), I_XFRC | I_EPDISD);
	wr(outInt(2), I_XFRC | I_STUP_OR_AHBERR);
	fifoWrite(1, 0x11223344);
	hostOtgRxFill(0xAB, 3);
	CHECK(HAL_PCD_DeInit(&h) == HAL_OK);
	CHECK(rd(oDIEPMSK) == 0u);
	CHECK(rd(oDOEPMSK) == 0u);
	CHECK(rd(oDAINTMSK) == 0u);
	CHECK((rd(inInt(1)) & (I_XFRC | I_EPDISD)) == 0);
	CHECK((rd(outInt(2)) & (I_XFRC | I_STUP_OR_AHBERR)) == 0);
	CHECK(hostOtgTxWordsUsed(1) == 0u);
	CHECK(hostOtgRxWordsUsed() == 0u);
	clearFaults();  // StopDevice also clears the flags of the endpoints 4 to 14 (reserved offsets): see NR345_9
}

// NR345_5: UM1725 does not say what HAL_PCD_DeInit does when the FIFO flush fails. The HAL returns HAL_ERROR and leaves the state BUSY, without calling MspDeInit.
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI1 HAL_PCD_DeInit returns HAL_ERROR and stays BUSY without MspDeInit when the FIFO flush never completes") {
	REQUIRE(init() == HAL_OK);
	hostOtgSetPollReads(0);
	CHECK(HAL_PCD_DeInit(&h) == HAL_ERROR);
	CHECK(h.State == HAL_PCD_STATE_BUSY);
	CHECK(otgCounts.mspDeInit == 0);
	clearFaults();
}

// NR345_6: RM0090 and UM1725 do not say whether DeInit must disconnect. The HAL does not set SDIS: after DeInit the pull-up state is the one before.
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI1 HAL_PCD_DeInit does not change the soft disconnect bit") {
	bringUp();
	REQUIRE(HAL_PCD_DevConnect(&h) == HAL_OK);
	REQUIRE((rd(oDCTL) & DCTL_SDIS) == 0);
	CHECK(HAL_PCD_DeInit(&h) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_SDIS) == 0);
	clearFaults();
}

TEST_CASE_FIXTURE(PiFx, "PI1 HAL_PCD_GetState returns the state field of the handle") {
	h.State = HAL_PCD_STATE_TIMEOUT;
	CHECK(HAL_PCD_GetState(&h) == HAL_PCD_STATE_TIMEOUT);
	h.State = HAL_PCD_STATE_BUSY;
	CHECK(HAL_PCD_GetState(&h) == HAL_PCD_STATE_BUSY);
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// PI2 HAL_PCD_Start / Stop / DevConnect / DevDisconnect / SetAddress / SetTestMode / RemoteWakeup / WKUP_IRQHandler
// ---------------------------------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_Start enables the global interrupt and connects the device") {
	REQUIRE(init() == HAL_OK);
	CHECK(HAL_PCD_Start(&h) == HAL_OK);
	CHECK((rd(oGAHBCFG) & GINT) == GINT);
	CHECK((rd(oDCTL) & DCTL_SDIS) == 0);  // RM0090 SDIS 0 = normal operation, the core connects
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_Start makes a pending unmasked interrupt reach the CPU") {
	REQUIRE(init() == HAL_OK);
	hostOtgVbus(1);
	REQUIRE(hostOtgIrqPending() == 0);
	REQUIRE(HAL_PCD_Start(&h) == HAL_OK);
	CHECK(hostOtgIrqPending() == 1);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_Start ungates the PHY clock (PCGCCTL STPPCLK and GATEHCLK cleared)") {
	REQUIRE(init() == HAL_OK);
	wr(oPCGCCTL, kMspMarkerPcgc);
	REQUIRE(HAL_PCD_Start(&h) == HAL_OK);
	CHECK(rd(oPCGCCTL) == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_Start of a locked handle returns HAL_BUSY and changes nothing") {
	REQUIRE(init() == HAL_OK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_PCD_Start(&h) == HAL_BUSY);
	CHECK((rd(oGAHBCFG) & GINT) == 0);
	CHECK((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
	CHECK(h.Lock == HAL_LOCKED);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_Start with battery_charging_enable activates the transceiver") {
	h.Init.battery_charging_enable = ENABLE;
	REQUIRE(init() == HAL_OK);
	REQUIRE((rd(oGCCFG) & CC_PWRDWN) == 0);
	CHECK(HAL_PCD_Start(&h) == HAL_OK);
	CHECK((rd(oGCCFG) & CC_PWRDWN) == CC_PWRDWN);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_Stop disables the global interrupt and disconnects the device") {
	bringUp();
	REQUIRE((rd(oGAHBCFG) & GINT) == GINT);
	CHECK(HAL_PCD_Stop(&h) == HAL_OK);
	CHECK((rd(oGAHBCFG) & GINT) == 0);
	CHECK((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
	CHECK(h.Lock == HAL_UNLOCKED);
	hostOtgVbus(1);
	CHECK(hostOtgIrqPending() == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_Stop flushes all four transmit FIFOs") {
	bringUp();
	for (unsigned ep = 0; ep < 4; ++ep) fifoWrite(ep, 0xA5A50000u + ep);
	for (unsigned ep = 0; ep < 4; ++ep) REQUIRE(hostOtgTxWordsUsed(ep) == 1u);
	CHECK(HAL_PCD_Stop(&h) == HAL_OK);
	for (unsigned ep = 0; ep < 4; ++ep) CHECK(hostOtgTxWordsUsed(ep) == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_Stop of a locked handle returns HAL_BUSY and changes nothing") {
	bringUp();
	h.Lock = HAL_LOCKED;
	fifoWrite(0, 1);
	CHECK(HAL_PCD_Stop(&h) == HAL_BUSY);
	CHECK((rd(oGAHBCFG) & GINT) == GINT);
	CHECK(hostOtgTxWordsUsed(0) == 1u);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_Stop with battery_charging_enable deactivates the transceiver") {
	h.Init.battery_charging_enable = ENABLE;
	bringUp();
	REQUIRE((rd(oGCCFG) & CC_PWRDWN) == CC_PWRDWN);
	CHECK(HAL_PCD_Stop(&h) == HAL_OK);
	CHECK((rd(oGCCFG) & CC_PWRDWN) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_Stop ignores a flush that never completes and returns HAL_OK") {
	bringUp();
	hostOtgSetPollReads(0);
	CHECK(HAL_PCD_Stop(&h) == HAL_OK);  // the HAL discards the result of USB_FlushTxFifo
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_DevConnect clears the soft disconnect bit and ungates the PHY clock") {
	REQUIRE(init() == HAL_OK);
	wr(oPCGCCTL, kMspMarkerPcgc);
	CHECK(HAL_PCD_DevConnect(&h) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_SDIS) == 0);
	CHECK(rd(oPCGCCTL) == 0u);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_DevConnect does not change the global interrupt enable") {
	REQUIRE(init() == HAL_OK);
	CHECK(HAL_PCD_DevConnect(&h) == HAL_OK);
	CHECK((rd(oGAHBCFG) & GINT) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_DevConnect of a locked handle returns HAL_BUSY and changes nothing") {
	REQUIRE(init() == HAL_OK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_PCD_DevConnect(&h) == HAL_BUSY);
	CHECK((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_DevConnect with battery_charging_enable activates the transceiver") {
	h.Init.battery_charging_enable = ENABLE;
	REQUIRE(init() == HAL_OK);
	CHECK(HAL_PCD_DevConnect(&h) == HAL_OK);
	CHECK((rd(oGCCFG) & CC_PWRDWN) == CC_PWRDWN);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_DevDisconnect sets the soft disconnect bit and ungates the PHY clock") {
	bringUp();
	wr(oPCGCCTL, kMspMarkerPcgc);
	CHECK(HAL_PCD_DevDisconnect(&h) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
	CHECK(rd(oPCGCCTL) == 0u);
	CHECK(h.Lock == HAL_UNLOCKED);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_DevDisconnect keeps the global interrupt enable") {
	bringUp();
	CHECK(HAL_PCD_DevDisconnect(&h) == HAL_OK);
	CHECK((rd(oGAHBCFG) & GINT) == GINT);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_DevDisconnect of a locked handle returns HAL_BUSY and changes nothing") {
	bringUp();
	REQUIRE(HAL_PCD_DevConnect(&h) == HAL_OK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_PCD_DevDisconnect(&h) == HAL_BUSY);
	CHECK((rd(oDCTL) & DCTL_SDIS) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_DevDisconnect with battery_charging_enable deactivates the transceiver") {
	h.Init.battery_charging_enable = ENABLE;
	bringUp();
	REQUIRE(HAL_PCD_DevConnect(&h) == HAL_OK);
	REQUIRE((rd(oGCCFG) & CC_PWRDWN) == CC_PWRDWN);
	CHECK(HAL_PCD_DevDisconnect(&h) == HAL_OK);
	CHECK((rd(oGCCFG) & CC_PWRDWN) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI2 connecting after a disconnect clears the soft disconnect bit again") {
	bringUp();
	REQUIRE(HAL_PCD_DevDisconnect(&h) == HAL_OK);
	REQUIRE((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
	CHECK(HAL_PCD_DevConnect(&h) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_SDIS) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_SetAddress programs DCFG.DAD and stores the address, for 0, 1, 42 and 127") {
	REQUIRE(init() == HAL_OK);
	const uint8_t addr[] = {0, 1, 42, 127};
	for (uint8_t a : addr) {
		CAPTURE((int)a);
		CHECK(HAL_PCD_SetAddress(&h, a) == HAL_OK);
		CHECK(dad() == a);  // RM0090 DCFG bits 10:4 DAD
		CHECK(h.USB_Address == a);
		CHECK(h.Lock == HAL_UNLOCKED);
	}
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_SetAddress keeps the device speed bits and replaces an earlier address") {
	REQUIRE(init() == HAL_OK);
	REQUIRE(HAL_PCD_SetAddress(&h, 0x7F) == HAL_OK);
	REQUIRE(HAL_PCD_SetAddress(&h, 0x01) == HAL_OK);
	CHECK(dad() == 1u);
	CHECK(dspd() == 3u);
}

// NR345_7: the USB address is 7 bits (RM0090 DAD bits 10:4); UM1725 says nothing about values above 127. The HAL stores the 8 bit value in USB_Address and masks DAD.
// NEEDS REVIEW 416
TEST_CASE_FIXTURE(PiFx, "characterization: PI2 HAL_PCD_SetAddress of 0xFF keeps USB_Address 0xFF and programs DAD 0x7F") {
	REQUIRE(init() == HAL_OK);
	CHECK(HAL_PCD_SetAddress(&h, 0xFF) == HAL_OK);
	CHECK(h.USB_Address == 0xFF);
	CHECK(dad() == 0x7Fu);
}

// NEEDS REVIEW 416
TEST_CASE_FIXTURE(PiFx, "characterization: PI2 HAL_PCD_SetAddress of 0x80 programs DAD 0") {
	REQUIRE(init() == HAL_OK);
	CHECK(HAL_PCD_SetAddress(&h, 0x80) == HAL_OK);
	CHECK(h.USB_Address == 0x80);
	CHECK(dad() == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_SetAddress of a locked handle returns HAL_BUSY and changes nothing") {
	REQUIRE(init() == HAL_OK);
	h.Lock = HAL_LOCKED;
	CHECK(HAL_PCD_SetAddress(&h, 9) == HAL_BUSY);
	CHECK(dad() == 0u);
	CHECK(h.USB_Address == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_SetTestMode programs TCTL for each of the five test modes") {
	REQUIRE(init() == HAL_OK);
	const uint8_t modes[] = {TEST_J, TEST_K, TEST_SE0_NAK, TEST_PACKET, TEST_FORCE_EN};
	for (uint8_t m : modes) {
		CAPTURE((int)m);
		wr(oDCTL, rd(oDCTL) & ~(7u << 4));
		CHECK(HAL_PCD_SetTestMode(&h, m) == HAL_OK);
		CHECK(tctl() == m);  // RM0090 DCTL TCTL 001 J, 010 K, 011 SE0_NAK, 100 Packet, 101 Force_Enable
	}
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_SetTestMode ignores the reserved values 0, 6, 7 and 255 and returns HAL_OK") {
	REQUIRE(init() == HAL_OK);
	const uint8_t modes[] = {0, 6, 7, 255};
	for (uint8_t m : modes) {
		CAPTURE((int)m);
		CHECK(HAL_PCD_SetTestMode(&h, m) == HAL_OK);
		CHECK(tctl() == 0u);
	}
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_SetTestMode keeps the soft disconnect bit") {
	REQUIRE(init() == HAL_OK);
	REQUIRE((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
	CHECK(HAL_PCD_SetTestMode(&h, TEST_K) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
}

// NR345_21: RM0090 DCTL TCTL "010: Test_K mode" selects a mode; HAL_PCD_SetTestMode ORs the mode into TCTL, so Test_J (001) then Test_K (010) gives 011 =
// Test_SE0_NAK. USB 2.0 7.1.20 (test modes): a device leaves a test mode only by power cycle, so changing mode without a reset is not a defined use. Recorded as is.
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI2 HAL_PCD_SetTestMode of Test_K after Test_J ORs the codes and gives TCTL 011") {
	REQUIRE(init() == HAL_OK);
	REQUIRE(HAL_PCD_SetTestMode(&h, TEST_J) == HAL_OK);
	REQUIRE(tctl() == 1u);
	REQUIRE(HAL_PCD_SetTestMode(&h, TEST_K) == HAL_OK);
	CHECK(tctl() == 3u);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_ActivateRemoteWakeup sets RWUSIG while the bus is suspended") {
	REQUIRE(init() == HAL_OK);
	hostOtgSuspend();
	REQUIRE((rd(oDSTS) & DSTS_SUSPSTS) == DSTS_SUSPSTS);
	CHECK(HAL_PCD_ActivateRemoteWakeup(&h) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_RWUSIG) == DCTL_RWUSIG);
}

// NR345_8: RM0090 DCTL RWUSIG: "the application must set this bit to instruct the core to exit the Suspend state". The HAL sets RWUSIG only when DSTS.SUSPSTS is 1
// (USB 2.0 allows remote wakeup only from suspend) and returns HAL_OK in both cases.
// NEEDS REVIEW 338
TEST_CASE_FIXTURE(PiFx, "characterization: PI2 HAL_PCD_ActivateRemoteWakeup without suspend leaves RWUSIG clear and returns HAL_OK") {
	REQUIRE(init() == HAL_OK);
	REQUIRE((rd(oDSTS) & DSTS_SUSPSTS) == 0);
	CHECK(HAL_PCD_ActivateRemoteWakeup(&h) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_RWUSIG) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_DeActivateRemoteWakeup clears RWUSIG and keeps the other DCTL bits") {
	REQUIRE(init() == HAL_OK);
	hostOtgSuspend();
	REQUIRE(HAL_PCD_ActivateRemoteWakeup(&h) == HAL_OK);
	REQUIRE((rd(oDCTL) & DCTL_RWUSIG) == DCTL_RWUSIG);
	CHECK(HAL_PCD_DeActivateRemoteWakeup(&h) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_RWUSIG) == 0);
	CHECK((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_DeActivateRemoteWakeup with RWUSIG already clear returns HAL_OK") {
	REQUIRE(init() == HAL_OK);
	CHECK(HAL_PCD_DeActivateRemoteWakeup(&h) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_RWUSIG) == 0);
}

// RM0090 EXTI: line 18 = USB OTG FS wakeup event, line 20 = USB OTG HS wakeup event; EXTI_PR bits are cleared by writing 1 (rc_w1).
TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_WKUP_IRQHandler of the FS instance clears the EXTI pending bit of line 18 only") {
	hostRegsPoke(oEXTI_PR, (1u << 18) | (1u << 17) | (1u << 20));
	HAL_PCD_WKUP_IRQHandler(&h);
	CHECK((hostRegsPeek(oEXTI_PR) & (1u << 18)) == 0);
	CHECK((hostRegsPeek(oEXTI_PR) & (1u << 17)) == (1u << 17));
	CHECK((hostRegsPeek(oEXTI_PR) & (1u << 20)) == (1u << 20));
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_WKUP_IRQHandler of the HS instance clears the EXTI pending bit of line 20 only") {
	h.Instance = USB_OTG_HS;
	hostRegsPoke(oEXTI_PR, (1u << 18) | (1u << 17) | (1u << 20));
	HAL_PCD_WKUP_IRQHandler(&h);
	CHECK((hostRegsPeek(oEXTI_PR) & (1u << 20)) == 0);
	CHECK((hostRegsPeek(oEXTI_PR) & (1u << 18)) == (1u << 18));
	CHECK((hostRegsPeek(oEXTI_PR) & (1u << 17)) == (1u << 17));
}

TEST_CASE_FIXTURE(PiFx, "PI2 HAL_PCD_WKUP_IRQHandler does not touch the OTG_FS registers") {
	REQUIRE(init() == HAL_OK);
	const uint32_t before = rd(oGINTMSK);
	hostRegsPoke(oEXTI_PR, 1u << 18);
	HAL_PCD_WKUP_IRQHandler(&h);
	CHECK(rd(oGINTMSK) == before);
	CHECK(otgCounts.resume == 0);
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// PI3 HAL_PCDEx_SetRxFiFo / SetTxFiFo
// ---------------------------------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(PiFx, "PI3 HAL_PCDEx_SetRxFiFo writes the receive FIFO depth to GRXFSIZ and returns HAL_OK") {
	REQUIRE(init() == HAL_OK);
	const uint16_t sizes[] = {0x10, 0x50, 0x80, 0x140};
	for (uint16_t s : sizes) {
		CAPTURE(s);
		CHECK(HAL_PCDEx_SetRxFiFo(&h, s) == HAL_OK);
		CHECK(rd(oGRXFSIZ) == s);  // RM0090 GRXFSIZ bits 15:0 RXFD
	}
}

// RM0090 34.13.1 / 34.12 FIFO RAM allocation: each FIFO starts where the previous one ends. Hand derivation (RX 0x80, all TX 0x40):
// TX0 start 0x80, TX1 start 0x80 + 0x40 = 0xC0, TX2 start 0x100, TX3 start 0x140; each register is (depth << 16) | start.
TEST_CASE_FIXTURE(PiFx, "PI3 HAL_PCDEx_SetTxFiFo places TxFIFO 0 behind the receive FIFO") {
	REQUIRE(init() == HAL_OK);
	REQUIRE(HAL_PCDEx_SetRxFiFo(&h, 0x80) == HAL_OK);
	CHECK(HAL_PCDEx_SetTxFiFo(&h, 0, 0x40) == HAL_OK);
	CHECK(rd(oDIEPTXF0) == 0x00400080u);
}

// Hand derivation inside the 320 word RAM (RX 0x40, TX0 0x20, TX1-3 0x40 each = 288 words): TX0 start 0x40, TX1 start 0x40 + 0x20 = 0x60, TX2 start 0xA0, TX3 start 0xE0.
TEST_CASE_FIXTURE(PiFx, "PI3 HAL_PCDEx_SetTxFiFo places the TxFIFOs 1 to 3 one behind the other") {
	REQUIRE(init() == HAL_OK);
	REQUIRE(HAL_PCDEx_SetRxFiFo(&h, 0x40) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 0, 0x20) == HAL_OK);
	CHECK(HAL_PCDEx_SetTxFiFo(&h, 1, 0x40) == HAL_OK);
	CHECK(HAL_PCDEx_SetTxFiFo(&h, 2, 0x40) == HAL_OK);
	CHECK(HAL_PCDEx_SetTxFiFo(&h, 3, 0x40) == HAL_OK);
	CHECK(rd(oDIEPTXF0) == 0x00200040u);
	CHECK(rd(oDIEPTXF1) == 0x00400060u);
	CHECK(rd(oDIEPTXF1 + 4) == 0x004000A0u);
	CHECK(rd(oDIEPTXF1 + 8) == 0x004000E0u);
}

// E02: the start address of a FIFO is the end of the previous FIFO (RM0090 34.13.1), so it does not depend on the FIFO's own depth: setting FIFO 1 and FIFO 3 twice
// with different depths gives the same start address both times.
TEST_CASE_FIXTURE(PiFx, "PI3 HAL_PCDEx_SetTxFiFo of the same FIFO twice with different depths keeps its start address") {
	REQUIRE(init() == HAL_OK);
	REQUIRE(HAL_PCDEx_SetRxFiFo(&h, 0x40) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 0, 0x20) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 1, 0x20) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 2, 0x20) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 3, 0x20) == HAL_OK);
	CHECK(rd(oDIEPTXF1) == 0x00200060u);
	CHECK(HAL_PCDEx_SetTxFiFo(&h, 1, 0x40) == HAL_OK);
	CHECK(rd(oDIEPTXF1) == 0x00400060u);
	CHECK(HAL_PCDEx_SetTxFiFo(&h, 3, 0x30) == HAL_OK);
	CHECK((rd(oDIEPTXF1 + 8) & 0xFFFFu) == (0x40u + 0x20u + 0x40u + 0x20u));  // start: RX + TX0 + TX1 (now 0x40) + TX2
	CHECK((rd(oDIEPTXF1 + 8) >> 16) == 0x30u);
}

// Hand derivation: RX 0x50, TX0 0x10, TX1 0x20, TX2 0x30, TX3 0x40:
// TX0 start 0x50; TX1 start 0x50 + 0x10 = 0x60; TX2 start 0x60 + 0x20 = 0x80; TX3 start 0x80 + 0x30 = 0xB0. Total 0xF0 words (RAM is 320 = 0x140 words).
TEST_CASE_FIXTURE(PiFx, "PI3 HAL_PCDEx_SetTxFiFo with different depths computes every start address from the depths before") {
	REQUIRE(init() == HAL_OK);
	REQUIRE(HAL_PCDEx_SetRxFiFo(&h, 0x50) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 0, 0x10) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 1, 0x20) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 2, 0x30) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 3, 0x40) == HAL_OK);
	CHECK(rd(oDIEPTXF0) == 0x00100050u);
	CHECK(rd(oDIEPTXF1) == 0x00200060u);
	CHECK(rd(oDIEPTXF1 + 4) == 0x00300080u);
	CHECK(rd(oDIEPTXF1 + 8) == 0x004000B0u);
}

TEST_CASE_FIXTURE(PiFx, "PI3 HAL_PCDEx_SetTxFiFo of TxFIFO 0 does not change the other FIFO registers") {
	REQUIRE(init() == HAL_OK);
	REQUIRE(HAL_PCDEx_SetRxFiFo(&h, 0x80) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 1, 0x40) == HAL_OK);
	const uint32_t t1 = rd(oDIEPTXF1);
	CHECK(HAL_PCDEx_SetTxFiFo(&h, 0, 0x20) == HAL_OK);
	CHECK(rd(oDIEPTXF1) == t1);
	CHECK(rd(oGRXFSIZ) == 0x80u);
}

// NR345_9: the start addresses are computed from the registers at the time of the call. If TxFIFO 2 is set before TxFIFO 1, the depth of TxFIFO 1 is still 0 and the two overlap.
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI3 HAL_PCDEx_SetTxFiFo computes the start of TxFIFO 2 from the current depth of TxFIFO 1, 0 if it is not yet set") {
	REQUIRE(init() == HAL_OK);
	REQUIRE(HAL_PCDEx_SetRxFiFo(&h, 0x80) == HAL_OK);
	REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 0, 0x40) == HAL_OK);
	CHECK(HAL_PCDEx_SetTxFiFo(&h, 2, 0x40) == HAL_OK);
	CHECK(rd(oDIEPTXF1 + 4) == 0x004000C0u);  // 0x80 + 0x40 + 0 (TxFIFO 1 has depth 0 after USB_DevInit)
}

// NR345_10: RM0090 DIEPTXFx INEPTXFD minimum 16 words and the total RAM of 320 words (34.12): the HAL checks neither (its comment says "TXn min size = 16 words").
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI3 HAL_PCDEx_SetTxFiFo and SetRxFiFo accept depths below the 16 word minimum and above the 320 word RAM") {
	REQUIRE(init() == HAL_OK);
	CHECK(HAL_PCDEx_SetTxFiFo(&h, 0, 4) == HAL_OK);
	CHECK(HAL_PCDEx_SetRxFiFo(&h, 0x400) == HAL_OK);
	CHECK(rd(oGRXFSIZ) == 0x400u);
	CHECK((rd(oDIEPTXF0) >> 16) == 4u);
}

// NR345_11: the OTG_FS core has TxFIFOs 0-3 (DIEPTXF0 + three DIEPTXFx). Fifo number 4 addresses DIEPTXF[3] at 0x110, which only the OTG_HS core has.
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI3 HAL_PCDEx_SetTxFiFo of FIFO number 4 writes a register the OTG_FS core does not have") {
	REQUIRE(init() == HAL_OK);
	const uint32_t before = hostOtgHsOnlyAccesses();
	CHECK(HAL_PCDEx_SetTxFiFo(&h, 4, 0x20) == HAL_OK);
	CHECK(hostOtgHsOnlyAccesses() > before);
	CHECK(hostOtgHsOnlyWasWrite(before) == 1);
	CHECK(hostOtgHsOnlyOffset(before) == 0x110u);
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// PI4 LL core
// ---------------------------------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(PiFx, "PI4 USB_CoreInit with the embedded PHY resets the core and clears the interrupt masks and the other CSR bits") {
	wr(oGINTMSK, 0xFFFFu);
	wr(oGRXFSIZ, 0x123);
	CHECK(USB_CoreInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(rd(oGINTMSK) == 0u);       // RM0090 GRSTCTL CSRST: clears the interrupts and all the CSR register bits
	CHECK(rd(oGRXFSIZ) == 0x200u);   // GRXFSIZ reset value (RM0090 34.16.2)
	CHECK((rd(oGUSBCFG) & G_PHYSEL) == G_PHYSEL);
	CHECK((rd(oGRSTCTL) & 1u) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_CoreInit keeps DCFG.DSPD through the core soft reset and clears the device address") {
	wr(oDCFG, rd(oDCFG) | 3u | (0x2Au << 4));
	CHECK(USB_CoreInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(dspd() == 3u);  // RM0090 CSRST: all CSR bits are cleared except DSPD in DCFG
	CHECK(dad() == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_CoreInit waits for a core soft reset that completes only at the fifth poll") {
	hostOtgSetPollReads(5);
	CHECK(USB_CoreInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(oGRSTCTL) & 1u) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_CoreInit returns HAL_TIMEOUT when the soft reset bit never clears") {
	hostOtgSetPollReads(0);
	CHECK(USB_CoreInit(USB_OTG_FS, h.Init) == HAL_TIMEOUT);
	CHECK((rd(oGRSTCTL) & 1u) == 1u);  // CSRST is still set
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_CoreInit returns HAL_TIMEOUT when the AHB master never reports idle") {
	wr(oGRSTCTL, 0);  // AHBIDL (bit 31) reads 0
	CHECK(USB_CoreInit(USB_OTG_FS, h.Init) == HAL_TIMEOUT);
	CHECK((rd(oGRSTCTL) & 1u) == 0);  // the soft reset was never started
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_CoreInit with battery_charging_enable 0 activates the transceiver (PWRDWN set)") {
	h.Init.battery_charging_enable = 0;
	CHECK(USB_CoreInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(oGCCFG) & CC_PWRDWN) == CC_PWRDWN);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_CoreInit with battery_charging_enable 1 leaves the transceiver in power down") {
	h.Init.battery_charging_enable = 1;
	CHECK(USB_CoreInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(oGCCFG) & CC_PWRDWN) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_CoreInit with the ULPI interface resets the core and clears PWRDWN") {
	h.Init.phy_itface = USB_OTG_ULPI_PHY;
	wr(oGCCFG, CC_PWRDWN);
	CHECK(USB_CoreInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(oGCCFG) & CC_PWRDWN) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_CoreInit with the ULPI interface returns HAL_TIMEOUT when the soft reset never clears") {
	h.Init.phy_itface = USB_OTG_ULPI_PHY;
	hostOtgSetPollReads(0);
	CHECK(USB_CoreInit(USB_OTG_FS, h.Init) == HAL_TIMEOUT);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetCurrentMode device sets FDMOD, clears FHMOD, waits at least 10 ms and returns HAL_OK") {
	wr(oGUSBCFG, rd(oGUSBCFG) | G_FHMOD);
	CHECK(USB_SetCurrentMode(USB_OTG_FS, USB_DEVICE_MODE) == HAL_OK);
	CHECK((rd(oGUSBCFG) & G_FDMOD) == G_FDMOD);
	CHECK((rd(oGUSBCFG) & G_FHMOD) == 0);
	CHECK(hostTickPeek() >= 10u);  // HAL_Delay(10)
	CHECK(hostTickPeek() < 200u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetCurrentMode host sets FHMOD, clears FDMOD and returns HAL_OK once the core reports host mode") {
	hostTickSetStep(piStepHostAt);
	piHostCmodAt = 20;
	wr(oGUSBCFG, rd(oGUSBCFG) | G_FDMOD);
	CHECK(USB_SetCurrentMode(USB_OTG_FS, USB_HOST_MODE) == HAL_OK);
	CHECK((rd(oGUSBCFG) & G_FHMOD) == G_FHMOD);
	CHECK((rd(oGUSBCFG) & G_FDMOD) == 0);
	CHECK(USB_GetMode(USB_OTG_FS) == (uint32_t)USB_HOST_MODE);
}

// NR345_12: the model does not derive GINTSTS.CMOD from FHMOD / FDMOD (lead decision needed); this case documents the timeout of the HAL when the core never reports the mode.
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetCurrentMode returns HAL_ERROR after 200 ms when the core does not report the requested mode") {
	CHECK(USB_SetCurrentMode(USB_OTG_FS, USB_HOST_MODE) == HAL_ERROR);  // CMOD stays 0 (device)
	CHECK(hostTickPeek() >= 200u);
	CHECK((rd(oGUSBCFG) & G_FHMOD) == G_FHMOD);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetCurrentMode device returns HAL_ERROR after 200 ms when the core stays in host mode") {
	hostTickSetStep(piStepHostForever);
	CHECK(USB_SetCurrentMode(USB_OTG_FS, USB_DEVICE_MODE) == HAL_ERROR);
	CHECK(hostTickPeek() >= 200u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetCurrentMode device returns HAL_OK when the mode is reached after 100 ms") {
	hostTickSetStep(piStepDeviceAtThreshold);
	piCmodThreshold = 100;
	wr(oGINTSTS, rd(oGINTSTS) | 1u);
	CHECK(USB_SetCurrentMode(USB_OTG_FS, USB_DEVICE_MODE) == HAL_OK);
	CHECK(hostTickPeek() >= 100u);
	CHECK(hostTickPeek() < 200u);
}

// V119 (arguable): when the mode is reached at the very last poll (ms == 200) the loop ends because the mode matches, but the check `ms == HAL_USB_CURRENT_MODE_MAX_DELAY_MS`
// (stm32f4xx_ll_usb.c, end of USB_SetCurrentMode) then reports HAL_ERROR although the core is in the requested mode. Each HAL_Delay(10) costs 12 ticks here
// (HAL_Delay adds uwTickFreq = 1 to the wait, and one tick per HAL_GetTick call): after 19 delays the tick is 228, after 20 delays 240; CMOD changes at tick 235.
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetCurrentMode returns HAL_OK when the requested mode is reached at the last poll" *
                                 KNOWN_ISSUE("V119 (arguable: RM0090 gives no timing): USB_SetCurrentMode (stm32f4xx_ll_usb.c, final if (ms == HAL_USB_CURRENT_MODE_MAX_DELAY_MS) return HAL_ERROR) reports HAL_ERROR although the core reached the requested mode at the last poll")) {
	hostTickSetStep(piStepDeviceAtThreshold);
	piCmodThreshold = 235;
	wr(oGINTSTS, rd(oGINTSTS) | 1u);
	REQUIRE(hostTickPeek() == 0u);
	const HAL_StatusTypeDef r = USB_SetCurrentMode(USB_OTG_FS, USB_DEVICE_MODE);
	REQUIRE(USB_GetMode(USB_OTG_FS) == (uint32_t)USB_DEVICE_MODE);  // the core is in device mode
	CHECK(r == HAL_OK);
}

// NR345_13: an invalid mode value is not described by RM0090; the HAL clears FHMOD and FDMOD, returns HAL_ERROR and does not wait.
// NEEDS REVIEW 349
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_SetCurrentMode with an invalid mode clears FDMOD and FHMOD and returns HAL_ERROR without waiting") {
	wr(oGUSBCFG, rd(oGUSBCFG) | G_FDMOD);
	CHECK(USB_SetCurrentMode(USB_OTG_FS, (USB_OTG_ModeTypeDef)2) == HAL_ERROR);
	CHECK((rd(oGUSBCFG) & (G_FDMOD | G_FHMOD)) == 0);
	CHECK(hostTickPeek() == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_GetMode returns bit 0 of GINTSTS: 0 in device mode, 1 in host mode, and nothing else") {
	wr(oGINTSTS, 0xFFFFFFFEu);
	CHECK(USB_GetMode(USB_OTG_FS) == 0u);
	wr(oGINTSTS, 0x00000001u);
	CHECK(USB_GetMode(USB_OTG_FS) == 1u);
	wr(oGINTSTS, 0xFFFFFFFFu);
	CHECK(USB_GetMode(USB_OTG_FS) == 1u);
	wr(oGINTSTS, 0u);
	CHECK(USB_GetMode(USB_OTG_FS) == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetDevSpeed full speed programs DSPD 11 and keeps the device address") {
	wr(oDCFG, rd(oDCFG) | (0x15u << 4));
	CHECK(USB_SetDevSpeed(USB_OTG_FS, USB_OTG_SPEED_FULL) == HAL_OK);
	CHECK(dspd() == 3u);
	CHECK(dad() == 0x15u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetDevSpeed with the high speed codes 0 and 1 programs DSPD 00 and 01 from a cleared field") {
	wr(oDCFG, rd(oDCFG) & ~3u);
	CHECK(USB_SetDevSpeed(USB_OTG_FS, USB_OTG_SPEED_HIGH) == HAL_OK);
	CHECK(dspd() == 0u);
	CHECK(USB_SetDevSpeed(USB_OTG_FS, USB_OTG_SPEED_HIGH_IN_FULL) == HAL_OK);
	CHECK(dspd() == 1u);
}

// NR345_14: RM0090 DCFG DSPD describes the speed the application requires; the HAL ORs the new code into the field, so a later lower code cannot replace a set bit.
// NEEDS REVIEW 416
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_SetDevSpeed ORs the speed code into DSPD, so full speed then code 01 stays 11") {
	REQUIRE(USB_SetDevSpeed(USB_OTG_FS, USB_OTG_SPEED_FULL) == HAL_OK);
	CHECK(USB_SetDevSpeed(USB_OTG_FS, USB_OTG_SPEED_HIGH_IN_FULL) == HAL_OK);
	CHECK(dspd() == 3u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_GetDevSpeed decodes the full speed enumeration result 11 as USBD_FS_SPEED") {
	wr(oDSTS, 3u << 1);  // RM0090 DSTS ENUMSPD 11 = full speed (PHY clock 48 MHz)
	CHECK(USB_GetDevSpeed(USB_OTG_FS) == USBD_FS_SPEED);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_GetDevSpeed returns USBD_FS_SPEED after the host enumerated the device") {
	REQUIRE(init() == HAL_OK);
	hostOtgEnumDone();
	CHECK(USB_GetDevSpeed(USB_OTG_FS) == USBD_FS_SPEED);
}

// NR345_15: RM0090 DSTS ENUMSPD lists 01 and 10 as reserved and does not describe 00 for the OTG_FS core. The HAL decodes 00 as high speed, 01 as full speed and 10 as 0xF.
// NEEDS REVIEW 416
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_GetDevSpeed decodes ENUMSPD 00 as USBD_HS_SPEED, 01 as USBD_FS_SPEED and 10 as 0xF") {
	wr(oDSTS, 0u << 1);
	CHECK(USB_GetDevSpeed(USB_OTG_FS) == USBD_HS_SPEED);
	wr(oDSTS, 1u << 1);
	CHECK(USB_GetDevSpeed(USB_OTG_FS) == USBD_FS_SPEED);
	wr(oDSTS, 2u << 1);
	CHECK(USB_GetDevSpeed(USB_OTG_FS) == 0xFu);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_GetDevSpeed ignores the other DSTS bits") {
	wr(oDSTS, 0xFFFFFFF9u | (3u << 1));  // SUSPSTS, EERR, FNSOF set around ENUMSPD = 11
	CHECK(USB_GetDevSpeed(USB_OTG_FS) == USBD_FS_SPEED);
	wr(oDSTS, 0xFFFFFFF9u);  // ENUMSPD = 00
	CHECK(USB_GetDevSpeed(USB_OTG_FS) == USBD_HS_SPEED);
}

namespace {
// Table 203 (RM0090 OTG_FS_GUSBCFG): TRDT minimum value by AHB frequency range, full speed.
uint32_t trdtFor(uint32_t hclk) {
	REQUIRE(USB_SetTurnaroundTime(USB_OTG_FS, hclk, USBD_FS_SPEED) == HAL_OK);
	return (rd(oGUSBCFG) >> 10) & 0xFu;
}
}  // namespace

TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime programs TRDT 0xF at the lower edge of the 14.2 to 15 MHz band") {
	CHECK(trdtFor(14200000) == 0xFu);
	CHECK(trdtFor(14999999) == 0xFu);
}
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime programs TRDT 0xE for 15 to 16 MHz") {
	CHECK(trdtFor(15000000) == 0xEu);
	CHECK(trdtFor(15999999) == 0xEu);
}
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime programs TRDT 0xD for 16 to 17.2 MHz") {
	CHECK(trdtFor(16000000) == 0xDu);
	CHECK(trdtFor(17199999) == 0xDu);
}
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime programs TRDT 0xC for 17.2 to 18.5 MHz") {
	CHECK(trdtFor(17200000) == 0xCu);
	CHECK(trdtFor(18499999) == 0xCu);
}
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime programs TRDT 0xB for 18.5 to 20 MHz") {
	CHECK(trdtFor(18500000) == 0xBu);
	CHECK(trdtFor(19999999) == 0xBu);
}
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime programs TRDT 0xA for 20 to 21.8 MHz") {
	CHECK(trdtFor(20000000) == 0xAu);
	CHECK(trdtFor(21799999) == 0xAu);
}
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime programs TRDT 0x9 for 21.8 to 24 MHz") {
	CHECK(trdtFor(21800000) == 0x9u);
	CHECK(trdtFor(23999999) == 0x9u);
}
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime programs TRDT 0x8 for 24 to 27.5 MHz") {
	CHECK(trdtFor(24000000) == 0x8u);
	CHECK(trdtFor(27499999) == 0x8u);
}
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime programs TRDT 0x7 for 27.7 to 32 MHz") {
	CHECK(trdtFor(27700000) == 0x7u);
	CHECK(trdtFor(31999999) == 0x7u);
}
TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime programs TRDT 0x6 from 32 MHz up, including the 168 MHz of the firmware") {
	CHECK(trdtFor(32000000) == 0x6u);
	CHECK(trdtFor(168000000) == 0x6u);
	CHECK(trdtFor(200000000) == 0x6u);
}

// NR345_16: Table 203 lists 24-27.5 MHz as 0x8 and 27.5-32 MHz as 0x7 (the values are minimums). The HAL switches at 27.7 MHz, so 27.6 MHz gets 0x8, which is still
// not below the table's minimum 0x7.
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_SetTurnaroundTime programs TRDT 0x8 between 27.5 and 27.7 MHz") {
	CHECK(trdtFor(27500000) == 0x8u);
	CHECK(trdtFor(27600000) == 0x8u);
}

// NR345_17: below 14.2 MHz Table 203 has no entry (the OTG_FS needs at least 14.2 MHz AHB). The HAL's last branch gives the smallest value 0x6 to any frequency below
// 14.2 MHz, although a lower AHB frequency needs a longer turnaround time (Table 203 trend: 0xF at 14.2 MHz).
// NEEDS REVIEW 349
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_SetTurnaroundTime programs TRDT 0x6 below 14.2 MHz") {
	CHECK(trdtFor(14199999) == 0x6u);
	CHECK(trdtFor(8000000) == 0x6u);
	CHECK(trdtFor(0) == 0x6u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetTurnaroundTime replaces a previous TRDT and keeps the other GUSBCFG bits") {
	wr(oGUSBCFG, rd(oGUSBCFG) | G_FDMOD | 0x7u);  // FDMOD, TOCAL = 7
	const uint32_t others = rd(oGUSBCFG) & ~(0xFu << 10);
	CHECK(trdtFor(14200000) == 0xFu);
	CHECK(trdtFor(168000000) == 0x6u);
	CHECK((rd(oGUSBCFG) & ~(0xFu << 10)) == others);
}

// NR345_18: RM0090 Table 203 covers full speed only; the HAL uses 9 for USBD_HS_SPEED (USBD_HS_TRDT_VALUE) and 9 for any other speed value (USBD_DEFAULT_TRDT_VALUE).
// NEEDS REVIEW 416
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_SetTurnaroundTime programs TRDT 9 for the high speed code and for an unknown speed code") {
	CHECK(USB_SetTurnaroundTime(USB_OTG_FS, 168000000, USBD_HS_SPEED) == HAL_OK);
	CHECK(((rd(oGUSBCFG) >> 10) & 0xFu) == 9u);
	wr(oGUSBCFG, rd(oGUSBCFG) & ~(0xFu << 10));
	CHECK(USB_SetTurnaroundTime(USB_OTG_FS, 168000000, 0x77) == HAL_OK);
	CHECK(((rd(oGUSBCFG) >> 10) & 0xFu) == 9u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_EnableGlobalInt sets GAHBCFG.GINT and keeps the other bits") {
	wr(oGAHBCFG, 0x180u);
	CHECK(USB_EnableGlobalInt(USB_OTG_FS) == HAL_OK);
	CHECK(rd(oGAHBCFG) == 0x181u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DisableGlobalInt clears GAHBCFG.GINT and keeps the other bits") {
	wr(oGAHBCFG, 0x181u);
	CHECK(USB_DisableGlobalInt(USB_OTG_FS) == HAL_OK);
	CHECK(rd(oGAHBCFG) == 0x180u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_EnableGlobalInt and USB_DisableGlobalInt are idempotent") {
	CHECK(USB_EnableGlobalInt(USB_OTG_FS) == HAL_OK);
	CHECK(USB_EnableGlobalInt(USB_OTG_FS) == HAL_OK);
	CHECK((rd(oGAHBCFG) & GINT) == GINT);
	CHECK(USB_DisableGlobalInt(USB_OTG_FS) == HAL_OK);
	CHECK(USB_DisableGlobalInt(USB_OTG_FS) == HAL_OK);
	CHECK((rd(oGAHBCFG) & GINT) == 0);
}

// --- USB_DevInit, called directly with prepared register state -----------------------------------------------------------------------------

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit zeroes the three IN FIFO size registers and returns HAL_OK") {
	for (unsigned i = 0; i < 3; ++i) wr(oDIEPTXF1 + 4u * i, 0x00400100u);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	for (unsigned i = 0; i < 3; ++i) CHECK(rd(oDIEPTXF1 + 4u * i) == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit with VBUS sensing off sets SDIS and NOVBUSSENS and clears VBUSBSEN and VBUSASEN") {
	h.Init.vbus_sensing_enable = DISABLE;
	wr(oGCCFG, CC_VBUSBSEN | CC_VBUSASEN);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
	CHECK(rd(oGCCFG) == CC_NOVBUSSENS);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit with VBUS sensing on clears NOVBUSSENS, sets VBUSBSEN and leaves SDIS alone") {
	h.Init.vbus_sensing_enable = ENABLE;
	wr(oGCCFG, CC_NOVBUSSENS);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(rd(oGCCFG) == CC_VBUSBSEN);
	CHECK((rd(oDCTL) & DCTL_SDIS) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit clears PCGCCTL") {
	wr(oPCGCCTL, kMspMarkerPcgc);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(rd(oPCGCCTL) == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit with the embedded PHY programs DSPD 11") {
	h.Init.phy_itface = PCD_PHY_EMBEDDED;
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(dspd() == 3u);
}

// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_DevInit with ULPI and high speed leaves DSPD 00, with ULPI and another speed programs 01") {
	h.Init.phy_itface = USB_OTG_ULPI_PHY;
	h.Init.speed = USBD_HS_SPEED;
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(dspd() == 0u);
	h.Init.speed = USBD_FS_SPEED;
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(dspd() == 1u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit flushes all transmit FIFOs and the receive FIFO") {
	wr(oGRXFSIZ, 0x80);
	wr(oDIEPTXF0, 0x00400080u);
	for (unsigned i = 0; i < 3; ++i) wr(oDIEPTXF1 + 4u * i, 0x00400100u);
	for (unsigned ep = 0; ep < 4; ++ep) fifoWrite(ep, 0x11u + ep);
	hostOtgRxFill(0xAB, 3);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	for (unsigned ep = 0; ep < 4; ++ep) CHECK(hostOtgTxWordsUsed(ep) == 0u);
	CHECK(hostOtgRxWordsUsed() == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit clears the device interrupt masks") {
	wr(oDIEPMSK, 0x207B);
	wr(oDOEPMSK, 0x313B);
	wr(oDAINTMSK, 0x000F000F);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(rd(oDIEPMSK) == 0u);
	CHECK(rd(oDOEPMSK) == 0u);
	CHECK(rd(oDAINTMSK) == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit disables an enabled IN endpoint 1 to 3 and sets its NAK, and clears its flags, size and configuration") {
	for (unsigned n = 1; n < 4; ++n) {
		wr(inCtl(n), C_EPENA | C_USBAEP | C_STALL | 0x40u);
		wr(inSiz(n), 0x00080040u);
		wr(inInt(n), I_XFRC | I_EPDISD);
	}
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	for (unsigned n = 1; n < 4; ++n) {
		CAPTURE(n);
		CHECK((rd(inCtl(n)) & C_EPENA) == 0);   // RM0090 EPDIS: the endpoint is disabled, EPENA cleared
		CHECK((rd(inCtl(n)) & C_NAKSTS) == C_NAKSTS);  // SNAK
		CHECK((rd(inCtl(n)) & (C_STALL | C_USBAEP)) == 0);
		CHECK(rd(inSiz(n)) == 0u);
		CHECK((rd(inInt(n)) & (I_XFRC | I_EPDISD | I_INEPNE)) == 0);
	}
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit sets the NAK of an enabled IN endpoint 0 without disabling it") {
	wr(inCtl(0), C_EPENA);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(inCtl(0)) & C_NAKSTS) == C_NAKSTS);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit disables an enabled OUT endpoint 1 to 3 and sets its NAK, and clears its flags and size") {
	for (unsigned n = 1; n < 4; ++n) {
		wr(outCtl(n), C_EPENA | C_USBAEP | C_STALL | 0x40u);
		wr(outSiz(n), 0x00080040u);
		wr(outInt(n), I_XFRC | I_STUP_OR_AHBERR | I_EPDISD);
	}
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	for (unsigned n = 1; n < 4; ++n) {
		CAPTURE(n);
		CHECK((rd(outCtl(n)) & C_EPENA) == 0);
		CHECK((rd(outCtl(n)) & C_NAKSTS) == C_NAKSTS);
		CHECK((rd(outCtl(n)) & (C_STALL | C_USBAEP)) == 0);
		CHECK(rd(outSiz(n)) == 0u);
		CHECK((rd(outInt(n)) & (I_XFRC | I_STUP_OR_AHBERR | I_EPDISD)) == 0);
	}
}

// RM0090 DOEPCTL0 EPDIS: "The application cannot disable control OUT endpoint 0": the HAL only sets the NAK, the endpoint stays enabled.
TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit sets the NAK of an enabled OUT endpoint 0 and does not disable it") {
	wr(outCtl(0), C_EPENA | C_USBAEP);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(outCtl(0)) & C_NAKSTS) == C_NAKSTS);
	CHECK((rd(outCtl(0)) & C_EPENA) == C_EPENA);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit resets a disabled endpoint's pending flags and size") {
	wr(inSiz(2), 0x00080040u);
	wr(inInt(2), I_XFRC);
	wr(outSiz(1), 0x00080040u);
	wr(outInt(1), I_XFRC);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(rd(inSiz(2)) == 0u);
	CHECK((rd(inInt(2)) & I_XFRC) == 0);
	CHECK(rd(outSiz(1)) == 0u);
	CHECK((rd(outInt(1)) & I_XFRC) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit with dev_endpoints 2 leaves the registers of endpoints 2 and 3 alone") {
	h.Init.dev_endpoints = 2;
	wr(inSiz(2), 0x00080040u);
	wr(outSiz(3), 0x00080040u);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(rd(inSiz(2)) == 0x00080040u);
	CHECK(rd(outSiz(3)) == 0x00080040u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit clears pending global interrupts such as SOF and wakeup") {
	hostOtgSof(3);
	hostOtgResume();
	REQUIRE((rd(oGINTSTS) & (M_SOF | M_WUI)) == (M_SOF | M_WUI));
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(oGINTSTS) & (M_SOF | M_WUI)) == 0);
}

// NR345_19: USB_DevInit clears pending interrupts with GINTSTS = 0xBFFFFFFF, which leaves bit 30 (SRQINT, rc_w1 in RM0090) untouched: a pending session request survives.
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_DevInit does not clear a pending session request interrupt") {
	hostOtgVbus(1);
	REQUIRE((rd(oGINTSTS) & M_SRQ) == M_SRQ);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(oGINTSTS) & M_SRQ) == M_SRQ);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit builds the interrupt mask from the init flags: RXFLVL always in slave mode, SOF and VBUS only on request") {
	h.Init.Sof_enable = DISABLE;
	h.Init.vbus_sensing_enable = DISABLE;
	REQUIRE(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(rd(oGINTMSK) == 0x803C3810u);  // 0xC03C3814 without SRQ (bit 30) and OTGINT (bit 2), see NR345_1
	h.Init.Sof_enable = ENABLE;
	h.Init.vbus_sensing_enable = ENABLE;
	REQUIRE(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(rd(oGINTMSK) == 0xC03C381Cu);  // plus SOF (bit 3)
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit returns HAL_ERROR when the transmit FIFO flush never completes") {
	hostOtgSetPollReads(0);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_ERROR);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit returns HAL_ERROR when the AHB master never reports idle") {
	wr(oGRSTCTL, 0);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_ERROR);
	// the rest of the initialisation still runs: the interrupt masks are programmed
	CHECK((rd(oGINTMSK) & M_USBRST) == M_USBRST);
}

// --- connect / disconnect / address / stop / remote wakeup at LL level ------------------------------------------------------------------

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevConnect clears SDIS and the PHY clock stop bits and returns HAL_OK") {
	wr(oDCTL, DCTL_SDIS);
	wr(oPCGCCTL, kMspMarkerPcgc);
	CHECK(USB_DevConnect(USB_OTG_FS) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_SDIS) == 0);
	CHECK(rd(oPCGCCTL) == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevDisconnect sets SDIS, clears the PHY clock stop bits and returns HAL_OK") {
	wr(oPCGCCTL, kMspMarkerPcgc);
	CHECK(USB_DevDisconnect(USB_OTG_FS) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
	CHECK(rd(oPCGCCTL) == 0u);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevDisconnect keeps the soft disconnect state when applied twice") {
	CHECK(USB_DevDisconnect(USB_OTG_FS) == HAL_OK);
	CHECK(USB_DevDisconnect(USB_OTG_FS) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_SDIS) == DCTL_SDIS);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_SetDevAddress programs DAD for every 7 bit address boundary and keeps DSPD") {
	wr(oDCFG, rd(oDCFG) | 3u);
	const uint8_t addr[] = {0, 1, 0x40, 0x7E, 0x7F};
	for (uint8_t a : addr) {
		CAPTURE((int)a);
		CHECK(USB_SetDevAddress(USB_OTG_FS, a) == HAL_OK);
		CHECK(dad() == a);
		CHECK(dspd() == 3u);
	}
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_StopDevice clears the pending flags of endpoints 0 to 3 and the interrupt masks") {
	for (unsigned n = 0; n < 4; ++n) {
		wr(inInt(n), I_XFRC | I_EPDISD);
		wr(outInt(n), I_XFRC | I_STUP_OR_AHBERR);
	}
	wr(oDIEPMSK, 0x1);
	wr(oDOEPMSK, 0x1);
	wr(oDAINTMSK, 0x000F000F);
	CHECK(USB_StopDevice(USB_OTG_FS) == HAL_OK);
	for (unsigned n = 0; n < 4; ++n) {
		CAPTURE(n);
		CHECK((rd(inInt(n)) & (I_XFRC | I_EPDISD)) == 0);
		CHECK((rd(outInt(n)) & (I_XFRC | I_STUP_OR_AHBERR)) == 0);
	}
	CHECK(rd(oDIEPMSK) == 0u);
	CHECK(rd(oDOEPMSK) == 0u);
	CHECK(rd(oDAINTMSK) == 0u);
	clearFaults();
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_StopDevice flushes the receive FIFO and all transmit FIFOs") {
	wr(oGRXFSIZ, 0x80);
	wr(oDIEPTXF0, 0x00400080u);
	for (unsigned i = 0; i < 3; ++i) wr(oDIEPTXF1 + 4u * i, 0x00400100u);
	for (unsigned ep = 0; ep < 4; ++ep) fifoWrite(ep, 0x11u + ep);
	hostOtgRxFill(0xCD, 4);
	CHECK(USB_StopDevice(USB_OTG_FS) == HAL_OK);
	for (unsigned ep = 0; ep < 4; ++ep) CHECK(hostOtgTxWordsUsed(ep) == 0u);
	CHECK(hostOtgRxWordsUsed() == 0u);
	clearFaults();
}

// NR345_20: USB_StopDevice clears DIEPINT / DOEPINT of 15 endpoints; the OTG_FS core has 4, so endpoints 4 to 14 are 22 writes to reserved offsets (counted, no fault).
// NEEDS REVIEW 349
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_StopDevice writes the interrupt flag registers of endpoints 4 to 14 which the OTG_FS core does not have") {
	CHECK(USB_StopDevice(USB_OTG_FS) == HAL_OK);
	CHECK(hostOtgReservedAccesses() == 22u);
	CHECK(hostOtgReservedOffset(0) == 0x988u);  // DIEPINT of endpoint 4 = 0x900 + 4 * 0x20 + 8
	CHECK(hostOtgReservedWasWrite(0) == 1);
	clearFaults();
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_StopDevice returns HAL_TIMEOUT when the receive FIFO flush never completes") {
	hostOtgSetPollReads(0);
	CHECK(USB_StopDevice(USB_OTG_FS) == HAL_TIMEOUT);
	clearFaults();
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_StopDevice returns HAL_TIMEOUT when the AHB master never reports idle") {
	wr(oGRSTCTL, 0);
	CHECK(USB_StopDevice(USB_OTG_FS) == HAL_TIMEOUT);
	clearFaults();
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_ActivateRemoteWakeup sets RWUSIG when the bus is suspended and returns HAL_OK") {
	wr(oDSTS, DSTS_SUSPSTS);
	CHECK(USB_ActivateRemoteWakeup(USB_OTG_FS) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_RWUSIG) == DCTL_RWUSIG);
}

// NEEDS REVIEW 416
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_ActivateRemoteWakeup does not set RWUSIG when the bus is not suspended") {
	wr(oDSTS, 0);
	CHECK(USB_ActivateRemoteWakeup(USB_OTG_FS) == HAL_OK);
	CHECK((rd(oDCTL) & DCTL_RWUSIG) == 0);
}

TEST_CASE_FIXTURE(PiFx, "PI4 USB_DeActivateRemoteWakeup clears RWUSIG whether or not the bus is suspended") {
	wr(oDCTL, DCTL_RWUSIG | DCTL_SDIS);
	CHECK(USB_DeActivateRemoteWakeup(USB_OTG_FS) == HAL_OK);
	CHECK(rd(oDCTL) == DCTL_SDIS);
}

// L16: after USB_DevInit an enabled control endpoint 0 only gets the NAK, never EPDIS. RM0090 DOEPCTL0 EPDIS: "The application cannot disable control OUT endpoint 0",
// so the OUT endpoint stays enabled with NAKSTS set and no endpoint disabled flag.
TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit keeps an enabled OUT endpoint 0 enabled with NAKSTS set and EPDISD clear") {
	wr(outCtl(0), C_EPENA | C_USBAEP);
	wr(outInt(0), I_EPDISD);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(outCtl(0)) & C_EPENA) == C_EPENA);
	CHECK((rd(outCtl(0)) & C_NAKSTS) == C_NAKSTS);
	CHECK((rd(outInt(0)) & I_EPDISD) == 0);
}

// NR345_22: RM0090 DIEPCTL0 EPDIS does not say whether IN endpoint 0 can be disabled; the HAL writes SNAK only, so EPENA stays set, NAKSTS is set, EPDISD is clear.
// NEEDS REVIEW 345
TEST_CASE_FIXTURE(PiFx, "characterization: PI4 USB_DevInit keeps an enabled IN endpoint 0 enabled with NAKSTS set and EPDISD clear") {
	wr(inCtl(0), C_EPENA);
	wr(inInt(0), I_EPDISD);
	CHECK(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK((rd(inCtl(0)) & C_EPENA) == C_EPENA);
	CHECK((rd(inCtl(0)) & C_NAKSTS) == C_NAKSTS);
	CHECK((rd(inInt(0)) & I_EPDISD) == 0);
}

// R06: whatever GINTMSK held before, USB_DevInit leaves exactly the bits it lists: with SOF and VBUS sensing off the mask is 0x803C3810 (hand sum in NR345_1 without SOF,
// SRQ, OTGINT), with VBUS sensing on 0xC03C3814.
TEST_CASE_FIXTURE(PiFx, "PI4 USB_DevInit leaves only the listed GINTMSK bits when the mask was all ones before") {
	h.Init.Sof_enable = DISABLE;
	h.Init.vbus_sensing_enable = DISABLE;
	wr(oGINTMSK, 0xF73CFCFEu);  // every implemented mask bit set (RM0090 reserved bits read 0)
	REQUIRE(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(rd(oGINTMSK) == 0x803C3810u);
	h.Init.vbus_sensing_enable = ENABLE;
	wr(oGINTMSK, 0xF73CFCFEu);  // every implemented mask bit set (RM0090 reserved bits read 0)
	REQUIRE(USB_DevInit(USB_OTG_FS, h.Init) == HAL_OK);
	CHECK(rd(oGINTMSK) == 0xC03C3814u);
}
