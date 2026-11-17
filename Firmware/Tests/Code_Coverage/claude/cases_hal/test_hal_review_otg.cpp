// P12 review witnesses, USB OTG FS device (Codex review of P12, P12C-T8-001 and P12C-T8-002).
#include "OtgTestSupport.h"

TEST_CASE_FIXTURE(OtgFixture, "RV16 a USB reset deactivates every endpoint but EP0 before the interrupt is served (P12C-T8-001)") {
	// RM0090 DIEPCTLx / DOEPCTLx USBAEP: the core clears it for all endpoints (other than EP0) after detecting a USB reset.
	bringUp();
	for (unsigned e = 0; e < 4; ++e) {
		REQUIRE(HAL_PCD_EP_Open(&h, e, 64, EP_TYPE_BULK) == HAL_OK);
		REQUIRE(HAL_PCD_EP_Open(&h, 0x80 | e, 64, EP_TYPE_BULK) == HAL_OK);
	}
	hostOtgUsbReset();
	for (unsigned e = 1; e < 4; ++e) {
		CHECK((hostRegsPeek(HOST_OTG_BASE + 0x900 + e * 0x20) & 0x8000) == 0);  // DIEPCTLx.USBAEP
		CHECK((hostRegsPeek(HOST_OTG_BASE + 0xb00 + e * 0x20) & 0x8000) == 0);  // DOEPCTLx.USBAEP
	}
	CHECK((hostRegsPeek(HOST_OTG_BASE + 0x900) & 0x8000) != 0);  // EP0 stays active
	CHECK((hostRegsPeek(HOST_OTG_BASE + 0xb00) & 0x8000) != 0);
	uint8_t data[4] = {1, 2, 3, 4};
	unsigned n = 0;
	CHECK(hostOtgHostIn(1, data, 4, &n) == HOSTOTG_IGNORED);
	CHECK(hostOtgHostOut(1, data, 4, 0) == HOSTOTG_IGNORED);
}

TEST_CASE_FIXTURE(OtgFixture, "RV17 a bulk OUT packet that repeats the previous DATA PID is acknowledged and discarded (P12C-T8-002)") {
	// USB 2.0 data toggle synchronisation: a packet with the PID already received is a retransmission; the device ACKs it and drops the data.
	// Neither the Rx FIFO nor the packet count may change.
	bringUp();
	hostOtgUsbReset();
	drain();
	hostOtgEnumDone();
	drain();
	REQUIRE(HAL_PCD_EP_Open(&h, 1, 64, EP_TYPE_BULK) == HAL_OK);
	uint8_t rx[128] = {0}, data[64] = {1, 2, 3, 4};
	REQUIRE(HAL_PCD_EP_Receive(&h, 1, rx, 128) == HAL_OK);
	REQUIRE(hostOtgHostOut(1, data, 64, 0) == HOSTOTG_ACK);
	drain();
	const uint32_t before = hostRegsPeek(HOST_OTG_BASE + 0xb30);  // DOEPTSIZ1
	REQUIRE(HAL_PCD_EP_GetRxCount(&h, 1) == 64);
	REQUIRE(hostOtgHostOut(1, data, 64, 0) == HOSTOTG_ACK);  // the same DATA0 again
	CHECK(hostRegsPeek(HOST_OTG_BASE + 0xb30) == before);
	CHECK(hostOtgRxWordsUsed() == 0);
	CHECK(hostOtgDuplicatePackets() == 1u);
	drain();
	CHECK(HAL_PCD_EP_GetRxCount(&h, 1) == 64);
	CHECK(otgCounts.dataOut == 0);
}
