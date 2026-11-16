// P12 T8 probe of the OTG_FS model against the real HAL (lead only; replaced by the real files).
#include "OtgTestSupport.h"
#include <iomanip>
#include <iostream>

TEST_CASE_FIXTURE(OtgFixture, "probe: init and bring up") {
	bringUp();
	CHECK(h.State == HAL_PCD_STATE_READY);
	CHECK(faults() == 0);
	for (unsigned i = 0; i < 16; ++i) if (hostOtgReservedOffset(i) != 0xFFFFFFFFu) MESSAGE("reserved " << std::hex << hostOtgReservedOffset(i) << (hostOtgReservedWasWrite(i) ? " W" : " R"));
	CHECK(hostOtgReservedAccesses() == 0);
	MESSAGE("hsonly " << hostOtgHsOnlyAccesses() << " first " << std::hex << hostOtgHsOnlyOffset(0));
}

TEST_CASE_FIXTURE(OtgFixture, "probe: enumeration flow") {
	bringUp();
	hostOtgUsbReset();
	drain();
	MESSAGE("reset cb " << otgCounts.reset);
	hostOtgEnumDone();
	drain();
	MESSAGE("enum speed " << h.Init.speed << " faults " << faults());
	CHECK(HAL_PCD_EP_Open(&h, 0x80, 64, EP_TYPE_CTRL) == HAL_OK);
	CHECK(HAL_PCD_EP_Open(&h, 0x00, 64, EP_TYPE_CTRL) == HAL_OK);
	static uint8_t rx[64];
	CHECK(HAL_PCD_EP_Receive(&h, 0x00, rx, 64) == HAL_OK);
	const uint8_t setup[8] = {0x80, 6, 0, 1, 0, 0, 18, 0};
	CHECK(hostOtgHostSetup(setup) == HOSTOTG_ACK);
	MESSAGE("calls " << drain());
	CHECK(otgCounts.setup == 1);
	CHECK(otgCounts.setupWords[0] == 0x01000680u);
	static uint8_t tx[24]; // V118: USB_WritePacket over-reads up to 3 bytes
	for (int i = 0; i < 18; ++i) tx[i] = (uint8_t)(i + 1);
	CHECK(HAL_PCD_EP_Transmit(&h, 0x80, tx, 18) == HAL_OK);
	MESSAGE("calls " << drain());
	uint8_t got[64];
	unsigned n = 0;
	CHECK(hostOtgHostIn(0, got, 64, &n) == HOSTOTG_ACK);
	CHECK(n == 18);
	CHECK(got[17] == 18);
	drain();
	CHECK(otgCounts.dataIn == 1);
	CHECK(HAL_PCD_EP_Open(&h, 0x01, 64, EP_TYPE_BULK) == HAL_OK);
	CHECK(HAL_PCD_EP_Receive(&h, 0x01, rx, 64) == HAL_OK);
	const uint8_t pkt[5] = {1, 2, 3, 4, 5};
	CHECK(hostOtgHostOut(1, pkt, 5, 0) == HOSTOTG_ACK);
	MESSAGE("calls " << drain());
	CHECK(otgCounts.dataOut == 1);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x01) == 5);
	CHECK(rx[4] == 5);
	for (unsigned i = 0; i < 16; ++i) if (hostOtgReservedOffset(i) != 0xFFFFFFFFu) MESSAGE("reserved " << hostOtgReservedOffset(i) << (hostOtgReservedWasWrite(i) ? " W" : " R"));
	MESSAGE("reset cb " << otgCounts.reset << " hsonly " << hostOtgHsOnlyAccesses());
	CHECK(hostOtgReservedAccesses() == 0);
	CHECK(faults() == 0);
}
