/*
 * USB CDC helper (CDCcomm.cpp), directly on the fake TinyUSB device API (support/HostUsb.h).
 *
 * CDCcomm only has static members. Its constructor and destructor are private and never called,
 * so they can not be tested.
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "FirmwareFixture.h"

#include "CDCcomm.h"

#include <string>

namespace {

/**
 * FirmwareFixture does not reset the internal retry flag of CDCcomm (set while a reply is only partly sent,
 * not cleared by clearRemainingBuffer). A complete send clears it, then the recorded usb data is reset.
 */
struct CdcFixture : FirmwareFixture {
	CdcFixture() { resetCdc(); }
	~CdcFixture() override { resetCdc(); }
	static void resetCdc() {
		HostUsb::reset();
		CDCcomm::clearRemainingBuffer(0);
		std::string sync = "-";
		CDCcomm::cdcSend(&sync, 0); // Fits: clears the retry flag
		CDCcomm::cdcFinished(0);	// Releases the port
		HostUsb::reset();
	}
};

} // namespace

TEST_SUITE("CDCcomm") {

TEST_CASE_FIXTURE(CdcFixture, "connected reports the DTR state of the usb stack") {
	HostUsb::state().cdcConnected = true;
	CHECK(CDCcomm::connected());
	CHECK(CDCcomm::connected(0));
	HostUsb::state().cdcConnected = false;
	CHECK_FALSE(CDCcomm::connected());
}

TEST_CASE_FIXTURE(CdcFixture, "a reply that fits into the usb fifo is sent completely and flushed") {
	std::string reply = "[sys.hello]\n";
	CHECK(CDCcomm::cdcSend(&reply, 0) == reply.size());
	CHECK(HostUsb::state().cdcTx == reply);
	CHECK(HostUsb::state().cdcWrites == 1);
	CHECK(HostUsb::state().cdcFlushes == 1);
	CHECK(CDCcomm::remainingData() == 0);
}

TEST_CASE_FIXTURE(CdcFixture, "nothing is sent while usb is not ready") {
	HostUsb::state().mounted = false;
	std::string reply = "abc";
	CHECK(CDCcomm::cdcSend(&reply, 0) == 0);
	CHECK(HostUsb::state().cdcWrites == 0);
	CHECK(HostUsb::state().cdcFlushes == 0);
	CHECK(CDCcomm::remainingData() == 0); // Not buffered for later
}

TEST_CASE_FIXTURE(CdcFixture, "an empty reply is not sent") {
	std::string reply;
	CHECK(CDCcomm::cdcSend(&reply, 0) == 0);
	CHECK(HostUsb::state().cdcWrites == 0);
}

TEST_CASE_FIXTURE(CdcFixture, "the part of a reply that does not fit is sent when the transfer completes") {
	HostUsb::state().cdcWriteAvailable = 4;
	std::string reply = "abcdefghij";
	CHECK(CDCcomm::cdcSend(&reply, 0) == 4);
	CHECK(HostUsb::state().cdcTx == "abcd");
	CHECK(CDCcomm::remainingData() == 6);
	CHECK(HostUsb::state().cdcFlushes == 1);

	CDCcomm::cdcFinished(0); // tud_cdc_tx_complete_cb
	CHECK(HostUsb::state().cdcTx == "abcdefgh");
	CHECK(CDCcomm::remainingData() == 2);
	CHECK(HostUsb::state().cdcFlushes == 1); // Retries from the callback are flushed by tinyusb

	CDCcomm::cdcFinished(0);
	CHECK(HostUsb::state().cdcTx == "abcdefghij");
	CHECK(CDCcomm::remainingData() == 0);

	CDCcomm::cdcFinished(0); // Nothing left
	CHECK(HostUsb::state().cdcWrites == 3);
}

TEST_CASE_FIXTURE(CdcFixture, "a new reply after a completed retry is flushed again") {
	HostUsb::state().cdcWriteAvailable = 4;
	std::string reply = "abcdef";
	CDCcomm::cdcSend(&reply, 0);
	CDCcomm::cdcFinished(0); // Sends "ef", retry finished
	REQUIRE(CDCcomm::remainingData() == 0);
	CDCcomm::cdcFinished(0);

	std::string next = "xy";
	CHECK(CDCcomm::cdcSend(&next, 0) == 2);
	CHECK(HostUsb::state().cdcFlushes == 2);
}

TEST_CASE_FIXTURE(CdcFixture, "clearing the remaining buffer drops the unsent part") {
	HostUsb::state().cdcWriteAvailable = 2;
	std::string reply = "abcdef";
	CDCcomm::cdcSend(&reply, 0);
	REQUIRE(CDCcomm::remainingData() == 4);
	CDCcomm::clearRemainingBuffer();
	CHECK(CDCcomm::remainingData() == 0);
	CDCcomm::cdcFinished(0);
	CHECK(HostUsb::state().cdcTx == "ab");
	CHECK(HostUsb::state().cdcWrites == 1);
}

TEST_CASE_FIXTURE(CdcFixture, "a reply longer than 65535 bytes is sent in fifo sized parts" * KNOWN_ISSUE("CDCcomm::cdcSend limits the write with std::min<uint16_t>(reply->length(), bufferRemaining) (CDCcomm.cpp:69). The reply length is truncated to 16 bit first, so a 65546 byte reply is sent as 10 bytes per transfer instead of the free fifo space")) {
	std::string reply(65536 + 10, 'x');
	CHECK(CDCcomm::cdcSend(&reply, 0) == 1024); // Free space of the fake fifo
	CHECK(CDCcomm::remainingData() == reply.size() - 1024);
	CDCcomm::clearRemainingBuffer();
}

} // TEST_SUITE
