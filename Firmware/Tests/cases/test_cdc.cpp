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
#include "thread.hpp"

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
		CDCcomm::clearRemainingBuffer(0); // Nothing left to retry
		CDCcomm::cdcFinished(0);		  // Gives the port semaphore back first, so the next send can take it
		std::string sync = "-";
		CDCcomm::cdcSend(&sync, 0); // Fits: clears the retry flag
		CDCcomm::cdcFinished(0);	// Releases the port again
		HostUsb::reset();
	}
};

/** Firmware style thread sending one reply when requested. Retries from the start of Run() after blocking */
class SenderThread : public cpp_freertos::Thread {
public:
	SenderThread() : Thread("CDCTEST", 64, 1) { Start(); }
	void Run() override {
		while (true) {
			if (pending) {
				CDCcomm::cdcSend(&reply, 0); // Blocks while the previous transfer is running
				pending = false;
				sent++;
			}
			WaitForNotification();
		}
	}
	std::string reply = "second";
	bool pending = false;
	uint32_t sent = 0;
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

TEST_CASE_FIXTURE(CdcFixture, "a reply is buffered completely while the usb fifo is full") {
	HostUsb::state().cdcWriteAvailable = 0;
	std::string reply = "abc";
	CHECK(CDCcomm::cdcSend(&reply, 0) == 0);
	CHECK(HostUsb::state().cdcTx.empty());
	CHECK(CDCcomm::remainingData() == 3);

	HostUsb::state().cdcWriteAvailable = 1024;
	CDCcomm::cdcFinished(0);
	CHECK(HostUsb::state().cdcTx == "abc");
	CHECK(CDCcomm::remainingData() == 0);
}

TEST_CASE_FIXTURE(CdcFixture, "a second reply waits until the running transfer is finished") {
	SenderThread sender;
	std::string first = "first";
	REQUIRE(CDCcomm::cdcSend(&first, 0) == 5); // Takes the port until the transfer completes

	sender.pending = true;
	HostRtos::runUntilBlocked(sender);
	CHECK(sender.sent == 0); // Blocked in cdcSend
	CHECK(HostUsb::state().cdcTx == "first");

	CDCcomm::cdcFinished(0); // tud_cdc_tx_complete_cb gives the port back
	HostRtos::runUntilBlocked(sender);
	CHECK(sender.sent == 1);
	CHECK(HostUsb::state().cdcTx == "firstsecond");
}

TEST_CASE_FIXTURE(CdcFixture, "a reply longer than 65535 bytes is sent in fifo sized parts" * KNOWN_ISSUE("CDCcomm::cdcSend limits the write with std::min<uint16_t>(reply->length(), bufferRemaining) (CDCcomm.cpp:69). The length is truncated to 16 bit: a 65546 byte reply sends only 10 bytes, the remainder of exactly 65536 bytes is truncated to 0. The retry from cdcFinished writes 0 bytes without a flush (usb_busy_retry is set), so no tx complete follows, cdcSems[0] is never given back and every later cdcSend blocks: CDC output is dead. Same for any length that is a multiple of 65536")) {
	std::string reply(65536 + 10, 'x');
	CHECK(CDCcomm::cdcSend(&reply, 0) == 1024); // Free space of the fake fifo (actual: 10)
	CDCcomm::cdcFinished(0);					// Retry with the remainder
	CHECK(HostUsb::state().cdcTx.size() == 2048); // Next part sent (actual: 0 more bytes)
	CHECK(CDCcomm::remainingData() == reply.size() - 2048);
	// The fixture gives the port semaphore back and clears the remainder
}

} // TEST_SUITE
