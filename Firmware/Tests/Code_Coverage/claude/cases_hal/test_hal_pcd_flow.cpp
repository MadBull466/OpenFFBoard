// P12 T8 group PF: end-to-end USB device transfers through the real ST HAL PCD (device mode, OTG_FS) with the HalOtg model and the host side played by the test.
// Oracles: UM1725 (HAL PCD API), RM0090 34.17.5 (control / bulk flows), USB 2.0 packet rules (bytes in the host = bytes given to the HAL, packets of at most the max packet size,
// a short packet ends the transfer, DATA stage then zero length status stage). Expected values are hand derived in the comments.
#include "OtgTestSupport.h"

#include <cstring>
#include <vector>

namespace {

const uint8_t kGetDesc[8] = {0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00};  // GET_DESCRIPTOR(device), wLength 18
const uint8_t kSetVendor[8] = {0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00}; // vendor request host to device, wLength 20

uint8_t flowByte(unsigned seed, unsigned i) { return (uint8_t)(seed * 37u + i * 7u + 1u); }

struct FlowRig : public OtgFixture {
	uint8_t txBuf[512];
	uint8_t rxBuf[512];

	FlowRig() {
		std::memset(txBuf, 0, sizeof txBuf);
		std::memset(rxBuf, 0xEE, sizeof rxBuf);
	}

	// The firmware's bring-up with a chosen TxFIFO depth for endpoints 1-3, then connect, USB reset, enumeration done, EP0 and data endpoints opened.
	void enumerate(unsigned txWords = 0x40u, unsigned mps0 = 64u) {
		REQUIRE(init() == HAL_OK);
		REQUIRE(HAL_PCDEx_SetRxFiFo(&h, 0x40) == HAL_OK);
		REQUIRE(HAL_PCDEx_SetTxFiFo(&h, 0, 0x20) == HAL_OK);
		for (unsigned f = 1; f <= 3; ++f) REQUIRE(HAL_PCDEx_SetTxFiFo(&h, (uint8_t)f, (uint16_t)txWords) == HAL_OK);
		REQUIRE(HAL_PCD_Start(&h) == HAL_OK);
		hostOtgUsbReset();
		drain();
		hostOtgEnumDone();
		drain();
		REQUIRE(HAL_PCD_EP_Open(&h, 0x80, (uint16_t)mps0, EP_TYPE_CTRL) == HAL_OK);
		REQUIRE(HAL_PCD_EP_Open(&h, 0x00, (uint16_t)mps0, EP_TYPE_CTRL) == HAL_OK);
		for (uint8_t e = 1; e <= 3; ++e) {
			REQUIRE(HAL_PCD_EP_Open(&h, (uint8_t)(0x80 | e), 64, EP_TYPE_BULK) == HAL_OK);
			REQUIRE(HAL_PCD_EP_Open(&h, e, 64, EP_TYPE_BULK) == HAL_OK);
		}
		otgSupportReset();  // callbacks of the bring-up are not part of the flow under test
	}

	void setupStage(const uint8_t *s) {
		REQUIRE(hostOtgHostSetup(s) == HOSTOTG_ACK);
		drain();
	}

	// Host reads from an IN endpoint until a short packet or `len` bytes. Returns the bytes; packets and NAKs counted.
	std::vector<uint8_t> pullIn(unsigned ep, unsigned len, unsigned *packets = nullptr, unsigned *naks = nullptr) {
		std::vector<uint8_t> out;
		unsigned pk = 0, nk = 0;
		for (unsigned it = 0; it < 64; ++it) {
			drain();
			uint8_t buf[64];
			unsigned n = 0;
			HostOtgHandshake hs = hostOtgHostIn(ep, buf, 64, &n);
			if (hs == HOSTOTG_ACK) {
				out.insert(out.end(), buf, buf + n);
				++pk;
				if (n < 64 || out.size() >= len) break;
			} else {
				++nk;
			}
		}
		drain();
		if (packets) *packets = pk;
		if (naks) *naks = nk;
		return out;
	}

	// Host sends `len` bytes to an OUT endpoint in packets of at most 64 bytes (a zero length packet when len is 0).
	void pushOut(unsigned ep, const uint8_t *data, unsigned len, int pid1 = 0) {
		unsigned off = 0;
		do {
			unsigned n = len - off > 64u ? 64u : len - off;
			REQUIRE(hostOtgHostOut(ep, data + off, n, pid1) == HOSTOTG_ACK);
			drain();
			off += n;
			pid1 ^= 1;  // a real host toggles DATA0 / DATA1 per packet; a repeated PID is a retransmission (Codex review P12C-T8-002)
		} while (off < len);
	}

	unsigned countOf(unsigned kind) {
		unsigned c = 0;
		for (unsigned i = 0; i < otgCounts.orderCount; ++i) if (otgCounts.order[i] == kind) ++c;
		return c;
	}
};

}  // namespace

// ---------------------------------------------------------------- PF1 control read

TEST_CASE_FIXTURE(FlowRig, "PF1 a SETUP packet reaches the setup callback with its 8 bytes in the setup words") {
	enumerate();
	setupStage(kGetDesc);
	CHECK(otgCounts.setup == 1);
	// bytes 80 06 00 01 | 00 00 12 00 little endian words
	CHECK(otgCounts.setupWords[0] == 0x01000680u);
	CHECK(otgCounts.setupWords[1] == 0x00120000u);
	CHECK(otgCounts.dataOut == 0);
	CHECK(faults() == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF1 control read of 18 bytes with max packet 64 arrives as one packet in order") {
	enumerate();
	setupStage(kGetDesc);
	for (unsigned i = 0; i < 18; ++i) txBuf[i] = flowByte(1, i);
	CHECK(HAL_PCD_EP_Transmit(&h, 0x80, txBuf, 18) == HAL_OK);
	unsigned pk = 0;
	std::vector<uint8_t> got = pullIn(0, 18, &pk);
	CHECK(pk == 1);
	REQUIRE(got.size() == 18);
	CHECK(std::memcmp(got.data(), txBuf, 18) == 0);
	CHECK(otgCounts.dataIn == 1);
	CHECK(otgCounts.lastEp == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF1 control read of 18 bytes with max packet 8 arrives as packets of 8, 8 and 2 bytes") {
	enumerate(0x40, 8);
	setupStage(kGetDesc);
	for (unsigned i = 0; i < 18; ++i) txBuf[i] = flowByte(2, i);
	std::vector<uint8_t> all;
	unsigned sizes[3] = {0, 0, 0};
	unsigned off = 0, p = 0;
	// as a class driver does: Transmit the rest, the HAL sends one packet (EP0 length is limited to the max packet), the DataIn callback asks for the rest
	while (off < 18 && p < 3) {
		CHECK(HAL_PCD_EP_Transmit(&h, 0x80, txBuf + off, 18 - off) == HAL_OK);
		std::vector<uint8_t> chunk = pullIn(0, 8);
		sizes[p++] = (unsigned)chunk.size();
		all.insert(all.end(), chunk.begin(), chunk.end());
		off += (unsigned)chunk.size();
	}
	CHECK(sizes[0] == 8);
	CHECK(sizes[1] == 8);
	CHECK(sizes[2] == 2);
	REQUIRE(all.size() == 18);
	CHECK(std::memcmp(all.data(), txBuf, 18) == 0);
	CHECK(otgCounts.dataIn == 3);
}

TEST_CASE_FIXTURE(FlowRig, "PF1 the host IN token of EP0 is NAKed while no transfer is armed") {
	enumerate();
	setupStage(kGetDesc);
	uint8_t buf[64];
	unsigned n = 0;
	CHECK(hostOtgHostIn(0, buf, 64, &n) == HOSTOTG_NAK);
	CHECK(otgCounts.dataIn == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF1 the status stage of a control read is a zero length OUT that calls the data out callback of EP0") {
	enumerate();
	setupStage(kGetDesc);
	for (unsigned i = 0; i < 18; ++i) txBuf[i] = flowByte(3, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x80, txBuf, 18) == HAL_OK);
	pullIn(0, 18);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x00, nullptr, 0) == HAL_OK);  // status stage armed by the class driver
	REQUIRE(hostOtgHostOut(0, nullptr, 0, 1) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 1);
	CHECK(otgCounts.lastEp == 0);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x00) == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF1 callbacks of a complete control read come in the order setup, data in, data out") {
	enumerate();
	setupStage(kGetDesc);
	for (unsigned i = 0; i < 18; ++i) txBuf[i] = flowByte(4, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x80, txBuf, 18) == HAL_OK);
	pullIn(0, 18);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x00, nullptr, 0) == HAL_OK);
	REQUIRE(hostOtgHostOut(0, nullptr, 0, 1) == HOSTOTG_ACK);
	drain();
	REQUIRE(otgCounts.orderCount == 3);
	CHECK(otgCounts.order[0] == OTG_CB_SETUP);
	CHECK(otgCounts.order[1] == OTG_CB_DATA_IN);
	CHECK(otgCounts.order[2] == OTG_CB_DATA_OUT);
	CHECK(otgCounts.epOrder[1] == 0);
	CHECK(otgCounts.epOrder[2] == 0);
	CHECK(faults() == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF1 a second SETUP is received after the status stage of a control read") {
	enumerate();
	setupStage(kGetDesc);
	for (unsigned i = 0; i < 18; ++i) txBuf[i] = flowByte(5, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x80, txBuf, 18) == HAL_OK);
	pullIn(0, 18);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x00, nullptr, 0) == HAL_OK);
	REQUIRE(hostOtgHostOut(0, nullptr, 0, 1) == HOSTOTG_ACK);
	drain();
	const uint8_t second[8] = {0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x09, 0x00};
	setupStage(second);
	CHECK(otgCounts.setup == 2);
	CHECK(otgCounts.setupWords[0] == 0x02000680u);
	CHECK(otgCounts.setupWords[1] == 0x00090000u);
}

// ---------------------------------------------------------------- PF2 control write

TEST_CASE_FIXTURE(FlowRig, "PF2 control write of 20 bytes with max packet 64 arrives as one OUT packet") {
	enumerate();
	setupStage(kSetVendor);
	uint8_t data[20];
	for (unsigned i = 0; i < 20; ++i) data[i] = flowByte(6, i);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x00, rxBuf, 20) == HAL_OK);
	REQUIRE(hostOtgHostOut(0, data, 20, 1) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 1);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x00) == 20);
	CHECK(std::memcmp(rxBuf, data, 20) == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF2 control write of 64 bytes with max packet 64 fills the buffer in one packet") {
	enumerate();
	setupStage(kSetVendor);
	uint8_t data[64];
	for (unsigned i = 0; i < 64; ++i) data[i] = flowByte(7, i);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x00, rxBuf, 64) == HAL_OK);
	REQUIRE(hostOtgHostOut(0, data, 64, 1) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 1);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x00) == 64);
	CHECK(std::memcmp(rxBuf, data, 64) == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF2 control write of 20 bytes with max packet 8 arrives as packets of 8, 8 and 4 bytes with a callback each") {
	enumerate(0x40, 8);
	setupStage(kSetVendor);
	uint8_t data[20];
	for (unsigned i = 0; i < 20; ++i) data[i] = flowByte(8, i);
	unsigned off = 0, pk = 0;
	unsigned counts[3] = {0, 0, 0};
	while (off < 20 && pk < 3) {
		unsigned n = 20 - off > 8u ? 8u : 20 - off;
		REQUIRE(HAL_PCD_EP_Receive(&h, 0x00, rxBuf + off, (uint32_t)(20 - off)) == HAL_OK);
		REQUIRE(hostOtgHostOut(0, data + off, n, pk == 0 ? 1 : (int)(pk & 1u) ^ 1) == HOSTOTG_ACK);
		drain();
		counts[pk] = HAL_PCD_EP_GetRxCount(&h, 0x00);
		off += n;
		++pk;
	}
	CHECK(counts[0] == 8);
	CHECK(counts[1] == 8);
	CHECK(counts[2] == 4);
	CHECK(otgCounts.dataOut == 3);
	CHECK(std::memcmp(rxBuf, data, 20) == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF2 the status stage of a control write is a zero length IN that calls the data in callback of EP0") {
	enumerate();
	setupStage(kSetVendor);
	uint8_t data[20];
	for (unsigned i = 0; i < 20; ++i) data[i] = flowByte(9, i);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x00, rxBuf, 20) == HAL_OK);
	REQUIRE(hostOtgHostOut(0, data, 20, 1) == HOSTOTG_ACK);
	drain();
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x80, nullptr, 0) == HAL_OK);
	unsigned pk = 0;
	std::vector<uint8_t> got = pullIn(0, 0, &pk);
	CHECK(pk == 1);
	CHECK(got.size() == 0);
	CHECK(otgCounts.dataIn == 1);
	CHECK(otgCounts.lastEp == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF2 callbacks of a complete control write come in the order setup, data out, data in") {
	enumerate();
	setupStage(kSetVendor);
	uint8_t data[20];
	for (unsigned i = 0; i < 20; ++i) data[i] = flowByte(10, i);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x00, rxBuf, 20) == HAL_OK);
	REQUIRE(hostOtgHostOut(0, data, 20, 1) == HOSTOTG_ACK);
	drain();
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x80, nullptr, 0) == HAL_OK);
	pullIn(0, 0);
	REQUIRE(otgCounts.orderCount == 3);
	CHECK(otgCounts.order[0] == OTG_CB_SETUP);
	CHECK(otgCounts.order[1] == OTG_CB_DATA_OUT);
	CHECK(otgCounts.order[2] == OTG_CB_DATA_IN);
	CHECK(faults() == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF2 a second SETUP is received after the status stage of a control write") {
	enumerate();
	setupStage(kSetVendor);
	uint8_t data[20];
	for (unsigned i = 0; i < 20; ++i) data[i] = flowByte(11, i);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x00, rxBuf, 20) == HAL_OK);
	REQUIRE(hostOtgHostOut(0, data, 20, 1) == HOSTOTG_ACK);
	drain();
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x80, nullptr, 0) == HAL_OK);
	pullIn(0, 0);
	setupStage(kGetDesc);
	CHECK(otgCounts.setup == 2);
	CHECK(otgCounts.setupWords[0] == 0x01000680u);
}

// ---------------------------------------------------------------- PF3 bulk IN

namespace {
const unsigned kInSizes[] = {0, 1, 63, 64, 65, 128, 130};

// One bulk IN transfer of `len` bytes on `ep`: bytes, packet count and callback count are checked against the USB rules.
void flowCheckBulkIn(FlowRig &r, unsigned ep, unsigned len) {
	CAPTURE(ep);
	CAPTURE(len);
	for (unsigned i = 0; i < len; ++i) r.txBuf[i] = flowByte(ep, i + len);
	unsigned before = otgCounts.dataIn;
	REQUIRE(HAL_PCD_EP_Transmit(&r.h, (uint8_t)(0x80 | ep), r.txBuf, len) == HAL_OK);
	unsigned pk = 0;
	std::vector<uint8_t> got = r.pullIn(ep, len, &pk);
	// packets: ceil(len / 64), at least one (a zero length transfer is one zero length packet)
	unsigned expectPk = len == 0 ? 1u : (len + 63u) / 64u;
	CHECK(pk == expectPk);
	REQUIRE(got.size() == len);
	CHECK((len == 0 || std::memcmp(got.data(), r.txBuf, len) == 0));
	CHECK(otgCounts.dataIn == before + 1);
	CHECK(otgCounts.lastEp == ep);
}

void flowRunBulkIn(unsigned ep, unsigned txWords) {
	FlowRig r;
	r.enumerate(txWords);
	for (unsigned s : kInSizes) flowCheckBulkIn(r, ep, s);
	CHECK(HalFixture::faults() == 0);
}
}  // namespace

TEST_CASE("PF3 bulk IN on EP1 delivers 0, 1, 63, 64, 65, 128 and 130 bytes in order with a 64 word FIFO") {
	flowRunBulkIn(1, 0x40);
}
TEST_CASE("PF3 bulk IN on EP2 delivers 0, 1, 63, 64, 65, 128 and 130 bytes in order with a 64 word FIFO") {
	flowRunBulkIn(2, 0x40);
}
TEST_CASE("PF3 bulk IN on EP3 delivers 0, 1, 63, 64, 65, 128 and 130 bytes in order with a 64 word FIFO") {
	flowRunBulkIn(3, 0x40);
}
TEST_CASE("PF3 bulk IN on EP1 delivers 0, 1, 63, 64, 65, 128 and 130 bytes in order with a 16 word FIFO refilled per packet") {
	flowRunBulkIn(1, 0x10);
}
TEST_CASE("PF3 bulk IN on EP2 delivers 0, 1, 63, 64, 65, 128 and 130 bytes in order with a 16 word FIFO refilled per packet") {
	flowRunBulkIn(2, 0x10);
}
TEST_CASE("PF3 bulk IN on EP3 delivers 0, 1, 63, 64, 65, 128 and 130 bytes in order with a 16 word FIFO refilled per packet") {
	flowRunBulkIn(3, 0x10);
}

TEST_CASE_FIXTURE(FlowRig, "PF3 a transfer of 130 bytes with a 16 word FIFO needs one FIFO fill per packet") {
	enumerate(0x10);
	for (unsigned i = 0; i < 130; ++i) txBuf[i] = flowByte(12, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 130) == HAL_OK);
	// hand derivation: a packet of 64 bytes is 16 words, the FIFO holds exactly 16 words: after the first fill the FIFO is full
	drain();
	CHECK(hostOtgTxWordsUsed(1) == 16);
	uint8_t buf[64];
	unsigned n = 0;
	REQUIRE(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_ACK);
	CHECK(n == 64);
	CHECK(hostOtgTxWordsUsed(1) == 0);
	drain();
	CHECK(hostOtgTxWordsUsed(1) == 16);  // second packet written by the TXFE handling
	REQUIRE(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_ACK);
	CHECK(n == 64);
	drain();
	CHECK(hostOtgTxWordsUsed(1) == 1);  // the last 2 bytes are one word
	CHECK(otgCounts.dataIn == 0);       // not complete before the last packet leaves
	REQUIRE(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_ACK);
	CHECK(n == 2);
	drain();
	CHECK(otgCounts.dataIn == 1);
}

TEST_CASE_FIXTURE(FlowRig, "PF3 the host IN token is NAKed before the TxFIFO holds the data of an armed transfer") {
	enumerate();
	for (unsigned i = 0; i < 10; ++i) txBuf[i] = flowByte(13, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 10) == HAL_OK);
	uint8_t buf[64];
	unsigned n = 0;
	CHECK(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_NAK);  // nothing in the FIFO yet: the interrupt handler has not run
	drain();
	CHECK(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_ACK);
	CHECK(n == 10);
}

TEST_CASE_FIXTURE(FlowRig, "PF3 the host IN token of a bulk endpoint is NAKed while no transfer is armed") {
	enumerate();
	uint8_t buf[64];
	unsigned n = 0;
	for (unsigned e = 1; e <= 3; ++e) CHECK(hostOtgHostIn(e, buf, 64, &n) == HOSTOTG_NAK);
	CHECK(otgCounts.dataIn == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF3 a transfer of exactly 64 bytes is not followed by a zero length packet and the next token is NAKed") {
	enumerate();
	for (unsigned i = 0; i < 64; ++i) txBuf[i] = flowByte(14, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 64) == HAL_OK);
	std::vector<uint8_t> got = pullIn(1, 64);
	CHECK(got.size() == 64);
	uint8_t buf[64];
	unsigned n = 0;
	drain();
	CHECK(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_NAK);  // the class driver adds the ZLP, the HAL does not
	drain();
	CHECK(otgCounts.dataIn == 1);
}

TEST_CASE_FIXTURE(FlowRig, "PF3 the zero length packet of the class driver after 128 bytes is a second transfer with its own callback") {
	enumerate();
	for (unsigned i = 0; i < 128; ++i) txBuf[i] = flowByte(15, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 128) == HAL_OK);
	CHECK(pullIn(1, 128).size() == 128);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, nullptr, 0) == HAL_OK);
	unsigned pk = 0;
	CHECK(pullIn(1, 0, &pk).size() == 0);
	CHECK(pk == 1);
	CHECK(otgCounts.dataIn == 2);
}

TEST_CASE_FIXTURE(FlowRig, "PF3 two IN endpoints armed together deliver their own data") {
	enumerate();
	for (unsigned i = 0; i < 70; ++i) txBuf[i] = flowByte(16, i);
	for (unsigned i = 0; i < 70; ++i) txBuf[100 + i] = flowByte(17, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 70) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x82, txBuf + 100, 70) == HAL_OK);
	std::vector<uint8_t> b = pullIn(2, 70);
	std::vector<uint8_t> a = pullIn(1, 70);
	REQUIRE(a.size() == 70);
	REQUIRE(b.size() == 70);
	CHECK(std::memcmp(a.data(), txBuf, 70) == 0);
	CHECK(std::memcmp(b.data(), txBuf + 100, 70) == 0);
	CHECK(otgCounts.dataIn == 2);
}

// ---------------------------------------------------------------- PF4 bulk OUT

namespace {
// Receive `recv` bytes capacity on `ep`, host sends `send` bytes; checks data, count and one callback.
void flowCheckBulkOut(FlowRig &r, unsigned ep, unsigned send) {
	CAPTURE(ep);
	CAPTURE(send);
	uint8_t data[512];
	for (unsigned i = 0; i < send; ++i) data[i] = flowByte(ep + 20, i);
	std::memset(r.rxBuf, 0xEE, sizeof r.rxBuf);
	unsigned before = otgCounts.dataOut;
	REQUIRE(HAL_PCD_EP_Receive(&r.h, (uint8_t)ep, r.rxBuf, send) == HAL_OK);
	r.pushOut(ep, data, send);
	CHECK(otgCounts.dataOut == before + 1);
	CHECK(otgCounts.lastEp == ep);
	CHECK(HAL_PCD_EP_GetRxCount(&r.h, (uint8_t)ep) == send);
	CHECK((send == 0 || std::memcmp(r.rxBuf, data, send) == 0));
	CHECK(r.rxBuf[send] == 0xEE);  // nothing written past the received bytes
}
}  // namespace

TEST_CASE_FIXTURE(FlowRig, "PF4 bulk OUT of 1 byte on EP1 reaches the buffer with a count of 1") {
	enumerate();
	flowCheckBulkOut(*this, 1, 1);
	CHECK(faults() == 0);
}
TEST_CASE_FIXTURE(FlowRig, "PF4 bulk OUT of 64 bytes on EP1 is one full packet") {
	enumerate();
	flowCheckBulkOut(*this, 1, 64);
}
TEST_CASE_FIXTURE(FlowRig, "PF4 bulk OUT of 65 bytes on EP1 arrives as a packet of 64 and one of 1") {
	enumerate();
	flowCheckBulkOut(*this, 1, 65);
}
TEST_CASE_FIXTURE(FlowRig, "PF4 bulk OUT of 63 and 2 bytes on EP2 keeps odd tails") {
	enumerate();
	flowCheckBulkOut(*this, 2, 63);
	flowCheckBulkOut(*this, 2, 2);
}
TEST_CASE_FIXTURE(FlowRig, "PF4 bulk OUT of 3 and 130 bytes on EP3") {
	enumerate();
	flowCheckBulkOut(*this, 3, 3);
	flowCheckBulkOut(*this, 3, 130);
}

TEST_CASE_FIXTURE(FlowRig, "PF4 bulk OUT keeps every byte value over 256 bytes") {
	enumerate();
	uint8_t data[256];
	for (unsigned i = 0; i < 256; ++i) data[i] = (uint8_t)i;
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x01, rxBuf, 256) == HAL_OK);
	pushOut(1, data, 256);
	CHECK(otgCounts.dataOut == 1);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x01) == 256);
	CHECK(std::memcmp(rxBuf, data, 256) == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF4 a short packet ends a transfer that expected more data") {
	enumerate();
	uint8_t data[10];
	for (unsigned i = 0; i < 10; ++i) data[i] = flowByte(18, i);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x01, rxBuf, 200) == HAL_OK);
	REQUIRE(hostOtgHostOut(1, data, 10, 0) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 1);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x01) == 10);
}

TEST_CASE_FIXTURE(FlowRig, "PF4 the host OUT token is NAKed while the endpoint is not armed and nothing reaches the callback") {
	enumerate();
	uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	HostOtgHandshake hs = hostOtgHostOut(1, data, 8, 0);
	CHECK(hs == HOSTOTG_NAK);
	// DOEPINT1 (0xB28) NAK flag (bit 13) is set for a bulk endpoint; OTEPDIS (bit 4) applies to control OUT endpoints only (RM0090)
	CHECK((hostRegsPeek(HOST_OTG_BASE + 0xB28u) & (1u << 13)) != 0);
	CHECK((hostRegsPeek(HOST_OTG_BASE + 0xB28u) & 0x10u) == 0);
	drain();
	CHECK(otgCounts.dataOut == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF4 an endpoint receives after a NAKed token once it is armed") {
	enumerate();
	uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	CHECK(hostOtgHostOut(1, data, 8, 0) == HOSTOTG_NAK);
	drain();
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x01, rxBuf, 64) == HAL_OK);
	CHECK(hostOtgHostOut(1, data, 8, 0) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 1);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x01) == 8);
	CHECK(std::memcmp(rxBuf, data, 8) == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF4 the next OUT token is NAKed after a transfer ended until the endpoint is armed again") {
	enumerate();
	uint8_t data[8] = {9, 8, 7, 6, 5, 4, 3, 2};
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x01, rxBuf, 64) == HAL_OK);
	REQUIRE(hostOtgHostOut(1, data, 8, 0) == HOSTOTG_ACK);
	drain();
	CHECK(hostOtgHostOut(1, data, 8, 1) == HOSTOTG_NAK);
	drain();
	CHECK(otgCounts.dataOut == 1);
}

TEST_CASE_FIXTURE(FlowRig, "PF4 two OUT endpoints receiving interleaved packets keep their data apart") {
	enumerate();
	uint8_t a[128], b[128];
	for (unsigned i = 0; i < 128; ++i) { a[i] = flowByte(21, i); b[i] = flowByte(22, i); }
	uint8_t rxB[128];
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x01, rxBuf, 128) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x02, rxB, 128) == HAL_OK);
	REQUIRE(hostOtgHostOut(1, a, 64, 0) == HOSTOTG_ACK);
	drain();
	REQUIRE(hostOtgHostOut(2, b, 64, 0) == HOSTOTG_ACK);
	drain();
	REQUIRE(hostOtgHostOut(1, a + 64, 64, 1) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 1);  // EP1 complete, EP2 still waiting for its second packet
	CHECK(otgCounts.lastEp == 1);
	REQUIRE(hostOtgHostOut(2, b + 64, 64, 1) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 2);
	CHECK(otgCounts.lastEp == 2);
	CHECK(std::memcmp(rxBuf, a, 128) == 0);
	CHECK(std::memcmp(rxB, b, 128) == 0);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x01) == 128);
	CHECK(HAL_PCD_EP_GetRxCount(&h, 0x02) == 128);
	CHECK(faults() == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF4 an OUT endpoint and an IN endpoint of the same number work at the same time") {
	enumerate();
	for (unsigned i = 0; i < 40; ++i) txBuf[i] = flowByte(23, i);
	uint8_t data[40];
	for (unsigned i = 0; i < 40; ++i) data[i] = flowByte(24, i);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x01, rxBuf, 64) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 40) == HAL_OK);
	pushOut(1, data, 40);
	std::vector<uint8_t> got = pullIn(1, 40);
	REQUIRE(got.size() == 40);
	CHECK(std::memcmp(got.data(), txBuf, 40) == 0);
	CHECK(std::memcmp(rxBuf, data, 40) == 0);
	CHECK(otgCounts.dataOut == 1);
	CHECK(otgCounts.dataIn == 1);
}

// ---------------------------------------------------------------- PF5 faults and recovery

TEST_CASE_FIXTURE(FlowRig, "PF5 a USB reset in the middle of a bulk IN transfer flushes the TxFIFOs and EP0 receives a new SETUP") {
	enumerate(0x10);
	for (unsigned i = 0; i < 130; ++i) txBuf[i] = flowByte(25, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 130) == HAL_OK);
	drain();
	REQUIRE(hostOtgTxWordsUsed(1) == 16);
	hostOtgUsbReset();
	HAL_PCD_IRQHandler(&h);  // one call: the reset branch flushes with TXFNUM 0x10 = all FIFOs (RM0090 GRSTCTL TXFNUM 10000)
	CHECK(hostOtgTxWordsUsed(1) == 0);
	drain();
	CHECK(h.State == HAL_PCD_STATE_READY);
	setupStage(kGetDesc);
	CHECK(otgCounts.setup == 1);
	CHECK(otgCounts.setupWords[0] == 0x01000680u);
}

TEST_CASE_FIXTURE(FlowRig, "PF5 characterization: the IN endpoint interrupted by a USB reset stays armed and the HAL refills its TxFIFO, but the reset deactivates the endpoint") {
	// NR348_a: RM0090 does not say what the application must do with an armed IN endpoint at a USB reset; the HAL's reset branch does not disable it
	// (EPENA and DIEPEMPMSK stay), so the next TXFE interrupt writes the following packet of the old transfer into the flushed FIFO.
	enumerate(0x10);
	for (unsigned i = 0; i < 130; ++i) txBuf[i] = flowByte(25, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 130) == HAL_OK);
	drain();
	hostOtgUsbReset();
	drain();
	CHECK(hostOtgTxWordsUsed(1) == 16);
	// RM0090 DIEPCTLx USBAEP: the core clears it for every endpoint but EP0 at the USB reset, so the endpoint no longer answers (model since
	// the Codex review P12C-T8-001): the stale packet sits in the FIFO but the host's IN token is ignored until the endpoint is opened again.
	uint8_t buf[64];
	unsigned n = 0;
	CHECK(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_IGNORED);
	CHECK(n == 0);
	CHECK(hostOtgTxWordsUsed(1) == 16);
}

TEST_CASE_FIXTURE(FlowRig, "PF5 a USB reset sets the device address back to 0") {
	enumerate();
	REQUIRE(HAL_PCD_SetAddress(&h, 23) == HAL_OK);
	CHECK(((hostRegsPeek(HOST_OTG_BASE + 0x800u) >> 4) & 0x7Fu) == 23);
	hostOtgUsbReset();
	drain();
	CHECK(((hostRegsPeek(HOST_OTG_BASE + 0x800u) >> 4) & 0x7Fu) == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF5 a bulk IN stall is seen by the host until the stall is cleared and then the transfer works") {
	enumerate();
	uint8_t buf[64];
	unsigned n = 0;
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x81) == HAL_OK);
	CHECK(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_STALL);
	REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x81) == HAL_OK);
	for (unsigned i = 0; i < 20; ++i) txBuf[i] = flowByte(26, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 20) == HAL_OK);
	std::vector<uint8_t> got = pullIn(1, 20);
	REQUIRE(got.size() == 20);
	CHECK(std::memcmp(got.data(), txBuf, 20) == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF5 a bulk OUT stall is seen by the host until the stall is cleared and then the transfer works") {
	enumerate();
	uint8_t data[16];
	for (unsigned i = 0; i < 16; ++i) data[i] = flowByte(27, i);
	REQUIRE(HAL_PCD_EP_SetStall(&h, 0x02) == HAL_OK);
	CHECK(hostOtgHostOut(2, data, 16, 0) == HOSTOTG_STALL);
	drain();
	CHECK(otgCounts.dataOut == 0);
	REQUIRE(HAL_PCD_EP_ClrStall(&h, 0x02) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x02, rxBuf, 64) == HAL_OK);
	CHECK(hostOtgHostOut(2, data, 16, 0) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 1);
	CHECK(std::memcmp(rxBuf, data, 16) == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF5 suspend and resume between the arming and the host read of an IN transfer keep the data") {
	enumerate();
	for (unsigned i = 0; i < 30; ++i) txBuf[i] = flowByte(28, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 30) == HAL_OK);
	drain();
	hostOtgSuspend();
	drain();
	CHECK(otgCounts.suspend == 1);
	hostOtgResume();
	drain();
	CHECK(otgCounts.resume == 1);
	std::vector<uint8_t> got = pullIn(1, 30);
	REQUIRE(got.size() == 30);
	CHECK(std::memcmp(got.data(), txBuf, 30) == 0);
	CHECK(otgCounts.dataIn == 1);
}

TEST_CASE_FIXTURE(FlowRig, "PF5 suspend then resume callbacks arrive in that order") {
	enumerate();
	hostOtgSuspend();
	drain();
	hostOtgResume();
	drain();
	REQUIRE(otgCounts.orderCount == 2);
	CHECK(otgCounts.order[0] == OTG_CB_SUSPEND);
	CHECK(otgCounts.order[1] == OTG_CB_RESUME);
}

TEST_CASE_FIXTURE(FlowRig, "PF5 aborting an armed IN transfer stops the host from reading it") {
	enumerate();
	for (unsigned i = 0; i < 64; ++i) txBuf[i] = flowByte(29, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 64) == HAL_OK);
	drain();
	REQUIRE(HAL_PCD_EP_Abort(&h, 0x81) == HAL_OK);
	drain();
	uint8_t buf[64];
	unsigned n = 0;
	CHECK(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_NAK);
	CHECK(otgCounts.dataIn == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF5 after Abort and Flush a new IN transfer sends only the new data") {
	enumerate();
	for (unsigned i = 0; i < 64; ++i) txBuf[i] = flowByte(30, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, txBuf, 64) == HAL_OK);
	drain();
	REQUIRE(HAL_PCD_EP_Abort(&h, 0x81) == HAL_OK);
	drain();
	REQUIRE(HAL_PCD_EP_Flush(&h, 0x81) == HAL_OK);
	CHECK(hostOtgTxWordsUsed(1) == 0);
	uint8_t fresh[12] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5};  // V118: USB_WritePacket over-reads up to 3 bytes (5 bytes sent, 8 rounded up, 4 spare)
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x81, fresh, 5) == HAL_OK);
	std::vector<uint8_t> got = pullIn(1, 5);
	REQUIRE(got.size() == 5);
	CHECK(std::memcmp(got.data(), fresh, 5) == 0);
	CHECK(otgCounts.dataIn == 1);
}

TEST_CASE_FIXTURE(FlowRig, "PF5 the host gets no handshake while the device is disconnected and works again after the reconnect") {
	enumerate();
	uint8_t buf[64];
	unsigned n = 0;
	uint8_t data[4] = {1, 2, 3, 4};
	REQUIRE(HAL_PCD_DevDisconnect(&h) == HAL_OK);
	CHECK(hostOtgHostSetup(kGetDesc) == HOSTOTG_IGNORED);
	CHECK(hostOtgHostIn(1, buf, 64, &n) == HOSTOTG_IGNORED);
	CHECK(hostOtgHostOut(1, data, 4, 0) == HOSTOTG_IGNORED);
	REQUIRE(HAL_PCD_DevConnect(&h) == HAL_OK);
	REQUIRE(HAL_PCD_EP_Receive(&h, 0x01, rxBuf, 64) == HAL_OK);
	CHECK(hostOtgHostOut(1, data, 4, 0) == HOSTOTG_ACK);
	drain();
	CHECK(otgCounts.dataOut == 1);
	CHECK(std::memcmp(rxBuf, data, 4) == 0);
}

TEST_CASE_FIXTURE(FlowRig, "PF5 a new enumeration after a reconnect lets a control read run again") {
	enumerate();
	REQUIRE(HAL_PCD_DevDisconnect(&h) == HAL_OK);
	REQUIRE(HAL_PCD_DevConnect(&h) == HAL_OK);
	hostOtgUsbReset();
	drain();
	hostOtgEnumDone();
	drain();
	CHECK(otgCounts.reset == 1);
	setupStage(kGetDesc);
	CHECK(otgCounts.setup == 1);
	for (unsigned i = 0; i < 18; ++i) txBuf[i] = flowByte(31, i);
	REQUIRE(HAL_PCD_EP_Transmit(&h, 0x80, txBuf, 18) == HAL_OK);
	std::vector<uint8_t> got = pullIn(0, 18);
	REQUIRE(got.size() == 18);
	CHECK(std::memcmp(got.data(), txBuf, 18) == 0);
}
