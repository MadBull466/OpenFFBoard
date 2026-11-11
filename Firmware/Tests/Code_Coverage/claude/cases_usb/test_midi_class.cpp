/*
 * MidiClass (P17, group MCD): the descriptors of the MIDI configuration, the TinyUSB MIDI class driver's open / mount /
 * reset / control request / FIFO behaviour, the packet API and the bulk transfers, and the end to end path from the
 * simulated host to a MidiProbe and to MidiMain.
 *
 * Requirements (Tests/Code_Coverage/claude/charters/driver/MIDI_REQUIREMENTS.md): R7.1-R7.6 (descriptors of usb_cdc_midi_conf), R8.1-R8.7
 * (class driver: open, mount, bus reset, class and standard requests, refusal of a wrong layout, RX FIFO full), R11.1-R11.5
 * (tud_midi_packet_write, the TX FIFO, the zero length packet, writes before the mount, well formed packets),
 * R12.1-R12.3 (host -> handler / MidiMain, a burst of 16 packets).
 *
 * Oracles: UM = USB class definition for MIDI devices 1.0 (midi10.pdf) Tables 4-1, 4-2, 6-1, 6-2, 6-3, 6-4, 6-6, 6-7, Appendix
 * B (B-3 to B-6), section 7 (class requests); MS = MIDI 1.0 detailed specification 4.2.1; HD = hand derivation in a comment;
 * IF = the TinyUSB application API (midi_device.h); FW = the firmware / class driver as it is (the case is named
 * `characterization:`). Expected descriptor values are the numbers of the tables above, written as literals (never taken from
 * the firmware's macros).
 *
 * Capacities (class driver, full speed, tusb_config.h): TX FIFO 64 bytes, RX FIFO 64 bytes, endpoint buffers 64 bytes. A packet
 * written with tud_midi_packet_write is flushed at once: the first packet leaves the FIFO and sits in the endpoint buffer
 * (the controller transfer is armed), every later packet stays in the FIFO until the previous IN transfer has completed.
 * With the host not reading the TX capacity is therefore 1 packet (endpoint) + 16 packets (FIFO, 64 / 4) = 17 packets. The
 * RX endpoint is armed only if the RX FIFO has 64 free bytes (midi_device.c _prep_out_transaction), i.e. only when it is empty.
 *
 * Host versus target: no type of this file differs between host and target; the class driver is plain C.
 * Characterizations (NEEDS REVIEW): the layout refusal cases of R8.6, the RX FIFO full behaviour of R8.7 (the endpoint is
 * armed only while the FIFO is empty), the zero length packet rule of R11.3 (USB 2.0 bulk rule, not in the supplied documents),
 * the split of several packet writes into transfers (R11.1), the wTotalLength 0x0041 of the MS header (R7.2, question 61).
 * KNOWN_ISSUEs: none.
 */
#include "doctest.h"
#include "MidiFixture.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace {

using Handshake = HostDcd::Handshake;
using Packet = std::array<uint8_t, 4>;

// Descriptor constants of the USB class definition for MIDI devices 1.0 (written as numbers on purpose)
constexpr uint8_t DT_INTERFACE = 0x04, DT_ENDPOINT = 0x05, DT_CS_INTERFACE = 0x24, DT_CS_ENDPOINT = 0x25;
constexpr uint8_t AC_HEADER = 0x01;			  // audio control header subtype (Appendix B-3)
constexpr uint8_t MS_HEADER = 0x01, MIDI_IN_JACK = 0x02, MIDI_OUT_JACK = 0x03; // Tables 6-2, 6-3, 6-4
constexpr uint8_t JACK_EMBEDDED = 0x01, JACK_EXTERNAL = 0x02;				 // Table 6-3
constexpr uint8_t MS_GENERAL = 0x01;										 // Table 6-7

uint16_t totalLengthOf(const uint8_t *conf) { return static_cast<uint16_t>(conf[2] | conf[3] << 8); }

/** One descriptor of a configuration descriptor */
struct Desc {
	size_t off = 0;
	std::vector<uint8_t> b;
	uint8_t len() const { return b[0]; }
	uint8_t type() const { return b[1]; }
	uint8_t sub() const { return b.size() > 2 ? b[2] : 0; }
	uint16_t u16(size_t i) const { return static_cast<uint16_t>(b[i] | b[i + 1] << 8); }
};

/** Splits a configuration descriptor into descriptors; `exact` is false if a length is 0 / overruns or bytes are left over */
std::vector<Desc> walk(const uint8_t *conf, size_t total, bool *exact = nullptr) {
	std::vector<Desc> out;
	size_t pos = 0;
	bool ok = true;
	while (pos < total) {
		const uint8_t len = conf[pos];
		if (len < 2 || pos + len > total) {
			ok = false;
			break;
		}
		Desc d;
		d.off = pos;
		d.b.assign(conf + pos, conf + pos + len);
		out.push_back(d);
		pos += len;
	}
	if (exact) {
		*exact = ok && pos == total;
	}
	return out;
}

std::vector<Desc> midiConfDescriptors() { return walk(usb_cdc_midi_conf, totalLengthOf(usb_cdc_midi_conf)); }

/** The descriptors of the MIDI part, found by what they are (not by position) */
struct Layout {
	std::vector<Desc> all;
	const Desc *acItf = nullptr, *acHeader = nullptr, *msItf = nullptr, *msHeader = nullptr;
	const Desc *inEmb = nullptr, *inExt = nullptr, *outEmb = nullptr, *outExt = nullptr;
	const Desc *epOut = nullptr, *epIn = nullptr, *csOut = nullptr, *csIn = nullptr;
};

const Desc *findOne(const std::vector<Desc> &all, uint8_t type, uint8_t sub, int jackType = -1, int epDir = -1) {
	const Desc *found = nullptr;
	int n = 0;
	for (const Desc &d : all) {
		if (d.type() != type) {
			continue;
		}
		if (type != DT_ENDPOINT && d.sub() != sub) {
			continue;
		}
		if (jackType >= 0 && d.b[3] != jackType) {
			continue;
		}
		if (epDir >= 0 && ((d.b[2] & 0x80) != 0) != (epDir == 1)) {
			continue;
		}
		found = &d;
		n++;
	}
	INFO("descriptor type " << int(type) << " subtype " << int(sub) << " found " << n << " times");
	REQUIRE(n == 1);
	return found;
}

Layout layout() {
	Layout l;
	l.all = midiConfDescriptors();
	const std::vector<Desc> &a = l.all;
	// The interface descriptors: the AudioControl one is the first of class 1 (AUDIO)
	const Desc *itf0 = nullptr, *itf1 = nullptr;
	for (size_t i = 0; i < a.size(); i++) {
		if (a[i].type() == DT_INTERFACE && a[i].b[5] == 1) {
			if (itf0 == nullptr) {
				itf0 = &a[i];
			} else if (itf1 == nullptr) {
				itf1 = &a[i];
			}
		}
	}
	REQUIRE(itf0 != nullptr);
	REQUIRE(itf1 != nullptr);
	l.acItf = itf0;
	l.msItf = itf1;
	// The CDC functional descriptors are CS_INTERFACE descriptors too: search only after the AudioControl interface
	std::vector<Desc> acPart, msPart;
	for (const Desc &d : a) {
		if (d.off > l.acItf->off && d.off < l.msItf->off) {
			acPart.push_back(d);
		} else if (d.off > l.msItf->off) {
			msPart.push_back(d);
		}
	}
	auto inAll = [&](const Desc *d) { return &a[static_cast<size_t>(std::find_if(a.begin(), a.end(), [&](const Desc &x) { return x.off == d->off; }) - a.begin())]; };
	l.acHeader = inAll(findOne(acPart, DT_CS_INTERFACE, AC_HEADER));
	l.msHeader = inAll(findOne(msPart, DT_CS_INTERFACE, MS_HEADER));
	l.inEmb = inAll(findOne(msPart, DT_CS_INTERFACE, MIDI_IN_JACK, JACK_EMBEDDED));
	l.inExt = inAll(findOne(msPart, DT_CS_INTERFACE, MIDI_IN_JACK, JACK_EXTERNAL));
	l.outEmb = inAll(findOne(msPart, DT_CS_INTERFACE, MIDI_OUT_JACK, JACK_EMBEDDED));
	l.outExt = inAll(findOne(msPart, DT_CS_INTERFACE, MIDI_OUT_JACK, JACK_EXTERNAL));
	// The CDC endpoints are also endpoints: the MIDI ones are the endpoints after the MIDIStreaming interface descriptor
	for (size_t i = 0; i < a.size(); i++) {
		if (a[i].off > l.msItf->off && a[i].type() == DT_ENDPOINT) {
			if (a[i].b[2] & 0x80) {
				l.epIn = &a[i];
			} else {
				l.epOut = &a[i];
			}
		}
		if (a[i].off > l.msItf->off && a[i].type() == DT_CS_ENDPOINT && i > 0) {
			if (a[i - 1].type() == DT_ENDPOINT && (a[i - 1].b[2] & 0x80)) {
				l.csIn = &a[i];
			} else if (a[i - 1].type() == DT_ENDPOINT) {
				l.csOut = &a[i];
			}
		}
	}
	REQUIRE(l.epOut != nullptr);
	REQUIRE(l.epIn != nullptr);
	REQUIRE(l.csOut != nullptr);
	REQUIRE(l.csIn != nullptr);
	return l;
}

/** The MIDI configuration with one byte of the AudioControl (+ 0) or MIDIStreaming (+ 18) interface descriptor changed; stable storage */
const uint8_t *variantConf(bool streamingInterface, size_t fieldOffset, uint8_t value) {
	static auto *store = new std::map<std::pair<size_t, uint8_t>, std::vector<uint8_t>>();
	const std::vector<Desc> all = midiConfDescriptors();
	size_t acOff = 0;
	for (const Desc &d : all) {
		if (d.type() == DT_INTERFACE && d.b[5] == 1) {
			acOff = d.off;
			break;
		}
	}
	const size_t at = acOff + (streamingInterface ? 18 : 0) + fieldOffset;
	std::vector<uint8_t> &v = (*store)[{at, value}];
	if (v.empty()) {
		v.assign(usb_cdc_midi_conf, usb_cdc_midi_conf + totalLengthOf(usb_cdc_midi_conf));
		v[at] = value;
	}
	return v.data();
}

Packet note(uint8_t i) { return MidiFixture::pkt(0, 0x9, 0x90, i, 0x40); }

} // namespace

TEST_SUITE("MidiClass") {

// =============================================================== R7: descriptors (UM section 6, Appendix B)

TEST_CASE_FIXTURE(MidiFixture, "[R7.1 descriptors] the AudioControl interface descriptor has interface 2, no endpoint and class AUDIO / subclass CONTROL / protocol 0") {
	const Layout l = layout();
	// UM Appendix B-3 (standard AC interface descriptor): length 9, interface descriptor, number 2
	// (CDC uses 0 and 1), alternate 0, no endpoint, class 1 (AUDIO), subclass 1 (AUDIOCONTROL), protocol 0
	REQUIRE(l.acItf->len() == 9);
	CHECK(l.acItf->b[1] == 0x04);
	CHECK(l.acItf->b[2] == 2);
	CHECK(l.acItf->b[3] == 0);
	CHECK(l.acItf->b[4] == 0);
	CHECK(l.acItf->b[5] == 1);
	CHECK(l.acItf->b[6] == 1);
	CHECK(l.acItf->b[7] == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.1 descriptors] the AudioControl header is 9 bytes with bcdADC 0x0100, wTotalLength 9 and one streaming interface number 3") {
	const Layout l = layout();
	// UM Appendix B-3: bLength 9, CS_INTERFACE, HEADER, bcdADC 0x0100, wTotalLength 0x0009 (the header alone), bInCollection 1,
	// baInterfaceNr(1) = the MIDIStreaming interface
	REQUIRE(l.acHeader->len() == 9);
	CHECK(l.acHeader->b[1] == 0x24);
	CHECK(l.acHeader->b[2] == 0x01);
	CHECK(l.acHeader->u16(3) == 0x0100);
	CHECK(l.acHeader->u16(5) == 0x0009);
	CHECK(l.acHeader->b[7] == 1);
	CHECK(l.acHeader->b[8] == 3);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.1 descriptors] the MIDIStreaming interface descriptor has interface 3, two endpoints and class AUDIO / subclass MIDISTREAMING / protocol 0") {
	const Layout l = layout();
	// UM Table 6-1: bLength 9, interface descriptor, bInterfaceNumber 3, alternate 0, bNumEndpoints 2 (bulk OUT and bulk IN),
	// class 1, subclass 3 (MIDISTREAMING), protocol 0
	REQUIRE(l.msItf->len() == 9);
	CHECK(l.msItf->b[1] == 0x04);
	CHECK(l.msItf->b[2] == 3);
	CHECK(l.msItf->b[3] == 0);
	CHECK(l.msItf->b[4] == 2);
	CHECK(l.msItf->b[5] == 1);
	CHECK(l.msItf->b[6] == 3);
	CHECK(l.msItf->b[7] == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.1 descriptors] the class specific MIDIStreaming header is 7 bytes with MS_HEADER and bcdMSC 0x0100") {
	const Layout l = layout();
	// UM Table 6-2: bLength 7, CS_INTERFACE (0x24), MS_HEADER (0x01), bcdMSC 0x0100, wTotalLength
	REQUIRE(l.msHeader->len() == 7);
	CHECK(l.msHeader->b[1] == 0x24);
	CHECK(l.msHeader->b[2] == 0x01);
	CHECK(l.msHeader->u16(3) == 0x0100);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.1 descriptors] the AudioControl interface directly follows the two CDC interfaces and the MIDIStreaming interface follows the AudioControl header") {
	const std::vector<Desc> all = midiConfDescriptors();
	std::vector<uint8_t> interfaceNumbers, interfaceClasses;
	for (const Desc &d : all) {
		if (d.type() == DT_INTERFACE) {
			interfaceNumbers.push_back(d.b[2]);
			interfaceClasses.push_back(d.b[5]);
		}
	}
	// CDC control (class 2) 0, CDC data (class 0x0A) 1, AudioControl (class 1) 2, MIDIStreaming (class 1) 3
	CHECK(interfaceNumbers == std::vector<uint8_t>{0, 1, 2, 3});
	CHECK(interfaceClasses == std::vector<uint8_t>{0x02, 0x0A, 0x01, 0x01});
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.2 descriptors] the MIDIStreaming header wTotalLength is 0x0041 as in the worked example of Appendix B (Table B-6)" ) {
	// NEEDS REVIEW (question 61): the wording of Table 6-2 ("the header and all jack and element descriptors") would give 37
	// (7 + 30), the worked example Table B-6 and TinyUSB include the two endpoint blocks: 7 + 30 + 2 * 14 = 65 = 0x41.
	const Layout l = layout();
	CHECK(l.msHeader->u16(5) == 0x0041);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.2 descriptors] the MIDIStreaming header wTotalLength equals the size of the descriptors from the header to the end of the configuration") {
	// HD: header 7 + 4 jacks (6 + 6 + 9 + 9 = 30) + two endpoint blocks (9 + 5 each = 28) = 65; the byte count of the
	// walked descriptors is compared with the field, so the field cannot drift away from the content (Appendix B-6)
	const Layout l = layout();
	size_t sum = 0;
	for (const Desc &d : l.all) {
		if (d.off >= l.msHeader->off) {
			sum += d.len();
		}
	}
	CHECK(sum == 65);
	CHECK(l.msHeader->u16(5) == sum);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.3 descriptors] the embedded MIDI IN jack has id 1, length 6 and string index 0") {
	const Layout l = layout();
	// UM Table 6-3: bLength 6, CS_INTERFACE, MIDI_IN_JACK 0x02, bJackType EMBEDDED 0x01, bJackID, iJack
	REQUIRE(l.inEmb->len() == 6);
	CHECK(l.inEmb->b[1] == 0x24);
	CHECK(l.inEmb->b[3] == JACK_EMBEDDED);
	CHECK(l.inEmb->b[4] == 1);
	CHECK(l.inEmb->b[5] == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.3 descriptors] the external MIDI IN jack has id 2, length 6 and string index 0") {
	const Layout l = layout();
	REQUIRE(l.inExt->len() == 6); // UM Table 6-3
	CHECK(l.inExt->b[1] == 0x24);
	CHECK(l.inExt->b[3] == JACK_EXTERNAL);
	CHECK(l.inExt->b[4] == 2);
	CHECK(l.inExt->b[5] == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.3 descriptors] the embedded MIDI OUT jack has id 3 and one input pin fed by jack 2 pin 1") {
	const Layout l = layout();
	// UM Appendix B Tables B-7 to B-10 (jack descriptors of the worked example, p = 1 input pin): the OUT jack is 9 bytes
	// (Table 6-4 gives 6 + 2 p and puts iJack at 5 + 2 p, which disagree for p = 1; the worked example decides): CS_INTERFACE, MIDI_OUT_JACK, bJackType, bJackID, bNrInputPins,
	// baSourceID(1), baSourcePin(1), iJack
	REQUIRE(l.outEmb->len() == 9);
	CHECK(l.outEmb->b[1] == 0x24);
	CHECK(l.outEmb->b[3] == JACK_EMBEDDED);
	CHECK(l.outEmb->b[4] == 3);
	CHECK(l.outEmb->b[5] == 1);
	CHECK(l.outEmb->b[6] == 2);
	CHECK(l.outEmb->b[7] == 1);
	CHECK(l.outEmb->b[8] == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.3 descriptors] the external MIDI OUT jack has id 4 and one input pin fed by jack 1 pin 1") {
	const Layout l = layout();
	REQUIRE(l.outExt->len() == 9); // UM Table 6-4
	CHECK(l.outExt->b[1] == 0x24);
	CHECK(l.outExt->b[3] == JACK_EXTERNAL);
	CHECK(l.outExt->b[4] == 4);
	CHECK(l.outExt->b[5] == 1);
	CHECK(l.outExt->b[6] == 1);
	CHECK(l.outExt->b[7] == 1);
	CHECK(l.outExt->b[8] == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.3 descriptors] the jack ids are unique and not zero and every source and associated jack id names an existing jack") {
	const Layout l = layout();
	const std::vector<uint8_t> ids = {l.inEmb->b[4], l.inExt->b[4], l.outEmb->b[4], l.outExt->b[4]};
	for (size_t i = 0; i < ids.size(); i++) {
		CHECK(ids[i] != 0); // UM: bJackID is a non zero constant
		for (size_t j = i + 1; j < ids.size(); j++) {
			CHECK(ids[i] != ids[j]);
		}
	}
	auto exists = [&](uint8_t id) { return std::find(ids.begin(), ids.end(), id) != ids.end(); };
	CHECK(exists(l.outEmb->b[6]));
	CHECK(exists(l.outExt->b[6]));
	CHECK(exists(l.csOut->b[4]));
	CHECK(exists(l.csIn->b[4]));
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.4 descriptors] the OUT endpoint descriptor is bulk 0x02 of 64 bytes with 9 byte audio layout") {
	const Layout l = layout();
	// UM Table 6-6: bLength 9, ENDPOINT, address 0x02 (OUT), bmAttributes 0x02 (bulk, no synchronisation), wMaxPacketSize 64,
	// bInterval 0, bRefresh 0, bSynchAddress 0
	REQUIRE(l.epOut->len() == 9);
	CHECK(l.epOut->b[1] == 0x05);
	CHECK(l.epOut->b[2] == 0x02);
	CHECK(l.epOut->b[3] == 0x02);
	CHECK(l.epOut->u16(4) == 64);
	CHECK(l.epOut->b[6] == 0);
	CHECK(l.epOut->b[7] == 0);
	CHECK(l.epOut->b[8] == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.4 descriptors] the IN endpoint descriptor is bulk 0x83 of 64 bytes with 9 byte audio layout") {
	const Layout l = layout();
	REQUIRE(l.epIn->len() == 9); // UM Table 6-6
	CHECK(l.epIn->b[1] == 0x05);
	CHECK(l.epIn->b[2] == 0x83);
	CHECK(l.epIn->b[3] == 0x02);
	CHECK(l.epIn->u16(4) == 64);
	CHECK(l.epIn->b[6] == 0);
	CHECK(l.epIn->b[7] == 0);
	CHECK(l.epIn->b[8] == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.4 descriptors] the OUT endpoint is followed by its class specific descriptor naming the embedded IN jack 1") {
	const Layout l = layout();
	// UM Table 6-7: bLength 4 + n = 5, CS_ENDPOINT (0x25), MS_GENERAL (0x01), bNumEmbMIDIJack 1, baAssocJackID(1) = jack id
	// of the embedded MIDI IN jack (data received on the OUT endpoint enters the device through it)
	REQUIRE(l.csOut->len() == 5);
	CHECK(l.csOut->off == l.epOut->off + 9);
	CHECK(l.csOut->b[1] == 0x25);
	CHECK(l.csOut->b[2] == MS_GENERAL);
	CHECK(l.csOut->b[3] == 1);
	CHECK(l.csOut->b[4] == l.inEmb->b[4]);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.4 descriptors] the IN endpoint is followed by its class specific descriptor naming the embedded OUT jack 3") {
	const Layout l = layout();
	REQUIRE(l.csIn->len() == 5); // UM Table 6-7
	CHECK(l.csIn->off == l.epIn->off + 9);
	CHECK(l.csIn->b[1] == 0x25);
	CHECK(l.csIn->b[2] == MS_GENERAL);
	CHECK(l.csIn->b[3] == 1);
	CHECK(l.csIn->b[4] == l.outEmb->b[4]);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.4 descriptors] the OUT endpoint comes before the IN endpoint inside the MIDIStreaming interface") {
	const Layout l = layout();
	CHECK(l.epOut->off < l.epIn->off);
	CHECK(l.epOut->off > l.msHeader->off);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.5 descriptors] the configuration descriptor wTotalLength equals the sum of all descriptor lengths with no byte left over") {
	bool exact = false;
	const uint16_t total = totalLengthOf(usb_cdc_midi_conf);
	const std::vector<Desc> all = walk(usb_cdc_midi_conf, total, &exact);
	CHECK(exact);
	size_t sum = 0;
	for (const Desc &d : all) {
		sum += d.len();
	}
	CHECK(sum == total);
	CHECK(all[0].b[1] == 0x02);	  // the first descriptor is the configuration descriptor
	CHECK(all[0].len() == 9);
	// HD: configuration 9 + CDC 66 + MIDI 92 (head 34 + jacks 30 + two endpoint blocks 2 x 14) = 167
	CHECK(total == 167);
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.5 descriptors] the configuration descriptor declares 4 interfaces and the interface numbers 0 to 3 are consecutive and unused twice") {
	CHECK(usb_cdc_midi_conf[4] == 4); // bNumInterfaces
	std::vector<uint8_t> numbers;
	for (const Desc &d : midiConfDescriptors()) {
		if (d.type() == DT_INTERFACE) {
			numbers.push_back(d.b[2]);
		}
	}
	REQUIRE(numbers.size() == 4);
	for (size_t i = 0; i < numbers.size(); i++) {
		CHECK(numbers[i] == i);
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.5 descriptors] the string index 6 of the AudioControl interface resolves to a text that starts with FFBoard MIDI through a string descriptor request") {
	startAndEnumerate();
	const Layout l = layout();
	CHECK(l.acItf->b[8] == 6); // iInterface
	// USBdevice (appendSerial 4, as FFBWheel) appends "(" + the first 4 digits of the chip id + ")" to every string
	const std::string text = host.getString(6);
	CHECK(text.rfind("FFBoard MIDI", 0) == 0);
	CHECK(text.size() > std::string("FFBoard MIDI").size());
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.5 descriptors] the device returns the configuration descriptor byte for byte") {
	startAndEnumerate();
	const uint16_t total = totalLengthOf(usb_cdc_midi_conf);
	REQUIRE(host.configDescriptor.size() == total);
	CHECK(std::equal(host.configDescriptor.begin(), host.configDescriptor.end(), usb_cdc_midi_conf));
}

TEST_CASE_FIXTURE(MidiFixture, "[R7.6 descriptors] the class driver walks the whole configuration: the device is configured and both MIDI interfaces belong to a driver") {
	startAndEnumerate();
	// A wrong length returned by midid_open would leave a class specific descriptor where usbd.c expects an interface
	// descriptor (usbd.c process_set_config) and SET_CONFIGURATION would stall. Interfaces without a driver stall GET_INTERFACE.
	CHECK(tud_mounted());
	CHECK(tud_midi_mounted());
	for (uint16_t itf : {2, 3}) {
		INFO("interface " << itf);
		const UsbHost::ControlResult r = host.control(UsbHost::request(0x81, TUSB_REQ_GET_INTERFACE, 0, itf, 1));
		REQUIRE(r.ok());
		CHECK(r.data == std::vector<uint8_t>{0});
	}
	CHECK(HostDcd::endpoint(0x02).enabled);
	CHECK(HostDcd::endpoint(0x83).enabled);
	CHECK(HostDcd::endpoint(0x83).maxPacket == 64);
	CHECK(HostDcd::endpoint(0x02).maxPacket == 64);
}

// =============================================================== R8: class driver

TEST_CASE_FIXTURE(MidiFixture, "[R8.1 class] the MIDI interface is not mounted before the configuration is set") {
	startMidi();
	CHECK_FALSE(tud_midi_mounted());
	REQUIRE(host.enumerate(5, false)); // reset, address, descriptors, no SET_CONFIGURATION
	process();
	CHECK_FALSE(tud_midi_mounted());
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.1 class] the MIDI interface is mounted after SET_CONFIGURATION") {
	startMidi();
	REQUIRE(host.enumerate(5, false));
	REQUIRE(host.setConfiguration(1).ok());
	process();
	CHECK(tud_midi_mounted());
	CHECK(tud_mounted());
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.1 class] a configuration whose MIDIStreaming interface declares only one endpoint does not mount the MIDI interface") {
	// Test data: bNumEndpoints (byte 4 of the MIDIStreaming interface descriptor) changed from 2 to 1: at most the OUT endpoint is opened
	startDevice(variantConf(true, 4, 1));
	REQUIRE(host.enumerate(5, false));
	host.setConfiguration(1); // usbd may or may not accept this configuration: only the mount state is checked
	process();
	CHECK_FALSE(tud_midi_mounted()); // mounted = both endpoints open (tud_midi_n_mounted)
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.2 class] after the configuration the OUT endpoint 0x02 is armed and acknowledges a transfer") {
	startAndEnumerate();
	CHECK(HostDcd::endpoint(0x02).armed); // armed by midid_open (_prep_out_transaction)
	CHECK(sendPackets({noteOnPkt(0, 60, 100)}) == Handshake::Ack);
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.2 class] the IN endpoint 0x83 answers NAK while the device has nothing to send") {
	startAndEnumerate();
	std::vector<uint8_t> data;
	CHECK(HostDcd::in(host.address, 3, data) == Handshake::Nak);
	CHECK_FALSE(HostDcd::endpoint(0x83).armed);
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.3 class] a bus reset unmounts the MIDI interface") {
	startAndEnumerate();
	REQUIRE(tud_midi_mounted());
	host.busReset();
	CHECK_FALSE(tud_midi_mounted());
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.3 class] a packet still queued for the host is gone after a bus reset and a new enumeration") {
	startAndEnumerate();
	// Two writes: the first packet is handed to the controller at once (endpoint buffer), the second stays in the TX FIFO
	REQUIRE(tud_midi_packet_write(note(1).data()));
	REQUIRE(tud_midi_packet_write(note(2).data()));
	host.busReset();
	enumerate();
	Handshake status = Handshake::Ack;
	size_t rest = 0;
	const auto got = receivePackets(&status, &rest);
	CHECK(got.empty());
	CHECK(rest == 0);
	CHECK(status == Handshake::Nak);
	// A new packet written after the re-enumeration is the only thing the host gets (no stale packet queued before the reset)
	REQUIRE(tud_midi_packet_write(note(9).data()));
	process();
	size_t rest2 = 9;
	const auto fresh = receivePackets(&status, &rest2);
	REQUIRE(fresh.size() == 1);
	CHECK(rest2 == 0);
	CHECK(fresh[0] == note(9));
	process();
	CHECK(receivePackets(&status).empty());
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.3 class] received bytes and a half read event are gone after a bus reset and a new enumeration") {
	startAndEnumerate();
	// No handler: the packet stays in the RX FIFO. One byte read leaves the event half read (stream_read state)
	// Two packets (8 bytes): reading one byte takes the first packet out of the FIFO, 4 bytes stay in the RX FIFO
	REQUIRE(sendPackets({noteOnPkt(0, 60, 100), noteOnPkt(0, 61, 100)}) == Handshake::Ack);
	uint8_t one = 0;
	REQUIRE(tud_midi_stream_read(&one, 1) == 1);
	CHECK(one == 0x90);
	REQUIRE(tud_midi_available() == 2 + 4); // the two remaining bytes of the event + the second packet in the FIFO
	host.busReset();
	enumerate();
	CHECK(tud_midi_available() == 0);
	uint8_t p[4];
	CHECK_FALSE(tud_midi_packet_read(p));
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.3 class] a half written event is gone after a bus reset and a new enumeration") {
	startAndEnumerate();
	const uint8_t start[2] = {0x90, 0x3C};
	REQUIRE(tud_midi_stream_write(0, start, 2) == 2); // no packet yet: the event is incomplete
	host.busReset();
	enumerate();
	const uint8_t rest[1] = {0x7F};
	REQUIRE(tud_midi_stream_write(0, rest, 1) == 1);
	process();
	const auto got = receivePackets();
	REQUIRE(got.size() == 1);
	// The stream state is cleared: 0x7F is a new event, not the third byte of 90 3C (09 90 3C 7F)
	CHECK(got[0][1] == 0x7F);
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.3 class] after a bus reset and a new enumeration the interface is mounted and passes data in both directions") {
	startAndEnumerate();
	host.busReset();
	enumerate();
	CHECK(tud_midi_mounted());
	MidiProbe probe;
	CHECK(sendPackets({noteOnPkt(1, 61, 90)}) == Handshake::Ack);
	REQUIRE(probe.hooks.size() == 1);
	CHECK(probe.hooks[0].a == 61);
	REQUIRE(tud_midi_packet_write(note(7).data()));
	process();
	const auto got = receivePackets();
	REQUIRE(got.size() == 1);
	CHECK(got[0] == note(7));
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.4 class] class requests to the MIDIStreaming interface stall the control pipe") {
	startAndEnumerate();
	// UM 7.2: the class specific requests are optional, an unsupported request stalls. SET_CUR 0x01, GET_CUR 0x81, GET_STAT 0xFF
	CHECK(host.control(UsbHost::request(0x21, 0x01, 0, 3, 0)).stalled());
	CHECK(host.control(UsbHost::request(0xA1, 0x81, 0, 3, 1)).stalled());
	CHECK(host.control(UsbHost::request(0xA1, 0xFF, 0, 3, 1)).stalled());
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.4 class] class requests to the AudioControl interface stall the control pipe") {
	startAndEnumerate();
	CHECK(host.control(UsbHost::request(0x21, 0x01, 0, 2, 0)).stalled());
	CHECK(host.control(UsbHost::request(0xA1, 0x81, 0, 2, 1)).stalled());
	CHECK(host.control(UsbHost::request(0xA1, 0xFF, 0, 2, 1)).stalled());
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.4 class] class requests to the bulk endpoints 0x02 and 0x83 stall the control pipe") {
	startAndEnumerate();
	for (uint16_t ep : {0x02, 0x83}) {
		INFO("endpoint " << ep);
		CHECK(host.control(UsbHost::request(0x22, 0x01, 0, ep, 0)).stalled());
		CHECK(host.control(UsbHost::request(0xA2, 0x81, 0, ep, 1)).stalled());
		CHECK(host.control(UsbHost::request(0xA2, 0xFF, 0, ep, 1)).stalled());
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.4 class] after stalled class requests the device is still configured and the MIDI interface still works") {
	startAndEnumerate();
	REQUIRE(host.control(UsbHost::request(0x21, 0x01, 0, 3, 0)).stalled());
	REQUIRE(host.control(UsbHost::request(0xA2, 0x81, 0, 0x83, 1)).stalled());
	CHECK(tud_mounted());
	CHECK(tud_midi_mounted());
	CHECK(host.getConfiguration().data == std::vector<uint8_t>{1});
	MidiProbe probe;
	CHECK(sendPackets({noteOnPkt(0, 60, 100)}) == Handshake::Ack);
	CHECK(probe.hooks.size() == 1);
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.5 class] GET_INTERFACE on the AudioControl and MIDIStreaming interfaces answers alternate setting 0") {
	startAndEnumerate();
	for (uint16_t itf : {2, 3}) {
		INFO("interface " << itf);
		const UsbHost::ControlResult r = host.control(UsbHost::request(0x81, TUSB_REQ_GET_INTERFACE, 0, itf, 1));
		REQUIRE(r.ok());
		CHECK(r.data == std::vector<uint8_t>{0});
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.5 class] SET_INTERFACE with alternate setting 0 on the MIDI interfaces is acknowledged and the interface stays mounted") {
	startAndEnumerate();
	for (uint16_t itf : {2, 3}) {
		INFO("interface " << itf);
		CHECK(host.control(UsbHost::request(0x01, TUSB_REQ_SET_INTERFACE, 0, itf, 0)).ok());
	}
	CHECK(tud_midi_mounted());
}

namespace {
/** Enumerates with a changed configuration and checks that the configuration is refused and no MIDI endpoint was opened */
void expectRefused(MidiFixture &f, const uint8_t *conf) {
	f.startDevice(conf);
	REQUIRE(f.host.enumerate(5, false));
	CHECK(f.host.setConfiguration(1).stalled());
	f.process();
	CHECK_FALSE(tud_mounted());
	CHECK_FALSE(tud_midi_mounted());
	CHECK_FALSE(HostDcd::endpoint(0x02).enabled);
	CHECK_FALSE(HostDcd::endpoint(0x83).enabled);
}
} // namespace

TEST_CASE_FIXTURE(MidiFixture, "[R8.6 class] characterization: a first interface that is not AudioControl (subclass STREAMING) is refused and nothing is opened") {
	// NEEDS REVIEW: FW (midid_open TU_VERIFY of class / subclass / protocol); test data = copy of the configuration with byte 6
	// of the first AUDIO interface descriptor changed to 2 (AUDIOSTREAMING)
	expectRefused(*this, variantConf(false, 6, 2));
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.6 class] characterization: a first interface with a protocol other than 0 is refused and nothing is opened") {
	expectRefused(*this, variantConf(false, 7, 0x20));
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.6 class] characterization: a first interface of a vendor class is refused and nothing is opened") {
	// no class driver claims class 0xFF in this build (the refusal is made by usbd.c, not by midid_open)
	expectRefused(*this, variantConf(false, 5, 0xFF));
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.6 class] characterization: a second interface that is not MIDIStreaming (subclass CONTROL) is refused and no endpoint is opened") {
	expectRefused(*this, variantConf(true, 6, 1));
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.6 class] characterization: a second interface with a protocol other than 0 is refused and no endpoint is opened") {
	expectRefused(*this, variantConf(true, 7, 0x20));
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.6 class] characterization: a second interface of a vendor class is refused and no endpoint is opened") {
	expectRefused(*this, variantConf(true, 5, 0xFF));
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.7 class] characterization: with a full RX FIFO nobody reads the endpoint NAKs the next OUT transfer") {
	startAndEnumerate();
	// No handler: tud_midi_rx_cb returns without reading. HD: _prep_out_transaction arms the OUT endpoint only if the RX FIFO
	// has at least 64 free bytes (the endpoint buffer size); the FIFO holds 64 bytes, so it must be completely empty. After the
	// first transfer (4 bytes) 60 bytes are free: the endpoint is not armed again. One transfer is acknowledged, the second NAKs.
	CHECK(sendPackets({noteOnPkt(0, 60, 100)}) == Handshake::Ack);
	CHECK(sendPackets({noteOnPkt(0, 61, 100)}) == Handshake::Nak);
	CHECK_FALSE(HostDcd::endpoint(0x02).armed);
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.7 class] characterization: reading the whole RX FIFO arms the OUT endpoint again") {
	startAndEnumerate();
	REQUIRE(sendPackets({noteOnPkt(0, 60, 100)}) == Handshake::Ack);
	REQUIRE(sendPackets({noteOnPkt(0, 61, 100)}) == Handshake::Nak);
	uint8_t p[4];
	REQUIRE(tud_midi_packet_read(p)); // FIFO empty again: 64 free bytes
	CHECK(p[1] == 0x90);
	CHECK(p[2] == 60);
	CHECK(HostDcd::endpoint(0x02).armed);
	CHECK(sendPackets({noteOnPkt(0, 62, 100)}) == Handshake::Ack);
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.7 class] characterization: reading only part of the RX FIFO does not arm the OUT endpoint") {
	startAndEnumerate();
	REQUIRE(sendPackets({noteOnPkt(0, 60, 100), noteOnPkt(0, 61, 100)}) == Handshake::Ack);
	uint8_t p[4];
	REQUIRE(tud_midi_packet_read(p)); // 4 of 8 bytes left: 60 free bytes < 64
	CHECK_FALSE(HostDcd::endpoint(0x02).armed);
	CHECK(sendPackets({noteOnPkt(0, 62, 100)}) == Handshake::Nak);
	REQUIRE(tud_midi_packet_read(p)); // FIFO empty
	CHECK(p[2] == 61);
	CHECK(sendPackets({noteOnPkt(0, 62, 100)}) == Handshake::Ack);
}

TEST_CASE_FIXTURE(MidiFixture, "[R8.7 class] with a handler that reads every packet the OUT endpoint acknowledges every transfer") {
	startAndEnumerate();
	MidiProbe probe;
	for (uint8_t i = 0; i < 6; i++) {
		INFO("transfer " << int(i));
		CHECK(sendPackets({noteOnPkt(0, i, 100)}) == Handshake::Ack);
	}
	CHECK(probe.hooks.size() == 6);
}

// =============================================================== R11: packet API and bulk transfers

namespace {
/** Reads the IN endpoint packet by packet until it NAKs (the device is given time between packets). Returns the bytes; `sizes` the packet sizes */
std::vector<uint8_t> drainIn(MidiFixture &f, std::vector<size_t> *sizes = nullptr) {
	std::vector<uint8_t> all;
	for (int i = 0; i < 50; i++) {
		std::vector<uint8_t> data;
		if (HostDcd::in(f.host.address, 3, data) != Handshake::Ack) {
			break;
		}
		all.insert(all.end(), data.begin(), data.end());
		if (sizes) {
			sizes->push_back(data.size());
		}
		f.process();
	}
	return all;
}
std::vector<uint8_t> bytesOf(const std::vector<Packet> &packets) {
	std::vector<uint8_t> out;
	for (const Packet &p : packets) {
		out.insert(out.end(), p.begin(), p.end());
	}
	return out;
}
} // namespace

TEST_CASE_FIXTURE(MidiFixture, "[R11.1 packets] one packet written with tud_midi_packet_write is read by the host as one transfer of 4 bytes") {
	startAndEnumerate();
	const Packet p = pkt(2, 0xB, 0xB5, 7, 100);
	REQUIRE(tud_midi_packet_write(p.data()));
	process();
	Handshake status = Handshake::Nak;
	size_t rest = 9;
	const auto got = receivePackets(&status, &rest);
	CHECK(status == Handshake::Ack);
	CHECK(rest == 0);
	REQUIRE(got.size() == 1);
	CHECK(got[0] == p);
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.1 packets] several packets written before the host reads arrive in order") {
	startAndEnumerate();
	const std::vector<Packet> sent = {note(1), note(2), note(3)};
	for (const Packet &p : sent) {
		REQUIRE(tud_midi_packet_write(p.data()));
	}
	process();
	CHECK(drainIn(*this) == bytesOf(sent));
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.1 packets] characterization: three packets written one by one reach the host as a transfer of 4 bytes and a transfer of 8 bytes") {
	// NEEDS REVIEW (the charter expects one transfer): HD from midid_device.c: every write flushes at once, so the first
	// packet is handed to the controller alone; the other two wait in the TX FIFO until that transfer is complete and then go
	// out together. USB 2.0 does not ask for coalescing; no data is lost or reordered.
	startAndEnumerate();
	for (uint8_t i = 1; i <= 3; i++) {
		REQUIRE(tud_midi_packet_write(note(i).data()));
	}
	process();
	std::vector<size_t> sizes;
	drainIn(*this, &sizes);
	CHECK(sizes == std::vector<size_t>{4, 8});
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.1 packets] a packet with cable number 15 and a SysEx CIN arrives unchanged") {
	startAndEnumerate();
	const Packet p = pkt(15, 0x4, 0xF0, 0x7D, 0x01);
	REQUIRE(tud_midi_packet_write(p.data()));
	process();
	const auto got = receivePackets();
	REQUIRE(got.size() == 1);
	CHECK(got[0] == p);
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.2 packets] with the host not reading 17 packets are accepted and the 18th is refused") {
	startAndEnumerate();
	// HD: the first packet moves from the FIFO into the endpoint buffer at once (1), the TX FIFO holds 64 / 4 = 16 more
	uint8_t accepted = 0;
	for (uint8_t i = 0; i < 20; i++) {
		if (tud_midi_packet_write(note(i).data())) {
			accepted++;
		}
	}
	CHECK(accepted == 17);
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.2 packets] a refused write queues nothing: the host reads exactly the 17 accepted packets in order") {
	startAndEnumerate();
	std::vector<Packet> sent;
	for (uint8_t i = 0; i < 20; i++) {
		if (tud_midi_packet_write(note(i).data())) {
			sent.push_back(note(i));
		}
	}
	REQUIRE(sent.size() == 17);
	process();
	CHECK(drainIn(*this) == bytesOf(sent));
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.2 packets] after the host has read the packets writing works again") {
	startAndEnumerate();
	for (uint8_t i = 0; i < 17; i++) {
		REQUIRE(tud_midi_packet_write(note(i).data()));
	}
	REQUIRE_FALSE(tud_midi_packet_write(note(99).data()));
	process();
	CHECK(drainIn(*this).size() == 17 * 4);
	REQUIRE(tud_midi_packet_write(note(50).data()));
	process();
	const auto got = receivePackets();
	REQUIRE(got.size() == 1);
	CHECK(got[0] == note(50));
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.3 packets] characterization: a transfer of exactly 64 bytes is followed by a zero length packet and then the endpoint NAKs") {
	// NEEDS REVIEW: the USB 2.0 bulk rule (a transfer that is a multiple of wMaxPacketSize ends with a short packet) is not in
	// the supplied documents. HD: 17 writes with the host not reading give transfers of 4 bytes (endpoint buffer) and
	// 16 x 4 = 64 bytes (FIFO), then midid_xfer_cb sends the ZLP because 64 % 64 == 0 and the FIFO is empty.
	startAndEnumerate();
	for (uint8_t i = 0; i < 17; i++) {
		REQUIRE(tud_midi_packet_write(note(i).data()));
	}
	process();
	std::vector<size_t> sizes;
	drainIn(*this, &sizes);
	CHECK(sizes == std::vector<size_t>{4, 64, 0});
	std::vector<uint8_t> data;
	CHECK(HostDcd::in(host.address, 3, data) == Handshake::Nak);
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.3 packets] characterization: a transfer shorter than 64 bytes is not followed by a zero length packet") {
	startAndEnumerate();
	REQUIRE(tud_midi_packet_write(note(1).data()));
	process();
	std::vector<size_t> sizes;
	drainIn(*this, &sizes);
	CHECK(sizes == std::vector<size_t>{4});
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.4 packets] tud_midi_packet_write before the configuration returns false") {
	startMidi();
	REQUIRE_FALSE(tud_midi_mounted());
	CHECK_FALSE(tud_midi_packet_write(note(1).data()));
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.4 packets] tud_midi_stream_write before the configuration returns 0") {
	startMidi();
	const uint8_t bytes[3] = {0x90, 0x3C, 0x7F};
	CHECK(tud_midi_stream_write(0, bytes, 3) == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.4 packets] writes before the configuration queue nothing: the host reads nothing after the configuration") {
	startMidi();
	const uint8_t bytes[3] = {0x90, 0x3C, 0x7F};
	tud_midi_packet_write(note(1).data());
	tud_midi_stream_write(0, bytes, 3);
	enumerate();
	process();
	Handshake status = Handshake::Ack;
	const auto got = receivePackets(&status);
	CHECK(got.empty());
	CHECK(status == Handshake::Nak);
}

TEST_CASE_FIXTURE(MidiFixture, "[R11.5 packets] every channel voice event written as a byte stream is a well formed packet with the cable number, the status CIN and zero padding") {
	startAndEnumerate();
	// UM Table 4-1: CIN = status high nibble for 8n-En; Table 4-2 figure: byte 0 = cable number << 4 | CIN, unused bytes zero.
	// Program change (Cn) and channel pressure (Dn) carry 2 bytes, the others 3.
	for (unsigned status = 0x80; status <= 0xEF; status++) {
		const uint8_t cable = static_cast<uint8_t>(status % 16);
		const uint8_t type = static_cast<uint8_t>(status >> 4);
		const bool twoBytes = type == 0xC || type == 0xD;
		INFO("status " << std::hex << status << " cable " << std::dec << int(cable));
		const uint8_t bytes[3] = {static_cast<uint8_t>(status), 0x3C, 0x40};
		const uint32_t n = twoBytes ? 2 : 3;
		REQUIRE(tud_midi_stream_write(cable, bytes, n) == n);
		process();
		const auto got = receivePackets();
		REQUIRE(got.size() == 1);
		const Packet &p = got[0];
		CHECK((p[0] >> 4) == cable);
		CHECK((p[0] & 0x0F) == type);
		CHECK((p[0] & 0x0F) >= 2); // CIN 0 and 1 are reserved
		CHECK(p[1] == status);
		CHECK(p[2] == 0x3C);
		CHECK(p[3] == (twoBytes ? 0 : 0x40));
		process();
	}
}

// =============================================================== R12: end to end

TEST_CASE_FIXTURE(MidiFixture, "[R12.1 end to end] a note on, pitch bend, note off and clock sent in one transfer reach the handler as noteOn, pitchBend, noteOff and midiTick in order") {
	startAndEnumerate();
	MidiProbe probe;
	// MS: pitch bend centre is 00 40 (value 0); note on 9n kk vv, note off 8n kk vv, clock F8 (UM Table 4-2: 3F F8 xx xx on cable 3)
	REQUIRE(sendPackets({noteOnPkt(2, 60, 100), pkt(0, 0xE, 0xE2, 0x00, 0x40), noteOffPkt(2, 60, 64), pkt(3, 0xF, 0xF8)}) == Handshake::Ack);
	REQUIRE(probe.hooks.size() == 4);
	CHECK(probe.hooks[0] == MidiHook{"noteOn", 2, 60, 100});
	CHECK(probe.hooks[1] == MidiHook{"pitchBend", 2, 0, 0});
	CHECK(probe.hooks[2] == MidiHook{"noteOff", 2, 60, 64});
	CHECK(probe.hooks[3] == MidiHook{"midiTick", 0, 0, 0});
	for (const MidiHook &h : probe.hooks) {
		CHECK(h.itf == 0);
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R12.1 end to end] the same sequence sent as four separate transfers gives the same four hooks") {
	startAndEnumerate();
	MidiProbe probe;
	REQUIRE(sendPackets({noteOnPkt(2, 60, 100)}) == Handshake::Ack);
	REQUIRE(sendPackets({pkt(0, 0xE, 0xE2, 0x00, 0x40)}) == Handshake::Ack);
	REQUIRE(sendPackets({noteOffPkt(2, 60, 64)}) == Handshake::Ack);
	REQUIRE(sendPackets({pkt(3, 0xF, 0xF8)}) == Handshake::Ack);
	REQUIRE(probe.hooks.size() == 4);
	CHECK(probe.hooks[0] == MidiHook{"noteOn", 2, 60, 100});
	CHECK(probe.hooks[1] == MidiHook{"pitchBend", 2, 0, 0});
	CHECK(probe.hooks[2] == MidiHook{"noteOff", 2, 60, 64});
	CHECK(probe.hooks[3] == MidiHook{"midiTick", 0, 0, 0});
}

TEST_CASE_FIXTURE(MidiFixture, "[R12.1 end to end] a note on with velocity 0 sent by the host reaches the handler as noteOff") {
	startAndEnumerate();
	MidiProbe probe;
	REQUIRE(sendPackets({noteOnPkt(5, 72, 0)}) == Handshake::Ack); // MS: note on with velocity 0 is a note off
	REQUIRE(probe.hooks.size() == 1);
	CHECK(probe.hooks[0] == MidiHook{"noteOff", 5, 72, 0});
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R12.2 end to end] a note on and note off sent by the host make MidiMain call setUdUq(2500, 0), setPhiE_ext(value) and setUdUq(0, 0) on the driver") {
	startAndEnumerate();
	createMidiMain();
	FakeTmcPhase::state().calls.clear();
	REQUIRE(sendPackets({noteOnPkt(0, 69, 127)}) == Handshake::Ack);
	CHECK(tick(1) == 1);
	// HD (R5.4): note 69 = 440 Hz, velocity 127 -> volume 1.0, range 0x3fff, first tick t = 100e-6 s:
	// int16(sin(pi * 440 * 100e-6) * 16383) = int16(0.13779 * 16383) = 2257 (one count allowed for float versus double)
	REQUIRE(FakeTmcPhase::state().calls.size() == 2);
	CHECK(FakeTmcPhase::state().calls[0].name == "setUdUq");
	CHECK(FakeTmcPhase::state().calls[0].a == 2500); // default power
	CHECK(FakeTmcPhase::state().calls[0].b == 0);
	CHECK(FakeTmcPhase::state().calls[1].name == "setPhiE_ext");
	CHECK(std::abs(FakeTmcPhase::state().calls[1].a - 2257) <= 1);
	REQUIRE(sendPackets({noteOffPkt(0, 69, 64)}) == Handshake::Ack);
	CHECK(tick(1) == 1);
	REQUIRE(FakeTmcPhase::state().calls.size() == 3);
	CHECK(FakeTmcPhase::state().calls[2].name == "setUdUq");
	CHECK(FakeTmcPhase::state().calls[2].a == 0);
	CHECK(FakeTmcPhase::state().calls[2].b == 0);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R12.2 end to end] a note on of a channel other than 0 sent by the host causes no driver call on the next tick") {
	startAndEnumerate();
	createMidiMain();
	FakeTmcPhase::state().calls.clear();
	REQUIRE(sendPackets({noteOnPkt(1, 69, 127)}) == Handshake::Ack);
	tick(2);
	CHECK(FakeTmcPhase::state().calls.empty()); // only channel 0 is played (MidiMain.cpp, charter R5.6)
	// Control: a channel 0 note from the host does reach MidiMain and the driver (the case cannot pass when nothing arrives)
	REQUIRE(sendPackets({noteOnPkt(0, 69, 127)}) == Handshake::Ack);
	tick(1);
	CHECK(FakeTmcPhase::count("setUdUq") == 1);
	CHECK(FakeTmcPhase::count("setPhiE_ext") == 1);
}

TEST_CASE_FIXTURE(MidiFixture, "[R12.3 end to end] a burst of 16 packets (one 64 byte transfer) reaches the handler complete and in order") {
	startAndEnumerate();
	MidiProbe probe;
	std::vector<Packet> burst;
	for (uint8_t i = 0; i < 16; i++) {
		burst.push_back(note(i));
	}
	REQUIRE(sendPackets(burst) == Handshake::Ack);
	REQUIRE(probe.hooks.size() == 16);
	for (uint8_t i = 0; i < 16; i++) {
		INFO("packet " << int(i));
		CHECK(probe.hooks[i] == MidiHook{"noteOn", 0, i, 0x40});
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R12.3 end to end] the OUT endpoint accepts the next transfer after a burst of 16 packets") {
	startAndEnumerate();
	MidiProbe probe;
	std::vector<Packet> burst;
	for (uint8_t i = 0; i < 16; i++) {
		burst.push_back(note(i));
	}
	REQUIRE(sendPackets(burst) == Handshake::Ack);
	REQUIRE(probe.hooks.size() == 16);
	CHECK(sendPackets({noteOffPkt(0, 5, 0)}) == Handshake::Ack);
	REQUIRE(probe.hooks.size() == 17);
	CHECK(probe.hooks[16] == MidiHook{"noteOff", 0, 5, 0});
}

} // TEST_SUITE
