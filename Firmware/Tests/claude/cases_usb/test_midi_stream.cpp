/*
 * MidiStream: the byte stream API of TinyUSB's MIDI class driver (FFBoard/USB/class/midi/midi_device.c, vendor tier B:
 * tested, never fixed): tud_midi_stream_write (MIDI bytes -> USB-MIDI event packets, R9), tud_midi_stream_read and
 * tud_midi_available / tud_midi_packet_read (event packets -> MIDI bytes, R10). Charter:
 * Tests/claude/charters/driver/MIDI_REQUIREMENTS.md. Fixture: MidiFixture (support/MidiFixture.h), full speed, FIFOs and
 * endpoint buffers of 64 bytes, one MIDI interface.
 *
 * Oracles: UM = USB class definition for MIDI devices 1.0, section 4 (event packet: byte 0 = cable number << 4 | code
 * index number CIN, bytes 1-3 = MIDI_0..MIDI_2, unused bytes zero), Table 4-1 (CIN 0x2 two byte system common, 0x3 three
 * byte system common, 0x4 SysEx starts or continues, 0x5 single byte system common or SysEx ends with one byte, 0x6 SysEx
 * ends with two bytes, 0x7 SysEx ends with three bytes, 0x8-0xB and 0xE three byte voice messages, 0xC / 0xD two byte,
 * 0xF single byte) and the worked examples of Table 4-2. MS = MIDI 1.0 detailed specification (real-time messages may
 * interrupt other messages). MD = the code of midi_device.c where nothing above decides it. Expected packets are written
 * as hex strings straight from those tables ("09 90 3C 7F" is the packet 09 90 3C 7F; " | " separates packets).
 *
 * R9 writes with tud_midi_stream_write after startAndEnumerate() and reads the packets the host receives on endpoint 0x83.
 * R10 lets the host send packets on 0x02 with NO handler registered (tud_midi_rx_cb returns without reading), so they stay
 * in the RX FIFO, and reads them with the stream API. Only ONE host transfer is queued per test: without a handler the OUT
 * endpoint is armed again only when the 64 byte RX FIFO has 64 bytes free (_prep_out_transaction), so a second transfer
 * before a read is not accepted (that behaviour is R8.7 / B-MIDI-6, group MCD / MHD, not tested here).
 *
 * Derivation of the TX capacity used by R9.9: the TX FIFO holds 64 bytes = 16 packets. tud_midi_n_stream_write stops when
 * fewer than 4 bytes are free (checked before every byte, and a packet is queued only when complete, so a call always ends
 * on an event boundary) and then moves up to 64 bytes from the FIFO to the endpoint buffer (write_flush) unless the IN
 * endpoint is still busy with a previous transfer (host has not read). So one call consumes at most 16 packets, and while a
 * transfer is in flight the FIFO is the only room.
 *
 * Host-versus-target: only uint8_t / uint16_t / uint32_t are involved, no packing, no float; nothing differs on the target.
 * The SysEx continuation state of the writer is one byte (stream->buffer[0] low nibble); there is one MIDI interface
 * (CFG_TUD_MIDI = 1) so "the stream state is per interface" (R9.8, last clause) cannot be tested: not tested.
 *
 * Characterizations (NEEDS REVIEW): R9.7 (data byte as first byte of an event packed as CIN 0xF, two cases).
 * KNOWN_ISSUEs: B-MIDI-V1 (R9.4, real-time CIN), B-MIDI-V3 (R9.6, real-time inside SysEx), B-MIDI-V2 (R10.3, reserved
 * CIN, two cases).
 */
#include "doctest.h"
#include "MidiFixture.h"
#include "TestHelpers.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace {

using Pkt = std::array<uint8_t, 4>;
using Pkts = std::vector<Pkt>;

/** "09 90 3C 7F | 19 90 3C 7F": readable form of packets for comparisons */
std::string hex(const Pkts &ps) {
	std::string s;
	for (size_t i = 0; i < ps.size(); i++) {
		char b[16];
		std::snprintf(b, sizeof(b), "%02X %02X %02X %02X", ps[i][0], ps[i][1], ps[i][2], ps[i][3]);
		s += (i ? " | " : "");
		s += b;
	}
	return s;
}

/** "90 3C 7F": readable form of a byte stream */
std::string hexb(const std::vector<uint8_t> &v) {
	std::string s;
	for (size_t i = 0; i < v.size(); i++) {
		char b[8];
		std::snprintf(b, sizeof(b), "%02X", v[i]);
		s += (i ? " " : "");
		s += b;
	}
	return s;
}

struct StreamFixture : MidiFixture {
	/** USB device started, enumerated, MIDI mounted */
	void up() {
		startAndEnumerate();
		REQUIRE(tud_midi_mounted());
	}
	/** tud_midi_stream_write of the bytes on `cable`; the device then works (flush, IN completion) */
	uint32_t write(uint8_t cable, const std::vector<uint8_t> &bytes) {
		const uint32_t n = tud_midi_stream_write(cable, bytes.data(), static_cast<uint32_t>(bytes.size()));
		process();
		return n;
	}
	/** The host reads one transfer of the IN endpoint (a Nak gives no packets) and the device works */
	Pkts read() {
		const Pkts got = receivePackets();
		process();
		return got;
	}
	/** The host reads until the device NAKs (zero length packets are skipped) */
	Pkts drain() {
		Pkts all;
		for (int i = 0; i < 12; i++) {
			HostDcd::Handshake st = HostDcd::Handshake::Nak;
			const Pkts g = receivePackets(&st, nullptr, 64);
			process();
			all.insert(all.end(), g.begin(), g.end());
			if (st != HostDcd::Handshake::Ack) {
				break;
			}
		}
		return all;
	}
	/** The host sends the packets as one transfer (no handler exists, they stay in the RX FIFO) */
	void feed(const Pkts &ps) { REQUIRE(sendPackets(ps) == HostDcd::Handshake::Ack); }
	/** tud_midi_stream_read into a buffer of `cap` bytes */
	std::vector<uint8_t> readBytes(uint32_t cap = 64) {
		std::vector<uint8_t> buf(cap, 0xEE);
		const uint32_t n = tud_midi_stream_read(buf.data(), cap);
		REQUIRE(n <= cap);
		buf.resize(n);
		return buf;
	}
};

} // namespace

TEST_SUITE("MidiStream") {

// ============================================================================ R9: byte stream to packets

TEST_CASE_FIXTURE(StreamFixture, "[R9.1 stream_write] 90 3C 7F on cable 0 gives the packet 09 90 3C 7F") {
	up();
	CHECK(write(0, {0x90, 0x3C, 0x7F}) == 3);
	CHECK(hex(read()) == "09 90 3C 7F"); // Table 4-2: cable 0, CIN 9
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.1 stream_write] 90 3C 7F on cable 1 gives the packet 19 90 3C 7F") {
	up();
	CHECK(write(1, {0x90, 0x3C, 0x7F}) == 3);
	CHECK(hex(read()) == "19 90 3C 7F"); // Table 4-2 (the example "19 9n kk vv")
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.1 stream_write] B1 07 64 on cable 10 gives the packet AB B1 07 64") {
	up();
	CHECK(write(10, {0xB1, 0x07, 0x64}) == 3);
	CHECK(hex(read()) == "AB B1 07 64"); // Table 4-2 (the example "AB Bn pp vv")
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.1 stream_write] the cable number 15 is the high nibble of byte 0") {
	up();
	CHECK(write(15, {0x90, 0x3C, 0x7F}) == 3);
	CHECK(hex(read()) == "F9 90 3C 7F"); // cable 15 << 4 | CIN 9
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.1 stream_write] note off 8n kk vv gives CIN 8 on every channel") {
	up();
	for (uint8_t chan = 0; chan < 16; chan++) {
		CAPTURE(chan);
		REQUIRE(write(0, {static_cast<uint8_t>(0x80 | chan), 0x40, 0x10}) == 3);
		const Pkts got = read();
		REQUIRE(got.size() == 1);
		CHECK(got[0] == Pkt{0x08, static_cast<uint8_t>(0x80 | chan), 0x40, 0x10});
	}
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.1 stream_write] note on 9n kk vv gives CIN 9 on every channel") {
	up();
	for (uint8_t chan = 0; chan < 16; chan++) {
		CAPTURE(chan);
		REQUIRE(write(0, {static_cast<uint8_t>(0x90 | chan), 0x40, 0x10}) == 3);
		const Pkts got = read();
		REQUIRE(got.size() == 1);
		CHECK(got[0] == Pkt{0x09, static_cast<uint8_t>(0x90 | chan), 0x40, 0x10});
	}
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.1 stream_write] poly key pressure An pp vv gives CIN A on every channel") {
	up();
	for (uint8_t chan = 0; chan < 16; chan++) {
		CAPTURE(chan);
		REQUIRE(write(0, {static_cast<uint8_t>(0xA0 | chan), 0x40, 0x10}) == 3);
		const Pkts got = read();
		REQUIRE(got.size() == 1);
		CHECK(got[0] == Pkt{0x0A, static_cast<uint8_t>(0xA0 | chan), 0x40, 0x10});
	}
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.1 stream_write] control change Bn cc vv gives CIN B on every channel") {
	up();
	for (uint8_t chan = 0; chan < 16; chan++) {
		CAPTURE(chan);
		REQUIRE(write(0, {static_cast<uint8_t>(0xB0 | chan), 0x07, 0x64}) == 3);
		const Pkts got = read();
		REQUIRE(got.size() == 1);
		CHECK(got[0] == Pkt{0x0B, static_cast<uint8_t>(0xB0 | chan), 0x07, 0x64});
	}
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.1 stream_write] pitch bend En ll mm gives CIN E on every channel") {
	up();
	for (uint8_t chan = 0; chan < 16; chan++) {
		CAPTURE(chan);
		REQUIRE(write(0, {static_cast<uint8_t>(0xE0 | chan), 0x00, 0x40}) == 3);
		const Pkts got = read();
		REQUIRE(got.size() == 1);
		CHECK(got[0] == Pkt{0x0E, static_cast<uint8_t>(0xE0 | chan), 0x00, 0x40});
	}
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.1 stream_write] several events in one call give one packet each in order") {
	up();
	CHECK(write(0, {0x90, 0x3C, 0x7F, 0x80, 0x3C, 0x40, 0xC1, 0x05}) == 8);
	CHECK(hex(read()) == "09 90 3C 7F | 08 80 3C 40 | 0C C1 05 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.2 stream_write] C2 05 gives 0C C2 05 00 (program change padded with zero)") {
	up();
	CHECK(write(0, {0xC2, 0x05}) == 2);
	CHECK(hex(read()) == "0C C2 05 00"); // Table 4-1: CIN 0xC is two bytes
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.2 stream_write] D3 40 gives 0D D3 40 00 (channel pressure padded with zero)") {
	up();
	CHECK(write(0, {0xD3, 0x40}) == 2);
	CHECK(hex(read()) == "0D D3 40 00"); // Table 4-1: CIN 0xD is two bytes
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.2 stream_write] two byte events on every channel carry the cable number and zero padding") {
	up();
	for (uint8_t chan = 0; chan < 16; chan++) {
		CAPTURE(chan);
		REQUIRE(write(5, {static_cast<uint8_t>(0xC0 | chan), 0x7F}) == 2);
		REQUIRE(write(5, {static_cast<uint8_t>(0xD0 | chan), 0x01}) == 2);
		const Pkts got = drain();
		REQUIRE(got.size() == 2);
		CHECK(got[0] == Pkt{0x5C, static_cast<uint8_t>(0xC0 | chan), 0x7F, 0x00});
		CHECK(got[1] == Pkt{0x5D, static_cast<uint8_t>(0xD0 | chan), 0x01, 0x00});
	}
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.3 stream_write] F1 23 (time code quarter frame) gives CIN 2: 02 F1 23 00") {
	up();
	CHECK(write(0, {0xF1, 0x23}) == 2);
	CHECK(hex(read()) == "02 F1 23 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.3 stream_write] F3 05 (song select) gives CIN 2: 02 F3 05 00") {
	up();
	CHECK(write(0, {0xF3, 0x05}) == 2);
	CHECK(hex(read()) == "02 F3 05 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.3 stream_write] F2 10 20 (song position pointer) on cable 4 gives CIN 3: 43 F2 10 20") {
	up();
	CHECK(write(4, {0xF2, 0x10, 0x20}) == 3);
	CHECK(hex(read()) == "43 F2 10 20");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.3 stream_write] F6 (tune request) gives CIN 5 with two zero bytes: 05 F6 00 00") {
	up();
	CHECK(write(0, {0xF6}) == 1);
	CHECK(hex(read()) == "05 F6 00 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.3 stream_write] characterization: a lone F7 outside a SysEx gives CIN 5: 05 F7 00 00") {
	up();
	CHECK(write(0, {0xF7}) == 1);
	CHECK(hex(read()) == "05 F7 00 00"); // FW: Table 4-1 does not decide a stray EOX; CIN 5 is what the firmware writes
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.3 stream_write] characterization: the undefined system bytes F4 and F5 give CIN 5 single byte packets") {
	up();
	// Table 4-1 note 1: F4 / F5 are undefined; if they were ever defined they would be CIN 2 or 3. FW: CIN 5.
	CHECK(write(0, {0xF4}) == 1);
	CHECK(write(0, {0xF5}) == 1);
	CHECK(hex(drain()) == "05 F4 00 00 | 05 F5 00 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.4 stream_write] real-time bytes F8 F9 FA FB FC FD FE FF on cable 3 are single byte packets with CIN 0xF: 3F xx 00 00" *
								  KNOWN_ISSUE("B-MIDI-V1 (TinyUSB, vendor): tud_midi_n_stream_write (midi_device.c:288-312) treats every 0xF_ byte other "
											  "than F0 F1 F2 F3 like a single byte system common message and writes CIN 5 (35 F8 00 00); Table 4-1/4-2 "
											  "prescribe CIN 0xF for the real-time messages (3F F8 xx xx)")) {
	up();
	const uint8_t realtime[] = {0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF};
	for (uint8_t rt : realtime) {
		CAPTURE(int(rt));
		REQUIRE(write(3, {rt}) == 1);
		const Pkts got = read();
		REQUIRE(got.size() == 1);
		CHECK(got[0] == Pkt{0x3F, rt, 0x00, 0x00}); // Defect: the first iteration (F8) already fails on the CIN
	}
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] SysEx F0 00 01 F7 gives p4 F0 00 01 and p5 F7 00 00") {
	up();
	CHECK(write(0, {0xF0, 0x00, 0x01, 0xF7}) == 4);
	CHECK(hex(read()) == "04 F0 00 01 | 05 F7 00 00"); // Table 4-2: SysEx ends with single byte
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] SysEx F0 00 01 02 F7 gives p4 F0 00 01 and p6 02 F7 00") {
	up();
	CHECK(write(0, {0xF0, 0x00, 0x01, 0x02, 0xF7}) == 5);
	CHECK(hex(read()) == "04 F0 00 01 | 06 02 F7 00"); // Table 4-2: SysEx ends with two bytes
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] SysEx F0 00 01 02 03 F7 gives p4 F0 00 01 and p7 02 03 F7") {
	up();
	CHECK(write(0, {0xF0, 0x00, 0x01, 0x02, 0x03, 0xF7}) == 6);
	CHECK(hex(read()) == "04 F0 00 01 | 07 02 03 F7"); // Table 4-2: SysEx ends with three bytes
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] SysEx F0 F7 without payload gives the single packet p6 F0 F7 00") {
	up();
	CHECK(write(0, {0xF0, 0xF7}) == 2);
	CHECK(hex(read()) == "06 F0 F7 00"); // Table 4-2: SysEx starts and ends with two bytes
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] SysEx F0 mm F7 with one payload byte gives the single packet p7 F0 mm F7") {
	up();
	CHECK(write(0, {0xF0, 0x55, 0xF7}) == 3);
	CHECK(hex(read()) == "07 F0 55 F7"); // Table 4-2: SysEx starts and ends with three bytes
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] a SysEx with 7 payload bytes gives two continued CIN 4 packets and the end packet p7") {
	up();
	// F0 01 02 | 03 04 05 | 06 07 F7: the first packet holds F0 and two payload bytes, the next three payload bytes
	// each, the last packet 06 07 F7 closes with CIN 7 (three bytes)
	CHECK(write(0, {0xF0, 1, 2, 3, 4, 5, 6, 7, 0xF7}) == 9);
	CHECK(hex(read()) == "04 F0 01 02 | 04 03 04 05 | 07 06 07 F7");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] a SysEx with 12 payload bytes gives four continued CIN 4 packets and the end packet p6") {
	up();
	// F0 01 02 | 03 04 05 | 06 07 08 | 09 0A 0B | 0C F7: payload 1..12, the last packet holds 0C F7 (CIN 6)
	CHECK(write(0, {0xF0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0xF7}) == 14);
	CHECK(hex(read()) == "04 F0 01 02 | 04 03 04 05 | 04 06 07 08 | 04 09 0A 0B | 06 0C F7 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] a SysEx whose payload ends on a packet boundary closes with the lone packet p5 F7 00 00") {
	up();
	// 5 payload bytes: F0 01 02 | 03 04 05 | F7
	CHECK(write(0, {0xF0, 1, 2, 3, 4, 5, 0xF7}) == 7);
	CHECK(hex(read()) == "04 F0 01 02 | 04 03 04 05 | 05 F7 00 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] every packet of a SysEx on cable 2 carries the cable number") {
	up();
	CHECK(write(2, {0xF0, 1, 2, 3, 0xF7}) == 5);
	CHECK(hex(read()) == "24 F0 01 02 | 26 03 F7 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] a SysEx whose payload ends on a packet boundary on cable 2 keeps the cable number on every packet including the lone p5") {
	up();
	CHECK(write(2, {0xF0, 1, 2, 3, 4, 5, 0xF7}) == 7);
	CHECK(hex(read()) == "24 F0 01 02 | 24 03 04 05 | 25 F7 00 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.5 stream_write] a normal event after a finished SysEx is a normal packet") {
	up();
	CHECK(write(0, {0xF0, 0xF7, 0x90, 0x3C, 0x7F}) == 5);
	CHECK(hex(read()) == "06 F0 F7 00 | 09 90 3C 7F");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.6 stream_write] a real-time byte inside a SysEx is its own packet with CIN 0xF and the SysEx continues" *
								  KNOWN_ISSUE("B-MIDI-V3 (TinyUSB, vendor): inside a SysEx tud_midi_n_stream_write (midi_device.c:263-286) takes every byte "
											  "as SysEx payload, so F8 between F0 01 02 and 03 F7 becomes payload (37 F8 03 F7) instead of the packet 3F F8 00 00; "
											  "MIDI 1.0 allows real-time messages inside any message and Table 4-2 gives them CIN 0xF")) {
	up();
	// F0 01 02 fills the first packet (34 F0 01 02); F8 arrives at a packet boundary: intended packets are
	// 34 F0 01 02 | 3F F8 00 00 | 36 03 F7 00
	CHECK(write(3, {0xF0, 0x01, 0x02, 0xF8, 0x03, 0xF7}) == 6);
	const Pkts got = read();
	REQUIRE(got.size() == 3); // Defect: only two packets (the F8 is payload of the second)
	CHECK(hex(got) == "34 F0 01 02 | 3F F8 00 00 | 36 03 F7 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.6 stream_write] a real-time byte in the middle of a SysEx packet is sent as its own CIN 0xF packet" *
								  KNOWN_ISSUE("B-MIDI-V3 (TinyUSB, vendor): F8 between F0 01 and 02 F7 is taken as SysEx payload (04 F0 01 F8 | 36 02 F7 00 "
											  "on cable 3) instead of its own packet 3F F8 00 00; MIDI 1.0 real-time messages may interrupt any message")) {
	up();
	// intended: the F8 packet goes out as soon as the byte arrives; the SysEx packets carry F0 01 02 and F7:
	// 3F F8 00 00 | 34 F0 01 02 | 35 F7 00 00
	CHECK(write(3, {0xF0, 0x01, 0xF8, 0x02, 0xF7}) == 5);
	const Pkts got = drain();
	REQUIRE(got.size() == 3); // Defect: two packets, the F8 is payload
	CHECK(hex(got) == "3F F8 00 00 | 34 F0 01 02 | 35 F7 00 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.7 stream_write] characterization: a data byte as the first byte of an event gives a single byte packet with CIN 0xF" ) {
	up();
	CHECK(write(2, {0x3C}) == 1);
	CHECK(hex(read()) == "2F 3C 00 00"); // Table 4-1 note 2: CIN 0xF is for unparsed single bytes; FW: the stream API packs them
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.7 stream_write] characterization: running status data bytes 3C 7F give one CIN 0xF packet each") {
	up();
	CHECK(write(0, {0x3C, 0x7F}) == 2);
	CHECK(hex(read()) == "0F 3C 00 00 | 0F 7F 00 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.8 stream_write] 90 3C then 7F in two calls gives the packet only after the third byte") {
	up();
	CHECK(write(0, {0x90, 0x3C}) == 2);
	HostDcd::Handshake st = HostDcd::Handshake::Ack;
	CHECK(receivePackets(&st).empty()); // nothing was written to the FIFO yet
	CHECK(st == HostDcd::Handshake::Nak);
	process();
	CHECK(write(0, {0x7F}) == 1);
	CHECK(hex(read()) == "09 90 3C 7F");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.8 stream_write] an event split over three calls gives one correct packet") {
	up();
	CHECK(write(1, {0xB2}) == 1);
	CHECK(write(1, {0x07}) == 1);
	CHECK(write(1, {0x64}) == 1);
	CHECK(hex(read()) == "1B B2 07 64");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.8 stream_write] a two byte event split over two calls gives one correct packet") {
	up();
	CHECK(write(0, {0xC2}) == 1);
	CHECK(write(0, {0x05}) == 1);
	CHECK(hex(read()) == "0C C2 05 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.8 stream_write] a SysEx split across calls gives the same packets as one call") {
	up();
	CHECK(write(0, {0xF0, 0x00}) == 2);
	CHECK(write(0, {0x01, 0x02}) == 2);
	CHECK(write(0, {0xF7}) == 1);
	CHECK(hex(drain()) == "04 F0 00 01 | 06 02 F7 00"); // the first packet is in flight, the second waits in the FIFO
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.8 stream_write] a call that ends in the middle of an event followed by a call with more events keeps the order") {
	up();
	CHECK(write(0, {0x90, 0x3C, 0x7F, 0x80}) == 4);
	CHECK(write(0, {0x3C, 0x40, 0xC0, 0x01}) == 4);
	CHECK(hex(drain()) == "09 90 3C 7F | 08 80 3C 40 | 0C C0 01 00"); // the first packet is in flight, the rest waits in the FIFO
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.9 stream_write] one call with more events than the 16 packet TX FIFO consumes 48 bytes and the rest is written by the next call") {
	up();
	// 20 note on events (60 bytes). The FIFO takes 16 packets = 64 bytes: the call stops after 16 events = 48 bytes
	// (check before every byte: remaining < 4 after the 16th packet). write_flush then hands the FIFO to the endpoint,
	// so the FIFO is empty again and the remaining 12 bytes (4 events) fit.
	std::vector<uint8_t> data;
	for (uint8_t i = 0; i < 20; i++) {
		data.insert(data.end(), {0x90, static_cast<uint8_t>(0x20 + i), 0x7F});
	}
	const uint32_t first = tud_midi_stream_write(0, data.data(), 60);
	CHECK(first == 48);
	const uint32_t second = tud_midi_stream_write(0, data.data() + first, 60 - first);
	CHECK(second == 12);
	process();
	// the first IN transfer carries the 16 packets of the first call: a full 64 bytes
	size_t firstRest = 99;
	Pkts got = receivePackets(nullptr, &firstRest, 64);
	process();
	CHECK(got.size() == 16);
	CHECK(firstRest == 0);
	const Pkts more = drain();
	got.insert(got.end(), more.begin(), more.end());
	REQUIRE(got.size() == 20);
	for (uint8_t i = 0; i < 20; i++) {
		CAPTURE(int(i));
		CHECK(got[i] == Pkt{0x09, 0x90, static_cast<uint8_t>(0x20 + i), 0x7F});
	}
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.9 stream_write] with a transfer in flight and the host not reading, the FIFO takes 16 more events and a write while it is full consumes 0 bytes") {
	up();
	// Event 0 is flushed to the endpoint buffer at once (the FIFO is empty afterwards) and the IN endpoint stays busy
	// because the host does not read. Events 1..16 fill the FIFO (16 packets = 64 bytes); event 17 does not fit.
	CHECK(write(0, {0x90, 0x30, 0x7F}) == 3);
	std::vector<uint8_t> data;
	for (uint8_t i = 1; i <= 17; i++) {
		data.insert(data.end(), {0x90, static_cast<uint8_t>(0x30 + i), 0x7F});
	}
	const uint32_t n = tud_midi_stream_write(0, data.data(), 51);
	CHECK(n == 48); // events 1..16
	CHECK(tud_midi_stream_write(0, data.data() + n, 3) == 0); // FIFO full, endpoint busy: nothing consumed
	process();
	// after the host drains, the rest (event 17) is written and everything arrives once, in order
	// first transfer: the single event in flight (4 bytes); second transfer: the full FIFO, 16 packets = 64 bytes
	Pkts firstPart = receivePackets(nullptr, nullptr, 64);
	process();
	CHECK(firstPart.size() == 1);
	const Pkts fifoPart = receivePackets(nullptr, nullptr, 64);
	process();
	CHECK(fifoPart.size() == 16);
	firstPart.insert(firstPart.end(), fifoPart.begin(), fifoPart.end());
	const Pkts zlp = drain();
	CHECK(zlp.empty());
	REQUIRE(firstPart.size() == 17);
	CHECK(write(0, std::vector<uint8_t>(data.begin() + n, data.end())) == 3);
	const Pkts rest = drain();
	REQUIRE(rest.size() == 1);
	for (uint8_t i = 0; i < 17; i++) {
		CAPTURE(int(i));
		CHECK(firstPart[i] == Pkt{0x09, 0x90, static_cast<uint8_t>(0x30 + i), 0x7F});
	}
	CHECK(rest[0] == Pkt{0x09, 0x90, 0x41, 0x7F});
}

TEST_CASE_FIXTURE(StreamFixture, "[R9.9 stream_write] a SysEx larger than the TX FIFO is continued correctly after the host drained it") {
	up();
	// F0, payload 1..50, F7 = 52 bytes: packet 1 = F0 01 02, then 16 packets of 3 payload bytes (03..32), then F7 alone
	// = 18 packets. One call takes 16 packets = 48 bytes; the next call takes the remaining 4 bytes.
	std::vector<uint8_t> data{0xF0};
	for (uint8_t i = 1; i <= 50; i++) {
		data.push_back(i);
	}
	data.push_back(0xF7);
	REQUIRE(data.size() == 52);
	const uint32_t first = tud_midi_stream_write(0, data.data(), 52);
	CHECK(first == 48);
	CHECK(tud_midi_stream_write(0, data.data() + first, 52 - first) == 4);
	process();
	const Pkts got = drain();
	REQUIRE(got.size() == 18);
	CHECK(got[0] == Pkt{0x04, 0xF0, 0x01, 0x02});
	for (uint8_t k = 0; k < 16; k++) {
		CAPTURE(int(k));
		const uint8_t p = static_cast<uint8_t>(3 + 3 * k);
		CHECK(got[1 + k] == Pkt{0x04, p, static_cast<uint8_t>(p + 1), static_cast<uint8_t>(p + 2)});
	}
	CHECK(got[17] == Pkt{0x05, 0xF7, 0x00, 0x00});
}

// ============================================================================ R10: packets to byte stream

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] the packet 19 9n kk vv (cable 1) gives the bytes 9n kk vv") {
	up();
	feed({pkt(1, 0x9, 0x93, 0x3C, 0x7F)});
	CHECK(hexb(readBytes()) == "93 3C 7F"); // Table 4-2; the cable number is not part of the byte stream
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] the three byte voice packets (CIN 8, 9, A, B, E) give three bytes each") {
	up();
	feed({pkt(0, 0x8, 0x80, 1, 2), pkt(0, 0x9, 0x91, 3, 4), pkt(0, 0xA, 0xA2, 5, 6), pkt(0, 0xB, 0xB3, 7, 8), pkt(0, 0xE, 0xE4, 9, 10)});
	CHECK(hexb(readBytes()) == "80 01 02 91 03 04 A2 05 06 B3 07 08 E4 09 0A");
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] the two byte packets CIN C and D give two bytes each") {
	up();
	feed({pkt(0, 0xC, 0xC2, 0x05), pkt(0, 0xD, 0xD3, 0x40)});
	CHECK(hexb(readBytes()) == "C2 05 D3 40"); // Table 4-1: CIN 0xC / 0xD are two byte messages
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] the system common packets CIN 2 (two bytes) and CIN 3 (three bytes) give two and three bytes") {
	up();
	feed({pkt(0, 0x2, 0xF1, 0x23), pkt(0, 0x3, 0xF2, 0x10, 0x20)});
	CHECK(hexb(readBytes()) == "F1 23 F2 10 20");
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] the single byte packets CIN 5 (F6) and CIN 0xF (F8 on cable 3) give one byte each") {
	up();
	feed({pkt(0, 0x5, 0xF6), pkt(3, 0xF, 0xF8)});
	CHECK(hexb(readBytes()) == "F6 F8"); // Table 4-2: 3F F8 xx xx is the real-time clock
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] the SysEx pair p4 F0 00 01 and p5 F7 00 00 gives F0 00 01 F7") {
	up();
	feed({pkt(1, 0x4, 0xF0, 0x00, 0x01), pkt(1, 0x5, 0xF7)});
	CHECK(hexb(readBytes()) == "F0 00 01 F7");
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] the SysEx pair p4 F0 00 01 and p6 02 F7 00 gives F0 00 01 02 F7") {
	up();
	feed({pkt(0, 0x4, 0xF0, 0x00, 0x01), pkt(0, 0x6, 0x02, 0xF7)});
	CHECK(hexb(readBytes()) == "F0 00 01 02 F7"); // CIN 6 = two bytes
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] the SysEx pair p4 F0 00 01 and p7 02 03 F7 gives F0 00 01 02 03 F7") {
	up();
	feed({pkt(0, 0x4, 0xF0, 0x00, 0x01), pkt(0, 0x7, 0x02, 0x03, 0xF7)});
	CHECK(hexb(readBytes()) == "F0 00 01 02 03 F7"); // CIN 7 = three bytes
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] the packet p6 F0 F7 00 gives F0 F7 and p7 F0 mm F7 gives F0 mm F7") {
	up();
	feed({pkt(0, 0x6, 0xF0, 0xF7), pkt(0, 0x7, 0xF0, 0x55, 0xF7)});
	CHECK(hexb(readBytes()) == "F0 F7 F0 55 F7");
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] a mixed sequence of note on, clock and note off keeps the order of the bytes") {
	up();
	feed({noteOnPkt(0, 0x3C, 0x64), pkt(0, 0xF, 0xF8), noteOffPkt(0, 0x3C, 0x00)});
	CHECK(hexb(readBytes()) == "90 3C 64 F8 80 3C 00");
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] a full 64 byte transfer of 16 packets is read completely and the next transfer is accepted afterwards") {
	up();
	Pkts ps;
	std::string expected;
	for (uint8_t i = 0; i < 16; i++) {
		ps.push_back(pkt(0, 0x9, 0x90, i, 0x7F));
		char b[16];
		std::snprintf(b, sizeof(b), "%s90 %02X 7F", i ? " " : "", i);
		expected += b;
	}
	// 64 bytes into the 64 byte RX FIFO; no host zero length packet (a full FIFO is not armed again, so the ZLP would be NAKed)
	REQUIRE(host.sendOut(epOut(), epSize(), std::vector<uint8_t>(reinterpret_cast<const uint8_t *>(ps.data()), reinterpret_cast<const uint8_t *>(ps.data()) + 64), false) == HostDcd::Handshake::Ack);
	process();
	CHECK(hexb(readBytes(64)) == expected);
	feed({noteOnPkt(0, 0x40, 0x01)}); // the read re-armed the OUT endpoint (_prep_out_transaction)
	CHECK(hexb(readBytes()) == "90 40 01");
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] reading with nothing queued returns 0 and leaves the buffer unchanged") {
	up();
	uint8_t buf[4] = {0xEE, 0xEE, 0xEE, 0xEE};
	CHECK(tud_midi_stream_read(buf, 4) == 0);
	CHECK(buf[0] == 0xEE);
	CHECK(buf[3] == 0xEE);
	CHECK(tud_midi_available() == 0);
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] a buffer larger than the data returns exactly the event bytes and does not touch the rest") {
	up();
	feed({noteOnPkt(0, 0x3C, 0x7F)});
	uint8_t buf[16];
	for (uint8_t &b : buf) {
		b = 0xEE;
	}
	CHECK(tud_midi_stream_read(buf, 16) == 3);
	CHECK(buf[0] == 0x90);
	CHECK(buf[1] == 0x3C);
	CHECK(buf[2] == 0x7F);
	CHECK(buf[3] == 0xEE); // the padding byte of the packet is not copied
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.1 stream_read] a read of size 0 returns 0 and keeps the packet queued") {
	up();
	feed({noteOnPkt(0, 0x3C, 0x7F)});
	uint8_t buf[1] = {0xEE};
	CHECK(tud_midi_stream_read(buf, 0) == 0);
	CHECK(hexb(readBytes()) == "90 3C 7F");
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.2 stream_read] a packet read one byte at a time gives its bytes in order and available counts down") {
	up();
	feed({noteOnPkt(0, 0x3C, 0x7F)});
	// available = FIFO bytes + unread bytes of the current event: 4 (the packet) before the first read; after the
	// first byte the packet is taken from the FIFO (0 left) and 2 event bytes are unread, then 1, then 0
	CHECK(tud_midi_available() == 4);
	uint8_t b = 0;
	CHECK(tud_midi_stream_read(&b, 1) == 1);
	CHECK(b == 0x90);
	CHECK(tud_midi_available() == 2);
	CHECK(tud_midi_stream_read(&b, 1) == 1);
	CHECK(b == 0x3C);
	CHECK(tud_midi_available() == 1);
	CHECK(tud_midi_stream_read(&b, 1) == 1);
	CHECK(b == 0x7F);
	CHECK(tud_midi_available() == 0);
	CHECK(tud_midi_stream_read(&b, 1) == 0);
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.2 stream_read] a buffer of 2 bytes for a three byte event keeps the third byte for the next call") {
	up();
	feed({noteOnPkt(0, 0x3C, 0x7F)});
	uint8_t b[2] = {0, 0};
	CHECK(tud_midi_stream_read(b, 2) == 2);
	CHECK(b[0] == 0x90);
	CHECK(b[1] == 0x3C);
	CHECK(tud_midi_available() == 1);
	CHECK(tud_midi_stream_read(b, 2) == 1); // only the rest of the event, nothing else queued
	CHECK(b[0] == 0x7F);
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.2 stream_read] a buffer of 5 bytes across two packets returns 5 bytes and then the sixth") {
	up();
	feed({noteOnPkt(0, 0x3C, 0x7F), noteOnPkt(0, 0x3D, 0x40)});
	CHECK(hexb(readBytes(5)) == "90 3C 7F 90 3D");
	CHECK(hexb(readBytes(5)) == "40");
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.2 stream_read] available counts the bytes of all queued packets") {
	up();
	feed({noteOnPkt(0, 0x3C, 0x7F), noteOnPkt(0, 0x3D, 0x7F), noteOffPkt(0, 0x3C, 0x00)});
	CHECK(tud_midi_available() == 12); // 3 packets of 4 bytes in the FIFO
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.3 stream_read] a reserved CIN 1 packet between two note packets is skipped: 6 bytes are returned" *
								  KNOWN_ISSUE("B-MIDI-V2 (TinyUSB, vendor): tud_midi_n_stream_read (midi_device.c:155-160) executes `return 0` at a "
											  "reserved CIN (0 or 1) instead of skipping the packet, so the 3 bytes already copied are reported as 0 "
											  "(Table 4-1: reserved, to be ignored)")) {
	up();
	feed({noteOnPkt(0, 0x3C, 0x7F), pkt(0, 0x1, 0xAA, 0xBB, 0xCC), noteOnPkt(0, 0x3D, 0x7F)});
	uint8_t buf[16] = {};
	const uint32_t n = tud_midi_stream_read(buf, 16);
	CHECK(n == 6); // Defect: 0
	CHECK(hexb(std::vector<uint8_t>(buf, buf + 6)) == "90 3C 7F 90 3D 7F");
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.3 stream_read] a reserved CIN 0 packet between two note packets is skipped: 6 bytes are returned" *
								  KNOWN_ISSUE("B-MIDI-V2 (TinyUSB, vendor): same as the CIN 1 case, the reserved CIN 0 makes tud_midi_n_stream_read return "
											  "0 and the bytes copied before it are reported as 0")) {
	up();
	feed({noteOnPkt(0, 0x3C, 0x7F), pkt(0, 0x0, 0x00, 0x00, 0x00), noteOnPkt(0, 0x3D, 0x7F)});
	uint8_t buf[16] = {};
	const uint32_t n = tud_midi_stream_read(buf, 16);
	CHECK(n == 6); // Defect: 0
	CHECK(hexb(std::vector<uint8_t>(buf, buf + 6)) == "90 3C 7F 90 3D 7F");
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.4 packet_read] a transfer of 4 bytes is read as one packet and a second read returns false") {
	up();
	feed({noteOnPkt(0, 0x3C, 0x7F)});
	uint8_t p[4] = {0, 0, 0, 0};
	CHECK(tud_midi_packet_read(p));
	CHECK(Pkt{p[0], p[1], p[2], p[3]} == Pkt{0x09, 0x90, 0x3C, 0x7F});
	uint8_t q[4] = {0xEE, 0xEE, 0xEE, 0xEE};
	CHECK_FALSE(tud_midi_packet_read(q));
	CHECK(tud_midi_available() == 0);
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.4 packet_read] a transfer of 3 bytes (less than a packet) makes packet_read return false and leaves the FIFO empty") {
	up();
	// a malformed host transfer of 3 bytes: tu_fifo_read_n(.., 4) returns 3, so the read is not a packet
	REQUIRE(host.sendOut(epOut(), epSize(), {0x09, 0x90, 0x3C}, true) == HostDcd::Handshake::Ack);
	process();
	REQUIRE(tud_midi_available() == 3);
	uint8_t p[4] = {0, 0, 0, 0};
	CHECK_FALSE(tud_midi_packet_read(p));
	CHECK(tud_midi_available() == 0);
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.4 packet_read] packet_read with nothing queued returns false") {
	up();
	uint8_t p[4] = {0, 0, 0, 0};
	CHECK_FALSE(tud_midi_packet_read(p));
}

TEST_CASE_FIXTURE(StreamFixture, "[R10.4 packet_read] packet_read before the interface is configured returns false") {
	startMidi(); // not enumerated: no endpoint is open
	REQUIRE_FALSE(tud_midi_mounted());
	uint8_t p[4] = {0, 0, 0, 0};
	CHECK_FALSE(tud_midi_packet_read(p));
}

} // TEST_SUITE
