/*
 * MidiHandler: MidiHandler::midiRx (Firmware/FFBoard/Src/MidiHandler.cpp, decoding of one 4 byte USB-MIDI event packet
 * into the virtual hooks) and the TinyUSB callback tud_midi_rx_cb with the global pointer `midihandler`
 * (Src/global_callbacks.cpp:349-358).
 *
 * Requirements covered (Tests/claude/charters/driver/MIDI_REQUIREMENTS.md): R2.1 - R2.11 and R2.13 (R2.12, a note number
 * with bit 7 set, is tested at the MidiMain level in test_midi_main.cpp), R3.1 - R3.5.
 *
 * Oracles: UM = USB class definition for MIDI devices 1.0 (tables 4-1 and 4-2: event packets, code index numbers),
 * MS = MIDI 1.0 detailed specification 4.2.1 (status bytes, note on with velocity 0 = note off, pitch bend 14 bit with the
 * least significant byte first and centre 00 40, F8 timing clock), HD = hand derivation in a comment, FW = the firmware's own
 * behaviour where the documents are silent (those cases start with `characterization:`).
 *
 * Decoding cases (R2) call MidiProbe::midiRx directly with the packet (no USB); the USB cases (R3) send the packets as bulk
 * OUT transfers on endpoint 0x02 of the fake controller and let the real TinyUSB class driver deliver them.
 *
 * Host versus target: nothing here depends on type sizes; the pitch bend value is computed in int (the firmware stores it in
 * an int16_t, the range -8192..8191 fits); the Cortex-M4 is little endian like the host, but no multi byte memory is
 * reinterpreted.
 *
 * Characterizations (NEEDS REVIEW): channel pressure gives no hook (question 56), the cable number is ignored (question 57),
 * poly pressure / system common / real-time F9-FF go to otherPacket, the bit 7 masking of the pitch bend bytes, two handlers
 * share the global pointer (R3.3).
 * KNOWN_ISSUE: B-MIDI-3 (three cases, sysexState after a SysEx that ends with CIN 6 or 7), B-MIDI-4 (CIN ignored for single
 * byte packets), B-MIDI-6 (no handler: OUT endpoint is not armed again), see the comments of the cases.
 */
#include "doctest.h"
#include "MidiFixture.h"
#include "TestHelpers.h"

#include <array>
#include <memory>
#include <vector>

namespace {

using Packet = std::array<uint8_t, 4>;

/** Decodes one packet through the probe (midiRx takes a non const pointer) */
void rx(MidiProbe &p, Packet packet, uint8_t itf = 0) {
	uint8_t buf[4] = {packet[0], packet[1], packet[2], packet[3]};
	p.midiRx(itf, buf);
}

/** The hook list of a probe contains exactly this one hook */
bool onlyHook(const MidiProbe &p, const MidiHook &expected) {
	return p.hooks.size() == 1 && p.hooks[0] == expected;
}

MidiHook hook(const char *name, int chan, int a, int b = 0, int c = 0, int d = 0) {
	MidiHook h;
	h.name = name;
	h.chan = chan;
	h.a = a;
	h.b = b;
	h.c = c;
	h.d = d;
	return h;
}

/** otherPacket hook with the exact four packet bytes */
MidiHook otherHook(Packet p) { return hook("otherPacket", 0, p[0], p[1], p[2], p[3]); }

size_t noteHooks(const MidiProbe &p) { return p.count("noteOn") + p.count("noteOff"); }

/** A handler that only counts what it is given (to survive its own destruction in the R3.2 case) */
struct CountingHandler : MidiHandler {
	int *noteOns;
	explicit CountingHandler(int *counter) : noteOns(counter) {}
	void noteOn(uint8_t, uint8_t, uint8_t) override { ++*noteOns; }
	void noteOff(uint8_t, uint8_t, uint8_t) override {}
};

/** A handler that overrides only the two pure virtuals: every other hook is the (empty) base class default */
struct MinimalHandler : MidiHandler {
	int noteOns = 0;
	int noteOffs = 0;
	void noteOn(uint8_t, uint8_t, uint8_t) override { ++noteOns; }
	void noteOff(uint8_t, uint8_t, uint8_t) override { ++noteOffs; }
};

} // namespace

TEST_SUITE("MidiHandler") {

// ======================================================================= R2.1 note on

TEST_CASE_FIXTURE(MidiFixture, "[R2.1 midiRx] a handler starts with sysexState cleared") {
	MidiProbe probe;
	CHECK_FALSE(probe.sysex());
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.1 midiRx] note on 09 9n kk vv with velocity above 0 calls noteOn(n, kk, vv) on every channel") {
	for (int chan = 0; chan < 16; chan++) {
		MidiProbe probe;
		rx(probe, noteOnPkt(chan, 60, 100));
		CAPTURE(chan);
		CHECK(onlyHook(probe, hook("noteOn", chan, 60, 100)));
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.1 midiRx] note on with velocity 1 (the smallest note on) calls noteOn, not noteOff") {
	MidiProbe probe;
	rx(probe, noteOnPkt(0, 60, 1));
	CHECK(onlyHook(probe, hook("noteOn", 0, 60, 1)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.1 midiRx] note on with velocity 127 and note 127 keeps the 7 bit maximum values") {
	MidiProbe probe;
	rx(probe, noteOnPkt(15, 127, 127));
	CHECK(onlyHook(probe, hook("noteOn", 15, 127, 127)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.1 midiRx] note on with note 0 is passed on as note 0") {
	MidiProbe probe;
	rx(probe, noteOnPkt(2, 0, 64));
	CHECK(onlyHook(probe, hook("noteOn", 2, 0, 64)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.1 midiRx] note on with velocity 0 is a note off: noteOff(n, kk, 0) on every channel and no noteOn") {
	// MS: "Note On with velocity 0 is used for Note Off"
	for (int chan = 0; chan < 16; chan++) {
		MidiProbe probe;
		rx(probe, noteOnPkt(chan, 72, 0));
		CAPTURE(chan);
		CHECK(onlyHook(probe, hook("noteOff", chan, 72, 0)));
	}
}

// ======================================================================= R2.2 note off

TEST_CASE_FIXTURE(MidiFixture, "[R2.2 midiRx] note off 08 8n kk vv calls noteOff(n, kk, vv) with the release velocity on every channel") {
	for (int chan = 0; chan < 16; chan++) {
		MidiProbe probe;
		rx(probe, noteOffPkt(chan, 48, 33));
		CAPTURE(chan);
		CHECK(onlyHook(probe, hook("noteOff", chan, 48, 33)));
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.2 midiRx] note off with release velocity 127 passes 127 on") {
	MidiProbe probe;
	rx(probe, noteOffPkt(1, 100, 127));
	CHECK(onlyHook(probe, hook("noteOff", 1, 100, 127)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.2 midiRx] note off with release velocity 0 is a note off and never a note on") {
	MidiProbe probe;
	rx(probe, noteOffPkt(1, 100, 0));
	CHECK(onlyHook(probe, hook("noteOff", 1, 100, 0)));
}

// ======================================================================= R2.3 control change, program change

TEST_CASE_FIXTURE(MidiFixture, "[R2.3 midiRx] control change 0B Bn cc vv calls controlChange(n, cc, vv) on every channel") {
	for (int chan = 0; chan < 16; chan++) {
		MidiProbe probe;
		rx(probe, pkt(0, 0xB, 0xB0 | chan, 7, 99));
		CAPTURE(chan);
		CHECK(onlyHook(probe, hook("controlChange", chan, 7, 99)));
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.3 midiRx] control change 123 (all notes off) with value 0 reaches controlChange unchanged") {
	MidiProbe probe;
	rx(probe, pkt(0, 0xB, 0xB4, 123, 0));
	CHECK(onlyHook(probe, hook("controlChange", 4, 123, 0)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.3 midiRx] program change 0C Cn pp 00 calls programChange(n, pp) on every channel") {
	for (int chan = 0; chan < 16; chan++) {
		MidiProbe probe;
		rx(probe, pkt(0, 0xC, 0xC0 | chan, 42, 0));
		CAPTURE(chan);
		CHECK(onlyHook(probe, hook("programChange", chan, 42, 0)));
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.3 midiRx] program change ignores the third data byte of the packet") {
	// UM table 4-1: CIN 0xC has two MIDI bytes, the last packet byte is padding (zero) and is not part of the message
	MidiProbe probe;
	rx(probe, pkt(0, 0xC, 0xC3, 42, 0x55));
	CHECK(onlyHook(probe, hook("programChange", 3, 42, 0)));
}

// ======================================================================= R2.4 pitch bend

// HD: value = ((lsb & 0x7F) | ((msb & 0x7F) << 7)) - 8192

TEST_CASE_FIXTURE(MidiFixture, "[R2.4 midiRx] pitch bend 00 40 (centre 0x2000) gives 0") {
	// 0x00 | (0x40 << 7) = 8192; 8192 - 8192 = 0
	MidiProbe probe;
	rx(probe, pkt(0, 0xE, 0xE0, 0x00, 0x40));
	CHECK(onlyHook(probe, hook("pitchBend", 0, 0)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.4 midiRx] pitch bend 00 00 (lowest) gives -8192") {
	// 0 - 8192
	MidiProbe probe;
	rx(probe, pkt(0, 0xE, 0xE0, 0x00, 0x00));
	CHECK(onlyHook(probe, hook("pitchBend", 0, -8192)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.4 midiRx] pitch bend 7F 7F (highest) gives +8191") {
	// 0x7F | (0x7F << 7) = 16383; 16383 - 8192 = 8191
	MidiProbe probe;
	rx(probe, pkt(0, 0xE, 0xE0, 0x7F, 0x7F));
	CHECK(onlyHook(probe, hook("pitchBend", 0, 8191)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.4 midiRx] pitch bend 01 00 gives -8191 (the first data byte is the least significant)") {
	// 1 - 8192 = -8191
	MidiProbe probe;
	rx(probe, pkt(0, 0xE, 0xE0, 0x01, 0x00));
	CHECK(onlyHook(probe, hook("pitchBend", 0, -8191)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.4 midiRx] pitch bend 00 01 gives -8064 (the second data byte is the most significant)") {
	// (1 << 7) - 8192 = 128 - 8192 = -8064
	MidiProbe probe;
	rx(probe, pkt(0, 0xE, 0xE0, 0x00, 0x01));
	CHECK(onlyHook(probe, hook("pitchBend", 0, -8064)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.4 midiRx] pitch bend 55 2A (a mixed value) gives 0x55 + 0x2A * 128 - 8192 = -2731") {
	// 0x55 = 85, 0x2A = 42: 85 + 5376 = 5461; 5461 - 8192 = -2731
	MidiProbe probe;
	rx(probe, pkt(0, 0xE, 0xE0, 0x55, 0x2A));
	CHECK(onlyHook(probe, hook("pitchBend", 0, -2731)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.4 midiRx] pitch bend is delivered with the channel of the status byte on every channel") {
	for (int chan = 0; chan < 16; chan++) {
		MidiProbe probe;
		rx(probe, pkt(0, 0xE, 0xE0 | chan, 0x00, 0x60)); // (0x60 << 7) - 8192 = 12288 - 8192 = 4096
		CAPTURE(chan);
		CHECK(onlyHook(probe, hook("pitchBend", chan, 4096)));
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.4 midiRx] characterization: bit 7 of the pitch bend data bytes is masked off") {
	// MS: data bytes are 7 bit, so bit 7 is not part of the value; the firmware masks with 0x7f (MidiHandler.cpp:99).
	// The documents do not say what a receiver does with a malformed byte: FW, NEEDS REVIEW.
	// HD: lsb 0x80 -> 0, msb 0xC0 -> 0x40: 0 | (0x40 << 7) - 8192 = 0
	MidiProbe probe;
	rx(probe, pkt(0, 0xE, 0xE0, 0x80, 0xC0));
	CHECK(onlyHook(probe, hook("pitchBend", 0, 0)));
}

// ======================================================================= R2.5 timing clock

TEST_CASE_FIXTURE(MidiFixture, "[R2.5 midiRx] the timing clock 0F F8 00 00 calls midiTick once and no other hook") {
	MidiProbe probe;
	rx(probe, pkt(0, 0xF, 0xF8));
	CHECK(onlyHook(probe, hook("midiTick", 0, 0)));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.5 midiRx] the timing clock is recognised on every cable number 0..15") {
	for (int cable = 0; cable < 16; cable++) {
		MidiProbe probe;
		rx(probe, pkt(cable, 0xF, 0xF8));
		CAPTURE(cable);
		CHECK(onlyHook(probe, hook("midiTick", 0, 0)));
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.5 midiRx] three timing clock packets call midiTick three times") {
	MidiProbe probe;
	rx(probe, pkt(0, 0xF, 0xF8));
	rx(probe, pkt(0, 0xF, 0xF8));
	rx(probe, pkt(1, 0xF, 0xF8));
	CHECK(probe.count("midiTick") == 3);
	CHECK(probe.hooks.size() == 3);
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.5 midiRx] the timing clock does not change sysexState") {
	MidiProbe probe;
	rx(probe, pkt(0, 0x4, 0xF0, 0x00, 0x01));
	REQUIRE(probe.sysex());
	rx(probe, pkt(0, 0xF, 0xF8));
	CHECK(probe.sysex());
}

// ======================================================================= R2.6 SysEx packets (UM table 4-2)

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] SysEx start 04 F0 00 01 calls otherPacket with the exact bytes and sets sysexState") {
	MidiProbe probe;
	rx(probe, pkt(0, 0x4, 0xF0, 0x00, 0x01));
	CHECK(onlyHook(probe, otherHook({0x04, 0xF0, 0x00, 0x01})));
	CHECK(probe.sysex());
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] SysEx end 05 F7 00 00 calls otherPacket with the exact bytes and clears sysexState") {
	MidiProbe probe;
	rx(probe, pkt(0, 0x4, 0xF0, 0x00, 0x01));
	REQUIRE(probe.sysex());
	probe.hooks.clear();
	rx(probe, pkt(0, 0x5, 0xF7));
	CHECK(onlyHook(probe, otherHook({0x05, 0xF7, 0x00, 0x00})));
	CHECK_FALSE(probe.sysex());
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] SysEx data packet 04 02 03 04 calls otherPacket, never a note hook, and keeps sysexState") {
	MidiProbe probe;
	rx(probe, pkt(0, 0x4, 0xF0, 0x00, 0x01));
	probe.hooks.clear();
	rx(probe, pkt(0, 0x4, 0x02, 0x03, 0x04));
	CHECK(onlyHook(probe, otherHook({0x04, 0x02, 0x03, 0x04})));
	CHECK(probe.sysex());
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] a note on arriving during a SysEx leaves sysexState unchanged") {
	MidiProbe probe;
	rx(probe, pkt(0, 0x4, 0xF0, 0x00, 0x01));
	REQUIRE(probe.sysex());
	rx(probe, noteOnPkt(0, 60, 100));
	CHECK(probe.sysex());
	CHECK(probe.count("noteOn") == 1);
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] a control change arriving during a SysEx leaves sysexState unchanged") {
	MidiProbe probe;
	rx(probe, pkt(0, 0x4, 0xF0, 0x00, 0x01));
	REQUIRE(probe.sysex());
	rx(probe, pkt(0, 0xB, 0xB0, 7, 100));
	CHECK(probe.sysex());
	CHECK(probe.count("controlChange") == 1);
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] SysEx data bytes that look like status values when the high bit is ignored never reach a note hook") {
	// Data bytes are below 0x80 (MS): every value 0x00..0x7F in byte 1 of a CIN 4 packet. The values 0x00..0x7F have the
	// high nibble 0x0..0x7, which is none of 0x80 / 0x90 / 0xB0 / 0xC0 / 0xD0 / 0xE0, so no note, cc, pc or pb hook is called.
	MidiProbe probe;
	for (int b = 0; b < 0x80; b++) {
		rx(probe, pkt(0, 0x4, b, 0x7F, 0x7F));
	}
	CHECK(noteHooks(probe) == 0);
	CHECK(probe.count("controlChange") == 0);
	CHECK(probe.count("programChange") == 0);
	CHECK(probe.count("pitchBend") == 0);
	CHECK(probe.count("otherPacket") == 128);
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] SysEx end packet 06 02 F7 00 (two bytes) calls otherPacket with the exact bytes") {
	MidiProbe probe;
	rx(probe, pkt(0, 0x6, 0x02, 0xF7, 0x00));
	CHECK(onlyHook(probe, otherHook({0x06, 0x02, 0xF7, 0x00})));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] SysEx end packet 07 02 03 F7 (three bytes) calls otherPacket with the exact bytes") {
	MidiProbe probe;
	rx(probe, pkt(0, 0x7, 0x02, 0x03, 0xF7));
	CHECK(onlyHook(probe, otherHook({0x07, 0x02, 0x03, 0xF7})));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] the complete empty SysEx 06 F0 F7 00 calls otherPacket and no note hook") {
	MidiProbe probe;
	rx(probe, pkt(0, 0x6, 0xF0, 0xF7, 0x00));
	CHECK(onlyHook(probe, otherHook({0x06, 0xF0, 0xF7, 0x00})));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] a SysEx with one data byte 07 F0 55 F7 calls otherPacket and no note hook") {
	MidiProbe probe;
	rx(probe, pkt(0, 0x7, 0xF0, 0x55, 0xF7));
	CHECK(onlyHook(probe, otherHook({0x07, 0xF0, 0x55, 0xF7})));
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.6 midiRx] the SysEx packets of table 4-2 call otherPacket in order, one per packet") {
	MidiProbe probe;
	const std::vector<Packet> seq = {pkt(2, 0x4, 0xF0, 0x01, 0x02), pkt(2, 0x4, 0x03, 0x04, 0x05), pkt(2, 0x5, 0xF7)};
	for (const Packet &p : seq) {
		rx(probe, p);
	}
	REQUIRE(probe.hooks.size() == 3);
	CHECK(probe.hooks[0] == otherHook(seq[0]));
	CHECK(probe.hooks[1] == otherHook(seq[1]));
	CHECK(probe.hooks[2] == otherHook(seq[2]));
	CHECK_FALSE(probe.sysex());
}

// ======================================================================= R2.7 sysexState after CIN 6 / 7 (B-MIDI-3)

// UM table 4-2: a SysEx that ends in byte 2 or 3 of the packet is complete. MidiHandler.cpp:41 clears sysexState only when
// byte 1 of the packet is 0xF7, so after these endings the state stays set.

TEST_CASE_FIXTURE(MidiFixture, "[R2.7 midiRx] sysexState is cleared after a SysEx that ends with 06 02 F7 00" * KNOWN_ISSUE("B-MIDI-3: sysexState stays set after a SysEx that ends in byte 2 of a CIN 6 packet")) {
	MidiProbe probe;
	rx(probe, pkt(0, 0x4, 0xF0, 0x00, 0x01));
	REQUIRE(probe.sysex());
	rx(probe, pkt(0, 0x6, 0x02, 0xF7, 0x00));
	CHECK_FALSE(probe.sysex());
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.7 midiRx] sysexState is cleared after a SysEx that ends with 07 02 03 F7" * KNOWN_ISSUE("B-MIDI-3: sysexState stays set after a SysEx that ends in byte 3 of a CIN 7 packet")) {
	MidiProbe probe;
	rx(probe, pkt(0, 0x4, 0xF0, 0x00, 0x01));
	REQUIRE(probe.sysex());
	rx(probe, pkt(0, 0x7, 0x02, 0x03, 0xF7));
	CHECK_FALSE(probe.sysex());
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.7 midiRx] sysexState is cleared after the complete SysEx 06 F0 F7 00" * KNOWN_ISSUE("B-MIDI-3: the packet 06 F0 F7 00 sets sysexState (byte 1 is F0) and nothing clears it")) {
	MidiProbe probe;
	rx(probe, pkt(0, 0x6, 0xF0, 0xF7, 0x00));
	CHECK_FALSE(probe.sysex());
}

// ======================================================================= R2.8 other messages

TEST_CASE_FIXTURE(MidiFixture, "[R2.8 midiRx] poly key pressure, system common and real-time F9-FF never call a note, controller, program, bend or clock hook") {
	MidiProbe probe;
	for (int chan = 0; chan < 16; chan++) {
		rx(probe, pkt(0, 0xA, 0xA0 | chan, 60, 90));
	}
	for (int status : {0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF}) {
		rx(probe, pkt(0, 0x5, status, 0x10, 0x20));
	}
	CHECK(noteHooks(probe) == 0);
	CHECK(probe.count("controlChange") == 0);
	CHECK(probe.count("programChange") == 0);
	CHECK(probe.count("pitchBend") == 0);
	CHECK(probe.count("midiTick") == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.8 midiRx] a handler that overrides only noteOn and noteOff survives every kind of packet and gets the two note hooks only for note on and note off") {
	MinimalHandler h;
	REQUIRE(midihandler == &h);
	uint8_t packets[][4] = {
		{0x0B, 0xB0, 7, 100},   // control change
		{0x0C, 0xC0, 5, 0},     // program change
		{0x0E, 0xE0, 0, 0x40},  // pitch bend
		{0x0F, 0xF8, 0, 0},     // clock
		{0x04, 0xF0, 1, 2},     // SysEx start
		{0x05, 0xF7, 0, 0},     // SysEx end
		{0x0A, 0xA0, 60, 90},   // poly pressure
		{0x0D, 0xD0, 33, 0},    // channel pressure
	};
	for (auto &pk : packets) {
		h.midiRx(0, pk);
	}
	CHECK(h.noteOns == 0);
	CHECK(h.noteOffs == 0);
	uint8_t on[4] = {0x09, 0x90, 60, 100};
	uint8_t off[4] = {0x08, 0x80, 60, 0};
	h.midiRx(0, on);
	h.midiRx(0, off);
	CHECK(h.noteOns == 1);
	CHECK(h.noteOffs == 1);
	CHECK(midihandler == &h);
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.8 midiRx] the base class noteOn and noteOff do nothing when called explicitly") {
	MinimalHandler h;
	h.MidiHandler::noteOn(0, 60, 100);
	h.MidiHandler::noteOff(0, 60, 0);
	CHECK(h.noteOns == 0);
	CHECK(h.noteOffs == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.8 midiRx] characterization: poly key pressure 0A An kk vv calls otherPacket with the exact bytes on every channel") {
	for (int chan = 0; chan < 16; chan++) {
		MidiProbe probe;
		const Packet p = pkt(0, 0xA, 0xA0 | chan, 60, 90);
		rx(probe, p);
		CAPTURE(chan);
		CHECK(onlyHook(probe, otherHook(p)));
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.8 midiRx] characterization: system common F1, F2, F3, F4, F5, F6 call otherPacket with the exact bytes") {
	for (int status : {0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6}) {
		MidiProbe probe;
		const Packet p = pkt(0, status == 0xF2 ? 0x3 : (status == 0xF6 ? 0x5 : 0x2), status, 0x11, 0x22);
		rx(probe, p);
		CAPTURE(status);
		CHECK(onlyHook(probe, otherHook(p)));
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.8 midiRx] characterization: real-time messages F9, FA, FB, FC, FD, FE, FF call otherPacket with the exact bytes") {
	for (int status : {0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF}) {
		MidiProbe probe;
		const Packet p = pkt(0, 0xF, status);
		rx(probe, p);
		CAPTURE(status);
		CHECK(onlyHook(probe, otherHook(p)));
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.8 midiRx] characterization: a data byte as byte 1 (a value below 0x80) calls otherPacket") {
	MidiProbe probe;
	const Packet p = pkt(0, 0x4, 0x3C, 0x40, 0x41);
	rx(probe, p);
	CHECK(onlyHook(probe, otherHook(p)));
}

// ======================================================================= R2.9 channel pressure

TEST_CASE_FIXTURE(MidiFixture, "[R2.9 midiRx] characterization: channel pressure 0D Dn vv 00 calls no hook on any channel") {
	// FW, NEEDS REVIEW (question 56): the case 0xD0 of MidiHandler.cpp:93 is empty
	for (int chan = 0; chan < 16; chan++) {
		MidiProbe probe;
		rx(probe, pkt(0, 0xD, 0xD0 | chan, 77, 0));
		CAPTURE(chan);
		CHECK(probe.hooks.empty());
	}
}

// ======================================================================= R2.10 cable number

TEST_CASE_FIXTURE(MidiFixture, "[R2.10 midiRx] characterization: the cable number does not change the decoded call (cables 0..15)") {
	// FW, NEEDS REVIEW (question 57): UM 5.1 would route by cable number, midiRx never reads the high nibble of byte 0
	MidiProbe reference;
	rx(reference, noteOnPkt(3, 60, 100, 0));
	rx(reference, noteOffPkt(3, 60, 20, 0));
	rx(reference, pkt(0, 0xB, 0xB3, 7, 8));
	rx(reference, pkt(0, 0xE, 0xE3, 0x10, 0x50));
	REQUIRE(reference.hooks.size() == 4);
	for (int cable = 1; cable < 16; cable++) {
		MidiProbe probe;
		rx(probe, noteOnPkt(3, 60, 100, cable));
		rx(probe, noteOffPkt(3, 60, 20, cable));
		rx(probe, pkt(cable, 0xB, 0xB3, 7, 8));
		rx(probe, pkt(cable, 0xE, 0xE3, 0x10, 0x50));
		CAPTURE(cable);
		CHECK(probe.hooks == reference.hooks);
	}
}

// ======================================================================= R2.11 unparsed stream (B-MIDI-4)

TEST_CASE_FIXTURE(MidiFixture, "[R2.11 midiRx] single byte packets with data bytes (CIN F) do not call noteOff or noteOn" * KNOWN_ISSUE("B-MIDI-4: the CIN is ignored, the packet 0F 90 00 00 is decoded as a note on with velocity 0 and calls noteOff(0, 0, 0)")) {
	// UM table 4-1 note 2: CIN 0xF is a single unparsed byte; the stream 90 3C 64 sent byte by byte is three such packets.
	// The first one carries byte 1 = 0x90, which midiRx takes for a status byte.
	MidiProbe probe;
	rx(probe, pkt(0, 0xF, 0x90));
	rx(probe, pkt(0, 0xF, 0x3C));
	rx(probe, pkt(0, 0xF, 0x64));
	CHECK(noteHooks(probe) == 0);
}

// ======================================================================= R2.13 LED

TEST_CASE_FIXTURE(MidiFixture, "[R2.13 midiRx] a received note on pulses the system LED once") {
	const uint32_t before = HostPlatform::leds().sysPulses;
	MidiProbe probe;
	rx(probe, noteOnPkt(0, 60, 100));
	CHECK(HostPlatform::leds().sysPulses == before + 1);
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.13 midiRx] every packet pulses the system LED once, also a timing clock and a packet that calls no hook") {
	MidiProbe probe;
	const uint32_t before = HostPlatform::leds().sysPulses;
	rx(probe, pkt(0, 0xF, 0xF8));
	CHECK(HostPlatform::leds().sysPulses == before + 1);
	rx(probe, pkt(0, 0xD, 0xD0, 5, 0)); // channel pressure: no hook
	CHECK(HostPlatform::leds().sysPulses == before + 2);
	rx(probe, pkt(0, 0x4, 0xF0, 1, 2)); // SysEx start
	CHECK(HostPlatform::leds().sysPulses == before + 3);
}

TEST_CASE_FIXTURE(MidiFixture, "[R2.13 midiRx] five packets pulse the system LED five times") {
	MidiProbe probe;
	const uint32_t before = HostPlatform::leds().sysPulses;
	for (int i = 0; i < 5; i++) {
		rx(probe, noteOffPkt(0, 60 + i, 0));
	}
	CHECK(HostPlatform::leds().sysPulses == before + 5);
}

// ======================================================================= R3 tud_midi_rx_cb and the global pointer

TEST_CASE_FIXTURE(MidiFixture, "[R3.1 tud_midi_rx_cb] one transfer of three packets is delivered in order with interface number 0") {
	startAndEnumerate();
	MidiProbe probe;
	REQUIRE(sendPackets({noteOnPkt(1, 60, 100), pkt(0, 0xB, 0xB1, 7, 90), noteOffPkt(1, 60, 0)}) == HostDcd::Handshake::Ack);
	REQUIRE(probe.hooks.size() == 3);
	CHECK(probe.hooks[0] == hook("noteOn", 1, 60, 100));
	CHECK(probe.hooks[1] == hook("controlChange", 1, 7, 90));
	CHECK(probe.hooks[2] == hook("noteOff", 1, 60, 0));
	CHECK(probe.hooks[0].itf == 0);
	CHECK(probe.hooks[1].itf == 0);
	CHECK(probe.hooks[2].itf == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.1 tud_midi_rx_cb] midiRx receives exactly the four bytes of each packet") {
	startAndEnumerate();
	MidiProbe probe;
	const std::vector<Packet> sent = {pkt(5, 0x9, 0x9A, 0x7F, 0x01), pkt(0, 0xF, 0xF8), pkt(15, 0x4, 0xF0, 0x01, 0x02), pkt(0, 0xE, 0xE0, 0x00, 0x40)};
	REQUIRE(sendPackets(sent) == HostDcd::Handshake::Ack);
	REQUIRE(probe.rxPackets.size() == sent.size());
	for (size_t i = 0; i < sent.size(); i++) {
		CAPTURE(i);
		CHECK(probe.rxPackets[i] == sent[i]);
	}
	CHECK(probe.lastItf == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.1 tud_midi_rx_cb] two successive transfers are delivered in the order they were sent") {
	startAndEnumerate();
	MidiProbe probe;
	REQUIRE(sendPackets({noteOnPkt(0, 10, 1)}) == HostDcd::Handshake::Ack);
	REQUIRE(sendPackets({noteOnPkt(0, 11, 2), noteOnPkt(0, 12, 3)}) == HostDcd::Handshake::Ack);
	REQUIRE(probe.hooks.size() == 3);
	CHECK(probe.hooks[0] == hook("noteOn", 0, 10, 1));
	CHECK(probe.hooks[1] == hook("noteOn", 0, 11, 2));
	CHECK(probe.hooks[2] == hook("noteOn", 0, 12, 3));
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.1 tud_midi_rx_cb] a received transfer pulses the system LED once per packet") {
	startAndEnumerate();
	MidiProbe probe;
	const uint32_t before = HostPlatform::leds().sysPulses;
	REQUIRE(sendPackets({noteOnPkt(0, 60, 1), noteOnPkt(0, 61, 1), noteOnPkt(0, 62, 1)}) == HostDcd::Handshake::Ack);
	CHECK(HostPlatform::leds().sysPulses == before + 3);
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.2 tud_midi_rx_cb] a handler registers itself in the global pointer") {
	CHECK(midihandler == nullptr);
	MidiProbe probe;
	CHECK(midihandler == &probe);
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.2 tud_midi_rx_cb] after the handler is destroyed the pointer is cleared") {
	{
		MidiProbe probe;
		REQUIRE(midihandler == &probe);
	}
	CHECK(midihandler == nullptr);
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.2 tud_midi_rx_cb] a packet received after the handler was destroyed is accepted, not delivered and does not crash") {
	startAndEnumerate();
	int noteOns = 0;
	{
		CountingHandler handler(&noteOns);
		REQUIRE(sendPackets({noteOnPkt(0, 60, 100)}) == HostDcd::Handshake::Ack);
		REQUIRE(noteOns == 1);
	}
	REQUIRE(midihandler == nullptr);
	CHECK(sendPackets({noteOnPkt(0, 61, 100)}) == HostDcd::Handshake::Ack);
	CHECK(noteOns == 1);
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.2 tud_midi_rx_cb] without a handler the callback returns without reading: a packet stays queued until a handler appears and the callback runs again") {
	startAndEnumerate();
	REQUIRE(sendPackets({noteOnPkt(0, 60, 100)}) == HostDcd::Handshake::Ack);
	MidiProbe probe;
	CHECK(midihandler == &probe); // the handler exists now; the earlier packet was not read while there was none
	tud_midi_rx_cb(0);
	REQUIRE(probe.hooks.size() == 1);
	CHECK(probe.hooks[0] == hook("noteOn", 0, 60, 100));
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.3 tud_midi_rx_cb] characterization: a second handler replaces the first in the global pointer and receives the packets") {
	startAndEnumerate();
	auto first = std::make_unique<MidiProbe>();
	auto second = std::make_unique<MidiProbe>();
	CHECK(midihandler == second.get());
	REQUIRE(sendPackets({noteOnPkt(0, 60, 100)}) == HostDcd::Handshake::Ack);
	CHECK(first->hooks.empty());
	CHECK(second->hooks.size() == 1);
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.3 tud_midi_rx_cb] characterization: destroying the first handler clears the pointer of the second, so no packet is delivered afterwards") {
	startAndEnumerate();
	auto first = std::make_unique<MidiProbe>();
	auto second = std::make_unique<MidiProbe>();
	first.reset();
	CHECK(midihandler == nullptr);
	CHECK(sendPackets({noteOnPkt(0, 60, 100)}) == HostDcd::Handshake::Ack);
	CHECK(second->hooks.empty());
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.4 tud_midi_rx_cb] packets sent after a handler appears are delivered although packets arrived while no handler existed" * KNOWN_ISSUE("B-MIDI-6: with no handler nothing reads the RX FIFO, _prep_out_transaction (64 bytes free needed) never re-arms the OUT endpoint: the host gets NAK, measured on the fake controller")) {
	// Experiment 2026-11-10: after one 4 byte transfer without a handler the FIFO holds 4 bytes, 60 < 64 free, endpoint stays unarmed.
	// Intended: the later packet is accepted and reaches the handler (UM section 4: the device must keep receiving).
	startAndEnumerate();
	REQUIRE(sendPackets({noteOnPkt(0, 60, 100)}) == HostDcd::Handshake::Ack);
	MidiProbe probe;
	const HostDcd::Handshake h = sendPackets({noteOnPkt(0, 61, 100)});
	CHECK(h == HostDcd::Handshake::Ack);
	REQUIRE(probe.count("noteOn") == 1);
	CHECK(probe.hooks[0] == hook("noteOn", 0, 61, 100));
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.5 tud_midi_rx_cb] a burst of 17 packets (68 bytes, two transfers of 64 and 4 bytes) is delivered completely and in order") {
	startAndEnumerate();
	MidiProbe probe;
	std::vector<Packet> burst;
	for (int i = 0; i < 17; i++) {
		burst.push_back(noteOnPkt(i % 16, 20 + i, 1 + i));
	}
	REQUIRE(sendPackets(burst) == HostDcd::Handshake::Ack);
	REQUIRE(probe.hooks.size() == 17);
	for (int i = 0; i < 17; i++) {
		CAPTURE(i);
		CHECK(probe.hooks[i] == hook("noteOn", i % 16, 20 + i, 1 + i));
	}
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.5 tud_midi_rx_cb] a second burst of 17 packets works after the first") {
	startAndEnumerate();
	MidiProbe probe;
	std::vector<Packet> burst;
	for (int i = 0; i < 17; i++) {
		burst.push_back(noteOnPkt(0, 30 + i, 5));
	}
	REQUIRE(sendPackets(burst) == HostDcd::Handshake::Ack);
	REQUIRE(probe.hooks.size() == 17);
	probe.hooks.clear();
	REQUIRE(sendPackets(burst) == HostDcd::Handshake::Ack);
	REQUIRE(probe.hooks.size() == 17);
	CHECK(probe.hooks[0] == hook("noteOn", 0, 30, 5));
	CHECK(probe.hooks[16] == hook("noteOn", 0, 46, 5));
}

TEST_CASE_FIXTURE(MidiFixture, "[R3.5 tud_midi_rx_cb] a full transfer of 16 packets (64 bytes) is delivered completely and the next transfer works") {
	startAndEnumerate();
	MidiProbe probe;
	std::vector<Packet> burst;
	for (int i = 0; i < 16; i++) {
		burst.push_back(noteOnPkt(0, 40 + i, 9));
	}
	REQUIRE(sendPackets(burst) == HostDcd::Handshake::Ack);
	CHECK(probe.hooks.size() == 16);
	REQUIRE(sendPackets({noteOffPkt(0, 40, 0)}) == HostDcd::Handshake::Ack);
	REQUIRE(probe.hooks.size() == 17);
	CHECK(probe.hooks[16] == hook("noteOff", 0, 40, 0));
}

} // TEST_SUITE
