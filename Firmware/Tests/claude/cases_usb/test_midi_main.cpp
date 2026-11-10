/*
 * MidiMain: tests of the MIDI main class (P17, T2, writer MMN; Tests/claude/charters/driver/MIDI_REQUIREMENTS.md).
 *
 * Requirements covered: R1.1-R1.2 and R1.4 (identity, chooser, usbInit; R1.3 is a build check covered by test_midi_setup.cpp),
 * R4.1-R4.6 (note handling through the virtual hooks), R5.1-R5.9 (the synthesis MidiMain::play driven by the update event of
 * htim9), R6.1-R6.8 (construction, driver call order, commands, life cycle). A note is observable only through the calls the
 * recording double of the TMC4671 interface (fakes/FakeTmcPhase.h) sees after MidiMainFixture::tick(): setUdUq and setPhiE_ext.
 *
 * Oracles: MS = MIDI 1.0 Detailed Specification 4.2.1 (channel mode messages 120 / 121 / 123, 7 bit data bytes), HD = hand
 * derivation in a comment, FW = the firmware's own behaviour (`characterization:`), IF = the FFBoardMain / TimerHandler /
 * CommandHandler interfaces. The USB class definition is not used here (no USB in this file).
 *
 * HD of the phase values (used in many cases). play() makes, per tick k (100 microseconds = 1e-4 s, counter += 1e-4 * pitchbend):
 *     p = counter * f(note),   f(note) = 440 * 2^((note - 69) / 12)   (the table value; A4 = 440 Hz is not in the supplied documents)
 *     val = int16_t(sinf(pi * p) * (velocity / 127) * range)            (truncation toward zero; range default 0x3fff, power default 2500)
 * Table, computed in double (python, math.sin) for velocity 127 and range 0x3fff unless noted, tick 1, 2, 3, 4, 5:
 *     note 69 (440 Hz)          2257  4471  6600  8603 10442
 *     note 57 (220 Hz)          1131  2257  3372  4471  5549
 *     note 60 (261.63 Hz)       1345  2680  3998  5289  6544
 *     note 0  (8.1758 Hz)         42    84   126   168   210
 *     note 127 (12543.85 Hz)  -11743 16376                       (the sine wraps: pi * 12543.85 * 1e-4 = 3.94 rad)
 *     note 69, velocity 64      1137  2253  3326  4335  5262
 *     note 69, velocity 1         17    35    51    67    82
 *     note 69, range 0x7fff     4514  8943 13202 17208 20886
 *     note 69, range 0x1000      564  1118  1650  2151  2610
 *     note 69, ticks 20 21 22 23: 6030 3874 1644 -617 (zero crossing between tick 22 and 23: 440 * t = 1 for t = 22.7 ticks)
 * The firmware runs the same arithmetic in float (counter, volume, sinf), so every comparison allows +-1 count.
 *
 * Host versus target: no type of a different size is involved; the counter is a float that accumulates 1e-4 per tick (the
 * relative error of 6e-8 per addition is far below one count of the values above); the float to int16_t conversion is defined
 * for all values of this file except the two KNOWN_ISSUE_UB cases (B-MIDI-1 and the range case of B-MIDI-5); `sinf(M_PI * p)`
 * multiplies a float by a double, with the same IEEE result on the Cortex-M4F. The race between the USB task (note hooks) and
 * the timer interrupt (play) cannot be shown on the host (NEEDS REVIEW by code reading only).
 *
 * Characterizations (NEEDS REVIEW): the order of the driver calls of the constructor (R6.2), the choice sin(pi * f * t) with
 * the half frequency (R5.4, R5.9: owner question 54), the octave pitch bend range (R4.5, question 55), the mode messages 124-127
 * (R4.4, question 56), "only the last note of channel 0 is played" (R5.6), the timer that keeps running after the destruction
 * (R6.7), the table end notes 0 and 127 (R5.4 family), a power value above 32767 reaching setUdUq (see below, B-MIDI-5 is a
 * KNOWN_ISSUE, not a characterization).
 *
 * KNOWN_ISSUE: B-MIDI-2 (R4.5, a note started after a pitch bend ignores it), B-MIDI-1 (R5.8, KNOWN_ISSUE_UB, a note number
 * above 127 reads outside noteToFreq), B-MIDI-5 (R6.6, range above 0x7fff is accepted; and the KNOWN_ISSUE_UB case with 0x100000),
 * new B-MIDI-5 (R6.5: a power above 32767 reaches setUdUq(int16_t) as a negative value).
 */
#include "doctest.h"
#include "MidiFixture.h"
#include "TestHelpers.h"
#include "CommandHelpers.h"

#include "ClassChooser.h"
#include "TimerHandler.h"
#include "mainclass_chooser.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

extern ClassChooser<FFBoardMain> mainchooser; // cppmain.cpp
extern uint32_t SystemCoreClock;               // host_target.cpp

namespace {

using FakeTmcPhase::Call;

const std::vector<Call> &calls() { return FakeTmcPhase::state().calls; }
void forget() { FakeTmcPhase::state().calls.clear(); }

/** The arguments of all setPhiE_ext calls in order */
std::vector<int> phiValues() {
	std::vector<int> out;
	for (const Call &c : calls()) {
		if (c.name == "setPhiE_ext") {
			out.push_back(static_cast<int>(c.a));
		}
	}
	return out;
}

/** Restores SystemCoreClock at the end of a test */
struct CoreClockGuard {
	uint32_t saved = SystemCoreClock;
	~CoreClockGuard() { SystemCoreClock = saved; }
};

/** Resets the global usb_device pointer at the end of a test even when a REQUIRE fails (the device is owned by MidiMain::usbdev) */
struct UsbDeviceGuard {
	~UsbDeviceGuard() { usb_device = nullptr; }
};

/** MidiMainFixture with the MidiMain already built and the constructor's driver calls forgotten */
struct NoteFixture : MidiMainFixture {
	MidiMain *m = nullptr;

	NoteFixture() {
		m = &createMidiMain();
		forget();
	}
	/** n update events; returns the setPhiE_ext values of exactly these events. The recorded calls stay for inspection */
	std::vector<int> ticks(uint32_t n) {
		forget();
		REQUIRE(tick(n) == n);
		return phiValues();
	}
	/** One update event, returns the single setPhiE_ext value (REQUIREs there is exactly one) */
	int phi1() {
		const std::vector<int> v = ticks(1);
		REQUIRE(v.size() == 1);
		return v[0];
	}
	int status(const std::string &name, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr) {
		return static_cast<int>(CommandHelpers::run(*m, name, type, val, replies));
	}
};

} // namespace

TEST_SUITE("MidiMain") {

// ============================================================================================ R1: identity, chooser, usbInit

TEST_CASE_FIXTURE(MidiMainFixture, "[R1.1 MidiMain] getInfo returns the name 'MIDI (TMC)' and the class id CLSID_MAIN_MIDI (0xD)") {
	MidiMain &m = createMidiMain();
	const ClassIdentifier id = m.getInfo();
	CHECK(std::string(id.name) == "MIDI (TMC)");
	CHECK(id.id == CLSID_MAIN_MIDI);
	CHECK(id.id == 0xD);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R1.1 MidiMain] the class is a debug class, not shown to normal users") {
	MidiMain &m = createMidiMain();
	CHECK(m.getInfo().visibility == ClassVisibility::debug);
	CHECK(MidiMain::info.visibility == ClassVisibility::debug);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R1.1 MidiMain] the class is creatable") {
	CHECK(MidiMain::isCreatable());
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R1.1 MidiMain] the help string is 'Plays MIDI via TMC4671'") {
	MidiMain &m = createMidiMain();
	CHECK(m.getHelpstring() == "Plays MIDI via TMC4671");
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R1.2 MidiMain] the main class registry lists MidiMain exactly once with the id 0xD") {
	int found = 0;
	for (const class_entry<FFBoardMain> &e : class_registry) {
		if (std::string(e.info.name) == "MIDI (TMC)") {
			found++;
			CHECK(e.selectionId == 0xD);
			CHECK(e.info.id == CLSID_MAIN_MIDI);
		}
	}
	CHECK(found == 1);
	CHECK(mainchooser.isValidClassId(0xD));
	CHECK(mainchooser.isCreatable(0xD));
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R1.2 MidiMain] creating class 0xD through the chooser gives a working MidiMain that plays a note") {
	std::unique_ptr<FFBoardMain> created(mainchooser.Create(0xD));
	REQUIRE(created != nullptr);
	CHECK(created->getSelectionID() == 0xD);
	CHECK(created->getInfo().id == CLSID_MAIN_MIDI);
	MidiMain *mm = dynamic_cast<MidiMain *>(created.get());
	REQUIRE(mm != nullptr);
	forget();
	mm->noteOn(0, 69, 127);
	CHECK(tick(1) == 1);
	CHECK(FakeTmcPhase::count("setUdUq") == 1);
	CHECK(FakeTmcPhase::count("setPhiE_ext") == 1);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R1.2 MidiMain] the chooser lists '13:1:MIDI (TMC)' when the debug mode is on") {
	SystemCommands::debugMode = true;
	std::vector<CommandReply> replies;
	mainchooser.replyAvailableClasses(replies);
	bool found = false;
	for (const CommandReply &r : replies) {
		found = found || r.reply == "13:1:MIDI (TMC)";
	}
	CHECK(found);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R1.2 MidiMain] the chooser does not list the debug class MidiMain while the debug mode is off") {
	SystemCommands::debugMode = false;
	std::vector<CommandReply> replies;
	mainchooser.replyAvailableClasses(replies);
	REQUIRE(!replies.empty()); // the other classes are listed, so the loop below is not vacuous
	for (const CommandReply &r : replies) {
		CHECK(r.reply.find("MIDI (TMC)") == std::string::npos);
	}
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R1.4 MidiMain] usbInit registers a USB device with the composite device descriptor and the MIDI configuration") {
	MidiMain &m = createMidiMain();
	UsbDeviceGuard guard;
	m.usbInit();
	REQUIRE(usb_device != nullptr);
	CHECK(usb_device->getUsbDeviceDesc() == reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite));
	CHECK(usb_device->getUsbConfigurationDesc(0) == usb_cdc_midi_conf);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R1.4 MidiMain] the USB device of usbInit has the default string table (product string)") {
	MidiMain &m = createMidiMain();
	UsbDeviceGuard guard;
	m.usbInit();
	REQUIRE(usb_device != nullptr);
	const uint16_t *s = usb_device->getUsbStringDesc(usb_devdesc_ffboard_composite.iProduct, 0x0409);
	REQUIRE(s != nullptr);
	const std::string product = usb_ffboard_strings_default.product;
	// string descriptor: bLength = 2 * chars + 2 (truncated to the 31 characters of the buffer), bDescriptorType 3, UTF-16 characters
	const size_t chars = std::min<size_t>(product.size(), USB_STRING_DESC_BUF_SIZE - 1);
	CHECK((s[0] & 0xFF) == 2 * chars + 2);
	CHECK((s[0] >> 8) == TUSB_DESC_STRING);
	for (size_t i = 0; i < chars; i++) {
		CHECK(s[1 + i] == static_cast<uint16_t>(product[i]));
	}
}

// ============================================================================================ R4: note handling

TEST_CASE_FIXTURE(NoteFixture, "[R4.1 MidiMain] a note on adds one note that the next tick plays") {
	m->noteOn(0, 69, 127);
	CHECK_NEAR(phi1(), 2257, 1); // HD table, note 69 tick 1
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.1 MidiMain] with two notes the last one plays") {
	m->noteOn(0, 57, 127);
	m->noteOn(0, 69, 127);
	CHECK_NEAR(phi1(), 2257, 1); // note 69 tick 1 (note 57 would give 1131)
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.1 MidiMain] the note off of the playing note falls back to the previous note, which starts at its own counter") {
	m->noteOn(0, 57, 127);
	m->noteOn(0, 69, 127);
	m->noteOff(0, 69, 0);
	CHECK_NEAR(phi1(), 1131, 1); // note 57 never played: its counter is 0, tick 1 of note 57
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.1 MidiMain] the previous note continues where it stopped after the later note is released") {
	m->noteOn(0, 57, 127);
	const std::vector<int> first = ticks(3); // 57: ticks 1..3
	REQUIRE(first.size() == 3);
	m->noteOn(0, 69, 127);
	ticks(2);
	m->noteOff(0, 69, 0);
	CHECK_NEAR(phi1(), 4471, 1); // note 57 tick 4 (HD table): its counter stayed at 3 ticks while 69 played
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.1 MidiMain] a second note on of the same note restarts the counter at the first tick value") {
	m->noteOn(0, 69, 127);
	const std::vector<int> v = ticks(5);
	REQUIRE(v.size() == 5);
	CHECK_NEAR(v[4], 10442, 1);
	m->noteOn(0, 69, 127);
	CHECK_NEAR(phi1(), 2257, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.1 MidiMain] a repeated note on leaves one entry: one note off silences the channel") {
	m->noteOn(0, 69, 127);
	m->noteOn(0, 69, 127);
	ticks(1); // running
	m->noteOff(0, 69, 0);
	ticks(1);
	CHECK(phiValues().empty());
	CHECK(FakeTmcPhase::count("setUdUq") == 1); // the stop: setUdUq(0, 0)
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.1 MidiMain] the note off removes only its note, the other one keeps playing") {
	m->noteOn(0, 57, 127);
	m->noteOn(0, 69, 127);
	m->noteOff(0, 57, 0);
	CHECK_NEAR(phi1(), 2257, 1); // 69 still plays from its counter 0
	CHECK_NEAR(phi1(), 4471, 1); // tick 2
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.1 MidiMain] the note off of a note that is not playing changes nothing") {
	m->noteOn(0, 69, 127);
	m->noteOff(0, 70, 0);
	CHECK_NEAR(phi1(), 2257, 1);
	CHECK_NEAR(phi1(), 4471, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.1 MidiMain] the note off with nothing playing does not start the driver") {
	m->noteOff(0, 69, 0);
	ticks(3);
	CHECK(calls().empty());
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.1 MidiMain] the velocity of the note on is the volume of that note") {
	m->noteOn(0, 69, 64);
	CHECK_NEAR(phi1(), 1137, 1); // HD table, velocity 64
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.2 MidiMain] a note on channel 1 does not start the driver") {
	m->noteOn(1, 69, 127);
	ticks(3);
	CHECK(calls().empty());
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.2 MidiMain] a note off of the same note number on channel 1 does not stop the note of channel 0") {
	m->noteOn(0, 69, 127);
	m->noteOn(1, 69, 127);
	m->noteOff(1, 69, 0);
	CHECK_NEAR(phi1(), 2257, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.2 MidiMain] a note on of the same note number on channel 1 does not restart the note of channel 0") {
	m->noteOn(0, 69, 127);
	ticks(1);
	m->noteOn(1, 69, 127);
	CHECK_NEAR(phi1(), 4471, 1); // channel 0 continues with tick 2
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.3 MidiMain] control change 120 (all sound off) clears the notes of the channel") {
	m->noteOn(0, 57, 127);
	m->noteOn(0, 69, 127);
	ticks(1);
	m->controlChange(0, 120, 0);
	ticks(1);
	CHECK(phiValues().empty());
	REQUIRE(calls().size() == 1);
	CHECK(calls()[0].name == "setUdUq");
	CHECK(calls()[0].a == 0);
	CHECK(calls()[0].b == 0);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.3 MidiMain] control change 121 (reset all controllers) clears the notes of the channel") {
	m->noteOn(0, 57, 127);
	m->noteOn(0, 69, 127);
	ticks(1);
	m->controlChange(0, 121, 0);
	ticks(1);
	CHECK(phiValues().empty());
	CHECK(FakeTmcPhase::count("setUdUq") == 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.3 MidiMain] control change 123 (all notes off) clears the notes of the channel") {
	m->noteOn(0, 57, 127);
	m->noteOn(0, 69, 127);
	ticks(1);
	m->controlChange(0, 123, 0);
	ticks(1);
	CHECK(phiValues().empty());
	CHECK(FakeTmcPhase::count("setUdUq") == 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.3 MidiMain] all notes off on channel 1 leaves the notes of channel 0 playing") {
	m->noteOn(0, 69, 127);
	m->noteOn(1, 69, 127);
	m->controlChange(1, 123, 0);
	m->controlChange(1, 120, 0);
	m->controlChange(1, 121, 0);
	CHECK_NEAR(phi1(), 2257, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.3 MidiMain] control changes that are not channel mode messages (0, 1, 7, 64, 119, 122) change nothing") {
	const uint8_t controllers[] = {0, 1, 7, 64, 119, 122};
	m->noteOn(0, 69, 127);
	uint32_t k = 0;
	for (uint8_t c : controllers) {
		INFO("controller " << int(c));
		m->controlChange(0, c, 127);
		k++;
		const std::vector<int> v = ticks(1);
		REQUIRE(v.size() == 1);
		const int expected[] = {2257, 4471, 6600, 8603, 10442, 12082}; // note 69 ticks 1..6, HD: int(sin(pi * 440 * k * 1e-4) * 16383)
		CHECK_NEAR(v[0], expected[k - 1], 1);
	}
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.4 MidiMain] characterization: control changes 124-127 (omni off / on, mono, poly) do not clear the notes") {
	for (uint8_t c = 124; c <= 127; c++) {
		INFO("controller " << int(c));
		m->noteOn(0, 69, 127);
		ticks(1);
		m->controlChange(0, c, 0);
		const std::vector<int> v = ticks(1);
		CHECK(v.size() == 1);
		CHECK(FakeTmcPhase::count("setUdUq") == 0);
		m->controlChange(0, 123, 0); // back to idle for the next controller
		ticks(1);
	}
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.5 MidiMain] characterization: a pitch bend of 0 keeps the pitch step 1") {
	m->noteOn(0, 69, 127);
	m->pitchBend(0, 0); // 2^(0 / 8192) = 1
	CHECK_NEAR(phi1(), 2257, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.5 MidiMain] characterization: a pitch bend of -8192 halves the pitch step (octave range)") {
	m->noteOn(0, 69, 127);
	m->pitchBend(0, -8192); // 2^(-1) = 0.5: counter 0.5e-4 per tick
	const std::vector<int> v = ticks(2);
	REQUIRE(v.size() == 2);
	CHECK_NEAR(v[0], 1131, 1); // HD: int(sin(pi * 440 * 0.5e-4) * 16383) = note 57 tick 1
	CHECK_NEAR(v[1], 2257, 1); // counter 1e-4 = tick 1 of the unbent note
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.5 MidiMain] characterization: a pitch bend of +4096 multiplies the pitch step by the square root of two") {
	m->noteOn(0, 69, 127);
	m->pitchBend(0, 4096); // 2^(0.5) = 1.41421356: counter 1.41421e-4 per tick
	const std::vector<int> v = ticks(2);
	REQUIRE(v.size() == 2);
	CHECK_NEAR(v[0], 3182, 1); // HD: int(sin(pi * 440 * 1.41421356e-4) * 16383) = 3182
	CHECK_NEAR(v[1], 6243, 1); // counter 2.82843e-4: 6243
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.5 MidiMain] characterization: a pitch bend of +8191 is the factor 2^(8191 / 8192), not exactly 2 (tick 100)") {
	m->noteOn(0, 69, 127);
	m->pitchBend(0, 8191); // 2^(8191 / 8192) = 1.99991551
	const std::vector<int> v = ticks(100);
	REQUIRE(v.size() == 100);
	// HD: counter 100 * 1e-4 * 1.99991551 = 0.0199991551, sin(pi * 440 * counter) * 16383 = 9660.x; the exact factor 2 would give 9629 (31 counts apart)
	CHECK_NEAR(v[99], 9660, 2);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.5 MidiMain] characterization: a pitch bend applies to all notes of the channel that exist at that moment") {
	m->noteOn(0, 57, 127);
	m->noteOn(0, 69, 127);
	m->pitchBend(0, 8192); // factor 2
	m->noteOff(0, 69, 0);
	CHECK_NEAR(phi1(), 2257, 1); // note 57 (220 Hz) at counter 2e-4: 220 * 2e-4 = 440 * 1e-4 -> value of tick 1 of note 69
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.5 MidiMain] a pitch bend of channel 1 does not change the notes of channel 0") {
	m->noteOn(0, 69, 127);
	m->pitchBend(1, 8192);
	CHECK_NEAR(phi1(), 2257, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.5 MidiMain] a note started after a pitch bend keeps the pitch bend of its channel" * KNOWN_ISSUE("B-MIDI-2: MidiMain.cpp:148-155 stores the pitch bend in the notes that exist; a later note on starts with pitchbend 1 (a bend is channel state, MS)")) {
	m->pitchBend(0, 8192); // factor 2 on channel 0, no note yet
	m->noteOn(0, 69, 127);
	// intended (MS: pitch bend is channel state): the new note plays with the factor 2, tick 1 = 4471 (HD: int(sin(pi * 440 * 2e-4) * 16383));
	// measured: the note starts with factor 1: 2257
	CHECK_NEAR(phi1(), 4471, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.6 MidiMain] programChange does nothing observable") {
	m->noteOn(0, 69, 127);
	ticks(1);
	forget();
	m->programChange(0, 5);
	CHECK(calls().empty());
	CHECK_NEAR(phi1(), 4471, 1); // the note continues with tick 2
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.6 MidiMain] midiTick does nothing observable") {
	m->noteOn(0, 69, 127);
	ticks(1);
	forget();
	m->midiTick();
	CHECK(calls().empty());
	CHECK_NEAR(phi1(), 4471, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R4.6 MidiMain] otherPacket does nothing observable") {
	m->noteOn(0, 69, 127);
	ticks(1);
	forget();
	uint8_t packet[4] = {0x04, 0xF0, 0x01, 0x02};
	m->otherPacket(packet);
	CHECK(calls().empty());
	CHECK_NEAR(phi1(), 4471, 1);
}

// ============================================================================================ R5: the synthesis

TEST_CASE_FIXTURE(NoteFixture, "[R5.1 MidiMain] timerElapsed with another timer handle plays nothing") {
	m->noteOn(0, 69, 127);
	TIM_HandleTypeDef other = {};
	forget();
	m->timerElapsed(&other);
	CHECK(calls().empty());
	CHECK_NEAR(phi1(), 2257, 1); // the counter did not advance
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.2 MidiMain] without a note the ticks make no driver call") {
	ticks(5);
	CHECK(calls().empty());
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.2 MidiMain] after the last note off exactly one setUdUq(0, 0) follows and then no more calls") {
	m->noteOn(0, 69, 127);
	ticks(2);
	m->noteOff(0, 69, 0);
	ticks(1);
	REQUIRE(calls().size() == 1);
	CHECK(calls()[0].name == "setUdUq");
	CHECK(calls()[0].a == 0);
	CHECK(calls()[0].b == 0);
	ticks(4);
	CHECK(calls().empty());
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.2 MidiMain] after all notes off (123) exactly one setUdUq(0, 0) follows and then no more calls") {
	m->noteOn(0, 69, 127);
	m->noteOn(0, 60, 127);
	ticks(2);
	m->controlChange(0, 123, 0);
	ticks(1);
	REQUIRE(calls().size() == 1);
	CHECK(calls()[0].name == "setUdUq");
	CHECK(calls()[0].a == 0);
	ticks(3);
	CHECK(calls().empty());
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.3 MidiMain] the first note makes one setUdUq(2500, 0) and then setPhiE_ext, in this order") {
	m->noteOn(0, 69, 127);
	ticks(1);
	REQUIRE(calls().size() == 2);
	CHECK(calls()[0].name == "setUdUq");
	CHECK(calls()[0].a == 2500); // default power
	CHECK(calls()[0].b == 0);
	CHECK(calls()[1].name == "setPhiE_ext");
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.3 MidiMain] while the note continues only setPhiE_ext is called, once per tick") {
	m->noteOn(0, 69, 127);
	const std::vector<int> v = ticks(5);
	CHECK(v.size() == 5);
	CHECK(FakeTmcPhase::count("setUdUq") == 1);
	CHECK(calls().size() == 6);
	ticks(3);
	CHECK(FakeTmcPhase::count("setUdUq") == 0);
	CHECK(FakeTmcPhase::count("setPhiE_ext") == 3);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.3 MidiMain] a second note while one plays does not call setUdUq again") {
	m->noteOn(0, 69, 127);
	ticks(2);
	m->noteOn(0, 60, 127);
	ticks(1);
	CHECK(FakeTmcPhase::count("setUdUq") == 0);
	CHECK(FakeTmcPhase::count("setPhiE_ext") == 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.3 MidiMain] a note after a pause makes a new setUdUq(2500, 0)") {
	m->noteOn(0, 69, 127);
	ticks(2);
	m->noteOff(0, 69, 0);
	ticks(2);
	m->noteOn(0, 69, 127);
	ticks(1);
	REQUIRE(calls().size() == 2);
	CHECK(calls()[0].name == "setUdUq");
	CHECK(calls()[0].a == 2500);
	CHECK(calls()[1].name == "setPhiE_ext");
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.4 MidiMain] characterization: A4 (note 69, velocity 127, range 0x3fff) gives 2257 at tick 1") {
	m->noteOn(0, 69, 127);
	// HD: counter 1e-4, p = 440 * 1e-4 = 0.044, sinf(pi * 0.044) = 0.137790, * 16383 = 2257.4 -> 2257 (the argument pi * f * t is the firmware's choice, question 54)
	CHECK_NEAR(phi1(), 2257, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.4 MidiMain] characterization: A4 (note 69, velocity 127, range 0x3fff) gives 4471 at tick 2") {
	m->noteOn(0, 69, 127);
	const std::vector<int> v = ticks(2);
	REQUIRE(v.size() == 2);
	// HD: sin(pi * 440 * 2e-4) = 0.272914, * 16383 = 4471.1 -> 4471
	CHECK_NEAR(v[1], 4471, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.4 MidiMain] characterization: A4 gives 6600, 8603 and 10442 at ticks 3, 4 and 5") {
	m->noteOn(0, 69, 127);
	const std::vector<int> v = ticks(5);
	REQUIRE(v.size() == 5);
	// HD (python, double): sin(pi * 440 * k * 1e-4) * 16383 truncated, k = 3, 4, 5
	CHECK_NEAR(v[2], 6600, 1);
	CHECK_NEAR(v[3], 8603, 1);
	CHECK_NEAR(v[4], 10442, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.4 MidiMain] characterization: note 60 gives 1345 and 2680 at ticks 1 and 2") {
	m->noteOn(0, 60, 127);
	const std::vector<int> v = ticks(2);
	REQUIRE(v.size() == 2);
	// HD: f = 440 * 2^(-9/12) = 261.6256 Hz, sin(pi * 261.6256 * 1e-4) * 16383 = 1345.x
	CHECK_NEAR(v[0], 1345, 1);
	CHECK_NEAR(v[1], 2680, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.4 MidiMain] characterization: note 0 (first entry of the table, 8.1758 Hz) gives 42 and 84 at ticks 1 and 2") {
	m->noteOn(0, 0, 127);
	const std::vector<int> v = ticks(2);
	REQUIRE(v.size() == 2);
	// HD: f = 440 * 2^(-69/12) = 8.17580 Hz, sin(pi * 8.1758 * 1e-4) * 16383 = 42.07
	CHECK_NEAR(v[0], 42, 1);
	CHECK_NEAR(v[1], 84, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.4 MidiMain] characterization: note 127 (last entry of the table, 12543.85 Hz) gives -11743 and 16376 at ticks 1 and 2") {
	m->noteOn(0, 127, 127);
	const std::vector<int> v = ticks(2);
	REQUIRE(v.size() == 2);
	// HD: f = 440 * 2^(58/12) = 12543.85 Hz; arguments pi * f * 1e-4 = 3.9406 rad and 7.8812 rad
	CHECK_NEAR(v[0], -11743, 1);
	CHECK_NEAR(v[1], 16376, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.5 MidiMain] velocity 64 scales the value by 64 / 127 (1137, 2253, 3326, 4335, 5262)") {
	m->noteOn(0, 69, 64);
	const std::vector<int> v = ticks(5);
	REQUIRE(v.size() == 5);
	// HD: int(sin(pi * 440 * k * 1e-4) * (64 / 127) * 16383)
	const int expected[] = {1137, 2253, 3326, 4335, 5262};
	for (int i = 0; i < 5; i++) {
		INFO("tick " << i + 1);
		CHECK_NEAR(v[i], expected[i], 1);
	}
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.5 MidiMain] velocity 1 scales the value by 1 / 127 (17 and 35 at ticks 1 and 2)") {
	m->noteOn(0, 69, 1);
	const std::vector<int> v = ticks(2);
	REQUIRE(v.size() == 2);
	CHECK_NEAR(v[0], 17, 1); // HD: 0.137790 / 127 * 16383 = 17.77 -> 17
	CHECK_NEAR(v[1], 35, 1); // 0.272914 / 127 * 16383 = 35.2 -> 35
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.5 MidiMain] velocity 0 as a direct call plays the note silently (value 0)") {
	m->noteOn(0, 69, 0); // MidiHandler maps velocity 0 to noteOff; the direct call still adds a note with volume 0
	const std::vector<int> v = ticks(2);
	REQUIRE(v.size() == 2);
	CHECK(v[0] == 0);
	CHECK(v[1] == 0);
	CHECK(FakeTmcPhase::count("setUdUq") == 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.5 MidiMain] the range command 0x7fff scales the value (4514, 8943)") {
	REQUIRE(status("range", CMDtype::set, 0x7fff) == static_cast<int>(CommandStatus::OK));
	m->noteOn(0, 69, 127);
	const std::vector<int> v = ticks(2);
	REQUIRE(v.size() == 2);
	CHECK_NEAR(v[0], 4514, 1); // HD: 0.137790 * 32767 = 4514.9 (4514)
	CHECK_NEAR(v[1], 8943, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.5 MidiMain] the range command 0x1000 scales the value (564, 1118)") {
	REQUIRE(status("range", CMDtype::set, 0x1000) == static_cast<int>(CommandStatus::OK));
	m->noteOn(0, 69, 127);
	const std::vector<int> v = ticks(2);
	REQUIRE(v.size() == 2);
	CHECK_NEAR(v[0], 564, 1); // HD: 0.137790 * 4096 = 564.4
	CHECK_NEAR(v[1], 1118, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.6 MidiMain] characterization: notes on channels 1 to 15 never cause a driver call") {
	for (uint8_t chan = 1; chan < 16; chan++) {
		INFO("channel " << int(chan));
		m->noteOn(chan, 69, 127);
	}
	ticks(3);
	CHECK(calls().empty());
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.6 MidiMain] characterization: with notes on channel 0 and 5 only the note of channel 0 is played") {
	m->noteOn(0, 57, 127);
	m->noteOn(5, 69, 127);
	CHECK_NEAR(phi1(), 1131, 1); // note 57 of channel 0 (note 69 of channel 5 would give 2257)
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.7 MidiMain] a pitch bend of +1 octave doubles the phase step: after 10 ticks the value of the unbent tick 20") {
	m->noteOn(0, 69, 127);
	m->pitchBend(0, 8192); // factor 2^(8192 / 8192) = 2 (the argument is outside the 14 bit range -8192..8191 but the hook accepts int16_t)
	const std::vector<int> v = ticks(10);
	REQUIRE(v.size() == 10);
	// HD: counter after 10 ticks 20e-4, sin(pi * 440 * 20e-4) = 0.368125, * 16383 = 6030.9 -> 6030
	CHECK_NEAR(v[9], 6030, 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.7 MidiMain] without a bend the value after 20 ticks is 6030") {
	m->noteOn(0, 69, 127);
	const std::vector<int> v = ticks(20);
	REQUIRE(v.size() == 20);
	CHECK_NEAR(v[19], 6030, 1); // HD as above: the same phase as 10 ticks at twice the step
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.8 MidiMain] a note number above 127 is ignored: the tick makes no phase call" * KNOWN_ISSUE_UB("B-MIDI-1: MidiMain.cpp:107 noteToFreq[note->note] reads outside the 128 entry table for the note 0xFF (MidiHandler does not mask)")) {
	m->noteOn(0, 0xFF, 0x7F);
	const std::vector<int> v = ticks(1);
	// lead decision L13: the intended behaviour is that an invalid (8 bit) note number is ignored. The assertion looks at the NUMBER of phase calls only, never at the
	// value computed from the out of range table read (that would depend on stale heap content): today play() always calls setPhiE_ext once, whatever the value is.
	CHECK(v.empty());
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.9 MidiMain] characterization: the output has its first zero crossing between ticks 22 and 23 (half of the note frequency)") {
	m->noteOn(0, 69, 127);
	const std::vector<int> v = ticks(25);
	REQUIRE(v.size() == 25);
	// HD: sin(pi * 440 * t) is zero when 440 * t is an integer: t = 1 / 440 s = 22.73 ticks of 100 microseconds -> positive at tick 22 (1644), negative at tick 23 (-617).
	// The note period of 1 / 440 s would put the first crossing at 1 / 880 s = 11.4 ticks; the firmware crosses at 22.7 (question 54).
	CHECK(v[21] > 0);
	CHECK(v[22] < 0);
}

TEST_CASE_FIXTURE(NoteFixture, "[R5.9 MidiMain] characterization: the output stays positive through ticks 1 to 22, no zero crossing at 1 / 880 s (ticks 11 and 12)") {
	m->noteOn(0, 69, 127);
	const std::vector<int> v = ticks(22);
	REQUIRE(v.size() == 22);
	for (size_t i = 0; i < v.size(); i++) {
		INFO("tick " << i + 1);
		CHECK(v[i] > 0);
	}
	CHECK(v[10] > 0); // tick 11: HD 0.998737 * 16383 = 16362
	CHECK(v[11] > 0); // tick 12: 0.996134 * 16383 = 16319
}

// ============================================================================================ R6: construction, commands, life cycle

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.1 MidiMain] construction programs the user timer with ARR 100") {
	createMidiMain();
	CHECK(htim9.Instance->ARR == 100);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.1 MidiMain] construction programs the prescaler SystemCoreClock / 1000000 - 1 (167 at 168 MHz)") {
	createMidiMain();
	CHECK(SystemCoreClock == 168000000); // the fake platform's clock: 168 MHz / 1 MHz - 1 = 167
	CHECK(htim9.Instance->PSC == 167);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.1 MidiMain] the prescaler follows SystemCoreClock (83 at 84 MHz)") {
	CoreClockGuard guard;
	SystemCoreClock = 84000000; // 84 MHz / 1 MHz - 1 = 83
	createMidiMain();
	CHECK(htim9.Instance->PSC == 83);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.1 MidiMain] construction sets CR1 bit 0 and starts the timer interrupt (state BUSY)") {
	createMidiMain();
	CHECK((htim9.Instance->CR1 & 1) == 1);
	CHECK(htim9.State == HAL_TIM_STATE_BUSY);
	CHECK(HostTarget::timerRunning(&htim9));
	CHECK(HostTarget::timerUpdateIrqEnabled(&htim9));
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.1 MidiMain] the update interrupt of htim9 reaches timerElapsed: a note is played on the event") {
	MidiMain &m = createMidiMain();
	m.noteOn(0, 69, 127);
	forget();
	CHECK(m.timer_update == &htim9); // the handle the object listens to is TIM_USER
	CHECK(tick(1) == 1);
	CHECK(FakeTmcPhase::count("setPhiE_ext") == 1);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.2 MidiMain] characterization: the constructor calls the driver in the order construct, setLimits, setEncoderType, setPhiEtype, setUdUq, initialize, setPhiEtype, setMotionMode") {
	createMidiMain();
	const std::vector<std::string> expected = {"construct",  "setLimits", "setEncoderType", "setPhiEtype",
											   "setUdUq", "initialize", "setPhiEtype", "setMotionMode"};
	CHECK(FakeTmcPhase::names() == expected);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.2 MidiMain] the encoder type is set to NONE (0)") {
	createMidiMain();
	REQUIRE(FakeTmcPhase::count("setEncoderType") == 1);
	for (const Call &c : calls()) {
		if (c.name == "setEncoderType") {
			CHECK(c.a == static_cast<int>(EncoderType_TMC::NONE));
			CHECK(c.a == 0);
		}
	}
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.2 MidiMain] the phi type is set to ext (1) twice") {
	createMidiMain();
	CHECK(FakeTmcPhase::count("setPhiEtype") == 2);
	for (const Call &c : calls()) {
		if (c.name == "setPhiEtype") {
			CHECK(c.a == static_cast<int>(PhiE::ext));
			CHECK(c.a == 1);
		}
	}
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.2 MidiMain] the voltage is set to ud 0, uq 0 once") {
	createMidiMain();
	REQUIRE(FakeTmcPhase::count("setUdUq") == 1);
	for (const Call &c : calls()) {
		if (c.name == "setUdUq") {
			CHECK(c.a == 0);
			CHECK(c.b == 0);
		}
	}
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.2 MidiMain] the motion mode is set to uqudext (8) with force") {
	createMidiMain();
	REQUIRE(FakeTmcPhase::count("setMotionMode") == 1);
	for (const Call &c : calls()) {
		if (c.name == "setMotionMode") {
			CHECK(c.a == static_cast<int>(MotionMode::uqudext));
			CHECK(c.a == 8);
			CHECK(c.b == 1);
		}
	}
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.2 MidiMain] the limits are the default limits of the driver") {
	createMidiMain();
	const TMC4671Limits limits;
	REQUIRE(FakeTmcPhase::count("setLimits") == 1);
	for (const Call &c : calls()) {
		if (c.name == "setLimits") {
			CHECK(c.a == limits.pid_uq_ud);
			CHECK(c.b == limits.pid_torque_flux);
		}
	}
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.2 MidiMain] slow SPI is switched off on the driver") {
	MidiMain &m = createMidiMain();
	REQUIRE(m.drv != nullptr);
	CHECK_FALSE(m.drv->allowSlowSPI);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.2 MidiMain] the constructor creates exactly one driver and calls nothing else") {
	createMidiMain();
	CHECK(FakeTmcPhase::state().constructed == 1);
	CHECK(FakeTmcPhase::count("initializeWithPower") == 0);
	CHECK(FakeTmcPhase::count("setPhiE_ext") == 0);
	CHECK(FakeTmcPhase::count("hasPower") == 0);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.3 MidiMain] a motor type NONE pulses the error LED once and construction continues") {
	FakeTmcPhase::state().motorType = 0;
	const uint32_t before = HostPlatform::leds().errPulses;
	createMidiMain();
	CHECK(HostPlatform::leds().errPulses - before == 1);
	CHECK(FakeTmcPhase::count("initialize") == 1);
	CHECK(FakeTmcPhase::count("setMotionMode") == 1);
	CHECK(htim9.State == HAL_TIM_STATE_BUSY);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.3 MidiMain] a failing initialize pulses the error LED once and construction continues") {
	FakeTmcPhase::state().initializeResult = false;
	const uint32_t before = HostPlatform::leds().errPulses;
	createMidiMain();
	CHECK(HostPlatform::leds().errPulses - before == 1);
	CHECK(FakeTmcPhase::count("setMotionMode") == 1);
	CHECK(htim9.State == HAL_TIM_STATE_BUSY);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.3 MidiMain] a motor type NONE and a failing initialize pulse the error LED twice") {
	FakeTmcPhase::state().motorType = 0;
	FakeTmcPhase::state().initializeResult = false;
	const uint32_t before = HostPlatform::leds().errPulses;
	createMidiMain();
	CHECK(HostPlatform::leds().errPulses - before == 2);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.3 MidiMain] a BLDC motor type and a working initialize do not pulse the error LED") {
	const uint32_t before = HostPlatform::leds().errPulses;
	createMidiMain();
	CHECK(HostPlatform::leds().errPulses == before);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.4 MidiMain] update waits 500 ms of simulated time") {
	const uint64_t before = HostPlatform::nowUs();
	m->update();
	CHECK((HostPlatform::nowUs() - before) / 1000 == 500);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.4 MidiMain] update with power present and not set up calls initializeWithPower once") {
	FakeTmcPhase::state().hasPower = true;
	FakeTmcPhase::state().isSetUp = false;
	m->update();
	CHECK(FakeTmcPhase::count("initializeWithPower") == 1);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.4 MidiMain] update with power present and already set up does not initialise") {
	FakeTmcPhase::state().hasPower = true;
	FakeTmcPhase::state().isSetUp = true;
	m->update();
	CHECK(FakeTmcPhase::count("initializeWithPower") == 0);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.4 MidiMain] update without power and not set up does not initialise") {
	FakeTmcPhase::state().hasPower = false;
	FakeTmcPhase::state().isSetUp = false;
	m->update();
	CHECK(FakeTmcPhase::count("initializeWithPower") == 0);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.4 MidiMain] update without power and set up does not initialise") {
	FakeTmcPhase::state().hasPower = false;
	FakeTmcPhase::state().isSetUp = true;
	m->update();
	CHECK(FakeTmcPhase::count("initializeWithPower") == 0);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.4 MidiMain] each update call with power present and not set up initialises once more") {
	FakeTmcPhase::state().hasPower = true;
	FakeTmcPhase::state().isSetUp = false;
	m->update();
	m->update();
	CHECK(FakeTmcPhase::count("initializeWithPower") == 2);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.5 MidiMain] the commands power and range are registered for get and set") {
	CmdHandlerCommanddef *power = m->getCommandFromName("power");
	CmdHandlerCommanddef *range = m->getCommandFromName("range");
	REQUIRE(power != nullptr);
	REQUIRE(range != nullptr);
	CHECK(power->flags == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(range->flags == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(std::string(power->helpstring) == "Intensity");
	CHECK(std::string(range->helpstring) == "Range of phase change");
	CHECK(power->cmdId != range->cmdId);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.5 MidiMain] the command power returns the default 2500") {
	std::vector<CommandReply> replies;
	REQUIRE(status("power", CMDtype::get, 0, &replies) == static_cast<int>(CommandStatus::OK));
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 2500);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.5 MidiMain] the command range returns the default 0x3fff") {
	std::vector<CommandReply> replies;
	REQUIRE(status("range", CMDtype::get, 0, &replies) == static_cast<int>(CommandStatus::OK));
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 0x3fff);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.5 MidiMain] the command power stores 1, 2500 and 32767 and returns them") {
	const int64_t values[] = {1, 2500, 32767};
	for (int64_t v : values) {
		INFO("power " << v);
		REQUIRE(status("power", CMDtype::set, v) == static_cast<int>(CommandStatus::OK));
		CHECK(CommandHelpers::get(*m, "power") == v);
	}
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.5 MidiMain] the command power stores 0xFFFF and returns it") {
	REQUIRE(status("power", CMDtype::set, 0xFFFF) == static_cast<int>(CommandStatus::OK));
	CHECK(CommandHelpers::get(*m, "power") == 0xFFFF);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.5 MidiMain] the command range stores 1, 0x3fff and 0x7fff and returns them") {
	const int64_t values[] = {1, 0x3fff, 0x7fff};
	for (int64_t v : values) {
		INFO("range " << v);
		REQUIRE(status("range", CMDtype::set, v) == static_cast<int>(CommandStatus::OK));
		CHECK(CommandHelpers::get(*m, "range") == v);
	}
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.5 MidiMain] the next note after setting power 1 starts with setUdUq(1, 0)") {
	REQUIRE(status("power", CMDtype::set, 1) == static_cast<int>(CommandStatus::OK));
	m->noteOn(0, 69, 127);
	ticks(1);
	REQUIRE(!calls().empty());
	CHECK(calls()[0].name == "setUdUq");
	CHECK(calls()[0].a == 1);
	CHECK(calls()[0].b == 0);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.5 MidiMain] the next note after setting power 32767 starts with setUdUq(32767, 0)") {
	REQUIRE(status("power", CMDtype::set, 32767) == static_cast<int>(CommandStatus::OK));
	m->noteOn(0, 69, 127);
	ticks(1);
	REQUIRE(!calls().empty());
	CHECK(calls()[0].name == "setUdUq");
	CHECK(calls()[0].a == 32767);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.5 MidiMain] a power above 32767 does not reach setUdUq as a negative voltage" * KNOWN_ISSUE("B-MIDI-5: MidiMain.cpp:102 passes the uint32_t power to setUdUq(int16_t ud, ...); 0xFFFF becomes -1 (HD: 65535 modulo 65536 as int16_t)")) {
	REQUIRE(status("power", CMDtype::set, 0xFFFF) == static_cast<int>(CommandStatus::OK));
	m->noteOn(0, 69, 127);
	ticks(1);
	REQUIRE(!calls().empty());
	REQUIRE(calls()[0].name == "setUdUq");
	// intended: the intensity is an unsigned magnitude (the command is 'Intensity', a stored 0xFFFF), the voltage is never negative; measured: -1
	CHECK(calls()[0].a >= 0);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.5 MidiMain] a command id that does not exist is NOT_FOUND") {
	ParsedCommand cmd;
	cmd.cmdId = 99;
	cmd.type = CMDtype::get;
	cmd.target = m;
	std::vector<CommandReply> replies;
	CHECK(m->command(cmd, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.6 MidiMain] a range above 0x7fff is not stored" * KNOWN_ISSUE("B-MIDI-5: MidiMain.cpp:160-165 stores range unchecked (uint32_t); sine * range converted to int16_t is undefined above 32767")) {
	status("range", CMDtype::set, 0x8000);
	// intended: a range above 0x7fff (the largest int16_t value) is rejected or limited, so the stored value is at most 0x7fff; measured: 0x8000
	CHECK(CommandHelpers::get(*m, "range") <= 0x7fff);
}

TEST_CASE_FIXTURE(NoteFixture, "[R6.6 MidiMain] a range of 0x100000 does not make play convert out of range: the output is not larger than the value of range 0x7fff" * KNOWN_ISSUE_UB("B-MIDI-5: MidiMain.cpp:116 (int16_t)(sine * movementrange) converts 144483.6 (HD: 0.137790 * 1048576), which is out of range of int16_t")) {
	REQUIRE(status("range", CMDtype::set, 0x100000) == static_cast<int>(CommandStatus::OK));
	m->noteOn(0, 69, 127);
	// intended: a defined result. The defined maximum of a larger range is the value of range 0x7fff at the same tick (HD: 0.137790 * 32767 = 4514, +1 for rounding),
	// whether the range is limited or the output saturates; measured: the conversion is undefined (x86 keeps the low 16 bits: 13411)
	CHECK(phi1() <= 4515);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.7 MidiMain] destroying MidiMain clears the global MIDI handler pointer") {
	MidiMain &m = createMidiMain();
	CHECK(midihandler == static_cast<MidiHandler *>(&m));
	midi.reset();
	CHECK(midihandler == nullptr);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.7 MidiMain] destroying MidiMain destroys its TMC4671 driver") {
	createMidiMain();
	CHECK(FakeTmcPhase::state().destroyed == 0);
	midi.reset();
	CHECK(FakeTmcPhase::state().constructed == 1);
	CHECK(FakeTmcPhase::state().destroyed == 1);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.7 MidiMain] destroying MidiMain removes it from the timer handlers") {
	const size_t before = TimerHandler::timerHandlers.size();
	createMidiMain();
	CHECK(TimerHandler::timerHandlers.size() == before + 1);
	midi.reset();
	CHECK(TimerHandler::timerHandlers.size() == before);
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.7 MidiMain] characterization: the user timer keeps running after the destruction and its events make no driver call") {
	MidiMain &m = createMidiMain();
	m.noteOn(0, 69, 127);
	midi.reset();
	forget();
	CHECK(HostTarget::timerRunning(&htim9)); // TIM_USER is never stopped (no HAL_TIM_Base_Stop_IT in MidiMain.cpp)
	CHECK(htim9.State == HAL_TIM_STATE_BUSY);
	CHECK(tick(3) == 3); // the update interrupt is still delivered, nobody listens
	CHECK(calls().empty());
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.8 MidiMain] MidiMain created and destroyed twice leaves no leaked kernel object and two drivers were built and destroyed") {
	createMidiMain();
	midi.reset();
	createMidiMain();
	midi.reset();
	CHECK(FakeTmcPhase::state().constructed == 2);
	CHECK(FakeTmcPhase::state().destroyed == 2);
	CHECK(midihandler == nullptr);
	// the fixture destructor checks the kernel objects (tasks, queues, semaphores, timers) against the start of the test
}

TEST_CASE_FIXTURE(MidiMainFixture, "[R6.8 MidiMain] a second MidiMain created after the first was destroyed registers itself as the MIDI handler and plays") {
	createMidiMain();
	midi.reset();
	MidiMain &m = createMidiMain();
	CHECK(midihandler == static_cast<MidiHandler *>(&m));
	m.noteOn(0, 69, 127);
	forget();
	CHECK(tick(1) == 1);
	CHECK(FakeTmcPhase::count("setPhiE_ext") == 1);
}

} // TEST_SUITE
