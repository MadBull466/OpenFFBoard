/*
 * MidiSetup: the P17 skeleton (T1). It checks the build wiring and the test infrastructure of the MIDI tests, one smoke
 * case per area for the later groups, and nothing else: the requirements of Tests/Code_Coverage/claude/charters/driver/MIDI_REQUIREMENTS.md
 * are tested in test_midi_handler.cpp, test_midi_main.cpp, test_midi_class.cpp and test_midi_stream.cpp (T2).
 *
 * Infrastructure under test here: the feature macro MIDI (platform/usb_midi/target_constants.h), the recording double of the
 * TMC4671 interface (fakes/FakeTmcPhase.h, platform/usb_midi/TMC4671.h), the fake TIM9 (platform/src/host_midi_timer.cpp),
 * UsbHost::midiStreamingInterface(), MidiProbe and the packet builders of support/MidiFixture.h.
 * Oracles: USB class definition for MIDI devices 1.0 (midi10.pdf) sections 4 and 6 / Appendix B, MIDI 1.0 detailed
 * specification 4.2.1; the firmware's own behaviour where both are silent is a `characterization:` case.
 */
#include "doctest.h"
#include "MidiFixture.h"
#include "TestHelpers.h"

#include "TimerHandler.h"
#include "mainclass_chooser.h"

#include <memory>
#include <vector>

namespace {

/** A TimerHandler that records the timer handles it is called with */
struct TimerRecorder : TimerHandler {
	std::vector<TIM_HandleTypeDef *> seen;
	void timerElapsed(TIM_HandleTypeDef *htim) override { seen.push_back(htim); }
};

} // namespace

TEST_SUITE("MidiSetup") {

// ---------------------------------------------------------------- build wiring

TEST_CASE("the MIDI feature macro is defined in this executable") {
#ifdef MIDI
	CHECK(true);
#else
	FAIL("MIDI is not defined in ffb_usb_tests (platform/usb_midi/target_constants.h)");
#endif
}

TEST_CASE_FIXTURE(MidiFixture, "the recording double of the TMC4671 interface records constructor, destructor and answers") {
	CHECK(FakeTmcPhase::state().calls.empty());
	{
		TMC_1 drv;
		CHECK(drv.conf.motconf.motor_type == MotorType::BLDC); // default reported motor type
		CHECK(drv.allowSlowSPI); // plain member, true by default as TMC4671.h:610
		drv.setUdUq(5, -7);
		CHECK(drv.initialize()); // default initialize() result
		FakeTmcPhase::state().hasPower = true;
		CHECK(drv.hasPower());
		CHECK_FALSE(drv.isSetUp());
	}
	const auto &calls = FakeTmcPhase::state().calls;
	REQUIRE(calls.size() == 6);
	CHECK(calls[0].name == "construct");
	CHECK(calls[1].name == "setUdUq");
	CHECK(calls[1].a == 5);
	CHECK(calls[1].b == -7);
	CHECK(calls[2].name == "initialize");
	CHECK(calls[3].name == "hasPower");
	CHECK(calls[4].name == "isSetUp");
	CHECK(calls[5].name == "destruct");
	CHECK(FakeTmcPhase::state().constructed == 1);
	CHECK(FakeTmcPhase::state().destroyed == 1);
	FakeTmcPhase::reset();
	CHECK(FakeTmcPhase::state().calls.empty());
	CHECK(FakeTmcPhase::state().constructed == 0);
}

TEST_CASE_FIXTURE(MidiFixture, "the fake htim9 starts as MX_TIM9_Init leaves it and the user timer interrupt reaches the timer handlers") {
	// MX_TIM9_Init (Targets/F407VG/Core/Src/main.c:1154-1170): Prescaler 0, Period 65535, state READY, timer stopped
	REQUIRE(htim9.Instance != nullptr);
	CHECK(htim9.Init.Prescaler == 0);
	CHECK(htim9.Init.Period == 65535);
	CHECK(htim9.Instance->ARR == 65535);
	CHECK(htim9.State == HAL_TIM_STATE_READY);
	CHECK_FALSE(HostTarget::timerRunning(&htim9));
	TimerRecorder rec;
	CHECK_FALSE(HostTarget::timerUpdate(&htim9)); // stopped timer: no update interrupt
	CHECK(rec.seen.empty());
	REQUIRE(HAL_TIM_Base_Start_IT(&htim9) == HAL_OK);
	CHECK(htim9.State == HAL_TIM_STATE_BUSY);
	CHECK(HostTarget::timerRunning(&htim9));
	CHECK(HostTarget::timerUpdateIrqEnabled(&htim9));
	CHECK(HostTarget::timerUpdate(&htim9));
	REQUIRE(rec.seen.size() == 1);
	CHECK(rec.seen[0] == &htim9);
}

// ---------------------------------------------------------------- USB side

TEST_CASE_FIXTURE(MidiFixture, "the MIDI configuration enumerates with the MIDIStreaming bulk endpoints 0x02 and 0x83 of 64 bytes") {
	startMidi();
	CHECK_FALSE(tud_midi_mounted());
	enumerate();
	const UsbHost::Interface *ms = host.midiStreamingInterface();
	REQUIRE(ms != nullptr);
	CHECK(ms->number == 3); // CDC 0 and 1, AudioControl 2, MIDIStreaming 3 (usb_descriptors.cpp:83-92)
	CHECK(ms->epOut == 0x02);
	CHECK(ms->epIn == 0x83);
	CHECK(ms->epOutSize == 64);
	CHECK(ms->epInSize == 64);
	CHECK(tud_midi_mounted());
}

TEST_CASE("the packet builders follow the USB-MIDI event packet layout of the class definition 1.0") {
	// Table 4-2: note on on cable 1 (CN 1, CIN 9): 19 9n kk vv; control change on cable 10: AB Bn pp vv; real time on cable 3: 3F F8 xx xx
	CHECK(MidiFixture::noteOnPkt(2, 60, 100, 1) == std::array<uint8_t, 4>{0x19, 0x92, 60, 100});
	CHECK(MidiFixture::pkt(10, 0xB, 0xB1, 7, 100) == std::array<uint8_t, 4>{0xAB, 0xB1, 7, 100});
	CHECK(MidiFixture::pkt(3, 0xF, 0xF8) == std::array<uint8_t, 4>{0x3F, 0xF8, 0, 0});
	CHECK(MidiFixture::noteOffPkt(0, 60, 64) == std::array<uint8_t, 4>{0x08, 0x80, 60, 64});
}

// ---------------------------------------------------------------- MidiHandler smoke (group MHD)

TEST_CASE_FIXTURE(MidiFixture, "a MidiProbe registers itself as the global handler and clears the pointer when it is destroyed") {
	CHECK(midihandler == nullptr);
	{
		MidiProbe probe;
		CHECK(midihandler == &probe);
	}
	CHECK(midihandler == nullptr);
}

TEST_CASE_FIXTURE(MidiFixture, "a note on packet sent by the host on endpoint 0x02 reaches the handler as noteOn with the interface number 0") {
	startAndEnumerate();
	MidiProbe probe;
	REQUIRE(sendPackets({noteOnPkt(3, 60, 100)}) == HostDcd::Handshake::Ack);
	REQUIRE(probe.hooks.size() == 1);
	CHECK(probe.hooks[0].name == "noteOn");
	CHECK(probe.hooks[0].chan == 3);
	CHECK(probe.hooks[0].a == 60);
	CHECK(probe.hooks[0].b == 100);
	CHECK(probe.hooks[0].itf == 0);
}

// ---------------------------------------------------------------- class driver smoke (groups MCD, MCS)

TEST_CASE_FIXTURE(MidiFixture, "a packet written with tud_midi_packet_write arrives on endpoint 0x83 as the same four bytes") {
	startAndEnumerate();
	const uint8_t packet[4] = {0x09, 0x90, 60, 127};
	REQUIRE(tud_midi_packet_write(packet));
	process();
	HostDcd::Handshake status = HostDcd::Handshake::Nak;
	size_t rest = 99;
	const auto got = receivePackets(&status, &rest);
	REQUIRE(got.size() == 1);
	CHECK(rest == 0);
	CHECK(got[0] == std::array<uint8_t, 4>{0x09, 0x90, 60, 127});
}

TEST_CASE_FIXTURE(MidiFixture, "writing the byte stream 90 3C 7F on cable 0 gives the packet 09 90 3C 7F") {
	startAndEnumerate();
	const uint8_t bytes[3] = {0x90, 0x3C, 0x7F};
	CHECK(tud_midi_stream_write(0, bytes, 3) == 3);
	process();
	const auto got = receivePackets();
	REQUIRE(got.size() == 1);
	CHECK(got[0] == std::array<uint8_t, 4>{0x09, 0x90, 0x3C, 0x7F}); // class definition 1.0 table 4-1 (CIN 0x9) and 4-2
}

// ---------------------------------------------------------------- MidiMain smoke (group MMN)

TEST_CASE_FIXTURE(MidiMainFixture, "MidiMain is constructed on the driver double with its identity and the user timer started") {
	MidiMain &m = createMidiMain();
	const ClassIdentifier id = m.getInfo();
	CHECK(std::string(id.name) == "MIDI (TMC)");
	CHECK(id.id == CLSID_MAIN_MIDI);
	REQUIRE(!FakeTmcPhase::state().calls.empty());
	CHECK(FakeTmcPhase::state().calls[0].name == "construct");
	CHECK(FakeTmcPhase::count("initialize") == 1);
	CHECK(htim9.Instance->ARR == 100); // MidiMain.cpp:26
	CHECK(htim9.State == HAL_TIM_STATE_BUSY);
	CHECK(HostTarget::timerRunning(&htim9));
	CHECK_FALSE(m.drv->allowSlowSPI); // MidiMain.cpp:55
}

TEST_CASE_FIXTURE(MidiMainFixture, "a note on makes the next timer tick call setUdUq with the default power 2500 and setPhiE_ext once") {
	MidiMain &m = createMidiMain();
	FakeTmcPhase::state().calls.clear();
	m.noteOn(0, 69, 127);
	CHECK(tick(1) == 1);
	REQUIRE(FakeTmcPhase::state().calls.size() == 2);
	CHECK(FakeTmcPhase::state().calls[0].name == "setUdUq");
	CHECK(FakeTmcPhase::state().calls[0].a == 2500);
	CHECK(FakeTmcPhase::state().calls[0].b == 0);
	CHECK(FakeTmcPhase::state().calls[1].name == "setPhiE_ext");
}

TEST_CASE_FIXTURE(MidiMainFixture, "MidiMain::update waits 500 ms of simulated time through the osDelay shim and then initialises a powered driver once") {
	MidiMain &m = createMidiMain();
	FakeTmcPhase::state().calls.clear();
	FakeTmcPhase::state().hasPower = true;
	FakeTmcPhase::state().isSetUp = false;
	const uint64_t before = HostPlatform::nowUs();
	m.update();
	CHECK((HostPlatform::nowUs() - before) / 1000 == 500); // MidiMain.cpp:79 osDelay(500), tick rate 1 kHz (FreeRTOSConfig.h)
	CHECK(FakeTmcPhase::count("initializeWithPower") == 1);
}

} // TEST_SUITE
