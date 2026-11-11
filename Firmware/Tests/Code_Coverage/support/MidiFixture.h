/*
 * MidiFixture.h
 *
 * Fixture of the MIDI tests (P17, ffb_usb_tests only; Tests/Code_Coverage/claude/charters/driver/MIDI_REQUIREMENTS.md). It is the
 * UsbFixture (fake dwc2 controller, real TinyUSB, the firmware's USB glue) started with the firmware's MIDI
 * configuration usb_cdc_midi_conf, plus:
 *  - MidiProbe: a MidiHandler that records every hook call (hook name, up to three arguments, order, the raw packet of
 *    otherPacket) and exposes the protected sysexState. Constructing it registers it in the global `midihandler`
 *    (global_callbacks.cpp), destroying it clears the pointer, exactly like MidiMain.
 *  - MidiFixture::startMidi() / enumerate() / sendPackets() / receivePackets(): the host side of the MIDIStreaming
 *    interface (OUT endpoint 0x02, IN endpoint 0x83, 64 byte packets) with the helpers pkt() and noteOn() ... that build
 *    the 4 byte USB-MIDI event packets of the USB class definition 1.0 (section 4).
 *  - MidiMainFixture: a MidiMain built on the recording double of the TMC4671 interface (fakes/FakeTmcPhase.h,
 *    platform/usb_midi/TMC4671.h) and the fake htim9 (platform/src/host_midi_timer.cpp); tick(n) raises n update events
 *    of htim9 (HostTarget::timerUpdate): one event is one call of MidiMain::play().
 * Resets in the constructor and the destructor: FakeTmcPhase, the global midihandler pointer. Everything else is reset
 * by UsbFixture::resetAll (fake HAL, RTOS, USB controller). The destructor checks (UsbFixture) controller faults, host
 * errors, unmodelled HAL calls, kernel faults and leaked kernel objects.
 */
#ifndef SUPPORT_MIDIFIXTURE_H_
#define SUPPORT_MIDIFIXTURE_H_

#include "UsbFixture.h"

#include "../fakes/FakeTmcPhase.h"
#include "MidiHandler.h"
#include "MidiMain.h"
#include "midi_device.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

extern MidiHandler *midihandler; // global_callbacks.cpp

/** One recorded hook call of a MidiProbe */
struct MidiHook {
	std::string name; //!< "noteOn", "noteOff", "controlChange", "programChange", "pitchBend", "midiTick", "otherPacket"
	int chan = 0;
	int a = 0; //!< note / controller / program / pitch bend value / packet byte 0
	int b = 0; //!< velocity / controller value / packet byte 1
	int c = 0; //!< packet byte 2 (otherPacket)
	int d = 0; //!< packet byte 3 (otherPacket)
	uint8_t itf = 0; //!< interface number of the midiRx call that produced it
	bool operator==(const MidiHook &o) const {
		return name == o.name && chan == o.chan && a == o.a && b == o.b && c == o.c && d == o.d;
	}
};

/** A MidiHandler that records its hook calls */
class MidiProbe : public MidiHandler {
public:
	std::vector<MidiHook> hooks;
	uint8_t lastItf = 0;

	void midiRx(uint8_t itf, uint8_t packet[4]) override {
		lastItf = itf;
		rxPackets.push_back({packet[0], packet[1], packet[2], packet[3]});
		MidiHandler::midiRx(itf, packet);
	}
	void noteOn(uint8_t chan, uint8_t note, uint8_t velocity) override { add("noteOn", chan, note, velocity); }
	void noteOff(uint8_t chan, uint8_t note, uint8_t velocity) override { add("noteOff", chan, note, velocity); }
	void controlChange(uint8_t chan, uint8_t c, uint8_t val) override { add("controlChange", chan, c, val); }
	void programChange(uint8_t chan, uint8_t val) override { add("programChange", chan, val, 0); }
	void pitchBend(uint8_t chan, int16_t val) override { add("pitchBend", chan, val, 0); }
	void midiTick() override { add("midiTick", 0, 0, 0); }
	void otherPacket(uint8_t packet[4]) override {
		MidiHook h;
		h.name = "otherPacket";
		h.a = packet[0];
		h.b = packet[1];
		h.c = packet[2];
		h.d = packet[3];
		h.itf = lastItf;
		hooks.push_back(h);
	}

	bool sysex() const { return sysexState; } //!< the protected MidiHandler::sysexState
	size_t count(const std::string &name) const {
		size_t n = 0;
		for (const MidiHook &h : hooks) {
			n += h.name == name;
		}
		return n;
	}
	/** The packets midiRx received, in order (the 4 bytes as delivered by tud_midi_rx_cb) */
	std::vector<std::array<uint8_t, 4>> rxPackets;

private:
	void add(const char *name, int chan, int a, int b) {
		MidiHook h;
		h.name = name;
		h.chan = chan;
		h.a = a;
		h.b = b;
		h.itf = lastItf;
		hooks.push_back(h);
	}
};

/** The USB fixture with the MIDI configuration (usb_cdc_midi_conf) */
struct MidiFixture : UsbFixture {
	MidiFixture() {
		FakeTmcPhase::reset();
		midihandler = nullptr;
	}
	~MidiFixture() override {
		midihandler = nullptr;
		FakeTmcPhase::reset();
	}

	/** Starts the USB device with the MIDI configuration (FFBoardMain-like test main class, no HID report descriptor needed) */
	void startMidi() { startDevice(usb_cdc_midi_conf); }
	/** startMidi() and enumeration; REQUIREs the configured state and the MIDIStreaming interface */
	void startAndEnumerate() {
		startMidi();
		enumerate();
		REQUIRE(host.midiStreamingInterface() != nullptr);
	}

	uint8_t epOut() const { return host.midiStreamingInterface()->epOut; }
	uint8_t epIn() const { return host.midiStreamingInterface()->epIn; }
	uint16_t epSize() const { return host.midiStreamingInterface()->epOutSize; }

	/** USB-MIDI event packet: byte 0 = cable number << 4 | code index number (USB class definition 1.0, figure 8) */
	static std::array<uint8_t, 4> pkt(uint8_t cable, uint8_t cin, uint8_t b1, uint8_t b2 = 0, uint8_t b3 = 0) {
		return {static_cast<uint8_t>((cable << 4) | (cin & 0x0F)), b1, b2, b3};
	}
	static std::array<uint8_t, 4> noteOnPkt(uint8_t chan, uint8_t note, uint8_t vel, uint8_t cable = 0) { return pkt(cable, 0x9, 0x90 | chan, note, vel); }
	static std::array<uint8_t, 4> noteOffPkt(uint8_t chan, uint8_t note, uint8_t vel, uint8_t cable = 0) { return pkt(cable, 0x8, 0x80 | chan, note, vel); }

	/** The host sends the packets as one bulk OUT transfer on the MIDI OUT endpoint and the device works on them */
	HostDcd::Handshake sendPackets(const std::vector<std::array<uint8_t, 4>> &packets) {
		std::vector<uint8_t> bytes;
		for (const auto &p : packets) {
			bytes.insert(bytes.end(), p.begin(), p.end());
		}
		const HostDcd::Handshake h = host.sendOut(epOut(), epSize(), bytes, true);
		process();
		return h;
	}
	/** The host reads one transfer from the MIDI IN endpoint and splits it into packets (a non multiple of 4 is returned as is in `rest`) */
	std::vector<std::array<uint8_t, 4>> receivePackets(HostDcd::Handshake *status = nullptr, size_t *rest = nullptr, size_t maxBytes = 256) {
		const std::vector<uint8_t> bytes = host.receiveIn(epIn(), epSize(), maxBytes, status);
		std::vector<std::array<uint8_t, 4>> out;
		size_t i = 0;
		for (; i + 4 <= bytes.size(); i += 4) {
			out.push_back({bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]});
		}
		if (rest) {
			*rest = bytes.size() - i;
		}
		return out;
	}
};

/** MidiMain on the recording TMC4671 double and the fake htim9 (no USB needed for the synthesis) */
struct MidiMainFixture : MidiFixture {
	std::unique_ptr<MidiMain> midi;

	/** Constructs the MidiMain (the constructor's driver calls are recorded in FakeTmcPhase::state().calls) */
	MidiMain &createMidiMain() {
		midi = std::make_unique<MidiMain>();
		return *midi;
	}
	/** n update events of htim9 = n calls of MidiMain::play(); returns how many reached the callback */
	uint32_t tick(uint32_t n = 1) {
		uint32_t delivered = 0;
		for (uint32_t i = 0; i < n; i++) {
			delivered += HostTarget::timerUpdate(&htim9) ? 1 : 0;
		}
		return delivered;
	}
	~MidiMainFixture() override { midi.reset(); }
};

#endif
