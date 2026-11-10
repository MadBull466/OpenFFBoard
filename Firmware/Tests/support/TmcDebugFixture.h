/*
 * TmcDebugFixture.h
 *
 * Fixture of the TMCDebugBridge tests (ffb_driver_tests only, phase P13,
 * Tests/claude/charters/driver/TMCDEBUG_REQUIREMENTS.md "Fixture and test layout").
 *
 * Models: TmcFixture (FakeTmc4671 on chip select PA4 behind the fake SPI1, fake ADC supply, fake kernel) plus the CDC
 * capture of platform/src/host_usb.cpp (HostUsb::state().cdcTx / cdcWrites / cdcFlushes). The bridge owns a TMC_1 and
 * talks to chip1, so chip1 exists (TmcFixture) before the bridge is constructed: its constructor already writes to the chip.
 * Source: TMCDebugBridge.cpp:236-252 (constructor), :133-231 (cdcRcv), :128-131 (sendCdc). Does not model: the USB
 * enumeration (ffb_usb_tests), the TMCL-IDE, the timing of the real SPI clock. Confidence: High for the CDC capture and
 * the fake chip (both verified by their own self-tests), Medium for the thread interplay (the driver thread "TMC" is
 * only run through runThreadFor / runUntilState of TmcFixture).
 *
 * Helpers (public interface of the bridge only):
 *   makeBridge()      constructs the TMCDebugBridge (chip1 present, power on, 10 us per SPI transfer like reachShutdown)
 *   frame(cmd, addr, value, checksum)  the 9 byte request {1, cmd, addr, 0, value MSB first, checksum}; checksum = the 8-bit
 *                     sum of the first 8 bytes (TMCL reference v2.22 section 2.1.1) unless `badChecksum` is given
 *   send(bytes)       calls bridge.cdcRcv with a copy of the bytes (the firmware may rewrite the buffer)
 *   sent()            everything written to CDC interface 0 since the last clearSent() (raw bytes)
 *   clearSent()       clears the capture (cdcTx, cdcWrites, cdcFlushes)
 *   cdcWrites() / cdcFlushes()   counters of the capture
 */
#ifndef HOSTTEST_TMCDEBUGFIXTURE_H_
#define HOSTTEST_TMCDEBUGFIXTURE_H_

#include "TmcFixture.h"
#include "HostUsb.h"

#include "TMCDebugBridge.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct TmcDebugFixture : TmcFixture {
	TmcDebugFixture() { HostUsb::reset(); }

	/** The bridge, constructed after chip1 exists. Power is on and the chip answers SPI frames. */
	std::unique_ptr<TMCDebugBridge> makeBridge() {
		chip1.setPresent(true);
		if (chip1.microsPerTransfer() == 0) {
			chip1.setMicrosPerTransfer(10);
		}
		powerOn();
		return std::make_unique<TMCDebugBridge>();
	}

	/** The 8-bit sum of the first n bytes (TMCL reference v2.22 section 2.1.1) */
	static uint8_t sum8(const std::vector<uint8_t> &bytes, size_t n) {
		uint8_t s = 0;
		for (size_t i = 0; i < n && i < bytes.size(); i++) {
			s = static_cast<uint8_t>(s + bytes[i]);
		}
		return s;
	}

	/** Request frame {module address 1, cmd, type (addr), motor/bank 0, value MSB first, checksum} */
	static std::vector<uint8_t> frame(uint8_t cmd, uint8_t type, uint32_t value, int badChecksum = -1) {
		std::vector<uint8_t> f = {1, cmd, type, 0,
								  static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
								  static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
		f.push_back(badChecksum >= 0 ? static_cast<uint8_t>(badChecksum) : sum8(f, 8));
		return f;
	}

	/** Calls bridge.cdcRcv with a copy of the bytes */
	void send(TMCDebugBridge &bridge, const std::vector<uint8_t> &bytes) {
		std::vector<char> buf(bytes.begin(), bytes.end());
		uint32_t len = static_cast<uint32_t>(buf.size());
		bridge.cdcRcv(buf.data(), &len);
	}

	std::vector<uint8_t> sent() const {
		const std::string &s = HostUsb::state().cdcTx;
		return std::vector<uint8_t>(s.begin(), s.end());
	}
	void clearSent() {
		HostUsb::state().cdcTx.clear();
		HostUsb::state().cdcWrites = 0;
		HostUsb::state().cdcFlushes = 0;
	}
	uint32_t cdcWrites() const { return HostUsb::state().cdcWrites; }
	uint32_t cdcFlushes() const { return HostUsb::state().cdcFlushes; }
};

#endif
