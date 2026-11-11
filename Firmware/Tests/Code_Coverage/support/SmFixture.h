/*
 * SmFixture.h
 *
 * Fixture of the SimpleMotion driver tests (ffb_driver_tests only, phase P15,
 * Tests/Code_Coverage/claude/charters/driver/SIMPLEMOTION_REQUIREMENTS.md "Fixture and test layout").
 *
 * Models: TargetFixture (target state reset before and after the test, unmodelled HAL calls fail the test) plus
 *  - the fake flash cleared,
 *  - a FakeSmDevice (fakes/FakeSmDevice.h) installed as the UART peer of huart3 (the motor port) after the target reset and
 *    removed at teardown; defaults: healthy ABN1 servo in torque mode, answers every device address (B-SM-1: the driver's
 *    own address is not known to the test, tests use dev.frames() to see it),
 *  - makeSm(instance): constructs MotorSimplemotion1 / 2 by placement new into heap memory pre-filled with 0xA5, so the
 *    member `address` that the constructor initialises from itself (B-SM-1, `address(address + 1)`) is deterministic: it
 *    reads 0xA5 and becomes 0xA6 when the compiler reads the memory (an optimising build may do anything: KNOWN_ISSUE_UB
 *    tests are the only ones that may look at the value; every other test learns the address from the wire).
 *    The fixture destroys every driver it created at teardown (the in-use flags of the instances are private statics that the
 *    destructor clears),
 *  - initialize(sm): advances the time by 150 ms (getSettings refuses to run earlier) and runs getSettings,
 *  - process(): delivers what the peer has pending (transmit-complete interrupt, reply bytes), like the interrupts would,
 *  - at teardown fails the test on any peer violation the test did not declare with dev.expectViolation().
 * Never sleeps: simulated time only (HostPlatform::advanceMs, the peer's onPoll in the firmware's busy-wait loops).
 *
 * Source: MotorSimplemotion.cpp / .h, fakes/FakeSmDevice.h. Does not model: see FakeSmDevice.h. Confidence: High for the wiring.
 */
#ifndef HOSTTEST_SMFIXTURE_H_
#define HOSTTEST_SMFIXTURE_H_

#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "TestHelpers.h"
#include "../fakes/FakeSmDevice.h" // not an include directory: fakes/Axis.h would shadow the real Axis.h

#include "MotorSimplemotion.h"
#include "cpp_target_config.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <vector>

extern UART_HandleTypeDef huart3; // platform/src/host_target_config.cpp

struct SmFixture : TargetFixture {
	FakeSmDevice dev{&huart3};

	/** Owns one driver constructed in poisoned memory */
	struct SmHandle {
		MotorSimplemotion *sm = nullptr;
		void *raw = nullptr;
		SmHandle() = default;
		SmHandle(const SmHandle &) = delete;
		SmHandle &operator=(const SmHandle &) = delete;
		SmHandle(SmHandle &&o) noexcept : sm(o.sm), raw(o.raw) { o.sm = nullptr; o.raw = nullptr; }
		~SmHandle() { destroy(); }
		void destroy() {
			if (sm) {
				sm->~MotorSimplemotion();
				::operator delete(raw);
				sm = nullptr;
				raw = nullptr;
			}
		}
		MotorSimplemotion *operator->() const { return sm; }
		MotorSimplemotion &operator*() const { return *sm; }
		explicit operator bool() const { return sm != nullptr; }
	};

	SmFixture() {
		HostPlatform::flash().clear();
		HostPlatform::setUartPeer(&dev);
	}

	~SmFixture() override {
		for (SmHandle &h : handles) {
			h.destroy();
		}
		HostPlatform::setUartPeer(nullptr);
		for (const auto &v : dev.unexpectedViolations()) {
			FAIL_CHECK("SimpleMotion fake: " << v.text());
		}
	}

	/** Instance 1 (device address from the constructor) or 2. The memory is filled with 0xA5 before construction (B-SM-1) */
	SmHandle makeSm(int instance = 1) {
		SmHandle h;
		const size_t size = instance == 1 ? sizeof(MotorSimplemotion1) : sizeof(MotorSimplemotion2);
		h.raw = ::operator new(size);
		std::memset(h.raw, 0xA5, size);
		if (instance == 1) {
			h.sm = new (h.raw) MotorSimplemotion1();
		} else {
			h.sm = new (h.raw) MotorSimplemotion2();
		}
		return h;
	}

	/** makeSm for tests that want the fixture to destroy the driver at teardown even if the test returns early */
	MotorSimplemotion *makeOwnedSm(int instance = 1) {
		handles.push_back(makeSm(instance));
		return handles.back().sm;
	}

	/** Runs getSettings after the 150 ms interval. Returns its result */
	bool initialize(MotorSimplemotion &sm) {
		HostPlatform::advanceMs(150);
		return sm.getSettings();
	}

	/** Delivers everything the peer has pending (transmit complete interrupt, then the reply) */
	void process() { dev.pumpAll(); }

	void advanceMs(uint64_t ms) { HostPlatform::advanceMs(ms); }

	/** The address byte of the last frame the driver sent, 0 if none */
	uint8_t lastAddress() const {
		if (dev.frames().empty()) {
			return 0;
		}
		const auto &b = dev.frames().back().bytes;
		return b.size() > 2 ? (b[0] == 0x10 ? b[1] : b[2]) : 0;
	}

private:
	std::vector<SmHandle> handles;
};

#endif
