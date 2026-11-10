/*
 * RmdFixture.h
 *
 * Fixture of the MyActuator RMD CAN driver tests (ffb_driver_tests only, phase P16, charter
 * Tests/claude/charters/driver/RMD_REQUIREMENTS.md "RmdFixture").
 *
 * RmdFixture is a TargetFixture (target state reset before and after the test, unmodelled HAL calls fail the test) and
 *  - clears the fake flash (HostPlatform::flash()),
 *  - owns one FakeRmdPeer per instance: peer1 (node 1, RmdMotorCAN1 / motor 0) and peer2 (node 2, RmdMotorCAN2 / motor 1),
 *  - TX completion: the motor acknowledges every frame (as FakeOdrivePeer / OdriveFixture): process() completes all pending
 *    mailboxes before the peers decode the frames, and the fake HAL's auto-complete is on. setTxAutoComplete(false) leaves
 *    them pending (a bus without acknowledge: the peers see nothing).
 *  - FIFO 1 (instance 2): RmdMotorCAN2 registers its filter on receive buffer 1. On the F407 the CAN1_RX1 NVIC line is
 *    disabled (HAL_CAN_MspInit), so a frame for FIFO 1 raises no interrupt. MODELLING CHOICE (as OdriveFixture): the fixture
 *    ENABLES the Rx1 line so that frames for instance 2 are delivered at once like for instance 1. setRx1Interrupt(false)
 *    restores the real behaviour.
 *  - the thread "RMD" (both instances use this name): the fixture finds the handle of an instance through its Thread base (a
 *    C-style cast to the private base cpp_freertos::Thread; the firmware is not changed) and makes it resumable.
 *    RmdMotorCAN::Run has several Delay calls per cycle (100 ms start, then 20 ms when not available, 150 ms after the
 *    constant report set-up, 1000 ms). The fixture sets the HostRtos delay budget to 1 ms so EVERY Delay parks the thread
 *    after advancing the clock, then measures the jump and puts the clock back: the activation runs the code up to the next
 *    Delay at exactly the time the previous Delay ended, and the thread is scheduled again after the measured delay. The
 *    thread thus runs at its exact times: 100 ms, then 100 + 20 ... (see R4 of the charter). Delay budget 1 ms is restored to
 *    1000 ms in the destructor.
 *    Host-vs-target: the clock is set back after each activation; a test must not create other timed objects across runFor().
 *  - at teardown fails the test on any FakeRmdPeer violation the test did not declare with expectViolation().
 *
 * Time: runFor(ms) advances the simulated clock by ms, stepping from event to event (peer cyclic replies, delayed
 * replies, driver thread wake times). Never sleeps. peerN.advance(ms) advances the clock without running the driver threads.
 *
 * Helpers: makeRmd1() / makeRmd2() (unique_ptr to RmdMotorCAN1 / RmdMotorCAN2, constructed after the fixture),
 * reachAvailable(driver) (runs until motorReady(); the peer answers 0xB5 and the cyclic 0x92), txFrames(id) / txFramesCmd(id,
 * cmd), clearTxFrames(), injectReply(peer, payload) for a raw reply.
 */
#ifndef HOSTTEST_RMDFIXTURE_H_
#define HOSTTEST_RMDFIXTURE_H_

#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "../fakes/FakeRmdPeer.h" // not an include directory: fakes/Axis.h would shadow the real Axis.h
#include "HostRtos.h"
#include "TestHelpers.h"

#include "RmdMotorCAN.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <vector>

struct RmdFixture : TargetFixture {
	FakeRmdPeer peer1{1}; //!< motor of node 1 (RmdMotorCAN1)
	FakeRmdPeer peer2{2}; //!< motor of node 2 (RmdMotorCAN2)

	RmdFixture() {
		HostPlatform::flash().clear();
		HostRtos::setDelayBudgetMs(1); // every Delay parks the thread (see the header)
		HostTarget::canSetTxAutoComplete(true);
		HostTarget::canSetIrqLineEnabled(HostTarget::CanIrqLine::Rx1, true);
		peer1.reset();
		peer2.reset();
	}

	~RmdFixture() override {
		for (FakeRmdPeer *p : {&peer1, &peer2}) {
			for (const auto &v : p->unexpectedViolations()) {
				FAIL_CHECK("RMD fake, node " << int(p->nodeId()) << ": " << v.text());
			}
		}
		HostRtos::setDelayBudgetMs(1000);
	}

	// --- driver objects ---
	template <class T>
	struct Deleter {
		RmdFixture *fx;
		void operator()(T *p) const {
			fx->forget(p);
			delete p;
		}
	};
	template <class T>
	using Ptr = std::unique_ptr<T, Deleter<T>>;

	Ptr<RmdMotorCAN1> makeRmd1() { return make<RmdMotorCAN1>(); }
	Ptr<RmdMotorCAN2> makeRmd2() { return make<RmdMotorCAN2>(); }

	// --- CAN setup ---
	void setTxAutoComplete(bool on) {
		txAutoComplete_ = on;
		HostTarget::canSetTxAutoComplete(on);
	}
	/** true (fixture default): frames for instance 2 (FIFO 1) are delivered at once; false: the real F407 (CAN1_RX1 disabled) */
	void setRx1Interrupt(bool on) { HostTarget::canSetIrqLineEnabled(HostTarget::CanIrqLine::Rx1, on); }

	// --- time ---
	/** Completes pending TX mailboxes (if auto-complete is on) and lets both peers decode, answer and send their due replies */
	void process() {
		if (txAutoComplete_) {
			HostTarget::canTxCompleteAll();
		}
		peer1.process();
		peer2.process();
	}

	/** Advances the simulated time by `ms`, running the peers' cyclic replies and the driver threads at their exact times */
	void runFor(uint32_t ms) {
		const uint64_t end = HostPlatform::nowUs() + uint64_t(ms) * 1000;
		for (;;) {
			process();
			for (Th &t : threads_) {
				if (t.wakeUs <= HostPlatform::nowUs()) {
					activate(t);
					process();
				}
			}
			uint64_t next = end;
			for (const Th &t : threads_) {
				next = std::min(next, t.wakeUs);
			}
			next = std::min(next, std::min(peer1.nextEventUs(), peer2.nextEventUs()));
			if (next <= HostPlatform::nowUs()) {
				if (HostPlatform::nowUs() >= end) {
					break;
				}
				next = HostPlatform::nowUs() + 1000; // safety: never loop without progress
			}
			HostPlatform::advanceUs(next - HostPlatform::nowUs());
		}
	}

	// --- helpers ---
	/** Runs until motorReady() (the peer answers 0xB5 and the cyclic 0x92). Fails the test after 20 s simulated */
	void reachAvailable(RmdMotorCAN &driver) {
		uint32_t t = 0;
		while (!driver.motorReady() && t < 20000) {
			runFor(100);
			t += 100;
		}
		std::ostringstream s;
		s << "reachAvailable: motorReady() false after " << t << " ms simulated";
		REQUIRE_MESSAGE(driver.motorReady(), s.str());
	}

	FakeRmdPeer &peerOf(RmdMotorCAN &driver) { return dynamic_cast<RmdMotorCAN2 *>(&driver) ? peer2 : peer1; }

	/** Driver frames (HostTarget::canTxFrames) with arbitration id `id` */
	static std::vector<HostTarget::CanTxFrame> txFrames(uint32_t id) {
		std::vector<HostTarget::CanTxFrame> r;
		for (const auto &f : HostTarget::canTxFrames()) {
			if (f.id == id) {
				r.push_back(f);
			}
		}
		return r;
	}
	/** Driver frames with arbitration id `id` and command byte (data[0]) `cmd` */
	static std::vector<HostTarget::CanTxFrame> txFramesCmd(uint32_t id, uint8_t cmd) {
		std::vector<HostTarget::CanTxFrame> r;
		for (const auto &f : txFrames(id)) {
			if (f.data[0] == cmd) {
				r.push_back(f);
			}
		}
		return r;
	}

	/** Lets the peers decode what is pending, then clears the TX frame log and the peers' cursors */
	void clearTxFrames() {
		process();
		HostTarget::canClearTxFrames();
		peer1.syncCursor();
		peer2.syncCursor();
	}

private:
	struct Th {
		RmdMotorCAN *driver;
		TaskHandle_t handle;
		uint64_t wakeUs;
	};

	template <class T>
	Ptr<T> make() {
		Ptr<T> p(new T(), Deleter<T>{this});
		RmdMotorCAN *d = p.get();
		TaskHandle_t h = ((cpp_freertos::Thread *)d)->GetHandle(); // private base: C-style cast
		HostRtos::makeResumable(h);
		Th t{d, h, 0};
		threads_.push_back(t);
		activate(threads_.back()); // first activation: runs up to the Delay(100), the body has not run yet
		return p;
	}

	/** Runs the thread up to its next Delay, which advances the clock and parks it; the clock is put back, the jump is the wait */
	void activate(Th &t) {
		const uint64_t t0 = HostPlatform::nowUs();
		HostRtos::runUntilBlocked(t.handle);
		t.wakeUs = HostPlatform::nowUs();
		HostPlatform::setTimeUs(t0);
	}

	void forget(RmdMotorCAN *d) {
		threads_.erase(std::remove_if(threads_.begin(), threads_.end(), [d](const Th &t) { return t.driver == d; }), threads_.end());
	}

	bool txAutoComplete_ = true;
	std::vector<Th> threads_;
};

#endif
