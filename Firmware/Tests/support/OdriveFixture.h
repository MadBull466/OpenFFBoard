/*
 * OdriveFixture.h
 *
 * Fixture of the ODrive CAN driver tests (ffb_driver_tests only, driver track D2, charter
 * Firmware/Tests/driver/ODRIVE_REQUIREMENTS.md "Fixture").
 *
 * OdriveFixture is a TargetFixture (target state reset before and after the test, unmodelled HAL calls fail the test) and
 *  - clears the fake flash (HostPlatform::flash()),
 *  - owns one FakeOdrivePeer per node: peer0 (node 0, ODriveCAN1 / motor 0) and peer1 (node 1, ODriveCAN2 / motor 1),
 *    both with the default cyclic heartbeat (100 ms),
 *  - TX completion: another node (the peer) acknowledges every frame. process() completes all pending mailboxes
 *    (canTxCompleteAll) before the peers decode the frames, and the fake HAL's own auto-complete is on as well, so the
 *    three mailboxes never fill up. setTxAutoComplete(false) leaves them pending: the peers then see no frame (a bus
 *    without acknowledge); the test completes mailboxes with HostTarget::canTxComplete and calls process().
 *  - FIFO 1 (node 1): ODriveCAN2 listens on receive FIFO 1 (ODriveCAN.cpp:73). On the F407 the CAN1_RX1 NVIC line is
 *    disabled (HAL_CAN_MspInit), so a frame for node 1 raises no interrupt and waits until the next CAN1_TX or CAN1_RX0
 *    interrupt. MODELLING CHOICE: the fixture ENABLES the Rx1 line (canSetIrqLineEnabled(Rx1, true)) so that frames for
 *    node 1 are delivered at once, like for node 0. setRx1Interrupt(false) restores the real F407 behaviour (then the
 *    frames are processed at the next TX completion, which the fixture's own process() produces only if the driver
 *    transmitted). The TX completion is not used as the workaround because it would make the delivery depend on the
 *    driver's transmissions. The effect of the real line state on ODriveCAN2 is characterized by the O3 tests.
 *  - the thread "ODRIVE": both instances use this name, so the fixture finds the handle of an instance through its
 *    Thread base (a C-style cast to the private base cpp_freertos::Thread: ODriveCAN inherits it privately; the firmware
 *    is not changed) and makes it resumable. The ODriveCAN::Run loop is `Delay(500); body`. A Delay jumps the simulated
 *    clock by 500 ms inside the activation, which would hide the peer's cyclic frames of that interval. The fixture
 *    therefore schedules the thread itself: it activates the thread when the clock has reached its wake time (the
 *    activation runs the body, then the Delay advances the clock by 500 ms and parks the thread) and puts the clock back
 *    to the activation time with HostPlatform::setTimeUs. The body thus runs at exactly construction time + n * 500 ms.
 *    The delay budget is 500 ms (a Delay of 500 ms parks); the destructor restores 1000 ms. The first activation (at
 *    construction) runs the first Delay only, like the real thread.
 *    Host-vs-target: the clock is set back by 500 ms after each activation. Nothing else of the ODrive tests depends on
 *    a monotonic clock, but a test must not create other timed objects across runFor().
 *  - at teardown fails the test on any FakeOdrivePeer violation the test did not declare with expectViolation().
 *
 * Time: runFor(ms) advances the simulated clock to now + ms, stepping from event to event (peer cyclic frames, delayed
 * responses, state changes, driver thread wake times). Never sleeps. peerN.advance(ms) advances the clock without
 * running the driver thread.
 *
 * Helpers: makeOdrive1() / makeOdrive2() (unique_ptr to ODriveCAN1 / ODriveCAN2, constructed after the fixture),
 * reachRunning(driver) (heartbeat IDLE -> driver requests calibration -> peer calibrates -> driver requests closed loop
 * -> driver sets the modes -> motorReady()), txFrames(id) / txFramesCmd(node, cmd), clearTxFrames().
 */
#ifndef HOSTTEST_ODRIVEFIXTURE_H_
#define HOSTTEST_ODRIVEFIXTURE_H_

#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "../fakes/FakeOdrivePeer.h" // not an include directory: fakes/Axis.h would shadow the real Axis.h
#include "HostRtos.h"
#include "TestHelpers.h"

#include "ODriveCAN.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <vector>

struct OdriveFixture : TargetFixture {
	FakeOdrivePeer peer0{0}; //!< ODrive axis of node 0 (ODriveCAN1)
	FakeOdrivePeer peer1{1}; //!< ODrive axis of node 1 (ODriveCAN2)

	static constexpr uint32_t THREAD_PERIOD_MS = 500; //!< ODriveCAN::Run: Delay(500)

	OdriveFixture() {
		HostPlatform::flash().clear();
		HostRtos::setDelayBudgetMs(THREAD_PERIOD_MS);
		HostTarget::canSetTxAutoComplete(true);
		HostTarget::canSetIrqLineEnabled(HostTarget::CanIrqLine::Rx1, true);
		peer0.reset();
		peer1.reset();
	}

	~OdriveFixture() override {
		for (FakeOdrivePeer *p : {&peer0, &peer1}) {
			for (const auto &v : p->unexpectedViolations()) {
				FAIL_CHECK("ODrive fake, node " << int(p->nodeId()) << ": " << v.text());
			}
		}
		HostRtos::setDelayBudgetMs(1000);
	}

	// --- driver objects ---
	template <class T>
	struct Deleter {
		OdriveFixture *fx;
		void operator()(T *p) const {
			fx->forget(p);
			delete p;
		}
	};
	template <class T>
	using Ptr = std::unique_ptr<T, Deleter<T>>;

	Ptr<ODriveCAN1> makeOdrive1() { return make<ODriveCAN1>(); }
	Ptr<ODriveCAN2> makeOdrive2() { return make<ODriveCAN2>(); }

	// --- CAN setup ---
	void setTxAutoComplete(bool on) {
		txAutoComplete_ = on;
		HostTarget::canSetTxAutoComplete(on);
	}
	/** true (fixture default): frames for node 1 are delivered at once; false: the real F407 (CAN1_RX1 disabled in the NVIC) */
	void setRx1Interrupt(bool on) { HostTarget::canSetIrqLineEnabled(HostTarget::CanIrqLine::Rx1, on); }

	// --- time ---
	/** Completes pending TX mailboxes (if auto-complete is on) and lets both peers decode, answer and send their due frames */
	void process() {
		if (txAutoComplete_) {
			HostTarget::canTxCompleteAll();
		}
		peer0.process();
		peer1.process();
	}

	/** Advances the simulated time by `ms`, running the peers' cyclic frames and the driver threads at their exact times */
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
			next = std::min(next, std::min(peer0.nextEventUs(), peer1.nextEventUs()));
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
	/** Brings the driver to RUNNING: needs the peer's heartbeat (default on). Fails the test with the driver's view after 20 s simulated */
	void reachRunning(ODriveCAN &driver, FakeOdrivePeer *peer = nullptr) {
		FakeOdrivePeer &p = peer ? *peer : peerOf(driver);
		(void)p;
		uint32_t t = 0;
		while (!driver.motorReady() && t < 20000) {
			runFor(100);
			t += 100;
		}
		std::ostringstream s;
		s << "reachRunning: motorReady() false after " << t << " ms simulated, peer state " << p.state();
		REQUIRE_MESSAGE(driver.motorReady(), s.str());
	}

	FakeOdrivePeer &peerOf(ODriveCAN &driver) { return dynamic_cast<ODriveCAN2 *>(&driver) ? peer1 : peer0; }

	/** Sets the state of the peer and sends one heartbeat now (cyclic heartbeats continue) */
	void heartbeat(FakeOdrivePeer &peer, uint32_t state, uint32_t axisError = 0) {
		peer.setState(state);
		peer.setAxisError(axisError);
		peer.sendHeartbeat();
	}

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
	/** Driver frames of node `node` and command id `cmd` (standard id node << 5 | cmd) */
	static std::vector<HostTarget::CanTxFrame> txFramesCmd(uint8_t node, uint8_t cmd) { return txFrames((uint32_t(node) << 5) | cmd); }

	/** Lets the peers decode what is pending, then clears the TX frame log and the peers' cursors */
	void clearTxFrames() {
		process();
		HostTarget::canClearTxFrames();
		peer0.syncCursor();
		peer1.syncCursor();
	}

private:
	struct Th {
		ODriveCAN *driver;
		TaskHandle_t handle;
		uint64_t wakeUs;
	};

	template <class T>
	Ptr<T> make() {
		Ptr<T> p(new T(), Deleter<T>{this});
		ODriveCAN *d = p.get();
		TaskHandle_t h = ((cpp_freertos::Thread *)d)->GetHandle(); // private base: C-style cast
		HostRtos::makeResumable(h);
		Th t{d, h, 0};
		t.wakeUs = HostPlatform::nowUs();
		threads_.push_back(t);
		activate(threads_.back()); // first activation: the first Delay(500), the body has not run yet
		return p;
	}

	void activate(Th &t) {
		const uint64_t t0 = HostPlatform::nowUs();
		HostRtos::runUntilBlocked(t.handle);
		HostPlatform::setTimeUs(t0); // undo the jump of the Delay
		t.wakeUs = t0 + uint64_t(THREAD_PERIOD_MS) * 1000;
	}

	void forget(ODriveCAN *d) {
		threads_.erase(std::remove_if(threads_.begin(), threads_.end(), [d](const Th &t) { return t.driver == d; }), threads_.end());
	}

	bool txAutoComplete_ = true;
	std::vector<Th> threads_;
};

#endif
