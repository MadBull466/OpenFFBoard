/*
 * VescFixture.h
 *
 * Fixture of the VESC CAN driver tests (ffb_driver_tests only, driver track D3, charter
 * Tests/Code_Coverage/claude/charters/driver/VESC_REQUIREMENTS.md "Fake peer"). Same design as OdriveFixture.h.
 *
 * VescFixture is a TargetFixture (target state reset before and after the test, unmodelled HAL calls fail the test) and
 *  - clears the fake flash (HostPlatform::flash()),
 *  - owns two FakeVescPeer units: peer0 (unit id 0x01, present) and peer1 (unit id 0x02, ABSENT by default: call
 *    peer1.setPresent(true)). Both VESC classes default to the broadcast id 255 (VESC_can_Id, VescCAN.h:142), which every
 *    present peer answers, so a test that uses two drivers or two peers must set the VESC ids (flash or `vesccanid`)
 *    or make the other peer absent (setPresent(false)). peerOf(driver) maps VESC_2 to peer1, anything else to peer0.
 *  - TX completion: another node (the peer) acknowledges every frame. process() completes all pending mailboxes
 *    (canTxCompleteAll) before the peers decode the frames, and the fake HAL's own auto-complete is on as well, so the
 *    three mailboxes never fill up. setTxAutoComplete(false) leaves them pending: the peers then see no frame (a bus
 *    without acknowledge) (note: CANPort_2B::isWaiting() is only true while ANOTHER task is blocked in the send semaphore, CANPort2B.cpp:206-212; with one driver task the send aborts after 20 ms and sends anyway, so the driver's `!isWaiting()` branches are not reachable here, charter L8).
 *  - FIFO: both VESC filters are extended-id filters on FIFO 0 (VescCAN.cpp:94-121), so there is no FIFO 1 caveat.
 *  - the thread "VESC": both instances use this name, so the fixture finds the handle of an instance through its Thread
 *    base (a C-style cast to the private base cpp_freertos::Thread: VescCAN inherits it privately; the firmware is not
 *    changed) and makes it resumable. VescCAN::Run is `while(true){ Delay(500); body }` (VescCAN.cpp:355-356). A Delay
 *    jumps the simulated clock by 500 ms inside the activation, which would hide the time in between from the peers.
 *    The fixture therefore schedules the thread itself (charter D2 L9): it activates the thread when the clock has
 *    reached its wake time (the activation runs the body, then the Delay advances the clock by 500 ms and parks the
 *    thread) and puts the clock back to the activation time with HostPlatform::setTimeUs. The body thus runs at exactly
 *    construction time + n * 500 ms. The delay budget is 500 ms; the destructor restores 1000 ms. The first activation
 *    (at construction) runs the first Delay only, like the real thread.
 *    Host-vs-target: the clock is set back by 500 ms after each activation; a test must not create other timed objects
 *    across runFor().
 *  - at teardown fails the test on any FakeVescPeer violation the test did not declare with expectViolation().
 *
 * Time: runFor(ms) advances the simulated clock to now + ms, stepping from event to event (delayed replies, STATUS
 * frames, driver thread wake times). Never sleeps. peerN.advance(ms) advances the clock without running the driver
 * thread.
 *
 * Helpers: makeVesc1() / makeVesc2() (unique_ptr to VESC_1 / VESC_2, constructed after the fixture),
 * reachReady(driver) (500 ms: FW version request -> reply 7.01 -> COMPATIBLE; next body: values request -> reply with
 * fault 0 -> READY), txFrames(id) / txFramesPacket(unit, packet), clearTxFrames(), frame decoding helpers (big endian).
 */
#ifndef HOSTTEST_VESCFIXTURE_H_
#define HOSTTEST_VESCFIXTURE_H_

#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "../fakes/FakeVescPeer.h" // not an include directory: fakes/Axis.h would shadow the real Axis.h
#include "HostRtos.h"
#include "TestHelpers.h"

#include "VescCAN.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <vector>

struct VescFixture : TargetFixture {
	FakeVescPeer peer0{0x01}; //!< VESC unit 0x01 (VESC_1 by default)
	FakeVescPeer peer1{0x02}; //!< VESC unit 0x02, absent until peer1.setPresent(true)

	static constexpr uint32_t THREAD_PERIOD_MS = 500; //!< VescCAN::Run: Delay(500)

	VescFixture() {
		HostPlatform::flash().clear();
		HostRtos::setDelayBudgetMs(THREAD_PERIOD_MS);
		HostTarget::canSetTxAutoComplete(true);
		peer0.reset();
		peer1.reset();
		peer1.setPresent(false);
	}

	~VescFixture() override {
		for (FakeVescPeer *p : {&peer0, &peer1}) {
			for (const auto &v : p->unexpectedViolations()) {
				FAIL_CHECK("VESC fake, unit " << int(p->unitId()) << ": " << v.text());
			}
		}
		HostRtos::setDelayBudgetMs(1000);
	}

	// --- driver objects ---
	template <class T>
	struct Deleter {
		VescFixture *fx;
		void operator()(T *p) const {
			fx->forget(p);
			delete p;
		}
	};
	template <class T>
	using Ptr = std::unique_ptr<T, Deleter<T>>;

	Ptr<VESC_1> makeVesc1() { return make<VESC_1>(); }
	Ptr<VESC_2> makeVesc2() { return make<VESC_2>(); }

	// --- CAN setup ---
	void setTxAutoComplete(bool on) {
		txAutoComplete_ = on;
		HostTarget::canSetTxAutoComplete(on);
	}

	// --- time ---
	/** Completes pending TX mailboxes (if auto-complete is on) and lets both peers decode, answer and send their due frames */
	void process() {
		if (txAutoComplete_) {
			HostTarget::canTxCompleteAll();
		}
		peer0.process();
		peer1.process();
	}

	/** Advances the simulated time by `ms`, running the peers' due frames and the driver threads at their exact times */
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
	/** Brings the driver to READY against its peer (default FwInfo 7.01, fault 0). Fails the test with the driver's view after 20 s simulated */
	void reachReady(VescCAN &driver, FakeVescPeer *peer = nullptr) {
		FakeVescPeer &p = peer ? *peer : peerOf(driver);
		uint32_t t = 0;
		while (!driver.motorReady() && t < 20000) {
			runFor(100);
			t += 100;
		}
		std::ostringstream s;
		s << "reachReady: motorReady() false after " << t << " ms simulated, FW version requests " << p.requestCount(FakeVescPeer::COMM_FW_VERSION)
		  << ", values requests " << p.requestCount(FakeVescPeer::COMM_GET_VALUES_SELECTIVE);
		REQUIRE_MESSAGE(driver.motorReady(), s.str());
	}

	FakeVescPeer &peerOf(VescCAN &driver) { return dynamic_cast<VESC_2 *>(&driver) ? peer1 : peer0; }

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
	/** Driver frames to unit `unit` and packet number `packet` (extended id unit | packet << 8) */
	static std::vector<HostTarget::CanTxFrame> txFramesPacket(uint8_t unit, uint8_t packet) { return txFrames(FakeVescPeer::makeId(unit, packet)); }

	/** Big endian values of a frame's data (VESC payloads are big endian; CanTxFrame::value() is little endian) */
	static uint16_t be16(const HostTarget::CanTxFrame &f, int off = 0) { return static_cast<uint16_t>((f.data[off] << 8) | f.data[off + 1]); }
	static uint32_t be32(const HostTarget::CanTxFrame &f, int off = 0) {
		return (uint32_t(f.data[off]) << 24) | (uint32_t(f.data[off + 1]) << 16) | (uint32_t(f.data[off + 2]) << 8) | f.data[off + 3];
	}
	static int32_t bei32(const HostTarget::CanTxFrame &f, int off = 0) { return static_cast<int32_t>(be32(f, off)); }
	/** The first min(dlc, 8) bytes of the frame */
	static std::vector<uint8_t> bytes(const HostTarget::CanTxFrame &f) { return f.payload(); }

	/** Lets the peers decode what is pending, then clears the TX frame log and the peers' cursors */
	void clearTxFrames() {
		process();
		HostTarget::canClearTxFrames();
		peer0.syncCursor();
		peer1.syncCursor();
	}

private:
	struct Th {
		VescCAN *driver;
		TaskHandle_t handle;
		uint64_t wakeUs;
	};

	template <class T>
	Ptr<T> make() {
		Ptr<T> p(new T(), Deleter<T>{this});
		VescCAN *d = p.get();
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

	void forget(VescCAN *d) {
		threads_.erase(std::remove_if(threads_.begin(), threads_.end(), [d](const Th &t) { return t.driver == d; }), threads_.end());
	}

	bool txAutoComplete_ = true;
	std::vector<Th> threads_;
};

#endif
