/*
 * F1 HID entry, parameter block management (WO I1-1): F1-H1 (Create New Effect, Block Load, an index that later reports address,
 * Block Free) and F1-H3 (the pool is consistent with the number of effects the device accepts).
 *
 * Oracle: USB_PID1_01.pdf (Tests/Code_Coverage/shared/specs/USB_PID1_01.pdf)
 *  - 5.11.2 Device Managed: the host sends a Create New Effect Report with the requested effect type, then a HID Get_Report; the
 *    device answers with a PID Block Load Report containing the newly assigned Effect Block Index; "all subsequent references to the
 *    effect block by the host will use this index". Block Load Status: Success (completed), Full ("there was no room in the device's
 *    memory pool"), Error. RAM Pool Available: "the number of bytes remaining in the device's memory pool". PID Block Free Report
 *    "will contain the Block Handle to free". Device Managed Pool: "set to 1 for devices that manage their own memory pool".
 *  - 5.11.1 (PID Pool Report): RAM Pool Size "the size of the PID RAM parameter pool in bytes", Simultaneous Effects Max "the
 *    maximum number of simultaneous effects supported by the device".
 *  - 9 (the example sequence) is not a rule: it shows Effect Block Index 0 on a failed Block Load (INR 4 / IQ11, open).
 * HID 1.11 7.2.1 Get_Report, 7.2.2 Set_Report (the transfers FfbHost makes). Descriptor (DESC): read over USB by FfbHost, parsed by
 * HidReportDescriptor, reports built and decoded by PidReportBuilder: nothing here uses a firmware struct.
 *
 * Settings the cases depend on are set by CDC commands (charter rule 2 and 6): power and fxratio. A fresh flash reports power 5000
 * and fxratio 204 (axis.0.power?, axis.0.fxratio?) but the scalers that act on the torque stay 0 until the user sets them (see the
 * KNOWN_ISSUE case at the end, IB 7), so the torque cases set both.
 *
 * Settling rule of the torque cases: the force passes Axis::updateTorque's fade-in (forceFadeDuration of the callers of
 * startForceFadeIn, Axis.cpp:384, :449, :620, :640: 0.5 s or 1.0 s from multiplier 0 or 0.25) and the constant-force path; the
 * case waits 2500 ms after the last report and requires the last 100 cycles to be equal (TorqueTrace::steadyValue). The wait is
 * more than the 1.0 s of the longest fade-in, a bound from the callers, not from observed output.
 *
 * Sign (PID 4.1 / 5.1): a normalised force along an axis is positive in the positive direction of that axis; with Direction Enable
 * the Direction is a polar direction. A force along the polar direction theta acts on one axis with a factor that changes sign when
 * theta changes by 180 degrees. That reversal is the only sign property the cases assert. The PID text does not define the zero of the
 * polar angle (5.1 only says that with Direction Enable "the Direction is applied ... as a polar direction"), so which of the two
 * directions is the positive X direction is NOT asserted here: the lead's decision is that I1-3 (F1-E1) settles the absolute sign.
 */
#include "doctest.h"

#include "FfbHost.h"
#include "HidReportDescriptor.h"
#include "IntegrationFixture.h"
#include "PidReportBuilder.h"
#include "PositionScript.h"
#include "TestHelpers.h"

#include <algorithm>
#include <cstdlib>
#include <optional>
#include <set>
#include <string>
#include <vector>

using hid::ReportType;

namespace {

// The Direction collection of the Set Effect report holds two 16-bit values with Ordinal usages (page 0x0A, instances 1 and 2;
// INR 3, the descriptor dump of the I1-1 exploration); the first one is the X direction / the polar angle with Direction Enable.
constexpr uint32_t kDirectionX = hid::usage(0x0A, 1);

struct HidFixture : IntegrationFixture {
	std::optional<FfbHost> host;

	/** Chain up, steady state (0 us per SPI frame), host side created (reads the descriptor over USB) */
	void up() {
		bringUp();
		setSpiFrameUs(0);
		host.emplace(*this);
	}
	FfbHost &h() { return *host; }

	/** The effect types the Create New Effect report can select (usages of the Effect Type array field), in declaration order */
	std::vector<uint32_t> createTypes() {
		std::vector<uint32_t> out;
		const hid::HidReportDescriptor &d = h().descriptor();
		for (const hid::Report *r : d.findReportsByUsage(ReportType::Feature, pid::use::CreateNewEffectReport)) {
			for (size_t fi : r->fields) {
				const hid::Field &f = d.fields()[fi];
				bool inEt = false;
				for (const hid::CollectionRef &c : f.path) {
					inEt = inEt || c.usage == pid::use::EffectType;
				}
				if (inEt && f.isArray()) {
					out.insert(out.end(), f.usages.begin(), f.usages.end());
				}
			}
		}
		return out;
	}

	/** Declared logical range of the Effect Block Index of the Block Load report */
	void indexRange(int64_t &lo, int64_t &hi) {
		const hid::Field *f = h().descriptor().findField(ReportType::Feature, pid::use::PidBlockLoadReport, pid::use::EffectBlockIndex);
		REQUIRE(f != nullptr);
		lo = f->logicalMin;
		hi = f->logicalMax;
	}

	void configureTorque() {
		h().command("axis.0.power=5000");
		h().command("axis.0.fxratio=255");
	}

	/** Set Effect (constant force, Direction Enable, polar direction in 0.01 degrees, gain 255, duration 32767 ms = the logical maximum), Set Constant Force, Effect Operation Start */
	void startConstant(int64_t index, int64_t direction, int64_t magnitude) {
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, index).select(pid::use::EtConstantForce).set(pid::use::Duration, 32767).set(pid::use::Gain, 255);
		se.flag(pid::use::DirectionEnable, true);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		h().send(se);
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, index).set(pid::use::Magnitude, magnitude);
		h().send(sc);
		operation(index, pid::use::OpEffectStart);
	}

	void operation(int64_t index, uint32_t op) {
		pid::PidReport eo = h().builder().make(ReportType::Output, pid::use::EffectOperationReport);
		eo.set(pid::use::EffectBlockIndex, index).select(op).set(pid::use::LoopCount, 1);
		h().send(eo);
	}

	void freeBlock(int64_t index) {
		pid::PidReport bf = h().builder().make(ReportType::Output, pid::use::PidBlockFreeReport);
		bf.set(pid::use::EffectBlockIndex, index);
		h().send(bf);
	}

	/** Torque at the chip after `waitMs`, steady over the last 100 cycles (REQUIREd). The trace covers the wait */
	int16_t settledTorque(uint32_t waitMs = 2500) {
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().run(waitMs);
		h().observe(nullptr);
		const uint32_t last = ffbCycles();
		REQUIRE(last > 100);
		return trace.steadyValue(last - 99, last);
	}

	// ---- WO I1-2 helpers ----

	/** Set Effect of any type (Direction Enable, polar direction in 0.01 degrees, gain, duration in ms; 32767 = the logical maximum) */
	pid::PidReport makeSetEffect(int64_t index, uint32_t typeUsage, int64_t direction, int64_t gain = 255, int64_t duration = 32767) {
		pid::PidReport se = h().builder().make(ReportType::Output, pid::use::SetEffectReport);
		se.set(pid::use::EffectBlockIndex, index).select(typeUsage).set(pid::use::Duration, duration).set(pid::use::Gain, gain);
		se.flag(pid::use::DirectionEnable, true);
		se.setIn({pid::use::Direction}, kDirectionX, direction);
		return se;
	}
	void setEffect(int64_t index, uint32_t typeUsage, int64_t direction, int64_t gain = 255, int64_t duration = 32767) {
		h().send(makeSetEffect(index, typeUsage, direction, gain, duration));
	}
	pid::PidReport makeSetConstant(int64_t index, int64_t magnitude) {
		pid::PidReport sc = h().builder().make(ReportType::Output, pid::use::SetConstantForceReport);
		sc.set(pid::use::EffectBlockIndex, index).set(pid::use::Magnitude, magnitude);
		return sc;
	}
	void setConstant(int64_t index, int64_t magnitude) { h().send(makeSetConstant(index, magnitude)); }
	int64_t newEffect(uint32_t typeUsage) {
		const FfbHost::CreateResult c = h().createEffect(typeUsage);
		REQUIRE(c.load.success());
		return c.load.effectBlockIndex;
	}
	void control(uint32_t dcUsage) { h().send(h().builder().deviceControl(dcUsage)); }

	/** Torque at the chip of every FFB cycle of the next `ms` ms (one entry per cycle, TorqueTrace) */
	std::vector<int16_t> torques(uint32_t ms) {
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().run(ms);
		h().observe(nullptr);
		std::vector<int16_t> out;
		for (const TorqueTrace::Sample &s : trace.samples()) {
			out.push_back(s.torque);
		}
		return out;
	}

	FfbHost::PidState lastState() {
		FfbHost::PidState st;
		REQUIRE(h().latestState(st));
		return st;
	}

	/**
	 * F1-H5 body: a constant force runs at its steady torque; `bytes` is sent raw on the interrupt OUT endpoint; then the handshake is not a STALL and
	 * the endpoint is not halted, the chain recorded no fault, the torque is unchanged over the next 10 FFB cycles (and after settling), and a
	 * following valid Set Constant Force (half the magnitude) works: the torque halves (3 counts of rounding of the float chain).
	 * Returns the number of FFB cycles of the check window whose torque differed (0 = unchanged); the assertions that describe a defect come after the
	 * REQUIRE_CHAIN_MODELLED.
	 */
	void malformedLeavesTorque(const std::vector<uint8_t> &bytes, const std::string &what) {
		up();
		configureTorque();
		const int64_t idx = newEffect(pid::use::EtConstantForce);
		startConstant(idx, 9000, 10000);
		const int16_t steady = settledTorque();
		REQUIRE(steady != 0);
		const HostDcd::Handshake hs = h().sendRaw(bytes);
		INFO(what << ": " << bytes.size() << " bytes, handshake " << static_cast<int>(hs));
		const UsbHost::Interface *hid = IntegrationFixture::host.findInterface(TUSB_CLASS_HID);
		REQUIRE(hid != nullptr);
		CHECK(hs != HostDcd::Handshake::Stall);
		CHECK(hs != HostDcd::Handshake::NoResponse);
		CHECK_FALSE(HostDcd::endpoint(hid->epOut).stalled);
		const std::vector<int16_t> next = torques(10);
		REQUIRE(next.size() >= 9);
		REQUIRE_CHAIN_MODELLED();
		int differing = 0;
		for (int16_t t : next) {
			differing += (t != steady) ? 1 : 0;
		}
		MESSAGE(what << ": steady " << steady << ", first of the next cycles " << next.front() << ", last " << next.back() << ", cycles that differ " << differing);
		CHECK(differing == 0);
		CHECK(settledTorque() == steady);
		setConstant(idx, 5000);
		const int16_t half = settledTorque();
		CHECK_NEAR(2 * static_cast<int>(half), steady, 3);
	}

	/** F1-H6 setup: a running constant force (direction 9000, magnitude 10000) at its steady torque T (REQUIRE != 0), settled. Returns T; `idx` is the effect */
	int16_t pairScenario(int64_t &idx) {
		up();
		configureTorque();
		idx = newEffect(pid::use::EtConstantForce);
		startConstant(idx, 9000, 10000);
		const int16_t steady = settledTorque();
		REQUIRE(steady != 0);
		return steady;
	}
	/** The pair of F1-H6: Set Effect with the opposite direction, Set Constant Force with half the magnitude */
	void queuePair(int64_t idx) {
		h().queue(makeSetEffect(idx, pid::use::EtConstantForce, 27000));
		h().queue(makeSetConstant(idx, 5000));
	}

	/** Creates constant force effects until the Block Load status is not Success (at most maxTries); every result in order */
	std::vector<FfbHost::CreateResult> createUntilNotSuccess(uint32_t maxTries) {
		std::vector<FfbHost::CreateResult> all;
		for (uint32_t i = 0; i < maxTries; i++) {
			all.push_back(h().createEffect(pid::use::EtConstantForce));
			if (!all.back().load.success()) {
				break;
			}
		}
		return all;
	}
};

} // namespace

TEST_SUITE("F1Hid") {

// ---------------------------------------------------------------------------------------------------------------------
// F1-H1
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.11.2: the device answers the Create New Effect with a Block Load carrying the newly assigned Effect Block Index; success is
// the Block Load Success selector. The index is inside the Block Load report's declared logical range (the descriptor is the contract
// the host reads the field by) and every effect in the pool has its own index ("all subsequent references ... will use this index"
// only works if two live effects never share one).
TEST_CASE_FIXTURE(HidFixture, "F1-H1: Create New Effect returns Block Load Success and an Effect Block Index in the declared range for every effect type the descriptor offers, and the indices are distinct" * IT_DELETES_FFBHIDMAIN) {
	up();
	const std::vector<uint32_t> types = createTypes();
	REQUIRE(!types.empty());
	int64_t lo = 0, hi = 0;
	indexRange(lo, hi);
	std::set<int64_t> seen;
	for (uint32_t t : types) {
		INFO("effect type " << pid::nameOf(t));
		const FfbHost::CreateResult r = h().createEffect(t);
		CHECK(r.load.success());
		CHECK(r.load.effectBlockIndex >= lo);
		CHECK(r.load.effectBlockIndex <= hi);
		CHECK(seen.insert(r.load.effectBlockIndex).second);
	}
	CHECK(seen.size() == types.size());
	REQUIRE_CHAIN_MODELLED();
}

// A Set Effect, Set Constant Force and Effect Operation Start report addressed to the index returned for the second create make the
// torque at the chip (register 0x64, torque [31:16], DS 0x64) non-zero; Effect Operation Stop of that index takes it away again (PID
// 5.9). Only non-zero / zero is asserted (the absolute count is F1-E1's characterization, the sign is in the next case). That the index
// really selects the effect is proven by the case after the next one (two effects, different magnitudes).
TEST_CASE_FIXTURE(HidFixture, "F1-H1: a constant force started on the second created effect gives torque at the chip, and stopping that effect removes the torque" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const FfbHost::CreateResult first = h().createEffect(pid::use::EtConstantForce);
	const FfbHost::CreateResult second = h().createEffect(pid::use::EtConstantForce);
	REQUIRE(first.load.success());
	REQUIRE(second.load.success());
	REQUIRE(first.load.effectBlockIndex != second.load.effectBlockIndex);
	startConstant(second.load.effectBlockIndex, 9000, 10000);
	const int16_t running = settledTorque();
	CHECK(running != 0);
	operation(second.load.effectBlockIndex, pid::use::OpEffectStop);
	const int16_t stopped = settledTorque(500);
	CHECK(stopped == 0);
	REQUIRE_CHAIN_MODELLED();
}

// The index matters: two constant forces in the same direction, A (first create) of magnitude M and B (second create) of magnitude 2 M.
// Torque is linear in the magnitude (PID 5.6 / 4.1; ratios only, no absolute count), so with u = the torque of A alone:
//   A started alone: u;  A and B started: u + 2u = 3u;  A stopped by A's index: 2u (B alone);  then B stopped by B's index: 0.
// A firmware that sent every report to one slot would overwrite A's parameters with B's, and the torque with both started would be 2u
// (or 4u), not 3u; stopping A by its index could not leave exactly B. The tolerance of 3 counts covers the integer truncation of the
// float chain (each torque is a product truncated to an integer, once per effect path).
TEST_CASE_FIXTURE(HidFixture, "F1-H1: reports addressed to two returned indices act on their own effects: start both, stop the first by its index and the second's torque remains" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const FfbHost::CreateResult a = h().createEffect(pid::use::EtConstantForce);
	const FfbHost::CreateResult b = h().createEffect(pid::use::EtConstantForce);
	REQUIRE(a.load.success());
	REQUIRE(b.load.success());
	REQUIRE(a.load.effectBlockIndex != b.load.effectBlockIndex);
	startConstant(a.load.effectBlockIndex, 9000, 5000);
	const int16_t u = settledTorque();
	REQUIRE(u != 0);
	startConstant(b.load.effectBlockIndex, 9000, 10000);
	const int16_t both = settledTorque();
	CHECK_NEAR(both, 3 * static_cast<int>(u), 3);
	operation(a.load.effectBlockIndex, pid::use::OpEffectStop);
	const int16_t onlyB = settledTorque();
	CHECK_NEAR(onlyB, 2 * static_cast<int>(u), 3);
	operation(b.load.effectBlockIndex, pid::use::OpEffectStop);
	CHECK(settledTorque(500) == 0);
	REQUIRE_CHAIN_MODELLED();
}

// The direction the later reports act along is part of the effect addressed by the index: a Set Effect to the same index with the
// opposite polar direction (+18000 = 180 degrees) reverses the torque. Sign derivation: see the file comment (only the reversal is
// asserted). The reversal is exact here because sin(90 deg) = 1 and sin(270 deg) = -1 in float, and the count is an integer product;
// a tolerance of 2 counts covers a last-bit difference of the float sine, no more.
TEST_CASE_FIXTURE(HidFixture, "F1-H1: a Set Effect to the returned index with the opposite polar direction reverses the torque at the chip" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const FfbHost::CreateResult c = h().createEffect(pid::use::EtConstantForce);
	REQUIRE(c.load.success());
	const int64_t idx = c.load.effectBlockIndex;
	startConstant(idx, 9000, 10000);
	const int16_t forward = settledTorque();
	REQUIRE(forward != 0);
	startConstant(idx, 27000, 10000);
	const int16_t reverse = settledTorque();
	CHECK_NEAR(reverse, -forward, 2);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.11.2 (device managed pool: Create New Effect allocates, the Block Free Report frees): fill the pool, free one block, and
// the device can create again; the new block cannot get any index but the freed one because the other 39 (max - 1) are in use.
TEST_CASE_FIXTURE(HidFixture, "F1-H1: after a Block Free in a full pool Create New Effect succeeds again and returns the freed index" * IT_DELETES_FFBHIDMAIN) {
	up();
	const FfbHost::PoolResult pool = h().getPool();
	REQUIRE(pool.hasSimultaneousEffectsMax);
	// The "only the freed index is left" argument needs the declared index range to hold exactly Simultaneous Effects Max indices
	int64_t lo = 0, hi = 0;
	indexRange(lo, hi);
	REQUIRE(lo == 1);
	REQUIRE(hi - lo + 1 == pool.simultaneousEffectsMax);
	const std::vector<FfbHost::CreateResult> all = createUntilNotSuccess(static_cast<uint32_t>(pool.simultaneousEffectsMax) + 5);
	REQUIRE(all.size() == static_cast<size_t>(pool.simultaneousEffectsMax) + 1); // max successes and the Full that ended the loop
	REQUIRE(all.back().load.full());
	const int64_t freed = all[all.size() / 2].load.effectBlockIndex; // an index in the middle of the range
	freeBlock(freed);
	const FfbHost::CreateResult again = h().createEffect(pid::use::EtConstantForce);
	CHECK(again.load.success());
	CHECK(again.load.effectBlockIndex == freed);
	const FfbHost::CreateResult full = h().createEffect(pid::use::EtConstantForce);
	CHECK_FALSE(full.load.success()); // the freed block is taken again
	REQUIRE_CHAIN_MODELLED();
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-H3
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.11.1: Simultaneous Effects Max is the maximum number of effects the device supports; 5.11.2: Block Load Full means there is no
// room. So the number of Block Loads that report Success before the first one that does not equals the Pool report's value, the
// first one that does not is Full (not Error: the device is not faulty, the pool is full), and the pool stays full.
TEST_CASE_FIXTURE(HidFixture, "F1-H3: the number of successful Create New Effect requests before Block Load reports Full equals the Pool report's Simultaneous Effects Max" * IT_DELETES_FFBHIDMAIN) {
	up();
	const FfbHost::PoolResult pool = h().getPool();
	REQUIRE(pool.hasSimultaneousEffectsMax);
	REQUIRE(pool.simultaneousEffectsMax > 0);
	const uint32_t budget = static_cast<uint32_t>(pool.simultaneousEffectsMax) + 10;
	const std::vector<FfbHost::CreateResult> all = createUntilNotSuccess(budget);
	REQUIRE(all.size() <= budget);
	REQUIRE(!all.back().load.success()); // the loop ended because of a non-success, not because of the budget
	const int64_t successes = static_cast<int64_t>(all.size()) - 1;
	CHECK(successes == pool.simultaneousEffectsMax);
	CHECK(all.back().load.full());
	CHECK_FALSE(all.back().load.error());
	// a pool that is full stays full
	const FfbHost::CreateResult again = h().createEffect(pid::use::EtConstantForce);
	CHECK(again.load.full());
	// the indices of the successful blocks are distinct
	std::set<int64_t> seen;
	for (size_t i = 0; i + 1 < all.size(); i++) {
		seen.insert(all[i].load.effectBlockIndex);
	}
	CHECK(static_cast<int64_t>(seen.size()) == successes);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.11.2: "Device Managed Pool ... will be set to 1 for devices that manage their own memory pool"; the device is driven by Create
// New Effect / Block Load (the device managed protocol), so it declares itself that way. 5.11.1: RAM Pool Size is the size in bytes
// of the pool; it holds Simultaneous Effects Max blocks of at least one byte each.
TEST_CASE_FIXTURE(HidFixture, "F1-H3: the Pool report declares a device managed pool whose RAM size holds at least one byte per simultaneous effect" * IT_DELETES_FFBHIDMAIN) {
	up();
	const FfbHost::PoolResult pool = h().getPool();
	REQUIRE(pool.hasDeviceManagedPool);
	CHECK(pool.deviceManagedPool == 1);
	REQUIRE(pool.hasRamPoolSize);
	REQUIRE(pool.hasSimultaneousEffectsMax);
	CHECK(pool.ramPoolSize >= pool.simultaneousEffectsMax);
	CHECK(pool.simultaneousEffectsMax >= 1);
	REQUIRE_CHAIN_MODELLED();
}

// INR 4 / IQ11 (open): PID 1.01 section 9 is an example, not a rule, and shows Effect Block Index 0 on a failed Block Load (outside the
// declared range 1..40). What the firmware sends on failure is its own behaviour: HidFFB::new_effect sets only loadStatus = 2
// (HidFFB.cpp:262) and leaves blockLoad_report.effectBlockIndex and ramPoolAvailable at the last successful create. The values
// observed are printed. The status is not asserted here (plain case above).
TEST_CASE_FIXTURE(HidFixture, "characterization: [F1-H3] a failed Create New Effect returns the Effect Block Index of the last successful create, not 0 (INR 4)" * IT_DELETES_FFBHIDMAIN) {
	up();
	const FfbHost::PoolResult pool = h().getPool();
	REQUIRE(pool.hasSimultaneousEffectsMax);
	const std::vector<FfbHost::CreateResult> all = createUntilNotSuccess(static_cast<uint32_t>(pool.simultaneousEffectsMax) + 5);
	REQUIRE(all.size() >= 2);
	const FfbHost::BlockLoad &failed = all.back().load;
	const FfbHost::BlockLoad &lastOk = all[all.size() - 2].load;
	REQUIRE(!failed.success());
	MESSAGE("INR 4: failing Block Load: status " << failed.status << ", Effect Block Index " << failed.effectBlockIndex << ", RAM Pool Available raw " << failed.ramPoolAvailable
										<< "; last successful: index " << lastOk.effectBlockIndex << ", RAM Pool Available raw " << lastOk.ramPoolAvailable);
	CHECK(failed.effectBlockIndex == lastOk.effectBlockIndex); // INR 4: HidFFB.cpp:262 leaves the index untouched
	CHECK(failed.effectBlockIndex != 0);
	REQUIRE_CHAIN_MODELLED();
}

// IB 4: RAM Pool Available is declared as an Array field (Feature 0x00, usb_hid_ffb_desc.h Block Load report) while PID 5.11.2 calls it a
// number of bytes (a dynamic value; the section 9 example declares Variable). A host that reads the Block Load by its descriptor
// decodes an Array item as the index of a selected control, so it cannot read a count out of it. The first assertion that fails is
// the declaration; the value is also checked to be a count in range (<= RAM Pool Size), the property the declaration should give.
TEST_CASE_FIXTURE(HidFixture, "F1-H3: RAM Pool Available is declared as a Variable count of bytes"
	* KNOWN_ISSUE("(IB 4) RAM Pool Available is declared as an Array field in the PID Block Load report, so a host cannot read it as a byte count") * IT_DELETES_FFBHIDMAIN) {
	up();
	const FfbHost::PoolResult pool = h().getPool();
	const FfbHost::CreateResult r = h().createEffect(pid::use::EtConstantForce);
	REQUIRE(r.load.success());
	REQUIRE(r.load.ramPoolDeclared);
	REQUIRE_CHAIN_MODELLED();
	CHECK_FALSE(r.load.ramPoolAvailableIsArray);
	CHECK(r.load.ramPoolAvailable <= pool.ramPoolSize);
}

// PID 5.11.2: RAM Pool Available is the number of bytes remaining in the pool. The field's two bytes are read as an unsigned count (the
// value the firmware puts there; IB 4 only concerns how a host would read it by the descriptor).
// PID-only properties checked on every create: Available <= RAM Pool Size (5.11.1: the size of the pool), and Available is strictly
// smaller after each successful create (a create takes bytes out of the pool).
// Uniform-block derivation: the device manages a pool of Simultaneous Effects Max effect blocks (5.11.1) and the descriptor offers one
// pool for all effect types, with one Effect Block Index per block (Block Load index range 1..max): the blocks are equal, so
// block = RAM Pool Size / Max and after k successful creates Available = RAM Pool Size - k * block, 0 when the pool is full. (Unequal
// blocks would make Available depend on the effect type; every create here is the same type.) Both values come from the Pool report,
// no firmware constant.
// Observed (HidFFB.cpp:483): reset_ffb sets used_effects = 1 (the constructor starts at 0, HidFFB.h:193), the USB mount resets, so
// the count is one block too low after every create, and after the last successful create it wraps to 65536 - block in the 16-bit
// field. The first failing assertion is the uniform-block check of the first create (the PID-only checks of it pass).
TEST_CASE_FIXTURE(HidFixture, "F1-H3: RAM Pool Available after k successful creates is RAM Pool Size minus k blocks, and 0 when the pool is full"
	* KNOWN_ISSUE("(IB 5) used_effects starts at 1 after the USB reset (HidFFB.cpp:483), so RAM Pool Available is one block too low and wraps below 0 after the last create") * IT_DELETES_FFBHIDMAIN) {
	up();
	const FfbHost::PoolResult pool = h().getPool();
	REQUIRE(pool.hasRamPoolSize);
	REQUIRE(pool.hasSimultaneousEffectsMax);
	REQUIRE(pool.simultaneousEffectsMax > 0);
	REQUIRE(pool.ramPoolSize % pool.simultaneousEffectsMax == 0); // a whole number of bytes per effect
	const int64_t block = pool.ramPoolSize / pool.simultaneousEffectsMax;
	int64_t previous = pool.ramPoolSize + 1; // nothing known before the first create: only Available <= RAM Pool Size is required of it
	for (int64_t k = 1; k <= pool.simultaneousEffectsMax; k++) {
		const FfbHost::CreateResult r = h().createEffect(pid::use::EtConstantForce);
		REQUIRE(r.load.success());
		REQUIRE(r.load.ramPoolDeclared);
		if (k == 1) {
			REQUIRE_CHAIN_MODELLED();
		}
		INFO("create " << k << ": RAM Pool Available " << r.load.ramPoolAvailable << ", previous " << previous << ", RAM Pool Size " << pool.ramPoolSize);
		CHECK(r.load.ramPoolAvailable <= pool.ramPoolSize);
		CHECK(r.load.ramPoolAvailable < previous);
		CHECK(r.load.ramPoolAvailable == pool.ramPoolSize - k * block);
		previous = r.load.ramPoolAvailable;
	}
}

// PID 5.11.2: freeing a Parameter Block gives its memory back: after Create, Block Free and Create again there is one block in use, as
// after the first Create, so the RAM Pool Available of the second Block Load equals the first (same raw reading as above).
// Observed (HidFFB.cpp, HID_ID_BLKFRREP): the Block Free only calls EffectsCalculator::free_effect; used_effects, which RAM Pool
// Available is computed from, is never decremented, so the second Block Load shows one block less.
TEST_CASE_FIXTURE(HidFixture, "F1-H3: Block Free gives its bytes back: Create, Free, Create reports the same RAM Pool Available twice"
	* KNOWN_ISSUE("(IB 6) Block Free does not decrement used_effects (HidFFB.cpp HID_ID_BLKFRREP), so RAM Pool Available never grows back") * IT_DELETES_FFBHIDMAIN) {
	up();
	const FfbHost::CreateResult a = h().createEffect(pid::use::EtConstantForce);
	REQUIRE(a.load.success());
	freeBlock(a.load.effectBlockIndex);
	const FfbHost::CreateResult b = h().createEffect(pid::use::EtConstantForce);
	REQUIRE(b.load.success());
	REQUIRE_CHAIN_MODELLED();
	CHECK(b.load.ramPoolAvailable == a.load.ramPoolAvailable);
}

// ---------------------------------------------------------------------------------------------------------------------
// Fresh flash (IB 7; same as bug 23)
// ---------------------------------------------------------------------------------------------------------------------

// Intended: the settings the board reports are the settings in effect. After a fresh flash axis.0.power? says 5000 and
// axis.0.fxratio? says 204 (Axis.h:504, the member defaults), so a constant force of magnitude 10000 over HID reaches the chip as
// a non-zero torque without the user setting anything. Observed: Axis::restoreFlash calls setPower / setEffectRatio only when
// Flash_Read finds a stored value (Axis.cpp:255-264); on a fresh flash it does not, torqueScaler and effectRatioScaler keep their 0
// (Axis.h:506-507) and updateTorque multiplies the force by 0 (Axis.cpp:813, :863): torque 0 for ever (checked over 2.5 s).
TEST_CASE_FIXTURE(HidFixture, "F1-H1: on a fresh flash a constant force over HID reaches the chip without the user setting power and effect ratio"
	* KNOWN_ISSUE("(IB 7; same as bug 23) fresh flash: axis.0.power? and axis.0.fxratio? report non-zero defaults but torqueScaler and effectRatioScaler stay 0 (Axis.cpp:255-264, Axis.h:506-507)") * IT_DELETES_FFBHIDMAIN) {
	up();
	REQUIRE(h().getValue("axis.0.power?") > 0);
	REQUIRE(h().getValue("axis.0.fxratio?") > 0);
	const FfbHost::CreateResult c = h().createEffect(pid::use::EtConstantForce);
	REQUIRE(c.load.success());
	startConstant(c.load.effectBlockIndex, 9000, 10000);
	const int16_t torque = settledTorque();
	REQUIRE_CHAIN_MODELLED();
	CHECK(torque != 0);
}


// ---------------------------------------------------------------------------------------------------------------------
// F1-H2: one smoke case per output report the descriptor declares.
// Output reports of the descriptor (read over USB): 1 Set Effect, 2 Set Envelope, 3 Set Condition, 4 Set Periodic, 5 Set Constant Force,
// 6 Set Ramp Force, 10 Effect Operation, 11 PID Block Free, 12 PID Device Control, 13 Device Gain, and 161 (0xA1, vendor-defined, the
// firmware's HID command channel, not a PID report: not covered here, the CDC / HID command interface is I0's subject). Not declared
// (PID 1.01 section 6 example has them): Set Custom Force, Download Force Sample, Custom Force Data, PID Pool Move: the firmware's
// hidOut ignores the IDs 7 and 8 (cases HID_ID_CSTMREP, HID_ID_SMPLREP do nothing), the descriptor does not offer them, so there is nothing to send.
// Every case is a smoke test of the direction a PID section defines; the quantitative checks are in I1-3 (constant, ramp, envelope), I1-4
// (periodic, effect operation, device control), I1-5 (conditions).
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.1 (Gain, "a scaling factor ... applied to all magnitudes"): a smaller Set Effect Gain gives a smaller torque of the same sign.
// Quantitative: I1-3 F1-E1 (Effect Gain linearity).
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Set Effect with a smaller Gain lowers the torque of a running constant force and keeps its sign" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	const int16_t full = settledTorque();
	REQUIRE(full != 0);
	setEffect(idx, pid::use::EtConstantForce, 9000, 100);
	const int16_t less = settledTorque();
	MESSAGE("full " << full << " gain 100/255 " << less);
	CHECK(less != 0);
	CHECK(((less > 0) == (full > 0)));
	CHECK(std::abs(less) < std::abs(full));
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.2: the envelope shapes the effect's magnitude; with Attack Level 0 and an Attack Time the force starts below its steady value
// and reaches it after the attack time. The steady value S is measured first without an envelope (fade-in of the axis is over by then),
// the effect is stopped, the envelope set (Attack Level 0, Attack Time 2000 ms, Fade Level = Magnitude level, Fade Time 0) and the
// effect started again: the first cycles are below S, after the attack (plus the 2500 ms settling rule) the torque is S again.
// Quantitative: I1-3 F1-E4.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Set Envelope with Attack Level 0 makes a restarted constant force start below its steady torque and reach it after the attack time" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	const int16_t steady = settledTorque();
	REQUIRE(steady != 0);
	operation(idx, pid::use::OpEffectStop);
	h().run(50);
	pid::PidReport env = h().builder().make(ReportType::Output, pid::use::SetEnvelopeReport);
	env.set(pid::use::EffectBlockIndex, idx).set(pid::use::AttackLevel, 0).set(pid::use::AttackTime, 2000).set(pid::use::FadeLevel, 32767).set(pid::use::FadeTime, 0);
	h().send(env);
	operation(idx, pid::use::OpEffectStart);
	const std::vector<int16_t> early = torques(100);
	REQUIRE(!early.empty());
	MESSAGE("steady " << steady << ", first cycle " << early.front() << ", after 100 ms " << early.back());
	CHECK(std::abs(early.front()) < std::abs(steady) / 2);
	CHECK(std::abs(early.back()) < std::abs(steady));
	REQUIRE_CHAIN_MODELLED();
	const int16_t later = settledTorque(2500);
	CHECK(later == steady);
}

// PID 5.3: a spring condition gives a force that depends on the displacement from CP Offset and reverses with it. The position is
// scripted through 0x6B (PositionScript): +4096 counts (22.5 degrees) and -4096 counts. Only "zero at the centre, non-zero off it,
// opposite signs for opposite displacements, equal size for equal coefficients" is asserted (the restoring direction at the chip is the
// subject of I1-5 / I1-7).
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Set Condition gives a spring torque that is zero at the centre and reverses with the displacement" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	PositionScript script;
	script.setConstant(0);
	attachPositionScript(script);
	const int64_t idx = newEffect(pid::use::EtSpring);
	setEffect(idx, pid::use::EtSpring, 0);
	pid::PidReport cond = h().builder().make(ReportType::Output, pid::use::SetConditionReport);
	cond.set(pid::use::EffectBlockIndex, idx).set(pid::use::ParameterBlockOffset, 0).set(pid::use::CpOffset, 0);
	cond.set(pid::use::PositiveCoefficient, 10000).set(pid::use::NegativeCoefficient, 10000);
	cond.set(pid::use::PositiveSaturation, 32767).set(pid::use::NegativeSaturation, 32767).set(pid::use::DeadBand, 0);
	h().send(cond);
	operation(idx, pid::use::OpEffectStart);
	const int16_t centre = settledTorque();
	script.setConstant(4096);
	const int16_t plus = settledTorque();
	script.setConstant(-4096);
	const int16_t minus = settledTorque();
	MESSAGE("centre " << centre << ", +4096 counts " << plus << ", -4096 counts " << minus);
	CHECK(centre == 0);
	CHECK(plus != 0);
	CHECK(minus != 0);
	CHECK(((plus > 0) != (minus > 0)));
	CHECK_NEAR(plus, -minus, 3);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.5: a periodic effect's force varies over one Period. A sine of Period 500 ms and Magnitude 20000: over 1000 ms (two periods,
// after the 2500 ms settling rule) the torque takes both signs, and a period later it has the value it had (within 2 percent of the
// swing). Quantitative: I1-4 F1-E2.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Set Periodic gives a torque that changes sign within a Period and repeats after one Period" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtSine);
	setEffect(idx, pid::use::EtSine, 9000);
	pid::PidReport per = h().builder().make(ReportType::Output, pid::use::SetPeriodicReport);
	per.set(pid::use::EffectBlockIndex, idx).set(pid::use::Magnitude, 20000).set(pid::use::Offset, 0).set(pid::use::Phase, 0).set(pid::use::Period, 500);
	h().send(per);
	operation(idx, pid::use::OpEffectStart);
	h().run(2500);
	const std::vector<int16_t> t = torques(1000);
	REQUIRE(t.size() > 900);
	const int16_t hi = *std::max_element(t.begin(), t.end());
	const int16_t lo = *std::min_element(t.begin(), t.end());
	MESSAGE("min " << lo << " max " << hi << " cycles " << t.size());
	CHECK(hi > 0);
	CHECK(lo < 0);
	const int swing = hi - lo;
	int worst = 0;
	for (size_t i = 0; i + 500 < t.size(); i++) {
		worst = std::max(worst, std::abs(static_cast<int>(t[i]) - static_cast<int>(t[i + 500])));
	}
	MESSAGE("largest difference between samples 500 cycles apart " << worst << " of swing " << swing);
	CHECK(worst * 50 <= swing);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.6: Magnitude is the force. A larger Magnitude of a running constant force gives a larger |torque| of the same sign
// (linearity: I1-3 F1-E1; the three-way linearity of the F1-H1 case also shows it).
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Set Constant Force with a larger Magnitude raises the torque of a running constant force" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 5000);
	const int16_t small = settledTorque();
	setConstant(idx, 10000);
	const int16_t large = settledTorque();
	MESSAGE("magnitude 5000 -> " << small << ", 10000 -> " << large);
	REQUIRE(small != 0);
	CHECK(((large > 0) == (small > 0)));
	CHECK(std::abs(large) > std::abs(small));
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.7: a ramp force moves from Ramp Start to Ramp End over the Duration. Start +20000, End -20000, Duration 4000 ms: early in the
// effect the torque has one sign, late in it the other (a sign change cannot come from the axis fade-in, which only scales; which sign the
// Start has at the chip is I1-3's subject, so only "the sign differs" is asserted), and the torque moves monotonically in the direction
// from its early to its late value (sign(t_end - t_start), never against it) once the axis fade-in is over (from cycle 1100: the longest fade-in is 1.0 s, see the file comment; a
// monotone ramp, 3 counts of rounding of the float chain). Quantitative: I1-3 F1-E3.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Set Ramp Force moves the torque from the Ramp Start sign to the Ramp End sign over the Duration" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtRamp);
	setEffect(idx, pid::use::EtRamp, 9000, 255, 4000);
	pid::PidReport ramp = h().builder().make(ReportType::Output, pid::use::SetRampForceReport);
	ramp.set(pid::use::EffectBlockIndex, idx).set(pid::use::RampStart, 20000).set(pid::use::RampEnd, -20000);
	h().send(ramp);
	operation(idx, pid::use::OpEffectStart);
	const std::vector<int16_t> t = torques(3800);
	REQUIRE(t.size() > 3500);
	MESSAGE("cycle 400: " << t[400] << ", cycle 1900: " << t[1900] << ", cycle 3500: " << t[3500]);
	CHECK(t[400] != 0);
	CHECK(t[3500] != 0);
	CHECK(((t[400] > 0) != (t[3500] > 0)));
	const int dir = (t[3500] > t[400]) ? 1 : -1; // observed direction of the ramp; the monotonicity is checked along it
	int against = 0;
	for (size_t i = 1100; i < t.size(); i++) {
		against = std::max(against, -dir * (static_cast<int>(t[i]) - static_cast<int>(t[i - 1])));
	}
	MESSAGE("largest step against the ramp direction (" << dir << ") between cycles 1100 and the end " << against);
	CHECK(against <= 3);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.9: Effect Operation Start starts an effect that is defined but not started, Stop stops it, and it can be started again.
// A defined (Set Effect, Set Constant Force) but not started effect gives no torque. Quantitative and Loop Count / Solo: I1-4 F1-E10.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Effect Operation starts a defined effect, stops it, and starts it again" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000);
	setConstant(idx, 10000);
	CHECK(settledTorque(500) == 0);
	operation(idx, pid::use::OpEffectStart);
	const int16_t first = settledTorque();
	CHECK(first != 0);
	operation(idx, pid::use::OpEffectStop);
	CHECK(settledTorque(500) == 0);
	operation(idx, pid::use::OpEffectStart);
	const int16_t second = settledTorque();
	CHECK(second != 0);
	CHECK(second == first);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.11.2: the PID Block Free Report frees the Parameter Block; the effect no longer acts. A running constant force loses its torque
// when its block is freed.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Block Free of a running effect removes its torque" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	REQUIRE(settledTorque() != 0);
	freeBlock(idx);
	CHECK(settledTorque(500) == 0);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.10 (Device Gain: a scaling of all effects of the device): a smaller Device Gain gives a proportionally smaller torque.
// Ratio only (no absolute count): torque(g) * 255 is torque(255) * g within the rounding of one integer count per path (each of the two
// is truncated once: 255 counts of slack on the cross product, i.e. 1 count of the torque scaled by 255). Quantitative: I1-3 F1-E1.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Device Gain scales the torque of a running constant force in proportion" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	pid::PidReport gain = h().builder().make(ReportType::Output, pid::use::DeviceGainReport);
	gain.set(pid::use::DeviceGain, 255);
	h().send(gain);
	const int16_t full = settledTorque();
	REQUIRE(full != 0);
	pid::PidReport half = h().builder().make(ReportType::Output, pid::use::DeviceGainReport);
	half.set(pid::use::DeviceGain, 128);
	h().send(half);
	const int16_t scaled = settledTorque();
	MESSAGE("device gain 255 -> " << full << ", 128 -> " << scaled);
	CHECK(std::abs(static_cast<int>(scaled) * 255 - static_cast<int>(full) * 128) <= 2 * 255);
	REQUIRE_CHAIN_MODELLED();
}


// ---------------------------------------------------------------------------------------------------------------------
// F1-H2 / F1-H4: PID Device Control (5.13, Figure 16) and the PID State report (5.12)
// Encoding (PID 1.01 section 6 and Figure 16): one byte, the command's position in the usage list plus the logical minimum 1:
// 1 Enable Actuators, 2 Disable Actuators, 3 Stop All Effects, 4 Device Reset, 5 Device Pause, 6 Device Continue (PidReportBuilder::
// deviceControl). The firmware reads the byte as bit flags (HidFFB.cpp:113, ffb_control :221-239: 1 start, 2 stop, 4 stop, 8 reset, 16
// pause = stop, 32 continue = start): bytes 3, 4, 5 and 6 are 1|2, 4, 1|4 and 2|4 to it. The descriptor declares the field as 8 one-bit flags
// (usb_hid_ffb_desc.h:574-578). The owner's decision (IQ12): the command numbers 1..6 of PID 5.13 / Figure 16 are the intended encoding, so the
// declaration plus the bit-flag decoding are one defect (IB 8). Resulting behaviour of the firmware:
//   1 Enable  -> start                        (as PID)
//   2 Disable -> stop                         (as PID: effects_active false, forces 0)
//   3 Stop All-> start, then stop             (stops everything, like 2: but see the case "Stop All Effects ... Start works")
//   4 Reset   -> stop only                    (bit 4 is "stop": no reset_ffb, effects stay allocated)
//   5 Pause   -> start, then stop             (torque 0 only by accident of the flag decoding: 1|4 ends in stop_FFB; the Pause case passes for that reason)
//   6 Continue-> stop, then stop              (does NOT restart: Continue never gives the torque back)
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.13: DC Enable Actuators enables the device's actuators. After a Disable (torque 0) an Enable gives the running effect's torque back.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Device Control Enable Actuators gives the torque of a running constant force back after a Disable" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	const int16_t before = settledTorque();
	REQUIRE(before != 0);
	control(pid::use::DcDisableActuators);
	REQUIRE(settledTorque(500) == 0);
	control(pid::use::DcEnableActuators);
	const int16_t after = settledTorque();
	MESSAGE("before " << before << ", after disable + enable " << after);
	CHECK(after == before);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.13: DC Disable Actuators disables all actuators: no torque at the chip (the effect keeps its state).
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Device Control Disable Actuators takes the torque of a running constant force away" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	REQUIRE(settledTorque() != 0);
	control(pid::use::DcDisableActuators);
	CHECK(settledTorque(500) == 0);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.13: DC Stop All Effects "issues a stop on every running effect": no torque.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Device Control Stop All Effects takes the torque of a running constant force away" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	REQUIRE(settledTorque() != 0);
	control(pid::use::DcStopAllEffects);
	CHECK(settledTorque(500) == 0);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.13: Stop All Effects stops the effects, it does not free them and does not disable the actuators (that is Disable / Reset): the
// effect keeps its index and parameters, and Effect Operation Start of the same index plays it again. Root cause (IB 9, independent of the
// encoding): Stop All maps to stop_FFB (HidFFB.cpp:227-229, the start_FFB there is commented out), effects_active stays false until the next
// Set Effect (:348 start_FFB), and set_effect_operation (:400-436) never re-enables it: the Start gives no torque.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: after Device Control Stop All Effects an Effect Operation Start of the same effect plays it again"
	* KNOWN_ISSUE("(IB 9) Device Control Stop All Effects ends in stop_FFB (HidFFB.cpp:227-229, the start_FFB there is commented out) and Effect Operation Start (set_effect_operation, HidFFB.cpp:400-436) never re-enables the actuators, so a later Start gives no torque under either encoding") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	const int16_t before = settledTorque();
	REQUIRE(before != 0);
	control(pid::use::DcStopAllEffects);
	REQUIRE(settledTorque(500) == 0);
	operation(idx, pid::use::OpEffectStart);
	const int16_t again = settledTorque();
	MESSAGE("before " << before << ", after Stop All and Start " << again);
	REQUIRE_CHAIN_MODELLED();
	CHECK(again == before);
}

// PID 5.13: DC Device Reset clears all effects from memory: after it the pool is empty again, Simultaneous Effects Max effects can be
// created. Observed (IB 8): byte 4 is "stop" to ffb_control (HidFFB.cpp:227), reset_ffb (bit 8) is never called, so one effect stays allocated.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Device Control Device Reset clears the effects: the whole pool can be created again"
	* KNOWN_ISSUE("(IB 8) Device Control encoding: the descriptor declares 8 one-bit flags and the firmware decodes bit flags (HidFFB.cpp:221-239) while PID 5.13 / Figure 16 define command numbers 1..6 (owner, IQ12); Reset (4) is read as stop and reset_ffb (flag 8) is never called") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const FfbHost::PoolResult pool = h().getPool();
	REQUIRE(pool.hasSimultaneousEffectsMax);
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	REQUIRE(settledTorque() != 0);
	control(pid::use::DcDeviceReset);
	const std::vector<FfbHost::CreateResult> all = createUntilNotSuccess(static_cast<uint32_t>(pool.simultaneousEffectsMax) + 5);
	REQUIRE(!all.empty());
	const int64_t successes = static_cast<int64_t>(all.size()) - (all.back().load.success() ? 0 : 1);
	MESSAGE("creates that succeeded after the reset: " << successes << " of " << pool.simultaneousEffectsMax);
	REQUIRE_CHAIN_MODELLED();
	CHECK(successes == pool.simultaneousEffectsMax);
}

// PID 5.13: DC Device Reset "enables all actuators": a device that was reset is usable: a new constant force gives torque.
TEST_CASE_FIXTURE(HidFixture, "F1-H2: after Device Control Device Reset a new constant force gives torque" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t first = newEffect(pid::use::EtConstantForce);
	startConstant(first, 9000, 10000);
	REQUIRE(settledTorque() != 0);
	control(pid::use::DcDeviceReset);
	REQUIRE(settledTorque(500) == 0);
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	CHECK(settledTorque() != 0);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.13: DC Device Pause pauses all effects: no torque at the chip. (Passes only by accident of IB 8: byte 5 is the flags 1|4, start then stop.)
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Device Control Device Pause takes the torque of a running constant force away" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	REQUIRE(settledTorque() != 0);
	control(pid::use::DcDevicePause);
	CHECK(settledTorque(500) == 0);
	REQUIRE_CHAIN_MODELLED();
}

// PID 5.13: DC Device Continue restarts the effects that were running when the device was paused: the torque comes back. Observed (IB 8): byte 6
// is 2|4 to ffb_control, both "stop" (HidFFB.cpp:221-239); start_FFB (flag 32 = byte 32) cannot be reached by a PID byte (1..6).
TEST_CASE_FIXTURE(HidFixture, "F1-H2: Device Control Device Continue gives the torque of the paused constant force back"
	* KNOWN_ISSUE("(IB 8) Device Control encoding: the descriptor declares 8 one-bit flags and the firmware decodes bit flags (HidFFB.cpp:221-239) while PID 5.13 / Figure 16 define command numbers 1..6 (owner, IQ12); Continue (6) is read as the flags 2|4, both stop") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	const int16_t before = settledTorque();
	REQUIRE(before != 0);
	control(pid::use::DcDevicePause);
	REQUIRE(settledTorque(500) == 0);
	control(pid::use::DcDeviceContinue);
	const int16_t after = settledTorque();
	MESSAGE("before pause " << before << ", after continue " << after);
	REQUIRE_CHAIN_MODELLED();
	CHECK(after == before);
}


// ---------------------------------------------------------------------------------------------------------------------
// F1-H4: PID State input report (5.12)
// The report (descriptor: input report ID 2, 2 bytes) carries Effect Playing, Actuators Enabled, Safety Switch, Actuator Power and (INR 2)
// no Effect Block Index, so which effect is playing cannot be asserted; the flags are device-wide here. PID 1.01 does not say when the
// device must send a state report: every case asserts the LATEST captured report after the action (the host drains the IN endpoint before
// every transfer and after every step, so a report that was sent is captured). The firmware sends a state report only from
// new_effect and set_effect (HidFFB.cpp:285, :350 sendStatusReport); ffb_control and set_effect_operation do not (the calls are commented
// out, :115 and :434), so after a Device Control or an Effect Operation the latest report is the stale one of the last Create / Set Effect.
// ---------------------------------------------------------------------------------------------------------------------

// PID 5.13 / 5.12: after DC Disable Actuators the device's Actuators Enabled flag is clear. The latest report before the command says
// enabled (the Set Effect of the start sent one).
TEST_CASE_FIXTURE(HidFixture, "F1-H4: the latest PID State report after Device Control Disable Actuators shows Actuators Enabled clear"
	* KNOWN_ISSUE("(IB 10) PID State is not sent when its data change: sendStatusReport is commented out in ffb_control and set_effect_operation (HidFFB.cpp:115, :434); HID 1.11 7.2.4 Set_Idle (infinite idle rate: a report only when the data change) and PID 5.12") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	h().run(20);
	REQUIRE(lastState().hasActuatorsEnabled);
	REQUIRE(lastState().actuatorsEnabled);
	control(pid::use::DcDisableActuators);
	h().run(20);
	const FfbHost::PidState st = lastState();
	REQUIRE_CHAIN_MODELLED();
	CHECK(st.actuatorsEnabled == false);
}

// PID 5.13 / 5.12: after DC Enable Actuators of a disabled device the flag is set. The precondition report (flag clear) is the one a Create
// New Effect sends after the Disable.
TEST_CASE_FIXTURE(HidFixture, "F1-H4: the latest PID State report after Device Control Enable Actuators shows Actuators Enabled set"
	* KNOWN_ISSUE("(IB 10) PID State is not sent when its data change: sendStatusReport is commented out in ffb_control and set_effect_operation (HidFFB.cpp:115, :434); HID 1.11 7.2.4 Set_Idle (infinite idle rate: a report only when the data change) and PID 5.12") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	control(pid::use::DcDisableActuators);
	newEffect(pid::use::EtConstantForce); // sends a state report (HidFFB.cpp:285)
	h().run(20);
	REQUIRE(lastState().hasActuatorsEnabled);
	REQUIRE(lastState().actuatorsEnabled == false);
	control(pid::use::DcEnableActuators);
	h().run(20);
	const FfbHost::PidState st = lastState();
	REQUIRE_CHAIN_MODELLED();
	CHECK(st.actuatorsEnabled);
}

// PID 5.12: Device Paused means "all effects on the device are paused"; DC Disable Actuators is not a pause. A state report requested after
// the Disable (the one a Create New Effect sends) shows Actuators Enabled clear and Device Paused clear. Observed: sendStatusReport sets
// HID_EFFECT_PAUSE (bit 0 = Device Paused) whenever ffb_active is false (HidFFB.cpp:75-80), which a Disable also makes false.
TEST_CASE_FIXTURE(HidFixture, "F1-H4: a PID State report after Device Control Disable Actuators does not claim the device is paused"
	* KNOWN_ISSUE("(IB 11) the PID State flags are derived wrongly: Device Paused from ffb_active (a Disable reports paused) and Effect Playing set for a never-started effect (HidFFB.cpp:75-80, :348)") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	control(pid::use::DcDisableActuators);
	newEffect(pid::use::EtConstantForce);
	h().run(20);
	const FfbHost::PidState st = lastState();
	REQUIRE(st.hasDevicePaused);
	REQUIRE(st.hasActuatorsEnabled);
	REQUIRE(st.actuatorsEnabled == false);
	REQUIRE_CHAIN_MODELLED();
	CHECK(st.devicePaused == false);
}

// PID 5.12: after DC Device Pause the latest report shows Device Paused set. Precondition: the report of the start (not paused).
TEST_CASE_FIXTURE(HidFixture, "F1-H4: the latest PID State report after Device Control Device Pause shows Device Paused set"
	* KNOWN_ISSUE("(IB 10) PID State is not sent when its data change: sendStatusReport is commented out in ffb_control and set_effect_operation (HidFFB.cpp:115, :434); HID 1.11 7.2.4 Set_Idle (infinite idle rate: a report only when the data change) and PID 5.12") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	h().run(20);
	REQUIRE(lastState().hasDevicePaused);
	REQUIRE(lastState().devicePaused == false);
	control(pid::use::DcDevicePause);
	h().run(20);
	const FfbHost::PidState st = lastState();
	REQUIRE_CHAIN_MODELLED();
	CHECK(st.devicePaused);
}

// PID 5.12: after DC Device Continue the Device Paused flag is clear again. Precondition: a report that says paused (a Create New Effect
// after the Pause; the firmware reports paused whenever ffb_active is false).
TEST_CASE_FIXTURE(HidFixture, "F1-H4: the latest PID State report after Device Control Device Continue shows Device Paused clear"
	* KNOWN_ISSUE("(IB 10) PID State is not sent when its data change: sendStatusReport is commented out in ffb_control and set_effect_operation (HidFFB.cpp:115, :434); HID 1.11 7.2.4 Set_Idle (infinite idle rate: a report only when the data change) and PID 5.12; also IB 8: the Continue that should clear the flag is read as stop") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	control(pid::use::DcDevicePause);
	newEffect(pid::use::EtConstantForce);
	h().run(20);
	REQUIRE(lastState().hasDevicePaused);
	REQUIRE(lastState().devicePaused);
	control(pid::use::DcDeviceContinue);
	h().run(20);
	const FfbHost::PidState st = lastState();
	REQUIRE_CHAIN_MODELLED();
	CHECK(st.devicePaused == false);
}

// PID 5.12: Effect Playing "is set if the effect is currently playing". A defined effect that was not started is not playing. The report
// the Set Effect sends says playing (observed: sendStatusReport sets HID_EFFECT_PLAYING whenever ffb_active, HidFFB.cpp:75-78, and
// set_effect has just called start_FFB, :348). INR 2: the report has no Effect Block Index, so the one defined effect is the only one.
TEST_CASE_FIXTURE(HidFixture, "F1-H4: the PID State report sent for a defined but not started effect has Effect Playing clear"
	* KNOWN_ISSUE("(IB 11) the PID State flags are derived wrongly: Device Paused from ffb_active (a Disable reports paused) and Effect Playing set for a never-started effect (HidFFB.cpp:75-80, :348)") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	setEffect(idx, pid::use::EtConstantForce, 9000);
	setConstant(idx, 10000);
	h().run(20);
	const FfbHost::PidState st = lastState();
	REQUIRE(st.hasEffectPlaying);
	REQUIRE_CHAIN_MODELLED();
	CHECK(st.effectPlaying == false);
}

// PID 5.12: after Effect Operation Stop of the only running effect the latest report shows Effect Playing clear.
TEST_CASE_FIXTURE(HidFixture, "F1-H4: the latest PID State report after Effect Operation Stop of the only effect shows Effect Playing clear"
	* KNOWN_ISSUE("(IB 10) PID State is not sent when its data change: sendStatusReport is commented out in ffb_control and set_effect_operation (HidFFB.cpp:115, :434); HID 1.11 7.2.4 Set_Idle (infinite idle rate: a report only when the data change) and PID 5.12; and IB 11 (the precondition-free assertion: Effect Playing is derived from ffb_active)") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	h().run(20);
	operation(idx, pid::use::OpEffectStop);
	h().run(20);
	const FfbHost::PidState st = lastState();
	REQUIRE(st.hasEffectPlaying);
	REQUIRE_CHAIN_MODELLED();
	CHECK(st.effectPlaying == false);
}

// PID 5.12: Actuator Power "when read back indicates the current actuator power status" and Safety Switch "A control indicating the state of
// the Safety switch on the device. If the Safety Flag is set and the device is usable." The device is usable (it produces torque) and the 24 V
// supply is on, yet the firmware always reports Actuator Power set and Safety Switch clear (HidFFB.cpp:75: HID_ACTUATOR_POWER ored in every
// report, HID_SAFETY_SWITCH never). Read strictly, a usable device should have the Safety flag set; the firmware has no switch input, so the
// values are what it does today (INR 5).
TEST_CASE_FIXTURE(HidFixture, "characterization: [F1-H4] every PID State report has Actuator Power set and Safety Switch clear" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	control(pid::use::DcDisableActuators);
	newEffect(pid::use::EtConstantForce);
	h().run(20);
	const std::vector<FfbHost::PidState> all = h().states();
	REQUIRE(all.size() >= 3);
	for (const FfbHost::PidState &st : all) {
		REQUIRE(st.hasActuatorPower);
		REQUIRE(st.hasSafetySwitch);
		CHECK(st.actuatorPower); // INR 5: HidFFB.cpp:75
		CHECK(st.safetySwitch == false); // INR 5: HidFFB.cpp:75 never sets HID_SAFETY_SWITCH
	}
	REQUIRE_CHAIN_MODELLED();
}


// ---------------------------------------------------------------------------------------------------------------------
// F1-H5: robustness on the interrupt OUT endpoint (HID 1.11 7.2 / PID 7 fix the report lengths; what a device does with a report of the
// wrong length is not specified: the plain assertion is "the running effect does not change, the endpoint stays usable, the next valid
// report works"). The OUT endpoint's wMaxPacketSize is 64 (descriptor, read by the host); the Set Constant Force report is 4 bytes
// (ID, index, magnitude 16 bit), Set Effect 22 bytes.
// ---------------------------------------------------------------------------------------------------------------------

// (a) shorter than declared: a Set Constant Force with only its ID and the effect index (2 of 4 bytes). The firmware casts the buffer to
// its 4 byte struct (HidFFB.cpp:131 set_constant_effect(...(FFB_SetConstantForce_Data_t*)report)) and reads the magnitude from whatever the
// buffer held before.
// Observed: the torque falls to 39 counts (magnitude 257 = 0x0101): the missing magnitude bytes are those of the previous report in the
// endpoint buffer (the Effect Operation Start of the setup: operation 1, loop count 1). tud_hid_set_report_cb hands the firmware the whole
// 64 byte buffer and bufsize, and set_constant_effect ignores bufsize (HidFFB.cpp:130, :243).
TEST_CASE_FIXTURE(HidFixture, "F1-H5: a Set Constant Force report cut to its ID and index leaves the running constant force unchanged and the endpoint usable"
	* KNOWN_ISSUE("(IB 12) no length check of output reports: the handlers ignore bufsize and read the static epout_buf, so the missing magnitude bytes are those of the previous report (HidFFB.cpp:130, :243)") * IT_DELETES_FFBHIDMAIN) {
	malformedLeavesTorque({5, 1}, "short Set Constant Force");
}

// (a2) shorter than declared: a Set Effect cut to 3 bytes (ID, index, the first byte of the type). The firmware copies only the received bytes
// into a local struct that is not initialised (HidFFB.cpp:109 memcpy of min(sizeof, bufsize)), the rest is stack content.
// Observed: the effect's type and parameters change (torque 0, and the following valid Set Constant Force does not bring it back): the
// local struct is only filled for the received bytes, the other fields keep the struct's default member initialisers (ffb_defs.h:224-244),
// deterministic (not uninitialised stack): effect type 0 among them (HidFFB.cpp:108-110).
TEST_CASE_FIXTURE(HidFixture, "F1-H5: a Set Effect report cut to 3 bytes leaves the running constant force unchanged and the endpoint usable"
	* KNOWN_ISSUE("(IB 12) no length check of output reports: a short Set Effect is completed with the struct's default member initialisers (ffb_defs.h:224-244) after a memcpy of min(sizeof, bufsize) (HidFFB.cpp:108-110), deterministic") * IT_DELETES_FFBHIDMAIN) {
	malformedLeavesTorque({1, 1, 1}, "short Set Effect");
}

// (b) longer than declared, within one packet: a Set Constant Force with the same magnitude and 6 more bytes (10 bytes of 64).
TEST_CASE_FIXTURE(HidFixture, "F1-H5: a Set Constant Force report with trailing bytes (within one packet) leaves the running constant force unchanged and the endpoint usable" * IT_DELETES_FFBHIDMAIN) {
	// index 1 is the first effect created in the fresh pool; magnitude 10000 = 0x2710 is what the running effect has
	malformedLeavesTorque({5, 1, 0x10, 0x27, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}, "long Set Constant Force, 10 bytes");
}

// (b2) longer than one packet: 100 bytes = a full 64 byte packet (a valid Set Constant Force with the same magnitude and padding) and a
// 36 byte packet of zeros (the next report to the device: ID 0, undeclared). Claims only what it shows: trailing zero bytes are ignored.
TEST_CASE_FIXTURE(HidFixture, "F1-H5: a Set Constant Force report padded with zero bytes to 100 bytes (two packets) leaves the running constant force unchanged and the endpoint usable" * IT_DELETES_FFBHIDMAIN) {
	std::vector<uint8_t> big(100, 0);
	big[0] = 5;
	big[1] = 1;
	big[2] = 0x10;
	big[3] = 0x27;
	malformedLeavesTorque(big, "100 bytes");
}

// (b3) The same 100 byte transfer with bytes that matter: the packet boundary is the report boundary. The first 64 bytes are a valid Set
// Constant Force (magnitude 10000), the bytes after them start with 5, 1, 0, 0 = a Set Constant Force of the same effect with magnitude 0.
// If the firmware (TinyUSB HID) treated the 100 bytes as one report, the tail would be ignored (trailing bytes, as in (b)); observed: each
// 64-byte packet completes a transfer and is parsed as its own report, so the tail is a second Set Constant Force and the torque drops to 0.
// HID 1.11 does not define an output report longer than the endpoint's packet; what a device does with it is its own behaviour (INR 6).
TEST_CASE_FIXTURE(HidFixture, "characterization: [F1-H5] the bytes after the first 64 of a 100 byte OUT transfer are parsed as a new report (INR 6)" * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtConstantForce);
	startConstant(idx, 9000, 10000);
	const int16_t steady = settledTorque();
	REQUIRE(steady != 0);
	std::vector<uint8_t> big(100, 0);
	big[0] = 5;
	big[1] = static_cast<uint8_t>(idx);
	big[2] = 0x10;
	big[3] = 0x27;
	big[64] = 5; // second packet: Set Constant Force, same effect, magnitude 0
	big[65] = static_cast<uint8_t>(idx);
	CHECK(h().sendRaw(big) == HostDcd::Handshake::Ack);
	const int16_t after = settledTorque(500);
	MESSAGE("steady " << steady << ", after the 100 byte transfer " << after);
	CHECK(after == 0); // INR 6: the second packet was parsed as a report of its own
	REQUIRE_CHAIN_MODELLED();
}

// (c) a report ID the descriptor does not declare for output (0x30), 8 bytes of 0xFF.
TEST_CASE_FIXTURE(HidFixture, "F1-H5: an output report with an undeclared ID leaves the running constant force unchanged and the endpoint usable" * IT_DELETES_FFBHIDMAIN) {
	malformedLeavesTorque({0x30, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, "undeclared ID 0x30");
}

// (d) a zero-length packet on the OUT endpoint. (The last report before it is the Effect Operation Start of the setup: replaying it changes
// nothing visible here; the case after this one makes a replay visible.)
TEST_CASE_FIXTURE(HidFixture, "F1-H5: a zero-length packet on the OUT endpoint leaves the running constant force unchanged and the endpoint usable" * IT_DELETES_FFBHIDMAIN) {
	malformedLeavesTorque({}, "zero-length packet");
}

// (d2) A zero-length packet carries no report. tud_hid_set_report_cb reads the report ID from buffer[0] whatever bufsize is
// (global_callbacks.cpp:312-314), so the firmware replays the previous report from the stale endpoint buffer. Made visible with a ramp:
// after the Effect Operation Start of a 10 s ramp (start +20000, end -20000) has run 3 s, a zero-length packet replays the Start, which
// restarts the ramp (set_effect_operation sets startTime again, HidFFB.cpp:428-431): the torque jumps back towards the Ramp Start level.
// The ramp changes by well under 1 count per cycle, so after 10 cycles the torque must still be within 10 counts of its value before.
TEST_CASE_FIXTURE(HidFixture, "F1-H5: a zero-length packet after an Effect Operation Start does not restart the running ramp effect"
	* KNOWN_ISSUE("(IB 13) a zero-length OUT packet replays the previous report: tud_hid_set_report_cb reads *buffer at bufsize 0 (global_callbacks.cpp:312-314)") * IT_DELETES_FFBHIDMAIN) {
	up();
	configureTorque();
	const int64_t idx = newEffect(pid::use::EtRamp);
	setEffect(idx, pid::use::EtRamp, 9000, 255, 10000);
	pid::PidReport ramp = h().builder().make(ReportType::Output, pid::use::SetRampForceReport);
	ramp.set(pid::use::EffectBlockIndex, idx).set(pid::use::RampStart, 20000).set(pid::use::RampEnd, -20000);
	h().send(ramp);
	operation(idx, pid::use::OpEffectStart); // the last report on the endpoint
	h().run(3000);
	const std::vector<int16_t> before = torques(1);
	REQUIRE(before.size() == 1);
	const HostDcd::Handshake hs = h().sendRaw({});
	CHECK(hs != HostDcd::Handshake::Stall);
	const std::vector<int16_t> after = torques(10);
	REQUIRE(after.size() >= 9);
	MESSAGE("torque before the zero-length packet " << before[0] << ", ten cycles after " << after.back());
	REQUIRE_CHAIN_MODELLED();
	CHECK(std::abs(static_cast<int>(after.back()) - static_cast<int>(before[0])) <= 10);
}

// ---------------------------------------------------------------------------------------------------------------------
// F1-H6: reports sent within one ms act together in the next FFB cycle
// A running constant force (direction 9000 = 90 degrees, Magnitude 10000, steady torque T) gets a pair of reports for the same effect:
// Set Effect with the opposite direction (27000, sign reversal) and Set Constant Force with half the magnitude (5000). With the pair applied
// the torque is about -T/2; applied one at a time the intermediate values are -T (direction only) and T/2 (magnitude only). Candidates are
// ratios of the steady torque T, none is an absolute count. The reports are queued (FfbHost::queue): the scheduler's own USB step takes both
// in one step (ChainScheduler::usbFeed), so they are "within one ms".
// Limit (S11): the fake kernel does not preempt, so a report that arrives while calculateEffects is running on the target (the USB task
// has priority 40, FFBMAIN 30) is not modelled: the pair is applied between two FFB cycles here, never in the middle of one.
// ---------------------------------------------------------------------------------------------------------------------



// Case 1, default order of a scheduler step (USB (a), then time and timers (b), then FFBMAIN (c)): both reports are applied in step (a), so
// the first FFB cycle that shows a change (the first one after the queue) shows both: torque -T/2, never -T or T/2.
TEST_CASE_FIXTURE(HidFixture, "F1-H6: a Set Effect and a Set Constant Force queued within one ms act together in the first FFB cycle that shows a change" * IT_DELETES_FFBHIDMAIN) {
	int64_t idx = 0;
	const int16_t T = pairScenario(idx);
	queuePair(idx);
	REQUIRE(h().queued() == 2);
	const std::vector<int16_t> t = torques(6);
	REQUIRE(h().queued() == 0);
	REQUIRE(t.size() >= 5);
	std::string seq;
	for (int16_t v : t) {
		seq += std::to_string(v) + " ";
	}
	MESSAGE("T " << T << ", torque per cycle after the pair: " << seq);
	size_t first = t.size();
	for (size_t i = 0; i < t.size(); i++) {
		if (t[i] != T) {
			first = i;
			break;
		}
	}
	REQUIRE(first < t.size());
	CHECK_NEAR(2 * static_cast<int>(t[first]), -static_cast<int>(T), 4); // both applied: -T/2
	CHECK(t[first] != -T);	// not the direction alone
	CHECK(t[first] != T / 2); // not the magnitude alone
	for (size_t i = first; i < t.size(); i++) {
		CHECK_NEAR(2 * static_cast<int>(t[i]), -static_cast<int>(T), 4);
	}
	REQUIRE_CHAIN_MODELLED();
}

// Case 2: the same with the USB step moved behind the FFB step (ChainScheduler::setUsbAfterFfb): the FFB cycle of the step runs first and sees
// the old state; both reports are applied in the USB step behind it, so the next cycle shows both. The result (the torque of the pair, no
// intermediate value) does not depend on the host order; only the cycle in which it appears moves by one (plan 3.3 step 4).
// The first step is run with sched.advanceMs(1) directly, with the reports still queued before it (queued() == 2) and taken by that step
// alone (queued() == 0 after it): the feed runs only in the scheduler's own step (a), not in the host's drains or NAK retries (IB-free
// test design, fix round of I1-2). Mutation argument: if the moved USB step were removed (reports never processed in the step) t[1] would
// still be T and CHECK(first == 1) fails; if the USB step stayed before the FFB step the reports would act in t[0] and CHECK(t[0] == T) fails.
TEST_CASE_FIXTURE(HidFixture, "F1-H6: with the USB step after the FFB step the pair still acts together, one cycle later" * IT_DELETES_FFBHIDMAIN) {
	int64_t idx = 0;
	const int16_t T = pairScenario(idx);
	sched.setUsbAfterFfb(true);
	queuePair(idx);
	REQUIRE(h().queued() == 2);
	std::vector<int16_t> t;
	{
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		sched.advanceMs(1); // the scheduler's own step: FFB (c) first, then USB (a) with the feed
		CHECK(h().queued() == 0);
		h().run(5);
		h().observe(nullptr);
		for (const TorqueTrace::Sample &smp : trace.samples()) {
			t.push_back(smp.torque);
		}
	}
	REQUIRE(t.size() >= 5);
	std::string seq;
	for (int16_t v : t) {
		seq += std::to_string(v) + " ";
	}
	MESSAGE("T " << T << ", torque per cycle after the pair (USB after FFB): " << seq);
	CHECK(t[0] == T); // the FFB cycle of the step ran before the USB step
	size_t first = t.size();
	for (size_t i = 0; i < t.size(); i++) {
		if (t[i] != T) {
			first = i;
			break;
		}
	}
	REQUIRE(first < t.size());
	CHECK(first == 1);
	CHECK_NEAR(2 * static_cast<int>(t[first]), -static_cast<int>(T), 4);
	CHECK(t[first] != -T);
	CHECK(t[first] != T / 2);
	for (size_t i = first; i < t.size(); i++) {
		CHECK_NEAR(2 * static_cast<int>(t[i]), -static_cast<int>(T), 4);
	}
	REQUIRE_CHAIN_MODELLED();
}

// Case 3: the pair split over two ms (Set Effect, one FFB cycle, Set Constant Force): the cycle in between shows the half-applied state
// (the direction reversed, the magnitude not yet: -T), then the pair. This proves the test can see a half-applied pair, so cases 1 and 2
// are not vacuous: they would show -T or T/2 if the firmware applied the reports across an FFB cycle.
TEST_CASE_FIXTURE(HidFixture, "F1-H6: a pair split over two ms shows the half-applied state for exactly the cycles in between" * IT_DELETES_FFBHIDMAIN) {
	int64_t idx = 0;
	const int16_t T = pairScenario(idx);
	std::vector<int16_t> t;
	{
		TorqueTrace trace(chip1);
		trace.start(*this);
		h().observe(&trace);
		h().queue(makeSetEffect(idx, pid::use::EtConstantForce, 27000));
		h().run(1);
		h().queue(makeSetConstant(idx, 5000));
		h().run(4);
		h().observe(nullptr);
		for (const TorqueTrace::Sample &smp : trace.samples()) {
			t.push_back(smp.torque);
		}
	}
	REQUIRE(t.size() >= 4);
	std::string seq;
	for (int16_t v : t) {
		seq += std::to_string(v) + " ";
	}
	MESSAGE("T " << T << ", torque per cycle: " << seq);
	CHECK(t[0] != T);
	CHECK_NEAR(static_cast<int>(t[0]), -static_cast<int>(T), 2); // direction alone: -T
	for (size_t i = 1; i < t.size(); i++) {
		CHECK_NEAR(2 * static_cast<int>(t[i]), -static_cast<int>(T), 4); // then the pair
	}
	REQUIRE_CHAIN_MODELLED();
}

} // TEST_SUITE

