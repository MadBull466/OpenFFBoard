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
#include "TestHelpers.h"

#include <algorithm>
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

} // TEST_SUITE
