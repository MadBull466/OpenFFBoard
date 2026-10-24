/*
 * Tests of the ST AN3969 EEPROM emulation (FFBoard/Src/eeprom.c, FFBoard/Inc/eeprom.h; vendor code, tier A: tested,
 * never fixed) through its public functions EE_Init, EE_ReadVariable, EE_WriteVariable and EE_Format, directly on the
 * fake HAL FLASH (NOR semantics, see HostFlash.h) with the F407 layout: page 0 = sector 1 (0x08004000), page 1 =
 * sector 2 (0x08008000), 16 KiB each, 4095 record slots per page.
 *
 * eeprom.c does not unlock the flash itself (flash_helpers.cpp does that around every call), so the helpers below
 * unlock before and lock after each call like Flash_Init / Flash_Write / Flash_Format.
 *
 * Intended behaviour = the AN3969 design as described in the comments of eeprom.c / eeprom.h:
 *   - EE_Init "restores the pages to a known good state in case of page's status corruption after a power loss"
 *     (switch on the two page headers; invalid combinations format the emulation).
 *   - EE_Format "erases PAGE and PAGE1 and writes VALID_PAGE header to PAGE".
 *   - EE_ReadVariable returns the last stored value; EE_WriteVariable appends, and on a full page transfers the last
 *     value of every variable in VirtAddVarTab to the other page (EE_PageTransfer), the old page is erased and the
 *     new one becomes valid.
 *
 * Power-loss tests: reference run without power loss to count the program / erase operations, then one run per cut
 * point (HostFlash::cutPowerAfterPrograms / cutPowerDuringErase), power cycle (EE_Init), check that every variable has
 * its old or its new value. Return values of the firmware after the cut are not asserted. A torn program (only some
 * bits cleared) and a partial erase (a byte range erased) are a deterministic model of the indeterminate state a real
 * interrupted flash operation leaves behind.
 *
 * Suspected vendor bug V5 (Tests/PROGRESS.md): EE_VerifyPageFullyErased (eeprom.c:332-358) loops
 * `while (Address <= PAGE0_END_ADDRESS)` for both pages and steps by 4 bytes. For page 1 the loop never runs, so
 * page 1 is reported erased and never erased by EE_Format (eeprom.c:483) and the EE_Init branches at eeprom.c:187,
 * 203 and 249; for page 0 only the half-words at offsets 0, 4, 8, ... are checked. The KNOWN_ISSUE tests below
 * describe the intended behaviour and show the consequences.
 *
 * Suspected vendor bug V6 (new): the EE_Init recovery of an interrupted transfer marks the new page VALID before it
 * erases the old one (eeprom.c:286 / 296, also 177 / 187), the opposite order of EE_PageTransfer. A power loss in that
 * erase leaves two VALID pages and the next EE_Init formats the emulation.
 *
 * Host vs target: the fake memory is mapped at the target addresses, all addresses are uint32_t like on the 32-bit
 * target; the page layout uses only 16/32-bit little-endian accesses (same on both).
 */
#include "doctest.h"
#include "FlashFixture.h"
#include "TestHelpers.h"

#include <algorithm>
#include <functional>
#include <map>
#include <vector>

namespace {

constexpr uint16_t ADR_UNLISTED = 0x0F00; //!< A virtual address that is not in VirtAddVarTab

/** The variables used by most tests (all in VirtAddVarTab, no torn-address aliasing between them for mask 0) */
const std::vector<uint16_t> VARS = {ADR_CURRENT_CONFIG, ADR_CANCONF1, ADR_AXIS1_POWER};
constexpr uint16_t NEW_VAR = ADR_AXIS1_DEGREES; //!< Written by the tests (in VirtAddVarTab, not in VARS)
constexpr uint16_t NEW_VALUE = 0x1234;
constexpr uint16_t LATER_VAR = ADR_FFBWHEEL_CONF1; //!< Written after a recovery to check the store still works
constexpr uint16_t LATER_VALUE = 0x4321;

using Values = std::map<uint16_t, uint16_t>;

bool inVirtAddVarTab(uint16_t address) { return std::find(VirtAddVarTab, VirtAddVarTab + NB_OF_VAR, address) != VirtAddVarTab + NB_OF_VAR; }

struct EepromFixture : FlashFixture {
	// ---------------- The public functions, called like flash_helpers.cpp does (unlock / lock around) ----------------
	static uint16_t eeInit() {
		HAL_FLASH_Unlock();
		uint16_t result = EE_Init();
		HAL_FLASH_Lock();
		return result;
	}
	static uint16_t eeWrite(uint16_t address, uint16_t value) {
		HAL_FLASH_Unlock();
		uint16_t result = EE_WriteVariable(address, value);
		HAL_FLASH_Lock();
		return result;
	}
	static HAL_StatusTypeDef eeFormat() {
		HAL_FLASH_Unlock();
		HAL_StatusTypeDef result = EE_Format();
		HAL_FLASH_Lock();
		return result;
	}
	/** True if EE_ReadVariable finds the variable with exactly this value */
	static bool has(uint16_t address, uint16_t value) {
		uint16_t read = (uint16_t)~value;
		return EE_ReadVariable(address, &read) == 0 && read == value;
	}
	/** True if EE_ReadVariable reports the variable as not found */
	static bool missing(uint16_t address) {
		uint16_t read = 0;
		return EE_ReadVariable(address, &read) == 1;
	}
	/** Power cycle: hardware reset (memory kept), then the start-up runs EE_Init. The result is not asserted */
	static void powerCycle() {
		HostFlash::powerOn();
		DataVar = 0;
		eeInit();
	}
	static int validPages() { return (pageStatus(0) == VALID_PAGE ? 1 : 0) + (pageStatus(1) == VALID_PAGE ? 1 : 0); }

	/** Newest value of every address in a list of records (the last record of an address wins) */
	static Values newest(const std::vector<Record> &records) {
		Values values;
		for (const Record &r : records) {
			values[r.address] = r.value;
		}
		return values;
	}
	/** Builds a full valid page with records cycling through VARS (value = record index) and returns the newest values */
	static Values fullValidPage(int page) {
		fillPage(page, VALID_PAGE, RECORDS_PER_PAGE, VARS);
		return newest(pageRecords(page));
	}
	/** The program operations in the recording */
	static std::vector<HostFlash::Operation> programs() {
		std::vector<HostFlash::Operation> result;
		for (const auto &op : HostFlash::state().log) {
			if (op.kind == HostFlash::OpKind::Program) {
				result.push_back(op);
			}
		}
		return result;
	}

	/** CHECKs that every variable in `old` still has its old value; `changed` may have its old or its new value */
	static void checkOldOrNew(const Values &old, uint16_t changed, uint16_t newValue) {
		for (const auto &[address, value] : old) {
			CAPTURE(address);
			if (address == changed) {
				CHECK((has(address, value) || has(address, newValue)));
			} else {
				CHECK(has(address, value));
			}
		}
		if (old.find(changed) == old.end()) {
			CAPTURE(changed);
			CHECK((missing(changed) || has(changed, newValue)));
		}
	}

	/** Partial-erase variants of a sector erase interrupted by a power loss: erased byte range [offset, offset + length) */
	struct EraseCut {
		uint32_t offset;
		uint32_t length;
	};
	/**
	 * Erase cuts that keep the page header half-word only when nothing else is erased or all / the older records are
	 * erased. A cut that keeps the VALID header and erases the newest records but not older ones is NEEDS REVIEW 41
	 * (tested separately as a characterization).
	 */
	static std::vector<EraseCut> eraseCuts() {
		return {
			{0, 0},														   // power lost before any byte was erased
			{0, HostFlash::SECTOR_SIZE},								   // sector erased, power lost before the HAL returned
			{0, PAGE_HEADER_SIZE},										   // header erased, records left
			{PAGE_HEADER_SIZE, HostFlash::SECTOR_SIZE - PAGE_HEADER_SIZE}, // records erased, header left
			{PAGE_HEADER_SIZE, HostFlash::SECTOR_SIZE / 2},				   // older half of the records erased
		};
	}

	/**
	 * HostFlash::reset() between loop iterations, without losing the unmodelled-access check of the fixture (the
	 * fixture destructor only sees HostFlash::state().unexpected of the last iteration)
	 */
	static void resetFlashKeepingChecks() {
		const auto &unexpected = HostFlash::state().unexpected;
		INFO("first unmodelled flash access: " << (unexpected.empty() ? std::string() : unexpected.front()));
		CHECK_MESSAGE(unexpected.empty(), "the firmware reached flash that the fake does not model (see HostFlash.h)");
		HostFlash::reset();
	}

	/**
	 * Runs `action` once to count its applied program and erase operations, then once per cut point: `setup`, arm the
	 * power cut, `action` (result ignored), power cycle, `check`. Program cuts use the torn masks in `tornMasks`,
	 * erase cuts the partial-erase variants in `cuts`.
	 * The REQUIRE(powerLost()) preconditions of the cut model also run in the passing (non-KNOWN_ISSUE) loops that use
	 * this helper, so a KNOWN_ISSUE loop can not be satisfied by a broken cut model.
	 */
	static void forEveryPowerCut(const std::function<void()> &setup, const std::function<void()> &action,
								 const std::function<void()> &check, const std::vector<uint64_t> &tornMasks = {0},
								 const std::vector<EraseCut> &cuts = eraseCuts()) {
		resetFlashKeepingChecks();
		setup();
		action();
		REQUIRE_FALSE(HostFlash::powerLost());
		const uint32_t programCount = HostFlash::state().programsApplied;
		const uint32_t eraseCount = HostFlash::state().erasesApplied;
		if (programCount + eraseCount == 0) { // nothing to interrupt: check the state after a power cycle once
			powerCycle();
			check();
			return;
		}

		for (uint32_t completed = 0; completed < programCount; completed++) {
			for (uint64_t mask : tornMasks) {
				CAPTURE(completed);
				CAPTURE(mask);
				resetFlashKeepingChecks();
				setup();
				HostFlash::cutPowerAfterPrograms(completed, mask);
				action();
				REQUIRE(HostFlash::powerLost());
				powerCycle();
				check();
			}
		}
		for (uint32_t completed = 0; completed < eraseCount; completed++) {
			for (EraseCut cut : cuts) {
				CAPTURE(completed);
				CAPTURE(cut.offset);
				CAPTURE(cut.length);
				resetFlashKeepingChecks();
				setup();
				HostFlash::cutPowerDuringErase(completed, cut.offset, cut.length);
				action();
				REQUIRE(HostFlash::powerLost());
				powerCycle();
				check();
			}
		}
	}

	/** Checks that the last flash operation failed (the function stopped at the injected failure) */
	static void checkStoppedAtFailure() {
		REQUIRE_FALSE(HostFlash::state().log.empty());
		CHECK(HostFlash::state().log.back().result != HAL_OK);
	}
};

} // namespace

TEST_SUITE("EEPROM") {

// ================================================================================================================
// EE_Init: every combination of the two page headers
// ================================================================================================================

TEST_CASE_FIXTURE(EepromFixture, "EE_Init formats two erased pages so that page 0 is valid and empty") {
	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(pageStatus(1) == ERASED);
	CHECK(usedRecords(0) == 0);
	CHECK(HostFlash::state().erasesApplied == 0); // both pages are already erased: no erase needed
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init leaves an erased page 0 and a valid page 1 unchanged") {
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 7}});

	CHECK(eeInit() == EE_OK);

	CHECK(HostFlash::state().programCalls == 0);
	CHECK(HostFlash::state().eraseCalls == 0);
	CHECK(has(ADR_CURRENT_CONFIG, 7));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init erases leftover data in an erased page 0 when page 1 is valid") {
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 7}});
	writePage(0, ERASED, {{ADR_CANCONF1, 3}}); // header erased, a record left (e.g. an interrupted erase)

	CHECK(eeInit() == EE_OK);

	CHECK(HostFlash::isErased(pageBase(0), HostFlash::EEPROM_PAGE_SIZE));
	CHECK(pageStatus(1) == VALID_PAGE);
	CHECK(has(ADR_CURRENT_CONFIG, 7));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init marks a receiving page 1 valid when page 0 is erased and keeps its variables") {
	writePage(1, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 7}, {ADR_CANCONF1, 8}});

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(1) == VALID_PAGE);
	CHECK(pageStatus(0) == ERASED);
	CHECK(HostFlash::state().erasesApplied == 0);
	CHECK(has(ADR_CURRENT_CONFIG, 7));
	CHECK(has(ADR_CANCONF1, 8));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init erases leftover data in page 0 before it marks a receiving page 1 valid") {
	writePage(1, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 7}});
	writePage(0, ERASED, {{ADR_CURRENT_CONFIG, 1}});

	CHECK(eeInit() == EE_OK);

	CHECK(HostFlash::isErased(pageBase(0), HostFlash::EEPROM_PAGE_SIZE));
	CHECK(pageStatus(1) == VALID_PAGE);
	CHECK(has(ADR_CURRENT_CONFIG, 7));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init formats the emulation when page 0 is erased and page 1 has an invalid status") {
	writePage(1, 0x1234, {{ADR_CURRENT_CONFIG, 7}});

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(pageStatus(1) != VALID_PAGE);
	CHECK(missing(ADR_CURRENT_CONFIG));
	CHECK(eeWrite(ADR_CANCONF1, 5) == EE_OK);
	CHECK(has(ADR_CANCONF1, 5));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init marks a receiving page 0 valid when page 1 is erased and keeps its variables") {
	writePage(0, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 7}});

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(pageStatus(1) == ERASED);
	CHECK(has(ADR_CURRENT_CONFIG, 7));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init formats the emulation when both pages are receiving data") {
	writePage(0, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 7}});
	writePage(1, RECEIVE_DATA, {{ADR_CANCONF1, 8}});

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(usedRecords(0) == 0);
	CHECK(missing(ADR_CURRENT_CONFIG));
	CHECK(missing(ADR_CANCONF1));
}

TEST_CASE_FIXTURE(EepromFixture, "after EE_Init formatted two receiving pages a written variable reads back and formatted variables stay missing across a power cycle" *
								 KNOWN_ISSUE("V5: the format leaves page 1 RECEIVE_DATA (eeprom.c:483); writes then go to page 1 (eeprom.c:534), reads to page 0")) {
	writePage(0, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 7}});
	writePage(1, RECEIVE_DATA, {{ADR_CANCONF1, 8}});
	REQUIRE(eeInit() == EE_OK);

	CHECK(eeWrite(ADR_AXIS1_POWER, 0x0042) == EE_OK);

	REQUIRE_FLASH_ACCESSES_MODELLED();
	CHECK(has(ADR_AXIS1_POWER, 0x0042));
	powerCycle();
	CHECK(has(ADR_AXIS1_POWER, 0x0042));
	CHECK(missing(ADR_CURRENT_CONFIG));
	CHECK(missing(ADR_CANCONF1));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init completes an interrupted transfer from page 1 to page 0 and page 0 becomes valid") {
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}, {ADR_CANCONF1, 2}, {ADR_CURRENT_CONFIG, 3}});
	writePage(0, RECEIVE_DATA, {{ADR_AXIS1_POWER, 9}}); // the value written by the interrupted EE_WriteVariable

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(has(ADR_CURRENT_CONFIG, 3));
	CHECK(has(ADR_CANCONF1, 2));
	CHECK(has(ADR_AXIS1_POWER, 9));
	CHECK(usedRecords(0) == 3); // the new value plus the newest value of each other variable
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init formats the emulation when page 0 is receiving data and page 1 has an invalid status") {
	writePage(0, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 7}});
	writePage(1, 0x5A5A, {{ADR_CANCONF1, 8}});

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(pageStatus(1) != VALID_PAGE);
	CHECK(missing(ADR_CURRENT_CONFIG));
	CHECK(missing(ADR_CANCONF1));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init leaves a valid page 0 and an erased page 1 unchanged") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 7}});

	CHECK(eeInit() == EE_OK);

	CHECK(HostFlash::state().programCalls == 0);
	CHECK(HostFlash::state().eraseCalls == 0);
	CHECK(has(ADR_CURRENT_CONFIG, 7));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init completes an interrupted transfer from page 0 to page 1, page 1 becomes valid and page 0 is erased") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}, {ADR_CANCONF1, 2}, {ADR_CURRENT_CONFIG, 3}, {ADR_UNLISTED, 4}});
	writePage(1, RECEIVE_DATA, {{ADR_AXIS1_POWER, 9}});

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(1) == VALID_PAGE);
	CHECK(HostFlash::isErased(pageBase(0), HostFlash::EEPROM_PAGE_SIZE));
	CHECK(has(ADR_CURRENT_CONFIG, 3));
	CHECK(has(ADR_CANCONF1, 2));
	CHECK(has(ADR_AXIS1_POWER, 9));
	CHECK(missing(ADR_UNLISTED)); // only variables in VirtAddVarTab are transferred
}

TEST_CASE_FIXTURE(EepromFixture, "the EE_Init transfer keeps the value written first into the receiving page instead of copying the old one") {
	// EE_PageTransfer writes the new value first; EE_Init skips the variable of that first record (eeprom.c:264)
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}, {ADR_CANCONF1, 2}});
	writePage(1, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 9}});

	CHECK(eeInit() == EE_OK);

	CHECK(has(ADR_CURRENT_CONFIG, 9));
	CHECK(has(ADR_CANCONF1, 2));
	auto records = pageRecords(1);
	REQUIRE(records.size() == 2);
	CHECK(records[1].address == ADR_CANCONF1);
}

TEST_CASE_FIXTURE(EepromFixture, "the EE_Init transfer to page 0 keeps the value written first into the receiving page") {
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}, {ADR_CANCONF1, 2}});
	writePage(0, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 9}});

	CHECK(eeInit() == EE_OK);

	CHECK(has(ADR_CURRENT_CONFIG, 9));
	CHECK(has(ADR_CANCONF1, 2));
	CHECK(usedRecords(0) == 2);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init formats the emulation when both pages are valid") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 7}});
	writePage(1, VALID_PAGE, {{ADR_CANCONF1, 8}});

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(usedRecords(0) == 0);
	CHECK(missing(ADR_CURRENT_CONFIG));
	CHECK(missing(ADR_CANCONF1));
}

TEST_CASE_FIXTURE(EepromFixture, "characterization: EE_Init with a valid page 0 and an invalid page 1 status reports success and loses every variable") {
	// NEEDS REVIEW 49: the VALID_PAGE case handles every page 1 status other than VALID and ERASED as "Page0 valid, Page1
	// receive" (eeprom.c:259). The copies go through EE_FindValidPage(WRITE_IN_VALID_PAGE), which only selects page 1
	// for exactly RECEIVE_DATA, so they are appended to page 0 itself; then page 1 is marked VALID and page 0 erased.
	// Result: EE_OK, page 1 valid and empty. A torn RECEIVE_DATA header of page 1 (power loss at the first program of a
	// transfer to page 1) leads here. Invalid combinations are formatted elsewhere (also losing the data), so the
	// loss itself matches AN3969's "invalid state" handling, but the path is accidental.
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 7}});
	writePage(1, 0x1234);

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(1) == VALID_PAGE); // 0x1234 & 0x0000
	CHECK(usedRecords(1) == 0);
	CHECK(HostFlash::isErased(pageBase(0), HostFlash::EEPROM_PAGE_SIZE));
	CHECK(missing(ADR_CURRENT_CONFIG));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init formats the emulation when page 0 has an invalid status") {
	uint16_t page1Status = ERASED;
	SUBCASE("page 1 erased") { page1Status = ERASED; }
	SUBCASE("page 1 receiving") { page1Status = RECEIVE_DATA; }
	SUBCASE("page 1 invalid") { page1Status = 0x00FF; }
	CAPTURE(page1Status);
	writePage(0, 0x1234, {{ADR_CURRENT_CONFIG, 7}});
	writePage(1, page1Status);

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(usedRecords(0) == 0);
	CHECK(missing(ADR_CURRENT_CONFIG));
}

TEST_CASE_FIXTURE(EepromFixture, "characterization: EE_Init discards a valid page 1 when page 0 has an invalid status") {
	// NEEDS REVIEW 50: "Any other state -> format eeprom" (eeprom.c:308) also applies when page 1 is VALID, so a corrupted
	// page 0 header (e.g. a torn RECEIVE_DATA program at the start of a transfer to page 0) loses all variables of the
	// valid page 1.
	writePage(0, 0xFFEE); // RECEIVE_DATA (0xEEEE) programmed over 0xFFFF, interrupted after the low byte
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 7}});

	CHECK(eeInit() == EE_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(missing(ADR_CURRENT_CONFIG));
}

// ================================================================================================================
// EE_ReadVariable
// ================================================================================================================

TEST_CASE_FIXTURE(EepromFixture, "EE_ReadVariable returns the newest of several records of a variable") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}, {ADR_CANCONF1, 2}, {ADR_CURRENT_CONFIG, 3}, {ADR_CANCONF1, 4}});

	uint16_t value = 0;
	CHECK(EE_ReadVariable(ADR_CURRENT_CONFIG, &value) == 0);
	CHECK(value == 3);
	CHECK(EE_ReadVariable(ADR_CANCONF1, &value) == 0);
	CHECK(value == 4);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_ReadVariable reports a variable without a record as not found and leaves the buffer unchanged") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});

	uint16_t value = 0xBEEF;
	CHECK(EE_ReadVariable(ADR_CANCONF1, &value) == 1);
	CHECK(value == 0xBEEF);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_ReadVariable finds a variable in the first record slot") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 0x0102}});

	uint16_t value = 0;
	CHECK(EE_ReadVariable(ADR_CURRENT_CONFIG, &value) == 0);
	CHECK(value == 0x0102);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_ReadVariable finds a variable in the last record slot of a full page") {
	fillPage(0, VALID_PAGE, RECORDS_PER_PAGE - 1, {ADR_CURRENT_CONFIG});
	writeRecord(0, RECORDS_PER_PAGE - 1, {ADR_CANCONF1, 0x5678});
	REQUIRE(recordAddress(0, RECORDS_PER_PAGE - 1) + RECORD_SIZE == pageBase(0) + HostFlash::EEPROM_PAGE_SIZE);

	uint16_t value = 0;
	CHECK(EE_ReadVariable(ADR_CANCONF1, &value) == 0);
	CHECK(value == 0x5678);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_ReadVariable reads page 1 when page 1 is the valid page") {
	writePage(0, ERASED);
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 0x0A0B}});

	uint16_t value = 0;
	CHECK(EE_ReadVariable(ADR_CURRENT_CONFIG, &value) == 0);
	CHECK(value == 0x0A0B);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_ReadVariable reads the valid page and not the page receiving data") {
	int valid = 0;
	SUBCASE("page 0 valid, page 1 receiving") { valid = 0; }
	SUBCASE("page 1 valid, page 0 receiving") { valid = 1; }
	CAPTURE(valid);
	writePage(valid, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
	writePage(1 - valid, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 2}, {ADR_CANCONF1, 3}});

	CHECK(has(ADR_CURRENT_CONFIG, 1));
	CHECK(missing(ADR_CANCONF1));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_ReadVariable reports NO_VALID_PAGE when no page is valid") {
	writePage(0, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 1}});

	uint16_t value = 0xBEEF;
	CHECK(EE_ReadVariable(ADR_CURRENT_CONFIG, &value) == NO_VALID_PAGE);
	CHECK(value == 0xBEEF);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_ReadVariable ignores a record whose virtual address was not programmed") {
	// An EE_WriteVariable interrupted between the data and the address half-word
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
	HostFlash::write16(recordAddress(0, 1), 2); // data only

	CHECK(has(ADR_CURRENT_CONFIG, 1));
}

// ================================================================================================================
// EE_WriteVariable (without page transfer)
// ================================================================================================================

TEST_CASE_FIXTURE(EepromFixture, "EE_WriteVariable appends a record after the last used slot") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}, {ADR_CANCONF1, 2}});

	CHECK(eeWrite(ADR_CURRENT_CONFIG, 0x7777) == EE_OK);

	auto records = pageRecords(0);
	REQUIRE(records.size() == 3);
	CHECK(records[2].address == ADR_CURRENT_CONFIG);
	CHECK(records[2].value == 0x7777);
	CHECK(has(ADR_CURRENT_CONFIG, 0x7777));
	CHECK(HostFlash::state().zeroToOneRequests == 0);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_WriteVariable programs the data half-word before the virtual address") {
	// Power-loss safety: a record becomes visible only when its address is programmed, after the data
	writePage(0, VALID_PAGE);

	REQUIRE(eeWrite(ADR_CANCONF1, 0x00AA) == EE_OK);

	auto ops = programs();
	REQUIRE(ops.size() == 2);
	CHECK(ops[0].address == recordAddress(0, 0));
	CHECK(ops[0].data == 0x00AA);
	CHECK(ops[0].sectorOrType == FLASH_TYPEPROGRAM_HALFWORD);
	CHECK(ops[1].address == recordAddress(0, 0) + 2);
	CHECK(ops[1].data == ADR_CANCONF1);
	CHECK(ops[1].sectorOrType == FLASH_TYPEPROGRAM_HALFWORD);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_WriteVariable writes into page 1 when page 1 is the valid page") {
	writePage(0, ERASED);
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});

	CHECK(eeWrite(ADR_CANCONF1, 2) == EE_OK);

	CHECK(usedRecords(1) == 2);
	CHECK(HostFlash::isErased(pageBase(0), HostFlash::EEPROM_PAGE_SIZE));
	CHECK(has(ADR_CANCONF1, 2));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_WriteVariable writes into the page receiving data from the valid page") {
	int valid = 0;
	SUBCASE("page 0 valid, page 1 receiving") { valid = 0; }
	SUBCASE("page 1 valid, page 0 receiving") { valid = 1; }
	CAPTURE(valid);
	writePage(valid, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
	writePage(1 - valid, RECEIVE_DATA);

	CHECK(eeWrite(ADR_CANCONF1, 2) == EE_OK);

	CHECK(usedRecords(1 - valid) == 1);
	CHECK(usedRecords(valid) == 1);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_WriteVariable skips a slot whose data half-word is already programmed") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
	HostFlash::write16(recordAddress(0, 1), 0x0000); // data of an interrupted write, address still 0xFFFF

	CHECK(eeWrite(ADR_CANCONF1, 2) == EE_OK);

	CHECK(HostFlash::read16(recordAddress(0, 2)) == 2);
	CHECK(HostFlash::read16(recordAddress(0, 2) + 2) == ADR_CANCONF1);
	CHECK(HostFlash::state().zeroToOneRequests == 0);
	CHECK(has(ADR_CANCONF1, 2));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_WriteVariable uses the last record slot without a page transfer") {
	fillPage(0, VALID_PAGE, RECORDS_PER_PAGE - 1, VARS);

	CHECK(eeWrite(NEW_VAR, NEW_VALUE) == EE_OK);

	CHECK(usedRecords(0) == RECORDS_PER_PAGE);
	CHECK(pageStatus(1) == ERASED);
	CHECK(HostFlash::state().erasesApplied == 0);
	CHECK(has(NEW_VAR, NEW_VALUE));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_WriteVariable reports NO_VALID_PAGE and programs nothing when no page is valid") {
	CHECK(eeWrite(ADR_CURRENT_CONFIG, 1) == NO_VALID_PAGE);

	CHECK(HostFlash::state().programCalls == 0);
	CHECK(HostFlash::state().eraseCalls == 0);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_WriteVariable on a locked flash fails with a programming sequence error and writes nothing") {
	writePage(0, VALID_PAGE);

	CHECK(EE_WriteVariable(ADR_CURRENT_CONFIG, 1) == EE_ERROR); // without HAL_FLASH_Unlock

	CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_PGS);
	CHECK(usedRecords(0) == 0);
}

// ================================================================================================================
// EE_WriteVariable with page transfer (EE_PageTransfer)
// ================================================================================================================

TEST_CASE_FIXTURE(EepromFixture, "a write to a full page 0 moves the newest value of every variable to page 1 and page 1 becomes valid") {
	const Values old = fullValidPage(0);
	Values expected = old;
	expected[NEW_VAR] = NEW_VALUE;

	CHECK(eeWrite(NEW_VAR, NEW_VALUE) == EE_OK);

	CHECK(pageStatus(1) == VALID_PAGE);
	CHECK(HostFlash::isErased(pageBase(0), HostFlash::EEPROM_PAGE_SIZE));
	CHECK(newest(pageRecords(1)) == expected);
	CHECK(usedRecords(1) == 1 + VARS.size()); // one record per variable
	for (const auto &[address, value] : old) {
		CAPTURE(address);
		CHECK(has(address, value));
	}
	CHECK(has(NEW_VAR, NEW_VALUE));
}

TEST_CASE_FIXTURE(EepromFixture, "a write to a full page 1 moves the newest value of every variable to page 0 and page 0 becomes valid") {
	const Values old = fullValidPage(1);

	CHECK(eeWrite(NEW_VAR, NEW_VALUE) == EE_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(HostFlash::isErased(pageBase(1), HostFlash::EEPROM_PAGE_SIZE));
	CHECK(usedRecords(0) == 1 + VARS.size());
	for (const auto &[address, value] : old) {
		CAPTURE(address);
		CHECK(has(address, value));
	}
	CHECK(has(NEW_VAR, NEW_VALUE));
}

TEST_CASE_FIXTURE(EepromFixture, "the page transfer writes the new value first and does not copy the old value of the written variable") {
	fullValidPage(0);
	const uint16_t written = VARS[0];

	CHECK(eeWrite(written, NEW_VALUE) == EE_OK);

	auto records = pageRecords(1);
	REQUIRE(records.size() == VARS.size());
	CHECK(records[0].address == written);
	CHECK(records[0].value == NEW_VALUE);
	CHECK(std::count_if(records.begin(), records.end(), [&](const Record &r) { return r.address == written; }) == 1);
	CHECK(has(written, NEW_VALUE));
}

TEST_CASE_FIXTURE(EepromFixture, "the page transfer copies only variables listed in VirtAddVarTab") {
	REQUIRE_FALSE(inVirtAddVarTab(ADR_UNLISTED));
	fillPage(0, VALID_PAGE, RECORDS_PER_PAGE, {ADR_CURRENT_CONFIG, ADR_UNLISTED});

	CHECK(eeWrite(NEW_VAR, NEW_VALUE) == EE_OK);

	CHECK(missing(ADR_UNLISTED));
	CHECK(has(ADR_CURRENT_CONFIG, (uint16_t)(RECORDS_PER_PAGE - 1))); // record 4094 (even) is ADR_CURRENT_CONFIG
	CHECK(has(NEW_VAR, NEW_VALUE));
}

TEST_CASE_FIXTURE(EepromFixture, "a variable not listed in VirtAddVarTab keeps the value written during a page transfer") {
	REQUIRE_FALSE(inVirtAddVarTab(ADR_UNLISTED));
	const Values old = fullValidPage(0);

	CHECK(eeWrite(ADR_UNLISTED, NEW_VALUE) == EE_OK);

	CHECK(pageStatus(1) == VALID_PAGE);
	CHECK(has(ADR_UNLISTED, NEW_VALUE));
	for (const auto &[address, value] : old) {
		CAPTURE(address);
		CHECK(has(address, value));
	}
}

TEST_CASE_FIXTURE(EepromFixture, "the page transfer marks the new page receiving first and valid only after the old page is erased") {
	fullValidPage(0);

	REQUIRE(eeWrite(NEW_VAR, NEW_VALUE) == EE_OK);

	const auto &log = HostFlash::state().log;
	REQUIRE(log.size() == 1 + 2 + 2 * VARS.size() + 1 + 1);
	CHECK(log.front().kind == HostFlash::OpKind::Program);
	CHECK(log.front().address == pageBase(1));
	CHECK(log.front().data == RECEIVE_DATA);
	const auto &erase = log[log.size() - 2];
	CHECK(erase.kind == HostFlash::OpKind::Erase);
	CHECK(erase.sectorOrType == FLASH_SECTOR_1);
	CHECK(log.back().kind == HostFlash::OpKind::Program);
	CHECK(log.back().address == pageBase(1));
	CHECK(log.back().data == VALID_PAGE);
	CHECK(HostFlash::state().erasesPerSector[2] == 0); // the new page is not erased
}

TEST_CASE_FIXTURE(EepromFixture, "a page transfer from page 1 erases sector 2 and keeps sector 1") {
	fullValidPage(1);

	REQUIRE(eeWrite(NEW_VAR, NEW_VALUE) == EE_OK);

	CHECK(HostFlash::state().erasesPerSector[2] == 1);
	CHECK(HostFlash::state().erasesPerSector[1] == 0);
	CHECK(HostFlash::state().log.front().address == pageBase(0));
}

// ================================================================================================================
// EE_Format
// ================================================================================================================

TEST_CASE_FIXTURE(EepromFixture, "EE_Format erases page 0 and marks it valid") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}, {ADR_CANCONF1, 2}});

	CHECK(eeFormat() == HAL_OK);

	CHECK(pageStatus(0) == VALID_PAGE);
	CHECK(usedRecords(0) == 0);
	CHECK(HostFlash::state().erasesPerSector[1] == 1);
	CHECK(missing(ADR_CURRENT_CONFIG));
	CHECK(HostFlash::state().zeroToOneRequests == 0);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Format does not erase a page 0 that is already erased") {
	CHECK(eeFormat() == HAL_OK);

	CHECK(HostFlash::state().erasesApplied == 0);
	CHECK(pageStatus(0) == VALID_PAGE);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Format on a locked flash fails and page 0 is not valid") {
	writePage(0, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 1}});

	CHECK(EE_Format() == HAL_ERROR); // the erase is ignored by the locked flash, the program fails

	CHECK(pageStatus(0) == RECEIVE_DATA);
}

// ================================================================================================================
// Suspected vendor bug V5: EE_VerifyPageFullyErased (eeprom.c:332-358)
// ================================================================================================================

TEST_CASE_FIXTURE(EepromFixture, "EE_Format erases page 1 when it contains data" *
								 KNOWN_ISSUE("V5: EE_VerifyPageFullyErased reports page 1 as erased (eeprom.c:338), EE_Format never erases it (eeprom.c:483)")) {
	writePage(0, ERASED);
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});

	CHECK(eeFormat() == HAL_OK);

	REQUIRE_FLASH_ACCESSES_MODELLED();
	CHECK(HostFlash::isErased(pageBase(1), HostFlash::EEPROM_PAGE_SIZE));
	CHECK(pageStatus(0) == VALID_PAGE);
}

TEST_CASE_FIXTURE(EepromFixture, "a variable written after formatting a store whose page 1 was valid can be read back" *
								 KNOWN_ISSUE("V5: page 1 stays VALID after EE_Format; writes go to page 1, reads to page 0 (eeprom.c:483, 519, 549)")) {
	// Normal state after one page transfer from page 0 to page 1, then Flash_Format (e.g. the format command)
	writePage(0, ERASED);
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
	REQUIRE(eeFormat() == HAL_OK);

	CHECK(eeWrite(ADR_CANCONF1, 0x0042) == EE_OK);

	REQUIRE_FLASH_ACCESSES_MODELLED();
	CHECK(has(ADR_CANCONF1, 0x0042));
}

TEST_CASE_FIXTURE(EepromFixture, "after EE_Init completed an interrupted transfer to page 0 page 1 is erased and a written variable can be read back" *
								 KNOWN_ISSUE("V5: EE_Init does not erase page 1 after the transfer (eeprom.c:187), both pages stay VALID")) {
	writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}, {ADR_CANCONF1, 2}});
	writePage(0, RECEIVE_DATA, {{ADR_AXIS1_POWER, 9}});
	REQUIRE(eeInit() == EE_OK);

	CHECK(eeWrite(ADR_CANCONF1, 0x0042) == EE_OK);

	REQUIRE_FLASH_ACCESSES_MODELLED();
	CHECK(has(ADR_CANCONF1, 0x0042));
	CHECK(pageStatus(1) == ERASED);
}

TEST_CASE_FIXTURE(EepromFixture, "records left in page 1 with an erased header do not reappear after the next page transfer" *
								 KNOWN_ISSUE("V5: EE_Init does not erase page 1 (eeprom.c:249), the next transfer to page 1 appends to old records")) {
	// Page 1 header erased but records left: e.g. a power loss while erasing page 1 at the end of a transfer to page 0
	// (EE_Init then takes the RECEIVE_DATA / ERASED branch, also without erasing page 1, eeprom.c:203)
	REQUIRE_FALSE(inVirtAddVarTab(ADR_UNLISTED));
	fullValidPage(0);
	writePage(1, ERASED, {{ADR_UNLISTED, 0x0BAD}});
	REQUIRE(eeInit() == EE_OK);

	REQUIRE(eeWrite(NEW_VAR, NEW_VALUE) == EE_OK); // page 0 full: transfer to page 1

	REQUIRE_FLASH_ACCESSES_MODELLED();
	CHECK(missing(ADR_UNLISTED));
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init erases a page 0 in which only virtual-address half-words are programmed" *
								 KNOWN_ISSUE("V5: EE_VerifyPageFullyErased checks only every other half-word (eeprom.c:353), a record with data 0xFFFF is not seen")) {
	// A record of a variable with the value 0xFFFF left in page 0 (header erased), page 1 valid
	REQUIRE_FALSE(inVirtAddVarTab(ADR_UNLISTED));
	writePage(0, ERASED, {{ADR_UNLISTED, 0xFFFF}});
	fullValidPage(1);
	REQUIRE(eeInit() == EE_OK);

	REQUIRE_FLASH_ACCESSES_MODELLED();
	CHECK(HostFlash::isErased(pageBase(0), HostFlash::EEPROM_PAGE_SIZE));
	// Consequence: after the next transfer to page 0 the stale record is visible again
	REQUIRE(eeWrite(NEW_VAR, NEW_VALUE) == EE_OK);
	CHECK(missing(ADR_UNLISTED));
}

// ================================================================================================================
// Error propagation (injected HAL errors)
// ================================================================================================================

TEST_CASE_FIXTURE(EepromFixture, "EE_Init returns the status of a failed flash operation and stops") {
	SUBCASE("page 0 erased with leftovers, page 1 valid: erase of page 0 fails") {
		writePage(0, ERASED, {{ADR_CANCONF1, 3}});
		writePage(1, VALID_PAGE);
		HostFlash::failNextWithStatus(1, HAL_TIMEOUT, HostFlash::Target::Erase);
		CHECK(eeInit() == HAL_TIMEOUT);
	}
	SUBCASE("page 0 erased with leftovers, page 1 receiving: erase of page 0 fails") {
		writePage(0, ERASED, {{ADR_CANCONF1, 3}});
		writePage(1, RECEIVE_DATA);
		HostFlash::failNextWithStatus(1, HAL_TIMEOUT, HostFlash::Target::Erase);
		CHECK(eeInit() == HAL_TIMEOUT);
		CHECK(pageStatus(1) == RECEIVE_DATA);
	}
	SUBCASE("page 0 erased, page 1 receiving: marking page 1 valid fails") {
		writePage(1, RECEIVE_DATA);
		HostFlash::failNext(1, FLASH_FLAG_PGPERR, HostFlash::Target::Program);
		CHECK(eeInit() == HAL_ERROR);
	}
	SUBCASE("page 0 erased, page 1 invalid: the format fails") {
		writePage(1, 0x1234);
		HostFlash::failNextWithStatus(1, HAL_BUSY, HostFlash::Target::Program);
		CHECK(eeInit() == HAL_BUSY);
	}
	SUBCASE("page 0 receiving, page 1 valid: copying a variable fails") {
		writePage(0, RECEIVE_DATA);
		writePage(1, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
		HostFlash::failNext(1, FLASH_FLAG_PGSERR, HostFlash::Target::Program);
		CHECK(eeInit() == HAL_ERROR);
		CHECK(pageStatus(0) == RECEIVE_DATA);
	}
	SUBCASE("page 0 receiving, page 1 valid: marking page 0 valid fails") {
		writePage(0, RECEIVE_DATA);
		writePage(1, VALID_PAGE);
		HostFlash::failNext(1, FLASH_FLAG_PGSERR, HostFlash::Target::Program);
		CHECK(eeInit() == HAL_ERROR);
		CHECK(pageStatus(0) == RECEIVE_DATA);
	}
	SUBCASE("page 0 receiving, page 1 erased: marking page 0 valid fails") {
		writePage(0, RECEIVE_DATA);
		HostFlash::failNextWithStatus(1, HAL_TIMEOUT, HostFlash::Target::Program);
		CHECK(eeInit() == HAL_TIMEOUT);
	}
	SUBCASE("page 0 receiving, page 1 invalid: the format fails") {
		writePage(0, RECEIVE_DATA, {{ADR_CURRENT_CONFIG, 1}});
		writePage(1, 0x1234);
		HostFlash::failNextWithStatus(1, HAL_TIMEOUT, HostFlash::Target::Erase);
		CHECK(eeInit() == HAL_TIMEOUT);
	}
	SUBCASE("both pages valid: the format fails") {
		writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
		writePage(1, VALID_PAGE);
		HostFlash::failNextWithStatus(1, HAL_TIMEOUT, HostFlash::Target::Erase);
		CHECK(eeInit() == HAL_TIMEOUT);
	}
	SUBCASE("page 0 valid, page 1 receiving: copying a variable fails") {
		writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
		writePage(1, RECEIVE_DATA);
		HostFlash::failNextAfter(1, 1, FLASH_FLAG_PGPERR, HostFlash::Target::Program); // the address half-word
		CHECK(eeInit() == HAL_ERROR);
		CHECK(pageStatus(1) == RECEIVE_DATA);
	}
	SUBCASE("page 0 valid, page 1 receiving: marking page 1 valid fails") {
		writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
		writePage(1, RECEIVE_DATA);
		HostFlash::failNextAfter(2, 1, FLASH_FLAG_PGPERR, HostFlash::Target::Program);
		CHECK(eeInit() == HAL_ERROR);
		// No state check here: which of "mark page 1 valid" and "erase page 0" comes first is suspected bug V6
	}
	SUBCASE("page 0 valid, page 1 receiving: erasing page 0 fails") {
		writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
		writePage(1, RECEIVE_DATA);
		HostFlash::failNextWithStatus(1, HAL_TIMEOUT, HostFlash::Target::Erase);
		CHECK(eeInit() == HAL_TIMEOUT);
		// No state check here: today page 1 is already VALID when the erase fails, so both pages are VALID and the next
		// EE_Init formats (suspected bug V6); with the safe order the erase fails before page 1 is marked
	}
	SUBCASE("page 0 invalid: the format fails") {
		writePage(0, 0x1234);
		HostFlash::failNextWithStatus(1, HAL_TIMEOUT, HostFlash::Target::Erase);
		CHECK(eeInit() == HAL_TIMEOUT);
	}
	checkStoppedAtFailure();
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Format returns the status of a failed flash operation and stops") {
	writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}});
	SUBCASE("erasing page 0 fails") {
		HostFlash::failNextWithStatus(1, HAL_TIMEOUT, HostFlash::Target::Erase);
		CHECK(eeFormat() == HAL_TIMEOUT);
		CHECK(HostFlash::state().programCalls == 0);
	}
	SUBCASE("marking page 0 valid fails") {
		HostFlash::failNext(1, FLASH_FLAG_WRPERR, HostFlash::Target::Program);
		CHECK(eeFormat() == HAL_ERROR);
		CHECK(HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP);
		CHECK(pageStatus(0) == ERASED);
	}
	checkStoppedAtFailure();
}

TEST_CASE_FIXTURE(EepromFixture, "EE_WriteVariable returns the status of a failed program operation") {
	writePage(0, VALID_PAGE);
	SUBCASE("the data half-word fails") {
		HostFlash::failNextWithStatus(1, HAL_TIMEOUT, HostFlash::Target::Program);
		CHECK(eeWrite(ADR_CURRENT_CONFIG, 1) == HAL_TIMEOUT);
		CHECK(HostFlash::state().programCalls == 1);
	}
	SUBCASE("the virtual address half-word fails") {
		HostFlash::failNextAfter(1, 1, FLASH_FLAG_PGPERR, HostFlash::Target::Program);
		CHECK(eeWrite(ADR_CURRENT_CONFIG, 1) == HAL_ERROR);
	}
	checkStoppedAtFailure();
	CHECK(missing(ADR_CURRENT_CONFIG));
}

TEST_CASE_FIXTURE(EepromFixture, "a page transfer returns the status of a failed flash operation and stops") {
	fullValidPage(0);
	const uint32_t transferPrograms = 2 * (uint32_t)VARS.size();
	SUBCASE("marking page 1 receiving fails") {
		HostFlash::failNext(1, FLASH_FLAG_PGSERR, HostFlash::Target::Program);
		CHECK(eeWrite(NEW_VAR, NEW_VALUE) == HAL_ERROR);
		CHECK(pageStatus(1) == ERASED);
	}
	SUBCASE("the data of the new value fails") {
		HostFlash::failNextAfter(1, 1, FLASH_FLAG_PGSERR, HostFlash::Target::Program);
		CHECK(eeWrite(NEW_VAR, NEW_VALUE) == HAL_ERROR);
	}
	SUBCASE("the virtual address of the new value fails") {
		HostFlash::failNextAfter(2, 1, FLASH_FLAG_PGSERR, HostFlash::Target::Program);
		CHECK(eeWrite(NEW_VAR, NEW_VALUE) == HAL_ERROR);
	}
	SUBCASE("copying a variable fails") {
		HostFlash::failNextAfter(3, 1, FLASH_FLAG_PGSERR, HostFlash::Target::Program);
		CHECK(eeWrite(NEW_VAR, NEW_VALUE) == HAL_ERROR);
		CHECK(HostFlash::state().eraseCalls == 0);
	}
	SUBCASE("erasing the old page fails") {
		HostFlash::failNextWithStatus(1, HAL_TIMEOUT, HostFlash::Target::Erase);
		CHECK(eeWrite(NEW_VAR, NEW_VALUE) == HAL_TIMEOUT);
		CHECK(pageStatus(0) == VALID_PAGE);
	}
	SUBCASE("marking page 1 valid fails") {
		HostFlash::failNextAfter(3 + transferPrograms, 1, FLASH_FLAG_PGSERR, HostFlash::Target::Program);
		CHECK(eeWrite(NEW_VAR, NEW_VALUE) == HAL_ERROR);
		CHECK(pageStatus(1) == RECEIVE_DATA);
	}
	checkStoppedAtFailure();
	CHECK(pageStatus(1) != VALID_PAGE);
}

TEST_CASE_FIXTURE(EepromFixture, "EE_Init restores every variable after any failed step of a page transfer") {
	const uint32_t steps = 1 + 2 + 2 * (uint32_t)VARS.size() + 1 + 1; // programs, the erase, the VALID program
	const uint32_t eraseStep = steps - 2;
	for (uint32_t failing = 0; failing < steps; failing++) {
		CAPTURE(failing);
		resetFlashKeepingChecks();
		const Values old = fullValidPage(0);
		if (failing == eraseStep) {
			HostFlash::failNext(1, FLASH_FLAG_OPERR, HostFlash::Target::Erase);
		} else {
			HostFlash::failNextAfter(failing < eraseStep ? failing : failing - 1, 1, FLASH_FLAG_PGPERR, HostFlash::Target::Program);
		}
		REQUIRE(eeWrite(NEW_VAR, NEW_VALUE) != EE_OK);

		powerCycle();

		CHECK(validPages() == 1);
		checkOldOrNew(old, NEW_VAR, NEW_VALUE);
	}
}

// ================================================================================================================
// Power loss
// ================================================================================================================

TEST_CASE_FIXTURE(EepromFixture, "a power loss at any step of a write keeps the old or the new value of the variable") {
	Values old;
	forEveryPowerCut(
		[&] {
			writePage(0, VALID_PAGE, {{ADR_CURRENT_CONFIG, 1}, {ADR_CANCONF1, 2}});
			old = newest(pageRecords(0));
		},
		[] { eeWrite(ADR_CURRENT_CONFIG, 0x00F0); },
		[&] {
			CHECK(validPages() == 1);
			checkOldOrNew(old, ADR_CURRENT_CONFIG, 0x00F0);
		},
		{0, 0x0F0F, 0xFFFF}); // nothing, some or all bits of the interrupted half-word programmed
}

TEST_CASE_FIXTURE(EepromFixture, "a power loss at any step of a page transfer from page 0 to page 1 keeps every variable at its old or new value") {
	Values old;
	forEveryPowerCut(
		[&] { old = fullValidPage(0); },
		[] { eeWrite(NEW_VAR, NEW_VALUE); },
		[&] {
			CHECK(validPages() == 1);
			checkOldOrNew(old, NEW_VAR, NEW_VALUE);
			// The store keeps working: one more write survives the next power cycle
			CHECK(eeWrite(LATER_VAR, LATER_VALUE) == EE_OK);
			powerCycle();
			CHECK(validPages() == 1);
			CHECK(has(LATER_VAR, LATER_VALUE));
			checkOldOrNew(old, NEW_VAR, NEW_VALUE);
		});
}

TEST_CASE_FIXTURE(EepromFixture, "a power loss at any step of a page transfer from page 1 to page 0 keeps every variable at its old or new value" *
								 KNOWN_ISSUE("V5: the EE_Init recovery leaves page 1 VALID next to the new valid page 0 (eeprom.c:187), later writes are lost")) {
	Values old;
	forEveryPowerCut(
		[&] {
			writePage(0, ERASED);
			old = fullValidPage(1);
		},
		[] { eeWrite(NEW_VAR, NEW_VALUE); },
		[&] {
			REQUIRE_FLASH_ACCESSES_MODELLED();
			CHECK(validPages() == 1);
			checkOldOrNew(old, NEW_VAR, NEW_VALUE);
			CHECK(eeWrite(LATER_VAR, LATER_VALUE) == EE_OK);
			powerCycle();
			CHECK(has(LATER_VAR, LATER_VALUE));
			checkOldOrNew(old, NEW_VAR, NEW_VALUE);
		});
}

namespace {
/** Memory images (both pages) an interrupted page transfer leaves behind, and the values before the write */
struct TransferStates {
	Values old;
	std::vector<std::vector<uint8_t>> states;
};
} // namespace

/**
 * Collects the states of a transfer from `fromPage` (full and valid; the other page erased) interrupted at every program
 * step and with every partial-erase variant of the old-page erase.
 */
static TransferStates interruptedTransferStates(EepromFixture &f, int fromPage) {
	TransferStates result;
	auto setup = [&] {
		f.writePage(1 - fromPage, ERASED);
		return f.fullValidPage(fromPage);
	};
	f.resetFlashKeepingChecks();
	result.old = setup();
	REQUIRE(f.eeWrite(NEW_VAR, NEW_VALUE) == EE_OK);
	const uint32_t transferPrograms = HostFlash::state().programsApplied;
	auto capture = [&] {
		REQUIRE(HostFlash::powerLost());
		result.states.push_back(HostFlash::snapshot(f.pageBase(0), 2 * HostFlash::EEPROM_PAGE_SIZE));
	};
	for (uint32_t completed = 0; completed < transferPrograms; completed++) {
		f.resetFlashKeepingChecks();
		setup();
		HostFlash::cutPowerAfterPrograms(completed);
		f.eeWrite(NEW_VAR, NEW_VALUE);
		capture();
	}
	for (EepromFixture::EraseCut cut : EepromFixture::eraseCuts()) {
		f.resetFlashKeepingChecks();
		setup();
		HostFlash::cutPowerDuringErase(0, cut.offset, cut.length);
		f.eeWrite(NEW_VAR, NEW_VALUE);
		capture();
	}
	return result;
}

/** Power loss at every step of the EE_Init recovery of every interrupted transfer state, then power cycle and check */
static void checkRecoveryPowerLoss(EepromFixture &f, const TransferStates &t, const std::vector<EepromFixture::EraseCut> &cuts,
								   bool knownIssue) {
	for (size_t i = 0; i < t.states.size(); i++) {
		CAPTURE(i);
		f.forEveryPowerCut(
			[&] { HostFlash::restore(f.pageBase(0), t.states[i]); },
			[&] { f.eeInit(); },
			[&] {
				if (knownIssue) {
					REQUIRE_FLASH_ACCESSES_MODELLED();
				}
				CHECK(f.validPages() == 1);
				f.checkOldOrNew(t.old, NEW_VAR, NEW_VALUE);
			},
			{0}, cuts);
	}
}

TEST_CASE_FIXTURE(EepromFixture, "every step of a page transfer in both directions can be interrupted by the power-loss model") {
	// Precondition check (passing) for the KNOWN_ISSUE recovery loops below: every cut point of the reference transfer
	// is reached (interruptedTransferStates REQUIREs powerLost() for each)
	for (int from = 0; from < 2; from++) {
		CAPTURE(from);
		const TransferStates t = interruptedTransferStates(*this, from);
		CHECK(t.states.size() == 1 + 2 + 2 * VARS.size() + 1 + eraseCuts().size());
	}
}

TEST_CASE_FIXTURE(EepromFixture, "a power loss at any program step of the EE_Init recovery of an interrupted page transfer keeps every variable at its old or new value") {
	// Recovery of a transfer from page 0 to page 1 (EE_Init VALID / RECEIVE_DATA branch).
	// Erase cuts of the recovery are limited to those that erase the header of page 0: if the VALID header of page 0
	// survives the cut, both pages are VALID (suspected bug V6, KNOWN_ISSUE twin below with all erase cuts).
	// Torn masks are limited to 0: with partly programmed half-words the failures come from torn page headers
	// (NEEDS REVIEW 49 and 51), not from the recovery itself.
	const TransferStates t = interruptedTransferStates(*this, 0);
	checkRecoveryPowerLoss(*this, t, {{0, HostFlash::SECTOR_SIZE}, {0, PAGE_HEADER_SIZE}}, false);
}

TEST_CASE_FIXTURE(EepromFixture, "a power loss at any step of the EE_Init recovery of an interrupted page transfer to page 1 keeps every variable at its old or new value" *
								 KNOWN_ISSUE("V6: EE_Init marks page 1 VALID before erasing page 0 (eeprom.c:286, 296); a cut in that erase leaves two VALID pages")) {
	// Same as the passing test above with every erase-cut variant
	const TransferStates t = interruptedTransferStates(*this, 0);
	checkRecoveryPowerLoss(*this, t, eraseCuts(), true);
}

TEST_CASE_FIXTURE(EepromFixture, "a power loss at any step of the EE_Init recovery of an interrupted page transfer to page 0 keeps every variable at its old or new value" *
								 KNOWN_ISSUE("V5 + V6: EE_Init marks page 0 VALID (eeprom.c:177) and then does not erase page 1 (V5, eeprom.c:187); "
											 "with V5 fixed the order still leaves two VALID pages after a cut in that erase (V6)")) {
	// Recovery of a transfer from page 1 to page 0 (EE_Init RECEIVE_DATA / VALID branch), every cut point. Passes only
	// with both V5 and V6 fixed.
	const TransferStates t = interruptedTransferStates(*this, 1);
	checkRecoveryPowerLoss(*this, t, eraseCuts(), true);
}

TEST_CASE_FIXTURE(EepromFixture, "a power loss when the EE_Init recovery starts to erase page 0 keeps every variable at its old or new value" *
								 KNOWN_ISSUE("V6: EE_Init marks page 1 VALID before erasing page 0 (eeprom.c:286, 296); both pages VALID -> the next EE_Init formats")) {
	// State: a transfer from page 0 to page 1 interrupted right before the old page is erased (page 0 VALID and full,
	// page 1 RECEIVE_DATA with all variables). EE_Init copies again, marks page 1 VALID, then erases page 0. A power loss
	// before that erase changed anything leaves two VALID pages, and the next EE_Init formats (eeprom.c:233) -> every
	// variable is lost. EE_PageTransfer itself uses the safe order (erase the old page, then mark the new one VALID).
	// The REQUIRE preconditions below (cut points reached, page 1 receiving) are also checked by the passing tests
	// "every step of a page transfer ... can be interrupted" and the passing recovery loop.
	const Values old = fullValidPage(0);
	HostFlash::cutPowerDuringErase(0, 0, 0);
	eeWrite(NEW_VAR, NEW_VALUE);
	REQUIRE(HostFlash::powerLost());
	REQUIRE(pageStatus(1) == RECEIVE_DATA);
	HostFlash::powerOn();
	HostFlash::cutPowerDuringErase(0, 0, 0); // the recovery is interrupted at the same point
	eeInit();
	REQUIRE(HostFlash::powerLost());

	powerCycle();

	REQUIRE_FLASH_ACCESSES_MODELLED();
	checkOldOrNew(old, NEW_VAR, NEW_VALUE);
	CHECK(validPages() == 1);
}

TEST_CASE_FIXTURE(EepromFixture, "a power loss at any step of EE_Format leaves one valid page with every variable at its old value or erased") {
	Values old;
	forEveryPowerCut(
		[&] { old = fullValidPage(0); },
		[] { eeFormat(); },
		[&] {
			CHECK(validPages() == 1);
			// Either nothing was formatted or the emulation is empty; no mix of old and missing values
			const bool formatted = missing(VARS[0]);
			for (const auto &[address, value] : old) {
				CAPTURE(address);
				CHECK((formatted ? missing(address) : has(address, value)));
			}
		});
}

TEST_CASE_FIXTURE(EepromFixture, "characterization: a power loss while erasing the old page after a transfer can bring back an older value") {
	// NEEDS REVIEW 41 (AN3969 limitation): the old page is erased before the new page is marked VALID. If the power is
	// lost during that erase and the old page's VALID header survives while its newest records are already erased,
	// EE_Init (VALID / RECEIVE_DATA branch, eeprom.c:259) copies the old page again and appends the newest SURVIVING
	// record of every variable after the correctly transferred ones, so an older value wins. AN3969 has no
	// "erase in progress" state. Which bytes an interrupted erase leaves is indeterminate; this is one possible state.
	const Values old = fullValidPage(0);
	const uint32_t keptRecords = RECORDS_PER_PAGE / 2; // records 0..2046 survive, the newer half is erased
	const uint32_t erasedFrom = PAGE_HEADER_SIZE + keptRecords * RECORD_SIZE;
	HostFlash::cutPowerDuringErase(0, erasedFrom, HostFlash::SECTOR_SIZE - erasedFrom);
	eeWrite(NEW_VAR, NEW_VALUE);
	REQUIRE(HostFlash::powerLost());
	REQUIRE(pageStatus(0) == VALID_PAGE);
	REQUIRE(pageStatus(1) == RECEIVE_DATA);

	powerCycle();

	CHECK(pageStatus(1) == VALID_PAGE);
	CHECK(has(NEW_VAR, NEW_VALUE));
	// The newest surviving record of VARS[0] (records cycle through VARS, value = record index)
	const uint16_t stale = (uint16_t)(keptRecords - 1 - ((keptRecords - 1) % VARS.size()));
	REQUIRE(old.at(VARS[0]) != stale);
	CHECK(has(VARS[0], stale));
}

TEST_CASE_FIXTURE(EepromFixture, "characterization: a torn VALID header after the old page was erased makes EE_Init format the emulation") {
	// NEEDS REVIEW 51: the last program of a page transfer (VALID_PAGE over RECEIVE_DATA) interrupted after some bits:
	// page 1 status is neither RECEIVE_DATA nor VALID_PAGE and page 0 is erased, so EE_Init formats (eeprom.c:137) and
	// every variable is lost. AN3969 treats any unknown header combination as corrupted.
	fullValidPage(0);
	const uint32_t programsBeforeValid = 1 + 2 + 2 * (uint32_t)VARS.size();
	HostFlash::cutPowerAfterPrograms(programsBeforeValid, 0x00FF); // only the low byte of 0xEEEE -> 0x0000 cleared
	eeWrite(NEW_VAR, NEW_VALUE);
	REQUIRE(HostFlash::powerLost());
	REQUIRE(pageStatus(1) == 0xEE00);

	powerCycle();

	CHECK(pageStatus(0) == VALID_PAGE);
	for (uint16_t address : VARS) {
		CAPTURE(address);
		CHECK(missing(address));
	}
	CHECK(missing(NEW_VAR));
}

TEST_CASE_FIXTURE(EepromFixture, "characterization: a torn virtual address of an interrupted write can make the value appear under another variable") {
	// NEEDS REVIEW 52: records have no checksum (AN3969). Programming ADR_HW_VERSION (0x0001) over 0xFFFF clears bits
	// 1..15; if only some of them are cleared the half-word can equal another listed address, here
	// ADR_FFBWHEEL_BUTTONCONF (0x0101), which then reads the data of the interrupted write.
	REQUIRE(inVirtAddVarTab(ADR_FFBWHEEL_BUTTONCONF));
	writePage(0, VALID_PAGE, {{ADR_FFBWHEEL_BUTTONCONF, 0x0007}});
	HostFlash::cutPowerAfterPrograms(1, 0xFEFE); // data complete, address: bit 8 stays set
	eeWrite(ADR_HW_VERSION, 0x0BAD);
	REQUIRE(HostFlash::powerLost());

	powerCycle();

	CHECK(has(ADR_FFBWHEEL_BUTTONCONF, 0x0BAD));
	CHECK(missing(ADR_HW_VERSION));
}

} // TEST_SUITE
