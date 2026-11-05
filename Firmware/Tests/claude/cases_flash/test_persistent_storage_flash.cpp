/*
 * Tests of PersistentStorage.cpp on the real flash layer (flash_helpers.cpp + eeprom.c on the fake HAL FLASH):
 * handlers saving and restoring their settings through Flash_Write / Flash_Read, across reboots, page transfers
 * and power loss, and the delayed restore at start-up in the order of cppmain.cpp:
 *   global / early objects call restoreFlashDelayed() -> Flash_Init() (cppmain.cpp:46)
 *   -> PersistentStorage::restoreFlashStartupCb() (cppmain.cpp:92) -> main class created (restores directly).
 * The handler list and the start-up logic alone are tested with the map-backed flash in
 * claude/cases/test_persistent_storage.cpp.
 */
#include "doctest.h"
#include "FlashFixture.h"

#include <string>
#include <vector>

namespace {

/** A settings class like the firmware's: a few half-word settings, saved with Flash_Write, restored with defaults */
class Settings : public PersistentStorage {
public:
	Settings(std::vector<uint16_t> addresses, std::vector<uint16_t> defaults, std::vector<std::string> *log = nullptr,
			 std::string name = "")
		: addresses(std::move(addresses)), values(std::move(defaults)), log(log), name(std::move(name)) {}

	void saveFlash() override {
		for (size_t i = 0; i < addresses.size(); i++) {
			Flash_Write(addresses[i], values[i]);
		}
	}
	void restoreFlash() override {
		restoreCalls++;
		if (log) {
			log->push_back(name);
		}
		for (size_t i = 0; i < addresses.size(); i++) {
			values[i] = Flash_ReadDefault(addresses[i], values[i]);
		}
	}

	std::vector<uint16_t> addresses;
	std::vector<uint16_t> values;
	std::vector<std::string> *log;
	std::string name;
	int restoreCalls = 0;
};

/** Same loops as the "save" system command (SystemCommands.cpp) and a full restore */
void saveAll() {
	for (PersistentStorage *handler : PersistentStorage::getFlashHandlers()) {
		handler->saveFlash();
	}
}
void restoreAll() {
	for (PersistentStorage *handler : PersistentStorage::getFlashHandlers()) {
		handler->restoreFlash();
	}
}

const std::vector<uint16_t> wheelAddresses = {ADR_FFBWHEEL_CONF1, ADR_FFBWHEEL_BUTTONCONF, ADR_FFBWHEEL_ANALOGCONF};
const std::vector<uint16_t> effectAddresses = {ADR_FFB_EFFECTS1, ADR_FFB_EFFECTS2};

} // namespace

TEST_SUITE("PersistentStorageFlash") {

TEST_CASE_FIXTURE(FlashFixture, "values saved by all handlers are restored by new handlers after a reboot") {
	REQUIRE(init());
	{
		Settings wheel(wheelAddresses, {0x0101, 0x0202, 0x0303});
		Settings effects(effectAddresses, {0x1111, 0x2222});
		saveAll();
	}
	REQUIRE(reboot());

	Settings wheel(wheelAddresses, {0, 0, 0});
	Settings effects(effectAddresses, {0, 0});
	restoreAll();

	CHECK(wheel.values == std::vector<uint16_t>{0x0101, 0x0202, 0x0303});
	CHECK(effects.values == std::vector<uint16_t>{0x1111, 0x2222});
}

TEST_CASE_FIXTURE(FlashFixture, "a handler restored from an empty flash keeps its defaults and nothing is written") {
	REQUIRE(init());
	const auto before = HostFlash::state();
	Settings wheel(wheelAddresses, {1, 2, 3});

	restoreAll();

	CHECK(wheel.values == std::vector<uint16_t>{1, 2, 3});
	CHECK(HostFlash::state().programCalls == before.programCalls);
}

TEST_CASE_FIXTURE(FlashFixture, "saving unchanged settings again programs nothing") {
	REQUIRE(init());
	Settings wheel(wheelAddresses, {1, 2, 3});
	saveAll();
	const auto before = HostFlash::state();

	saveAll();

	CHECK(HostFlash::state().programCalls == before.programCalls);
	CHECK(HostFlash::state().unlocks == before.unlocks);
	CHECK(usedRecords(0) == 3);
}

TEST_CASE_FIXTURE(FlashFixture, "only the changed setting of a handler adds a record when saving") {
	REQUIRE(init());
	Settings wheel(wheelAddresses, {1, 2, 3});
	saveAll();

	wheel.values[1] = 20;
	saveAll();

	auto records = pageRecords(0);
	REQUIRE(records.size() == 4);
	CHECK(records[3].address == ADR_FFBWHEEL_BUTTONCONF);
	CHECK(records[3].value == 20);
}

TEST_CASE_FIXTURE(FlashFixture, "saved settings survive a page transfer caused by a later save and a reboot") {
	REQUIRE(init());
	Settings wheel(wheelAddresses, {0x0A, 0x0B, 0x0C});
	Settings effects(effectAddresses, {0, 0});
	saveAll();
	// Many saves of the effect settings since then: page 0 is full
	for (uint32_t i = usedRecords(0); i < RECORDS_PER_PAGE; i++) {
		writeRecord(0, i, Record{effectAddresses[i % 2], (uint16_t)i});
	}
	effects.values = {0xE1, 0xE2};

	saveAll(); // wheel unchanged, effects -> transfer to page 1

	REQUIRE(pageStatus(1) == VALID_PAGE);
	REQUIRE(pageStatus(0) == ERASED);
	REQUIRE(reboot());
	Settings wheel2(wheelAddresses, {0, 0, 0});
	Settings effects2(effectAddresses, {0, 0});
	wheel2.restoreFlash();
	effects2.restoreFlash();
	CHECK(wheel2.values == std::vector<uint16_t>{0x0A, 0x0B, 0x0C});
	CHECK(effects2.values == std::vector<uint16_t>{0xE1, 0xE2});
}

TEST_CASE_FIXTURE(FlashFixture, "a power loss at any program step of a save keeps every setting at its old or new value") {
	const std::vector<uint16_t> oldValues = {0x0001, 0x0002, 0x0003};
	const std::vector<uint16_t> newValues = {0x0100, 0x0200, 0x0300};

	// Reference run: number of program operations of the save
	REQUIRE(init());
	{
		Settings wheel(wheelAddresses, oldValues);
		saveAll();
		const uint32_t start = HostFlash::state().programsApplied;
		wheel.values = newValues;
		saveAll();
		REQUIRE(HostFlash::state().programsApplied - start == 2 * 3); // data and virtual address half-word each
	}

	for (uint32_t completed = 0; completed < 6; completed++) {
		for (uint64_t torn : {0x0000ULL, 0xFF00ULL}) { // nothing / only the high byte of the interrupted half-word
			CAPTURE(completed);
			CAPTURE(torn);
			resetIteration();
			REQUIRE(init());
			{
				Settings wheel(wheelAddresses, oldValues);
				saveAll();
				HostFlash::cutPowerAfterPrograms(completed, torn);
				wheel.values = newValues;
				saveAll(); // results meaningless after the cut
				REQUIRE(HostFlash::powerLost());
			}

			// The torn half-word is a deterministic model of an indeterminate cell state
			CHECK(reboot());
			Settings wheel(wheelAddresses, {0, 0, 0});
			wheel.restoreFlash();
			for (size_t i = 0; i < 3; i++) {
				CAPTURE(i);
				CHECK((wheel.values[i] == oldValues[i] || wheel.values[i] == newValues[i]));
			}
			// Settings completed before the cut have the new value
			for (size_t i = 0; i < completed / 2; i++) {
				CAPTURE(i);
				CHECK(wheel.values[i] == newValues[i]);
			}
		}
	}
}

TEST_CASE_FIXTURE(FlashFixture, "a delayed restore requested before the flash start-up reads the values of the last run after the startup callback") {
	// Last run stored the settings
	REQUIRE(init());
	{
		Settings wheel(wheelAddresses, {7, 8, 9});
		saveAll();
	}
	powerCycle(); // RAM cleared (startup flag), no Flash_Init yet

	// Next start: an object created before Flash_Init requests its restore
	Settings wheel(wheelAddresses, {0, 0, 0});
	wheel.restoreFlashDelayed();
	CHECK(wheel.restoreCalls == 0);

	REQUIRE(Flash_Init()); // cppmain.cpp:46
	CHECK(wheel.restoreCalls == 0); // Flash_Init does not restore

	PersistentStorage::restoreFlashStartupCb(); // cppmain.cpp:92

	CHECK(wheel.restoreCalls == 1);
	CHECK(wheel.values == std::vector<uint16_t>{7, 8, 9});
}

TEST_CASE_FIXTURE(FlashFixture, "a delayed restore reads the values that Flash_Init repaired after a power loss during a page transfer") {
	// Power loss after the new value and some variables were copied to page 1 (RECEIVE_DATA) but before page 1 was
	// marked valid: Flash_Init finishes the transfer
	writePage(0, VALID_PAGE, {{ADR_FFBWHEEL_CONF1, 1}, {ADR_FFBWHEEL_BUTTONCONF, 2}, {ADR_FFBWHEEL_ANALOGCONF, 3}});
	writePage(1, RECEIVE_DATA, {{ADR_FFBWHEEL_CONF1, 0x10}, {ADR_FFBWHEEL_BUTTONCONF, 2}});
	Settings wheel(wheelAddresses, {0, 0, 0});
	wheel.restoreFlashDelayed();

	REQUIRE(Flash_Init());
	PersistentStorage::restoreFlashStartupCb();

	CHECK(pageStatus(1) == VALID_PAGE);
	CHECK(wheel.values == std::vector<uint16_t>{0x10, 2, 3});
}

TEST_CASE_FIXTURE(FlashFixture, "the startup callback restores the waiting handlers in registration order, each once") {
	REQUIRE(init());
	std::vector<std::string> log;
	Settings first(wheelAddresses, {0, 0, 0}, &log, "first");
	Settings notWaiting(effectAddresses, {0, 0}, &log, "notWaiting");
	Settings second(effectAddresses, {0, 0}, &log, "second");
	second.restoreFlashDelayed();
	first.restoreFlashDelayed();

	PersistentStorage::restoreFlashStartupCb();
	PersistentStorage::restoreFlashStartupCb();

	CHECK(log == std::vector<std::string>{"first", "second"});
}

TEST_CASE_FIXTURE(FlashFixture, "a handler created after the startup callback restores immediately from flash") {
	REQUIRE(init());
	REQUIRE(Flash_Write(ADR_FFB_EFFECTS1, 0x4444));
	PersistentStorage::restoreFlashStartupCb();

	Settings effects(effectAddresses, {1, 2}); // like the main class created after cppmain.cpp:92
	effects.restoreFlashDelayed();

	CHECK(effects.restoreCalls == 1);
	CHECK(effects.values == std::vector<uint16_t>{0x4444, 2});
}

TEST_CASE_FIXTURE(FlashFixture, "settings saved after Flash_Format are stored and restored after a reboot") {
	// cppmain.cpp:51/73 formats the flash on start-up errors or a major version change
	REQUIRE(init());
	Settings wheel(wheelAddresses, {1, 2, 3});
	saveAll();
	REQUIRE(Flash_Format());

	wheel.values = {4, 5, 6};
	saveAll();
	REQUIRE(reboot());

	Settings restored(wheelAddresses, {0, 0, 0});
	restored.restoreFlash();
	CHECK(restored.values == std::vector<uint16_t>{4, 5, 6});
}

} // TEST_SUITE
