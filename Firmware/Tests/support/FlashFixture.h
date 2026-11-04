/*
 * FlashFixture.h
 *
 * Fixture of ffb_flash_tests (real flash_helpers.cpp, eeprom.c, eeprom_addresses.c and PersistentStorage.cpp on
 * the fake HAL FLASH, see HostFlash.h). Use with TEST_CASE_FIXTURE(FlashFixture, "...").
 *
 * Resets before and after every test: the fake flash (all modelled memory erased, registers after HAL_Init,
 * injections and recording cleared), the simulated time and RTOS (HostPlatform, HostRtos), the global state of
 * the firmware under test (PersistentStorage handler list and startup flag, eeprom.c's DataVar).
 * The flash is left ERASED: call init() to run the firmware start-up (Flash_Init -> EE_Init formats it).
 *
 * At the end of every test the fixture CHECKs that the firmware only touched modelled flash
 * (HostFlash::state().unexpected). In a KNOWN_ISSUE (should_fail) test that check would count as the expected
 * failure, so such tests call REQUIRE_FLASH_ACCESSES_MODELLED() right before the assertion describing the defect.
 *
 * EEPROM emulation page layout (eeprom.c, ST AN3969), 2 pages of 16 KiB (sector 1 = page 0, sector 2 = page 1):
 *   offset 0: page status   ERASED 0xFFFF, RECEIVE_DATA 0xEEEE, VALID_PAGE 0x0000
 *   offset 2: unused (stays 0xFFFF)
 *   offset 4 + 4*i: record i: data half-word, then the virtual address half-word (offset 6 + 4*i)
 * A record is free while both half-words are 0xFFFF. The newest record of an address is the one with the highest
 * offset. EE_Init repairs header combinations after a power loss (see the switch in EE_Init).
 */
#ifndef HOSTTEST_FLASHFIXTURE_H_
#define HOSTTEST_FLASHFIXTURE_H_

#include "HostFlash.h"
#include "HostPlatform.h"
#include "HostRtos.h"

#include "PersistentStorage.h"
#include "eeprom_addresses.h"
#include "flash_helpers.h"

#include "doctest.h"

#include <cstdint>
#include <string>
#include <vector>

extern "C" uint16_t DataVar; // eeprom.c: global buffer of the variable transfer

/**
 * Fails the test (REQUIRE) if the firmware reached flash the fake does not model. Use it in KNOWN_ISSUE tests before
 * the assertion that describes the defect (see the file comment).
 */
#define REQUIRE_FLASH_ACCESSES_MODELLED()                                                                                \
	do {                                                                                                                  \
		INFO("first unmodelled flash access: "                                                                            \
			 << (HostFlash::state().unexpected.empty() ? std::string() : HostFlash::state().unexpected.front()));          \
		REQUIRE(HostFlash::state().unexpected.empty());                                                                   \
	} while (0)

/** Access to the protected PersistentStorage::startupComplete (set by restoreFlashStartupCb, no public reset) */
struct FlashPersistentStorageStartup : PersistentStorage {
	static void reset() { startupComplete = false; }
	static bool complete() { return startupComplete; }
};

struct FlashFixture {
	/** One variable record of an EEPROM emulation page */
	struct Record {
		uint16_t address;
		uint16_t value;
	};

	static constexpr uint32_t PAGE_HEADER_SIZE = 4; //!< Status half-word + unused half-word
	static constexpr uint32_t RECORD_SIZE = 4;
	static constexpr uint32_t RECORDS_PER_PAGE = (HostFlash::EEPROM_PAGE_SIZE - PAGE_HEADER_SIZE) / RECORD_SIZE; //!< 4095

	FlashFixture() { resetAll(); }
	virtual ~FlashFixture() {
		const auto &unexpected = HostFlash::state().unexpected;
		INFO("first unmodelled flash access: " << (unexpected.empty() ? std::string() : unexpected.front()));
		CHECK_MESSAGE(unexpected.empty(), "the firmware reached flash that the fake does not model (see HostFlash.h)");
		resetAll();
	}

	static void resetAll() {
		HostPlatform::reset();
		HostRtos::reset();
		HostFlash::reset();
		PersistentStorage::getFlashHandlers().clear();
		FlashPersistentStorageStartup::reset();
		DataVar = 0;
	}

	// ---------------- Firmware start-up ----------------

	/** Firmware start-up of the flash (cppmain.cpp calls Flash_Init before the main class is created) */
	static bool init() { return Flash_Init(); }

	/**
	 * Power cycle without the firmware start-up: the hardware comes back (HostFlash::powerOn: power-loss state and
	 * injections cleared, flash locked, flags cleared) and the memory keeps its content. The RAM state of the flash
	 * layer is cleared like after a real reset: eeprom.c's DataVar and PersistentStorage::startupComplete. The
	 * PersistentStorage handler list is kept: it holds handler objects the test owns (a test models the objects of
	 * the next run by destroying the old handlers and creating new ones).
	 */
	static void powerCycle() {
		HostFlash::powerOn();
		DataVar = 0;
		FlashPersistentStorageStartup::reset();
	}

	/** Power cycle (powerCycle()) followed by the firmware start-up (Flash_Init). Returns the result of Flash_Init */
	static bool reboot() {
		powerCycle();
		return Flash_Init();
	}

	/**
	 * Reset between the iterations of a loop inside one test case (e.g. one iteration per power-cut point). Checks
	 * first that the iteration did not reach unmodelled flash (resetAll() clears HostFlash::state().unexpected, so the
	 * destructor check would miss it), then resets everything like a new test (resetAll()).
	 */
	static void resetIteration() {
		const auto &unexpected = HostFlash::state().unexpected;
		INFO("first unmodelled flash access: " << (unexpected.empty() ? std::string() : unexpected.front()));
		CHECK_MESSAGE(unexpected.empty(), "the firmware reached flash that the fake does not model (see HostFlash.h)");
		resetAll();
	}

	// ---------------- EEPROM emulation pages ----------------

	static uint32_t pageBase(int page) { return page == 0 ? HostFlash::EEPROM_PAGE0 : HostFlash::EEPROM_PAGE1; }
	static uint16_t pageStatus(int page) { return HostFlash::read16(pageBase(page)); }
	static uint32_t recordAddress(int page, uint32_t index) { return pageBase(page) + PAGE_HEADER_SIZE + index * RECORD_SIZE; }

	/** Raw: erases the page (sector) to 0xFF and writes `status` to its header. Not recorded, no NOR semantics */
	static void erasePage(int page, uint16_t status = 0xFFFF) {
		HostFlash::fill(pageBase(page), HostFlash::EEPROM_PAGE_SIZE, 0xFF);
		HostFlash::write16(pageBase(page), status);
	}

	/**
	 * Raw: builds a page image. Erases the page, writes `status` (ERASED, RECEIVE_DATA, VALID_PAGE or any other value
	 * for corrupted headers) and the records in order starting at offset 4.
	 */
	static void writePage(int page, uint16_t status, const std::vector<Record> &records = {}) {
		REQUIRE(records.size() <= RECORDS_PER_PAGE);
		erasePage(page, status);
		for (uint32_t i = 0; i < records.size(); i++) {
			writeRecord(page, i, records[i]);
		}
	}

	/** Raw: writes one record (data at the record address, virtual address 2 bytes later) */
	static void writeRecord(int page, uint32_t index, Record record) {
		HostFlash::write16(recordAddress(page, index), record.value);
		HostFlash::write16(recordAddress(page, index) + 2, record.address);
	}

	/** Raw: fills a page with `count` records cycling through the given addresses (value = record index) */
	static void fillPage(int page, uint16_t status, uint32_t count, const std::vector<uint16_t> &addresses) {
		std::vector<Record> records;
		for (uint32_t i = 0; i < count; i++) {
			records.push_back(Record{addresses[i % addresses.size()], (uint16_t)i});
		}
		writePage(page, status, records);
	}

	/** The records of a page in write order, up to the first free slot (both half-words 0xFFFF) */
	static std::vector<Record> pageRecords(int page) {
		std::vector<Record> records;
		for (uint32_t i = 0; i < RECORDS_PER_PAGE; i++) {
			uint32_t a = recordAddress(page, i);
			if (HostFlash::read32(a) == 0xFFFFFFFFU) {
				break;
			}
			records.push_back(Record{HostFlash::read16(a + 2), HostFlash::read16(a)});
		}
		return records;
	}

	/** Number of used record slots of a page */
	static uint32_t usedRecords(int page) { return (uint32_t)pageRecords(page).size(); }
};

#endif
