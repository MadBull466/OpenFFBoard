/*
 * Persistent storage (PersistentStorage.cpp): registry of flash handlers, save/restore
 * dispatch, delayed restore until the flash is initialized (restoreFlashStartupCb).
 *
 * The flash is the map-backed fake Flash_* API (HostPlatform::flash()).
 * The "save" command loop over all handlers lives in SystemCommands.cpp (not compiled
 * in ffb_unit_tests); the tests iterate getFlashHandlers() the same way it does.
 *
 * PersistentStorage::startupComplete is a global flag without public reset. The fixture
 * resets it to false (its value at power on) through a derived class before and after each test.
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "FirmwareFixture.h"

#include "PersistentStorage.h"
#include "flash_helpers.h"

#include <algorithm>
#include <vector>

namespace {

/** Stores one value at its own address and counts the calls */
class TestStorage : public PersistentStorage {
public:
	explicit TestStorage(uint16_t address) : address(address) {}

	void saveFlash() override {
		saveCalls++;
		Flash_Write(address, value);
	}
	void restoreFlash() override {
		restoreCalls++;
		uint16_t v;
		if (Flash_Read(address, &v)) {
			value = v;
		}
	}
	bool restorePending() const { return restoreDelayedFlag; }

	uint16_t address;
	uint16_t value = 0;
	int saveCalls = 0;
	int restoreCalls = 0;
};

/** Access to the protected global startup flag */
struct StartupFlag : PersistentStorage {
	static void set(bool v) { startupComplete = v; }
	static bool get() { return startupComplete; }
};

struct StorageFixture : FirmwareFixture {
	StorageFixture() { StartupFlag::set(false); }
	~StorageFixture() override { StartupFlag::set(false); }

	static bool isRegistered(PersistentStorage *s) {
		auto &v = PersistentStorage::getFlashHandlers();
		return std::find(v.begin(), v.end(), s) != v.end();
	}
	/** Same loop as the "save" system command (SystemCommands.cpp) */
	static void saveAll() {
		for (PersistentStorage *handler : PersistentStorage::getFlashHandlers()) {
			handler->saveFlash();
		}
	}
	static void restoreAll() {
		for (PersistentStorage *handler : PersistentStorage::getFlashHandlers()) {
			handler->restoreFlash();
		}
	}
};

} // namespace

TEST_SUITE("PersistentStorage") {

TEST_CASE_FIXTURE(StorageFixture, "a storage handler registers itself on construction and unregisters on destruction") {
	const size_t before = PersistentStorage::getFlashHandlers().size();
	auto *s = new TestStorage(0x100);
	CHECK(isRegistered(s));
	CHECK(PersistentStorage::getFlashHandlers().size() == before + 1);
	delete s;
	CHECK_FALSE(isRegistered(s));
	CHECK(PersistentStorage::getFlashHandlers().size() == before);
}

TEST_CASE_FIXTURE(StorageFixture, "destroying one handler keeps the others registered") {
	TestStorage a(0x100);
	TestStorage c(0x102);
	{
		TestStorage b(0x101);
		CHECK(isRegistered(&b));
	}
	CHECK(isRegistered(&a));
	CHECK(isRegistered(&c));
}

TEST_CASE_FIXTURE(StorageFixture, "saving over all registered handlers writes the values of every handler") {
	TestStorage a(0x100), b(0x101);
	a.value = 1234;
	b.value = 4321;
	saveAll();
	CHECK(a.saveCalls == 1);
	CHECK(b.saveCalls == 1);
	CHECK(HostPlatform::flash()[0x100] == 1234);
	CHECK(HostPlatform::flash()[0x101] == 4321);
}

TEST_CASE_FIXTURE(StorageFixture, "restoring over all registered handlers reads the stored values") {
	TestStorage a(0x100), b(0x101);
	HostPlatform::flash()[0x100] = 77;
	HostPlatform::flash()[0x101] = 88;
	restoreAll();
	CHECK(a.restoreCalls == 1);
	CHECK(b.restoreCalls == 1);
	CHECK(a.value == 77);
	CHECK(b.value == 88);
}

TEST_CASE_FIXTURE(StorageFixture, "a destroyed handler is not saved") {
	TestStorage kept(0x100);
	{
		TestStorage gone(0x101);
		gone.value = 5;
	}
	saveAll();
	CHECK(kept.saveCalls == 1);
	CHECK(HostPlatform::flash().count(0x101) == 0);
}

TEST_CASE_FIXTURE(StorageFixture, "the default save and restore of the base class do not touch the flash") {
	auto *plain = new PersistentStorage();
	CHECK(isRegistered(plain));
	HostPlatform::flash()[0x10] = 3;
	plain->saveFlash();
	plain->restoreFlash();
	CHECK(HostPlatform::flash().size() == 1);
	CHECK(HostPlatform::flash()[0x10] == 3);
	delete plain;
	CHECK_FALSE(isRegistered(plain));
}

TEST_CASE_FIXTURE(StorageFixture, "a delayed restore before startup does not restore yet") {
	TestStorage s(0x100);
	HostPlatform::flash()[0x100] = 42;
	s.restoreFlashDelayed();
	CHECK(s.restoreCalls == 0);
	CHECK(s.value == 0);
	CHECK(s.restorePending());
}

TEST_CASE_FIXTURE(StorageFixture, "a delayed restore does not depend on time, only on the startup callback") {
	TestStorage s(0x100);
	s.restoreFlashDelayed();
	HostPlatform::advanceMs(60000);
	HostRtos::runUntilIdle();
	CHECK(s.restoreCalls == 0);
	PersistentStorage::restoreFlashStartupCb();
	CHECK(s.restoreCalls == 1);
}

TEST_CASE_FIXTURE(StorageFixture, "the startup callback restores the handlers waiting for a delayed restore") {
	TestStorage waiting(0x100);
	TestStorage notWaiting(0x101);
	HostPlatform::flash()[0x100] = 42;
	HostPlatform::flash()[0x101] = 43;
	waiting.restoreFlashDelayed();
	PersistentStorage::restoreFlashStartupCb();
	CHECK(waiting.restoreCalls == 1);
	CHECK(waiting.value == 42);
	CHECK_FALSE(waiting.restorePending());
	CHECK(notWaiting.restoreCalls == 0);
	CHECK(notWaiting.value == 0);
}

TEST_CASE_FIXTURE(StorageFixture, "the startup callback marks the startup as complete") {
	CHECK_FALSE(StartupFlag::get());
	PersistentStorage::restoreFlashStartupCb();
	CHECK(StartupFlag::get());
}

TEST_CASE_FIXTURE(StorageFixture, "a delayed restore is done only once") {
	TestStorage s(0x100);
	s.restoreFlashDelayed();
	s.restoreFlashDelayed(); // Requested twice before startup
	PersistentStorage::restoreFlashStartupCb();
	PersistentStorage::restoreFlashStartupCb(); // A second callback restores nothing
	CHECK(s.restoreCalls == 1);
}

TEST_CASE_FIXTURE(StorageFixture, "a delayed restore after startup restores immediately") {
	PersistentStorage::restoreFlashStartupCb();
	TestStorage s(0x100);
	HostPlatform::flash()[0x100] = 99;
	s.restoreFlashDelayed();
	CHECK(s.restoreCalls == 1);
	CHECK(s.value == 99);
	CHECK_FALSE(s.restorePending());
}

TEST_CASE_FIXTURE(StorageFixture, "a handler destroyed before startup is not restored by the startup callback") {
	TestStorage kept(0x100);
	kept.restoreFlashDelayed();
	{
		TestStorage gone(0x101);
		gone.restoreFlashDelayed();
	}
	PersistentStorage::restoreFlashStartupCb();
	CHECK(kept.restoreCalls == 1);
}

TEST_CASE_FIXTURE(StorageFixture, "values saved by one handler are restored by a new handler after a restart") {
	{
		TestStorage before(0x200);
		before.value = 0xBEEF;
		saveAll();
	}
	TestStorage after(0x200);
	after.restoreFlashDelayed();
	CHECK(after.value == 0);
	PersistentStorage::restoreFlashStartupCb();
	CHECK(after.value == 0xBEEF);
}

} // TEST_SUITE
