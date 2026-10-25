/*
 * Tests of the start-up code cppmain.cpp (cppmain(), micros(), refreshWatchdog(), getRunTimeCounterValue(), its
 * malloc/free), of USBdevice.cpp (USB task, registration, descriptors and string descriptors) and of AdcHandler.cpp
 * (handler registry, resolution).
 *
 * cppmain() runs through HostTarget::runCppmain(loops) on the map flash of host_platform.cpp (Flash_Init always
 * succeeds there, so cppmain.cpp:47-49 is not reachable). Start-up paths that would create the default FFBWheel are
 * avoided (BUTTON_A pressed, or a stored non-FFBHIDMain class) so the tests also run in sanitizer builds; the empty
 * flash path with FFBWheel is the characterization in test_system_setup.cpp (NEEDS REVIEW 53).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "AdcHandler.h"
#include "ClassIDs.h"
#include "PersistentStorage.h"
#include "USBdevice.h"
#include "constants.h"
#include "cppmain.h"
#include "eeprom_addresses.h"
#include "global_callbacks.h"
#include "ledEffects.h"
#include "usb_descriptors.h"

#include <cstring>
#include <string>
#include <vector>

extern bool mainclassChosen;	   // cppmain.cpp
extern volatile bool forceErase;   // cppmain.cpp
extern IWDG_HandleTypeDef hiwdg;   // main.c (host_target_system.cpp)
extern uint16_t _desc_str[];	   // USBdevice.cpp: string descriptor buffer

namespace {

/** Access to the protected PersistentStorage::startupComplete (set by restoreFlashStartupCb) */
struct StartupFlag : PersistentStorage {
	static bool complete() { return startupComplete; }
};

struct CppmainFixture : TargetFixture {
	/** Flash as written by an earlier start with the current flash layout */
	static void storeFlashVersion(uint16_t version = FLASH_VERSION) { HostPlatform::flash()[ADR_FLASH_VERSION] = version; }
	static void pressButtonA() { HostTarget::setPin(BUTTON_A_GPIO_Port, BUTTON_A_Pin, true); }
};

/** String of a USB string descriptor (UTF-16, only the low bytes) and its header word */
std::string descString(const uint16_t *desc) {
	REQUIRE(desc != nullptr);
	uint8_t length = desc[0] & 0xFF;
	std::string s;
	for (uint8_t i = 1; i < length / 2; i++) {
		s += static_cast<char>(desc[i]);
	}
	return s;
}

const tusb_desc_device_t &compositeDesc() { return usb_devdesc_ffboard_composite; }

struct UsbFixture : TargetFixture {
	/** String table with a short interface list for the descriptor tests */
	usb_string_desc_t strings{0x0409, "Maker", "Product", {"CDC", "HID"}};
};

struct RecordingAdcHandler : AdcHandler {
	void adcUpd(volatile uint32_t *, uint8_t chans, ADC_HandleTypeDef *hadc) override {
		updates.push_back(hadc);
		lastChans = chans;
	}
	std::vector<ADC_HandleTypeDef *> updates;
	uint8_t lastChans = 0;
};

size_t registeredCount(AdcHandler *h) {
	size_t n = 0;
	for (AdcHandler *a : AdcHandler::adcHandlers) {
		n += (a == h);
	}
	return n;
}

} // namespace

// =====================================================================================================================
TEST_SUITE("cppmain") {

TEST_CASE_FIXTURE(CppmainFixture, "with a stored main class id cppmain creates that main class without calling Error_Handler") {
	storeFlashVersion();
	HostPlatform::flash()[ADR_CURRENT_CONFIG] = CLSID_MAIN_CAN; // CAN bridge: not an FFBHIDMain

	HostTarget::runCppmain(0);

	REQUIRE(mainclass != nullptr);
	CHECK(mainclass->getInfo().id == CLSID_MAIN_CAN);
	CHECK(HostTarget::state().errorHandlerCalls == 0);
}

TEST_CASE_FIXTURE(CppmainFixture, "a stored main class id that is not in the registry starts the failsafe main class and stays stored") {
	storeFlashVersion();
	HostPlatform::flash()[ADR_CURRENT_CONFIG] = 0x77;

	HostTarget::runCppmain(0);

	REQUIRE(mainclass != nullptr);
	CHECK(mainclass->getInfo().id == 0);
	CHECK(HostPlatform::flash().at(ADR_CURRENT_CONFIG) == 0x77);
	CHECK(HostTarget::state().errorHandlerCalls == 0);
}

TEST_CASE_FIXTURE(CppmainFixture, "BUTTON_A pressed at start-up starts the failsafe main class and keeps the stored main class id") {
	storeFlashVersion();
	HostPlatform::flash()[ADR_CURRENT_CONFIG] = CLSID_MAIN_CAN;
	pressButtonA();

	HostTarget::runCppmain(0);

	REQUIRE(mainclass != nullptr);
	CHECK(mainclass->getInfo().id == 0);
	CHECK(HostPlatform::flash().at(ADR_CURRENT_CONFIG) == CLSID_MAIN_CAN);
}

TEST_CASE_FIXTURE(CppmainFixture, "on an empty flash cppmain writes the flash version, and with BUTTON_A pressed no main class id") {
	pressButtonA();

	HostTarget::runCppmain(0);

	CHECK(HostPlatform::flash().at(ADR_FLASH_VERSION) == FLASH_VERSION);
	CHECK(HostPlatform::flash().count(ADR_CURRENT_CONFIG) == 0);
	CHECK(HostTarget::state().errorHandlerCalls == 0);
}

TEST_CASE_FIXTURE(CppmainFixture, "a flash with the current flash version keeps its content") {
	storeFlashVersion();
	HostPlatform::flash()[ADR_AXIS1_POWER] = 1234;
	pressButtonA();

	HostTarget::runCppmain(0);

	CHECK(HostPlatform::flash().at(ADR_AXIS1_POWER) == 1234);
	CHECK(HostPlatform::flash().at(ADR_FLASH_VERSION) == FLASH_VERSION);
}

TEST_CASE_FIXTURE(CppmainFixture, "a flash with another flash version is formatted and gets the current version") {
	storeFlashVersion(FLASH_VERSION + 1);
	HostPlatform::flash()[ADR_AXIS1_POWER] = 1234;
	HostPlatform::flash()[ADR_CURRENT_CONFIG] = CLSID_MAIN_CAN;
	pressButtonA();

	HostTarget::runCppmain(0);

	CHECK(HostPlatform::flash().count(ADR_AXIS1_POWER) == 0);
	CHECK(HostPlatform::flash().count(ADR_CURRENT_CONFIG) == 0);
	CHECK(HostPlatform::flash().at(ADR_FLASH_VERSION) == FLASH_VERSION);
}

TEST_CASE_FIXTURE(CppmainFixture, "forceErase formats the flash at start-up and the current flash version is written again") {
	storeFlashVersion();
	HostPlatform::flash()[ADR_AXIS1_POWER] = 1234;
	forceErase = true;
	pressButtonA();

	HostTarget::runCppmain(0);

	CHECK(HostPlatform::flash().count(ADR_AXIS1_POWER) == 0);
	CHECK(HostPlatform::flash().at(ADR_FLASH_VERSION) == FLASH_VERSION);
}

TEST_CASE_FIXTURE(CppmainFixture, "cppmain allows the delayed flash restore and marks the main class as chosen") {
	pressButtonA();
	REQUIRE_FALSE(StartupFlag::complete());
	REQUIRE_FALSE(mainclassChosen);

	HostTarget::runCppmain(0);

	CHECK(StartupFlag::complete());
	CHECK(mainclassChosen);
}

TEST_CASE_FIXTURE(CppmainFixture, "cppmain lets the main class initialise USB, which registers the USB device and starts its task") {
	pressButtonA();

	HostTarget::runCppmain(0);

	CHECK(usb_device != nullptr);
	CHECK(HostRtos::findTask("USB") != nullptr);
}

TEST_CASE_FIXTURE(CppmainFixture, "without main loop passes cppmain neither updates the main class nor refreshes the watchdog") {
	pressButtonA();

	HostTarget::runCppmain(0);

	CHECK(HostTarget::state().watchdogRefreshes == 0);
	CHECK(HostTarget::state().taskYields == 0);
	CHECK(HAL_GetTick() == 0); // FFBoardMain::update would wait 1 ms
}

TEST_CASE_FIXTURE(CppmainFixture, "every main loop pass updates the main class, refreshes the watchdog and yields") {
	pressButtonA();

	HostTarget::runCppmain(5);

	CHECK(HAL_GetTick() == 5); // FFBoardMain::update waits 1 ms per pass
	CHECK(HostTarget::state().watchdogRefreshes == 5);
	CHECK(HostTarget::state().taskYields == 5);
	CHECK(hiwdg.Instance->KR == IWDG_KEY_RELOAD);
}

TEST_CASE_FIXTURE(CppmainFixture, "the main loop updates the LEDs: a system LED pulse ends while cppmain runs") {
	pressButtonA();
	pulseSysLed();
	REQUIRE(HostTarget::outputPin(LED_SYS_GPIO_Port, LED_SYS_Pin));

	HostTarget::runCppmain(30); // 30 ms: the 25 ms pulse is over

	CHECK_FALSE(HostTarget::outputPin(LED_SYS_GPIO_Port, LED_SYS_Pin));
}

TEST_CASE_FIXTURE(CppmainFixture, "on a chip with another device id cppmain only calls Error_Handler and initialises nothing") {
	// cppmain.cpp:33-42 loops forever; the fake's Error_Handler limit ends the loop after 3 calls
	HostTarget::state().devId = 0x419; // STM32F42x/F43x
	HostTarget::state().errorHandlerLimit = 3;
	pressButtonA();

	CHECK_THROWS_AS(HostTarget::runCppmain(1), HostTarget::ErrorHandlerLimitReached);

	CHECK(HostTarget::state().errorHandlerCalls == 3);
	CHECK(HostPlatform::flash().empty());
	CHECK_FALSE(HostTarget::adcDmaRunning(&hadc1));
	CHECK(mainclass == nullptr);
	CHECK(HostTarget::state().watchdogRefreshes == 0);
}

TEST_CASE_FIXTURE(CppmainFixture, "refreshWatchdog reloads the independent watchdog") {
	refreshWatchdog();

	CHECK(HostTarget::state().watchdogRefreshes == 1);
	CHECK(hiwdg.Instance->KR == IWDG_KEY_RELOAD);
}

TEST_CASE_FIXTURE(CppmainFixture, "micros counts every microsecond between two HAL ticks") {
	HostPlatform::setTimeUs(7000);
	CHECK(micros() == 7000);
	HostPlatform::advanceUs(1);
	CHECK(micros() == 7001);
	HostPlatform::advanceUs(998);
	CHECK(micros() == 7999);
}

TEST_CASE_FIXTURE(CppmainFixture, "micros wraps around at 2^32 microseconds like a 32 bit counter") {
	HostPlatform::setTimeUs((1ull << 32) - 1);
	CHECK(micros() == 0xFFFFFFFFu);

	HostPlatform::advanceUs(1235);

	CHECK(micros() == 1234u);
}

TEST_CASE_FIXTURE(CppmainFixture, "the RTOS run time counter is the microsecond time") {
	HostPlatform::setTimeUs(123456789);
	CHECK(getRunTimeCounterValue() == 123456789ul);

	HostPlatform::setTimeUs((1ull << 32) + 42);
	CHECK(getRunTimeCounterValue() == 42ul);
}

TEST_CASE_FIXTURE(CppmainFixture, "malloc and free of cppmain allocate usable memory from the RTOS heap") {
	const HostRtos::HeapStats before = HostRtos::heapStats();

	auto *p = static_cast<uint8_t *>(hosttest_cppmain_malloc(64));

	REQUIRE(p != nullptr);
	CHECK(HostRtos::heapStats().mallocCalls == before.mallocCalls + 1);
	CHECK(HostRtos::heapStats().lastMallocSize == 64);
	CHECK(HostRtos::heapStats().lastMallocResult == p);
	std::memset(p, 0xA5, 64);
	CHECK(p[63] == 0xA5);

	hosttest_cppmain_free(p);

	CHECK(HostRtos::heapStats().freeCalls == before.freeCalls + 1);
	CHECK(HostRtos::heapStats().lastFreed == p);
}

} // TEST_SUITE cppmain

// =====================================================================================================================
TEST_SUITE("USBdevice") {

TEST_CASE_FIXTURE(UsbFixture, "registerUsb makes the device the global USB device and starts the USB task") {
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings);
	REQUIRE(HostRtos::findTask("USB") == nullptr);

	device.registerUsb();

	CHECK(usb_device == &device);
	CHECK(HostRtos::findTask("USB") == device.GetHandle());
}

TEST_CASE_FIXTURE(UsbFixture, "the USB task initialises TinyUSB and then processes the USB events") {
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings);
	device.registerUsb();

	HostRtos::runUntilBlocked(device);
	CHECK(HostTarget::state().usbInits == 1);
	CHECK(HostTarget::state().usbEventsProcessed == 0);

	HostTarget::queueUsbEvent(device.GetHandle());
	HostRtos::runUntilBlocked(device);

	CHECK(HostTarget::state().usbEventsProcessed == 1);
}

TEST_CASE_FIXTURE(UsbFixture, "the device descriptor and the configuration descriptor of every index are the ones given to the device") {
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings);

	CHECK(device.getUsbDeviceDesc() == reinterpret_cast<const uint8_t *>(&compositeDesc()));
	CHECK(device.getUsbConfigurationDesc(0) == usb_cdc_conf);
	CHECK(device.getUsbConfigurationDesc(1) == usb_cdc_conf);
}

TEST_CASE_FIXTURE(UsbFixture, "the USB serial number is the 96 bit unique id as 24 hex digits, highest word first") {
	HostTarget::state().uid[0] = 0x00410032;
	HostTarget::state().uid[1] = 0x34385111;
	HostTarget::state().uid[2] = 0x3038363A;
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings);

	CHECK(device.getUsbSerial() == "3038363a3438511100410032");
}

TEST_CASE_FIXTURE(UsbFixture, "string descriptor 0 is the language id") {
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings);

	const uint16_t *desc = device.getUsbStringDesc(0, 0);

	REQUIRE(desc != nullptr);
	CHECK(desc[0] == ((TUSB_DESC_STRING << 8) | 4));
	CHECK(desc[1] == 0x0409);
}

TEST_CASE_FIXTURE(UsbFixture, "the manufacturer, product and serial number strings are at the indices of the device descriptor") {
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings);

	CHECK(descString(device.getUsbStringDesc(compositeDesc().iManufacturer, 0x0409)) == "Maker");
	CHECK(descString(device.getUsbStringDesc(compositeDesc().iProduct, 0x0409)) == "Product");
	CHECK(descString(device.getUsbStringDesc(compositeDesc().iSerialNumber, 0x0409)) == device.getUsbSerial());
}

TEST_CASE_FIXTURE(UsbFixture, "a string descriptor starts with its byte length and the string descriptor type and holds one UTF-16 unit per character") {
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings);

	const uint16_t *desc = device.getUsbStringDesc(compositeDesc().iProduct, 0x0409);

	REQUIRE(desc != nullptr);
	CHECK(desc[0] == ((TUSB_DESC_STRING << 8) | (2 * 7 + 2)));
	CHECK(desc[1] == u'P');
	CHECK(desc[7] == u't');
}

TEST_CASE_FIXTURE(UsbFixture, "interface strings start at index 4 without serial suffix when appendSerial is 0") {
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings, 0);

	CHECK(descString(device.getUsbStringDesc(4, 0x0409)) == "CDC");
	CHECK(descString(device.getUsbStringDesc(5, 0x0409)) == "HID");
}

TEST_CASE_FIXTURE(UsbFixture, "with appendSerial 4 an interface string is followed by 4 characters in brackets") {
	// Which 4 characters is bug 47 (USBdevice.cpp:81, KNOWN_ISSUE in test_usb_descriptors.cpp); this holds either way
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings); // appendSerial default 4

	const std::string s = descString(device.getUsbStringDesc(4, 0x0409));

	CHECK(s.size() == 9); // "CDC" + "(" + 4 + ")"
	CHECK(s.rfind("CDC(", 0) == 0);
	CHECK(s.back() == ')');
}

TEST_CASE_FIXTURE(UsbFixture, "a string index beyond the interface strings has no string descriptor") {
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings);

	CHECK(device.getUsbStringDesc(6, 0x0409) == nullptr);
	CHECK(device.getUsbStringDesc(0xEE, 0x0409) == nullptr);
}

TEST_CASE_FIXTURE(UsbFixture, "a string index below 4 that the device descriptor does not use has no string descriptor") {
	tusb_desc_device_t desc = compositeDesc();
	desc.iSerialNumber = 0; // no serial number string
	USBdevice device(&desc, usb_cdc_conf, &strings);

	CHECK(device.getUsbStringDesc(3, 0x0409) == nullptr);
}

TEST_CASE_FIXTURE(UsbFixture, "a string longer than the descriptor buffer is cut to 31 characters") {
	usb_string_desc_t longStrings{0x0409, "Maker", std::string(40, 'x') + "END", {}};
	USBdevice device(&compositeDesc(), usb_cdc_conf, &longStrings);

	const uint16_t *desc = device.getUsbStringDesc(compositeDesc().iProduct, 0x0409);

	REQUIRE(desc != nullptr);
	CHECK(desc[0] == ((TUSB_DESC_STRING << 8) | (2 * 31 + 2)));
	CHECK(descString(desc) == std::string(31, 'x'));
}

TEST_CASE_FIXTURE(UsbFixture, "a string of 256 characters is cut to 31 characters" *
	KNOWN_ISSUE("USBdevice.cpp:87 std::min<uint8_t>(USB_STRING_DESC_BUF_SIZE-1, field.length()) truncates the length to "
				"8 bit before the comparison: a 256 character string becomes 0 characters (latent: no current string "
				"has 256 characters or more)")) {
	usb_string_desc_t longStrings{0x0409, "Maker", std::string(256, 'x'), {}};
	USBdevice device(&compositeDesc(), usb_cdc_conf, &longStrings);

	const uint16_t *desc = device.getUsbStringDesc(compositeDesc().iProduct, 0x0409);

	REQUIRE(desc != nullptr);
	CHECK(desc[0] == ((TUSB_DESC_STRING << 8) | (2 * 31 + 2)));
}

TEST_CASE_FIXTURE(UsbFixture, "all string descriptors are returned in the same buffer") {
	USBdevice device(&compositeDesc(), usb_cdc_conf, &strings);

	CHECK(device.getUsbStringDesc(0, 0) == _desc_str);
	CHECK(device.getUsbStringDesc(compositeDesc().iProduct, 0) == _desc_str);
}

} // TEST_SUITE USBdevice

// =====================================================================================================================
TEST_SUITE("AdcHandler") {

TEST_CASE_FIXTURE(TargetFixture, "an ADC handler is registered once when it is created and removed when it is destroyed") {
	RecordingAdcHandler *handler = new RecordingAdcHandler();
	CHECK(registeredCount(handler) == 1);

	delete handler;

	CHECK(registeredCount(handler) == 0);
}

TEST_CASE_FIXTURE(TargetFixture, "registering an ADC handler again does not add it twice") {
	RecordingAdcHandler handler;

	addCallbackHandler<AdcHandler *>(AdcHandler::adcHandlers, &handler);

	CHECK(registeredCount(&handler) == 1);
}

TEST_CASE_FIXTURE(TargetFixture, "destroying one ADC handler keeps the other handlers registered") {
	RecordingAdcHandler first;
	auto *second = new RecordingAdcHandler();
	RecordingAdcHandler third;

	delete second;

	CHECK(registeredCount(&first) == 1);
	CHECK(registeredCount(&third) == 1);
	startADC();
	REQUIRE(HostTarget::adcConvert(&hadc2, {1, 2}));
	CHECK(first.updates == std::vector<ADC_HandleTypeDef *>{&hadc2});
	CHECK(third.updates == std::vector<ADC_HandleTypeDef *>{&hadc2});
	CHECK(third.lastChans == ADC2_CHANNELS);
}

TEST_CASE_FIXTURE(TargetFixture, "the default ADC update of a handler ignores the conversion") {
	AdcHandler handler;
	uint32_t buf[2] = {7, 8};

	handler.adcUpd(buf, 2, &hadc2);

	CHECK(buf[0] == 7);
	CHECK(buf[1] == 8);
}

TEST_CASE_FIXTURE(TargetFixture, "the ADC resolution in bits follows the resolution setting of the ADC") {
	ADC_HandleTypeDef hadc = {};
	const std::vector<std::pair<uint32_t, uint8_t>> cases = {
		{ADC_RESOLUTION_12B, 12}, {ADC_RESOLUTION_10B, 10}, {ADC_RESOLUTION_8B, 8}, {ADC_RESOLUTION_6B, 6}};
	for (const auto &c : cases) {
		CAPTURE(c.first);
		hadc.Init.Resolution = c.first;
		CHECK(AdcHandler::getAdcResolutionBits(&hadc) == c.second);
	}
}

TEST_CASE_FIXTURE(TargetFixture, "the configured F407 ADCs have 12 bit resolution") {
	CHECK(AdcHandler::getAdcResolutionBits(&hadc1) == 12);
	CHECK(AdcHandler::getAdcResolutionBits(&hadc2) == 12);
}

TEST_CASE_FIXTURE(TargetFixture, "an unknown ADC resolution setting gives 0 bits") {
	ADC_HandleTypeDef hadc = {};
	hadc.Init.Resolution = 0x04000000U;

	CHECK(AdcHandler::getAdcResolutionBits(&hadc) == 0);
}

} // TEST_SUITE AdcHandler
