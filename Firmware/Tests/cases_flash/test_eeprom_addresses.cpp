/*
 * Tests of eeprom_addresses.c / eeprom_addresses.h (generated from Firmware/scripts/memory_map.csv):
 * VirtAddVarTab, the list of virtual addresses the EEPROM emulation (eeprom.c) keeps in a page transfer and that
 * Flash_Dump(includeAll) exports, and exportableFlashAddresses, the default Flash_Dump list.
 *
 * Why it matters: EE_PageTransfer and the EE_Init repair copy only the variables listed in VirtAddVarTab. A
 * variable the firmware stores under an address missing from the table is silently lost at the next page transfer.
 * An address of 0xFFFF can not be stored (it marks an erased record).
 *
 * The consistency with the ADR_ constants is checked against the header text: the test reads
 * FFBoard/UserExtensions/Inc/eeprom_addresses.h relative to this source file (__FILE__) and parses its
 * "#define ADR_x value" lines. That is the only way to find a constant that was added to the header but not to the
 * table. If the header can not be found (sources moved), the test fails with the path it tried.
 */
#include "doctest.h"
#include "FlashFixture.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <string>
#include <vector>

namespace {

std::vector<uint16_t> table() { return std::vector<uint16_t>(VirtAddVarTab, VirtAddVarTab + NB_OF_VAR); }
std::vector<uint16_t> exportable() {
	return std::vector<uint16_t>(exportableFlashAddresses, exportableFlashAddresses + NB_EXPORTABLE_ADR);
}
bool contains(const std::vector<uint16_t> &list, uint16_t adr) { return std::find(list.begin(), list.end(), adr) != list.end(); }

std::filesystem::path headerPath() {
	// cases_flash/<this file> -> Tests -> Firmware
	return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "FFBoard" / "UserExtensions" / "Inc" /
		   "eeprom_addresses.h";
}

/** name -> value of every "#define ADR_x value" line of eeprom_addresses.h. Fails the test if the file is missing */
std::map<std::string, uint32_t> headerConstants() {
	std::map<std::string, uint32_t> constants;
	std::ifstream file(headerPath());
	INFO("header: " << headerPath().string());
	REQUIRE(file.is_open());
	const std::regex define(R"(^\s*#define\s+(ADR_\w+)\s+(0[xX][0-9a-fA-F]+|[0-9]+)\b)");
	std::string line;
	while (std::getline(file, line)) {
		std::smatch m;
		if (std::regex_search(line, m, define)) {
			INFO("line: " << line);
			CHECK(constants.count(m[1].str()) == 0); // every constant defined once
			constants[m[1].str()] = (uint32_t)std::stoul(m[2].str(), nullptr, 0);
		}
	}
	return constants;
}

/**
 * The flash addresses the in-scope firmware uses (grep of FFBoard/ outside the drivers TMC4671, ODrive, VESC,
 * RMD, MIDI): axes (Axis.h/Axis.cpp), main classes (FFBHIDMain, CanInputMain, cppmain, SystemCommands), effects
 * (EffectsCalculator), ports (CANPort2B, I2C), inputs and encoders. Compile-time references: a renamed or removed
 * constant breaks the build of this test.
 */
const std::vector<uint16_t> firmwareAddresses = {
	// System (cppmain.cpp, SystemCommands.cpp)
	ADR_HW_VERSION, ADR_SW_VERSION, ADR_FLASH_VERSION, ADR_CURRENT_CONFIG,
	// Ports
	ADR_CANCONF1, ADR_I2CCONF1,
	// Main classes
	ADR_FFBWHEEL_BUTTONCONF, ADR_FFBWHEEL_ANALOGCONF, ADR_FFBWHEEL_CONF1, ADR_CANREMOTE_CONF1, ADR_CANREMOTE_CONF2,
	// Effects calculator
	ADR_FFB_CF_FILTER, ADR_FFB_FR_FILTER, ADR_FFB_DA_FILTER, ADR_FFB_IN_FILTER, ADR_FFB_EFFECTS1, ADR_FFB_EFFECTS2,
	ADR_FFB_EFFECTS3, ADR_FFB_RECONSTRUCTION_FILTER,
	// Axis 1..3 (AxisFlashAddresses in Axis.h, Axis.cpp:120-145)
	ADR_AXIS1_CONFIG, ADR_AXIS1_MAX_SPEED, ADR_AXIS1_MAX_ACCEL, ADR_AXIS1_MAX_SLEWRATE_DRV, ADR_AXIS1_ENDSTOP,
	ADR_AXIS1_POWER, ADR_AXIS1_DEGREES, ADR_AXIS1_EFFECTS1, ADR_AXIS1_EFFECTS2, ADR_AXIS1_ENC_RATIO,
	ADR_AXIS1_SPEEDACCEL_FILTER, ADR_AXIS1_POSTPROCESS1, ADR_AXIS1_EQ1, ADR_AXIS1_EQ2, ADR_AXIS1_EQ3,
	ADR_AXIS1_HANDSOFF_CONF, ADR_AXIS1_HANDSOFF_ACCEL,
	ADR_AXIS2_CONFIG, ADR_AXIS2_MAX_SPEED, ADR_AXIS2_MAX_ACCEL, ADR_AXIS2_MAX_SLEWRATE_DRV, ADR_AXIS2_ENDSTOP,
	ADR_AXIS2_POWER, ADR_AXIS2_DEGREES, ADR_AXIS2_EFFECTS1, ADR_AXIS2_EFFECTS2, ADR_AXIS2_ENC_RATIO,
	ADR_AXIS2_SPEEDACCEL_FILTER, ADR_AXIS2_POSTPROCESS1, ADR_AXIS2_EQ1, ADR_AXIS2_EQ2, ADR_AXIS2_EQ3,
	ADR_AXIS2_HANDSOFF_CONF, ADR_AXIS2_HANDSOFF_ACCEL,
	ADR_AXIS3_CONFIG, ADR_AXIS3_MAX_SPEED, ADR_AXIS3_MAX_ACCEL, ADR_AXIS3_MAX_SLEWRATE_DRV, ADR_AXIS3_ENDSTOP,
	ADR_AXIS3_POWER, ADR_AXIS3_DEGREES, ADR_AXIS3_EFFECTS1, ADR_AXIS3_EFFECTS2, ADR_AXIS3_ENC_RATIO,
	ADR_AXIS3_SPEEDACCEL_FILTER, ADR_AXIS3_POSTPROCESS1, ADR_AXIS3_EQ1, ADR_AXIS3_EQ2, ADR_AXIS3_EQ3,
	ADR_AXIS3_HANDSOFF_CONF, ADR_AXIS3_HANDSOFF_ACCEL,
	// Buttons
	ADR_SPI_BTN_1_CONF, ADR_SPI_BTN_1_CONF_2, ADR_SPI_BTN_2_CONF, ADR_SPI_BTN_2_CONF_2, ADR_LOCAL_BTN_CONF,
	ADR_LOCAL_BTN_CONF_2, ADR_LOCAL_BTN_CONF_3, ADR_PCFBTN_CONF1, ADR_CANBTN_CONF1, ADR_CANBTN_CONF2,
	// Shifter
	ADR_SHIFTERANALOG_CONF, ADR_SHIFTERANALOG_CONF_2, ADR_SHIFTERANALOG_CONF_3, ADR_SHIFTERANALOG_X_12,
	ADR_SHIFTERANALOG_X_56, ADR_SHIFTERANALOG_Y_135, ADR_SHIFTERANALOG_Y_246,
	// Analog sources
	ADR_LOCALANALOG_MASK, ADR_CANANALOG_CONF1, ADR_ADS111X_CONF1,
	ADR_LOCALANALOG_MIN_0, ADR_LOCALANALOG_MAX_0, ADR_LOCALANALOG_MIN_1, ADR_LOCALANALOG_MAX_1, ADR_LOCALANALOG_MIN_2,
	ADR_LOCALANALOG_MAX_2, ADR_LOCALANALOG_MIN_3, ADR_LOCALANALOG_MAX_3, ADR_LOCALANALOG_MIN_4, ADR_LOCALANALOG_MAX_4,
	ADR_LOCALANALOG_MIN_5, ADR_LOCALANALOG_MAX_5, ADR_LOCALANALOG_MIN_6, ADR_LOCALANALOG_MAX_6, ADR_LOCALANALOG_MIN_7,
	ADR_LOCALANALOG_MAX_7, ADR_ADS111X_MIN_0, ADR_ADS111X_MAX_0, ADR_ADS111X_MIN_1, ADR_ADS111X_MAX_1, ADR_ADS111X_MIN_2,
	ADR_ADS111X_MAX_2, ADR_ADS111X_MIN_3, ADR_ADS111X_MAX_3,
	// Encoders, PWM
	ADR_ENCLOCAL_CPR, ADR_ENCLOCAL_OFS, ADR_MTENC_CONF1, ADR_MTENC_OFS, ADR_BISSENC_CONF1, ADR_BISSENC_OFS,
	ADR_SSI_CONF1, ADR_SSI_OFS, ADR_PWM_MODE,
};

} // namespace

TEST_SUITE("EepromAddresses") {

TEST_CASE_FIXTURE(FlashFixture, "NB_OF_VAR equals the number of ADR_ constants in eeprom_addresses.h") {
	CHECK(headerConstants().size() == NB_OF_VAR);
}

TEST_CASE_FIXTURE(FlashFixture, "no entry of VirtAddVarTab is 0, which would be a slot left empty by a too large NB_OF_VAR") {
	for (uint16_t adr : table()) {
		CHECK(adr != 0);
	}
}

TEST_CASE_FIXTURE(FlashFixture, "all entries of VirtAddVarTab are unique") {
	std::set<uint16_t> unique;
	for (uint16_t adr : table()) {
		CAPTURE(adr);
		CHECK(unique.insert(adr).second);
	}
	CHECK(unique.size() == NB_OF_VAR);
}

TEST_CASE_FIXTURE(FlashFixture, "no entry of VirtAddVarTab is 0xFFFF, the virtual address of an erased record") {
	CHECK_FALSE(contains(table(), 0xFFFF));
}

TEST_CASE_FIXTURE(FlashFixture, "every ADR_ constant of eeprom_addresses.h is in VirtAddVarTab") {
	const auto tab = table();
	for (const auto &[name, value] : headerConstants()) {
		CAPTURE(name);
		CHECK(contains(tab, (uint16_t)value));
	}
}

TEST_CASE_FIXTURE(FlashFixture, "every entry of VirtAddVarTab is the value of an ADR_ constant of eeprom_addresses.h") {
	std::set<uint32_t> values;
	for (const auto &[name, value] : headerConstants()) {
		values.insert(value);
	}
	for (uint16_t adr : table()) {
		CAPTURE(adr);
		CHECK(values.count(adr) == 1);
	}
}

TEST_CASE_FIXTURE(FlashFixture, "every ADR_ constant fits a 16 bit virtual address below the erased marker") {
	for (const auto &[name, value] : headerConstants()) {
		CAPTURE(name);
		CHECK(value < 0xFFFFU);
	}
}

TEST_CASE_FIXTURE(FlashFixture, "every flash address the axes, main classes, effects and inputs use is in VirtAddVarTab") {
	const auto tab = table();
	for (uint16_t adr : firmwareAddresses) {
		CAPTURE(adr);
		CHECK(contains(tab, adr));
	}
}

TEST_CASE_FIXTURE(FlashFixture, "exportableFlashAddresses has NB_EXPORTABLE_ADR unique entries that are all in VirtAddVarTab") {
	const auto tab = table();
	std::set<uint16_t> unique;
	for (uint16_t adr : exportable()) {
		CAPTURE(adr);
		CHECK(adr != 0); // a slot left empty by a too large NB_EXPORTABLE_ADR
		CHECK(unique.insert(adr).second);
		CHECK(contains(tab, adr));
	}
	CHECK(unique.size() == NB_EXPORTABLE_ADR);
}

TEST_CASE_FIXTURE(FlashFixture, "the version and the board calibration addresses are not exportable") {
	// eeprom_addresses.c: hardware / software version and the TMC ADC and encoder calibration of the board
	// ("could break something if imported on a different board", flash_helpers.cpp Flash_Dump comment)
	const std::vector<uint16_t> boardSpecific = {
		ADR_HW_VERSION, ADR_SW_VERSION,
		ADR_TMC1_ADC_I0_OFS, ADR_TMC1_ADC_I1_OFS, ADR_TMC1_ENC_OFFSET, ADR_TMC1_PHIE_OFS,
		ADR_TMC2_ADC_I0_OFS, ADR_TMC2_ADC_I1_OFS, ADR_TMC2_ENC_OFFSET, ADR_TMC2_PHIE_OFS,
		ADR_TMC3_ADC_I0_OFS, ADR_TMC3_ADC_I1_OFS, ADR_TMC3_ENC_OFFSET, ADR_TMC3_PHIE_OFS,
	};
	const auto exp = exportable();
	for (uint16_t adr : boardSpecific) {
		CAPTURE(adr);
		CHECK_FALSE(contains(exp, adr));
	}
}

TEST_CASE_FIXTURE(FlashFixture, "characterization: the CAN analog configuration is the only non-calibration address that is not exportable") {
	// NEEDS REVIEW: ADR_CANANALOG_CONF1 has no export flag in memory_map.csv (commented out in
	// exportableFlashAddresses), unlike every other input configuration. Intended?
	const auto exp = exportable();
	std::vector<uint16_t> notExported;
	for (uint16_t adr : table()) {
		if (!contains(exp, adr)) {
			notExported.push_back(adr);
		}
	}
	CHECK(notExported.size() == NB_OF_VAR - NB_EXPORTABLE_ADR);
	CHECK(notExported.size() == 15);
	CHECK(contains(notExported, ADR_CANANALOG_CONF1));
}

TEST_CASE_FIXTURE(FlashFixture, "the system variables are the only addresses below 0x100") {
	// eeprom_addresses.h: "Anything below 0x00ff is reserved for system variables"
	const std::vector<uint16_t> system = {ADR_HW_VERSION, ADR_SW_VERSION, ADR_FLASH_VERSION, ADR_CURRENT_CONFIG,
										  ADR_CANCONF1, ADR_I2CCONF1};
	for (uint16_t adr : table()) {
		if (adr < 0x100) {
			CAPTURE(adr);
			CHECK(contains(system, adr));
		}
	}
}

TEST_CASE_FIXTURE(FlashFixture, "every address of VirtAddVarTab keeps its value through a page transfer") {
	// The purpose of the table: EE_PageTransfer copies exactly the listed variables to the new page.
	// Store every variable once (value = 0x1000 + index), fill the rest of page 0 with updates of one variable
	// (same value), then one more write triggers the transfer.
	REQUIRE(init());
	const auto tab = table();
	for (uint32_t i = 0; i < tab.size(); i++) {
		REQUIRE(Flash_Write(tab[i], (uint16_t)(0x1000 + i)));
	}
	const uint16_t busy = ADR_CURRENT_CONFIG; // index 3 of the table
	REQUIRE(tab[3] == busy);
	for (uint32_t i = usedRecords(0); i < RECORDS_PER_PAGE; i++) {
		writeRecord(0, i, Record{busy, (uint16_t)(0x1000 + 3)}); // same value: the newest stays 0x1003
	}
	REQUIRE(usedRecords(0) == RECORDS_PER_PAGE);

	REQUIRE(Flash_Write(ADR_HW_VERSION, 0xABCD)); // page full -> transfer to page 1
	REQUIRE(pageStatus(1) == VALID_PAGE);
	REQUIRE(reboot());

	for (uint32_t i = 0; i < tab.size(); i++) {
		uint16_t v = 0;
		CAPTURE(tab[i]);
		CHECK(Flash_Read(tab[i], &v));
		CHECK(v == (tab[i] == ADR_HW_VERSION ? 0xABCD : (uint16_t)(0x1000 + i)));
	}
	CHECK(usedRecords(1) == NB_OF_VAR); // one record per variable, nothing else
}

TEST_CASE_FIXTURE(FlashFixture, "an address that is not in VirtAddVarTab is dropped by a page transfer") {
	// Consequence of a missing table entry (the reason for the consistency tests above)
	const uint16_t unlisted = 0x7FFE;
	REQUIRE_FALSE(contains(table(), unlisted));
	REQUIRE(init());
	REQUIRE(Flash_Write(unlisted, 0x1234));
	for (uint32_t i = usedRecords(0); i < RECORDS_PER_PAGE; i++) {
		writeRecord(0, i, Record{ADR_CURRENT_CONFIG, (uint16_t)i});
	}

	REQUIRE(Flash_Write(ADR_HW_VERSION, 1)); // transfer
	REQUIRE(pageStatus(1) == VALID_PAGE);

	uint16_t v = 0;
	CHECK_FALSE(Flash_Read(unlisted, &v));
}

} // TEST_SUITE
