/*
 * Axis configuration tests (P2 group B): the command table and every axis command (get, set, info, limits and
 * invalid values), driver and encoder selection through the class choosers and the drvtype / enctype commands,
 * zeroing and position commands, out of range error reporting, USB suspend / resume, and the flash layout
 * (saveFlash / restoreFlash, encodeConfToInt / decodeConfFromInt) of FFBoard/Src/Axis.cpp.
 * The torque path of the axis is tested in test_axis_torque.cpp.
 *
 * Flash: HostPlatform::flash() (map backed fake of Flash_Read / Flash_Write, empty after each fixture reset).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "MockCommandInterface.h"
#include "TestHelpers.h"

#include "EncoderLocal.h"

#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace {

/** Flash addresses of axis 1 (eeprom_addresses.h) as they are used by AxisFlashAddresses */
constexpr uint16_t X_CONFIG = 0x301, X_POWER = 0x302, X_DEGREES = 0x303, X_MAX_SPEED = 0x304, X_MAX_ACCEL = 0x305,
				   X_MAX_SLEWRATE_DRV = 0x306, X_ENDSTOP = 0x307, X_EFFECTS1 = 0x308, X_SPEEDACCEL_FILTER = 0x309,
				   X_ENC_RATIO = 0x30A, X_EFFECTS2 = 0x30B, X_POSTPROCESS1 = 0x30C, X_EQ1 = 0x30D, X_EQ2 = 0x30E,
				   X_EQ3 = 0x30F, X_HANDSOFF_CONF = 0x310, X_HANDSOFF_ACCEL = 0x311;

/** Config word as Axis::encodeConfToInt builds it: encoder type in bits 0-5, driver type in bits 6-11 */
constexpr uint16_t confWord(uint8_t drv, uint8_t enc) { return (uint16_t)((enc & 0x3f) | ((drv & 0x3f) << 6)); }

/** Writes a value to the fake flash */
void flashWrite(uint16_t adr, uint16_t value) { HostPlatform::flash()[adr] = value; }

/** Reads a value from the fake flash. Fails the test if the address was never written */
uint16_t flashRead(uint16_t adr) {
	auto it = HostPlatform::flash().find(adr);
	INFO("flash address 0x" << std::hex << adr);
	REQUIRE(it != HostPlatform::flash().end());
	return it->second;
}

/** The reply strings of an info command */
std::vector<std::string> replyStrings(const std::vector<CommandReply> &replies) {
	std::vector<std::string> s;
	for (const CommandReply &r : replies) {
		s.push_back(r.reply);
	}
	return s;
}

/**
 * Axes with the real class choosers of the F407 feature set (no out-of-scope drivers: only "None"; encoders None,
 * local ABN, MT SPI, BissC, SSI), constructed by the test after it prepared the flash.
 */
struct RealAxisFixture : TargetFixture {
	std::unique_ptr<Axis> makeAxis(char name) { return std::make_unique<Axis>(name, &control); }

	static int64_t get(Axis &a, Axis_commands cmd) {
		std::vector<CommandReply> replies;
		ParsedCommand parsed;
		parsed.cmdId = static_cast<uint32_t>(cmd);
		parsed.type = CMDtype::get;
		parsed.target = &a;
		CHECK(a.command(parsed, replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}
	static CommandStatus command(Axis &a, Axis_commands cmd, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr) {
		std::vector<CommandReply> local;
		ParsedCommand parsed;
		parsed.cmdId = static_cast<uint32_t>(cmd);
		parsed.type = type;
		parsed.val = val;
		parsed.target = &a;
		return a.command(parsed, replies ? *replies : local);
	}

	volatile Control_t control;
};

/** AxisFixture with a command interface that records broadcasts (CommandHandler::broadcastCommandReply) */
struct AxisBroadcastFixture : AxisFixture {
	/** All broadcast results for a command id of the axis */
	std::vector<CommandResult> broadcasts(Axis_commands cmd) {
		std::vector<CommandResult> found;
		for (const CommandResult &r : itf.allResults()) {
			if (r.originalCommand.cmdId == static_cast<uint32_t>(cmd) && r.commandHandler == &axis()) {
				found.push_back(r);
			}
		}
		return found;
	}
	MockCommandInterface itf;
};

} // namespace

TEST_SUITE("Axis") {

// ---------------------------------------------------------------- command table

TEST_CASE_FIXTURE(AxisFixture, "the axis registers every command with its documented name, id and access flags") {
	struct Expected {
		const char *name;
		Axis_commands id;
		uint32_t flags;
	};
	const uint32_t GS = CMDFLAG_GET | CMDFLAG_SET;
	const std::vector<Expected> expected = {
		{"power", Axis_commands::power, GS},
		{"degrees", Axis_commands::degrees, GS},
		{"esgain", Axis_commands::esgain, GS},
		{"zeroenc", Axis_commands::zeroenc, CMDFLAG_GET},
		{"invert", Axis_commands::invert, GS},
		{"idlespring", Axis_commands::idlespring, GS},
		{"axisdamper", Axis_commands::axisdamper, GS},
		{"axisfriction", Axis_commands::axisfriction, GS},
		{"axisinertia", Axis_commands::axisinertia, GS},
		{"enctype", Axis_commands::enctype, GS | CMDFLAG_INFOSTRING},
		{"drvtype", Axis_commands::drvtype, GS | CMDFLAG_INFOSTRING},
		{"pos", Axis_commands::pos, CMDFLAG_GET},
		{"maxspeed", Axis_commands::maxspeed, GS},
		{"slewrate", Axis_commands::slewrate, GS},
		{"fxratio", Axis_commands::fxratio, GS},
		{"curtorque", Axis_commands::curtorque, CMDFLAG_GET},
		{"curpos", Axis_commands::curpos, CMDFLAG_GET},
		{"curspd", Axis_commands::curspd, CMDFLAG_GET},
		{"curaccel", Axis_commands::curaccel, CMDFLAG_GET},
		{"reduction", Axis_commands::reductionScaler, CMDFLAG_GET | CMDFLAG_SETADR},
		{"filterProfile_id", Axis_commands::filterProfileId, GS},
		{"filterSpeed", Axis_commands::filterSpeed, CMDFLAG_GET},
		{"filterAccel", Axis_commands::filterAccel, CMDFLAG_GET},
		{"cpr", Axis_commands::cpr, CMDFLAG_GET},
		{"expo", Axis_commands::expo, GS},
		{"exposcale", Axis_commands::exposcale, CMDFLAG_GET},
		{"equalizer", Axis_commands::equalizer, GS},
		{"eqb1", Axis_commands::eqb1, GS},
		{"eqb2", Axis_commands::eqb2, GS},
		{"eqb3", Axis_commands::eqb3, GS},
		{"eqb4", Axis_commands::eqb4, GS},
		{"eqb5", Axis_commands::eqb5, GS},
		{"eqb6", Axis_commands::eqb6, GS},
		{"handsoff", Axis_commands::handsoff, GS},
		{"handsoff_speed", Axis_commands::handsoff_speed, GS},
		{"handsoff_accel", Axis_commands::handsoff_accel, GS},
		{"maxSlewRateDrv", Axis_commands::maxSlewRateDrv, CMDFLAG_GET},
		{"calibrate_maxSlewRateDrv", Axis_commands::calibrate_maxSlewRateDrv, CMDFLAG_GET},
	};
	for (const Expected &e : expected) {
		INFO("command " << e.name);
		CmdHandlerCommanddef *def = axis().getCommandFromName(e.name);
		REQUIRE(def != nullptr);
		CHECK(def->cmdId == static_cast<uint32_t>(e.id));
		CHECK(def->flags == e.flags);
	}
}

TEST_CASE_FIXTURE(AxisFixture, "an unknown command id is not found by the axis") {
	std::vector<CommandReply> replies;
	CHECK(axisCommand(static_cast<Axis_commands>(0x99), CMDtype::get, 0, &replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

TEST_CASE("the axes X, Y and Z are command handler instances 0, 1 and 2 of class axis") {
	AxisFixture three(3);
	CHECK(three.axis(0).getCommandHandlerInstance() == 0);
	CHECK(three.axis(1).getCommandHandlerInstance() == 1);
	CHECK(three.axis(2).getCommandHandlerInstance() == 2);
	CHECK(three.axis(0).getInfo().id == CLSID_AXIS);
	CHECK(std::string(three.axis(0).getInfo().name) == "Axis");
}

// ---------------------------------------------------------------- defaults and empty flash

TEST_CASE_FIXTURE(RealAxisFixture, "an axis restored from empty flash reports the default settings") {
	auto a = makeAxis('X');
	CHECK(get(*a, Axis_commands::power) == 5000);
	CHECK(get(*a, Axis_commands::degrees) == 900);
	CHECK(get(*a, Axis_commands::esgain) == 127);
	CHECK(get(*a, Axis_commands::invert) == 1);
	CHECK(get(*a, Axis_commands::idlespring) == 127);
	CHECK(get(*a, Axis_commands::axisdamper) == 30);
	CHECK(get(*a, Axis_commands::axisfriction) == 0);
	CHECK(get(*a, Axis_commands::axisinertia) == 0);
	CHECK(get(*a, Axis_commands::drvtype) == 0);
	CHECK(get(*a, Axis_commands::enctype) == 0);
	CHECK(get(*a, Axis_commands::maxspeed) == 0);
	CHECK(get(*a, Axis_commands::slewrate) == MAX_SLEW_RATE); // getDrvSlewRate() of the "None" driver
	CHECK(get(*a, Axis_commands::maxSlewRateDrv) == MAX_SLEW_RATE);
	CHECK(get(*a, Axis_commands::fxratio) == 204);
	CHECK(get(*a, Axis_commands::filterProfileId) == 1);
	CHECK(get(*a, Axis_commands::expo) == 0);
	CHECK(get(*a, Axis_commands::exposcale) == 50);
	CHECK(get(*a, Axis_commands::equalizer) == 0);
	CHECK(get(*a, Axis_commands::eqb1) == 0);
	CHECK(get(*a, Axis_commands::eqb6) == 0);
	CHECK(get(*a, Axis_commands::handsoff) == 0);
	CHECK(get(*a, Axis_commands::handsoff_speed) == 720);
	CHECK(get(*a, Axis_commands::handsoff_accel) == 100);
	std::vector<CommandReply> replies;
	CHECK(command(*a, Axis_commands::reductionScaler, CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 1);
	CHECK(replies[0].adr == 1);
}

TEST_CASE_FIXTURE(RealAxisFixture, "restoring from empty flash pulses the error LED for each missing driver, speed and slew rate setting") {
	auto a = makeAxis('X');
	// config, max speed, max accel (slew rate) and driver slew rate. The other settings are optional
	CHECK(HostPlatform::leds().errPulses == 4);
}

TEST_CASE_FIXTURE(RealAxisFixture, "an axis restored from empty flash writes nothing to the flash") {
	auto a = makeAxis('X');
	CHECK(HostPlatform::flash().empty());
}

// ---------------------------------------------------------------- saveFlash

TEST_CASE_FIXTURE(AxisFixture, "saveFlash writes every setting of axis X to its flash address in the documented bit layout") {
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 700);
	axisCommand(Axis_commands::slewrate, CMDtype::set, 3000);
	axisCommand(Axis_commands::fxratio, CMDtype::set, 0x12);
	axisCommand(Axis_commands::esgain, CMDtype::set, 0x34);
	axisCommand(Axis_commands::power, CMDtype::set, 12345);
	axisCommand(Axis_commands::invert, CMDtype::set, 1);
	axisCommand(Axis_commands::degrees, CMDtype::set, 1080);
	update(); // the range is applied in the next update
	axisCommand(Axis_commands::idlespring, CMDtype::set, 0x56);
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 0x78);
	axisCommand(Axis_commands::axisfriction, CMDtype::set, 0x11);
	axisCommand(Axis_commands::axisinertia, CMDtype::set, 0x22);
	axisCommand(Axis_commands::reductionScaler, CMDtype::setat, 3, nullptr, 5); // 3/5: numerator 2, denominator 4
	axisCommand(Axis_commands::filterProfileId, CMDtype::set, 2);
	axisCommand(Axis_commands::expo, CMDtype::set, -5);
	axisCommand(Axis_commands::equalizer, CMDtype::set, 1);
	axisCommand(Axis_commands::eqb1, CMDtype::set, -1);
	axisCommand(Axis_commands::eqb2, CMDtype::set, 2);
	axisCommand(Axis_commands::eqb3, CMDtype::set, 30);
	axisCommand(Axis_commands::eqb4, CMDtype::set, -40);
	axisCommand(Axis_commands::eqb5, CMDtype::set, 120);
	axisCommand(Axis_commands::eqb6, CMDtype::set, -120);
	axisCommand(Axis_commands::handsoff, CMDtype::set, 1);
	axisCommand(Axis_commands::handsoff_speed, CMDtype::set, 600);
	axisCommand(Axis_commands::handsoff_accel, CMDtype::set, 250);

	axis().saveFlash();

	CHECK(flashRead(X_CONFIG) == confWord(MockMotorDriver::SELECTION_ID, MockEncoder::SELECTION_ID));
	CHECK(flashRead(X_CONFIG) == 0x0F3C);
	CHECK(flashRead(X_MAX_SPEED) == 700);
	CHECK(flashRead(X_MAX_ACCEL) == 3000);
	CHECK(flashRead(X_MAX_SLEWRATE_DRV) == MAX_SLEW_RATE);
	CHECK(flashRead(X_ENDSTOP) == 0x3412);		  // effect ratio low byte, endstop strength high byte
	CHECK(flashRead(X_POWER) == 12345);
	CHECK(flashRead(X_DEGREES) == (0x8000 | 1080)); // bit 15 invert
	CHECK(flashRead(X_EFFECTS1) == 0x7856);		  // idle spring low byte, damper high byte
	CHECK(flashRead(X_EFFECTS2) == 0x2211);		  // friction low byte, inertia high byte
	CHECK(flashRead(X_ENC_RATIO) == 0x0402);		  // numerator low byte, denominator high byte (both -1)
	CHECK(flashRead(X_SPEEDACCEL_FILTER) == 2);
	CHECK(flashRead(X_POSTPROCESS1) == 0x01FB);	  // expo -5 as int8 low byte, equalizer bit 8
	CHECK(flashRead(X_EQ1) == 0x02FF);			  // band 1 low byte, band 2 high byte, int8
	CHECK(flashRead(X_EQ2) == 0xD81E);
	CHECK(flashRead(X_EQ3) == 0x8878);
	CHECK(flashRead(X_HANDSOFF_CONF) == (0x8000 | 600)); // bit 15 enable
	CHECK(flashRead(X_HANDSOFF_ACCEL) == 250);
	CHECK(HostPlatform::flash().size() == 17);
}

TEST_CASE("the axes Y and Z save their settings at their own flash addresses") {
	AxisFixture f(3);
	f.axisCommand(Axis_commands::power, CMDtype::set, 1111, nullptr, 0, 1);
	f.axisCommand(Axis_commands::power, CMDtype::set, 2222, nullptr, 0, 2);

	f.axis(1).saveFlash();
	CHECK(HostPlatform::flash().size() == 17);
	CHECK(flashRead(0x342) == 1111); // ADR_AXIS2_POWER
	CHECK(flashRead(0x341) == confWord(MockMotorDriver::SELECTION_ID, MockEncoder::SELECTION_ID));
	CHECK(flashRead(0x351) == 100); // ADR_AXIS2_HANDSOFF_ACCEL, last address of axis 2
	HostPlatform::flash().clear();

	f.axis(2).saveFlash();
	CHECK(HostPlatform::flash().size() == 17);
	CHECK(flashRead(0x382) == 2222); // ADR_AXIS3_POWER
	CHECK(flashRead(0x381) == confWord(MockMotorDriver::SELECTION_ID, MockEncoder::SELECTION_ID));
	CHECK(flashRead(0x391) == 100); // ADR_AXIS3_HANDSOFF_ACCEL
}

// ---------------------------------------------------------------- restoreFlash

TEST_CASE_FIXTURE(AxisFixture, "restoreFlash applies every setting of the documented bit layout") {
	flashWrite(X_CONFIG, confWord(MockMotorDriver::SELECTION_ID, MockEncoder::SELECTION_ID));
	flashWrite(X_MAX_SPEED, 650);
	flashWrite(X_MAX_ACCEL, 1234);
	flashWrite(X_MAX_SLEWRATE_DRV, 4000);
	flashWrite(X_ENDSTOP, 0x4321);	// effect ratio 0x21, endstop 0x43
	flashWrite(X_POWER, 20000);
	flashWrite(X_DEGREES, 540);		// not inverted
	flashWrite(X_EFFECTS1, 0x0A0B); // idle spring 0x0B, damper 0x0A
	flashWrite(X_EFFECTS2, 0x0C0D); // friction 0x0D, inertia 0x0C
	flashWrite(X_ENC_RATIO, 0x0201); // numerator 1, denominator 2: reduction 2/3
	flashWrite(X_SPEEDACCEL_FILTER, 3);
	flashWrite(X_POSTPROCESS1, 0x0181); // expo -127, equalizer on
	flashWrite(X_EQ1, 0x0AF6);		  // -10, 10
	flashWrite(X_EQ2, 0x8814);		  // 20, -120
	flashWrite(X_EQ3, 0x7F80);		  // -128 and 127: clipped to -120 and 120
	flashWrite(X_HANDSOFF_CONF, 0x8000 | 300);
	flashWrite(X_HANDSOFF_ACCEL, 42);

	axis().restoreFlash();

	CHECK(axisGet(Axis_commands::drvtype) == MockMotorDriver::SELECTION_ID);
	CHECK(axisGet(Axis_commands::enctype) == MockEncoder::SELECTION_ID);
	CHECK(axisGet(Axis_commands::maxspeed) == 650);
	CHECK(axisGet(Axis_commands::slewrate) == 1234);
	CHECK(axisGet(Axis_commands::maxSlewRateDrv) == 4000);
	CHECK(axisGet(Axis_commands::fxratio) == 0x21);
	CHECK(axisGet(Axis_commands::esgain) == 0x43);
	CHECK(axisGet(Axis_commands::power) == 20000);
	CHECK(driver().powerLimit == 20000);
	CHECK(axisGet(Axis_commands::degrees) == 540);
	CHECK(axisGet(Axis_commands::invert) == 0);
	CHECK(axisGet(Axis_commands::idlespring) == 0x0B);
	CHECK(axisGet(Axis_commands::axisdamper) == 0x0A);
	CHECK(axisGet(Axis_commands::axisfriction) == 0x0D);
	CHECK(axisGet(Axis_commands::axisinertia) == 0x0C);
	std::vector<CommandReply> replies;
	axisCommand(Axis_commands::reductionScaler, CMDtype::get, 0, &replies);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 2);
	CHECK(replies[0].adr == 3);
	CHECK(axisGet(Axis_commands::filterProfileId) == 3);
	CHECK(axisGet(Axis_commands::expo) == -127);
	CHECK(axisGet(Axis_commands::equalizer) == 1);
	CHECK(axisGet(Axis_commands::eqb1) == -10);
	CHECK(axisGet(Axis_commands::eqb2) == 10);
	CHECK(axisGet(Axis_commands::eqb3) == 20);
	CHECK(axisGet(Axis_commands::eqb4) == -120);
	CHECK(axisGet(Axis_commands::eqb5) == -120);
	CHECK(axisGet(Axis_commands::eqb6) == 120);
	CHECK(axisGet(Axis_commands::handsoff) == 1);
	CHECK(axisGet(Axis_commands::handsoff_speed) == 300);
	CHECK(axisGet(Axis_commands::handsoff_accel) == 42);
}

TEST_CASE_FIXTURE(AxisFixture, "settings saved to flash are restored unchanged") {
	axisCommand(Axis_commands::power, CMDtype::set, 30000);
	axisCommand(Axis_commands::fxratio, CMDtype::set, 99);
	axisCommand(Axis_commands::esgain, CMDtype::set, 200);
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	axisCommand(Axis_commands::degrees, CMDtype::set, 360);
	update();
	axisCommand(Axis_commands::idlespring, CMDtype::set, 5);
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 6);
	axisCommand(Axis_commands::axisfriction, CMDtype::set, 7);
	axisCommand(Axis_commands::axisinertia, CMDtype::set, 8);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 1000);
	axisCommand(Axis_commands::slewrate, CMDtype::set, 500);
	axisCommand(Axis_commands::reductionScaler, CMDtype::setat, 7, nullptr, 3);
	axisCommand(Axis_commands::filterProfileId, CMDtype::set, 0);
	axisCommand(Axis_commands::expo, CMDtype::set, 33);
	axisCommand(Axis_commands::eqb3, CMDtype::set, -77);
	axisCommand(Axis_commands::handsoff_speed, CMDtype::set, 1500);
	axis().saveFlash();

	// Change everything, then restore
	axisCommand(Axis_commands::power, CMDtype::set, 1);
	axisCommand(Axis_commands::fxratio, CMDtype::set, 1);
	axisCommand(Axis_commands::esgain, CMDtype::set, 1);
	axisCommand(Axis_commands::invert, CMDtype::set, 1);
	axisCommand(Axis_commands::idlespring, CMDtype::set, 1);
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 1);
	axisCommand(Axis_commands::axisfriction, CMDtype::set, 1);
	axisCommand(Axis_commands::axisinertia, CMDtype::set, 1);
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 1);
	axisCommand(Axis_commands::reductionScaler, CMDtype::setat, 1, nullptr, 1);
	axisCommand(Axis_commands::filterProfileId, CMDtype::set, 3);
	axisCommand(Axis_commands::expo, CMDtype::set, 1);
	axisCommand(Axis_commands::eqb3, CMDtype::set, 1);
	axisCommand(Axis_commands::handsoff_speed, CMDtype::set, 1);
	axis().restoreFlash();

	CHECK(axisGet(Axis_commands::power) == 30000);
	CHECK(axisGet(Axis_commands::fxratio) == 99);
	CHECK(axisGet(Axis_commands::esgain) == 200);
	CHECK(axisGet(Axis_commands::invert) == 0);
	CHECK(axisGet(Axis_commands::degrees) == 360);
	CHECK(axisGet(Axis_commands::idlespring) == 5);
	CHECK(axisGet(Axis_commands::axisdamper) == 6);
	CHECK(axisGet(Axis_commands::axisfriction) == 7);
	CHECK(axisGet(Axis_commands::axisinertia) == 8);
	CHECK(axisGet(Axis_commands::maxspeed) == 1000);
	CHECK(axisGet(Axis_commands::slewrate) == 500);
	std::vector<CommandReply> replies;
	axisCommand(Axis_commands::reductionScaler, CMDtype::get, 0, &replies);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 7);
	CHECK(replies[0].adr == 3);
	CHECK(axisGet(Axis_commands::filterProfileId) == 0);
	CHECK(axisGet(Axis_commands::expo) == 33);
	CHECK(axisGet(Axis_commands::eqb3) == -77);
	CHECK(axisGet(Axis_commands::handsoff_speed) == 1500);
}

TEST_CASE_FIXTURE(RealAxisFixture, "the config word stores the encoder type in bits 0-5 and the driver type in bits 6-11") {
	// Driver 0 (None), encoder 2 (local ABN). Bits 12-15 are ignored
	flashWrite(X_CONFIG, 0xF000 | confWord(0, 2));
	auto a = makeAxis('X');
	CHECK(get(*a, Axis_commands::enctype) == 2);
	CHECK(get(*a, Axis_commands::drvtype) == 0);
	CHECK(dynamic_cast<EncoderLocal *>(a->getEncoder()) != nullptr);
	CHECK(HostPlatform::leds().errPulses == 3); // only the config word was present
	a->saveFlash();
	CHECK(flashRead(X_CONFIG) == confWord(0, 2)); // = 2
}

TEST_CASE_FIXTURE(RealAxisFixture, "characterization: a stored driver type that is not available is reported although the axis has no such driver") {
	// NEEDS REVIEW: restoreFlash decodes drvtype 1 (TMC4671, compiled out) into conf before setDrvType(1) rejects it
	// (Axis.cpp:226, 514-516). The axis keeps its "None" driver but drvtype reports 1 and saveFlash keeps writing 1.
	// Keeping the user's selection for a later firmware may be intended
	flashWrite(X_CONFIG, confWord(1, 0));
	auto a = makeAxis('X');
	CHECK(get(*a, Axis_commands::drvtype) == 1);
	REQUIRE(a->getDriver() != nullptr);
	CHECK(std::string(a->getDriver()->getInfo().name) == "None");
}

TEST_CASE_FIXTURE(RealAxisFixture, "a stored range of 0 degrees is replaced by the default range at the first update") {
	flashWrite(X_DEGREES, 0x8000); // inverted, 0 degrees
	auto a = makeAxis('X');
	CHECK(get(*a, Axis_commands::invert) == 1);
	a->prepareForUpdate();
	CHECK(get(*a, Axis_commands::degrees) == 900);
}

TEST_CASE_FIXTURE(AxisFixture, "a stored filter profile out of range is limited to the last profile like the command does" *
		doctest::skip() *
		doctest::description("KNOWN ISSUE (crash): restoreFlash uses the stored profile byte as index into the 4 element "
							 "filterSpeedCst / filterAccelCst arrays without a range check (Axis.cpp:294-299): out of "
							 "bounds read, aborts the run with _GLIBCXX_ASSERTIONS. Skipped")) {
	flashWrite(X_SPEEDACCEL_FILTER, 7);
	axis().restoreFlash();
	CHECK(axisGet(Axis_commands::filterProfileId) == 3);
}

TEST_CASE_FIXTURE(AxisFixture, "a hands-off acceleration threshold set by command survives a flash save and restore" *
		KNOWN_ISSUE("saveFlash truncates (uint16_t)(handsOffAccelThreshold * 1000.0) (Axis.cpp:366): the float 0.01f "
					"times 1000 is 9.99999977, saved as 9. About half of all values lose 1 per save. The first CHECK "
					"on the flash value fails")) {
	axisCommand(Axis_commands::handsoff_accel, CMDtype::set, 10);
	axis().saveFlash();
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(flashRead(X_HANDSOFF_ACCEL) == 10);
	axis().restoreFlash();
	CHECK(axisGet(Axis_commands::handsoff_accel) == 10);
}

// ---------------------------------------------------------------- scalar get / set commands

TEST_CASE_FIXTURE(AxisFixture, "power is read back as set and passed to the driver as power limit") {
	CHECK(axisCommand(Axis_commands::power, CMDtype::set, 12000) == CommandStatus::OK);
	CHECK(axisGet(Axis_commands::power) == 12000);
	CHECK(driver().powerLimit == 12000);
}

TEST_CASE_FIXTURE(AxisFixture, "characterization: a power value above 16 bits keeps only its low 16 bits") {
	// NEEDS REVIEW: the command value is converted to setPower(uint16_t) without a range check (Axis.cpp:1051, 498)
	axisCommand(Axis_commands::power, CMDtype::set, 0x10000 + 7);
	CHECK(axisGet(Axis_commands::power) == 7);
}

TEST_CASE_FIXTURE(AxisFixture, "power above 0x7fff is limited to 0x7fff" *
		KNOWN_ISSUE("Bug 32: setPower accepts any uint16_t (Axis.cpp:498-501); 0x7fff is full scale and the torque "
					"clipped to +-power is passed to MotorDriver::turn(int16_t), so a power above 0x7fff can reverse "
					"the motor. The first CHECK on the stored power fails")) {
	axisCommand(Axis_commands::power, CMDtype::set, 40000);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(axisGet(Axis_commands::power) <= 0x7fff);
	CHECK(driver().powerLimit <= 0x7fff);
}

TEST_CASE_FIXTURE(AxisFixture, "characterization: the 8 bit settings keep the low byte of a value above 255") {
	// NEEDS REVIEW: no range check, the value is converted to uint8_t (Axis.cpp:1060, 1086, 1097, 1108, 1119, 1202)
	axisCommand(Axis_commands::esgain, CMDtype::set, 300);
	axisCommand(Axis_commands::idlespring, CMDtype::set, 301);
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 302);
	axisCommand(Axis_commands::axisfriction, CMDtype::set, 303);
	axisCommand(Axis_commands::axisinertia, CMDtype::set, 304);
	axisCommand(Axis_commands::fxratio, CMDtype::set, 305);
	CHECK(axisGet(Axis_commands::esgain) == 44);
	CHECK(axisGet(Axis_commands::idlespring) == 45);
	CHECK(axisGet(Axis_commands::axisdamper) == 46);
	CHECK(axisGet(Axis_commands::axisfriction) == 47);
	CHECK(axisGet(Axis_commands::axisinertia) == 48);
	CHECK(axisGet(Axis_commands::fxratio) == 49);
}

TEST_CASE_FIXTURE(AxisFixture, "endstop gain, idle spring, damper, friction, inertia and effect ratio are read back as set") {
	axisCommand(Axis_commands::esgain, CMDtype::set, 10);
	axisCommand(Axis_commands::idlespring, CMDtype::set, 20);
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 0);
	axisCommand(Axis_commands::axisfriction, CMDtype::set, 255);
	axisCommand(Axis_commands::axisinertia, CMDtype::set, 40);
	axisCommand(Axis_commands::fxratio, CMDtype::set, 50);
	CHECK(axisGet(Axis_commands::esgain) == 10);
	CHECK(axisGet(Axis_commands::idlespring) == 20);
	CHECK(axisGet(Axis_commands::axisdamper) == 0);
	CHECK(axisGet(Axis_commands::axisfriction) == 255);
	CHECK(axisGet(Axis_commands::axisinertia) == 40);
	CHECK(axisGet(Axis_commands::fxratio) == 50);
}

TEST_CASE_FIXTURE(AxisFixture, "invert accepts 1 or more as on and everything else as off") {
	axisCommand(Axis_commands::invert, CMDtype::set, 5);
	CHECK(axisGet(Axis_commands::invert) == 1);
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	CHECK(axisGet(Axis_commands::invert) == 0);
	axisCommand(Axis_commands::invert, CMDtype::set, 1);
	CHECK(axisGet(Axis_commands::invert) == 1);
	axisCommand(Axis_commands::invert, CMDtype::set, -1);
	CHECK(axisGet(Axis_commands::invert) == 0);
}

TEST_CASE_FIXTURE(AxisFixture, "changing the inversion flips the reported position immediately") {
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	encoder().setDegrees(45.0f);
	update();
	REQUIRE(metrics().posDegrees == doctest::Approx(45.0).epsilon(1e-4));

	axisCommand(Axis_commands::invert, CMDtype::set, 1);

	CHECK(metrics().posDegrees == doctest::Approx(-45.0).epsilon(1e-4));
	CHECK(metrics().speed == 0); // metrics are reset
	update();
	CHECK(metrics().posDegrees == doctest::Approx(-45.0).epsilon(1e-4));
}

TEST_CASE_FIXTURE(AxisFixture, "a new range is reported after the next update") {
	axisCommand(Axis_commands::degrees, CMDtype::set, 1080);
	CHECK(axisGet(Axis_commands::degrees) == 900);
	update();
	CHECK(axisGet(Axis_commands::degrees) == 1080);
}

TEST_CASE_FIXTURE(AxisFixture, "a new range is not applied while the position is outside of it") {
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	encoder().setDegrees(60.0f);
	axisCommand(Axis_commands::degrees, CMDtype::set, 90); // 60 deg is outside of +-45 deg
	run(10);
	CHECK(axisGet(Axis_commands::degrees) == 900);

	encoder().setDegrees(40.0f); // now inside
	run(1);
	CHECK(axisGet(Axis_commands::degrees) == 90);
}

TEST_CASE_FIXTURE(AxisFixture, "characterization: a range of 0 degrees restores the range before the last change") {
	// NEEDS REVIEW: setDegrees(0) selects previousDegreesOfRotation (Axis.cpp:1010-1011) instead of rejecting 0
	axisCommand(Axis_commands::degrees, CMDtype::set, 1080);
	update();
	axisCommand(Axis_commands::degrees, CMDtype::set, 0);
	update();
	CHECK(axisGet(Axis_commands::degrees) == 900);
}

TEST_CASE_FIXTURE(AxisFixture, "characterization: the range keeps only the lower 15 bits of the value") {
	// NEEDS REVIEW: bit 15 is the inversion flag in flash; a larger value is masked, not rejected (Axis.cpp:1009)
	axisCommand(Axis_commands::degrees, CMDtype::set, 0x8000 | 540);
	update();
	CHECK(axisGet(Axis_commands::degrees) == 540);
}

TEST_CASE_FIXTURE(AxisFixture, "maxspeed is read back as set") {
	axisCommand(Axis_commands::maxspeed, CMDtype::set, 1234);
	CHECK(axisGet(Axis_commands::maxspeed) == 1234);
}

TEST_CASE_FIXTURE(AxisFixture, "the slew rate is limited to the maximum slew rate of the driver") {
	flashWrite(X_MAX_SLEWRATE_DRV, 2000);
	axis().restoreFlash();
	REQUIRE(axisGet(Axis_commands::maxSlewRateDrv) == 2000);

	axisCommand(Axis_commands::slewrate, CMDtype::set, 1500);
	CHECK(axisGet(Axis_commands::slewrate) == 1500);
	axisCommand(Axis_commands::slewrate, CMDtype::set, 5000);
	CHECK(axisGet(Axis_commands::slewrate) == 2000);
	axisCommand(Axis_commands::slewrate, CMDtype::set, 0); // 0 disables the limiter
	CHECK(axisGet(Axis_commands::slewrate) == 0);
}

TEST_CASE_FIXTURE(AxisFixture, "a slew rate set above the driver maximum is stored as the driver maximum") {
	flashWrite(X_MAX_SLEWRATE_DRV, 2000);
	axis().restoreFlash();
	axisCommand(Axis_commands::slewrate, CMDtype::set, 5000);
	axis().saveFlash(); // without a get command in between (the get also limits the value)
	CHECK(flashRead(X_MAX_ACCEL) == 2000);
}

TEST_CASE_FIXTURE(AxisFixture, "a stored slew rate above the driver maximum is reported as the driver maximum") {
	flashWrite(X_MAX_ACCEL, 5000);
	flashWrite(X_MAX_SLEWRATE_DRV, 2000);
	axis().restoreFlash();
	CHECK(axisGet(Axis_commands::slewrate) == 2000);
	axis().saveFlash();
	CHECK(flashRead(X_MAX_ACCEL) == 2000); // the get command also lowered the setting
}

TEST_CASE_FIXTURE(AxisFixture, "selecting a driver sets the slew rate to the maximum slew rate of that driver") {
	axisCommand(Axis_commands::slewrate, CMDtype::set, 100);
	axisCommand(Axis_commands::drvtype, CMDtype::set, MockMotorDriver::SELECTION_ID);
	CHECK(axisGet(Axis_commands::slewrate) == MAX_SLEW_RATE); // MockMotorDriver::getDrvSlewRate default
}

TEST_CASE_FIXTURE(AxisFixture, "the filter profile is limited to the 4 profiles and selects their speed and accel filter constants") {
	struct Profile {
		int64_t speedFreq, speedQ, accelFreq, accelQ;
	};
	const Profile profiles[4] = {{40, 55, 40, 30}, {70, 55, 55, 30}, {120, 55, 70, 30}, {180, 55, 120, 55}};
	for (int64_t id : {0, 1, 2, 3, 9, -1}) {
		INFO("profile " << id);
		axisCommand(Axis_commands::filterProfileId, CMDtype::set, id);
		const int64_t expected = id < 0 ? 3 : (id > 3 ? 3 : id); // negative values become large unsigned values
		CHECK(axisGet(Axis_commands::filterProfileId) == expected);
		std::vector<CommandReply> speed, accel;
		axisCommand(Axis_commands::filterSpeed, CMDtype::get, 0, &speed);
		axisCommand(Axis_commands::filterAccel, CMDtype::get, 0, &accel);
		REQUIRE(speed.size() == 1);
		REQUIRE(accel.size() == 1);
		CHECK(speed[0].type == CommandReplyType::DOUBLEINTS);
		CHECK(speed[0].val == profiles[expected].speedFreq);
		CHECK(speed[0].adr == profiles[expected].speedQ);
		CHECK(accel[0].val == profiles[expected].accelFreq);
		CHECK(accel[0].adr == profiles[expected].accelQ);
	}
}

TEST_CASE_FIXTURE(AxisFixture, "the filter constants can only be read") {
	std::vector<CommandReply> replies;
	CHECK(axisCommand(Axis_commands::filterSpeed, CMDtype::set, 5, &replies) == CommandStatus::OK);
	CHECK(axisCommand(Axis_commands::filterAccel, CMDtype::set, 5, &replies) == CommandStatus::OK);
	CHECK(axisCommand(Axis_commands::maxSlewRateDrv, CMDtype::set, 5, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
	CHECK(axisGet(Axis_commands::maxSlewRateDrv) == MAX_SLEW_RATE);
}

TEST_CASE_FIXTURE(AxisFixture, "expo is limited to -127..127 and exposcale reports the constant 50") {
	axisCommand(Axis_commands::expo, CMDtype::set, 64);
	CHECK(axisGet(Axis_commands::expo) == 64);
	axisCommand(Axis_commands::expo, CMDtype::set, 200);
	CHECK(axisGet(Axis_commands::expo) == 127);
	axisCommand(Axis_commands::expo, CMDtype::set, -200);
	CHECK(axisGet(Axis_commands::expo) == -127);
	CHECK(axisGet(Axis_commands::exposcale) == 50);
}

TEST_CASE_FIXTURE(AxisFixture, "the equalizer switch is read back as 0 or 1") {
	axisCommand(Axis_commands::equalizer, CMDtype::set, 2);
	CHECK(axisGet(Axis_commands::equalizer) == 1);
	axisCommand(Axis_commands::equalizer, CMDtype::set, 0);
	CHECK(axisGet(Axis_commands::equalizer) == 0);
}

TEST_CASE_FIXTURE(AxisFixture, "each equalizer band gain is stored separately and limited to -120..120") {
	const Axis_commands bands[6] = {Axis_commands::eqb1, Axis_commands::eqb2, Axis_commands::eqb3,
									Axis_commands::eqb4, Axis_commands::eqb5, Axis_commands::eqb6};
	for (int i = 0; i < 6; i++) {
		axisCommand(bands[i], CMDtype::set, 10 * (i + 1));
	}
	for (int i = 0; i < 6; i++) {
		INFO("band " << i + 1);
		CHECK(axisGet(bands[i]) == 10 * (i + 1));
	}
	axisCommand(Axis_commands::eqb2, CMDtype::set, 127);
	axisCommand(Axis_commands::eqb5, CMDtype::set, -128);
	CHECK(axisGet(Axis_commands::eqb2) == 120);
	CHECK(axisGet(Axis_commands::eqb5) == -120);
}

TEST_CASE_FIXTURE(AxisFixture, "characterization: an equalizer gain outside of -128..127 wraps around before it is limited") {
	// NEEDS REVIEW: setEqGain(uint8_t, int8_t) converts the command value to int8_t before the clip (Axis.cpp:1287, 1413-1415):
	// 200 becomes -56 instead of 120
	axisCommand(Axis_commands::eqb1, CMDtype::set, 200);
	CHECK(axisGet(Axis_commands::eqb1) == -56);
}

TEST_CASE_FIXTURE(AxisFixture, "hands-off detection settings are read back as set") {
	axisCommand(Axis_commands::handsoff, CMDtype::set, 1);
	axisCommand(Axis_commands::handsoff_speed, CMDtype::set, 500);
	axisCommand(Axis_commands::handsoff_accel, CMDtype::set, 250);
	CHECK(axisGet(Axis_commands::handsoff) == 1);
	CHECK(axisGet(Axis_commands::handsoff_speed) == 500);
	CHECK(axisGet(Axis_commands::handsoff_accel) == 250);
	axisCommand(Axis_commands::handsoff, CMDtype::set, 0);
	CHECK(axisGet(Axis_commands::handsoff) == 0);
}

TEST_CASE_FIXTURE(AxisFixture, "the hands-off acceleration threshold reads back the value that was set" *
		KNOWN_ISSUE("The get reply is handsOffAccelThreshold * 1000 in float, truncated to an integer (Axis.cpp:1320): "
					"0.251f * 1000 = 250.99998, read back as 250 (10 of the values 0..1999). The CHECK fails")) {
	axisCommand(Axis_commands::handsoff_accel, CMDtype::set, 251);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(axisGet(Axis_commands::handsoff_accel) == 251);
}

TEST_CASE_FIXTURE(AxisFixture, "the gear reduction is set with value and address and read back as numerator and denominator") {
	std::vector<CommandReply> replies;
	CHECK(axisCommand(Axis_commands::reductionScaler, CMDtype::setat, 3, nullptr, 2) == CommandStatus::OK);
	axisCommand(Axis_commands::reductionScaler, CMDtype::get, 0, &replies);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].type == CommandReplyType::DOUBLEINTS);
	CHECK(replies[0].val == 3);
	CHECK(replies[0].adr == 2);

	replies.clear();
	axisCommand(Axis_commands::reductionScaler, CMDtype::setat, 256, nullptr, 256);
	axisCommand(Axis_commands::reductionScaler, CMDtype::get, 0, &replies);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 256);
	CHECK(replies[0].adr == 256);
}

TEST_CASE_FIXTURE(AxisFixture, "a plain set of the gear reduction is ignored") {
	std::vector<CommandReply> replies;
	axisCommand(Axis_commands::reductionScaler, CMDtype::set, 5);
	axisCommand(Axis_commands::reductionScaler, CMDtype::get, 0, &replies);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 1);
	CHECK(replies[0].adr == 1);
}

TEST_CASE_FIXTURE(AxisFixture, "characterization: a gear reduction value of 0 wraps around to 256") {
	// NEEDS REVIEW: setGearRatio(cmd.val-1, cmd.adr-1) into uint8_t without a range check (Axis.cpp:1223); the help text says 1-256
	std::vector<CommandReply> replies;
	axisCommand(Axis_commands::reductionScaler, CMDtype::setat, 0, nullptr, 1);
	axisCommand(Axis_commands::reductionScaler, CMDtype::get, 0, &replies);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 256);
}

TEST_CASE_FIXTURE(AxisFixture, "the gear reduction scales the encoder angle to the axis angle") {
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	axisCommand(Axis_commands::reductionScaler, CMDtype::setat, 2, nullptr, 1); // encoder turns half per axis turn... ratio 2/1
	encoder().setDegrees(45.0f);
	update();
	CHECK(metrics().posDegrees == doctest::Approx(90.0).epsilon(1e-4));
}

// ---------------------------------------------------------------- encoder related commands

TEST_CASE_FIXTURE(AxisFixture, "pos reads the encoder count with the sign of the axis inversion") {
	encoder().setRawPosition(1234);
	CHECK(axisGet(Axis_commands::pos) == -1234); // inverted by default
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	CHECK(axisGet(Axis_commands::pos) == 1234);
}

TEST_CASE_FIXTURE(AxisFixture, "pos sets the encoder count with the sign of the axis inversion") {
	encoder().setRawPosition(1000);
	CHECK(axisCommand(Axis_commands::pos, CMDtype::set, 500) == CommandStatus::OK);
	CHECK(encoder().setPosHistory == std::vector<int32_t>{-500});
	CHECK(axisGet(Axis_commands::pos) == 500);

	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	axisCommand(Axis_commands::pos, CMDtype::set, 300);
	CHECK(encoder().setPosHistory.back() == 300);
	CHECK(axisGet(Axis_commands::pos) == 300);
}

TEST_CASE_FIXTURE(AxisFixture, "pos rejects an info request") {
	std::vector<CommandReply> replies;
	CHECK(axisCommand(Axis_commands::pos, CMDtype::info, 0, &replies) == CommandStatus::ERR);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(AxisFixture, "cpr reports the counts per rotation of the encoder and rejects a set") {
	encoder().setCpr(4096);
	CHECK(axisGet(Axis_commands::cpr) == 4096);
	CHECK(axisCommand(Axis_commands::cpr, CMDtype::set, 100) == CommandStatus::ERR);
	CHECK(encoder().getCpr() == 4096);
}

TEST_CASE_FIXTURE(AxisFixture, "zeroenc sets the encoder position to zero") {
	encoder().setDegrees(30.0f);
	std::vector<CommandReply> replies;
	CHECK(axisCommand(Axis_commands::zeroenc, CMDtype::get, 0, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
	CHECK(encoder().setPosHistory == std::vector<int32_t>{0});
	CHECK(axisGet(Axis_commands::pos) == 0);
}

TEST_CASE_FIXTURE(AxisFixture, "zeroenc restarts the force fade-in at 25 percent") {
	setFullScale();
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 0); // no damping of the position jump
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);
	run(1100);
	REQUIRE(std::abs(axis().getTorque()) == doctest::Approx(10000).epsilon(0.001));

	axisCommand(Axis_commands::zeroenc, CMDtype::get);
	run(1);
	CHECK(std::abs(axis().getTorque()) == doctest::Approx(2500).epsilon(0.01));
	run(100);
	CHECK(std::abs(axis().getTorque()) > 2500); // fading in
}

TEST_CASE_FIXTURE(AxisFixture, "zeroenc fades the force in within the 0.5 s fade time" *
		KNOWN_ISSUE("startForceFadeIn(start, fadeTime) is documented as 'fade from start to 1 over fadeTime' "
					"(Axis.h:356-361, Axis.cpp:995-1001), but updateTorque adds fadeTime / samplerate per cycle "
					"(Axis.cpp:875): the fade takes (1 - start) / fadeTime seconds, 1.5 s instead of 0.5 s after zeroenc "
					"(setPos: start 0.25, fadeTime 0.5). The CHECK after 0.5 s fails")) {
	setFullScale();
	axisCommand(Axis_commands::axisdamper, CMDtype::set, 0);
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);
	run(1100);
	REQUIRE(std::abs(axis().getTorque()) == doctest::Approx(10000).epsilon(0.001));

	axisCommand(Axis_commands::zeroenc, CMDtype::get);
	run(501); // 0.5 s after the first faded cycle
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(std::abs(axis().getTorque()) == doctest::Approx(10000).epsilon(0.001));
}

// ---------------------------------------------------------------- live values

TEST_CASE_FIXTURE(AxisFixture, "curtorque reports the torque of the last update") {
	setFullScale();
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);
	run(1100);
	REQUIRE(axis().getTorque() != 0);
	CHECK(axisGet(Axis_commands::curtorque) == axis().getTorque());
}

TEST_CASE_FIXTURE(AxisFixture, "characterization: curpos, curspd and curaccel report the metrics of the cycle before the last update") {
	// NEEDS REVIEW: the commands read metric.previous (Axis.cpp:1207, 1213, 1216), one cycle behind the axis metrics
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	encoder().setDegrees(90.0f);
	run(1);
	const metric_t before = metrics();
	encoder().setDegrees(180.0f);
	run(1);
	CHECK(axisGet(Axis_commands::curpos) == before.pos_scaled_16b);
	CHECK(axisGet(Axis_commands::curspd) == (int64_t)before.speed);
	CHECK(axisGet(Axis_commands::curaccel) == (int64_t)before.accel);
	CHECK(axisGet(Axis_commands::curpos) != metrics().pos_scaled_16b);
}

// ---------------------------------------------------------------- slew rate calibration

TEST_CASE_FIXTURE(AxisBroadcastFixture, "a slew rate calibration request to a driver without calibration is acknowledged and reported as unsupported") {
	std::vector<CommandReply> replies;
	CHECK(axisCommand(Axis_commands::calibrate_maxSlewRateDrv, CMDtype::get, 0, &replies) == CommandStatus::OK);
	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 1);
	CHECK(driver().slewCalibrationRequests == 1);
	auto b = broadcasts(Axis_commands::calibrate_maxSlewRateDrv);
	REQUIRE(b.size() == 1);
	REQUIRE(b[0].reply.size() == 1);
	CHECK(b[0].reply[0].reply == "Slew rate calibration unsupported");
	CHECK(b[0].reply[0].val == 1);
	run(5);
	CHECK(broadcasts(Axis_commands::maxSlewRateDrv).empty());
}

TEST_CASE_FIXTURE(AxisBroadcastFixture, "a finished slew rate calibration stores the measured rate, lowers the slew rate and broadcasts both") {
	driver().supportsSlewCalibration = true;
	axisCommand(Axis_commands::slewrate, CMDtype::set, 5000);
	axisCommand(Axis_commands::calibrate_maxSlewRateDrv, CMDtype::get);
	CHECK(broadcasts(Axis_commands::calibrate_maxSlewRateDrv).empty());
	run(3); // still measuring
	CHECK(itf.allResults().empty());

	driver().slewRate = 1200;
	driver().slewCalibrationInProgress = false;
	run(1);

	CHECK(axisGet(Axis_commands::maxSlewRateDrv) == 1200);
	CHECK(axisGet(Axis_commands::slewrate) == 1200);
	auto rate = broadcasts(Axis_commands::slewrate);
	REQUIRE(rate.size() == 1);
	CHECK(rate[0].reply.at(0).val == 1200);
	auto done = broadcasts(Axis_commands::calibrate_maxSlewRateDrv);
	REQUIRE(done.size() == 1);
	CHECK(done[0].reply.at(0).reply == "Slew rate calibration complete");
	CHECK(done[0].reply.at(0).val == 0);
	auto drvRate = broadcasts(Axis_commands::maxSlewRateDrv);
	REQUIRE(drvRate.size() == 1);
	CHECK(drvRate[0].reply.at(0).val == 1200);

	run(5); // reported once
	CHECK(broadcasts(Axis_commands::maxSlewRateDrv).size() == 1);
}

TEST_CASE_FIXTURE(AxisBroadcastFixture, "a finished slew rate calibration keeps a lower slew rate setting") {
	driver().supportsSlewCalibration = true;
	axisCommand(Axis_commands::slewrate, CMDtype::set, 800);
	axisCommand(Axis_commands::calibrate_maxSlewRateDrv, CMDtype::get);
	driver().slewRate = 1200;
	driver().slewCalibrationInProgress = false;
	run(1);
	CHECK(axisGet(Axis_commands::slewrate) == 800);
	CHECK(axisGet(Axis_commands::maxSlewRateDrv) == 1200);
	CHECK(broadcasts(Axis_commands::slewrate).empty());
	CHECK(broadcasts(Axis_commands::maxSlewRateDrv).size() == 1);
}

// ---------------------------------------------------------------- driver selection

TEST_CASE_FIXTURE(AxisFixture, "drvtype info lists the selectable drivers with id, creatable flag and name") {
	std::vector<CommandReply> replies;
	CHECK(axisCommand(Axis_commands::drvtype, CMDtype::info, 0, &replies) == CommandStatus::OK);
	CHECK(replyStrings(replies) == std::vector<std::string>{"0:1:None", "60:1:Mock driver"});
	REQUIRE(replies.size() == 2);
	CHECK(replies[1].type == CommandReplyType::STRING_OR_DOUBLEINT);
	CHECK(replies[1].adr == MockMotorDriver::SELECTION_ID);
	CHECK(replies[1].val == CLSID_CUSTOM);
}

TEST_CASE_FIXTURE(AxisFixture, "drvtype info shows the selected driver as available although no further instance can be created") {
	MockMotorDriver::creatable() = false;
	std::vector<CommandReply> replies;
	axisCommand(Axis_commands::drvtype, CMDtype::info, 0, &replies);
	CHECK(replyStrings(replies) == std::vector<std::string>{"0:1:None", "60:1:Mock driver"});
	REQUIRE(replies.size() == 2);
	CHECK(replies[1].type == CommandReplyType::STRING); // not creatable: no ids
}

TEST_CASE_FIXTURE(RealAxisFixture, "the test feature set (F407 without out-of-scope drivers) offers only the None driver to the axes X, Y and Z") {
	auto x = makeAxis('X');
	auto y = makeAxis('Y');
	auto z = makeAxis('Z');
	for (Axis *a : {x.get(), y.get(), z.get()}) {
		std::vector<CommandReply> replies;
		CHECK(command(*a, Axis_commands::drvtype, CMDtype::info, 0, &replies) == CommandStatus::OK);
		CHECK(replyStrings(replies) == std::vector<std::string>{"0:1:None"});
	}
}

TEST_CASE_FIXTURE(AxisFixture, "drvtype set creates, sets up and starts the selected driver") {
	MockMotorDriver *old = &driver();
	CHECK(axisCommand(Axis_commands::drvtype, CMDtype::set, MockMotorDriver::SELECTION_ID) == CommandStatus::OK);
	CHECK(&driver() != old); // a new instance
	CHECK(MockMotorDriver::instances() == 1);
	CHECK(driver().setupCalls == 1);
	CHECK(driver().startCalls == 1);
	CHECK(driver().running);
	CHECK(axisGet(Axis_commands::drvtype) == MockMotorDriver::SELECTION_ID);
}

TEST_CASE_FIXTURE(AxisFixture, "a new driver gets the encoder of the axis") {
	MockEncoder *enc = &encoder();
	axisCommand(Axis_commands::drvtype, CMDtype::set, 0);
	CHECK(MockMotorDriver::instances() == 0);
	CHECK(axisGet(Axis_commands::drvtype) == 0);
	CHECK(axis().getEncoder() == enc);
	axisCommand(Axis_commands::drvtype, CMDtype::set, MockMotorDriver::SELECTION_ID);
	CHECK(axis().getEncoder() == enc);
	CHECK(driver().assignedEncoder().get() == enc);
}

TEST_CASE_FIXTURE(AxisFixture, "an unknown driver type is ignored") {
	MockMotorDriver *old = &driver();
	axisCommand(Axis_commands::drvtype, CMDtype::set, 5); // ODrive: compiled out
	CHECK(&driver() == old);
	CHECK(driver().setupCalls == 1);
	CHECK(axisGet(Axis_commands::drvtype) == MockMotorDriver::SELECTION_ID);
}

TEST_CASE_FIXTURE(AxisFixture, "selecting a driver that cannot be created keeps a driver on the axis" *
		KNOWN_ISSUE("setDrvType resets the driver to the nullptr returned by ClassChooser::Create for a valid but not "
					"creatable id (Axis.cpp:519-525). The axis has no driver and getEncType, setEncType, "
					"updateDriveTorque, emergencyStop would dereference nullptr. The first CHECK fails")) {
	MockMotorDriver::creatable() = false;
	axisCommand(Axis_commands::drvtype, CMDtype::set, MockMotorDriver::SELECTION_ID);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(axis().getDriver() != nullptr);
	if (axis().getDriver() != nullptr) {
		CHECK(axisGet(Axis_commands::drvtype) == MockMotorDriver::SELECTION_ID);
	}
}

TEST_CASE_FIXTURE(AxisFixture, "a driver selected while USB is not connected is set up but stopped with zero torque") {
	HostUsb::state().mounted = false;
	axisCommand(Axis_commands::drvtype, CMDtype::set, MockMotorDriver::SELECTION_ID);
	CHECK(driver().setupCalls == 1);
	CHECK(driver().startCalls == 0);
	CHECK(driver().stopCalls == 1);
	CHECK_FALSE(driver().running);
	CHECK(driver().torqueHistory == std::vector<int16_t>{0, 0}); // turn(0) by usbSuspend and by MotorDriver::stopMotor
}

TEST_CASE_FIXTURE(AxisFixture, "characterization: selecting a driver while USB is not connected clears the usb_disabled flag") {
	// NEEDS REVIEW: Axis.cpp:535-538 sets control->usb_disabled = false when tud_connected() is false. The flag means
	// "FFB is disabled by USB" and FFBHIDMain::usbSuspend returns early while it is set. Intended (so that the next
	// suspend runs), or inverted?
	control.usb_disabled = true;
	HostUsb::state().mounted = false;
	axisCommand(Axis_commands::drvtype, CMDtype::set, MockMotorDriver::SELECTION_ID);
	CHECK_FALSE(control.usb_disabled);
}

// ---------------------------------------------------------------- encoder selection

TEST_CASE_FIXTURE(AxisFixture, "enctype info lists the selectable encoders with id, creatable flag and name") {
	std::vector<CommandReply> replies;
	CHECK(axisCommand(Axis_commands::enctype, CMDtype::info, 0, &replies) == CommandStatus::OK);
	CHECK(replyStrings(replies) == std::vector<std::string>{"0:1:None", "60:1:Mock encoder"});
}

TEST_CASE_FIXTURE(RealAxisFixture, "the F407 feature set offers the local, MT SPI, BissC and SSI encoders") {
	auto a = makeAxis('X');
	std::vector<CommandReply> replies;
	command(*a, Axis_commands::enctype, CMDtype::info, 0, &replies);
	std::vector<std::string> ids;
	for (const std::string &s : replyStrings(replies)) {
		ids.push_back(s.substr(0, s.find(':')));
	}
	CHECK(ids == std::vector<std::string>{"0", "2", "4", "5", "6"});
	CHECK(replyStrings(replies).at(1) == "2:1:Local ABN");
}

TEST_CASE_FIXTURE(AxisFixture, "enctype set creates a new encoder and passes it to the driver") {
	MockEncoder *old = &encoder();
	encoder().setDegrees(10.0f);
	run(3);
	CHECK(axisCommand(Axis_commands::enctype, CMDtype::set, MockEncoder::SELECTION_ID) == CommandStatus::OK);
	CHECK(&encoder() != old);
	CHECK(driver().assignedEncoder().get() == &encoder());
	CHECK(axisGet(Axis_commands::enctype) == MockEncoder::SELECTION_ID);
	CHECK(metrics().posDegrees == 0); // metrics reset to the new encoder position
	CHECK(metrics().speed == 0);
}

TEST_CASE_FIXTURE(AxisFixture, "enctype 0 selects the None encoder that always reports position 0") {
	axisCommand(Axis_commands::enctype, CMDtype::set, 0);
	CHECK(axisGet(Axis_commands::enctype) == 0);
	REQUIRE(axis().getEncoder() != nullptr);
	CHECK(dynamic_cast<MockEncoder *>(axis().getEncoder()) == nullptr);
	CHECK(axisGet(Axis_commands::pos) == 0);
	CHECK(axisGet(Axis_commands::cpr) == 0);
}

TEST_CASE_FIXTURE(AxisFixture, "characterization: an unknown encoder type reports the None encoder but keeps the current encoder") {
	// NEEDS REVIEW: setEncType sets conf.enctype = 0 for an invalid id but does not replace the encoder (Axis.cpp:561-563).
	// The reported (and saved) type no longer matches the encoder in use
	MockEncoder *old = &encoder();
	axisCommand(Axis_commands::enctype, CMDtype::set, 7);
	CHECK(axisGet(Axis_commands::enctype) == 0);
	CHECK(axis().getEncoder() == old);
}

TEST_CASE_FIXTURE(AxisFixture, "a driver with an integrated encoder reports encoder type 255 and keeps its own encoder") {
	std::shared_ptr<MockEncoder> integrated = driver().useIntegratedEncoder();
	CHECK(axisGet(Axis_commands::enctype) == 255);

	axisCommand(Axis_commands::enctype, CMDtype::set, MockEncoder::SELECTION_ID);

	CHECK(axisGet(Axis_commands::enctype) == 255);
	CHECK(axis().getEncoder() == integrated.get());
	axis().saveFlash();
	CHECK(flashRead(X_CONFIG) == confWord(MockMotorDriver::SELECTION_ID, 0)); // no external encoder saved
}

TEST_CASE_FIXTURE(RealAxisFixture, "characterization: an encoder that is in use by another axis leaves the second axis without encoder") {
	// NEEDS REVIEW: ClassChooser::Create returns nullptr for the local encoder (EncoderLocal::isCreatable is false while
	// it is in use); setEncType stores type 2 and passes nullptr to the driver (Axis.cpp:557-560) instead of rejecting it
	flashWrite(X_CONFIG, confWord(0, 2));
	auto x = makeAxis('X');
	auto y = makeAxis('Y');
	REQUIRE(dynamic_cast<EncoderLocal *>(x->getEncoder()) != nullptr);

	CHECK(command(*y, Axis_commands::enctype, CMDtype::set, 2) == CommandStatus::OK);

	CHECK(get(*y, Axis_commands::enctype) == 2);
	CHECK(y->getEncoder() == nullptr);
	CHECK(dynamic_cast<EncoderLocal *>(x->getEncoder()) != nullptr); // the first axis keeps it
}

TEST_CASE_FIXTURE(RealAxisFixture, "an axis without encoder rejects pos and reports cpr 0 and position 0") {
	flashWrite(X_CONFIG, confWord(0, 2));
	auto x = makeAxis('X');
	auto y = makeAxis('Y');
	command(*y, Axis_commands::enctype, CMDtype::set, 2); // in use by X: no encoder (see characterization above)
	REQUIRE(y->getEncoder() == nullptr);

	std::vector<CommandReply> replies;
	CHECK(command(*y, Axis_commands::pos, CMDtype::get, 0, &replies) == CommandStatus::ERR);
	CHECK(command(*y, Axis_commands::pos, CMDtype::set, 10, &replies) == CommandStatus::ERR);
	CHECK(replies.empty());
	CHECK(get(*y, Axis_commands::cpr) == 0);
	CHECK(command(*y, Axis_commands::zeroenc, CMDtype::get) == CommandStatus::OK);
	y->prepareForUpdate();
	CHECK(y->getMetrics()->posDegrees == 0);
}

TEST_CASE_FIXTURE(RealAxisFixture, "an encoder released by one axis can be selected by another axis") {
	flashWrite(X_CONFIG, confWord(0, 2));
	auto x = makeAxis('X');
	auto y = makeAxis('Y');
	command(*x, Axis_commands::enctype, CMDtype::set, 0);
	CHECK(command(*y, Axis_commands::enctype, CMDtype::set, 2) == CommandStatus::OK);
	CHECK(dynamic_cast<EncoderLocal *>(y->getEncoder()) != nullptr);
}

// ---------------------------------------------------------------- out of range error

TEST_CASE_FIXTURE(AxisFixture, "an axis far outside of its range stops the motor and reports an out of range warning once") {
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	run(1);
	const uint32_t errPulses = HostPlatform::leds().errPulses;
	encoder().setDegrees(1000.0f); // 1000 deg of a 900 deg range: more than twice the half range

	run(1);
	CHECK_FALSE(driver().running);
	CHECK(driver().stopCalls == 1);
	CHECK(HostPlatform::leds().errPulses == errPulses + 1);
	CHECK(errorsWithCode(ErrorCode::axisOutOfRange).size() == 1);

	run(1); // still outside: stopped again, no second error
	CHECK(errorsWithCode(ErrorCode::axisOutOfRange).size() == 1);
}

TEST_CASE_FIXTURE(AxisFixture, "an axis slightly outside of its range keeps the motor running") {
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	encoder().setDegrees(600.0f); // outside +-450 deg but less than twice
	run(2);
	CHECK(driver().running);
	CHECK(driver().stopCalls == 0);
	CHECK(errorsWithCode(ErrorCode::axisOutOfRange).empty());
}

TEST_CASE_FIXTURE(AxisFixture, "an axis outside of its range is not stopped while the driver is not ready") {
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	driver().ready = false;
	encoder().setDegrees(1000.0f);
	run(2);
	CHECK(driver().stopCalls == 0);
	CHECK(errorsWithCode(ErrorCode::axisOutOfRange).empty());
}

TEST_CASE_FIXTURE(AxisFixture, "clearing the out of range warning starts the motor again") {
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	encoder().setDegrees(1000.0f);
	run(1);
	REQUIRE_FALSE(driver().running);
	encoder().setDegrees(0.0f);
	run(1);
	CHECK_FALSE(driver().running); // only the cleared error restarts it

	ErrorHandler::clearError(Error(ErrorCode::axisOutOfRange, ErrorType::warning, ""));

	CHECK(driver().running);
	CHECK(driver().startCalls == 2);
}

TEST_CASE_FIXTURE(AxisFixture, "clearing another error does not start the motor") {
	driver().stopMotor();
	const Error other(ErrorCode::emergencyStop, ErrorType::critical, "other");
	ErrorHandler::addError(other);
	ErrorHandler::clearError(other);
	CHECK_FALSE(driver().running);
}

// ---------------------------------------------------------------- USB suspend and resume, sample rate

TEST_CASE_FIXTURE(AxisFixture, "usbSuspend sends zero torque and stops the driver") {
	setFullScale();
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);
	run(1100);
	REQUIRE(driver().lastTorque != 0);

	axis().usbSuspend();

	CHECK(driver().lastTorque == 0);
	CHECK_FALSE(driver().running);
}

TEST_CASE_FIXTURE(AxisFixture, "usbResume starts the driver with a new force fade-in") {
	setFullScale();
	uint8_t cf = startEffect(FFB_EFFECT_CONSTANT);
	host.setConstantForce(cf, 10000);
	run(1100);
	axis().usbSuspend();

	axis().usbResume();

	CHECK(driver().running);
	run(1);
	CHECK(std::abs(axis().getTorque()) < 50); // fade starts at 0
	run(250);
	CHECK(std::abs(axis().getTorque()) > 50); // fading in
	CHECK(std::abs(axis().getTorque()) < 9000);
}

TEST_CASE_FIXTURE(AxisFixture, "after a sample rate change the speed is computed with the new rate") {
	axisCommand(Axis_commands::invert, CMDtype::set, 0);
	axis().updateSamplerate(500.0f);
	float deg = 0;
	for (int i = 0; i < 400; i++) { // 1 deg per update at 500 Hz = 500 deg/s
		deg += 1.0f;
		encoder().setDegrees(deg);
		update();
	}
	CHECK(metrics().speed == doctest::Approx(500.0).epsilon(0.01));
	CHECK(axisGet(Axis_commands::filterProfileId) == 1); // profile kept
}

} // TEST_SUITE
