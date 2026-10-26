/*
 * LocalAnalog (FFBoard/UserExtensions/Src/LocalAnalog.cpp): the analog pins of the board read from the ADC1 DMA buffer
 * (AIN_HADC = hadc1, ADC_PINS = 6 pins from rank ADC_CHAN_FPIN = 0), the pin mask, the processing through
 * AnalogAxisProcessing, its commands and its flash configuration.
 *
 * The ADC runs on the fake HAL ADC: startADC() (global_callbacks.cpp, as cppmain does) arms the circular DMA into
 * ADC1_BUF, HostTarget::adcConvert writes one converted sequence into it. Commands are executed directly with the ids
 * registered by name.
 *
 * Host-vs-target (FMA): scaled values (manual range, autorange) are compared with a tolerance of 2 counts: the
 * target's single precision FPU may contract the multiply-adds of AnalogAxisProcessing into VFMA (GCC
 * -ffp-contract=fast), the x86-64 host without -mfma does not, which can change the truncated integer by 1.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "LocalAnalog.h"
#include "eeprom_addresses.h"
#include "global_callbacks.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

/** Executes a command of a handler by name; replies in `replies` */
struct Commands {
	CommandHandler &handler;
	std::vector<CommandReply> replies;
	CommandStatus run(const std::string &name, CMDtype type, int64_t val = 0, int64_t adr = 0) {
		CmdHandlerCommanddef *def = handler.getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		replies.clear();
		ParsedCommand parsed;
		parsed.cmdId = def->cmdId;
		parsed.type = type;
		parsed.val = val;
		parsed.adr = adr;
		parsed.target = &handler;
		return handler.command(parsed, replies);
	}
	int64_t get(const std::string &name) {
		REQUIRE(run(name, CMDtype::get) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies[0].val;
	}
};

/** 12 bit ADC value as LocalAnalog reports it without processing: shifted to 16 bit and centred */
constexpr int32_t centred12(uint32_t adc) { return static_cast<int32_t>(adc << 4) - 0x7fff; }

struct LocalAnalogFixture : TargetFixture {
	/** Converts one sequence of the 8 ranks of ADC1 (6 pins, temperature, VREFINT) */
	void convert(const std::vector<uint32_t> &pins) {
		std::vector<uint32_t> ranks = pins;
		ranks.resize(ADC1_CHANNELS, 0);
		REQUIRE(HostTarget::adcConvert(&hadc1, ranks));
	}
};

} // namespace

TEST_SUITE("LocalAnalog") {

TEST_CASE_FIXTURE(LocalAnalogFixture, "each analog pin is read from its rank of the ADC1 DMA buffer") {
	LocalAnalog analog;
	startADC();
	convert({10, 20, 30, 40, 50, 60});
	CHECK(*analog.getAxes() == std::vector<int32_t>{centred12(10), centred12(20), centred12(30), centred12(40), centred12(50), centred12(60)});
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the 12 bit range of a pin is mapped to minus 0x7fff up to 0x7ff1") {
	LocalAnalog analog;
	startADC();
	convert({0, 0xFFF, 0x800});
	std::vector<int32_t> axes = *analog.getAxes();
	CHECK(axes[0] == -0x7fff);
	CHECK(axes[1] == 0x7ff1);
	CHECK(axes[2] == 1);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the ranks after the analog pins are not reported as axes") {
	LocalAnalog analog;
	startADC();
	REQUIRE(HostTarget::adcConvert(&hadc1, {0, 0, 0, 0, 0, 0, 0xFFF, 0xFFF}));
	std::vector<int32_t> axes = *analog.getAxes();
	CHECK(axes.size() == ADC_PINS);
	CHECK(axes == std::vector<int32_t>(ADC_PINS, -0x7fff));
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the pin values follow the latest ADC conversion") {
	LocalAnalog analog;
	startADC();
	convert({100});
	REQUIRE(analog.getAxes()->at(0) == centred12(100));
	convert({200});
	CHECK(analog.getAxes()->at(0) == centred12(200));
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "a 10 bit ADC resolution is shifted to the 16 bit range") {
	hadc1.Init.Resolution = ADC_RESOLUTION_10B;
	LocalAnalog analog;
	startADC();
	convert({0x3FF, 0x200, 0});
	std::vector<int32_t> axes = *analog.getAxes();
	CHECK(axes[0] == (0x3FF << 6) - 0x7fff);
	CHECK(axes[1] == (0x200 << 6) - 0x7fff);
	CHECK(axes[2] == -0x7fff);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "bits of the DMA buffer above the ADC resolution are ignored") {
	LocalAnalog analog;
	startADC();
	convert({100});
	ADC1_BUF[0] = 0xF000 | 100; // e.g. a stale value of another resolution
	CHECK(analog.getAxes()->at(0) == centred12(100));
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the pin mask leaves out disabled pins and keeps the order of the others") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("mask", CMDtype::set, 0b101010) == CommandStatus::OK);
	startADC();
	convert({10, 20, 30, 40, 50, 60});
	CHECK(*analog.getAxes() == std::vector<int32_t>{centred12(20), centred12(40), centred12(60)});
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "with no pin enabled there is no axis") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("mask", CMDtype::set, 0) == CommandStatus::OK);
	startADC();
	convert({10, 20});
	CHECK(analog.getAxes()->empty());
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "mask bits above the number of pins add no axis") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("mask", CMDtype::set, 0b11000001) == CommandStatus::OK);
	startADC();
	convert({10});
	CHECK(analog.getAxes()->size() == 1);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "all pins are enabled by default") {
	LocalAnalog analog;
	Commands c{analog};
	CHECK(c.get("mask") == 0xff);
	CHECK(analog.getAxes()->size() == ADC_PINS);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the mask command reads the mask that was set") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("mask", CMDtype::set, 0x15) == CommandStatus::OK);
	CHECK(c.get("mask") == 0x15);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the pins command reports the number of analog pins") {
	LocalAnalog analog;
	Commands c{analog};
	CHECK(c.get("pins") == ADC_PINS);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the pins command can not be set") {
	LocalAnalog analog;
	Commands c{analog};
	CHECK(c.run("pins", CMDtype::set, 3) == CommandStatus::ERR);
	CHECK(c.get("pins") == ADC_PINS);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the local analog source registers the processing commands") {
	LocalAnalog analog;
	for (const char *name : {"mask", "pins", "filter", "autocal", "values", "rawval", "min", "max"}) {
		CAPTURE(name);
		CHECK(analog.getCommandFromName(name) != nullptr);
	}
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the local analog source is the class AIN-Pins with the command name apin") {
	LocalAnalog analog;
	CHECK(std::string(analog.getInfo().name) == "AIN-Pins");
	CHECK(analog.getInfo().id == CLSID_ANALOG_LOCAL);
	CHECK(std::string(analog.getCommandHandlerInfo()->clsname) == "apin");
	CHECK(analog.getClassType() == ClassType::Analogsource);
	CHECK(LocalAnalog::isCreatable());
	CHECK(analog.getHelpstring() == "Analog pins source\n");
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "an unknown command id is not found") {
	LocalAnalog analog;
	ParsedCommand parsed;
	parsed.cmdId = 0x1234;
	parsed.type = CMDtype::get;
	std::vector<CommandReply> replies;
	CHECK(analog.command(parsed, replies) == CommandStatus::NOT_FOUND);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the values command replies with the processed pin values") {
	LocalAnalog analog;
	Commands c{analog};
	startADC();
	convert({0, 0x800, 0xFFF});
	REQUIRE(c.run("values", CMDtype::get) == CommandStatus::OK);
	REQUIRE(c.replies.size() == ADC_PINS);
	CHECK(c.replies[0].val == -0x7fff);
	CHECK(c.replies[1].val == 1);
	CHECK(c.replies[2].val == 0x7ff1);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "a manual range set by command scales the pin value") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("min", CMDtype::setat, centred12(1000), 0) == CommandStatus::OK);
	REQUIRE(c.run("max", CMDtype::setat, centred12(3000), 0) == CommandStatus::OK);
	startADC();
	convert({1000});
	CHECK_NEAR(analog.getAxes()->at(0), -32767, 2);
	convert({3000});
	CHECK(analog.getAxes()->at(0) == 0x7fff);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "autorange enabled by command learns the range of a pin from the conversions") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	startADC();
	convert({1000});
	analog.getAxes();
	convert({3000});
	analog.getAxes();
	REQUIRE(c.run("min", CMDtype::getat, 0, 0) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == centred12(1000));
	REQUIRE(c.run("max", CMDtype::getat, 0, 0) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == centred12(3000));
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "the rawval command replies with the unscaled pin values") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("min", CMDtype::setat, centred12(1000), 0) == CommandStatus::OK);
	REQUIRE(c.run("max", CMDtype::setat, centred12(3000), 0) == CommandStatus::OK);
	startADC();
	convert({2000});
	analog.getAxes();
	REQUIRE(c.run("rawval", CMDtype::get) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == centred12(2000));
}

// NEEDS REVIEW 88: getAxes compacts the enabled pins (LocalAnalog.cpp:79-87) before processAxes, so the range and the
// filter of "channel" n (min/max adr) belong to the n-th enabled pin, not to pin n. Changing the mask moves every
// stored range to another pin. The help says "adr=chan". Per pin intended?
TEST_CASE_FIXTURE(LocalAnalogFixture, "characterization: with pin 0 disabled the manual range of channel 0 applies to pin 1") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("mask", CMDtype::set, 0b10) == CommandStatus::OK);
	REQUIRE(c.run("min", CMDtype::setat, centred12(1000), 0) == CommandStatus::OK);
	REQUIRE(c.run("max", CMDtype::setat, centred12(3000), 0) == CommandStatus::OK);
	startADC();
	convert({0, 1000});
	CHECK_NEAR(analog.getAxes()->at(0), -32767, 2);
}

// ---------------------------------------------------------------- flash

TEST_CASE_FIXTURE(LocalAnalogFixture, "saving stores the pin mask in the low byte and the processing config in the high byte") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("mask", CMDtype::set, 0x2D) == CommandStatus::OK);
	REQUIRE(c.run("filter", CMDtype::set, 1) == CommandStatus::OK);
	analog.saveFlash();
	CHECK(HostPlatform::flash().at(ADR_LOCALANALOG_MASK) == (0x2D | (0b10 << 8)));
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "saving with autorange on stores the autorange bit") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	analog.saveFlash();
	CHECK(HostPlatform::flash().at(ADR_LOCALANALOG_MASK) == (0xff | (0b01 << 8)));
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "saving stores the manual range of every pin") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("min", CMDtype::setat, -100, 0) == CommandStatus::OK);
	REQUIRE(c.run("max", CMDtype::setat, 200, 5) == CommandStatus::OK);
	analog.saveFlash();
	auto &flash = HostPlatform::flash();
	CHECK(flash.at(ADR_LOCALANALOG_MIN_0) == static_cast<uint16_t>(-100));
	CHECK(flash.at(ADR_LOCALANALOG_MAX_5) == 200);
	CHECK(flash.at(ADR_LOCALANALOG_MIN_5) == 0x7fff); // never set: default
	CHECK(flash.count(ADR_LOCALANALOG_MIN_6) == 0);	 // only ADC_PINS pairs
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "saving with autorange on does not store the range") {
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	analog.saveFlash();
	CHECK(HostPlatform::flash().count(ADR_LOCALANALOG_MIN_0) == 0);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "a new local analog source restores the mask and the processing config from flash") {
	HostPlatform::flash()[ADR_LOCALANALOG_MASK] = 0x05 | (0b11 << 8);
	LocalAnalog analog;
	Commands c{analog};
	CHECK(c.get("mask") == 0x05);
	CHECK(c.get("filter") == 1);
	CHECK(c.get("autocal") == 1);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "all 8 mask bits are restored from flash") {
	HostPlatform::flash()[ADR_LOCALANALOG_MASK] = 0xA5;
	LocalAnalog analog;
	Commands c{analog};
	CHECK(c.get("mask") == 0xA5);
	CHECK(c.get("filter") == 0);
	CHECK(c.get("autocal") == 0);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "a new local analog source restores the manual range from flash") {
	HostPlatform::flash()[ADR_LOCALANALOG_MIN_2] = static_cast<uint16_t>(-500);
	HostPlatform::flash()[ADR_LOCALANALOG_MAX_2] = 700;
	LocalAnalog analog;
	Commands c{analog};
	REQUIRE(c.run("min", CMDtype::getat, 0, 2) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == -500);
	REQUIRE(c.run("max", CMDtype::getat, 0, 2) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == 700);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "without a stored config a new local analog source keeps the defaults") {
	LocalAnalog analog;
	Commands c{analog};
	CHECK(c.get("mask") == 0xff);
	CHECK(c.get("filter") == 0);
	CHECK(c.get("autocal") == 0);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "a saved configuration is restored by a new local analog source") {
	{
		LocalAnalog analog;
		Commands c{analog};
		REQUIRE(c.run("mask", CMDtype::set, 0x0F) == CommandStatus::OK);
		REQUIRE(c.run("filter", CMDtype::set, 1) == CommandStatus::OK);
		REQUIRE(c.run("min", CMDtype::setat, -1000, 3) == CommandStatus::OK);
		REQUIRE(c.run("max", CMDtype::setat, 1000, 3) == CommandStatus::OK);
		analog.saveFlash();
	}
	LocalAnalog analog;
	Commands c{analog};
	CHECK(c.get("mask") == 0x0F);
	CHECK(c.get("filter") == 1);
	CHECK(c.get("autocal") == 0);
	REQUIRE(c.run("min", CMDtype::getat, 0, 3) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == -1000);
	REQUIRE(c.run("max", CMDtype::getat, 0, 3) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == 1000);
}

// NEEDS REVIEW 90: with autorange stored as on, the manual range that is still in flash (from an earlier save with
// autorange off) is restored too (restoreMinMaxValues does not check autorange) and becomes the start of the autorange:
// the learned range can only grow from it, and a narrower real range is never learned until autocal is set again.
// Start autorange from an empty range after a restore intended?
TEST_CASE_FIXTURE(LocalAnalogFixture, "characterization: with autorange restored as on the stored manual range is the starting range") {
	HostPlatform::flash()[ADR_LOCALANALOG_MASK] = 0xff | (0b01 << 8); // autorange on
	HostPlatform::flash()[ADR_LOCALANALOG_MIN_0] = static_cast<uint16_t>(-0x7fff);
	HostPlatform::flash()[ADR_LOCALANALOG_MAX_0] = 0x7fff;
	LocalAnalog analog;
	Commands c{analog};
	startADC();
	convert({1000});
	analog.getAxes();
	convert({1100});
	analog.getAxes();
	REQUIRE(c.run("min", CMDtype::getat, 0, 0) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == -0x7fff);
	REQUIRE(c.run("max", CMDtype::getat, 0, 0) == CommandStatus::OK);
	CHECK(c.replies.at(0).val == 0x7fff);
}

TEST_CASE_FIXTURE(LocalAnalogFixture, "a local analog source can be created and deleted through the analog source registry") {
	const class_entry<AnalogSource> *entry = nullptr;
	for (const auto &e : AnalogSource::all_analogsources) {
		if (e.info.id == CLSID_ANALOG_LOCAL) {
			entry = &e;
		}
	}
	REQUIRE(entry != nullptr);
	std::unique_ptr<AnalogSource> source(entry->create());
	REQUIRE(source != nullptr);
	CHECK(source->getInfo().id == CLSID_ANALOG_LOCAL);
	CHECK(source->getAxes()->size() == ADC_PINS);
}

} // TEST_SUITE
