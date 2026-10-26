/*
 * AnalogAxisProcessing (FFBoard/Src/AnalogAxisProcessing.cpp, Inc/AnalogAxisProcessing.h): the shared processing of
 * the analog sources (LocalAnalog, ADS111X_AnalogSource): per axis lowpass filter, autorange / manual range scaling
 * to -0x7fff..0x7fff, clipping, its commands (filter, autocal, values, rawval, min, max), the config bits and the
 * min/max flash helpers.
 *
 * The class is tested through a small command handler that owns it like the firmware sources do (Proc below: a
 * CommandHandler and an AnalogAxisProcessing that registers its commands in itself). Commands are executed directly
 * with the ids the processing registered, looked up by name.
 *
 * Host-vs-target notes: the scaling is single precision float math with int <-> float conversions (val *= scaler
 * truncates, then the offset is subtracted in float). x86-64 SSE and the Cortex-M4 FPU both evaluate float in single
 * precision (FLT_EVAL_METHOD 0), so the results are expected to match; the tests still allow a tolerance of a few
 * counts where truncation is involved. The lowpass filter is a CMSIS-DSP biquad in the DSP build and the plain C
 * biquad otherwise: the expected values are computed by an independent double precision biquad and both builds must
 * stay within 2 counts of it.
 * FMA: the Cortex-M4 FPU has VFMA and GCC may contract `val * scaler - (scaler * min + 0x7fff)` and the biquad
 * multiply-adds into fused operations on the target (-ffp-contract=fast is GCC's default outside ISO C mode), while the
 * x86-64 host build without -mfma does not. A fused result can differ in the last float bit, i.e. by 1 count after the
 * truncation to int; the tolerances of 2 counts (scaling, filter) cover this. Exact comparisons are only used where the
 * result is clipped or not scaled.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "MockAnalogSource.h"
#include "TestHelpers.h"

#include "AnalogAxisProcessing.h"
#include "CommandHandler.h"
#include "ClassIDs.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace {

/** A command handler owning an AnalogAxisProcessing like LocalAnalog / ADS111X_AnalogSource do */
struct Proc : CommandHandler, AnalogAxisProcessing {
	Proc(uint32_t axes, AnalogSource *source, bool filters = true, bool autoscale = true, bool raw = true, bool manual = true)
		: CommandHandler("aproc", CLSID_CUSTOM, 0), AnalogAxisProcessing(axes, source, this, filters, autoscale, raw, manual) {}
	static inline ClassIdentifier info = {.name = "aproc", .id = CLSID_CUSTOM, .visibility = ClassVisibility::visible};
	const ClassIdentifier getInfo() override { return info; }
	CommandStatus command(const ParsedCommand &cmd, std::vector<CommandReply> &replies) override { return AnalogAxisProcessing::command(cmd, replies); }

	std::vector<CommandReply> replies;
	/** Executes a registered command by name; the replies are kept in `replies` */
	CommandStatus run(const std::string &name, CMDtype type, int64_t val = 0, int64_t adr = 0) {
		CmdHandlerCommanddef *def = getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		return runId(def->cmdId, type, val, adr);
	}
	CommandStatus runId(uint32_t id, CMDtype type, int64_t val = 0, int64_t adr = 0) {
		replies.clear();
		ParsedCommand parsed;
		parsed.cmdId = id;
		parsed.type = type;
		parsed.val = val;
		parsed.adr = adr;
		parsed.target = this;
		return command(parsed, replies);
	}
	int64_t getAt(const std::string &name, int64_t adr) {
		REQUIRE(run(name, CMDtype::getat, 0, adr) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies[0].val;
	}
	int64_t get(const std::string &name) {
		REQUIRE(run(name, CMDtype::get) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies[0].val;
	}
	/** Processes one sample per axis and returns the result */
	std::vector<int32_t> process(std::vector<int32_t> values) {
		processAxes(values);
		return values;
	}
	void setRange(uint8_t axis, int32_t min, int32_t max) {
		REQUIRE(run("min", CMDtype::setat, min, axis) == CommandStatus::OK);
		REQUIRE(run("max", CMDtype::setat, max, axis) == CommandStatus::OK);
	}
	bool registered(const std::string &name) { return getCommandFromName(name) != nullptr; }
};

/** Default min/max pair (MinMaxPair: nothing learned) */
constexpr int32_t NO_MIN = 0x7fff;
constexpr int32_t NO_MAX = -0x7fff;

/** Expected scaling of AnalogAxisProcessing: (val * 0xffff / range * 1.05) shifted so that min maps to -0x7fff */
double scaled(double val, double min, double max) {
	double scaler = 65535.0 / (max - min) * 1.05;
	double out = val * scaler - (scaler * min + 32767.0);
	return std::max(-32767.0, std::min(32767.0, out));
}

/** Independent lowpass biquad (Fc 0.03, Q 0.5 like AnalogAxisProcessing::filterF / filterQ), double precision */
struct ReferenceLowpass {
	double a0, a1, a2, b1, b2;
	double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
	ReferenceLowpass() {
		const double K = std::tan(M_PI * 0.03), Q = 0.5;
		const double norm = 1 / (1 + K / Q + K * K);
		a0 = K * K * norm;
		a1 = 2 * a0;
		a2 = a0;
		b1 = 2 * (K * K - 1) * norm;
		b2 = (1 - K / Q + K * K) * norm;
	}
	double process(double x) {
		double y = a0 * x + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
		x2 = x1;
		x1 = x;
		y2 = y1;
		y1 = y;
		return y;
	}
};

} // namespace

TEST_SUITE("AnalogAxisProcessing") {

// ---------------------------------------------------------------- construction and command registration

TEST_CASE_FIXTURE(TargetFixture, "with every option allowed the processing registers the filter, autocal, values, rawval, min and max commands") {
	MockAnalogSource source;
	Proc p(2, &source);
	REQUIRE(p.registered("filter"));
	CHECK(p.getCommandFromName("filter")->flags == (CMDFLAG_GET | CMDFLAG_SET));
	REQUIRE(p.registered("autocal"));
	CHECK(p.getCommandFromName("autocal")->flags == (CMDFLAG_GET | CMDFLAG_SET));
	REQUIRE(p.registered("values"));
	CHECK(p.getCommandFromName("values")->flags == CMDFLAG_GET);
	REQUIRE(p.registered("rawval"));
	CHECK(p.getCommandFromName("rawval")->flags == CMDFLAG_GET);
	REQUIRE(p.registered("min"));
	CHECK(p.getCommandFromName("min")->flags == (CMDFLAG_GETADR | CMDFLAG_SETADR));
	REQUIRE(p.registered("max"));
	CHECK(p.getCommandFromName("max")->flags == (CMDFLAG_GETADR | CMDFLAG_SETADR));
}

TEST_CASE_FIXTURE(TargetFixture, "without options and without a source the processing registers no command") {
	Proc p(2, nullptr, false, false, false, false);
	for (const char *name : {"filter", "autocal", "values", "rawval", "min", "max"}) {
		CAPTURE(name);
		CHECK_FALSE(p.registered(name));
	}
}

TEST_CASE_FIXTURE(TargetFixture, "with a source but no options the processing registers only the values command") {
	MockAnalogSource source;
	Proc p(2, &source, false, false, false, false);
	CHECK(p.registered("values"));
	for (const char *name : {"filter", "autocal", "rawval", "min", "max"}) {
		CAPTURE(name);
		CHECK_FALSE(p.registered(name));
	}
}

TEST_CASE("a processing created without a command handler can be used without commands" * doctest::skip() *
		  doctest::description("KNOWN ISSUE (crash): the constructor documents cmdHandler nullptr as 'disable commands', but "
							   "registers values (AnalogAxisProcessing.cpp:44-45, when a source is given) and rawval (:47-48) "
							   "without checking cmdHandler: null pointer call, SIGSEGV. Latent: every firmware caller passes "
							   "itself. Skipped")) {
	MockAnalogSource source;
	AnalogAxisProcessing p(1, &source, nullptr, true, true, true, true);
	std::vector<int32_t> values{100};
	p.processAxes(values);
	CHECK(values[0] == 100);
}

// ---------------------------------------------------------------- scaling and clipping

TEST_CASE_FIXTURE(TargetFixture, "without a learned or manual range the values pass unchanged") {
	Proc p(2, nullptr);
	CHECK(p.process({1234, -20000}) == std::vector<int32_t>{1234, -20000});
}

TEST_CASE_FIXTURE(TargetFixture, "values beyond the 16 bit range are clipped to plus and minus 0x7fff") {
	Proc p(2, nullptr);
	CHECK(p.process({40000, -40000}) == std::vector<int32_t>{0x7fff, -0x7fff});
}

TEST_CASE_FIXTURE(TargetFixture, "values are clipped also when scaling is not available") {
	Proc p(1, nullptr, false, false, false, false);
	CHECK(p.process({0x8000}) == std::vector<int32_t>{0x7fff});
}

TEST_CASE_FIXTURE(TargetFixture, "only the first axisAmount values of the buffer are processed") {
	Proc p(1, nullptr);
	CHECK(p.process({40000, 40000}) == std::vector<int32_t>{0x7fff, 40000});
}

TEST_CASE_FIXTURE(TargetFixture, "a buffer shorter than the axis amount is processed without touching other memory") {
	Proc p(4, nullptr);
	CHECK(p.process({-50000}) == std::vector<int32_t>{-0x7fff});
}

TEST_CASE_FIXTURE(TargetFixture, "a manual range maps its minimum to the lowest output value") {
	Proc p(1, nullptr);
	p.setRange(0, -1000, 1000);
	CHECK_NEAR(p.process({-1000})[0], -32767, 2);
}

TEST_CASE_FIXTURE(TargetFixture, "a value below the manual range is clipped to the lowest output value") {
	Proc p(1, nullptr);
	p.setRange(0, -1000, 1000);
	CHECK(p.process({-5000})[0] == -0x7fff);
}

TEST_CASE_FIXTURE(TargetFixture, "a value above the manual range is clipped to the highest output value") {
	Proc p(1, nullptr);
	p.setRange(0, -1000, 1000);
	CHECK(p.process({5000})[0] == 0x7fff);
}

// NEEDS REVIEW 86: the autorange margin (autorangeScale 1.05, AnalogAxisProcessing.h:96) is applied only on the top side:
// the minimum maps to -0x7fff, the middle of the range to about +1638 and the top 4.8 % of the range saturate. The same
// factor applies to a manual range, so a manual max is reached already at about 95 % of the range. Symmetric margin
// (or no margin for a manual range) intended?
TEST_CASE_FIXTURE(TargetFixture, "characterization: the middle of the range maps to about 5 percent of full scale above zero") {
	Proc p(1, nullptr);
	p.setRange(0, -1000, 1000);
	CHECK_NEAR(p.process({0})[0], scaled(0, -1000, 1000), 2);
	CHECK_NEAR(p.process({0})[0], 1638, 2);
}

// NEEDS REVIEW 86
TEST_CASE_FIXTURE(TargetFixture, "characterization: the full scale output is reached at about 95 percent of the range") {
	Proc p(1, nullptr);
	p.setRange(0, 0, 10000);
	CHECK(p.process({9524})[0] == 0x7fff);
	CHECK(p.process({9500})[0] < 0x7fff);
	CHECK_NEAR(p.process({5000})[0], scaled(5000, 0, 10000), 2);
}

TEST_CASE_FIXTURE(TargetFixture, "each axis is scaled with its own range") {
	Proc p(2, nullptr);
	p.setRange(0, 0, 1000);
	p.setRange(1, -20000, 0);
	// Margin independent (NEEDS REVIEW 86): the range minimum maps to -0x7fff and the maximum to 0x7fff for any margin >= 1
	std::vector<int32_t> out = p.process({0, -20000});
	CHECK_NEAR(out[0], -32767, 2);
	CHECK_NEAR(out[1], -32767, 2);
	out = p.process({1000, 0});
	CHECK(out[0] == 0x7fff);
	CHECK(out[1] == 0x7fff);
	out = p.process({-20000, 1000});
	CHECK(out[0] == -0x7fff); // below the range of axis 0
	CHECK(out[1] == 0x7fff);  // above the range of axis 1
}

TEST_CASE_FIXTURE(TargetFixture, "a range of one count or less is not scaled") {
	Proc p(1, nullptr);
	p.setRange(0, 100, 101);
	CHECK(p.process({100})[0] == 100);
	p.setRange(0, 100, 100);
	CHECK(p.process({250})[0] == 250);
}

TEST_CASE_FIXTURE(TargetFixture, "a range of two counts is scaled") {
	Proc p(1, nullptr);
	p.setRange(0, 100, 102);
	CHECK(p.process({100})[0] == -0x7fff);
	CHECK(p.process({102})[0] == 0x7fff);
}

// NEEDS REVIEW 86: with the full range the only visible effect of the scaling is the autorange margin (0 -> about +1638)
TEST_CASE_FIXTURE(TargetFixture, "characterization: the full manual range of plus and minus 0x7fff is still scaled (NEEDS REVIEW 86)") {
	Proc p(1, nullptr);
	p.setRange(0, -0x7fff, 0x7fff);
	CHECK_NEAR(p.process({0})[0], scaled(0, -0x7fff, 0x7fff), 2);
	CHECK(p.process({0})[0] != 0);
}

TEST_CASE_FIXTURE(TargetFixture, "a learned range wider than 16 bit is not scaled") {
	Proc p(1, nullptr);
	REQUIRE(p.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	p.process({-40000});
	p.process({40000});
	CHECK(p.process({1000})[0] == 1000);
}

// NEEDS REVIEW 89: a manual range with min above max (a way to invert an axis in other tools) leaves the values unscaled
// (range <= 1); the axis is neither inverted nor rejected. Inverting is not offered anywhere else. Intended?
TEST_CASE_FIXTURE(TargetFixture, "characterization: a manual range with the minimum above the maximum leaves the values unscaled") {
	Proc p(1, nullptr);
	p.setRange(0, 1000, -1000);
	CHECK(p.process({500})[0] == 500);
	CHECK(p.process({-500})[0] == -500);
}

TEST_CASE_FIXTURE(TargetFixture, "raw mode disables the scaling") {
	Proc p(1, nullptr);
	p.setRange(0, -1000, 1000);
	AnalogProcessingConfig conf = p.getAnalogProcessingConfig();
	conf.raw = true;
	p.setAnalogProcessingConfig(conf);
	CHECK(p.process({500})[0] == 500);
}

TEST_CASE_FIXTURE(TargetFixture, "raw mode still clips the values") {
	Proc p(1, nullptr);
	p.setAnalogProcessingConfig(AnalogProcessingConfig{.autorange = false, .filtersEnabled = false, .raw = true});
	CHECK(p.process({70000})[0] == 0x7fff);
}

// ---------------------------------------------------------------- autorange

TEST_CASE_FIXTURE(TargetFixture, "autorange learns the smallest and the largest value of each axis") {
	Proc p(2, nullptr);
	REQUIRE(p.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	p.process({100, -5});
	p.process({-300, 7});
	p.process({50, 2});
	CHECK(p.getAt("min", 0) == -300);
	CHECK(p.getAt("max", 0) == 100);
	CHECK(p.getAt("min", 1) == -5);
	CHECK(p.getAt("max", 1) == 7);
}

TEST_CASE_FIXTURE(TargetFixture, "autorange scales the values to the learned range") {
	Proc p(1, nullptr);
	REQUIRE(p.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	p.process({-2000});
	p.process({2000});
	// Margin independent (NEEDS REVIEW 86): ends of the learned range and monotonic in between
	CHECK_NEAR(p.process({-2000})[0], -32767, 2);
	CHECK(p.process({2000})[0] == 0x7fff);
	const int32_t low = p.process({-1000})[0], mid = p.process({0})[0], high = p.process({1000})[0];
	CHECK(-32767 < low);
	CHECK(low < mid);
	CHECK(mid < high);
}

TEST_CASE_FIXTURE(TargetFixture, "the first autorange sample passes unchanged because the range is still empty") {
	Proc p(1, nullptr);
	REQUIRE(p.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(p.process({1234})[0] == 1234);
}

TEST_CASE_FIXTURE(TargetFixture, "without autorange the range is not learned from the values") {
	Proc p(1, nullptr);
	p.process({100});
	p.process({-100});
	CHECK(p.getAt("min", 0) == NO_MIN);
	CHECK(p.getAt("max", 0) == NO_MAX);
}

TEST_CASE_FIXTURE(TargetFixture, "switching autorange off keeps the learned range") {
	Proc p(1, nullptr);
	REQUIRE(p.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	p.process({-100});
	p.process({100});
	REQUIRE(p.run("autocal", CMDtype::set, 0) == CommandStatus::OK);
	p.process({500});
	CHECK(p.getAt("max", 0) == 100);
	CHECK(p.process({500})[0] == 0x7fff);
}

TEST_CASE_FIXTURE(TargetFixture, "switching autorange on discards the previous range") {
	Proc p(1, nullptr);
	p.setRange(0, -1000, 1000);
	REQUIRE(p.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(p.getAt("min", 0) == NO_MIN);
	CHECK(p.getAt("max", 0) == NO_MAX);
	CHECK(p.getAnalogProcessingConfig().autorange);
}

TEST_CASE_FIXTURE(TargetFixture, "switching autorange off by command keeps a manual range when manual scaling is allowed") {
	Proc p(1, nullptr);
	p.setRange(0, -1000, 1000);
	REQUIRE(p.run("autocal", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(p.getAt("min", 0) == -1000);
	CHECK(p.getAt("max", 0) == 1000);
	CHECK_FALSE(p.getAnalogProcessingConfig().autorange);
}

TEST_CASE_FIXTURE(TargetFixture, "the autocal command reads the autorange setting") {
	Proc p(1, nullptr);
	CHECK(p.get("autocal") == 0);
	p.setAnalogProcessingConfig(AnalogProcessingConfig{.autorange = true});
	CHECK(p.get("autocal") == 1);
}

TEST_CASE_FIXTURE(TargetFixture, "reading the autocal setting keeps the learned range when manual scaling is allowed") {
	Proc p(1, nullptr);
	REQUIRE(p.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	p.process({-100});
	p.process({100});
	p.get("autocal");
	CHECK(p.getAt("min", 0) == -100);
	CHECK(p.getAt("max", 0) == 100);
}

// NEEDS REVIEW 85: without manual scaling (allowManualScale false) every autocal command, also a get and a set to 0,
// resets the learned range (AnalogAxisProcessing.cpp:159: `(set && val != 0) || !allowManualScale`). A GUI that polls the
// setting would keep the range empty. Latent: LocalAnalog and ADS111X allow manual scaling. Intended?
TEST_CASE_FIXTURE(TargetFixture, "characterization: without manual scaling reading the autocal setting resets the learned range") {
	Proc p(1, nullptr, true, true, true, false);
	REQUIRE(p.run("autocal", CMDtype::set, 1) == CommandStatus::OK);
	p.process({-100});
	p.process({100});
	REQUIRE(p.process({100})[0] == 0x7fff);
	CHECK(p.get("autocal") == 1);
	CHECK(p.process({50})[0] == 50); // range empty again: the first new sample passes unchanged
}

TEST_CASE_FIXTURE(TargetFixture, "with autorange allowed but manual scaling not allowed there are no min and max commands") {
	Proc p(1, nullptr, true, true, true, false);
	CHECK(p.registered("autocal"));
	CHECK_FALSE(p.registered("min"));
	CHECK_FALSE(p.registered("max"));
}

TEST_CASE_FIXTURE(TargetFixture, "without autorange and manual scaling an autorange setting has no effect") {
	Proc p(1, nullptr, true, false, true, false);
	p.setAnalogProcessingConfig(AnalogProcessingConfig{.autorange = true});
	p.process({-100});
	p.process({100});
	CHECK(p.process({50})[0] == 50);
}

TEST_CASE_FIXTURE(TargetFixture, "with manual scaling but without autorange there is no autocal command") {
	Proc p(1, nullptr, true, false, true, true);
	CHECK_FALSE(p.registered("autocal"));
	CHECK(p.registered("min"));
}

// ---------------------------------------------------------------- filters

TEST_CASE_FIXTURE(TargetFixture, "the lowpass filter follows a step like a biquad with Fc 0.03 and Q 0.5") {
	Proc p(1, nullptr);
	REQUIRE(p.run("filter", CMDtype::set, 1) == CommandStatus::OK);
	ReferenceLowpass ref;
	for (int i = 0; i < 60; i++) {
		CAPTURE(i);
		int32_t out = p.process({10000})[0];
		CHECK_NEAR(out, ref.process(10000), 2);
	}
}

TEST_CASE_FIXTURE(TargetFixture, "the lowpass filter settles at a constant input") {
	Proc p(1, nullptr);
	REQUIRE(p.run("filter", CMDtype::set, 1) == CommandStatus::OK);
	int32_t out = 0;
	for (int i = 0; i < 400; i++) {
		out = p.process({-12345})[0];
	}
	CHECK_NEAR(out, -12345, 2);
}

TEST_CASE_FIXTURE(TargetFixture, "each axis has its own filter") {
	Proc p(2, nullptr);
	REQUIRE(p.run("filter", CMDtype::set, 1) == CommandStatus::OK);
	std::vector<int32_t> out;
	for (int i = 0; i < 10; i++) {
		out = p.process({10000, 0});
	}
	CHECK(out[0] > 0);
	CHECK(out[1] == 0);
}

TEST_CASE_FIXTURE(TargetFixture, "with the filters disabled the values pass unfiltered") {
	Proc p(1, nullptr);
	REQUIRE(p.run("filter", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(p.process({10000})[0] == 10000);
}

TEST_CASE_FIXTURE(TargetFixture, "the filter command reads the filter setting") {
	Proc p(1, nullptr);
	CHECK(p.get("filter") == 0);
	REQUIRE(p.run("filter", CMDtype::set, 1) == CommandStatus::OK);
	CHECK(p.get("filter") == 1);
}

TEST_CASE_FIXTURE(TargetFixture, "setting the filter command restarts the filters from zero") {
	Proc p(1, nullptr);
	REQUIRE(p.run("filter", CMDtype::set, 1) == CommandStatus::OK);
	for (int i = 0; i < 100; i++) {
		p.process({20000});
	}
	REQUIRE(p.run("filter", CMDtype::set, 1) == CommandStatus::OK);
	ReferenceLowpass ref;
	CHECK_NEAR(p.process({0})[0], ref.process(0), 2);
	CHECK_NEAR(p.process({0})[0], 0, 2);
}

TEST_CASE_FIXTURE(TargetFixture, "filters enabled by the config are not used when filters are not allowed") {
	Proc p(1, nullptr, false, true, true, true);
	p.setAnalogProcessingConfig(AnalogProcessingConfig{.filtersEnabled = true});
	CHECK(p.process({10000})[0] == 10000);
}

TEST_CASE_FIXTURE(TargetFixture, "with filters and autorange the range is learned only after 500 filtered samples") {
	Proc p(1, nullptr);
	p.setAnalogProcessingConfig(AnalogProcessingConfig{.autorange = true, .filtersEnabled = true});
	for (int i = 0; i < 500; i++) {
		p.process({1000});
	}
	CHECK(p.getAt("max", 0) == NO_MAX);
	p.process({1000});
	CHECK_NEAR(p.getAt("max", 0), 1000, 2);
	CHECK_NEAR(p.getAt("min", 0), 1000, 2);
}

TEST_CASE_FIXTURE(TargetFixture, "setting the filter command restarts the wait for 500 filtered samples") {
	Proc p(1, nullptr);
	p.setAnalogProcessingConfig(AnalogProcessingConfig{.autorange = true, .filtersEnabled = true});
	for (int i = 0; i < 400; i++) {
		p.process({1000});
	}
	REQUIRE(p.run("filter", CMDtype::set, 1) == CommandStatus::OK);
	for (int i = 0; i < 400; i++) {
		p.process({1000});
	}
	CHECK(p.getAt("max", 0) == NO_MAX);
}

// NEEDS REVIEW 87: the filter sample counter is shared by all axes and counted once per axis and call
// (AnalogAxisProcessing.cpp:97-99), so with n axes the autorange starts after 500 / n calls, i.e. after less filter
// settling time (2 axes: 251 calls). One counter per call (or per axis) intended?
TEST_CASE_FIXTURE(TargetFixture, "characterization: with two filtered axes the autorange starts after 251 calls") {
	Proc p(2, nullptr);
	p.setAnalogProcessingConfig(AnalogProcessingConfig{.autorange = true, .filtersEnabled = true});
	for (int i = 0; i < 250; i++) {
		p.process({1000, 1000});
	}
	CHECK(p.getAt("max", 0) == NO_MAX);
	CHECK(p.getAt("max", 1) == NO_MAX);
	p.process({1000, 1000});
	CHECK(p.getAt("max", 0) != NO_MAX);
	CHECK(p.getAt("max", 1) != NO_MAX);
}

// ---------------------------------------------------------------- commands

TEST_CASE_FIXTURE(TargetFixture, "the values command replies with the axis values of the source") {
	MockAnalogSource source;
	source.axes = {11, -22, 33};
	Proc p(3, &source);
	REQUIRE(p.run("values", CMDtype::get) == CommandStatus::OK);
	REQUIRE(p.replies.size() == 3);
	CHECK(p.replies[0].val == 11);
	CHECK(p.replies[1].val == -22);
	CHECK(p.replies[2].val == 33);
	CHECK(source.getAxesCalls == 1);
}

TEST_CASE_FIXTURE(TargetFixture, "the values command can not be set") {
	MockAnalogSource source;
	Proc p(1, &source);
	CHECK(p.run("values", CMDtype::set, 1) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(TargetFixture, "the rawval command replies with the unprocessed values of the last processing") {
	Proc p(2, nullptr);
	p.setRange(0, -1000, 1000);
	p.process({500, 40000});
	REQUIRE(p.run("rawval", CMDtype::get) == CommandStatus::OK);
	REQUIRE(p.replies.size() == 2);
	CHECK(p.replies[0].val == 500);
	CHECK(p.replies[1].val == 40000);
}

TEST_CASE_FIXTURE(TargetFixture, "the rawval command replies with nothing before the first processing") {
	Proc p(2, nullptr);
	REQUIRE(p.run("rawval", CMDtype::get) == CommandStatus::OK);
	CHECK(p.replies.empty());
}

TEST_CASE_FIXTURE(TargetFixture, "the rawval command can not be set") {
	Proc p(1, nullptr);
	CHECK(p.run("rawval", CMDtype::set, 1) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(TargetFixture, "the min and max commands set and read the manual range of an axis") {
	Proc p(2, nullptr);
	CHECK(p.run("min", CMDtype::setat, -1234, 1) == CommandStatus::OK);
	CHECK(p.run("max", CMDtype::setat, 4321, 1) == CommandStatus::OK);
	CHECK(p.getAt("min", 1) == -1234);
	CHECK(p.getAt("max", 1) == 4321);
	CHECK(p.getAt("min", 0) == NO_MIN);
}

TEST_CASE_FIXTURE(TargetFixture, "the min and max commands accept the limits plus and minus 0x7fff") {
	Proc p(1, nullptr);
	CHECK(p.run("min", CMDtype::setat, -0x7fff, 0) == CommandStatus::OK);
	CHECK(p.run("max", CMDtype::setat, 0x7fff, 0) == CommandStatus::OK);
	CHECK(p.getAt("min", 0) == -0x7fff);
	CHECK(p.getAt("max", 0) == 0x7fff);
}

TEST_CASE_FIXTURE(TargetFixture, "the min and max commands reject values outside the 16 bit range") {
	Proc p(1, nullptr);
	CHECK(p.run("min", CMDtype::setat, -0x8000, 0) == CommandStatus::ERR);
	CHECK(p.run("max", CMDtype::setat, 0x8000, 0) == CommandStatus::ERR);
	CHECK(p.getAt("min", 0) == NO_MIN);
	CHECK(p.getAt("max", 0) == NO_MAX);
}

TEST_CASE_FIXTURE(TargetFixture, "the min and max commands reject a negative axis") {
	Proc p(1, nullptr);
	CHECK(p.run("min", CMDtype::setat, 1, -1) == CommandStatus::ERR);
	CHECK(p.run("max", CMDtype::getat, 0, -1) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(TargetFixture, "the min and max commands reject a plain get and set without an axis") {
	Proc p(1, nullptr);
	CHECK(p.run("min", CMDtype::get) == CommandStatus::ERR);
	CHECK(p.run("max", CMDtype::set, 5) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(TargetFixture, "the min and max commands reject the axis number equal to the axis amount" * doctest::skip() *
				  doctest::description("KNOWN ISSUE (crash): the address check is `cmd.adr <= axisAmount` "
									   "(AnalogAxisProcessing.cpp:174/187) while minMaxVals has axisAmount entries: "
									   "min/max getat/setat with adr == axisAmount read / write one MinMaxPair past the "
									   "vector (heap out of bounds; aborts the run with _GLIBCXX_ASSERTIONS). Skipped")) {
	Proc p(2, nullptr);
	CHECK(p.run("min", CMDtype::setat, 5, 2) == CommandStatus::ERR);
	CHECK(p.run("max", CMDtype::getat, 0, 2) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(TargetFixture, "the min and max commands reject an axis beyond the axis amount") {
	Proc p(2, nullptr);
	CHECK(p.run("min", CMDtype::setat, 5, 3) == CommandStatus::ERR);
	CHECK(p.run("max", CMDtype::getat, 0, 3) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(TargetFixture, "the autocal and filter commands reject an info request") {
	Proc p(1, nullptr);
	CHECK(p.run("filter", CMDtype::info) == CommandStatus::ERR);
	CHECK(p.run("autocal", CMDtype::info) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(TargetFixture, "an unknown command id is not found by the processing") {
	Proc p(1, nullptr);
	CHECK(p.runId(0x1234, CMDtype::get) == CommandStatus::NOT_FOUND);
}

// ---------------------------------------------------------------- config bits

TEST_CASE("the processing config is encoded as autorange in bit 0 and filters in bit 1") {
	AnalogProcessingConfig conf;
	CHECK(AnalogAxisProcessing::encodeAnalogProcessingConfToInt(conf) == 0);
	conf.autorange = true;
	CHECK(AnalogAxisProcessing::encodeAnalogProcessingConfToInt(conf) == 0b01);
	conf = AnalogProcessingConfig{};
	conf.filtersEnabled = true;
	CHECK(AnalogAxisProcessing::encodeAnalogProcessingConfToInt(conf) == 0b10);
}

TEST_CASE("raw mode is not part of the encoded processing config") {
	AnalogProcessingConfig conf;
	conf.raw = true;
	CHECK(AnalogAxisProcessing::encodeAnalogProcessingConfToInt(conf) == 0);
}

TEST_CASE("every encoded processing config decodes to the same config") {
	for (uint16_t bits = 0; bits < 4; bits++) {
		CAPTURE(bits);
		AnalogProcessingConfig conf = AnalogAxisProcessing::decodeAnalogProcessingConfFromInt(bits);
		CHECK(conf.autorange == ((bits & 1) != 0));
		CHECK(conf.filtersEnabled == ((bits & 2) != 0));
		CHECK_FALSE(conf.raw);
		CHECK(AnalogAxisProcessing::encodeAnalogProcessingConfToInt(conf) == bits);
	}
}

TEST_CASE("decoding the processing config ignores the bits above bit 1") {
	AnalogProcessingConfig conf = AnalogAxisProcessing::decodeAnalogProcessingConfFromInt(0xFFFC);
	CHECK_FALSE(conf.autorange);
	CHECK_FALSE(conf.filtersEnabled);
	CHECK_FALSE(conf.raw);
}

TEST_CASE_FIXTURE(TargetFixture, "a processing config that is set is returned unchanged") {
	Proc p(1, nullptr);
	p.setAnalogProcessingConfig(AnalogProcessingConfig{.autorange = true, .filtersEnabled = false, .raw = true});
	CHECK(p.getAnalogProcessingConfig().autorange);
	CHECK_FALSE(p.getAnalogProcessingConfig().filtersEnabled);
	CHECK(p.getAnalogProcessingConfig().raw);
}

TEST_CASE_FIXTURE(TargetFixture, "a new processing has autorange, filters and raw mode off") {
	Proc p(1, nullptr);
	CHECK_FALSE(p.getAnalogProcessingConfig().autorange);
	CHECK_FALSE(p.getAnalogProcessingConfig().filtersEnabled);
	CHECK_FALSE(p.getAnalogProcessingConfig().raw);
}

// ---------------------------------------------------------------- min / max flash helpers

constexpr std::array<std::pair<uint16_t, uint16_t>, 3> TEST_ADDR = {
	std::pair<uint16_t, uint16_t>(0x700, 0x701), std::pair<uint16_t, uint16_t>(0x702, 0x703), std::pair<uint16_t, uint16_t>(0x704, 0x705)};

TEST_CASE_FIXTURE(TargetFixture, "saving the min and max values writes each axis range as 16 bit values to its address pair") {
	Proc p(2, nullptr);
	p.setRange(0, -1000, 2000);
	p.setRange(1, 5, 0x7fff);
	p.saveMinMaxValues(TEST_ADDR);
	auto &flash = HostPlatform::flash();
	CHECK(flash.at(0x700) == static_cast<uint16_t>(-1000));
	CHECK(flash.at(0x701) == 2000);
	CHECK(flash.at(0x702) == 5);
	CHECK(flash.at(0x703) == 0x7fff);
	CHECK(flash.count(0x704) == 0); // only axisAmount pairs
}

TEST_CASE_FIXTURE(TargetFixture, "saving the min and max values writes only as many pairs as addresses are given") {
	Proc p(4, nullptr);
	p.saveMinMaxValues(TEST_ADDR);
	CHECK(HostPlatform::flash().size() == 6);
}

TEST_CASE_FIXTURE(TargetFixture, "the min and max values are not saved while autorange is on") {
	Proc p(1, nullptr);
	p.setRange(0, -1000, 2000);
	p.setAnalogProcessingConfig(AnalogProcessingConfig{.autorange = true});
	p.saveMinMaxValues(TEST_ADDR);
	CHECK(HostPlatform::flash().empty());
}

TEST_CASE_FIXTURE(TargetFixture, "the min and max values are not saved without manual scaling") {
	Proc p(1, nullptr, true, true, true, false);
	p.saveMinMaxValues(TEST_ADDR);
	CHECK(HostPlatform::flash().empty());
}

TEST_CASE_FIXTURE(TargetFixture, "restoring the min and max values reads them back as signed 16 bit values") {
	auto &flash = HostPlatform::flash();
	flash[0x700] = static_cast<uint16_t>(-1000);
	flash[0x701] = 2000;
	flash[0x702] = 0x8001; // -0x7fff
	flash[0x703] = 0x7fff;
	Proc p(2, nullptr);
	p.restoreMinMaxValues(TEST_ADDR);
	CHECK(p.getAt("min", 0) == -1000);
	CHECK(p.getAt("max", 0) == 2000);
	CHECK(p.getAt("min", 1) == -0x7fff);
	CHECK(p.getAt("max", 1) == 0x7fff);
}

TEST_CASE_FIXTURE(TargetFixture, "an axis whose min or max is missing in flash keeps its range when restoring") {
	auto &flash = HostPlatform::flash();
	flash[0x700] = 10; // max of axis 0 missing
	flash[0x703] = 20; // min of axis 1 missing
	Proc p(2, nullptr);
	p.restoreMinMaxValues(TEST_ADDR);
	CHECK(p.getAt("min", 0) == NO_MIN);
	CHECK(p.getAt("max", 0) == NO_MAX);
	CHECK(p.getAt("min", 1) == NO_MIN);
	CHECK(p.getAt("max", 1) == NO_MAX);
}

TEST_CASE_FIXTURE(TargetFixture, "the min and max values are not restored without manual scaling") {
	auto &flash = HostPlatform::flash();
	flash[0x700] = 10;
	flash[0x701] = 20;
	Proc p(1, nullptr, true, true, true, false);
	p.restoreMinMaxValues(TEST_ADDR);
	CHECK(p.process({50})[0] == 50); // no range: unscaled (a restored range 10..20 would clip it to 0x7fff)
}

TEST_CASE_FIXTURE(TargetFixture, "saved and restored min and max values give the same scaling") {
	Proc a(1, nullptr);
	a.setRange(0, -3000, 1000);
	a.saveMinMaxValues(TEST_ADDR);
	Proc b(1, nullptr);
	b.restoreMinMaxValues(TEST_ADDR);
	CHECK(b.process({-1000}) == a.process({-1000}));
}

} // TEST_SUITE
