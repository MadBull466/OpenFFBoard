/*
 * Tests of FFBHIDMain (FFBHIDMain.cpp, FFBHIDMain_commmands.cpp): the HID gamepad report (axes, analog sources,
 * buttons, double buffer, send conditions), the update rate and the FFB timer (TIM_FFB = htim13), the main thread,
 * emergency stop (command, E-stop pin, errors), USB suspend/resume and every command of the class.
 *
 * The tests use a real FFBWheel (16 bit axes, the F407 configuration) and a local 32 bit variant
 * (HidMain32, FFBHIDMain(1, true)) for the 32 bit report branches. Every test deletes its main class at the end
 * and therefore carries DELETES_FFBHIDMAIN (suspected bug 24, skipped in sanitizer builds).
 *
 * Smoke tests already in test_target_setup.cpp (not repeated here): axis position in the report after the first
 * timer update, button A zeroing the axis.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "FFBJoystick.h"
#include "FFBWheel.h"
#include "eeprom_addresses.h"
#include "usb_descriptors.h"
#include "usb_hid_ffb_desc.h"

#include <cstring>
#include <string>
#include <vector>

extern volatile uint32_t ADC1_BUF[ADC1_CHANNELS]; // host_stubs_target.cpp (global_callbacks.cpp on the target)

namespace {

/** Access to the protected global PersistentStorage startup flag (no public reset), like test_persistent_storage.cpp */
struct HidMainStartupFlag : PersistentStorage {
	static void set(bool v) { startupComplete = v; }
};

/** FFBHIDMain with 32 bit HID axes (HIDAXISRES_USE_32B_DESC is off in the F407 configuration) and 1 axis */
class HidMain32 : public FFBHIDMain {
public:
	static ClassIdentifier info;
	HidMain32() : FFBHIDMain(1, true) { FFBHIDMain::setFFBEffectsCalc(ffbItf, calc); }
	const ClassIdentifier getInfo() override { return info; }
	void usbInit() override {}

private:
	std::shared_ptr<EffectsCalculator> calc = std::make_shared<EffectsCalculator>();
	std::shared_ptr<EffectsControlItf> ffbItf = std::make_shared<HidFFB>(calc, 1);
};
ClassIdentifier HidMain32::info = {.name = "HID main 32 bit (test)", .id = 0x7f01};

struct HidMainFixture : TargetFixture {
	HidMainFixture() { resetInputs(); }
	~HidMainFixture() override { resetInputs(); }

	static void resetInputs() {
		HidMainStartupFlag::set(false);
		for (auto &v : ADC1_BUF) {
			v = 0;
		}
	}

	/** The gamepad reports (report id 1) sent so far. HidFFB also sends PID state reports on the same interface */
	static std::vector<std::vector<uint8_t>> gamepadReports() {
		std::vector<std::vector<uint8_t>> reports;
		for (const HostUsb::HidReport &r : HostUsb::state().hidReports) {
			if (!r.data.empty() && r.data[0] == 1) {
				reports.push_back(r.data);
			}
		}
		return reports;
	}
	static size_t gamepadReportCount() { return gamepadReports().size(); }

	/** Axis slot `i` (0 = X ... 7 = Slider) of a 16 bit report: id, 64 bit buttons, 8 x int16 */
	static int16_t axis16(const std::vector<uint8_t> &report, int i) {
		int16_t v = 0;
		REQUIRE(report.size() == 25);
		std::memcpy(&v, &report[9 + 2 * i], sizeof(v));
		return v;
	}
	static uint64_t buttons(const std::vector<uint8_t> &report) {
		uint64_t b = 0;
		REQUIRE(report.size() >= 9);
		std::memcpy(&b, &report[1], sizeof(b));
		return b;
	}

	/** Axis of the main class (created by its AxesManager) */
	static Axis &mainAxis(uint8_t instance = 0) {
		Axis *axis = dynamic_cast<Axis *>(CommandHandler::getHandlerFromClassName("axis", instance));
		REQUIRE(axis != nullptr);
		return *axis;
	}

	/** Selects MockMotorDriver and MockEncoder on an axis of the main class. Returns the encoder */
	static MockEncoder &selectMocks(uint8_t instance = 0) {
		Axis &axis = mainAxis(instance);
		axis.driverChooser = ClassChooser<MotorDriver>(AxisFixture::driverRegistry());
		axis.encoderChooser = ClassChooser<Encoder>(AxisFixture::encoderRegistry());
		axis.setDrvType(MockMotorDriver::SELECTION_ID);
		axis.setEncType(MockEncoder::SELECTION_ID);
		MockEncoder *encoder = dynamic_cast<MockEncoder *>(axis.getEncoder());
		REQUIRE(encoder != nullptr);
		return *encoder;
	}
	static MockMotorDriver &driverOf(uint8_t instance = 0) {
		MockMotorDriver *drv = dynamic_cast<MockMotorDriver *>(mainAxis(instance).getDriver());
		REQUIRE(drv != nullptr);
		return *drv;
	}

	/** Starts the main thread: Run() starts TIM_FFB and waits for the first update */
	template <class T>
	static void start(T &main) {
		HostRtos::runUntilBlocked(main);
		REQUIRE(HostTarget::timerRunning(&htim13));
	}

	/** One FFB cycle: 1 ms, TIM_FFB update interrupt, main thread runs updateControl() once */
	template <class T>
	static void tick(T &main, uint32_t cycles = 1) {
		for (uint32_t i = 0; i < cycles; i++) {
			HostPlatform::advanceMs(1);
			REQUIRE(HostTarget::timerUpdate(&htim13));
			HostRtos::runUntilBlocked(main);
		}
	}

	/**
	 * Full scale torque on the main class axis (power 0x7fff, effect ratio 255) and a constant force of `magnitude`
	 * started by the PID host. The torque then changes in every cycle of the 1 s fade-in.
	 */
	static void startConstantForce(PidHost &host, int16_t magnitude = 10000) {
		REQUIRE(cmd(mainAxis(), "power", CMDtype::set, 0x7fff) == CommandStatus::OK);
		REQUIRE(cmd(mainAxis(), "fxratio", CMDtype::set, 255) == CommandStatus::OK);
		host.deviceControl(0x01);
		uint8_t cf = host.allocateEffect(FFB_EFFECT_CONSTANT);
		host.setEffect(cf, FFB_EFFECT_CONSTANT);
		host.startEffect(cf);
		host.setConstantForce(cf, magnitude);
	}

	/** Executes a command of `handler` by its name like the command thread does */
	static CommandStatus cmd(CommandHandler &handler, const std::string &name, CMDtype type, int64_t val = 0,
							 std::vector<CommandReply> *replies = nullptr, int64_t adr = 0) {
		CmdHandlerCommanddef *def = handler.getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		ParsedCommand parsed;
		parsed.cmdId = def->cmdId;
		parsed.type = type;
		parsed.val = val;
		parsed.adr = adr;
		parsed.target = &handler;
		std::vector<CommandReply> local;
		return handler.command(parsed, replies ? *replies : local);
	}

	/** Reads a value with a get command. Fails the test unless there is exactly one reply */
	static int64_t get(CommandHandler &handler, const std::string &name) {
		std::vector<CommandReply> replies;
		CHECK(cmd(handler, name, CMDtype::get, 0, &replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}
};

} // namespace

TEST_SUITE("FFBHIDMain") {

// ---------------- HID gamepad report ----------------

TEST_CASE_FIXTURE(HidMainFixture, "the gamepad report is 25 bytes with report id 1, 64 buttons and 8 int16 axes" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks().setDegrees(45.0f);
	start(wheel);

	tick(wheel);

	auto reports = gamepadReports();
	REQUIRE(reports.size() == 1);
	CHECK(reports[0].size() == 25);
	CHECK(reports[0][0] == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "the X axis of the gamepad report is the scaled axis position shifted to 16 bit" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	MockEncoder &enc = selectMocks();
	enc.setDegrees(-180.0f); // 180 of 900 degrees, inverted by default: +0.4 of the range
	start(wheel);

	tick(wheel);

	REQUIRE(gamepadReportCount() == 1);
	int16_t x = axis16(gamepadReports().back(), 0);
	CHECK(x == static_cast<int16_t>(mainAxis().getLastScaledEnc() >> 16));
	CHECK_NEAR(x, 0.4 * 32767, 2);
}

TEST_CASE_FIXTURE(HidMainFixture, "the buttons of the local button source are reported in the gamepad report" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	REQUIRE(cmd(wheel, "btntypes", CMDtype::set, 1) == CommandStatus::OK); // bit 0: LocalButtons ("D-Pins")
	start(wheel);

	HostTarget::setPin(DIN0_GPIO_Port, DIN0_Pin, false); // DIN0 pressed (low active by default)
	HostTarget::setPin(DIN2_GPIO_Port, DIN2_Pin, false); // DIN2 pressed
	tick(wheel);

	REQUIRE(gamepadReportCount() == 1);
	CHECK(buttons(gamepadReports().back()) == 0b101);
}

TEST_CASE_FIXTURE(HidMainFixture, "without a button source no button is reported although pins are pressed" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks().setDegrees(10.0f); // Makes the report differ from the empty one
	start(wheel);
	HostTarget::setPin(DIN0_GPIO_Port, DIN0_Pin, false);

	tick(wheel);

	REQUIRE(gamepadReportCount() == 1);
	CHECK(buttons(gamepadReports().back()) == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "the local analog inputs fill the report slots after the encoder axis" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks().setDegrees(90.0f);
	REQUIRE(cmd(wheel, "aintypes", CMDtype::set, 1) == CommandStatus::OK); // bit 0: LocalAnalog
	// 12 bit ADC: value = (adc << 4) - 0x7fff for the default configuration (no autorange, no filter)
	for (int i = 0; i < ADC_PINS; i++) {
		ADC1_BUF[i] = 0x100 * (i + 1);
	}
	start(wheel);

	tick(wheel);

	REQUIRE(gamepadReportCount() == 1);
	const auto report = gamepadReports().back();
	CHECK(axis16(report, 0) == -6554); // the encoder axis stays in X
	for (int i = 0; i < ADC_PINS; i++) {
		CAPTURE(i);
		CHECK(axis16(report, i + 1) == ((0x100 * (i + 1)) << 4) - 0x7fff);
	}
	CHECK(axis16(report, 7) == 0); // 1 axis + 6 analog pins: the slider stays 0
}

TEST_CASE_FIXTURE(HidMainFixture, "analog values beyond the 8 report slots are dropped" * DELETES_FFBHIDMAIN) {
	FFBJoystick joystick; // 2 encoder axes + 6 local analog pins fill all 8 slots, the CAN analog values are dropped
	selectMocks(0).setDegrees(90.0f);
	selectMocks(1).setDegrees(-90.0f);
	REQUIRE(cmd(joystick, "aintypes", CMDtype::set, 0b11) == CommandStatus::OK); // LocalAnalog + CanAnalog (MockCANPort)
	for (int i = 0; i < ADC_PINS; i++) {
		ADC1_BUF[i] = 0x100 * (i + 1);
	}
	start(joystick);

	tick(joystick);

	REQUIRE(gamepadReportCount() == 1);
	const auto report = gamepadReports().back();
	CHECK(axis16(report, 0) == -6554);
	CHECK(axis16(report, 1) == 6553);
	for (int i = 0; i < ADC_PINS; i++) {
		CAPTURE(i);
		CHECK(axis16(report, i + 2) == ((0x100 * (i + 1)) << 4) - 0x7fff);
	}
}

TEST_CASE_FIXTURE(HidMainFixture, "characterization: without an encoder the analog inputs start at the X slot of the report" * DELETES_FFBHIDMAIN) {
	// NEEDS REVIEW: AxesManager::getAxisValues skips axes without an encoder, so the analog inputs move into the slots
	// of the FFB axes. A host game then sees an analog input as the steering axis. Intended?
	FFBWheel wheel; // default axis: no encoder selected
	REQUIRE(cmd(wheel, "aintypes", CMDtype::set, 1) == CommandStatus::OK);
	ADC1_BUF[0] = 0xC00;
	start(wheel);

	tick(wheel);

	REQUIRE(gamepadReportCount() == 1);
	CHECK(axis16(gamepadReports().back(), 0) == (0xC00 << 4) - 0x7fff);
}

TEST_CASE_FIXTURE(HidMainFixture, "slots of a removed analog source are reported as 0 again" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks().setDegrees(90.0f);
	REQUIRE(cmd(wheel, "aintypes", CMDtype::set, 1) == CommandStatus::OK);
	for (int i = 0; i < ADC_PINS; i++) {
		ADC1_BUF[i] = 0xC00;
	}
	start(wheel);
	tick(wheel, 2); // fill both report buffers with the analog values

	REQUIRE(cmd(wheel, "aintypes", CMDtype::set, 0) == CommandStatus::OK);
	tick(wheel);

	const auto reports = gamepadReports();
	REQUIRE(reports.size() == 2); // the first one, then the one without analog values (the second tick was unchanged)
	for (int i = 1; i < 8; i++) {
		CAPTURE(i);
		CHECK(axis16(reports.back(), i) == 0);
	}
	CHECK(axis16(reports.back(), 0) == -6554);
}

TEST_CASE_FIXTURE(HidMainFixture, "a 32 bit main class reports the axis unshifted and analog values in the 32 bit slot shifted up by 16 bit" * DELETES_FFBHIDMAIN) {
	HidMain32 main;
	selectMocks().setDegrees(90.0f);
	REQUIRE(cmd(main, "aintypes", CMDtype::set, 1) == CommandStatus::OK);
	ADC1_BUF[0] = 0xC00; // -> Y (int32 slot, shifted)
	ADC1_BUF[1] = 0x400; // -> Z (int16 slot, not shifted)
	start(main);

	tick(main);

	const auto reports = gamepadReports();
	REQUIRE(reports.size() == 1);
	const auto &r = reports.back();
	REQUIRE(r.size() == 29); // id, 64 buttons, X/Y int32, Z..Slider 6 x int16 (MAX_AXIS 2)
	int32_t x = 0, y = 0;
	int16_t z = 0;
	std::memcpy(&x, &r[9], 4);
	std::memcpy(&y, &r[13], 4);
	std::memcpy(&z, &r[17], 2);
	CHECK(x == mainAxis().getLastScaledEnc());
	CHECK_NEAR(x, -0.2 * 0x7fffffff, 256);
	CHECK(y == static_cast<int32_t>(static_cast<uint32_t>((0xC00 << 4) - 0x7fff) << 16));
	CHECK(z == (0x400 << 4) - 0x7fff);
}

// ---------------- Send conditions and double buffer ----------------

TEST_CASE_FIXTURE(HidMainFixture, "an unchanged all-zero report is not sent" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel; // no encoder, no sources: the report equals the initial (empty) buffer
	start(wheel);

	tick(wheel, 10);

	CHECK(gamepadReportCount() == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "an unchanged report is sent again after 100 cycles without a report" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks().setDegrees(90.0f);
	start(wheel);
	tick(wheel, 2); // first report sent, second identical: both buffers hold the same data
	REQUIRE(gamepadReportCount() == 1);

	tick(wheel, 99); // 100 cycles after the first report
	CHECK(gamepadReportCount() == 1);
	tick(wheel);
	CHECK(gamepadReportCount() == 2);
}

TEST_CASE_FIXTURE(HidMainFixture, "every change of the axis position is sent in the next cycle" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	MockEncoder &enc = selectMocks();
	start(wheel);
	std::vector<int16_t> sent;

	for (float deg : {10.0f, 20.0f, 20.0f, 30.0f, 10.0f}) {
		enc.setDegrees(deg);
		tick(wheel);
	}

	for (const auto &r : gamepadReports()) {
		sent.push_back(axis16(r, 0));
	}
	REQUIRE(sent.size() == 4); // 20 deg was not sent twice
	CHECK(sent[0] < 0);
	CHECK(sent[1] < sent[0]);
	CHECK(sent[2] < sent[1]);
	CHECK(sent[3] == sent[0]); // back to 10 deg: differs from the last report although equal to the older buffer
}

TEST_CASE_FIXTURE(HidMainFixture, "no gamepad report is sent while the HID endpoint is busy" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks().setDegrees(90.0f);
	start(wheel);
	HostUsb::state().hidReady = false;

	tick(wheel, 5);
	CHECK(gamepadReportCount() == 0);

	HostUsb::state().hidReady = true;
	tick(wheel);
	CHECK(gamepadReportCount() == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "pending HID command replies delay the gamepad report by two cycles" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks().setDegrees(90.0f);
	start(wheel);
	// A reply for the HID command interface of the main class (queued, its thread is not run by the test)
	REQUIRE(HID_CommandInterface::globalInterface != nullptr);
	CommandResult result;
	result.type = CommandStatus::OK;
	result.originalCommand.type = CMDtype::get;
	result.originalCommand.target = &wheel;
	result.reply.emplace_back(42);
	HID_CommandInterface::globalInterface->sendReplies({result}, HID_CommandInterface::globalInterface);
	REQUIRE(HID_CommandInterface::globalInterface->waitingToSend());

	tick(wheel, 2);
	CHECK(gamepadReportCount() == 0);
	tick(wheel);
	CHECK(gamepadReportCount() == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "characterization: a report rejected by the USB stack is not sent again while unchanged" * DELETES_FFBHIDMAIN) {
	// NEEDS REVIEW: send_report ignores the result of tud_hid_report and swaps the buffers anyway. The rejected report
	// counts as sent: the host gets the position only after the next change or the 100 cycle timeout. tud_hid_n_ready
	// is checked before, so a rejection should be rare. Intended?
	FFBWheel wheel;
	selectMocks().setDegrees(90.0f);
	start(wheel);
	HostUsb::state().hidReportResult = false;
	tick(wheel);
	HostUsb::state().hidReportResult = true;

	tick(wheel, 5);

	CHECK(gamepadReportCount() == 0);
}

// ---------------- Update rate, FFB timer and main thread ----------------

TEST_CASE_FIXTURE(HidMainFixture, "the main thread starts the FFB timer with its update interrupt" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	CHECK_FALSE(HostTarget::timerRunning(&htim13));

	HostRtos::runUntilBlocked(wheel);

	CHECK(HostTarget::timerRunning(&htim13));
	CHECK(HostTarget::timerUpdateIrqEnabled(&htim13));
}

TEST_CASE_FIXTURE(HidMainFixture, "an update of another timer does not wake the main thread" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	start(wheel);
	{
		HostPlatform::IsrContext isr;
		HAL_TIM_PeriodElapsedCallback(&htim3);
	}
	CHECK(HostRtos::pendingNotifications(wheel) == 0);
	REQUIRE(HostTarget::timerUpdate(&htim13));
	CHECK(HostRtos::pendingNotifications(wheel) == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "each FFB timer update runs one axis update with a new torque" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;
	selectMocks();
	MockMotorDriver &drv = driverOf();
	startConstantForce(host);
	start(wheel);
	tick(wheel, 2); // the fade-in starts at 0
	size_t before = drv.torqueHistory.size();

	tick(wheel, 3);

	REQUIRE(drv.torqueHistory.size() == before + 3); // fade-in: a new torque in every cycle
	CHECK(std::abs(drv.torqueHistory[before + 2]) > std::abs(drv.torqueHistory[before]));
}

TEST_CASE_FIXTURE(HidMainFixture, "without a timer update the main thread does not update the axes" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;
	selectMocks();
	MockMotorDriver &drv = driverOf();
	startConstantForce(host);
	start(wheel);
	tick(wheel, 2);
	size_t before = drv.torqueHistory.size();
	REQUIRE(before > 0);

	HostPlatform::advanceMs(5);
	HostRtos::runUntilBlocked(wheel);

	CHECK(drv.torqueHistory.size() == before);
}

TEST_CASE_FIXTURE(HidMainFixture, "restoring from empty flash selects the 1 kHz rate with a 1000 us timer period" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;

	wheel.restoreFlash();

	CHECK(htim13.Instance->ARR == 1000);
	CHECK(get(wheel, "hidsendspd") == 0);
	CHECK(wheel.getCurFFBFreq() == doctest::Approx(1000.0f));
	CHECK(get(wheel, "btntypes") == 0);
	CHECK(get(wheel, "aintypes") == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "each hidsendspd index sets the FFB timer period and frequency of its rate" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	const uint32_t arr[] = {1000, 2000, 4000, 8000};
	const float freq[] = {1000, 500, 250, 125};

	for (int idx = 0; idx < 4; idx++) {
		CAPTURE(idx);
		REQUIRE(cmd(wheel, "hidsendspd", CMDtype::set, idx) == CommandStatus::OK);
		CHECK(get(wheel, "hidsendspd") == idx);
		CHECK(htim13.Instance->ARR == arr[idx]);
		CHECK(wheel.getCurFFBFreq() == doctest::Approx(freq[idx]));
	}
}

TEST_CASE_FIXTURE(HidMainFixture, "the rate does not depend on the USB speed in a full speed build" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	HostTarget::state().usbSpeed = TUSB_SPEED_HIGH;

	REQUIRE(cmd(wheel, "hidsendspd", CMDtype::set, 0) == CommandStatus::OK);

	CHECK(htim13.Instance->ARR == 1000);
	selectMocks().setDegrees(90.0f);
	start(wheel);
	tick(wheel);
	CHECK(gamepadReportCount() == 1); // a report in every cycle at 1 kHz
}

TEST_CASE_FIXTURE(HidMainFixture, "at the lowest rate a gamepad report is still sent in every FFB cycle" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks();
	REQUIRE(cmd(wheel, "hidsendspd", CMDtype::set, 3) == CommandStatus::OK); // 125 Hz
	start(wheel);
	// usb divider 1 for every full speed rate
	MockEncoder &enc = selectMocks();
	enc.setDegrees(10.0f);
	tick(wheel);
	enc.setDegrees(20.0f);
	tick(wheel);
	CHECK(gamepadReportCount() == 2);
}

TEST_CASE_FIXTURE(HidMainFixture, "hidsendspd info lists the FFB and USB rate of every index" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	std::vector<CommandReply> replies;

	CHECK(cmd(wheel, "hidsendspd", CMDtype::info, 0, &replies) == CommandStatus::OK);

	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "FFB 1000Hz\nUSB 1000Hz:0,FFB 500Hz\nUSB 500Hz:1,FFB 250Hz\nUSB 250Hz:2,FFB 125Hz\nUSB 125Hz:3");
}

TEST_CASE_FIXTURE(HidMainFixture, "hidsendspd with an index beyond the last rate selects the last rate" *
		doctest::description("KNOWN ISSUE (crash): FFBHIDMain::setReportRate (FFBHIDMain.cpp:250) clips the index to "
							 "dividers.size() instead of size()-1 and reads ffbrates.dividers[4] of a 4 element std::array "
							 "(lines 256, 260, 262). Out of bounds: aborts the run with _GLIBCXX_ASSERTIONS (GCC 16 -O0). "
							 "Skipped like known issue 3") *
		doctest::skip() * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;

	CHECK(cmd(wheel, "hidsendspd", CMDtype::set, 4) == CommandStatus::OK);

	CHECK(get(wheel, "hidsendspd") == 3);
	CHECK(htim13.Instance->ARR == 8000);
}

// ---------------- Flash ----------------

TEST_CASE_FIXTURE(HidMainFixture, "saveFlash stores the button sources, analog sources and rate index" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	REQUIRE(cmd(wheel, "btntypes", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd(wheel, "aintypes", CMDtype::set, 1) == CommandStatus::OK);
	REQUIRE(cmd(wheel, "hidsendspd", CMDtype::set, 2) == CommandStatus::OK);

	wheel.saveFlash();

	CHECK(HostPlatform::flash()[ADR_FFBWHEEL_BUTTONCONF] == 1);
	CHECK(HostPlatform::flash()[ADR_FFBWHEEL_ANALOGCONF] == 1);
	CHECK(HostPlatform::flash()[ADR_FFBWHEEL_CONF1] == 2);
}

TEST_CASE_FIXTURE(HidMainFixture, "restoreFlash applies the stored sources and rate index" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	HostPlatform::flash()[ADR_FFBWHEEL_BUTTONCONF] = 1;
	HostPlatform::flash()[ADR_FFBWHEEL_ANALOGCONF] = 1;
	HostPlatform::flash()[ADR_FFBWHEEL_CONF1] = 0x0002 | 0xfc; // only the 2 low bits are the rate index

	wheel.restoreFlash();

	CHECK(get(wheel, "btntypes") == 1);
	CHECK(get(wheel, "aintypes") == 1);
	CHECK(get(wheel, "hidsendspd") == 2);
	CHECK(htim13.Instance->ARR == 4000);
	CHECK(CommandHandler::getHandlersFromClassName("dpin").size() == 1); // LocalButtons created
	CHECK(CommandHandler::getHandlersFromClassName("apin").size() == 1); // LocalAnalog created
}

// ---------------- Commands ----------------

TEST_CASE_FIXTURE(HidMainFixture, "the main class registers its commands with the documented flags" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	struct Expected {
		const char *name;
		uint32_t flags;
	};
	const Expected expected[] = {
		{"ffbactive", CMDFLAG_GET},
		{"btntypes", CMDFLAG_GET | CMDFLAG_SET},
		{"addbtn", CMDFLAG_SET},
		{"lsbtn", CMDFLAG_GET | CMDFLAG_STR_ONLY},
		{"aintypes", CMDFLAG_GET | CMDFLAG_SET},
		{"lsain", CMDFLAG_GET | CMDFLAG_STR_ONLY},
		{"addain", CMDFLAG_SET},
		{"hidrate", CMDFLAG_GET},
		{"cfrate", CMDFLAG_GET},
		{"hidsendspd", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING},
		{"estop", CMDFLAG_GET | CMDFLAG_SET},
	};
	for (const Expected &e : expected) {
		CAPTURE(e.name);
		CmdHandlerCommanddef *def = wheel.getCommandFromName(e.name);
		REQUIRE(def != nullptr);
		CHECK(def->flags == e.flags);
	}
	CHECK(wheel.getCommandFromName("axes") == nullptr); // commented out in registerCommands
}

TEST_CASE_FIXTURE(HidMainFixture, "an unknown command id is not found" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	std::vector<CommandReply> replies;
	ParsedCommand parsed;
	parsed.type = CMDtype::get;
	parsed.target = &wheel;

	parsed.cmdId = 1; // FFBWheel_commands::axes: dummy, not registered
	CHECK(wheel.command(parsed, replies) == CommandStatus::NOT_FOUND);
	parsed.cmdId = 0x1234;
	CHECK(wheel.command(parsed, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(HidMainFixture, "ffbactive reports 0 before the host enables FFB and 1 after" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;

	CHECK(get(wheel, "ffbactive") == 0);
	host.deviceControl(0x01); // PID device control: enable actuators

	CHECK(get(wheel, "ffbactive") == 1);
	CHECK(wheel.getFfbActive());
}

TEST_CASE_FIXTURE(HidMainFixture, "ffbactive reports -1 during an emergency stop" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;
	host.deviceControl(0x01);

	REQUIRE(cmd(wheel, "estop", CMDtype::set, 1) == CommandStatus::OK);

	CHECK(get(wheel, "ffbactive") == -1);
}

TEST_CASE_FIXTURE(HidMainFixture, "ffbactive cannot be set" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	CHECK(cmd(wheel, "ffbactive", CMDtype::set, 1) == CommandStatus::ERR);
	CHECK(get(wheel, "ffbactive") == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "estop set 1 stops the motor and estop get reports it" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks();
	MockMotorDriver &drv = driverOf();

	REQUIRE(cmd(wheel, "estop", CMDtype::set, 1) == CommandStatus::OK);

	CHECK(get(wheel, "estop") == 1);
	CHECK(drv.emergency);
	CHECK(drv.lastTorque == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "estop set 0 releases the emergency stop" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks();
	MockMotorDriver &drv = driverOf();
	REQUIRE(cmd(wheel, "estop", CMDtype::set, 1) == CommandStatus::OK);

	REQUIRE(cmd(wheel, "estop", CMDtype::set, 0) == CommandStatus::OK);

	CHECK(get(wheel, "estop") == 0);
	CHECK_FALSE(drv.emergency);
}

TEST_CASE_FIXTURE(HidMainFixture, "estop with another type than get or set changes nothing" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	std::vector<CommandReply> replies;
	CHECK(cmd(wheel, "estop", CMDtype::info, 1, &replies) == CommandStatus::OK);
	CHECK(replies.empty());
	CHECK(get(wheel, "estop") == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "during an emergency stop the cycle sends no torque and pulses the clip LED" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;
	selectMocks().setDegrees(20.0f);
	MockMotorDriver &drv = driverOf();
	startConstantForce(host);
	start(wheel);
	tick(wheel, 2);
	REQUIRE(cmd(wheel, "estop", CMDtype::set, 1) == CommandStatus::OK);
	size_t torques = drv.torqueHistory.size();
	uint32_t pulses = HostPlatform::leds().clipPulses;

	tick(wheel, 3);

	CHECK(drv.torqueHistory.size() == torques);
	CHECK(HostPlatform::leds().clipPulses > pulses);
	CHECK(gamepadReportCount() == 1); // reports continue
}

TEST_CASE_FIXTURE(HidMainFixture, "btntypes set creates the selected button sources and get returns the mask" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	CHECK(get(wheel, "btntypes") == 0);

	REQUIRE(cmd(wheel, "btntypes", CMDtype::set, 1) == CommandStatus::OK);

	CHECK(get(wheel, "btntypes") == 1);
	CHECK(CommandHandler::getHandlersFromClassName("dpin").size() == 1);
	REQUIRE(cmd(wheel, "btntypes", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(CommandHandler::getHandlersFromClassName("dpin").empty());
}

TEST_CASE_FIXTURE(HidMainFixture, "addbtn adds a button source whose buttons are reported" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	start(wheel);

	CHECK(cmd(wheel, "addbtn", CMDtype::set, 0) == CommandStatus::OK);
	HostTarget::setPin(DIN1_GPIO_Port, DIN1_Pin, false);
	tick(wheel);

	CHECK(CommandHandler::getHandlersFromClassName("dpin").size() == 1);
	REQUIRE(gamepadReportCount() == 1);
	CHECK(buttons(gamepadReports().back()) == 0b10);
}

TEST_CASE_FIXTURE(HidMainFixture, "addbtn adds a button source once" *
		KNOWN_ISSUE("SelectableInputs::addBtnType (SelectableInputs.cpp:44-46) compares the class id of the existing "
					"sources (getInfo().id, LocalButtons 0x21) with the selection id (0): the duplicate check never "
					"matches and a second LocalButtons is created. The CHECK on the handler count fails") *
		DELETES_FFBHIDMAIN) {
	FFBWheel wheel;

	CHECK(cmd(wheel, "addbtn", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(cmd(wheel, "addbtn", CMDtype::set, 0) == CommandStatus::OK);

	CHECK(CommandHandler::getHandlersFromClassName("dpin").size() == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "addbtn with a get request adds nothing" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	CHECK(cmd(wheel, "addbtn", CMDtype::get, 0) == CommandStatus::OK);
	CHECK(CommandHandler::getHandlersFromClassName("dpin").empty());
}

TEST_CASE_FIXTURE(HidMainFixture, "characterization: addbtn does not change the btntypes mask that is saved to flash" * DELETES_FFBHIDMAIN) {
	// NEEDS REVIEW: SelectableInputs::addBtnType/addAinType create the source without setting its bit in
	// btnsources/ainsources. saveFlash then stores the old mask: the added source is gone after a restart. Intended?
	FFBWheel wheel;
	REQUIRE(cmd(wheel, "addbtn", CMDtype::set, 0) == CommandStatus::OK);

	CHECK(get(wheel, "btntypes") == 0);
	wheel.saveFlash();
	CHECK(HostPlatform::flash()[ADR_FFBWHEEL_BUTTONCONF] == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "aintypes set creates the selected analog sources and get returns the mask" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	CHECK(get(wheel, "aintypes") == 0);

	REQUIRE(cmd(wheel, "aintypes", CMDtype::set, 1) == CommandStatus::OK);

	CHECK(get(wheel, "aintypes") == 1);
	CHECK(CommandHandler::getHandlersFromClassName("apin").size() == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "addain adds an analog source" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;

	CHECK(cmd(wheel, "addain", CMDtype::set, 0) == CommandStatus::OK);

	CHECK(CommandHandler::getHandlersFromClassName("apin").size() == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "addain with a get request adds nothing" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	CHECK(cmd(wheel, "addain", CMDtype::get, 0) == CommandStatus::OK);
	CHECK(CommandHandler::getHandlersFromClassName("apin").empty());
}

TEST_CASE_FIXTURE(HidMainFixture, "addain adds an analog source once" *
		KNOWN_ISSUE("SelectableInputs::addAinType (SelectableInputs.cpp:76-78) compares the class id of the existing "
					"sources (getInfo().id, LocalAnalog 0x41) with the selection id (0): the duplicate check never "
					"matches and a second LocalAnalog is created. The CHECK on the handler count fails") *
		DELETES_FFBHIDMAIN) {
	FFBWheel wheel;

	CHECK(cmd(wheel, "addain", CMDtype::set, 0) == CommandStatus::OK);
	CHECK(cmd(wheel, "addain", CMDtype::set, 0) == CommandStatus::OK);

	CHECK(CommandHandler::getHandlersFromClassName("apin").size() == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "lsbtn lists every button source with id, creatable flag and name" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	std::vector<CommandReply> replies;

	CHECK(cmd(wheel, "lsbtn", CMDtype::get, 0, &replies) == CommandStatus::OK);

	std::vector<std::string> names;
	for (const CommandReply &r : replies) {
		names.push_back(r.reply);
	}
	REQUIRE(names.size() == ButtonSource::all_buttonsources.size());
	CHECK(names.front() == "0:1:D-Pins");
}

TEST_CASE_FIXTURE(HidMainFixture, "lsain lists every analog source with id, creatable flag and name" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	std::vector<CommandReply> replies;

	CHECK(cmd(wheel, "lsain", CMDtype::get, 0, &replies) == CommandStatus::OK);

	REQUIRE(replies.size() == AnalogSource::all_analogsources.size());
	CHECK(replies.front().reply.rfind("0:1:", 0) == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "hidrate reports the rate of the PID output reports of the host" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;
	CHECK(get(wheel, "hidrate") == 0);

	for (int i = 0; i < 20; i++) {
		HostPlatform::advanceMs(2); // 500 Hz
		host.deviceGain(255);
	}

	CHECK_NEAR(get(wheel, "hidrate"), 500, 5);
	CHECK(wheel.getRate() == get(wheel, "hidrate"));
}

TEST_CASE_FIXTURE(HidMainFixture, "cfrate reports the rate of the constant force updates" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;
	uint8_t cf = host.allocateEffect(FFB_EFFECT_CONSTANT);
	host.setEffect(cf, FFB_EFFECT_CONSTANT);
	CHECK(get(wheel, "cfrate") == 0);

	for (int i = 0; i < 20; i++) {
		HostPlatform::advanceMs(4); // 250 Hz
		host.setConstantForce(cf, 1000);
	}

	CHECK_NEAR(get(wheel, "cfrate"), 250, 3);
}

TEST_CASE_FIXTURE(HidMainFixture, "hidrate and cfrate cannot be set" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	CHECK(cmd(wheel, "hidrate", CMDtype::set, 1) == CommandStatus::ERR);
	CHECK(cmd(wheel, "cfrate", CMDtype::set, 1) == CommandStatus::ERR);
}

TEST_CASE_FIXTURE(HidMainFixture, "the help string describes a force feedback game controller" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	CHECK(wheel.getHelpstring() == "Force feedback HID game controller");
}

// ---------------- E-stop pin, errors ----------------

TEST_CASE_FIXTURE(HidMainFixture, "an E-stop pressed at startup puts the main class into emergency stop" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	HostTarget::setPin(E_STOP_GPIO_Port, E_STOP_Pin, false); // low active

	HostRtos::runUntilBlocked(wheel);

	CHECK(get(wheel, "estop") == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "a released E-stop at startup leaves the main class running" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	HostRtos::runUntilBlocked(wheel);
	CHECK(get(wheel, "estop") == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "pressing the E-stop triggers an emergency stop through its interrupt" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks();
	start(wheel);
	HostPlatform::advanceMs(1001); // 1 s debounce after startup

	HostTarget::setPin(E_STOP_GPIO_Port, E_STOP_Pin, false);
	HostTarget::triggerExti(E_STOP_Pin);

	CHECK(get(wheel, "estop") == 1);
	CHECK(driverOf().emergency);
}

TEST_CASE_FIXTURE(HidMainFixture, "releasing the E-stop more than 1 s later resets the emergency stop" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	start(wheel);
	HostPlatform::advanceMs(1001);
	HostTarget::setPin(E_STOP_GPIO_Port, E_STOP_Pin, false);
	HostTarget::triggerExti(E_STOP_Pin);
	REQUIRE(get(wheel, "estop") == 1);

	HostPlatform::advanceMs(1001);
	HostTarget::setPin(E_STOP_GPIO_Port, E_STOP_Pin, true);
	HostTarget::triggerExti(E_STOP_Pin);

	CHECK(get(wheel, "estop") == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "E-stop pin changes within 1 s of the last one are ignored" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	start(wheel);
	HostPlatform::advanceMs(1001);
	HostTarget::setPin(E_STOP_GPIO_Port, E_STOP_Pin, false);
	HostTarget::triggerExti(E_STOP_Pin);
	REQUIRE(get(wheel, "estop") == 1);

	HostPlatform::advanceMs(1000); // not more than 1000 ms
	HostTarget::setPin(E_STOP_GPIO_Port, E_STOP_Pin, true);
	HostTarget::triggerExti(E_STOP_Pin);

	CHECK(get(wheel, "estop") == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "an E-stop interrupt without a level change does nothing" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks();
	start(wheel);
	HostPlatform::advanceMs(1001);

	HostTarget::triggerExti(E_STOP_Pin); // pin still released

	CHECK(get(wheel, "estop") == 0);
	CHECK(driverOf().emergencyStopCalls == 0); // no reset of an emergency stop that is not active
}

TEST_CASE_FIXTURE(HidMainFixture, "a second E-stop interrupt while the E-stop stays pressed does not stop again" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks();
	start(wheel);
	HostPlatform::advanceMs(1001);
	HostTarget::setPin(E_STOP_GPIO_Port, E_STOP_Pin, false);
	HostTarget::triggerExti(E_STOP_Pin);
	REQUIRE(driverOf().emergencyStopCalls == 1);

	HostPlatform::advanceMs(1001);
	HostTarget::triggerExti(E_STOP_Pin); // still pressed

	CHECK(driverOf().emergencyStopCalls == 1);
	CHECK(get(wheel, "estop") == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "button A released does not zero the axis" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	MockEncoder &enc = selectMocks();
	enc.setDegrees(30.0f);
	start(wheel);
	enc.setPosHistory.clear();

	HostTarget::setPin(BUTTON_A_GPIO_Port, BUTTON_A_Pin, false);
	HostTarget::triggerExti(BUTTON_A_Pin);
	tick(wheel);

	CHECK(enc.setPosHistory.empty());
}

TEST_CASE_FIXTURE(HidMainFixture, "an emergency stop error puts the main class into emergency stop and clearing it releases it" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	const Error estop(ErrorCode::emergencyStop, ErrorType::critical, "test");

	ErrorHandler::addError(estop);
	CHECK(get(wheel, "estop") == 1);

	ErrorHandler::clearError(estop);
	CHECK(get(wheel, "estop") == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "a new error pulses the error LED" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	uint32_t pulses = HostPlatform::leds().errPulses;

	ErrorHandler::addError(Error(ErrorCode::undervoltage, ErrorType::warning, "test"));

	CHECK(HostPlatform::leds().errPulses > pulses);
	CHECK(get(wheel, "estop") == 0); // a warning does not stop
}

TEST_CASE_FIXTURE(HidMainFixture, "a critical error puts the main class into emergency stop" *
		KNOWN_ISSUE("FFBHIDMain::errorCallback (FFBHIDMain.cpp:361-364) calls emergencyStop(true) for a new critical "
					"error. The parameter means reset (line 297-299: control.emergency = !reset), so a critical error "
					"RELEASES an active emergency stop instead of triggering one. The first CHECK fails") *
		DELETES_FFBHIDMAIN) {
	FFBWheel wheel;

	ErrorHandler::addError(Error(ErrorCode::overtemp, ErrorType::critical, "test"));

	CHECK(get(wheel, "estop") == 1);
}

// ---------------- USB suspend / resume ----------------

TEST_CASE_FIXTURE(HidMainFixture, "USB suspend disables FFB and deletes all effects" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;
	wheel.usbResume(); // USB mounted (tud_mount_cb)
	host.deviceControl(0x01);
	host.allocateEffect(FFB_EFFECT_CONSTANT);
	REQUIRE(host.pool().ramPoolSize > 0);
	REQUIRE(get(wheel, "ffbactive") == 1);

	wheel.usbSuspend();

	CHECK(get(wheel, "ffbactive") == 0);
	// All effect slots are free again: the next effect gets block index 1
	CHECK(host.allocateEffect(FFB_EFFECT_CONSTANT) == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "USB suspend suspends the axes and resume restarts them" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	selectMocks();
	MockMotorDriver &drv = driverOf();
	wheel.usbResume(); // USB mounted (tud_mount_cb)
	REQUIRE(drv.running);

	wheel.usbSuspend();
	CHECK(drv.lastTorque == 0);
	CHECK_FALSE(drv.running);

	wheel.usbResume();
	CHECK(drv.running);
}

TEST_CASE_FIXTURE(HidMainFixture, "a second USB suspend does not reset FFB enabled by the host in between" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;
	wheel.usbResume();
	wheel.usbSuspend();
	host.deviceControl(0x01);

	wheel.usbSuspend(); // already suspended: ignored

	CHECK(get(wheel, "ffbactive") == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "a USB suspend before the first USB resume changes nothing" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel; // Control_t::usb_disabled is true until the USB stack reports the mount (usbResume)
	PidHost host;
	host.deviceControl(0x01);

	wheel.usbSuspend();

	CHECK(get(wheel, "ffbactive") == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "USB resume after suspend allows a new suspend" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	PidHost host;
	wheel.usbSuspend();
	wheel.usbResume();
	host.deviceControl(0x01);

	wheel.usbSuspend();

	CHECK(get(wheel, "ffbactive") == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "USB resume releases an emergency stop when the E-stop pin is released" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	REQUIRE(cmd(wheel, "estop", CMDtype::set, 1) == CommandStatus::OK);

	wheel.usbResume();

	CHECK(get(wheel, "estop") == 0);
}

TEST_CASE_FIXTURE(HidMainFixture, "USB resume keeps the emergency stop while the E-stop pin is pressed" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	REQUIRE(cmd(wheel, "estop", CMDtype::set, 1) == CommandStatus::OK);
	HostTarget::setPin(E_STOP_GPIO_Port, E_STOP_Pin, false);

	wheel.usbResume();

	CHECK(get(wheel, "estop") == 1);
}

TEST_CASE_FIXTURE(HidMainFixture, "USB resume applies the selected rate again" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	REQUIRE(cmd(wheel, "hidsendspd", CMDtype::set, 2) == CommandStatus::OK);
	htim13.Instance->ARR = 100; // as after a timer re-init

	wheel.usbResume();

	CHECK(htim13.Instance->ARR == 4000);
}

TEST_CASE_FIXTURE(HidMainFixture, "the idle update of the main class waits 4 ms" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	uint64_t before = HostPlatform::nowUs();

	wheel.update();

	CHECK(HostPlatform::nowUs() - before == 4000);
}

} // TEST_SUITE
