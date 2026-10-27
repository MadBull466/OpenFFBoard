/*
 * Tests of the CAN remote input main class (UserExtensions/Src/CanInputMain.cpp): button and analog sources selected
 * through SelectableInputs (MockButtonSource / MockAnalogSource through the real class choosers, support/InputSources.h)
 * are sent as CAN frames on the global `canport` (the real CANPort_2B on the fake HAL CAN): button frame (8 bytes, 64
 * buttons little endian, id canidbtn), analog frames (4 values of 16 bit little endian per frame, ids canidain,
 * canidain+1, ...), only when the values changed, every report_rates[rate] update cycles; the commands, flash
 * save / restore and the FFBMAIN thread.
 *
 * Test design: CANInputMain::digitalBuffer has no initializer (CanInputMain.h; bug 114). To keep the cases
 * deterministic the fixture constructs the class with placement new in zero-filled storage (InputMainFixture::create);
 * the bug 114 case uses a non-zero pattern. The FFBMAIN thread restarts at Delay(1) on the host: the cases call
 * updateControl() / sendReport() directly, one case runs the thread with a delay budget.
 * The transmit mailboxes are acknowledged automatically (canSetTxAutoComplete) unless a case says otherwise. The fake
 * acknowledges at once: on the target a report of more than 3 frames (buttons + more than 8 analog values) finds no
 * free mailbox for frame 4+, the send fails at once (CANPort2B bug 110) and the frame is not resent (NEEDS REVIEW 127).
 * CANInputMain uses the flash addresses of FFBWheel for its sources (ADR_FFBWHEEL_BUTTONCONF / _ANALOGCONF,
 * CanInputMain.cpp:56-60, 78-79; NEEDS REVIEW 140).
 *
 * Host-vs-target notes: digitalBuffer is 64 bit, written by the FFBMAIN thread and read by the command thread (can
 * tear on the 32-bit target, not testable without preemption).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "InputSources.h"
#include "TestHelpers.h"

#include "CANPort2B.h"
#include "CanInputMain.h"
#include "cpp_target_config.h"
#include "eeprom_addresses.h"

#include <cstring>
#include <new>
#include <string>
#include <vector>

namespace {

struct InputMainFixture : TargetFixture {
	alignas(CANInputMain) unsigned char storage[sizeof(CANInputMain)];
	CANInputMain *main = nullptr;

	InputMainFixture() { HostTarget::canSetTxAutoComplete(true); }
	~InputMainFixture() override { destroy(); }

	/** Constructs the main class in storage filled with `fill` (see the file comment) */
	CANInputMain &create(uint8_t fill = 0) {
		std::memset(storage, fill, sizeof(storage));
		main = new (storage) CANInputMain();
		return *main;
	}
	void destroy() {
		if (main != nullptr) {
			main->~CANInputMain();
			main = nullptr;
		}
	}

	CommandStatus run(const std::string &name, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr) {
		return CommandHelpers::run(*main, name, type, val, replies);
	}
	int64_t get(const std::string &name) { return CommandHelpers::get(*main, name); }

	/** Creates the main class with the mock registries and selects one MockButtonSource */
	MockButtonSource &withButtons() {
		if (main == nullptr) create();
		InputSources::useMockRegistries(*main);
		REQUIRE(run("btntypes", CMDtype::set, 1 << MockButtonSource::SELECTION_ID) == CommandStatus::OK);
		REQUIRE(MockButtonSource::last() != nullptr);
		return *MockButtonSource::last();
	}
	/** Creates the main class with the mock registries and selects one MockAnalogSource with these values */
	MockAnalogSource &withAnalog(std::vector<int32_t> values) {
		if (main == nullptr) create();
		InputSources::useMockRegistries(*main);
		REQUIRE(run("aintypes", CMDtype::set, 1 << MockAnalogSource::SELECTION_ID) == CommandStatus::OK);
		REQUIRE(MockAnalogSource::last() != nullptr);
		MockAnalogSource::last()->axes = std::move(values);
		return *MockAnalogSource::last();
	}

	static const std::vector<HostTarget::CanTxFrame> &frames() { return HostTarget::canTxFrames(); }
	static std::vector<uint8_t> payload(size_t i) { return frames().at(i).payload(); }
};

} // namespace

TEST_SUITE("CANInputMain") {

// ---------------------------------------------------------------- class, port, thread

TEST_CASE_FIXTURE(InputMainFixture, "the CAN input main class has the CAN remote input class id") {
	create();

	CHECK(main->getInfo().id == CLSID_MAIN_CANINPUT);
	CHECK(std::string(main->getInfo().name) == "CAN remote Digital/Analog");
	CHECK(main->getHelpstring() == "Remote CAN Analog/Digital source");
}

TEST_CASE_FIXTURE(InputMainFixture, "the CAN port runs while the CAN input main class exists") {
	create();

	// The class never takes the port itself; the CAN command interface of FFBoardMain (CANCOMMANDS) does. Without
	// CANCOMMANDS (F407VG_DISCO) nobody would start the port (NEEDS REVIEW 138)
	CHECK(canport.getPortUsers() >= 1);
	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(InputMainFixture, "the CAN input main class starts its FFBMAIN thread") {
	create();

	CHECK(HostRtos::findTask("FFBMAIN") == main->GetHandle());
}

TEST_CASE_FIXTURE(InputMainFixture, "the FFBMAIN thread runs one update cycle per millisecond") {
	MockButtonSource &buttons = withButtons();
	HostRtos::setDelayBudgetMs(9); // 8 x Delay(1) + updateControl, the 9th Delay blocks
	uint64_t start = HostPlatform::nowUs();

	HostRtos::runUntilBlocked(*main);

	CHECK(buttons.readCalls == 8);
	CHECK(HostPlatform::nowUs() - start == 9000);
}

TEST_CASE_FIXTURE(InputMainFixture, "usbInit of the CAN input main class registers a CDC-only USB device") {
	create();

	main->usbInit();

	REQUIRE(usb_device != nullptr);
	CHECK(usb_device->getUsbConfigurationDesc(0) == usb_cdc_conf);
}

// ---------------------------------------------------------------- buttons

TEST_CASE_FIXTURE(InputMainFixture, "without input sources no frame is sent") {
	// Holds for the button frame only because the fixture constructs over zero bytes: digitalBuffer is not
	// initialised (bug 114) and the first report compares against it
	create();

	main->sendReport();

	CHECK(frames().empty());
}

TEST_CASE_FIXTURE(InputMainFixture, "a changed button state is sent as 8 little endian bytes with the button id") {
	MockButtonSource &buttons = withButtons();
	buttons.buttons = 0x8000000000000201ULL;

	main->sendReport();

	REQUIRE(frames().size() == 1);
	CHECK(frames()[0].id == 100);
	CHECK_FALSE(frames()[0].extId);
	CHECK_FALSE(frames()[0].rtr);
	CHECK(frames()[0].dlc == 8);
	CHECK(payload(0) == std::vector<uint8_t>{0x01, 0x02, 0, 0, 0, 0, 0, 0x80});
}

TEST_CASE_FIXTURE(InputMainFixture, "an unchanged button state is not sent again") {
	MockButtonSource &buttons = withButtons();
	buttons.buttons = 0x5;
	main->sendReport();
	REQUIRE(frames().size() == 1);

	main->sendReport();

	CHECK(frames().size() == 1);
}

TEST_CASE_FIXTURE(InputMainFixture, "releasing all buttons sends a frame with all bits clear") {
	MockButtonSource &buttons = withButtons();
	buttons.buttons = 0x5;
	main->sendReport();

	buttons.buttons = 0;
	main->sendReport();

	REQUIRE(frames().size() == 2);
	CHECK(frames()[1].value() == 0);
}

TEST_CASE_FIXTURE(InputMainFixture, "the buttons of two sources are packed one after the other") {
	create();
	static const std::vector<class_entry<ButtonSource>> registry = {MockButtonSource::classEntry(14), MockButtonSource::classEntry(15)};
	InputSources::buttonChooser(*main) = ClassChooser<ButtonSource>(registry);
	REQUIRE(run("btntypes", CMDtype::set, (1 << 14) | (1 << 15)) == CommandStatus::OK);
	REQUIRE(MockButtonSource::instances().size() == 2);
	MockButtonSource::instances()[0]->setButtonCount(3);
	MockButtonSource::instances()[0]->buttons = 0b101;
	MockButtonSource::instances()[1]->buttons = 0b11;

	main->sendReport();

	REQUIRE(frames().size() == 1);
	CHECK(frames()[0].value() == 0b11101);
}

TEST_CASE_FIXTURE(InputMainFixture, "the button frame uses the id set with canidbtn") {
	MockButtonSource &buttons = withButtons();
	REQUIRE(run("canidbtn", CMDtype::set, 0x321) == CommandStatus::OK);
	buttons.buttons = 1;

	main->sendReport();

	REQUIRE(frames().size() == 1);
	CHECK(frames()[0].id == 0x321);
}

TEST_CASE_FIXTURE(InputMainFixture, "the button state reported by dvals is 0 before the first report" *
										KNOWN_ISSUE("bug 114: CANInputMain::digitalBuffer "
													"(CanInputMain.h:59) has no initializer and the constructor "
													"(CanInputMain.cpp:27-36) does not set it: dvals reads an indeterminate "
													"value and the first report compares against it (CanInputMain.cpp:112-115), "
													"so whether the first button frame is sent depends on the heap content. "
													"Constructed over 0xA5 bytes the CHECK fails")) {
	create(0xA5);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(get("dvals") == 0);
}

TEST_CASE_FIXTURE(InputMainFixture, "dvals reports the button state of the last report") {
	MockButtonSource &buttons = withButtons();
	buttons.buttons = 0x1234;
	main->sendReport();
	buttons.buttons = 0x1; // not reported yet

	std::vector<CommandReply> replies;
	CHECK(run("dvals", CMDtype::get, 0, &replies) == CommandStatus::OK);

	REQUIRE(replies.size() == 1);
	CHECK(replies[0].val == 0x1234);
	CHECK(replies[0].adr == 0);
}

TEST_CASE_FIXTURE(InputMainFixture, "characterization: a button frame that could not be sent is not repeated while the state is unchanged (NEEDS REVIEW 127)") {
	// sendReport ignores the result of can.sendMessage (CanInputMain.cpp:122) and stores the new state anyway: the
	// receiver keeps the old buttons until the next change. No periodic refresh either (see the next case)
	HostTarget::canSetTxAutoComplete(false);
	MockButtonSource &buttons = withButtons();
	withAnalog({1, 2, 3, 4, 5}); // 2 analog frames
	buttons.buttons = 1;
	main->sendReport(); // button frame + 2 analog frames: all 3 mailboxes pending
	REQUIRE(HostTarget::canTxPending() == 0b111);
	buttons.buttons = 3;
	main->sendReport(); // no free mailbox: the button frame is lost
	REQUIRE(frames().size() == 3);
	HostTarget::canTxCompleteAll();

	main->sendReport();

	CHECK(frames().size() == 3);
}

TEST_CASE_FIXTURE(InputMainFixture, "characterization: unchanged values are never sent again however many update cycles pass (NEEDS REVIEW 127)") {
	// "rate" (CAN interval rate) only sets how often the sources are polled; frames are sent on change only
	// (CanInputMain.cpp:115, 148). A receiver that starts later never gets the current state until something changes
	MockButtonSource &buttons = withButtons();
	withAnalog({100, 200});
	buttons.buttons = 1;
	main->updateControl();
	REQUIRE(frames().size() == 2);

	for (int i = 0; i < 1000; i++) {
		main->updateControl();
	}

	CHECK(frames().size() == 2);
}

// ---------------------------------------------------------------- analog

TEST_CASE_FIXTURE(InputMainFixture, "analog values are sent as 16 bit little endian values with the analog id") {
	withAnalog({0x0102, 0x0304, -2, 0x7FFF});

	main->sendReport();

	REQUIRE(frames().size() == 1);
	CHECK(frames()[0].id == 110);
	CHECK(frames()[0].dlc == 8);
	CHECK(payload(0) == std::vector<uint8_t>{0x02, 0x01, 0x04, 0x03, 0xFE, 0xFF, 0xFF, 0x7F});
}

TEST_CASE_FIXTURE(InputMainFixture, "more than 4 analog values are sent in frames with consecutive ids") {
	withAnalog({1, 2, 3, 4, 5, 6});

	main->sendReport();

	REQUIRE(frames().size() == 2);
	CHECK(frames()[0].id == 110);
	CHECK(frames()[1].id == 111);
	CHECK(payload(1) == std::vector<uint8_t>{5, 0, 6, 0, 0, 0, 0, 0});
}

TEST_CASE_FIXTURE(InputMainFixture, "the first analog frame uses the id set with canidain") {
	withAnalog({1});
	REQUIRE(run("canidain", CMDtype::set, 0x400) == CommandStatus::OK);

	main->sendReport();

	REQUIRE(frames().size() == 1);
	CHECK(frames()[0].id == 0x400);
}

TEST_CASE_FIXTURE(InputMainFixture, "an analog frame is only sent when one of its values changed") {
	MockAnalogSource &ain = withAnalog({1, 2, 3, 4, 5, 6});
	main->sendReport();
	REQUIRE(frames().size() == 2);

	ain.axes[1] = 20;
	main->sendReport();

	REQUIRE(frames().size() == 3);
	CHECK(frames()[2].id == 110);
	CHECK(payload(2) == std::vector<uint8_t>{1, 0, 20, 0, 3, 0, 4, 0});
}

TEST_CASE_FIXTURE(InputMainFixture, "an analog frame is sent when one of its values decreases") {
	MockAnalogSource &ain = withAnalog({1, 2, 3, 4});
	main->sendReport();
	REQUIRE(frames().size() == 1);

	ain.axes[3] = -4;
	main->sendReport();

	REQUIRE(frames().size() == 2);
	CHECK(payload(1) == std::vector<uint8_t>{1, 0, 2, 0, 3, 0, 0xFC, 0xFF});
}

TEST_CASE_FIXTURE(InputMainFixture, "characterization: analog values that are 0 are not sent until they change (NEEDS REVIEW 127)") {
	// After a change of the number of values the comparison buffer is set to 0 (CanInputMain.cpp:126-128), so a
	// group of zero values counts as unchanged
	withAnalog({0, 0, 0, 0, 7});

	main->sendReport();

	REQUIRE(frames().size() == 1);
	CHECK(frames()[0].id == 111);
}

TEST_CASE_FIXTURE(InputMainFixture, "the unused bytes of the last analog frame are zero when the frame before it was not sent" *
										KNOWN_ISSUE("bug 115: sendReport reuses one CAN_tx_msg for all "
													"analog frames and clears it only after a frame was sent "
													"(CanInputMain.cpp:130, 148-151). When an unchanged group is skipped, the "
													"next (shorter) group's frame keeps the bytes of the skipped group: values "
													"3 and 4 appear in bytes 4-7 of the frame with value 5. The CHECK fails")) {
	MockAnalogSource &ain = withAnalog({1, 2, 3, 4, 5, 6});
	main->sendReport();
	REQUIRE(frames().size() == 2);

	ain.axes[4] = 50;
	main->sendReport();
	REQUIRE_ALL_HAL_MODELLED();

	REQUIRE(frames().size() == 3);
	REQUIRE(frames()[2].id == 111);
	CHECK(payload(2) == std::vector<uint8_t>{50, 0, 6, 0, 0, 0, 0, 0});
}

TEST_CASE_FIXTURE(InputMainFixture, "avals reports each analog value of the last report with its index") {
	MockAnalogSource &ain = withAnalog({10, -20, 30});
	main->sendReport();
	ain.axes[0] = 99; // not reported yet
	std::vector<CommandReply> replies;

	CHECK(run("avals", CMDtype::get, 0, &replies) == CommandStatus::OK);

	REQUIRE(replies.size() == 3);
	CHECK(replies[0].val == 10);
	CHECK(replies[0].adr == 0);
	CHECK(replies[1].val == -20);
	CHECK(replies[1].adr == 1);
	CHECK(replies[2].val == 30);
	CHECK(replies[2].adr == 2);
}

TEST_CASE_FIXTURE(InputMainFixture, "avals reports nothing before the first report") {
	create();
	std::vector<CommandReply> replies;

	CHECK(run("avals", CMDtype::get, 0, &replies) == CommandStatus::OK);

	CHECK(replies.empty());
}

// ---------------------------------------------------------------- report rate

TEST_CASE_FIXTURE(InputMainFixture, "with rate 0 the sources are read in every update cycle") {
	MockButtonSource &buttons = withButtons();

	for (int i = 0; i < 3; i++) main->updateControl();

	CHECK(buttons.readCalls == 3);
}

TEST_CASE_FIXTURE(InputMainFixture, "rate 2 reads the sources in every 4th update cycle") {
	MockButtonSource &buttons = withButtons();
	REQUIRE(run("rate", CMDtype::set, 2) == CommandStatus::OK);

	for (int i = 0; i < 3; i++) main->updateControl();
	CHECK(buttons.readCalls == 0);
	main->updateControl();
	CHECK(buttons.readCalls == 1);
	for (int i = 0; i < 4; i++) main->updateControl();
	CHECK(buttons.readCalls == 2);
}

TEST_CASE_FIXTURE(InputMainFixture, "rate 6 reads the sources in every 32nd update cycle") {
	MockButtonSource &buttons = withButtons();
	REQUIRE(run("rate", CMDtype::set, 6) == CommandStatus::OK);

	for (int i = 0; i < 64; i++) main->updateControl();

	CHECK(buttons.readCalls == 2);
}

TEST_CASE_FIXTURE(InputMainFixture, "the rate command reports the rate index that was set") {
	create();
	CHECK(get("rate") == 0);

	REQUIRE(run("rate", CMDtype::set, 6) == CommandStatus::OK);

	CHECK(get("rate") == 6);
}

TEST_CASE_FIXTURE(InputMainFixture, "the rate info lists the 7 report rates with their index") {
	create();
	std::vector<CommandReply> replies;

	CHECK(run("rate", CMDtype::info, 0, &replies) == CommandStatus::OK);

	REQUIRE(replies.size() == 1);
	CHECK(replies[0].reply == "1000Hz:0,500Hz:1,250Hz:2,125Hz:3,100Hz:4,62Hz:5,31Hz:6");
}

TEST_CASE_FIXTURE(InputMainFixture, "a report rate index beyond the last rate never selects a rate that does not exist" * doctest::skip() *
										doctest::description("KNOWN ISSUE (crash): bug 116: setReportRate clips the index to "
															 "report_rates.size() = 7 instead of size()-1 (CanInputMain.cpp:160) "
															 "and reads report_rates[7] (:162), one past the std::array: aborts the "
															 "run with _GLIBCXX_ASSERTIONS (GCC 16 -O0). Reachable with rate=7 (or "
															 "more) and with a stored rate index 7 (CanInputMain.cpp:65-66, 3 bit "
															 "field). Skipped")) {
	create();

	run("rate", CMDtype::set, 7); // a fix may clip (OK) or reject (ERR)

	CHECK(get("rate") <= 6);
}

// ---------------------------------------------------------------- commands

TEST_CASE_FIXTURE(InputMainFixture, "the default CAN ids are 100 for buttons and 110 for analog values") {
	create();

	CHECK(get("canidbtn") == 100);
	CHECK(get("canidain") == 110);
}

TEST_CASE_FIXTURE(InputMainFixture, "characterization: a button id above 11 bits is accepted and loses its upper bits on the bus (NEEDS REVIEW 128)") {
	// canidbtn / canidain store any value (handleGetSet, CanInputMain.cpp:200-205); the HAL puts StdId << 21 into
	// TIR without masking, so 0x923 goes out as 0x123; saveFlash masks with 0x7FF (CanInputMain.cpp:82-86)
	MockButtonSource &buttons = withButtons();
	REQUIRE(run("canidbtn", CMDtype::set, 0x923) == CommandStatus::OK);
	buttons.buttons = 1;

	main->sendReport();

	CHECK(get("canidbtn") == 0x923);
	REQUIRE(frames().size() == 1);
	CHECK(frames()[0].id == 0x123);
}

TEST_CASE_FIXTURE(InputMainFixture, "btntypes reports and sets the enabled button sources") {
	create();
	InputSources::useMockRegistries(*main);
	CHECK(get("btntypes") == 0);

	REQUIRE(run("btntypes", CMDtype::set, 1 << MockButtonSource::SELECTION_ID) == CommandStatus::OK);

	CHECK(get("btntypes") == (1 << MockButtonSource::SELECTION_ID));
	CHECK(MockButtonSource::instances().size() == 1);
}

TEST_CASE_FIXTURE(InputMainFixture, "aintypes reports and sets the enabled analog sources") {
	create();
	InputSources::useMockRegistries(*main);
	CHECK(get("aintypes") == 0);

	REQUIRE(run("aintypes", CMDtype::set, 1 << MockAnalogSource::SELECTION_ID) == CommandStatus::OK);

	CHECK(get("aintypes") == (1 << MockAnalogSource::SELECTION_ID));
	CHECK(MockAnalogSource::instances().size() == 1);
}

TEST_CASE_FIXTURE(InputMainFixture, "clearing btntypes and aintypes deletes the sources") {
	withButtons();
	withAnalog({1});

	REQUIRE(run("btntypes", CMDtype::set, 0) == CommandStatus::OK);
	REQUIRE(run("aintypes", CMDtype::set, 0) == CommandStatus::OK);

	CHECK(MockButtonSource::instances().empty());
	CHECK(MockAnalogSource::instances().empty());
}

TEST_CASE_FIXTURE(InputMainFixture, "addbtn adds a button source whose buttons are sent") {
	create();
	InputSources::useMockRegistries(*main);

	REQUIRE(run("addbtn", CMDtype::set, MockButtonSource::SELECTION_ID) == CommandStatus::OK);
	REQUIRE(MockButtonSource::instances().size() == 1);
	MockButtonSource::last()->buttons = 0x40;
	main->sendReport();

	REQUIRE(frames().size() == 1);
	CHECK(frames()[0].value() == 0x40);
}

TEST_CASE_FIXTURE(InputMainFixture, "addain adds an analog source whose values are sent") {
	create();
	InputSources::useMockRegistries(*main);

	REQUIRE(run("addain", CMDtype::set, MockAnalogSource::SELECTION_ID) == CommandStatus::OK);
	REQUIRE(MockAnalogSource::instances().size() == 1);
	MockAnalogSource::last()->axes = {0x55};
	main->sendReport();

	REQUIRE(frames().size() == 1);
	CHECK(payload(0) == std::vector<uint8_t>{0x55, 0, 0, 0, 0, 0, 0, 0});
}

TEST_CASE_FIXTURE(InputMainFixture, "a get of addbtn or addain adds no source") {
	create();
	InputSources::useMockRegistries(*main);

	run("addbtn", CMDtype::get); // set-only commands (the command interface rejects a get before the class sees it)
	run("addain", CMDtype::get);

	CHECK(MockButtonSource::instances().empty());
	CHECK(MockAnalogSource::instances().empty());
}

TEST_CASE_FIXTURE(InputMainFixture, "lsbtn and lsain list the selectable sources") {
	create();
	InputSources::useMockRegistries(*main);
	std::vector<CommandReply> btn;
	std::vector<CommandReply> ain;

	CHECK(run("lsbtn", CMDtype::get, 0, &btn) == CommandStatus::OK);
	CHECK(run("lsain", CMDtype::get, 0, &ain) == CommandStatus::OK);

	REQUIRE(btn.size() == MockButtonSource::registry().size());
	CHECK(btn.back().reply == "15:1:Mock buttons");
	REQUIRE(ain.size() == MockAnalogSource::registry().size());
	CHECK(ain.back().reply == "15:1:Mock analog");
}

TEST_CASE_FIXTURE(InputMainFixture, "an unknown command id of the CAN input main class is not found") {
	create();
	ParsedCommand cmd;
	cmd.cmdId = 0x7777;
	cmd.type = CMDtype::get;
	cmd.target = main;
	std::vector<CommandReply> replies;

	CHECK(main->command(cmd, replies) == CommandStatus::NOT_FOUND);
}

TEST_CASE_FIXTURE(InputMainFixture, "commands of the CAN input main class work through the CDC port") {
	create();
	CommandHelpers::CdcConsole console(*main);

	CHECK(console.send("main.0.canidbtn=200\n") == "[main.0.canidbtn=200|OK]\n");
	CHECK(console.send("main.0.canidbtn?\n") == "[main.0.canidbtn?|200]\n");
}

// ---------------------------------------------------------------- flash

TEST_CASE_FIXTURE(InputMainFixture, "saveFlash stores the sources, the button id with the rate index and the analog id") {
	withButtons();
	withAnalog({1});
	REQUIRE(run("canidbtn", CMDtype::set, 0x123) == CommandStatus::OK);
	REQUIRE(run("canidain", CMDtype::set, 0x456) == CommandStatus::OK);
	REQUIRE(run("rate", CMDtype::set, 5) == CommandStatus::OK);

	main->saveFlash();

	auto &flash = HostPlatform::flash();
	CHECK(flash[ADR_FFBWHEEL_BUTTONCONF] == (1 << MockButtonSource::SELECTION_ID));
	CHECK(flash[ADR_FFBWHEEL_ANALOGCONF] == (1 << MockAnalogSource::SELECTION_ID));
	CHECK(flash[ADR_CANREMOTE_CONF1] == (0x123 | (5 << 11)));
	CHECK(flash[ADR_CANREMOTE_CONF2] == 0x456);
}

TEST_CASE_FIXTURE(InputMainFixture, "the CAN ids and the rate are restored from flash when the class is created") {
	HostPlatform::flash()[ADR_CANREMOTE_CONF1] = 0x5AB | (3 << 11);
	HostPlatform::flash()[ADR_CANREMOTE_CONF2] = 0x7F0;

	create();

	CHECK(get("canidbtn") == 0x5AB);
	CHECK(get("rate") == 3);
	CHECK(get("canidain") == 0x7F0);
}

TEST_CASE_FIXTURE(InputMainFixture, "the button and analog sources are restored from flash when the class is created") {
	HostPlatform::flash()[ADR_FFBWHEEL_BUTTONCONF] = 1; // selection 0: local buttons ("dpin")
	HostPlatform::flash()[ADR_FFBWHEEL_ANALOGCONF] = 1; // selection 0: local analog ("apin")

	create();

	CHECK(get("btntypes") == 1);
	CHECK(get("aintypes") == 1);
	CHECK(CommandHandler::getHandlerFromClassName("dpin", 0) != nullptr);
	CHECK(CommandHandler::getHandlerFromClassName("apin", 0) != nullptr);
}

TEST_CASE_FIXTURE(InputMainFixture, "settings saved with the system save command are restored by a new instance") {
	withButtons();
	REQUIRE(run("canidbtn", CMDtype::set, 0x222) == CommandStatus::OK);
	REQUIRE(run("canidain", CMDtype::set, 0x333) == CommandStatus::OK);
	REQUIRE(run("rate", CMDtype::set, 4) == CommandStatus::OK);
	CommandHelpers::CdcConsole console(*main);
	console.send("sys.save?\n");
	destroy();

	create();

	CHECK(get("canidbtn") == 0x222);
	CHECK(get("canidain") == 0x333);
	CHECK(get("rate") == 4);
	CHECK(get("btntypes") == (1 << MockButtonSource::SELECTION_ID)); // the mask; the firmware chooser has no mock
}

} // TEST_SUITE
