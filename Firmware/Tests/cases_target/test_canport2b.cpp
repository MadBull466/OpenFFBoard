/*
 * Tests of CANPort_2B (FFBoard/Src/CANPort2B.cpp), the bxCAN port of the F407 target. The global `canport` is the real
 * CANPort_2B on hcan1 with the F407 presets and the silent pin CAN_S (HostTarget::canPort2B(), see "CAN part (P7)" in
 * the README), running on the fake HAL CAN (platform/src/host_target_can.cpp). The smoke cases of P7 batch 0
 * (cases_target/test_can_setup.cpp) already cover: start with the notifications (IER), a frame occupying a mailbox, the
 * transmit complete callback with the mailbox index, the rejection with three pending mailboxes on the test thread,
 * reception through a standard filter, bus-off with and without the status change interrupt, the bugs 106-108 and
 * NEEDS REVIEW 116. They are not repeated here.
 *
 * Host-vs-target notes: the fake acknowledges mode changes at once and transmits a frame only when the test completes
 * its mailbox. A send from a task that waits for the port semaphore runs on a resumable host task and takes no
 * simulated time: the 20 ms timeout of that wait is modelled as "an activation while no mailbox became free ends the
 * wait" (HostRtos::makeResumable). Integer truncations (uint8_t presets / lengths from int64_t command values) are the
 * same on the 32 bit target.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "HostPlatform.h"
#include "HostRtos.h"
#include "HostTarget.h"
#include "TestHelpers.h"

#include "CANPort2B.h"
#include "CanHandler.h"
#include "FFBoardMain.h"
#include "cpp_target_config.h"
#include "thread.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr uint16_t ADR_CANCONF = 0xC1; // ADR_CANCONF1, UserExtensions/Inc/eeprom_addresses.h:44

/** Bit timing (CAN_BTR) and speed of the presets 0..5 of the F407 (Targets/F407VG/Core/Src/cpp_target_config.cpp:52) */
constexpr std::array<uint32_t, 6> PRESET_BTR = {0x001b0037, 0x001b001b, 0x001c0014, 0x001a000b, 0x001a0005, 0x001a0002};
constexpr std::array<uint32_t, 6> PRESET_SPEED = {50000, 100000, 125000, 250000, 500000, 1000000};
constexpr uint8_t DEFAULT_PRESET = 4; // 500k (CANPort2B.h speedPreset initialiser)

/** Presets of a second port in the tests of the destructor and of other instances (same as the F407) */
const auto testPresetEntries = std::to_array<CANPortHardwareConfig::PresetEntry>(
	{{0x001b0037, 50000, "50k"}, {0x001b001b, 100000, "100k"}, {0x001c0014, 125000, "125k"}, {0x001a000b, 250000, "250k"}, {0x001a0005, 500000, "500k"}, {0x001a0002, 1000000, "1000k"}});
const CANPortHardwareConfig testPresets(true, testPresetEntries);

/** Records the received frames of `canport` that reach the CAN handlers (global_callbacks.cpp) */
struct RxRecorder : CanHandler {
	std::vector<CAN_rx_msg> frames;
	void canRxPendCallback(CANPort *port, CAN_rx_msg &msg) override {
		if (port == &canport) {
			frames.push_back(msg);
		}
	}
};

/** A firmware-like task running `body` once, then waiting for notifications forever (as in test_spi_port.cpp) */
struct BodyTask : cpp_freertos::Thread {
	explicit BodyTask(std::function<void()> f, const char *name = "CANTASK") : cpp_freertos::Thread(name, 128, 1), body(std::move(f)) { Start(); }
	void Run() override {
		body();
		done = true;
		while (true) {
			WaitForNotification();
		}
	}
	std::function<void()> body;
	bool done = false;
};

CAN_tx_msg frame(uint32_t id, const std::vector<uint8_t> &bytes, bool extId = false, bool rtr = false) {
	CAN_tx_msg msg;
	msg.header.id = id;
	msg.header.length = static_cast<uint32_t>(bytes.size());
	msg.header.extId = extId;
	msg.header.rtr = rtr;
	std::copy(bytes.begin(), bytes.end(), msg.data);
	return msg;
}

CAN_filter stdFilter(uint32_t id, uint32_t mask = 0x7FF) {
	CAN_filter f;
	f.filter_id = id;
	f.filter_mask = mask;
	return f;
}

CAN_filter extFilter(uint32_t id, uint32_t mask = 0x1FFFFFFF) {
	CAN_filter f;
	f.filter_id = id;
	f.filter_mask = mask;
	f.extid = true;
	return f;
}

size_t halCallCount(const std::string &function) {
	size_t n = 0;
	for (const HostTarget::CanHalCall &call : HostTarget::canHalCalls()) {
		n += call.function == function ? 1 : 0;
	}
	return n;
}

std::vector<std::string> halCallNames() {
	std::vector<std::string> names;
	for (const HostTarget::CanHalCall &call : HostTarget::canHalCalls()) {
		names.push_back(call.function);
	}
	return names;
}

struct CanPortFixture : TargetFixture {
	CANPort_2B &port = HostTarget::canPort2B();

	static bool silentPinHigh() { return HostTarget::outputPin(CAN_S_GPIO_Port, CAN_S_Pin); }

	bool send(uint32_t id, const std::vector<uint8_t> &bytes = {0}) {
		CAN_tx_msg msg = frame(id, bytes);
		return port.sendMessage(msg);
	}

	/** Takes the port and occupies all three transmit mailboxes (ids 1, 2, 3 in mailboxes 0, 1, 2) */
	void fillMailboxes() {
		port.takePort();
		for (uint32_t id = 1; id <= 3; id++) {
			REQUIRE(send(id));
		}
		REQUIRE(HostTarget::canTxPending() == 0b111);
	}

	/** True if the port semaphore could be taken now (taken by the test afterwards) */
	bool takeSemaphoreNow() { return port.takeSemaphore(0); }

	CommandStatus cmd(const std::string &name, CMDtype type, int64_t val = 0, std::vector<CommandReply> *replies = nullptr, int64_t adr = 0) {
		return CommandHelpers::run(port, name, type, val, replies, adr);
	}
};

} // namespace

TEST_SUITE("CANPort_2B") {

// ---------------------------------------------------------------- identity, construction, destruction

TEST_CASE_FIXTURE(CanPortFixture, "the CAN port is the port class can.0 with the commands speed, send and len") {
	CHECK(std::string(port.getInfo().name) == "Can port");
	CHECK(port.getInfo().id == CLSID_CANPORT);
	CHECK(port.getClassType() == ClassType::Port);
	CHECK(std::string(port.getCommandHandlerInfo()->clsname) == "can");
	CHECK(port.getCommandHandlerInfo()->instance == 0);
	CHECK(port.getCommandFromName("speed") != nullptr);
	CHECK(port.getCommandFromName("send") != nullptr);
	CHECK(port.getCommandFromName("len") != nullptr);
}

TEST_CASE_FIXTURE(CanPortFixture, "the CAN port is found by its HAL handle") {
	CHECK(port.getHandle() == &hcan1);
	CHECK(CANPort::handleToPort(&hcan1) == &canport);
}

TEST_CASE_FIXTURE(CanPortFixture, "deleting a CAN port disables its filters, stops CAN1 and sets its silent pin high") {
	const OutputPin silentPin(*CAN_S_GPIO_Port, CAN_S_Pin);
	auto *second = new CANPort_2B(hcan1, testPresets, &silentPin, 1); // instance 1: no flash access, no clash with can.0
	second->takePort();
	REQUIRE(second->addCanFilter(stdFilter(0x321)) == 0);
	REQUIRE(HostTarget::canAccepts(0x321));
	REQUIRE_FALSE(silentPinHigh());

	delete second;

	CHECK_FALSE(HostTarget::canAccepts(0x321));
	CHECK(HostTarget::canFilterConfigs().back().FilterBank == 0);
	CHECK(HostTarget::canFilterConfigs().back().FilterActivation == CAN_FILTER_DISABLE);
	CHECK(hcan1.State == HAL_CAN_STATE_READY);
	CHECK(silentPinHigh());
	CHECK(CANPort::handleToPort(&hcan1) == &canport);
}

TEST_CASE_FIXTURE(CanPortFixture, "a CAN port other than instance 0 neither saves nor restores its speed") {
	auto *second = new CANPort_2B(hcan1, testPresets, nullptr, 1);
	HostPlatform::flash()[ADR_CANCONF] = 1;
	second->setSpeedPreset(2);

	second->saveFlash();
	second->restoreFlash();

	CHECK(HostPlatform::flash()[ADR_CANCONF] == 1);
	CHECK(second->getSpeedPreset() == 2);
	delete second;
}

// ---------------------------------------------------------------- port users, start, stop

TEST_CASE_FIXTURE(CanPortFixture, "a second user of the port does not start CAN1 again") {
	port.takePort();
	HostTarget::canClearHalCalls();

	port.takePort();

	CHECK(port.getPortUsers() == 2);
	CHECK(HostTarget::canHalCalls().empty());
	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(CanPortFixture, "the port keeps running until its last user frees it") {
	port.takePort();
	port.takePort();

	port.freePort();
	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
	CHECK_FALSE(silentPinHigh());

	port.freePort();
	CHECK(port.getPortUsers() == 0);
	CHECK(hcan1.State == HAL_CAN_STATE_READY);
	CHECK(silentPinHigh());
}

TEST_CASE_FIXTURE(CanPortFixture, "starting the port switches the transceiver out of silent mode") {
	port.setSilentMode(true);
	REQUIRE(silentPinHigh());

	port.start();

	CHECK_FALSE(silentPinHigh());
}

TEST_CASE_FIXTURE(CanPortFixture, "starting the port writes the bit timing of the selected preset again") {
	port.setSpeedPreset(1);
	CAN1->BTR = 0; // changed behind the port's back

	port.start();

	CHECK(CAN1->BTR == PRESET_BTR[1]);
	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(CanPortFixture, "stopping a running port reports success, leaves the bus and makes the transceiver silent") {
	port.start();

	CHECK(port.stop());

	CHECK(hcan1.State == HAL_CAN_STATE_READY);
	CHECK(silentPinHigh());
}

TEST_CASE_FIXTURE(CanPortFixture, "stopping a port whose CAN controller is not running reports failure") {
	REQUIRE(hcan1.State == HAL_CAN_STATE_READY); // MX_CAN1_Init, never started

	CHECK_FALSE(port.stop());
}

TEST_CASE_FIXTURE(CanPortFixture, "stopping the port reports failure when CAN1 does not enter the initialisation mode") {
	port.start();
	HostTarget::canFailNextModeChange();

	CHECK_FALSE(port.stop());
	CHECK(hcan1.State == HAL_CAN_STATE_ERROR);
}

// ---------------------------------------------------------------- speed presets

TEST_CASE_FIXTURE(CanPortFixture, "each speed preset writes its bit timing to CAN1 and reports its speed") {
	for (uint8_t preset = 0; preset < PRESET_BTR.size(); preset++) {
		CAPTURE(preset);
		port.setSpeedPreset(preset);

		CHECK(port.getSpeedPreset() == preset);
		CHECK(port.getSpeed() == PRESET_SPEED[preset]);
		CHECK(CAN1->BTR == PRESET_BTR[preset]);
	}
}

TEST_CASE_FIXTURE(CanPortFixture, "the CAN port starts with the 500 kbit/s preset") {
	CHECK(port.getSpeedPreset() == DEFAULT_PRESET);
	CHECK(port.getSpeed() == 500000);
}

TEST_CASE_FIXTURE(CanPortFixture, "setting the speed in bit/s selects the preset with that speed") {
	port.setSpeed(250000);

	CHECK(port.getSpeedPreset() == 3);
	CHECK(CAN1->BTR == PRESET_BTR[3]);
}

TEST_CASE_FIXTURE(CanPortFixture, "a speed in bit/s without a preset leaves the speed and CAN1 unchanged") {
	port.takePort();
	HostTarget::canClearHalCalls();

	port.setSpeed(123456);

	CHECK(port.getSpeedPreset() == DEFAULT_PRESET);
	CHECK(CAN1->BTR == PRESET_BTR[DEFAULT_PRESET]);
	CHECK(HostTarget::canHalCalls().empty());
}

TEST_CASE_FIXTURE(CanPortFixture, "a speed preset that does not exist is ignored") {
	port.takePort();
	HostTarget::canClearHalCalls();

	port.setSpeedPreset(6);
	port.setSpeedPreset(255);

	CHECK(port.getSpeedPreset() == DEFAULT_PRESET);
	CHECK(CAN1->BTR == PRESET_BTR[DEFAULT_PRESET]);
	CHECK(HostTarget::canHalCalls().empty());
}

TEST_CASE_FIXTURE(CanPortFixture, "changing the speed of a running port stops CAN1, aborts pending frames and starts it again") {
	port.takePort();
	HostTarget::canClearHalCalls();

	port.setSpeedPreset(2);

	CHECK(halCallNames() == std::vector<std::string>{"HAL_CAN_Stop", "HAL_CAN_AbortTxRequest", "HAL_CAN_ResetError", "HAL_CAN_Start"});
	CHECK(HostTarget::canHalCalls().back().result == HAL_OK);
	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
	CHECK(CAN1->BTR == PRESET_BTR[2]);
}

TEST_CASE_FIXTURE(CanPortFixture, "changing the speed aborts a frame that is still waiting in a transmit mailbox") {
	port.takePort();
	REQUIRE(send(0x10));

	port.setSpeedPreset(3);

	CHECK(HostTarget::canTxFrames()[0].result == HostTarget::CanTxResult::Aborted);
	CHECK(HostTarget::canTxPending() == 0);
}

TEST_CASE_FIXTURE(CanPortFixture, "characterization: changing the speed preset starts CAN1 also while the port has no users (NEEDS REVIEW 119)") {
	// CANPort2B.cpp:142-147 stop and start CAN1 regardless of `active` / portUsers; restoreFlash at startup does the
	// same (CANPort2B.cpp:54). The transceiver stays silent (CAN_S high), but the controller listens to the bus.
	port.setSilentMode(true);
	REQUIRE(hcan1.State == HAL_CAN_STATE_READY);

	port.setSpeedPreset(3);

	CHECK(port.getPortUsers() == 0);
	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
	CHECK(silentPinHigh());
}

// ---------------------------------------------------------------- silent mode

TEST_CASE_FIXTURE(CanPortFixture, "silent mode drives the silent pin of the transceiver") {
	port.setSilentMode(true);
	CHECK(silentPinHigh());

	port.setSilentMode(false);
	CHECK_FALSE(silentPinHigh());
}

TEST_CASE_FIXTURE(CanPortFixture, "sending a frame switches a silent transceiver to normal mode") {
	port.takePort();
	port.setSilentMode(true);

	REQUIRE(send(0x10));

	CHECK_FALSE(silentPinHigh());
}

// ---------------------------------------------------------------- filters

TEST_CASE_FIXTURE(CanPortFixture, "a standard id filter is a 32 bit mask filter with id and mask in bits 31 to 21 for FIFO 0") {
	port.takePort();

	REQUIRE(port.addCanFilter(stdFilter(0x123, 0x7F0)) == 0);

	REQUIRE_FALSE(HostTarget::canFilterConfigs().empty());
	const CAN_FilterTypeDef &cfg = HostTarget::canFilterConfigs().back();
	CHECK(cfg.FilterBank == 0);
	CHECK(cfg.FilterMode == CAN_FILTERMODE_IDMASK);
	CHECK(cfg.FilterScale == CAN_FILTERSCALE_32BIT);
	CHECK(cfg.FilterFIFOAssignment == CAN_RX_FIFO0);
	CHECK(cfg.FilterActivation == CAN_FILTER_ENABLE);
	CHECK(cfg.FilterIdHigh == (0x123U << 5));
	CHECK(cfg.FilterIdLow == 0);
	CHECK(cfg.FilterMaskIdHigh == (0x7F0U << 5)); // FilterMaskIdLow (IDE / RTR not compared): NEEDS REVIEW 117
	CHECK(HostTarget::canAccepts(0x12A)); // masked bits 3:0
	CHECK_FALSE(HostTarget::canAccepts(0x133));
}

TEST_CASE_FIXTURE(CanPortFixture, "an extended id filter has id and mask in bits 31 to 3 and requires the IDE bit") {
	port.takePort();

	REQUIRE(port.addCanFilter(extFilter(0x1ABCDEF5, 0x1FFFFFF0)) == 0);

	const CAN_FilterTypeDef &cfg = HostTarget::canFilterConfigs().back();
	CHECK(cfg.FilterIdHigh == ((0x1ABCDEF5U >> 13) & 0xFFFF));
	CHECK(cfg.FilterIdLow == (((0x1ABCDEF5U << 3) & 0xFFF8) | 0x04));
	CHECK(cfg.FilterMaskIdHigh == ((0x1FFFFFF0U >> 13) & 0xFFFF));
	CHECK(cfg.FilterMaskIdLow == (((0x1FFFFFF0U << 3) & 0xFFF8) | 0x04));
	CHECK(HostTarget::canAccepts(0x1ABCDEF0, true));
	CHECK_FALSE(HostTarget::canAccepts(0x1ABCDEE5, true));
	CHECK_FALSE(HostTarget::canAccepts(0x1ABCDEF5 & 0x7FF, false)); // a standard frame never matches (IDE)
}

TEST_CASE_FIXTURE(CanPortFixture, "a filter with buffer 1 assigns its frames to receive FIFO 1") {
	port.takePort();
	CAN_filter f = stdFilter(0x55);
	f.buffer = 1;

	REQUIRE(port.addCanFilter(f) == 0);

	CHECK(HostTarget::canFilterConfigs().back().FilterFIFOAssignment == CAN_RX_FIFO1);
	uint32_t fifo = 0;
	CHECK(HostTarget::canAccepts(0x55, false, false, &fifo));
	CHECK(fifo == 1);
}

TEST_CASE_FIXTURE(CanPortFixture, "an inactive filter is configured but accepts no frames") {
	port.takePort();
	CAN_filter f = stdFilter(0x55);
	f.active = false;

	CHECK(port.addCanFilter(f) == 0);

	CHECK(HostTarget::canFilterConfigs().back().FilterActivation == CAN_FILTER_DISABLE);
	CHECK_FALSE(HostTarget::canAccepts(0x55));
}

TEST_CASE_FIXTURE(CanPortFixture, "each new filter gets the lowest free filter bank") {
	port.takePort();

	CHECK(port.addCanFilter(stdFilter(0x10)) == 0);
	CHECK(port.addCanFilter(stdFilter(0x11)) == 1);
	CHECK(port.addCanFilter(stdFilter(0x12)) == 2);
	CHECK(HostTarget::canFilterConfigs().back().FilterBank == 2);
}

TEST_CASE_FIXTURE(CanPortFixture, "the bank of a removed filter is reused by the next filter") {
	port.takePort();
	REQUIRE(port.addCanFilter(stdFilter(0x10)) == 0);
	REQUIRE(port.addCanFilter(stdFilter(0x11)) == 1);
	REQUIRE(port.addCanFilter(stdFilter(0x12)) == 2);

	port.removeCanFilter(1);

	CHECK(port.addCanFilter(stdFilter(0x13)) == 1);
	CHECK(port.addCanFilter(stdFilter(0x14)) == 3);
	CHECK(HostTarget::canAccepts(0x13));
	CHECK_FALSE(HostTarget::canAccepts(0x11));
}

TEST_CASE_FIXTURE(CanPortFixture, "removing a filter deactivates only its own bank") {
	port.takePort();
	REQUIRE(port.addCanFilter(stdFilter(0x10)) == 0);
	REQUIRE(port.addCanFilter(stdFilter(0x11)) == 1);

	port.removeCanFilter(0);

	CHECK(HostTarget::canFilterConfigs().back().FilterBank == 0);
	CHECK(HostTarget::canFilterConfigs().back().FilterActivation == CAN_FILTER_DISABLE);
	CHECK_FALSE(HostTarget::canAccepts(0x10));
	CHECK(HostTarget::canAccepts(0x11));
}

TEST_CASE_FIXTURE(CanPortFixture, "removing a bank that holds no filter of the port changes no filter") {
	port.takePort();
	REQUIRE(port.addCanFilter(stdFilter(0x10)) == 0);
	HostTarget::canClearHalCalls();

	port.removeCanFilter(5);

	CHECK(halCallCount("HAL_CAN_ConfigFilter") == 0);
	CHECK(HostTarget::canAccepts(0x10));
}

TEST_CASE_FIXTURE(CanPortFixture, "a filter that CAN1 rejects is reported as not added" *
									  KNOWN_ISSUE("CANPort2B.cpp:314-322: when HAL_CAN_ConfigFilter fails the filter is not stored, but "
												  "addCanFilter still returns the bank number found (0) instead of -1; the documentation at "
												  "CANPort2B.cpp:258-260 promises a bank id only if successful, so the caller believes the "
												  "filter is active (bug 113). The CHECK fails")) {
	port.takePort();
	HostTarget::canFailNextModeChange();
	port.stop(); // CAN1 does not acknowledge the initialisation mode: State ERROR, HAL_CAN_ConfigFilter is rejected
	REQUIRE(hcan1.State == HAL_CAN_STATE_ERROR);

	int32_t bank = port.addCanFilter(stdFilter(0x10));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(bank == -1);
}

TEST_CASE_FIXTURE(CanPortFixture, "a filter rejected by CAN1 does not take a filter bank") {
	port.takePort();
	HostTarget::canFailNextModeChange();
	port.stop();
	REQUIRE(hcan1.State == HAL_CAN_STATE_ERROR);
	(void)port.addCanFilter(stdFilter(0x10)); // rejected by the HAL (return value: bug 113)
	hcan1.State = HAL_CAN_STATE_READY;		  // the controller recovered (e.g. HAL_CAN_Init)

	CHECK(port.addCanFilter(stdFilter(0x11)) == 0);
}

TEST_CASE_FIXTURE(CanPortFixture, "characterization: a standard id filter also accepts an extended frame whose upper 11 id bits equal the filter id (NEEDS REVIEW 117)") {
	// CANPort2B.cpp:272-277 leave the IDE bit out of the mask of a standard filter (FilterMaskIdLow = 0), while an
	// extended filter forces it (CANPort2B.cpp:285-286). RM0090 32.7.4: STID[10:0] overlaps EXID[28:18]
	port.takePort();
	REQUIRE(port.addCanFilter(stdFilter(0x123)) == 0);

	CHECK(HostTarget::canAccepts((0x123U << 18) | 0x2AAAA, true));
}

// ---------------------------------------------------------------- reception through the filters of the port

TEST_CASE_FIXTURE(CanPortFixture, "an extended frame accepted by an extended filter reaches the CAN handlers with its id") {
	RxRecorder handler;
	port.takePort();
	REQUIRE(port.addCanFilter(extFilter(0x1234567)) == 0);

	REQUIRE(HostTarget::canReceive(0x1234567, {0x01, 0x02, 0x03}, true) == 1);

	REQUIRE(handler.frames.size() == 1);
	CHECK(handler.frames[0].header.id == 0x1234567);
	CHECK(handler.frames[0].header.extId);
	CHECK(handler.frames[0].header.length == 3);
	CHECK(handler.frames[0].data[2] == 0x03);
}

TEST_CASE_FIXTURE(CanPortFixture, "a frame of a FIFO 1 filter reaches the CAN handlers on a board that enables the FIFO 1 interrupt") {
	RxRecorder handler;
	port.takePort();
	HostTarget::canSetIrqLineEnabled(HostTarget::CanIrqLine::Rx1, true); // not enabled on the F407 (NEEDS REVIEW 116)
	CAN_filter f = stdFilter(0x77);
	f.buffer = 1;
	REQUIRE(port.addCanFilter(f) == 0);

	REQUIRE(HostTarget::canReceive(0x77, {0xAB}) == 1);

	REQUIRE(handler.frames.size() == 1);
	CHECK(handler.frames[0].header.id == 0x77);
	CHECK(HostTarget::canRxPending(CAN_RX_FIFO1) == 0);
}

// ---------------------------------------------------------------- sending

TEST_CASE_FIXTURE(CanPortFixture, "a standard data frame carries its id in StdId, its length as DLC and its data") {
	port.takePort();
	CAN_tx_msg msg = frame(0x7FF, {1, 2, 3, 4, 5});

	REQUIRE(port.sendMessage(msg));

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const HostTarget::CanTxFrame &sent = HostTarget::canTxFrames()[0];
	CHECK(sent.header.IDE == CAN_ID_STD);
	CHECK(sent.header.RTR == CAN_RTR_DATA);
	CHECK(sent.header.StdId == 0x7FF);
	CHECK(sent.header.ExtId == 0);
	CHECK(sent.header.DLC == 5);
	CHECK(sent.id == 0x7FF);
	CHECK(sent.payload() == std::vector<uint8_t>{1, 2, 3, 4, 5});
}

TEST_CASE_FIXTURE(CanPortFixture, "an extended remote frame carries its id in ExtId with IDE and RTR set") {
	port.takePort();
	CAN_tx_msg msg = frame(0x1ABCDEF, {}, true, true);
	msg.header.length = 4; // requested length of a remote frame

	REQUIRE(port.sendMessage(msg));

	const HostTarget::CanTxFrame &sent = HostTarget::canTxFrames()[0];
	CHECK(sent.header.IDE == CAN_ID_EXT);
	CHECK(sent.header.RTR == CAN_RTR_REMOTE);
	CHECK(sent.header.ExtId == 0x1ABCDEF);
	CHECK(sent.header.StdId == 0);
	CHECK(sent.extId);
	CHECK(sent.rtr);
	CHECK(sent.id == 0x1ABCDEF);
	CHECK(sent.dlc == 4);
}

TEST_CASE_FIXTURE(CanPortFixture, "a standard data frame sent after an extended remote frame has neither IDE nor RTR") {
	port.takePort();
	CAN_tx_msg ext = frame(0x1ABCDEF, {}, true, true);
	REQUIRE(port.sendMessage(ext));

	REQUIRE(send(0x22, {9}));

	const HostTarget::CanTxFrame &sent = HostTarget::canTxFrames()[1];
	CHECK_FALSE(sent.extId);
	CHECK_FALSE(sent.rtr);
	CHECK(sent.id == 0x22);
	CHECK(sent.header.ExtId == 0);
}

TEST_CASE_FIXTURE(CanPortFixture, "a frame sent with a mailbox pointer reports the mailbox it was put into") {
	port.takePort();
	CAN_msg_header_tx header;
	header.id = 0x30;
	header.length = 1;
	uint8_t data[8] = {0x42};
	uint32_t first = 0, second = 0;

	REQUIRE(port.sendMessage(&header, data, &first));
	REQUIRE(port.sendMessage(&header, data, &second));

	CHECK(first == CAN_TX_MAILBOX0);
	CHECK(second == CAN_TX_MAILBOX1);
}

TEST_CASE_FIXTURE(CanPortFixture, "a frame is sent into a mailbox that a completed frame freed") {
	fillMailboxes();
	REQUIRE(HostTarget::canTxComplete(1));

	CHECK(send(4));

	REQUIRE(HostTarget::canTxFrames().size() == 4);
	CHECK(HostTarget::canTxFrames()[3].mailbox == 1);
}

TEST_CASE_FIXTURE(CanPortFixture, "a frame is rejected when CAN1 is in the error state") {
	port.takePort();
	HostTarget::canFailNextModeChange();
	port.stop();
	REQUIRE(hcan1.State == HAL_CAN_STATE_ERROR);

	CHECK_FALSE(send(0x10));
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(CanPortFixture, "pending frames are aborted and the new frame is sent when no frame was acknowledged for more than 20 ms") {
	port.takePort();
	HostPlatform::advanceMs(100);
	REQUIRE(send(0x10));
	REQUIRE(HostTarget::canTxComplete(0)); // last acknowledged frame at 100 ms
	for (uint32_t id = 1; id <= 3; id++) {
		REQUIRE(send(id));
	}
	HostPlatform::advanceMs(21);

	CHECK(send(4));

	REQUIRE(HostTarget::canTxFrames().size() == 5);
	CHECK(HostTarget::canTxFrames()[1].result == HostTarget::CanTxResult::Aborted);
	CHECK(HostTarget::canTxFrames()[2].result == HostTarget::CanTxResult::Aborted);
	CHECK(HostTarget::canTxFrames()[3].result == HostTarget::CanTxResult::Aborted);
	CHECK(HostTarget::canTxFrames()[4].id == 4);
	CHECK(HostTarget::canTxFrames()[4].result == HostTarget::CanTxResult::Pending);
}

TEST_CASE_FIXTURE(CanPortFixture, "pending frames are not aborted within 20 ms of the last acknowledged frame") {
	port.takePort();
	HostPlatform::advanceMs(100);
	REQUIRE(send(0x10));
	REQUIRE(HostTarget::canTxComplete(0));
	for (uint32_t id = 1; id <= 3; id++) {
		REQUIRE(send(id));
	}
	HostPlatform::advanceMs(20);

	CHECK_FALSE(send(4));

	CHECK(HostTarget::canTxFrames().size() == 4);
	CHECK(HostTarget::canTxPending() == 0b111);
	CHECK(halCallCount("HAL_CAN_AbortTxRequest") == 1); // only the one of takePort -> start -> setSpeedPreset
}

TEST_CASE_FIXTURE(CanPortFixture, "a transmit completion of the port restarts the 20 ms abort timer") {
	fillMailboxes();
	HostPlatform::advanceMs(100);
	port.canTxCpltCallback(&canport, 0); // as from HAL_CAN_TxMailbox0CompleteCallback
	HostPlatform::advanceMs(15);

	CHECK_FALSE(send(4));
	CHECK(HostTarget::canTxPending() == 0b111);
}

TEST_CASE_FIXTURE(CanPortFixture, "a transmit completion of another port does not restart the 20 ms abort timer") {
	fillMailboxes();
	HostPlatform::advanceMs(100);
	port.canTxCpltCallback(&HostTarget::canPort(), 0); // another CANPort object
	HostPlatform::advanceMs(15);

	CHECK(send(4));
	CHECK(HostTarget::canTxFrames()[0].result == HostTarget::CanTxResult::Aborted);
}

TEST_CASE_FIXTURE(CanPortFixture, "characterization: after an idle bus a burst of four frames aborts the three frames queued just before (NEEDS REVIEW 118)") {
	// CANPort2B.cpp:202-205 measure the 20 ms from the last acknowledged frame (lastSentTime), not from when the frames
	// were queued: after more than 20 ms without traffic the fourth frame of a burst aborts the first three at once
	port.takePort();
	HostPlatform::advanceMs(1000);
	for (uint32_t id = 1; id <= 3; id++) {
		REQUIRE(send(id));
	}

	CHECK(send(4));

	REQUIRE(HostTarget::canTxFrames().size() == 4);
	CHECK(HostTarget::canTxFrames()[0].result == HostTarget::CanTxResult::Aborted);
	CHECK(HostTarget::canTxFrames()[1].result == HostTarget::CanTxResult::Aborted);
	CHECK(HostTarget::canTxFrames()[2].result == HostTarget::CanTxResult::Aborted);
	CHECK(HostTarget::canTxFrames()[3].time == 1000);
}

// ---------------------------------------------------------------- waiting for a free mailbox (port semaphore)

TEST_CASE_FIXTURE(CanPortFixture, "a frame sent from a task while all mailboxes are pending waits for a free mailbox and is sent when one completes" *
									  KNOWN_ISSUE("CANPort2B.cpp:207-214 wait for the port semaphore when no mailbox is free, but the "
												  "semaphore starts available (CAN.h:191 BinarySemaphore(true)) and the failure path gives "
												  "it back (CANPort2B.cpp:220-221): takeSemaphore succeeds at once, HAL_CAN_AddTxMessage "
												  "fails and sendMessage returns false without waiting for the transmit complete callback "
												  "that gives the semaphore (bug 110). The first CHECK fails")) {
	fillMailboxes();
	bool result = false;
	BodyTask task([&] {
		CAN_tx_msg msg = frame(4, {4});
		result = port.sendMessage(msg);
	});
	HostRtos::makeResumable(task.GetHandle());

	HostRtos::runUntilBlocked(task);
	const bool waited = !task.done;
	REQUIRE(HostTarget::canTxComplete(0));
	HostRtos::runUntilBlocked(task);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(waited);
	CHECK(result);
	CHECK(HostTarget::canTxFrames().size() == 4);
}

TEST_CASE_FIXTURE(CanPortFixture, "while the port semaphore is taken a send from a task waits until a mailbox completes and then sends") {
	fillMailboxes();
	port.takeSemaphore(0); // semaphore unavailable (no assertion: its initial state is bug 110)
	bool result = false;
	BodyTask task([&] {
		CAN_tx_msg msg = frame(4, {4});
		result = port.sendMessage(msg);
	});
	HostRtos::makeResumable(task.GetHandle());

	HostRtos::runUntilBlocked(task);
	REQUIRE_FALSE(task.done);
	CHECK(port.isWaiting());

	REQUIRE(HostTarget::canTxComplete(2)); // transmit complete callback gives the semaphore
	HostRtos::runUntilBlocked(task);

	CHECK(task.done);
	CHECK(result);
	CHECK_FALSE(port.isWaiting());
	REQUIRE(HostTarget::canTxFrames().size() == 4);
	CHECK(HostTarget::canTxFrames()[3].mailbox == 2);
}

TEST_CASE_FIXTURE(CanPortFixture, "a send from a task gives up and reports failure when no mailbox becomes free while it waits") {
	fillMailboxes();
	port.takeSemaphore(0);
	bool result = true;
	BodyTask task([&] {
		CAN_tx_msg msg = frame(4, {4});
		result = port.sendMessage(msg);
	});
	HostRtos::makeResumable(task.GetHandle());
	HostRtos::runUntilBlocked(task);
	REQUIRE_FALSE(task.done);

	HostRtos::runUntilBlocked(task); // the wait times out (no mailbox completed)

	CHECK(task.done);
	CHECK_FALSE(result);
	CHECK_FALSE(port.isWaiting());
	CHECK(HostTarget::canTxFrames().size() == 3);
}

TEST_CASE_FIXTURE(CanPortFixture, "aborting the transmit requests ends the wait of a send from a task, which then sends its frame") {
	fillMailboxes();
	port.takeSemaphore(0);
	bool result = false;
	BodyTask task([&] {
		CAN_tx_msg msg = frame(4, {4});
		result = port.sendMessage(msg);
	});
	HostRtos::makeResumable(task.GetHandle());
	HostRtos::runUntilBlocked(task);
	REQUIRE_FALSE(task.done);

	port.abortTxRequests(); // abort callbacks give the semaphore
	HostRtos::runUntilBlocked(task);

	CHECK(task.done);
	CHECK(result);
	CHECK(HostTarget::canTxFrames()[0].result == HostTarget::CanTxResult::Aborted);
	CHECK(HostTarget::canTxFrames().back().id == 4);
}

TEST_CASE_FIXTURE(CanPortFixture, "a transmit callback of another port does not end the wait of a send") {
	fillMailboxes();
	port.takeSemaphore(0);
	BodyTask task([&] {
		CAN_tx_msg msg = frame(4, {4});
		port.sendMessage(msg);
	});
	HostRtos::makeResumable(task.GetHandle());
	HostRtos::runUntilBlocked(task);
	REQUIRE(port.isWaiting());

	port.canTxCpltCallback(&HostTarget::canPort(), 0);
	port.canTxAbortCallback(&HostTarget::canPort(), 0);

	CHECK(port.isWaiting());
	CHECK_FALSE(takeSemaphoreNow());
}

TEST_CASE_FIXTURE(CanPortFixture, "a transmit error of the port releases the port semaphore") {
	const uint32_t codes[] = {HAL_CAN_ERROR_TX_ALST0, HAL_CAN_ERROR_TX_TERR0, HAL_CAN_ERROR_TX_ALST1,
							  HAL_CAN_ERROR_TX_TERR1, HAL_CAN_ERROR_TX_ALST2, HAL_CAN_ERROR_TX_TERR2};
	for (uint32_t code : codes) {
		CAPTURE(code);
		port.takeSemaphore(0);

		port.canErrorCallback(&canport, code | HAL_CAN_ERROR_ACK);

		CHECK(takeSemaphoreNow());
	}
}

TEST_CASE_FIXTURE(CanPortFixture, "an error without a transmit error does not release the port semaphore") {
	port.takeSemaphore(0);

	port.canErrorCallback(&canport, HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_ACK);

	CHECK_FALSE(takeSemaphoreNow());
}

TEST_CASE_FIXTURE(CanPortFixture, "a transmit error of another port does not release the port semaphore") {
	port.takeSemaphore(0);

	port.canErrorCallback(&HostTarget::canPort(), HAL_CAN_ERROR_TX_TERR0);

	CHECK_FALSE(takeSemaphoreNow());
}

TEST_CASE_FIXTURE(CanPortFixture, "a failed transmission reported by the HAL ends the wait of a send from a task") {
	fillMailboxes();
	port.takeSemaphore(0);
	bool result = true;
	BodyTask task([&] {
		CAN_tx_msg msg = frame(4, {4});
		result = port.sendMessage(msg);
	});
	HostRtos::makeResumable(task.GetHandle());
	HostRtos::runUntilBlocked(task);
	REQUIRE_FALSE(task.done);
	CAN1->MCR |= 0x00000010U; // CAN_MCR_NART (no automatic retransmission): a failed attempt completes the request
	REQUIRE(HostTarget::canTxError(0));

	HostRtos::runUntilBlocked(task);

	CHECK(task.done);
	CHECK(result); // mailbox 0 is free again
	CHECK(HostTarget::canTxFrames()[0].result == HostTarget::CanTxResult::Failed);
}

// ---------------------------------------------------------------- commands

TEST_CASE_FIXTURE(CanPortFixture, "the speed command reads the speed preset") {
	port.setSpeedPreset(2);

	CHECK(CommandHelpers::get(port, "speed") == 2);
}

TEST_CASE_FIXTURE(CanPortFixture, "the speed command sets the speed preset and its bit timing") {
	CHECK(cmd("speed", CMDtype::set, 5) == CommandStatus::OK);

	CHECK(port.getSpeedPreset() == 5);
	CHECK(CAN1->BTR == PRESET_BTR[5]);
}

TEST_CASE_FIXTURE(CanPortFixture, "the speed command lists the presets with their index") {
	std::vector<CommandReply> replies;

	CHECK(cmd("speed", CMDtype::info, 0, &replies) == CommandStatus::OK);

	std::vector<std::string> texts;
	for (const CommandReply &r : replies) {
		texts.push_back(r.reply);
	}
	CHECK(texts == std::vector<std::string>{"50k:0", "100k:1", "125k:2", "250k:3", "500k:4", "1000k:5"});
}

TEST_CASE_FIXTURE(CanPortFixture, "characterization: the speed command accepts a preset that does not exist without changing the speed (NEEDS REVIEW 120)") {
	// CANPort2B.cpp:350-351 ignore the result of setSpeedPreset (which ignores presets > 5, CANPort2B.cpp:137-138)
	CHECK(cmd("speed", CMDtype::set, 6) == CommandStatus::OK);

	CHECK(port.getSpeedPreset() == DEFAULT_PRESET);
	CHECK(CAN1->BTR == PRESET_BTR[DEFAULT_PRESET]);
}

TEST_CASE_FIXTURE(CanPortFixture, "a speed preset above 255 is ignored like any other preset that does not exist" *
									  KNOWN_ISSUE("CANPort2B.cpp:351 passes the int64_t command value to setSpeedPreset(uint8_t): 257 "
												  "is truncated to preset 1 (100 kbit/s) and passes the check `preset > 5` "
												  "(CANPort2B.cpp:137) (bug 111). The CHECK fails")) {
	cmd("speed", CMDtype::set, 257);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(port.getSpeedPreset() == DEFAULT_PRESET);
}

TEST_CASE_FIXTURE(CanPortFixture, "the len command reads the length of the next frames, 8 at first") {
	CHECK(CommandHelpers::get(port, "len") == 8);
}

TEST_CASE_FIXTURE(CanPortFixture, "the len command sets the length of the frames of the send command") {
	REQUIRE(cmd("len", CMDtype::set, 3) == CommandStatus::OK);

	REQUIRE(cmd("send", CMDtype::setat, 0x0807060504030201, nullptr, 0x123) == CommandStatus::OK);

	CHECK(CommandHelpers::get(port, "len") == 3);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK(HostTarget::canTxFrames()[0].dlc == 3);
	CHECK(HostTarget::canTxFrames()[0].payload() == std::vector<uint8_t>{1, 2, 3});
}

TEST_CASE_FIXTURE(CanPortFixture, "the len command limits the length to 8 bytes") {
	CHECK(cmd("len", CMDtype::set, 20) == CommandStatus::OK);

	CHECK(CommandHelpers::get(port, "len") == 8);
}

TEST_CASE_FIXTURE(CanPortFixture, "a length above 255 is limited to 8 bytes like any other length above 8" *
									  KNOWN_ISSUE("CANPort2B.cpp:376-377: handleGetSet stores the int64_t value in the uint8_t "
												  "nextLen before the limit to 8, so 260 becomes 4 (bug 112). The CHECK fails")) {
	cmd("len", CMDtype::set, 260);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(CommandHelpers::get(port, "len") == 8);
}

TEST_CASE_FIXTURE(CanPortFixture, "the send command sends the value as little endian data to the address") {
	port.takePort();

	CHECK(cmd("send", CMDtype::setat, 0x0807060504030201, nullptr, 0x2AB) == CommandStatus::OK);

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const HostTarget::CanTxFrame &sent = HostTarget::canTxFrames()[0];
	CHECK(sent.id == 0x2AB);
	CHECK_FALSE(sent.extId);
	CHECK_FALSE(sent.rtr);
	CHECK(sent.dlc == 8);
	CHECK(sent.value() == 0x0807060504030201U);
}

TEST_CASE_FIXTURE(CanPortFixture, "the send command starts a port that is not running") {
	port.setSilentMode(true);
	REQUIRE(hcan1.State == HAL_CAN_STATE_READY);

	CHECK(cmd("send", CMDtype::setat, 1, nullptr, 0x10) == CommandStatus::OK);

	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
	CHECK_FALSE(silentPinHigh());
	CHECK(CAN1->IER != 0);
	REQUIRE(HostTarget::canTxFrames().size() == 1);
}

TEST_CASE_FIXTURE(CanPortFixture, "the send command starts a port again after it was stopped") {
	port.start();
	port.stop();
	REQUIRE(hcan1.State == HAL_CAN_STATE_READY);

	cmd("send", CMDtype::setat, 1, nullptr, 0x10);

	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(CanPortFixture, "the send command does not restart a running port") {
	port.takePort();
	HostTarget::canClearHalCalls();

	cmd("send", CMDtype::setat, 1, nullptr, 0x10);

	CHECK(halCallCount("HAL_CAN_Start") == 0);
	CHECK(halCallCount("HAL_CAN_AddTxMessage") == 1);
}

TEST_CASE_FIXTURE(CanPortFixture, "characterization: the send command reports OK when the frame can not be sent (NEEDS REVIEW 121)") {
	// CANPort2B.cpp:369 ignores the result of sendMessage
	fillMailboxes();

	CHECK(cmd("send", CMDtype::setat, 1, nullptr, 0x10) == CommandStatus::OK);

	CHECK(HostTarget::canTxFrames().size() == 3);
}

TEST_CASE_FIXTURE(CanPortFixture, "characterization: the send command sends an address above 0x7FF as a standard frame with its lowest 11 bits (NEEDS REVIEW 122)") {
	// CANPort2B.cpp:365-368 never set extId; HAL_CAN_AddTxMessage writes StdId << 21 unmasked (stm32f4xx_hal_can.c)
	port.takePort();

	cmd("send", CMDtype::setat, 1, nullptr, 0x12345);

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK_FALSE(HostTarget::canTxFrames()[0].extId);
	CHECK(HostTarget::canTxFrames()[0].id == (0x12345U & 0x7FF));
}

TEST_CASE_FIXTURE(CanPortFixture, "the send command without an address is not found") {
	port.takePort();

	CHECK(cmd("send", CMDtype::set, 1) == CommandStatus::NOT_FOUND);
	CHECK(HostTarget::canTxFrames().empty());
}

TEST_CASE_FIXTURE(CanPortFixture, "a command id the CAN port does not know is not found") {
	ParsedCommand parsed;
	parsed.cmdId = 99;
	parsed.type = CMDtype::get;
	parsed.target = &port;
	std::vector<CommandReply> replies;

	CHECK(port.command(parsed, replies) == CommandStatus::NOT_FOUND);
	CHECK(replies.empty());
}

TEST_CASE_FIXTURE(CanPortFixture, "the CAN speed can be read and set with command lines on the CDC port") {
	FFBoardMain main;
	CommandHelpers::CdcConsole console(main);

	CHECK(console.send("can.0.speed?\n") == "[can.0.speed?|4]\n");
	CHECK(console.send("can.0.speed=3\n") == "[can.0.speed=3|OK]\n");
	CHECK(port.getSpeedPreset() == 3);
}

TEST_CASE_FIXTURE(CanPortFixture, "a CAN frame can be sent with a command line on the CDC port") {
	FFBoardMain main;
	CommandHelpers::CdcConsole console(main);

	CHECK(console.send("can.0.send=258?291\n") == "[can.0.send=258?291|OK]\n");

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	CHECK(HostTarget::canTxFrames()[0].id == 291);
	CHECK(HostTarget::canTxFrames()[0].value() == 258);
}

// ---------------------------------------------------------------- flash

TEST_CASE_FIXTURE(CanPortFixture, "saving writes the speed preset to ADR_CANCONF1") {
	port.setSpeedPreset(2);

	port.saveFlash();

	CHECK(HostPlatform::flash().count(ADR_CANCONF) == 1);
	CHECK(HostPlatform::flash()[ADR_CANCONF] == 2);
}

TEST_CASE_FIXTURE(CanPortFixture, "restoring reads the speed preset from ADR_CANCONF1 and applies its bit timing") {
	HostPlatform::flash()[ADR_CANCONF] = 1;

	port.restoreFlash();

	CHECK(port.getSpeedPreset() == 1);
	CHECK(CAN1->BTR == PRESET_BTR[1]);
}

TEST_CASE_FIXTURE(CanPortFixture, "restoring uses only the lowest three bits of the stored value") {
	HostPlatform::flash()[ADR_CANCONF] = 0xFFF8 | 3;

	port.restoreFlash();

	CHECK(port.getSpeedPreset() == 3);
}

TEST_CASE_FIXTURE(CanPortFixture, "a stored preset that does not exist is ignored when restoring") {
	HostPlatform::flash()[ADR_CANCONF] = 6;

	port.restoreFlash();

	CHECK(port.getSpeedPreset() == DEFAULT_PRESET);
}

TEST_CASE_FIXTURE(CanPortFixture, "restoring without a stored value keeps the speed and does not touch CAN1") {
	HostPlatform::flash().erase(ADR_CANCONF);

	port.restoreFlash();

	CHECK(port.getSpeedPreset() == DEFAULT_PRESET);
	CHECK(HostTarget::canHalCalls().empty());
}

TEST_CASE_FIXTURE(CanPortFixture, "a saved speed preset is restored") {
	port.setSpeedPreset(5);
	port.saveFlash();
	port.setSpeedPreset(0);

	port.restoreFlash();

	CHECK(port.getSpeedPreset() == 5);
}

} // TEST_SUITE
