/*
 * Smoke tests of the CAN infrastructure of P7 (batch 0): the global `canport` is the real CANPort_2B on hcan1 (like
 * cpp_target_config.cpp) on the fake HAL CAN (platform/src/host_target_can.cpp, control in support/HostTarget.h), the
 * CAN main classes CanBridge and CANInputMain, and SerialFFB commands through the CDC command interface
 * (support/CommandHelpers.h). The detailed tests of these modules are in test_canport2b.cpp, test_canbridge.cpp,
 * test_caninputmain.cpp and test_serial_ffb.cpp.
 *
 * Host-vs-target notes: the fake acknowledges mode changes at once and completes transmissions only when the test
 * says so (canTxComplete); interrupts are raised right after the event that causes them.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "CommandHelpers.h"
#include "InputSources.h"
#include "TestHelpers.h"

#include "CANPort2B.h"
#include "CanBridge.h"
#include "CanInputMain.h"
#include "EffectsCalculator.h"
#include "SerialFFB.h"
#include "cpp_target_config.h"

#include <memory>
#include <string>
#include <vector>

namespace {

/** Records every CAN callback of global_callbacks.cpp that reaches the CAN handlers */
struct RecordingCanHandler : CanHandler {
	struct Call {
		std::string what;
		CANPort *port = nullptr;
		uint32_t arg = 0;
		CAN_rx_msg msg{};
	};
	std::vector<Call> calls;

	void canRxPendCallback(CANPort *port, CAN_rx_msg &msg) override { calls.push_back({"rx", port, 0, msg}); }
	void canErrorCallback(CANPort *port, uint32_t errcode) override { calls.push_back({"error", port, errcode, {}}); }
	void canRxFullCallback(CANPort *port, uint32_t fifo) override { calls.push_back({"full", port, fifo, {}}); }
	void canTxCpltCallback(CANPort *port, uint32_t mailbox) override { calls.push_back({"txcplt", port, mailbox, {}}); }
	void canTxAbortCallback(CANPort *port, uint32_t mailbox) override { calls.push_back({"txabort", port, mailbox, {}}); }
};

CAN_tx_msg frame(uint32_t id, std::vector<uint8_t> bytes) {
	CAN_tx_msg msg;
	msg.header.id = id;
	msg.header.length = static_cast<uint32_t>(bytes.size());
	std::copy(bytes.begin(), bytes.end(), msg.data);
	return msg;
}

CAN_filter exactFilter(uint32_t id) {
	CAN_filter f;
	f.filter_id = id;
	f.filter_mask = 0x7FF;
	return f;
}

constexpr uint32_t PORT_NOTIFICATIONS = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO0_FULL |
										CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN |
										CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_TX_MAILBOX_EMPTY; // CANPort2B.cpp:106
constexpr uint32_t ESR_BOFF = 0x00000004U; // stm32f407xx.h CAN_ESR_BOFF

} // namespace

TEST_SUITE("CanSetup") {

// ---------------------------------------------------------------- CANPort_2B on the fake HAL CAN

TEST_CASE_FIXTURE(TargetFixture, "taking the CAN port starts CAN1 with the notifications of the port at 500 kbit/s") {
	REQUIRE(&canport == static_cast<CANPort *>(&HostTarget::canPort2B()));
	REQUIRE(hcan1.State == HAL_CAN_STATE_READY); // MX_CAN1_Init

	canport.takePort();

	CHECK(hcan1.State == HAL_CAN_STATE_LISTENING);
	CHECK(CAN1->IER == PORT_NOTIFICATIONS);
	CHECK(CAN1->BTR == 0x001a0005U); // preset 4 "500k" of cpp_target_config.cpp
}

TEST_CASE_FIXTURE(TargetFixture, "a sent frame occupies a transmit mailbox until the bus acknowledges it") {
	canport.takePort();
	CAN_tx_msg msg = frame(0x123, {0x11, 0x22, 0x33});

	REQUIRE(canport.sendMessage(msg));

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const HostTarget::CanTxFrame &sent = HostTarget::canTxFrames()[0];
	CHECK(sent.id == 0x123);
	CHECK_FALSE(sent.extId);
	CHECK_FALSE(sent.rtr);
	CHECK(sent.payload() == std::vector<uint8_t>{0x11, 0x22, 0x33});
	CHECK(HostTarget::canTxPending() == 0b001);
	REQUIRE(HostTarget::canTxComplete(sent.mailbox));
	CHECK(HostTarget::canTxPending() == 0);
	CHECK(HostTarget::canTxFrames()[0].result == HostTarget::CanTxResult::Sent);
}

TEST_CASE_FIXTURE(TargetFixture, "the transmit complete interrupt reaches the CAN handlers with the mailbox index") {
	RecordingCanHandler handler;
	canport.takePort();
	CAN_tx_msg first = frame(0x10, {1});
	CAN_tx_msg second = frame(0x11, {2});
	REQUIRE(canport.sendMessage(first));
	REQUIRE(canport.sendMessage(second));

	REQUIRE(HostTarget::canTxComplete(1));

	REQUIRE(handler.calls.size() == 1);
	CHECK(handler.calls[0].what == "txcplt");
	CHECK(handler.calls[0].port == &canport);
	CHECK(handler.calls[0].arg == 1);
}

TEST_CASE_FIXTURE(TargetFixture, "a frame is rejected while all three transmit mailboxes are pending") {
	canport.takePort();
	for (uint32_t id = 1; id <= 3; id++) {
		CAN_tx_msg msg = frame(id, {0});
		REQUIRE(canport.sendMessage(msg));
	}
	REQUIRE(HostTarget::canTxPending() == 0b111);
	CAN_tx_msg fourth = frame(4, {0});

	CHECK_FALSE(canport.sendMessage(fourth));
	CHECK(HostTarget::canTxFrames().size() == 3);
}

TEST_CASE_FIXTURE(TargetFixture, "aborting the transmit requests of the port aborts a frame that is still pending after another mailbox completed" *
									 KNOWN_ISSUE("CANPort2B.cpp:233 (and :241) clear `txMailboxes &= ~mailbox` with the mailbox index 0..2 that "
												 "global_callbacks.cpp:155-183 passes, while txMailboxes holds the bits CAN_TX_MAILBOX0..2 "
												 "(1 << index) of HAL_CAN_AddTxMessage: completing mailbox 1 clears the bit of mailbox 0, "
												 "so abortTxRequests skips the frame still pending there (bug 106). The CHECK fails")) {
	canport.takePort();
	CAN_tx_msg first = frame(0x10, {1});
	CAN_tx_msg second = frame(0x11, {2});
	REQUIRE(canport.sendMessage(first));  // mailbox 0
	REQUIRE(canport.sendMessage(second)); // mailbox 1
	REQUIRE(HostTarget::canTxComplete(1));

	canport.abortTxRequests();
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(HostTarget::canTxFrames()[0].result == HostTarget::CanTxResult::Aborted);
}

TEST_CASE_FIXTURE(TargetFixture, "starting the CAN port reports success" *
									 KNOWN_ISSUE("CANPort2B.cpp:107-108: start() first calls setSpeedPreset, which stops and starts "
												 "CAN1 (CANPort2B.cpp:142-147), then calls HAL_CAN_Start again; the HAL rejects the "
												 "second start (State LISTENING: HAL_ERROR, ErrorCode |= HAL_CAN_ERROR_NOT_READY, "
												 "stm32f4xx_hal_can.c:1034-1076), so start() returns false although the port runs "
												 "(bug 107). The CHECK fails")) {
	bool started = HostTarget::canPort2B().start();
	REQUIRE_ALL_HAL_MODELLED();
	REQUIRE(hcan1.State == HAL_CAN_STATE_LISTENING);

	CHECK(started);
}

TEST_CASE_FIXTURE(TargetFixture, "a filter added while all 14 filter banks of the port are in use is rejected" *
									 KNOWN_ISSUE("CANPort2B.cpp:295-320: when banks 0..13 are all used, the search loop ends with "
												 "foundId = 13 (it is assigned before the 'existing' check), so the new filter "
												 "overwrites bank 13 and 13 is returned instead of -1 as documented at "
												 "CANPort2B.cpp:259 (bug 108). The CHECK fails")) {
	canport.takePort();
	for (int32_t i = 0; i < CANPort_2B::slaveFilterStart; i++) {
		REQUIRE(canport.addCanFilter(exactFilter(0x100 + i)) == i);
	}

	int32_t bank = canport.addCanFilter(exactFilter(0x200));
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(bank == -1);
}

TEST_CASE_FIXTURE(TargetFixture, "a received frame that matches a filter of the port reaches the CAN handlers") {
	RecordingCanHandler handler;
	canport.takePort();
	REQUIRE(canport.addCanFilter(exactFilter(0x123)) >= 0);

	REQUIRE(HostTarget::canReceive(0x123, {0xAA, 0xBB}) == 1);

	REQUIRE(handler.calls.size() == 1);
	CHECK(handler.calls[0].what == "rx");
	CHECK(handler.calls[0].port == &canport);
	CHECK(handler.calls[0].msg.header.id == 0x123);
	CHECK(handler.calls[0].msg.header.length == 2);
	CHECK(handler.calls[0].msg.data[0] == 0xAA);
	CHECK(handler.calls[0].msg.data[1] == 0xBB);
	CHECK(HostTarget::canRxPending(CAN_RX_FIFO0) == 0);
}

TEST_CASE_FIXTURE(TargetFixture, "a received frame that no filter accepts is not stored") {
	RecordingCanHandler handler;
	canport.takePort();
	REQUIRE(canport.addCanFilter(exactFilter(0x123)) >= 0);

	CHECK(HostTarget::canReceive(0x124, {0xAA}) == 0);

	CHECK(handler.calls.empty());
	CHECK(HostTarget::canRxPending(CAN_RX_FIFO0) == 0);
}

TEST_CASE_FIXTURE(TargetFixture, "characterization: bus-off is not reported to the CAN handlers because the CAN1 status change interrupt is not enabled (NEEDS REVIEW 116)") {
	// CANPort_2B::start enables CAN_IT_ERROR | CAN_IT_BUSOFF (CANPort2B.cpp:106), but HAL_CAN_MspInit enables only
	// CAN1_TX_IRQn and CAN1_RX0_IRQn in the NVIC (Targets/F407VG/Core/Src/stm32f4xx_hal_msp.c:291-294; no
	// CAN1_SCE_IRQHandler in stm32f4xx_it.c), so HAL_CAN_ErrorCallback never sees bus-off on the F407
	RecordingCanHandler handler;
	canport.takePort();

	HostTarget::canSetErrorState(HostTarget::CanErrorState::BusOff);

	CHECK((CAN1->ESR & ESR_BOFF) != 0);
	CHECK(handler.calls.empty());
}

TEST_CASE_FIXTURE(TargetFixture, "with the status change interrupt enabled bus-off reaches the CAN handlers as HAL_CAN_ERROR_BOF") {
	RecordingCanHandler handler;
	canport.takePort();
	HostTarget::canSetIrqLineEnabled(HostTarget::CanIrqLine::Sce, true); // a board whose NVIC enables CAN1_SCE_IRQn

	HostTarget::canSetErrorState(HostTarget::CanErrorState::BusOff);

	REQUIRE(handler.calls.size() == 1);
	CHECK(handler.calls[0].what == "error");
	CHECK(handler.calls[0].port == &canport);
	CHECK((handler.calls[0].arg & HAL_CAN_ERROR_BOF) != 0);
	CHECK(hcan1.ErrorCode == 0); // cleared by HAL_CAN_ErrorCallback of global_callbacks.cpp:190
}

// ---------------------------------------------------------------- CAN main classes and SerialFFB

TEST_CASE_FIXTURE(TargetFixture, "the CAN bridge sends a frame received from the bus to the host in GVRET format") {
	CanBridge bridge;
	std::string gvretOn = "\xE7";
	uint32_t len = static_cast<uint32_t>(gvretOn.size());
	bridge.cdcRcv(gvretOn.data(), &len); // SavvyCAN switches the bridge to the GVRET binary protocol
	HostUsb::state().cdcTx.clear();
	REQUIRE(HostTarget::canReceive(0x123, {0x11, 0x22}) == 1);

	bridge.update();

	// 0xF1, 0 (received frame), timestamp (4 bytes LE, 0: TTCM off), id (4 bytes LE), length | bus 0, data, 0
	const std::string expected("\xF1\x00\x00\x00\x00\x00\x23\x01\x00\x00\x02\x11\x22\x00", 14);
	CHECK(HostUsb::state().cdcTx == expected);
}

TEST_CASE_FIXTURE(TargetFixture, "the CAN input main class sends a changed button state as a CAN frame with its button id") {
	CANInputMain main;
	InputSources::useMockRegistries(main);
	REQUIRE(CommandHelpers::run(main, "btntypes", CMDtype::set, 1 << MockButtonSource::SELECTION_ID) == CommandStatus::OK);
	REQUIRE(MockButtonSource::last() != nullptr);
	MockButtonSource::last()->buttons = 0x0201;

	main.sendReport();

	REQUIRE(HostTarget::canTxFrames().size() == 1);
	const HostTarget::CanTxFrame &sent = HostTarget::canTxFrames()[0];
	CHECK(sent.id == 100); // default buttons_id (CanInputMain.h)
	CHECK(sent.dlc == 8);
	CHECK(sent.value() == 0x0201);
}

TEST_CASE_FIXTURE(TargetFixture, "a SerialFFB command line sent on the CDC port creates an effect") {
	FFBoardMain main;
	auto effects = std::make_shared<EffectsCalculator>();
	SerialFFB fxm(effects);
	CommandHelpers::CdcConsole console(main);

	std::string reply = console.send("fxm.0.new=1\n"); // 1 = FFB_EFFECT_CONSTANT

	CHECK(reply == "[fxm.0.new=1|0]\n"); // index of the new effect
	CHECK(effects->effects[0].type == 1);
}

} // TEST_SUITE
