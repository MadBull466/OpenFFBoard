// P12 review witnesses, CAN (Codex review of P12, P12C-T5-001 and P12C-T5-002).
#include "CanTestSupport.h"

TEST_CASE_FIXTURE(CanFixture, "RV11 the transmit time stamp is the timer value at the start of the frame, on CAN1 and on CAN2 (P12C-T5-001)") {
	// RM0090 CAN_TDTxR TIME: "the 16-bit timer value captured at the SOF transmission".
	for (unsigned n = 0; n < 2; ++n) {
		CAN_HandleTypeDef h{};
		h.Instance = n ? CAN2 : CAN1;
		h.Init.Prescaler = 6;
		h.Init.Mode = CAN_MODE_NORMAL;
		h.Init.SyncJumpWidth = CAN_SJW_1TQ;
		h.Init.TimeSeg1 = CAN_BS1_11TQ;
		h.Init.TimeSeg2 = CAN_BS2_2TQ;
		h.Init.TimeTriggeredMode = ENABLE;
		REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
		REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
		hostCanSetFrameTicks(1000);
		hostCanSetTimer(n, 0x1234);
		CAN_TxHeaderTypeDef t{};
		t.StdId = 0x123;
		t.DLC = 8;
		uint8_t d[8]{};
		uint32_t m = 0;
		REQUIRE(HAL_CAN_AddTxMessage(&h, &t, d, &m) == HAL_OK);
		REQUIRE(HAL_CAN_IsTxMessagePending(&h, m) == 1u);
		hostCanSetTimer(n, 0x5678);  // the timer moves on while the frame is on the bus
		hostTickSet(hostTickPeek() + 1100);
		hostCanPoll(n);
		REQUIRE(hostCanTxCount(n) == 1u);
		CHECK(HAL_CAN_GetTxTimestamp(&h, m) == 0x1234u);
	}
}

TEST_CASE_FIXTURE(CanFixture, "RV12 each of the three transmit mailboxes sends its own payload (P12C-T5-002)") {
	// A payload copied wrongly for one mailbox escaped the old tests, which compared the data registers of mailbox 0 only.
	CAN_HandleTypeDef h{};
	h.Instance = CAN1;
	h.Init.Prescaler = 6;
	h.Init.TimeSeg1 = CAN_BS1_11TQ;
	h.Init.TimeSeg2 = CAN_BS2_2TQ;
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	hostCanHoldBus(0, 1);
	CAN_TxHeaderTypeDef t{};
	t.DLC = 8;
	uint8_t d[3][8];
	for (unsigned i = 0; i < 3; ++i)
		for (unsigned b = 0; b < 8; ++b) d[i][b] = static_cast<uint8_t>(0x11u * (b + 1u) + i);  // distinct bytes per mailbox
	uint32_t mb = 0;
	for (unsigned i = 0; i < 3; ++i) {
		t.StdId = 0x100 + i;
		REQUIRE(HAL_CAN_AddTxMessage(&h, &t, d[i], &mb) == HAL_OK);
		REQUIRE(mb == (1u << i));
	}
	for (unsigned i = 0; i < 3; ++i) {
		const uint32_t lo = d[i][0] | (d[i][1] << 8) | (d[i][2] << 16) | (static_cast<uint32_t>(d[i][3]) << 24);
		const uint32_t hi = d[i][4] | (d[i][5] << 8) | (d[i][6] << 16) | (static_cast<uint32_t>(d[i][7]) << 24);
		CHECK(CAN1->sTxMailBox[i].TDLR == lo);
		CHECK(CAN1->sTxMailBox[i].TDHR == hi);
	}
	hostCanHoldBus(0, 0);
	hostCanPoll(0);
	REQUIRE(hostCanTxCount(0) == 3u);
	// the frames leave in identifier order: 0x100 (mailbox 0), 0x101 (mailbox 1), 0x102 (mailbox 2)
	for (unsigned i = 0; i < 3; ++i)
		for (unsigned b = 0; b < 8; ++b) CHECK(hostCanTxAt(0, i).data[b] == d[i][b]);
}
