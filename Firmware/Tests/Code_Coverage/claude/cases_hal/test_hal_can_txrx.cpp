// P12 T5 group CT: bxCAN transmit mailboxes and receive FIFOs through the real ST HAL (CT1-CT4).
// Oracles: UM1725 chapter 9 (HAL CAN), RM0090 chapter 32 (32.7.1 transmission, 32.7.3 reception, 32.7.4 filtering, registers 32.9).
// Register layouts (hand derived from RM0090): TIxR = STID[10:0] at bit 21 | EXID[28:0] at bit 3 | IDE bit 2 | RTR bit 1 | TXRQ bit 0;
// TDTxR = DLC[3:0] | TGT bit 8 | TIME[15:0] at bit 16; TDLxR / TDHxR = data bytes 0..3 / 4..7 (byte 0 in the low byte).
// TSR: RQCPx = bit 8*x, TXOKx = 8*x+1, ALSTx = 8*x+2, TERRx = 8*x+3, TMEx = 26+x. RFxR: FMP[1:0], FULL bit 3, FOVR bit 4, RFOM bit 5.
//
// NEEDS REVIEW 329 list (documents silent: cases named "characterization:"):
//  NEEDS REVIEW 329 AddTxMessage with all mailboxes full sets ErrorCode PARAM.
//  NEEDS REVIEW 329 GetRxMessage on an empty FIFO 0 / 1 returns HAL_ERROR with ErrorCode PARAM.
//  NEEDS REVIEW 329 a received DLC of 9..15 is reported as 8.
//  NEEDS REVIEW 329 the error code HAL_CAN_ERROR_NOT_INITIALIZED and HAL_ERROR returned by AddTxMessage / AbortTxRequest / GetRxMessage in a state other
//           than READY / LISTENING (UM1725 names the states required, not the return value or code); includes that the rejected call changes nothing.
#include "CanTestSupport.h"

#include <cstdint>
#include <cstring>

namespace {

const uint8_t ctD[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
const uint32_t ctTdl = 0x44332211u, ctTdh = 0x88776655u;

struct CtF : CanFixture {
	CAN_HandleTypeDef h{};
	CtF() {
		h.Instance = CAN1;
		h.Init.Prescaler = 6;
		h.Init.Mode = CAN_MODE_NORMAL;
		h.Init.SyncJumpWidth = CAN_SJW_1TQ;
		h.Init.TimeSeg1 = CAN_BS1_11TQ;
		h.Init.TimeSeg2 = CAN_BS2_2TQ;
		h.Init.AutoRetransmission = ENABLE;
	}
	unsigned idx() const { return canIndexOf(h.Instance); }
	CAN_TypeDef *can() const { return h.Instance; }
	void filterPassAll(uint32_t bank, uint32_t fifo) {
		CAN_FilterTypeDef f{};
		f.FilterBank = bank;
		f.FilterMode = CAN_FILTERMODE_IDMASK;
		f.FilterScale = CAN_FILTERSCALE_32BIT;
		f.FilterFIFOAssignment = fifo;
		f.FilterActivation = CAN_FILTER_ENABLE;
		f.SlaveStartFilterBank = 14;
		REQUIRE(HAL_CAN_ConfigFilter(&h, &f) == HAL_OK);
	}
	// 32 bit mask filter that accepts exactly the standard data frame `id` (mask covers the 11 bit id, IDE and RTR).
	void filterExact(uint32_t bank, uint32_t id, uint32_t fifo) {
		CAN_FilterTypeDef f{};
		f.FilterBank = bank;
		f.FilterIdHigh = id << 5;
		f.FilterMaskIdHigh = 0xFFE0u;
		f.FilterMaskIdLow = 0x6u;
		f.FilterMode = CAN_FILTERMODE_IDMASK;
		f.FilterScale = CAN_FILTERSCALE_32BIT;
		f.FilterFIFOAssignment = fifo;
		f.FilterActivation = CAN_FILTER_ENABLE;
		f.SlaveStartFilterBank = 14;
		REQUIRE(HAL_CAN_ConfigFilter(&h, &f) == HAL_OK);
	}
	// Instance CAN2 uses banks from 14 up.
	uint32_t bank0() const { return idx() == 0u ? 0u : 14u; }
	void up(bool start = true) {
		REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
		filterPassAll(bank0(), CAN_RX_FIFO0);
		if (start) REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	}
	static CAN_TxHeaderTypeDef stdHdr(uint32_t id, uint32_t dlc = 8, uint32_t rtr = CAN_RTR_DATA) {
		CAN_TxHeaderTypeDef t{};
		t.StdId = id;
		t.IDE = CAN_ID_STD;
		t.RTR = rtr;
		t.DLC = dlc;
		return t;
	}
	static CAN_TxHeaderTypeDef extHdr(uint32_t id, uint32_t dlc = 8, uint32_t rtr = CAN_RTR_DATA) {
		CAN_TxHeaderTypeDef t{};
		t.ExtId = id;
		t.IDE = CAN_ID_EXT;
		t.RTR = rtr;
		t.DLC = dlc;
		return t;
	}
	uint32_t add(const CAN_TxHeaderTypeDef &t, HAL_StatusTypeDef expect = HAL_OK) {
		uint32_t mb = 0xDEADu;
		CHECK(HAL_CAN_AddTxMessage(&h, &t, ctD, &mb) == expect);
		return mb;
	}
	uint32_t addStd(uint32_t id) { return add(stdHdr(id)); }
	static HostCanFrame rxf(uint32_t id, uint8_t ide, uint8_t rtr, uint8_t dlc, uint8_t d0 = 0) {
		HostCanFrame f{};
		f.id = id;
		f.ide = ide;
		f.rtr = rtr;
		f.dlc = dlc;
		for (unsigned i = 0; i < 8; ++i) f.data[i] = (uint8_t)(d0 + i);
		return f;
	}
	void hold(bool on) { hostCanHoldBus(idx(), on ? 1 : 0); }
};

}  // namespace

// ------------------------------------------------------------------ CT1 AddTxMessage

TEST_CASE_FIXTURE(CtF, "CT1 characterization: AddTxMessage is rejected with NOT_INITIALIZED in state RESET") {
	h.State = HAL_CAN_STATE_RESET;
	uint32_t mb = 0x55u;
	const CAN_TxHeaderTypeDef t = stdHdr(0x123);
	CHECK(HAL_CAN_AddTxMessage(&h, &t, ctD, &mb) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
	CHECK(mb == 0x55u);
	CHECK(hostCanTxCount(0) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT1 characterization: AddTxMessage is rejected with NOT_INITIALIZED in state ERROR") {
	up();
	h.State = HAL_CAN_STATE_ERROR;
	uint32_t mb = 0u;
	const CAN_TxHeaderTypeDef t = stdHdr(0x123);
	CHECK(HAL_CAN_AddTxMessage(&h, &t, ctD, &mb) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
	CHECK(hostCanTxCount(0) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT1 characterization: AddTxMessage is rejected in state SLEEP_ACTIVE") {
	up();
	h.State = HAL_CAN_STATE_SLEEP_ACTIVE;
	const CAN_TxHeaderTypeDef t = stdHdr(0x123);
	uint32_t mb = 0u;
	CHECK(HAL_CAN_AddTxMessage(&h, &t, ctD, &mb) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage is accepted in state READY and the request waits for Start") {
	up(false);
	REQUIRE(h.State == HAL_CAN_STATE_READY);
	CHECK(addStd(0x123) == CAN_TX_MAILBOX0);
	// Not started: the peripheral is still in initialization mode (RM0090 32.4.1), nothing is sent.
	CHECK(hostCanTxCount(0) == 0u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 1u);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	hostCanPoll(0);
	CHECK(hostCanTxCount(0) == 1u);
	CHECK(hostCanTxAt(0, 0).id == 0x123u);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage is accepted in state LISTENING and the frame is sent at once") {
	up();
	REQUIRE(h.State == HAL_CAN_STATE_LISTENING);
	CHECK(addStd(0x123) == CAN_TX_MAILBOX0);
	REQUIRE(hostCanTxCount(0) == 1u);
	const HostCanFrame f = hostCanTxAt(0, 0);
	CHECK(f.id == 0x123u);
	CHECK(f.ide == 0u);
	CHECK(f.rtr == 0u);
	CHECK(f.dlc == 8u);
	CHECK(std::memcmp(f.data, ctD, 8) == 0);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage reuses mailbox 0 when each frame has been sent") {
	up();
	CHECK(addStd(0x10) == CAN_TX_MAILBOX0);
	CHECK(addStd(0x20) == CAN_TX_MAILBOX0);
	CHECK(addStd(0x30) == CAN_TX_MAILBOX0);
	CHECK(hostCanTxCount(0) == 3u);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage fills the three mailboxes in the order 0, 1, 2 and returns bit masks") {
	up();
	hold(true);
	CHECK(addStd(0x10) == 1u);
	CHECK(addStd(0x20) == 2u);
	CHECK(addStd(0x30) == 4u);
	CHECK(CAN_TX_MAILBOX0 == 1u);
	CHECK(CAN_TX_MAILBOX1 == 2u);
	CHECK(CAN_TX_MAILBOX2 == 4u);
	CHECK(hostCanTxCount(0) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage with all mailboxes full returns HAL_ERROR and leaves the pending frames untouched") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	uint32_t mb = 0x77u;
	const CAN_TxHeaderTypeDef t = stdHdr(0x40);
	CHECK(HAL_CAN_AddTxMessage(&h, &t, ctD, &mb) == HAL_ERROR);
	CHECK(mb == 0x77u);  // pTxMailbox is not written
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 0u);
	CHECK((can()->sTxMailBox[0].TIR >> 21) == 0x10u + 0u);
	CHECK((can()->sTxMailBox[1].TIR >> 21) == 0x20u);
	CHECK((can()->sTxMailBox[2].TIR >> 21) == 0x30u);
}

TEST_CASE_FIXTURE(CtF, "CT1 characterization: AddTxMessage with all mailboxes full sets the error code PARAM") {
	// NEEDS REVIEW 329: UM1725 does not say which error code a full mailbox set gives; the HAL sets HAL_CAN_ERROR_PARAM.
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	const CAN_TxHeaderTypeDef t = stdHdr(0x40);
	uint32_t mb = 0;
	CHECK(HAL_CAN_AddTxMessage(&h, &t, ctD, &mb) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_PARAM) != 0u);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage uses mailbox 0 when it is the only free one") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_OK);
	REQUIRE(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 1u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) == 1u);
	CHECK(addStd(0x40) == CAN_TX_MAILBOX0);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 0u);
	CHECK((can()->sTxMailBox[0].TIR >> 21) == 0x40u);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage uses mailbox 1 when it is the only free one") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX1) == HAL_OK);
	REQUIRE(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 1u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX2) == 1u);
	CHECK(addStd(0x40) == CAN_TX_MAILBOX1);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 0u);
	CHECK((can()->sTxMailBox[1].TIR >> 21) == 0x40u);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage uses mailbox 2 when it is the only free one") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX2) == HAL_OK);
	REQUIRE(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 1u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1) == 1u);
	CHECK(addStd(0x40) == CAN_TX_MAILBOX2);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 0u);
	CHECK((can()->sTxMailBox[2].TIR >> 21) == 0x40u);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage takes the lowest empty mailbox when mailbox 1 was freed by an abort") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX1) == HAL_OK);
	CHECK(addStd(0x40) == CAN_TX_MAILBOX1);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage takes mailbox 0 when mailboxes 0 and 2 were freed (lowest empty)") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX2) == HAL_OK);
	CHECK(addStd(0x40) == CAN_TX_MAILBOX0);
	CHECK(addStd(0x50) == CAN_TX_MAILBOX2);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage writes a standard identifier into TIR bits 31:21") {
	up();
	hold(true);
	addStd(0x7FF);
	// STID at bit 21, IDE = 0, RTR = 0, TXRQ = 1 (pending).
	CHECK(can()->sTxMailBox[0].TIR == ((0x7FFu << 21) | 1u));
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage writes an extended identifier into TIR bits 31:3 with IDE set") {
	up();
	hold(true);
	add(extHdr(0x1FFFFFFFu));
	CHECK(can()->sTxMailBox[0].TIR == ((0x1FFFFFFFu << 3) | 4u | 1u));
	add(extHdr(0x12345678u));
	CHECK(can()->sTxMailBox[1].TIR == ((0x12345678u << 3) | 4u | 1u));
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage writes the RTR bit of a remote frame") {
	up();
	hold(true);
	add(stdHdr(0x100, 0, CAN_RTR_REMOTE));
	CHECK(can()->sTxMailBox[0].TIR == ((0x100u << 21) | 2u | 1u));
	add(extHdr(0x100, 0, CAN_RTR_REMOTE));
	CHECK(can()->sTxMailBox[1].TIR == ((0x100u << 3) | 4u | 2u | 1u));
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage ignores ExtId for a standard frame and StdId for an extended frame") {
	up();
	hold(true);
	CAN_TxHeaderTypeDef s = stdHdr(0x123);
	s.ExtId = 0x1FFFFFFFu;
	add(s);
	CAN_TxHeaderTypeDef e = extHdr(0x0ABCDEF0u);
	e.StdId = 0x7FFu;
	add(e);
	CHECK(can()->sTxMailBox[0].TIR == ((0x123u << 21) | 1u));
	CHECK(can()->sTxMailBox[1].TIR == ((0x0ABCDEF0u << 3) | 4u | 1u));
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage writes DLC 0 to 8 into TDTR bits 3:0") {
	up();
	for (uint32_t dlc = 0; dlc <= 8; ++dlc) {
		hostCanTxClear(0);
		add(stdHdr(0x100 + dlc, dlc));
		REQUIRE(hostCanTxCount(0) == 1u);
		CHECK(hostCanTxAt(0, 0).dlc == dlc);
		CHECK((can()->sTxMailBox[0].TDTR & 0xFu) == dlc);
	}
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage sets TDTR bit 8 (TGT) only when TransmitGlobalTime is enabled") {
	up();
	hold(true);
	CAN_TxHeaderTypeDef t = stdHdr(0x10, 8);
	t.TransmitGlobalTime = DISABLE;
	add(t);
	t.TransmitGlobalTime = ENABLE;
	t.StdId = 0x20;
	add(t);
	CHECK(can()->sTxMailBox[0].TDTR == 8u);
	CHECK(can()->sTxMailBox[1].TDTR == ((1u << 8) | 8u));
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage writes the eight data bytes into TDLR and TDHR, byte 0 in the low byte") {
	up();
	hold(true);
	addStd(0x10);
	CHECK(can()->sTxMailBox[0].TDLR == ctTdl);
	CHECK(can()->sTxMailBox[0].TDHR == ctTdh);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage requests the transmission only after the whole mailbox is written (sent frame is complete)") {
	up();
	// The model snapshots the mailbox when the frame is sent: a TXRQ before the data would send stale zero bytes.
	add(extHdr(0x1ABCDEF0u, 5));
	REQUIRE(hostCanTxCount(0) == 1u);
	const HostCanFrame f = hostCanTxAt(0, 0);
	CHECK(f.ide == 1u);
	CHECK(f.id == 0x1ABCDEF0u);
	CHECK(f.dlc == 5u);
	CHECK(std::memcmp(f.data, ctD, 8) == 0);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage sets TXRQ in TIR of a pending mailbox") {
	up();
	hold(true);
	addStd(0x10);
	CHECK((can()->sTxMailBox[0].TIR & 1u) == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT1 AddTxMessage on CAN2 sends through CAN2 and not CAN1") {
	h.Instance = CAN2;
	up();
	REQUIRE(h.State == HAL_CAN_STATE_LISTENING);
	CHECK(addStd(0x321) == CAN_TX_MAILBOX0);
	CHECK(hostCanTxCount(1) == 1u);
	CHECK(hostCanTxCount(0) == 0u);
	CHECK(hostCanTxAt(1, 0).id == 0x321u);
}

// ------------------------------------------------------------------ CT2 free level, pending, abort, priority

TEST_CASE_FIXTURE(CtF, "CT2 GetTxMailboxesFreeLevel is 3 with no pending frame") {
	up();
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 3u);
}

TEST_CASE_FIXTURE(CtF, "CT2 GetTxMailboxesFreeLevel counts 3, 2, 1, 0 as mailboxes fill") {
	up();
	hold(true);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 3u);
	addStd(0x10);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 2u);
	addStd(0x20);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 1u);
	addStd(0x30);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 GetTxMailboxesFreeLevel returns to 3 when the held frames are sent") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	hold(false);
	hostCanPoll(0);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 3u);
	CHECK(hostCanTxCount(0) == 2u);
}

TEST_CASE_FIXTURE(CtF, "CT2 GetTxMailboxesFreeLevel is 0 in states RESET and ERROR") {
	up();
	h.State = HAL_CAN_STATE_RESET;
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 0u);
	h.State = HAL_CAN_STATE_ERROR;
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 GetTxMailboxesFreeLevel works in state READY") {
	up(false);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 3u);
	addStd(0x10);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 2u);
}

TEST_CASE_FIXTURE(CtF, "CT2 IsTxMessagePending is 0 for every mailbox mask with no pending frame") {
	up();
	for (uint32_t m = 1; m <= 7; ++m) CHECK(HAL_CAN_IsTxMessagePending(&h, m) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 IsTxMessagePending reports a frame in mailbox 0 for every mask containing mailbox 0") {
	up();
	hold(true);
	addStd(0x10);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 1u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX1) == 0u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX2) == 0u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1) == 1u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) == 0u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX2) == 1u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, 7u) == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT2 IsTxMessagePending reports mailbox 1 and 2 separately") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_OK);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 0u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX1) == 1u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX2) == 0u);
	addStd(0x30);  // goes into mailbox 0
	addStd(0x40);  // goes into mailbox 2
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX2) == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT2 IsTxMessagePending is 0 after the frame has been sent") {
	up();
	addStd(0x10);
	CHECK(HAL_CAN_IsTxMessagePending(&h, 7u) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 IsTxMessagePending is 0 in states RESET and ERROR even with a pending frame") {
	up();
	hold(true);
	addStd(0x10);
	h.State = HAL_CAN_STATE_RESET;
	CHECK(HAL_CAN_IsTxMessagePending(&h, 7u) == 0u);
	h.State = HAL_CAN_STATE_ERROR;
	CHECK(HAL_CAN_IsTxMessagePending(&h, 7u) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 AbortTxRequest empties a pending mailbox so the frame is never sent") {
	up();
	hold(true);
	addStd(0x10);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_OK);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 0u);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 3u);
	hold(false);
	hostCanPoll(0);
	CHECK(hostCanTxCount(0) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 AbortTxRequest sets RQCP and leaves TXOK clear for an aborted pending mailbox") {
	up();
	hold(true);
	addStd(0x10);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_OK);
	// RM0090 TSR: the request completed (RQCP0) without success (TXOK0 = 0).
	CHECK((can()->TSR & 1u) == 1u);
	CHECK((can()->TSR & 2u) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 AbortTxRequest aborts only the selected mailboxes") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX1) == HAL_OK);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 1u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX1) == 0u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX2) == 1u);
	hold(false);
	hostCanPoll(0);
	REQUIRE(hostCanTxCount(0) == 2u);
	CHECK(hostCanTxAt(0, 0).id == 0x10u);
	CHECK(hostCanTxAt(0, 1).id == 0x30u);
}

TEST_CASE_FIXTURE(CtF, "CT2 AbortTxRequest of mailbox 2 alone leaves mailboxes 0 and 1") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX2) == HAL_OK);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX2) == 0u);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT2 AbortTxRequest of all three mailboxes empties them") {
	up();
	hold(true);
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) == HAL_OK);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 3u);
	CHECK(HAL_CAN_IsTxMessagePending(&h, 7u) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 AbortTxRequest of an empty mailbox returns HAL_OK and changes nothing") {
	up();
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX1) == HAL_OK);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 3u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	// A following frame still goes out normally (the abort request did not stay latched).
	addStd(0x10);
	CHECK(hostCanTxCount(0) == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT2 AbortTxRequest in state READY returns HAL_OK") {
	up(false);
	addStd(0x10);
	CHECK(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_OK);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 characterization: AbortTxRequest is rejected with NOT_INITIALIZED in states RESET and ERROR") {
	up();
	h.State = HAL_CAN_STATE_RESET;
	CHECK(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
	h.ErrorCode = 0;
	h.State = HAL_CAN_STATE_ERROR;
	CHECK(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 characterization: AbortTxRequest of a mailbox rejected by state does not abort the pending frame") {
	up();
	hold(true);
	addStd(0x10);
	h.State = HAL_CAN_STATE_RESET;
	CHECK(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_ERROR);
	h.State = HAL_CAN_STATE_LISTENING;
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT2 AbortTxRequest of a mailbox being sent lets the frame finish (RM0090 32.7.1 abort)") {
	up();
	hostCanSetFrameTicks(1000);
	const uint32_t t0 = hostTickPeek();
	addStd(0x10);  // starts now, done 1000 ticks later (the HAL's own polling moves the tick by a few)
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_OK);
	hostTickSet(t0 + 20);
	hostCanPoll(0);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 1u);
	CHECK(hostCanTxCount(0) == 0u);
	hostTickSet(t0 + 1100);
	hostCanPoll(0);
	CHECK(hostCanTxCount(0) == 1u);  // RM0090: the frame in progress is completed successfully
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 0u);
	CHECK((can()->TSR & 2u) == 2u);  // TXOK0
}

TEST_CASE_FIXTURE(CtF, "CT2 AbortTxRequest of a mailbox being sent frees it without retry when the frame fails") {
	up();
	hostCanSetFrameTicks(1000);
	const uint32_t t0 = hostTickPeek();
	addStd(0x10);
	hostCanQueueTxOutcome(0, HOST_CAN_TX_ERROR);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_OK);
	hostTickSet(t0 + 1100);
	hostCanPoll(0);
	hostTickSet(t0 + 5000);
	hostCanPoll(0);
	CHECK(hostCanTxCount(0) == 0u);  // no automatic retransmission after an abort request
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 3u);
	CHECK((can()->TSR & 2u) == 0u);  // TXOK0 clear
}

TEST_CASE_FIXTURE(CtF, "CT2 frames in the mailboxes are sent in the order of their identifiers, lowest first") {
	up();
	hold(true);
	addStd(0x300);  // mailbox 0
	addStd(0x100);  // mailbox 1
	addStd(0x200);  // mailbox 2
	hold(false);
	hostCanPoll(0);
	REQUIRE(hostCanTxCount(0) == 3u);
	CHECK(hostCanTxAt(0, 0).id == 0x100u);
	CHECK(hostCanTxAt(0, 1).id == 0x200u);
	CHECK(hostCanTxAt(0, 2).id == 0x300u);
}

TEST_CASE_FIXTURE(CtF, "CT2 a standard frame beats an extended frame with the same base identifier") {
	// RM0090 32.7.1: with equal 11 bit base identifiers the standard (IDE = 0) frame wins.
	up();
	hold(true);
	add(extHdr(0x100u << 18));  // base identifier 0x100, mailbox 0
	addStd(0x100);              // mailbox 1
	hold(false);
	hostCanPoll(0);
	REQUIRE(hostCanTxCount(0) == 2u);
	CHECK(hostCanTxAt(0, 0).ide == 0u);
	CHECK(hostCanTxAt(0, 1).ide == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT2 frames with equal identifiers are sent lowest mailbox first") {
	// RM0090 32.7.1: "If there are several mailboxes with the same priority, the lowest mailbox number" is sent first.
	up();
	hold(true);
	CAN_TxHeaderTypeDef t = stdHdr(0x123, 1);
	uint8_t d1[8] = {0xA1}, d2[8] = {0xA2};
	uint32_t mb;
	HAL_CAN_AddTxMessage(&h, &t, d1, &mb);
	HAL_CAN_AddTxMessage(&h, &t, d2, &mb);
	hold(false);
	hostCanPoll(0);
	REQUIRE(hostCanTxCount(0) == 2u);
	CHECK(hostCanTxAt(0, 0).data[0] == 0xA1u);
	CHECK(hostCanTxAt(0, 1).data[0] == 0xA2u);
}

TEST_CASE_FIXTURE(CtF, "CT2 with TransmitFifoPriority the frames are sent in request order") {
	h.Init.TransmitFifoPriority = ENABLE;
	up();
	hold(true);
	addStd(0x300);
	addStd(0x100);
	addStd(0x200);
	hold(false);
	hostCanPoll(0);
	REQUIRE(hostCanTxCount(0) == 3u);
	CHECK(hostCanTxAt(0, 0).id == 0x300u);
	CHECK(hostCanTxAt(0, 1).id == 0x100u);
	CHECK(hostCanTxAt(0, 2).id == 0x200u);
}

TEST_CASE_FIXTURE(CtF, "CT2 without TransmitFifoPriority the same requests go out by identifier") {
	h.Init.TransmitFifoPriority = DISABLE;
	up();
	hold(true);
	addStd(0x300);
	addStd(0x100);
	hold(false);
	hostCanPoll(0);
	REQUIRE(hostCanTxCount(0) == 2u);
	CHECK(hostCanTxAt(0, 0).id == 0x100u);
}

TEST_CASE_FIXTURE(CtF, "CT2 with automatic retransmission a frame that lost arbitration is sent again") {
	up();
	hostCanQueueTxOutcome(0, HOST_CAN_TX_ARB_LOST);
	addStd(0x10);
	CHECK(hostCanTxCount(0) == 1u);  // one successful frame after the lost attempt
	CHECK(HAL_CAN_IsTxMessagePending(&h, 7u) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT2 with automatic retransmission a frame with a bus error is sent again") {
	up();
	hostCanQueueTxOutcome(0, HOST_CAN_TX_ERROR);
	addStd(0x10);
	CHECK(hostCanTxCount(0) == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT2 with NART (AutoRetransmission off) a frame that lost arbitration is not sent again") {
	h.Init.AutoRetransmission = DISABLE;
	up();
	hostCanQueueTxOutcome(0, HOST_CAN_TX_ARB_LOST);
	addStd(0x10);
	CHECK(hostCanTxCount(0) == 0u);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 3u);
	CHECK((can()->TSR & (1u << 2)) != 0u);  // ALST0
	CHECK((can()->TSR & 1u) != 0u);         // RQCP0
	CHECK((can()->TSR & 2u) == 0u);         // TXOK0
}

TEST_CASE_FIXTURE(CtF, "CT2 with NART a frame with a bus error is not sent again and sets TERR") {
	h.Init.AutoRetransmission = DISABLE;
	up();
	hostCanQueueTxOutcome(0, HOST_CAN_TX_ERROR);
	addStd(0x10);
	CHECK(hostCanTxCount(0) == 0u);
	CHECK(HAL_CAN_GetTxMailboxesFreeLevel(&h) == 3u);
	CHECK((can()->TSR & (1u << 3)) != 0u);  // TERR0
	CHECK((can()->TSR & 2u) == 0u);         // TXOK0
}

TEST_CASE_FIXTURE(CtF, "CT2 with NART and a successful frame the next frame is accepted normally") {
	h.Init.AutoRetransmission = DISABLE;
	up();
	hostCanQueueTxOutcome(0, HOST_CAN_TX_ERROR);
	addStd(0x10);
	CHECK(addStd(0x20) == CAN_TX_MAILBOX0);
	CHECK(hostCanTxCount(0) == 1u);
	CHECK(hostCanTxAt(0, 0).id == 0x20u);
}

TEST_CASE_FIXTURE(CtF, "CT2 without an acknowledging peer and with NART the frame fails with TERR") {
	// RM0090 32.7.6: a transmitted frame that is not acknowledged is an acknowledgement error (TERR set).
	h.Init.AutoRetransmission = DISABLE;
	up();
	hostCanSetBusAck(0, 0);
	addStd(0x10);
	CHECK(hostCanTxCount(0) == 0u);
	CHECK((can()->TSR & (1u << 3)) != 0u);
	CHECK(((can()->ESR >> 4) & 7u) == 3u);  // LEC = 3 acknowledgement error
}

TEST_CASE_FIXTURE(CtF, "CT2 a frame stays pending while the bus is held and goes out when it is released") {
	up();
	hold(true);
	addStd(0x10);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 1u);
	hold(false);
	hostCanPoll(0);
	CHECK(HAL_CAN_IsTxMessagePending(&h, CAN_TX_MAILBOX0) == 0u);
	CHECK(hostCanTxCount(0) == 1u);
}

// ------------------------------------------------------------------ CT3 timestamps and reception

TEST_CASE_FIXTURE(CtF, "CT3 GetTxTimestamp returns the TIME captured for a sent frame in time triggered mode") {
	h.Init.TimeTriggeredMode = ENABLE;
	up();
	hostCanSetTimer(0, 0x1234);
	addStd(0x10);
	CHECK(HAL_CAN_GetTxTimestamp(&h, CAN_TX_MAILBOX0) == 0x1234u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetTxTimestamp reads the TIME of the mailbox given as argument") {
	h.Init.TimeTriggeredMode = ENABLE;
	up();
	hostCanSetFrameTicks(1000);
	hold(true);
	addStd(0x10);  // mailbox 0
	addStd(0x20);  // mailbox 1
	addStd(0x30);  // mailbox 2
	hold(false);
	hostCanPoll(0);  // mailbox 0 starts (identifier order), 1000 ticks long
	const uint32_t t0 = hostTickPeek();
	hostCanSetTimer(0, 0x0A0A);
	hostTickSet(t0 + 1000);
	hostCanPoll(0);  // mailbox 0 done, mailbox 1 starts
	hostCanSetTimer(0, 0x0B0B);
	hostTickSet(t0 + 2000);
	hostCanPoll(0);
	hostCanSetTimer(0, 0x0C0C);
	hostTickSet(t0 + 3000);
	hostCanPoll(0);
	REQUIRE(hostCanTxCount(0) == 3u);
	CHECK(HAL_CAN_GetTxTimestamp(&h, CAN_TX_MAILBOX0) == 0x0A0Au);
	CHECK(HAL_CAN_GetTxTimestamp(&h, CAN_TX_MAILBOX1) == 0x0B0Bu);
	CHECK(HAL_CAN_GetTxTimestamp(&h, CAN_TX_MAILBOX2) == 0x0C0Cu);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetTxTimestamp returns the 16 bit maximum 0xFFFF unchanged") {
	h.Init.TimeTriggeredMode = ENABLE;
	up();
	hostCanSetTimer(0, 0xFFFF);
	addStd(0x10);
	CHECK(HAL_CAN_GetTxTimestamp(&h, CAN_TX_MAILBOX0) == 0xFFFFu);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetTxTimestamp is 0 for a mailbox that never sent a frame") {
	h.Init.TimeTriggeredMode = ENABLE;
	up();
	CHECK(HAL_CAN_GetTxTimestamp(&h, CAN_TX_MAILBOX2) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetTxTimestamp returns 0 in states RESET and ERROR") {
	h.Init.TimeTriggeredMode = ENABLE;
	up();
	hostCanSetTimer(0, 0x1234);
	addStd(0x10);
	h.State = HAL_CAN_STATE_RESET;
	CHECK(HAL_CAN_GetTxTimestamp(&h, CAN_TX_MAILBOX0) == 0u);
	h.State = HAL_CAN_STATE_ERROR;
	CHECK(HAL_CAN_GetTxTimestamp(&h, CAN_TX_MAILBOX0) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetTxTimestamp works in state READY") {
	h.Init.TimeTriggeredMode = ENABLE;
	up(false);
	hostCanSetTimer(0, 0x2222);
	addStd(0x10);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	hostCanPoll(0);
	REQUIRE(HAL_CAN_Stop(&h) == HAL_OK);
	REQUIRE(h.State == HAL_CAN_STATE_READY);
	CHECK(HAL_CAN_GetTxTimestamp(&h, CAN_TX_MAILBOX0) == 0x2222u);
}

TEST_CASE_FIXTURE(CtF, "CT3 characterization: GetRxMessage is rejected with NOT_INITIALIZED in state RESET") {
	up();
	hostCanRx(0, &(const HostCanFrame &)rxf(0x100, 0, 0, 1));
	h.State = HAL_CAN_STATE_RESET;
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	CHECK(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
	h.State = HAL_CAN_STATE_LISTENING;
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 1u);  // the message was not released
}

TEST_CASE_FIXTURE(CtF, "CT3 characterization: GetRxMessage is rejected with NOT_INITIALIZED in state ERROR") {
	up();
	h.State = HAL_CAN_STATE_ERROR;
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	CHECK(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO1, &rh, d) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 characterization: GetRxMessage on an empty FIFO 0 returns HAL_ERROR with PARAM") {
	// NEEDS REVIEW 329 (see CT1 NEEDS REVIEW 329): UM1725 names no error code for an empty FIFO; the HAL uses PARAM.
	up();
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	CHECK(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_PARAM) != 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 characterization: GetRxMessage on an empty FIFO 1 returns HAL_ERROR with PARAM") {
	up();
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	CHECK(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO1, &rh, d) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_PARAM) != 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage of an empty FIFO 1 is not satisfied by a message in FIFO 0") {
	up();
	hostCanRx(0, &(const HostCanFrame &)rxf(0x100, 0, 0, 1));
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	CHECK(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO1, &rh, d) == HAL_ERROR);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reads a standard data frame (identifier, IDE, RTR, DLC, data)") {
	up();
	const HostCanFrame f = rxf(0x4A5, 0, 0, 8, 0x10);
	CHECK(hostCanRx(0, &f) == 1);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8] = {};
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.IDE == CAN_ID_STD);
	CHECK(rh.StdId == 0x4A5u);
	CHECK(rh.RTR == CAN_RTR_DATA);
	CHECK(rh.DLC == 8u);
	for (unsigned i = 0; i < 8; ++i) CHECK(d[i] == 0x10 + i);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reads the full 11 bit standard identifier range") {
	up();
	const uint32_t ids[] = {0x000, 0x001, 0x400, 0x7FF};
	for (uint32_t id : ids) {
		const HostCanFrame f = rxf(id, 0, 0, 1);
		hostCanRx(0, &f);
		CAN_RxHeaderTypeDef rh{};
		uint8_t d[8];
		REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
		CHECK(rh.StdId == id);
	}
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reads an extended frame with the 29 bit identifier in ExtId") {
	up();
	const HostCanFrame f = rxf(0x1ABCDEF5u, 1, 0, 4, 0x20);
	CHECK(hostCanRx(0, &f) == 1);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.IDE == CAN_ID_EXT);
	CHECK(rh.ExtId == 0x1ABCDEF5u);
	CHECK(rh.DLC == 4u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reads the extended identifier extremes 0 and 0x1FFFFFFF") {
	up();
	const uint32_t ids[] = {0u, 0x1FFFFFFFu, 0x10000000u, 0x7FFu, 0x1FFFF800u};
	for (uint32_t id : ids) {
		const HostCanFrame f = rxf(id, 1, 0, 1);
		hostCanRx(0, &f);
		CAN_RxHeaderTypeDef rh{};
		uint8_t d[8];
		REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
		CHECK(rh.IDE == CAN_ID_EXT);
		CHECK(rh.ExtId == id);
	}
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reports a standard remote frame with RTR set") {
	up();
	const HostCanFrame f = rxf(0x222, 0, 1, 2);
	hostCanRx(0, &f);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.RTR == CAN_RTR_REMOTE);
	CHECK(rh.IDE == CAN_ID_STD);
	CHECK(rh.StdId == 0x222u);
	CHECK(rh.DLC == 2u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reports an extended remote frame with RTR and IDE set") {
	up();
	const HostCanFrame f = rxf(0x0123456u, 1, 1, 0);
	hostCanRx(0, &f);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.RTR == CAN_RTR_REMOTE);
	CHECK(rh.IDE == CAN_ID_EXT);
	CHECK(rh.ExtId == 0x0123456u);
	CHECK(rh.DLC == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reports DLC 0 to 8 as received") {
	up();
	for (uint8_t dlc = 0; dlc <= 8; ++dlc) {
		const HostCanFrame f = rxf(0x100, 0, 0, dlc);
		hostCanRx(0, &f);
		CAN_RxHeaderTypeDef rh{};
		uint8_t d[8];
		REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
		CHECK(rh.DLC == dlc);
	}
}

TEST_CASE_FIXTURE(CtF, "CT3 characterization: GetRxMessage limits a received DLC of 9 to 15 to 8") {
	// NEEDS REVIEW 329: UM1725 / RM0090 do not say what a DLC field above 8 gives; ISO 11898 treats 9..15 as 8 data bytes and the HAL truncates to 8.
	up();
	for (uint8_t dlc = 9; dlc <= 15; ++dlc) {
		const HostCanFrame f = rxf(0x100, 0, 0, dlc);
		hostCanRx(0, &f);
		CAN_RxHeaderTypeDef rh{};
		uint8_t d[8];
		REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
		CHECK(rh.DLC == 8u);
	}
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage delivers every data byte at its own index") {
	up();
	HostCanFrame f{};
	f.id = 0x100;
	f.dlc = 8;
	const uint8_t bytes[8] = {0x01, 0x80, 0xFF, 0x00, 0xA5, 0x5A, 0xC3, 0x3C};
	std::memcpy(f.data, bytes, 8);
	hostCanRx(0, &f);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8] = {};
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(std::memcmp(d, bytes, 8) == 0);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reports the time stamp of the received frame") {
	up();
	hostCanSetTimer(0, 0xBEEF);
	const HostCanFrame f = rxf(0x100, 0, 0, 1);
	hostCanRx(0, &f);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.Timestamp == 0xBEEFu);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage keeps the time stamp of each queued message") {
	up();
	const HostCanFrame f = rxf(0x100, 0, 0, 1);
	hostCanSetTimer(0, 0x0001);
	hostCanRx(0, &f);
	hostCanSetTimer(0, 0x0002);
	hostCanRx(0, &f);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.Timestamp == 1u);
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.Timestamp == 2u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reports the filter match index of bank 0 as 0") {
	up();
	const HostCanFrame f = rxf(0x100, 0, 0, 1);
	hostCanRx(0, &f);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.FilterMatchIndex == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reports the filter match index of the matching filter") {
	// Bank 0 accepts only 0x100, bank 1 accepts everything: 0x555 matches filter 1 only.
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	filterExact(0, 0x100, CAN_RX_FIFO0);
	filterPassAll(1, CAN_RX_FIFO0);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	const HostCanFrame f = rxf(0x555, 0, 0, 1);
	hostCanRx(0, &f);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.FilterMatchIndex == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reports the lower filter number when two filters match") {
	// RM0090 32.7.4: when several filters match, the one with the lowest number is used.
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	filterExact(0, 0x100, CAN_RX_FIFO0);
	filterPassAll(1, CAN_RX_FIFO0);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	const HostCanFrame f = rxf(0x100, 0, 0, 1);
	hostCanRx(0, &f);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.FilterMatchIndex == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage reads from FIFO 1 when the filter assigns the frame to FIFO 1") {
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	filterExact(0, 0x100, CAN_RX_FIFO0);
	filterExact(1, 0x200, CAN_RX_FIFO1);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	const HostCanFrame f0 = rxf(0x100, 0, 0, 1, 0xA0), f1 = rxf(0x200, 0, 0, 2, 0xB0);
	CHECK(hostCanRx(0, &f0) == 1);
	CHECK(hostCanRx(0, &f1) == 1);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 1u);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO1) == 1u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO1, &rh, d) == HAL_OK);
	CHECK(rh.StdId == 0x200u);
	CHECK(rh.DLC == 2u);
	CHECK(d[0] == 0xB0u);
	CHECK(rh.FilterMatchIndex == 0u);  // numbering restarts per FIFO
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 1u);  // FIFO 0 untouched
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.StdId == 0x100u);
	CHECK(d[0] == 0xA0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage on FIFO 1 with FIFO 0 empty returns the FIFO 1 frame") {
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	filterPassAll(0, CAN_RX_FIFO1);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	const HostCanFrame f = rxf(0x321, 0, 0, 1);
	hostCanRx(0, &f);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	CHECK(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_ERROR);
	CHECK(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO1, &rh, d) == HAL_OK);
	CHECK(rh.StdId == 0x321u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage releases the FIFO output mailbox and the next message follows") {
	up();
	const HostCanFrame a = rxf(0x101, 0, 0, 1, 1), b = rxf(0x102, 0, 0, 1, 2), c = rxf(0x103, 0, 0, 1, 3);
	hostCanRx(0, &a);
	hostCanRx(0, &b);
	hostCanRx(0, &c);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.StdId == 0x101u);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 2u);
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.StdId == 0x102u);
	CHECK(d[0] == 2u);
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.StdId == 0x103u);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 0u);
	CHECK(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_ERROR);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage does not clear the error code after a success") {
	up();
	const HostCanFrame a = rxf(0x101, 0, 0, 1);
	hostCanRx(0, &a);
	h.ErrorCode = HAL_CAN_ERROR_EWG;
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_EWG);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage works in state READY") {
	up(false);
	REQUIRE(h.State == HAL_CAN_STATE_READY);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	CHECK(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_ERROR);  // empty, not a state error
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxFifoFillLevel counts 0 to 3 as messages arrive") {
	up();
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 0u);
	const HostCanFrame f = rxf(0x100, 0, 0, 1);
	hostCanRx(0, &f);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 1u);
	hostCanRx(0, &f);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 2u);
	hostCanRx(0, &f);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 3u);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO1) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxFifoFillLevel of FIFO 1 counts its own messages") {
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	filterPassAll(0, CAN_RX_FIFO1);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	const HostCanFrame f = rxf(0x100, 0, 0, 1);
	hostCanRx(0, &f);
	hostCanRx(0, &f);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO1) == 2u);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxFifoFillLevel is 0 in states RESET and ERROR even with stored messages") {
	up();
	const HostCanFrame f = rxf(0x100, 0, 0, 1);
	hostCanRx(0, &f);
	h.State = HAL_CAN_STATE_RESET;
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 0u);
	h.State = HAL_CAN_STATE_ERROR;
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO1) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxFifoFillLevel reports the stored messages in state READY and 0 in a rejected state") {
	up(false);
	REQUIRE(h.State == HAL_CAN_STATE_READY);
	// Not started: frames are not received (RM0090 32.4.1), so Start first, store frames, then Stop back to READY.
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	const HostCanFrame f = rxf(0x100, 0, 0, 1);
	for (unsigned n = 1; n <= 3; ++n) {
		hostCanRx(0, &f);
		REQUIRE(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == n);
	}
	h.State = HAL_CAN_STATE_READY;
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 3u);
	h.State = HAL_CAN_STATE_SLEEP_PENDING;
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 0u);
}

TEST_CASE_FIXTURE(CtF, "CT3 FIFO 0 with RFLM 0 overwrites the newest message when it overruns") {
	// RM0090 32.7.3: FIFO full and not locked: "the last message in the FIFO is overwritten by the new incoming message".
	h.Init.ReceiveFifoLocked = DISABLE;
	up();
	for (uint8_t i = 1; i <= 3; ++i) {
		const HostCanFrame f = rxf(0x100 + i, 0, 0, 1, i);
		CHECK(hostCanRx(0, &f) == 1);
	}
	CHECK((can()->RF0R & (1u << 3)) != 0u);  // FULL0
	const HostCanFrame over = rxf(0x104, 0, 0, 1, 4);
	CHECK(hostCanRx(0, &over) == -1);
	CHECK((can()->RF0R & (1u << 4)) != 0u);  // FOVR0
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 3u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	const uint32_t expect[3] = {0x101, 0x102, 0x104};
	for (uint32_t e : expect) {
		REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
		CHECK(rh.StdId == e);
	}
}

TEST_CASE_FIXTURE(CtF, "CT3 FIFO 0 with RFLM 1 discards the new message when it overruns") {
	// RM0090 32.7.3: RFLM = 1 "the FIFO is locked against overrun, the next incoming message will be discarded".
	h.Init.ReceiveFifoLocked = ENABLE;
	up();
	for (uint8_t i = 1; i <= 3; ++i) {
		const HostCanFrame f = rxf(0x100 + i, 0, 0, 1, i);
		hostCanRx(0, &f);
	}
	const HostCanFrame over = rxf(0x104, 0, 0, 1, 4);
	CHECK(hostCanRx(0, &over) == -1);
	CHECK((can()->RF0R & (1u << 4)) != 0u);  // FOVR0
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 3u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	const uint32_t expect[3] = {0x101, 0x102, 0x103};
	for (uint32_t e : expect) {
		REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
		CHECK(rh.StdId == e);
	}
	CHECK(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_ERROR);
}

TEST_CASE_FIXTURE(CtF, "CT3 FIFO 1 with RFLM 0 overwrites the newest message when it overruns") {
	h.Init.ReceiveFifoLocked = DISABLE;
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	filterPassAll(0, CAN_RX_FIFO1);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	for (uint8_t i = 1; i <= 4; ++i) {
		const HostCanFrame f = rxf(0x100 + i, 0, 0, 1, i);
		hostCanRx(0, &f);
	}
	CHECK((can()->RF1R & (1u << 4)) != 0u);  // FOVR1
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO1) == 3u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	const uint32_t expect[3] = {0x101, 0x102, 0x104};
	for (uint32_t e : expect) {
		REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO1, &rh, d) == HAL_OK);
		CHECK(rh.StdId == e);
	}
}

TEST_CASE_FIXTURE(CtF, "CT3 FIFO 1 with RFLM 1 discards the new message when it overruns") {
	h.Init.ReceiveFifoLocked = ENABLE;
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	filterPassAll(0, CAN_RX_FIFO1);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	for (uint8_t i = 1; i <= 4; ++i) {
		const HostCanFrame f = rxf(0x100 + i, 0, 0, 1, i);
		hostCanRx(0, &f);
	}
	CHECK((can()->RF1R & (1u << 4)) != 0u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	const uint32_t expect[3] = {0x101, 0x102, 0x103};
	for (uint32_t e : expect) {
		REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO1, &rh, d) == HAL_OK);
		CHECK(rh.StdId == e);
	}
}

TEST_CASE_FIXTURE(CtF, "CT3 an overrun in FIFO 0 leaves FIFO 1 unaffected") {
	h.Init.ReceiveFifoLocked = ENABLE;
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	filterExact(0, 0x100, CAN_RX_FIFO0);
	filterExact(1, 0x200, CAN_RX_FIFO1);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	const HostCanFrame f0 = rxf(0x100, 0, 0, 1), f1 = rxf(0x200, 0, 0, 1);
	for (int i = 0; i < 4; ++i) hostCanRx(0, &f0);
	hostCanRx(0, &f1);
	CHECK((can()->RF0R & (1u << 4)) != 0u);
	CHECK((can()->RF1R & (1u << 4)) == 0u);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO1) == 1u);
}

TEST_CASE_FIXTURE(CtF, "CT3 GetRxMessage on CAN2 reads frames received by CAN2") {
	h.Instance = CAN2;
	up();
	const HostCanFrame f = rxf(0x2B2, 0, 0, 3, 0x40);
	CHECK(hostCanRx(1, &f) == 1);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 1u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.StdId == 0x2B2u);
	CHECK(rh.DLC == 3u);
	CHECK(d[0] == 0x40u);
}

// ------------------------------------------------------------------ CT4 loop back end to end

namespace {
struct CtLoop : CtF {
	CtLoop() { h.Init.Mode = CAN_MODE_LOOPBACK; }
};
}  // namespace

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back returns a standard frame sent with all eight data bytes") {
	up();
	CHECK(addStd(0x7FF) == CAN_TX_MAILBOX0);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 1u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8] = {};
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.IDE == CAN_ID_STD);
	CHECK(rh.StdId == 0x7FFu);
	CHECK(rh.RTR == CAN_RTR_DATA);
	CHECK(rh.DLC == 8u);
	CHECK(std::memcmp(d, ctD, 8) == 0);
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back returns an extended frame with a 29 bit identifier") {
	up();
	add(extHdr(0x1FFFFFFFu));
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8] = {};
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.IDE == CAN_ID_EXT);
	CHECK(rh.ExtId == 0x1FFFFFFFu);
	CHECK(std::memcmp(d, ctD, 8) == 0);
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back keeps the 11 bit and 29 bit identifiers apart") {
	up();
	// Same numeric value 0x123: one standard frame, one extended frame.
	addStd(0x123);
	add(extHdr(0x123));
	CAN_RxHeaderTypeDef r1{}, r2{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &r1, d) == HAL_OK);
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &r2, d) == HAL_OK);
	CHECK(r1.IDE == CAN_ID_STD);
	CHECK(r1.StdId == 0x123u);
	CHECK(r2.IDE == CAN_ID_EXT);
	CHECK(r2.ExtId == 0x123u);
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back uses ExtId for an extended frame even when StdId is also set") {
	up();
	CAN_TxHeaderTypeDef t = extHdr(0x0AAAAAAAu);
	t.StdId = 0x7FF;
	add(t);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.ExtId == 0x0AAAAAAAu);
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back uses StdId for a standard frame even when ExtId is also set") {
	up();
	CAN_TxHeaderTypeDef t = stdHdr(0x2AA);
	t.ExtId = 0x1FFFFFFFu;
	add(t);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.IDE == CAN_ID_STD);
	CHECK(rh.StdId == 0x2AAu);
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back returns a remote frame as remote") {
	up();
	add(stdHdr(0x321, 0, CAN_RTR_REMOTE));
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.RTR == CAN_RTR_REMOTE);
	CHECK(rh.StdId == 0x321u);
	CHECK(rh.DLC == 0u);
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back returns the DLC sent for DLC 0 to 8") {
	up();
	for (uint32_t dlc = 0; dlc <= 8; ++dlc) {
		add(stdHdr(0x100, dlc));
		CAN_RxHeaderTypeDef rh{};
		uint8_t d[8];
		REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
		CHECK(rh.DLC == dlc);
	}
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back delivers identifiers in sending order through the FIFO") {
	up();
	addStd(0x10);
	addStd(0x20);
	addStd(0x30);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 3u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	const uint32_t expect[3] = {0x10, 0x20, 0x30};
	for (uint32_t e : expect) {
		REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
		CHECK(rh.StdId == e);
	}
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back only receives frames accepted by the filter") {
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	filterExact(0, 0x123, CAN_RX_FIFO0);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	addStd(0x124);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 0u);
	addStd(0x123);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 1u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.StdId == 0x123u);
	CHECK(hostCanTxCount(0) == 2u);  // both frames were sent
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back puts a frame into FIFO 1 when the filter assigns FIFO 1") {
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	filterPassAll(0, CAN_RX_FIFO1);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	addStd(0x55);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 0u);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO1) == 1u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO1, &rh, d) == HAL_OK);
	CHECK(rh.StdId == 0x55u);
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back receives nothing without a filter") {
	REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	addStd(0x55);
	CHECK(hostCanTxCount(0) == 1u);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO0) == 0u);
	CHECK(HAL_CAN_GetRxFifoFillLevel(&h, CAN_RX_FIFO1) == 0u);
}

TEST_CASE_FIXTURE(CtLoop, "CT4 loop back on CAN2 is received through CAN2's filter bank") {
	h.Instance = CAN2;
	up();
	addStd(0x66);
	CHECK(hostCanTxCount(1) == 1u);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.StdId == 0x66u);
	CHECK(std::memcmp(d, ctD, 8) == 0);
}

TEST_CASE_FIXTURE(CtLoop, "CT4 a loop back frame keeps data byte order through the whole path") {
	up();
	const uint8_t data[8] = {0x80, 0x01, 0xFE, 0x7F, 0x00, 0xFF, 0x55, 0xAA};
	const CAN_TxHeaderTypeDef t = stdHdr(0x1);
	uint32_t mb;
	REQUIRE(HAL_CAN_AddTxMessage(&h, &t, data, &mb) == HAL_OK);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8] = {};
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(std::memcmp(d, data, 8) == 0);
}
