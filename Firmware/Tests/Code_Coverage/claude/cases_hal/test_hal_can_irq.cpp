/*
 * P12 T5 group CQ: CAN notifications and the interrupt handler (HAL_CAN_ActivateNotification, HAL_CAN_DeactivateNotification,
 * HAL_CAN_IRQHandler and the callbacks) of stm32f4xx_hal_can.c against the bxCAN model (fakes/stm32_hal/HalCan.h).
 * Requirements CQ1-CQ5 of Tests/Code_Coverage/claude/charters/hal/P12_T5_REQUIREMENTS.md; compiled into ffb_hal_tests and
 * ffb_hal_tests_disco.
 *
 * Oracles: UM1725 Rev 8 chapter 9 (HAL CAN: callbacks run when the event happens, error code names, "ActivateNotification:
 * enable interrupts") and RM0090 Rev 22 chapter 32 (32.8 interrupts, Figure 348; 32.9.3 / 32.9.4 / 32.9.5 MSR, TSR, RFxR flag
 * semantics; 32.9.6 IER bit positions; 32.9.7 ESR with the LEC coding 001 stuff, 010 form, 011 acknowledgment, 100 bit
 * recessive, 101 bit dominant, 110 CRC). Interrupts are not delivered by the model: the tests call HAL_CAN_IRQHandler themselves.
 *
 * NEEDS REVIEW proposals (characterization cases, the lead assigns the numbers):
 *  NEEDS REVIEW 330: IER bits that RM0090 marks reserved are not stored when written through ActivateNotification (model mask).
 *  NEEDS REVIEW 330: when ALST and TERR of one mailbox are both set with TXOK clear, the handler reports only ALST (UM1725 is silent).
 *  NEEDS REVIEW 330: order of callbacks inside one handler call (TX0, TX1, TX2, FIFO0 full, FIFO0 pending, FIFO1 full, FIFO1 pending, sleep,
 *            wake-up, error last) in the FIFO 0 full / pending / error case, and the FIFO callbacks running again at every handler call while the flag stays (UM1725 is silent).
 *  NEEDS REVIEW 330: an ESR LEC of 7 (set by software) maps to no error code but is cleared (RM0090 says software may write 7).
 *  NEEDS REVIEW 330: the error code is only OR-ed (never cleared by the handler); ErrorCallback sees the accumulated code.
 *  NEEDS REVIEW 330: the same callback order for all sources of one handler call (the all-sources case in CQ5).
 *  NEEDS REVIEW 330: ActivateNotification / DeactivateNotification in a state other than READY / LISTENING return HAL_ERROR and OR NOT_INITIALIZED into
 *            ErrorCode (UM1725 only says "HAL status"; the code names are the HAL header's).
 */
#include "CanTestSupport.h"
#include "TestHelpers.h"

#include <cstdint>
#include <cstring>

namespace {

const uint32_t kTsrRqcp[3] = {1u << 0, 1u << 8, 1u << 16};
const uint32_t kTsrTxok[3] = {1u << 1, 1u << 9, 1u << 17};
const uint32_t kTsrAlst[3] = {1u << 2, 1u << 10, 1u << 18};
const uint32_t kTsrTerr[3] = {1u << 3, 1u << 11, 1u << 19};
const uint32_t kErrAlst[3] = {0x800u, 0x2000u, 0x8000u};  // UM1725 / header coding of HAL_CAN_ERROR_TX_ALSTx
const uint32_t kErrTerr[3] = {0x1000u, 0x4000u, 0x10000u};

// IER bit values by RM0090 32.9.6 (bit position in the comment).
const uint32_t kIerTme = 1u << 0, kIerFmp0 = 1u << 1, kIerFf0 = 1u << 2, kIerFov0 = 1u << 3;
const uint32_t kIerFmp1 = 1u << 4, kIerFf1 = 1u << 5, kIerFov1 = 1u << 6;
const uint32_t kIerEwg = 1u << 8, kIerEpv = 1u << 9, kIerBof = 1u << 10, kIerLec = 1u << 11, kIerErr = 1u << 15;
const uint32_t kIerWkui = 1u << 16, kIerSlki = 1u << 17;

const uint32_t kMsrErri = 1u << 2, kMsrWkui = 1u << 3, kMsrSlaki = 1u << 4;
const uint32_t kRfFull = 1u << 3, kRfFovr = 1u << 4;

struct Q : CanFixture {
	CAN_HandleTypeDef h, h2;
	Q() {
		std::memset(&h, 0, sizeof h);
		std::memset(&h2, 0, sizeof h2);
		cfg(h, CAN1);
		cfg(h2, CAN2);
	}
	static void cfg(CAN_HandleTypeDef &x, CAN_TypeDef *inst) {
		x.Instance = inst;
		x.Init.Prescaler = 6;
		x.Init.Mode = CAN_MODE_NORMAL;
		x.Init.SyncJumpWidth = CAN_SJW_1TQ;
		x.Init.TimeSeg1 = CAN_BS1_11TQ;
		x.Init.TimeSeg2 = CAN_BS2_2TQ;
		x.Init.AutoRetransmission = ENABLE;
	}
	static unsigned ix(const CAN_HandleTypeDef &x) { return canIndexOf(x.Instance); }
	// Two accept-by-identifier 32-bit mask filters: bank b -> FIFO 0 accepts standard id 0x100, bank b+1 -> FIFO 1 accepts 0x200.
	void filters(CAN_HandleTypeDef &x, uint32_t b) {
		for (unsigned k = 0; k < 2; ++k) {
			CAN_FilterTypeDef f;
			std::memset(&f, 0, sizeof f);
			f.FilterBank = b + k;
			f.FilterMode = CAN_FILTERMODE_IDMASK;
			f.FilterScale = CAN_FILTERSCALE_32BIT;
			f.FilterIdHigh = (k == 0 ? 0x100u : 0x200u) << 5;  // standard id sits in bits 31:21 of the 32-bit register
			f.FilterMaskIdHigh = 0xFFE0u;                      // all 11 id bits compared, IDE / RTR don't care
			f.FilterFIFOAssignment = k == 0 ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
			f.FilterActivation = CAN_FILTER_ENABLE;
			f.SlaveStartFilterBank = 14;
			REQUIRE(HAL_CAN_ConfigFilter(&x, &f) == HAL_OK);
		}
	}
	void init(CAN_HandleTypeDef &x) { REQUIRE(HAL_CAN_Init(&x) == HAL_OK); }
	void start(CAN_HandleTypeDef &x) {
		init(x);
		filters(x, ix(x) == 0 ? 0u : 14u);
		REQUIRE(HAL_CAN_Start(&x) == HAL_OK);
	}
	void send(CAN_HandleTypeDef &x, uint32_t id) {
		CAN_TxHeaderTypeDef th;
		std::memset(&th, 0, sizeof th);
		th.StdId = id;
		th.IDE = CAN_ID_STD;
		th.RTR = CAN_RTR_DATA;
		th.DLC = 1;
		uint8_t d[8] = {0x5A};
		uint32_t mb = 0;
		REQUIRE(HAL_CAN_AddTxMessage(&x, &th, d, &mb) == HAL_OK);
	}
	// Hold the bus, queue three frames (ids 0x10 < 0x20 < 0x30: sent in the order of mailbox 0, 1, 2), queue the outcomes, release.
	void threeFrames(CAN_HandleTypeDef &x, const int outcome[3]) {
		const unsigned i = ix(x);
		hostCanHoldBus(i, 1);
		send(x, 0x10);
		send(x, 0x20);
		send(x, 0x30);
		for (unsigned k = 0; k < 3; ++k) hostCanQueueTxOutcome(i, outcome[k]);
		hostCanHoldBus(i, 0);
		hostCanPoll(i);
	}
	void rx(CAN_HandleTypeDef &x, uint32_t id, unsigned n) {
		HostCanFrame f;
		std::memset(&f, 0, sizeof f);
		f.id = id;
		f.dlc = 1;
		for (unsigned k = 0; k < n; ++k) hostCanRx(ix(x), &f);
	}
	void irq(CAN_HandleTypeDef &x) { HAL_CAN_IRQHandler(&x); }
	void enable(CAN_HandleTypeDef &x, uint32_t bits) { REQUIRE(HAL_CAN_ActivateNotification(&x, bits) == HAL_OK); }
};

const int kOk[3] = {HOST_CAN_TX_OK, HOST_CAN_TX_OK, HOST_CAN_TX_OK};

}  // namespace

// ---------------------------------------------------------------------------------------------------------------- CQ1

TEST_CASE_FIXTURE(Q, "CQ1 activating each of the 14 interrupt sources sets exactly its IER bit") {
	const uint32_t bits[14] = {1u << 0, 1u << 1, 1u << 2, 1u << 3, 1u << 4, 1u << 5, 1u << 6, 1u << 8, 1u << 9, 1u << 10, 1u << 11, 1u << 15, 1u << 16, 1u << 17};
	init(h);
	for (unsigned k = 0; k < 14; ++k) {
		CAN1->IER = 0;
		CHECK(HAL_CAN_ActivateNotification(&h, bits[k]) == HAL_OK);
		CHECK(CAN1->IER == bits[k]);
	}
	CHECK(faults() == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ1 the CAN_IT_* macros have the RM0090 IER bit values") {
	CHECK(CAN_IT_TX_MAILBOX_EMPTY == (1u << 0));
	CHECK(CAN_IT_RX_FIFO0_MSG_PENDING == (1u << 1));
	CHECK(CAN_IT_RX_FIFO0_FULL == (1u << 2));
	CHECK(CAN_IT_RX_FIFO0_OVERRUN == (1u << 3));
	CHECK(CAN_IT_RX_FIFO1_MSG_PENDING == (1u << 4));
	CHECK(CAN_IT_RX_FIFO1_FULL == (1u << 5));
	CHECK(CAN_IT_RX_FIFO1_OVERRUN == (1u << 6));
	CHECK(CAN_IT_ERROR_WARNING == (1u << 8));
	CHECK(CAN_IT_ERROR_PASSIVE == (1u << 9));
	CHECK(CAN_IT_BUSOFF == (1u << 10));
	CHECK(CAN_IT_LAST_ERROR_CODE == (1u << 11));
	CHECK(CAN_IT_ERROR == (1u << 15));
	CHECK(CAN_IT_WAKEUP == (1u << 16));
	CHECK(CAN_IT_SLEEP_ACK == (1u << 17));
}

TEST_CASE_FIXTURE(Q, "CQ1 deactivating each of the 14 interrupt sources clears exactly its IER bit") {
	const uint32_t bits[14] = {1u << 0, 1u << 1, 1u << 2, 1u << 3, 1u << 4, 1u << 5, 1u << 6, 1u << 8, 1u << 9, 1u << 10, 1u << 11, 1u << 15, 1u << 16, 1u << 17};
	const uint32_t all = 0x38F7Fu;  // the 14 bits above
	init(h);
	for (unsigned k = 0; k < 14; ++k) {
		REQUIRE(HAL_CAN_ActivateNotification(&h, all) == HAL_OK);
		CHECK(HAL_CAN_DeactivateNotification(&h, bits[k]) == HAL_OK);
		CHECK(CAN1->IER == (all & ~bits[k]));
	}
	CHECK(faults() == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ1 activating several interrupt sources at once sets all their bits and keeps the earlier ones") {
	init(h);
	enable(h, CAN_IT_ERROR);
	enable(h, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_WAKEUP);
	CHECK(CAN1->IER == (kIerErr | kIerTme | kIerFmp0 | kIerWkui));
}

TEST_CASE_FIXTURE(Q, "CQ1 deactivating several interrupt sources at once clears only those bits") {
	init(h);
	enable(h, kIerTme | kIerFmp0 | kIerFmp1 | kIerErr | kIerLec);
	CHECK(HAL_CAN_DeactivateNotification(&h, kIerFmp0 | kIerLec | kIerSlki) == HAL_OK);
	CHECK(CAN1->IER == (kIerTme | kIerFmp1 | kIerErr));
}

TEST_CASE_FIXTURE(Q, "CQ1 activating an already active source and deactivating an inactive one change nothing") {
	init(h);
	enable(h, kIerTme);
	CHECK(HAL_CAN_ActivateNotification(&h, kIerTme) == HAL_OK);
	CHECK(CAN1->IER == kIerTme);
	CHECK(HAL_CAN_DeactivateNotification(&h, kIerErr) == HAL_OK);
	CHECK(CAN1->IER == kIerTme);
	CHECK(HAL_CAN_DeactivateNotification(&h, 0u) == HAL_OK);
	CHECK(HAL_CAN_ActivateNotification(&h, 0u) == HAL_OK);
	CHECK(CAN1->IER == kIerTme);
}

TEST_CASE_FIXTURE(Q, "CQ1 notifications can be activated and deactivated in the READY state and in the LISTENING state") {
	init(h);
	REQUIRE(h.State == HAL_CAN_STATE_READY);
	CHECK(HAL_CAN_ActivateNotification(&h, kIerTme) == HAL_OK);
	CHECK(HAL_CAN_DeactivateNotification(&h, kIerTme) == HAL_OK);
	filters(h, 0);
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	REQUIRE(h.State == HAL_CAN_STATE_LISTENING);
	CHECK(HAL_CAN_ActivateNotification(&h, kIerFmp0) == HAL_OK);
	CHECK(CAN1->IER == kIerFmp0);
	CHECK(HAL_CAN_DeactivateNotification(&h, kIerFmp0) == HAL_OK);
	CHECK(CAN1->IER == 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(Q, "CQ1 characterization: activating a notification in the states RESET, SLEEP_PENDING, SLEEP_ACTIVE and ERROR is refused with NOT_INITIALIZED (NEEDS REVIEW 330)") {
	const HAL_CAN_StateTypeDef states[4] = {HAL_CAN_STATE_RESET, HAL_CAN_STATE_SLEEP_PENDING, HAL_CAN_STATE_SLEEP_ACTIVE, HAL_CAN_STATE_ERROR};
	for (unsigned k = 0; k < 4; ++k) {
		h.State = states[k];
		h.ErrorCode = HAL_CAN_ERROR_NONE;
		CHECK(HAL_CAN_ActivateNotification(&h, kIerTme) == HAL_ERROR);
		CHECK(h.ErrorCode == HAL_CAN_ERROR_NOT_INITIALIZED);
		CHECK(CAN1->IER == 0u);
		CHECK(h.State == states[k]);
	}
}

TEST_CASE_FIXTURE(Q, "CQ1 characterization: deactivating a notification in a state other than READY / LISTENING is refused with NOT_INITIALIZED and keeps IER (NEEDS REVIEW 330)") {
	init(h);
	enable(h, kIerTme | kIerErr);
	const HAL_CAN_StateTypeDef states[4] = {HAL_CAN_STATE_RESET, HAL_CAN_STATE_SLEEP_PENDING, HAL_CAN_STATE_SLEEP_ACTIVE, HAL_CAN_STATE_ERROR};
	for (unsigned k = 0; k < 4; ++k) {
		h.State = states[k];
		h.ErrorCode = HAL_CAN_ERROR_NONE;
		CHECK(HAL_CAN_DeactivateNotification(&h, kIerTme) == HAL_ERROR);
		CHECK(h.ErrorCode == HAL_CAN_ERROR_NOT_INITIALIZED);
		CHECK(CAN1->IER == (kIerTme | kIerErr));
	}
}

TEST_CASE_FIXTURE(Q, "CQ1 characterization: a refused call adds NOT_INITIALIZED to the error code already set (NEEDS REVIEW 330)") {
	h.State = HAL_CAN_STATE_ERROR;
	h.ErrorCode = HAL_CAN_ERROR_EWG;
	CHECK(HAL_CAN_ActivateNotification(&h, kIerTme) == HAL_ERROR);
	CHECK(h.ErrorCode == (HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_NOT_INITIALIZED));
}

TEST_CASE_FIXTURE(Q, "CQ1 a successful call leaves the error code unchanged") {
	init(h);
	h.ErrorCode = HAL_CAN_ERROR_EPV;
	CHECK(HAL_CAN_ActivateNotification(&h, kIerTme) == HAL_OK);
	CHECK(HAL_CAN_DeactivateNotification(&h, kIerTme) == HAL_OK);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_EPV);
}

TEST_CASE_FIXTURE(Q, "CQ1 characterization: IER bits RM0090 marks reserved are not stored (NEEDS REVIEW 330)") {
	init(h);
	CHECK(HAL_CAN_ActivateNotification(&h, (1u << 7) | (1u << 12) | (1u << 18) | (1u << 31)) == HAL_OK);
	CHECK(CAN1->IER == 0u);
	CHECK(HAL_CAN_ActivateNotification(&h, kIerTme | (1u << 7)) == HAL_OK);
	CHECK(CAN1->IER == kIerTme);
}

TEST_CASE_FIXTURE(Q, "CQ1 notifications of CAN2 write the IER of CAN2 only") {
	init(h);
	init(h2);
	enable(h2, kIerFmp1 | kIerSlki);
	CHECK(CAN2->IER == (kIerFmp1 | kIerSlki));
	CHECK(CAN1->IER == 0u);
	CHECK(HAL_CAN_DeactivateNotification(&h2, kIerFmp1) == HAL_OK);
	CHECK(CAN2->IER == kIerSlki);
}

// ---------------------------------------------------------------------------------------------------------------- CQ2

TEST_CASE_FIXTURE(Q, "CQ2 a completed transmission of mailbox 0 runs the complete callback of mailbox 0 once and clears the flags") {
	start(h);
	enable(h, kIerTme);
	send(h, 0x10);
	REQUIRE((CAN1->TSR & kTsrRqcp[0]) != 0u);
	REQUIRE((CAN1->TSR & kTsrTxok[0]) != 0u);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_TX);
	irq(h);
	CHECK(canCounts.txComplete[0] == 1u);
	CHECK(canCounts.txComplete[1] + canCounts.txComplete[2] == 0u);
	CHECK(canCounts.txAbort[0] + canCounts.txAbort[1] + canCounts.txAbort[2] == 0u);
	CHECK(canCounts.error == 0u);
	CHECK((CAN1->TSR & (kTsrRqcp[0] | kTsrTxok[0])) == 0u);
	CHECK(hostCanIrqPending(0) == 0u);
	CHECK(canCounts.last == &h);
	irq(h);
	CHECK(canCounts.txComplete[0] == 1u);
}

TEST_CASE_FIXTURE(Q, "CQ2 completed transmissions of the three mailboxes run their own complete callbacks in mailbox order") {
	start(h);
	enable(h, kIerTme);
	threeFrames(h, kOk);
	REQUIRE((CAN1->TSR & (kTsrRqcp[0] | kTsrRqcp[1] | kTsrRqcp[2])) == (kTsrRqcp[0] | kTsrRqcp[1] | kTsrRqcp[2]));
	irq(h);
	CHECK(canCounts.txComplete[0] == 1u);
	CHECK(canCounts.txComplete[1] == 1u);
	CHECK(canCounts.txComplete[2] == 1u);
	REQUIRE(canCounts.orderCount == 3u);
	CHECK(canCounts.order[0] == CAN_CB_TX0);
	CHECK(canCounts.order[1] == CAN_CB_TX1);
	CHECK(canCounts.order[2] == CAN_CB_TX2);
	CHECK((CAN1->TSR & 0x000F0F0Fu) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 only the mailbox with a request-complete flag is reported (mailbox 1 alone)") {
	start(h);
	enable(h, kIerTme);
	threeFrames(h, kOk);
	CAN1->TSR = kTsrRqcp[0] | kTsrRqcp[2];  // software clears the flags of mailbox 0 and 2 (rc_w1)
	irq(h);
	CHECK(canCounts.txComplete[0] == 0u);
	CHECK(canCounts.txComplete[1] == 1u);
	CHECK(canCounts.txComplete[2] == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 only the mailbox with a request-complete flag is reported (mailbox 2 alone)") {
	start(h);
	enable(h, kIerTme);
	threeFrames(h, kOk);
	CAN1->TSR = kTsrRqcp[0] | kTsrRqcp[1];
	irq(h);
	CHECK(canCounts.txComplete[2] == 1u);
	CHECK(canCounts.txComplete[0] + canCounts.txComplete[1] == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 the transmit branch is gated by TMEIE: no callback and the flags stay") {
	start(h);
	send(h, 0x10);
	REQUIRE((CAN1->TSR & kTsrRqcp[0]) != 0u);
	CHECK(hostCanIrqPending(0) == 0u);
	irq(h);
	CHECK(canCounts.orderCount == 0u);
	CHECK((CAN1->TSR & kTsrRqcp[0]) != 0u);
	CHECK((CAN1->TSR & kTsrTxok[0]) != 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 the handler with no enabled source and no flag does nothing") {
	start(h);
	irq(h);
	CHECK(canCounts.orderCount == 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK(hostCanIrqPending(0) == 0u);
	CHECK(CAN1->IER == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 the handler with every source enabled and no flag does nothing") {
	start(h);
	enable(h, 0x38F7Fu);
	irq(h);
	CHECK(canCounts.orderCount == 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(Q, "CQ2 arbitration lost of mailbox 0 sets ALST0 in the error code and runs the error callback, no complete callback") {
	h.Init.AutoRetransmission = DISABLE;  // NART: the failed request is completed (RQCP = 1, TXOK = 0)
	start(h);
	enable(h, kIerTme);
	const int o[3] = {HOST_CAN_TX_ARB_LOST, HOST_CAN_TX_OK, HOST_CAN_TX_OK};
	threeFrames(h, o);
	REQUIRE((CAN1->TSR & kTsrAlst[0]) != 0u);
	irq(h);
	CHECK(h.ErrorCode == kErrAlst[0]);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.errorCodeAtError == kErrAlst[0]);
	CHECK(canCounts.txComplete[0] == 0u);
	CHECK(canCounts.txAbort[0] == 0u);
	CHECK(canCounts.txComplete[1] == 1u);
	CHECK(canCounts.txComplete[2] == 1u);
	CHECK((CAN1->TSR & (kTsrRqcp[0] | kTsrAlst[0])) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 arbitration lost of mailbox 1 sets ALST1 in the error code") {
	h.Init.AutoRetransmission = DISABLE;
	start(h);
	enable(h, kIerTme);
	const int o[3] = {HOST_CAN_TX_OK, HOST_CAN_TX_ARB_LOST, HOST_CAN_TX_OK};
	threeFrames(h, o);
	irq(h);
	CHECK(h.ErrorCode == kErrAlst[1]);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.txComplete[0] == 1u);
	CHECK(canCounts.txComplete[1] == 0u);
	CHECK(canCounts.txComplete[2] == 1u);
	CHECK((CAN1->TSR & (kTsrRqcp[1] | kTsrAlst[1])) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 arbitration lost of mailbox 2 sets ALST2 in the error code") {
	h.Init.AutoRetransmission = DISABLE;
	start(h);
	enable(h, kIerTme);
	const int o[3] = {HOST_CAN_TX_OK, HOST_CAN_TX_OK, HOST_CAN_TX_ARB_LOST};
	threeFrames(h, o);
	irq(h);
	CHECK(h.ErrorCode == kErrAlst[2]);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.txComplete[2] == 0u);
	CHECK(canCounts.txComplete[0] + canCounts.txComplete[1] == 2u);
	CHECK((CAN1->TSR & (kTsrRqcp[2] | kTsrAlst[2])) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 a transmit error of mailbox 0 sets TERR0 in the error code and runs the error callback") {
	h.Init.AutoRetransmission = DISABLE;
	start(h);
	enable(h, kIerTme);
	const int o[3] = {HOST_CAN_TX_ERROR, HOST_CAN_TX_OK, HOST_CAN_TX_OK};
	threeFrames(h, o);
	REQUIRE((CAN1->TSR & kTsrTerr[0]) != 0u);
	irq(h);
	CHECK(h.ErrorCode == kErrTerr[0]);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.errorCodeAtError == kErrTerr[0]);
	CHECK(canCounts.txComplete[0] == 0u);
	CHECK(canCounts.txAbort[0] == 0u);
	CHECK((CAN1->TSR & (kTsrRqcp[0] | kTsrTerr[0])) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 a transmit error of mailbox 1 sets TERR1 in the error code") {
	h.Init.AutoRetransmission = DISABLE;
	start(h);
	enable(h, kIerTme);
	const int o[3] = {HOST_CAN_TX_OK, HOST_CAN_TX_ERROR, HOST_CAN_TX_OK};
	threeFrames(h, o);
	irq(h);
	CHECK(h.ErrorCode == kErrTerr[1]);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.txComplete[1] == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 a transmit error of mailbox 2 sets TERR2 in the error code") {
	h.Init.AutoRetransmission = DISABLE;
	start(h);
	enable(h, kIerTme);
	const int o[3] = {HOST_CAN_TX_OK, HOST_CAN_TX_OK, HOST_CAN_TX_ERROR};
	threeFrames(h, o);
	irq(h);
	CHECK(h.ErrorCode == kErrTerr[2]);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.txComplete[2] == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 failures of all three mailboxes accumulate in one error code and one error callback") {
	h.Init.AutoRetransmission = DISABLE;
	start(h);
	enable(h, kIerTme);
	const int o[3] = {HOST_CAN_TX_ARB_LOST, HOST_CAN_TX_ERROR, HOST_CAN_TX_ARB_LOST};
	threeFrames(h, o);
	irq(h);
	CHECK(h.ErrorCode == (kErrAlst[0] | kErrTerr[1] | kErrAlst[2]));
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.errorCodeAtError == (kErrAlst[0] | kErrTerr[1] | kErrAlst[2]));
	CHECK(canCounts.txComplete[0] + canCounts.txComplete[1] + canCounts.txComplete[2] == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 an aborted mailbox 0 runs the abort callback of mailbox 0 and no error callback") {
	start(h);
	enable(h, kIerTme);
	hostCanHoldBus(0, 1);
	send(h, 0x10);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0) == HAL_OK);
	REQUIRE((CAN1->TSR & kTsrRqcp[0]) != 0u);
	REQUIRE((CAN1->TSR & kTsrTxok[0]) == 0u);
	irq(h);
	CHECK(canCounts.txAbort[0] == 1u);
	CHECK(canCounts.txAbort[1] + canCounts.txAbort[2] == 0u);
	CHECK(canCounts.txComplete[0] == 0u);
	CHECK(canCounts.error == 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK((CAN1->TSR & kTsrRqcp[0]) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 an aborted mailbox 1 runs the abort callback of mailbox 1") {
	start(h);
	enable(h, kIerTme);
	hostCanHoldBus(0, 1);
	send(h, 0x10);
	send(h, 0x20);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX1) == HAL_OK);
	irq(h);
	CHECK(canCounts.txAbort[1] == 1u);
	CHECK(canCounts.txAbort[0] + canCounts.txAbort[2] == 0u);
	CHECK(canCounts.error == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ2 an aborted mailbox 2 runs the abort callback of mailbox 2") {
	start(h);
	enable(h, kIerTme);
	hostCanHoldBus(0, 1);
	send(h, 0x10);
	send(h, 0x20);
	send(h, 0x30);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX2) == HAL_OK);
	irq(h);
	CHECK(canCounts.txAbort[2] == 1u);
	CHECK(canCounts.txAbort[0] + canCounts.txAbort[1] == 0u);
	CHECK(canCounts.error == 0u);
}

// V111: HAL_CAN_AbortTxRequest uses SET_BIT (read-modify-write) on TSR. RQCPx / TXOKx / ALSTx / TERRx are rc_w1 (RM0090 32.9.5),
// so the second SET_BIT writes back the RQCP flag the first abort has just set and clears it before the handler sees it.
TEST_CASE_FIXTURE(Q, "CQ2 abort of all three mailboxes runs the three abort callbacks in mailbox order" * KNOWN_ISSUE("V111 HAL_CAN_AbortTxRequest (stm32f4xx_hal_can.c:1365-1385) sets ABRQx with SET_BIT on TSR; the read-modify-write writes back the already set rc_w1 flag RQCP of the mailbox aborted before and clears it, so UM1725's abort callback of that mailbox is lost")) {
	start(h);
	enable(h, kIerTme);
	hostCanHoldBus(0, 1);
	send(h, 0x10);
	send(h, 0x20);
	send(h, 0x30);
	REQUIRE(HAL_CAN_AbortTxRequest(&h, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) == HAL_OK);
	irq(h);
	REQUIRE(canCounts.orderCount == 3u);
	CHECK(canCounts.order[0] == CAN_CB_ABORT0);
	CHECK(canCounts.order[1] == CAN_CB_ABORT1);
	CHECK(canCounts.order[2] == CAN_CB_ABORT2);
}

TEST_CASE_FIXTURE(Q, "CQ2 characterization: with ALST and TERR both set and no TXOK only the arbitration lost is reported (NEEDS REVIEW 330)") {
	start(h);  // automatic retransmission on: the failed frame stays pending and is sent again
	enable(h, kIerTme);
	hostCanSetFrameTicks(1);
	hostCanQueueTxOutcome(0, HOST_CAN_TX_ARB_LOST);
	hostCanQueueTxOutcome(0, HOST_CAN_TX_ERROR);
	hostCanQueueTxOutcome(0, HOST_CAN_TX_ERROR);
	send(h, 0x10);
	hostTickSet(hostTickPeek() + 1);
	hostCanPoll(0);
	hostTickSet(hostTickPeek() + 1);
	hostCanPoll(0);
	CAN1->TSR = 1u << 7;  // ABRQ0 written alone (the HAL's read-modify-write would also write back the rc_w1 flags): ends with the frame being sent
	hostTickSet(hostTickPeek() + 1);
	hostCanPoll(0);
	REQUIRE((CAN1->TSR & kTsrRqcp[0]) != 0u);
	REQUIRE((CAN1->TSR & kTsrTxok[0]) == 0u);
	REQUIRE((CAN1->TSR & kTsrAlst[0]) != 0u);
	REQUIRE((CAN1->TSR & kTsrTerr[0]) != 0u);
	irq(h);
	CHECK(h.ErrorCode == kErrAlst[0]);
	CHECK(canCounts.txAbort[0] == 0u);
	CHECK(canCounts.error == 1u);
}

TEST_CASE_FIXTURE(Q, "CQ2 the interrupt request line of transmit is pending while a flag with TMEIE is set and drops after the handler") {
	start(h);
	enable(h, kIerTme);
	CHECK(hostCanIrqPending(0) == 0u);
	send(h, 0x10);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_TX);
	irq(h);
	CHECK(hostCanIrqPending(0) == 0u);
}

// ---------------------------------------------------------------------------------------------------------------- CQ3

TEST_CASE_FIXTURE(Q, "CQ3 a message in FIFO 0 with FMPIE0 runs the pending callback of FIFO 0") {
	start(h);
	enable(h, kIerFmp0);
	rx(h, 0x100, 1);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_FIFO0);
	irq(h);
	CHECK(canCounts.rx0Pending == 1u);
	CHECK(canCounts.rx1Pending == 0u);
	CHECK(canCounts.error == 0u);
	CHECK(canCounts.last == &h);
	CHECK(canCounts.stateAtLast == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(Q, "CQ3 a message in FIFO 1 with FMPIE1 runs the pending callback of FIFO 1") {
	start(h);
	enable(h, kIerFmp1);
	rx(h, 0x200, 1);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_FIFO1);
	irq(h);
	CHECK(canCounts.rx1Pending == 1u);
	CHECK(canCounts.rx0Pending == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ3 the pending callback runs again at every handler call while a message stays and stops after the release") {
	start(h);
	enable(h, kIerFmp0);
	rx(h, 0x100, 1);
	irq(h);
	irq(h);
	CHECK(canCounts.rx0Pending == 2u);
	CAN_RxHeaderTypeDef rh;
	uint8_t d[8];
	REQUIRE(HAL_CAN_GetRxMessage(&h, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(hostCanIrqPending(0) == 0u);
	irq(h);
	CHECK(canCounts.rx0Pending == 2u);
}

TEST_CASE_FIXTURE(Q, "CQ3 an empty FIFO 0 with FMPIE0 runs no callback") {
	start(h);
	enable(h, kIerFmp0 | kIerFmp1);
	irq(h);
	CHECK(canCounts.orderCount == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ3 a message in FIFO 0 without FMPIE0 runs no callback") {
	start(h);
	enable(h, kIerFf0 | kIerFov0 | kIerFmp1);
	rx(h, 0x100, 1);
	irq(h);
	CHECK(canCounts.rx0Pending == 0u);
	CHECK(canCounts.orderCount == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ3 a message in FIFO 1 without FMPIE1 runs no callback") {
	start(h);
	enable(h, kIerFf1 | kIerFov1 | kIerFmp0);
	rx(h, 0x200, 1);
	irq(h);
	CHECK(canCounts.orderCount == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ3 FIFO 0 full with FFIE0 runs the full callback once and clears FULL0") {
	start(h);
	enable(h, kIerFf0);
	rx(h, 0x100, 3);
	REQUIRE((CAN1->RF0R & kRfFull) != 0u);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_FIFO0);
	irq(h);
	CHECK(canCounts.rx0Full == 1u);
	CHECK(canCounts.rx0Pending == 0u);
	CHECK((CAN1->RF0R & kRfFull) == 0u);
	CHECK(hostCanIrqPending(0) == 0u);
	irq(h);
	CHECK(canCounts.rx0Full == 1u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(Q, "CQ3 FIFO 1 full with FFIE1 runs the full callback once and clears FULL1") {
	start(h);
	enable(h, kIerFf1);
	rx(h, 0x200, 3);
	REQUIRE((CAN1->RF1R & kRfFull) != 0u);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_FIFO1);
	irq(h);
	CHECK(canCounts.rx1Full == 1u);
	CHECK(canCounts.rx0Full == 0u);
	CHECK((CAN1->RF1R & kRfFull) == 0u);
	irq(h);
	CHECK(canCounts.rx1Full == 1u);
}

TEST_CASE_FIXTURE(Q, "CQ3 a full FIFO 0 without FFIE0 runs no full callback and keeps FULL0") {
	start(h);
	enable(h, kIerFmp0 | kIerFov0);
	rx(h, 0x100, 3);
	irq(h);
	CHECK(canCounts.rx0Full == 0u);
	CHECK((CAN1->RF0R & kRfFull) != 0u);
	CHECK(canCounts.rx0Pending == 1u);
}

TEST_CASE_FIXTURE(Q, "CQ3 a full FIFO 1 without FFIE1 runs no full callback and keeps FULL1") {
	start(h);
	enable(h, kIerFmp1 | kIerFov1);
	rx(h, 0x200, 3);
	irq(h);
	CHECK(canCounts.rx1Full == 0u);
	CHECK((CAN1->RF1R & kRfFull) != 0u);
}

TEST_CASE_FIXTURE(Q, "CQ3 FFIE0 with FIFO 0 not full runs no full callback") {
	start(h);
	enable(h, kIerFf0 | kIerFf1);
	rx(h, 0x100, 2);
	rx(h, 0x200, 2);
	irq(h);
	CHECK(canCounts.orderCount == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ3 a FIFO 0 overrun with FOVIE0 adds RX_FOV0 to the error code, runs the error callback once and clears FOVR0") {
	start(h);
	enable(h, kIerFov0);
	rx(h, 0x100, 4);
	REQUIRE((CAN1->RF0R & kRfFovr) != 0u);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_FIFO0);
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_RX_FOV0);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.errorCodeAtError == HAL_CAN_ERROR_RX_FOV0);
	CHECK((CAN1->RF0R & kRfFovr) == 0u);
	CHECK(hostCanIrqPending(0) == 0u);
	irq(h);
	CHECK(canCounts.error == 1u);
}

TEST_CASE_FIXTURE(Q, "CQ3 a FIFO 1 overrun with FOVIE1 adds RX_FOV1 to the error code, runs the error callback once and clears FOVR1") {
	start(h);
	enable(h, kIerFov1);
	rx(h, 0x200, 4);
	REQUIRE((CAN1->RF1R & kRfFovr) != 0u);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_FIFO1);
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_RX_FOV1);
	CHECK(canCounts.error == 1u);
	CHECK((CAN1->RF1R & kRfFovr) == 0u);
	irq(h);
	CHECK(canCounts.error == 1u);
}

TEST_CASE_FIXTURE(Q, "CQ3 an overrun with the receive FIFO locked (RFLM) is reported like one without") {
	h.Init.ReceiveFifoLocked = ENABLE;
	start(h);
	enable(h, kIerFov0);
	rx(h, 0x100, 4);
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_RX_FOV0);
	CHECK(canCounts.error == 1u);
	CHECK((CAN1->RF0R & kRfFovr) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ3 an overrun without FOVIE keeps FOVR and the error code") {
	start(h);
	enable(h, kIerFmp0 | kIerFf0);
	rx(h, 0x100, 4);
	irq(h);
	CHECK((CAN1->RF0R & kRfFovr) != 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK(canCounts.error == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ3 a FIFO 1 overrun without FOVIE1 keeps FOVR1 and the error code") {
	start(h);
	enable(h, kIerFmp1 | kIerFf1 | kIerFov0);
	rx(h, 0x200, 4);
	irq(h);
	CHECK((CAN1->RF1R & kRfFovr) != 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK(canCounts.error == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ3 overruns of both FIFOs give one error callback with both error bits") {
	start(h);
	enable(h, kIerFov0 | kIerFov1);
	rx(h, 0x100, 4);
	rx(h, 0x200, 4);
	irq(h);
	CHECK(h.ErrorCode == (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1));
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.errorCodeAtError == (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1));
	CHECK((CAN1->RF0R & kRfFovr) == 0u);
	CHECK((CAN1->RF1R & kRfFovr) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ3 characterization: a second overrun after the clear is reported again and the error code only accumulates (NEEDS REVIEW 330)") {
	start(h);
	enable(h, kIerFov0);
	h.ErrorCode = HAL_CAN_ERROR_NOT_STARTED;
	rx(h, 0x100, 4);
	irq(h);
	CHECK(h.ErrorCode == (HAL_CAN_ERROR_NOT_STARTED | HAL_CAN_ERROR_RX_FOV0));
	CHECK(canCounts.error == 1u);
	rx(h, 0x100, 1);
	irq(h);
	CHECK(canCounts.error == 2u);
	CHECK(h.ErrorCode == (HAL_CAN_ERROR_NOT_STARTED | HAL_CAN_ERROR_RX_FOV0));
}

TEST_CASE_FIXTURE(Q, "CQ3 characterization: a full, overrun FIFO 0 runs the full callback, the pending callback, then the error callback (NEEDS REVIEW 330)") {
	start(h);
	enable(h, kIerFmp0 | kIerFf0 | kIerFov0);
	rx(h, 0x100, 4);
	irq(h);
	REQUIRE(canCounts.orderCount == 3u);
	CHECK(canCounts.order[0] == CAN_CB_RX0_FULL);
	CHECK(canCounts.order[1] == CAN_CB_RX0_PENDING);
	CHECK(canCounts.order[2] == CAN_CB_ERROR);
}

TEST_CASE_FIXTURE(Q, "CQ3 a full, overrun FIFO 1 runs the full callback, the pending callback, then the error callback") {
	start(h);
	enable(h, kIerFmp1 | kIerFf1 | kIerFov1);
	rx(h, 0x200, 4);
	irq(h);
	REQUIRE(canCounts.orderCount == 3u);
	CHECK(canCounts.order[0] == CAN_CB_RX1_FULL);
	CHECK(canCounts.order[1] == CAN_CB_RX1_PENDING);
	CHECK(canCounts.order[2] == CAN_CB_ERROR);
}

// ---------------------------------------------------------------------------------------------------------------- CQ4

// Note: ActivateNotification / DeactivateNotification are refused in SLEEP_ACTIVE (CQ1), so the sleep tests set IER before RequestSleep
// and change it with a register write afterwards.

TEST_CASE_FIXTURE(Q, "CQ4 bus activity in sleep with WKUIE runs the wake-up callback once and clears WKUI") {
	start(h);
	enable(h, kIerWkui);
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	hostCanBusActivity(0);
	REQUIRE((CAN1->MSR & kMsrWkui) != 0u);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_SCE);
	irq(h);
	CHECK(canCounts.wakeUp == 1u);
	CHECK(canCounts.sleep == 0u);
	CHECK(canCounts.error == 0u);
	CHECK((CAN1->MSR & kMsrWkui) == 0u);
	CHECK(hostCanIrqPending(0) == 0u);
	irq(h);
	CHECK(canCounts.wakeUp == 1u);
}

TEST_CASE_FIXTURE(Q, "CQ4 WKUI without WKUIE runs no callback and stays set") {
	start(h);
	enable(h, kIerErr);
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	hostCanBusActivity(0);
	REQUIRE((CAN1->MSR & kMsrWkui) != 0u);
	CHECK(hostCanIrqPending(0) == 0u);
	irq(h);
	CHECK(canCounts.wakeUp == 0u);
	CHECK((CAN1->MSR & kMsrWkui) != 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 WKUIE without a wake-up event runs no callback") {
	start(h);
	enable(h, kIerWkui);
	irq(h);
	CHECK(canCounts.orderCount == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 entering sleep with SLKIE runs the sleep callback once and clears SLAKI") {
	start(h);
	enable(h, kIerSlki);
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	REQUIRE((CAN1->MSR & kMsrSlaki) != 0u);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_SCE);
	irq(h);
	CHECK(canCounts.sleep == 1u);
	CHECK(canCounts.wakeUp == 0u);
	CHECK((CAN1->MSR & kMsrSlaki) == 0u);
	CHECK(hostCanIrqPending(0) == 0u);
	irq(h);
	CHECK(canCounts.sleep == 1u);
}

TEST_CASE_FIXTURE(Q, "CQ4 SLAKI with SLKIE cleared before the handler runs no callback and stays set") {
	start(h);
	enable(h, kIerSlki);
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	CAN1->IER = 0;
	REQUIRE((CAN1->MSR & kMsrSlaki) != 0u);
	irq(h);
	CHECK(canCounts.sleep == 0u);
	CHECK((CAN1->MSR & kMsrSlaki) != 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 SLKIE without a sleep event runs no callback") {
	start(h);
	enable(h, kIerSlki);
	irq(h);
	CHECK(canCounts.orderCount == 0u);
}

namespace {
// ERRIE plus the sources in `src`, raise the error, run the handler.
void cqErrSetup(Q &q, uint32_t src) {
	q.start(q.h);
	q.enable(q.h, kIerErr | src);
}
}  // namespace

TEST_CASE_FIXTURE(Q, "CQ4 an error warning with ERRIE and EWGIE adds EWG to the error code, runs the error callback and clears ERRI") {
	cqErrSetup(*this, kIerEwg);
	hostCanSetCounters(0, 96, 0);  // RM0090: EWGF when REC or TEC reaches 96
	REQUIRE((CAN1->ESR & 1u) != 0u);
	REQUIRE((CAN1->MSR & kMsrErri) != 0u);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_SCE);
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_EWG);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.errorCodeAtError == HAL_CAN_ERROR_EWG);
	CHECK((CAN1->MSR & kMsrErri) == 0u);
	CHECK((CAN1->ESR & 1u) != 0u);  // EWGF is read only and stays while the counter is high
	CHECK(hostCanIrqPending(0) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 an error passive state with ERRIE and EPVIE adds EPV to the error code") {
	cqErrSetup(*this, kIerEpv);
	hostCanSetCounters(0, 128, 0);  // RM0090: error passive above 127
	REQUIRE((CAN1->ESR & 2u) != 0u);
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_EPV);
	CHECK(canCounts.error == 1u);
	CHECK((CAN1->MSR & kMsrErri) == 0u);
	CHECK((CAN1->ESR & 2u) != 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 bus off with ERRIE and BOFIE adds BOF to the error code") {
	cqErrSetup(*this, kIerBof);
	hostCanSetCounters(0, 0, 256);  // RM0090: bus off when TEC exceeds 255
	REQUIRE((CAN1->ESR & 4u) != 0u);
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_BOF);
	CHECK(canCounts.error == 1u);
	CHECK((CAN1->MSR & kMsrErri) == 0u);
	CHECK((CAN1->ESR & 4u) != 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 the last error codes 1 to 6 map to STF, FOR, ACK, BR, BD and CRC and are cleared") {
	// RM0090 ESR LEC coding and UM1725 names.
	const struct { unsigned lec; uint32_t err; } map[6] = {{1, HAL_CAN_ERROR_STF}, {2, HAL_CAN_ERROR_FOR}, {3, HAL_CAN_ERROR_ACK},
	                                                       {4, HAL_CAN_ERROR_BR}, {5, HAL_CAN_ERROR_BD}, {6, HAL_CAN_ERROR_CRC}};
	cqErrSetup(*this, kIerLec);
	for (unsigned k = 0; k < 6; ++k) {
		h.ErrorCode = HAL_CAN_ERROR_NONE;
		hostCanRaiseError(0, map[k].lec);
		REQUIRE((CAN1->MSR & kMsrErri) != 0u);
		irq(h);
		CHECK(h.ErrorCode == map[k].err);
		CHECK(canCounts.error == k + 1u);
		CHECK(canCounts.errorCodeAtError == map[k].err);
		CHECK(((CAN1->ESR >> 4) & 7u) == 0u);
		CHECK((CAN1->MSR & kMsrErri) == 0u);
	}
}

TEST_CASE_FIXTURE(Q, "CQ4 characterization: a software set last error code 7 maps to no error but is cleared (NEEDS REVIEW 330)") {
	cqErrSetup(*this, kIerLec | kIerEwg);
	hostCanSetCounters(0, 96, 0);  // makes ERRI rise; EWGIE is switched off again so only LEC is looked at
	CAN1->IER = kIerErr | kIerLec;
	hostCanRaiseError(0, 7);
	REQUIRE((CAN1->MSR & kMsrErri) != 0u);
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK(canCounts.error == 0u);
	CHECK(((CAN1->ESR >> 4) & 7u) == 0u);
	CHECK((CAN1->MSR & kMsrErri) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 the last error code is ignored without LECIE and stays in ESR") {
	cqErrSetup(*this, kIerEwg);
	hostCanSetCounters(0, 96, 0);
	hostCanRaiseError(0, 3);
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_EWG);  // no ACK
	CHECK(((CAN1->ESR >> 4) & 7u) == 3u);
}

TEST_CASE_FIXTURE(Q, "CQ4 an error source without ERRIE runs no error callback and ERRI stays") {
	start(h);
	enable(h, kIerEwg | kIerErr);
	hostCanSetCounters(0, 96, 0);
	REQUIRE((CAN1->MSR & kMsrErri) != 0u);
	CAN1->IER = kIerEwg;  // ERRIE off
	irq(h);
	CHECK(canCounts.error == 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK((CAN1->MSR & kMsrErri) != 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 ERRIE without ERRI runs no error callback") {
	start(h);
	enable(h, kIerErr | kIerLec | kIerEwg);
	irq(h);
	CHECK(canCounts.error == 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(Q, "CQ4 a warning flag without its enable bit adds no error but ERRI is cleared") {
	start(h);
	enable(h, kIerErr | kIerEwg);
	hostCanSetCounters(0, 96, 0);
	REQUIRE((CAN1->MSR & kMsrErri) != 0u);
	CAN1->IER = kIerErr;  // EWGIE off
	irq(h);
	CHECK(canCounts.error == 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK((CAN1->MSR & kMsrErri) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 an error passive flag without EPVIE adds no EPV while EWGIE reports the warning") {
	cqErrSetup(*this, kIerEwg);
	hostCanSetCounters(0, 130, 0);  // EWGF and EPVF
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_EWG);
}

TEST_CASE_FIXTURE(Q, "CQ4 a bus off flag without BOFIE adds no BOF") {
	cqErrSetup(*this, kIerEwg);
	hostCanSetCounters(0, 0, 300);
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_EWG);
}

TEST_CASE_FIXTURE(Q, "CQ4 a bus off enable with BOFF clear (ERRI from the warning) adds no BOF") {
	cqErrSetup(*this, kIerEwg);
	hostCanSetCounters(0, 96, 0);  // EWGF only: BOFF clear (TEC <= 255), EPVF clear (<= 127)
	REQUIRE((CAN1->ESR & 4u) == 0u);
	CAN1->IER = kIerErr | kIerBof;
	irq(h);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_BOF) == 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK((CAN1->MSR & kMsrErri) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 a bus off enable with BOFF clear while error passive is set adds no BOF") {
	cqErrSetup(*this, kIerEpv);
	hostCanSetCounters(0, 130, 0);  // EWGF and EPVF, BOFF clear
	REQUIRE((CAN1->ESR & 4u) == 0u);
	CAN1->IER = kIerErr | kIerBof;
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(Q, "CQ4 an error passive enable with EPVF clear (warning only) adds no EPV") {
	cqErrSetup(*this, kIerEwg);
	hostCanSetCounters(0, 96, 0);
	REQUIRE((CAN1->ESR & 2u) == 0u);
	CAN1->IER = kIerErr | kIerEpv;
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK(canCounts.error == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 warning, passive and bus off enables with all three flags clear and only a last error code add only that code") {
	cqErrSetup(*this, kIerEwg | kIerEpv | kIerBof | kIerLec);
	hostCanRaiseError(0, 3);
	REQUIRE((CAN1->ESR & 7u) == 0u);
	irq(h);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_ACK);
}

TEST_CASE_FIXTURE(Q, "CQ4 combined errors give one error callback with the accumulated error code") {
	cqErrSetup(*this, kIerEwg | kIerEpv | kIerBof | kIerLec);
	hostCanSetCounters(0, 0, 300);  // EWGF, EPVF, BOFF
	hostCanRaiseError(0, 3);        // ACK
	irq(h);
	const uint32_t want = HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_ACK;
	CHECK(h.ErrorCode == want);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.errorCodeAtError == want);
	CHECK(((CAN1->ESR >> 4) & 7u) == 0u);
	CHECK((CAN1->MSR & kMsrErri) == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ4 the error code of earlier events is kept and the new bits are added") {
	cqErrSetup(*this, kIerLec);
	h.ErrorCode = HAL_CAN_ERROR_RX_FOV1;
	hostCanRaiseError(0, 6);
	irq(h);
	CHECK(h.ErrorCode == (HAL_CAN_ERROR_RX_FOV1 | HAL_CAN_ERROR_CRC));
	CHECK(canCounts.errorCodeAtError == (HAL_CAN_ERROR_RX_FOV1 | HAL_CAN_ERROR_CRC));
}

TEST_CASE_FIXTURE(Q, "CQ4 two errors in two handler calls accumulate in the error code and call the callback twice") {
	cqErrSetup(*this, kIerLec);
	hostCanRaiseError(0, 1);
	irq(h);
	hostCanRaiseError(0, 5);
	irq(h);
	CHECK(h.ErrorCode == (HAL_CAN_ERROR_STF | HAL_CAN_ERROR_BD));
	CHECK(canCounts.error == 2u);
	CHECK(canCounts.errorCodeAtError == (HAL_CAN_ERROR_STF | HAL_CAN_ERROR_BD));
}

TEST_CASE_FIXTURE(Q, "CQ4 a handler call after the clear raises no further error callback") {
	cqErrSetup(*this, kIerLec);
	hostCanRaiseError(0, 2);
	irq(h);
	irq(h);
	CHECK(canCounts.error == 1u);
}

// ---------------------------------------------------------------------------------------------------------------- CQ5

TEST_CASE_FIXTURE(Q, "CQ5 the handler of CAN2 reports its own transmit, FIFO and error events with the handle of CAN2") {
	start(h2);
	enable(h2, kIerTme | kIerFmp0 | kIerLec | kIerErr);
	send(h2, 0x10);
	rx(h2, 0x100, 1);
	hostCanRaiseError(1, 4);  // bit recessive
	REQUIRE(hostCanIrqPending(1) == (HOST_CAN_IRQ_TX | HOST_CAN_IRQ_FIFO0 | HOST_CAN_IRQ_SCE));
	irq(h2);
	CHECK(canCounts.txComplete[0] == 1u);
	CHECK(canCounts.rx0Pending == 1u);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.last == &h2);
	CHECK(h2.ErrorCode == HAL_CAN_ERROR_BR);
	CHECK(hostCanIrqPending(1) == HOST_CAN_IRQ_FIFO0);  // the message is still in the FIFO
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(Q, "CQ5 the handler of CAN2 ignores the flags of CAN1") {
	start(h);
	start(h2);
	enable(h, kIerFmp0 | kIerLec | kIerErr);
	enable(h2, kIerFmp0 | kIerLec | kIerErr);
	rx(h, 0x100, 1);
	hostCanRaiseError(0, 4);
	irq(h2);
	CHECK(canCounts.orderCount == 0u);
	CHECK(hostCanIrqPending(0) == (HOST_CAN_IRQ_FIFO0 | HOST_CAN_IRQ_SCE));
	CHECK(hostCanIrqPending(1) == 0u);
	irq(h);
	CHECK(canCounts.rx0Pending == 1u);
	CHECK(canCounts.error == 1u);
	CHECK(canCounts.last == &h);
}

TEST_CASE_FIXTURE(Q, "CQ5 the callbacks see the handle in the state LISTENING") {
	start(h);
	enable(h, kIerTme);
	send(h, 0x10);
	irq(h);
	CHECK(canCounts.last == &h);
	CHECK(canCounts.stateAtLast == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(Q, "CQ5 the four interrupt request lines are pending exactly for an enabled source with its flag") {
	start(h);
	CHECK(hostCanIrqPending(0) == 0u);
	send(h, 0x10);       // TXOK0 / RQCP0
	rx(h, 0x100, 1);     // FIFO 0
	rx(h, 0x200, 1);     // FIFO 1
	hostCanSetCounters(0, 96, 0);
	CHECK(hostCanIrqPending(0) == 0u);  // nothing enabled
	enable(h, kIerTme);
	CHECK(hostCanIrqPending(0) == HOST_CAN_IRQ_TX);
	enable(h, kIerFmp0);
	CHECK(hostCanIrqPending(0) == (HOST_CAN_IRQ_TX | HOST_CAN_IRQ_FIFO0));
	enable(h, kIerFmp1);
	CHECK(hostCanIrqPending(0) == (HOST_CAN_IRQ_TX | HOST_CAN_IRQ_FIFO0 | HOST_CAN_IRQ_FIFO1));
}

TEST_CASE_FIXTURE(Q, "CQ5 characterization: all sources in one handler call give the callbacks in the order TX, FIFO 0, FIFO 1, sleep, wake-up, error (NEEDS REVIEW 330)") {
	start(h);
	enable(h, 0x38F7Fu);
	threeFrames(h, kOk);  // TXOK of mailbox 0, 1, 2
	rx(h, 0x100, 4);      // FIFO 0 full and overrun
	rx(h, 0x200, 4);      // FIFO 1 full and overrun
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);  // SLAKI (SLKIE is on)
	hostCanBusActivity(0);                        // WKUI
	hostCanSetCounters(0, 0, 300);                // EWGF, EPVF, BOFF
	hostCanRaiseError(0, 6);                      // CRC
	REQUIRE(hostCanIrqPending(0) == (HOST_CAN_IRQ_TX | HOST_CAN_IRQ_FIFO0 | HOST_CAN_IRQ_FIFO1 | HOST_CAN_IRQ_SCE));
	irq(h);
	const unsigned want[11] = {CAN_CB_TX0, CAN_CB_TX1, CAN_CB_TX2, CAN_CB_RX0_FULL, CAN_CB_RX0_PENDING, CAN_CB_RX1_FULL,
	                           CAN_CB_RX1_PENDING, CAN_CB_SLEEP, CAN_CB_WAKEUP, CAN_CB_ERROR, 0};
	REQUIRE(canCounts.orderCount == 10u);
	for (unsigned k = 0; k < 10u; ++k) CHECK(canCounts.order[k] == want[k]);
	const uint32_t code = HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1 | HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_CRC;
	CHECK(h.ErrorCode == code);
	CHECK(canCounts.errorCodeAtError == code);
	// Every cleared flag is gone; the FIFO messages are still there, so only the two FIFO lines remain.
	CHECK(hostCanIrqPending(0) == (HOST_CAN_IRQ_FIFO0 | HOST_CAN_IRQ_FIFO1));
}

TEST_CASE_FIXTURE(Q, "CQ5 every callback runs exactly once per event in a single handler call") {
	start(h);
	enable(h, 0x38F7Fu);
	threeFrames(h, kOk);
	rx(h, 0x100, 3);
	rx(h, 0x200, 3);
	irq(h);
	CHECK(canCounts.txComplete[0] == 1u);
	CHECK(canCounts.txComplete[1] == 1u);
	CHECK(canCounts.txComplete[2] == 1u);
	CHECK(canCounts.rx0Full == 1u);
	CHECK(canCounts.rx0Pending == 1u);
	CHECK(canCounts.rx1Full == 1u);
	CHECK(canCounts.rx1Pending == 1u);
	CHECK(canCounts.error == 0u);
	CHECK(canCounts.sleep + canCounts.wakeUp == 0u);
}

TEST_CASE_FIXTURE(Q, "CQ5 the handler works in loop back mode for a sent frame received through a filter") {
	h.Init.Mode = CAN_MODE_LOOPBACK;
	start(h);
	enable(h, kIerTme | kIerFmp0);
	send(h, 0x100);
	irq(h);
	CHECK(canCounts.txComplete[0] == 1u);
	CHECK(canCounts.rx0Pending == 1u);
	REQUIRE(canCounts.orderCount == 2u);
	CHECK(canCounts.order[0] == CAN_CB_TX0);
	CHECK(canCounts.order[1] == CAN_CB_RX0_PENDING);
}
