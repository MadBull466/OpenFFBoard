/*
 * host_target_can.cpp
 *
 * Fake HAL CAN (F8) of the F407 platform for ffb_target_tests: the bxCAN peripheral CAN1 (`hosttest_can1`, handle
 * `hcan1`) with its registers, 3 transmit mailboxes, 2 receive FIFOs of 3 frames, 28 filter banks and the interrupt
 * handler. Declarations in platform/target_f407/main.h, test control in support/HostTarget.h.
 *
 * Every HAL function is modelled on stm32f4xx_hal_can.c (Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src, read only;
 * the F407 device header defines CAN2, so the dual instance branches apply) and cited per function. The peripheral
 * side (what the hardware does between the HAL calls) follows RM0090 chapter 32 (bxCAN):
 *  - Mode changes (INRQ -> INAK, SLEEP -> SLAK) are acknowledged at once (bus idle); a timeout can be injected
 *    (canFailNextModeChange). No simulated time passes in any call.
 *  - A transmit request stays pending until the test completes it (canTxComplete, canTxError) or the firmware aborts
 *    it; an abort of a pending mailbox takes effect at once (the frame waits for the bus, it is not being sent).
 *  - Received frames pass the filter banks (canReceive) into the FIFO of the matching filter.
 *  - Interrupt lines: only CAN1_TX and CAN1_RX0 are enabled in the NVIC (HAL_CAN_MspInit, stm32f4xx_hal_msp.c:291-294),
 *    CAN1_RX1 and CAN1_SCE are not; their events are serviced by the handler at the next TX / RX0 interrupt. An interrupt is raised (HAL_CAN_IRQHandler in interrupt context) when an enabled
 *    line has a pending, enabled event; it is level triggered and raised again while the event stays pending.
 *
 * Not modelled (see HostTarget.h): bit timing and bus speed, time-triggered mode, error counters (TEC/REC), the
 * automatic bus-off management (ABOM) and automatic wake-up (AWUM), loop back / silent mode bits of BTR (recorded as
 * unmodelled at HAL_CAN_Start), the ownership of the filter banks by CAN1/CAN2 (FMR CAN2SB is written like the HAL but
 * all 28 banks filter CAN1), a frame that is being transmitted when an abort is requested.
 */
#include "HostTarget.h"
#include "HostPlatform.h"

#include "main.h"

#include <algorithm>
#include <array>
#include <deque>
#include <string>
#include <vector>

CAN_TypeDef hosttest_can1 = {};

namespace {
using HostTarget::state;

// ---- Register bits (stm32f407xx.h) ----
constexpr uint32_t MCR_INRQ = 0x00000001U, MCR_SLEEP = 0x00000002U, MCR_TXFP = 0x00000004U, MCR_RFLM = 0x00000008U,
				   MCR_NART = 0x00000010U, MCR_AWUM = 0x00000020U, MCR_ABOM = 0x00000040U, MCR_TTCM = 0x00000080U;
constexpr uint32_t MSR_INAK = 0x00000001U, MSR_SLAK = 0x00000002U, MSR_ERRI = 0x00000004U, MSR_WKUI = 0x00000008U,
				   MSR_SLAKI = 0x00000010U;
constexpr uint32_t RFR_FMP = 0x00000003U, RFR_FULL = 0x00000008U, RFR_FOVR = 0x00000010U;
constexpr uint32_t ESR_EWGF = 0x00000001U, ESR_EPVF = 0x00000002U, ESR_BOFF = 0x00000004U, ESR_LEC = 0x00000070U;
constexpr uint32_t ESR_LEC_0 = 0x00000010U, ESR_LEC_1 = 0x00000020U, ESR_LEC_2 = 0x00000040U;
constexpr uint32_t FMR_FINIT = 0x00000001U, FMR_CAN2SB = 0x00003F00U;
constexpr uint32_t BTR_LBKM = 0x40000000U, BTR_SILM = 0x80000000U;
constexpr uint32_t TI_TXRQ = 0x00000001U, TDT_TGT = 0x00000100U;
constexpr uint32_t TSR_CODE_POS = 24U, TSR_CODE = 0x03000000U;
constexpr uint32_t tsrRqcp(uint32_t m) { return 0x01U << (8U * m); }
constexpr uint32_t tsrTxok(uint32_t m) { return 0x02U << (8U * m); }
constexpr uint32_t tsrAlst(uint32_t m) { return 0x04U << (8U * m); }
constexpr uint32_t tsrTerr(uint32_t m) { return 0x08U << (8U * m); }
constexpr uint32_t tsrAbrq(uint32_t m) { return 0x80U << (8U * m); }
constexpr uint32_t tsrTme(uint32_t m) { return 0x01U << (26U + m); }
constexpr uint32_t TSR_RQCP_ALL = 0x00010101U;

// Reset values (RM0090 32.9: CAN_MCR 0x0001 0002, CAN_MSR 0x0000 0C02, CAN_TSR 0x1C00 0000, CAN_BTR 0x0123 0000,
// CAN_FMR 0x2A1C 0E01)
constexpr uint32_t MCR_RESET_VALUE = 0x00010002U, MSR_RESET_VALUE = 0x00000C02U, TSR_RESET_VALUE = 0x1C000000U,
				   BTR_RESET_VALUE = 0x01230000U, FMR_RESET_VALUE = 0x2A1C0E01U;

constexpr size_t CAN_FIFO_DEPTH = 3; // RM0090 32.7.5: 3 mailboxes per receive FIFO
constexpr uint32_t CAN_FILTER_BANKS = 28; // F407: 28 banks shared by CAN1 and CAN2

struct CanRxFrame {
	CAN_RxHeaderTypeDef header;
	std::array<uint8_t, 8> data;
};

struct CanFake {
	std::array<std::deque<CanRxFrame>, 2> fifos;
	std::vector<HostTarget::CanTxFrame> txFrames;
	std::array<long, 3> mailboxFrame{-1, -1, -1}; //!< Index in txFrames of the frame pending in a mailbox
	std::vector<CAN_FilterTypeDef> filterConfigs;
	std::vector<HostTarget::CanHalCall> calls;
	std::array<bool, 4> nvic{true, true, false, false}; //!< CAN1_TX, CAN1_RX0, CAN1_RX1, CAN1_SCE (HAL_CAN_MspInit)
	bool txAutoComplete = false;
	bool failNextModeChange = false;
	bool inIrq = false;
	uint32_t msgPendingCallbacks = 0; //!< HAL_CAN_RxFifoXMsgPendingCallback calls by the interrupt handler
};

/**
 * The state outlives all static objects (never destroyed): the global canport (CANPort_2B) calls the HAL in its
 * destructor when the executable exits.
 */
CanFake &fake() {
	static CanFake *f = new CanFake();
	return *f;
}

void unmodelled(const std::string &what) { state().unmodelledHalCalls.push_back(what); }

void updateTsrCode();

uint32_t record(const char *function, uint32_t arg, uint32_t result) {
	fake().calls.push_back({function, arg, result});
	return result;
}

volatile uint32_t &rfr(uint32_t fifo) { return fifo == 0 ? hosttest_can1.RF0R : hosttest_can1.RF1R; }

void syncFmp(uint32_t fifo) {
	rfr(fifo) = (rfr(fifo) & ~RFR_FMP) | static_cast<uint32_t>(fake().fifos[fifo].size());
}

/** Normal mode: not in initialisation mode (INAK) and not sleeping (SLAK), RM0090 32.4 */
bool normalMode() { return (hosttest_can1.MSR & (MSR_INAK | MSR_SLAK)) == 0U; }
bool busOff() { return (hosttest_can1.ESR & ESR_BOFF) != 0U; }
bool isCan1(const CAN_HandleTypeDef *hcan) { return hcan == &hcan1 && hcan->Instance == CAN1; }
bool readyOrListening(const CAN_HandleTypeDef *hcan) {
	return hcan->State == HAL_CAN_STATE_READY || hcan->State == HAL_CAN_STATE_LISTENING;
}

/** Interrupt lines with a pending, enabled event (RM0090 32.8, figure "Event flags and interrupt generation") */
bool linePending(HostTarget::CanIrqLine line) {
	const CAN_TypeDef &c = hosttest_can1;
	CanFake &f = fake();
	switch (line) {
	case HostTarget::CanIrqLine::Tx:
		return (c.IER & CAN_IT_TX_MAILBOX_EMPTY) && (c.TSR & TSR_RQCP_ALL);
	case HostTarget::CanIrqLine::Rx0:
		return ((c.IER & CAN_IT_RX_FIFO0_MSG_PENDING) && !f.fifos[0].empty()) || ((c.IER & CAN_IT_RX_FIFO0_FULL) && (c.RF0R & RFR_FULL)) ||
			   ((c.IER & CAN_IT_RX_FIFO0_OVERRUN) && (c.RF0R & RFR_FOVR));
	case HostTarget::CanIrqLine::Rx1:
		return ((c.IER & CAN_IT_RX_FIFO1_MSG_PENDING) && !f.fifos[1].empty()) || ((c.IER & CAN_IT_RX_FIFO1_FULL) && (c.RF1R & RFR_FULL)) ||
			   ((c.IER & CAN_IT_RX_FIFO1_OVERRUN) && (c.RF1R & RFR_FOVR));
	case HostTarget::CanIrqLine::Sce:
		return ((c.IER & CAN_IT_ERROR) && (c.MSR & MSR_ERRI)) || ((c.IER & CAN_IT_WAKEUP) && (c.MSR & MSR_WKUI)) ||
			   ((c.IER & CAN_IT_SLEEP_ACK) && (c.MSR & MSR_SLAKI));
	}
	return false;
}

bool anyLinePending() {
	for (uint8_t l = 0; l < 4; l++) {
		if (fake().nvic[l] && linePending(static_cast<HostTarget::CanIrqLine>(l))) {
			return true;
		}
	}
	return false;
}

/**
 * Raises the interrupt (HAL_CAN_IRQHandler(&hcan1) in interrupt context) while an NVIC-enabled line has a pending
 * event. Inside the handler (a callback calling the HAL) nothing is raised: the outer loop sees the new event, like a
 * pending interrupt of the same priority runs after the current one. Stops if the handler changed nothing (a callback
 * that does not read the FIFO: the target would re-enter the interrupt forever, counted in state().canRxStalls).
 * @return number of handler runs
 */
uint32_t raiseIrqs() {
	CanFake &f = fake();
	if (f.inIrq || hcan1.Instance != CAN1) {
		return 0;
	}
	uint32_t runs = 0;
	while (anyLinePending()) {
		const CAN_TypeDef before = hosttest_can1;
		const size_t fifo0 = f.fifos[0].size(), fifo1 = f.fifos[1].size();
		{
			HostPlatform::IsrContext isr;
			f.inIrq = true;
			HAL_CAN_IRQHandler(&hcan1);
			f.inIrq = false;
		}
		runs++;
		const CAN_TypeDef &now = hosttest_can1;
		if (now.TSR == before.TSR && now.RF0R == before.RF0R && now.RF1R == before.RF1R && now.MSR == before.MSR &&
			now.ESR == before.ESR && now.IER == before.IER && f.fifos[0].size() == fifo0 && f.fifos[1].size() == fifo1) {
			state().canRxStalls++;
			break;
		}
	}
	return runs;
}

/** The pending frame of mailbox m was transmitted and acknowledged (RM0090 32.7.1: RQCP, TXOK and TME set) */
void completeMailbox(uint32_t m) {
	CanFake &f = fake();
	hosttest_can1.TSR &= ~(tsrAlst(m) | tsrTerr(m) | tsrAbrq(m));
	hosttest_can1.TSR |= tsrRqcp(m) | tsrTxok(m) | tsrTme(m);
	hosttest_can1.sTxMailBox[m].TIR &= ~TI_TXRQ;
	if (f.mailboxFrame[m] >= 0) {
		f.txFrames[f.mailboxFrame[m]].result = HostTarget::CanTxResult::Sent;
	}
	f.mailboxFrame[m] = -1;
	updateTsrCode();
}

bool mailboxPending(uint32_t m) { return (hosttest_can1.TSR & tsrTme(m)) == 0U; }

/**
 * TSR CODE (RM0090 32.9.2): the number of the next empty mailbox; with all mailboxes pending the number of the
 * mailbox with the lowest transmit priority (RM0090 32.7.1: MCR TXFP = 0 by identifier, the highest identifier in
 * arbitration order loses, equal identifiers: the higher mailbox number; TXFP = 1 by request order, the newest loses).
 */
void updateTsrCode() {
	CAN_TypeDef &c = hosttest_can1;
	uint32_t code = 0;
	bool found = false;
	for (uint32_t m = 0; m < 3 && !found; m++) {
		if (c.TSR & tsrTme(m)) {
			code = m;
			found = true;
		}
	}
	if (!found) {
		for (uint32_t m = 1; m < 3; m++) {
			bool lower;
			if (c.MCR & MCR_TXFP) {
				lower = fake().mailboxFrame[m] > fake().mailboxFrame[code];
			} else {
				// TIR without TXRQ in arbitration order: STID / EXID[28:18], then (standard: RTR, IDE 0) or (IDE 1, EXID[17:0], RTR)
				lower = (c.sTxMailBox[m].TIR & ~TI_TXRQ) >= (c.sTxMailBox[code].TIR & ~TI_TXRQ);
			}
			code = lower ? m : code;
		}
	}
	c.TSR = (c.TSR & ~TSR_CODE) | (code << TSR_CODE_POS);
}

/** Transmission is possible: normal mode, not bus-off (RM0090 32.4, 32.7.6) */
bool canTransmit() { return normalMode() && !busOff(); }

/** Auto-complete (HostTarget::canSetTxAutoComplete): the bus acknowledged all pending frames since the last call */
void beginCall() {
	CanFake &f = fake();
	if (!f.txAutoComplete || f.inIrq || !canTransmit()) {
		return;
	}
	bool any = false;
	for (uint32_t m = 0; m < 3; m++) {
		if (mailboxPending(m)) {
			completeMailbox(m);
			any = true;
		}
	}
	if (any) {
		raiseIrqs();
	}
}

/** Leaves the initialisation mode (INRQ cleared, INAK cleared) unless a timeout is injected */
bool modeChange() {
	CanFake &f = fake();
	if (f.failNextModeChange) {
		f.failNextModeChange = false;
		return false;
	}
	return true;
}

/** Filter element of a frame in the 32 bit filter register layout (RM0090 figure 230): STID/EXID, IDE, RTR */
uint32_t frameReg32(uint32_t id, bool ext, bool rtr) {
	return ext ? (((id & 0x1FFFFFFFU) << 3) | 0x4U | (rtr ? 0x2U : 0U)) : (((id & 0x7FFU) << 21) | (rtr ? 0x2U : 0U));
}
/** ... in the 16 bit layout: STID[10:0] (or EXID[28:18]) 15:5, RTR 4, IDE 3, EXID[17:15] 2:0 */
uint32_t frameReg16(uint32_t id, bool ext, bool rtr) {
	uint32_t upper = ext ? ((id >> 18) & 0x7FFU) : (id & 0x7FFU);
	return (upper << 5) | (rtr ? 0x10U : 0U) | (ext ? 0x8U : 0U) | (ext ? ((id >> 15) & 0x7U) : 0U);
}

struct FilterMatch {
	bool found = false;
	uint32_t fifo = 0;
	uint32_t fmi = 0;
	uint32_t bank = 0;
	bool scale32 = false;
	bool list = false;
	uint32_t element = 0;
};

/** Filter elements per bank (RM0090 32.7.4 "Filter match index"): 32 bit mask 1, 32 bit list 2, 16 bit mask 2, 16 bit list 4 */
uint32_t elementsOfBank(uint32_t b) {
	bool scale32 = (hosttest_can1.FS1R >> b) & 1U;
	bool list = (hosttest_can1.FM1R >> b) & 1U;
	return scale32 ? (list ? 2U : 1U) : (list ? 4U : 2U);
}

/**
 * Acceptance filtering (RM0090 32.7.4): every active bank is checked; on several matches the priority rules apply
 * (32 bit before 16 bit, identifier list before mask, then the lower filter number). The filter match index counts
 * the filter elements of all lower banks assigned to the same FIFO (active or not) plus the element in the bank.
 * The ownership of the banks by CAN1/CAN2 (FMR CAN2SB) is not applied: all 28 banks filter CAN1.
 * While FINIT is set the reception is deactivated (the HAL clears it before it returns).
 */
FilterMatch matchFilters(uint32_t id, bool ext, bool rtr) {
	const CAN_TypeDef &c = hosttest_can1;
	FilterMatch best;
	if (c.FMR & FMR_FINIT) {
		return best;
	}
	const uint32_t f32 = frameReg32(id, ext, rtr), f16 = frameReg16(id, ext, rtr);
	for (uint32_t b = 0; b < CAN_FILTER_BANKS; b++) {
		if (((c.FA1R >> b) & 1U) == 0U) {
			continue;
		}
		const bool scale32 = (c.FS1R >> b) & 1U;
		const bool list = (c.FM1R >> b) & 1U;
		const uint32_t fr1 = c.sFilterRegister[b].FR1, fr2 = c.sFilterRegister[b].FR2;
		int element = -1;
		if (scale32 && !list) {
			element = ((f32 ^ fr1) & fr2) == 0U ? 0 : -1;
		} else if (scale32 && list) {
			element = f32 == fr1 ? 0 : (f32 == fr2 ? 1 : -1);
		} else if (!scale32 && !list) {
			if (((f16 ^ fr1) & (fr1 >> 16) & 0xFFFFU) == 0U) {
				element = 0;
			} else if (((f16 ^ fr2) & (fr2 >> 16) & 0xFFFFU) == 0U) {
				element = 1;
			}
		} else {
			const uint32_t ids[4] = {fr1 & 0xFFFFU, fr1 >> 16, fr2 & 0xFFFFU, fr2 >> 16};
			for (int i = 0; i < 4 && element < 0; i++) {
				element = ids[i] == f16 ? i : -1;
			}
		}
		if (element < 0) {
			continue;
		}
		bool better = !best.found || (scale32 && !best.scale32) || (scale32 == best.scale32 && list && !best.list);
		if (better) {
			best.found = true;
			best.bank = b;
			best.scale32 = scale32;
			best.list = list;
			best.element = static_cast<uint32_t>(element);
			best.fifo = (c.FFA1R >> b) & 1U;
		}
	}
	if (best.found) {
		uint32_t fmi = 0;
		for (uint32_t b = 0; b < best.bank; b++) {
			if (((c.FFA1R >> b) & 1U) == best.fifo) {
				fmi += elementsOfBank(b);
			}
		}
		best.fmi = fmi + best.element;
	}
	return best;
}

/** Peripheral and handle as after MX_CAN1_Init (main.c:486-511) on a freshly reset peripheral */
void initCan1() {
	hosttest_can1 = CAN_TypeDef();
	hosttest_can1.MCR = MCR_RESET_VALUE;
	hosttest_can1.MSR = MSR_RESET_VALUE;
	hosttest_can1.TSR = TSR_RESET_VALUE;
	hosttest_can1.BTR = BTR_RESET_VALUE;
	hosttest_can1.FMR = FMR_RESET_VALUE;
	hcan1 = CAN_HandleTypeDef();
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 6;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	HAL_CAN_Init(&hcan1);
}
} // namespace

namespace HostTarget {

void resetCan() {
	CanFake &f = fake();
	f = CanFake();
	initCan1();
	f.calls.clear(); // the HAL_CAN_Init of MX_CAN1_Init is not a call of the test
}

// ---- Transmit ----
const std::vector<CanTxFrame> &canTxFrames() { return fake().txFrames; }

void canClearTxFrames() {
	CanFake &f = fake();
	f.txFrames.clear();
	f.mailboxFrame = {-1, -1, -1};
}

uint32_t canTxPending() {
	uint32_t mask = 0;
	for (uint32_t m = 0; m < 3; m++) {
		mask |= mailboxPending(m) ? (1U << m) : 0U;
	}
	return mask;
}

bool canTxComplete(uint32_t mailbox) {
	if (mailbox > 2 || !mailboxPending(mailbox) || !canTransmit()) {
		return false;
	}
	completeMailbox(mailbox);
	raiseIrqs();
	return true;
}

uint32_t canTxCompleteAll() {
	uint32_t count = 0;
	if (!canTransmit()) {
		return 0;
	}
	for (uint32_t m = 0; m < 3; m++) {
		if (mailboxPending(m)) {
			completeMailbox(m);
			count++;
		}
	}
	raiseIrqs();
	return count;
}

bool canTxError(uint32_t mailbox, bool arbitrationLost) {
	CanFake &f = fake();
	if (mailbox > 2 || !mailboxPending(mailbox) || !canTransmit()) {
		return false;
	}
	// RM0090 32.9.2 CAN_TSR: ALST / TERR of the last attempt. With automatic retransmission (NART = 0, MX_CAN1_Init) the
	// request stays pending and is retried; with NART = 1 the request completes (RQCP set, TXOK 0, TME set)
	hosttest_can1.TSR &= ~(tsrAlst(mailbox) | tsrTerr(mailbox) | tsrTxok(mailbox));
	hosttest_can1.TSR |= arbitrationLost ? tsrAlst(mailbox) : tsrTerr(mailbox);
	if (hosttest_can1.MCR & MCR_NART) {
		hosttest_can1.TSR |= tsrRqcp(mailbox) | tsrTme(mailbox);
		hosttest_can1.sTxMailBox[mailbox].TIR &= ~TI_TXRQ;
		if (f.mailboxFrame[mailbox] >= 0) {
			f.txFrames[f.mailboxFrame[mailbox]].result = CanTxResult::Failed;
		}
		f.mailboxFrame[mailbox] = -1;
		updateTsrCode();
		raiseIrqs();
	}
	return true;
}

void canSetTxAutoComplete(bool on) { fake().txAutoComplete = on; }

// ---- Errors and interrupts ----
void canSetErrorState(CanErrorState errorState) {
	uint32_t flags = 0;
	switch (errorState) {
	case CanErrorState::Active:
		break;
	case CanErrorState::Warning:
		flags = ESR_EWGF;
		break;
	case CanErrorState::Passive:
		flags = ESR_EWGF | ESR_EPVF;
		break;
	case CanErrorState::BusOff:
		flags = ESR_EWGF | ESR_EPVF | ESR_BOFF;
		break;
	}
	const uint32_t newFlags = flags & ~hosttest_can1.ESR;
	hosttest_can1.ESR = (hosttest_can1.ESR & ~(ESR_EWGF | ESR_EPVF | ESR_BOFF)) | flags;
	// RM0090 32.9.2 CAN_MSR ERRI: set when an error flag of CAN_ESR is set and its interrupt is enabled in CAN_IER
	const uint32_t ier = hosttest_can1.IER;
	if (((newFlags & ESR_EWGF) && (ier & CAN_IT_ERROR_WARNING)) || ((newFlags & ESR_EPVF) && (ier & CAN_IT_ERROR_PASSIVE)) ||
		((newFlags & ESR_BOFF) && (ier & CAN_IT_BUSOFF))) {
		hosttest_can1.MSR |= MSR_ERRI;
	}
	raiseIrqs();
}

void canSetLastErrorCode(uint8_t lec) {
	hosttest_can1.ESR = (hosttest_can1.ESR & ~ESR_LEC) | ((static_cast<uint32_t>(lec) << 4) & ESR_LEC);
	if (lec != 0 && (hosttest_can1.IER & CAN_IT_LAST_ERROR_CODE)) {
		hosttest_can1.MSR |= MSR_ERRI;
	}
	raiseIrqs();
}

void canSetIrqLineEnabled(CanIrqLine line, bool enabled) {
	fake().nvic[static_cast<uint8_t>(line)] = enabled;
	raiseIrqs();
}

bool canIrqLineEnabled(CanIrqLine line) { return fake().nvic[static_cast<uint8_t>(line)]; }

uint32_t canIrq() {
	beginCall();
	return raiseIrqs();
}

void canFailNextModeChange() { fake().failNextModeChange = true; }

// ---- Filters ----
bool canAccepts(uint32_t id, bool extId, bool rtr, uint32_t *fifo, uint32_t *filterMatchIndex) {
	FilterMatch m = matchFilters(id, extId, rtr);
	if (m.found) {
		if (fifo) {
			*fifo = m.fifo;
		}
		if (filterMatchIndex) {
			*filterMatchIndex = m.fmi;
		}
	}
	return m.found;
}

uint32_t canActiveFilterBanks() {
	uint32_t count = 0;
	for (uint32_t b = 0; b < CAN_FILTER_BANKS; b++) {
		count += (hosttest_can1.FA1R >> b) & 1U;
	}
	return count;
}

const std::vector<CAN_FilterTypeDef> &canFilterConfigs() { return fake().filterConfigs; }

const std::vector<CanHalCall> &canHalCalls() { return fake().calls; }
void canClearHalCalls() { fake().calls.clear(); }

// ---- Receive ----
void canQueueRx(uint32_t fifo, const CAN_RxHeaderTypeDef &header, const uint8_t data[8]) {
	std::deque<CanRxFrame> &queue = fake().fifos.at(fifo);
	CanRxFrame frame;
	frame.header = header;
	// Stored like the mailbox registers CAN_RIxR / CAN_RDTxR (stm32f407xx.h field widths)
	frame.header.StdId &= 0x7FFU;
	frame.header.ExtId &= 0x1FFFFFFFU;
	frame.header.IDE &= CAN_ID_EXT;
	frame.header.RTR &= CAN_RTR_REMOTE;
	frame.header.DLC &= 0xFU;
	frame.header.FilterMatchIndex &= 0xFFU;
	frame.header.Timestamp &= 0xFFFFU;
	std::copy(data, data + 8, frame.data.begin());
	if (queue.size() >= CAN_FIFO_DEPTH) {
		// RM0090 32.7.5 overrun: FOVR set; not locked (RFLM 0, MX_CAN1_Init) the newest frame is overwritten, locked the
		// new frame is discarded
		rfr(fifo) |= RFR_FOVR;
		if ((hosttest_can1.MCR & MCR_RFLM) == 0U) {
			queue.back() = frame;
		}
		state().canRxOverruns[fifo]++;
		return;
	}
	queue.push_back(frame);
	if (queue.size() == CAN_FIFO_DEPTH) {
		rfr(fifo) |= RFR_FULL;
	}
	syncFmp(fifo);
}

size_t canRxPending(uint32_t fifo) { return fake().fifos.at(fifo).size(); }

uint32_t canRxInterrupt(uint32_t fifo) {
	CanFake &f = fake();
	uint32_t calls = 0;
	while (!f.fifos.at(fifo).empty()) {
		size_t before = f.fifos[fifo].size();
		{
			HostPlatform::IsrContext isr;
			f.inIrq = true; // HAL calls of the callback raise no further interrupt
			if (fifo == CAN_RX_FIFO0) {
				HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
			} else {
				HAL_CAN_RxFifo1MsgPendingCallback(&hcan1);
			}
			f.inIrq = false;
		}
		calls++;
		if (f.fifos[fifo].size() >= before) {
			state().canRxStalls++; // The target would re-enter the interrupt forever
			break;
		}
	}
	return calls;
}

uint32_t canReceive(uint32_t id, const std::vector<uint8_t> &data, bool extId, bool rtr) {
	// RM0090 32.4: frames are received in normal mode only (not in initialisation or sleep mode, not bus-off)
	if (hcan1.Instance != CAN1 || !normalMode() || busOff()) {
		return 0;
	}
	FilterMatch match = matchFilters(id, extId, rtr);
	if (!match.found) {
		return 0; // No acceptance filter matches: the frame is not stored (RM0090 32.7.4)
	}
	CAN_RxHeaderTypeDef header = {};
	header.StdId = extId ? 0 : id;
	header.ExtId = extId ? id : 0;
	header.IDE = extId ? CAN_ID_EXT : CAN_ID_STD;
	header.RTR = rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
	header.DLC = static_cast<uint32_t>(std::min<size_t>(data.size(), 8));
	header.FilterMatchIndex = match.fmi;
	uint8_t bytes[8] = {};
	std::copy(data.begin(), data.begin() + header.DLC, bytes);
	canQueueRx(match.fifo, header, bytes);
	CanFake &f = fake();
	const uint32_t before = f.msgPendingCallbacks;
	raiseIrqs();
	return f.msgPendingCallbacks - before;
}

} // namespace HostTarget

// ---------------- Fake HAL CAN (stm32f4xx_hal_can.c) ----------------
extern "C" {

/**
 * Models HAL_CAN_Init (stm32f4xx_hal_can.c:275): NULL -> HAL_ERROR; MspInit if State RESET (HAL_CAN_MspInit of
 * stm32f4xx_hal_msp.c: enables CAN1_TX and CAN1_RX0 in the NVIC, not RX1 / SCE); initialisation request (INRQ, INAK
 * acknowledged at once, or TIMEOUT -> State ERROR when injected), sleep mode left, MCR option bits and BTR from Init,
 * ErrorCode NONE, State READY.
 */
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan) {
	if (hcan == nullptr) {
		return HAL_ERROR;
	}
	if (!isCan1(hcan)) {
		unmodelled("HAL_CAN_Init: only CAN1 (hcan1) exists in the F407 configuration");
		return HAL_ERROR;
	}
	CanFake &f = fake();
	if (hcan->State == HAL_CAN_STATE_RESET) {
		f.nvic = {true, true, false, false};
	}
	CAN_TypeDef &c = *hcan->Instance;
	c.MCR |= MCR_INRQ;
	if (!modeChange()) {
		hcan->ErrorCode |= HAL_CAN_ERROR_TIMEOUT;
		hcan->State = HAL_CAN_STATE_ERROR;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_Init", 0, HAL_ERROR));
	}
	c.MSR |= MSR_INAK;
	c.MCR &= ~MCR_SLEEP;
	c.MSR &= ~MSR_SLAK;
	auto setBit = [&c](bool on, uint32_t bit) { c.MCR = on ? (c.MCR | bit) : (c.MCR & ~bit); };
	setBit(hcan->Init.TimeTriggeredMode == ENABLE, MCR_TTCM);
	setBit(hcan->Init.AutoBusOff == ENABLE, MCR_ABOM);
	setBit(hcan->Init.AutoWakeUp == ENABLE, MCR_AWUM);
	setBit(hcan->Init.AutoRetransmission != ENABLE, MCR_NART);
	setBit(hcan->Init.ReceiveFifoLocked == ENABLE, MCR_RFLM);
	setBit(hcan->Init.TransmitFifoPriority == ENABLE, MCR_TXFP);
	c.BTR = hcan->Init.Mode | hcan->Init.SyncJumpWidth | hcan->Init.TimeSeg1 | hcan->Init.TimeSeg2 | (hcan->Init.Prescaler - 1U);
	hcan->ErrorCode = HAL_CAN_ERROR_NONE;
	hcan->State = HAL_CAN_STATE_READY;
	return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_Init", 0, HAL_OK));
}

/**
 * Models HAL_CAN_DeInit (stm32f4xx_hal_can.c:460): HAL_CAN_Stop, MspDeInit (CAN1_TX / CAN1_RX0 disabled in the NVIC),
 * software master reset (MCR RESET: MCR to its reset value = sleep mode, FMP bits cleared, i.e. the receive FIFOs
 * flushed; RM0090 32.9.2), ErrorCode NONE, State RESET. Pending transmit mailboxes are not changed.
 */
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *hcan) {
	if (hcan == nullptr) {
		return HAL_ERROR;
	}
	if (!isCan1(hcan)) {
		unmodelled("HAL_CAN_DeInit: only CAN1 (hcan1) exists in the F407 configuration");
		return HAL_ERROR;
	}
	(void)HAL_CAN_Stop(hcan);
	CanFake &f = fake();
	f.nvic[0] = f.nvic[1] = false;
	hcan->Instance->MCR = MCR_RESET_VALUE;
	hcan->Instance->MSR = (hcan->Instance->MSR & ~MSR_INAK) | MSR_SLAK;
	for (uint32_t fifo = 0; fifo < 2; fifo++) {
		f.fifos[fifo].clear();
		syncFmp(fifo);
	}
	hcan->ErrorCode = HAL_CAN_ERROR_NONE;
	hcan->State = HAL_CAN_STATE_RESET;
	return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_DeInit", 0, HAL_OK));
}

/**
 * Models HAL_CAN_ConfigFilter (stm32f4xx_hal_can.c:840) with the dual instance branch of the F407 (CAN2 defined):
 * State READY or LISTENING (else ErrorCode |= NOT_INITIALIZED, HAL_ERROR); FINIT set, CAN2SB = SlaveStartFilterBank,
 * bank deactivated, scale (FS1R) and FR1/FR2 (16 bit: low halves / mask in the high halves; 32 bit: IdHigh:IdLow and
 * MaskIdHigh:MaskIdLow), mode (FM1R), FIFO (FFA1R), activation (FA1R), FINIT cleared. Every call is recorded
 * (HostTarget::canFilterConfigs). A bank above 27 (the HAL would write beyond sFilterRegister) is not modelled.
 */
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, const CAN_FilterTypeDef *sFilterConfig) {
	beginCall();
	CanFake &f = fake();
	f.filterConfigs.push_back(*sFilterConfig);
	if (!readyOrListening(hcan)) {
		hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_ConfigFilter", sFilterConfig->FilterBank, HAL_ERROR));
	}
	if (!isCan1(hcan) || sFilterConfig->FilterBank >= CAN_FILTER_BANKS) {
		unmodelled("HAL_CAN_ConfigFilter: only CAN1 and filter banks 0..27 are modelled");
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_ConfigFilter", sFilterConfig->FilterBank, HAL_ERROR));
	}
	CAN_TypeDef &c = *CAN1; // the filter banks belong to the master instance CAN1
	c.FMR |= FMR_FINIT;
	c.FMR &= ~FMR_CAN2SB;
	c.FMR |= sFilterConfig->SlaveStartFilterBank << 8U;
	const uint32_t bit = 1U << (sFilterConfig->FilterBank & 0x1FU);
	c.FA1R &= ~bit;
	CAN_FilterRegister_TypeDef &bank = c.sFilterRegister[sFilterConfig->FilterBank];
	if (sFilterConfig->FilterScale == CAN_FILTERSCALE_16BIT) {
		c.FS1R &= ~bit;
		bank.FR1 = ((0x0000FFFFU & sFilterConfig->FilterMaskIdLow) << 16U) | (0x0000FFFFU & sFilterConfig->FilterIdLow);
		bank.FR2 = ((0x0000FFFFU & sFilterConfig->FilterMaskIdHigh) << 16U) | (0x0000FFFFU & sFilterConfig->FilterIdHigh);
	}
	if (sFilterConfig->FilterScale == CAN_FILTERSCALE_32BIT) {
		c.FS1R |= bit;
		bank.FR1 = ((0x0000FFFFU & sFilterConfig->FilterIdHigh) << 16U) | (0x0000FFFFU & sFilterConfig->FilterIdLow);
		bank.FR2 = ((0x0000FFFFU & sFilterConfig->FilterMaskIdHigh) << 16U) | (0x0000FFFFU & sFilterConfig->FilterMaskIdLow);
	}
	if (sFilterConfig->FilterMode == CAN_FILTERMODE_IDMASK) {
		c.FM1R &= ~bit;
	} else {
		c.FM1R |= bit;
	}
	if (sFilterConfig->FilterFIFOAssignment == CAN_FILTER_FIFO0) {
		c.FFA1R &= ~bit;
	} else {
		c.FFA1R |= bit;
	}
	if (sFilterConfig->FilterActivation == CAN_FILTER_ENABLE) {
		c.FA1R |= bit;
	}
	c.FMR &= ~FMR_FINIT;
	return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_ConfigFilter", sFilterConfig->FilterBank, HAL_OK));
}

/**
 * Models HAL_CAN_Start (stm32f4xx_hal_can.c:1034): State READY (else ErrorCode |= NOT_READY, HAL_ERROR); State
 * LISTENING, INRQ cleared, INAK cleared at once (or TIMEOUT -> State ERROR when injected), ErrorCode NONE.
 * Leaving the initialisation mode while bus-off starts the bus-off recovery (MCR ABOM = 0, RM0090 32.7.6); the
 * 128 x 11 recessive bits are assumed to follow at once: BOFF cleared and the error counters reset, so EPVF and EWGF
 * are cleared too. Without bus-off, entering / leaving the initialisation mode does not reset the error counters
 * (EWGF / EPVF stay). Loop back / silent mode (BTR LBKM,
 * SILM) are not modelled (recorded).
 */
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan) {
	beginCall();
	if (hcan->State != HAL_CAN_STATE_READY) {
		hcan->ErrorCode |= HAL_CAN_ERROR_NOT_READY;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_Start", 0, HAL_ERROR));
	}
	if (!isCan1(hcan)) {
		unmodelled("HAL_CAN_Start: only CAN1 (hcan1) exists in the F407 configuration");
		return HAL_ERROR;
	}
	hcan->State = HAL_CAN_STATE_LISTENING;
	hcan->Instance->MCR &= ~MCR_INRQ;
	if (!modeChange()) {
		hcan->ErrorCode |= HAL_CAN_ERROR_TIMEOUT;
		hcan->State = HAL_CAN_STATE_ERROR;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_Start", 0, HAL_ERROR));
	}
	if (hcan->Instance->BTR & (BTR_LBKM | BTR_SILM)) {
		unmodelled("HAL_CAN_Start: loop back and silent mode (BTR LBKM / SILM) are not modelled");
	}
	hcan->Instance->MSR &= ~MSR_INAK;
	if (hcan->Instance->ESR & ESR_BOFF) {
		hcan->Instance->ESR &= ~(ESR_BOFF | ESR_EPVF | ESR_EWGF);
	}
	hcan->ErrorCode = HAL_CAN_ERROR_NONE;
	HAL_StatusTypeDef result = static_cast<HAL_StatusTypeDef>(record("HAL_CAN_Start", 0, HAL_OK));
	raiseIrqs(); // pending transmit completions / received frames of before
	return result;
}

/**
 * Models HAL_CAN_Stop (stm32f4xx_hal_can.c:1086): State LISTENING (else ErrorCode |= NOT_STARTED, HAL_ERROR);
 * INRQ set, INAK at once (or TIMEOUT -> State ERROR when injected), SLEEP cleared, State READY. Pending transmit
 * requests stay pending (the transfers on the bus stop, RM0090 32.4.1).
 */
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan) {
	beginCall();
	if (hcan->State != HAL_CAN_STATE_LISTENING) {
		hcan->ErrorCode |= HAL_CAN_ERROR_NOT_STARTED;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_Stop", 0, HAL_ERROR));
	}
	if (!isCan1(hcan)) {
		unmodelled("HAL_CAN_Stop: only CAN1 (hcan1) exists in the F407 configuration");
		return HAL_ERROR;
	}
	hcan->Instance->MCR |= MCR_INRQ;
	if (!modeChange()) {
		hcan->ErrorCode |= HAL_CAN_ERROR_TIMEOUT;
		hcan->State = HAL_CAN_STATE_ERROR;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_Stop", 0, HAL_ERROR));
	}
	hcan->Instance->MSR |= MSR_INAK;
	hcan->Instance->MCR &= ~MCR_SLEEP;
	hcan->Instance->MSR &= ~MSR_SLAK;
	hcan->State = HAL_CAN_STATE_READY;
	return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_Stop", 0, HAL_OK));
}

/**
 * Models HAL_CAN_RequestSleep (stm32f4xx_hal_can.c:1141): State READY or LISTENING (else NOT_INITIALIZED); SLEEP set.
 * The bus is idle, so the peripheral enters sleep mode at once (RM0090 32.4.3: SLAK and SLAKI set, INAK cleared).
 */
HAL_StatusTypeDef HAL_CAN_RequestSleep(CAN_HandleTypeDef *hcan) {
	beginCall();
	if (!readyOrListening(hcan)) {
		hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_RequestSleep", 0, HAL_ERROR));
	}
	hcan->Instance->MCR |= MCR_SLEEP;
	hcan->Instance->MSR = (hcan->Instance->MSR & ~MSR_INAK) | MSR_SLAK | MSR_SLAKI;
	HAL_StatusTypeDef result = static_cast<HAL_StatusTypeDef>(record("HAL_CAN_RequestSleep", 0, HAL_OK));
	raiseIrqs();
	return result;
}

/**
 * Models HAL_CAN_WakeUp (stm32f4xx_hal_can.c:1172): State READY or LISTENING (else NOT_INITIALIZED); SLEEP cleared,
 * the peripheral leaves sleep mode at once (SLAK cleared) into initialisation mode if INRQ is set, else normal mode.
 */
HAL_StatusTypeDef HAL_CAN_WakeUp(CAN_HandleTypeDef *hcan) {
	beginCall();
	if (!readyOrListening(hcan)) {
		hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_WakeUp", 0, HAL_ERROR));
	}
	hcan->Instance->MCR &= ~MCR_SLEEP;
	hcan->Instance->MSR &= ~MSR_SLAK;
	hcan->Instance->MSR = (hcan->Instance->MCR & MCR_INRQ) ? (hcan->Instance->MSR | MSR_INAK) : (hcan->Instance->MSR & ~MSR_INAK);
	return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_WakeUp", 0, HAL_OK));
}

/** Models HAL_CAN_IsSleepActive (stm32f4xx_hal_can.c:1219): SLAK if State READY or LISTENING, else 0 */
uint32_t HAL_CAN_IsSleepActive(const CAN_HandleTypeDef *hcan) {
	beginCall();
	uint32_t status = readyOrListening(hcan) && (hcan->Instance->MSR & MSR_SLAK) ? 1U : 0U;
	return record("HAL_CAN_IsSleepActive", 0, status);
}

/**
 * Models HAL_CAN_AddTxMessage (stm32f4xx_hal_can.c:1250): State READY or LISTENING (else ErrorCode |= NOT_INITIALIZED,
 * HAL_ERROR); no empty mailbox (TME0-2 all 0) -> ErrorCode |= PARAM, HAL_ERROR. Otherwise the mailbox of TSR CODE (the
 * lowest numbered empty mailbox) gets TIR (StdId << 21 | RTR, or ExtId << 3 | IDE | RTR; the HAL does not mask the
 * ids), TDTR = DLC (unmasked; TGT if TransmitGlobalTime), the 8 data bytes (aData[0..7] are always read), TXRQ;
 * *pTxMailbox = 1 << mailbox. The mailbox is pending (TME 0) until the test completes or the firmware aborts it.
 * Recorded in HostTarget::canTxFrames (fields decoded from the mailbox registers, like the frame on the bus).
 * In READY (initialisation mode) the request waits until the peripheral is started.
 */
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *pHeader, const uint8_t aData[], uint32_t *pTxMailbox) {
	beginCall();
	if (!readyOrListening(hcan)) {
		hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_AddTxMessage", 0, HAL_ERROR));
	}
	if (!isCan1(hcan)) {
		unmodelled("HAL_CAN_AddTxMessage: only CAN1 (hcan1) exists in the F407 configuration");
		return HAL_ERROR;
	}
	CAN_TypeDef &c = *hcan->Instance;
	int free = -1;
	for (uint32_t m = 0; m < 3 && free < 0; m++) {
		free = (c.TSR & tsrTme(m)) ? static_cast<int>(m) : -1;
	}
	if (free < 0) {
		hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_AddTxMessage", 0, HAL_ERROR));
	}
	const uint32_t m = static_cast<uint32_t>(free);
	c.TSR = (c.TSR & ~TSR_CODE) | (m << TSR_CODE_POS);
	*pTxMailbox = 1U << m;
	CAN_TxMailBox_TypeDef &mb = c.sTxMailBox[m];
	if (pHeader->IDE == CAN_ID_STD) {
		mb.TIR = (pHeader->StdId << 21U) | pHeader->RTR;
	} else {
		mb.TIR = (pHeader->ExtId << 3U) | pHeader->IDE | pHeader->RTR;
	}
	mb.TDTR = pHeader->DLC;
	if (pHeader->TransmitGlobalTime == ENABLE) {
		mb.TDTR |= TDT_TGT;
	}
	mb.TDHR = (static_cast<uint32_t>(aData[7]) << 24) | (static_cast<uint32_t>(aData[6]) << 16) | (static_cast<uint32_t>(aData[5]) << 8) | aData[4];
	mb.TDLR = (static_cast<uint32_t>(aData[3]) << 24) | (static_cast<uint32_t>(aData[2]) << 16) | (static_cast<uint32_t>(aData[1]) << 8) | aData[0];
	mb.TIR |= TI_TXRQ;
	c.TSR &= ~(tsrTme(m) | tsrRqcp(m) | tsrTxok(m) | tsrAlst(m) | tsrTerr(m));

	HostTarget::CanTxFrame frame;
	frame.header = *pHeader;
	frame.mailbox = m;
	frame.extId = (mb.TIR & CAN_ID_EXT) != 0U;
	frame.rtr = (mb.TIR & CAN_RTR_REMOTE) != 0U;
	frame.id = frame.extId ? (mb.TIR >> 3U) : (mb.TIR >> 21U);
	frame.dlc = static_cast<uint8_t>(mb.TDTR & 0xFU);
	for (uint32_t i = 0; i < 4; i++) {
		frame.data[i] = static_cast<uint8_t>(mb.TDLR >> (8 * i));
		frame.data[i + 4] = static_cast<uint8_t>(mb.TDHR >> (8 * i));
	}
	frame.time = HAL_GetTick();
	CanFake &f = fake();
	f.txFrames.push_back(frame);
	f.mailboxFrame[m] = static_cast<long>(f.txFrames.size() - 1);
	updateTsrCode();
	return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_AddTxMessage", *pTxMailbox, HAL_OK));
}

/**
 * Models HAL_CAN_AbortTxRequest (stm32f4xx_hal_can.c:1350): State READY or LISTENING (else NOT_INITIALIZED,
 * HAL_ERROR); ABRQx set for the selected mailboxes. Peripheral (RM0090 32.9.2 ABRQx): a pending mailbox is emptied at
 * once (TME, RQCP set, TXOK cleared; ALST/TERR of a failed attempt stay), ABRQ is cleared; no effect on an empty
 * mailbox. The transmit interrupt (abort callback, or error callback after a failed attempt) is raised before the
 * function returns, like an interrupt right after the register write.
 */
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes) {
	beginCall();
	if (!readyOrListening(hcan)) {
		hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_AbortTxRequest", TxMailboxes, HAL_ERROR));
	}
	CanFake &f = fake();
	for (uint32_t m = 0; m < 3; m++) {
		if ((TxMailboxes & (1U << m)) == 0U) {
			continue;
		}
		if (mailboxPending(m)) {
			hosttest_can1.TSR &= ~(tsrTxok(m) | tsrAbrq(m));
			hosttest_can1.TSR |= tsrRqcp(m) | tsrTme(m);
			hosttest_can1.sTxMailBox[m].TIR &= ~TI_TXRQ;
			if (f.mailboxFrame[m] >= 0) {
				f.txFrames[f.mailboxFrame[m]].result = HostTarget::CanTxResult::Aborted;
			}
			f.mailboxFrame[m] = -1;
			updateTsrCode();
		}
	}
	HAL_StatusTypeDef result = static_cast<HAL_StatusTypeDef>(record("HAL_CAN_AbortTxRequest", TxMailboxes, HAL_OK));
	raiseIrqs();
	return result;
}

/** Models HAL_CAN_GetTxMailboxesFreeLevel (stm32f4xx_hal_can.c:1399): number of empty mailboxes (TME) if State READY or LISTENING, else 0 */
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *hcan) {
	beginCall();
	uint32_t level = 0;
	if (readyOrListening(hcan)) {
		for (uint32_t m = 0; m < 3; m++) {
			level += (hcan->Instance->TSR & tsrTme(m)) ? 1U : 0U;
		}
	}
	return record("HAL_CAN_GetTxMailboxesFreeLevel", 0, level);
}

/** Models HAL_CAN_IsTxMessagePending (stm32f4xx_hal_can.c:1442): 1 if a selected mailbox is not empty (State READY or LISTENING) */
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *hcan, uint32_t TxMailboxes) {
	beginCall();
	uint32_t status = 0;
	if (readyOrListening(hcan) && (hcan->Instance->TSR & (TxMailboxes << 26U)) != (TxMailboxes << 26U)) {
		status = 1U;
	}
	return record("HAL_CAN_IsTxMessagePending", TxMailboxes, status);
}

/**
 * Models HAL_CAN_GetRxMessage (stm32f4xx_hal_can.c:1508): State must be READY or LISTENING (else ErrorCode |=
 * NOT_INITIALIZED, HAL_ERROR); an empty FIFO gives ErrorCode |= PARAM, HAL_ERROR. Otherwise the oldest frame is copied
 * (StdId or ExtId depending on IDE, RTR, DLC truncated to 8, FilterMatchIndex, Timestamp, 8 data bytes) and the FIFO
 * released (RFOM: FMP decremented; FULL / FOVR stay until cleared by the interrupt handler).
 */
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]) {
	beginCall();
	if (!readyOrListening(hcan)) {
		hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
		return HAL_ERROR;
	}
	const uint32_t fifo = RxFifo == CAN_RX_FIFO0 ? 0 : 1;
	std::deque<CanRxFrame> *queue = hcan == &hcan1 ? &fake().fifos[fifo] : nullptr;
	if (queue == nullptr || queue->empty()) {
		hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;
		return HAL_ERROR;
	}
	const CanRxFrame &frame = queue->front();
	pHeader->IDE = frame.header.IDE;
	if (pHeader->IDE == CAN_ID_STD) {
		pHeader->StdId = frame.header.StdId;
	} else {
		pHeader->ExtId = frame.header.ExtId;
	}
	pHeader->RTR = frame.header.RTR;
	pHeader->DLC = frame.header.DLC >= 8U ? 8U : frame.header.DLC;
	pHeader->FilterMatchIndex = frame.header.FilterMatchIndex;
	pHeader->Timestamp = frame.header.Timestamp;
	std::copy(frame.data.begin(), frame.data.end(), aData);
	queue->pop_front();
	syncFmp(fifo);
	return HAL_OK;
}

/** Models HAL_CAN_GetRxFifoFillLevel (stm32f4xx_hal_can.c:1608): number of pending frames (FMPx) if State is READY or LISTENING, else 0 */
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *hcan, uint32_t RxFifo) {
	beginCall();
	if (!readyOrListening(hcan)) {
		return 0;
	}
	if (hcan != &hcan1) {
		return 0; // Only CAN1 exists on the F407 configuration
	}
	return static_cast<uint32_t>(fake().fifos[RxFifo == CAN_RX_FIFO0 ? 0 : 1].size());
}

/**
 * Models HAL_CAN_ActivateNotification (stm32f4xx_hal_can.c:1661): State READY or LISTENING (else NOT_INITIALIZED,
 * HAL_ERROR); IER |= ActiveITs. An event that is already pending raises its interrupt at once (if its NVIC line is
 * enabled).
 */
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs) {
	beginCall();
	if (!readyOrListening(hcan)) {
		hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_ActivateNotification", ActiveITs, HAL_ERROR));
	}
	hcan->Instance->IER |= ActiveITs;
	HAL_StatusTypeDef result = static_cast<HAL_StatusTypeDef>(record("HAL_CAN_ActivateNotification", ActiveITs, HAL_OK));
	raiseIrqs();
	return result;
}

/** Models HAL_CAN_DeactivateNotification (stm32f4xx_hal_can.c:1694): State READY or LISTENING (else NOT_INITIALIZED, HAL_ERROR); IER &= ~InactiveITs */
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs) {
	beginCall();
	if (!readyOrListening(hcan)) {
		hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_DeactivateNotification", InactiveITs, HAL_ERROR));
	}
	hcan->Instance->IER &= ~InactiveITs;
	return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_DeactivateNotification", InactiveITs, HAL_OK));
}

/**
 * Models HAL_CAN_IRQHandler (stm32f4xx_hal_can.c:1725) step by step on the fake registers: transmit mailboxes 0-2
 * (RQCP: complete callback if TXOK, else error code TX_ALSTx / TX_TERRx, else abort callback; RQCP, TXOK, ALST, TERR
 * cleared), FIFO 0 overrun (RX_FOV0), full (callback), message pending (callback), the same for FIFO 1, sleep and
 * wake-up flags (cleared; the HAL's weak callbacks do nothing, the firmware defines none), error flags (EWG, EPV, BOF,
 * LEC if their interrupts are enabled; LEC and ERRI cleared), then ErrorCode |= error code and HAL_CAN_ErrorCallback.
 * Runs only through HostTarget (raised interrupts, canIrq), in interrupt context.
 */
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan) {
	CAN_TypeDef &c = *hcan->Instance;
	CanFake &f = fake();
	uint32_t errorcode = HAL_CAN_ERROR_NONE;
	const uint32_t interrupts = c.IER;
	const uint32_t msrflags = c.MSR;
	const uint32_t tsrflags = c.TSR;
	const uint32_t rf0rflags = c.RF0R;
	const uint32_t rf1rflags = c.RF1R;
	const uint32_t esrflags = c.ESR;
	static void (*const complete[3])(CAN_HandleTypeDef *) = {HAL_CAN_TxMailbox0CompleteCallback, HAL_CAN_TxMailbox1CompleteCallback, HAL_CAN_TxMailbox2CompleteCallback};
	static void (*const abort[3])(CAN_HandleTypeDef *) = {HAL_CAN_TxMailbox0AbortCallback, HAL_CAN_TxMailbox1AbortCallback, HAL_CAN_TxMailbox2AbortCallback};
	static const uint32_t alstCode[3] = {HAL_CAN_ERROR_TX_ALST0, HAL_CAN_ERROR_TX_ALST1, HAL_CAN_ERROR_TX_ALST2};
	static const uint32_t terrCode[3] = {HAL_CAN_ERROR_TX_TERR0, HAL_CAN_ERROR_TX_TERR1, HAL_CAN_ERROR_TX_TERR2};

	if (interrupts & CAN_IT_TX_MAILBOX_EMPTY) {
		for (uint32_t m = 0; m < 3; m++) {
			if (tsrflags & tsrRqcp(m)) {
				c.TSR &= ~(tsrRqcp(m) | tsrTxok(m) | tsrAlst(m) | tsrTerr(m)); // __HAL_CAN_CLEAR_FLAG(RQCPx)
				if (tsrflags & tsrTxok(m)) {
					complete[m](hcan);
				} else if (tsrflags & tsrAlst(m)) {
					errorcode |= alstCode[m];
				} else if (tsrflags & tsrTerr(m)) {
					errorcode |= terrCode[m];
				} else {
					abort[m](hcan);
				}
			}
		}
	}
	const uint32_t rfflags[2] = {rf0rflags, rf1rflags};
	const uint32_t overrunIt[2] = {CAN_IT_RX_FIFO0_OVERRUN, CAN_IT_RX_FIFO1_OVERRUN};
	const uint32_t fullIt[2] = {CAN_IT_RX_FIFO0_FULL, CAN_IT_RX_FIFO1_FULL};
	const uint32_t pendingIt[2] = {CAN_IT_RX_FIFO0_MSG_PENDING, CAN_IT_RX_FIFO1_MSG_PENDING};
	const uint32_t fovCode[2] = {HAL_CAN_ERROR_RX_FOV0, HAL_CAN_ERROR_RX_FOV1};
	for (uint32_t fifo = 0; fifo < 2; fifo++) {
		if ((interrupts & overrunIt[fifo]) && (rfflags[fifo] & RFR_FOVR)) {
			errorcode |= fovCode[fifo];
			rfr(fifo) &= ~RFR_FOVR;
		}
		if ((interrupts & fullIt[fifo]) && (rfflags[fifo] & RFR_FULL)) {
			rfr(fifo) &= ~RFR_FULL;
			if (fifo == 0) {
				HAL_CAN_RxFifo0FullCallback(hcan);
			} else {
				HAL_CAN_RxFifo1FullCallback(hcan);
			}
		}
		if ((interrupts & pendingIt[fifo]) && (rfr(fifo) & RFR_FMP)) { // "Check if message is still pending"
			f.msgPendingCallbacks++;
			if (fifo == 0) {
				HAL_CAN_RxFifo0MsgPendingCallback(hcan);
			} else {
				HAL_CAN_RxFifo1MsgPendingCallback(hcan);
			}
		}
	}
	if ((interrupts & CAN_IT_SLEEP_ACK) && (msrflags & MSR_SLAKI)) {
		c.MSR &= ~MSR_SLAKI; // HAL_CAN_SleepCallback: weak, not defined by the firmware
	}
	if ((interrupts & CAN_IT_WAKEUP) && (msrflags & MSR_WKUI)) {
		c.MSR &= ~MSR_WKUI; // HAL_CAN_WakeUpFromRxMsgCallback: weak, not defined by the firmware
	}
	if (interrupts & CAN_IT_ERROR) {
		if (msrflags & MSR_ERRI) {
			if ((interrupts & CAN_IT_ERROR_WARNING) && (esrflags & ESR_EWGF)) {
				errorcode |= HAL_CAN_ERROR_EWG;
			}
			if ((interrupts & CAN_IT_ERROR_PASSIVE) && (esrflags & ESR_EPVF)) {
				errorcode |= HAL_CAN_ERROR_EPV;
			}
			if ((interrupts & CAN_IT_BUSOFF) && (esrflags & ESR_BOFF)) {
				errorcode |= HAL_CAN_ERROR_BOF;
			}
			if ((interrupts & CAN_IT_LAST_ERROR_CODE) && (esrflags & ESR_LEC)) {
				switch (esrflags & ESR_LEC) {
				case ESR_LEC_0:
					errorcode |= HAL_CAN_ERROR_STF;
					break;
				case ESR_LEC_1:
					errorcode |= HAL_CAN_ERROR_FOR;
					break;
				case ESR_LEC_1 | ESR_LEC_0:
					errorcode |= HAL_CAN_ERROR_ACK;
					break;
				case ESR_LEC_2:
					errorcode |= HAL_CAN_ERROR_BR;
					break;
				case ESR_LEC_2 | ESR_LEC_0:
					errorcode |= HAL_CAN_ERROR_BD;
					break;
				case ESR_LEC_2 | ESR_LEC_1:
					errorcode |= HAL_CAN_ERROR_CRC;
					break;
				default:
					break;
				}
				c.ESR &= ~ESR_LEC;
			}
		}
		c.MSR &= ~MSR_ERRI;
	}
	if (errorcode != HAL_CAN_ERROR_NONE) {
		hcan->ErrorCode |= errorcode;
		HAL_CAN_ErrorCallback(hcan);
	}
}

/** Models HAL_CAN_GetState (stm32f4xx_hal_can.c:2376): SLEEP_ACTIVE if SLAK, SLEEP_PENDING if SLEEP requested (State READY / LISTENING), else State */
HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *hcan) {
	beginCall();
	HAL_CAN_StateTypeDef st = hcan->State;
	if (readyOrListening(hcan)) {
		if (hcan->Instance->MSR & MSR_SLAK) {
			st = HAL_CAN_STATE_SLEEP_ACTIVE;
		} else if (hcan->Instance->MCR & MCR_SLEEP) {
			st = HAL_CAN_STATE_SLEEP_PENDING;
		}
	}
	return static_cast<HAL_CAN_StateTypeDef>(record("HAL_CAN_GetState", 0, st));
}

/** Models HAL_CAN_GetError (stm32f4xx_hal_can.c:2411): ErrorCode */
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *hcan) {
	beginCall();
	return record("HAL_CAN_GetError", 0, hcan->ErrorCode);
}

/** Models HAL_CAN_ResetError (stm32f4xx_hal_can.c:2423): ErrorCode = 0 if State READY or LISTENING, else ErrorCode |= NOT_INITIALIZED, HAL_ERROR */
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan) {
	beginCall();
	if (readyOrListening(hcan)) {
		hcan->ErrorCode = 0U;
		return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_ResetError", 0, HAL_OK));
	}
	hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
	return static_cast<HAL_StatusTypeDef>(record("HAL_CAN_ResetError", 0, HAL_ERROR));
}

} // extern "C"
