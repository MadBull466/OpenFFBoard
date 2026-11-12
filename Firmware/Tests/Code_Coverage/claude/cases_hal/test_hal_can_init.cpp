// P12 T5 group CI (CI1-CI6): CAN lifecycle through the real ST HAL: Init, DeInit, Start, Stop, RequestSleep, WakeUp, IsSleepActive,
// GetState, GetError, ResetError.
// Oracles: UM1725 chapter "HAL CAN Generic Driver" (names, states, error codes), RM0090 chapter 32 (register bit positions, handshakes).
// Hand derivation of the register layout used below (RM0090 32.9):
//   MCR: INRQ bit 0, SLEEP bit 1, TXFP bit 2, RFLM bit 3, NART bit 4, AWUM bit 5, ABOM bit 6, TTCM bit 7, RESET bit 15, DBF bit 16 (reset value 0x00010002).
//   MSR: INAK bit 0, SLAK bit 1.
//   BTR: BRP[9:0], TS1[19:16] (value = TQ - 1), TS2[22:20] (TQ - 1), SJW[25:24] (TQ - 1), LBKM bit 30, SILM bit 31.
// The HAL writes NART = 1 when AutoRetransmission is DISABLE (UM1725: "Enable or disable the non-automatic retransmission mode" is the inverse).
// NEEDS REVIEW 327: WakeUp timeout leaves the state field unchanged (only ErrorCode |= TIMEOUT); case "characterization: CI4 WakeUp returns ERROR with TIMEOUT ...".
// NEEDS REVIEW 327: DeInit runs Stop before MspDeInit (state READY inside the callback); case "characterization: CI2 DeInit stops ...".
// NEEDS REVIEW 327: failing calls OR their error bit into the existing ErrorCode; cases "characterization: CI3 Start adds ...", "CI4 RequestSleep from state ERROR", "CI5 ResetError ...".
// NEEDS REVIEW 327: after an Init timeout the state is ERROR and a retry does not call MspInit again; case "characterization: CI1 Init after a timeout ...".
// NEEDS REVIEW 327: UM1725 / RM0090 do not state the return codes and error-code bits of the failing calls (Init / Start / Stop timeouts: HAL_ERROR + TIMEOUT + state ERROR;
// Start from a state other than READY: NOT_READY; Stop from a state other than LISTENING: NOT_STARTED; RequestSleep / WakeUp from RESET / ERROR: NOT_INITIALIZED).
// NEEDS REVIEW 327: the 10 tick timeout of the mode waits is the HAL's CAN_TIMEOUT_VALUE (error when GetTick - tickstart > 10); the boundary cases "characterization: CI1 Init ... limit".
#include "CanTestSupport.h"

#include <cstdint>

namespace {
// HAL: timeout when GetTick - tickstart > CAN_TIMEOUT_VALUE (10). Hand derivation: the INRQ write happens at tick t, tickstart = GetTick() = t + 1, so an acknowledge due at t + 11
// is seen at a poll whose difference is exactly 10 (no timeout); t + 12 is seen with difference 11 (timeout).
constexpr unsigned LIMIT_OK = 11;
constexpr uint32_t ci_INRQ = 1u << 0, ci_SLEEP = 1u << 1, ci_TXFP = 1u << 2, ci_RFLM = 1u << 3, ci_NART = 1u << 4, ci_AWUM = 1u << 5,
                   ci_ABOM = 1u << 6, ci_TTCM = 1u << 7, ci_DBF = 1u << 16;
constexpr uint32_t ci_INAK = 1u << 0, ci_SLAK = 1u << 1;
constexpr uint32_t ci_LBKM = 1u << 30, ci_SILM = 1u << 31;

CAN_TypeDef *const ci_inst[2] = {CAN1, CAN2};

struct CiFixture : CanFixture {
	CAN_HandleTypeDef h{};
	CiFixture() { setup(CAN1); }
	void setup(CAN_TypeDef *inst) {
		h = CAN_HandleTypeDef{};
		h.Instance = inst;
		h.Init.Prescaler = 6;
		h.Init.Mode = CAN_MODE_NORMAL;
		h.Init.SyncJumpWidth = CAN_SJW_1TQ;
		h.Init.TimeSeg1 = CAN_BS1_11TQ;
		h.Init.TimeSeg2 = CAN_BS2_2TQ;
		h.Init.TimeTriggeredMode = DISABLE;
		h.Init.AutoBusOff = DISABLE;
		h.Init.AutoWakeUp = DISABLE;
		h.Init.AutoRetransmission = DISABLE;
		h.Init.ReceiveFifoLocked = DISABLE;
		h.Init.TransmitFifoPriority = DISABLE;
	}
	unsigned idx() const { return canIndexOf(h.Instance); }
	uint32_t mcr() const { return h.Instance->MCR; }
	uint32_t msr() const { return h.Instance->MSR; }
	uint32_t btr() const { return h.Instance->BTR; }
	void initOk() { REQUIRE(HAL_CAN_Init(&h) == HAL_OK); }
	void startOk() {
		initOk();
		REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	}
};
}

// ------------------------------------------------------------------------------------------------------------- CI1 Init

TEST_CASE_FIXTURE(CiFixture, "CI1 Init of a NULL handle returns HAL_ERROR and calls nothing") {
	CHECK(HAL_CAN_Init(nullptr) == HAL_ERROR);
	CHECK(canCounts.mspInit == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init from state RESET calls MspInit once with the handle") {
	REQUIRE(h.State == HAL_CAN_STATE_RESET);
	initOk();
	CHECK(canCounts.mspInit == 1u);
	CHECK(canCounts.last == &h);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init ends in state READY with ErrorCode NONE on both instances") {
	for (unsigned k = 0; k < 2; ++k) {
		setup(ci_inst[k]);
		h.ErrorCode = HAL_CAN_ERROR_NOT_READY | HAL_CAN_ERROR_EWG; // stale code is cleared by a successful Init
		initOk();
		CHECK(h.State == HAL_CAN_STATE_READY);
		CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init leaves initialisation mode requested and sleep mode left (INRQ = 1, SLEEP = 0, INAK = 1, SLAK = 0)") {
	for (unsigned k = 0; k < 2; ++k) {
		setup(ci_inst[k]);
		initOk();
		CHECK((mcr() & ci_INRQ) != 0u);
		CHECK((mcr() & ci_SLEEP) == 0u);
		CHECK((msr() & ci_INAK) != 0u);
		CHECK((msr() & ci_SLAK) == 0u);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init with every feature DISABLE writes MCR = DBF | NART | INRQ") {
	// All six switches off: only NART is set (AutoRetransmission DISABLE = no automatic retransmission), DBF keeps its reset value.
	initOk();
	CHECK(mcr() == (ci_DBF | ci_NART | ci_INRQ));
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init with every feature ENABLE writes MCR = DBF | TTCM | ABOM | AWUM | RFLM | TXFP | INRQ") {
	h.Init.TimeTriggeredMode = ENABLE;
	h.Init.AutoBusOff = ENABLE;
	h.Init.AutoWakeUp = ENABLE;
	h.Init.AutoRetransmission = ENABLE; // clears NART
	h.Init.ReceiveFifoLocked = ENABLE;
	h.Init.TransmitFifoPriority = ENABLE;
	initOk();
	// 0x00010000 | 0x80 | 0x40 | 0x20 | 0x08 | 0x04 | 0x01
	CHECK(mcr() == 0x000100EDu);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init sets TTCM only for TimeTriggeredMode") {
	h.Init.TimeTriggeredMode = ENABLE;
	initOk();
	CHECK(mcr() == (ci_DBF | ci_NART | ci_INRQ | ci_TTCM));
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init sets ABOM only for AutoBusOff") {
	h.Init.AutoBusOff = ENABLE;
	initOk();
	CHECK(mcr() == (ci_DBF | ci_NART | ci_INRQ | ci_ABOM));
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init sets AWUM only for AutoWakeUp") {
	h.Init.AutoWakeUp = ENABLE;
	initOk();
	CHECK(mcr() == (ci_DBF | ci_NART | ci_INRQ | ci_AWUM));
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init clears NART only for AutoRetransmission") {
	h.Init.AutoRetransmission = ENABLE;
	initOk();
	CHECK(mcr() == (ci_DBF | ci_INRQ));
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init sets RFLM only for ReceiveFifoLocked") {
	h.Init.ReceiveFifoLocked = ENABLE;
	initOk();
	CHECK(mcr() == (ci_DBF | ci_NART | ci_INRQ | ci_RFLM));
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init sets TXFP only for TransmitFifoPriority") {
	h.Init.TransmitFifoPriority = ENABLE;
	initOk();
	CHECK(mcr() == (ci_DBF | ci_NART | ci_INRQ | ci_TXFP));
}

TEST_CASE_FIXTURE(CiFixture, "CI1 a second Init with every feature DISABLE clears the bits the first one set") {
	h.Init.TimeTriggeredMode = ENABLE;
	h.Init.AutoBusOff = ENABLE;
	h.Init.AutoWakeUp = ENABLE;
	h.Init.AutoRetransmission = ENABLE;
	h.Init.ReceiveFifoLocked = ENABLE;
	h.Init.TransmitFifoPriority = ENABLE;
	initOk();
	REQUIRE(mcr() == 0x000100EDu);
	h.Init.TimeTriggeredMode = DISABLE;
	h.Init.AutoBusOff = DISABLE;
	h.Init.AutoWakeUp = DISABLE;
	h.Init.AutoRetransmission = DISABLE;
	h.Init.ReceiveFifoLocked = DISABLE;
	h.Init.TransmitFifoPriority = DISABLE;
	initOk();
	CHECK(mcr() == (ci_DBF | ci_NART | ci_INRQ));
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init packs BTR = mode | SJW | TS1 | TS2 | (prescaler - 1)") {
	// Prescaler 6 -> BRP 5; SJW 3 TQ -> 2 << 24 = 0x02000000; TS1 11 TQ -> 10 << 16 = 0x000A0000; TS2 2 TQ -> 1 << 20 = 0x00100000.
	h.Init.SyncJumpWidth = CAN_SJW_3TQ;
	initOk();
	CHECK(btr() == 0x021A0005u);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init with the smallest bit timing values writes BTR = 0") {
	h.Init.Prescaler = 1; // BRP 0
	h.Init.SyncJumpWidth = CAN_SJW_1TQ;
	h.Init.TimeSeg1 = CAN_BS1_1TQ;
	h.Init.TimeSeg2 = CAN_BS2_1TQ;
	initOk();
	CHECK(btr() == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init with the largest bit timing values writes BTR = 0x037F03FF") {
	// BRP 1023 (prescaler 1024), TS1 16 TQ = 0xF << 16, TS2 8 TQ = 0x7 << 20, SJW 4 TQ = 0x3 << 24.
	h.Init.Prescaler = 1024;
	h.Init.SyncJumpWidth = CAN_SJW_4TQ;
	h.Init.TimeSeg1 = CAN_BS1_16TQ;
	h.Init.TimeSeg2 = CAN_BS2_8TQ;
	initOk();
	CHECK(btr() == 0x037F03FFu);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init writes each SJW value into BTR bits 25:24 as TQ - 1") {
	const uint32_t sjw[4] = {CAN_SJW_1TQ, CAN_SJW_2TQ, CAN_SJW_3TQ, CAN_SJW_4TQ};
	for (uint32_t n = 0; n < 4; ++n) {
		h.State = HAL_CAN_STATE_RESET;
		h.Init.SyncJumpWidth = sjw[n];
		h.Init.Prescaler = 1;
		h.Init.TimeSeg1 = CAN_BS1_1TQ;
		h.Init.TimeSeg2 = CAN_BS2_1TQ;
		initOk();
		CHECK(btr() == (n << 24));
		REQUIRE(HAL_CAN_DeInit(&h) == HAL_OK);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init writes each TS1 value into BTR bits 19:16 as TQ - 1") {
	const uint32_t ts1[16] = {CAN_BS1_1TQ,  CAN_BS1_2TQ,  CAN_BS1_3TQ,  CAN_BS1_4TQ,  CAN_BS1_5TQ,  CAN_BS1_6TQ,  CAN_BS1_7TQ,  CAN_BS1_8TQ,
	                          CAN_BS1_9TQ,  CAN_BS1_10TQ, CAN_BS1_11TQ, CAN_BS1_12TQ, CAN_BS1_13TQ, CAN_BS1_14TQ, CAN_BS1_15TQ, CAN_BS1_16TQ};
	for (uint32_t n = 0; n < 16; ++n) {
		h.Init.Prescaler = 1;
		h.Init.TimeSeg1 = ts1[n];
		h.Init.TimeSeg2 = CAN_BS2_1TQ;
		initOk();
		CHECK(btr() == (n << 16));
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init writes each TS2 value into BTR bits 22:20 as TQ - 1") {
	const uint32_t ts2[8] = {CAN_BS2_1TQ, CAN_BS2_2TQ, CAN_BS2_3TQ, CAN_BS2_4TQ, CAN_BS2_5TQ, CAN_BS2_6TQ, CAN_BS2_7TQ, CAN_BS2_8TQ};
	for (uint32_t n = 0; n < 8; ++n) {
		h.Init.Prescaler = 1;
		h.Init.TimeSeg1 = CAN_BS1_1TQ;
		h.Init.TimeSeg2 = ts2[n];
		initOk();
		CHECK(btr() == (n << 20));
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init writes the prescaler minus one into BTR bits 9:0") {
	const uint32_t presc[5] = {1, 2, 3, 512, 1024};
	for (uint32_t p : presc) {
		h.Init.Prescaler = p;
		h.Init.TimeSeg1 = CAN_BS1_1TQ;
		h.Init.TimeSeg2 = CAN_BS2_1TQ;
		initOk();
		CHECK(btr() == p - 1u);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init in each of the four modes sets LBKM / SILM in BTR bits 30 / 31") {
	const uint32_t modes[4] = {CAN_MODE_NORMAL, CAN_MODE_LOOPBACK, CAN_MODE_SILENT, CAN_MODE_SILENT_LOOPBACK};
	const uint32_t expect[4] = {0u, ci_LBKM, ci_SILM, ci_LBKM | ci_SILM};
	for (unsigned n = 0; n < 4; ++n) {
		h.Init.Mode = modes[n];
		initOk();
		CHECK((btr() & 0xC0000000u) == expect[n]);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init writes BTR only after the model reports initialisation mode (no write protected access)") {
	initOk();
	CHECK(hostCanProtectedWrites(idx()) == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init of the second instance does not touch the first instance") {
	setup(CAN2);
	h.Init.Mode = CAN_MODE_LOOPBACK;
	initOk();
	CHECK(CAN2->BTR != CAN1->BTR);
	CHECK(CAN1->MCR == 0x00010002u); // RM0090 reset value
	CHECK((CAN1->MSR & ci_INAK) == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init of an already initialised handle does not call MspInit again") {
	initOk();
	initOk();
	CHECK(canCounts.mspInit == 1u);
	CHECK(h.State == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init of a started handle rewrites the registers and ends in READY") {
	startOk();
	REQUIRE(h.State == HAL_CAN_STATE_LISTENING);
	h.Init.Mode = CAN_MODE_SILENT;
	initOk();
	CHECK(h.State == HAL_CAN_STATE_READY);
	CHECK((btr() & 0xC0000000u) == ci_SILM);
	CHECK((msr() & ci_INAK) != 0u);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI1 Init times out with ERROR, TIMEOUT and state ERROR when INAK is never set") {
	hostCanStuckSync(0, 1);
	CHECK(HAL_CAN_Init(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_TIMEOUT) != 0u);
	CHECK(h.State == HAL_CAN_STATE_ERROR);
	CHECK(canCounts.mspInit == 1u); // MspInit runs before the wait
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI1 Init times out with ERROR, TIMEOUT and state ERROR when SLAK is never cleared") {
	initOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK); // SLAK = 1 and INAK = 1
	REQUIRE((msr() & ci_SLAK) != 0u);
	hostCanStuckSync(0, 1);
	CHECK(HAL_CAN_Init(&h) == HAL_ERROR); // INAK already 1, the sleep exit never completes
	CHECK((h.ErrorCode & HAL_CAN_ERROR_TIMEOUT) != 0u);
	CHECK(h.State == HAL_CAN_STATE_ERROR);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI1 Init after a timeout succeeds once the hardware answers and clears the error code") {
	hostCanStuckSync(0, 1);
	REQUIRE(HAL_CAN_Init(&h) == HAL_ERROR);
	hostCanStuckSync(0, 0);
	CHECK(HAL_CAN_Init(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_READY);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK(canCounts.mspInit == 1u); // state was ERROR, not RESET
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI1 Init of the second instance times out on its own mode flags") {
	setup(CAN2);
	hostCanStuckSync(1, 1);
	CHECK(HAL_CAN_Init(&h) == HAL_ERROR);
	CHECK(h.State == HAL_CAN_STATE_ERROR);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init waits for a mode acknowledge that arrives after a few ticks") {
	hostCanSetSyncTicks(5);
	initOk();
	CHECK(h.State == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI1 Init gives up when the mode acknowledge needs far longer than the 10 ms the HAL waits") {
	hostCanSetSyncTicks(100);
	CHECK(HAL_CAN_Init(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_TIMEOUT) != 0u);
}

// ----------------------------------------------------------------------------------------------------------- CI2 DeInit

TEST_CASE_FIXTURE(CiFixture, "CI2 DeInit of a NULL handle returns HAL_ERROR and calls nothing") {
	CHECK(HAL_CAN_DeInit(nullptr) == HAL_ERROR);
	CHECK(canCounts.mspDeInit == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI2 DeInit of a started handle ends in state RESET with ErrorCode NONE and MspDeInit called once") {
	startOk();
	CHECK(HAL_CAN_DeInit(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_RESET);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
	CHECK(canCounts.mspDeInit == 1u);
	CHECK(canCounts.last == &h);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI2 DeInit stops a started handle before MspDeInit (state is READY inside the callback)") {
	startOk();
	REQUIRE(HAL_CAN_DeInit(&h) == HAL_OK);
	CHECK(canCounts.stateAtLast == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "CI2 DeInit of a started handle resets MCR to its reset value and enters sleep mode") {
	for (unsigned k = 0; k < 2; ++k) {
		setup(ci_inst[k]);
		h.Init.AutoBusOff = ENABLE;
		h.Init.ReceiveFifoLocked = ENABLE;
		startOk();
		REQUIRE(HAL_CAN_DeInit(&h) == HAL_OK);
		CHECK(mcr() == 0x00010002u);       // RM0090: reset value, SLEEP = 1
		CHECK((msr() & ci_SLAK) != 0u);    // sleep mode entered by the master reset
		CHECK((msr() & ci_INAK) == 0u);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI2 DeInit of a READY handle succeeds although Stop is rejected, and clears the error code") {
	initOk();
	CHECK(HAL_CAN_DeInit(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_RESET);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE); // NOT_STARTED from the inner Stop is cleared
	CHECK(canCounts.mspDeInit == 1u);
	CHECK(mcr() == 0x00010002u);
}

TEST_CASE_FIXTURE(CiFixture, "CI2 DeInit of a handle in state RESET still resets the peripheral and calls MspDeInit") {
	REQUIRE(h.State == HAL_CAN_STATE_RESET);
	CHECK(HAL_CAN_DeInit(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_RESET);
	CHECK(canCounts.mspDeInit == 1u);
}

TEST_CASE_FIXTURE(CiFixture, "CI2 DeInit of a handle in state ERROR ends in RESET with ErrorCode NONE") {
	hostCanStuckSync(0, 1);
	REQUIRE(HAL_CAN_Init(&h) == HAL_ERROR);
	hostCanStuckSync(0, 0);
	CHECK(HAL_CAN_DeInit(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_RESET);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(CiFixture, "CI2 Init after DeInit calls MspInit again") {
	initOk();
	REQUIRE(HAL_CAN_DeInit(&h) == HAL_OK);
	initOk();
	CHECK(canCounts.mspInit == 2u);
	CHECK(h.State == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "CI2 DeInit with a stuck mode handshake still completes the reset") {
	startOk();
	hostCanStuckSync(0, 1); // inner Stop times out and sets state ERROR
	CHECK(HAL_CAN_DeInit(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_RESET);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

// ------------------------------------------------------------------------------------------------------------ CI3 Start

TEST_CASE_FIXTURE(CiFixture, "CI3 Start from READY returns OK and moves to state LISTENING") {
	for (unsigned k = 0; k < 2; ++k) {
		setup(ci_inst[k]);
		initOk();
		CHECK(HAL_CAN_Start(&h) == HAL_OK);
		CHECK(h.State == HAL_CAN_STATE_LISTENING);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI3 Start clears INRQ and the model acknowledges with INAK = 0 (normal mode)") {
	for (unsigned k = 0; k < 2; ++k) {
		setup(ci_inst[k]);
		startOk();
		CHECK((mcr() & ci_INRQ) == 0u);
		CHECK((msr() & ci_INAK) == 0u);
		CHECK((mcr() & ci_SLEEP) == 0u);
		CHECK((msr() & ci_SLAK) == 0u);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI3 Start keeps the other MCR bits written by Init") {
	h.Init.AutoBusOff = ENABLE;
	h.Init.TransmitFifoPriority = ENABLE;
	startOk();
	CHECK(mcr() == (ci_DBF | ci_NART | ci_ABOM | ci_TXFP));
}

TEST_CASE_FIXTURE(CiFixture, "CI3 Start clears a stale error code") {
	initOk();
	h.ErrorCode = HAL_CAN_ERROR_NOT_STARTED | HAL_CAN_ERROR_TX_ALST1;
	REQUIRE(HAL_CAN_Start(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(CiFixture, "CI3 Start waits for an acknowledge that arrives after a few ticks") {
	initOk();
	hostCanSetSyncTicks(5);
	CHECK(HAL_CAN_Start(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Start from state LISTENING returns ERROR with NOT_READY and keeps the state") {
	startOk();
	CHECK(HAL_CAN_Start(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_READY) != 0u);
	CHECK(h.State == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Start from state RESET returns ERROR with NOT_READY") {
	CHECK(HAL_CAN_Start(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_READY) != 0u);
	CHECK(h.State == HAL_CAN_STATE_RESET);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Start from state ERROR returns ERROR with NOT_READY and keeps state ERROR") {
	hostCanStuckSync(0, 1);
	REQUIRE(HAL_CAN_Init(&h) == HAL_ERROR);
	hostCanStuckSync(0, 0);
	h.ErrorCode = 0;
	CHECK(HAL_CAN_Start(&h) == HAL_ERROR);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NOT_READY);
	CHECK(h.State == HAL_CAN_STATE_ERROR);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Start adds NOT_READY to an existing error code instead of replacing it") {
	h.ErrorCode = HAL_CAN_ERROR_EWG;
	CHECK(HAL_CAN_Start(&h) == HAL_ERROR);
	CHECK(h.ErrorCode == (HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_NOT_READY));
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Start times out with ERROR, TIMEOUT and state ERROR when INAK never clears") {
	initOk();
	hostCanStuckSync(0, 1);
	CHECK(HAL_CAN_Start(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_TIMEOUT) != 0u);
	CHECK(h.State == HAL_CAN_STATE_ERROR);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Start gives up when the acknowledge needs far longer than 10 ms") {
	initOk();
	hostCanSetSyncTicks(100);
	CHECK(HAL_CAN_Start(&h) == HAL_ERROR);
	CHECK(h.State == HAL_CAN_STATE_ERROR);
}

// ------------------------------------------------------------------------------------------------------------- CI3 Stop

TEST_CASE_FIXTURE(CiFixture, "CI3 Stop from LISTENING returns OK and moves to state READY") {
	for (unsigned k = 0; k < 2; ++k) {
		setup(ci_inst[k]);
		startOk();
		CHECK(HAL_CAN_Stop(&h) == HAL_OK);
		CHECK(h.State == HAL_CAN_STATE_READY);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI3 Stop sets INRQ, waits for INAK = 1 and leaves SLEEP clear") {
	for (unsigned k = 0; k < 2; ++k) {
		setup(ci_inst[k]);
		startOk();
		REQUIRE(HAL_CAN_Stop(&h) == HAL_OK);
		CHECK((mcr() & ci_INRQ) != 0u);
		CHECK((msr() & ci_INAK) != 0u);
		CHECK((mcr() & ci_SLEEP) == 0u);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI3 Stop clears a SLEEP request made while listening") {
	startOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	REQUIRE((mcr() & ci_SLEEP) != 0u);
	CHECK(HAL_CAN_Stop(&h) == HAL_OK);
	CHECK((mcr() & ci_SLEEP) == 0u);
	CHECK((msr() & ci_SLAK) == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI3 Stop keeps the configuration bits of MCR") {
	h.Init.ReceiveFifoLocked = ENABLE;
	h.Init.AutoWakeUp = ENABLE;
	startOk();
	REQUIRE(HAL_CAN_Stop(&h) == HAL_OK);
	CHECK(mcr() == (ci_DBF | ci_NART | ci_RFLM | ci_AWUM | ci_INRQ));
}

TEST_CASE_FIXTURE(CiFixture, "CI3 Stop followed by Start returns to LISTENING") {
	startOk();
	REQUIRE(HAL_CAN_Stop(&h) == HAL_OK);
	CHECK(HAL_CAN_Start(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(CiFixture, "CI3 Stop waits for an acknowledge that arrives after a few ticks") {
	startOk();
	hostCanSetSyncTicks(5);
	CHECK(HAL_CAN_Stop(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Stop from READY returns ERROR with NOT_STARTED and keeps the state") {
	initOk();
	CHECK(HAL_CAN_Stop(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_STARTED) != 0u);
	CHECK(h.State == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Stop from state RESET returns ERROR with NOT_STARTED") {
	CHECK(HAL_CAN_Stop(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_STARTED) != 0u);
	CHECK(h.State == HAL_CAN_STATE_RESET);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Stop from state ERROR returns ERROR with NOT_STARTED") {
	h.State = HAL_CAN_STATE_ERROR;
	CHECK(HAL_CAN_Stop(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_STARTED) != 0u);
	CHECK(h.State == HAL_CAN_STATE_ERROR);
}

TEST_CASE_FIXTURE(CiFixture, "CI3 Stop does not touch the registers when the state is wrong") {
	initOk();
	const uint32_t before = mcr();
	REQUIRE(HAL_CAN_Stop(&h) == HAL_ERROR);
	CHECK(mcr() == before);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Stop times out with ERROR, TIMEOUT and state ERROR when INAK never sets") {
	startOk();
	hostCanStuckSync(0, 1);
	CHECK(HAL_CAN_Stop(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_TIMEOUT) != 0u);
	CHECK(h.State == HAL_CAN_STATE_ERROR);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI3 Stop gives up when the acknowledge needs far longer than 10 ms") {
	startOk();
	hostCanSetSyncTicks(100);
	CHECK(HAL_CAN_Stop(&h) == HAL_ERROR);
	CHECK(h.State == HAL_CAN_STATE_ERROR);
}

// ------------------------------------------------------------------------------------------------- CI4 sleep and wake-up

TEST_CASE_FIXTURE(CiFixture, "CI4 RequestSleep from READY sets SLEEP and returns OK without changing the state field") {
	for (unsigned k = 0; k < 2; ++k) {
		setup(ci_inst[k]);
		initOk();
		CHECK(HAL_CAN_RequestSleep(&h) == HAL_OK);
		CHECK((mcr() & ci_SLEEP) != 0u);
		CHECK(h.State == HAL_CAN_STATE_READY);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI4 RequestSleep from LISTENING sets SLEEP and returns OK") {
	startOk();
	CHECK(HAL_CAN_RequestSleep(&h) == HAL_OK);
	CHECK((mcr() & ci_SLEEP) != 0u);
	CHECK(h.State == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(CiFixture, "CI4 RequestSleep makes the model acknowledge with SLAK = 1") {
	initOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	CHECK((msr() & ci_SLAK) != 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI4 RequestSleep keeps the other MCR bits") {
	h.Init.TimeTriggeredMode = ENABLE;
	initOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	CHECK(mcr() == (ci_DBF | ci_NART | ci_TTCM | ci_INRQ | ci_SLEEP));
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI4 RequestSleep from state RESET returns ERROR with NOT_INITIALIZED") {
	CHECK(HAL_CAN_RequestSleep(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI4 RequestSleep from state ERROR returns ERROR with NOT_INITIALIZED and leaves SLEEP alone") {
	h.State = HAL_CAN_STATE_ERROR;
	h.ErrorCode = HAL_CAN_ERROR_EPV;
	const uint32_t before = mcr();
	CHECK(HAL_CAN_RequestSleep(&h) == HAL_ERROR);
	CHECK(h.ErrorCode == (HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_NOT_INITIALIZED));
	CHECK(mcr() == before);
}

TEST_CASE_FIXTURE(CiFixture, "CI4 WakeUp from sleep returns OK, clears SLEEP and the model drops SLAK") {
	for (unsigned k = 0; k < 2; ++k) {
		setup(ci_inst[k]);
		initOk();
		REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
		REQUIRE((msr() & ci_SLAK) != 0u);
		CHECK(HAL_CAN_WakeUp(&h) == HAL_OK);
		CHECK((mcr() & ci_SLEEP) == 0u);
		CHECK((msr() & ci_SLAK) == 0u);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI4 WakeUp of a listening handle that sleeps returns OK") {
	startOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	CHECK(HAL_CAN_WakeUp(&h) == HAL_OK);
	CHECK((msr() & ci_SLAK) == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI4 WakeUp of a handle that is not asleep returns OK") {
	initOk();
	CHECK(HAL_CAN_WakeUp(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI4 WakeUp returns ERROR with TIMEOUT when SLAK never clears") {
	initOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	hostCanStuckSync(0, 1);
	CHECK(HAL_CAN_WakeUp(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_TIMEOUT) != 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI4 WakeUp returns OK when SLAK clears only after a delay of model ticks") {
	initOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	hostCanSetSyncTicks(3);
	hostCanSetPollReads(1); // every MSR read is one tick: the HAL's counter loop sees time pass
	CHECK(HAL_CAN_WakeUp(&h) == HAL_OK);
	CHECK((msr() & ci_SLAK) == 0u);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI4 WakeUp from state RESET returns ERROR with NOT_INITIALIZED") {
	CHECK(HAL_CAN_WakeUp(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI4 WakeUp from state ERROR returns ERROR with NOT_INITIALIZED and does not write MCR") {
	h.State = HAL_CAN_STATE_ERROR;
	const uint32_t before = mcr();
	CHECK(HAL_CAN_WakeUp(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
	CHECK(mcr() == before);
}

TEST_CASE_FIXTURE(CiFixture, "CI4 IsSleepActive is 0 after Init and 1 after RequestSleep") {
	for (unsigned k = 0; k < 2; ++k) {
		setup(ci_inst[k]);
		initOk();
		CHECK(HAL_CAN_IsSleepActive(&h) == 0u);
		REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
		CHECK(HAL_CAN_IsSleepActive(&h) == 1u);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI4 IsSleepActive is 1 for a listening handle that sleeps and 0 again after WakeUp") {
	startOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	CHECK(HAL_CAN_IsSleepActive(&h) == 1u);
	REQUIRE(HAL_CAN_WakeUp(&h) == HAL_OK);
	CHECK(HAL_CAN_IsSleepActive(&h) == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI4 IsSleepActive is 0 in state RESET although the peripheral is in sleep mode after reset") {
	REQUIRE((msr() & ci_SLAK) != 0u); // RM0090: bxCAN starts in sleep mode
	CHECK(HAL_CAN_IsSleepActive(&h) == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI4 IsSleepActive is 0 in state ERROR although SLAK is set") {
	h.State = HAL_CAN_STATE_ERROR;
	REQUIRE((msr() & ci_SLAK) != 0u);
	CHECK(HAL_CAN_IsSleepActive(&h) == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI4 IsSleepActive does not change any register") {
	initOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	const uint32_t m = mcr(), s = msr();
	(void)HAL_CAN_IsSleepActive(&h);
	CHECK(mcr() == m);
	CHECK(msr() == s);
}

// ------------------------------------------------------------------------------------ CI5 GetState / GetError / ResetError

TEST_CASE("CI5 HAL_CAN_StateTypeDef values follow the UM1725 coding 0 .. 5") {
	CHECK((uint32_t)HAL_CAN_STATE_RESET == 0x00u);
	CHECK((uint32_t)HAL_CAN_STATE_READY == 0x01u);
	CHECK((uint32_t)HAL_CAN_STATE_LISTENING == 0x02u);
	CHECK((uint32_t)HAL_CAN_STATE_SLEEP_PENDING == 0x03u);
	CHECK((uint32_t)HAL_CAN_STATE_SLEEP_ACTIVE == 0x04u);
	CHECK((uint32_t)HAL_CAN_STATE_ERROR == 0x05u);
}

TEST_CASE("CI5 HAL_CAN_ERROR_* codes are the header's one-bit values") {
	CHECK(HAL_CAN_ERROR_NONE == 0x00000000u);
	CHECK(HAL_CAN_ERROR_EWG == 0x00000001u);
	CHECK(HAL_CAN_ERROR_EPV == 0x00000002u);
	CHECK(HAL_CAN_ERROR_BOF == 0x00000004u);
	CHECK(HAL_CAN_ERROR_STF == 0x00000008u);
	CHECK(HAL_CAN_ERROR_FOR == 0x00000010u);
	CHECK(HAL_CAN_ERROR_ACK == 0x00000020u);
	CHECK(HAL_CAN_ERROR_BR == 0x00000040u);
	CHECK(HAL_CAN_ERROR_BD == 0x00000080u);
	CHECK(HAL_CAN_ERROR_CRC == 0x00000100u);
	CHECK(HAL_CAN_ERROR_RX_FOV0 == 0x00000200u);
	CHECK(HAL_CAN_ERROR_RX_FOV1 == 0x00000400u);
	CHECK(HAL_CAN_ERROR_TX_ALST0 == 0x00000800u);
	CHECK(HAL_CAN_ERROR_TX_TERR0 == 0x00001000u);
	CHECK(HAL_CAN_ERROR_TX_ALST1 == 0x00002000u);
	CHECK(HAL_CAN_ERROR_TX_TERR1 == 0x00004000u);
	CHECK(HAL_CAN_ERROR_TX_ALST2 == 0x00008000u);
	CHECK(HAL_CAN_ERROR_TX_TERR2 == 0x00010000u);
	CHECK(HAL_CAN_ERROR_TIMEOUT == 0x00020000u);
	CHECK(HAL_CAN_ERROR_NOT_INITIALIZED == 0x00040000u);
	CHECK(HAL_CAN_ERROR_NOT_READY == 0x00080000u);
	CHECK(HAL_CAN_ERROR_NOT_STARTED == 0x00100000u);
	CHECK(HAL_CAN_ERROR_PARAM == 0x00200000u);
	CHECK(HAL_CAN_ERROR_INTERNAL == 0x00800000u);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetState returns RESET for a fresh handle even though the peripheral sleeps") {
	CHECK(HAL_CAN_GetState(&h) == HAL_CAN_STATE_RESET);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetState returns READY after Init") {
	initOk();
	CHECK(HAL_CAN_GetState(&h) == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetState returns LISTENING after Start") {
	startOk();
	CHECK(HAL_CAN_GetState(&h) == HAL_CAN_STATE_LISTENING);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetState returns ERROR after a timeout without reading the registers' sleep flags") {
	hostCanStuckSync(0, 1);
	REQUIRE(HAL_CAN_Init(&h) == HAL_ERROR);
	REQUIRE((msr() & ci_SLAK) != 0u); // SLAK is set, the state must stay ERROR
	CHECK(HAL_CAN_GetState(&h) == HAL_CAN_STATE_ERROR);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetState returns SLEEP_ACTIVE for a ready handle in sleep mode") {
	initOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	CHECK(HAL_CAN_GetState(&h) == HAL_CAN_STATE_SLEEP_ACTIVE);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetState returns SLEEP_ACTIVE for a listening handle in sleep mode") {
	startOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	CHECK(HAL_CAN_GetState(&h) == HAL_CAN_STATE_SLEEP_ACTIVE);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetState returns SLEEP_PENDING while SLEEP is requested but SLAK is not yet set") {
	initOk();
	hostCanSetSyncTicks(5);
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	CHECK(HAL_CAN_GetState(&h) == HAL_CAN_STATE_SLEEP_PENDING);
	hostTickSet(hostTickPeek() + 5u);
	CHECK(HAL_CAN_GetState(&h) == HAL_CAN_STATE_SLEEP_ACTIVE);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetState returns READY again after WakeUp") {
	initOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	REQUIRE(HAL_CAN_WakeUp(&h) == HAL_OK);
	CHECK(HAL_CAN_GetState(&h) == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetState does not modify the handle") {
	initOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	(void)HAL_CAN_GetState(&h);
	CHECK(h.State == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetError returns the error code of the handle for every error value") {
	const uint32_t codes[] = {HAL_CAN_ERROR_NONE,      HAL_CAN_ERROR_EWG,        HAL_CAN_ERROR_EPV,       HAL_CAN_ERROR_BOF,
	                          HAL_CAN_ERROR_STF,       HAL_CAN_ERROR_FOR,        HAL_CAN_ERROR_ACK,       HAL_CAN_ERROR_BR,
	                          HAL_CAN_ERROR_BD,        HAL_CAN_ERROR_CRC,        HAL_CAN_ERROR_RX_FOV0,   HAL_CAN_ERROR_RX_FOV1,
	                          HAL_CAN_ERROR_TX_ALST0,  HAL_CAN_ERROR_TX_TERR0,   HAL_CAN_ERROR_TX_ALST1,  HAL_CAN_ERROR_TX_TERR1,
	                          HAL_CAN_ERROR_TX_ALST2,  HAL_CAN_ERROR_TX_TERR2,   HAL_CAN_ERROR_TIMEOUT,    HAL_CAN_ERROR_NOT_INITIALIZED,
	                          HAL_CAN_ERROR_NOT_READY, HAL_CAN_ERROR_NOT_STARTED, HAL_CAN_ERROR_PARAM,     HAL_CAN_ERROR_INTERNAL};
	for (uint32_t c : codes) {
		h.ErrorCode = c;
		CHECK(HAL_CAN_GetError(&h) == c);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetError returns a combination of error bits unchanged") {
	h.ErrorCode = HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_TX_TERR2 | HAL_CAN_ERROR_TIMEOUT;
	CHECK(HAL_CAN_GetError(&h) == 0x00030001u);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 GetError of a fresh handle is NONE") {
	CHECK(HAL_CAN_GetError(&h) == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 ResetError in READY clears the error code and returns OK") {
	initOk();
	h.ErrorCode = HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_BOF;
	CHECK(HAL_CAN_ResetError(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 ResetError in LISTENING clears the error code and returns OK") {
	startOk();
	h.ErrorCode = HAL_CAN_ERROR_CRC | HAL_CAN_ERROR_RX_FOV1;
	CHECK(HAL_CAN_ResetError(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(CiFixture, "CI5 ResetError of a sleeping ready handle clears the error code") {
	initOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	h.ErrorCode = HAL_CAN_ERROR_ACK;
	CHECK(HAL_CAN_ResetError(&h) == HAL_OK);
	CHECK(h.ErrorCode == HAL_CAN_ERROR_NONE);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI5 ResetError in state RESET returns ERROR and adds NOT_INITIALIZED to the code") {
	h.ErrorCode = HAL_CAN_ERROR_EPV;
	CHECK(HAL_CAN_ResetError(&h) == HAL_ERROR);
	CHECK(h.ErrorCode == (HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_NOT_INITIALIZED));
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI5 ResetError in state ERROR returns ERROR and keeps the existing error bits") {
	h.State = HAL_CAN_STATE_ERROR;
	h.ErrorCode = HAL_CAN_ERROR_TIMEOUT;
	CHECK(HAL_CAN_ResetError(&h) == HAL_ERROR);
	CHECK(h.ErrorCode == (HAL_CAN_ERROR_TIMEOUT | HAL_CAN_ERROR_NOT_INITIALIZED));
}

// ----------------------------------------------------------------------------------------------- CI6 scope and test modes

TEST_CASE("CI6 register callbacks are compiled out (USE_HAL_CAN_REGISTER_CALLBACKS is 0)") {
	CHECK(USE_HAL_CAN_REGISTER_CALLBACKS == 0);
}

TEST_CASE_FIXTURE(CiFixture, "CI6 the four test modes differ only in BTR bits 31 and 30, not in MCR or the rest of BTR") {
	const uint32_t modes[4] = {CAN_MODE_NORMAL, CAN_MODE_LOOPBACK, CAN_MODE_SILENT, CAN_MODE_SILENT_LOOPBACK};
	uint32_t mcrRef = 0, btrRef = 0;
	for (unsigned n = 0; n < 4; ++n) {
		h.State = HAL_CAN_STATE_RESET;
		h.Init.Mode = modes[n];
		initOk();
		if (n == 0) {
			mcrRef = mcr();
			btrRef = btr();
		}
		CHECK(mcr() == mcrRef);
		CHECK((btr() & 0x3FFFFFFFu) == (btrRef & 0x3FFFFFFFu));
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI6 mode constants are the BTR bits LBKM (30) and SILM (31)") {
	CHECK(CAN_MODE_NORMAL == 0u);
	CHECK(CAN_MODE_LOOPBACK == (1u << 30));
	CHECK(CAN_MODE_SILENT == (1u << 31));
	CHECK(CAN_MODE_SILENT_LOOPBACK == ((1u << 30) | (1u << 31)));
}

TEST_CASE_FIXTURE(CiFixture, "CI6 a handle initialised in loopback mode can be started and stopped") {
	h.Init.Mode = CAN_MODE_LOOPBACK;
	startOk();
	CHECK(h.State == HAL_CAN_STATE_LISTENING);
	CHECK(HAL_CAN_Stop(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "CI6 the whole lifecycle raises no register model fault") {
	startOk();
	REQUIRE(HAL_CAN_RequestSleep(&h) == HAL_OK);
	REQUIRE(HAL_CAN_WakeUp(&h) == HAL_OK);
	REQUIRE(HAL_CAN_Stop(&h) == HAL_OK);
	REQUIRE(HAL_CAN_DeInit(&h) == HAL_OK);
	CHECK(HalFixture::faults() == 0u);
	CHECK(hostCanProtectedWrites(0) == 0u);
}

// ------------------------------------------------------------------------------------------------ mutation review additions

TEST_CASE_FIXTURE(CiFixture, "CI1 a second Init with AutoRetransmission ENABLE clears NART set by the first Init") {
	initOk();
	REQUIRE((mcr() & ci_NART) != 0u);
	h.Init.AutoRetransmission = ENABLE;
	initOk();
	CHECK((mcr() & ci_NART) == 0u);
}

TEST_CASE_FIXTURE(CiFixture, "CI1 a second Init clears each of TTCM, ABOM, AWUM, RFLM, TXFP that was set before, one at a time") {
	const uint32_t bits[5] = {ci_TTCM, ci_ABOM, ci_AWUM, ci_RFLM, ci_TXFP};
	for (unsigned n = 0; n < 5; ++n) {
		h.Init.TimeTriggeredMode = n == 0 ? ENABLE : DISABLE;
		h.Init.AutoBusOff = n == 1 ? ENABLE : DISABLE;
		h.Init.AutoWakeUp = n == 2 ? ENABLE : DISABLE;
		h.Init.ReceiveFifoLocked = n == 3 ? ENABLE : DISABLE;
		h.Init.TransmitFifoPriority = n == 4 ? ENABLE : DISABLE;
		initOk();
		REQUIRE((mcr() & bits[n]) != 0u);
		h.Init.TimeTriggeredMode = h.Init.AutoBusOff = h.Init.AutoWakeUp = h.Init.ReceiveFifoLocked = h.Init.TransmitFifoPriority = DISABLE;
		initOk();
		CHECK((mcr() & bits[n]) == 0u);
	}
}

TEST_CASE_FIXTURE(CiFixture, "CI1 Init succeeds when INAK and SLAK each take 6 ticks (each wait is within the timeout, the sum is not)") {
	hostCanSetSyncTicks(6);
	CHECK(HAL_CAN_Init(&h) == HAL_OK);
	CHECK(h.State == HAL_CAN_STATE_READY);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI1 Init succeeds when the acknowledges arrive at the 10 tick limit") {
	// NEEDS REVIEW 327
	hostCanSetSyncTicks(LIMIT_OK);
	CHECK(HAL_CAN_Init(&h) == HAL_OK);
}

TEST_CASE_FIXTURE(CiFixture, "characterization: CI1 Init times out when the acknowledges arrive one tick after the limit") {
	// NEEDS REVIEW 327
	hostCanSetSyncTicks(LIMIT_OK + 1);
	CHECK(HAL_CAN_Init(&h) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_TIMEOUT) != 0u);
}
