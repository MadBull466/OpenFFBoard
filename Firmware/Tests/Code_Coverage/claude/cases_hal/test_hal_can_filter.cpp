/*
 * P12 T5 group CF: HAL_CAN_ConfigFilter of stm32f4xx_hal_can.c and the filter behaviour seen through hostCanRx and
 * HAL_CAN_GetRxMessage (FilterMatchIndex), against the bxCAN model (fakes/stm32_hal/HalCan.*). Requirements CF1-CF4 of
 * Tests/Code_Coverage/claude/charters/hal/P12_T5_REQUIREMENTS.md; compiled into ffb_hal_tests and ffb_hal_tests_disco.
 *
 * Oracles: RM0090 Rev 22 32.7.4 (filter bank scale / mode, Figure 342 mapping, Figure 343 numbering, filter priority rules),
 * 32.9.x (CAN_FMR, FM1R, FS1R, FFA1R, FA1R, FiR1 / FiR2), UM1725 Rev 8 chapter 9 (HAL CAN: state requirement, error code).
 *
 * Hand derivation of the filter registers (RM0090 Figure 342), used by the helpers below:
 *  - 32-bit: STID[10:0] in bits 31:21, EXID[17:0] in bits 20:3, IDE bit 2, RTR bit 1, bit 0 reserved.
 *    A standard identifier s gives s << 21; an extended identifier e (29 bit, STID = e[28:18], EXID = e[17:0]) gives (e << 3) | 4.
 *  - 16-bit: STID[10:0] in bits 15:5, RTR bit 4, IDE bit 3, EXID[17:15] in bits 2:0.
 *    A standard identifier s gives s << 5; an extended identifier e gives ((e >> 18) << 5) | 8 | ((e >> 15) & 7).
 *  - FiR1 / FiR2 hold: 32-bit mask = (ID, mask); 32-bit list = (ID n, ID n+1); 16-bit mask = FiR1 (ID n in 15:0, mask n in 31:16),
 *    FiR2 (ID n+1, mask n+1); 16-bit list = FiR1 (ID n 15:0, ID n+1 31:16), FiR2 (ID n+2, ID n+3).
 *
 * NEEDS REVIEW proposals (the lead numbers them):
 *  NEEDS REVIEW 328 characterization: in 16-bit scale the HAL puts FilterIdLow / FilterMaskIdLow into FiR1 (filter n) and FilterIdHigh /
 *       FilterMaskIdHigh into FiR2 (filter n+1); UM1725 calls the High fields "first" and the Low fields "second". Helpers f16mask /
 *       f16list follow the HAL's packing (a register level view); only the register values are asserted as characterization.
 *  NEEDS REVIEW 328 characterization: a FilterScale that is neither 16 nor 32 bit leaves FS1R and FiR1 / FiR2 unchanged (the bank is
 *       still deactivated, mode / FIFO / activation still written). UM1725 says the parameter must be one of the two values.
 *  NEEDS REVIEW 328 characterization: FilterMode other than IDMASK is treated as list mode, FilterFIFOAssignment other than FIFO0 as FIFO1,
 *       FilterActivation other than ENABLE as disabled. UM1725 names only the defined values.
 *  NEEDS REVIEW 328 characterization: FilterBank 28 selects bit 28 of the bank registers but writes FiR1 of an index beyond the bank array
 *       (a register that does not exist; the model records a fault). UM1725 gives the range 0..27 for dual instances.
 *  NEEDS REVIEW 328 characterization: bits already set in ErrorCode stay set when ConfigFilter adds HAL_CAN_ERROR_NOT_INITIALIZED.
 *  NEEDS REVIEW 328 characterization: ConfigFilter in a state other than READY / LISTENING returns HAL_ERROR and sets HAL_CAN_ERROR_NOT_INITIALIZED
 *       (UM1725 and RM0090 do not state the return code or the error bit for this case; two cases).
 */
#include "CanTestSupport.h"

#include <cstdint>

namespace {

typedef CAN_FilterTypeDef Fi;

const uint32_t kFmrReservedMask = ~0x3F01u; /* the model's write mask for CAN_FMR is 0x3F01 (FINIT, CAN2SB) */

Fi fbase(uint32_t bank, uint32_t fifo, uint32_t mode, uint32_t scale, uint32_t act, uint32_t sb) {
	Fi f{};
	f.FilterBank = bank;
	f.FilterFIFOAssignment = fifo;
	f.FilterMode = mode;
	f.FilterScale = scale;
	f.FilterActivation = act;
	f.SlaveStartFilterBank = sb;
	return f;
}
/* 32-bit mask: FiR1 = id, FiR2 = mask (register images, see header). */
Fi f32mask(uint32_t bank, uint32_t fifo, uint32_t id, uint32_t mask, uint32_t act = CAN_FILTER_ENABLE, uint32_t sb = 14) {
	Fi f = fbase(bank, fifo, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, act, sb);
	f.FilterIdHigh = id >> 16;
	f.FilterIdLow = id & 0xFFFFu;
	f.FilterMaskIdHigh = mask >> 16;
	f.FilterMaskIdLow = mask & 0xFFFFu;
	return f;
}
/* 32-bit list: FiR1 = a (filter n), FiR2 = b (filter n+1). */
Fi f32list(uint32_t bank, uint32_t fifo, uint32_t a, uint32_t b, uint32_t act = CAN_FILTER_ENABLE, uint32_t sb = 14) {
	Fi f = fbase(bank, fifo, CAN_FILTERMODE_IDLIST, CAN_FILTERSCALE_32BIT, act, sb);
	f.FilterIdHigh = a >> 16;
	f.FilterIdLow = a & 0xFFFFu;
	f.FilterMaskIdHigh = b >> 16;
	f.FilterMaskIdLow = b & 0xFFFFu;
	return f;
}
/* 16-bit mask: FiR1 = (mask0 << 16) | id0 (filter n), FiR2 = (mask1 << 16) | id1 (filter n+1). The HAL takes the FiR1 half from the Low fields. */
Fi f16mask(uint32_t bank, uint32_t fifo, uint32_t id0, uint32_t m0, uint32_t id1, uint32_t m1, uint32_t act = CAN_FILTER_ENABLE, uint32_t sb = 14) {
	Fi f = fbase(bank, fifo, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_16BIT, act, sb);
	f.FilterIdLow = id0;
	f.FilterMaskIdLow = m0;
	f.FilterIdHigh = id1;
	f.FilterMaskIdHigh = m1;
	return f;
}
/* 16-bit list: FiR1 = (i1 << 16) | i0, FiR2 = (i3 << 16) | i2. */
Fi f16list(uint32_t bank, uint32_t fifo, uint32_t i0, uint32_t i1, uint32_t i2, uint32_t i3, uint32_t act = CAN_FILTER_ENABLE, uint32_t sb = 14) {
	Fi f = fbase(bank, fifo, CAN_FILTERMODE_IDLIST, CAN_FILTERSCALE_16BIT, act, sb);
	f.FilterIdLow = i0;
	f.FilterMaskIdLow = i1;
	f.FilterIdHigh = i2;
	f.FilterMaskIdHigh = i3;
	return f;
}

uint32_t r32(uint32_t id, unsigned ide, unsigned rtr) { return (ide ? ((id << 3) | 4u) : (id << 21)) | ((uint32_t)rtr << 1); }
uint32_t r16(uint32_t id, unsigned ide, unsigned rtr) {
	return (ide ? (((id >> 18) << 5) | 8u | ((id >> 15) & 7u)) : (id << 5)) | ((uint32_t)rtr << 4);
}
const uint32_t kM32All = 0xFFFFFFFEu;   /* every bit of a 32-bit identifier register (bit 0 is reserved) */
const uint32_t kM32IdOnly = 0xFFFFFFF8u; /* STID and EXID, IDE and RTR don't care */
const uint32_t kM16All = 0xFFFFu;

HostCanFrame fr(uint32_t id, unsigned ide = 0, unsigned rtr = 0) {
	HostCanFrame f{};
	f.id = id;
	f.ide = (uint8_t)ide;
	f.rtr = (uint8_t)rtr;
	f.dlc = 1;
	f.data[0] = 0x5A;
	return f;
}

struct Cf : CanFixture {
	CAN_HandleTypeDef h1{}, h2{};
	Cf() {
		setup(h1, CAN1);
		setup(h2, CAN2);
	}
	static void setup(CAN_HandleTypeDef &h, CAN_TypeDef *inst) {
		h.Instance = inst;
		h.Init.Prescaler = 6;
		h.Init.Mode = CAN_MODE_NORMAL;
		h.Init.SyncJumpWidth = CAN_SJW_1TQ;
		h.Init.TimeSeg1 = CAN_BS1_11TQ;
		h.Init.TimeSeg2 = CAN_BS2_2TQ;
		REQUIRE(HAL_CAN_Init(&h) == HAL_OK);
	}
	void cfgOk(const Fi &f, CAN_HandleTypeDef *h = nullptr) {
		REQUIRE(HAL_CAN_ConfigFilter(h ? h : &h1, &f) == HAL_OK);
	}
	void startBoth() {
		REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
		REQUIRE(HAL_CAN_Start(&h2) == HAL_OK);
	}
	/* The filter match index of the oldest message of the FIFO, or -1 when empty; releases it. */
	static int fmi(CAN_HandleTypeDef &h, uint32_t fifo) {
		CAN_RxHeaderTypeDef rh{};
		uint8_t d[8] = {};
		if (HAL_CAN_GetRxMessage(&h, fifo, &rh, d) != HAL_OK) return -1;
		return (int)rh.FilterMatchIndex;
	}
	static uint32_t level(CAN_HandleTypeDef &h, uint32_t fifo) { return HAL_CAN_GetRxFifoFillLevel(&h, fifo); }
	/* Receive on CAN1 and return the FIFO 0 match index (or -1 when the frame was not stored in FIFO 0). */
	int rx0(const HostCanFrame &f) {
		if (hostCanRx(0, &f) != 1) return -1;
		return fmi(h1, CAN_RX_FIFO0);
	}
	static bool stored(unsigned can, const HostCanFrame &f) { return hostCanRx(can, &f) == 1; }
	static uint32_t reg(const volatile void *p) { return HalFixture::peek(p); }
	uint32_t fr1(unsigned n) const { return reg(&CAN1->sFilterRegister[n].FR1); }
	uint32_t fr2(unsigned n) const { return reg(&CAN1->sFilterRegister[n].FR2); }
};

} // namespace

/* ---------------------------------------------------------------- CF1: HAL_CAN_ConfigFilter */

TEST_CASE_FIXTURE(Cf, "characterization: CF1 ConfigFilter of a handle that was never initialised returns an error and writes nothing") {
	// NEEDS REVIEW 328
	CAN_HandleTypeDef h{};
	h.Instance = CAN1;
	const uint32_t fmr = reg(&CAN1->FMR);
	REQUIRE(h.State == HAL_CAN_STATE_RESET);
	CHECK(HAL_CAN_ConfigFilter(&h, &(const Fi &)f32mask(0, 0, 0, 0)) == HAL_ERROR);
	CHECK((h.ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) != 0u);
	CHECK(reg(&CAN1->FMR) == fmr);
	CHECK(reg(&CAN1->FA1R) == 0u);
	CHECK(reg(&CAN1->FS1R) == 0u);
	CHECK(fr1(0) == 0u);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(Cf, "characterization: CF1 ConfigFilter is rejected in the states sleep pending, sleep active and error") {
	// NEEDS REVIEW 328
	const HAL_CAN_StateTypeDef bad[] = {HAL_CAN_STATE_SLEEP_PENDING, HAL_CAN_STATE_SLEEP_ACTIVE, HAL_CAN_STATE_ERROR, HAL_CAN_STATE_RESET};
	for (HAL_CAN_StateTypeDef s : bad) {
		h1.State = s;
		h1.ErrorCode = 0;
		const Fi f = f32mask(2, 0, 0x123u << 21, kM32All);
		CHECK(HAL_CAN_ConfigFilter(&h1, &f) == HAL_ERROR);
		CHECK(h1.ErrorCode == HAL_CAN_ERROR_NOT_INITIALIZED);
		CHECK(reg(&CAN1->FA1R) == 0u);
		CHECK(fr1(2) == 0u);
	}
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter succeeds in the state READY and leaves the error code at zero") {
	REQUIRE(h1.State == HAL_CAN_STATE_READY);
	const Fi f = f32mask(1, 0, 0x7Fu << 21, kM32IdOnly);
	CHECK(HAL_CAN_ConfigFilter(&h1, &f) == HAL_OK);
	CHECK(h1.ErrorCode == 0u);
	CHECK(h1.State == HAL_CAN_STATE_READY);
	CHECK((reg(&CAN1->FA1R) & 2u) != 0u);
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter succeeds in the state LISTENING and keeps that state") {
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	REQUIRE(h1.State == HAL_CAN_STATE_LISTENING);
	const Fi f = f32mask(1, 0, 0x7Fu << 21, kM32IdOnly);
	CHECK(HAL_CAN_ConfigFilter(&h1, &f) == HAL_OK);
	CHECK(h1.ErrorCode == 0u);
	CHECK(h1.State == HAL_CAN_STATE_LISTENING);
	CHECK((reg(&CAN1->FA1R) & 2u) != 0u);
	CHECK(fr1(1) == (0x7Fu << 21));
}

TEST_CASE_FIXTURE(Cf, "characterization: CF1 a rejected ConfigFilter keeps error code bits that were already set") {
	// NEEDS REVIEW 328
	h1.State = HAL_CAN_STATE_ERROR;
	h1.ErrorCode = HAL_CAN_ERROR_TIMEOUT;
	const Fi f = f32mask(0, 0, 0, 0);
	CHECK(HAL_CAN_ConfigFilter(&h1, &f) == HAL_ERROR);
	CHECK(h1.ErrorCode == (HAL_CAN_ERROR_TIMEOUT | HAL_CAN_ERROR_NOT_INITIALIZED));
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter leaves the filter initialisation mode: FINIT is 1 at reset and 0 after the call") {
	REQUIRE((reg(&CAN1->FMR) & 1u) == 1u); // RM0090 32.9: CAN_FMR reset value 0x2A1C0E01, FINIT = 1
	cfgOk(f32mask(0, 0, 0, 0));
	CHECK((reg(&CAN1->FMR) & 1u) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter reaches the filter registers with FINIT set, so a reconfigured active bank is not write protected") {
	// RM0090 32.9.x: FiRx are writable only when FACTx = 0 or FINIT = 1.
	cfgOk(f32mask(4, 0, r32(0x100, 0, 0), kM32All));
	REQUIRE((reg(&CAN1->FA1R) & 0x10u) != 0u);
	cfgOk(f32mask(4, 1, r32(0x200, 0, 0), kM32IdOnly));
	CHECK(hostCanProtectedWrites(0) == 0u);
	CHECK(fr1(4) == r32(0x200, 0, 0));
	CHECK(fr2(4) == kM32IdOnly);
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter keeps the bits of CAN_FMR outside FINIT and CAN2SB") {
	const uint32_t before = reg(&CAN1->FMR) & kFmrReservedMask;
	cfgOk(f32mask(0, 0, 0, 0, CAN_FILTER_ENABLE, 3));
	CHECK((reg(&CAN1->FMR) & kFmrReservedMask) == before);
	CHECK(before == 0x2A1C0000u); // reset value 0x2A1C0E01 without FINIT and CAN2SB
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter writes SlaveStartFilterBank to CAN2SB for 0, 1, 14, 27 and 28") {
	const uint32_t values[] = {0, 1, 14, 27, 28};
	for (uint32_t v : values) {
		cfgOk(f32mask(0, 0, 0, 0, CAN_FILTER_ENABLE, v));
		CHECK(((reg(&CAN1->FMR) >> 8) & 0x3Fu) == v);
	}
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter replaces the previous CAN2SB value: bits of the old value are cleared") {
	cfgOk(f32mask(0, 0, 0, 0, CAN_FILTER_ENABLE, 27)); // 0b011011
	cfgOk(f32mask(0, 0, 0, 0, CAN_FILTER_ENABLE, 4));  // 0b000100
	CHECK(((reg(&CAN1->FMR) >> 8) & 0x3Fu) == 4u);
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter through the CAN2 handle writes the shared filter registers of CAN1") {
	cfgOk(f32mask(20, 1, r32(0x321, 0, 0), kM32All, CAN_FILTER_ENABLE, 10), &h2);
	CHECK(((reg(&CAN1->FMR) >> 8) & 0x3Fu) == 10u);
	CHECK((reg(&CAN1->FA1R) & (1u << 20)) != 0u);
	CHECK((reg(&CAN1->FFA1R) & (1u << 20)) != 0u);
	CHECK(fr1(20) == r32(0x321, 0, 0));
	CHECK(fr2(20) == kM32All);
	CHECK((reg(&CAN1->FMR) & 1u) == 0u);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF1 a filter configured with DISABLE clears the activation bit of an active bank") {
	cfgOk(f32mask(6, 0, 0, 0));
	REQUIRE((reg(&CAN1->FA1R) & (1u << 6)) != 0u);
	cfgOk(f32mask(6, 0, 0, 0, CAN_FILTER_DISABLE));
	CHECK((reg(&CAN1->FA1R) & (1u << 6)) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF1 a filter configured with ENABLE sets only its own activation bit") {
	cfgOk(f32mask(3, 0, 0, 0));
	cfgOk(f32mask(5, 0, 0, 0));
	CHECK(reg(&CAN1->FA1R) == ((1u << 3) | (1u << 5)));
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter writes the FM1R bit of its own bank only (mask 0, list 1)") {
	cfgOk(f32list(3, 0, 0, 0));
	cfgOk(f32mask(5, 0, 0, 0));
	CHECK(reg(&CAN1->FM1R) == (1u << 3));
	cfgOk(f32mask(3, 0, 0, 0)); // list -> mask clears the bit
	CHECK(reg(&CAN1->FM1R) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter writes the FS1R bit of its own bank only (16 bit 0, 32 bit 1)") {
	cfgOk(f32mask(3, 0, 0, 0));
	cfgOk(f16mask(5, 0, 0, 0, 0, 0));
	CHECK(reg(&CAN1->FS1R) == (1u << 3));
	cfgOk(f16mask(3, 0, 0, 0, 0, 0)); // 32 -> 16 bit clears the bit
	CHECK(reg(&CAN1->FS1R) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF1 ConfigFilter writes the FFA1R bit of its own bank only (FIFO 0 is 0, FIFO 1 is 1)") {
	cfgOk(f32mask(3, CAN_FILTER_FIFO1, 0, 0));
	cfgOk(f32mask(5, CAN_FILTER_FIFO0, 0, 0));
	CHECK(reg(&CAN1->FFA1R) == (1u << 3));
	cfgOk(f32mask(3, CAN_FILTER_FIFO0, 0, 0));
	CHECK(reg(&CAN1->FFA1R) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF1 32-bit mask mode writes FiR1 = identifier and FiR2 = mask (IdHigh / IdLow / MaskIdHigh / MaskIdLow packing)") {
	Fi f = fbase(2, CAN_FILTER_FIFO0, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, CAN_FILTER_ENABLE, 14);
	f.FilterIdHigh = 0x1234;
	f.FilterIdLow = 0x5678;
	f.FilterMaskIdHigh = 0x9ABC;
	f.FilterMaskIdLow = 0xDEF0;
	cfgOk(f);
	CHECK(fr1(2) == 0x12345678u); // 32-bit identifier: MSBs from IdHigh (UM1725), LSBs from IdLow
	CHECK(fr2(2) == 0x9ABCDEF0u);
	CHECK((reg(&CAN1->FS1R) & 4u) != 0u);
	CHECK((reg(&CAN1->FM1R) & 4u) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF1 32-bit list mode writes FiR1 and FiR2 as two identifiers and sets FS1R and FM1R") {
	Fi f = fbase(7, CAN_FILTER_FIFO1, CAN_FILTERMODE_IDLIST, CAN_FILTERSCALE_32BIT, CAN_FILTER_ENABLE, 14);
	f.FilterIdHigh = 0xAAAA;
	f.FilterIdLow = 0x5555;
	f.FilterMaskIdHigh = 0x0F0F;
	f.FilterMaskIdLow = 0xF0F0;
	cfgOk(f);
	CHECK(fr1(7) == 0xAAAA5555u);
	CHECK(fr2(7) == 0x0F0FF0F0u);
	CHECK((reg(&CAN1->FS1R) & 0x80u) != 0u);
	CHECK((reg(&CAN1->FM1R) & 0x80u) != 0u);
	CHECK((reg(&CAN1->FFA1R) & 0x80u) != 0u);
}

TEST_CASE_FIXTURE(Cf, "characterization: CF1 16-bit mask mode takes the Low fields as FiR1 and the High fields as FiR2") {
	// NEEDS REVIEW 328. RM0090 Figure 342: FiR1[15:0] = ID of filter n, FiR1[31:16] = its mask; FiR2 = filter n+1.
	Fi f = fbase(9, CAN_FILTER_FIFO0, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_16BIT, CAN_FILTER_ENABLE, 14);
	f.FilterIdHigh = 0x1111;
	f.FilterIdLow = 0x2222;
	f.FilterMaskIdHigh = 0x3333;
	f.FilterMaskIdLow = 0x4444;
	cfgOk(f);
	CHECK(fr1(9) == 0x44442222u);
	CHECK(fr2(9) == 0x33331111u);
	CHECK((reg(&CAN1->FS1R) & (1u << 9)) == 0u);
	CHECK((reg(&CAN1->FM1R) & (1u << 9)) == 0u);
}

TEST_CASE_FIXTURE(Cf, "characterization: CF1 16-bit list mode takes the Low fields as FiR1 and the High fields as FiR2") {
	// NEEDS REVIEW 328
	Fi f = fbase(10, CAN_FILTER_FIFO1, CAN_FILTERMODE_IDLIST, CAN_FILTERSCALE_16BIT, CAN_FILTER_ENABLE, 14);
	f.FilterIdHigh = 0x1111;
	f.FilterIdLow = 0x2222;
	f.FilterMaskIdHigh = 0x3333;
	f.FilterMaskIdLow = 0x4444;
	cfgOk(f);
	CHECK(fr1(10) == 0x44442222u);
	CHECK(fr2(10) == 0x33331111u);
	CHECK((reg(&CAN1->FS1R) & (1u << 10)) == 0u);
	CHECK((reg(&CAN1->FM1R) & (1u << 10)) != 0u);
	CHECK((reg(&CAN1->FFA1R) & (1u << 10)) != 0u);
}

TEST_CASE_FIXTURE(Cf, "CF1 a 16-bit configuration writes only 16 bits of each filter parameter (upper bits are masked off)") {
	Fi f = fbase(1, CAN_FILTER_FIFO0, CAN_FILTERMODE_IDLIST, CAN_FILTERSCALE_16BIT, CAN_FILTER_ENABLE, 14);
	f.FilterIdHigh = 0xFFFF0001u;
	f.FilterIdLow = 0xFFFF0002u;
	f.FilterMaskIdHigh = 0xFFFF0003u;
	f.FilterMaskIdLow = 0xFFFF0004u;
	cfgOk(f);
	CHECK(fr1(1) == 0x00040002u);
	CHECK(fr2(1) == 0x00030001u);
}

TEST_CASE_FIXTURE(Cf, "CF1 a 32-bit configuration writes only 16 bits of each filter parameter (upper bits are masked off)") {
	Fi f = fbase(1, CAN_FILTER_FIFO0, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, CAN_FILTER_ENABLE, 14);
	f.FilterIdHigh = 0xFFFF0001u;
	f.FilterIdLow = 0xFFFF0002u;
	f.FilterMaskIdHigh = 0xFFFF0003u;
	f.FilterMaskIdLow = 0xFFFF0004u;
	cfgOk(f);
	CHECK(fr1(1) == 0x00010002u);
	CHECK(fr2(1) == 0x00030004u);
}

TEST_CASE_FIXTURE(Cf, "CF1 all 28 filter banks can be configured, each in its own registers") {
	for (uint32_t n = 0; n < 28; ++n) {
		cfgOk(f32mask(n, n & 1u, (n + 1u) << 16, (n + 100u) << 16, CAN_FILTER_ENABLE, 28));
		CHECK(fr1(n) == ((n + 1u) << 16));
		CHECK(fr2(n) == ((n + 100u) << 16));
		CHECK((reg(&CAN1->FA1R) & (1u << n)) != 0u);
	}
	CHECK(reg(&CAN1->FA1R) == 0x0FFFFFFFu);
	CHECK(reg(&CAN1->FFA1R) == 0x0AAAAAAAu); // odd banks assigned to FIFO 1
	CHECK(reg(&CAN1->FS1R) == 0x0FFFFFFFu);
	CHECK(reg(&CAN1->FM1R) == 0u);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF1 configuring one bank leaves the registers of its neighbours untouched") {
	cfgOk(f32mask(10, 0, 0xAAAAAAA8u, 0x55555554u));
	cfgOk(f32mask(12, 0, 0x11111110u, 0x22222222u));
	cfgOk(f32mask(11, 0, 0x33333330u, 0x44444444u));
	CHECK(fr1(10) == 0xAAAAAAA8u);
	CHECK(fr2(10) == 0x55555554u);
	CHECK(fr1(12) == 0x11111110u);
	CHECK(fr2(12) == 0x22222222u);
	CHECK(fr1(9) == 0u);
	CHECK(fr1(13) == 0u);
}

TEST_CASE_FIXTURE(Cf, "characterization: CF1 an unknown FilterScale leaves FS1R and the bank registers unchanged") {
	// NEEDS REVIEW 328
	cfgOk(f32mask(3, 0, 0x12340000u, 0x56780000u));
	Fi f = f32mask(3, 0, 0xFFFF0000u, 0xFFFF0000u);
	f.FilterScale = 5;
	cfgOk(f);
	CHECK(fr1(3) == 0x12340000u);
	CHECK(fr2(3) == 0x56780000u);
	CHECK((reg(&CAN1->FS1R) & 8u) != 0u);
	CHECK((reg(&CAN1->FA1R) & 8u) != 0u); // activation is still applied
}

TEST_CASE_FIXTURE(Cf, "characterization: CF1 undefined mode, FIFO and activation values fall back to list, FIFO 1 and disabled") {
	// NEEDS REVIEW 328
	Fi f = f32mask(8, 0, 0, 0);
	f.FilterMode = 7;
	f.FilterFIFOAssignment = 7;
	f.FilterActivation = 7;
	cfgOk(f);
	CHECK((reg(&CAN1->FM1R) & (1u << 8)) != 0u);
	CHECK((reg(&CAN1->FFA1R) & (1u << 8)) != 0u);
	CHECK((reg(&CAN1->FA1R) & (1u << 8)) == 0u);
}

TEST_CASE_FIXTURE(Cf, "characterization: CF1 bank 28 uses an index beyond the 28 banks (a register that does not exist)") {
	// NEEDS REVIEW 328. RM0090: the dual instance has banks 0..27 (FiR1 / FiR2 end at 0x31C).
	cfgOk(f32mask(28, 0, 0, 0));
	CHECK(HalFixture::faults() > 0u);
	HalFixture::clearFaults();
}

/* ---------------------------------------------------------------- CF2: receive through the filters */

TEST_CASE_FIXTURE(Cf, "CF2 no frame is stored when no filter was ever configured (FINIT = 1 after reset)") {
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(hostCanRx(0, &(const HostCanFrame &)fr(0x123)) == 0);
	CHECK(level(h1, CAN_RX_FIFO0) == 0u);
	CHECK(level(h1, CAN_RX_FIFO1) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF2 no frame is stored when the only configured filter bank is deactivated") {
	cfgOk(f32mask(0, 0, 0, 0, CAN_FILTER_DISABLE)); // mask 0 would accept everything if it were active
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(hostCanRx(0, &(const HostCanFrame &)fr(0x123)) == 0);
	CHECK(level(h1, CAN_RX_FIFO0) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF2 an active bank with an all-zero mask accepts every frame") {
	cfgOk(f32mask(0, 0, 0, 0));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x123)));
	CHECK(stored(0, fr(0x1FFFFFFF, 1)));
	CHECK(stored(0, fr(0x7FF, 0, 1)));
	CHECK(level(h1, CAN_RX_FIFO0) == 3u);
}

TEST_CASE_FIXTURE(Cf, "CF2 reception is deactivated while FINIT = 1 and works again when FINIT is cleared") {
	cfgOk(f32mask(0, 0, 0, 0));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CAN1->FMR |= 1u;
	CHECK(hostCanRx(0, &(const HostCanFrame &)fr(0x55)) == 0);
	CHECK(level(h1, CAN_RX_FIFO0) == 0u);
	CAN1->FMR &= ~1u;
	CHECK(stored(0, fr(0x55)));
	CHECK(level(h1, CAN_RX_FIFO0) == 1u);
}

TEST_CASE_FIXTURE(Cf, "CF2 32-bit mask with all identifier bits compared accepts only the exact standard identifier") {
	cfgOk(f32mask(0, 0, r32(0x123, 0, 0), kM32All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x123)));
	CHECK_FALSE(stored(0, fr(0x122))); // differs in bit 0 of the identifier
	CHECK_FALSE(stored(0, fr(0x323))); // differs in bit 9
	CHECK_FALSE(stored(0, fr(0x7FF)));
	CHECK(level(h1, CAN_RX_FIFO0) == 1u);
}

TEST_CASE_FIXTURE(Cf, "CF2 32-bit mask don't care bits of a standard identifier let both values pass") {
	// mask: STID[10:4] compared, STID[3:0] don't care, IDE and RTR compared
	const uint32_t mask = (0x7F0u << 21) | 4u | 2u;
	cfgOk(f32mask(0, 0, r32(0x120, 0, 0), mask));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x120)));
	CHECK(stored(0, fr(0x12F)));
	CHECK_FALSE(stored(0, fr(0x130)));
	CHECK_FALSE(stored(0, fr(0x110)));
}

TEST_CASE_FIXTURE(Cf, "CF2 32-bit mask compares the 29 bit extended identifier bit by bit") {
	const uint32_t id = 0x1ABCDEF1u;
	cfgOk(f32mask(0, 0, r32(id, 1, 0), kM32All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(id, 1)));
	CHECK_FALSE(stored(0, fr(id ^ 1u, 1)));          // bit 0 (EXID[0])
	CHECK_FALSE(stored(0, fr(id ^ (1u << 17), 1)));  // EXID[17]
	CHECK_FALSE(stored(0, fr(id ^ (1u << 18), 1)));  // STID[0]
	CHECK_FALSE(stored(0, fr(id ^ (1u << 28), 1)));  // STID[10]
	CHECK(level(h1, CAN_RX_FIFO0) == 1u);
}

TEST_CASE_FIXTURE(Cf, "CF2 32-bit mask with IDE compared rejects an extended frame whose bits equal a standard filter") {
	cfgOk(f32mask(0, 0, r32(0x123, 0, 0), kM32All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	// extended identifier with STID = 0x123 and EXID = 0 sits at the same register bits as the standard one except IDE
	CHECK_FALSE(stored(0, fr(0x123u << 18, 1)));
	CHECK(stored(0, fr(0x123)));
}

TEST_CASE_FIXTURE(Cf, "CF2 32-bit mask with IDE not compared accepts a standard and an extended frame with equal STID") {
	// STID[10:0] compared, everything else don't care
	cfgOk(f32mask(0, 0, r32(0x123, 0, 0), 0xFFE00000u));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x123)));
	CHECK(stored(0, fr((0x123u << 18) | 0x2AAAAu, 1)));
	CHECK_FALSE(stored(0, fr(0x124)));
}

TEST_CASE_FIXTURE(Cf, "CF2 32-bit mask with RTR compared separates data frames from remote frames") {
	cfgOk(f32mask(0, 0, r32(0x123, 0, 1), kM32All)); // remote frames of 0x123 only
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x123, 0, 1)));
	CHECK_FALSE(stored(0, fr(0x123, 0, 0)));
}

TEST_CASE_FIXTURE(Cf, "CF2 32-bit mask with RTR not compared accepts data and remote frames") {
	cfgOk(f32mask(0, 0, r32(0x123, 0, 0), kM32All & ~2u));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x123, 0, 0)));
	CHECK(stored(0, fr(0x123, 0, 1)));
	CHECK(level(h1, CAN_RX_FIFO0) == 2u);
}

TEST_CASE_FIXTURE(Cf, "CF2 32-bit list accepts exactly the two listed identifiers of the bank") {
	cfgOk(f32list(0, 0, r32(0x100, 0, 0), r32(0x1FFFFFF0u, 1, 0)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x100)));
	CHECK(stored(0, fr(0x1FFFFFF0u, 1)));
	CHECK_FALSE(stored(0, fr(0x101)));
	CHECK_FALSE(stored(0, fr(0x1FFFFFF1u, 1)));
	CHECK(level(h1, CAN_RX_FIFO0) == 2u);
}

TEST_CASE_FIXTURE(Cf, "CF2 32-bit list compares IDE and RTR as well as the identifier") {
	cfgOk(f32list(0, 0, r32(0x100, 0, 0), r32(0x100, 0, 1)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x100, 0, 0)));
	CHECK(stored(0, fr(0x100, 0, 1)));
	CHECK_FALSE(stored(0, fr(0x100u << 18, 1, 0))); // same register bits 31:21 but IDE = 1
}

TEST_CASE_FIXTURE(Cf, "CF2 16-bit mask bank filters two identifiers (slot 0 and slot 1) of standard frames") {
	cfgOk(f16mask(0, 0, r16(0x111, 0, 0), kM16All, r16(0x222, 0, 0), kM16All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x111)));
	CHECK(stored(0, fr(0x222)));
	CHECK_FALSE(stored(0, fr(0x333)));
	CHECK_FALSE(stored(0, fr(0x110)));
	CHECK(level(h1, CAN_RX_FIFO0) == 2u);
}

TEST_CASE_FIXTURE(Cf, "CF2 16-bit mask don't care bits let both values pass") {
	// slot 0: STID[10:4] compared; slot 1 never matches
	cfgOk(f16mask(0, 0, r16(0x120, 0, 0), (0x7F0u << 5) | 0x18u, r16(0x7FF, 0, 0), kM16All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x120)));
	CHECK(stored(0, fr(0x12F)));
	CHECK_FALSE(stored(0, fr(0x130)));
}

TEST_CASE_FIXTURE(Cf, "CF2 16-bit filters compare STID and EXID[17:15] of an extended frame, not the lower 15 bits") {
	const uint32_t id = (0x155u << 18) | (5u << 15) | 0x1234u;
	cfgOk(f16mask(0, 0, r16(id, 1, 0), kM16All, r16(0x7FF, 0, 0), kM16All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(id, 1)));
	CHECK(stored(0, fr(id ^ 0x7FFFu, 1)));                 // EXID[14:0] are not part of a 16-bit filter
	CHECK_FALSE(stored(0, fr(id ^ (1u << 15), 1)));        // EXID[15]
	CHECK_FALSE(stored(0, fr(id ^ (1u << 18), 1)));        // STID[0]
	CHECK_FALSE(stored(0, fr((id >> 18), 0)));             // standard frame with the same STID: IDE differs
}

TEST_CASE_FIXTURE(Cf, "CF2 16-bit filters compare RTR") {
	cfgOk(f16mask(0, 0, r16(0x111, 0, 1), kM16All, r16(0x7FF, 0, 0), kM16All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x111, 0, 1)));
	CHECK_FALSE(stored(0, fr(0x111, 0, 0)));
}

TEST_CASE_FIXTURE(Cf, "CF2 16-bit list bank accepts the four listed identifiers, each of the four slots") {
	const uint32_t ids[4] = {0x101, 0x202, 0x303, 0x404};
	cfgOk(f16list(0, 0, r16(ids[0], 0, 0), r16(ids[1], 0, 0), r16(ids[2], 0, 0), r16(ids[3], 0, 0)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	for (uint32_t id : ids) {
		CHECK(stored(0, fr(id)));
		CHECK(level(h1, CAN_RX_FIFO0) == 1u);
		CHECK(fmi(h1, CAN_RX_FIFO0) >= 0); // release it (the FIFO holds three)
	}
	CHECK_FALSE(stored(0, fr(0x505)));
	CHECK_FALSE(stored(0, fr(0x100)));
}

TEST_CASE_FIXTURE(Cf, "CF2 16-bit list compares IDE: an extended frame with the listed STID is rejected") {
	cfgOk(f16list(0, 0, r16(0x101, 0, 0), r16(0x7FF, 1, 1), r16(0x7FE, 1, 1), r16(0x7FD, 1, 1)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK_FALSE(stored(0, fr(0x101u << 18, 1)));
	CHECK(stored(0, fr(0x101)));
}

TEST_CASE_FIXTURE(Cf, "CF2 a bank assigned to FIFO 1 stores matching frames in FIFO 1 only") {
	cfgOk(f32mask(0, CAN_FILTER_FIFO1, r32(0x123, 0, 0), kM32All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x123)));
	CHECK(level(h1, CAN_RX_FIFO1) == 1u);
	CHECK(level(h1, CAN_RX_FIFO0) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF2 banks of FIFO 0 and FIFO 1 deliver to their own FIFO") {
	cfgOk(f32mask(0, CAN_FILTER_FIFO0, r32(0x100, 0, 0), kM32All));
	cfgOk(f32mask(1, CAN_FILTER_FIFO1, r32(0x200, 0, 0), kM32All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(stored(0, fr(0x100)));
	CHECK(stored(0, fr(0x200)));
	CHECK(stored(0, fr(0x200)));
	CHECK(level(h1, CAN_RX_FIFO0) == 1u);
	CHECK(level(h1, CAN_RX_FIFO1) == 2u);
}

TEST_CASE_FIXTURE(Cf, "CF2 a frame that matches no active bank is not stored and leaves the FIFOs empty") {
	cfgOk(f32mask(0, 0, r32(0x100, 0, 0), kM32All));
	cfgOk(f32list(1, 1, r32(0x200, 0, 0), r32(0x300, 0, 0)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(hostCanRx(0, &(const HostCanFrame &)fr(0x400)) == 0);
	CHECK(level(h1, CAN_RX_FIFO0) == 0u);
	CHECK(level(h1, CAN_RX_FIFO1) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF2 reconfiguring a bank replaces the old filter") {
	cfgOk(f32mask(0, 0, r32(0x100, 0, 0), kM32All));
	cfgOk(f32mask(0, 0, r32(0x200, 0, 0), kM32All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK_FALSE(stored(0, fr(0x100)));
	CHECK(stored(0, fr(0x200)));
}

TEST_CASE_FIXTURE(Cf, "CF2 disabling a bank stops its frames while another bank keeps receiving") {
	cfgOk(f32mask(0, 0, r32(0x100, 0, 0), kM32All));
	cfgOk(f32mask(1, 0, r32(0x200, 0, 0), kM32All));
	cfgOk(f32mask(0, 0, r32(0x100, 0, 0), kM32All, CAN_FILTER_DISABLE));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK_FALSE(stored(0, fr(0x100)));
	CHECK(stored(0, fr(0x200)));
}

TEST_CASE_FIXTURE(Cf, "CF2 a filter configured while the node is LISTENING takes effect for the next frame") {
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	cfgOk(f32mask(0, 0, r32(0x123, 0, 0), kM32All));
	CHECK(stored(0, fr(0x123)));
}

/* ---------------------------------------------------------------- CF3: filter match index and priority */

TEST_CASE_FIXTURE(Cf, "CF3 32-bit list gives index 0 to the first identifier and 1 to the second") {
	cfgOk(f32list(0, 0, r32(0x100, 0, 0), r32(0x200, 0, 0)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x100)) == 0);
	CHECK(rx0(fr(0x200)) == 1);
}

TEST_CASE_FIXTURE(Cf, "CF3 16-bit list gives the indices 0 to 3 to its four slots") {
	cfgOk(f16list(0, 0, r16(0x101, 0, 0), r16(0x202, 0, 0), r16(0x303, 0, 0), r16(0x404, 0, 0)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x101)) == 0);
	CHECK(rx0(fr(0x202)) == 1);
	CHECK(rx0(fr(0x303)) == 2);
	CHECK(rx0(fr(0x404)) == 3);
}

TEST_CASE_FIXTURE(Cf, "CF3 16-bit mask gives the indices 0 and 1 to its two slots") {
	cfgOk(f16mask(0, 0, r16(0x111, 0, 0), kM16All, r16(0x222, 0, 0), kM16All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x111)) == 0);
	CHECK(rx0(fr(0x222)) == 1);
}

TEST_CASE_FIXTURE(Cf, "CF3 numbering continues across banks and scales (Figure 343): 2 + 1 + 4 + 2 + 2 + 1 filters") {
	// bank 0: 32-bit list (0,1); bank 1: 32-bit mask (2); bank 2: 16-bit list (3..6); bank 3: 16-bit mask (7,8);
	// bank 4: 32-bit list (9,10); bank 5: 32-bit mask (11)
	cfgOk(f32list(0, 0, r32(0x010, 0, 0), r32(0x011, 0, 0)));
	cfgOk(f32mask(1, 0, r32(0x020, 0, 0), kM32All));
	cfgOk(f16list(2, 0, r16(0x030, 0, 0), r16(0x031, 0, 0), r16(0x032, 0, 0), r16(0x033, 0, 0)));
	cfgOk(f16mask(3, 0, r16(0x040, 0, 0), kM16All, r16(0x041, 0, 0), kM16All));
	cfgOk(f32list(4, 0, r32(0x050, 0, 0), r32(0x051, 0, 0)));
	cfgOk(f32mask(5, 0, r32(0x060, 0, 0), kM32All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x011)) == 1);
	CHECK(rx0(fr(0x020)) == 2);
	CHECK(rx0(fr(0x030)) == 3);
	CHECK(rx0(fr(0x033)) == 6);
	CHECK(rx0(fr(0x040)) == 7);
	CHECK(rx0(fr(0x041)) == 8);
	CHECK(rx0(fr(0x050)) == 9);
	CHECK(rx0(fr(0x051)) == 10);
	CHECK(rx0(fr(0x060)) == 11);
}

TEST_CASE_FIXTURE(Cf, "CF3 a deactivated bank still takes its filter numbers (RM0090: the index ignores the activation state)") {
	cfgOk(f32list(0, 0, r32(0x010, 0, 0), r32(0x011, 0, 0), CAN_FILTER_DISABLE)); // numbers 0, 1
	cfgOk(f32mask(1, 0, r32(0x020, 0, 0), kM32All, CAN_FILTER_DISABLE));          // number 2
	cfgOk(f32mask(2, 0, r32(0x030, 0, 0), kM32All));                              // number 3
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(hostCanRx(0, &(const HostCanFrame &)fr(0x010)) == 0);
	CHECK(rx0(fr(0x030)) == 3);
}

TEST_CASE_FIXTURE(Cf, "CF3 an unconfigured bank (all registers zero, FIFO 0, 16-bit mask) takes two numbers") {
	// banks reset to FS1R = 0 (16-bit), FM1R = 0 (mask), FFA1R = 0 (FIFO 0): two filters each
	cfgOk(f32mask(2, 0, r32(0x030, 0, 0), kM32All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x030)) == 4); // banks 0 and 1 take 2 numbers each
}

TEST_CASE_FIXTURE(Cf, "CF3 numbers restart at 0 for each FIFO") {
	cfgOk(f32mask(0, CAN_FILTER_FIFO0, r32(0x100, 0, 0), kM32All));
	cfgOk(f32mask(1, CAN_FILTER_FIFO1, r32(0x200, 0, 0), kM32All));
	cfgOk(f32mask(2, CAN_FILTER_FIFO0, r32(0x300, 0, 0), kM32All));
	cfgOk(f32mask(3, CAN_FILTER_FIFO1, r32(0x400, 0, 0), kM32All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	REQUIRE(stored(0, fr(0x100)));
	REQUIRE(stored(0, fr(0x200)));
	REQUIRE(stored(0, fr(0x300)));
	REQUIRE(stored(0, fr(0x400)));
	CHECK(fmi(h1, CAN_RX_FIFO0) == 0);
	CHECK(fmi(h1, CAN_RX_FIFO0) == 1);
	CHECK(fmi(h1, CAN_RX_FIFO1) == 0);
	CHECK(fmi(h1, CAN_RX_FIFO1) == 1);
}

TEST_CASE_FIXTURE(Cf, "CF3 the highest index of 28 banks of 16-bit lists is 111 and fits the FMI field") {
	for (uint32_t n = 0; n < 27; ++n) cfgOk(f16list(n, 0, 0, 0, 0, 0, CAN_FILTER_DISABLE, 28));
	cfgOk(f16list(27, 0, r16(0x700, 0, 0), r16(0x701, 0, 0), r16(0x702, 0, 0), r16(0x703, 0, 0), CAN_FILTER_ENABLE, 28));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x700)) == 108);
	CHECK(rx0(fr(0x703)) == 111);
}

TEST_CASE_FIXTURE(Cf, "CF3 priority: a 32-bit list entry wins over a 32-bit mask on a lower bank") {
	cfgOk(f32mask(0, 0, r32(0x123, 0, 0), kM32All)); // number 0
	cfgOk(f32list(1, 0, r32(0x7FF, 0, 0), r32(0x123, 0, 0))); // numbers 1, 2
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x123)) == 2);
}

TEST_CASE_FIXTURE(Cf, "CF3 priority: a 32-bit mask wins over a 16-bit list on a lower bank") {
	cfgOk(f16list(0, 0, r16(0x123, 0, 0), 0, 0, 0)); // numbers 0..3, matches
	cfgOk(f32mask(1, 0, r32(0x123, 0, 0), kM32All));  // number 4
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x123)) == 4);
}

TEST_CASE_FIXTURE(Cf, "CF3 priority: a 16-bit list wins over a 16-bit mask on a lower bank") {
	cfgOk(f16mask(0, 0, r16(0x123, 0, 0), kM16All, r16(0x7FF, 0, 0), kM16All)); // numbers 0, 1
	cfgOk(f16list(1, 0, r16(0x7FF, 0, 0), r16(0x7FE, 0, 0), r16(0x7FD, 0, 0), r16(0x123, 0, 0))); // numbers 2..5
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x123)) == 5);
}

TEST_CASE_FIXTURE(Cf, "CF3 priority: a 32-bit list wins over everything else (list 32, mask 32, list 16, mask 16)") {
	cfgOk(f16mask(0, 0, r16(0x123, 0, 0), kM16All, 0, 0xFFFF));                         // numbers 0, 1
	cfgOk(f16list(1, 0, r16(0x123, 0, 0), 0, 0, 0));                                     // numbers 2..5
	cfgOk(f32mask(2, 0, r32(0x123, 0, 0), kM32All));                                     // number 6
	cfgOk(f32list(3, 0, r32(0x123, 0, 0), r32(0x7FF, 0, 0)));                            // numbers 7, 8
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x123)) == 7);
}

TEST_CASE_FIXTURE(Cf, "CF3 priority: of two 32-bit masks the lower bank wins") {
	cfgOk(f32mask(0, 0, r32(0x100, 0, 0), 0xFFE00000u | 0x0u)); // number 0, STID only
	cfgOk(f32mask(1, 0, r32(0x100, 0, 0), kM32All));            // number 1
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x100)) == 0);
}

TEST_CASE_FIXTURE(Cf, "CF3 priority: of two equal 32-bit list entries in one bank the first (lower number) wins") {
	cfgOk(f32list(0, 0, r32(0x100, 0, 0), r32(0x100, 0, 0)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x100)) == 0);
}

TEST_CASE_FIXTURE(Cf, "CF3 priority: of equal 16-bit list entries in slots 1 and 3 the lower number wins") {
	cfgOk(f16list(0, 0, r16(0x7FF, 0, 0), r16(0x123, 0, 0), r16(0x7FE, 0, 0), r16(0x123, 0, 0)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x123)) == 1);
}

TEST_CASE_FIXTURE(Cf, "CF3 priority: of two 16-bit masks in one bank the first slot wins") {
	cfgOk(f16mask(0, 0, r16(0x120, 0, 0), (0x7F0u << 5) | 0x18u, r16(0x123, 0, 0), kM16All));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	CHECK(rx0(fr(0x123)) == 0);
}

TEST_CASE_FIXTURE(Cf, "CF3 priority across FIFOs: the list entry of FIFO 1 wins over the mask of FIFO 0 and the frame goes to FIFO 1 only") {
	cfgOk(f32mask(0, CAN_FILTER_FIFO0, r32(0x123, 0, 0), kM32All));
	cfgOk(f32list(1, CAN_FILTER_FIFO1, r32(0x123, 0, 0), r32(0x7FF, 0, 0)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	REQUIRE(stored(0, fr(0x123)));
	CHECK(level(h1, CAN_RX_FIFO0) == 0u);
	CHECK(level(h1, CAN_RX_FIFO1) == 1u);
	CHECK(fmi(h1, CAN_RX_FIFO1) == 0); // number 0 of FIFO 1 (the list entry of bank 1 is the first filter of that FIFO)
}

TEST_CASE_FIXTURE(Cf, "CF3 an extended frame is reported with its filter match index") {
	cfgOk(f32list(0, 0, r32(0x100, 0, 0), r32(0x1ABCDEF1u, 1, 0)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	REQUIRE(hostCanRx(0, &(const HostCanFrame &)fr(0x1ABCDEF1u, 1)) == 1);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8] = {};
	REQUIRE(HAL_CAN_GetRxMessage(&h1, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.IDE == CAN_ID_EXT);
	CHECK(rh.ExtId == 0x1ABCDEF1u);
	CHECK(rh.FilterMatchIndex == 1u);
}

TEST_CASE_FIXTURE(Cf, "CF3 a standard frame is reported with its filter match index") {
	cfgOk(f32list(0, 0, r32(0x100, 0, 0), r32(0x1ABCDEF1u, 1, 0)));
	REQUIRE(HAL_CAN_Start(&h1) == HAL_OK);
	REQUIRE(hostCanRx(0, &(const HostCanFrame &)fr(0x100)) == 1);
	CAN_RxHeaderTypeDef rh{};
	uint8_t d[8] = {};
	REQUIRE(HAL_CAN_GetRxMessage(&h1, CAN_RX_FIFO0, &rh, d) == HAL_OK);
	CHECK(rh.IDE == CAN_ID_STD);
	CHECK(rh.StdId == 0x100u);
	CHECK(rh.FilterMatchIndex == 0u);
}

/* ---------------------------------------------------------------- CF4: CAN1 / CAN2 bank split */

TEST_CASE_FIXTURE(Cf, "CF4 with CAN2SB = 14 bank 13 filters CAN1 and bank 14 filters CAN2") {
	cfgOk(f32mask(13, 0, r32(0x100, 0, 0), kM32All, CAN_FILTER_ENABLE, 14));
	cfgOk(f32mask(14, 0, r32(0x200, 0, 0), kM32All, CAN_FILTER_ENABLE, 14));
	startBoth();
	CHECK(stored(0, fr(0x100)));
	CHECK_FALSE(stored(0, fr(0x200)));
	CHECK(stored(1, fr(0x200)));
	CHECK_FALSE(stored(1, fr(0x100)));
	CHECK(level(h1, CAN_RX_FIFO0) == 1u);
	CHECK(level(h2, CAN_RX_FIFO0) == 1u);
}

TEST_CASE_FIXTURE(Cf, "CF4 with CAN2SB = 0 no filter belongs to CAN1 and every bank filters CAN2") {
	cfgOk(f32mask(0, 0, 0, 0, CAN_FILTER_ENABLE, 0)); // accepts everything
	startBoth();
	CHECK_FALSE(stored(0, fr(0x100)));
	CHECK(stored(1, fr(0x100)));
	CHECK(level(h1, CAN_RX_FIFO0) == 0u);
	CHECK(level(h2, CAN_RX_FIFO0) == 1u);
}

TEST_CASE_FIXTURE(Cf, "CF4 with CAN2SB = 28 all filter banks belong to CAN1 and CAN2 receives nothing") {
	cfgOk(f32mask(27, 0, 0, 0, CAN_FILTER_ENABLE, 28));
	startBoth();
	CHECK(stored(0, fr(0x100)));
	CHECK_FALSE(stored(1, fr(0x100)));
	CHECK(level(h1, CAN_RX_FIFO0) == 1u);
	CHECK(level(h2, CAN_RX_FIFO0) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF4 banks of CAN2 deliver to the FIFO selected by FFA1R of the CAN2 instance") {
	cfgOk(f32mask(20, CAN_FILTER_FIFO1, 0, 0, CAN_FILTER_ENABLE, 14), &h2);
	startBoth();
	CHECK(stored(1, fr(0x321)));
	CHECK(level(h2, CAN_RX_FIFO1) == 1u);
	CHECK(level(h2, CAN_RX_FIFO0) == 0u);
	CHECK(level(h1, CAN_RX_FIFO1) == 0u);
}

TEST_CASE_FIXTURE(Cf, "CF4 moving CAN2SB re-assigns an existing bank from one instance to the other") {
	cfgOk(f32mask(5, 0, 0, 0, CAN_FILTER_ENABLE, 14)); // bank 5 filters CAN1
	startBoth();
	REQUIRE(stored(0, fr(0x100)));
	REQUIRE_FALSE(stored(1, fr(0x100)));
	cfgOk(f32mask(6, 0, 0, 0, CAN_FILTER_DISABLE, 5)); // bank 5 and above now belong to CAN2
	CHECK_FALSE(stored(0, fr(0x100)));
	CHECK(stored(1, fr(0x100)));
}

TEST_CASE_FIXTURE(Cf, "CF4 a bank of CAN2 does not take part in the numbering of CAN1") {
	// RM0090 Figure 343 shows one instance; the split by CAN2SB gives each instance its own banks.
	cfgOk(f32mask(0, 0, r32(0x100, 0, 0), kM32All, CAN_FILTER_ENABLE, 2));
	cfgOk(f32mask(1, 0, r32(0x101, 0, 0), kM32All, CAN_FILTER_ENABLE, 2));
	cfgOk(f32mask(2, 0, r32(0x200, 0, 0), kM32All, CAN_FILTER_ENABLE, 2));
	startBoth();
	CHECK(rx0(fr(0x101)) == 1);
	CHECK_FALSE(stored(0, fr(0x200)));
}
