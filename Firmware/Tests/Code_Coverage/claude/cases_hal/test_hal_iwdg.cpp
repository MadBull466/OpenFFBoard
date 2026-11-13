// P12 T7 group IW: independent watchdog through the real HAL (HAL_IWDG_Init / HAL_IWDG_Refresh).
// Oracles: UM1725 Rev 8 "HAL IWDG Generic Driver"; RM0090 Rev 22 chapter 21 (KR 0xAAAA reload / 0x5555 write access / 0xCCCC start,
// PR codes 000..110 = /4../256, RLR 12 bit reset 0xFFF, SR PVU bit 0 / RVU bit 1, write access to PR / RLR only after 0x5555,
// PR / RLR updates only while PVU / RVU = 0, a counter that reaches 0 resets the device).
// Cases named "model: ..." exercise only the register model (HalIwdg.c): they are not vendor coverage.
// Not testable: the charter row "NULL handle of Refresh": HAL_IWDG_Refresh dereferences hiwdg without a check and UM1725 gives no NULL rule.
#include "FlashTestSupport.h"

namespace {

IWDG_HandleTypeDef iwMake(uint32_t prescaler, uint32_t reload) {
	IWDG_HandleTypeDef h{};
	h.Instance = IWDG;
	h.Init.Prescaler = prescaler;
	h.Init.Reload = reload;
	return h;
}

struct IwPrescaler {
	uint32_t hal;
	uint32_t pr; // RM0090 21.4.2 table: PR[2:0]
};

const IwPrescaler iwPrescalers[] = {
	{IWDG_PRESCALER_4, 0u},  {IWDG_PRESCALER_8, 1u},   {IWDG_PRESCALER_16, 2u}, {IWDG_PRESCALER_32, 3u},
	{IWDG_PRESCALER_64, 4u}, {IWDG_PRESCALER_128, 5u}, {IWDG_PRESCALER_256, 6u},
};

} // namespace

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init with a NULL handle returns HAL_ERROR and touches no register") {
	CHECK(HAL_IWDG_Init(nullptr) == HAL_ERROR);
	CHECK(hostIwdgRunning() == 0);
	CHECK(hostIwdgReloads() == 0u);
	CHECK(IWDG->PR == 0u);
	CHECK(IWDG->RLR == 0xFFFu);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init returns HAL_OK and starts the watchdog") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_16, 500u);
	CHECK(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(hostIwdgRunning() == 1);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init writes the PR code of every prescaler (4 to 256)") {
	for (const IwPrescaler &p : iwPrescalers) {
		CAPTURE(p.hal);
		IWDG_HandleTypeDef h = iwMake(p.hal, 100u);
		REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
		CHECK(IWDG->PR == p.pr);
	}
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init writes the reload value 0 to RLR") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_4, 0u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(IWDG->RLR == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init writes the reload value 1 to RLR") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_4, 1u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(IWDG->RLR == 1u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init writes the maximum reload value 0xFFF to RLR") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_256, 0xFFFu);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(IWDG->RLR == 0xFFFu);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init writes an intermediate reload value to RLR") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_32, 0x2A5u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(IWDG->RLR == 0x2A5u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init leaves the counter at the reload value") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(hostIwdgCounter() == 100u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init ends with a reload of the counter (the 0xAAAA key)") {
	// UM1725: "Before exiting function, watchdog is refreshed in order to have correct time base".
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(hostIwdgReloads() == 1u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init clears both update flags before it returns") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK((IWDG->SR & 3u) == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init succeeds when the update flags clear only at the third SR read") {
	hostIwdgSetPollReads(3);
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_64, 200u);
	CHECK(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(IWDG->PR == 4u);
	CHECK(IWDG->RLR == 200u);
	CHECK(hostIwdgReloads() == 1u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init succeeds when the update flags clear at the first SR read") {
	hostIwdgSetPollReads(1);
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_4, 10u);
	CHECK(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK((IWDG->SR & 3u) == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init returns HAL_TIMEOUT when the update flags never clear") {
	hostIwdgSetPollReads(0);
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	CHECK(HAL_IWDG_Init(&h) == HAL_TIMEOUT);
	CHECK(HalFixture::faults() == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init waits at least the update time of 6 LSI periods at prescaler 256 before the timeout") {
	// UM1725 / HAL comment: the status register needs up to 5 LSI clock periods divided by the prescaler; rounded to 6.
	// Hand derivation: 6 * 256 / 32000 Hz = 48 ms (LSI_VALUE 32000 Hz of the target), so the timeout may not elapse before 48 ticks.
	// The upper bound 53 has no document source (UM1725 does not define HAL_IWDG_DEFAULT_TIMEOUT): it is a loose sanity limit (48 + startup time + margin).
	hostIwdgSetPollReads(0);
	hostTickSet(0x10000u);
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_256, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_TIMEOUT);
	const uint32_t elapsed = hostTickPeek() - 0x10000u;
	CHECK(elapsed >= 48u);
	CHECK(elapsed <= 53u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init timeout leaves the watchdog started but not reloaded after the start") {
	// 0xCCCC started it (counter loaded with the RLR reset value 0xFFF, RM0090 21.4.3); the final 0xAAAA is not written on a timeout.
	hostIwdgSetPollReads(0);
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_TIMEOUT);
	CHECK(hostIwdgRunning() == 1);
	CHECK(hostIwdgReloads() == 0u);
	CHECK(hostIwdgCounter() == 0xFFFu);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init timeout keeps the update flags set") {
	hostIwdgSetPollReads(0);
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_TIMEOUT);
	CHECK((IWDG->SR & 3u) == 3u); // PVU and RVU both set by the two writes
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init disables the write access to PR and RLR when it returns") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	const uint32_t ignored = hostIwdgIgnoredWrites();
	IWDG->PR = 5u;
	IWDG->RLR = 7u;
	CHECK(IWDG->PR == 1u);
	CHECK(IWDG->RLR == 100u);
	CHECK(hostIwdgIgnoredWrites() == ignored + 2u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init reads the key register as 0 (write only)") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(IWDG->KR == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init a second time reprograms PR and RLR and reloads the counter") {
	IWDG_HandleTypeDef h1 = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h1) == HAL_OK);
	hostIwdgAdvance(30);
	IWDG_HandleTypeDef h2 = iwMake(IWDG_PRESCALER_128, 300u);
	REQUIRE(HAL_IWDG_Init(&h2) == HAL_OK);
	CHECK(IWDG->PR == 5u);
	CHECK(IWDG->RLR == 300u);
	CHECK(hostIwdgCounter() == 300u);
	CHECK(hostIwdgReloads() == 2u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init with a reload of 0 gives counter 0 that is not yet a reset request") {
	// RM0090 21.3: the reset happens when the counter reaches 0 while counting down; loading 0 is not counting down.
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_4, 0u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(hostIwdgCounter() == 0u);
	CHECK(hostIwdgResetRequested() == 0);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW1 the watchdog is not running before Init") {
	CHECK(hostIwdgRunning() == 0);
	CHECK(IWDG->PR == 0u);
	CHECK(IWDG->RLR == 0xFFFu); // RM0090 21.4.3 reset value
}

TEST_CASE_FIXTURE(FlashFixture, "IW2 Refresh returns HAL_OK") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(HAL_IWDG_Refresh(&h) == HAL_OK);
}

TEST_CASE_FIXTURE(FlashFixture, "IW2 Refresh reloads the counter from RLR") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	hostIwdgAdvance(40);
	REQUIRE(hostIwdgCounter() == 60u);
	CHECK(HAL_IWDG_Refresh(&h) == HAL_OK);
	CHECK(hostIwdgCounter() == 100u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW2 Refresh writes exactly one 0xAAAA key per call") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	const uint32_t before = hostIwdgReloads();
	HAL_IWDG_Refresh(&h);
	CHECK(hostIwdgReloads() == before + 1u);
	HAL_IWDG_Refresh(&h);
	HAL_IWDG_Refresh(&h);
	CHECK(hostIwdgReloads() == before + 3u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW2 Refresh leaves PR and RLR unchanged") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_32, 0x123u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	HAL_IWDG_Refresh(&h);
	CHECK(IWDG->PR == 3u);
	CHECK(IWDG->RLR == 0x123u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW2 Refresh does not stop the watchdog") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	HAL_IWDG_Refresh(&h);
	CHECK(hostIwdgRunning() == 1);
}

TEST_CASE_FIXTURE(FlashFixture, "IW2 a write access enabled by hand is disabled again by Refresh") {
	// RM0090 21.4.1: writing 0xAAAA reloads the counter; write access is protected again (any key other than 0x5555 disables it).
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	IWDG->KR = 0x5555u;
	HAL_IWDG_Refresh(&h);
	const uint32_t ignored = hostIwdgIgnoredWrites();
	IWDG->PR = 6u;
	CHECK(IWDG->PR == 1u);
	CHECK(hostIwdgIgnoredWrites() == ignored + 1u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW2 Refresh after Init does not request a reset when the counter never reaches 0") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 10u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	for (int i = 0; i < 5; ++i) {
		hostIwdgAdvance(9u);
		HAL_IWDG_Refresh(&h);
	}
	CHECK(hostIwdgResetRequested() == 0);
	CHECK(hostIwdgCounter() == 10u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW2 the counter one tick before expiry requests no reset") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 10u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(hostIwdgAdvance(9u) == 1u);
	CHECK(hostIwdgResetRequested() == 0);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW2 a counter that counts down to 0 without Refresh requests a reset") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 10u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	CHECK(hostIwdgAdvance(10u) == 0u);
	CHECK(hostIwdgResetRequested() == 1);
}

TEST_CASE_FIXTURE(FlashFixture, "IW2 a Refresh just before expiry prevents the reset that would have followed") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 10u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_OK);
	hostIwdgAdvance(9u);
	HAL_IWDG_Refresh(&h);
	hostIwdgAdvance(9u);
	CHECK(hostIwdgResetRequested() == 0);
	hostIwdgAdvance(1u);
	CHECK(hostIwdgResetRequested() == 1);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW2 a watchdog that was never started does not request a reset") {
	CHECK(hostIwdgAdvance(5000u) == 0u);
	CHECK(hostIwdgResetRequested() == 0);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 PR and RLR writes without the 0x5555 key are ignored") {
	IWDG->PR = 5u;
	IWDG->RLR = 0x10u;
	CHECK(IWDG->PR == 0u);
	CHECK(IWDG->RLR == 0xFFFu);
	CHECK(hostIwdgIgnoredWrites() == 2u);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 PR and RLR writes after the 0x5555 key are accepted") {
	IWDG->KR = 0x5555u;
	IWDG->PR = 5u;
	hostIwdgComplete();
	IWDG->RLR = 0x10u;
	CHECK(IWDG->PR == 5u);
	CHECK(IWDG->RLR == 0x10u);
	CHECK(hostIwdgIgnoredWrites() == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 a key other than 0x5555 after 0x5555 protects the registers again") {
	IWDG->KR = 0x5555u;
	IWDG->KR = 0x1234u;
	IWDG->PR = 5u;
	CHECK(IWDG->PR == 0u);
	CHECK(hostIwdgIgnoredWrites() == 1u);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 the key 0xAAAA reloads the counter from RLR and protects the registers") {
	IWDG->KR = 0x5555u;
	IWDG->RLR = 0x40u;
	hostIwdgComplete();
	IWDG->KR = 0xAAAAu;
	CHECK(hostIwdgCounter() == 0x40u);
	IWDG->RLR = 0x20u;
	CHECK(IWDG->RLR == 0x40u);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 the key 0xCCCC starts the watchdog with the counter loaded from RLR") {
	IWDG->KR = 0xCCCCu;
	CHECK(hostIwdgRunning() == 1);
	CHECK(hostIwdgCounter() == 0xFFFu); // RLR reset value
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 a PR write sets PVU (bit 0) and not RVU") {
	hostIwdgSetPollReads(0);
	IWDG->KR = 0x5555u;
	IWDG->PR = 3u;
	CHECK((IWDG->SR & 3u) == 1u);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 an RLR write sets RVU (bit 1) and not PVU") {
	hostIwdgSetPollReads(0);
	IWDG->KR = 0x5555u;
	IWDG->RLR = 0x80u;
	CHECK((IWDG->SR & 3u) == 2u);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 PR cannot be updated while PVU is set") {
	// RM0090 21.4.3: PR can be updated only when PVU = 0.
	hostIwdgSetPollReads(0);
	IWDG->KR = 0x5555u;
	IWDG->PR = 3u;
	IWDG->PR = 6u;
	CHECK(IWDG->PR == 3u);
	CHECK(hostIwdgIgnoredWrites() == 1u);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 RLR cannot be updated while RVU is set") {
	hostIwdgSetPollReads(0);
	IWDG->KR = 0x5555u;
	IWDG->RLR = 0x80u;
	IWDG->RLR = 0x90u;
	CHECK(IWDG->RLR == 0x80u);
	CHECK(hostIwdgIgnoredWrites() == 1u);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 the update flags clear and PR accepts a new value after the update completes") {
	hostIwdgSetPollReads(0);
	IWDG->KR = 0x5555u;
	IWDG->PR = 3u;
	hostIwdgComplete();
	CHECK((IWDG->SR & 3u) == 0u);
	IWDG->PR = 6u;
	CHECK(IWDG->PR == 6u);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 the status register is read only") {
	IWDG->SR = 3u;
	CHECK(IWDG->SR == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "model: IW3 PR keeps only its 3 bits and RLR its 12 bits") {
	IWDG->KR = 0x5555u;
	IWDG->PR = 0xFFFFFFFFu;
	hostIwdgComplete();
	IWDG->RLR = 0xFFFFFFFFu;
	CHECK(IWDG->PR == 7u);
	CHECK(IWDG->RLR == 0xFFFu);
}

TEST_CASE_FIXTURE(FlashFixture, "IW3 Init after a refused PR update (PVU stuck) times out and the old PR value stays") {
	// A first update never completes; the second Init finds PVU / RVU set, its writes are ignored, and it times out.
	hostIwdgSetPollReads(0);
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	REQUIRE(HAL_IWDG_Init(&h) == HAL_TIMEOUT);
	IWDG_HandleTypeDef h2 = iwMake(IWDG_PRESCALER_256, 200u);
	CHECK(HAL_IWDG_Init(&h2) == HAL_TIMEOUT);
	CHECK(IWDG->PR == 1u);
	CHECK(IWDG->RLR == 100u);
	CHECK(hostIwdgIgnoredWrites() == 2u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW3 no register access of the IWDG HAL is a model fault") {
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	HAL_IWDG_Init(&h);
	HAL_IWDG_Refresh(&h);
	CHECK(HalFixture::faults() == 0u);
}

namespace {
void iwStepKeepPvu() {
	HAL_IncTick();
	hostRegsPoke(0x4000300Cu, 1u);
}
void iwStepKeepRvu() {
	HAL_IncTick();
	hostRegsPoke(0x4000300Cu, 2u);
}
} // namespace

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init times out when only PVU stays set") {
	// The tick hook (replaces the default step) keeps SR = PVU while the HAL waits; RVU is clear.
	hostIwdgSetPollReads(0);
	hostTickSetStep(iwStepKeepPvu);
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	const HAL_StatusTypeDef r = HAL_IWDG_Init(&h);
	hostTickSetStep(nullptr);
	CHECK(r == HAL_TIMEOUT);
	CHECK(hostIwdgReloads() == 0u);
}

TEST_CASE_FIXTURE(FlashFixture, "IW1 Init times out when only RVU stays set") {
	hostIwdgSetPollReads(0);
	hostTickSetStep(iwStepKeepRvu);
	IWDG_HandleTypeDef h = iwMake(IWDG_PRESCALER_8, 100u);
	const HAL_StatusTypeDef r = HAL_IWDG_Init(&h);
	hostTickSetStep(nullptr);
	CHECK(r == HAL_TIMEOUT);
	CHECK(hostIwdgReloads() == 0u);
}
