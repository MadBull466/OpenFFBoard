#include "TestHelpers.h"
#include "stm32f4xx_hal.h"
#include <initializer_list>

namespace {
struct DelayFixture {
    DelayFixture() { p12_reset_tick_boundary(); }
    ~DelayFixture() { p12_reset_tick_boundary(); }
    void script(std::initializer_list<std::uint32_t> values) {
        p12_set_tick_script(values.begin(), values.size());
    }
};
}
TEST_SUITE("P12RealHalDelay") {
TEST_CASE_FIXTURE(DelayFixture, "scripted tick boundary returns supplied rollover values in order") {
    script({0xffffffffU, 0, 1});
    CHECK(HAL_GetTick() == 0xffffffffU);
    CHECK(HAL_GetTick() == 0);
    CHECK(HAL_GetTick() == 1);
    CHECK(p12_tick_reads() == 3);
    CHECK(p12_elapsed_ticks() == 2);
    CHECK(p12_tick_script_exhausted() == 0);
}
TEST_CASE_FIXTURE(DelayFixture, "scripted tick boundary records exhaustion instead of silently accepting extra reads") {
    script({7});
    CHECK(HAL_GetTick() == 7);
    (void)HAL_GetTick();
    CHECK(p12_tick_script_exhausted() == 1);
}
TEST_CASE_FIXTURE(DelayFixture, "delay does not finish before the requested sampled millisecond interval") {
    script({200, 200, 202, 204, 205, 206});
    HAL_Delay(5);
    CHECK(p12_tick_script_exhausted() == 0);
    CHECK(p12_elapsed_ticks() >= 5);
}
TEST_CASE_FIXTURE(DelayFixture, "delay preserves its minimum interval across counter rollover") {
    script({0xfffffffdU, 0xfffffffeU, 0xffffffffU, 0, 1, 2, 3});
    HAL_Delay(5);
    CHECK(p12_tick_script_exhausted() == 0);
    CHECK(p12_elapsed_ticks() >= 5);
}
TEST_CASE_FIXTURE(DelayFixture, "coarse tick frequency does not let a short delay return before its requested interval") {
    REQUIRE(HAL_InitTick(4) == HAL_OK);
    REQUIRE(HAL_SetTickFreq(HAL_TICK_FREQ_100HZ) == HAL_OK);
    script({500, 500, 510, 520, 530});
    HAL_Delay(15);
    CHECK(p12_tick_script_exhausted() == 0);
    CHECK(p12_elapsed_ticks() >= 15);
}
TEST_CASE_FIXTURE(DelayFixture, "maximum delay does not overflow into an immediate return") {
    script({0, 0, 0xffffffffU});
    HAL_Delay(HAL_MAX_DELAY);
    CHECK(p12_tick_script_exhausted() == 0);
    CHECK(p12_last_tick() == 0xffffffffU);
}
TEST_CASE_FIXTURE(DelayFixture, "large finite delay at the default tick retains its requested minimum interval") {
    script({0, 0, 100, 4294967200U, 4294967201U});
    HAL_Delay(4294967200U);
    CHECK(p12_tick_script_exhausted() == 0);
    CHECK(p12_elapsed_ticks() >= 4294967200ULL);
}
TEST_CASE_FIXTURE(DelayFixture, "large finite delay at ten hertz retains its requested minimum interval"
    * KNOWN_ISSUE("P12-TICK-001: adding the 100ms quantum wraps a near-maximum wait into 4ms")) {
    REQUIRE(HAL_InitTick(4) == HAL_OK);
    REQUIRE(HAL_SetTickFreq(HAL_TICK_FREQ_10HZ) == HAL_OK);
    // Requested 4294967200ms is divisible by 100; every sample is a possible
    // tick value at this frequency. No time actually elapses on the host.
    script({0, 0, 100, 4294967200U, 4});
    HAL_Delay(4294967200U);
    // The verifier requires this exact first failure and no model fault.
    CHECK(p12_elapsed_ticks() >= 4294967200ULL);
    CHECK(p12_tick_script_exhausted() == 0);
}
}
