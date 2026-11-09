#include "doctest.h"
#include "stm32f4xx_hal.h"

namespace {
struct TickFixture {
    TickFixture() { p12_reset_tick_boundary(); }
    ~TickFixture() { p12_reset_tick_boundary(); }
};
}
TEST_SUITE("P12RealHalTick") {
TEST_CASE_FIXTURE(TickFixture, "real tick increments and reads milliseconds at the default rate") {
    HAL_IncTick(); HAL_IncTick();
    CHECK(HAL_GetTick() == 2);
    CHECK(HAL_GetTickFreq() == HAL_TICK_FREQ_1KHZ);
}
TEST_CASE_FIXTURE(TickFixture, "real tick increment uses the selected ten millisecond quantum") {
    REQUIRE(HAL_InitTick(4) == HAL_OK);
    REQUIRE(HAL_SetTickFreq(HAL_TICK_FREQ_100HZ) == HAL_OK);
    CHECK(p12_tick_observations.reload == 160000);
    HAL_IncTick(); HAL_IncTick();
    CHECK(HAL_GetTick() == 20);
}
TEST_CASE_FIXTURE(TickFixture, "real tick counter wraps using unsigned millisecond arithmetic") {
    uwTick = 0xffffffffU;
    HAL_IncTick();
    CHECK(HAL_GetTick() == 0);
}
TEST_CASE_FIXTURE(TickFixture, "tick initialization requests the reload for one millisecond at sixteen megahertz") {
    REQUIRE(HAL_InitTick(4) == HAL_OK);
    CHECK(p12_tick_observations.reload == 16000);
    CHECK(p12_tick_observations.irq == SysTick_IRQn);
    CHECK(p12_tick_observations.preempt == 4);
    CHECK(p12_tick_observations.sub == 0);
    CHECK(HAL_GetTickPrio() == 4);
}
TEST_CASE_FIXTURE(TickFixture, "ten hertz selection requests a hundred millisecond reload and increment") {
    REQUIRE(HAL_InitTick(4) == HAL_OK);
    REQUIRE(HAL_SetTickFreq(HAL_TICK_FREQ_10HZ) == HAL_OK);
    CHECK(p12_tick_observations.reload == 1600000);
    HAL_IncTick();
    CHECK(HAL_GetTick() == 100);
}
TEST_CASE_FIXTURE(TickFixture, "selecting the active frequency does not reconfigure the lower time source") {
    REQUIRE(HAL_InitTick(4) == HAL_OK);
    const auto calls = p12_tick_observations.configCalls;
    CHECK(HAL_SetTickFreq(HAL_TICK_FREQ_1KHZ) == HAL_OK);
    CHECK(p12_tick_observations.configCalls == calls);
}
TEST_CASE_FIXTURE(TickFixture, "failed time-source configuration leaves the previous frequency selected") {
    REQUIRE(HAL_InitTick(4) == HAL_OK);
    p12_tick_observations.configResult = 1;
    CHECK(HAL_SetTickFreq(HAL_TickFreqTypeDef::HAL_TICK_FREQ_100HZ) == HAL_ERROR);
    CHECK(HAL_GetTickFreq() == HAL_TICK_FREQ_1KHZ);
    HAL_IncTick();
    CHECK(HAL_GetTick() == 1);
}
TEST_CASE_FIXTURE(TickFixture, "failed time-source configuration does not publish a new interrupt priority") {
    REQUIRE(HAL_InitTick(4) == HAL_OK);
    const auto priorities = p12_tick_observations.priorityCalls;
    p12_tick_observations.configResult = 1;
    CHECK(HAL_InitTick(7) == HAL_ERROR);
    CHECK(HAL_GetTickPrio() == 4);
    CHECK(p12_tick_observations.priorityCalls == priorities);
}
TEST_CASE_FIXTURE(TickFixture, "out of range priority is rejected without changing the recorded priority") {
    REQUIRE(HAL_InitTick(4) == HAL_OK);
    const auto priorities = p12_tick_observations.priorityCalls;
    CHECK(HAL_InitTick(16) == HAL_ERROR); // real F407 device header: four priority bits
    CHECK(HAL_GetTickPrio() == 4);
    CHECK(p12_tick_observations.priorityCalls == priorities);
}
TEST_CASE_FIXTURE(TickFixture, "suspending the tick clears only the interrupt request bit") {
    p12_tick_control.CTRL = 7;
    HAL_SuspendTick();
    CHECK(p12_tick_control.CTRL == 5);
}
TEST_CASE_FIXTURE(TickFixture, "resuming the tick preserves other control bits while enabling its interrupt") {
    p12_tick_control.CTRL = 5;
    HAL_ResumeTick();
    CHECK(p12_tick_control.CTRL == 7);
}
}
