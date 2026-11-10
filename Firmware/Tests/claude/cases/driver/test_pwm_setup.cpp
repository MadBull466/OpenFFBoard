/*
 * PWM driver, phase P14 T1 skeleton: MotorPWM builds into ffb_driver_tests (PWMDRIVER on in this executable only) and the
 * fake TIM1 behaves like the HAL. Charter: Tests/claude/charters/driver/PWM_REQUIREMENTS.md (R1.4, R2.1 and the fake
 * "Fake: TIM1 PWM" section); the full test groups follow in T2.
 *
 * Oracles: HAL (stm32f4xx_hal_tim.c, the copy in Targets/F411RE/Drivers) for the call contracts of the fake, MX_TIM1_Init
 * (Targets/F407VG/Core/Src/main.c:716-721) for the initial state, HD (hand derivations in the comments).
 */
#include "doctest.h"
#include "PwmFixture.h"

#include "constants.h"

TEST_SUITE("PWM setup") {

TEST_CASE_FIXTURE(PwmFixture, "the fake TIM1 starts as MX_TIM1_Init leaves it: prescaler 0, period 0xffff, no output enabled") {
	// MX_TIM1_Init: Prescaler 0, Period 0xffff, ARPE enabled (main.c:716-721); HAL_TIM_PWM_Init sets the channels READY
	CHECK(htim1.Instance == &tim());
	CHECK(psc() == 0);
	CHECK(arr() == 0xffff);
	CHECK(htim1.Init.AutoReloadPreload != 0);
	for (int ch = 1; ch <= 4; ++ch) {
		CHECK_FALSE(channelOn(ch));
		CHECK(ccr(ch) == 0);
	}
	CHECK_FALSE(mainOutput());
	CHECK_FALSE(counterOn());
}

TEST_CASE_FIXTURE(PwmFixture, "the fake HAL_TIM_PWM_Start enables channel, main output and counter and rejects a second start of the same channel") {
	TIM_OC_InitTypeDef oc = {};
	oc.OCMode = TIM_OCMODE_PWM1;
	oc.Pulse = 1234;
	REQUIRE(HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2) == HAL_OK);
	CHECK(ccr(2) == 1234); // ConfigChannel loads CCR2 from Pulse
	CHECK((tim().CCMR1 & (0xFFU << 8)) != 0); // OC2M, OC2PE set
	REQUIRE(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) == HAL_OK);
	CHECK(channelOn(2));
	CHECK(mainOutput()); // TIM1 is a break instance: __HAL_TIM_MOE_ENABLE
	CHECK(counterOn());
	// the real HAL returns HAL_ERROR for a channel that is not READY (stm32f4xx_hal_tim.c: HAL_TIM_PWM_Start)
	CHECK(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) == HAL_ERROR);
	CHECK(HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2) == HAL_OK);
	CHECK(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) == HAL_OK); // READY again after Stop
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}

TEST_CASE_FIXTURE(PwmFixture, "the fake HAL_TIM_PWM_Stop clears main output and counter only when the last channel stops") {
	TIM_OC_InitTypeDef oc = {};
	oc.OCMode = TIM_OCMODE_PWM1;
	for (uint32_t c : {TIM_CHANNEL_1, TIM_CHANNEL_3}) {
		REQUIRE(HAL_TIM_PWM_ConfigChannel(&htim1, &oc, c) == HAL_OK);
		REQUIRE(HAL_TIM_PWM_Start(&htim1, c) == HAL_OK);
	}
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	CHECK_FALSE(channelOn(1));
	CHECK(channelOn(3));
	CHECK(mainOutput()); // __HAL_TIM_MOE_DISABLE only when no CCxE / CCxNE remains
	CHECK(counterOn());
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	CHECK_FALSE(mainOutput());
	CHECK_FALSE(counterOn());
}

TEST_CASE_FIXTURE(PwmFixture, "the fake rejects a configuration the HAL asserts against and records it for the fixture check") {
	TIM_OC_InitTypeDef oc = {};
	oc.OCMode = 0x10; // not PWM1 / PWM2 (assert_param IS_TIM_PWM_MODE)
	CHECK(HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) == HAL_ERROR);
	oc.OCMode = TIM_OCMODE_PWM1;
	CHECK(HAL_TIM_PWM_ConfigChannel(&htim1, &oc, 0x10U) == HAL_ERROR); // not TIM_CHANNEL_1..4
	CHECK(HostTarget::state().unmodelledHalCalls.size() == 2);
	HostTarget::state().unmodelledHalCalls.clear(); // declared: the fixture check would otherwise fail the test
}

TEST_CASE_FIXTURE(PwmFixture, "the MotorPWM is constructed on the fake TIM1 with the default timer set-up of 3 kHz centred PWM") {
	auto pwm = makePwm();
	REQUIRE(pwm != nullptr);
	// default mode CENTERED, speed LOW on an empty flash: ARR = 168000000 / 3000 = 56000, PSC = 0 (R2.1, R1.4)
	CHECK(arr() == 56000);
	CHECK(psc() == 0);
	CHECK(pwm->getMode() == ModePWM_DRV::CENTERED_PWM);
	CHECK(pwm->getPwmSpeed() == SpeedPWM_DRV::LOW);
	for (int ch = 1; ch <= 4; ++ch) {
		CHECK(channelOn(ch));
	}
	CHECK(mainOutput());
	CHECK(counterOn());
	CHECK(hosttestTim1MspPostInitCalls >= 1);
	CHECK(MotorPWM::pwmDriverInUse);
	CHECK_FALSE(MotorPWM::isCreatable());
	pwm.reset();
	CHECK_FALSE(MotorPWM::pwmDriverInUse);
	CHECK(MotorPWM::isCreatable());
	for (int ch = 1; ch <= 4; ++ch) {
		CHECK_FALSE(channelOn(ch)); // destructor stops all four channels
	}
	CHECK_FALSE(mainOutput());
	CHECK_FALSE(counterOn());
}

}
