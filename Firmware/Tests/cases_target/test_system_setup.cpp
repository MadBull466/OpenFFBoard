/*
 * Smoke tests of the P4 batch 0 setup of ffb_target_tests: the real cppmain.cpp, global_callbacks.cpp,
 * voltagesense.cpp and ledEffects.cpp on the fake HAL ADC, TIM7 HAL tick, IWDG and CAN receive FIFO
 * (platform/src/host_target_system.cpp). Detailed tests of these modules are in their own files.
 */
#include "doctest.h"
#include "AxisFixture.h"

#include "CanHandler.h"
#include "SystemCommands.h"
#include "TimerHandler.h"
#include "UartHandler.h"
#include "cpp_target_config.h"
#include "cppmain.h"
#include "eeprom_addresses.h"
#include "global_callbacks.h"
#include "ledEffects.h"
#include "voltagesense.h"

#include <cstring>
#include <vector>

extern Ledstruct_t sysled; // ledEffects.cpp

namespace {

/** VREFINT raw value equal to the calibration value: the analog reference is exactly VREFINT_CAL_VREF = 3300 mV */
constexpr uint32_t VREFINT_RAW = 1500;

struct SystemFixture : TargetFixture {
	SystemFixture() {
		HostTarget::setAdcCalibration(VREFINT_RAW, 950, 1200);
		startADC(); // global_callbacks.cpp, called by cppmain() on the target
	}

	/** One conversion of ADC1 with VREFINT at its calibration value and the given chip temperature sensor value */
	void convertAdc1(uint32_t tempSensorRaw = 950) {
		HostTarget::adcSetInput(&hadc1, 6, tempSensorRaw);
		HostTarget::adcSetInput(&hadc1, 7, VREFINT_RAW);
		REQUIRE(HostTarget::adcConversionComplete(&hadc1));
	}

	/** Executes a get command of SystemCommands and returns the value of its single reply */
	int64_t sysGet(SystemCommands &sys, FFBoardMain_commands cmd) {
		ParsedCommand parsed;
		parsed.cmdId = static_cast<uint32_t>(cmd);
		parsed.type = CMDtype::get;
		parsed.target = &sys;
		std::vector<CommandReply> replies;
		CHECK(sys.internalCommand(parsed, replies) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies.front().val;
	}
};

struct CountingTimerHandler : TimerHandler {
	void timerElapsed(TIM_HandleTypeDef *htim) override { elapsed.push_back(htim); }
	std::vector<TIM_HandleTypeDef *> elapsed;
};

struct CountingUartHandler : UartHandler {
	void uartRxComplete(UART_HandleTypeDef *huart) override { rx.push_back(huart); }
	std::vector<UART_HandleTypeDef *> rx;
};

struct RecordingCanHandler : CanHandler {
	void canRxPendCallback(CANPort *port, CAN_rx_msg &msg) override {
		ports.push_back(port);
		msgs.push_back(msg);
	}
	std::vector<CANPort *> ports;
	std::vector<CAN_rx_msg> msgs;
};

} // namespace

TEST_SUITE("SystemSetup") {

TEST_CASE_FIXTURE(SystemFixture, "an ADC2 conversion complete interrupt updates the internal and external voltage of voltagesense") {
	convertAdc1();
	// VEXT 1241 and VINT 2482 of 4095 at 3300 mV: 1000 mV and 2000 mV at the pins, times VOLTAGE_MULT_DEFAULT 30.67
	CHECK(HostTarget::adcConvert(&hadc2, {1241, 2482}));

	CHECK(ADC2_BUF[ADC_CHAN_VEXT] == 1241);
	CHECK(getExtV() == 30670);
	CHECK(getIntV() == 61340);
}

TEST_CASE_FIXTURE(SystemFixture, "SystemCommands answers the vint, vext and temp commands from the converted ADC values") {
	SystemCommands sys;
	convertAdc1(1075); // TS_CAL1 950 (30 C) .. TS_CAL2 1200 (110 C): 1075 is 70 C
	HostTarget::adcConvert(&hadc2, {1241, 2482});

	CHECK(sysGet(sys, FFBoardMain_commands::vext) == 30670);
	CHECK(sysGet(sys, FFBoardMain_commands::vint) == 61340);
	CHECK(sysGet(sys, FFBoardMain_commands::temp) == 70);
}

TEST_CASE_FIXTURE(SystemFixture, "the brake pin is switched on while the internal voltage exceeds the supply by more than 5 V") {
	convertAdc1();
	HostTarget::adcConvert(&hadc2, {1241, 1489}); // VEXT 30670 mV, VINT 36773 mV: 6.1 V above the supply
	CHECK(HostTarget::outputPin(DRV_BRAKE_GPIO_Port, DRV_BRAKE_Pin));

	HostTarget::adcConvert(&hadc2, {1241, 1241}); // equal again
	CHECK_FALSE(HostTarget::outputPin(DRV_BRAKE_GPIO_Port, DRV_BRAKE_Pin));
}

TEST_CASE_FIXTURE(TargetFixture, "micros follows the simulated time across a HAL tick") {
	HostPlatform::setTimeUs(4999);
	CHECK(micros() == 4999);
	CHECK(HostTarget::tickTimerCounter() == 999);

	HostPlatform::advanceUs(2);

	CHECK(HAL_GetTick() == 5);
	CHECK(HostTarget::tickTimerCounter() == 1);
	CHECK(micros() == 5001);
}

TEST_CASE_FIXTURE(TargetFixture, "a TIM7 update increments the HAL tick and only other timers reach the timer handlers") {
	CountingTimerHandler handler;
	REQUIRE(HAL_TIM_Base_Start_IT(&htim13) == HAL_OK);

	CHECK(HostTarget::timerUpdate(&htim7));
	CHECK(HostTarget::state().halIncTicks == 1);
	CHECK(handler.elapsed.empty());

	CHECK(HostTarget::timerUpdate(&htim13));
	CHECK(HostTarget::state().halIncTicks == 1);
	REQUIRE(handler.elapsed.size() == 1);
	CHECK(handler.elapsed[0] == &htim13);
}

TEST_CASE_FIXTURE(TargetFixture, "a sys LED pulse ends after its 25 ms period and the next update after that switches the LED off") {
	pulseSysLed();
	CHECK(HostTarget::outputPin(LED_SYS_GPIO_Port, LED_SYS_Pin)); // blinkLed: on (state 0)

	HostPlatform::advanceMs(25);
	updateLeds();
	CHECK(HostTarget::outputPin(LED_SYS_GPIO_Port, LED_SYS_Pin)); // HAL_GetTick() > tick + period is not yet true

	HostPlatform::advanceMs(1);
	updateLeds();
	CHECK_FALSE(HostTarget::outputPin(LED_SYS_GPIO_Port, LED_SYS_Pin)); // toggled off, last blink step
	CHECK(sysled.period == 25);

	HostPlatform::advanceMs(26);
	updateLeds();
	CHECK_FALSE(HostTarget::outputPin(LED_SYS_GPIO_Port, LED_SYS_Pin));
	CHECK(sysled.period == 0); // effect finished: a new pulse can start
}

TEST_CASE_FIXTURE(TargetFixture, "a UART receive complete interrupt reaches the registered UART handlers") {
	CountingUartHandler handler;
	{
		HostPlatform::IsrContext isr;
		HAL_UART_RxCpltCallback(&huart1);
	}
	REQUIRE(handler.rx.size() == 1);
	CHECK(handler.rx[0] == &huart1);
}

TEST_CASE_FIXTURE(TargetFixture, "a frame in CAN receive FIFO 0 reaches the CAN handlers with the global CAN port") {
	RecordingCanHandler handler;
	CAN_RxHeaderTypeDef header = {};
	header.StdId = 0x123;
	header.IDE = CAN_ID_STD;
	header.DLC = 3;
	const uint8_t data[8] = {1, 2, 3, 0, 0, 0, 0, 0};
	HostTarget::canQueueRx(CAN_RX_FIFO0, header, data);

	CHECK(HostTarget::canRxInterrupt(CAN_RX_FIFO0) == 1);

	REQUIRE(handler.msgs.size() == 1);
	CHECK(handler.ports[0] == &canport);
	CHECK(handler.msgs[0].header.id == 0x123);
	CHECK(handler.msgs[0].header.length == 3);
	CHECK_FALSE(handler.msgs[0].header.extId);
	CHECK(handler.msgs[0].fifo == 0);
	CHECK(std::memcmp(handler.msgs[0].data, data, 3) == 0);
	CHECK(HostTarget::canRxPending(CAN_RX_FIFO0) == 0);
}

TEST_CASE_FIXTURE(TargetFixture, "cppmain with BUTTON_A pressed starts both ADC DMA transfers, creates the failsafe main class and refreshes the watchdog in every loop pass") {
	HostTarget::setPin(BUTTON_A_GPIO_Port, BUTTON_A_Pin, true);

	HostTarget::runCppmain(3);

	CHECK(HostTarget::adcDmaBuffer(&hadc1) == const_cast<uint32_t *>(ADC1_BUF));
	CHECK(HostTarget::adcDmaLength(&hadc1) == ADC1_CHANNELS);
	CHECK(HostTarget::adcDmaBuffer(&hadc2) == const_cast<uint32_t *>(ADC2_BUF));
	REQUIRE(mainclass != nullptr);
	CHECK(mainclass == HostTarget::cppmainMainclass());
	CHECK(mainclass->getInfo().id == 0);
	CHECK(usb_device != nullptr); // FFBoardMain::usbInit registered the USB device
	CHECK(HostTarget::state().watchdogRefreshes == 3);
	CHECK(HAL_GetTick() == 3); // FFBoardMain::update waits 1 ms per pass
	CHECK(HostTarget::state().errorHandlerCalls == 0);
}

TEST_CASE_FIXTURE(TargetFixture, "characterization: cppmain on an empty flash writes the default main class id and calls Error_Handler once (NEEDS REVIEW 53)" * DELETES_FFBHIDMAIN) {
	// Flash_ReadWriteDefault returns false after writing the default (also the real flash_helpers.cpp:105-120), and
	// cppmain.cpp:88-90 treats that as an error: main.c's Error_Handler only switches the error LED off and returns.
	HostTarget::runCppmain(0);

	CHECK(HostPlatform::flash().at(ADR_CURRENT_CONFIG) == DEFAULTMAIN);
	CHECK(HostPlatform::flash().at(ADR_FLASH_VERSION) == FLASH_VERSION);
	REQUIRE(mainclass != nullptr);
	CHECK(mainclass->getInfo().id == DEFAULTMAIN);
	CHECK(HostTarget::state().errorHandlerCalls == 1);
}

} // TEST_SUITE
