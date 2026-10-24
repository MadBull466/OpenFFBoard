/*
 * Tests of global_callbacks.cpp: the HAL interrupt callbacks (ADC, TIM, EXTI, UART, CAN, SPI, I2C) dispatched to the
 * registered handler classes, the analog buffers (getAnalogBuffer, startADC) and the TinyUSB device callbacks
 * (descriptors, CDC, HID, mount/suspend).
 *
 * Interrupt callbacks are raised through the fake HAL where it models the interrupt (HostTarget::adcConversionComplete,
 * timerUpdate, triggerExti, canRxInterrupt) and called directly with the handle otherwise (the CAN transmit/error
 * interrupts, SPI and I2C: their HAL is not modelled yet), in simulated interrupt context like on the target.
 * Handles that belong to no port are local objects, so the global port objects (which are handlers themselves and
 * compare the handle) ignore them.
 *
 * Not in global_callbacks.cpp, so not tested here: a UART error callback (see the P4 report of group B).
 * HAL_TIM_PeriodElapsedCallback of global_callbacks.cpp is __weak and replaced by main.c's (modelled in
 * host_target_system.cpp, as on the target): only HAL_TIM_PeriodElapsedCallback_CPP is reachable.
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "MockCommandHandler.h"
#include "TestHelpers.h"

#include "AdcHandler.h"
#include "CAN.h"
#include "CanHandler.h"
#include "ExtiHandler.h"
#include "FFBoardMainCommandThread.h"
#include "HidCommandInterface.h"
#include "I2CHandler.h"
#include "SpiHandler.h"
#include "TimerHandler.h"
#include "UartHandler.h"
#include "UsbHidHandler.h"
#include "USBdevice.h"
#include "cpp_target_config.h"
#include "global_callbacks.h"
#include "tusb.h"
#include "usb_descriptors.h"

#include <cstring>
#include <string>
#include <utility>
#include <vector>

/**
 * Model of TinyUSB's weak default of the suspend callback (USB/device/usbd.c:77, declared in usbd.h:162 as
 * tud_suspend_cb(bool)). TinyUSB is not compiled into ffb_target_tests; this weak definition stands in for the default
 * that the firmware is supposed to replace. A firmware definition with the TinyUSB signature overrides it.
 */
extern "C" __attribute__((weak)) void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }

/** The firmware's suspend callback (global_callbacks.cpp:357): an overload with C++ linkage, declared by no header */
void tud_suspend_cb();
/**
 * The firmware's HID transfer complete callback (global_callbacks.cpp:340) takes `uint8_t len`, TinyUSB declares and
 * calls tud_hid_report_complete_cb(uint8_t, uint8_t const*, uint16_t) (hid_device.h:146, hid_device.c:389): the
 * firmware function is a C++ overload that TinyUSB never calls. Without effect today (both transferComplete
 * implementations are empty). Called directly here.
 */
void tud_hid_report_complete_cb(uint8_t itf, uint8_t const *report, uint8_t len);

namespace {

// HAL_CAN_ERROR_BOF / _ACK of stm32f4xx_hal_can.h (not in the fake main.h)
constexpr uint32_t CAN_ERROR_BOF = 0x00000004U;
constexpr uint32_t CAN_ERROR_ACK = 0x00000020U;

/** Call log shared by several handlers: (handler name, event) in call order */
using CallLog = std::vector<std::string>;

/** Saves a handler list, empties it for the test and restores it at the end (keeps the global ports out) */
template <class H> struct IsolatedHandlers {
	explicit IsolatedHandlers(std::vector<H *> &list) : list(list), saved(list) { list.clear(); }
	~IsolatedHandlers() { list = saved; }
	std::vector<H *> &list;
	std::vector<H *> saved;
};

struct RecordingAdcHandler : AdcHandler {
	struct Call {
		volatile uint32_t *buf;
		uint8_t chans;
		ADC_HandleTypeDef *hadc;
		std::vector<uint32_t> values;
		bool brakeOn; // DRV_BRAKE output while the handler ran
	};
	void adcUpd(volatile uint32_t *buf, uint8_t chans, ADC_HandleTypeDef *hadc) override {
		Call c{buf, chans, hadc, {}, HostTarget::outputPin(DRV_BRAKE_GPIO_Port, DRV_BRAKE_Pin)};
		for (uint8_t i = 0; i < chans; i++) {
			c.values.push_back(static_cast<uint32_t>(buf[i]));
		}
		calls.push_back(c);
	}
	std::vector<Call> calls;
};

struct RecordingTimerHandler : TimerHandler {
	RecordingTimerHandler(CallLog &log, std::string name) : log(log), name(std::move(name)) {}
	void timerElapsed(TIM_HandleTypeDef *htim) override {
		handles.push_back(htim);
		log.push_back(name);
	}
	CallLog &log;
	std::string name;
	std::vector<TIM_HandleTypeDef *> handles;
};

struct RecordingExtiHandler : ExtiHandler {
	RecordingExtiHandler(CallLog &log, std::string name) : log(log), name(std::move(name)) {}
	void exti(uint16_t pin) override {
		pins.push_back(pin);
		log.push_back(name);
	}
	CallLog &log;
	std::string name;
	std::vector<uint16_t> pins;
};

struct RecordingUartHandler : UartHandler {
	RecordingUartHandler(CallLog &log, std::string name) : log(log), name(std::move(name)) {}
	void uartRxComplete(UART_HandleTypeDef *huart) override {
		calls.emplace_back("rx", huart);
		log.push_back(name + " rx");
	}
	void uartTxComplete(UART_HandleTypeDef *huart) override {
		calls.emplace_back("tx", huart);
		log.push_back(name + " tx");
	}
	CallLog &log;
	std::string name;
	std::vector<std::pair<std::string, UART_HandleTypeDef *>> calls;
};

struct RecordingCanHandler : CanHandler {
	struct Call {
		std::string event;
		CANPort *port;
		uint32_t arg; // fifo, mailbox or error code
	};
	void canRxPendCallback(CANPort *port, CAN_rx_msg &msg) override {
		calls.push_back({"rx", port, msg.fifo});
		msgs.push_back(msg);
	}
	void canErrorCallback(CANPort *port, uint32_t errcode) override { calls.push_back({"error", port, errcode}); }
	void canRxFullCallback(CANPort *port, uint32_t fifo) override { calls.push_back({"full", port, fifo}); }
	void canTxCpltCallback(CANPort *port, uint32_t mailbox) override { calls.push_back({"txcplt", port, mailbox}); }
	void canTxAbortCallback(CANPort *port, uint32_t mailbox) override { calls.push_back({"txabort", port, mailbox}); }
	std::vector<Call> calls;
	std::vector<CAN_rx_msg> msgs;
};

struct RecordingSpiHandler : SpiHandler {
	explicit RecordingSpiHandler(CallLog &log, std::string name = "spi") : log(log), name(std::move(name)) {}
	void record(const char *event, SPI_HandleTypeDef *hspi) {
		calls.emplace_back(event, hspi);
		log.push_back(name + " " + event);
	}
	void SpiTxCplt(SPI_HandleTypeDef *hspi) override { record("TxCplt", hspi); }
	void SpiRxCplt(SPI_HandleTypeDef *hspi) override { record("RxCplt", hspi); }
	void SpiTxRxCplt(SPI_HandleTypeDef *hspi) override { record("TxRxCplt", hspi); }
	void SpiTxHalfCplt(SPI_HandleTypeDef *hspi) override { record("TxHalfCplt", hspi); }
	void SpiRxHalfCplt(SPI_HandleTypeDef *hspi) override { record("RxHalfCplt", hspi); }
	void SpiTxRxHalfCplt(SPI_HandleTypeDef *hspi) override { record("TxRxHalfCplt", hspi); }
	void SpiError(SPI_HandleTypeDef *hspi) override { record("Error", hspi); }
	CallLog &log;
	std::string name;
	std::vector<std::pair<std::string, SPI_HandleTypeDef *>> calls;
};

struct RecordingI2CHandler : I2CHandler {
	explicit RecordingI2CHandler(CallLog &log, std::string name = "i2c") : log(log), name(std::move(name)) {}
	void record(const char *event, I2C_HandleTypeDef *hi2c) {
		calls.emplace_back(event, hi2c);
		log.push_back(name + " " + event);
	}
	void I2cTxCplt(I2C_HandleTypeDef *hi2c) override { record("TxCplt", hi2c); }
	void I2cRxCplt(I2C_HandleTypeDef *hi2c) override { record("RxCplt", hi2c); }
	void I2cError(I2C_HandleTypeDef *hi2c) override { record("Error", hi2c); }
	CallLog &log;
	std::string name;
	std::vector<std::pair<std::string, I2C_HandleTypeDef *>> calls;
};

/** HID handler recording the reports of the TinyUSB HID callbacks */
struct RecordingHidHandler : UsbHidHandler {
	struct Out {
		uint8_t reportId;
		hid_report_type_t type;
		std::vector<uint8_t> data;
	};
	void hidOut(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) override {
		outs.push_back({report_id, report_type, std::vector<uint8_t>(buffer, buffer + bufsize)});
	}
	uint16_t hidGet(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) override {
		gets.push_back({report_id, report_type, {}});
		getReqlen = reqlen;
		buffer[0] = 0x5A;
		return getResult;
	}
	std::vector<Out> outs;
	std::vector<Out> gets;
	uint16_t getReqlen = 0;
	uint16_t getResult = 7;
};

/** Main class recording the USB events of the TinyUSB callbacks */
struct RecordingMain : FFBoardMain {
	void usbSuspend() override { events.push_back("suspend"); }
	void usbResume() override { events.push_back("resume"); }
	void cdcRcvReady(uint8_t itf) override { events.push_back("cdc" + std::to_string(itf)); }
	std::vector<std::string> events;
};

/** Raises an interrupt callback in simulated interrupt context */
template <class F> void inIsr(F f) {
	HostPlatform::IsrContext isr;
	f();
}

CAN_RxHeaderTypeDef stdHeader(uint32_t id, uint32_t dlc) {
	CAN_RxHeaderTypeDef header = {};
	header.StdId = id;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = dlc;
	return header;
}

/**
 * CDCcomm keeps a retry flag after a partial send that FirmwareFixture does not clear (see cases/test_cdc.cpp).
 * A complete send clears it.
 */
void resetCdc() {
	HostUsb::reset();
	CDCcomm::clearRemainingBuffer(0);
	CDCcomm::cdcFinished(0);
	std::string sync = "-";
	CDCcomm::cdcSend(&sync, 0);
	CDCcomm::cdcFinished(0);
	HostUsb::reset();
}

struct CallbackFixture : TargetFixture {
	CallLog log;
};

struct HidCallbackFixture : TargetFixture {
	HidCallbackFixture() : savedHidDesc(UsbHidHandler::getHidDesc()) {}
	~HidCallbackFixture() override { UsbHidHandler::setHidDesc(savedHidDesc); }
	const uint8_t *savedHidDesc;
	FFBoardMainCommandThread commandThread{nullptr}; // HID_CommandInterface wakes the command thread
};

struct CdcFixture : TargetFixture {
	CdcFixture() { resetCdc(); }
	~CdcFixture() override { resetCdc(); }
};

} // namespace

TEST_SUITE("GlobalCallbacks") {

// ---------------------------------------------------------------- ADC ----------------------------------------------------------------

TEST_CASE_FIXTURE(CallbackFixture, "an ADC1 conversion passes the ADC1 DMA buffer with its 8 channels and the handle to every ADC handler") {
	RecordingAdcHandler first, second;
	startADC();

	REQUIRE(HostTarget::adcConvert(&hadc1, {10, 11, 12, 13, 14, 15, 16, 17}));

	for (RecordingAdcHandler *h : {&first, &second}) {
		REQUIRE(h->calls.size() == 1);
		CHECK((h->calls[0].buf == ADC1_BUF));
		CHECK(h->calls[0].chans == ADC1_CHANNELS);
		CHECK(h->calls[0].hadc == &hadc1);
		CHECK(h->calls[0].values == std::vector<uint32_t>{10, 11, 12, 13, 14, 15, 16, 17});
	}
}

TEST_CASE_FIXTURE(CallbackFixture, "an ADC2 conversion passes the ADC2 DMA buffer with its 2 channels and the handle to the ADC handlers") {
	RecordingAdcHandler handler;
	startADC();

	REQUIRE(HostTarget::adcConvert(&hadc2, {1241, 1300}));

	REQUIRE(handler.calls.size() == 1);
	CHECK((handler.calls[0].buf == ADC2_BUF));
	CHECK(handler.calls[0].chans == ADC2_CHANNELS);
	CHECK(handler.calls[0].hadc == &hadc2);
	CHECK(handler.calls[0].values == std::vector<uint32_t>{1241, 1300});
}

TEST_CASE_FIXTURE(CallbackFixture, "a conversion of the voltage sense ADC checks the brake before the ADC handlers run") {
	RecordingAdcHandler handler;
	startADC();
	HostTarget::setAdcCalibration(1500, 950, 1200);
	HostTarget::adcSetInput(&hadc1, 7, 1500); // VREFINT at its calibration value: 3300 mV reference
	REQUIRE(HostTarget::adcConversionComplete(&hadc1));

	REQUIRE(HostTarget::adcConvert(&hadc2, {1241, 1489})); // VINT 6.1 V above VEXT: brake on

	REQUIRE(handler.calls.size() == 2);
	CHECK(handler.calls[1].hadc == &hadc2);
	CHECK(handler.calls[1].brakeOn);
}

TEST_CASE_FIXTURE(CallbackFixture, "a conversion of another ADC than the voltage sense ADC does not check the brake") {
	startADC();
	HostTarget::setAdcCalibration(1500, 950, 1200);
	ADC2_BUF[0] = 1241; // VEXT and VINT as converted, 6.1 V difference: brakeCheck would switch the brake on
	ADC2_BUF[1] = 1489;

	std::vector<uint32_t> adc1(ADC1_CHANNELS, 0);
	adc1[7] = 1500;
	REQUIRE(HostTarget::adcConvert(&hadc1, adc1));

	CHECK_FALSE(HostTarget::outputPin(DRV_BRAKE_GPIO_Port, DRV_BRAKE_Pin));
}

TEST_CASE_FIXTURE(CallbackFixture, "a conversion complete callback of an ADC without DMA buffer reaches no ADC handler") {
	RecordingAdcHandler handler;
	ADC_HandleTypeDef hadc3 = {};

	inIsr([&] { HAL_ADC_ConvCpltCallback(&hadc3); });

	CHECK(handler.calls.empty());
}

TEST_CASE_FIXTURE(CallbackFixture, "the analog buffer of ADC1 and ADC2 is their DMA buffer with the configured channel count") {
	uint8_t chans = 0;
	CHECK((getAnalogBuffer(&hadc1, &chans) == ADC1_BUF));
	CHECK(chans == ADC1_CHANNELS);
	CHECK((getAnalogBuffer(&hadc2, &chans) == ADC2_BUF));
	CHECK(chans == ADC2_CHANNELS);
}

TEST_CASE_FIXTURE(CallbackFixture, "an ADC without DMA buffer has no analog buffer and its channel count is left unchanged") {
	ADC_HandleTypeDef other = {};
	uint8_t chans = 42;
	CHECK((getAnalogBuffer(&other, &chans) == nullptr));
	CHECK(chans == 42);
}

TEST_CASE_FIXTURE(CallbackFixture, "startADC starts the DMA of ADC1 and ADC2 into their buffers with one word per channel") {
	REQUIRE_FALSE(HostTarget::adcDmaRunning(&hadc1));
	REQUIRE_FALSE(HostTarget::adcDmaRunning(&hadc2));

	startADC();

	CHECK(HostTarget::adcDmaBuffer(&hadc1) == const_cast<uint32_t *>(ADC1_BUF));
	CHECK(HostTarget::adcDmaLength(&hadc1) == ADC1_CHANNELS);
	CHECK(HostTarget::adcDmaBuffer(&hadc2) == const_cast<uint32_t *>(ADC2_BUF));
	CHECK(HostTarget::adcDmaLength(&hadc2) == ADC2_CHANNELS);
}

// ---------------------------------------------------------------- TIM, EXTI ----------------------------------------------------------------

TEST_CASE_FIXTURE(CallbackFixture, "a timer period elapsed callback reaches every timer handler with the timer handle in registration order") {
	RecordingTimerHandler first(log, "first"), second(log, "second");

	inIsr([] { HAL_TIM_PeriodElapsedCallback_CPP(&htim13); });

	CHECK(log == CallLog{"first", "second"});
	CHECK(first.handles == std::vector<TIM_HandleTypeDef *>{&htim13});
	CHECK(second.handles == std::vector<TIM_HandleTypeDef *>{&htim13});
}

TEST_CASE_FIXTURE(CallbackFixture, "an update of a running timer other than the HAL tick reaches the timer handlers through main.c") {
	RecordingTimerHandler handler(log, "timer");
	REQUIRE(HAL_TIM_Base_Start_IT(&htim3) == HAL_OK);

	CHECK(HostTarget::timerUpdate(&htim3));

	CHECK(handler.handles == std::vector<TIM_HandleTypeDef *>{&htim3});
}

TEST_CASE_FIXTURE(CallbackFixture, "an external interrupt reaches every EXTI handler with the pin in registration order") {
	RecordingExtiHandler first(log, "first"), second(log, "second");

	HostTarget::triggerExti(BUTTON_A_Pin);

	CHECK(log == CallLog{"first", "second"});
	CHECK(first.pins == std::vector<uint16_t>{BUTTON_A_Pin});
	CHECK(second.pins == std::vector<uint16_t>{BUTTON_A_Pin});
}

TEST_CASE_FIXTURE(CallbackFixture, "a destroyed EXTI handler no longer receives external interrupts") {
	RecordingExtiHandler remaining(log, "remaining");
	{
		RecordingExtiHandler removed(log, "removed");
	}

	HostTarget::triggerExti(E_STOP_Pin);

	CHECK(log == CallLog{"remaining"});
}

// ---------------------------------------------------------------- UART ----------------------------------------------------------------

TEST_CASE_FIXTURE(CallbackFixture, "a UART receive complete reaches every UART handler with the UART handle") {
	UART_HandleTypeDef other = {};
	RecordingUartHandler first(log, "first"), second(log, "second");

	inIsr([&] { HAL_UART_RxCpltCallback(&other); });

	CHECK(log == CallLog{"first rx", "second rx"});
	REQUIRE(second.calls.size() == 1);
	CHECK(second.calls[0].second == &other);
}

TEST_CASE_FIXTURE(CallbackFixture, "a UART transmit complete reaches every UART handler as a transmit event with the UART handle") {
	UART_HandleTypeDef other = {};
	RecordingUartHandler first(log, "first"), second(log, "second");

	inIsr([&] { HAL_UART_TxCpltCallback(&other); });

	CHECK(log == CallLog{"first tx", "second tx"});
	REQUIRE(first.calls.size() == 1);
	CHECK(first.calls[0].second == &other);
}

// ---------------------------------------------------------------- CAN ----------------------------------------------------------------

TEST_CASE_FIXTURE(CallbackFixture, "a frame in CAN receive FIFO 1 reaches the CAN handlers with fifo 1, its header and data") {
	RecordingCanHandler handler;
	CAN_RxHeaderTypeDef header = {};
	header.ExtId = 0x1ABCDE;
	header.IDE = CAN_ID_EXT;
	header.RTR = CAN_RTR_REMOTE;
	header.DLC = 8;
	header.FilterMatchIndex = 5;
	header.Timestamp = 1234;
	const uint8_t data[8] = {8, 7, 6, 5, 4, 3, 2, 1};
	HostTarget::canQueueRx(CAN_RX_FIFO1, header, data);

	CHECK(HostTarget::canRxInterrupt(CAN_RX_FIFO1) == 1);

	REQUIRE(handler.msgs.size() == 1);
	CHECK(handler.calls[0].port == &canport);
	const CAN_rx_msg &msg = handler.msgs[0];
	CHECK(msg.fifo == 1);
	CHECK(msg.header.id == 0x1ABCDE);
	CHECK(msg.header.extId);
	CHECK(msg.header.rtr);
	CHECK(msg.header.length == 8);
	CHECK(msg.header.filter == 5);
	CHECK(msg.header.timestamp == 1234);
	CHECK(std::memcmp(msg.data, data, 8) == 0);
	CHECK(HostTarget::canRxPending(CAN_RX_FIFO1) == 0);
}

TEST_CASE_FIXTURE(CallbackFixture, "three frames in a CAN receive FIFO reach every CAN handler once each in arrival order") {
	RecordingCanHandler first, second;
	const uint8_t data[8] = {};
	for (uint32_t id : {0x101u, 0x102u, 0x103u}) {
		HostTarget::canQueueRx(CAN_RX_FIFO0, stdHeader(id, 1), data);
	}

	HostTarget::canRxInterrupt(CAN_RX_FIFO0);

	for (RecordingCanHandler *h : {&first, &second}) {
		REQUIRE(h->msgs.size() == 3);
		CHECK(h->msgs[0].header.id == 0x101);
		CHECK(h->msgs[1].header.id == 0x102);
		CHECK(h->msgs[2].header.id == 0x103);
		CHECK(h->msgs[2].fifo == 0);
	}
	CHECK(HostTarget::state().canRxStalls == 0);
}

TEST_CASE_FIXTURE(CallbackFixture, "characterization: one message pending interrupt with three frames reads two, the third is read when the interrupt is raised again") {
	// NEEDS REVIEW 60: global_callbacks.cpp:118/134 compare the loop index with the fill level read again after each
	// HAL_CAN_GetRxMessage: the level drops while i rises, so one call reads ceil(n/2) frames. No frame is lost (the
	// level-triggered interrupt fires again), but it costs extra interrupts. Intended: read until the FIFO is empty?
	RecordingCanHandler handler;
	const uint8_t data[8] = {};
	for (uint32_t id : {0x101u, 0x102u, 0x103u}) {
		HostTarget::canQueueRx(CAN_RX_FIFO0, stdHeader(id, 1), data);
	}

	CHECK(HostTarget::canRxInterrupt(CAN_RX_FIFO0) == 2);
	CHECK(handler.msgs.size() == 3);
}

TEST_CASE_FIXTURE(CallbackFixture, "a message pending interrupt of a CAN handle without frames reaches no CAN handler") {
	RecordingCanHandler handler;
	CAN_HandleTypeDef other = {};
	other.State = HAL_CAN_STATE_READY;

	inIsr([&] {
		HAL_CAN_RxFifo0MsgPendingCallback(&other);
		HAL_CAN_RxFifo1MsgPendingCallback(&other);
	});

	CHECK(handler.calls.empty());
}

TEST_CASE_FIXTURE(CallbackFixture, "a full CAN receive FIFO is reported to the CAN handlers with the port and the FIFO number") {
	RecordingCanHandler handler;

	inIsr([] {
		HAL_CAN_RxFifo0FullCallback(&hcan1);
		HAL_CAN_RxFifo1FullCallback(&hcan1);
	});

	REQUIRE(handler.calls.size() == 2);
	CHECK(handler.calls[0].event == "full");
	CHECK(handler.calls[0].port == &canport);
	CHECK(handler.calls[0].arg == 0);
	CHECK(handler.calls[1].event == "full");
	CHECK(handler.calls[1].arg == 1);
}

TEST_CASE_FIXTURE(CallbackFixture, "a completed transmit mailbox is reported to the CAN handlers with the port and the mailbox number") {
	RecordingCanHandler handler;

	inIsr([] {
		HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
		HAL_CAN_TxMailbox1CompleteCallback(&hcan1);
		HAL_CAN_TxMailbox2CompleteCallback(&hcan1);
	});

	REQUIRE(handler.calls.size() == 3);
	for (uint32_t i = 0; i < 3; i++) {
		CAPTURE(i);
		CHECK(handler.calls[i].event == "txcplt");
		CHECK(handler.calls[i].port == &canport);
		CHECK(handler.calls[i].arg == i);
	}
}

TEST_CASE_FIXTURE(CallbackFixture, "an aborted transmit mailbox is reported to the CAN handlers with the port and the mailbox number") {
	RecordingCanHandler handler;

	inIsr([] {
		HAL_CAN_TxMailbox0AbortCallback(&hcan1);
		HAL_CAN_TxMailbox1AbortCallback(&hcan1);
		HAL_CAN_TxMailbox2AbortCallback(&hcan1);
	});

	REQUIRE(handler.calls.size() == 3);
	for (uint32_t i = 0; i < 3; i++) {
		CAPTURE(i);
		CHECK(handler.calls[i].event == "txabort");
		CHECK(handler.calls[i].port == &canport);
		CHECK(handler.calls[i].arg == i);
	}
}

TEST_CASE_FIXTURE(CallbackFixture, "a CAN error is passed to every CAN handler with the error code, which is cleared afterwards") {
	RecordingCanHandler first, second;
	hcan1.ErrorCode = CAN_ERROR_BOF | CAN_ERROR_ACK;

	inIsr([] { HAL_CAN_ErrorCallback(&hcan1); });

	for (RecordingCanHandler *h : {&first, &second}) {
		REQUIRE(h->calls.size() == 1);
		CHECK(h->calls[0].event == "error");
		CHECK(h->calls[0].port == &canport);
		CHECK(h->calls[0].arg == (CAN_ERROR_BOF | CAN_ERROR_ACK));
	}
	CHECK(hcan1.ErrorCode == 0);
}

TEST_CASE_FIXTURE(CallbackFixture, "CAN callbacks of a handle without a CAN port pass no port to the CAN handlers") {
	RecordingCanHandler handler;
	CAN_HandleTypeDef other = {};

	inIsr([&] {
		HAL_CAN_RxFifo0FullCallback(&other);
		HAL_CAN_TxMailbox0CompleteCallback(&other);
		HAL_CAN_TxMailbox0AbortCallback(&other);
		HAL_CAN_ErrorCallback(&other);
	});

	REQUIRE(handler.calls.size() == 4);
	for (const auto &call : handler.calls) {
		CAPTURE(call.event);
		CHECK(call.port == nullptr);
	}
}

// ---------------------------------------------------------------- SPI, I2C ----------------------------------------------------------------

TEST_CASE_FIXTURE(CallbackFixture, "each SPI completion callback reaches every SPI handler as its own event with the SPI handle") {
	SPI_HandleTypeDef other = {};
	RecordingSpiHandler first(log, "first"), second(log, "second");

	inIsr([&] {
		HAL_SPI_TxCpltCallback(&other);
		HAL_SPI_RxCpltCallback(&other);
		HAL_SPI_TxRxCpltCallback(&other);
		HAL_SPI_TxHalfCpltCallback(&other);
		HAL_SPI_RxHalfCpltCallback(&other);
		HAL_SPI_TxRxHalfCpltCallback(&other);
		HAL_SPI_ErrorCallback(&other);
	});

	CHECK(log == CallLog{"first TxCplt", "second TxCplt", "first RxCplt", "second RxCplt", "first TxRxCplt", "second TxRxCplt",
						 "first TxHalfCplt", "second TxHalfCplt", "first RxHalfCplt", "second RxHalfCplt",
						 "first TxRxHalfCplt", "second TxRxHalfCplt", "first Error", "second Error"});
	for (const auto &call : first.calls) {
		CHECK(call.second == &other);
	}
}

TEST_CASE_FIXTURE(CallbackFixture, "I2C master and memory transfers complete as transmit and receive events of every I2C handler with the handle") {
	I2C_HandleTypeDef other = {};
	RecordingI2CHandler first(log, "first"), second(log, "second");

	inIsr([&] {
		HAL_I2C_MasterTxCpltCallback(&other);
		HAL_I2C_MasterRxCpltCallback(&other);
		HAL_I2C_MemTxCpltCallback(&other);
		HAL_I2C_MemRxCpltCallback(&other);
	});

	CHECK(log == CallLog{"first TxCplt", "second TxCplt", "first RxCplt", "second RxCplt", "first TxCplt", "second TxCplt",
						 "first RxCplt", "second RxCplt"});
	for (const auto &call : second.calls) {
		CHECK(call.second == &other);
	}
}

TEST_CASE_FIXTURE(CallbackFixture, "an I2C error reaches every I2C handler as an error event with the handle") {
	// The global i2cport is taken out of the list: I2CPort::I2cError resets the port for any handle (I2C.cpp:250-256)
	// and that needs the unmodelled HAL I2C (P5).
	IsolatedHandlers<I2CHandler> isolated(I2CHandler::getI2CHandlers());
	I2C_HandleTypeDef other = {};
	RecordingI2CHandler first(log, "first"), second(log, "second");

	inIsr([&] { HAL_I2C_ErrorCallback(&other); });

	CHECK(log == CallLog{"first Error", "second Error"});
	REQUIRE(first.calls.size() == 1);
	CHECK(first.calls[0].second == &other);
}

// ---------------------------------------------------------------- TinyUSB ----------------------------------------------------------------

TEST_CASE_FIXTURE(CallbackFixture, "the USB device and configuration descriptors come from the registered USB device") {
	USBdevice device(&usb_devdesc_ffboard_composite, usb_cdc_conf, &usb_ffboard_strings_default);
	device.registerUsb();

	CHECK(tud_descriptor_device_cb() == reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite));
	CHECK(tud_descriptor_configuration_cb(0) == usb_cdc_conf);
}

TEST_CASE_FIXTURE(CallbackFixture, "the USB string descriptor callback returns the string of the registered USB device") {
	USBdevice device(&usb_devdesc_ffboard_composite, usb_cdc_conf, &usb_ffboard_strings_default);
	device.registerUsb();

	const uint16_t *desc = tud_descriptor_string_cb(1, 0x0409);

	REQUIRE(desc != nullptr);
	const std::string manufacturer = "Open FFBoard";
	CHECK(desc[0] == ((TUSB_DESC_STRING << 8) | (2 * manufacturer.size() + 2)));
	CHECK(desc[1] == 'O');
	CHECK(desc[manufacturer.size()] == 'd');
}

TEST_CASE_FIXTURE(HidCallbackFixture, "the HID report descriptor callback returns the descriptor set by the HID handler") {
	static const uint8_t desc[] = {0x05, 0x01, 0x09, 0x04};
	UsbHidHandler::setHidDesc(desc);

	CHECK(tud_hid_descriptor_report_cb(0) == desc);
}

TEST_CASE_FIXTURE(HidCallbackFixture, "an output report sent without report id passes the first data byte as report id to the HID handler") {
	RecordingHidHandler handler;
	handler.registerHidCallback();
	const uint8_t report[] = {0x05, 0x01, 0x02};

	tud_hid_set_report_cb(0, 0, HID_REPORT_TYPE_OUTPUT, report, sizeof(report));

	REQUIRE(handler.outs.size() == 1);
	CHECK(handler.outs[0].reportId == 0x05);
	CHECK(handler.outs[0].type == HID_REPORT_TYPE_OUTPUT);
	CHECK(handler.outs[0].data == std::vector<uint8_t>{0x05, 0x01, 0x02});
}

TEST_CASE_FIXTURE(HidCallbackFixture, "an interrupt OUT report of invalid type without report id passes the first data byte as report id") {
	RecordingHidHandler handler;
	handler.registerHidCallback();
	const uint8_t report[] = {0x0A, 0x01};

	tud_hid_set_report_cb(0, 0, HID_REPORT_TYPE_INVALID, report, sizeof(report));

	REQUIRE(handler.outs.size() == 1);
	CHECK(handler.outs[0].reportId == 0x0A);
	CHECK(handler.outs[0].type == HID_REPORT_TYPE_INVALID);
}

TEST_CASE_FIXTURE(HidCallbackFixture, "a feature report keeps report id 0 and an output report keeps its own report id") {
	RecordingHidHandler handler;
	handler.registerHidCallback();
	const uint8_t report[] = {0x0A, 0x01};

	tud_hid_set_report_cb(0, 0, HID_REPORT_TYPE_FEATURE, report, sizeof(report));
	tud_hid_set_report_cb(0, 0x0B, HID_REPORT_TYPE_OUTPUT, report, sizeof(report));

	REQUIRE(handler.outs.size() == 2);
	CHECK(handler.outs[0].reportId == 0);
	CHECK(handler.outs[0].type == HID_REPORT_TYPE_FEATURE);
	CHECK(handler.outs[1].reportId == 0x0B);
}

TEST_CASE_FIXTURE(HidCallbackFixture, "a HID command report is passed to the HID command interface and to the HID handler") {
	MockCommandHandler mock;
	HID_CommandInterface hidCmd;
	RecordingHidHandler handler;
	handler.registerHidCallback();
	HID_CMD_Data_t data;
	data.type = HidCmdType::request;
	data.clsid = CLSID_CUSTOM;
	data.instance = 0;
	data.cmd = static_cast<uint32_t>(MockCmd::value);
	data.data = 1234;

	tud_hid_set_report_cb(0, 0, HID_REPORT_TYPE_OUTPUT, reinterpret_cast<const uint8_t *>(&data), sizeof(data));

	std::vector<ParsedCommand> commands;
	REQUIRE(hidCmd.getNewCommands(commands));
	REQUIRE(commands.size() == 1);
	CHECK(commands[0].cmdId == static_cast<uint32_t>(MockCmd::value));
	CHECK(commands[0].type == CMDtype::get);
	CHECK(commands[0].val == 1234);
	CHECK(commands[0].target == &mock);
	REQUIRE(handler.outs.size() == 1);
	CHECK(handler.outs[0].reportId == HID_ID_HIDCMD);
}

TEST_CASE_FIXTURE(HidCallbackFixture, "a HID command report reaches the HID command interface without a HID handler") {
	HID_CommandInterface hidCmd;
	HID_CMD_Data_t data;
	data.type = HidCmdType::request;
	data.clsid = 0x7777; // no such class: answered with notFound
	data.instance = 0;

	tud_hid_set_report_cb(0, HID_ID_HIDCMD, HID_REPORT_TYPE_OUTPUT, reinterpret_cast<const uint8_t *>(&data), sizeof(data));

	CHECK(hidCmd.waitingToSend());
}

TEST_CASE_FIXTURE(HidCallbackFixture, "a report of another id does not reach the HID command interface") {
	HID_CommandInterface hidCmd;
	HID_CMD_Data_t data;
	data.reportId = 0x05;
	data.type = HidCmdType::request;
	data.clsid = 0x7777; // would be answered with notFound by the command interface

	tud_hid_set_report_cb(0, 0, HID_REPORT_TYPE_OUTPUT, reinterpret_cast<const uint8_t *>(&data), sizeof(data));

	CHECK_FALSE(hidCmd.waitingToSend());
}

TEST_CASE_FIXTURE(HidCallbackFixture, "a HID command report without a HID command interface is ignored") {
	RecordingHidHandler handler;
	handler.registerHidCallback();
	HID_CMD_Data_t data;
	REQUIRE(HID_CommandInterface::globalInterface == nullptr);

	tud_hid_set_report_cb(0, HID_ID_HIDCMD, HID_REPORT_TYPE_OUTPUT, reinterpret_cast<const uint8_t *>(&data), sizeof(data));

	CHECK(handler.outs.size() == 1);
}

TEST_CASE_FIXTURE(HidCallbackFixture, "a get report request is answered by the HID handler with its length") {
	RecordingHidHandler handler;
	handler.registerHidCallback();
	uint8_t buffer[16] = {};

	CHECK(tud_hid_get_report_cb(0, 0x22, HID_REPORT_TYPE_FEATURE, buffer, sizeof(buffer)) == 7);

	REQUIRE(handler.gets.size() == 1);
	CHECK(handler.gets[0].reportId == 0x22);
	CHECK(handler.gets[0].type == HID_REPORT_TYPE_FEATURE);
	CHECK(handler.getReqlen == sizeof(buffer));
	CHECK(buffer[0] == 0x5A);
}

TEST_CASE_FIXTURE(HidCallbackFixture, "a get report request without a HID handler is answered with length 0") {
	uint8_t buffer[16] = {};
	CHECK(tud_hid_get_report_cb(0, 0x22, HID_REPORT_TYPE_FEATURE, buffer, sizeof(buffer)) == 0);
}

TEST_CASE_FIXTURE(HidCallbackFixture, "a completed HID IN transfer is accepted with and without HID command interface and handler") {
	// NEEDS REVIEW 61: UsbHidHandler::transferComplete (UsbHidHandler.h:27) is not virtual, so global_callbacks.cpp:345 always
	// calls the empty base version; a HID handler can not get the completion. No current handler needs it. TinyUSB
	// does not call this function at all (len parameter type, see the declaration above).
	const uint8_t report[] = {1, 2, 3};
	tud_hid_report_complete_cb(0, report, static_cast<uint8_t>(sizeof(report)));

	HID_CommandInterface hidCmd;
	RecordingHidHandler handler;
	handler.registerHidCallback();
	tud_hid_report_complete_cb(0, report, static_cast<uint8_t>(sizeof(report)));

	CHECK(handler.outs.empty());
	CHECK(handler.gets.empty());
	CHECK_FALSE(hidCmd.waitingToSend());
}

TEST_CASE_FIXTURE(CdcFixture, "received CDC data pulses the system LED and tells the main class which interface is ready") {
	RecordingMain main;
	mainclass = &main;
	uint32_t pulses = HostPlatform::leds().sysPulses;

	tud_cdc_rx_cb(0);

	CHECK(HostPlatform::leds().sysPulses == pulses + 1);
	CHECK(main.events == std::vector<std::string>{"cdc0"});
	mainclass = nullptr;
}

TEST_CASE_FIXTURE(CdcFixture, "received CDC data before a main class exists only pulses the system LED") {
	REQUIRE(mainclass == nullptr);
	uint32_t pulses = HostPlatform::leds().sysPulses;

	tud_cdc_rx_cb(0);

	CHECK(HostPlatform::leds().sysPulses == pulses + 1);
}

TEST_CASE_FIXTURE(CdcFixture, "a CDC transmit complete sends the remainder of a reply that did not fit into the USB buffer") {
	HostUsb::state().cdcWriteAvailable = 3;
	std::string reply = "abcdef";
	CDCcomm::cdcSend(&reply, 0);
	REQUIRE(HostUsb::state().cdcTx == "abc");

	tud_cdc_tx_complete_cb(0);

	CHECK(HostUsb::state().cdcTx == "abcdef");
}

TEST_CASE_FIXTURE(CallbackFixture, "USB mount and resume resume the main class, unmount and suspend suspend it") {
	// tud_suspend_cb() here is the firmware function (global_callbacks.cpp:357), called directly; TinyUSB calls
	// tud_suspend_cb(bool) instead (see the KNOWN_ISSUE below)
	// NEEDS REVIEW 63: the four callbacks (global_callbacks.cpp:356-372) dereference mainclass without the null check of
	// tud_cdc_rx_cb. Not tested with nullptr (crash): unreachable today, the USB task is only started by
	// mainclass->usbInit() after cppmain has set mainclass (cppmain.cpp:93-100). A null check would make it robust.
	RecordingMain main;
	mainclass = &main;

	tud_mount_cb();
	tud_suspend_cb();
	tud_resume_cb();
	tud_umount_cb();

	CHECK(main.events == std::vector<std::string>{"resume", "suspend", "resume", "suspend"});
	mainclass = nullptr;
}

TEST_CASE_FIXTURE(CallbackFixture, "a USB bus suspend reported by TinyUSB suspends the main class" *
	KNOWN_ISSUE("global_callbacks.cpp:357 defines tud_suspend_cb() without the bool parameter of TinyUSB's "
				"tud_suspend_cb(bool remote_wakeup_en) (usbd.h:162, called at usbd.c:659): a C++ overload, so TinyUSB "
				"keeps its empty weak default and the main class is never suspended on a bus suspend")) {
	RecordingMain main;
	mainclass = &main;

	tud_suspend_cb(true); // As usbd.c:659 calls it on a bus suspend

	CHECK(main.events == std::vector<std::string>{"suspend"});
	mainclass = nullptr;
}

} // TEST_SUITE
