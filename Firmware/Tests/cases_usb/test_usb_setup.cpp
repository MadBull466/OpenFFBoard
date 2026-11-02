/*
 * Smoke tests of ffb_usb_tests: the real TinyUSB device stack with the firmware's USB glue on the fake device
 * controller (support/HostDcd.h), driven by the simulated host (support/UsbHost.h). One test per path; the details
 * are covered by the module tests.
 */
#include "doctest.h"
#include "UsbFixture.h"

#include "ffb_defs.h"

#include <string>
#include <vector>

TEST_SUITE("UsbSetup") {

TEST_CASE_FIXTURE(UsbFixture, "tusb_init connects the pull-up and the device answers only after a bus reset") {
	startDevice();
	CHECK(HostDcd::state().initialised);
	CHECK(HostDcd::state().pullup);
	CHECK(HostDcd::state().interruptsEnabled);
	CHECK_FALSE(HostDcd::reachable());
	CHECK(host.getDescriptor(TUSB_DESC_DEVICE, 0, 18).status == HostDcd::Handshake::NoResponse);

	host.busReset();
	CHECK(HostDcd::reachable());
	CHECK(tud_speed_get() == TUSB_SPEED_FULL);
	CHECK(host.getDescriptor(TUSB_DESC_DEVICE, 0, 18).ok());
}

TEST_CASE_FIXTURE(UsbFixture, "the device enumerates with the descriptors of the F407 wheel and calls the mount callback") {
	startDevice();
	enumerate();

	// usb_descriptors.cpp: VID 0x1209, PID 0xFFB0, EP0 64 bytes, one configuration
	REQUIRE(host.deviceDescriptor.size() == 18);
	CHECK(host.deviceDescriptor[7] == 64);
	CHECK((host.deviceDescriptor[8] | host.deviceDescriptor[9] << 8) == 0x1209);
	CHECK((host.deviceDescriptor[10] | host.deviceDescriptor[11] << 8) == 0xFFB0);
	CHECK(host.deviceDescriptor[17] == 1);
	// USB_CONF_DESC_HID_CDC: CDC control + data, HID with IN 0x83 and OUT 0x02
	REQUIRE(host.interfaces.size() == 3);
	const UsbHost::Interface *data = host.findInterface(TUSB_CLASS_CDC_DATA);
	const UsbHost::Interface *hid = host.findInterface(TUSB_CLASS_HID);
	REQUIRE(data != nullptr);
	REQUIRE(hid != nullptr);
	CHECK(data->epOut == 0x01);
	CHECK(data->epIn == 0x81);
	CHECK(hid->epIn == 0x83);
	CHECK(hid->epOut == 0x02);

	CHECK(HostDcd::state().address == 5);
	CHECK(tud_mounted());
	CHECK(main->resumes == 1); // tud_mount_cb -> mainclass->usbResume()
	CHECK(host.getConfiguration().data == std::vector<uint8_t>{1});
}

TEST_CASE_FIXTURE(UsbFixture, "the new address takes effect after the status stage of SET_ADDRESS") {
	startDevice();
	host.busReset();
	REQUIRE(HostDcd::setup(0, UsbHost::request(0x00, TUSB_REQ_SET_ADDRESS, 9, 0, 0)) == HostDcd::Handshake::Ack);
	process();
	CHECK(HostDcd::state().address == 0); // Still answers at address 0 for the status stage
	std::vector<uint8_t> zlp;
	REQUIRE(HostDcd::in(0, 0, zlp) == HostDcd::Handshake::Ack);
	CHECK(zlp.empty());
	CHECK(HostDcd::state().address == 9);
	host.address = 9;
	process();
	CHECK(host.getDescriptor(TUSB_DESC_DEVICE, 0, 18).ok());
}

TEST_CASE_FIXTURE(UsbFixture, "the product string descriptor is the firmware's product name") {
	startDevice();
	enumerate();
	CHECK(host.getString(2) == std::string("FFBoard ") + HW_TYPE);
	CHECK(host.getString(1) == "Open FFBoard");
}

TEST_CASE_FIXTURE(UsbFixture, "a CDC line coding set by the host is read back") {
	startDevice();
	enumerate();
	REQUIRE(host.cdcSetLineCoding(250000, 0, 0, 8).ok());
	UsbHost::ControlResult r = host.cdcGetLineCoding();
	REQUIRE(r.ok());
	CHECK(r.data == std::vector<uint8_t>{0x90, 0xD0, 0x03, 0x00, 0, 0, 8});
	cdc_line_coding_t coding;
	tud_cdc_n_get_line_coding(0, &coding);
	CHECK(uint32_t(coding.bit_rate) == 250000);
}

TEST_CASE_FIXTURE(UsbFixture, "a command sent over CDC reaches the main class and its reply is sent back over CDC") {
	startDevice();
	enumerate();
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	REQUIRE(host.cdcWrite("sys.main?\n") == HostDcd::Handshake::Ack);
	process();
	CHECK(main->cdcReadyCalls >= 1); // tud_cdc_rx_cb -> mainclass->cdcRcvReady
	CHECK(host.cdcRead() == "[sys.main?|0]\n");
}

TEST_CASE_FIXTURE(UsbFixture, "an HID input report sent by the firmware reaches the host") {
	startDevice();
	HidFFB &hidffb = createHidFfb();
	enumerate();
	hidffb.sendStatusReport(0);
	UsbHost::Handshake status;
	std::vector<uint8_t> report = host.hidRead(&status);
	REQUIRE(status == HostDcd::Handshake::Ack);
	REQUIRE(report.size() == sizeof(reportFFB_status_t));
	CHECK(report[0] == HID_ID_STATE + FFB_ID_OFFSET);
}

TEST_CASE_FIXTURE(UsbFixture, "an HID output report on the interrupt endpoint reaches the HID handler") {
	startDevice();
	HidFFB &hidffb = createHidFfb();
	enumerate();
	REQUIRE_FALSE(hidffb.getFfbActive());
	// PID device control (report 0x0C): 1 = enable actuators
	REQUIRE(host.hidWrite({HID_ID_CTRLREP + FFB_ID_OFFSET, 0x01}) == HostDcd::Handshake::Ack);
	process();
	CHECK(hidffb.getFfbActive());
}

TEST_CASE_FIXTURE(UsbFixture, "an HID feature report set by the host reaches the HID handler and its answer is read back") {
	startDevice();
	createHidFfb();
	enumerate();
	// PID create new effect (feature 0x11): constant force, byte size 0
	REQUIRE(host.hidSetReport(HID_REPORT_TYPE_FEATURE, HID_ID_NEWEFREP + FFB_ID_OFFSET, {HID_ID_NEWEFREP + FFB_ID_OFFSET, 1, 0, 0}).ok());
	// PID block load (feature 0x12): the new effect got index 1, load status 1 = success
	UsbHost::ControlResult r = host.hidGetReport(HID_REPORT_TYPE_FEATURE, HID_ID_BLKLDREP + FFB_ID_OFFSET, 5);
	REQUIRE(r.ok());
	REQUIRE(r.data.size() >= 3);
	CHECK(r.data[0] == HID_ID_BLKLDREP + FFB_ID_OFFSET);
	CHECK(r.data[1] == 1);
	CHECK(r.data[2] == 1);
}

TEST_CASE_FIXTURE(UsbFixture, "an unsupported standard request is answered with a stall that the next setup clears") {
	startDevice();
	enumerate();
	// SET_DESCRIPTOR (USB 2.0 9.4.8) is optional and not supported by TinyUSB: request error -> STALL on EP0
	UsbHost::ControlResult r = host.control(UsbHost::request(0x00, TUSB_REQ_SET_DESCRIPTOR, TUSB_DESC_DEVICE << 8, 0, 0));
	CHECK(r.stalled());
	CHECK(HostDcd::endpoint(0x80).stalled);
	CHECK(host.getConfiguration().ok()); // USB 2.0 8.5.3.4: the next SETUP clears the stall
}

TEST_CASE_FIXTURE(UsbFixture, "a bus resume reported by TinyUSB resumes the main class") {
	startDevice();
	enumerate();
	const uint32_t resumes = main->resumes;
	HostDcd::suspend();
	process();
	CHECK(tud_suspended());
	HostDcd::resume();
	process();
	CHECK_FALSE(tud_suspended());
	CHECK(main->resumes == resumes + 1); // tud_resume_cb -> mainclass->usbResume()
}

TEST_CASE_FIXTURE(UsbFixture, "a USB bus suspend reported by TinyUSB suspends the main class (bug 42)" * doctest::should_fail() *
								  doctest::description("KNOWN ISSUE bug 42: global_callbacks.cpp defines tud_suspend_cb() without the bool parameter of "
													   "usbd.h:162, so TinyUSB calls its weak default (usbd.c:77) and never the main class")) {
	startDevice();
	enumerate();
	HostDcd::suspend();
	process();
	REQUIRE(tud_suspended()); // TinyUSB saw the suspend and called tud_suspend_cb(remote_wakeup_en)
	REQUIRE_USB_MODELLED();
	CHECK(main->suspends == 1); // Defect: the firmware's tud_suspend_cb() overload is never called
}

TEST_CASE_FIXTURE(UsbFixture, "the CDC data endpoint receives again after the host clears its halt (V21)" * doctest::should_fail() *
								  doctest::description("KNOWN ISSUE V21 (TinyUSB, vendor): SET_FEATURE(ENDPOINT_HALT) drops the armed OUT transfer "
													   "(dcd_edpt_stall); CLEAR_FEATURE only clears the halt (usbd.c usbd_edpt_clear_stall) and "
													   "cdcd_control_xfer_cb ignores standard requests, so nothing re-arms the endpoint: the device NAKs "
													   "every OUT packet until the application reads (the firmware only reads in tud_cdc_rx_cb)")) {
	startDevice();
	enumerate();
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	const uint8_t epOut = host.findInterface(TUSB_CLASS_CDC_DATA)->epOut;
	REQUIRE(host.control(UsbHost::request(0x02, TUSB_REQ_SET_FEATURE, TUSB_REQ_FEATURE_EDPT_HALT, epOut, 0)).ok());
	REQUIRE(host.cdcWrite("x") == HostDcd::Handshake::Stall);
	REQUIRE(host.control(UsbHost::request(0x02, TUSB_REQ_CLEAR_FEATURE, TUSB_REQ_FEATURE_EDPT_HALT, epOut, 0)).ok());
	REQUIRE_FALSE(HostDcd::endpoint(epOut).stalled);
	REQUIRE_USB_MODELLED();
	CHECK(host.cdcWrite("sys.main?\n") == HostDcd::Handshake::Ack); // Defect: NAK (no transfer armed) until the host times out
	process();
	CHECK(host.cdcRead() == "[sys.main?|0]\n");
}

TEST_CASE_FIXTURE(UsbFixture, "the OSAL of the host builds TinyUSB's semaphore, mutex and queue on the fake FreeRTOS") {
	// osal_freertos.h with configSUPPORT_STATIC_ALLOCATION 1 (static creation, platform/usb/hosttest_osal)
	osal_semaphore_def_t semDef;
	osal_semaphore_t sem = osal_semaphore_create(&semDef);
	REQUIRE(sem != nullptr);
	CHECK(osal_semaphore_post(sem, false));
	osal_semaphore_reset(sem); // xQueueReset on a semaphore: taken afterwards
	CHECK_FALSE(osal_semaphore_wait(sem, 0));
	CHECK(osal_semaphore_post(sem, false));
	CHECK(osal_semaphore_wait(sem, 0));
	osal_semaphore_delete(sem);

	osal_mutex_def_t mutexDef;
	osal_mutex_t mutex = osal_mutex_create(&mutexDef);
	REQUIRE(mutex != nullptr);
	CHECK(osal_mutex_lock(mutex, OSAL_TIMEOUT_NOTIMEOUT));
	CHECK(osal_mutex_unlock(mutex));
	osal_mutex_delete(mutex);

	OSAL_QUEUE_DEF(usbd_int_set, testQueueDef, 2, uint32_t);
	osal_queue_t queue = osal_queue_create(&testQueueDef);
	REQUIRE(queue != nullptr);
	const uint32_t value = 0x12345678;
	CHECK(osal_queue_empty(queue));
	CHECK(osal_queue_send(queue, &value, true));
	uint32_t received = 0;
	CHECK(osal_queue_receive(queue, &received, 0));
	CHECK(received == value);
	CHECK(osal_queue_empty(queue));
	osal_queue_delete(queue);
}

} // TEST_SUITE
