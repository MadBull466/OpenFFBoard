/*
 * UsbFixture.h
 *
 * Fixture of ffb_usb_tests: the real TinyUSB device stack on the fake device controller (support/HostDcd.h) with the
 * firmware's USB glue (USBdevice descriptors, the TinyUSB callbacks of global_callbacks.cpp, CDCcomm, UsbHidHandler),
 * driven by a simulated host (support/UsbHost.h). Use with TEST_CASE_FIXTURE(UsbFixture, "...").
 *
 * Before and after every test: TinyUSB is shut down (tud_deinit: event queue, mutexes) and the fake controller, the
 * simulated time, the fake FreeRTOS (F11, HOSTTEST_RTOS_EXT), the F407 platform (HostTarget::reset) and the firmware
 * globals the USB callbacks use (mainclass, usb_device, the global HID handler, CDCcomm, ErrorHandler, ...) are reset.
 *
 * startDevice(conf, hidReportDescriptor) does what FFBWheel::usbInit + USBdevice::Run do on the F407, without the USB
 * thread:
 *  - mainclass = a UsbTestMain (FFBoardMain, the failsafe main class, that counts the USB callbacks it receives); its
 *    threads "CDCCMD" and "CMD_MAIN" are resumable (HostRtos::makeResumable): the CDC reply thread waits for the end of
 *    the previous CDC transfer in the middle of its loop;
 *  - usb_device = a USBdevice with the real device descriptor (usb_devdesc_ffboard_composite), `conf` (default: the
 *    F407 FFBWheel configuration usb_cdc_hid_conf_2axis) and the default strings, appendSerial 4 (as FFBWheel);
 *  - UsbHidHandler::setHidDesc(hidReportDescriptor) (default hid_2ffb_desc, as FFBWheel::usbInit);
 *  - tusb_init() (USBdevice::Run's first call; dcd_init connects the pull-up), then process().
 * The USBdevice objects are never destroyed (one per configuration, kept for the whole run): ~Thread of a thread that
 * was never started deletes an uninitialised task handle (vendor V2), and the thread is not started here.
 * createHidFfb() adds a real EffectsCalculator + HidFFB (2 axes, direction enable 0x04 like FFBWheel) as the global HID
 * handler.
 *
 * Event processing: process() = what the USB thread (USBdevice::Run: while(1) tud_task()) and the other firmware
 * threads do between interrupts: tud_task_ext(0, false) on the test thread until TinyUSB's event queue is empty, and
 * HostRtos::runUntilIdle() for the firmware threads (command threads), repeated until nothing happens.
 * Host-vs-target: tud_task_ext(0, ...) returns when the queue is empty; on the target tud_task() blocks in
 * xQueueReceive until the next event. The order of events and their processing is the same.
 * The simulated host (`host`) calls process() between its transactions (after a SETUP, after a NAK).
 *
 * Checks at the end of every test (CHECK): no fault of the fake controller (unmodelled DCD call or USB protocol
 * violation), no host error (UsbHost::errors), no unmodelled HAL call or kernel fault (HostTarget), no kernel object
 * left over from the test (queues, semaphores, tasks, timers compared with the start). In a KNOWN_ISSUE test use
 * REQUIRE_USB_MODELLED() right before the assertion that describes the defect.
 */
#ifndef HOSTTEST_USBFIXTURE_H_
#define HOSTTEST_USBFIXTURE_H_

#include "HostDcd.h"
#include "HostPlatform.h"
#include "HostRtos.h"
#include "HostRtosExt.h"
#include "HostTarget.h"
#include "UsbHost.h"

#include "CDCcomm.h"
#include "CommandHandler.h"
#include "EffectsCalculator.h"
#include "ErrorHandler.h"
#include "FFBoardMain.h"
#include "HidFFB.h"
#include "PersistentStorage.h"
#include "SystemCommands.h"
#include "USBdevice.h"
#include "UsbHidHandler.h"
#include "usb_descriptors.h"
#include "usb_hid_ffb_desc.h"

#include "tusb.h"

#include "doctest.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

extern FFBoardMain *mainclass; // cppmain.cpp
extern USBdevice *usb_device;  // global_callbacks.cpp

/**
 * Fails the test (REQUIRE) if the fake controller or the host recorded a fault, or a HAL / kernel call was not
 * modelled. Use it in KNOWN_ISSUE tests right before the assertion that describes the defect.
 */
#define REQUIRE_USB_MODELLED()                                                                                         \
	do {                                                                                                               \
		INFO("first fault: " << UsbFixture::firstFault(host));                                                         \
		REQUIRE(UsbFixture::firstFault(host).empty());                                                                 \
	} while (0)

/** The failsafe main class (FFBoardMain), counting the USB callbacks of global_callbacks.cpp it receives */
class UsbTestMain : public FFBoardMain {
public:
	uint32_t suspends = 0;	  //!< usbSuspend calls (tud_suspend_cb / tud_umount_cb)
	uint32_t resumes = 0;	  //!< usbResume calls (tud_mount_cb / tud_resume_cb)
	uint32_t cdcReadyCalls = 0; //!< cdcRcvReady calls (tud_cdc_rx_cb)
	/**
	 * Opt-in: false makes cdcRcvReady return without reading (the received bytes stay in TinyUSB's CDC RX FIFO, e.g. to
	 * fill it). Default true: FFBoardMain::cdcRcvReady reads them into the CDC command interface.
	 */
	bool readCdc = true;

	void usbSuspend() override {
		suspends++;
		FFBoardMain::usbSuspend();
	}
	void usbResume() override {
		resumes++;
		FFBoardMain::usbResume();
	}
	void cdcRcvReady(uint8_t itf) override {
		cdcReadyCalls++;
		if (readCdc) {
			FFBoardMain::cdcRcvReady(itf);
		}
	}
};

/** Access to the protected PersistentStorage::startupComplete (as in AxisFixture.h) */
struct UsbPersistentStorageStartup : PersistentStorage {
	static void reset() { startupComplete = false; }
};

struct UsbFixture {
	UsbHost host{[this] { process(); }};
	std::unique_ptr<UsbTestMain> main; //!< mainclass while the device is started
	std::shared_ptr<EffectsCalculator> effects; //!< createHidFfb()
	std::unique_ptr<HidFFB> ffb;				//!< createHidFfb()

	UsbFixture() {
		resetAll();
		atStart = HostRtos::objectCounts();
	}
	virtual ~UsbFixture() {
		const std::string fault = firstFault(host);
		INFO("first fault: " << fault);
		CHECK_MESSAGE(fault.empty(), "the fake USB controller, the host or the fake platform recorded a fault (see UsbFixture.h)");
		// CDCcomm keeps a private retry flag (usb_busy_retry) while a reply is only partly sent; clearRemainingBuffer does
		// not clear it and a set flag suppresses the flush of the next send. A test must not end with unsent CDC data;
		// a complete 1-byte send clears the flag (as claude/cases/test_cdc.cpp CdcFixture), then the port is released.
		CHECK_MESSAGE(CDCcomm::remainingData(0) == 0, "the test left CDC data unsent: CDCcomm's retry flag is left set (see UsbFixture.h)");
		if (tud_ready()) {
			CDCcomm::clearRemainingBuffer(0);
			CDCcomm::cdcFinished(0);
			std::string sync = "-";
			CDCcomm::cdcSend(&sync, 0);
			CDCcomm::cdcFinished(0);
		}
		stopDevice();
		const HostRtos::ObjectCounts now = HostRtos::objectCounts();
		INFO("kernel objects at the start / end: tasks " << atStart.tasks << "/" << now.tasks << ", queues " << atStart.queues << "/"
														 << now.queues << ", semaphores " << atStart.semaphores << "/" << now.semaphores << ", timers "
														 << atStart.timers << "/" << now.timers);
		CHECK_MESSAGE((now.tasks <= atStart.tasks && now.queues <= atStart.queues && now.semaphores <= atStart.semaphores + keptSemaphores && now.timers <= atStart.timers &&
					   now.eventGroups <= atStart.eventGroups),
					  "kernel objects leaked by the test");
		resetAll();
	}

	/** First recorded fault: fake controller, host, unmodelled HAL call / kernel fault. Empty if none */
	static std::string firstFault(const UsbHost &usbHost) {
		if (!HostDcd::state().faults.empty()) {
			return "HostDcd: " + HostDcd::state().faults.front();
		}
		if (!usbHost.errors().empty()) {
			return "UsbHost: " + usbHost.errors().front();
		}
		if (!HostTarget::state().unmodelledHalCalls.empty()) {
			return "HostTarget: " + HostTarget::state().unmodelledHalCalls.front();
		}
		if (!HostRtos::kernelFaults().empty()) {
			return "HostRtos: " + HostRtos::kernelFaults().front();
		}
		return {};
	}

	/**
	 * One USBdevice per configuration, never destroyed (see the file comment). Its Thread base owns a semaphore: the
	 * leak check allows one semaphore per USBdevice created during the test.
	 */
	USBdevice *usbDevice(const uint8_t *conf) {
		static auto *devices = new std::map<const uint8_t *, USBdevice *>();
		USBdevice *&dev = (*devices)[conf];
		if (dev == nullptr) {
			dev = new USBdevice(&usb_devdesc_ffboard_composite, conf, &usb_ffboard_strings_default);
			keptSemaphores++;
		}
		return dev;
	}

	/** Starts the USB device like FFBWheel::usbInit + USBdevice::Run (see the file comment) */
	void startDevice(const uint8_t *conf = usb_cdc_hid_conf_2axis, const uint8_t *hidReportDescriptor = hid_2ffb_desc) {
		main = std::make_unique<UsbTestMain>();
		// The CDC reply path blocks in the middle of its loop (CDC_CommandInterface::Run -> CDCcomm::cdcSend ->
		// cdcSems.Take() until tud_cdc_tx_complete_cb): with the restart model of the fake RTOS the pending reply would be
		// lost, so these threads continue after their blocking call like on the target (HostRtos::makeResumable)
		REQUIRE(HostRtos::makeResumable("CDCCMD") != nullptr);
		REQUIRE(HostRtos::makeResumable("CMD_MAIN") != nullptr);
		mainclass = main.get();
		usb_device = usbDevice(conf);
		UsbHidHandler::setHidDesc(hidReportDescriptor);
		REQUIRE(tusb_init());
		process();
	}

	/** Real EffectsCalculator + HidFFB (2 axes) registered as the global HID handler, like an FFBWheel */
	HidFFB &createHidFfb() {
		effects = std::make_shared<EffectsCalculator>();
		ffb = std::make_unique<HidFFB>(effects, 2);
		ffb->setDirectionEnableMask(0x04);
		return *ffb;
	}

	/**
	 * Lets the device work: TinyUSB's event processing (tud_task_ext(0, false), see the file comment) and the firmware
	 * threads, until nothing is left. @return number of rounds
	 */
	uint32_t process() {
		uint32_t rounds = 0;
		for (; rounds < 1000; rounds++) {
			const bool events = tud_task_event_ready();
			tud_task_ext(0, false);
			const uint32_t activations = HostRtos::runUntilIdle();
			if (!events && activations == 0 && !tud_task_event_ready()) {
				break;
			}
		}
		return rounds;
	}

	/** Enumerates the device (UsbHost::enumerate) and REQUIREs success and the configured state */
	void enumerate() {
		REQUIRE(host.enumerate());
		REQUIRE(tud_mounted());
		// Successful setup is a fault boundary: derived fixtures may subsequently
		// clear expected-fault records. Do not let initialization faults vanish.
		INFO("first setup fault: " << firstFault(host));
		REQUIRE(firstFault(host).empty());
	}

	/** Deletes the firmware objects of the test and shuts TinyUSB down (also done by the destructor) */
	void stopDevice() {
		ffb.reset();
		effects.reset();
		UsbHidHandler::globalHidHandler = nullptr;
		mainclass = nullptr;
		main.reset();
		usb_device = nullptr;
		tud_deinit(0);
	}

	static void resetAll() {
		tud_deinit(0);
		HostDcd::reset();
		HostTarget::deleteCppmainMainclass();
		HostPlatform::reset();
		HostRtos::reset();
		HostTarget::reset();
		ErrorHandler::clearAll();
		SystemCommands::debugMode = false;
		SystemCommands::errorPrintingEnabled = true;
		CommandHandler::setLogsEnabled(true);
		UsbPersistentStorageStartup::reset();
		UsbHidHandler::globalHidHandler = nullptr;
		UsbHidHandler::setHidDesc(nullptr);
		mainclass = nullptr;
		usb_device = nullptr;
		CDCcomm::clearRemainingBuffer(0);
		CDCcomm::cdcFinished(0); // Gives the CDC semaphore back if a test left a transfer unfinished
	}

	HostRtos::ObjectCounts atStart;
	size_t keptSemaphores = 0; //!< Semaphores of USBdevice objects created by usbDevice() during the test
};

#endif
