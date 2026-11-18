/*
 * IntegrationFixture.cpp: see IntegrationFixture.h.
 */
#include "IntegrationFixture.h"

#include "CDCcomm.h"
#include "CommandHandler.h"
#include "ErrorHandler.h"
#include "HostPlatform.h"
#include "PersistentStorage.h"
#include "SystemCommands.h"
#include "UsbHidHandler.h"
#include "main.h"
#include "voltagesense.h"

#include <cmath>
#include <sstream>

extern float vSenseMult; // voltagesense.cpp (not declared in the header)

namespace {
/** Access to the protected PersistentStorage::startupComplete (no public reset) */
struct ItPersistentStorageStartup : PersistentStorage {
	static void reset() { startupComplete = false; }
};
} // namespace

/*
 * Order of plan 3.3 step 1 (tud_deinit, HostDcd::reset, HostPlatform::reset, HostRtos::reset, HostTarget::reset, ErrorHandler,
 * globals), with one addition: deleteCppmainMainclass() sits before HostPlatform::reset / HostRtos::reset. It is a safety net
 * (the destructor already deleted the main class); deleting a thread unwinds its parked host thread and needs the kernel
 * registry and the flash map of that case, which the later resets clear.
 * Host-vs-target: function-local statics of the firmware are not re-initialised by a reset (TMC4671.cpp:478 `static uint8_t
 * powerCheckCounter` in the waitPower state keeps its value between cases; the fixture cannot reset it). The state machine
 * sets it to 0 whenever power is absent, and every bring-up has power present before the first count, so it starts at 0.
 */
void IntegrationFixture::resetAll() {
	tud_deinit(0);
	HostDcd::reset();
	HostTarget::deleteCppmainMainclass();
	HostPlatform::reset(); // time 0, flash map cleared
	HostRtos::reset();
	HostTarget::reset();
	ErrorHandler::clearAll();
	SystemCommands::debugMode = false;
	SystemCommands::errorPrintingEnabled = true;
	CommandHandler::setLogsEnabled(true);
	ItPersistentStorageStartup::reset();
	UsbHidHandler::globalHidHandler = nullptr;
	UsbHidHandler::setHidDesc(nullptr);
	mainclass = nullptr;
	usb_device = nullptr;
	CDCcomm::clearRemainingBuffer(0);
	CDCcomm::cdcFinished(0); // gives the CDC semaphore back if an earlier case left a transfer unfinished
}

IntegrationFixture::IntegrationFixture() {
	resetAll();
	// Board. The target reset leaves SPI1_SS2 / SS3 low like MX_GPIO_Init (main.c): idle is high.
	HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port, SPI1_SS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_SS2_GPIO_Port, SPI1_SS2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_SS3_GPIO_Port, SPI1_SS3_Pin, GPIO_PIN_SET);
	for (FakeTmc4671 *c : {&chip1, &chip2}) {
		c->reset(); // power-on state (the chips were constructed before the global reset)
		c->setPresent(true);
		c->setAdcRaw(0, 0x7FFF7FFF); // I1 [31:16], I0 [15:0] at mid scale (DS 0x02): zero current offsets
		c->setMicrosPerTransfer(kSetupFrameUs);
	}
	FakeTmc4671::installResponder(); // HostTarget::reset removed it
	// Analog reference 4095 mV (VREFINT_CAL 1365, VREFINT reading 1100), as TmcFixture: one count = 1 mV at the pin
	HostTarget::setAdcCalibration(1365, 950, 1200);
	HostTarget::adcSetInput(&hadc1, 7, 1100);
	HostTarget::adcSetInput(&hadc1, 6, 950);
	sched.reset();
	atStart_ = HostRtos::objectCounts();
}

IntegrationFixture::~IntegrationFixture() {
	{
		const std::string plantFault = firstFault();
		INFO("first fault: " << plantFault);
		CHECK_MESSAGE(plantFault.empty(), "the chain recorded a fault (fake USB controller, simulated host, unmodelled HAL call, kernel fault, FakeTmc4671 or WheelPlant violation)");
	}
	detachPlant();
	if (tud_ready()) {
		// CDCcomm keeps a retry flag while a reply is only partly sent (see UsbFixture): do not leave data unsent
		CDCcomm::clearRemainingBuffer(0);
		CDCcomm::cdcFinished(0);
	}
	// Delete the main class (and with it the USBdevice, the axis, the TMC driver and their threads), then TinyUSB
	HostTarget::deleteCppmainMainclass();
	mainclass = nullptr;
	usb_device = nullptr;
	UsbHidHandler::globalHidHandler = nullptr;
	tud_deinit(0);
	// Faults raised while the main class and TinyUSB were torn down (a thread that blocks in a destructor, a USB fault, ...)
	const std::string teardownFault = firstFault();
	INFO("first fault during teardown: " << teardownFault);
	CHECK_MESSAGE(teardownFault.empty(), "the chain recorded a fault while the main class and TinyUSB were deleted");
	const HostRtos::ObjectCounts now = HostRtos::objectCounts();
	INFO("kernel objects at the start / end: tasks " << atStart_.tasks << "/" << now.tasks << ", queues " << atStart_.queues << "/" << now.queues
													 << ", semaphores " << atStart_.semaphores << "/" << now.semaphores << ", timers "
													 << atStart_.timers << "/" << now.timers << ", event groups " << atStart_.eventGroups << "/"
													 << now.eventGroups);
	// Equality, not <=: a missing object (deleted too early or never created) is as wrong as a leaked one
	CHECK_MESSAGE((now.tasks == atStart_.tasks && now.queues == atStart_.queues && now.semaphores == atStart_.semaphores &&
				   now.timers == atStart_.timers && now.eventGroups == atStart_.eventGroups),
				  "kernel object counts differ from the start of the case (the main class and its USBdevice are deleted before this check)");
	resetAll();
}

std::string IntegrationFixture::firstFault() const {
	if (!HostDcd::state().faults.empty()) {
		return "HostDcd: " + HostDcd::state().faults.front();
	}
	if (!host.errors().empty()) {
		return "UsbHost: " + host.errors().front();
	}
	if (!HostTarget::state().unmodelledHalCalls.empty()) {
		return "HostTarget: " + HostTarget::state().unmodelledHalCalls.front();
	}
	if (!HostRtos::kernelFaults().empty()) {
		return "HostRtos: " + HostRtos::kernelFaults().front();
	}
	if (plant_ != nullptr && !plant_->violations().empty()) {
		return "WheelPlant: " + plant_->violations().front();
	}
	for (const FakeTmc4671 *c : {&chip1, &chip2}) {
		const auto v = c->unexpectedViolations();
		if (!v.empty()) {
			return "FakeTmc4671 CS " + std::to_string(c->cs()) + ": " + v.front().text();
		}
	}
	return {};
}

void IntegrationFixture::setSupply(int32_t extMv, int32_t intMv) {
	supplyExt_ = extMv;
	supplyInt_ = intMv;
	auto counts = [](int32_t mv) { return static_cast<uint32_t>(std::lround(static_cast<double>(mv) / vSenseMult)); };
	REQUIRE(HostTarget::adcConvert(&hadc2, {counts(supplyExt_), counts(supplyInt_)}));
}

void IntegrationFixture::attachPlant(WheelPlant &plant, FakeTmc4671 &chip) {
	detachPlant();
	plant_ = &plant;
	plant.setDestroyHook([this] {
		sched.plantHook = nullptr;
		plant_ = nullptr; // the plant is being destroyed: forget it
	});
	plant.attach(chip, HostPlatform::nowUs());
	sched.plantHook = [&plant](uint64_t nowUs) { plant.stepToUs(nowUs); };
}

void IntegrationFixture::detachPlant() {
	sched.plantHook = nullptr;
	if (plant_ != nullptr) {
		plant_->detach();
		plant_->setDestroyHook(nullptr);
		plant_ = nullptr;
	}
}

void IntegrationFixture::setSpiFrameUs(uint32_t us) {
	chip1.setMicrosPerTransfer(us);
	chip2.setMicrosPerTransfer(us);
}

void IntegrationFixture::boot() {
	HostTarget::runCppmain(0); // the real cppmain(): chip id, flash, startADC, main class, usbInit
	REQUIRE(mainclass != nullptr);
	REQUIRE(usb_device != nullptr);
	powerOn(); // needs the ADC DMA that cppmain started
	sched.reset();
	sched.adoptThreads(); // before the first activation of any thread
	sched.pumpUsb();	  // the "USB" thread: tusb_init, then tud_task waits for the first event
	REQUIRE(HostDcd::state().initialised);
}

void IntegrationFixture::enumerateUsb() {
	REQUIRE(host.enumerate());
	REQUIRE(tud_mounted());
	REQUIRE(host.cdcSetControlLineState(true, true).ok());
	INFO("first fault: " << firstFault());
	REQUIRE(firstFault().empty());
}

uint16_t IntegrationFixture::hidReportDescriptorLength() const {
	const UsbHost::Interface *h = host.findInterface(TUSB_CLASS_HID);
	return h ? h->hidReportLength : 0;
}

std::vector<uint8_t> IntegrationFixture::readHidReportDescriptor() {
	const uint16_t wLength = hidReportDescriptorLength();
	REQUIRE(wLength > 0);
	const UsbHost::ControlResult r = host.hidGetReportDescriptor();
	INFO("GET_DESCRIPTOR(report) status stage '" << r.stage << "'");
	REQUIRE(r.ok());
	REQUIRE(r.data.size() == wLength);
	return r.data;
}

void IntegrationFixture::drainHidIn() {
	// Host behaviour: a host's HID driver keeps polling the interrupt IN endpoint (every bInterval) and takes every report, so
	// the default is to drain it in cdcCommand (drainHid = true). If nobody reads it, the HID command interface thread waits in
	// Delay(1) for the endpoint while the command thread appends further replies to the vector it is iterating: a use after
	// free in the firmware, IB 3 (HidCommandInterface.cpp:39-46 and :159). The dedicated case in test_f1_setup.cpp sets
	// drainHid = false to show it; every other case drains.
	for (uint32_t i = 0; i < 8; i++) {
		HostDcd::Handshake h = HostDcd::Handshake::Nak;
		host.hidRead(&h);
		if (h != HostDcd::Handshake::Ack) {
			break;
		}
	}
}

std::string IntegrationFixture::cdcCommand(const std::string &line, uint32_t maxMs) {
	REQUIRE(host.cdcWrite(line + "\n") == HostDcd::Handshake::Ack);
	std::string reply;
	for (uint32_t ms = 0; ms < maxMs; ms++) {
		sched.advanceMs(1);
		if (drainHid) {
			drainHidIn();
		}
		reply += host.cdcRead();
		if (!reply.empty() && reply.back() == '\n') {
			break;
		}
	}
	return reply;
}

void IntegrationFixture::selectDriver(uint32_t classId) {
	const std::string reply = cdcCommand("axis.0.drvtype=" + std::to_string(classId));
	INFO("reply: " << reply);
	REQUIRE(!reply.empty());
}

TMC4671 *IntegrationFixture::tmc() {
	CommandHandler *h = CommandHandler::getHandlerFromClassName("tmc", 0);
	return dynamic_cast<TMC4671 *>(h);
}

bool IntegrationFixture::runUntilTmcState(TMC_ControlState state, uint32_t maxMs) {
	for (uint32_t ms = 0; ms < maxMs; ms++) {
		TMC4671 *t = tmc();
		if (t != nullptr && t->getState() == state) {
			return true;
		}
		sched.advanceMs(1);
	}
	TMC4671 *t = tmc();
	return t != nullptr && t->getState() == state;
}

void IntegrationFixture::replugUsb() {
	HostDcd::detach();
	sched.pumpUsb(); // tud_umount_cb -> usbSuspend (Axis::usbSuspend: turn(0), stopMotor)
	HostDcd::attach();
	enumerateUsb();	 // tud_mount_cb -> usbResume (Axis::usbResume: startMotor)
}

void IntegrationFixture::reachShutdown(uint32_t budgetMs) {
	boot();
	enumerateUsb();
	startUs_ = HostPlatform::nowUs();
	selectDriver(1); // CLSID_MOT_TMC0 = TMC_1 (MotorDriver.cpp:28 add_class<TMC_1, MotorDriver>(1))
	REQUIRE(tmc() != nullptr);
	// The driver initialises (ping, constants, power, ADC calibration, encoder type NONE) and stops in Shutdown (INR 1: a fresh
	// flash has motor type NONE and encoder NONE, so EncoderFinished is never used). The start request the axis made at set-up
	// (Axis::setDrvType: startMotor) came while the state machine was still uninitialised and does not move it out of Shutdown.
	REQUIRE_MESSAGE(runUntilTmcState(TMC_ControlState::Shutdown, budgetMs), "reachShutdown: Shutdown not reached, TMC state " << static_cast<uint32_t>(tmc()->getState()));
}

void IntegrationFixture::mountAndRun(uint32_t budgetMs) {
	// Board configured, PC connects afterwards: the next startMotor() is the one of the USB mount (tud_mount_cb ->
	// Axis::usbResume -> startMotor, global_callbacks.cpp:375, Axis.cpp:639), as when the PC connects after the board finished
	// its start-up: the host plugs the device in again.
	startMark = chip1.log().size();
	replugUsb();
	bool up = false;
	for (uint32_t ms = 0; ms < budgetMs && !up; ms++) {
		TMC4671 *t = tmc();
		up = t->getState() == TMC_ControlState::Running && t->motorReady();
		if (!up) {
			sched.advanceMs(1);
		}
	}
	std::ostringstream s;
	s << "bringUp: TMC state " << static_cast<uint32_t>(tmc()->getState()) << " after " << (HostPlatform::nowUs() - startUs_) / 1000 << " ms simulated";
	REQUIRE_MESSAGE(up, s.str());
	bringUpMs = static_cast<uint32_t>((HostPlatform::nowUs() - startUs_) / 1000);
}

uint32_t IntegrationFixture::bringUp(uint32_t budgetMs) {
	reachShutdown(budgetMs);
	mountAndRun(budgetMs);
	return bringUpMs;
}

bool IntegrationFixture::isFfbWheel() const { return mainclass != nullptr && mainclass->getInfo().id == CLSID_MAIN_FFBWHEEL; }

std::vector<IntegrationFixture::FluxTorque> IntegrationFixture::fluxTorqueWrites(const FakeTmc4671 &chip) {
	std::vector<FluxTorque> out;
	for (uint32_t w : chip.writesTo(0x64)) {
		FluxTorque ft;
		ft.flux = static_cast<int16_t>(w & 0xFFFF);
		ft.torque = static_cast<int16_t>((w >> 16) & 0xFFFF);
		out.push_back(ft);
	}
	return out;
}
