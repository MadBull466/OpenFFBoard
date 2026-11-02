/*
 * host_dcd.cpp (ffb_usb_tests only)
 *
 * F15: fake USB device controller. Implements the TinyUSB DCD port API (FFBoard/USB/device/dcd.h) on a model of the
 * F407 OTG_FS core as TinyUSB's DWC2 port drives it (FFBoard/USB/portable/synopsys/dwc2/dcd_dwc2.c, cited as
 * "dwc2:<line>") and of the USB 2.0 bus. See support/HostDcd.h for the model and the test control.
 */
#include "HostDcd.h"

#include "device/dcd.h"

#include <algorithm>
#include <cstring>
#include <sstream>

extern uint32_t hosttest_ipsr; // host_platform.cpp: interrupt program status (0 = thread mode)

namespace {

constexpr uint8_t EP_COUNT = 4;			// stm32f407xx.h USB_OTG_FS_MAX_IN_ENDPOINTS (dwc2_stm32.h EP_MAX_FS)
constexpr uint16_t EP0_SIZE = 64;		// dwc2:319-323 "Fixed both control EP0 size to 64 bytes"
constexpr uint32_t OTG_FS_IPSR = 67 + 16; // OTG_FS_IRQn (stm32f407xx.h) + 16: IPSR while the USB interrupt runs

HostDcd::State dcd;
HostDcd::Endpoint endpoints[EP_COUNT][2]; // [number][tusb_dir_t]
HostDcd::Endpoint invalidEndpoint;		  // returned for endpoint numbers the core does not have
std::vector<dcd_event_t> heldEvents;	  // raised while the interrupt was disabled

HostDcd::Endpoint &ep(uint8_t epAddr) {
	const uint8_t num = tu_edpt_number(epAddr);
	if (num >= EP_COUNT) {
		invalidEndpoint = HostDcd::Endpoint();
		return invalidEndpoint;
	}
	return endpoints[num][tu_edpt_dir(epAddr)];
}

std::string hex(uint8_t v) {
	std::ostringstream s;
	s << "0x" << std::hex << (int)v;
	return s.str();
}

void fault(const std::string &what) { dcd.faults.push_back(what); }

/**
 * Raises an event to the stack like the DWC2 interrupt handler does (dcd_event_handler with in_isr = true, in the
 * OTG_FS interrupt). While the interrupt is disabled (dcd_int_disable) the event is held until dcd_int_enable.
 */
void raise(const dcd_event_t &event) {
	if (!dcd.interruptsEnabled) {
		heldEvents.push_back(event);
		return;
	}
	dcd.events.push_back(event);
	const uint32_t savedIpsr = hosttest_ipsr;
	hosttest_ipsr = OTG_FS_IPSR;
	dcd_event_handler(&event, true);
	hosttest_ipsr = savedIpsr;
}

dcd_event_t event(dcd_eventid_t id) {
	dcd_event_t e;
	std::memset(&e, 0, sizeof(e));
	e.rhport = 0;
	e.event_id = (uint8_t)id;
	return e;
}

void complete(uint8_t epAddr) {
	HostDcd::Endpoint &e = ep(epAddr);
	e.armed = false;
	e.completions++;
	dcd_event_t ev = event(DCD_EVENT_XFER_COMPLETE);
	ev.xfer_complete.ep_addr = epAddr;
	ev.xfer_complete.len = e.done;
	ev.xfer_complete.result = XFER_RESULT_SUCCESS;
	raise(ev);
}

void dropTransfer(HostDcd::Endpoint &e) {
	e.armed = false;
	e.buffer = nullptr;
	e.length = 0;
	e.done = 0;
}

/** Bus reset of the core (dwc2:286-334): transfers dropped, SOF interrupt off, address 0, only EP0 enabled (64 bytes) */
void coreBusReset() {
	for (auto &num : endpoints) {
		for (auto &e : num) {
			const uint32_t transfers = e.transfers;
			const uint32_t completions = e.completions;
			e = HostDcd::Endpoint();
			e.transfers = transfers;
			e.completions = completions;
		}
	}
	for (auto &e : endpoints[0]) {
		e.enabled = true;
		e.type = TUSB_XFER_CONTROL;
		e.maxPacket = EP0_SIZE;
	}
	dcd.sofInterrupt = false; // dwc2:291 _sof_en = false
	dcd.sofForWakeup = false;
	dcd.address = 0;		  // dwc2:316 DCFG.DAD = 0
	dcd.pendingAddress = -1;
}

bool visible() { return dcd.attached && dcd.pullup; }

/** Common checks of a transaction. @return true if the device answers */
bool answers(uint8_t address) { return HostDcd::reachable() && address == dcd.address; }

void record(HostDcd::Packet::Pid pid, uint8_t address, uint8_t epAddr, const std::vector<uint8_t> &data, bool data1, HostDcd::Handshake h) {
	HostDcd::Packet p;
	p.pid = pid;
	p.address = address;
	p.ep = epAddr;
	p.data = data;
	p.data1 = data1;
	p.handshake = h;
	dcd.packets.push_back(p);
}

/** The status stage of SET_ADDRESS is complete: the new address takes effect (USB 2.0 9.4.6) */
void applyPendingAddress() {
	if (dcd.pendingAddress >= 0) {
		dcd.address = (uint8_t)dcd.pendingAddress;
		dcd.pendingAddress = -1;
	}
}

} // namespace

namespace HostDcd {

const char *handshakeName(Handshake h) {
	switch (h) {
	case Handshake::Ack:
		return "ACK";
	case Handshake::Nak:
		return "NAK";
	case Handshake::Stall:
		return "STALL";
	case Handshake::NoResponse:
		return "no response";
	}
	return "?";
}

const State &state() { return dcd; }
const Endpoint &endpoint(uint8_t epAddr) { return ep(epAddr); }

void reset() {
	dcd = State();
	for (auto &num : endpoints) {
		for (auto &e : num) {
			e = Endpoint();
		}
	}
	heldEvents.clear();
}

void recordFault(const std::string &what) { fault(what); }

std::vector<std::string> takeFaults() {
	std::vector<std::string> f;
	f.swap(dcd.faults);
	return f;
}

bool reachable() { return visible() && dcd.resetSinceConnect; }

void attach() { dcd.attached = true; }

void detach() {
	if (!dcd.attached) {
		return;
	}
	dcd.attached = false;
	dcd.resetSinceConnect = false;
	dcd.suspended = false;
	// OTG session end (GOTGINT.SEDET, dwc2:960-967) -> DCD_EVENT_UNPLUGGED
	raise(event(DCD_EVENT_UNPLUGGED));
}

void busReset() {
	if (!visible()) {
		return; // No device on the bus to reset
	}
	dcd.resetSinceConnect = true;
	dcd.suspended = false;
	coreBusReset(); // USBRST (dwc2:906-910)
	// ENUMDNE (dwc2:912-935): full speed (the F407 OTG_FS has only the full-speed PHY)
	dcd_event_t e = event(DCD_EVENT_BUS_RESET);
	e.bus_reset.speed = TUSB_SPEED_FULL;
	raise(e);
}

void suspend() {
	if (!visible()) {
		return;
	}
	dcd.suspended = true;
	raise(event(DCD_EVENT_SUSPEND)); // USBSUSP (dwc2:937-940)
}

void resume() {
	if (!visible()) {
		return;
	}
	dcd.suspended = false;
	raise(event(DCD_EVENT_RESUME)); // WKUINT (dwc2:942-945)
}

void sof() {
	if (!reachable()) {
		return; // No SOF reaches a device that is not on the bus
	}
	dcd.suspended = false;
	dcd.frame = (dcd.frame + 1) & 0x7FF; // 11-bit frame number (USB 2.0 8.4.3)
	// dwc2:969-979: SOF interrupt if enabled by dcd_sof_enable or once after dcd_remote_wakeup
	if (dcd.sofInterrupt || dcd.sofForWakeup) {
		dcd.sofForWakeup = false;
		dcd_event_t e = event(DCD_EVENT_SOF);
		e.sof.frame_count = dcd.frame;
		raise(e);
	}
}

Handshake setup(uint8_t address, const tusb_control_request_t &request) {
	std::vector<uint8_t> bytes(sizeof(request));
	std::memcpy(bytes.data(), &request, sizeof(request));
	if (!answers(address)) {
		record(Packet::Pid::Setup, address, 0, bytes, false, Handshake::NoResponse);
		return Handshake::NoResponse;
	}
	// A SETUP is always accepted (USB 2.0 8.5.3), clears a halt of the control endpoint (8.5.3.4) and starts a new
	// control transfer: pending EP0 transfers are dropped. SETUP is DATA0, the first data / status packet DATA1 (8.5.3).
	for (auto &e : endpoints[0]) {
		e.stalled = false;
		dropTransfer(e);
		e.data1 = true;
	}
	dcd.pendingAddress = -1; // A SET_ADDRESS whose status stage did not complete has no effect
	record(Packet::Pid::Setup, address, 0, bytes, false, Handshake::Ack);
	dcd_event_t e = event(DCD_EVENT_SETUP_RECEIVED);
	std::memcpy(&e.setup_received, &request, sizeof(request));
	raise(e);
	return Handshake::Ack;
}

Handshake out(uint8_t address, uint8_t epNum, const std::vector<uint8_t> &data) {
	const uint8_t epAddr = epNum & 0x0F;
	Endpoint &e = ep(epAddr);
	Handshake h;
	if (!answers(address) || epNum >= EP_COUNT || !e.enabled) {
		h = Handshake::NoResponse;
	} else if (e.stalled) {
		h = Handshake::Stall;
	} else if (!e.armed) {
		h = Handshake::Nak;
	} else if (epNum == 0 && e.length == 0 && !data.empty()) {
		// Status stage of a control read with data: DCFG.NZLSOHSK (dwc2:444) answers STALL and discards the packet
		h = Handshake::Stall;
	} else {
		h = Handshake::Ack;
	}
	record(Packet::Pid::Out, address, epAddr, data, e.data1, h);
	if (h != Handshake::Ack) {
		return h;
	}
	if (data.size() > e.maxPacket) {
		fault("host sent " + std::to_string(data.size()) + " bytes to OUT endpoint " + hex(epAddr) + " with wMaxPacketSize " +
			  std::to_string(e.maxPacket) + " (babble, not modelled)");
		return Handshake::NoResponse;
	}
	if (data.size() > (size_t)(e.length - e.done)) {
		fault("host sent " + std::to_string(data.size()) + " bytes to OUT endpoint " + hex(epAddr) + " with only " +
			  std::to_string(e.length - e.done) + " bytes left in the armed transfer (overrun, not modelled)");
		return Handshake::NoResponse;
	}
	if (!data.empty()) {
		std::memcpy(e.buffer + e.done, data.data(), data.size());
	}
	e.done += (uint16_t)data.size();
	e.data1 = !e.data1;
	// A transfer ends with a short packet (also a ZLP) or when the requested length is reached (USB 2.0 5.8.3)
	if (data.size() < e.maxPacket || e.done == e.length) {
		complete(epAddr);
	}
	return Handshake::Ack;
}

Handshake in(uint8_t address, uint8_t epNum, std::vector<uint8_t> &data) {
	data.clear();
	const uint8_t epAddr = (uint8_t)(epNum | TUSB_DIR_IN_MASK);
	Endpoint &e = ep(epAddr);
	Handshake h;
	if (!answers(address) || epNum >= EP_COUNT || !e.enabled) {
		h = Handshake::NoResponse;
	} else if (e.stalled) {
		h = Handshake::Stall;
	} else if (!e.armed) {
		h = Handshake::Nak;
	} else {
		h = Handshake::Ack;
		const uint16_t n = (uint16_t)std::min<int>(e.maxPacket, e.length - e.done);
		if (n > 0) {
			data.assign(e.buffer + e.done, e.buffer + e.done + n);
		}
	}
	record(Packet::Pid::In, address, epAddr, data, e.data1, h);
	if (h != Handshake::Ack) {
		return h;
	}
	e.done += (uint16_t)data.size();
	e.data1 = !e.data1;
	// ceil(n / wMaxPacketSize) packets, one ZLP for n = 0 (dwc2:612-618)
	if (e.done == e.length) {
		const bool setAddressStatus = epNum == 0 && e.length == 0 && dcd.pendingAddress >= 0;
		complete(epAddr);
		if (setAddressStatus) {
			applyPendingAddress();
		}
	}
	return Handshake::Ack;
}

} // namespace HostDcd

// ---------------- TinyUSB DCD port API (device/dcd.h) ----------------
extern "C" {

/** dwc2:409-461: core init, dcd_disconnect, full speed (DCFG.DSPD FS), interrupts, dcd_connect */
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t *rh_init) {
	dcd.initCalls++;
	if (rhport != 0) {
		fault("dcd_init on rhport " + std::to_string(rhport) + " (the F407 has only rhport 0)");
		return false;
	}
	if (rh_init != nullptr && rh_init->speed == TUSB_SPEED_HIGH) {
		fault("dcd_init with high speed (not modelled: the F407 OTG_FS core is full speed only)");
	}
	dcd.initialised = true;
	dcd_disconnect(rhport);
	dcd_connect(rhport);
	return true;
}

/** dwc2_dcd_int_enable: NVIC enable of OTG_FS. Held events are raised now */
void dcd_int_enable(uint8_t) {
	dcd.interruptsEnabled = true;
	std::vector<dcd_event_t> held;
	held.swap(heldEvents);
	for (const dcd_event_t &e : held) {
		raise(e);
	}
}

void dcd_int_disable(uint8_t) { dcd.interruptsEnabled = false; }

/** Not modelled: there is no interrupt source; the fake raises the events itself */
void dcd_int_handler(uint8_t) { fault("dcd_int_handler called (not modelled)"); }

/** dwc2:472-478: new address, then the status-stage ZLP on EP0 IN. The address takes effect after that ZLP */
void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
	dcd.setAddressCalls.push_back(dev_addr);
	dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), nullptr, 0);
	dcd.pendingAddress = dev_addr; // after the xfer: dcd_edpt_xfer does not clear it
}

/** dwc2:480-496: remote wakeup signalling; enables the SOF interrupt to detect the resume of the bus */
void dcd_remote_wakeup(uint8_t) {
	dcd.remoteWakeups++;
	if (!dcd.suspended) {
		fault("dcd_remote_wakeup while the bus is not suspended (USB 2.0 7.1.7.7)");
	}
	dcd.sofForWakeup = true;
}

/** dwc2:498-513: D+ pull-up on (DCTL.SDIS cleared) */
void dcd_connect(uint8_t) {
	dcd.connectCalls++;
	if (!dcd.pullup) {
		dcd.resetSinceConnect = false; // The host resets a newly connected device first
	}
	dcd.pullup = true;
}

/** dwc2:515-531: D+ pull-up off (DCTL.SDIS set): the device disappears from the bus */
void dcd_disconnect(uint8_t) {
	dcd.disconnectCalls++;
	dcd.pullup = false;
	dcd.resetSinceConnect = false;
	dcd.suspended = false;
}

/** dwc2:533-546 */
void dcd_sof_enable(uint8_t, bool en) { dcd.sofInterrupt = en; }

/** dwc2:551-556 (dfifo_alloc: epnum < ep_count; edpt_activate :212-233: MPS, type, DATA0) */
bool dcd_edpt_open(uint8_t, tusb_desc_endpoint_t const *desc_ep) {
	const uint8_t epAddr = desc_ep->bEndpointAddress;
	if (tu_edpt_number(epAddr) >= EP_COUNT) {
		fault("dcd_edpt_open of endpoint " + hex(epAddr) + ": the F407 OTG_FS core has 4 endpoints per direction");
		return false;
	}
	if (desc_ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
		fault("dcd_edpt_open of the isochronous endpoint " + hex(epAddr) + " (not modelled)");
		return false;
	}
	HostDcd::Endpoint &e = ep(epAddr);
	e.enabled = true;
	e.type = desc_ep->bmAttributes.xfer;
	e.maxPacket = tu_edpt_packet_size(desc_ep);
	e.interval = desc_ep->bInterval;
	e.stalled = false;
	e.data1 = false;
	dropTransfer(e);
	return true;
}

/** dwc2:558-581: endpoints 1..3 disabled, transfers cancelled (no completion events) */
void dcd_edpt_close_all(uint8_t) {
	dcd.closeAllCalls++;
	for (uint8_t n = 1; n < EP_COUNT; n++) {
		for (auto &e : endpoints[n]) {
			e.enabled = false;
			e.stalled = false;
			dropTransfer(e);
		}
	}
}

/** dwc2:583-593: not modelled (only isochronous endpoints, none in the firmware's configurations) */
bool dcd_edpt_iso_alloc(uint8_t, uint8_t ep_addr, uint16_t) {
	fault("dcd_edpt_iso_alloc of endpoint " + hex(ep_addr) + " (not modelled)");
	return false;
}
bool dcd_edpt_iso_activate(uint8_t, tusb_desc_endpoint_t const *desc_ep) {
	fault("dcd_edpt_iso_activate of endpoint " + hex(desc_ep->bEndpointAddress) + " (not modelled)");
	return false;
}

/** dwc2:595-624: arms a transfer of total_bytes (IN: sent when the host reads, OUT: filled when the host writes) */
bool dcd_edpt_xfer(uint8_t, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes) {
	HostDcd::Endpoint &e = ep(ep_addr);
	if (tu_edpt_number(ep_addr) >= EP_COUNT || !e.enabled) {
		fault("dcd_edpt_xfer on the closed endpoint " + hex(ep_addr) + " (never completes)");
		return true; // dwc2 does not check: the transfer never happens
	}
	if (e.armed) {
		fault("dcd_edpt_xfer on endpoint " + hex(ep_addr) + " while a transfer is armed (dwc2 overwrites it)");
	}
	if (buffer == nullptr && total_bytes > 0) {
		fault("dcd_edpt_xfer on endpoint " + hex(ep_addr) + " with a NULL buffer and " + std::to_string(total_bytes) + " bytes");
		return false;
	}
	e.transfers++;
	e.armed = true;
	e.buffer = buffer;
	e.length = total_bytes;
	e.done = 0;
	return true;
}

/** dwc2:627-651: not modelled (only used by isochronous / audio classes) */
bool dcd_edpt_xfer_fifo(uint8_t, uint8_t ep_addr, tu_fifo_t *, uint16_t) {
	fault("dcd_edpt_xfer_fifo on endpoint " + hex(ep_addr) + " (not modelled)");
	return false;
}

/** dwc2:657-663 (edpt_disable with stall :235-283): halted, the armed transfer is dropped without completion */
void dcd_edpt_stall(uint8_t, uint8_t ep_addr) {
	HostDcd::Endpoint &e = ep(ep_addr);
	e.stalled = true;
	dropTransfer(e);
}

/** dwc2:665-675: halt cleared, data toggle reset to DATA0. A transfer dropped by the stall stays dropped */
void dcd_edpt_clear_stall(uint8_t, uint8_t ep_addr) {
	HostDcd::Endpoint &e = ep(ep_addr);
	e.stalled = false;
	e.data1 = false;
}

} // extern "C"
