/*
 * HostDcd.h
 *
 * F15: fake USB device controller of ffb_usb_tests (implementation: platform/src/host_dcd.cpp). It implements the
 * TinyUSB device controller port API (FFBoard/USB/device/dcd.h) for the real TinyUSB device stack (usbd.c,
 * usbd_control.c, the CDC / HID / MIDI class drivers) and lets a test play the USB host and the bus.
 *
 * Reference model: the DCD of the F407 target, TinyUSB's Synopsys DWC2 port
 * (FFBoard/USB/portable/synopsys/dwc2/dcd_dwc2.c, not compiled), and USB 2.0 (chapters 5, 8, 9). Each function of
 * host_dcd.cpp cites the lines it models. Summary:
 *
 *  - Controller: dcd_init (dcd_dwc2.c:409-461) disconnects, configures full speed and connects the pull-up; a
 *    high-speed request is recorded as a fault (the F407 OTG_FS core is full speed only). dcd_int_enable /
 *    dcd_int_disable: while disabled, events are held and delivered (in order) when enabled again. dcd_connect /
 *    dcd_disconnect switch the D+ pull-up (dcd_dwc2.c:498-531). dcd_deinit is not defined (the DWC2 port of this
 *    TinyUSB version has none; usbd.c:91 weak default). Endpoints 0..3 per direction (stm32f407xx.h
 *    USB_OTG_FS_MAX_IN_ENDPOINTS 4; dcd_dwc2.c dfifo_alloc asserts epnum < ep_count).
 *  - Bus (driven by the test): attach / detach of the cable (VBUS session; detach raises DCD_EVENT_UNPLUGGED like
 *    the OTG session-end interrupt, dcd_dwc2.c:960-967), bus reset (bus_reset dcd_dwc2.c:286-334 + ENUMDNE :911-935:
 *    address 0, non-control endpoints disabled, transfers dropped, SOF interrupt off, EP0 64 bytes, then
 *    DCD_EVENT_BUS_RESET full speed), suspend (USB 2.0 7.1.7.6, USBSUSP -> DCD_EVENT_SUSPEND), host resume (WKUINT ->
 *    DCD_EVENT_RESUME), SOF (DCD_EVENT_SOF only while the SOF interrupt is enabled by dcd_sof_enable or for one SOF
 *    after dcd_remote_wakeup, dcd_dwc2.c:480-496, :969-979).
 *  - The device is visible on the bus only while the cable is attached and the pull-up is on, and answers only after
 *    a bus reset following the connection (a host resets a newly connected device before talking to it, USB 2.0
 *    9.1.2). Otherwise every transaction gets no response (Handshake::NoResponse), like one to a wrong address.
 *  - SET_ADDRESS timing (USB 2.0 9.4.6, 9.2.6.3): dcd_set_address queues the status-stage ZLP on EP0 IN like
 *    dcd_dwc2.c:472-478; the new address takes effect when the host has received that ZLP. Until then the device
 *    answers at its old address. (The DWC2 writes DCFG.DAD at once and the core keeps the old address for the
 *    status stage; the observable behaviour is the same.)
 *  - Transactions (USB 2.0 8.4.6, 8.5): setup() is always accepted (8.5.3, a device cannot NAK a SETUP); it clears
 *    a STALL of EP0 (8.5.3.4) and drops pending EP0 transfers, then raises DCD_EVENT_SETUP_RECEIVED.
 *    out() delivers one OUT data packet: STALL if the endpoint is halted, NAK if no transfer is armed (dcd_edpt_xfer),
 *    else the bytes are copied into the armed buffer; the transfer completes (DCD_EVENT_XFER_COMPLETE with the bytes
 *    received) after a short packet (including a ZLP) or when the requested length is reached (5.8.3, 8.5.2; the DWC2
 *    XFRC interrupt). in() reads one IN data packet: STALL / NAK like above, else up to wMaxPacketSize bytes of the
 *    armed buffer; a transfer of n bytes is ceil(n / wMaxPacketSize) packets, a 0-byte transfer is one ZLP, no
 *    automatic ZLP after a full last packet (dcd_dwc2.c:595-624: num_packets; the stack queues ZLPs itself).
 *    A non-zero-length OUT packet in the status stage of a control read (an armed 0-byte EP0 OUT transfer) is
 *    answered with STALL (DCFG.NZLSOHSK set by dcd_dwc2.c:444).
 *  - Endpoints: dcd_edpt_open (wMaxPacketSize, type, DATA0), dcd_edpt_close_all (endpoints 1..3 disabled, transfers
 *    dropped, dcd_dwc2.c:558-581), dcd_edpt_stall (halt, the armed transfer is dropped: edpt_disable :235-283),
 *    dcd_edpt_clear_stall (halt cleared, data toggle DATA0, :665-675; an armed transfer stays dropped).
 *    Data toggles are tracked per endpoint and recorded with each packet (DATA0 / DATA1); they are not checked.
 *  - Events are raised "in the USB interrupt": hosttest_ipsr = 83 (OTG_FS_IRQn 67 + 16) and in_isr = true, so they
 *    go through xQueueSendToBackFromISR into TinyUSB's event queue. TinyUSB processes them in tud_task.
 *
 * Not modelled (recorded in faults(), which fails the UsbFixture test): high speed, isochronous endpoints
 * (dcd_edpt_iso_alloc / _activate), dcd_edpt_xfer_fifo, dcd_int_handler (no interrupt source: the fake raises the
 * events directly), test mode, a transfer armed on a closed endpoint or on an endpoint with an armed transfer, a
 * packet larger than wMaxPacketSize or larger than the rest of the armed transfer, dcd_remote_wakeup while the bus is
 * not suspended (USB 2.0 7.1.7.7), more than 4 endpoints per direction. Also not modelled: the FIFO RAM allocation
 * (1.25 KB of the F407 OTG_FS core), DMA, bus timing (suspend after 3 ms, resume 20 ms, remote wakeup 1-15 ms),
 * babble, CRC / bit errors, low speed, LPM.
 */
#ifndef HOSTTEST_HOSTDCD_H_
#define HOSTTEST_HOSTDCD_H_

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include "tusb.h"
#include "device/dcd.h"

namespace HostDcd {

/** Answer of the device to one transaction (USB 2.0 8.4.5) */
enum class Handshake {
	Ack,		//!< Data accepted (OUT / SETUP) or sent (IN)
	Nak,		//!< No transfer armed on the endpoint: the host retries later
	Stall,		//!< Endpoint halted, or request not supported (EP0)
	NoResponse, //!< Device not on the bus, not reset yet, other address, or endpoint not enabled (timeout)
};
const char *handshakeName(Handshake h);
inline std::ostream &operator<<(std::ostream &os, Handshake h) { return os << handshakeName(h); }

/** One packet that went over the bus, in the order of the bus */
struct Packet {
	enum class Pid { Setup, Out, In };
	Pid pid = Pid::Out;
	uint8_t address = 0;	   //!< Device address used by the host
	uint8_t ep = 0;			   //!< Endpoint address (with direction bit for IN)
	std::vector<uint8_t> data; //!< Payload (SETUP: the 8 request bytes)
	bool data1 = false;		   //!< Data toggle of the data packet (DATA1 if true)
	Handshake handshake = Handshake::Ack;
};

/** State of one endpoint direction of the controller */
struct Endpoint {
	bool enabled = false;		 //!< Opened (dcd_edpt_open), EP0 after a bus reset
	uint8_t type = 0;			 //!< tusb_xfer_type_t
	uint16_t maxPacket = 0;		 //!< wMaxPacketSize
	uint8_t interval = 0;		 //!< bInterval
	bool stalled = false;		 //!< Halted (dcd_edpt_stall)
	bool armed = false;			 //!< A transfer is armed (dcd_edpt_xfer) and not complete
	uint8_t *buffer = nullptr;	 //!< Buffer of the armed transfer
	uint16_t length = 0;		 //!< Length of the armed transfer
	uint16_t done = 0;			 //!< Bytes moved so far
	bool data1 = false;			 //!< Next data toggle
	uint32_t transfers = 0;		 //!< dcd_edpt_xfer calls (since the controller reset)
	uint32_t completions = 0;	 //!< DCD_EVENT_XFER_COMPLETE raised
};

struct State {
	// Controller (dcd_*)
	bool initialised = false;	 //!< dcd_init called
	uint32_t initCalls = 0;		 //!< dcd_init calls
	bool interruptsEnabled = false; //!< dcd_int_enable / dcd_int_disable
	bool pullup = false;		 //!< D+ pull-up (dcd_connect / dcd_disconnect)
	uint32_t connectCalls = 0;
	uint32_t disconnectCalls = 0;
	bool sofInterrupt = false;	 //!< dcd_sof_enable
	bool sofForWakeup = false;	 //!< SOF interrupt enabled once by dcd_remote_wakeup
	uint32_t remoteWakeups = 0;	 //!< dcd_remote_wakeup calls (remote wakeup signalling)
	uint8_t address = 0;		 //!< Address the device answers to
	int pendingAddress = -1;	 //!< dcd_set_address value waiting for the status stage, -1 = none
	std::vector<uint8_t> setAddressCalls; //!< dcd_set_address arguments
	uint32_t closeAllCalls = 0;	 //!< dcd_edpt_close_all calls
	// Bus
	bool attached = true;		 //!< Cable plugged (VBUS)
	bool resetSinceConnect = false; //!< A bus reset happened since the device appeared on the bus
	bool suspended = false;		 //!< Bus suspended (suspend() until resume() / SOF / bus reset)
	uint32_t frame = 0;			 //!< Frame number of the last SOF
	std::vector<Packet> packets; //!< Every transaction, in bus order
	std::vector<dcd_event_t> events; //!< Every event raised to TinyUSB (dcd_event_handler calls), in order
	/** Unmodelled calls and USB protocol violations by the device. UsbFixture fails the test if not empty */
	std::vector<std::string> faults;
};

const State &state();
/** State of an endpoint direction (ep address with the direction bit, e.g. 0x81) */
const Endpoint &endpoint(uint8_t epAddr);

/** Power-on state of the controller and of the bus (cable attached), records cleared. TinyUSB is not touched */
void reset();
/** Returns and clears faults() */
std::vector<std::string> takeFaults();
/** Adds an entry to faults() (also used by the OSAL additions in host_usb_osal.cpp) */
void recordFault(const std::string &what);

// ---------------- Bus (the test acts as cable and host) ----------------
/** True if the host can talk to the device: cable attached, pull-up on and a bus reset since the connection */
bool reachable();
/** Plugs the cable. The host sees the device if the pull-up is on (it must reset it before the first transaction) */
void attach();
/** Unplugs the cable: DCD_EVENT_UNPLUGGED (OTG session end), the device is gone for the host */
void detach();
/** USB reset by the host (USB 2.0 7.1.7.5): needs the cable and the pull-up, see the file comment */
void busReset();
/** Bus idle for more than 3 ms (7.1.7.6): DCD_EVENT_SUSPEND */
void suspend();
/** Resume signalling by the host (7.1.7.7): DCD_EVENT_RESUME */
void resume();
/** Start of frame (frame number + 1): ends a suspend; DCD_EVENT_SOF if the SOF interrupt is enabled (see above) */
void sof();

// ---------------- Transactions (one packet each) ----------------
/** SETUP transaction to EP0 at `address` */
Handshake setup(uint8_t address, const tusb_control_request_t &request);
/** OUT transaction with one data packet to endpoint `epNum` (0..15) */
Handshake out(uint8_t address, uint8_t epNum, const std::vector<uint8_t> &data);
/** IN transaction from endpoint `epNum`: on Ack `data` holds the packet (may be empty: ZLP) */
Handshake in(uint8_t address, uint8_t epNum, std::vector<uint8_t> &data);

} // namespace HostDcd

#endif
