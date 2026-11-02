/*
 * UsbHost.h
 *
 * A simulated USB host for ffb_usb_tests: control transfers (USB 2.0 8.5.3: SETUP, data stage, status stage),
 * enumeration, bulk / interrupt transfers and helpers for the CDC ACM and HID classes, on top of the transactions of
 * the fake device controller (support/HostDcd.h).
 *
 * Like a real host it retries a transaction the device answers with NAK. Between retries it calls `pump`, which lets
 * the device process its events (UsbFixture::process: tud_task and the firmware threads); after maxRetries NAKs the
 * transfer ends with Handshake::Nak (a host timeout). STALL and no response end a transfer at once.
 * Protocol errors of the device seen by the host (a data packet in the IN status stage, more data than requested) are
 * recorded in errors(); UsbFixture fails the test if any is left.
 */
#ifndef HOSTTEST_USBHOST_H_
#define HOSTTEST_USBHOST_H_

#include "HostDcd.h"

#include "tusb.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

class UsbHost {
public:
	using Handshake = HostDcd::Handshake;

	explicit UsbHost(std::function<void()> pump) : pump(std::move(pump)) {}

	uint8_t address = 0;	  //!< Address the host uses for the device
	uint32_t maxRetries = 20; //!< NAKs (each followed by pump) before a transaction times out

	/** Result of a control transfer */
	struct ControlResult {
		Handshake status = Handshake::NoResponse; //!< Ack if all stages completed; else the answer that ended it
		std::string stage;						  //!< Stage that ended it: "setup", "data", "status" (empty if Ack)
		std::vector<uint8_t> data;				  //!< Data stage of a control read
		bool ok() const { return status == Handshake::Ack; }
		bool stalled() const { return status == Handshake::Stall; }
	};

	/** Information about one interface of the configuration descriptor */
	struct Interface {
		uint8_t number = 0, alternate = 0, cls = 0, subclass = 0, protocol = 0, stringIndex = 0;
		uint8_t epIn = 0, epOut = 0;		  //!< Endpoint addresses (0 = none). CDC: the data endpoints
		uint16_t epInSize = 0, epOutSize = 0; //!< wMaxPacketSize
		uint8_t epNotify = 0;				  //!< CDC: notification (interrupt IN) endpoint
		uint16_t hidReportLength = 0;		  //!< HID: wDescriptorLength of the report descriptor
	};

	/** Descriptors read by enumerate() */
	std::vector<uint8_t> deviceDescriptor, configDescriptor;
	std::vector<Interface> interfaces;

	const std::vector<std::string> &errors() const { return errorList; }
	std::vector<std::string> takeErrors() {
		std::vector<std::string> e;
		e.swap(errorList);
		return e;
	}

	static tusb_control_request_t request(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength) {
		tusb_control_request_t r;
		std::memset(&r, 0, sizeof(r));
		r.bmRequestType = bmRequestType;
		r.bRequest = bRequest;
		r.wValue = wValue;
		r.wIndex = wIndex;
		r.wLength = wLength;
		return r;
	}

	// ---------------- Transactions with NAK retries ----------------
	Handshake in(uint8_t epNum, std::vector<uint8_t> &data) {
		Handshake h = Handshake::Nak;
		for (uint32_t i = 0; i <= maxRetries; i++) {
			h = HostDcd::in(address, epNum, data);
			if (h != Handshake::Nak) {
				break;
			}
			pump();
		}
		return h;
	}
	Handshake out(uint8_t epNum, const std::vector<uint8_t> &data) {
		Handshake h = Handshake::Nak;
		for (uint32_t i = 0; i <= maxRetries; i++) {
			h = HostDcd::out(address, epNum, data);
			if (h != Handshake::Nak) {
				break;
			}
			pump();
		}
		return h;
	}

	// ---------------- Control transfers (USB 2.0 8.5.3) ----------------
	/** Control transfer on EP0. A control write sends `outData` (its size must be wLength) */
	ControlResult control(const tusb_control_request_t &req, const std::vector<uint8_t> &outData = {}) {
		ControlResult r;
		const bool deviceToHost = (req.bmRequestType & 0x80) != 0;
		r.status = HostDcd::setup(address, req);
		if (r.status != Handshake::Ack) {
			r.stage = "setup";
			return r;
		}
		pump();
		// Data stage
		if (req.wLength > 0 && deviceToHost) {
			while (true) {
				std::vector<uint8_t> packet;
				r.status = in(0, packet);
				if (r.status != Handshake::Ack) {
					r.stage = "data";
					return r;
				}
				r.data.insert(r.data.end(), packet.begin(), packet.end());
				if (r.data.size() > req.wLength) {
					errorList.push_back("control read returned " + std::to_string(r.data.size()) + " bytes for wLength " + std::to_string(req.wLength));
				}
				// Ends with a short packet or when wLength bytes were received (USB 2.0 5.5.3)
				if (packet.size() < CFG_TUD_ENDPOINT0_SIZE || r.data.size() >= req.wLength) {
					break;
				}
			}
		} else if (req.wLength > 0) {
			if (outData.size() != req.wLength) {
				errorList.push_back("UsbHost::control: control write with " + std::to_string(outData.size()) + " bytes for wLength " + std::to_string(req.wLength));
			}
			size_t pos = 0;
			do {
				const size_t n = std::min<size_t>(CFG_TUD_ENDPOINT0_SIZE, outData.size() - pos);
				r.status = out(0, std::vector<uint8_t>(outData.begin() + (long)pos, outData.begin() + (long)(pos + n)));
				if (r.status != Handshake::Ack) {
					r.stage = "data";
					return r;
				}
				pos += n;
			} while (pos < outData.size());
		}
		// Status stage: opposite direction of the data stage, IN for a transfer without data stage
		if (deviceToHost && req.wLength > 0) {
			r.status = out(0, {});
		} else {
			std::vector<uint8_t> zlp;
			r.status = in(0, zlp);
			if (r.status == Handshake::Ack && !zlp.empty()) {
				errorList.push_back("status stage IN returned " + std::to_string(zlp.size()) + " bytes instead of a ZLP");
			}
		}
		if (r.status != Handshake::Ack) {
			r.stage = "status";
			return r;
		}
		pump();
		return r;
	}

	// ---------------- Standard requests (USB 2.0 9.4) ----------------
	ControlResult getDescriptor(uint8_t type, uint8_t index, uint16_t length, uint16_t langId = 0) {
		return control(request(0x80, TUSB_REQ_GET_DESCRIPTOR, (uint16_t)(type << 8 | index), langId, length));
	}
	/** GET_DESCRIPTOR(STRING) decoded from UTF-16LE (only the low bytes; the firmware's strings are ASCII) */
	std::string getString(uint8_t index, uint16_t langId = 0x0409) {
		ControlResult r = getDescriptor(TUSB_DESC_STRING, index, 255, langId);
		std::string s;
		if (r.ok() && r.data.size() >= 2) {
			for (size_t i = 2; i + 1 < r.data.size() && i < r.data[0]; i += 2) {
				s += (char)r.data[i];
			}
		}
		return s;
	}
	ControlResult setAddress(uint8_t newAddress) {
		ControlResult r = control(request(0x00, TUSB_REQ_SET_ADDRESS, newAddress, 0, 0));
		if (r.ok()) {
			address = newAddress;
		}
		return r;
	}
	ControlResult setConfiguration(uint8_t config) { return control(request(0x00, TUSB_REQ_SET_CONFIGURATION, config, 0, 0)); }
	ControlResult getConfiguration() { return control(request(0x80, TUSB_REQ_GET_CONFIGURATION, 0, 0, 1)); }
	ControlResult setDeviceFeature(uint16_t feature) { return control(request(0x00, TUSB_REQ_SET_FEATURE, feature, 0, 0)); }

	/** Bus reset; afterwards the device answers at address 0 */
	void busReset() {
		HostDcd::busReset();
		address = 0;
		pump();
	}

	/**
	 * Enumeration like a host does it: bus reset, GET_DESCRIPTOR(DEVICE, 64) at address 0, SET_ADDRESS, GET_DESCRIPTOR
	 * (DEVICE), GET_DESCRIPTOR(CONFIGURATION) 9 bytes then wTotalLength, string descriptor 0 (languages),
	 * SET_CONFIGURATION(1). Stores the descriptors and the interfaces. @return true if every step succeeded
	 */
	bool enumerate(uint8_t newAddress = 5, bool configure = true) {
		busReset();
		ControlResult r = getDescriptor(TUSB_DESC_DEVICE, 0, 64);
		if (!r.ok() || !setAddress(newAddress).ok()) {
			return false;
		}
		r = getDescriptor(TUSB_DESC_DEVICE, 0, sizeof(tusb_desc_device_t));
		if (!r.ok()) {
			return false;
		}
		deviceDescriptor = r.data;
		r = getDescriptor(TUSB_DESC_CONFIGURATION, 0, sizeof(tusb_desc_configuration_t));
		if (!r.ok() || r.data.size() < 4) {
			return false;
		}
		const uint16_t total = (uint16_t)(r.data[2] | r.data[3] << 8);
		r = getDescriptor(TUSB_DESC_CONFIGURATION, 0, total);
		if (!r.ok()) {
			return false;
		}
		configDescriptor = r.data;
		parseConfiguration();
		if (!getDescriptor(TUSB_DESC_STRING, 0, 255).ok()) {
			return false;
		}
		return !configure || setConfiguration(1).ok();
	}

	/** First interface of a class (TUSB_CLASS_CDC, TUSB_CLASS_CDC_DATA, TUSB_CLASS_HID). nullptr if none */
	const Interface *findInterface(uint8_t cls) const {
		for (const Interface &i : interfaces) {
			if (i.cls == cls) {
				return &i;
			}
		}
		return nullptr;
	}

	// ---------------- Bulk / interrupt transfers ----------------
	/** Sends `data` to an OUT endpoint in packets of `maxPacket` bytes, with a ZLP after a full last packet if `zlp` */
	Handshake sendOut(uint8_t epAddr, uint16_t maxPacket, const std::vector<uint8_t> &data, bool zlp = true) {
		size_t pos = 0;
		do {
			const size_t n = std::min<size_t>(maxPacket, data.size() - pos);
			Handshake h = out(epAddr & 0x0F, std::vector<uint8_t>(data.begin() + (long)pos, data.begin() + (long)(pos + n)));
			if (h != Handshake::Ack) {
				return h;
			}
			pos += n;
			if (pos == data.size() && n == maxPacket && zlp) {
				return out(epAddr & 0x0F, {});
			}
		} while (pos < data.size());
		return Handshake::Ack;
	}
	/**
	 * Reads one transfer from an IN endpoint: packets until a short packet (or `maxLength` bytes). Returns the data;
	 * `status` is Nak if the device had nothing to send (after the retries).
	 */
	std::vector<uint8_t> receiveIn(uint8_t epAddr, uint16_t maxPacket, size_t maxLength, Handshake *status = nullptr) {
		std::vector<uint8_t> all;
		Handshake h;
		while (true) {
			std::vector<uint8_t> packet;
			h = in(epAddr & 0x0F, packet);
			if (h != Handshake::Ack) {
				break;
			}
			all.insert(all.end(), packet.begin(), packet.end());
			if (packet.size() < maxPacket || all.size() >= maxLength) {
				break;
			}
		}
		if (status != nullptr) {
			*status = h;
		}
		return all;
	}

	// ---------------- CDC ACM (CDC PSTN 1.2, 6.3) ----------------
	ControlResult cdcSetLineCoding(uint32_t baud, uint8_t stopBits, uint8_t parity, uint8_t dataBits) {
		std::vector<uint8_t> coding = {(uint8_t)baud, (uint8_t)(baud >> 8), (uint8_t)(baud >> 16), (uint8_t)(baud >> 24), stopBits, parity, dataBits};
		return control(request(0x21, CDC_REQUEST_SET_LINE_CODING, 0, cdcControlInterface(), 7), coding);
	}
	ControlResult cdcGetLineCoding() { return control(request(0xA1, CDC_REQUEST_GET_LINE_CODING, 0, cdcControlInterface(), 7)); }
	ControlResult cdcSetControlLineState(bool dtr, bool rts) {
		return control(request(0x21, CDC_REQUEST_SET_CONTROL_LINE_STATE, (uint16_t)((dtr ? 1 : 0) | (rts ? 2 : 0)), cdcControlInterface(), 0));
	}
	/** Writes text to the CDC data OUT endpoint */
	Handshake cdcWrite(const std::string &text) {
		const Interface *d = findInterface(TUSB_CLASS_CDC_DATA);
		if (d == nullptr) {
			errorList.push_back("UsbHost::cdcWrite: no CDC data interface (enumerate first)");
			return Handshake::NoResponse;
		}
		return sendOut(d->epOut, d->epOutSize, std::vector<uint8_t>(text.begin(), text.end()));
	}
	/** Reads from the CDC data IN endpoint until the device has nothing more to send */
	std::string cdcRead() {
		const Interface *d = findInterface(TUSB_CLASS_CDC_DATA);
		if (d == nullptr) {
			errorList.push_back("UsbHost::cdcRead: no CDC data interface (enumerate first)");
			return {};
		}
		std::string text;
		while (true) {
			Handshake h;
			std::vector<uint8_t> data = receiveIn(d->epIn, d->epInSize, 65536, &h);
			text.append(data.begin(), data.end());
			if (h != Handshake::Ack) {
				break;
			}
		}
		return text;
	}

	// ---------------- HID (HID 1.11, 7.2) ----------------
	ControlResult hidSetReport(uint8_t type, uint8_t id, const std::vector<uint8_t> &data) {
		return control(request(0x21, HID_REQ_CONTROL_SET_REPORT, (uint16_t)(type << 8 | id), hidInterface(), (uint16_t)data.size()), data);
	}
	ControlResult hidGetReport(uint8_t type, uint8_t id, uint16_t length) {
		return control(request(0xA1, HID_REQ_CONTROL_GET_REPORT, (uint16_t)(type << 8 | id), hidInterface(), length));
	}
	ControlResult hidGetReportDescriptor() {
		const Interface *h = findInterface(TUSB_CLASS_HID);
		return control(request(0x81, TUSB_REQ_GET_DESCRIPTOR, HID_DESC_TYPE_REPORT << 8, hidInterface(), h ? h->hidReportLength : 0));
	}
	/** Output report on the HID interrupt OUT endpoint */
	Handshake hidWrite(const std::vector<uint8_t> &report) {
		const Interface *h = findInterface(TUSB_CLASS_HID);
		if (h == nullptr || h->epOut == 0) {
			errorList.push_back("UsbHost::hidWrite: no HID OUT endpoint (enumerate first)");
			return Handshake::NoResponse;
		}
		return sendOut(h->epOut, h->epOutSize, report, false);
	}
	/** One input report from the HID interrupt IN endpoint; `status` Nak if there was none */
	std::vector<uint8_t> hidRead(Handshake *status = nullptr) {
		const Interface *h = findInterface(TUSB_CLASS_HID);
		if (h == nullptr) {
			errorList.push_back("UsbHost::hidRead: no HID interface (enumerate first)");
			return {};
		}
		return receiveIn(h->epIn, h->epInSize, h->epInSize, status);
	}

	uint16_t cdcControlInterface() const {
		const Interface *c = findInterface(TUSB_CLASS_CDC);
		return c ? c->number : 0;
	}
	uint16_t hidInterface() const {
		const Interface *h = findInterface(TUSB_CLASS_HID);
		return h ? h->number : 0;
	}

	/** Parses configDescriptor into `interfaces` */
	void parseConfiguration() {
		interfaces.clear();
		size_t pos = 0;
		Interface *current = nullptr;
		while (pos + 2 <= configDescriptor.size() && configDescriptor[pos] >= 2) {
			const uint8_t *d = &configDescriptor[pos];
			const uint8_t len = d[0];
			if (pos + len > configDescriptor.size()) {
				errorList.push_back("configuration descriptor: descriptor at offset " + std::to_string(pos) + " exceeds wTotalLength");
				break;
			}
			if (d[1] == TUSB_DESC_INTERFACE && len >= 9) {
				Interface i;
				i.number = d[2];
				i.alternate = d[3];
				i.cls = d[5];
				i.subclass = d[6];
				i.protocol = d[7];
				i.stringIndex = d[8];
				interfaces.push_back(i);
				current = &interfaces.back();
			} else if (d[1] == HID_DESC_TYPE_HID && current != nullptr && len >= 9) {
				current->hidReportLength = (uint16_t)(d[7] | d[8] << 8);
			} else if (d[1] == TUSB_DESC_ENDPOINT && current != nullptr && len >= 7) {
				const uint8_t epAddr = d[2];
				const uint16_t size = (uint16_t)(d[4] | d[5] << 8) & 0x7FF;
				if (current->cls == TUSB_CLASS_CDC && (epAddr & 0x80)) {
					current->epNotify = epAddr;
				} else if (epAddr & 0x80) {
					current->epIn = epAddr;
					current->epInSize = size;
				} else {
					current->epOut = epAddr;
					current->epOutSize = size;
				}
			}
			pos += len;
		}
	}

private:
	std::function<void()> pump;
	std::vector<std::string> errorList;
};

#endif
