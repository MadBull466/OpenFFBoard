/*
 * FakeSmDevice.cpp: see FakeSmDevice.h (models, sources, limits, confidence).
 */
#include "FakeSmDevice.h"

#include "main.h"

#include <algorithm>

uint8_t FakeSmDevice::crc8(const uint8_t *buf, size_t len, uint8_t init) {
	uint8_t crc = init;
	for (size_t i = 0; i < len; ++i) {
		crc ^= buf[i];
		for (int b = 0; b < 8; ++b) {
			crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
		}
	}
	return crc;
}

uint16_t FakeSmDevice::crc16arc(const uint8_t *buf, size_t len) {
	uint16_t crc = 0;
	for (size_t i = 0; i < len; ++i) {
		crc ^= buf[i];
		for (int b = 0; b < 8; ++b) {
			crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
		}
	}
	return crc;
}

FakeSmDevice::FakeSmDevice(const void *huart) : port(huart) {
	// Healthy ABN1 servo in torque mode (assumption, see the header)
	params[SMP_DEVICE_TYPE] = 1;
	params[SMP_STATUS] = STAT_INITIALIZED;
	params[SMP_ENCODER_PPR] = 1000;
	params[SMP_FB1_DEVICE_SELECTION] = 1; // MotorSimplemotion_FBR::ABN1
	params[SMP_CONTROL_MODE] = 3;		   // torque
	params[SMP_ACTUAL_BUS_VOLTAGE] = 4800; // 10 mV per count: 48 V
	params[SMP_ACTUAL_TORQUE] = 0;
	params[SMP_CUMULATIVE_STATUS] = 0;
	params[SMP_FAULTS] = 0;
	params[SMP_SYSTEM_CONTROL] = 0;
	params[SMP_CONTROL_BITS1] = 0;
}

FakeSmDevice::~FakeSmDevice() = default;

void FakeSmDevice::clearFaults() {
	silent = false;
	corruptCrc = false;
	replyAddress = -1;
	truncateReply = 0;
	padSubpackets = 0;
	chunk = 0;
	gapMs = 0;
	errorOnByte = -1;
	autoTxComplete = true;
}

void FakeSmDevice::clearFrames() {
	frameLog.clear();
	writeLog.clear();
	setpointLog.clear();
	retAddrLog.clear();
}

void FakeSmDevice::injectRaw(const std::vector<uint8_t> &bytes) {
	pendingReply = bytes;
	replyPos = 0;
	chunkStarted = false;
	gapPaid = false;
}

void FakeSmDevice::queueReply(std::vector<uint8_t> bytes) {
	if (silent) {
		return;
	}
	if (corruptCrc && !bytes.empty()) {
		bytes.back() ^= 1;
	}
	bytes.resize(bytes.size() > truncateReply ? bytes.size() - truncateReply : 0);
	injectRaw(bytes);
}

uint32_t FakeSmDevice::readParam(uint32_t id) {
	if (id == SMP_RETURN_PARAM_ADDR) {
		return retAddr;
	}
	if (id == SMP_RETURN_PARAM_LEN) {
		return retLen;
	}
	auto it = params.find(static_cast<uint16_t>(id));
	if (it == params.end()) {
		unknownReadCount++;
		return 0;
	}
	return it->second;
}

void FakeSmDevice::appendReplySubpacket(std::vector<uint8_t> &out) {
	const uint32_t v = readParam(retAddr);
	switch (retLen & 3) {
	case 0: // 32 bit: type 00, 30 bit value
		out.push_back(static_cast<uint8_t>(((v >> 24) & 0x3f) | 0x00));
		out.push_back(static_cast<uint8_t>(v >> 16));
		out.push_back(static_cast<uint8_t>(v >> 8));
		out.push_back(static_cast<uint8_t>(v));
		break;
	case 1: // 24 bit: type 01, 22 bit value
		out.push_back(static_cast<uint8_t>(((v >> 16) & 0x3f) | 0x40));
		out.push_back(static_cast<uint8_t>(v >> 8));
		out.push_back(static_cast<uint8_t>(v));
		break;
	case 2: // 16 bit: type 10, 14 bit value
		out.push_back(static_cast<uint8_t>(((v >> 8) & 0x3f) | 0x80));
		out.push_back(static_cast<uint8_t>(v));
		break;
	default: // status byte: type 11, ACK (0)
		out.push_back(0xC0);
		break;
	}
}

void FakeSmDevice::handleFast(const std::vector<uint8_t> &f) {
	if (f.size() != 7) {
		violate("fast update frame is not 7 bytes (" + std::to_string(f.size()) + ")");
		return;
	}
	if (crc8(f.data(), 6, 0x52) != f[6]) {
		violate("fast update frame with a wrong CRC-8");
		return;
	}
	if (filterAddress && f[1] != address) {
		foreign++;
		return;
	}
	setpointLog.push_back(static_cast<int16_t>(static_cast<uint16_t>(f[2] | (f[3] << 8))));
	std::vector<uint8_t> r = {0x11, static_cast<uint8_t>(position16), static_cast<uint8_t>(position16 >> 8),
							  static_cast<uint8_t>(status16), static_cast<uint8_t>(status16 >> 8)};
	r.push_back(crc8(r.data(), 5, 0x52));
	queueReply(r);
}

void FakeSmDevice::handleInstant(const std::vector<uint8_t> &f) {
	if (f.size() < 5) {
		violate("instant frame shorter than 5 bytes");
		return;
	}
	const size_t len = f[1];
	if (f.size() != len + 5) {
		violate("instant frame length byte " + std::to_string(len) + " does not match the frame size " + std::to_string(f.size()));
		return;
	}
	const uint16_t crc = crc16arc(f.data(), len + 3);
	if (f[len + 3] != (crc & 0xff) || f[len + 4] != (crc >> 8)) {
		violate("instant frame with a wrong CRC-16");
		return;
	}
	if (filterAddress && f[2] != address) {
		foreign++;
		return;
	}
	std::vector<uint8_t> sub;
	uint32_t writeAddr = 0;
	size_t i = 0;
	const uint8_t *p = f.data() + 3;
	while (i < len) {
		const unsigned type = p[i] >> 6;
		const size_t size = 4 - type;
		if (type == 3) {
			violate("outbound subpacket of type 3 (status is an inbound type)");
			return;
		}
		if (i + size > len) {
			violate("subpacket runs over the end of the payload");
			return;
		}
		uint32_t v = p[i] & 0x3f;
		for (size_t b = 1; b < size; ++b) {
			v = (v << 8) | p[i + b];
		}
		if (type == 2) {
			writeAddr = v;
		} else {
			writeLog.emplace_back(static_cast<uint16_t>(writeAddr), v);
			if (writeAddr == SMP_RETURN_PARAM_ADDR) {
				retAddr = v;
				retAddrLog.push_back(v);
			} else if (writeAddr == SMP_RETURN_PARAM_LEN) {
				retLen = v & 3;
			} else {
				if (writeAddr == SMP_SYSTEM_CONTROL && v == 2) {
					restarts++;
				}
				params[static_cast<uint16_t>(writeAddr)] = v;
			}
		}
		appendReplySubpacket(sub);
		i += size;
	}
	for (size_t k = 0; k < padSubpackets; ++k) {
		sub.insert(sub.end(), {0x00, 0x00, 0x00, 0x00});
	}
	std::vector<uint8_t> r = {0x25, static_cast<uint8_t>(sub.size()),
							  static_cast<uint8_t>(replyAddress >= 0 ? replyAddress : f[2])};
	r.insert(r.end(), sub.begin(), sub.end());
	const uint16_t rc = crc16arc(r.data(), r.size());
	r.push_back(static_cast<uint8_t>(rc & 0xff));
	r.push_back(static_cast<uint8_t>(rc >> 8));
	queueReply(r);
}

void FakeSmDevice::onTransmit(const void *huart, const uint8_t *data, uint16_t size) {
	if (huart != port) {
		return;
	}
	if (!pendingReply.empty()) {
		dropped++; // a new frame while the previous reply was not delivered (no collision model)
		pendingReply.clear();
		replyPos = 0;
	}
	txPending = true;
	std::vector<uint8_t> f(data, data + size);
	Frame fr{Kind::Invalid, f, HostPlatform::nowUs()};
	if (f.empty()) {
		violate("empty transmit");
		frameLog.push_back(fr);
		return;
	}
	if (f[0] == 0x10) {
		fr.kind = Kind::Fast;
	} else if (f[0] == 0x24) {
		fr.kind = Kind::Instant;
	} else {
		violate("frame header " + std::to_string(f[0]) + " (decimal) is neither a fast update (0x10) nor an instant command (0x24)");
	}
	frameLog.push_back(fr);
	if (fr.kind == Kind::Fast) {
		handleFast(f);
	} else if (fr.kind == Kind::Instant) {
		handleInstant(f);
	}
}

void FakeSmDevice::onReceiveArmed(const void *huart) {
	if (huart == port) {
		armed = true;
	}
}

bool FakeSmDevice::pump() {
	if (txPending && autoTxComplete) {
		txPending = false;
		HostPlatform::uartCompleteTx(port);
		return true;
	}
	if (pendingReply.empty() || !armed) {
		return false;
	}
	const size_t remaining = pendingReply.size() - replyPos;
	const size_t n = chunk ? std::min(chunk, remaining) : remaining;
	if (chunkStarted && gapMs && !gapPaid) {
		// The gap passes first and the poll ends: the firmware checks its 10 ms deadline before the next chunk arrives
		HostPlatform::advanceMs(gapMs);
		gapPaid = true;
		return true;
	}
	gapPaid = false;
	chunkStarted = true;
	UART_HandleTypeDef *h = const_cast<UART_HandleTypeDef *>(static_cast<const UART_HandleTypeDef *>(port));
	for (size_t k = 0; k < n; ++k) {
		armed = false;
		if (errorOnByte >= 0 && replyPos == static_cast<size_t>(errorOnByte)) {
			h->ErrorCode = 1;
		}
		HostPlatform::uartReceive(std::string(1, static_cast<char>(pendingReply[replyPos])));
		h->ErrorCode = 0;
		replyPos++;
		if (!armed && replyPos < pendingReply.size()) {
			notAccepted += static_cast<uint32_t>(pendingReply.size() - replyPos); // the firmware stopped listening
			replyPos = pendingReply.size();
			break;
		}
	}
	if (replyPos >= pendingReply.size()) {
		pendingReply.clear();
		replyPos = 0;
		chunkStarted = false;
	}
	return true;
}

void FakeSmDevice::pumpAll(unsigned maxSteps) {
	for (unsigned i = 0; i < maxSteps && pump(); ++i) {
	}
}

void FakeSmDevice::onPoll() {
	if (!pump()) {
		HostPlatform::advanceMs(1); // nothing to deliver: simulated time passes while the firmware spins
	}
}
