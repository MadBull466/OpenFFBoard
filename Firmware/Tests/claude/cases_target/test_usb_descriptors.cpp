/*
 * Tests of the USB descriptor tables (usb_descriptors.cpp, usb_hid_1ffb_desc.c, usb_hid_2ffb_desc.c, usb_hid_gamepad.c)
 * and of the TinyUSB descriptor callbacks of global_callbacks.cpp that serve them for each main class.
 *
 * The descriptors are parsed byte by byte in this file (configuration descriptor chain, HID report descriptor items).
 * Expected lengths are never taken from the firmware's length macros (TUD_*_DESC_LEN, USB_HID_*_REPORT_DESC_SIZE,
 * HIDDESC_*_SIZE): a parsed length is compared with the length field that refers to it, and report sizes are compared
 * with the C structs the firmware sends and receives (ffb_defs.h, HidCommandInterface.h).
 *
 * The F407 build only uses the 16 bit 2 axis descriptor (FFBWheel, FFBJoystick) and the gamepad descriptor (FFBHIDExt).
 * The variants of other configurations (1 axis: FFBWHEEL_USE_1AXIS_DESC, 32 bit axes: HIDAXISRES_USE_32B_DESC) are
 * compiled in this file from the unmodified firmware sources by including them into their own namespace with the
 * feature macros set below. These macros must come before every include. They only select descriptor tables (only
 * constants.h and the descriptor files read them).
 * Not testable: the 1 axis 32 bit variant (AXIS1_FFB_HID_DESC_32B) does not compile (usb_hid_ffb_desc.h:757 defines
 * USB_HID_1FFB_REPORT_DESC_SIZE instead of USB_HID_1FFB_REPORT_DESC_32B_SIZE; usb_descriptors.h does not declare
 * usb_cdc_hid_conf_1axis_32b used by FFBWheel.cpp:40). See PROGRESS.md (suspected bug list).
 */
#define AXIS1_FFB_HID_DESC		// 1 axis 16 bit descriptor (hid_1ffb_desc)
#define AXIS2_FFB_HID_DESC_32B	// 2 axis 32 bit descriptor (hid_2ffb_desc_32b)
#define FFB_HID_DESC_GAMEPAD_32B // gamepad 32 bit descriptor (hid_gamepad_desc_32b)

#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "FFBHIDExt.h"
#include "FFBJoystick.h"
#include "FFBWheel.h"
#include "HidCommandInterface.h"
#include "USBdevice.h"
#include "ffb_defs.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "usb_hid_ffb_desc.h"

#include <cstdint>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// The unmodified descriptor sources, compiled again with the variant macros above. Each gets its own namespace so the
// tables that the regular build also defines (hid_2ffb_desc) do not clash. The headers they include are already
// included above (include guards), so only the tables end up in the namespaces.
namespace variant_1axis {
#include "../../../FFBoard/UserExtensions/Src/usb_hid_1ffb_desc.c"
}
namespace variant_2axis {
#include "../../../FFBoard/UserExtensions/Src/usb_hid_2ffb_desc.c"
}
namespace variant_gamepad {
#include "../../../FFBoard/UserExtensions/Src/usb_hid_gamepad.c"
}

namespace {

// ---------------------------------------------------------------------------------------------------------------------
// Configuration descriptor parser (USB 2.0 chapter 9.6, IAD ECN, CDC 1.2, HID 1.11 chapter 6.2.1)
// ---------------------------------------------------------------------------------------------------------------------

constexpr uint8_t DESC_CONFIGURATION = 0x02;
constexpr uint8_t DESC_STRING = 0x03;
constexpr uint8_t DESC_INTERFACE = 0x04;
constexpr uint8_t DESC_ENDPOINT = 0x05;
constexpr uint8_t DESC_IAD = 0x0B;
constexpr uint8_t DESC_HID = 0x21;
constexpr uint8_t DESC_HID_REPORT = 0x22;
constexpr uint8_t DESC_CS_INTERFACE = 0x24;
constexpr uint8_t DESC_CS_ENDPOINT = 0x25;

constexpr uint8_t CLASS_AUDIO = 0x01;
constexpr uint8_t CLASS_CDC = 0x02;
constexpr uint8_t CLASS_HID = 0x03;
constexpr uint8_t CLASS_CDC_DATA = 0x0A;

constexpr uint8_t CDC_FUNC_CALL_MANAGEMENT = 0x01;
constexpr uint8_t CDC_FUNC_UNION = 0x06;

constexpr uint8_t XFER_BULK = 2;
constexpr uint8_t XFER_INTERRUPT = 3;

uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

struct Endpoint {
	uint8_t address = 0;
	uint8_t attributes = 0;
	uint16_t maxPacketSize = 0;
	uint8_t interval = 0;
};

struct Interface {
	uint8_t number = 0;
	uint8_t alternate = 0;
	uint8_t numEndpoints = 0;
	uint8_t cls = 0, subclass = 0, protocol = 0;
	uint8_t stringIndex = 0;
	std::vector<Endpoint> endpoints;
	std::vector<std::vector<uint8_t>> classDescriptors; // CS_INTERFACE descriptors following the interface
	bool hasHidDescriptor = false;
	uint16_t hidReportLength = 0; // wDescriptorLength of the report descriptor in the HID descriptor
	uint8_t hidNumDescriptors = 0;
	uint8_t hidFirstDescriptorType = 0;
};

struct Iad {
	uint8_t firstInterface = 0, interfaceCount = 0, functionClass = 0;
};

struct ConfigDesc {
	std::string error; // empty if the descriptor chain is well formed
	uint16_t totalLength = 0;
	uint16_t parsedLength = 0; // sum of bLength of all descriptors
	uint8_t numInterfaces = 0;
	uint8_t configValue = 0;
	uint8_t stringIndex = 0;
	uint8_t attributes = 0;
	uint8_t maxPower = 0;
	std::vector<Interface> interfaces;
	std::vector<Iad> iads;
	std::vector<uint8_t> types; // bDescriptorType of every descriptor in order
};

/** Parses the descriptor chain of a configuration descriptor up to its wTotalLength */
ConfigDesc parseConfig(const uint8_t *conf) {
	ConfigDesc c;
	if (conf[0] != 9 || conf[1] != DESC_CONFIGURATION) {
		c.error = "first descriptor is not a 9 byte configuration descriptor";
		return c;
	}
	c.totalLength = le16(conf + 2);
	c.numInterfaces = conf[4];
	c.configValue = conf[5];
	c.stringIndex = conf[6];
	c.attributes = conf[7];
	c.maxPower = conf[8];
	size_t off = 0;
	while (off < c.totalLength) {
		const uint8_t *d = conf + off;
		const uint8_t len = d[0];
		std::ostringstream where;
		where << "descriptor at offset " << off;
		if (len < 2) {
			c.error = where.str() + " has bLength < 2";
			return c;
		}
		if (off + len > c.totalLength) {
			c.error = where.str() + " extends beyond wTotalLength";
			return c;
		}
		const uint8_t type = d[1];
		c.types.push_back(type);
		switch (type) {
		case DESC_CONFIGURATION:
			if (off != 0) {
				c.error = where.str() + ": second configuration descriptor";
				return c;
			}
			break;
		case DESC_INTERFACE: {
			if (len != 9) {
				c.error = where.str() + ": interface descriptor bLength != 9";
				return c;
			}
			Interface itf;
			itf.number = d[2];
			itf.alternate = d[3];
			itf.numEndpoints = d[4];
			itf.cls = d[5];
			itf.subclass = d[6];
			itf.protocol = d[7];
			itf.stringIndex = d[8];
			c.interfaces.push_back(itf);
			break;
		}
		case DESC_ENDPOINT: {
			if ((len != 7 && len != 9) || c.interfaces.empty()) { // 9: audio class endpoint (MIDI)
				c.error = where.str() + ": endpoint descriptor of wrong length or outside an interface";
				return c;
			}
			Endpoint ep;
			ep.address = d[2];
			ep.attributes = d[3];
			ep.maxPacketSize = le16(d + 4);
			ep.interval = d[6];
			c.interfaces.back().endpoints.push_back(ep);
			break;
		}
		case DESC_IAD: {
			if (len != 8) {
				c.error = where.str() + ": IAD bLength != 8";
				return c;
			}
			c.iads.push_back(Iad{d[2], d[3], d[4]});
			break;
		}
		case DESC_HID: {
			if (len < 9 || c.interfaces.empty()) {
				c.error = where.str() + ": HID descriptor too short or outside an interface";
				return c;
			}
			Interface &itf = c.interfaces.back();
			itf.hasHidDescriptor = true;
			itf.hidNumDescriptors = d[5];
			itf.hidFirstDescriptorType = d[6];
			itf.hidReportLength = le16(d + 7);
			break;
		}
		case DESC_CS_INTERFACE:
			if (c.interfaces.empty()) {
				c.error = where.str() + ": class specific interface descriptor outside an interface";
				return c;
			}
			c.interfaces.back().classDescriptors.emplace_back(d, d + len);
			break;
		case DESC_CS_ENDPOINT:
			break;
		default:
			c.error = where.str() + ": unexpected descriptor type " + std::to_string(type);
			return c;
		}
		off += len;
	}
	c.parsedLength = (uint16_t)off;
	return c;
}

const Interface *findInterface(const ConfigDesc &c, uint8_t cls) {
	for (const Interface &i : c.interfaces) {
		if (i.cls == cls) {
			return &i;
		}
	}
	return nullptr;
}

// ---------------------------------------------------------------------------------------------------------------------
// HID report descriptor parser (HID 1.11 chapter 6.2.2: short items, main/global/local tags, push/pop)
// ---------------------------------------------------------------------------------------------------------------------

enum class MainKind { input, output, feature };

struct HidReportDesc {
	std::string error;		   // empty if all items are valid
	size_t length = 0;		   // bytes consumed by complete items
	size_t lastEndCollection = 0; // offset after the End Collection that closed the last top level collection
	int maxDepth = 0;
	int depthAtEnd = 0;
	bool mainItemWithoutId = false; // a data main item before any Report ID item (HID 1.11 6.2.2.7)
	std::map<std::pair<MainKind, uint8_t>, uint32_t> bits; // report size in bits per (kind, report id)
	std::set<uint16_t> usagePages;
	std::vector<std::string> rangeErrors; // main items whose logical minimum is above the logical maximum
};

int32_t signedValue(uint32_t v, uint8_t size) {
	switch (size) {
	case 1:
		return (int8_t)v;
	case 2:
		return (int16_t)v;
	default:
		return (int32_t)v;
	}
}

HidReportDesc parseHidReport(const uint8_t *desc, size_t size) {
	HidReportDesc r;
	struct Globals {
		uint32_t reportSize = 0, reportCount = 0;
		uint8_t reportId = 0;
		uint16_t usagePage = 0;
		int32_t logicalMin = 0, logicalMax = 0;
	};
	Globals g;
	std::vector<Globals> stack;
	bool anyId = false;
	int depth = 0;
	size_t off = 0;
	while (off < size) {
		const uint8_t prefix = desc[off];
		std::ostringstream where;
		where << "item 0x" << std::hex << (int)prefix << std::dec << " at offset " << off;
		if (prefix == 0xFE) {
			r.error = where.str() + ": long items are not used by HID class devices";
			return r;
		}
		const uint8_t bsize = (prefix & 3) == 3 ? 4 : (prefix & 3);
		const uint8_t type = (prefix >> 2) & 3;
		const uint8_t tag = prefix >> 4;
		if (off + 1 + bsize > size) {
			r.error = where.str() + ": data beyond the end of the descriptor";
			return r;
		}
		uint32_t udata = 0;
		for (uint8_t i = 0; i < bsize; i++) {
			udata |= (uint32_t)desc[off + 1 + i] << (8 * i);
		}
		const int32_t sdata = signedValue(udata, bsize);
		switch (type) {
		case 0: // Main
			switch (tag) {
			case 0x8: // Input
			case 0x9: // Output
			case 0xB: // Feature
			{
				const MainKind kind = tag == 0x8 ? MainKind::input : (tag == 0x9 ? MainKind::output : MainKind::feature);
				if (!anyId) {
					r.mainItemWithoutId = true;
				}
				if (g.reportSize == 0 || g.reportCount == 0) {
					r.error = where.str() + ": main item without report size or count";
					return r;
				}
				const bool constant = (udata & 1) != 0;
				if (!constant && g.logicalMin > g.logicalMax) {
					std::ostringstream e;
					e << where.str() << " (report " << (int)g.reportId << "): logical minimum " << g.logicalMin << " > maximum " << g.logicalMax;
					r.rangeErrors.push_back(e.str());
				}
				r.bits[{kind, g.reportId}] += g.reportSize * g.reportCount;
				break;
			}
			case 0xA: // Collection
				depth++;
				r.maxDepth = std::max(r.maxDepth, depth);
				break;
			case 0xC: // End Collection
				if (depth == 0) {
					r.error = where.str() + ": End Collection without Collection";
					return r;
				}
				depth--;
				if (depth == 0) {
					r.lastEndCollection = off + 1 + bsize;
				}
				break;
			default:
				r.error = where.str() + ": reserved main item tag";
				return r;
			}
			break;
		case 1: // Global
			switch (tag) {
			case 0x0:
				g.usagePage = (uint16_t)udata;
				r.usagePages.insert(g.usagePage);
				break;
			case 0x1:
				g.logicalMin = sdata;
				break;
			case 0x2:
				g.logicalMax = sdata;
				break;
			case 0x3: // Physical Minimum
			case 0x4: // Physical Maximum
			case 0x5: // Unit Exponent
			case 0x6: // Unit
				break;
			case 0x7:
				g.reportSize = udata;
				break;
			case 0x8:
				if (udata == 0 || udata > 255) {
					r.error = where.str() + ": report id 0 or above 255";
					return r;
				}
				g.reportId = (uint8_t)udata;
				anyId = true;
				break;
			case 0x9:
				g.reportCount = udata;
				break;
			case 0xA: // Push
				stack.push_back(g);
				break;
			case 0xB: // Pop
				if (stack.empty()) {
					r.error = where.str() + ": Pop without Push";
					return r;
				}
				g = stack.back();
				stack.pop_back();
				break;
			default:
				r.error = where.str() + ": reserved global item tag";
				return r;
			}
			break;
		case 2: // Local
			if (tag > 0xA || tag == 0x6) {
				r.error = where.str() + ": reserved local item tag";
				return r;
			}
			break;
		default:
			r.error = where.str() + ": reserved item type";
			return r;
		}
		off += 1 + bsize;
	}
	r.length = off;
	r.depthAtEnd = depth;
	return r;
}

/** Size of a report in bytes (without the report id). Reports with a size that is not a whole number of bytes are
 * reported as 0xFFFF (the firmware's packed structs only have whole bytes) */
uint32_t reportBytes(const HidReportDesc &r, MainKind kind, uint8_t id) {
	auto it = r.bits.find({kind, id});
	if (it == r.bits.end()) {
		return 0;
	}
	return it->second % 8 == 0 ? it->second / 8 : 0xFFFF;
}

bool hasReport(const HidReportDesc &r, MainKind kind, uint8_t id) { return r.bits.count({kind, id}) != 0; }

std::set<uint8_t> reportIds(const HidReportDesc &r, MainKind kind) {
	std::set<uint8_t> ids;
	for (const auto &e : r.bits) {
		if (e.first.first == kind) {
			ids.insert(e.first.second);
		}
	}
	return ids;
}

// ---------------------------------------------------------------------------------------------------------------------
// Tables under test
// ---------------------------------------------------------------------------------------------------------------------

// Configurations of the 1 axis and 32 bit variants, built with the firmware's configuration macro (usb_descriptors.h)
const uint8_t conf_1axis[] = {USB_CONF_DESC_HID_CDC(USB_HID_1FFB_REPORT_DESC_SIZE, 64)};
const uint8_t conf_2axis_32b[] = {USB_CONF_DESC_HID_CDC(USB_HID_2FFB_REPORT_DESC_32B_SIZE, 64)};
const uint8_t conf_gamepad_32b[] = {USB_CONF_DESC_HID_CDC(USB_HID_GAMEPAD_REPORT_DESC_32B_SIZE, 64)};

struct NamedConfig {
	const char *name;
	const uint8_t *conf;
};
const std::vector<NamedConfig> &allConfigs() {
	static const std::vector<NamedConfig> configs = {
		{"usb_cdc_conf", usb_cdc_conf},
		{"usb_cdc_hid_conf_2axis", usb_cdc_hid_conf_2axis},
		{"usb_cdc_hid_conf_gamepad", usb_cdc_hid_conf_gamepad},
		{"usb_cdc_midi_conf", usb_cdc_midi_conf},
		{"1 axis configuration (USB_CONF_DESC_HID_CDC)", conf_1axis},
		{"2 axis 32 bit configuration (USB_CONF_DESC_HID_CDC)", conf_2axis_32b},
		{"gamepad 32 bit configuration (USB_CONF_DESC_HID_CDC)", conf_gamepad_32b},
	};
	return configs;
}

struct NamedReport {
	const char *name;
	const uint8_t *desc;
	size_t size; // size of the array (the bytes the firmware hands to TinyUSB)
	bool ffb;	 // contains the PID reports
	bool axes32; // 32 bit main axes
};
const std::vector<NamedReport> &allReports() {
	static const std::vector<NamedReport> reports = {
		{"hid_2ffb_desc", hid_2ffb_desc, sizeof(hid_2ffb_desc), true, false},
		{"hid_gamepad_desc", hid_gamepad_desc, sizeof(hid_gamepad_desc), false, false},
		{"hid_1ffb_desc", variant_1axis::hid_1ffb_desc, sizeof(variant_1axis::hid_1ffb_desc), true, false},
		{"hid_2ffb_desc_32b", variant_2axis::hid_2ffb_desc_32b, sizeof(variant_2axis::hid_2ffb_desc_32b), true, true},
		{"hid_gamepad_desc_32b", variant_gamepad::hid_gamepad_desc_32b, sizeof(variant_gamepad::hid_gamepad_desc_32b), false, true},
	};
	return reports;
}

/** Configuration and report descriptor that are used together */
struct ConfigReportPair {
	const char *name;
	const uint8_t *conf;
	const uint8_t *desc;
	size_t size;
};
const std::vector<ConfigReportPair> &configReportPairs() {
	static const std::vector<ConfigReportPair> pairs = {
		{"2 axis", usb_cdc_hid_conf_2axis, hid_2ffb_desc, sizeof(hid_2ffb_desc)},
		{"gamepad", usb_cdc_hid_conf_gamepad, hid_gamepad_desc, sizeof(hid_gamepad_desc)},
		{"1 axis", conf_1axis, variant_1axis::hid_1ffb_desc, sizeof(variant_1axis::hid_1ffb_desc)},
		{"2 axis 32 bit", conf_2axis_32b, variant_2axis::hid_2ffb_desc_32b, sizeof(variant_2axis::hid_2ffb_desc_32b)},
		{"gamepad 32 bit", conf_gamepad_32b, variant_gamepad::hid_gamepad_desc_32b, sizeof(variant_gamepad::hid_gamepad_desc_32b)},
	};
	return pairs;
}

/** Decodes a string descriptor (bLength, bDescriptorType, UTF-16LE) returned by tud_descriptor_string_cb */
std::string decodeString(const uint16_t *desc) {
	REQUIRE(desc != nullptr);
	const uint8_t len = desc[0] & 0xFF;
	CHECK((desc[0] >> 8) == DESC_STRING);
	REQUIRE(len >= 2);
	REQUIRE(len % 2 == 0);
	std::string s;
	for (int i = 1; i < len / 2; i++) {
		REQUIRE(desc[i] < 0x80); // only ASCII is converted
		s += (char)desc[i];
	}
	return s;
}

/** Registers a USB device with the given configuration like a main class's usbInit (without starting its thread) */
struct DescriptorFixture : TargetFixture {
	std::unique_ptr<USBdevice> device;
	void registerDevice(const uint8_t *conf) {
		device = std::make_unique<USBdevice>(&usb_devdesc_ffboard_composite, conf, &usb_ffboard_strings_default);
		usb_device = device.get();
	}
	~DescriptorFixture() override { usb_device = nullptr; }

	/** Checks the configuration and the report descriptor served by the TinyUSB callbacks belong together */
	static void checkServedDescriptorsMatch() {
		REQUIRE(usb_device != nullptr);
		const uint8_t *dev = tud_descriptor_device_cb();
		CHECK(dev == reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite));
		const ConfigDesc c = parseConfig(tud_descriptor_configuration_cb(0));
		REQUIRE(c.error.empty());
		const Interface *hid = findInterface(c, CLASS_HID);
		REQUIRE(hid != nullptr);
		const uint8_t *report = tud_hid_descriptor_report_cb(0);
		REQUIRE(report != nullptr);
		const HidReportDesc r = parseHidReport(report, hid->hidReportLength);
		INFO(r.error);
		CHECK(r.error.empty());
		CHECK(r.depthAtEnd == 0);
		// The report descriptor ends where the HID descriptor says
		CHECK(r.lastEndCollection == hid->hidReportLength);
	}
};

} // namespace

TEST_SUITE("UsbDescriptors") {

// ---------------------------------------------------------------------------------------------------------------------
// Device descriptor
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE("the device descriptor is an 18 byte USB 2.0 device descriptor with the pid.codes ids 1209:FFB0") {
	const uint8_t *d = reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite);
	CHECK(sizeof(usb_devdesc_ffboard_composite) == 18);
	CHECK(d[0] == 18);
	CHECK(d[1] == 0x01);
	CHECK(le16(d + 2) == 0x0200);
	CHECK(le16(d + 8) == 0x1209);
	CHECK(le16(d + 10) == 0xFFB0);
	CHECK(le16(d + 12) == 0x0100);
}

TEST_CASE("the device descriptor has a 64 byte control endpoint, one configuration and string indices 1 to 3") {
	const uint8_t *d = reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite);
	CHECK(d[7] == 64); // bMaxPacketSize0: 8, 16, 32 or 64 at full speed
	CHECK(d[17] == 1); // bNumConfigurations
	CHECK(d[14] == 1); // iManufacturer
	CHECK(d[15] == 2); // iProduct
	CHECK(d[16] == 3); // iSerialNumber
}

TEST_CASE("the device descriptor uses the interface association class codes 0xEF/2/1 for its IAD composite configurations" *
		  KNOWN_ISSUE("(bug 46) usb_descriptors.cpp:25 bDeviceClass is 0 (TUSB_CLASS_UNSPECIFIED) with subclass 2 and protocol 1; USB 2.0 9.6.1 "
					  "requires subclass 0 for class 0, the IAD ECN requires class 0xEF")) {
	// KNOWN_ISSUE: the comment at usb_descriptors.cpp:24 states the IAD rule, but bDeviceClass is TUSB_CLASS_UNSPECIFIED (0).
	// IAD ECN / USB 2.0 9.6.1: with class 0 the subclass and protocol must also be 0; IAD devices use class 0xEF (misc),
	// subclass 2 (common class), protocol 1 (IAD). First failing assertion: the class.
	const uint8_t *d = reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite);
	CHECK(d[4] == 0xEF);
	CHECK(d[5] == 0x02);
	CHECK(d[6] == 0x01);
}

TEST_CASE("the device descriptor sets subclass 2 and protocol 1 of the interface association class") {
	const uint8_t *d = reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite);
	CHECK(d[5] == MISC_SUBCLASS_COMMON);
	CHECK(d[6] == MISC_PROTOCOL_IAD);
}

// ---------------------------------------------------------------------------------------------------------------------
// Configuration descriptors
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE("the descriptor chain of every configuration ends exactly at wTotalLength") {
	for (const NamedConfig &nc : allConfigs()) {
		INFO(nc.name);
		const ConfigDesc c = parseConfig(nc.conf);
		CHECK(c.error.empty()); // no descriptor crosses wTotalLength, every bLength >= 2
		INFO(c.error);
		CHECK(c.parsedLength == c.totalLength);
	}
}

TEST_CASE("the wTotalLength of the configurations built in this file equals the size of their arrays") {
	// usb_descriptors.cpp's own arrays are declared without a size (extern const uint8_t[]): see the hand sums below
	CHECK(parseConfig(conf_1axis).totalLength == sizeof(conf_1axis));
	CHECK(parseConfig(conf_2axis_32b).totalLength == sizeof(conf_2axis_32b));
	CHECK(parseConfig(conf_gamepad_32b).totalLength == sizeof(conf_gamepad_32b));
}

TEST_CASE("the configuration descriptors have the expected total lengths") {
	// Independent sums: config 9, CDC IAD 8 + comm interface 9 + header 5 + call management 5 + ACM 4 + union 5 +
	// notification endpoint 7 + data interface 9 + 2 endpoints 14 = 66, HID interface 9 + HID 9 + 2 endpoints 14 = 32
	CHECK(parseConfig(usb_cdc_conf).totalLength == 9 + 66);
	CHECK(parseConfig(usb_cdc_hid_conf_2axis).totalLength == 9 + 66 + 32);
	CHECK(parseConfig(usb_cdc_hid_conf_gamepad).totalLength == 9 + 66 + 32);
	// MIDI: audio control interface 9 + AC header 9 + MIDI streaming interface 9 + MS header 7 + jacks (in 6 + 6, out
	// 9 + 9) + 2 x (audio endpoint 9 + class specific endpoint 5) = 92
	CHECK(parseConfig(usb_cdc_midi_conf).totalLength == 9 + 66 + 92);
}

TEST_CASE("every configuration declares as many interfaces as it contains, numbered from 0 without gaps or duplicates") {
	for (const NamedConfig &nc : allConfigs()) {
		INFO(nc.name);
		const ConfigDesc c = parseConfig(nc.conf);
		REQUIRE(c.error.empty());
		std::set<uint8_t> numbers;
		for (const Interface &i : c.interfaces) {
			CHECK(i.alternate == 0);
			numbers.insert(i.number);
		}
		CHECK(numbers.size() == c.interfaces.size());
		CHECK(c.interfaces.size() == c.numInterfaces);
		CHECK(*numbers.begin() == 0);
		CHECK(*numbers.rbegin() == c.numInterfaces - 1);
	}
}

TEST_CASE("the interface counts are 2 for CDC only, 3 for CDC with HID and 4 for CDC with MIDI") {
	CHECK(parseConfig(usb_cdc_conf).numInterfaces == 2);
	CHECK(parseConfig(usb_cdc_hid_conf_2axis).numInterfaces == 3);
	CHECK(parseConfig(usb_cdc_hid_conf_gamepad).numInterfaces == 3);
	CHECK(parseConfig(usb_cdc_midi_conf).numInterfaces == 4);
}

TEST_CASE("every interface is followed by as many endpoint descriptors as its bNumEndpoints") {
	for (const NamedConfig &nc : allConfigs()) {
		INFO(nc.name);
		const ConfigDesc c = parseConfig(nc.conf);
		REQUIRE(c.error.empty());
		for (const Interface &i : c.interfaces) {
			INFO("interface " << (int)i.number);
			CHECK(i.endpoints.size() == i.numEndpoints);
		}
	}
}

TEST_CASE("endpoint addresses are unique in every configuration and use endpoints 1 to 3 of the OTG FS core") {
	// STM32F407 OTG_FS: 4 bidirectional endpoints including the control endpoint 0 (RM0090 34.3.2)
	for (const NamedConfig &nc : allConfigs()) {
		INFO(nc.name);
		const ConfigDesc c = parseConfig(nc.conf);
		REQUIRE(c.error.empty());
		std::set<uint8_t> addresses;
		size_t count = 0;
		for (const Interface &i : c.interfaces) {
			for (const Endpoint &e : i.endpoints) {
				INFO("endpoint 0x" << std::hex << (int)e.address);
				CHECK((e.address & 0x70) == 0); // reserved bits
				CHECK((e.address & 0x0F) >= 1);
				CHECK((e.address & 0x0F) <= 3);
				addresses.insert(e.address);
				count++;
			}
		}
		CHECK(addresses.size() == count);
	}
}

TEST_CASE("the CDC function uses notification endpoint 0x82 and data endpoints 0x01 and 0x81 in every configuration") {
	for (const NamedConfig &nc : allConfigs()) {
		INFO(nc.name);
		const ConfigDesc c = parseConfig(nc.conf);
		REQUIRE(c.error.empty());
		const Interface *comm = findInterface(c, CLASS_CDC);
		const Interface *data = findInterface(c, CLASS_CDC_DATA);
		REQUIRE(comm != nullptr);
		REQUIRE(data != nullptr);
		REQUIRE(comm->endpoints.size() == 1);
		CHECK(comm->endpoints[0].address == 0x82);
		CHECK((comm->endpoints[0].attributes & 3) == XFER_INTERRUPT);
		CHECK(comm->endpoints[0].maxPacketSize == 8);
		REQUIRE(data->endpoints.size() == 2);
		CHECK(data->endpoints[0].address == 0x01);
		CHECK(data->endpoints[1].address == 0x81);
		for (const Endpoint &e : data->endpoints) {
			CHECK((e.attributes & 3) == XFER_BULK);
			CHECK(e.maxPacketSize == 64); // full speed bulk: 8, 16, 32 or 64
		}
		CHECK(comm->stringIndex == 4);
	}
}

TEST_CASE("the interface association descriptor groups the two CDC interfaces") {
	for (const NamedConfig &nc : allConfigs()) {
		INFO(nc.name);
		const ConfigDesc c = parseConfig(nc.conf);
		REQUIRE(c.error.empty());
		const Interface *comm = findInterface(c, CLASS_CDC);
		REQUIRE(comm != nullptr);
		REQUIRE(c.iads.size() >= 1);
		const Iad &iad = c.iads.front();
		CHECK(iad.functionClass == CLASS_CDC);
		CHECK(iad.firstInterface == comm->number);
		CHECK(iad.interfaceCount == 2);
		const Interface *data = findInterface(c, CLASS_CDC_DATA);
		REQUIRE(data != nullptr);
		CHECK(data->number == comm->number + 1);
		CHECK(c.types[1] == DESC_IAD); // the IAD precedes the interfaces it groups
	}
}

TEST_CASE("the CDC union and call management descriptors name the CDC control and data interfaces") {
	for (const NamedConfig &nc : allConfigs()) {
		INFO(nc.name);
		const ConfigDesc c = parseConfig(nc.conf);
		REQUIRE(c.error.empty());
		const Interface *comm = findInterface(c, CLASS_CDC);
		const Interface *data = findInterface(c, CLASS_CDC_DATA);
		REQUIRE(comm != nullptr);
		REQUIRE(data != nullptr);
		bool union_found = false, call_found = false;
		for (const auto &d : comm->classDescriptors) {
			if (d[2] == CDC_FUNC_UNION) {
				union_found = true;
				REQUIRE(d.size() >= 5);
				CHECK(d[3] == comm->number);
				CHECK(d[4] == data->number);
			}
			if (d[2] == CDC_FUNC_CALL_MANAGEMENT) {
				call_found = true;
				REQUIRE(d.size() >= 5);
				CHECK(d[4] == data->number);
			}
		}
		CHECK(union_found);
		CHECK(call_found);
	}
}

TEST_CASE("every configuration is self powered with remote wakeup, reserved bit 7 set and 100 mA") {
	for (const NamedConfig &nc : allConfigs()) {
		INFO(nc.name);
		const ConfigDesc c = parseConfig(nc.conf);
		CHECK(c.attributes == (0x80 | 0x40 | 0x20));
		CHECK(c.maxPower == 50); // 2 mA units
		CHECK(c.configValue == 1);
		CHECK(c.stringIndex == 0);
	}
}

TEST_CASE("the HID interface has an interrupt IN 0x83 and OUT 0x02 endpoint of 64 bytes polled every millisecond") {
	// USB_CONF_DESC_HID_CDC passes 0x83 as _epout and 0x02 as _epin of TUD_HID_INOUT_DESCRIPTOR: the IN endpoint is
	// listed first. The direction is in the address (bit 7), so the order does not matter.
	for (const ConfigReportPair &p : configReportPairs()) {
		INFO(p.name);
		const ConfigDesc c = parseConfig(p.conf);
		REQUIRE(c.error.empty());
		const Interface *hid = findInterface(c, CLASS_HID);
		REQUIRE(hid != nullptr);
		CHECK(hid->number == 2);
		CHECK(hid->subclass == 0); // no boot interface
		CHECK(hid->protocol == 0);
		CHECK(hid->stringIndex == 5);
		REQUIRE(hid->endpoints.size() == 2);
		std::set<uint8_t> addresses = {hid->endpoints[0].address, hid->endpoints[1].address};
		CHECK(addresses == std::set<uint8_t>{0x02, 0x83});
		for (const Endpoint &e : hid->endpoints) {
			CHECK((e.attributes & 3) == XFER_INTERRUPT);
			CHECK(e.maxPacketSize == 64);
			CHECK(e.maxPacketSize <= CFG_TUD_HID_EP_BUFSIZE); // TinyUSB's HID endpoint buffer
			CHECK(e.interval == 1);
		}
	}
}

TEST_CASE("the HID descriptor of every HID configuration refers to exactly one report descriptor") {
	for (const ConfigReportPair &p : configReportPairs()) {
		INFO(p.name);
		const ConfigDesc c = parseConfig(p.conf);
		REQUIRE(c.error.empty());
		const Interface *hid = findInterface(c, CLASS_HID);
		REQUIRE(hid != nullptr);
		CHECK(hid->hasHidDescriptor);
		CHECK(hid->hidNumDescriptors == 1);
		CHECK(hid->hidFirstDescriptorType == DESC_HID_REPORT);
	}
}

TEST_CASE("the HID descriptor's wDescriptorLength equals the parsed length of the report descriptor used with it") {
	for (const ConfigReportPair &p : configReportPairs()) {
		INFO(p.name);
		const ConfigDesc c = parseConfig(p.conf);
		REQUIRE(c.error.empty());
		const Interface *hid = findInterface(c, CLASS_HID);
		REQUIRE(hid != nullptr);
		const HidReportDesc r = parseHidReport(p.desc, p.size);
		INFO(r.error);
		REQUIRE(r.error.empty());
		CHECK(r.lastEndCollection == hid->hidReportLength);
	}
}

TEST_CASE("the CDC only configuration has no HID interface") {
	const ConfigDesc c = parseConfig(usb_cdc_conf);
	REQUIRE(c.error.empty());
	CHECK(findInterface(c, CLASS_HID) == nullptr);
}

TEST_CASE("the MIDI configuration has an audio control and a MIDI streaming interface with endpoints 0x02 and 0x83") {
	const ConfigDesc c = parseConfig(usb_cdc_midi_conf);
	REQUIRE(c.error.empty());
	REQUIRE(c.interfaces.size() == 4);
	CHECK(c.interfaces[2].cls == CLASS_AUDIO);
	CHECK(c.interfaces[2].subclass == 1); // audio control
	CHECK(c.interfaces[2].stringIndex == 6);
	CHECK(c.interfaces[3].cls == CLASS_AUDIO);
	CHECK(c.interfaces[3].subclass == 3); // MIDI streaming
	REQUIRE(c.interfaces[3].endpoints.size() == 2);
	CHECK(c.interfaces[3].endpoints[0].address == 0x02);
	CHECK(c.interfaces[3].endpoints[1].address == 0x83);
	for (const Endpoint &e : c.interfaces[3].endpoints) {
		CHECK((e.attributes & 3) == XFER_BULK);
		CHECK(e.maxPacketSize == 64);
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// HID report descriptors
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE("every HID report descriptor consists of valid items and its array ends with the End Collection of its last collection") {
	for (const NamedReport &nr : allReports()) {
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		INFO(r.error);
		CHECK(r.error.empty());
		CHECK(r.length == nr.size);			  // no truncated item
		CHECK(r.depthAtEnd == 0);			  // all collections closed
		CHECK(r.lastEndCollection == nr.size); // no bytes (e.g. zero padding of a too large array) after the last collection
	}
}

TEST_CASE("every HID report descriptor starts with a Generic Desktop joystick application collection") {
	const uint8_t expected[] = {0x05, 0x01, 0x09, 0x04, 0xA1, 0x01}; // usage page 1, usage 4 (joystick), collection 1
	for (const NamedReport &nr : allReports()) {
		INFO(nr.name);
		REQUIRE(nr.size > sizeof(expected));
		CHECK(std::memcmp(nr.desc, expected, sizeof(expected)) == 0);
	}
}

TEST_CASE("every report of the HID report descriptors has a report id") {
	for (const NamedReport &nr : allReports()) {
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		REQUIRE(r.error.empty());
		CHECK_FALSE(r.mainItemWithoutId);
		CHECK(r.bits.count({MainKind::input, 0}) == 0);
		CHECK(r.bits.count({MainKind::output, 0}) == 0);
		CHECK(r.bits.count({MainKind::feature, 0}) == 0);
	}
}

TEST_CASE("the logical minimum of every data field of the HID report descriptors is not above its logical maximum") {
	for (const NamedReport &nr : allReports()) {
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		REQUIRE(r.error.empty());
		for (const std::string &e : r.rangeErrors) {
			FAIL_CHECK(e);
		}
	}
}

TEST_CASE("every input and output report fits into one 64 byte HID packet including its report id") {
	for (const NamedReport &nr : allReports()) {
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		REQUIRE(r.error.empty());
		for (const auto &e : r.bits) {
			if (e.first.first == MainKind::feature) {
				continue; // feature reports use the control endpoint
			}
			INFO("report id " << (int)e.first.second);
			CHECK(e.second % 8 == 0);
			CHECK((e.second + 7) / 8 + 1 <= 64);
		}
	}
}

TEST_CASE("the gamepad input report 1 has the size of the firmware's gamepad report struct") {
	// reportHID_t<T> (ffb_defs.h) is sent including its id: 64 buttons, MAX_AXIS main axes of type T and 6 16 bit axes
	for (const NamedReport &nr : allReports()) {
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		REQUIRE(r.error.empty());
		REQUIRE(hasReport(r, MainKind::input, 1));
		const size_t expected = nr.axes32 ? sizeof(reportHID_t<int32_t>) : sizeof(reportHID_t<int16_t>);
		CHECK(reportBytes(r, MainKind::input, 1) + 1 == expected);
	}
}

TEST_CASE("the HID command reports 0xA1 in and out have the size of HID_CMD_Data_t") {
	for (const NamedReport &nr : allReports()) {
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		REQUIRE(r.error.empty());
		REQUIRE(hasReport(r, MainKind::input, HID_ID_HIDCMD));
		REQUIRE(hasReport(r, MainKind::output, HID_ID_HIDCMD));
		CHECK(reportBytes(r, MainKind::input, HID_ID_HIDCMD) + 1 == sizeof(HID_CMD_Data_t));
		CHECK(reportBytes(r, MainKind::output, HID_ID_HIDCMD) + 1 == sizeof(HID_CMD_Data_t));
	}
}

TEST_CASE("the gamepad descriptors contain no PID reports and only the gamepad and HID command report ids") {
	for (const NamedReport &nr : allReports()) {
		if (nr.ffb) {
			continue;
		}
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		REQUIRE(r.error.empty());
		CHECK(r.usagePages.count(0x0F) == 0); // Physical Interface Device page
		CHECK(reportIds(r, MainKind::input) == std::set<uint8_t>{1, HID_ID_HIDCMD});
		CHECK(reportIds(r, MainKind::output) == std::set<uint8_t>{HID_ID_HIDCMD});
		CHECK(reportIds(r, MainKind::feature).empty());
	}
}

TEST_CASE("the input report ids of the FFB descriptors are the gamepad, the PID state and the HID command report") {
	for (const NamedReport &nr : allReports()) {
		if (!nr.ffb) {
			continue;
		}
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		REQUIRE(r.error.empty());
		CHECK(r.usagePages.count(0x0F) == 1);
		CHECK(reportIds(r, MainKind::input) == std::set<uint8_t>{1, HID_ID_STATE + FFB_ID_OFFSET, HID_ID_HIDCMD});
	}
}

TEST_CASE("the output report ids of the FFB descriptors are the PID reports HidFFB handles and the HID command report") {
	// HidFFB::hidOut (HidFFB.cpp:101-150) handles these output report ids
	const std::set<uint8_t> expected = {
		HID_ID_EFFREP + FFB_ID_OFFSET, HID_ID_ENVREP + FFB_ID_OFFSET, HID_ID_CONDREP + FFB_ID_OFFSET,
		HID_ID_PRIDREP + FFB_ID_OFFSET, HID_ID_CONSTREP + FFB_ID_OFFSET, HID_ID_RAMPREP + FFB_ID_OFFSET,
		HID_ID_EFOPREP + FFB_ID_OFFSET, HID_ID_BLKFRREP + FFB_ID_OFFSET, HID_ID_CTRLREP + FFB_ID_OFFSET,
		HID_ID_GAINREP + FFB_ID_OFFSET, HID_ID_HIDCMD};
	for (const NamedReport &nr : allReports()) {
		if (!nr.ffb) {
			continue;
		}
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		REQUIRE(r.error.empty());
		CHECK(reportIds(r, MainKind::output) == expected);
	}
}

TEST_CASE("the feature report ids of the FFB descriptors are create new effect, block load and PID pool") {
	for (const NamedReport &nr : allReports()) {
		if (!nr.ffb) {
			continue;
		}
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		REQUIRE(r.error.empty());
		CHECK(reportIds(r, MainKind::feature) ==
			  std::set<uint8_t>{HID_ID_NEWEFREP + FFB_ID_OFFSET, HID_ID_BLKLDREP + FFB_ID_OFFSET, HID_ID_POOLREP + FFB_ID_OFFSET});
	}
}

TEST_CASE("the FFB report sizes of the descriptors match the structs HidFFB copies them into") {
	// Output reports include the report id in the struct; feature reports are exchanged without it (ffb_defs.h)
	for (const NamedReport &nr : allReports()) {
		if (!nr.ffb) {
			continue;
		}
		INFO(nr.name);
		const HidReportDesc r = parseHidReport(nr.desc, nr.size);
		REQUIRE(r.error.empty());
		CHECK(reportBytes(r, MainKind::input, HID_ID_STATE) + 1 == sizeof(reportFFB_status_t));
		CHECK(reportBytes(r, MainKind::output, HID_ID_CONDREP) + 1 == sizeof(FFB_SetCondition_Data_t));
		CHECK(reportBytes(r, MainKind::output, HID_ID_PRIDREP) + 1 == sizeof(FFB_SetPeriodic_Data_t));
		CHECK(reportBytes(r, MainKind::output, HID_ID_ENVREP) + 1 == sizeof(FFB_SetEnvelope_Data_t));
		CHECK(reportBytes(r, MainKind::output, HID_ID_RAMPREP) + 1 == sizeof(FFB_SetRamp_Data_t));
		CHECK(reportBytes(r, MainKind::output, HID_ID_CONSTREP) + 1 == sizeof(FFB_SetConstantForce_Data_t));
		CHECK(reportBytes(r, MainKind::output, HID_ID_EFOPREP) + 1 == sizeof(FFB_EffOp_Data_t));
		CHECK(reportBytes(r, MainKind::feature, HID_ID_NEWEFREP) == sizeof(FFB_CreateNewEffect_Feature_Data_t));
		CHECK(reportBytes(r, MainKind::feature, HID_ID_BLKLDREP) == sizeof(FFB_BlockLoad_Feature_Data_t));
		CHECK(reportBytes(r, MainKind::feature, HID_ID_POOLREP) == sizeof(FFB_PIDPool_Feature_Data_t));
	}
}

TEST_CASE("the set effect report of the 2 axis descriptors is FFB_SetEffect_t followed by one 16 bit block offset per axis") {
	// The descriptors end the set effect report with the "type specific block offset" of each axis (2 bytes each),
	// which FFB_SetEffect_t does not contain: HidFFB::hidOut copies min(sizeof(FFB_SetEffect_t), bufsize) and ignores
	// them. The 1 axis descriptor has no Y direction, so its offset lands in FFB_SetEffect_t::directionY.
	struct Expected {
		const uint8_t *desc;
		size_t size;
		uint32_t axes;
	};
	const Expected expected[] = {
		{hid_2ffb_desc, sizeof(hid_2ffb_desc), 2},
		{variant_2axis::hid_2ffb_desc_32b, sizeof(variant_2axis::hid_2ffb_desc_32b), 2},
	};
	const size_t directions = 2 * sizeof(uint16_t); // directionX, directionY
	for (const Expected &e : expected) {
		INFO("axes " << e.axes);
		const HidReportDesc r = parseHidReport(e.desc, e.size);
		REQUIRE(r.error.empty());
		CHECK(reportBytes(r, MainKind::output, HID_ID_EFFREP) + 1 ==
			  sizeof(FFB_SetEffect_t) - directions + e.axes * sizeof(uint16_t) + e.axes * sizeof(uint16_t));
	}
}

TEST_CASE("characterization: the set effect report of the 1 axis descriptor puts the X block offset where FFB_SetEffect_t has directionY (NEEDS REVIEW 75)") {
	// NEEDS REVIEW 75: the 1 axis descriptor (FFBWHEEL_USE_1AXIS_DESC, not used by the F407) has no Y direction, so the
	// type specific block offset of X lands in FFB_SetEffect_t::directionY (HidFFB::hidOut copies min(sizeof, bufsize)).
	// Is directionY ignored for 1 axis devices?
	const HidReportDesc r = parseHidReport(variant_1axis::hid_1ffb_desc, sizeof(variant_1axis::hid_1ffb_desc));
	REQUIRE(r.error.empty());
	CHECK(reportBytes(r, MainKind::output, HID_ID_EFFREP) + 1 == sizeof(FFB_SetEffect_t)); // directionY replaced by the offset
}

// ---------------------------------------------------------------------------------------------------------------------
// TinyUSB descriptor callbacks (global_callbacks.cpp -> USBdevice) per main class configuration
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(DescriptorFixture, "the device and configuration callbacks return the tables of the registered USB device") {
	registerDevice(usb_cdc_conf);
	CHECK(tud_descriptor_device_cb() == reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite));
	CHECK(tud_descriptor_configuration_cb(0) == usb_cdc_conf);
}

TEST_CASE_FIXTURE(DescriptorFixture, "the string callback returns language 0x0409 for string index 0") {
	registerDevice(usb_cdc_conf);
	const uint16_t *s = tud_descriptor_string_cb(0, 0);
	REQUIRE(s != nullptr);
	CHECK(s[0] == ((DESC_STRING << 8) | 4));
	CHECK(s[1] == 0x0409);
}

TEST_CASE_FIXTURE(DescriptorFixture, "the manufacturer and product strings are Open FFBoard and FFBoard with the hardware type") {
	registerDevice(usb_cdc_conf);
	CHECK(decodeString(tud_descriptor_string_cb(1, 0x0409)) == "Open FFBoard");
	CHECK(decodeString(tud_descriptor_string_cb(2, 0x0409)) == "FFBoard F407VG");
}

TEST_CASE_FIXTURE(DescriptorFixture, "the serial number string is the 96 bit unique id as 24 hex digits, word 2 first") {
	registerDevice(usb_cdc_conf);
	HostTarget::state().uid[0] = 0x00410032;
	HostTarget::state().uid[1] = 0x34385111;
	HostTarget::state().uid[2] = 0x3038363a;
	CHECK(decodeString(tud_descriptor_string_cb(3, 0x0409)) == "3038363a3438511100410032");
}

TEST_CASE_FIXTURE(DescriptorFixture, "the interface strings 4 to 6 start with the CDC, HID and MIDI names followed by a bracketed suffix") {
	registerDevice(usb_cdc_conf);
	const std::pair<uint8_t, std::string> names[] = {{4, "FFBoard CDC("}, {5, "FFBoard HID("}, {6, "FFBoard MIDI("}};
	for (const auto &n : names) {
		const std::string s = decodeString(tud_descriptor_string_cb(n.first, 0x0409));
		INFO(s);
		CHECK(s.rfind(n.second, 0) == 0);
		CHECK(s.back() == ')');
	}
}

TEST_CASE_FIXTURE(DescriptorFixture, "string index 7 and above have no string") {
	registerDevice(usb_cdc_conf);
	CHECK(tud_descriptor_string_cb(7, 0x0409) == nullptr);
	CHECK(tud_descriptor_string_cb(0xEE, 0x0409) == nullptr); // Microsoft OS string descriptor
}

TEST_CASE_FIXTURE(DescriptorFixture, "the interface strings end with the last 4 characters of the serial number in brackets" *
		  KNOWN_ISSUE("(bug 47) USBdevice.cpp:81 appends the first 4 decimal digits of UID word 0, USBdevice.h:24 documents the last n "
					  "characters of the serial number")) {
	registerDevice(usb_cdc_conf);
	const std::string serial = decodeString(tud_descriptor_string_cb(3, 0x0409));
	REQUIRE(serial.size() == 24);
	// Default uid: serial 30383632 34385111 00410032, so the suffix is (0032). Today: (4259) = first digits of 4259890
	const std::string cdc = decodeString(tud_descriptor_string_cb(4, 0x0409)); // reaches HAL_GetUIDw0
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(cdc == "FFBoard CDC(" + serial.substr(20) + ")"); // first failing assertion
}

TEST_CASE_FIXTURE(DescriptorFixture, "every string index used by the device and configuration descriptors has a string") {
	registerDevice(usb_cdc_conf);
	const uint8_t *d = reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite);
	std::set<uint8_t> indices = {d[14], d[15], d[16]};
	for (const NamedConfig &nc : allConfigs()) {
		const ConfigDesc c = parseConfig(nc.conf);
		REQUIRE(c.error.empty());
		indices.insert(c.stringIndex);
		for (const Interface &i : c.interfaces) {
			indices.insert(i.stringIndex);
		}
	}
	indices.erase(0); // 0 = no string
	for (uint8_t index : indices) {
		INFO("string index " << (int)index);
		const uint16_t *s = tud_descriptor_string_cb(index, 0x0409);
		REQUIRE(s != nullptr);
		CHECK_FALSE(decodeString(s).empty());
	}
}

TEST_CASE_FIXTURE(DescriptorFixture, "the failsafe main class serves the CDC only configuration") {
	FFBoardMain main;
	main.usbInit();
	REQUIRE(usb_device != nullptr);
	const ConfigDesc c = parseConfig(tud_descriptor_configuration_cb(0));
	REQUIRE(c.error.empty());
	CHECK(c.numInterfaces == 2);
	CHECK(findInterface(c, CLASS_HID) == nullptr);
	CHECK(tud_descriptor_device_cb() == reinterpret_cast<const uint8_t *>(&usb_devdesc_ffboard_composite));
}

TEST_CASE_FIXTURE(DescriptorFixture, "a wheel serves a configuration whose HID report length matches the report descriptor it serves" * DELETES_FFBHIDMAIN) {
	FFBWheel wheel;
	wheel.usbInit();
	checkServedDescriptorsMatch();
	CHECK(tud_hid_descriptor_report_cb(0) == hid_2ffb_desc);
}

TEST_CASE_FIXTURE(DescriptorFixture, "a joystick serves a configuration whose HID report length matches the report descriptor it serves" * DELETES_FFBHIDMAIN) {
	FFBJoystick joystick;
	joystick.usbInit();
	checkServedDescriptorsMatch();
	CHECK(tud_hid_descriptor_report_cb(0) == hid_2ffb_desc);
}

TEST_CASE_FIXTURE(DescriptorFixture, "an external FFB gamepad serves a configuration whose HID report length matches the report descriptor it serves" * DELETES_FFBHIDMAIN) {
	FFBHIDExt ext;
	ext.usbInit();
	checkServedDescriptorsMatch();
	CHECK(tud_hid_descriptor_report_cb(0) == hid_gamepad_desc);
}

} // TEST_SUITE
