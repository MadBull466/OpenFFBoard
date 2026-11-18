/*
 * F1 support self-tests (WO I0-6): the HID report-descriptor parser (support/HidReportDescriptor) and the PID report builder
 * (support/PidReportBuilder). Pure host code, no fixture, no firmware: every expected value is derived by hand from
 *  - HID 1.11 (Tests/Code_Coverage/shared/specs/hid1_11.pdf): 5.8 (little-endian values), 6.2.2.2 items (abbreviated data),
 *    6.2.2.3 long items, 6.2.2.4-6.2.2.8 main / global / local items, Appendix D.1 (joystick, p. 64-65) and Appendix E.6 / E.10
 *    (keyboard p. 69, mouse p. 71 report descriptors);
 *  - USB_PID1_01.pdf Table 2 (usage IDs, pages 6-7).
 * The descriptors are written byte by byte below; the item prefix bytes follow 6.2.2.2: tag(4 bits) type(2) size(2) with
 * type 0 main, 1 global, 2 local and size codes 0/1/2/3 = 0/1/2/4 data bytes (so 0x05 = Usage Page 1 byte, 0x15 = Logical
 * Minimum 1 byte, 0x25 = Logical Maximum 1 byte, 0x75 = Report Size 1 byte, 0x95 = Report Count 1 byte, 0x81 = Input 1 byte,
 * 0x91 = Output 1 byte, 0xB1 = Feature 1 byte, 0xA1 = Collection, 0xC0 = End Collection, 0x09 = Usage, 0x19 / 0x29 = Usage
 * Minimum / Maximum, 0x85 = Report ID, 0xA4 = Push, 0xB4 = Pop).
 */
#include "doctest.h"

#include "HidReportDescriptor.h"
#include "PidReportBuilder.h"
#include "FrequencyResponse.h"
#include "OscillationDetector.h"
#include "../../fakes/WheelPlant.h"

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <vector>

using hid::Field;
using hid::HidReportDescriptor;
using hid::Report;
using hid::ReportType;
using hid::usage;

namespace {

std::vector<uint8_t> B(std::initializer_list<int> v) {
	std::vector<uint8_t> out;
	for (int x : v) {
		out.push_back(static_cast<uint8_t>(x));
	}
	return out;
}

std::vector<uint8_t> cat(std::vector<uint8_t> a, const std::vector<uint8_t> &b) {
	a.insert(a.end(), b.begin(), b.end());
	return a;
}

// HID 1.11 Appendix D.1 (p. 64) example joystick. The text of D.1 has two typos that are corrected here: the second range item
// is Usage Maximum (not a second Usage Minimum), and the Throttle usage ID is a bare number (not asserted).
const std::vector<uint8_t> kJoystick = B({
	0x05, 0x01, 0x09, 0x04, 0xA1, 0x01,					 // Usage Page GD, Usage Joystick, Collection Application
	0x05, 0x01, 0x09, 0x01, 0xA1, 0x00,					 // Usage Page GD, Usage Pointer, Collection Physical
	0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x02,		 // Logical -127..127, size 8, count 2
	0xA4,												 // Push
	0x09, 0x30, 0x09, 0x31, 0x81, 0x02,					 // Usage X, Y; Input (Data, Var, Abs)
	0x09, 0x39,											 // Usage Hat switch
	0x15, 0x00, 0x25, 0x03,								 // Logical 0..3
	0x35, 0x00, 0x46, 0x0E, 0x01,						 // Physical 0..270 (0x010E, 2 byte data)
	0x65, 0x14,											 // Unit: degrees (English rotation, length nibble 1)
	0x95, 0x01, 0x75, 0x04, 0x81, 0x42,					 // count 1, size 4, Input (Data, Var, Abs, Null state)
	0x15, 0x00, 0x25, 0x01, 0x95, 0x02, 0x75, 0x01,		 // Logical 0..1, count 2, size 1
	0x05, 0x09, 0x19, 0x01, 0x29, 0x02, 0x65, 0x00,		 // Usage Page Buttons, Usage Min 1, Max 2, Unit none
	0x81, 0x02,											 // Input (Data, Var, Abs)
	0xC0,												 // End Collection
	0x19, 0x03, 0x29, 0x04, 0x81, 0x02,					 // Usage Min 3, Max 4, Input
	0xB4,												 // Pop
	0x09, 0xBB, 0x95, 0x01, 0x81, 0x02,					 // Usage (throttle id, not asserted), count 1, Input
	0xC0,												 // End Collection
});

// Appendix E.6 (p. 69) keyboard
const std::vector<uint8_t> kKeyboard = B({
	0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
	0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03,
	0x91, 0x01, 0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0,
});

// Appendix E.10 (p. 71) mouse, 50 bytes (wItemLength 0x32 in E.8)
const std::vector<uint8_t> kMouse = B({
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01, 0x95, 0x03,
	0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08,
	0x95, 0x02, 0x81, 0x06, 0xC0, 0xC0,
});

// Synthetic PID-like descriptor, derived by hand (see the cases). Output report 1 = Logical collection 0x0F:0x21 ("Set Effect
// Report", USB_PID1_01.pdf Table 2) with, in this order and after the ID byte:
//   bits  0-7   Effect Block Index (0x22), 8 bit, logical 1..40
//   bits  8-15  Effect Type (0x25) named array of the selectors ET Constant Force (0x26, value 1) and ET Sine (0x31, value 2)
//   bits 16-31  Duration (0x50), 16 bit, logical 0..32767
//   bit  32     Direction Enable (0x56), 1 bit
//   bits 33-48  Magnitude (0x70), 16 bit signed -32767..32767: straddles three bytes
// 49 bits -> 7 bytes, plus the ID byte = 8.
// Input report 2 = Logical collection 0x92 ("PID State Report"): bit 0 Device Paused (0x9F), bit 1 Actuators Enabled (0xA0),
// bit 2 Effect Playing (0x94), bits 3-7 constant padding, bits 8-15 Effect Block Index (1..40): 2 bytes + ID = 3.
const std::vector<uint8_t> kPidLike = B({
	0x05, 0x0F,
	0x09, 0x21, 0xA1, 0x02, 0x85, 0x01,
	0x09, 0x22, 0x15, 0x01, 0x25, 0x28, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
	0x09, 0x25, 0xA1, 0x02, 0x09, 0x26, 0x09, 0x31, 0x15, 0x01, 0x25, 0x02, 0x75, 0x08, 0x95, 0x01, 0x91, 0x00, 0xC0,
	0x09, 0x50, 0x15, 0x00, 0x26, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
	0x09, 0x56, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x91, 0x02,
	0x09, 0x70, 0x16, 0x01, 0x80, 0x26, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
	0xC0,
	0x09, 0x92, 0xA1, 0x02, 0x85, 0x02,
	0x09, 0x9F, 0x09, 0xA0, 0x09, 0x94, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x03, 0x81, 0x02,
	0x75, 0x05, 0x95, 0x01, 0x81, 0x03,
	0x09, 0x22, 0x15, 0x01, 0x25, 0x28, 0x75, 0x08, 0x95, 0x01, 0x81, 0x02,
	0xC0,
});

// PID 1.01 section 6 "PID Device Control Report Definition" (bytes 91 02, Report Size 1, Count 8, Logical 1..6), report ID 12
const std::vector<uint8_t> kDeviceControl = B({
	0x05, 0x0F, 0x09, 0x95, 0xA1, 0x02, 0x85, 0x0C, 0x09, 0x96, 0xA1, 0x02,
	0x09, 0x97, 0x09, 0x98, 0x09, 0x99, 0x09, 0x9A, 0x09, 0x9B, 0x09, 0x9C,
	0x15, 0x01, 0x25, 0x06, 0x75, 0x01, 0x95, 0x08, 0x91, 0x02, 0xC0, 0xC0,
});

bool parses(HidReportDescriptor &d, const std::vector<uint8_t> &b) {
	const bool ok = d.parse(b);
	INFO("parse error: " << d.error());
	return ok;
}

// A descriptor that must fail with a message containing `what`
void expectError(const std::vector<uint8_t> &bytes, const char *what) {
	HidReportDescriptor d;
	CHECK_FALSE(d.parse(bytes));
	INFO("error text: " << d.error());
	CHECK(d.error().find(what) != std::string::npos);
	CHECK(d.fields().empty());
	CHECK(d.reports().empty());
}

} // namespace

TEST_SUITE("F1Support") {

// ---------------------------------------------------------------------------------------------------------------------
// Parser, examples of the specification
// ---------------------------------------------------------------------------------------------------------------------

// Hand derivation from Appendix D.1 (p. 64-65), which also gives the byte layout table: X byte 0, Y byte 1, hat bits 0-3 of byte
// 2, buttons 1..4 bits 4..7 of byte 2, throttle byte 3. Without Report ID the report has no ID byte: 4 bytes.
// Push at (size 8, count 2, -127..127) and Pop restore those for the throttle (size 8, -127..127) after the buttons changed them.
TEST_CASE("HID 1.11 Appendix D.1 joystick: fields, offsets, push/pop, physical range and unit") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kJoystick));
	CHECK_FALSE(d.usesReportIds());
	REQUIRE(d.reports().size() == 1);
	const Report &r = d.reports()[0];
	CHECK(r.type == ReportType::Input);
	CHECK(r.reportId == 0);
	CHECK_FALSE(r.hasIdByte);
	CHECK(r.bitLength == 32);
	CHECK(r.byteLength() == 4);
	REQUIRE(d.fields().size() == 5);
	const Field &xy = d.fields()[0];
	CHECK(xy.bitOffset == 0);
	CHECK(xy.bitSize == 8);
	CHECK(xy.count == 2);
	CHECK(xy.logicalMin == -127);
	CHECK(xy.logicalMax == 127);
	CHECK(xy.isSigned);
	CHECK(xy.usages == std::vector<uint32_t>{usage(1, 0x30), usage(1, 0x31)});
	CHECK(xy.usageOf(0) == usage(1, 0x30));
	CHECK(xy.usageOf(1) == usage(1, 0x31));
	CHECK(xy.isVariable());
	CHECK_FALSE(xy.hasPhysical);
	CHECK(xy.physicalMin == -127); // undeclared physical extents equal the logical ones (6.2.2.7 remarks)
	CHECK(xy.physicalMax == 127);
	const Field &hat = d.fields()[1];
	CHECK(hat.bitOffset == 16);
	CHECK(hat.bitSize == 4);
	CHECK(hat.count == 1);
	CHECK(hat.logicalMin == 0);
	CHECK(hat.logicalMax == 3);
	CHECK_FALSE(hat.isSigned);
	CHECK(hat.hasPhysical);
	CHECK(hat.physicalMin == 0);
	CHECK(hat.physicalMax == 270);
	CHECK(hat.unit == 0x14);
	CHECK((hat.flags & hid::FlagNullState) != 0);
	CHECK(hat.usages == std::vector<uint32_t>{usage(1, 0x39)});
	const Field &b12 = d.fields()[2];
	CHECK(b12.bitOffset == 20);
	CHECK(b12.bitSize == 1);
	CHECK(b12.count == 2);
	CHECK(b12.hasRange);
	CHECK(b12.usageMin == usage(9, 1));
	CHECK(b12.usageMax == usage(9, 2));
	CHECK(b12.usageOf(1) == usage(9, 2));
	CHECK(b12.unit == 0);
	const Field &b34 = d.fields()[3];
	CHECK(b34.bitOffset == 22);
	CHECK(b34.usageMin == usage(9, 3));
	CHECK(b34.usageMax == usage(9, 4));
	const Field &thr = d.fields()[4];
	CHECK(thr.bitOffset == 24);
	CHECK(thr.bitSize == 8); // restored by Pop
	CHECK(thr.count == 1);
	CHECK(thr.logicalMin == -127); // restored by Pop
	CHECK(thr.logicalMax == 127);
	CHECK(thr.usages == std::vector<uint32_t>{usage(1, 0xBB)});
	CHECK_FALSE(thr.hasPhysical); // the hat's physical extents were declared after Push
	// the collection path of X: Application (Joystick, GD 4) then Physical (Pointer, GD 1)
	REQUIRE(xy.path.size() == 2);
	CHECK(xy.path[0].type == hid::CollApplication);
	CHECK(xy.path[0].usage == usage(1, 4));
	CHECK(xy.path[1].type == hid::CollPhysical);
	CHECK(xy.path[1].usage == usage(1, 1));
	// the throttle is after End Collection of the pointer
	REQUIRE(thr.path.size() == 1);
	CHECK(thr.path[0].usage == usage(1, 4));
}

// Appendix E.6 (p. 69): input = 8 modifier bits (usage range 224..231), 1 constant byte, 6 key-code bytes as an array
// (0..101); output = 5 LED bits (range 1..5) and 3 constant padding bits. Input 8 + 8 + 48 = 64 bits = 8 bytes; output 8 bits = 1 byte.
TEST_CASE("HID 1.11 Appendix E.6 keyboard: input and output reports, constant fields and the key array") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kKeyboard));
	const Report *in = d.findReport(ReportType::Input, 0);
	const Report *out = d.findReport(ReportType::Output, 0);
	REQUIRE(in != nullptr);
	REQUIRE(out != nullptr);
	CHECK(d.findReport(ReportType::Feature, 0) == nullptr);
	CHECK(in->byteLength() == 8);
	CHECK(out->byteLength() == 1);
	REQUIRE(in->fields.size() == 3);
	const Field &mod = d.fields()[in->fields[0]];
	CHECK(mod.hasRange);
	CHECK(mod.usageMin == usage(7, 224));
	CHECK(mod.usageMax == usage(7, 231));
	CHECK(mod.bitOffset == 0);
	CHECK(mod.count == 8);
	const Field &pad = d.fields()[in->fields[1]];
	CHECK(pad.isConstant());
	CHECK(pad.bitOffset == 8);
	CHECK(pad.bitSize == 8);
	CHECK(pad.usageOf(0) == 0);
	const Field &keys = d.fields()[in->fields[2]];
	CHECK(keys.isArray());
	CHECK(keys.bitOffset == 16);
	CHECK(keys.count == 6);
	CHECK(keys.logicalMin == 0);
	CHECK(keys.logicalMax == 101);
	CHECK(keys.hasRange);
	CHECK(keys.usageMin == usage(7, 0));
	CHECK(keys.usageMax == usage(7, 101));
	int64_t sel = -1;
	CHECK(keys.selectorValue(usage(7, 4), sel)); // array: the value reports the usage index (logical minimum 0 + 4)
	CHECK(sel == 4);
	CHECK_FALSE(keys.selectorValue(usage(7, 102), sel));
	REQUIRE(out->fields.size() == 2);
	const Field &led = d.fields()[out->fields[0]];
	CHECK(led.usageMin == usage(8, 1));
	CHECK(led.usageMax == usage(8, 5));
	CHECK(led.count == 5);
	const Field &ledPad = d.fields()[out->fields[1]];
	CHECK(ledPad.isConstant());
	CHECK(ledPad.bitOffset == 5);
	CHECK(ledPad.bitSize == 3);
}

// Appendix E.10 (p. 71): 3 button bits + 5 constant bits + X, Y relative signed bytes: 3 bytes, no ID byte.
TEST_CASE("HID 1.11 Appendix E.10 mouse: three bytes, relative signed axes after the padded buttons") {
	REQUIRE(kMouse.size() == 50); // wItemLength of E.8
	HidReportDescriptor d;
	REQUIRE(parses(d, kMouse));
	REQUIRE(d.reports().size() == 1);
	CHECK(d.reports()[0].byteLength() == 3);
	REQUIRE(d.fields().size() == 3);
	CHECK(d.fields()[0].bitOffset == 0);
	CHECK(d.fields()[0].count == 3);
	CHECK(d.fields()[1].isConstant());
	CHECK(d.fields()[1].bitOffset == 3);
	CHECK(d.fields()[1].bitSize == 5);
	const Field &xy = d.fields()[2];
	CHECK(xy.bitOffset == 8);
	CHECK(xy.count == 2);
	CHECK((xy.flags & hid::FlagRelative) != 0); // 0x06: Data, Variable, Relative
	CHECK(xy.logicalMin == -127);
	CHECK(xy.logicalMax == 127);
	REQUIRE(xy.path.size() == 2);
	CHECK(xy.path[1].usage == usage(1, 1));
}

// ---------------------------------------------------------------------------------------------------------------------
// Parser, synthetic descriptors
// ---------------------------------------------------------------------------------------------------------------------

// Hand derivation. Interpretation (not one rule of the spec, see HidReportDescriptor.h): 6.2.2.2 lets a data item be abbreviated when
// its high-order bits are zero, and the spec's examples use sign-extended small items (E.10 / D.1 "15 81" = -127). The data bytes are
// little-endian and sign-extended from their size for the minimum; a minimum >= 0 makes the field unsigned and the maximum unsigned.
//   15 80 25 7F          1-byte min 0x80 -> -128, max 0x7F -> 127 (min < 0: signed field)
//   16 00 80 26 FF 7F    2-byte min 0x8000 -> -32768, max 0x7FFF -> 32767
//   15 00 27 FF FF FF FF min 0 (>= 0: unsigned field), 4-byte max 0xFFFFFFFF taken unsigned -> 4294967295
//   15 00 26 FF 00       2-byte max 0x00FF -> 255
//   15 00 25 FF          1-byte max 0xFF with min 0: unsigned interpretation -> 255 (not -1)
//   17 00 00 00 80 27 FF FF FF 7F   4-byte min -2147483648, max 2147483647
// Offsets: sizes 8, 16, 32, 8, 8, 32 -> 0, 8, 24, 56, 64, 72; 104 bits = 13 bytes.
TEST_CASE("item data of size 1, 2 and 4 are sign-extended and the logical maximum follows the unsigned interpretation for a minimum of 0 or more") {
	const std::vector<uint8_t> desc = B({
		0x05, 0x01, 0x09, 0x01, 0xA1, 0x01, 0x95, 0x01,
		0x15, 0x80, 0x25, 0x7F, 0x75, 0x08, 0x09, 0x30, 0x81, 0x02,
		0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F, 0x75, 0x10, 0x09, 0x31, 0x81, 0x02,
		0x15, 0x00, 0x27, 0xFF, 0xFF, 0xFF, 0xFF, 0x75, 0x20, 0x09, 0x32, 0x81, 0x02,
		0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x09, 0x33, 0x81, 0x02,
		0x15, 0x00, 0x25, 0xFF, 0x75, 0x08, 0x09, 0x34, 0x81, 0x02,
		0x17, 0x00, 0x00, 0x00, 0x80, 0x27, 0xFF, 0xFF, 0xFF, 0x7F, 0x75, 0x20, 0x09, 0x35, 0x81, 0x02,
		0xC0,
	});
	HidReportDescriptor d;
	REQUIRE(parses(d, desc));
	REQUIRE(d.fields().size() == 6);
	const int64_t mn[] = {-128, -32768, 0, 0, 0, -2147483648ll};
	const int64_t mx[] = {127, 32767, 4294967295ll, 255, 255, 2147483647ll};
	const uint32_t off[] = {0, 8, 24, 56, 64, 72};
	const bool sgn[] = {true, true, false, false, false, true};
	for (size_t i = 0; i < 6; i++) {
		INFO("field " << i);
		CHECK(d.fields()[i].logicalMin == mn[i]);
		CHECK(d.fields()[i].logicalMax == mx[i]);
		CHECK(d.fields()[i].bitOffset == off[i]);
		CHECK(d.fields()[i].isSigned == sgn[i]);
	}
	CHECK(d.reports()[0].byteLength() == 13);
}

// 6.2.2.8: a 4-byte Usage item is a full 32 bit usage (page in the high 16 bits), otherwise the Usage Page is the high half.
// 0B 21 00 0F 00 = Usage 0x000F0021 (extended) although the current page is 0x0001; 06 0F 00 = Usage Page 0x000F with a
// 2-byte data; 0A 22 00 = Usage with a 2-byte ID 0x0022 on that page.
TEST_CASE("a 4-byte usage is a 32 bit extended usage and a shorter usage takes the current usage page, 2-byte page and usage data included") {
	const std::vector<uint8_t> desc = B({
		0x05, 0x01, 0x09, 0x01, 0xA1, 0x01,
		0x0B, 0x21, 0x00, 0x0F, 0x00, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x81, 0x02,
		0x06, 0x0F, 0x00, 0x0A, 0x22, 0x00, 0x81, 0x02,
		0xC0,
	});
	HidReportDescriptor d;
	REQUIRE(parses(d, desc));
	REQUIRE(d.fields().size() == 2);
	CHECK(d.fields()[0].usages == std::vector<uint32_t>{0x000F0021u});
	CHECK(d.fields()[1].usages == std::vector<uint32_t>{0x000F0022u});
}

// 6.2.2.3: a long item is 0xFE, bDataSize, bLongItemTag, data. It is skipped and counted: here FE 03 10 AA BB CC (data size 3).
TEST_CASE("a long item is recognised, counted and skipped") {
	const std::vector<uint8_t> desc = B({
		0x05, 0x01, 0x09, 0x01, 0xA1, 0x01,
		0xFE, 0x03, 0x10, 0xAA, 0xBB, 0xCC,
		0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x09, 0x30, 0x81, 0x02,
		0xC0,
	});
	HidReportDescriptor d;
	REQUIRE(parses(d, desc));
	CHECK(d.longItems() == 1);
	REQUIRE(d.fields().size() == 1);
	CHECK(d.fields()[0].bitSize == 1);
}

// 6.2.2.7: Push copies the whole global state, including the Report ID; Pop restores it. Hand derivation (data items are
// Output (Data, Var, Abs), logical 0..255, count 1):
//   report 1: usage 0x30 size 8 at bit 0, then (after Pop) usage 0x32 size 8 at bit 8: 16 bits, 3 bytes with the ID
//   report 2 (between Push and Pop): usage 0x31 size 16 at bit 0: 16 bits, 3 bytes
TEST_CASE("Pop restores the global state including report ID and size, and report IDs split the reports") {
	const std::vector<uint8_t> desc = B({
		0x05, 0x01, 0x09, 0x01, 0xA1, 0x01,
		0x85, 0x01, 0x75, 0x08, 0x95, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x09, 0x30, 0x91, 0x02,
		0xA4,
		0x85, 0x02, 0x75, 0x10, 0x09, 0x31, 0x91, 0x02,
		0xB4,
		0x09, 0x32, 0x91, 0x02,
		0xC0,
	});
	HidReportDescriptor d;
	REQUIRE(parses(d, desc));
	CHECK(d.usesReportIds());
	REQUIRE(d.reports().size() == 2);
	const Report *r1 = d.findReport(ReportType::Output, 1);
	const Report *r2 = d.findReport(ReportType::Output, 2);
	REQUIRE(r1 != nullptr);
	REQUIRE(r2 != nullptr);
	CHECK(r1->byteLength() == 3);
	CHECK(r2->byteLength() == 3);
	REQUIRE(r1->fields.size() == 2);
	CHECK(d.fields()[r1->fields[0]].bitOffset == 0);
	CHECK(d.fields()[r1->fields[1]].bitOffset == 8);
	CHECK(d.fields()[r1->fields[1]].bitSize == 8);
	CHECK(d.fields()[r1->fields[1]].usages == std::vector<uint32_t>{usage(1, 0x32)});
	REQUIRE(r2->fields.size() == 1);
	CHECK(d.fields()[r2->fields[0]].bitSize == 16);
}

// 6.2.2.7 Report ID: report IDs are per report type; the same ID may name an Input and an Output report (HID 1.11 8.1).
// The offsets of each (type, id) start at zero after the ID byte.
TEST_CASE("the same report ID in an input, an output and a feature report gives three reports with their own offsets") {
	const std::vector<uint8_t> desc = B({
		0x05, 0x01, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x07, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01,
		0x09, 0x30, 0x81, 0x02, 0x09, 0x31, 0x91, 0x02, 0x09, 0x32, 0xB1, 0x02, 0x09, 0x33, 0x81, 0x02,
		0xC0,
	});
	HidReportDescriptor d;
	REQUIRE(parses(d, desc));
	REQUIRE(d.reports().size() == 3);
	const Report *in = d.findReport(ReportType::Input, 7);
	const Report *out = d.findReport(ReportType::Output, 7);
	const Report *feat = d.findReport(ReportType::Feature, 7);
	REQUIRE(in != nullptr);
	REQUIRE(out != nullptr);
	REQUIRE(feat != nullptr);
	CHECK(in->bitLength == 2); // two input bits
	CHECK(in->byteLength() == 2);
	CHECK(out->bitLength == 1);
	CHECK(out->byteLength() == 2);
	CHECK(d.fields()[in->fields[1]].bitOffset == 1);
	CHECK(d.fields()[out->fields[0]].bitOffset == 0);
	CHECK(d.findReport(ReportType::Input, 8) == nullptr);
}

// 6.2.2.7 Unit Exponent: a 4-bit two's complement nibble: 0x0E = -2, 0x02 = 2, 0x08 = -8. Physical extents 0,0 mean "same as logical".
TEST_CASE("unit exponent is a four bit signed nibble and physical extents of zero fall back to the logical extents") {
	const std::vector<uint8_t> desc = B({
		0x05, 0x01, 0x09, 0x01, 0xA1, 0x01, 0x15, 0x00, 0x25, 0x64, 0x75, 0x08, 0x95, 0x01,
		0x55, 0x0E, 0x09, 0x30, 0x81, 0x02,
		0x35, 0x00, 0x45, 0x00, 0x55, 0x02, 0x09, 0x31, 0x81, 0x02,
		0x35, 0x0A, 0x45, 0x14, 0x55, 0x08, 0x09, 0x32, 0x81, 0x02,
		0xC0,
	});
	HidReportDescriptor d;
	REQUIRE(parses(d, desc));
	REQUIRE(d.fields().size() == 3);
	CHECK(d.fields()[0].unitExponent == -2);
	CHECK_FALSE(d.fields()[0].hasPhysical);
	CHECK(d.fields()[0].physicalMax == 100);
	CHECK(d.fields()[1].unitExponent == 2);
	CHECK_FALSE(d.fields()[1].hasPhysical); // declared 0..0
	CHECK(d.fields()[1].physicalMin == 0);
	CHECK(d.fields()[1].physicalMax == 100);
	CHECK(d.fields()[2].unitExponent == -8);
	CHECK(d.fields()[2].hasPhysical);
	CHECK(d.fields()[2].physicalMin == 10);
	CHECK(d.fields()[2].physicalMax == 20);
}

// Collection path: PID-like descriptor: the Effect Type field sits in Logical(0x0F:0x21) > Logical(0x0F:0x25); the Duration in
// Logical(0x0F:0x21) only. findField by report usage + field usage, with and without the sub-path.
TEST_CASE("fields are found by report usage and field usage, optionally through a sub-path of collections") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kPidLike));
	const uint32_t setEffect = usage(0x0F, 0x21);
	const Field *dur = d.findField(ReportType::Output, setEffect, usage(0x0F, 0x50));
	REQUIRE(dur != nullptr);
	REQUIRE(dur->path.size() == 1);
	CHECK(dur->path[0].type == hid::CollLogical);
	CHECK(dur->path[0].usage == setEffect);
	const Field *et = d.findField(ReportType::Output, setEffect, usage(0x0F, 0x31));
	REQUIRE(et != nullptr);
	REQUIRE(et->path.size() == 2);
	CHECK(et->path[1].usage == usage(0x0F, 0x25));
	CHECK(d.findField(ReportType::Output, setEffect, usage(0x0F, 0x31), {usage(0x0F, 0x25)}) == et);
	CHECK(d.findField(ReportType::Output, setEffect, usage(0x0F, 0x50), {usage(0x0F, 0x25)}) == nullptr);
	CHECK(d.findField(ReportType::Input, setEffect, usage(0x0F, 0x50)) == nullptr); // wrong type
	CHECK(d.findField(ReportType::Output, usage(0x0F, 0x92), usage(0x0F, 0x50)) == nullptr); // wrong report
	CHECK(d.findField(ReportType::Output, setEffect, usage(0x0F, 0x22), {}, 1) == nullptr); // only one Block Index in the output report
	const std::vector<const Report *> rs = d.findReportsByUsage(ReportType::Output, setEffect);
	REQUIRE(rs.size() == 1);
	CHECK(rs[0]->reportId == 1);
}

// Value placement: bit offsets across byte boundaries, little-endian, sign: see the kPidLike comment. The Magnitude field
// (bits 33..48) straddles the report bytes 5, 6, 7 (after the ID byte, bit 33 is bit 1 of post-ID byte 4 = report byte 5).
//   idx 3, type ET Sine (2), duration 1000 = 0x03E8, direction enable 1, magnitude -2 = 0xFFFE:
//   byte0 ID 01, byte1 03, byte2 02, byte3 E8, byte4 03,
//   byte5 = bit0 dir enable 1 | (magnitude bits 0..6 = 0x7E) << 1 = 0x01 | 0xFC = 0xFD,
//   byte6 = magnitude bits 7..14 = (0xFFFE >> 7) & 0xFF = 0xFF, byte7 = magnitude bit 15 = 1.
TEST_CASE("insert and extract place a signed 16 bit value that straddles three bytes, little-endian") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kPidLike));
	const Report *r = d.findReport(ReportType::Output, 1);
	REQUIRE(r != nullptr);
	CHECK(r->byteLength() == 8);
	const Field *mag = d.findField(ReportType::Output, usage(0x0F, 0x21), usage(0x0F, 0x70));
	REQUIRE(mag != nullptr);
	CHECK(mag->bitOffset == 33);
	CHECK(mag->isSigned);
	CHECK(mag->logicalMin == -32767);
	CHECK(mag->logicalMax == 32767);
	std::vector<uint8_t> buf(8, 0);
	REQUIRE(HidReportDescriptor::insert(*mag, 0, buf, true, -2));
	CHECK(buf == B({0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x01}));
	int64_t v = 0;
	REQUIRE(HidReportDescriptor::extract(*mag, 0, buf, true, v));
	CHECK(v == -2);
	// the neighbours stay untouched: the direction enable bit (bit 32 = byte 5 bit 0)
	const Field *dir = d.findField(ReportType::Output, usage(0x0F, 0x21), usage(0x0F, 0x56));
	REQUIRE(dir != nullptr);
	CHECK(dir->bitOffset == 32);
	REQUIRE(HidReportDescriptor::insert(*dir, 0, buf, true, 1));
	CHECK(buf[5] == 0xFD);
	REQUIRE(HidReportDescriptor::insert(*mag, 0, buf, true, 0));
	CHECK(buf == B({0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00}));
	// too short buffer, nonexistent element
	std::vector<uint8_t> shortBuf(7, 0);
	CHECK_FALSE(HidReportDescriptor::insert(*mag, 0, shortBuf, true, 1));
	CHECK_FALSE(HidReportDescriptor::extract(*mag, 1, buf, true, v));
}

// ---------------------------------------------------------------------------------------------------------------------
// Parser, malformed descriptors: an error (with the byte offset), never a guess
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE("a descriptor with a short item whose data are past the end is rejected") {
	expectError(B({0x05, 0x01, 0x09}), "past the end"); // Usage with 1 data byte missing
	expectError(B({0x06, 0x01}), "past the end");		// 2-byte Usage Page, one byte present
}

TEST_CASE("a long item whose size field or data are past the end is rejected") {
	expectError(B({0xFE, 0x02}), "past the end");
	expectError(B({0xFE, 0x03, 0x10, 0x01, 0x02}), "past the end");
}

TEST_CASE("a Collection that is never closed is rejected") {
	expectError(B({0x05, 0x01, 0x09, 0x01, 0xA1, 0x01, 0xA1, 0x00, 0xC0}), "not closed");
}

TEST_CASE("an End Collection without a Collection is rejected") {
	expectError(B({0x05, 0x01, 0x09, 0x01, 0xA1, 0x01, 0xC0, 0xC0}), "End Collection without Collection");
}

TEST_CASE("a Pop without a Push is rejected") {
	expectError(B({0x05, 0x01, 0xB4}), "Pop without Push");
}

TEST_CASE("Report ID 0 is rejected as reserved (HID 1.11 6.2.2.7)") {
	expectError(B({0x05, 0x01, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x00, 0xC0}), "Report ID 0");
}

TEST_CASE("a Report ID above 255 is rejected") {
	expectError(B({0x05, 0x01, 0x09, 0x01, 0xA1, 0x01, 0x86, 0x00, 0x01, 0xC0}), "above 255");
}

TEST_CASE("a data item with Report Size 0 and a Report Count above 0 is rejected") {
	expectError(B({0x05, 0x01, 0x09, 0x01, 0xA1, 0x01, 0x15, 0x00, 0x25, 0x01, 0x75, 0x00, 0x95, 0x02, 0x09, 0x30, 0x81, 0x02, 0xC0}), "Report Size 0");
}

TEST_CASE("a Usage Minimum without a Usage Maximum is rejected") {
	expectError(B({0x05, 0x09, 0x19, 0x01, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x81, 0x02}), "pair");
}

TEST_CASE("a Usage Minimum above the Usage Maximum is rejected") {
	expectError(B({0x05, 0x09, 0x19, 0x05, 0x29, 0x01, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x81, 0x02}), "Usage Minimum above");
}

TEST_CASE("a usage list together with a usage range on one item is rejected") {
	expectError(B({0x05, 0x09, 0x09, 0x01, 0x19, 0x01, 0x29, 0x02, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x02, 0x81, 0x02}), "range");
}

TEST_CASE("a Delimiter item is rejected as unsupported") {
	expectError(B({0x05, 0x01, 0xA9, 0x01}), "Delimiter");
}

TEST_CASE("a data item without Logical Minimum and Maximum is rejected but a constant padding item needs none") {
	expectError(B({0x05, 0x01, 0x09, 0x30, 0x75, 0x08, 0x95, 0x01, 0x81, 0x02}), "without Logical");
	HidReportDescriptor d;
	CHECK(parses(d, B({0x75, 0x08, 0x95, 0x01, 0x81, 0x01})));
}

TEST_CASE("a Logical Minimum above the Logical Maximum is rejected") {
	expectError(B({0x05, 0x01, 0x09, 0x30, 0x15, 0x05, 0x25, 0x01, 0x75, 0x08, 0x95, 0x01, 0x81, 0x02}), "Logical Minimum above");
}

TEST_CASE("reserved item tags are rejected") {
	// prefix = tag << 4 | type << 2 | size code (6.2.2.2)
	expectError(B({0xC4}), "reserved global item tag"); // tag 0xC, type 1 (global)
	expectError(B({0x68}), "reserved local item tag");	// tag 6, type 2 (local)
	expectError(B({0xD0}), "reserved main item tag");	// tag 0xD, type 0 (main)
	expectError(B({0x0C}), "reserved item type 3");		// type 3
}

TEST_CASE("an item before the first Report ID is rejected when report IDs are used") {
	expectError(B({0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x81, 0x02, 0x85, 0x01, 0x81, 0x02}), "before the first Report ID");
}

TEST_CASE("an empty descriptor parses to no reports") {
	HidReportDescriptor d;
	CHECK(d.parse(std::vector<uint8_t>{}));
	CHECK(d.reports().empty());
}

// ---------------------------------------------------------------------------------------------------------------------
// PID table and builder
// ---------------------------------------------------------------------------------------------------------------------

// USB_PID1_01.pdf Table 2 (p. 6-7): hand-checked IDs of the usages the tests use; the table has no duplicate IDs.
TEST_CASE("the PID usage table has the IDs of USB_PID1_01 Table 2 and no duplicates") {
	CHECK(pid::usageByName("Set Effect Report") == usage(0x0F, 0x21));
	CHECK(pid::usageByName("Set Envelope Report") == usage(0x0F, 0x5A));
	CHECK(pid::usageByName("Set Condition Report") == usage(0x0F, 0x5F));
	CHECK(pid::usageByName("Set Periodic Report") == usage(0x0F, 0x6E));
	CHECK(pid::usageByName("Set Constant Force Report") == usage(0x0F, 0x73));
	CHECK(pid::usageByName("Set Ramp Force Report") == usage(0x0F, 0x74));
	CHECK(pid::usageByName("Effect Operation Report") == usage(0x0F, 0x77));
	CHECK(pid::usageByName("Device Gain Report") == usage(0x0F, 0x7D));
	CHECK(pid::usageByName("PID Pool Report") == usage(0x0F, 0x7F));
	CHECK(pid::usageByName("PID Block Load Report") == usage(0x0F, 0x89));
	CHECK(pid::usageByName("PID Block Free Report") == usage(0x0F, 0x90));
	CHECK(pid::usageByName("PID State Report") == usage(0x0F, 0x92));
	CHECK(pid::usageByName("PID Device Control Report") == usage(0x0F, 0x95));
	CHECK(pid::usageByName("Create New Effect Report") == usage(0x0F, 0xAB));
	CHECK(pid::usageByName("RAM Pool Available") == usage(0x0F, 0xAC));
	CHECK(pid::usageByName("No Such Usage") == 0xFFFFFFFFu);
	CHECK(pid::nameOf(usage(0x0F, 0x21)) == "Set Effect Report");
	CHECK(pid::nameOf(usage(0x01, 0x21)) == "");
	for (size_t i = 0; i < pid::usageTable().size(); i++) {
		for (size_t j = i + 1; j < pid::usageTable().size(); j++) {
			INFO(pid::usageTable()[i].name << " / " << pid::usageTable()[j].name);
			CHECK(pid::usageTable()[i].id != pid::usageTable()[j].id);
		}
		CHECK(pid::usageByName(pid::usageTable()[i].name) == usage(0x0F, pid::usageTable()[i].id));
	}
	// the named constants of the header agree with the table
	CHECK(pid::use::EtSine == pid::usageByName("ET Sine"));
	CHECK(pid::use::DcDeviceContinue == pid::usageByName("DC Device Continue"));
	CHECK(pid::use::PidStateReport == pid::usageByName("PID State Report"));
}

// The report of the kPidLike comment: idx 3, ET Sine (selector value 2), duration 1000, direction enable, magnitude -2
// = 01 03 02 E8 03 FD FF 01 (hand derivation at the insert/extract case).
TEST_CASE("a report built by usage equals the hand-computed bytes") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kPidLike));
	pid::PidReportBuilder b(d, false);
	pid::PidReport r = b.make(ReportType::Output, pid::use::SetEffectReport);
	REQUIRE(r.valid());
	r.set(pid::use::EffectBlockIndex, 3).select(pid::use::EtSine).set(pid::use::Duration, 1000).flag(pid::use::DirectionEnable, true).set(pid::use::Magnitude, -2);
	CHECK(r.errors().empty());
	CHECK(r.bytes() == B({0x01, 0x03, 0x02, 0xE8, 0x03, 0xFD, 0xFF, 0x01}));
	CHECK(r.length() == 8);
	CHECK(r.reportId() == 1);
}

TEST_CASE("an untouched report is all zero except the report ID byte and has the descriptor's length") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kPidLike));
	pid::PidReportBuilder b(d, false);
	pid::PidReport r = b.make(ReportType::Output, "Set Effect Report");
	CHECK(r.bytes() == B({0x01, 0, 0, 0, 0, 0, 0, 0}));
	pid::PidReport s = b.make(ReportType::Input, "PID State Report");
	CHECK(s.bytes() == B({0x02, 0, 0}));
}

TEST_CASE("the selector of a named array is the logical minimum plus its position, ET Constant Force = 1") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kPidLike));
	pid::PidReportBuilder b(d, false);
	pid::PidReport r = b.make(ReportType::Output, pid::use::SetEffectReport);
	r.select(pid::use::EtConstantForce);
	CHECK(r.errors().empty());
	CHECK(r.bytes()[2] == 1);
}

TEST_CASE("a value above or below the logical range is an error and is not clamped") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kPidLike));
	pid::PidReportBuilder b(d, false);
	pid::PidReport r = b.make(ReportType::Output, pid::use::SetEffectReport);
	r.set(pid::use::EffectBlockIndex, 41); // logical 1..40
	CHECK(r.errors().size() == 1);
	r.set(pid::use::EffectBlockIndex, 0);
	CHECK(r.errors().size() == 2);
	r.set(pid::use::Magnitude, 32768);
	CHECK(r.errors().size() == 3);
	r.set(pid::use::Magnitude, -32768); // the signed field's minimum is -32767
	CHECK(r.errors().size() == 4);
	r.set(pid::use::Duration, -1);
	CHECK(r.errors().size() == 5);
	CHECK(r.bytes() == B({0x01, 0, 0, 0, 0, 0, 0, 0})); // nothing was written
	r.set(pid::use::EffectBlockIndex, 40).set(pid::use::Magnitude, 32767).set(pid::use::Magnitude, -32767);
	CHECK(r.errors().size() == 5);
}

// PID 1.01 section 6 Device Control: Logical 1..6 in Report Size 1 elements (Output 91 02). Usage 0x97 is element 0, 0x98 element 1.
// 1 fits one bit; 2 is inside the logical range but needs two bits: error, nothing written.
TEST_CASE("a value inside the logical range that does not fit the report size is an error") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kDeviceControl));
	pid::PidReportBuilder b(d, false);
	pid::PidReport r = b.make(ReportType::Output, pid::use::PidDeviceControlReport);
	REQUIRE(r.valid());
	CHECK(r.length() == 2);
	r.set(pid::use::DcEnableActuators, 1);
	CHECK(r.errors().empty());
	CHECK(r.bytes() == B({0x0C, 0x01}));
	r.set(pid::use::DcDisableActuators, 2);
	CHECK(r.errors().size() == 1);
	CHECK(r.bytes() == B({0x0C, 0x01}));
}

// PID 1.01 Figure 16 (p. 40): ID + one command byte; command = logical minimum 1 + position of the usage among the six (0x97..0x9C -> 1..6).
TEST_CASE("deviceControl builds the Figure 16 report: the report ID and the command number 1 to 6 of the DC usage") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kDeviceControl));
	pid::PidReportBuilder b(d, false);
	const uint32_t dc[] = {pid::use::DcEnableActuators, pid::use::DcDisableActuators, pid::use::DcStopAllEffects,
						   pid::use::DcDeviceReset,		pid::use::DcDevicePause,	 pid::use::DcDeviceContinue};
	for (int i = 0; i < 6; i++) {
		pid::PidReport r = b.deviceControl(dc[i]);
		INFO("command " << i + 1);
		CHECK(r.errors().empty());
		CHECK(r.bytes() == B({0x0C, i + 1}));
	}
	CHECK_FALSE(b.deviceControl(pid::use::Magnitude).errors().empty()); // not a command
	HidReportDescriptor other;
	REQUIRE(parses(other, kPidLike)); // no Device Control report
	pid::PidReportBuilder bo(other, false);
	pid::PidReport none = bo.deviceControl(pid::use::DcEnableActuators);
	CHECK_FALSE(none.valid());
}

TEST_CASE("an unknown usage, a field of another report and an unknown selector are errors") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kPidLike));
	pid::PidReportBuilder b(d, false);
	pid::PidReport r = b.make(ReportType::Output, pid::use::SetEffectReport);
	r.set(pid::use::DevicePaused, 1); // a PID State (input) field
	CHECK(r.errors().size() == 1);
	r.set(usage(0x0F, 0x1234), 1);
	CHECK(r.errors().size() == 2);
	r.select(pid::use::EtSquare); // not a selector of this descriptor's Effect Type array
	CHECK(r.errors().size() == 3);
	r.set(pid::use::EffectType, 1); // Effect Type is an array field: set() is for Variable fields
	CHECK(r.errors().size() == 4);
	CHECK(r.bytes() == B({0x01, 0, 0, 0, 0, 0, 0, 0}));
}

TEST_CASE("a report usage the descriptor does not declare gives an invalid report that fails every set") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kPidLike));
	pid::PidReportBuilder b(d, false);
	pid::PidReport r = b.make(ReportType::Output, pid::use::SetRampForceReport);
	CHECK_FALSE(r.valid());
	CHECK(r.length() == 0);
	r.set(pid::use::RampStart, 1);
	CHECK(r.errors().size() == 1);
	CHECK_FALSE(b.make(ReportType::Output, "No Such Report").valid());
	CHECK(b.find(ReportType::Output, pid::use::SetEffectReport) != nullptr);
	CHECK(b.find(ReportType::Input, pid::use::SetEffectReport) == nullptr); // the report has another type
}

// Input report 2: byte 1 = 0x05 = Device Paused (bit 0) 1, Actuators Enabled (bit 1) 0, Effect Playing (bit 2) 1, padding 0; byte 2 = block index 7.
TEST_CASE("an input report is decoded into usage values and a wrong length or ID is an error") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kPidLike));
	pid::PidReportBuilder b(d, false);
	const pid::DecodedReport dec = b.decode(ReportType::Input, pid::use::PidStateReport, B({0x02, 0x05, 0x07}));
	CHECK(dec.errors.empty());
	int64_t v = -1;
	CHECK(dec.get(pid::use::DevicePaused, v));
	CHECK(v == 1);
	CHECK(dec.get(pid::use::ActuatorsEnabled, v));
	CHECK(v == 0);
	CHECK(dec.get(pid::use::EffectPlaying, v));
	CHECK(v == 1);
	CHECK(dec.get(pid::use::EffectBlockIndex, v));
	CHECK(v == 7);
	CHECK_FALSE(dec.get(pid::use::Duration, v));
	CHECK_FALSE(b.decode(ReportType::Input, pid::use::PidStateReport, B({0x02, 0x05})).errors.empty());
	CHECK_FALSE(b.decode(ReportType::Input, pid::use::PidStateReport, B({0x03, 0x05, 0x07})).errors.empty());
	CHECK_FALSE(b.decode(ReportType::Input, pid::use::SetEffectReport, B({0x01})).errors.empty()); // not an input report
}

TEST_CASE("an output report built by the builder is decoded back to the values that were set, selectors included") {
	HidReportDescriptor d;
	REQUIRE(parses(d, kPidLike));
	pid::PidReportBuilder b(d, false);
	pid::PidReport r = b.make(ReportType::Output, pid::use::SetEffectReport);
	r.set(pid::use::EffectBlockIndex, 40).select(pid::use::EtConstantForce).set(pid::use::Duration, 32767).set(pid::use::Magnitude, -32767);
	const pid::DecodedReport dec = b.decode(ReportType::Output, pid::use::SetEffectReport, r.bytes());
	CHECK(dec.errors.empty());
	int64_t v = 0;
	CHECK(dec.get(pid::use::EffectBlockIndex, v));
	CHECK(v == 40);
	CHECK(dec.get(pid::use::Duration, v));
	CHECK(v == 32767);
	CHECK(dec.get(pid::use::Magnitude, v));
	CHECK(v == -32767);
	CHECK(dec.get(pid::use::DirectionEnable, v));
	CHECK(v == 0);
	CHECK(dec.selects(pid::use::EtConstantForce));
	CHECK_FALSE(dec.selects(pid::use::EtSine));
	CHECK(dec.getIn({pid::use::EffectType}, pid::use::EffectBlockIndex, v) == false);
}

// Two reports of the same type, built from a descriptor that declares both: IDs and lengths come from the descriptor.
TEST_CASE("the builder takes the report ID and length of every declared report from the descriptor") {
	const std::vector<uint8_t> two = cat(kPidLike, B({
		0x09, 0x7D, 0xA1, 0x02, 0x85, 0x0D, 0x09, 0x7E, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x01, 0xB1, 0x02, 0xC0,
	}));
	HidReportDescriptor d;
	REQUIRE(parses(d, two));
	pid::PidReportBuilder b(d, false);
	pid::PidReport g = b.make(ReportType::Feature, pid::use::DeviceGainReport);
	REQUIRE(g.valid());
	g.set(pid::use::DeviceGain, 200);
	CHECK(g.errors().empty());
	CHECK(g.bytes() == B({0x0D, 200}));
	CHECK(g.reportId() == 0x0D);
	CHECK(b.make(ReportType::Output, pid::use::SetEffectReport).reportId() == 1);
}


// ---------------------------------------------------------------------------------------------------------------------
// Review follow-up (WO I0-6)
// ---------------------------------------------------------------------------------------------------------------------

// HID 1.11 6.2.2.7: "All items succeeding the first Report ID tag but preceding a second Report ID tag are included in a report
// prefixed by a 1-byte ID"; a Report ID prefix names one report, so the same ID in the collection of another report would merge two
// reports. Descriptor: two Output reports, each in its own Logical collection (usages GD 0x21 and 0x22), both Report ID 1.
TEST_CASE("the same report ID in the collections of two different reports is an error, in one collection it is not") {
	const std::vector<uint8_t> two = B({
		0x05, 0x01, 0x09, 0x21, 0xA1, 0x02, 0x85, 0x01, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x09, 0x30, 0x91, 0x02, 0xC0,
		0x09, 0x22, 0xA1, 0x02, 0x85, 0x01, 0x09, 0x31, 0x91, 0x02, 0xC0,
	});
	expectError(two, "used by two report collections");
	// the same usage (same report) twice, or the ID stated once per type, is fine
	const std::vector<uint8_t> same = B({
		0x05, 0x01, 0x09, 0x21, 0xA1, 0x02, 0x85, 0x01, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x09, 0x30, 0x91, 0x02, 0x09, 0x31, 0x91, 0x02, 0xC0,
	});
	HidReportDescriptor d;
	CHECK(parses(d, same));
	// two reports with different IDs
	const std::vector<uint8_t> ok = B({
		0x05, 0x01, 0x09, 0x21, 0xA1, 0x02, 0x85, 0x01, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x09, 0x30, 0x91, 0x02, 0xC0,
		0x09, 0x22, 0xA1, 0x02, 0x85, 0x02, 0x09, 0x31, 0x91, 0x02, 0xC0,
	});
	CHECK(parses(d, ok));
	CHECK(d.reports().size() == 2);
}

// HID 1.11 6.2.2.8: a 16 bit Usage gets the Usage Page declared last when the main item arrives. "05 01 09 30 05 09 ... 81 02":
// the Usage Page changes to 9 between the Usage and the Input: the usage is 0x00090030.
TEST_CASE("a 16 bit usage takes the usage page that is current at the main item, not at the usage item") {
	HidReportDescriptor d;
	REQUIRE(parses(d, B({0x05, 0x01, 0x09, 0x30, 0x05, 0x09, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x81, 0x02})));
	REQUIRE(d.fields().size() == 1);
	CHECK(d.fields()[0].usages == std::vector<uint32_t>{0x00090030u});
	// a usage range and a Collection follow the same rule: Collection (Usage 0x21 declared under page 1, page 0x0F at the Collection)
	HidReportDescriptor c;
	REQUIRE(parses(c, B({0x05, 0x01, 0x09, 0x21, 0x05, 0x0F, 0xA1, 0x02, 0x19, 0x01, 0x29, 0x02, 0x05, 0x09, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x02, 0x81, 0x02, 0xC0})));
	REQUIRE(c.fields().size() == 1);
	CHECK(c.fields()[0].path[0].usage == 0x000F0021u);
	CHECK(c.fields()[0].usageMin == 0x00090001u);
	CHECK(c.fields()[0].usageMax == 0x00090002u);
}

// 6.2.2.2 interpretation: a negative minimum makes the field two's complement; "15 80 26 FF 00" has minimum -128 and a 2-byte maximum
// 0x00FF = 255 that cannot be sent in a signed 8 bit field.
TEST_CASE("a negative logical minimum whose range does not fit the signed range of the report size is rejected") {
	expectError(B({0x05, 0x01, 0x09, 0x30, 0x15, 0x80, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x01, 0x81, 0x02}), "does not fit Report Size");
	expectError(B({0x05, 0x01, 0x09, 0x30, 0x15, 0x80, 0x25, 0x7F, 0x75, 0x07, 0x95, 0x01, 0x81, 0x02}), "does not fit Report Size");
	HidReportDescriptor d;
	CHECK(parses(d, B({0x05, 0x01, 0x09, 0x30, 0x15, 0x80, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x81, 0x02})));
}

// 6.2.2.8: "if there are more controls than usages, the last usage applies to the remaining ones". Three 8 bit elements with two usages.
TEST_CASE("the usage of an element past the end of the usage list is the last usage of the list") {
	HidReportDescriptor d;
	REQUIRE(parses(d, B({0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x00, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x02})));
	REQUIRE(d.fields().size() == 1);
	const Field &f = d.fields()[0];
	CHECK(f.usageOf(0) == usage(1, 0x30));
	CHECK(f.usageOf(1) == usage(1, 0x31));
	CHECK(f.usageOf(2) == usage(1, 0x31));
	CHECK(f.usageOf(7) == usage(1, 0x31));
	// a range stops at its maximum the same way
	HidReportDescriptor r;
	REQUIRE(parses(r, B({0x05, 0x09, 0x19, 0x01, 0x29, 0x02, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x04, 0x81, 0x02})));
	CHECK(r.fields()[0].usageOf(3) == usage(9, 2));
}

// A Collection takes the first pending usage (6.2.2.8: the usage list assigns in order), the others are dropped with the local state
TEST_CASE("a collection takes the first pending usage and the usages do not leak into the next item") {
	HidReportDescriptor d;
	REQUIRE(parses(d, B({0x05, 0x01, 0x09, 0x05, 0x09, 0x06, 0xA1, 0x01, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, 0x81, 0x02, 0xC0})));
	REQUIRE(d.fields().size() == 1);
	REQUIRE(d.fields()[0].path.size() == 1);
	CHECK(d.fields()[0].path[0].usage == usage(1, 5));
	CHECK(d.fields()[0].usages.empty());
}

// The usage table typed from USB_PID1_01.pdf Table 2 (pp. 6-7) row by row, independent of PidReportBuilder.cpp: every row, every
// header constant and the table size must agree ("Undefined" and "Reserved" rows are not in the table).
TEST_CASE("every usage ID and name of PID 1.01 Table 2 matches the table and the constants of the builder") {
	struct Row {
		uint16_t id;
		const char *name;
		uint32_t constant;
	};
	static const Row rows[] = {
	{0x01, "Physical Interface Device", pid::use::PhysicalInterfaceDevice},
	{0x20, "Normal", pid::use::Normal},
	{0x21, "Set Effect Report", pid::use::SetEffectReport},
	{0x22, "Effect Block Index", pid::use::EffectBlockIndex},
	{0x23, "Parameter Block Offset", pid::use::ParameterBlockOffset},
	{0x24, "ROM Flag", pid::use::RomFlag},
	{0x25, "Effect Type", pid::use::EffectType},
	{0x26, "ET Constant Force", pid::use::EtConstantForce},
	{0x27, "ET Ramp", pid::use::EtRamp},
	{0x28, "ET Custom Force Data", pid::use::EtCustomForceData},
	{0x30, "ET Square", pid::use::EtSquare},
	{0x31, "ET Sine", pid::use::EtSine},
	{0x32, "ET Triangle", pid::use::EtTriangle},
	{0x33, "ET Sawtooth Up", pid::use::EtSawtoothUp},
	{0x34, "ET Sawtooth Down", pid::use::EtSawtoothDown},
	{0x40, "ET Spring", pid::use::EtSpring},
	{0x41, "ET Damper", pid::use::EtDamper},
	{0x42, "ET Inertia", pid::use::EtInertia},
	{0x43, "ET Friction", pid::use::EtFriction},
	{0x50, "Duration", pid::use::Duration},
	{0x51, "Sample Period", pid::use::SamplePeriod},
	{0x52, "Gain", pid::use::Gain},
	{0x53, "Trigger Button", pid::use::TriggerButton},
	{0x54, "Trigger Repeat Interval", pid::use::TriggerRepeatInterval},
	{0x55, "Axes Enable", pid::use::AxesEnable},
	{0x56, "Direction Enable", pid::use::DirectionEnable},
	{0x57, "Direction", pid::use::Direction},
	{0x58, "Type Specific Block Offset", pid::use::TypeSpecificBlockOffset},
	{0x59, "Block Type", pid::use::BlockType},
	{0x5A, "Set Envelope Report", pid::use::SetEnvelopeReport},
	{0x5B, "Attack Level", pid::use::AttackLevel},
	{0x5C, "Attack Time", pid::use::AttackTime},
	{0x5D, "Fade Level", pid::use::FadeLevel},
	{0x5E, "Fade Time", pid::use::FadeTime},
	{0x5F, "Set Condition Report", pid::use::SetConditionReport},
	{0x60, "CP Offset", pid::use::CpOffset},
	{0x61, "Positive Coefficient", pid::use::PositiveCoefficient},
	{0x62, "Negative Coefficient", pid::use::NegativeCoefficient},
	{0x63, "Positive Saturation", pid::use::PositiveSaturation},
	{0x64, "Negative Saturation", pid::use::NegativeSaturation},
	{0x65, "Dead Band", pid::use::DeadBand},
	{0x66, "Download Force Sample", pid::use::DownloadForceSample},
	{0x67, "Isoch Custom Force Enable", pid::use::IsochCustomForceEnable},
	{0x68, "Custom Force Data Report", pid::use::CustomForceDataReport},
	{0x69, "Custom Force Data", pid::use::CustomForceData},
	{0x6A, "Custom Force Vendor Defined Data", pid::use::CustomForceVendorDefinedData},
	{0x6B, "Set Custom Force Report", pid::use::SetCustomForceReport},
	{0x6C, "Custom Force Data Offset", pid::use::CustomForceDataOffset},
	{0x6D, "Sample Count", pid::use::SampleCount},
	{0x6E, "Set Periodic Report", pid::use::SetPeriodicReport},
	{0x6F, "Offset", pid::use::Offset},
	{0x70, "Magnitude", pid::use::Magnitude},
	{0x71, "Phase", pid::use::Phase},
	{0x72, "Period", pid::use::Period},
	{0x73, "Set Constant Force Report", pid::use::SetConstantForceReport},
	{0x74, "Set Ramp Force Report", pid::use::SetRampForceReport},
	{0x75, "Ramp Start", pid::use::RampStart},
	{0x76, "Ramp End", pid::use::RampEnd},
	{0x77, "Effect Operation Report", pid::use::EffectOperationReport},
	{0x78, "Effect Operation", pid::use::EffectOperation},
	{0x79, "Op Effect Start", pid::use::OpEffectStart},
	{0x7A, "Op Effect Start Solo", pid::use::OpEffectStartSolo},
	{0x7B, "Op Effect Stop", pid::use::OpEffectStop},
	{0x7C, "Loop Count", pid::use::LoopCount},
	{0x7D, "Device Gain Report", pid::use::DeviceGainReport},
	{0x7E, "Device Gain", pid::use::DeviceGain},
	{0x7F, "PID Pool Report", pid::use::PidPoolReport},
	{0x80, "RAM Pool Size", pid::use::RamPoolSize},
	{0x81, "ROM Pool Size", pid::use::RomPoolSize},
	{0x82, "ROM Effect Block Count", pid::use::RomEffectBlockCount},
	{0x83, "Simultaneous Effects Max", pid::use::SimultaneousEffectsMax},
	{0x84, "Pool Alignment", pid::use::PoolAlignment},
	{0x85, "PID Pool Move Report", pid::use::PidPoolMoveReport},
	{0x86, "Move Source", pid::use::MoveSource},
	{0x87, "Move Destination", pid::use::MoveDestination},
	{0x88, "Move Length", pid::use::MoveLength},
	{0x89, "PID Block Load Report", pid::use::PidBlockLoadReport},
	{0x8B, "Block Load Status", pid::use::BlockLoadStatus},
	{0x8C, "Block Load Success", pid::use::BlockLoadSuccess},
	{0x8D, "Block Load Full", pid::use::BlockLoadFull},
	{0x8E, "Block Load Error", pid::use::BlockLoadError},
	{0x8F, "Block Handle", pid::use::BlockHandle},
	{0x90, "PID Block Free Report", pid::use::PidBlockFreeReport},
	{0x91, "Type Specific Block Handle", pid::use::TypeSpecificBlockHandle},
	{0x92, "PID State Report", pid::use::PidStateReport},
	{0x94, "Effect Playing", pid::use::EffectPlaying},
	{0x95, "PID Device Control Report", pid::use::PidDeviceControlReport},
	{0x96, "PID Device Control", pid::use::PidDeviceControl},
	{0x97, "DC Enable Actuators", pid::use::DcEnableActuators},
	{0x98, "DC Disable Actuators", pid::use::DcDisableActuators},
	{0x99, "DC Stop All Effects", pid::use::DcStopAllEffects},
	{0x9A, "DC Device Reset", pid::use::DcDeviceReset},
	{0x9B, "DC Device Pause", pid::use::DcDevicePause},
	{0x9C, "DC Device Continue", pid::use::DcDeviceContinue},
	{0x9F, "Device Paused", pid::use::DevicePaused},
	{0xA0, "Actuators Enabled", pid::use::ActuatorsEnabled},
	{0xA4, "Safety Switch", pid::use::SafetySwitch},
	{0xA5, "Actuator Override Switch", pid::use::ActuatorOverrideSwitch},
	{0xA6, "Actuator Power", pid::use::ActuatorPower},
	{0xA7, "Start Delay", pid::use::StartDelay},
	{0xA8, "Parameter Block Size", pid::use::ParameterBlockSize},
	{0xA9, "Device Managed Pool", pid::use::DeviceManagedPool},
	{0xAA, "Shared Parameter Blocks", pid::use::SharedParameterBlocks},
	{0xAB, "Create New Effect Report", pid::use::CreateNewEffectReport},
	{0xAC, "RAM Pool Available", pid::use::RamPoolAvailable},
	};
	CHECK(pid::usageTable().size() == sizeof(rows) / sizeof(rows[0]));
	for (const Row &r : rows) {
		INFO(r.name);
		CHECK(r.constant == usage(0x0F, r.id));
		CHECK(pid::usageByName(r.name) == usage(0x0F, r.id));
		CHECK(pid::nameOf(usage(0x0F, r.id)) == r.name);
	}
}

// =====================================================================================================================
// F1-A3: WheelPlant self-tests (fakes/WheelPlant.h). Every expected value is derived here by hand (MATH) from the stated model
//     J theta'' = tau - b theta' - F_c sign(theta') [- k (theta - h) - c theta'],   tau = counts * fullScale / 32767,
// not read from the plant. The plant is driven through a bare FakeTmc4671 (no firmware): the registers 0x63 / 0x1A / 0x76 /
// 0x5E / 0x64 are set as the datasheet defines them (TMC4671 DS Rev 2.07 p84-85, p97-98, p104).
// =====================================================================================================================
namespace {

constexpr double kStep = 1e-3; // one scheduler step

WheelPlant::Params plantParams(double J, double b, double Fc, double full, double lag = 0.0) {
	WheelPlant::Params p;
	p.inertia = J;
	p.viscous = b;
	p.coulomb = Fc;
	p.fullScaleTorque = full;
	p.currentLagS = lag;
	return p;
}

/** Sets the chip so the torque target reaches the wheel (torque mode 1, PWM mode 7, ENI high, limit 0x7FFF) unless told otherwise */
void driveChip(FakeTmc4671 &c, int counts, uint32_t mode = 1, uint32_t pwm = 7, bool eni = true, uint32_t limit = 0x7FFF) {
	c.setReg(0x64, static_cast<uint32_t>(static_cast<uint16_t>(static_cast<int16_t>(counts))) << 16);
	c.setReg(0x63, mode);
	c.setReg(0x1A, pwm);
	c.setEnableIn(eni);
	c.setReg(0x5E, limit);
	c.setReg(0x51, 9); // POSITION_SELECTION: phi_m_abn, a mechanical angle (DS p95), so 0x6B counts 65536 per revolution
}

double torqueOf(int counts, double full) { return static_cast<double>(counts) * full / 32767.0; }

void runSteps(WheelPlant &p, unsigned n, double dt = kStep) {
	for (unsigned i = 0; i < n; i++) {
		p.step(dt);
	}
}

bool near(double a, double b, double rel, double abs = 0.0) { return std::fabs(a - b) <= rel * std::fabs(b) + abs; }

} // namespace

// MATH: frictionless, undamped, constant torque tau from omega0: a = tau / J, omega = omega0 + a t, theta = omega0 t + a t^2 / 2.
// Kinetic energy change: J/2 (omega^2 - omega0^2) = J/2 (2 omega0 a t + a^2 t^2) = tau (omega0 t + a t^2 / 2) = tau * theta (work =
// torque times the angle travelled). J 0.02, full scale 10 N m, 16384 counts, omega0 3 rad/s, 1000 steps of 1 ms (t = 1 s).
TEST_CASE("F1-A3: the plant under a constant torque without friction keeps energy exactly: the work of the torque equals the change of kinetic energy") {
	FakeTmc4671 chip{0};
	WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
	plant.attach(chip);
	plant.setState(0.0, 3.0);
	driveChip(chip, 16384);
	const double tau = torqueOf(16384, 10.0);
	const double a = tau / 0.02;
	const double e0 = plant.energy();
	runSteps(plant, 1000);
	CHECK(plant.steps() == 1000);
	CHECK(near(plant.tauApplied(), tau, 1e-15));
	CHECK(near(plant.omega(), 3.0 + a * 1.0, 1e-12));
	CHECK(near(plant.theta(), 3.0 * 1.0 + a * 1.0 / 2.0, 1e-12));
	CHECK(near(plant.energy() - e0, tau * plant.theta(), 1e-10));
	CHECK(near(plant.chipWork(), plant.energy() - e0, 1e-10));
	CHECK(plant.violations().empty());
}

// MATH: free motion with b = 0 and F_c = 0 and no torque: omega is constant and theta = omega t exactly, kinetic energy constant.
TEST_CASE("F1-A3: the plant without torque, friction or damping keeps omega and energy constant and theta advances linearly") {
	FakeTmc4671 chip{0};
	WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
	plant.attach(chip);
	plant.setState(1.0, -2.5);
	driveChip(chip, 0);
	const double e0 = plant.energy();
	runSteps(plant, 500);
	CHECK(plant.omega() == -2.5);
	CHECK(near(plant.theta(), 1.0 - 2.5 * 0.5, 1e-12)); // 500 additions of -2.5e-3: rounding only
	CHECK(plant.energy() == e0);
}

// MATH: J omega' = -b omega -> omega(t) = omega0 e^{-b t / J}, theta(t) = theta0 + omega0 (J / b)(1 - e^{-b t / J}).
// J 0.02, b 0.005 (time constant J / b = 4 s), omega0 10, t = 2 s: omega = 10 e^{-0.5}, theta = 40 (1 - e^{-0.5}).
// With a held torque tau from rest: w_ss = tau / b, omega = w_ss (1 - e^{-b t / J}), theta = w_ss (t - (J / b)(1 - e^{-b t / J})).
TEST_CASE("F1-A3: viscous damping decays omega as omega0 exp(-b t / J) with the matching angle, and a held torque approaches tau / b") {
	{
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.005, 0.0, 10.0));
		plant.attach(chip);
		plant.setState(0.0, 10.0);
		driveChip(chip, 0);
		runSteps(plant, 2000);
		CHECK(near(plant.omega(), 10.0 * std::exp(-0.5), 1e-12));
		CHECK(near(plant.theta(), 40.0 * (1.0 - std::exp(-0.5)), 1e-12));
		// the exact solution does not depend on the step: one step of 2 s gives the same state
		WheelPlant one(plantParams(0.02, 0.005, 0.0, 10.0));
		one.setState(0.0, 10.0);
		one.step(2.0);
		CHECK(near(one.omega(), plant.omega(), 1e-12));
		CHECK(near(one.theta(), plant.theta(), 1e-12));
	}
	{
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.005, 0.0, 10.0));
		plant.attach(chip);
		driveChip(chip, 100);
		const double wss = torqueOf(100, 10.0) / 0.005;
		runSteps(plant, 1000);
		CHECK(near(plant.omega(), wss * (1.0 - std::exp(-0.25)), 1e-12));
		CHECK(near(plant.theta(), wss * (1.0 - 4.0 * (1.0 - std::exp(-0.25))), 1e-12));
	}
}

// MATH: Coulomb friction F_c = 0.1 N m, J 0.02, b 0, no torque: deceleration a = F_c / J = 5 rad/s^2, stop time omega0 J / F_c =
// 10.0025 * 0.02 / 0.1 = 2.0005 s (not on a 1 ms step boundary), stop angle omega0^2 / (2 a) = omega0^2 / 10. At t = 2 s omega =
// 10.0025 - 10 = 0.0025. The wheel stops inside step 2001 (omega exactly 0) and never reverses afterwards.
TEST_CASE("F1-A3: Coulomb friction stops the wheel at omega0 J / F_c inside a step, omega never changes sign and the wheel then stays at rest") {
	for (double dir : {1.0, -1.0}) {
		INFO("direction " << dir);
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.0, 0.1, 10.0));
		plant.attach(chip);
		const double w0 = 10.0025 * dir;
		plant.setState(0.0, w0);
		driveChip(chip, 0);
		bool reversed = false;
		for (unsigned i = 0; i < 2000; i++) {
			plant.step(kStep);
			reversed = reversed || plant.omega() * dir < 0.0;
		}
		CHECK(near(plant.omega(), 0.0025 * dir, 1e-9));
		plant.step(kStep); // step 2001 contains the stop at 2.0005 s
		CHECK(plant.omega() == 0.0);
		const double thetaStop = dir * 10.0025 * 10.0025 / 10.0;
		CHECK(near(plant.theta(), thetaStop, 1e-12));
		for (unsigned i = 0; i < 100; i++) {
			plant.step(kStep);
			reversed = reversed || plant.omega() * dir < 0.0;
		}
		CHECK_FALSE(reversed);
		CHECK(plant.omega() == 0.0);
		CHECK(near(plant.theta(), thetaStop, 1e-12)); // no creep after the stop
	}
}

// MATH: Coulomb with viscous damping, no torque, omega0 > 0: J w' = -b w - F_c -> w(t) = (w0 + F_c / b) e^{-b t / J} - F_c / b, which is
// 0 at tc = (J / b) ln(1 + b w0 / F_c). Angle: theta = (J / b)(w0 + F_c / b)(1 - e^{-b tc / J}) - F_c tc / b; with
// e^{-b tc / J} = F_c / (F_c + b w0) this is J w0 / b - F_c tc / b. J 0.02, b 0.005, F_c 0.1, w0 10: tc = 4 ln(1.5) = 1.62186 s, theta =
// 40 - 20 tc = 7.5628. The stop is inside step 1622 (1.621 s < tc < 1.622 s); afterwards the wheel stays (|0| <= F_c).
TEST_CASE("F1-A3: Coulomb friction with viscous damping stops the wheel at (J / b) ln(1 + b w0 / F_c) and at the matching angle, both directions") {
	const double tc = 4.0 * std::log(1.5);
	for (double dir : {1.0, -1.0}) {
		INFO("direction " << dir);
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.005, 0.1, 10.0));
		plant.attach(chip);
		plant.setState(0.0, 10.0 * dir);
		driveChip(chip, 0);
		bool reversed = false;
		runSteps(plant, 1621);
		CHECK(plant.omega() * dir > 0.0); // still moving at 1.621 s < tc
		const double wExpected = (10.0 + 0.1 / 0.005) * std::exp(-0.005 * 1.621 / 0.02) - 0.1 / 0.005;
		CHECK(near(plant.omega(), dir * wExpected, 1e-9));
		for (unsigned i = 0; i < 100; i++) {
			plant.step(kStep);
			reversed = reversed || plant.omega() * dir < 0.0;
		}
		CHECK_FALSE(reversed);
		CHECK(plant.omega() == 0.0);
		CHECK(near(plant.theta(), dir * (40.0 - 20.0 * tc), 1e-9));
	}
	// one step that contains the stop time (1 + 1e-9 of it): omega 0 (stuck for the remainder) and the angle above
	WheelPlant one(plantParams(0.02, 0.005, 0.1, 10.0));
	one.setState(0.0, 10.0);
	one.step(tc * (1.0 + 1e-9));
	CHECK(one.omega() == 0.0);
	CHECK(near(one.theta(), 40.0 - 20.0 * tc, 1e-9));
}

// MATH: spring and Coulomb together. J 0.02, no damping, no torque, k = 1 N m/rad to the hand at h = 0.5 rad, F_c 0.1, release from rest at
// theta = 0: the spring force k h = 0.5 > F_c slips the wheel towards the hand. While moving forward J th'' = k (h - th) - F_c, a harmonic
// motion about th_eq = h - F_c / k = 0.4 with w0^2 = k / J = 50: th = 0.4 - 0.4 cos(sqrt(50) t). The first stop (omega = 0) is at
// sqrt(50) t = pi, th = 0.8, at t = pi / sqrt(50) = 0.44429 s. There |k (h - th)| = 0.3 > F_c: the wheel slips back, harmonic about
// h + F_c / k = 0.6 with amplitude 0.2: the second stop is at th = 0.4 after another pi / sqrt(50). One step of exactly that duration
// ends at the stop (omega exactly 0), so the stop positions are checked, not a neighbourhood.
TEST_CASE("F1-A3: a hand spring with Coulomb friction stops at 0.8 rad and then at 0.4 rad, released from rest where the spring exceeds the friction") {
	const double half = WheelPlant::kPi / std::sqrt(50.0);
	FakeTmc4671 chip{0};
	WheelPlant plant(plantParams(0.02, 0.0, 0.1, 10.0));
	plant.attach(chip);
	driveChip(chip, 0);
	plant.setHand(1.0, 0.0, 0.5);
	plant.step(half); // the stop is at the very end of the step: omega is 0 up to rounding
	CHECK(std::fabs(plant.omega()) < 1e-9);
	CHECK(near(plant.theta(), 0.8, 1e-9));
	plant.step(half);
	CHECK(std::fabs(plant.omega()) < 1e-9);
	CHECK(near(plant.theta(), 0.4, 1e-9));
	// the third stop: |k (h - 0.4)| = 0.1 = F_c (equality up to rounding): the wheel stays within 1e-7 of 0.4 rad
	plant.step(half);
	CHECK(near(plant.theta(), 0.4, 1e-7));
	// the same run with 1 ms steps: the first turning point reached within the step size (a turning point inside a step shifts the end position by at most a * dt^2 / 2 = 5e-6)
	WheelPlant fine(plantParams(0.02, 0.0, 0.1, 10.0));
	fine.setHand(1.0, 0.0, 0.5);
	double peak = 0.0;
	for (unsigned i = 0; i < 445; i++) {
		fine.step(kStep);
		peak = std::max(peak, fine.theta());
	}
	CHECK(near(peak, 0.8, 1e-5));
}

// DS p99 and p98-99: PID_POSITION_ACTUAL has 65536 counts per mechanical revolution, i.e. 2 pi / 65536 = 9.5873799e-5 rad per count
// (literal below, not the plant's constant). Turning by (n + 0.5) counts gives n whole counts: n = 16384 is a quarter revolution (pi / 2),
// 32768 a half (pi), 65536 a full revolution (2 pi). With 65535 per revolution the full turn would show 65535, with 4096 per
// revolution 4096: both fail here.
TEST_CASE("F1-A3: turning the wheel by pi / 2, pi and 2 pi moves 0x6B by 16384, 32768 and 65536 counts") {
	const double radPerCount = 9.587379924285257e-5; // 2 pi / 65536
	for (double dir : {1.0, -1.0}) {
		for (int n : {16384, 32768, 65536}) {
			INFO("counts " << n << " direction " << dir);
			FakeTmc4671 chip{0};
			WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
			chip.setReg(0x6B, 0);
			plant.attach(chip);
			driveChip(chip, 0);
			plant.setState(0.0, dir * (n + 0.5) * radPerCount); // one step of 1 s turns by omega rad
			plant.step(1.0);
			CHECK(static_cast<int32_t>(chip.reg(0x6B)) == static_cast<int32_t>(dir) * n);
		}
	}
	// the angles themselves: 16384 counts is pi / 2 = 1.5707963267948966 rad
	CHECK(near(16384 * 9.587379924285257e-5, 1.5707963267948966, 1e-12));
}

// DS p95: the position source must be a mechanical angle (POSITION_SELECTION 9, 10, 11) for 0x6B to be 65536 counts per revolution; any other
// value (default 0 = phi_e selected via PHI_E_SELECTION) records one violation per distinct value.
TEST_CASE("F1-A3: a position source that is not a mechanical angle (0x51) records a plant violation once per value") {
	FakeTmc4671 chip{0};
	WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
	plant.attach(chip);
	driveChip(chip, 0);
	for (uint32_t good : {9u, 10u, 11u}) {
		chip.setReg(0x51, good);
		plant.step(kStep);
	}
	CHECK(plant.violations().empty());
	chip.setReg(0x51, 3); // phi_e_abn
	plant.step(kStep);
	plant.step(kStep);
	CHECK(plant.violations().size() == 1);
	chip.setReg(0x51, 0);
	plant.step(kStep);
	CHECK(plant.violations().size() == 2);
	chip.setReg(0x51, 0x109); // bits above [7:0] are not part of the field
	plant.step(kStep);
	CHECK(plant.violations().size() == 2);
	WheelPlant lax(plantParams(0.02, 0.0, 0.0, 10.0));
	lax.attach(chip);
	lax.setRequireMechanicalPositionSource(false);
	chip.setReg(0x51, 1);
	lax.step(kStep);
	CHECK(lax.violations().empty());
}

// MATH: an opposing torque below F_c: omega0 = 1, tau = -0.050005 (-164 counts, 10 N m full scale), F_c 0.1: while moving the
// wheel decelerates with a = (|tau| + F_c) / J = 7.5002.. until omega = 0, stop angle omega0^2 / (2 a). At rest |tau| < F_c, so
// it sticks: it does not run backwards although the torque points backwards.
TEST_CASE("F1-A3: a torque below the Coulomb friction that opposes the motion stops the wheel and does not drive it backwards") {
	FakeTmc4671 chip{0};
	WheelPlant plant(plantParams(0.02, 0.0, 0.1, 10.0));
	plant.attach(chip);
	plant.setState(0.0, 1.0);
	driveChip(chip, -164);
	const double tau = torqueOf(-164, 10.0);
	const double a = (std::fabs(tau) + 0.1) / 0.02;
	runSteps(plant, 300);
	CHECK(plant.omega() == 0.0);
	CHECK(near(plant.theta(), 1.0 / (2.0 * a), 1e-12));
}

// MATH: at omega = 0 the wheel sticks while |tau| <= F_c (equality sticks) and slips beyond: F_c 0.1, J 0.02, full scale 10.
// 100 counts -> 0.0305 N m: no motion at all. 400 counts -> 0.12207 N m: a = (tau - F_c) / J, after 1 s omega = a, theta = a / 2.
// Boundary: full scale 0.5 N m and 32767 counts give exactly 0.5 N m = F_c 0.5: the wheel does not move (note: a stick and a slip
// with zero net torque give the same state, so this case shows only that the boundary value produces no motion and no NaN, it
// does not distinguish the two rules; the rule "equality sticks" is a documented choice). A hand spring holds the wheel at rest while
// |k (h - theta)| <= F_c (k 1 N m/rad: h 0.05 -> 0.05 N m sticks, h 0.15 -> 0.15 N m slips towards the hand).
TEST_CASE("F1-A3: the wheel at rest sticks while the torque is within the Coulomb friction, does not move at the boundary value and slips in the direction of the torque beyond it") {
	{
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.0, 0.1, 10.0));
		plant.attach(chip);
		driveChip(chip, 100);
		runSteps(plant, 1000);
		CHECK(plant.theta() == 0.0);
		CHECK(plant.omega() == 0.0);
	}
	for (double dir : {1.0, -1.0}) {
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.0, 0.1, 10.0));
		plant.attach(chip);
		driveChip(chip, static_cast<int>(400 * dir));
		const double a = (std::fabs(torqueOf(400, 10.0)) - 0.1) / 0.02;
		runSteps(plant, 1000);
		CHECK(near(plant.omega(), dir * a, 1e-12));
		CHECK(near(plant.theta(), dir * a / 2.0, 1e-12));
	}
	{
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.0, 0.5, 0.5));
		plant.attach(chip);
		driveChip(chip, 32767);
		CHECK(plant.effectiveTorque() == 0.5);
		runSteps(plant, 100);
		CHECK(plant.theta() == 0.0);
	}
	{
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.0, 0.1, 10.0));
		plant.attach(chip);
		driveChip(chip, 0);
		plant.setHand(1.0, 0.0, 0.05);
		runSteps(plant, 100);
		CHECK(plant.theta() == 0.0);
		plant.setHand(1.0, 0.0, 0.15);
		plant.step(kStep);
		CHECK(plant.omega() > 0.0);
		CHECK(plant.theta() > 0.0);
	}
}

// MATH: hands-on spring, J 0.02, k 2, no damping: w0 = sqrt(k / J) = 10 rad/s; from theta0 0.1 at rest, h 0: theta(t) = 0.1 cos(10 t),
// energy k theta0^2 / 2 = 0.01 J constant. After t = 1 s: theta = 0.1 cos(10).
// Closed forms agree across a step split in all three regimes (underdamped c 0.1: alpha = c / 2J = 2.5 < 10; critical c 0.4:
// alpha = 10 = w0; overdamped c 2: alpha = 50): one step of 0.5 s equals 500 steps of 1 ms (semigroup property of the exact
// solution). Critical, from rest at x0 = 0.1: x = x0 e^{-alpha t}(1 + alpha t) = 0.1 e^{-5} 6 at t = 0.5 s, and
// omega = -alpha^2 x0 t e^{-alpha t} = -100 * 0.1 * 0.5 e^{-5}.
TEST_CASE("F1-A3: the hands-on spring and damper use the exact closed forms: undamped cosine with constant energy, and the same state after one step or many in every damping regime") {
	{
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
		plant.attach(chip);
		driveChip(chip, 0);
		plant.setHand(2.0, 0.0, 0.0);
		plant.setState(0.1, 0.0);
		runSteps(plant, 1000);
		CHECK(near(plant.theta(), 0.1 * std::cos(10.0), 1e-9, 1e-12));
		CHECK(near(plant.energy(), 0.01, 1e-9));
	}
	for (double c : {0.1, 0.4, 2.0}) {
		INFO("hand damper " << c);
		WheelPlant fine(plantParams(0.02, 0.0, 0.0, 10.0));
		WheelPlant coarse(plantParams(0.02, 0.0, 0.0, 10.0));
		for (WheelPlant *w : {&fine, &coarse}) {
			w->setHand(2.0, c, 0.02);
			w->setState(0.1, 0.5);
		}
		runSteps(fine, 500);
		coarse.step(0.5);
		CHECK(near(coarse.theta(), fine.theta(), 1e-9, 1e-12));
		CHECK(near(coarse.omega(), fine.omega(), 1e-9, 1e-12));
	}
	// Absolute values at t = 0.5 s from x0 = 0.1, v0 = 0, h = 0, J 0.02, k 2 (w0^2 = 100), b = 0:
	//  underdamped c = 0.1: alpha = 2.5, wd = sqrt(100 - 6.25) = 9.68246: x = e^{-alpha t}(x0 cos wd t + (alpha x0 / wd) sin wd t),
	//     v = -e^{-alpha t} (w0^2 x0 / wd) sin wd t  ->  x = -0.0036550787389, v = 0.2934483299035 (checked against an RK4 integration);
	//  overdamped c = 2: alpha = 50, s = sqrt(2500 - 100) = 48.9898, r1 = -1.0102, r2 = -98.9898, B = (v0 - r1 x0) / (r2 - r1),
	//     A = x0 - B, x = A e^{r1 t} + B e^{r2 t}  ->  x = 0.0609665399121, v = -0.0615887122516 (RK4 agrees to 1e-12).
	{
		WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
		plant.setHand(2.0, 0.1, 0.0);
		plant.setState(0.1, 0.0);
		plant.step(0.5);
		CHECK(near(plant.theta(), -0.0036550787389343776, 1e-9));
		CHECK(near(plant.omega(), 0.29344832990349096, 1e-9));
	}
	{
		WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
		plant.setHand(2.0, 2.0, 0.0);
		plant.setState(0.1, 0.0);
		plant.step(0.5);
		CHECK(near(plant.theta(), 0.06096653991212485, 1e-9));
		CHECK(near(plant.omega(), -0.06158871225162134, 1e-9));
	}
	{
		WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
		plant.setHand(2.0, 0.4, 0.0);
		plant.setState(0.1, 0.0);
		plant.step(0.5);
		CHECK(near(plant.theta(), 0.1 * std::exp(-5.0) * 6.0, 1e-9));
		CHECK(near(plant.omega(), -100.0 * 0.1 * 0.5 * std::exp(-5.0), 1e-9));
	}
}

// MATH: first-order current-loop lag Tl = 0.2 ms behind a commanded torque u switched on at t = 0: the applied torque is
// u (1 - e^{-t / Tl}), its impulse over T is u (T - Tl (1 - e^{-T / Tl})) and with b = F_c = 0 omega = impulse / J (the plant uses
// the mean of the lag output over each step, so the impulse is exact). u = 16384 counts of 10 N m, J 0.02, T = 10 ms.
TEST_CASE("F1-A3: the optional current-loop lag delivers the impulse of a first-order lag and the default is ideal") {
	FakeTmc4671 chip{0};
	WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0, 0.0002));
	plant.attach(chip);
	driveChip(chip, 16384);
	const double u = torqueOf(16384, 10.0);
	// first step: mean of u (1 - e^{-t/Tl}) over dt = u (1 - (Tl / dt)(1 - e^{-dt/Tl})) = u (1 - 0.2 (1 - e^{-5}))
	plant.step(kStep);
	CHECK(near(plant.tauApplied(), u * (1.0 - 0.2 * (1.0 - std::exp(-5.0))), 1e-12));
	runSteps(plant, 9);
	const double impulse = u * (0.01 - 0.0002 * (1.0 - std::exp(-0.01 / 0.0002)));
	CHECK(near(plant.omega(), impulse / 0.02, 1e-12));
	CHECK(near(plant.tauCommanded(), u, 1e-15));
	FakeTmc4671 chip2{0};
	WheelPlant ideal(plantParams(0.02, 0.0, 0.0, 10.0));
	ideal.attach(chip2);
	driveChip(chip2, 16384);
	ideal.step(kStep);
	CHECK(ideal.tauApplied() == ideal.tauCommanded());
}

// Datasheet DS p98 (target), p97-98 (mode), p84-85 (PWM mode), p104 and p52 (ENI), p97 (limit): the torque reaches the wheel only in
// torque mode (0x63 [7:0] = 1) with PWM mode FOC (0x1A [7:0] = 7) and ENABLE_IN high (0x76 bit 15); otherwise 0. Clamped to the
// limit 0x5E [15:0]. Upper bits of 0x63 / 0x1A and the flux half of 0x64 do not matter.
TEST_CASE("F1-A3: the torque target reaches the wheel only in torque mode with FOC PWM and ENI high, and each condition off gives no torque") {
	FakeTmc4671 chip{0};
	WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
	plant.attach(chip);
	driveChip(chip, 2000);
	CHECK(plant.effectiveTorque() == torqueOf(2000, 10.0));
	for (uint32_t mode : {0u, 2u, 3u, 4u, 8u}) { // stopped, velocity, position, prbs flux, uq_ud_ext: none is torque mode (DS p97-98)
		driveChip(chip, 2000, mode);
		INFO("motion mode " << mode);
		CHECK(plant.effectiveTorque() == 0.0);
	}
	for (uint32_t pwm : {0u, 1u, 2u, 3u, 5u, 6u}) { // every PWM_CHOP value except 7 is off or a chopper mode (DS p84-85)
		driveChip(chip, 2000, 1, pwm);
		INFO("pwm mode " << pwm);
		CHECK(plant.effectiveTorque() == 0.0);
	}
	driveChip(chip, 2000, 1, 7, false);
	CHECK(plant.effectiveTorque() == 0.0);
	// A step with a gated-off torque leaves a resting wheel at rest
	plant.step(kStep);
	CHECK(plant.tauApplied() == 0.0);
	CHECK(plant.omega() == 0.0);
	// Bits outside the fields do not matter: PID_FLUX_TARGET (15:0), 0x63 bits 31:8 (PID type, sample rate), 0x1A bit 8 (SVPWM)
	driveChip(chip, 500, 0x80000001u, 0x107u);
	chip.setReg(0x64, (500u << 16) | 0x1234u);
	CHECK(plant.effectiveTorque() == torqueOf(500, 10.0));
	// not attached: no torque
	plant.detach();
	CHECK(plant.effectiveTorque() == 0.0);
}

TEST_CASE("F1-A3: the torque target is clamped symmetrically to the limit of 0x5E and a negative target keeps its sign") {
	FakeTmc4671 chip{0};
	WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
	plant.attach(chip);
	driveChip(chip, 20000, 1, 7, true, 1000);
	CHECK(plant.effectiveTorque() == torqueOf(1000, 10.0));
	driveChip(chip, -20000, 1, 7, true, 1000);
	CHECK(plant.effectiveTorque() == torqueOf(-1000, 10.0));
	driveChip(chip, 600, 1, 7, true, 1000); // inside the limit: unchanged
	CHECK(plant.effectiveTorque() == torqueOf(600, 10.0));
	driveChip(chip, 600, 1, 7, true, 0); // limit 0: no torque
	CHECK(plant.effectiveTorque() == 0.0);
	driveChip(chip, 32767, 1, 7, true, 0x7FFF); // full scale: exactly the full-scale torque
	CHECK(plant.effectiveTorque() == 10.0);
	driveChip(chip, -32768, 1, 7, true, 0x7FFF); // s16 minimum with the default limit: -32767 counts
	CHECK(plant.effectiveTorque() == -10.0);
	// the static helper agrees on the same bit fields
	CHECK(WheelPlant::effectiveTorqueCounts(0xFC180000u, 1, 7, 0x8000, 1000) == -1000); // 0xFC18 = -1000 as s16
	CHECK(WheelPlant::effectiveTorqueCounts(0x00640000u, 1, 7, 0x0000, 1000) == 0);	 // ENI low
}

// MATH (0x6B, DS p98-99: 65536 counts per revolution, differences accumulated, wrap-around of the s32): omega = 0.35 counts/step
// is 0.35 / 1e-3 / (65536 / 2 pi) rad/s. After 101 steps the total is 35.35 counts, so the register moved by trunc(35.35) = 35
// (the remainder 0.35 is carried, never lost) and each step added 0 or 1. A rounding without carry would give 0. Negative
// direction: -35 (two's complement, truncation towards zero).
TEST_CASE("F1-A3: 0x6B advances by whole counts with the fractional remainder carried, in both directions") {
	for (double dir : {1.0, -1.0}) {
		INFO("direction " << dir);
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
		chip.setReg(0x6B, 0);
		plant.attach(chip);
		driveChip(chip, 0);
		plant.setState(0.0, dir * 0.35 / kStep / WheelPlant::kCountsPerRad);
		uint32_t prev = chip.reg(0x6B);
		bool stepsOk = true;
		for (unsigned i = 0; i < 101; i++) {
			plant.step(kStep);
			const int32_t d = static_cast<int32_t>(chip.reg(0x6B) - prev);
			stepsOk = stepsOk && (d == 0 || d == static_cast<int32_t>(dir));
			prev = chip.reg(0x6B);
		}
		CHECK(stepsOk);
		CHECK(chip.reg(0x6B) == static_cast<uint32_t>(static_cast<int32_t>(35 * dir)));
		CHECK(plant.lastPositionRegister() == chip.reg(0x6B));
		CHECK(chip.reg(0x68) == 0); // the plant does not move the position target (no mirror)
	}
}

// MATH: 20.25 counts/step. Start 0x7FFFFFF0, 2 steps = 40.5 -> +40: 0x7FFFFFF0 + 40 = 0x80000018 = -2147483624 as s32 (wrap past
// INT32_MAX). Downwards from 0x80000010 (-2147483632): -40 -> 0x7FFFFFE8 = 2147483624.
TEST_CASE("F1-A3: 0x6B wraps around at the 32 bit limits in both directions") {
	const double w = 20.25 / kStep / WheelPlant::kCountsPerRad;
	{
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
		plant.attach(chip);
		driveChip(chip, 0);
		chip.setReg(0x6B, 0x7FFFFFF0u);
		plant.setState(0.0, w);
		runSteps(plant, 2);
		CHECK(static_cast<int32_t>(chip.reg(0x6B)) == -2147483624);
	}
	{
		FakeTmc4671 chip{0};
		WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
		plant.attach(chip);
		driveChip(chip, 0);
		chip.setReg(0x6B, 0x80000010u);
		plant.setState(0.0, -w);
		runSteps(plant, 2);
		CHECK(chip.reg(0x6B) == 0x7FFFFFE8u);
	}
}

// MATH: 20.25 counts/step. 10 steps = 202.5 counts: register 202, remainder 0.5. The firmware (Axis / TMC4671::setTmcPos) writes
// 0 to 0x6B. The plant reads the register each step, so it rebases: 9 more steps are 182.25 counts, plus the carried 0.5 =
// 182.75 -> the register is 182, not 202 + 182 (the write is not overwritten by the plant's own count). The angle is continuous.
TEST_CASE("F1-A3: a firmware write to 0x6B rebases the count and the plant continues from the written value") {
	const double w = 20.25 / kStep / WheelPlant::kCountsPerRad;
	FakeTmc4671 chip{0};
	WheelPlant plant(plantParams(0.02, 0.0, 0.0, 10.0));
	chip.setReg(0x6B, 0);
	plant.attach(chip);
	driveChip(chip, 0);
	plant.setState(0.0, w);
	runSteps(plant, 10);
	CHECK(chip.reg(0x6B) == 202u);
	chip.setReg(0x6B, 0); // the firmware's zeroing write
	runSteps(plant, 9);
	CHECK(chip.reg(0x6B) == 182u);
	CHECK(near(plant.thetaCounts(), 19 * 20.25, 1e-12)); // the plant's own angle was not rebased
}

TEST_CASE("F1-A3: a plant without valid parameters records a violation and does not integrate, a bad step size too") {
	FakeTmc4671 chip{0};
	WheelPlant plant;
	plant.attach(chip);
	plant.setState(0.0, 1.0);
	plant.step(kStep);
	CHECK_FALSE(plant.violations().empty());
	CHECK(plant.theta() == 0.0);
	WheelPlant ok(plantParams(0.02, 0.0, 0.0, 10.0));
	ok.step(0.0);
	ok.step(-1.0);
	CHECK(ok.violations().size() == 2);
	WheelPlant bad(plantParams(0.0, 0.0, 0.0, 10.0)); // J = 0
	CHECK_FALSE(bad.violations().empty());
	// the named nominal sets are inside the ranges of plan 6.3
	for (const WheelPlant::Params &p : {WheelPlant::Params::nominalLight(), WheelPlant::Params::nominalMid(), WheelPlant::Params::nominalHeavy()}) {
		WheelPlant n(p);
		CHECK(n.violations().empty());
		CHECK_FALSE(n.outsideNominal());
	}
	WheelPlant outside(plantParams(0.5, 0.0, 0.0, 10.0));
	CHECK(outside.outsideNominal());
}

// =====================================================================================================================
// F1-A5: OscillationDetector self-tests on synthetic signals x(t) = A e^{sigma t} sin(2 pi f t + phi). Same-sign peaks are one
// period T = 1 / f apart and the sine factor is identical there, so the amplitude ratio per cycle is exactly e^{sigma T} =
// e^{sigma / f}. Sample rate 1 kHz (100 samples per period at f = 10 Hz). sigma = f ln(r) gives a ratio r per cycle.
// =====================================================================================================================
namespace {

std::vector<double> synth(double seconds, double f, double ratioPerCycle, double A, double phi = 0.3, double offset = 0.0, double lsb = 0.0) {
	const double dt = 1e-3;
	const double sigma = f * std::log(ratioPerCycle);
	std::vector<double> x;
	for (size_t i = 0; i < static_cast<size_t>(seconds / dt + 0.5); i++) {
		const double t = static_cast<double>(i) * dt;
		double v = A * std::exp(sigma * t) * std::sin(2.0 * WheelPlant::kPi * f * t + phi) + offset;
		if (lsb > 0.0) {
			v = std::round(v / lsb) * lsb; // quantised like an encoder count
		}
		x.push_back(v);
	}
	return x;
}

OscillationDetector::Config detCfg(double floor, double eps = 0.05) {
	OscillationDetector::Config c;
	c.samplePeriodS = 1e-3;
	c.floor = floor;
	c.eps = eps;
	return c;
}

} // namespace

TEST_CASE("F1-A5: a decaying sinusoid is classified decaying with its frequency and cycle ratio e^(sigma / f)") {
	const auto r = OscillationDetector::analyse(synth(2.0, 10.0, 0.8, 1.0), detCfg(0.01));
	REQUIRE(r.valid);
	CHECK(r.cls == OscillationDetector::Class::Decaying);
	CHECK(near(r.frequencyHz, 10.0, 1e-3));
	CHECK(near(r.ratio, 0.8, 1e-3));
	CHECK(r.cycles >= 15);
	for (double q : r.cycleRatios) {
		CHECK(near(q, 0.8, 2e-3));
	}
}

TEST_CASE("F1-A5: a sinusoid with a constant amplitude is sustained from 10 cycles on and a ratio inside 1 +- eps counts as sustained") {
	for (double ratio : {1.0, 1.03, 0.97}) {
		INFO("ratio per cycle " << ratio);
		const auto r = OscillationDetector::analyse(synth(2.0, 10.0, ratio, 0.5), detCfg(0.01));
		REQUIRE(r.valid);
		CHECK(r.cls == OscillationDetector::Class::Sustained);
		CHECK(near(r.ratio, ratio, 1e-3));
		CHECK(near(r.frequencyHz, 10.0, 1e-3));
	}
}

TEST_CASE("F1-A5: a growing sinusoid is classified growing and a ratio just outside 1 +- eps is a trend") {
	const auto g = OscillationDetector::analyse(synth(1.2, 10.0, 1.2, 0.01), detCfg(0.001));
	REQUIRE(g.valid);
	CHECK(g.cls == OscillationDetector::Class::Growing);
	CHECK(near(g.ratio, 1.2, 1e-3));
	const auto g2 = OscillationDetector::analyse(synth(2.0, 10.0, 1.08, 0.5), detCfg(0.01));
	CHECK(g2.cls == OscillationDetector::Class::Growing);
	const auto d2 = OscillationDetector::analyse(synth(2.0, 10.0, 0.92, 0.5), detCfg(0.01));
	CHECK(d2.cls == OscillationDetector::Class::Decaying);
}

TEST_CASE("F1-A5: too few cycles above the floor give insufficient: a short sustained record, a record shorter than the trend minimum, a signal inside the floor") {
	// sustained signal over 0.6 s = 6 periods: 5 cycles between same-sign peaks, below the 10 of plan 6.3
	const auto shortRec = OscillationDetector::analyse(synth(0.6, 10.0, 1.0, 0.5), detCfg(0.01));
	CHECK(shortRec.cls == OscillationDetector::Class::Insufficient);
	CHECK(shortRec.cycles >= 4);
	CHECK(shortRec.cycles < 10);
	// 0.25 s = 2.5 periods: 3 peaks of a sign = 2 cycles, below the trend minimum of 3 cycles, even for a strongly decaying signal
	const auto tiny = OscillationDetector::analyse(synth(0.25, 10.0, 0.5, 1.0), detCfg(0.01));
	CHECK(tiny.cls == OscillationDetector::Class::Insufficient);
	// amplitude 0.5 below the floor 1: no half-wave at all
	const auto below = OscillationDetector::analyse(synth(2.0, 10.0, 1.0, 0.5), detCfg(1.0));
	CHECK(below.cls == OscillationDetector::Class::Insufficient);
	CHECK(below.peaks.empty());
	// a fast decay: only the peaks above the floor count (0.5^k > 0.05 for k <= 4), and that is enough for a trend
	const auto fast = OscillationDetector::analyse(synth(2.0, 10.0, 0.5, 1.0), detCfg(0.05));
	CHECK(fast.cls == OscillationDetector::Class::Decaying);
	CHECK(fast.cycles >= 3);
	CHECK(fast.cycles <= 5);
	CHECK(near(fast.ratio, 0.5, 1e-3));
}

TEST_CASE("F1-A5: quantised signals near the floor keep their classification and a signal that never leaves the quantisation noise is insufficient") {
	// amplitude 100 counts, floor 5 counts: 1 count error on a peak is 1 %
	const auto sus = OscillationDetector::analyse(synth(3.0, 10.0, 1.0, 100.0, 0.3, 0.0, 1.0), detCfg(5.0));
	CHECK(sus.cls == OscillationDetector::Class::Sustained);
	CHECK(near(sus.ratio, 1.0, 0.01));
	// decaying 0.9 per cycle from 50 counts, floor 3 counts: peaks above the floor for 0.9^k * 50 > 3, k <= 26
	const auto dec = OscillationDetector::analyse(synth(2.0, 10.0, 0.9, 50.0, 0.3, 0.0, 1.0), detCfg(3.0));
	CHECK(dec.cls == OscillationDetector::Class::Decaying);
	CHECK(near(dec.ratio, 0.9, 0.02));
	// amplitude 0.8 count rounds to values within +-1, which never exceed the floor 1.0: no half-wave
	const auto noise = OscillationDetector::analyse(synth(3.0, 10.0, 1.0, 0.8, 0.3, 0.0, 1.0), detCfg(1.0));
	CHECK(noise.cls == OscillationDetector::Class::Insufficient);
	// a centre offset is removed with Config::center
	OscillationDetector::Config c = detCfg(0.01);
	c.center = 5.0;
	const auto off = OscillationDetector::analyse(synth(2.0, 10.0, 0.8, 1.0, 0.3, 5.0), c);
	CHECK(off.cls == OscillationDetector::Class::Decaying);
	CHECK(near(off.ratio, 0.8, 1e-3));
}

TEST_CASE("F1-A5: the detector refuses an unusable configuration or a signal with a NaN") {
	OscillationDetector::Config bad = detCfg(0.01);
	bad.samplePeriodS = 0.0;
	CHECK_FALSE(OscillationDetector::analyse(synth(1.0, 10.0, 1.0, 1.0), bad).valid);
	bad = detCfg(0.0);
	CHECK_FALSE(OscillationDetector::analyse(synth(1.0, 10.0, 1.0, 1.0), bad).valid);
	std::vector<double> x = synth(1.0, 10.0, 1.0, 1.0);
	x[10] = std::nan("");
	const auto r = OscillationDetector::analyse(x, detCfg(0.01));
	CHECK_FALSE(r.valid);
	CHECK_FALSE(r.error.empty());
	CHECK(r.cls == OscillationDetector::Class::Insufficient);
}

// Lead decision 2026-11-18: classification on the trailing window of the last minCycles cycle ratios (OscillationDetector.h step 5).
// Reviewer's signal: amplitude 100 falling to 30 in 5 cycles (ratio 0.3^(1/5) = 0.786 per cycle), then 12 cycles at constant amplitude 30
// (10 Hz, 1.7 s): the last 10 cycle ratios are 1 (constant envelope), so the end state is sustained although the whole record decays. A
// limit cycle of amplitude 1 for 1.2 s that then grows by 1.15 per cycle for 12 cycles: the last 10 ratios are 1.15: growing.
namespace {
std::vector<double> synthEnv(double seconds, double f, double (*envelope)(double), double phi = 0.3) {
	std::vector<double> x;
	for (size_t i = 0; i < static_cast<size_t>(seconds / 1e-3 + 0.5); i++) {
		const double t = static_cast<double>(i) * 1e-3;
		x.push_back(envelope(t) * std::sin(2.0 * WheelPlant::kPi * f * t + phi));
	}
	return x;
}
double decayThenLimit(double t) { return t < 0.5 ? 100.0 * std::exp(10.0 * std::log(std::pow(0.3, 0.2)) * t) : 30.0; }
double limitThenGrow(double t) { return t < 1.2 ? 1.0 : std::exp(10.0 * std::log(1.15) * (t - 1.2)); }
} // namespace

TEST_CASE("F1-A5: a decay that settles into a limit cycle above the floor is sustained, and a limit cycle that starts to grow at the end is growing") {
	const auto lim = OscillationDetector::analyse(synthEnv(1.7, 10.0, decayThenLimit), detCfg(1.0));
	REQUIRE(lim.valid);
	CHECK(lim.cls == OscillationDetector::Class::Sustained);
	CHECK(lim.windowCycles == 10);
	CHECK(near(lim.windowRatio, 1.0, 1e-3));
	CHECK(lim.ratio < 0.97); // the whole record still shows the decay (reported, not used to classify)
	CHECK(lim.cycles >= 15);
	const auto grow = OscillationDetector::analyse(synthEnv(2.4, 10.0, limitThenGrow), detCfg(0.01));
	REQUIRE(grow.valid);
	CHECK(grow.cls == OscillationDetector::Class::Growing);
	CHECK(near(grow.windowRatio, 1.15, 2e-3));
	CHECK(grow.ratio > 1.0);
	CHECK(grow.ratio < grow.windowRatio); // the whole record mixes the flat part in
}

// A wrong centre biases a trend (OscillationDetector.h step 3): signal 10 * 0.8^(t f) sin(...) + d, analysed with centre 0. Peaks on the
// offset's side are A_k + d: ratio (A + d) pulled towards 1; on the other side A_k - d: ratio pushed away from 1. d = +2 and d = -2 give
// a ratio on opposite sides of the true 0.8, and neither is 0.8.
TEST_CASE("F1-A5: a wrong centre biases a decaying ratio, towards 1 on the side of the offset and away from 1 on the other side") {
	const auto plus = OscillationDetector::analyse(synth(2.0, 10.0, 0.8, 10.0, 0.3, 2.0), detCfg(0.01));
	const auto minus = OscillationDetector::analyse(synth(2.0, 10.0, 0.8, 10.0, 0.3, -2.0), detCfg(0.01));
	REQUIRE(plus.valid);
	REQUIRE(minus.valid);
	MESSAGE("wrong centre: ratio " << plus.ratio << " for offset +2, " << minus.ratio << " for offset -2 (true 0.8)");
	CHECK(std::fabs(plus.ratio - 0.8) > 0.005);
	CHECK(std::fabs(minus.ratio - 0.8) > 0.005);
	CHECK(((plus.ratio - 0.8) * (minus.ratio - 0.8)) < 0.0); // opposite sides of the true value
	// The detector uses the positive peaks when both streams have as many peaks (it prefers positive). For d = +2 the positive peaks are on
	// the offset's side: the ratio is pulled towards 1 (between 0.8 and 1). For d = -2 the positive peaks are on the far side: pushed away
	// (below 0.8), which is what the opposite-sides check above shows.
	CHECK(plus.ratio > 0.8);
	CHECK(plus.ratio < 1.0);
	OscillationDetector::Config c = detCfg(0.01);
	c.center = 0.0;
	CHECK(OscillationDetector::analyse(synth(2.0, 10.0, 1.0, 10.0, 0.3, 2.0), c).cls == OscillationDetector::Class::Sustained); // constant amplitude: no bias
}

// Non-integer samples per period: 8.1 samples per period (f = 1000 / 8.1 = 123.457 Hz at 1 kHz), decay 0.9 per cycle, 0.3 s = 37 cycles.
// The parabola through three samples near a peak of a sine at 8.1 samples per period has a worst-case peak value error of 0.81 %
// (h = 2 pi / 8.1 rad between samples, vertex error of the parabola through cos at offset d in [-h/2, h/2]; derived with the three-point
// vertex formula, maximum over d). A peak ratio is wrong by up to about 1.6 % per pair, the geometric mean of the whole record by
// 0.81 % * 2 / n or less (n >= 25 cycles above the floor 0.02 -> 0.07 %); tolerance 0.5 % on the ratio and 0.3 % on the frequency.
TEST_CASE("F1-A5: at 8.1 samples per period the parabolic peak refinement gives the cycle ratio within 0.5 % and the frequency within 0.3 %") {
	const double f = 1000.0 / 8.1;
	const auto r = OscillationDetector::analyse(synth(0.3, f, 0.9, 1.0), detCfg(0.02));
	REQUIRE(r.valid);
	CHECK(r.cls == OscillationDetector::Class::Decaying);
	CHECK(r.cycles >= 25);
	CHECK(near(r.ratio, 0.9, 0.005));
	CHECK(near(r.frequencyHz, f, 0.003));
}

// =====================================================================================================================
// F1-A5 (frequency response): FrequencyResponse self-tests. fs = 1000 Hz, f = 50 Hz (20 samples per period), N = 200 = 10 periods.
//  - pure delay y[n] = x[n - d]: gain 1, phase = -2 pi f d / fs: d = 3 samples -> -2 pi 50 3 / 1000 = -0.3 * pi = -0.94248 rad;
//  - first-order IIR y[n] = a y[n-1] + (1 - a) x[n], H(z) = (1 - a) / (1 - a z^-1) (Oppenheim and Schafer, the one-pole low-pass),
//    at w = 2 pi f / fs: |H| = (1 - a) / sqrt(1 - 2 a cos w + a^2), arg H = -atan2(a sin w, 1 - a cos w) (negative: a low-pass lags).
// =====================================================================================================================
namespace {
std::vector<double> sine(size_t from, size_t n, double f, double fs, double amp = 1.0, double offset = 0.0) {
	std::vector<double> x;
	for (size_t i = from; i < from + n; i++) {
		x.push_back(offset + amp * std::sin(2.0 * WheelPlant::kPi * f * static_cast<double>(i) / fs + 0.4));
	}
	return x;
}
FrequencyResponse::Options freqOpt(double f = 50.0, size_t minPeriods = 5) {
	FrequencyResponse::Options o;
	o.sampleRateHz = 1000.0;
	o.frequencyHz = f;
	o.minPeriods = minPeriods;
	return o;
}
} // namespace

TEST_CASE("F1-A5: the DFT of a pure delay of 3 samples has gain 1 and phase -2 pi f d / fs, output lagging is negative") {
	const auto in = sine(0, 200, 50.0, 1000.0, 2.0);
	std::vector<double> out;
	for (size_t i = 0; i < 200; i++) { // y[n] = x[n - 3], x[n] = 2 sin(2 pi 50 n / 1000 + 0.4)
		out.push_back(2.0 * std::sin(2.0 * WheelPlant::kPi * 50.0 * (static_cast<double>(i) - 3.0) / 1000.0 + 0.4));
	}
	const auto r = FrequencyResponse::measure(in, out, freqOpt());
	REQUIRE(r.valid);
	CHECK(near(r.gain, 1.0, 1e-12));
	CHECK(r.phaseRad == doctest::Approx(-0.3 * WheelPlant::kPi).epsilon(1e-12));
	CHECK(r.phaseDeg == doctest::Approx(-54.0).epsilon(1e-12));
	CHECK(near(r.periods, 10.0, 1e-12));
}

TEST_CASE("F1-A5: the DFT of a one-pole low-pass applied to a sine matches |H| and arg H of the analytic response, and a constant offset does not bias it") {
	const double a = 0.9;
	const double w = 2.0 * WheelPlant::kPi * 50.0 / 1000.0;
	const double gain = (1.0 - a) / std::sqrt(1.0 - 2.0 * a * std::cos(w) + a * a);
	const double phase = -std::atan2(a * std::sin(w), 1.0 - a * std::cos(w));
	for (double offset : {0.0, 3.0}) {
		INFO("offset " << offset);
		const size_t warm = 600; // 0.9^600 = 3e-28: settled
		const auto x = sine(0, warm + 200, 50.0, 1000.0, 1.5, offset);
		std::vector<double> y(x.size(), 0.0);
		double s = 0.0;
		for (size_t i = 0; i < x.size(); i++) {
			s = a * s + (1.0 - a) * x[i];
			y[i] = s;
		}
		// the window starts at sample 600 = 30 periods: whole periods, phase of the sine at its start is identical for in and out
		const std::vector<double> in(x.begin() + warm, x.end());
		const std::vector<double> out(y.begin() + warm, y.end());
		const auto r = FrequencyResponse::measure(in, out, freqOpt());
		REQUIRE(r.valid);
		CHECK(near(r.gain, gain, 1e-9));
		CHECK(r.phaseRad == doctest::Approx(phase).epsilon(1e-9));
		CHECK(r.phaseRad < 0.0);
	}
}

TEST_CASE("F1-A5: the DFT reports an error for a window that is not a whole number of periods, is too short, or has unusable arguments") {
	const auto in = sine(0, 200, 50.0, 1000.0);
	// 205 samples at 50 Hz / 1 kHz = 10.25 periods
	CHECK_FALSE(FrequencyResponse::measure(sine(0, 205, 50.0, 1000.0), sine(0, 205, 50.0, 1000.0), freqOpt()).valid);
	CHECK_FALSE(FrequencyResponse::measure(sine(0, 205, 50.0, 1000.0), sine(0, 205, 50.0, 1000.0), freqOpt()).error.empty());
	// 3 periods but 5 required
	CHECK_FALSE(FrequencyResponse::measure(sine(0, 60, 50.0, 1000.0), sine(0, 60, 50.0, 1000.0), freqOpt()).valid);
	// 3 periods and 3 required: fine
	CHECK(FrequencyResponse::measure(sine(0, 60, 50.0, 1000.0), sine(0, 60, 50.0, 1000.0), freqOpt(50.0, 3)).valid);
	// lengths differ, empty, Nyquist, zero frequency, minPeriods 0, no energy at the bin
	CHECK_FALSE(FrequencyResponse::measure(in, sine(0, 100, 50.0, 1000.0), freqOpt()).valid);
	CHECK_FALSE(FrequencyResponse::measure({}, {}, freqOpt()).valid);
	CHECK_FALSE(FrequencyResponse::measure(in, in, freqOpt(500.0)).valid);
	CHECK_FALSE(FrequencyResponse::measure(in, in, freqOpt(0.0)).valid);
	CHECK_FALSE(FrequencyResponse::measure(in, in, freqOpt(50.0, 0)).valid);
	CHECK_FALSE(FrequencyResponse::measure(std::vector<double>(200, 1.0), in, freqOpt()).valid);
	// a window of a whole number of periods but under 4 samples per period (f = 300 Hz: 3.33 samples per period, 300 samples = 90 periods)
	CHECK_FALSE(FrequencyResponse::measure(sine(0, 300, 300.0, 1000.0), sine(0, 300, 300.0, 1000.0), freqOpt(300.0)).valid);
}
} // TEST_SUITE
