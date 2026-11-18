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

} // TEST_SUITE
