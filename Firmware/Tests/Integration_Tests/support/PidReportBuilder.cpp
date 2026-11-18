/*
 * PidReportBuilder.cpp: see the header. The usage table is USB_PID1_01.pdf Table 2 "Physical Input Device Page" (pages 6-7).
 */
#include "PidReportBuilder.h"

#include "doctest.h"

#include <sstream>

namespace pid {

const std::vector<UsageName> &usageTable() {
	static const std::vector<UsageName> table = {
	{0x01, "Physical Interface Device"},
	{0x20, "Normal"},
	{0x21, "Set Effect Report"},
	{0x22, "Effect Block Index"},
	{0x23, "Parameter Block Offset"},
	{0x24, "ROM Flag"},
	{0x25, "Effect Type"},
	{0x26, "ET Constant Force"},
	{0x27, "ET Ramp"},
	{0x28, "ET Custom Force Data"},
	{0x30, "ET Square"},
	{0x31, "ET Sine"},
	{0x32, "ET Triangle"},
	{0x33, "ET Sawtooth Up"},
	{0x34, "ET Sawtooth Down"},
	{0x40, "ET Spring"},
	{0x41, "ET Damper"},
	{0x42, "ET Inertia"},
	{0x43, "ET Friction"},
	{0x50, "Duration"},
	{0x51, "Sample Period"},
	{0x52, "Gain"},
	{0x53, "Trigger Button"},
	{0x54, "Trigger Repeat Interval"},
	{0x55, "Axes Enable"},
	{0x56, "Direction Enable"},
	{0x57, "Direction"},
	{0x58, "Type Specific Block Offset"},
	{0x59, "Block Type"},
	{0x5A, "Set Envelope Report"},
	{0x5B, "Attack Level"},
	{0x5C, "Attack Time"},
	{0x5D, "Fade Level"},
	{0x5E, "Fade Time"},
	{0x5F, "Set Condition Report"},
	{0x60, "CP Offset"},
	{0x61, "Positive Coefficient"},
	{0x62, "Negative Coefficient"},
	{0x63, "Positive Saturation"},
	{0x64, "Negative Saturation"},
	{0x65, "Dead Band"},
	{0x66, "Download Force Sample"},
	{0x67, "Isoch Custom Force Enable"},
	{0x68, "Custom Force Data Report"},
	{0x69, "Custom Force Data"},
	{0x6A, "Custom Force Vendor Defined Data"},
	{0x6B, "Set Custom Force Report"},
	{0x6C, "Custom Force Data Offset"},
	{0x6D, "Sample Count"},
	{0x6E, "Set Periodic Report"},
	{0x6F, "Offset"},
	{0x70, "Magnitude"},
	{0x71, "Phase"},
	{0x72, "Period"},
	{0x73, "Set Constant Force Report"},
	{0x74, "Set Ramp Force Report"},
	{0x75, "Ramp Start"},
	{0x76, "Ramp End"},
	{0x77, "Effect Operation Report"},
	{0x78, "Effect Operation"},
	{0x79, "Op Effect Start"},
	{0x7A, "Op Effect Start Solo"},
	{0x7B, "Op Effect Stop"},
	{0x7C, "Loop Count"},
	{0x7D, "Device Gain Report"},
	{0x7E, "Device Gain"},
	{0x7F, "PID Pool Report"},
	{0x80, "RAM Pool Size"},
	{0x81, "ROM Pool Size"},
	{0x82, "ROM Effect Block Count"},
	{0x83, "Simultaneous Effects Max"},
	{0x84, "Pool Alignment"},
	{0x85, "PID Pool Move Report"},
	{0x86, "Move Source"},
	{0x87, "Move Destination"},
	{0x88, "Move Length"},
	{0x89, "PID Block Load Report"},
	{0x8B, "Block Load Status"},
	{0x8C, "Block Load Success"},
	{0x8D, "Block Load Full"},
	{0x8E, "Block Load Error"},
	{0x8F, "Block Handle"},
	{0x90, "PID Block Free Report"},
	{0x91, "Type Specific Block Handle"},
	{0x92, "PID State Report"},
	{0x94, "Effect Playing"},
	{0x95, "PID Device Control Report"},
	{0x96, "PID Device Control"},
	{0x97, "DC Enable Actuators"},
	{0x98, "DC Disable Actuators"},
	{0x99, "DC Stop All Effects"},
	{0x9A, "DC Device Reset"},
	{0x9B, "DC Device Pause"},
	{0x9C, "DC Device Continue"},
	{0x9F, "Device Paused"},
	{0xA0, "Actuators Enabled"},
	{0xA4, "Safety Switch"},
	{0xA5, "Actuator Override Switch"},
	{0xA6, "Actuator Power"},
	{0xA7, "Start Delay"},
	{0xA8, "Parameter Block Size"},
	{0xA9, "Device Managed Pool"},
	{0xAA, "Shared Parameter Blocks"},
	{0xAB, "Create New Effect Report"},
	{0xAC, "RAM Pool Available"},
	};
	return table;
}

uint32_t usageByName(const std::string &name) {
	for (const UsageName &e : usageTable()) {
		if (name == e.name) {
			return u(e.id);
		}
	}
	return 0xFFFFFFFFu;
}

std::string nameOf(uint32_t extendedUsage) {
	if ((extendedUsage >> 16) != kPage) {
		return "";
	}
	for (const UsageName &e : usageTable()) {
		if (e.id == (extendedUsage & 0xFFFF)) {
			return e.name;
		}
	}
	return "";
}

namespace {
std::string usageText(uint32_t ext) {
	std::ostringstream s;
	const std::string n = nameOf(ext);
	s << (n.empty() ? "usage" : n) << " (page 0x" << std::hex << (ext >> 16) << ", id 0x" << (ext & 0xFFFF) << ")";
	return s.str();
}
} // namespace

// ---------------------------------------------------------------------------------------------------------------------
bool DecodedReport::get(uint32_t usage, int64_t &value, size_t occurrence) const {
	for (const DecodedValue &v : values) {
		if (v.field->isVariable() && v.usage == usage) {
			if (occurrence == 0) {
				value = v.value;
				return true;
			}
			occurrence--;
		}
	}
	return false;
}

bool DecodedReport::selects(uint32_t selector) const {
	for (const DecodedValue &v : values) {
		if (!v.field->isVariable() && !v.field->isConstant()) {
			int64_t sel = 0;
			if (v.field->selectorValue(selector, sel) && sel == v.value) {
				return true;
			}
		}
	}
	return false;
}

bool DecodedReport::getIn(const std::vector<uint32_t> &subPath, uint32_t usage, int64_t &value) const {
	for (const DecodedValue &v : values) {
		if (!v.field->isVariable() || v.usage != usage) {
			continue;
		}
		size_t want = 0;
		for (const hid::CollectionRef &c : v.field->path) {
			if (want < subPath.size() && c.usage == subPath[want]) {
				want++;
			}
		}
		if (want == subPath.size()) {
			value = v.value;
			return true;
		}
	}
	return false;
}

// ---------------------------------------------------------------------------------------------------------------------
PidReport::PidReport(const hid::HidReportDescriptor &desc, const hid::Report *report, std::string name, bool failTests)
	: desc_(desc), report_(report), name_(std::move(name)), failTests_(failTests) {
	if (report_ != nullptr) {
		bytes_.assign(report_->byteLength(), 0);
		if (report_->hasIdByte) {
			bytes_[0] = report_->reportId;
		}
	}
}

void PidReport::error(const std::string &what) {
	const std::string msg = "PidReport '" + name_ + "': " + what;
	errors_.push_back(msg);
	if (failTests_) {
		FAIL_CHECK(msg);
	}
}

PidReport &PidReport::setImpl(const std::vector<uint32_t> &subPath, uint32_t fieldUsage, uint32_t element, bool useElement, int64_t value, size_t occurrence) {
	if (report_ == nullptr) {
		error("the report is not declared by the descriptor (exactly once), cannot set " + usageText(fieldUsage));
		return *this;
	}
	const hid::Field *found = nullptr;
	for (size_t fi : report_->fields) {
		const hid::Field &f = desc_.fields()[fi];
		if (f.isConstant() || !f.isVariable() || !f.hasUsage(fieldUsage)) {
			continue;
		}
		size_t want = 0;
		for (const hid::CollectionRef &c : f.path) {
			if (want < subPath.size() && c.usage == subPath[want]) {
				want++;
			}
		}
		if (want != subPath.size()) {
			continue;
		}
		if (occurrence == 0) {
			found = &f;
			break;
		}
		occurrence--;
	}
	if (found == nullptr) {
		error(usageText(fieldUsage) + " is not a Variable field of this report");
		return *this;
	}
	uint32_t el = element;
	if (!useElement) {
		el = 0xFFFFFFFFu;
		for (uint32_t e = 0; e < found->count; e++) {
			if (found->usageOf(e) == fieldUsage) {
				el = e;
				break;
			}
		}
	}
	if (el >= found->count) {
		error(usageText(fieldUsage) + ": no such element");
		return *this;
	}
	if (value < found->logicalMin || value > found->logicalMax) {
		std::ostringstream s;
		s << usageText(fieldUsage) << ": value " << value << " outside the logical range " << found->logicalMin << ".." << found->logicalMax;
		error(s.str());
		return *this;
	}
	// the value must also be representable in the report size (a logical range wider than the field, like Logical 1..6 in a
	// 1 bit field, cannot be sent: it would be truncated)
	if (found->bitSize > 32) {
		error(usageText(fieldUsage) + ": field larger than 32 bits");
		return *this;
	}
	const int64_t lo = found->isSigned ? -(1ll << (found->bitSize - 1)) : 0;
	const int64_t hi = found->isSigned ? (1ll << (found->bitSize - 1)) - 1 : (1ll << found->bitSize) - 1;
	if (value < lo || value > hi) {
		std::ostringstream s;
		s << usageText(fieldUsage) << ": value " << value << " is inside the logical range but does not fit the " << found->bitSize << " bit report size";
		error(s.str());
		return *this;
	}
	if (!hid::HidReportDescriptor::insert(*found, el, bytes_, report_->hasIdByte, value)) {
		error(usageText(fieldUsage) + ": cannot place the value (outside the report)");
	}
	return *this;
}

void PidReport::setCommandByte(uint8_t value) {
	if (report_ == nullptr || bytes_.size() != 2) {
		error("Device Control: the report is not ID + one byte");
		return;
	}
	bytes_[1] = value;
}

PidReport &PidReport::set(uint32_t fieldUsage, int64_t value, size_t occurrence) { return setImpl({}, fieldUsage, 0, false, value, occurrence); }
PidReport &PidReport::setIn(const std::vector<uint32_t> &subPath, uint32_t fieldUsage, int64_t value, size_t occurrence) {
	return setImpl(subPath, fieldUsage, 0, false, value, occurrence);
}
PidReport &PidReport::setElement(uint32_t fieldUsage, uint32_t element, int64_t value) { return setImpl({}, fieldUsage, element, true, value, 0); }

PidReport &PidReport::select(uint32_t selector) {
	if (report_ == nullptr) {
		error("the report is not declared by the descriptor (exactly once), cannot select " + usageText(selector));
		return *this;
	}
	for (size_t fi : report_->fields) {
		const hid::Field &f = desc_.fields()[fi];
		int64_t v = 0;
		if (f.isConstant() || f.isVariable() || !f.selectorValue(selector, v)) {
			continue;
		}
		if (v < f.logicalMin || v > f.logicalMax) {
			error(usageText(selector) + ": selector value outside the logical range of its array");
			return *this;
		}
		if (!hid::HidReportDescriptor::insert(f, 0, bytes_, report_->hasIdByte, v)) {
			error(usageText(selector) + ": cannot place the selector");
		}
		return *this;
	}
	error(usageText(selector) + " is not a selector of an array field of this report");
	return *this;
}

// ---------------------------------------------------------------------------------------------------------------------
const hid::Report *PidReportBuilder::find(hid::ReportType type, uint32_t reportUsage) const {
	const std::vector<const hid::Report *> r = desc_.findReportsByUsage(type, reportUsage);
	return r.size() == 1 ? r[0] : nullptr;
}

PidReport PidReportBuilder::make(hid::ReportType type, uint32_t reportUsage) const {
	const std::vector<const hid::Report *> r = desc_.findReportsByUsage(type, reportUsage);
	const std::string name = std::string(hid::reportTypeName(type)) + " " + usageText(reportUsage);
	if (r.size() != 1) {
		PidReport bad(desc_, nullptr, name, failTests_);
		const std::string msg = "PidReportBuilder: " + name + " declared " + std::to_string(r.size()) + " times by the descriptor (need exactly 1)";
		if (failTests_) {
			FAIL_CHECK(msg);
		}
		return bad;
	}
	return PidReport(desc_, r[0], name, failTests_);
}

PidReport PidReportBuilder::make(hid::ReportType type, const std::string &reportUsageName) const {
	const uint32_t usage = usageByName(reportUsageName);
	if (usage == 0xFFFFFFFFu) {
		PidReport bad(desc_, nullptr, reportUsageName, failTests_);
		if (failTests_) {
			const std::string msg = "PidReportBuilder: unknown PID usage name '" + reportUsageName + "' (USB_PID1_01.pdf Table 2)";
			FAIL_CHECK(msg);
		}
		return bad;
	}
	return make(type, usage);
}

PidReport PidReportBuilder::deviceControl(uint32_t command) const {
	PidReport rep = make(hid::ReportType::Output, use::PidDeviceControlReport);
	if (!rep.valid()) {
		return rep;
	}
	const hid::Field *f = desc_.findField(hid::ReportType::Output, use::PidDeviceControlReport, command);
	int64_t sel = 0;
	if (f == nullptr || !f->selectorValue(command, sel)) {
		rep.error(usageText(command) + " is not a command of the Device Control report");
		return rep;
	}
	if (f->bitOffset != 0 || f->totalBits() != 8 || rep.length() != 2) {
		rep.error("Device Control field is not one byte at bit 0 of a 2-byte report");
		return rep;
	}
	if (sel < f->logicalMin || sel > f->logicalMax || sel > 255) {
		rep.error(usageText(command) + ": command number outside the logical range");
		return rep;
	}
	rep.setCommandByte(static_cast<uint8_t>(sel));
	return rep;
}

DecodedReport PidReportBuilder::decode(hid::ReportType type, uint32_t reportUsage, const std::vector<uint8_t> &bytes) const {
	const hid::Report *r = find(type, reportUsage);
	if (r == nullptr) {
		DecodedReport d;
		d.errors.push_back(usageText(reportUsage) + " is not declared exactly once by the descriptor");
		return d;
	}
	return decode(*r, bytes);
}

DecodedReport PidReportBuilder::decode(const hid::Report &report, const std::vector<uint8_t> &bytes) const {
	DecodedReport d;
	if (bytes.size() != report.byteLength()) {
		d.errors.push_back("report length " + std::to_string(bytes.size()) + " instead of " + std::to_string(report.byteLength()));
		return d;
	}
	if (report.hasIdByte && bytes[0] != report.reportId) {
		d.errors.push_back("report ID byte " + std::to_string(bytes[0]) + " instead of " + std::to_string(report.reportId));
		return d;
	}
	for (size_t fi : report.fields) {
		const hid::Field &f = desc_.fields()[fi];
		if (f.isConstant()) {
			continue;
		}
		for (uint32_t e = 0; e < f.count; e++) {
			DecodedValue v;
			v.field = &f;
			v.element = e;
			v.usage = f.usageOf(e);
			if (!hid::HidReportDescriptor::extract(f, e, bytes, report.hasIdByte, v.value)) {
				d.errors.push_back("cannot extract an element (field larger than 32 bits)");
				continue;
			}
			d.values.push_back(v);
		}
	}
	return d;
}

} // namespace pid
