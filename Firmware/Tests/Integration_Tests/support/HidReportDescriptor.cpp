/*
 * HidReportDescriptor.cpp: see the header for the model and the HID 1.11 sections.
 */
#include "HidReportDescriptor.h"

#include <algorithm>
#include <cstdio>

namespace hid {

const char *reportTypeName(ReportType t) {
	switch (t) {
	case ReportType::Input:
		return "Input";
	case ReportType::Output:
		return "Output";
	case ReportType::Feature:
		return "Feature";
	}
	return "?";
}

uint32_t Field::usageOf(uint32_t element) const {
	if (isConstant()) {
		return 0;
	}
	if (hasRange) {
		const uint64_t u = static_cast<uint64_t>(usageMin) + element;
		return u > usageMax ? usageMax : static_cast<uint32_t>(u);
	}
	if (usages.empty()) {
		return 0;
	}
	return element < usages.size() ? usages[element] : usages.back();
}

bool Field::hasUsage(uint32_t u) const {
	if (hasRange) {
		return u >= usageMin && u <= usageMax;
	}
	return std::find(usages.begin(), usages.end(), u) != usages.end();
}

bool Field::selectorValue(uint32_t u, int64_t &value) const {
	if (hasRange) {
		if (u < usageMin || u > usageMax) {
			return false;
		}
		value = logicalMin + static_cast<int64_t>(u - usageMin);
		return true;
	}
	for (size_t i = 0; i < usages.size(); i++) {
		if (usages[i] == u) {
			value = logicalMin + static_cast<int64_t>(i);
			return true;
		}
	}
	return false;
}

uint32_t Report::byteLength() const { return (hasIdByte ? 1u : 0u) + (bitLength + 7) / 8; }

namespace {

struct Global {
	uint32_t usagePage = 0;
	bool haveLogMin = false, haveLogMax = false;
	int64_t logMinS = 0, logMaxS = 0, logMaxU = 0;
	bool havePhysMin = false, havePhysMax = false;
	int64_t physMinS = 0, physMaxS = 0, physMaxU = 0;
	int32_t unitExp = 0;
	uint32_t unit = 0;
	uint32_t size = 0, count = 0;
	uint32_t id = 0;
	bool haveId = false;
	uint32_t idScopeUsage = 0; //!< usage of the innermost collection open at the Report ID item
};

// A local Usage item as declared: `ext` = 4 data bytes (a complete 32 bit usage), otherwise a Usage ID that gets the Usage Page
// that is current when the MAIN item arrives (HID 1.11 6.2.2.8: "When the parser encounters a main item it concatenates the last
// declared Usage Page with a Usage")
struct LUsage {
	uint32_t v = 0;
	bool ext = false;
};

struct Local {
	std::vector<LUsage> usages;
	bool haveMin = false, haveMax = false;
	LUsage usageMin, usageMax;
	void clear() { *this = Local(); }
};

int64_t signExtend(uint32_t raw, unsigned size) {
	switch (size) {
	case 1:
		return static_cast<int8_t>(raw);
	case 2:
		return static_cast<int16_t>(raw);
	case 4:
		return static_cast<int32_t>(raw);
	default:
		return 0;
	}
}

} // namespace

void HidReportDescriptor::clear() {
	fields_.clear();
	reports_.clear();
	longItems_ = 0;
	usesIds_ = false;
}

bool HidReportDescriptor::fail(size_t offset, const std::string &what) {
	clear();
	error_ = "HID report descriptor, byte offset " + std::to_string(offset) + ": " + what;
	return false;
}

bool HidReportDescriptor::parse(const std::vector<uint8_t> &bytes) { return parse(bytes.data(), bytes.size()); }

bool HidReportDescriptor::parse(const uint8_t *d, size_t n) {
	clear();
	error_.clear();
	Global g;
	std::vector<Global> stack;
	Local l;
	std::vector<CollectionRef> path;
	std::vector<Field> fields;
	// fields per (type, id), the running bit length
	struct Key {
		ReportType type;
		uint8_t id;
		uint32_t bits;
		uint32_t scopeUsage; //!< usage of the innermost collection that was open where the Report ID item stood (0 at top level)
	};
	std::vector<Key> keys;
	bool usesIds = false;

	// HID 1.11 6.2.2.8: the Usage Page current at the main item is concatenated with a 16 bit Usage
	auto resolve = [&g](const LUsage &u) { return u.ext ? u.v : usage(static_cast<uint16_t>(g.usagePage), static_cast<uint16_t>(u.v)); };

	size_t i = 0;
	while (i < n) {
		const size_t at = i;
		const uint8_t prefix = d[i++];
		if (prefix == 0xFE) { // long item (6.2.2.3): 0xFE, bDataSize, bLongItemTag, data
			if (i + 2 > n) {
				return fail(at, "long item prefix without its size and tag bytes (data past the end)");
			}
			const uint8_t size = d[i];
			i += 2;
			if (i + size > n) {
				return fail(at, "long item data past the end");
			}
			i += size;
			longItems_++;
			continue;
		}
		const unsigned sizeCode = prefix & 3u;
		const unsigned size = sizeCode == 3 ? 4 : sizeCode;
		const unsigned type = (prefix >> 2) & 3u;
		const unsigned tag = prefix >> 4;
		if (i + size > n) {
			return fail(at, "item data past the end");
		}
		uint32_t raw = 0;
		for (unsigned b = 0; b < size; b++) {
			raw |= static_cast<uint32_t>(d[i + b]) << (8 * b);
		}
		i += size;
		const int64_t sraw = size ? signExtend(raw, size) : 0;

		if (type == 1) { // global
			switch (tag) {
			case 0x0:
				g.usagePage = raw;
				if (raw > 0xFFFF) {
					return fail(at, "Usage Page above 0xFFFF");
				}
				break;
			case 0x1:
				g.haveLogMin = true;
				g.logMinS = sraw;
				break;
			case 0x2:
				g.haveLogMax = true;
				g.logMaxS = sraw;
				g.logMaxU = raw;
				break;
			case 0x3:
				g.havePhysMin = true;
				g.physMinS = sraw;
				break;
			case 0x4:
				g.havePhysMax = true;
				g.physMaxS = sraw;
				g.physMaxU = raw;
				break;
			case 0x5: { // Unit Exponent: 4 bit two's complement nibble (table 6.2.2.7)
				const uint32_t nib = raw & 0xF;
				g.unitExp = nib > 7 ? static_cast<int32_t>(nib) - 16 : static_cast<int32_t>(nib);
				break;
			}
			case 0x6:
				g.unit = raw;
				break;
			case 0x7:
				g.size = raw;
				break;
			case 0x8:
				if (raw == 0) {
					return fail(at, "Report ID 0 (reserved, HID 1.11 6.2.2.7)");
				}
				if (raw > 255) {
					return fail(at, "Report ID above 255");
				}
				g.id = raw;
				g.haveId = true;
				g.idScopeUsage = path.empty() ? 0 : path.back().usage;
				usesIds = true;
				break;
			case 0x9:
				g.count = raw;
				break;
			case 0xA:
				stack.push_back(g);
				break;
			case 0xB:
				if (stack.empty()) {
					return fail(at, "Pop without Push");
				}
				g = stack.back();
				stack.pop_back();
				break;
			default:
				return fail(at, "reserved global item tag " + std::to_string(tag));
			}
			continue;
		}

		if (type == 2) { // local
			switch (tag) {
			case 0x0: // Usage
				l.usages.push_back({raw, size == 4});
				break;
			case 0x1:
				l.haveMin = true;
				l.usageMin = {raw, size == 4};
				break;
			case 0x2:
				l.haveMax = true;
				l.usageMax = {raw, size == 4};
				break;
			case 0x3: // Designator Index / Minimum / Maximum, String Index / Minimum / Maximum: parsed, ignored
			case 0x4:
			case 0x5:
			case 0x7:
			case 0x8:
			case 0x9:
				break;
			case 0xA:
				return fail(at, "Delimiter item (local tag 0xA) is not supported");
			default:
				return fail(at, "reserved local item tag " + std::to_string(tag));
			}
			continue;
		}

		if (type == 3) {
			return fail(at, "reserved item type 3");
		}

		// main item
		if (tag == 0xA) { // Collection
			CollectionRef c;
			c.type = raw;
			c.usage = !l.usages.empty() ? resolve(l.usages.front()) : (l.haveMin ? resolve(l.usageMin) : 0);
			path.push_back(c);
			l.clear();
			continue;
		}
		if (tag == 0xC) { // End Collection
			if (path.empty()) {
				return fail(at, "End Collection without Collection");
			}
			path.pop_back();
			l.clear();
			continue;
		}
		if (tag != 0x8 && tag != 0x9 && tag != 0xB) {
			return fail(at, "reserved main item tag " + std::to_string(tag));
		}

		Field f;
		f.type = tag == 0x8 ? ReportType::Input : tag == 0x9 ? ReportType::Output : ReportType::Feature;
		f.flags = raw;
		f.reportId = g.haveId ? static_cast<uint8_t>(g.id) : 0;
		f.bitSize = g.size;
		f.count = g.count;
		if (g.size == 0 && g.count > 0) {
			return fail(at, "data item with Report Size 0 and Report Count " + std::to_string(g.count));
		}
		if (l.haveMin != l.haveMax) {
			return fail(at, "Usage Minimum and Usage Maximum must come as a pair");
		}
		if (l.haveMin) {
			if (!l.usages.empty()) {
				return fail(at, "usage list and usage range on the same item are not supported");
			}
			if (resolve(l.usageMin) > resolve(l.usageMax)) {
				return fail(at, "Usage Minimum above Usage Maximum");
			}
			f.hasRange = true;
			f.usageMin = resolve(l.usageMin);
			f.usageMax = resolve(l.usageMax);
		} else {
			for (const LUsage &u : l.usages) {
				f.usages.push_back(resolve(u));
			}
		}
		if (!f.isConstant()) {
			if (!g.haveLogMin || !g.haveLogMax) {
				return fail(at, "data item without Logical Minimum / Maximum");
			}
			f.hasLogical = true;
			f.logicalMin = g.logMinS;
			f.isSigned = g.logMinS < 0;
			f.logicalMax = f.isSigned ? g.logMaxS : g.logMaxU;
			if (f.logicalMin > f.logicalMax) {
				return fail(at, "Logical Minimum above Logical Maximum");
			}
			// a negative minimum makes the field two's complement: both extents must fit the signed range of Report Size
			if (f.isSigned && f.bitSize > 0 && f.bitSize <= 32 &&
				(f.logicalMin < -(1ll << (f.bitSize - 1)) || f.logicalMax > (1ll << (f.bitSize - 1)) - 1)) {
				return fail(at, "signed Logical range " + std::to_string(f.logicalMin) + ".." + std::to_string(f.logicalMax) +
									" does not fit Report Size " + std::to_string(f.bitSize));
			}
		} else if (g.haveLogMin && g.haveLogMax) {
			f.hasLogical = true;
			f.logicalMin = g.logMinS;
			f.isSigned = g.logMinS < 0;
			f.logicalMax = f.isSigned ? g.logMaxS : g.logMaxU;
		}
		f.unitExponent = g.unitExp;
		f.unit = g.unit;
		if (g.havePhysMin && g.havePhysMax && !(g.physMinS == 0 && g.physMaxS == 0 && g.physMaxU == 0)) {
			f.hasPhysical = true;
			f.physicalMin = g.physMinS;
			f.physicalMax = g.physMinS < 0 ? g.physMaxS : g.physMaxU;
		} else {
			f.physicalMin = f.logicalMin;
			f.physicalMax = f.logicalMax;
		}
		f.path = path;

		Key *k = nullptr;
		for (Key &c : keys) {
			if (c.type == f.type && c.id == f.reportId) {
				k = &c;
			}
		}
		if (k == nullptr) {
			keys.push_back({f.type, f.reportId, 0, g.idScopeUsage});
			k = &keys.back();
		} else if (g.haveId && k->scopeUsage != g.idScopeUsage) {
			// HID 1.11 6.2.2.7: a Report ID prefix names ONE report ("each with a unique ID"); the same ID in the collection of
			// another report would silently merge two reports
			return fail(at, std::string(reportTypeName(f.type)) + " Report ID " + std::to_string(f.reportId) + " is used by two report collections (usage 0x" +
								[](uint32_t u) { char b[16]; std::snprintf(b, sizeof(b), "%08X", static_cast<unsigned>(u)); return std::string(b); }(k->scopeUsage) +
								" and 0x" + [](uint32_t u) { char b[16]; std::snprintf(b, sizeof(b), "%08X", static_cast<unsigned>(u)); return std::string(b); }(g.idScopeUsage) + ")");
		}
		f.bitOffset = k->bits;
		k->bits += f.bitSize * f.count;
		fields.push_back(f);
		l.clear();
	}

	if (!path.empty()) {
		return fail(n, std::to_string(path.size()) + " Collection(s) not closed at the end of the descriptor");
	}
	for (const Field &f : fields) {
		if (usesIds && f.reportId == 0) {
			return fail(n, "a data item before the first Report ID item although report IDs are used (HID 1.11 6.2.2.7)");
		}
	}

	fields_ = fields;
	usesIds_ = usesIds;
	for (const Key &k : keys) {
		Report r;
		r.type = k.type;
		r.reportId = k.id;
		r.hasIdByte = usesIds;
		r.bitLength = k.bits;
		for (size_t fi = 0; fi < fields_.size(); fi++) {
			if (fields_[fi].type == k.type && fields_[fi].reportId == k.id) {
				r.fields.push_back(fi);
			}
		}
		reports_.push_back(r);
	}
	return true;
}

const Report *HidReportDescriptor::findReport(ReportType type, uint8_t reportId) const {
	for (const Report &r : reports_) {
		if (r.type == type && r.reportId == reportId) {
			return &r;
		}
	}
	return nullptr;
}

std::vector<const Report *> HidReportDescriptor::findReportsByUsage(ReportType type, uint32_t collectionUsage) const {
	std::vector<const Report *> out;
	for (const Report &r : reports_) {
		if (r.type != type) {
			continue;
		}
		bool found = false;
		for (size_t fi : r.fields) {
			for (const CollectionRef &c : fields_[fi].path) {
				if (c.usage == collectionUsage) {
					found = true;
				}
			}
		}
		if (found) {
			out.push_back(&r);
		}
	}
	return out;
}

const Field *HidReportDescriptor::findField(ReportType type, uint32_t reportUsage, uint32_t fieldUsage, const std::vector<uint32_t> &subPath,
											size_t occurrence) const {
	for (const Report *r : findReportsByUsage(type, reportUsage)) {
		for (size_t fi : r->fields) {
			const Field &f = fields_[fi];
			if (f.isConstant() || !f.hasUsage(fieldUsage)) {
				continue;
			}
			// the report usage must enclose the field
			bool enclosed = false;
			for (const CollectionRef &c : f.path) {
				enclosed = enclosed || c.usage == reportUsage;
			}
			if (!enclosed) {
				continue;
			}
			size_t want = 0;
			for (const CollectionRef &c : f.path) {
				if (want < subPath.size() && c.usage == subPath[want]) {
					want++;
				}
			}
			if (want != subPath.size()) {
				continue;
			}
			if (occurrence == 0) {
				return &f;
			}
			occurrence--;
		}
	}
	return nullptr;
}

bool HidReportDescriptor::extract(const Field &f, uint32_t element, const std::vector<uint8_t> &report, bool hasIdByte, int64_t &value) {
	if (element >= f.count || f.bitSize == 0 || f.bitSize > 32) {
		return false;
	}
	const uint64_t start = (hasIdByte ? 8u : 0u) + f.bitOffset + static_cast<uint64_t>(element) * f.bitSize;
	if ((start + f.bitSize + 7) / 8 > report.size()) {
		return false;
	}
	uint64_t v = 0;
	for (uint32_t b = 0; b < f.bitSize; b++) {
		const uint64_t bit = start + b;
		if (report[bit / 8] & (1u << (bit % 8))) {
			v |= (1ull << b);
		}
	}
	if (f.isSigned && (v & (1ull << (f.bitSize - 1)))) {
		value = static_cast<int64_t>(v) - (1ll << f.bitSize);
	} else {
		value = static_cast<int64_t>(v);
	}
	return true;
}

bool HidReportDescriptor::insert(const Field &f, uint32_t element, std::vector<uint8_t> &report, bool hasIdByte, int64_t value) {
	if (element >= f.count || f.bitSize == 0 || f.bitSize > 32) {
		return false;
	}
	const uint64_t start = (hasIdByte ? 8u : 0u) + f.bitOffset + static_cast<uint64_t>(element) * f.bitSize;
	if ((start + f.bitSize + 7) / 8 > report.size()) {
		return false;
	}
	const uint64_t v = static_cast<uint64_t>(value);
	for (uint32_t b = 0; b < f.bitSize; b++) {
		const uint64_t bit = start + b;
		const uint8_t mask = static_cast<uint8_t>(1u << (bit % 8));
		if (v & (1ull << b)) {
			report[bit / 8] |= mask;
		} else {
			report[bit / 8] &= static_cast<uint8_t>(~mask);
		}
	}
	return true;
}

} // namespace hid
