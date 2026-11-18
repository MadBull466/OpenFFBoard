/*
 * HidReportDescriptor.h
 *
 * Parser of USB HID report descriptors (HID 1.11, Tests/Code_Coverage/shared/specs/hid1_11.pdf), host side of the integration
 * tests. Written from the specification only: it knows nothing of the firmware (no firmware header is included, the
 * descriptor bytes the device sends over USB are the only input).
 *
 * What it models (HID 1.11 section numbers):
 *  - 6.2.2.2 items: short items (prefix = tag(4) type(2) size(2); data size code 0/1/2/3 = 0/1/2/4 bytes), long items
 *    (prefix 0xFE, bDataSize, bLongItemTag: recognised, counted and skipped);
 *  - 6.2.2.4 main items: Input / Output / Feature (flag bits of the data byte), Collection / End Collection;
 *  - 6.2.2.7 global items: Usage Page, Logical Min / Max, Physical Min / Max, Unit Exponent, Unit, Report Size, Report ID,
 *    Report Count, Push / Pop (the whole global state, including Report ID, is pushed and popped);
 *  - 6.2.2.8 local items: Usage, Usage Minimum / Maximum, extended usages (4 data bytes = full 32 bit usage, otherwise the
 *    Usage Page current at the MAIN item is the high half: "When the parser encounters a main item it concatenates the last
 *    declared Usage Page with a Usage"), Designator and String items (parsed, ignored), local state cleared after every main
 *    item. Delimiter (tag 0xA) is rejected as unsupported;
 *  - Logical extents (an INTERPRETATION, HID 1.11 does not state one rule for all cases): 6.2.2.2 allows a data item to be
 *    abbreviated "if all of its high-order bits are zero", and the spec's own examples use sign-extended small items (E.10 / D.1:
 *    "15 81" = -127). Hence: item data are sign-extended from their size for the Logical Minimum; if that minimum is >= 0 the field
 *    is unsigned and the Logical Maximum is taken as the unsigned number of the item's size ("15 00 25 FF" and "15 00 26 FF 00"
 *    both give 0..255); if the minimum is negative the field is two's complement and the maximum is sign-extended too. A
 *    negative minimum whose range does not fit the signed range of Report Size ("15 80 26 FF 00" with size 8) is an error. An
 *    unsigned range wider than Report Size is NOT an error (the PID Device Control field of the PID 1.01 section 6 example
 *    has Logical 1..6 in 1 bit elements);
 *  - 6.2.2.7 Physical Min / Max: undefined until declared; (0, 0) or undefined means "equal to the logical extents"
 *    (physMin / physMax of Field hold the effective values, hasPhysical tells if they were declared);
 *  - 8.1 report layout: fields of one (type, report id) follow each other in declaration order; the bit offset is counted after
 *    the report ID byte; the report length is [1 byte if report IDs are used] + ceil(bits / 8). If no Report ID item occurs
 *    anywhere, the single report of each type has id 0 and no ID byte.
 *
 * Malformed descriptors give a parse error (never a guess): data past the end, Collection nesting unbalanced, End Collection
 * without Collection, Pop without Push, Report ID 0 (reserved, 6.2.2.7) or above 255, data item with Report Size 0 and Report Count > 0,
 * a Report ID reused by the collection of another report (6.2.2.7: a Report ID prefix names one report), an unfinished usage range (Minimum without Maximum), Usage Minimum > Maximum, data item without
 * Logical Minimum / Maximum (6.2.2.7: a Variable or Array item must have extents; constant padding items need none), data
 * item with Logical Minimum > Maximum after the sign rule, unknown (reserved) item tags, delimiters.
 */
#ifndef HOSTTEST_INTEGRATION_HIDREPORTDESCRIPTOR_H_
#define HOSTTEST_INTEGRATION_HIDREPORTDESCRIPTOR_H_

#include <cstdint>
#include <string>
#include <vector>

namespace hid {

enum class ReportType { Input, Output, Feature };
const char *reportTypeName(ReportType t);

/** Extended usage: usage page in the high 16 bits, usage ID in the low 16 bits (HID 1.11 6.2.2.8) */
constexpr uint32_t usage(uint16_t page, uint16_t id) { return (static_cast<uint32_t>(page) << 16) | id; }

/** Collection types (HID 1.11 6.2.2.4, Collection item data) */
enum CollectionType : uint32_t {
	CollPhysical = 0,
	CollApplication = 1,
	CollLogical = 2,
	CollReport = 3,
	CollNamedArray = 4,
	CollUsageSwitch = 5,
	CollUsageModifier = 6
};

/** One enclosing collection */
struct CollectionRef {
	uint32_t type = 0;
	uint32_t usage = 0; //!< extended usage of the Collection item (0 if it had none)
};

/** Flag bits of an Input / Output / Feature item (HID 1.11 6.2.2.5) */
enum FieldFlags : uint32_t {
	FlagConstant = 1u << 0,		 //!< 0 Data, 1 Constant
	FlagVariable = 1u << 1,		 //!< 0 Array, 1 Variable
	FlagRelative = 1u << 2,		 //!< 0 Absolute, 1 Relative
	FlagWrap = 1u << 3,			 //!< 1 Wrap
	FlagNonLinear = 1u << 4,	 //!< 1 Non Linear
	FlagNoPreferred = 1u << 5,	 //!< 1 No Preferred state
	FlagNullState = 1u << 6,	 //!< 1 Null state
	FlagVolatile = 1u << 7,		 //!< 1 Volatile (Output / Feature)
	FlagBufferedBytes = 1u << 8, //!< 1 Buffered bytes
};

/** One Input / Output / Feature main item: `count` elements of `bitSize` bits each */
struct Field {
	ReportType type = ReportType::Input;
	uint8_t reportId = 0;
	uint32_t flags = 0;

	uint32_t bitOffset = 0; //!< of the first element, counted after the report ID byte
	uint32_t bitSize = 0;	//!< Report Size: bits per element
	uint32_t count = 0;		//!< Report Count: number of elements
	uint32_t totalBits() const { return bitSize * count; }

	int64_t logicalMin = 0, logicalMax = 0; //!< after the sign rule
	bool isSigned = false;					//!< logicalMin < 0: two's complement in the report
	bool hasLogical = false;				//!< the descriptor declared both extents
	bool hasPhysical = false;				//!< Physical Min / Max declared (and not both 0)
	int64_t physicalMin = 0, physicalMax = 0; //!< effective values (equal to the logical extents if undeclared)
	int32_t unitExponent = 0;				//!< 0 if undeclared
	uint32_t unit = 0;

	/** Usage list of the item (extended usages, in declaration order); empty if declared by a range or not at all */
	std::vector<uint32_t> usages;
	bool hasRange = false;
	uint32_t usageMin = 0, usageMax = 0; //!< extended, valid if hasRange

	/** Enclosing collections, outermost first */
	std::vector<CollectionRef> path;

	bool isConstant() const { return (flags & FlagConstant) != 0; }
	bool isVariable() const { return (flags & FlagVariable) != 0; }
	bool isArray() const { return (flags & FlagVariable) == 0 && !isConstant(); }

	/**
	 * Usage of element i (HID 1.11 6.2.2.8: usages are assigned in order; if there are more elements than usages, the last
	 * usage applies to the remaining ones). 0 for constant items and items without usage.
	 */
	uint32_t usageOf(uint32_t element) const;
	/** True if `u` is one of the usages the item declares (list or range) */
	bool hasUsage(uint32_t u) const;
	/**
	 * Array / named-array selector: the value that selects usage `u` (logicalMin + position of `u` in the declaration),
	 * HID 1.11 6.2.2.5 (array items report the index of the control). false if `u` is not declared
	 */
	bool selectorValue(uint32_t u, int64_t &value) const;
};

/** One report (type, id) with its fields in declaration order */
struct Report {
	ReportType type = ReportType::Input;
	uint8_t reportId = 0;
	bool hasIdByte = false;
	uint32_t bitLength = 0;		 //!< sum of the fields, without the ID byte
	uint32_t byteLength() const; //!< ID byte (if report IDs are used) + ceil(bitLength / 8)
	std::vector<size_t> fields;	 //!< indices into HidReportDescriptor::fields()
};

class HidReportDescriptor {
public:
	/** Parses the bytes. @return true on success; on failure error() holds the reason (with the byte offset) and the result is cleared */
	bool parse(const std::vector<uint8_t> &bytes);
	bool parse(const uint8_t *data, size_t length);
	const std::string &error() const { return error_; }

	const std::vector<Field> &fields() const { return fields_; }
	const std::vector<Report> &reports() const { return reports_; }
	/** Number of long items skipped */
	size_t longItems() const { return longItems_; }
	/** True if any Report ID item occurred (reports carry an ID byte) */
	bool usesReportIds() const { return usesIds_; }

	/** The report of that type and id, nullptr if none */
	const Report *findReport(ReportType type, uint8_t reportId) const;
	/**
	 * The reports of a type whose fields lie inside a collection with usage `collectionUsage` (any depth). A PID report is
	 * identified like that: Set Effect Report = Logical collection with usage 0x0F:0x21 around the report's fields.
	 */
	std::vector<const Report *> findReportsByUsage(ReportType type, uint32_t collectionUsage) const;
	/**
	 * The first field of the reports found by findReportsByUsage(type, reportUsage) that declares `fieldUsage`, in declaration
	 * order. `subPath`, if given, must appear in the field's collection path as a subsequence (e.g. {Effect Type 0x25} selects
	 * the usages inside that named array); `occurrence` skips that many earlier matches. nullptr if none.
	 */
	const Field *findField(ReportType type, uint32_t reportUsage, uint32_t fieldUsage, const std::vector<uint32_t> &subPath = {},
						   size_t occurrence = 0) const;
	/** The report that contains `f` */
	const Report *reportOf(const Field &f) const { return findReport(f.type, f.reportId); }

	/** Reads the value of element `element` of field `f` from a report buffer (with ID byte if the descriptor uses IDs). false if out of range */
	static bool extract(const Field &f, uint32_t element, const std::vector<uint8_t> &report, bool hasIdByte, int64_t &value);
	/** Writes a value of element `element` into a report buffer (little-endian, HID 1.11 5.8). false if the buffer is too short or the element does not exist. Does not check the range */
	static bool insert(const Field &f, uint32_t element, std::vector<uint8_t> &report, bool hasIdByte, int64_t value);

private:
	void clear();
	bool fail(size_t offset, const std::string &what);

	std::vector<Field> fields_;
	std::vector<Report> reports_;
	size_t longItems_ = 0;
	bool usesIds_ = false;
	std::string error_;
};

} // namespace hid

#endif
