/*
 * PidReportBuilder.h
 *
 * Builds and decodes USB PID (Physical Interface Device) reports from a PARSED HID report descriptor
 * (support/HidReportDescriptor.h), host side of the integration tests. It does not use the firmware's C structs or headers:
 * the layout of every report (offsets, sizes, logical ranges, the report ID) is read from the descriptor the device sends over
 * USB, so a mismatch between the firmware's structs and its descriptor shows up as a failing test instead of being repeated
 * on both sides (plan section 3.3, last paragraph).
 *
 * Oracle: Tests/Code_Coverage/shared/specs/USB_PID1_01.pdf (Device Class Definition for Physical Interface Devices, file
 * USB_PID1_01.pdf), Table 2 "Physical Input Device Page" (pages 6 and 7): usage page 0x0F and its usage IDs, and sections 5.1 to
 * 5.13 for the meaning of the reports. HID 1.11 section 5.8: values are little-endian, LSB of a value in the lowest bit.
 *
 * Strictness (a failing test, never a silent fix-up): a value outside the field's logical range (not clamped), an unknown usage
 * name, a usage that is not in the report, a report usage that the descriptor does not declare (or declares twice), a value
 * for an array field's selector that the field cannot select. Every such event is recorded in errors() and, unless the object was
 * created with failTests = false (the self-tests of this file do that), reported with FAIL_CHECK.
 */
#ifndef HOSTTEST_INTEGRATION_PIDREPORTBUILDER_H_
#define HOSTTEST_INTEGRATION_PIDREPORTBUILDER_H_

#include "HidReportDescriptor.h"

#include <cstdint>
#include <string>
#include <vector>

namespace pid {

constexpr uint16_t kPage = 0x0F; //!< Physical Interface Device usage page (USB_PID1_01.pdf Table 2; page number 0x0F of the HID Usage Tables)

/** Extended usage on the PID page */
constexpr uint32_t u(uint16_t id) { return hid::usage(kPage, id); }

/**
 * Usage IDs of the PID page, USB_PID1_01.pdf Table 2 "Physical Input Device Page" (pages 6-7). Names as in the table. Only
 * the usages with an ID; "Reserved" rows are not here. The names of the CA / CL collections (reports) end in "Report".
 */
namespace use {
// report collections (CL)
constexpr uint32_t PhysicalInterfaceDevice = u(0x01); //!< Table 2 p. 6, CA
constexpr uint32_t Normal = u(0x20);
constexpr uint32_t SetEffectReport = u(0x21);		   //!< Table 2 p. 6; section 5.1
constexpr uint32_t EffectBlockIndex = u(0x22);		   //!< 5.1
constexpr uint32_t ParameterBlockOffset = u(0x23);	   //!< 5.1
constexpr uint32_t RomFlag = u(0x24);				   //!< 5.1
constexpr uint32_t EffectType = u(0x25);			   //!< 5.1.1, NAry
constexpr uint32_t EtConstantForce = u(0x26);		   //!< 5.1.1
constexpr uint32_t EtRamp = u(0x27);
constexpr uint32_t EtCustomForceData = u(0x28);
constexpr uint32_t EtSquare = u(0x30);
constexpr uint32_t EtSine = u(0x31);
constexpr uint32_t EtTriangle = u(0x32);
constexpr uint32_t EtSawtoothUp = u(0x33);
constexpr uint32_t EtSawtoothDown = u(0x34);
constexpr uint32_t EtSpring = u(0x40);
constexpr uint32_t EtDamper = u(0x41);
constexpr uint32_t EtInertia = u(0x42);
constexpr uint32_t EtFriction = u(0x43); //!< Table 2 p. 7
constexpr uint32_t Duration = u(0x50);
constexpr uint32_t SamplePeriod = u(0x51);
constexpr uint32_t Gain = u(0x52);
constexpr uint32_t TriggerButton = u(0x53);
constexpr uint32_t TriggerRepeatInterval = u(0x54);
constexpr uint32_t AxesEnable = u(0x55);
constexpr uint32_t DirectionEnable = u(0x56);
constexpr uint32_t Direction = u(0x57);
constexpr uint32_t TypeSpecificBlockOffset = u(0x58); //!< 5.1.1.1
constexpr uint32_t BlockType = u(0x59);
constexpr uint32_t SetEnvelopeReport = u(0x5A); //!< 5.2
constexpr uint32_t AttackLevel = u(0x5B);
constexpr uint32_t AttackTime = u(0x5C);
constexpr uint32_t FadeLevel = u(0x5D);
constexpr uint32_t FadeTime = u(0x5E);
constexpr uint32_t SetConditionReport = u(0x5F); //!< 5.3
constexpr uint32_t CpOffset = u(0x60);
constexpr uint32_t PositiveCoefficient = u(0x61);
constexpr uint32_t NegativeCoefficient = u(0x62);
constexpr uint32_t PositiveSaturation = u(0x63);
constexpr uint32_t NegativeSaturation = u(0x64);
constexpr uint32_t DeadBand = u(0x65);
constexpr uint32_t DownloadForceSample = u(0x66); //!< 5.4
constexpr uint32_t IsochCustomForceEnable = u(0x67);
constexpr uint32_t CustomForceDataReport = u(0x68);
constexpr uint32_t CustomForceData = u(0x69);
constexpr uint32_t CustomForceVendorDefinedData = u(0x6A);
constexpr uint32_t SetCustomForceReport = u(0x6B);
constexpr uint32_t CustomForceDataOffset = u(0x6C);
constexpr uint32_t SampleCount = u(0x6D);
constexpr uint32_t SetPeriodicReport = u(0x6E); //!< 5.5
constexpr uint32_t Offset = u(0x6F);
constexpr uint32_t Magnitude = u(0x70);
constexpr uint32_t Phase = u(0x71);
constexpr uint32_t Period = u(0x72);
constexpr uint32_t SetConstantForceReport = u(0x73); //!< 5.6
constexpr uint32_t SetRampForceReport = u(0x74);	 //!< 5.7
constexpr uint32_t RampStart = u(0x75);
constexpr uint32_t RampEnd = u(0x76);
constexpr uint32_t EffectOperationReport = u(0x77); //!< 5.9
constexpr uint32_t EffectOperation = u(0x78);
constexpr uint32_t OpEffectStart = u(0x79);
constexpr uint32_t OpEffectStartSolo = u(0x7A);
constexpr uint32_t OpEffectStop = u(0x7B);
constexpr uint32_t LoopCount = u(0x7C);
constexpr uint32_t DeviceGainReport = u(0x7D); //!< 5.10
constexpr uint32_t DeviceGain = u(0x7E);
constexpr uint32_t PidPoolReport = u(0x7F); //!< 5.11.1
constexpr uint32_t RamPoolSize = u(0x80);
constexpr uint32_t RomPoolSize = u(0x81);
constexpr uint32_t RomEffectBlockCount = u(0x82);
constexpr uint32_t SimultaneousEffectsMax = u(0x83);
constexpr uint32_t PoolAlignment = u(0x84);
constexpr uint32_t PidPoolMoveReport = u(0x85);
constexpr uint32_t MoveSource = u(0x86);
constexpr uint32_t MoveDestination = u(0x87);
constexpr uint32_t MoveLength = u(0x88);			  //!< Table 2 p. 7
constexpr uint32_t PidBlockLoadReport = u(0x89);	  //!< 5.11.2
constexpr uint32_t BlockLoadStatus = u(0x8B);
constexpr uint32_t BlockLoadSuccess = u(0x8C);
constexpr uint32_t BlockLoadFull = u(0x8D);
constexpr uint32_t BlockLoadError = u(0x8E);
constexpr uint32_t BlockHandle = u(0x8F);
constexpr uint32_t PidBlockFreeReport = u(0x90); //!< 5.11.2
constexpr uint32_t TypeSpecificBlockHandle = u(0x91);
constexpr uint32_t PidStateReport = u(0x92); //!< 5.12
constexpr uint32_t EffectPlaying = u(0x94);
constexpr uint32_t PidDeviceControlReport = u(0x95); //!< 5.13
constexpr uint32_t PidDeviceControl = u(0x96);
constexpr uint32_t DcEnableActuators = u(0x97);
constexpr uint32_t DcDisableActuators = u(0x98);
constexpr uint32_t DcStopAllEffects = u(0x99);
constexpr uint32_t DcDeviceReset = u(0x9A);
constexpr uint32_t DcDevicePause = u(0x9B);
constexpr uint32_t DcDeviceContinue = u(0x9C);
constexpr uint32_t DevicePaused = u(0x9F);
constexpr uint32_t ActuatorsEnabled = u(0xA0);
constexpr uint32_t SafetySwitch = u(0xA4);
constexpr uint32_t ActuatorOverrideSwitch = u(0xA5);
constexpr uint32_t ActuatorPower = u(0xA6);
constexpr uint32_t StartDelay = u(0xA7);
constexpr uint32_t ParameterBlockSize = u(0xA8);
constexpr uint32_t DeviceManagedPool = u(0xA9);
constexpr uint32_t SharedParameterBlocks = u(0xAA);
constexpr uint32_t CreateNewEffectReport = u(0xAB); //!< 5.11.2
constexpr uint32_t RamPoolAvailable = u(0xAC);
} // namespace use

struct UsageName {
	uint16_t id;
	const char *name;
};
/** The whole Table 2 (named rows) */
const std::vector<UsageName> &usageTable();
/** Extended usage of a PID usage name of Table 2 (exact match, e.g. "Set Effect Report"); 0xFFFFFFFF if unknown */
uint32_t usageByName(const std::string &name);
/** Name of a usage of the PID page; "" if it is on another page or not in the table */
std::string nameOf(uint32_t extendedUsage);

/** One decoded element of a report */
struct DecodedValue {
	const hid::Field *field = nullptr;
	uint32_t element = 0;
	uint32_t usage = 0; //!< usage of the element (0 if none)
	int64_t value = 0;
};

/** A decoded input / output / feature report */
class DecodedReport {
public:
	std::vector<DecodedValue> values;
	std::vector<std::string> errors; //!< e.g. wrong length

	/** Value of the first element with that usage in a Variable field. false if none */
	bool get(uint32_t usage, int64_t &value, size_t occurrence = 0) const;
	/** True if a non-Variable (array / named array) field of the report selects `selector` with its current value */
	bool selects(uint32_t selector) const;
	/** Value of the field with that usage and collection sub-path, see HidReportDescriptor::findField */
	bool getIn(const std::vector<uint32_t> &subPath, uint32_t usage, int64_t &value) const;
};

/** A report under construction. All values in logical units of the field */
class PidReport {
public:
	PidReport(const hid::HidReportDescriptor &desc, const hid::Report *report, std::string name, bool failTests);

	/** Sets the first element of the first Variable field of the report that declares `fieldUsage` (the element at the position of the usage in the field's usage list) */
	PidReport &set(uint32_t fieldUsage, int64_t value, size_t occurrence = 0);
	/** Like set, restricted by the field's collection sub-path (see HidReportDescriptor::findField), e.g. {Direction} for the X axis of the Direction collection */
	PidReport &setIn(const std::vector<uint32_t> &subPath, uint32_t fieldUsage, int64_t value, size_t occurrence = 0);
	/** Sets element `element` of the field that declares `fieldUsage` (for count > 1 fields with one usage, e.g. byte arrays) */
	PidReport &setElement(uint32_t fieldUsage, uint32_t element, int64_t value);
	/** Selects `selector` in the array / named-array field that declares it (e.g. Effect Type = ET Sine, Effect Operation = Op Effect Start) */
	PidReport &select(uint32_t selector);
	/** Sets a Variable one-bit field (a flag, e.g. Direction Enable) */
	PidReport &flag(uint32_t fieldUsage, bool on, size_t occurrence = 0) { return set(fieldUsage, on ? 1 : 0, occurrence); }

	/** The bytes, length = the descriptor's report length (ID byte included if the descriptor uses report IDs), unset bits 0 */
	const std::vector<uint8_t> &bytes() const { return bytes_; }
	uint8_t reportId() const { return report_ ? report_->reportId : 0; }
	uint32_t length() const { return report_ ? report_->byteLength() : 0; }
	bool valid() const { return report_ != nullptr; }
	const std::vector<std::string> &errors() const { return errors_; }
	const std::string &name() const { return name_; }

private:
	friend class PidReportBuilder;
	/** Writes the single command byte of the Device Control report (see PidReportBuilder::deviceControl) */
	void setCommandByte(uint8_t value);
	void error(const std::string &what);
	PidReport &setImpl(const std::vector<uint32_t> &subPath, uint32_t fieldUsage, uint32_t element, bool useElement, int64_t value, size_t occurrence);
	const hid::HidReportDescriptor &desc_;
	const hid::Report *report_;
	std::string name_;
	bool failTests_;
	std::vector<uint8_t> bytes_;
	std::vector<std::string> errors_;
};

class PidReportBuilder {
public:
	/** `desc` must outlive the builder. failTests = false only for the self-tests of the builder */
	explicit PidReportBuilder(const hid::HidReportDescriptor &desc, bool failTests = true) : desc_(desc), failTests_(failTests) {}

	/** The report of that type that is wrapped in a collection with that PID usage (e.g. use::SetEffectReport); nullptr if none or more than one */
	const hid::Report *find(hid::ReportType type, uint32_t reportUsage) const;
	/** Report builder for the report with that usage. If the descriptor does not declare it exactly once, the failure is recorded and the PidReport is invalid */
	PidReport make(hid::ReportType type, uint32_t reportUsage) const;
	/** Same by the PID usage name of Table 2, e.g. make(Output, "Set Effect Report") */
	PidReport make(hid::ReportType type, const std::string &reportUsageName) const;

	/**
	 * PID Device Control output report in the form of USB_PID1_01.pdf Figure 16 (p. 40, "Example PID Device Control Output Report"):
	 * byte 0 the report ID, byte 1 "PID Device Control" = the command number. `command` is one of the DC usages 0x97-0x9C
	 * (Table 2: DC Enable Actuators ... DC Device Continue); its number is the descriptor's logical minimum plus its position in the
	 * field's usage list (section 6 example: Logical 1..6 for the six usages in Table 2 order, so 1..6). The descriptor declares the
	 * command as 8 elements of 1 bit (section 6: Report Size 1, Count 8), which cannot hold 2..6 element by element, so the byte is
	 * written as a whole; the descriptor must declare the field at bit 0 with 8 bits in a 2-byte report, otherwise this is an error.
	 */
	PidReport deviceControl(uint32_t command) const;

	/** Decodes a report received from the device (or built here) by report usage. `bytes` includes the ID byte if the descriptor uses report IDs */
	DecodedReport decode(hid::ReportType type, uint32_t reportUsage, const std::vector<uint8_t> &bytes) const;
	DecodedReport decode(const hid::Report &report, const std::vector<uint8_t> &bytes) const;

private:
	const hid::HidReportDescriptor &desc_;
	bool failTests_;
};

} // namespace pid

#endif
