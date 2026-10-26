/*
 * CanAnalog (FFBoard/UserExtensions/Src/CanAnalog.cpp): analog axes received as CAN frames (4 signed 16 bit little
 * endian values per 8 byte frame, frame n of the axes 4n..4n+3 on CAN id canId + n), its CAN filter on the global
 * `canport`, its commands and its flash configuration. The registry creates CanAnalog<8> (at most 8 axes).
 *
 * Frames are delivered the way the hardware does it (HostTarget::canReceive): through the filters of the canport mock,
 * the receive FIFO and the CAN callbacks of global_callbacks.cpp to every CanHandler.
 * There is no timeout in CanAnalog: the axes keep the last received values (see NEEDS REVIEW 92).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "MockCANPort.h"
#include "TestHelpers.h"

#include "CanAnalog.h"
#include "eeprom_addresses.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

struct Commands {
	CommandHandler &handler;
	std::vector<CommandReply> replies;
	CommandStatus run(const std::string &name, CMDtype type, int64_t val = 0) {
		CmdHandlerCommanddef *def = handler.getCommandFromName(name);
		REQUIRE_MESSAGE(def != nullptr, "command " << name << " not registered");
		replies.clear();
		ParsedCommand parsed;
		parsed.cmdId = def->cmdId;
		parsed.type = type;
		parsed.val = val;
		parsed.target = &handler;
		return handler.command(parsed, replies);
	}
	int64_t get(const std::string &name) {
		REQUIRE(run(name, CMDtype::get) == CommandStatus::OK);
		REQUIRE(replies.size() == 1);
		return replies[0].val;
	}
};

/** 8 byte payload with 4 signed 16 bit little endian values */
std::vector<uint8_t> frame(int16_t a, int16_t b = 0, int16_t c = 0, int16_t d = 0) {
	std::vector<uint8_t> data;
	for (int16_t v : {a, b, c, d}) {
		const uint16_t u = static_cast<uint16_t>(v);
		data.push_back(static_cast<uint8_t>(u & 0xFF));
		data.push_back(static_cast<uint8_t>(u >> 8));
	}
	return data;
}

using CanAnalog8 = CanAnalog<8>;

} // namespace

TEST_SUITE("CanAnalog") {

// ---------------------------------------------------------------- defaults and CAN port setup

TEST_CASE_FIXTURE(TargetFixture, "a new CAN analog source has one axis on CAN id 110") {
	CanAnalog8 analog;
	Commands c{analog};
	CHECK(c.get("canid") == 110);
	CHECK(c.get("amount") == 1);
	CHECK(c.get("maxamount") == 8);
	CHECK(*analog.getAxes() == std::vector<int32_t>{0});
}

TEST_CASE_FIXTURE(TargetFixture, "a new CAN analog source starts the CAN port with a filter for its two frame ids") {
	CanAnalog8 analog;
	MockCANPort &port = HostTarget::canPort();
	CHECK(port.started);
	REQUIRE(port.activeFilters() == 1);
	CHECK(port.filters[0].buffer == 0);
	CHECK(port.passesFilter(110));
	CHECK(port.passesFilter(111));
	CHECK_FALSE(port.passesFilter(112));
	CHECK_FALSE(port.passesFilter(109));
}

TEST_CASE_FIXTURE(TargetFixture, "deleting a CAN analog source removes its CAN filter") {
	{
		CanAnalog8 analog;
		REQUIRE(HostTarget::canPort().activeFilters() == 1);
	}
	CHECK(HostTarget::canPort().activeFilters() == 0);
}

TEST_CASE_FIXTURE(TargetFixture, "the CAN analog source is the class CAN Analog with the command name cananalog") {
	CanAnalog8 analog;
	CHECK(std::string(analog.getInfo().name) == "CAN Analog");
	CHECK(analog.getInfo().id == CLSID_ANALOG_CAN);
	CHECK(std::string(analog.getCommandHandlerInfo()->clsname) == "cananalog");
	CHECK(static_cast<AnalogSource &>(analog).getClassType() == ClassType::Analogsource);
	CHECK(analog.getHelpstring() == "4 16b axes per 64b packet");
}

TEST_CASE_FIXTURE(TargetFixture, "the CAN analog commands have the documented access flags") {
	CanAnalog8 analog;
	CHECK(analog.getCommandFromName("canid")->flags == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(analog.getCommandFromName("amount")->flags == (CMDFLAG_GET | CMDFLAG_SET));
	CHECK(analog.getCommandFromName("maxamount")->flags == CMDFLAG_GET);
}

// ---------------------------------------------------------------- frame decoding

TEST_CASE_FIXTURE(TargetFixture, "the axis values are little endian 16 bit values of the frame") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 4) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(110, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{0x0201, 0x0403, 0x0605, 0x0807});
}

TEST_CASE_FIXTURE(TargetFixture, "the axis values are signed") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 4) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(110, frame(-32768, 32767, -1, 0)) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{-32768, 32767, -1, 0});
}

TEST_CASE_FIXTURE(TargetFixture, "the axes 5 to 8 are received in the frame with the next CAN id") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 8) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(110, frame(1, 2, 3, 4)) == 1);
	REQUIRE(HostTarget::canReceive(111, frame(5, 6, 7, 8)) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{1, 2, 3, 4, 5, 6, 7, 8});
}

TEST_CASE_FIXTURE(TargetFixture, "with 6 axes the second frame sets only the axes 5 and 6") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 6) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(111, frame(5, 6, 7, 8)) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{0, 0, 0, 0, 5, 6});
}

TEST_CASE_FIXTURE(TargetFixture, "with one axis only the first value of the frame is used") {
	CanAnalog8 analog;
	REQUIRE(HostTarget::canReceive(110, frame(-7, 8, 9, 10)) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{-7});
}

TEST_CASE_FIXTURE(TargetFixture, "a frame with the next CAN id is ignored while all axes fit into one frame") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 4) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(111, frame(5, 6, 7, 8)) == 1); // passes the filter (ids 110 and 111)
	CHECK(*analog.getAxes() == std::vector<int32_t>{0, 0, 0, 0});
}

TEST_CASE_FIXTURE(TargetFixture, "a short frame sets only the axes whose two bytes it contains") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 4) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(110, {0x10, 0x00, 0x20, 0x00, 0x30}) == 1); // 5 bytes: axes 1 and 2 complete
	CHECK(*analog.getAxes() == std::vector<int32_t>{0x10, 0x20, 0, 0});
}

TEST_CASE_FIXTURE(TargetFixture, "a frame without data changes no axis") {
	CanAnalog8 analog;
	REQUIRE(HostTarget::canReceive(110, frame(5)) == 1);
	REQUIRE(HostTarget::canReceive(110, {}) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{5});
}

TEST_CASE_FIXTURE(TargetFixture, "a remote frame with the CAN id changes no axis") {
	CanAnalog8 analog;
	REQUIRE(HostTarget::canReceive(110, frame(5)) == 1);
	REQUIRE(HostTarget::canReceive(110, frame(9), false, true) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{5});
}

TEST_CASE_FIXTURE(TargetFixture, "a frame of another CAN id is not accepted by the CAN filter") {
	CanAnalog8 analog;
	CHECK(HostTarget::canReceive(112, frame(5)) == 0);
	CHECK(*analog.getAxes() == std::vector<int32_t>{0});
}

TEST_CASE_FIXTURE(TargetFixture, "a frame of another CAN id that passes another filter changes no axis") {
	CanAnalog8 analog;
	CAN_filter other;
	other.filter_id = 0;
	other.filter_mask = 0; // accepts every standard frame (e.g. another CAN handler)
	HostTarget::canPort().addCanFilter(other);
	REQUIRE(HostTarget::canReceive(300, frame(5)) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{0});
}

TEST_CASE_FIXTURE(TargetFixture, "an extended frame whose low 11 bits equal the CAN id changes no axis" *
				  KNOWN_ISSUE("canRxPendCallback compares only (id & 0x7FF) with the CAN id and does not check "
							  "msg.header.extId (CanAnalog.cpp:110-121): an extended frame accepted by any other filter "
							  "of the port (frames go to every CanHandler) is decoded as axis values. The CHECK on the "
							  "axis value fails")) {
	CanAnalog8 analog;
	CAN_filter ext;
	ext.filter_id = 0;
	ext.filter_mask = 0;
	ext.extid = true; // another handler accepting extended frames
	HostTarget::canPort().addCanFilter(ext);
	REQUIRE(HostTarget::canReceive(0x10000 | 110, frame(5), true) == 1);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(analog.getAxes()->at(0) == 0);
}

// NEEDS REVIEW 92: CanAnalog has no receive timeout: when the sender stops, the axes keep the last received values
// forever (an analog pedal or handbrake stays pressed). No error or reset to 0. Intended?
TEST_CASE_FIXTURE(TargetFixture, "characterization: the axes keep the last received values when no more frames arrive") {
	CanAnalog8 analog;
	REQUIRE(HostTarget::canReceive(110, frame(1234)) == 1);
	HostPlatform::advanceMs(60000);
	CHECK(*analog.getAxes() == std::vector<int32_t>{1234});
}

// ---------------------------------------------------------------- commands

TEST_CASE_FIXTURE(TargetFixture, "the amount command sets the number of axes") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 3) == CommandStatus::OK);
	CHECK(c.get("amount") == 3);
	CHECK(analog.getAxes()->size() == 3);
}

TEST_CASE_FIXTURE(TargetFixture, "the amount of axes is limited to the maximum of 8") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 12) == CommandStatus::OK);
	CHECK(c.get("amount") == 8);
	CHECK(analog.getAxes()->size() == 8);
}

TEST_CASE_FIXTURE(TargetFixture, "the maximum amount of axes is the template parameter") {
	CanAnalog<2> analog;
	Commands c{analog};
	CHECK(c.get("maxamount") == 2);
	REQUIRE(c.run("amount", CMDtype::set, 4) == CommandStatus::OK);
	CHECK(c.get("amount") == 2);
}

TEST_CASE_FIXTURE(TargetFixture, "an amount of 0 axes gives no axis values and ignores frames") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 0) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(110, frame(5)) == 1);
	CHECK(analog.getAxes()->empty());
}

TEST_CASE_FIXTURE(TargetFixture, "lowering the amount of axes keeps the values of the remaining axes") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 4) == CommandStatus::OK);
	REQUIRE(HostTarget::canReceive(110, frame(1, 2, 3, 4)) == 1);
	REQUIRE(c.run("amount", CMDtype::set, 2) == CommandStatus::OK);
	CHECK(*analog.getAxes() == std::vector<int32_t>{1, 2});
}

TEST_CASE_FIXTURE(TargetFixture, "the canid command moves the CAN filter and the frames to the new id") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("canid", CMDtype::set, 0x300) == CommandStatus::OK);
	CHECK(c.get("canid") == 0x300);
	MockCANPort &port = HostTarget::canPort();
	CHECK(port.activeFilters() == 1);
	CHECK(HostTarget::canReceive(110, frame(1)) == 0);
	REQUIRE(HostTarget::canReceive(0x300, frame(2)) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{2});
}

// NEEDS REVIEW 84: the filter mask is ~(canId ^ (canId + 1))
// (CanAnalog.cpp:39). For an even id it covers exactly canId and canId + 1; for an odd id the carry flips several low
// bits, e.g. canId 111 (0x6F) ^ 112 (0x70) = 0x1F: the hardware filter accepts the 32 ids 96..127 and the handler
// sorts the frames by id in software. Only the 2 ids that can carry axes are needed. A wider filter than the frames
// of the source intended (it costs receive interrupts and FIFO space, frames are never decoded wrongly)?
TEST_CASE_FIXTURE(TargetFixture, "characterization: for the odd CAN id 111 the CAN filter accepts the 32 ids 96 to 127") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 8) == CommandStatus::OK);
	REQUIRE(c.run("canid", CMDtype::set, 111) == CommandStatus::OK);
	MockCANPort &port = HostTarget::canPort();
	REQUIRE(port.activeFilters() == 1);
	const CAN_filter *filter = nullptr;
	for (const CAN_filter &f : port.filters) {
		if (f.active) {
			filter = &f;
		}
	}
	REQUIRE(filter != nullptr);
	CHECK(filter->filter_id == 111);
	CHECK(filter->filter_mask == ~static_cast<uint32_t>(0x1F));
	CHECK_FALSE(filter->extid);
	// Accepted by the (bxCAN mask) filter of the port: the whole block 96..127
	CHECK(HostTarget::canReceive(95, frame(1)) == 0);
	CHECK(HostTarget::canReceive(96, frame(2)) == 1);
	CHECK(HostTarget::canReceive(127, frame(3)) == 1);
	CHECK(HostTarget::canReceive(128, frame(4)) == 0);
	CHECK(*analog.getAxes() == std::vector<int32_t>(8, 0)); // accepted frames of other ids are not decoded
	REQUIRE(HostTarget::canReceive(111, frame(1, 2, 3, 4)) == 1);
	REQUIRE(HostTarget::canReceive(112, frame(5, 6, 7, 8)) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{1, 2, 3, 4, 5, 6, 7, 8});
}

TEST_CASE_FIXTURE(TargetFixture, "for the even CAN id 110 the CAN filter accepts only the ids 110 and 111") {
	CanAnalog8 analog;
	CHECK(HostTarget::canReceive(109, frame(1)) == 0);
	CHECK(HostTarget::canReceive(110, frame(1)) == 1);
	CHECK(HostTarget::canReceive(111, frame(1)) == 1);
	CHECK(HostTarget::canReceive(112, frame(1)) == 0);
}

TEST_CASE_FIXTURE(TargetFixture, "the canid command keeps the id within 11 bits") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("canid", CMDtype::set, 0x900) == CommandStatus::OK);
	CHECK(c.get("canid") == 0x100);
}

TEST_CASE_FIXTURE(TargetFixture, "with one axis the canid command accepts the highest standard id 0x7FF") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("canid", CMDtype::set, 0x7FF) == CommandStatus::OK);
	CHECK(c.get("canid") == 0x7FF);
	REQUIRE(HostTarget::canReceive(0x7FF, frame(3)) == 1);
	CHECK(*analog.getAxes() == std::vector<int32_t>{3});
}

TEST_CASE_FIXTURE(TargetFixture, "with 5 axes the canid command keeps the second frame id within 11 bits") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 5) == CommandStatus::OK);
	REQUIRE(c.run("canid", CMDtype::set, 0x7FF) == CommandStatus::OK);
	CHECK(c.get("canid") == 0x7FE);
}

TEST_CASE_FIXTURE(TargetFixture, "with 8 axes the canid command accepts the id 0x7FE for two frames" *
				  KNOWN_ISSUE("the canid limit is 0x7FF - axes/4 (CanAnalog.cpp:97) instead of 0x7FF - (packets - 1): "
							  "with 8 axes (2 frames, ids canId and canId+1) the id is limited to 0x7FD, so 0x7FE can not be "
							  "used (with 4 axes 0x7FF is likewise refused). The CHECK on the id fails")) {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("amount", CMDtype::set, 8) == CommandStatus::OK);
	REQUIRE(c.run("canid", CMDtype::set, 0x7FE) == CommandStatus::OK);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(c.get("canid") == 0x7FE);
}

TEST_CASE_FIXTURE(TargetFixture, "raising the amount of axes keeps the id of the last frame within 11 bits" *
				  KNOWN_ISSUE("setAxisNum does not re-check the CAN id (only the canid command clips it, CanAnalog.cpp:97): "
							  "canid 0x7FF with 1 axis, then amount 8 needs a second frame with id 0x800, which no standard "
							  "frame can have (the received id is masked with 0x7FF), so the axes 5 to 8 are never "
							  "received. The CHECK on the id fails")) {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("canid", CMDtype::set, 0x7FF) == CommandStatus::OK);
	REQUIRE(c.run("amount", CMDtype::set, 8) == CommandStatus::OK);
	REQUIRE_ALL_HAL_MODELLED();
	CHECK(c.get("canid") + 1 <= 0x7FF);
}

TEST_CASE_FIXTURE(TargetFixture, "an unknown command id is not found") {
	CanAnalog8 analog;
	ParsedCommand parsed;
	parsed.cmdId = 0x1234;
	parsed.type = CMDtype::get;
	std::vector<CommandReply> replies;
	CHECK(analog.command(parsed, replies) == CommandStatus::NOT_FOUND);
}

// ---------------------------------------------------------------- flash

TEST_CASE_FIXTURE(TargetFixture, "saving stores the CAN id in bits 0 to 10 and the amount of axes in bits 11 to 14") {
	CanAnalog8 analog;
	Commands c{analog};
	REQUIRE(c.run("canid", CMDtype::set, 0x2A5) == CommandStatus::OK);
	REQUIRE(c.run("amount", CMDtype::set, 6) == CommandStatus::OK);
	analog.saveFlash();
	CHECK(HostPlatform::flash().at(ADR_CANANALOG_CONF1) == (0x2A5 | (6 << 11)));
}

TEST_CASE_FIXTURE(TargetFixture, "a new CAN analog source restores the CAN id and the amount of axes from flash") {
	HostPlatform::flash()[ADR_CANANALOG_CONF1] = 0x123 | (7 << 11);
	CanAnalog8 analog;
	Commands c{analog};
	CHECK(c.get("canid") == 0x123);
	CHECK(c.get("amount") == 7);
	CHECK(analog.getAxes()->size() == 7);
	CHECK(HostTarget::canPort().passesFilter(0x123));
	CHECK(HostTarget::canPort().passesFilter(0x124));
}

TEST_CASE_FIXTURE(TargetFixture, "a stored amount above the maximum is limited to the maximum") {
	HostPlatform::flash()[ADR_CANANALOG_CONF1] = 110 | (15 << 11);
	CanAnalog8 analog;
	Commands c{analog};
	CHECK(c.get("amount") == 8);
}

TEST_CASE_FIXTURE(TargetFixture, "a saved configuration is restored by a new CAN analog source") {
	{
		CanAnalog8 analog;
		Commands c{analog};
		REQUIRE(c.run("canid", CMDtype::set, 0x400) == CommandStatus::OK);
		REQUIRE(c.run("amount", CMDtype::set, 8) == CommandStatus::OK);
		analog.saveFlash();
	}
	CanAnalog8 analog;
	Commands c{analog};
	CHECK(c.get("canid") == 0x400);
	CHECK(c.get("amount") == 8);
	REQUIRE(HostTarget::canReceive(0x401, frame(5, 6, 7, 8)) == 1);
	CHECK(analog.getAxes()->at(7) == 8);
}

TEST_CASE_FIXTURE(TargetFixture, "a CAN analog source can be created and deleted through the analog source registry") {
	const class_entry<AnalogSource> *entry = nullptr;
	for (const auto &e : AnalogSource::all_analogsources) {
		if (e.info.id == CLSID_ANALOG_CAN) {
			entry = &e;
		}
	}
	REQUIRE(entry != nullptr);
	std::unique_ptr<AnalogSource> source(entry->create());
	REQUIRE(source != nullptr);
	CHECK(HostTarget::canPort().activeFilters() == 1);
	CHECK(source->getAxes()->size() == 1);
	source.reset();
	CHECK(HostTarget::canPort().activeFilters() == 0);
}

} // TEST_SUITE
