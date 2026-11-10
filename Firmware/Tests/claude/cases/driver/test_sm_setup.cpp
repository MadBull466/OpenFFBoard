/*
 * SimpleMotion driver, phase P15: T1 skeleton (the first ten cases, unchanged) plus the T2 work order SMS: identity,
 * chooser, construction and life cycle of MotorSimplemotion. Charter: Tests/claude/charters/driver/SIMPLEMOTION_REQUIREMENTS.md.
 * Requirements covered: R1.1-R1.4, R2.1, R2.3 (R2.2 is proved by the wire bytes of R3.1 / R5.1 in the SMF / SMP files), R11.1, R11.2.
 *
 * Oracles: SM (SimpleMotion V2 library, Tests/shared/specs/SimpleMotionV2-master, never copied) for frame ids, CRCs and
 * subpacket sizes of the T1 cases; HD (hand derivations in the comments); FW (identity facts of the source, plain);
 * IF (MotorDriver / Encoder interface: getEncoder() returns the driver, absolute encoder, class type Motordriver). The fake is
 * checked against the library values by script.
 *
 * Host versus target: nothing here depends on type sizes, packing or byte order. The handler class id of both instances is
 * a 16 bit constant. MotorSimplemotion.cpp is compiled with -funsigned-char (charter L4), which only matters for the
 * receive path (other files).
 *
 * Characterizations of this file (NEEDS REVIEW): "[R1.1 handler]" (handler name "sm2" and class id 0x89 of both instances),
 * "[R1.2 axis lists]" (the second axis list holds no SimpleMotion entry: the charter said both lists contain 9 and 10, the
 * firmware comments the second one out), "[R11.2 ...]" (two instances on the one motor port).
 * KNOWN_ISSUE_UB: "[R1.4 address]" (B-SM-1: `address(address + 1)` initialises the member from itself; the fixture builds the
 * driver in 0xA5-filled memory so the value is 0xA6 on every host build, and the case is skipped in sanitizer builds).
 */
#include "doctest.h"
#include "SmFixture.h"

#include "Axis.h"
#include "ClassChooser.h"
#include "CommandHandler.h"
#include "MotorDriver.h"
#include "constants.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

extern UARTPort motor_uart; // platform/src/host_target_config.cpp (declared in cpp_target_config.h)

namespace {

constexpr uint16_t kSm1SelectionId = 9;  // MotorDriver.cpp:47, Axis.cpp:66
constexpr uint16_t kSm2SelectionId = 10; // MotorDriver.cpp:48
constexpr uint16_t kSm1ClassId = 0x89;   // CLSID_MOT_SM1
constexpr uint16_t kSm2ClassId = 0x8A;   // CLSID_MOT_SM2

/** Entries of a driver list with the given selection id */
std::vector<const class_entry<MotorDriver> *> entriesWithId(const std::vector<class_entry<MotorDriver>> &list, uint16_t id) {
	std::vector<const class_entry<MotorDriver> *> found;
	for (const auto &e : list) {
		if (e.selectionId == id) {
			found.push_back(&e);
		}
	}
	return found;
}

/** A second device that wants the motor port: the UARTDevice constructor tries to reserve it (the result is ignored there), tryReserve asks again */
struct PortProbe : UARTDevice {
	PortProbe() : UARTDevice(motor_uart) {}
	bool tryReserve() { return motor_uart.reservePort(this); }
};

} // namespace

TEST_SUITE("SM setup") {

TEST_CASE("the fake device's CRC-8 gives the library value for the fast update header {0x10, 0x01, 0, 0, 0, 0}") {
	// HD: CRC-8 poly 0x07, init 0x52, over 10 01 00 00 00 00 = 0x46 (charter R3.1, checked against sm_consts.c table_crc8 by script)
	const uint8_t b[6] = {0x10, 0x01, 0, 0, 0, 0};
	CHECK(FakeSmDevice::crc8(b, 6, 0x52) == 0x46);
}

TEST_CASE("the fake device's CRC-16 of the check string 123456789 is the CRC-16/ARC value 0xBB3D (library value byte swapped: 0x3DBB)") {
	const uint8_t s[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
	CHECK(FakeSmDevice::crc16arc(s, 9) == 0xBB3D);
}

TEST_CASE_FIXTURE(SmFixture, "a MotorSimplemotion1 can be constructed and destroyed and the motor UART is reserved while it exists") {
	CHECK(MotorSimplemotion1::isCreatable());
	{
		auto sm = makeSm(1);
		REQUIRE(static_cast<bool>(sm));
		CHECK_FALSE(MotorSimplemotion1::isCreatable());
		CHECK(MotorSimplemotion2::isCreatable());
	}
	CHECK(MotorSimplemotion1::isCreatable());
}

TEST_CASE_FIXTURE(SmFixture, "the constructor configures the motor UART to 460800 baud 8N1 with TX and RX and oversampling 8, write enable low") {
	HAL_GPIO_WritePin(GPIOE, DRV_GP1_Pin, GPIO_PIN_SET); // the constructor must drive the pin low (review finding: it is low after reset anyway)
	auto sm = makeSm(1);
	CHECK(huart3.Init.BaudRate == 460800);
	CHECK(huart3.Init.WordLength == UART_WORDLENGTH_8B);
	CHECK(huart3.Init.StopBits == UART_STOPBITS_1);
	CHECK(huart3.Init.Parity == UART_PARITY_NONE);
	CHECK(huart3.Init.Mode == UART_MODE_TX_RX);
	CHECK(huart3.Init.OverSampling == UART_OVERSAMPLING_8);
	CHECK((GPIOE->ODR & DRV_GP1_Pin) == 0);
}

TEST_CASE_FIXTURE(SmFixture, "the driver lists contain the SimpleMotion drivers 9 and 10 once each") {
	int one = 0, two = 0;
	for (const auto &c : MotorDriver::all_drivers) {
		one += c.selectionId == 9 && c.info.id == CLSID_MOT_SM1;
		two += c.selectionId == 10 && c.info.id == CLSID_MOT_SM2;
	}
	CHECK(one == 1);
	CHECK(two == 1);
}

TEST_CASE_FIXTURE(SmFixture, "on an initialized driver turn(0) sends the 7 byte fast update {0x10, address, 0, 0, 0, 0, crc8} with a valid CRC-8 in one transfer") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	const uint32_t transfers = HostPlatform::uart().txTransfers;
	sm->turn(0);
	REQUIRE(dev.frameCount() == 1);
	const auto &f = dev.frames()[0];
	CHECK(f.kind == FakeSmDevice::Kind::Fast);
	REQUIRE(f.bytes.size() == 7);
	CHECK(f.bytes[0] == 0x10);
	CHECK(f.bytes[2] == 0);
	CHECK(f.bytes[3] == 0);
	CHECK(f.bytes[4] == 0);
	CHECK(f.bytes[5] == 0);
	CHECK(f.bytes[6] == FakeSmDevice::crc8(f.bytes.data(), 6, 0x52)); // the fake also checked it: no violation
	CHECK(HostPlatform::uart().txTransfers == transfers + 1);
}

TEST_CASE_FIXTURE(SmFixture, "the settings sequence on a healthy device ends initialized and the exchanges run without a peer violation") {
	auto sm = makeSm(1);
	CHECK_FALSE(sm->motorReady());
	REQUIRE(initialize(*sm));
	CHECK(sm->motorReady());
	// devtype, faults, status, FBR, FBD, ControlMode: six instant commands
	CHECK(dev.frameCount() == 6);
	CHECK(dev.violations().empty());
}

TEST_CASE_FIXTURE(SmFixture, "with a silent peer every parameter request times out after 10 ms of simulated time and the driver stays uninitialized") {
	dev.silent = true;
	auto sm = makeSm(1);
	advanceMs(150); // getSettings refuses to run earlier
	const uint64_t before = HostPlatform::nowUs();
	CHECK_FALSE(sm->getSettings());
	CHECK_FALSE(sm->motorReady());
	// Every request waits 10 ms (characterization of the constant, charter R4.6). The sequence sends 3 requests (devtype, faults, status:
	// the status read failing with an uninitialised `st` (B-SM-4) may end it there) up to 6: 30 ms .. 60 ms plus the 1 ms poll steps.
	const uint64_t elapsedUs = HostPlatform::nowUs() - before;
	CHECK(elapsedUs >= 30000);
	CHECK(elapsedUs <= 70000);
}

TEST_CASE_FIXTURE(SmFixture, "a fast update reply delivered by the fake updates the position and releases the port for the next frame") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.position16 = 5;
	sm->turn(100);
	process(); // transmit complete interrupt, then the reply bytes
	CHECK(dev.bytesNotAccepted() == 0);
	// HD: report 5 -> value 5 + 0x7fff = 0x8004, lastPosRep 0x7fff: diff 5, no overflow, position 0x8004 - 0x7fff = 5
	CHECK(sm->getPos() == 5);
	dev.clearFrames();
	sm->turn(200);
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.setpoints().back() == 200);
}

TEST_CASE_FIXTURE(SmFixture, "the write enable pin is high after the transmit and low after the transmit complete interrupt") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	sm->turn(0);
	CHECK((GPIOE->ODR & DRV_GP1_Pin) != 0); // set by startUartTransfer(transmit)
	process();
	CHECK((GPIOE->ODR & DRV_GP1_Pin) == 0); // reset by endUartTransfer(transmit) in the tx complete callback
}


// ---------------------------------------------------------------- R1.1 identity

TEST_CASE("[R1.1 identity] MotorSimplemotion1::info is named Simplemotion 1 with class id 0x89") {
	REQUIRE(MotorSimplemotion1::info.name != nullptr);
	CHECK(std::string(MotorSimplemotion1::info.name) == "Simplemotion 1");
	CHECK(MotorSimplemotion1::info.id == kSm1ClassId);
}

TEST_CASE("[R1.1 identity] MotorSimplemotion2::info is named Simplemotion 2 with class id 0x8A") {
	REQUIRE(MotorSimplemotion2::info.name != nullptr);
	CHECK(std::string(MotorSimplemotion2::info.name) == "Simplemotion 2");
	CHECK(MotorSimplemotion2::info.id == kSm2ClassId);
}

TEST_CASE_FIXTURE(SmFixture, "[R1.1 identity] getInfo of a live instance returns the info of its class through the command handler base") {
	auto one = makeSm(1);
	CommandHandler *h1 = one.sm; // virtual call through the CommandHandler base
	CHECK(h1->getInfo().id == kSm1ClassId);
	CHECK(std::string(h1->getInfo().name) == "Simplemotion 1");
	one.destroy();
	auto two = makeSm(2);
	CommandHandler *h2 = two.sm;
	CHECK(h2->getInfo().id == kSm2ClassId);
	CHECK(std::string(h2->getInfo().name) == "Simplemotion 2");
}

TEST_CASE_FIXTURE(SmFixture, "[R1.1 identity] the help string is RS485 Simplemotion interface") {
	auto sm = makeSm(1);
	CHECK(sm->getHelpstring() == "RS485 Simplemotion interface");
}

TEST_CASE_FIXTURE(SmFixture, "[R1.1 identity] the driver is a motor driver class") {
	auto sm = makeSm(1);
	MotorDriver *drv = sm.sm; // the class type is ambiguous between the MotorDriver and CommandHandler bases
	CHECK(drv->getClassType() == ClassType::Motordriver);
}

TEST_CASE_FIXTURE(SmFixture, "[R1.1 identity] the driver has an integrated encoder and getEncoder returns the driver itself") {
	auto sm = makeSm(1);
	CHECK(sm->hasIntegratedEncoder());
	Encoder *enc = sm->getEncoder();
	REQUIRE(enc != nullptr);
	CHECK(enc == static_cast<Encoder *>(sm.sm));
}

TEST_CASE_FIXTURE(SmFixture, "[R1.1 identity] the encoder type is absolute") {
	auto sm = makeSm(1);
	CHECK(sm->getEncoderType() == EncoderType::absolute);
}

TEST_CASE_FIXTURE(SmFixture, "[R1.1 handler] characterization: both instances register as command handler sm2 with class id 0x89 and the instance index 0 and 1") {
	// FW: CommandHandler("sm2", CLSID_MOT_SM1, instance) (MotorSimplemotion.cpp:30). The name sm2 and the class id of instance 1
	// for instance 2 look like leftovers (charter R12 item 8, informational). Instance index 0 / 1 is the constructor argument.
	auto one = makeSm(1);
	CmdHandlerInfo *i1 = one->getCommandHandlerInfo();
	REQUIRE(i1 != nullptr);
	CHECK(std::string(i1->clsname) == "sm2");
	CHECK(i1->clsTypeid == kSm1ClassId);
	CHECK(i1->instance == 0);
	one.destroy();
	auto two = makeSm(2);
	CmdHandlerInfo *i2 = two->getCommandHandlerInfo();
	REQUIRE(i2 != nullptr);
	CHECK(std::string(i2->clsname) == "sm2");
	CHECK(i2->clsTypeid == kSm1ClassId);
	CHECK(i2->instance == 1);
}

// ---------------------------------------------------------------- R1.2 chooser lists

TEST_CASE_FIXTURE(SmFixture, "[R1.2 chooser] MotorDriver::all_drivers lists the infos of Simplemotion 1 under id 9 and Simplemotion 2 under id 10") {
	auto e1 = entriesWithId(MotorDriver::all_drivers, kSm1SelectionId);
	auto e2 = entriesWithId(MotorDriver::all_drivers, kSm2SelectionId);
	REQUIRE(e1.size() == 1);
	REQUIRE(e2.size() == 1);
	CHECK(&e1.front()->info == &MotorSimplemotion1::info);
	CHECK(&e2.front()->info == &MotorSimplemotion2::info);
}

TEST_CASE_FIXTURE(SmFixture, "[R1.2 axis lists] characterization: the first axis list holds Simplemotion 1 under id 9 once and the second axis list holds no SimpleMotion entry") {
	// FW: Axis.cpp:65-67 adds MotorSimplemotion1 (9) to axis 1; the entry of MotorSimplemotion2 (10) of axis 2 is commented out
	// (Axis.cpp:95-97, "likely does not work reliably with a single uart port and multiple devices"). The charter R1.2
	// expected both lists to hold 9 and 10: owner decision needed.
	auto a1 = entriesWithId(Axis::axis1_drivers, kSm1SelectionId);
	REQUIRE(a1.size() == 1);
	CHECK(&a1.front()->info == &MotorSimplemotion1::info);
	CHECK(entriesWithId(Axis::axis1_drivers, kSm2SelectionId).empty());
	CHECK(entriesWithId(Axis::axis2_drivers, kSm1SelectionId).empty());
	CHECK(entriesWithId(Axis::axis2_drivers, kSm2SelectionId).empty());
}

TEST_CASE_FIXTURE(SmFixture, "[R1.2 chooser] creating id 9 from the driver list builds a Simplemotion 1 that remembers its selection id and initializes against a healthy device") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::unique_ptr<MotorDriver> drv(chooser.Create(kSm1SelectionId));
	REQUIRE(drv != nullptr);
	auto *sm = dynamic_cast<MotorSimplemotion1 *>(drv.get());
	REQUIRE(sm != nullptr);
	CHECK(drv->getSelectionID() == kSm1SelectionId);
	CHECK(drv->getClassType() == ClassType::Motordriver);
	CHECK(initialize(*sm));
	CHECK(sm->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R1.2 chooser] creating id 10 from the driver list builds a Simplemotion 2") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::unique_ptr<MotorDriver> drv(chooser.Create(kSm2SelectionId));
	REQUIRE(drv != nullptr);
	CHECK(dynamic_cast<MotorSimplemotion2 *>(drv.get()) != nullptr);
	CHECK(drv->getSelectionID() == kSm2SelectionId);
}

TEST_CASE_FIXTURE(SmFixture, "[R1.2 chooser] creating id 9 from the first axis list builds a Simplemotion 1") {
	ClassChooser<MotorDriver> chooser(Axis::axis1_drivers);
	std::unique_ptr<MotorDriver> drv(chooser.Create(kSm1SelectionId));
	REQUIRE(drv != nullptr);
	CHECK(dynamic_cast<MotorSimplemotion1 *>(drv.get()) != nullptr);
}

// ---------------------------------------------------------------- R1.3 in-use flags

TEST_CASE_FIXTURE(SmFixture, "[R1.3 flags] MotorSimplemotion2::isCreatable is true, false while an instance 2 exists and true after its destruction, instance 1 unaffected") {
	CHECK(MotorSimplemotion2::isCreatable());
	{
		auto sm = makeSm(2);
		CHECK_FALSE(MotorSimplemotion2::isCreatable());
		CHECK(MotorSimplemotion1::isCreatable());
	}
	CHECK(MotorSimplemotion2::isCreatable());
	CHECK(MotorSimplemotion1::isCreatable());
}

TEST_CASE_FIXTURE(SmFixture, "[R1.3 flags] a second creation of id 9 through the chooser while the first exists returns no object") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::unique_ptr<MotorDriver> first(chooser.Create(kSm1SelectionId));
	REQUIRE(first != nullptr);
	MotorDriver *second = chooser.Create(kSm1SelectionId);
	CHECK(second == nullptr);
	delete second; // nothing to delete if the chooser behaves, keeps the sanitizer build leak free if it does not
	CHECK_FALSE(MotorSimplemotion1::isCreatable()); // the first one still holds the flag
}

TEST_CASE_FIXTURE(SmFixture, "[R1.3 flags] the chooser reports id 9 creatable only while no Simplemotion 1 exists and id 10 creatable meanwhile") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	CHECK(chooser.isCreatable(kSm1SelectionId));
	std::unique_ptr<MotorDriver> drv(chooser.Create(kSm1SelectionId));
	REQUIRE(drv != nullptr);
	CHECK_FALSE(chooser.isCreatable(kSm1SelectionId));
	CHECK(chooser.isCreatable(kSm2SelectionId));
	drv.reset();
	CHECK(chooser.isCreatable(kSm1SelectionId));
}

TEST_CASE_FIXTURE(SmFixture, "[R1.3 flags] id 9 can be created through the chooser again after the first object was destroyed") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	{
		std::unique_ptr<MotorDriver> first(chooser.Create(kSm1SelectionId));
		REQUIRE(first != nullptr);
	}
	std::unique_ptr<MotorDriver> again(chooser.Create(kSm1SelectionId));
	CHECK(again != nullptr);
}

// ---------------------------------------------------------------- R1.4 address (B-SM-1)

TEST_CASE_FIXTURE(SmFixture, "[R1.4 address] instance 1 puts device address 1 on the wire" *
					  KNOWN_ISSUE_UB("B-SM-1: the constructor initialises address(address + 1) from itself, instance is unused; in 0xA5 poisoned memory the wire address is 0xA6 instead of 1")) {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	REQUIRE(dev.frameCount() > 0);
	CHECK(lastAddress() == 1); // intended: instance + 1 = 0 + 1
}

TEST_CASE_FIXTURE(SmFixture, "[R1.4 address] instance 2 puts device address 2 on the wire" *
					  KNOWN_ISSUE_UB("B-SM-1: the constructor initialises address(address + 1) from itself, instance is unused; in 0xA5 poisoned memory the wire address is 0xA6 instead of 2")) {
	auto sm = makeSm(2);
	REQUIRE(initialize(*sm));
	REQUIRE(dev.frameCount() > 0);
	CHECK(lastAddress() == 2); // intended: instance + 1 = 1 + 1
}

// ---------------------------------------------------------------- R2.1 construction

TEST_CASE_FIXTURE(SmFixture, "[R2.1 port] the motor UART is reserved while the driver exists and free before and after") {
	CHECK_FALSE(motor_uart.isReserved());
	{
		auto sm = makeSm(1);
		CHECK(motor_uart.isReserved());
	}
	CHECK_FALSE(motor_uart.isReserved());
}

TEST_CASE_FIXTURE(SmFixture, "[R2.1 port] a second device cannot reserve the motor UART while the driver lives and can after its destruction") {
	{
		auto sm = makeSm(1);
		PortProbe refused; // constructed while the driver owns the port
		CHECK_FALSE(refused.tryReserve());
	}
	PortProbe accepted; // the constructor reserves the free port
	CHECK(accepted.tryReserve());
}

TEST_CASE_FIXTURE(SmFixture, "[R2.1 port] the constructor configures no hardware flow control") {
	auto sm = makeSm(1);
	CHECK(huart3.Init.HwFlowCtl == UART_HWCONTROL_NONE);
}

TEST_CASE_FIXTURE(SmFixture, "[R2.1 port] the constructor sends nothing on the bus") {
	const uint32_t transfers = HostPlatform::uart().txTransfers;
	auto sm = makeSm(1);
	CHECK(dev.frameCount() == 0);
	CHECK(HostPlatform::uart().txTransfers == transfers);
}

// ---------------------------------------------------------------- R2.3 commands

TEST_CASE_FIXTURE(SmFixture, "[R2.3 commands] the eight commands are registered with their ids and access flags") {
	// cmdId = position in the enum MotorSimplemotion_commands (MotorSimplemotion.h:26): crcerrors 0, uarterrors 1, voltage 2,
	// torque 3, status 4, restart 5, reg 6, devtype 7. Flags as in registerCommands() (MotorSimplemotion.cpp:~485).
	struct Expect {
		const char *name;
		uint32_t id;
		uint32_t flags;
	};
	const Expect table[] = {
		{"crcerr", 0, CMDFLAG_GET},
		{"uarterr", 1, CMDFLAG_GET},
		{"voltage", 2, CMDFLAG_GET},
		{"torque", 3, CMDFLAG_GET},
		{"state", 4, CMDFLAG_GET},
		{"restart", 5, CMDFLAG_GET},
		{"reg", 6, CMDFLAG_GETADR | CMDFLAG_SETADR | CMDFLAG_DEBUG}, // 0x10 | 0x20 | 0x80 = 0xB0
		{"devtype", 7, CMDFLAG_GET},
	};
	auto sm = makeSm(1);
	// reg carries CMDFLAG_DEBUG: the lookup hides it unless the debug mode is on
	const bool debugBefore = SystemCommands::debugMode;
	SystemCommands::debugMode = true;
	for (const Expect &e : table) {
		INFO("command " << std::string(e.name));
		CmdHandlerCommanddef *def = sm->getCommandFromName(e.name);
		REQUIRE(def != nullptr);
		CHECK(def->cmdId == e.id);
		CHECK(def->flags == e.flags);
	}
	SystemCommands::debugMode = debugBefore;
}

TEST_CASE_FIXTURE(SmFixture, "[R2.3 commands] the reg command is hidden from the lookup unless the debug mode is on") {
	auto sm = makeSm(1);
	const bool debugBefore = SystemCommands::debugMode;
	SystemCommands::debugMode = false;
	CHECK(sm->getCommandFromName("reg") == nullptr);
	CHECK(sm->getCommandFromName("crcerr") != nullptr);
	SystemCommands::debugMode = true;
	CHECK(sm->getCommandFromName("reg") != nullptr);
	SystemCommands::debugMode = debugBefore;
}

TEST_CASE_FIXTURE(SmFixture, "[R2.3 commands] a name that is not one of the eight is not registered") {
	auto sm = makeSm(1);
	CHECK(sm->getCommandFromName("nosuchcmd") == nullptr);
	CHECK(sm->getCommandFromName("sm") == nullptr);
}

TEST_CASE_FIXTURE(SmFixture, "[R2.3 commands] the driver is in the global command handler list while it exists and removed after its destruction") {
	auto &handlers = CommandHandler::getCommandHandlers();
	CommandHandler *raw = nullptr;
	{
		auto sm = makeSm(1);
		raw = sm.sm;
		CHECK(std::find(handlers.begin(), handlers.end(), raw) != handlers.end());
	}
	CHECK(std::find(handlers.begin(), handlers.end(), raw) == handlers.end());
}

// ---------------------------------------------------------------- R11 life cycle

TEST_CASE_FIXTURE(SmFixture, "[R11.1 lifecycle] a new instance created after the destruction of the first reserves the port and initializes against the device") {
	{
		auto first = makeSm(1);
		REQUIRE(initialize(*first));
	}
	CHECK_FALSE(motor_uart.isReserved());
	dev.clearFrames();
	auto again = makeSm(1);
	CHECK(motor_uart.isReserved());
	REQUIRE(initialize(*again));
	CHECK(again->motorReady());
	CHECK(dev.frameCount() == 6);
	CHECK(dev.violations().empty()); // the CRC tables are still valid: the fake checks every CRC16 and CRC8
}

TEST_CASE_FIXTURE(SmFixture, "[R11.1 lifecycle] after a destruction and re-creation a fast update carries a valid CRC-8") {
	{
		auto first = makeSm(1);
		REQUIRE(initialize(*first));
	}
	auto again = makeSm(1);
	REQUIRE(initialize(*again));
	dev.clearFrames();
	again->turn(1000);
	REQUIRE(dev.frameCount() == 1);
	const auto &f = dev.frames()[0];
	REQUIRE(f.bytes.size() == 7);
	CHECK(f.bytes[6] == FakeSmDevice::crc8(f.bytes.data(), 6, 0x52));
	CHECK(dev.violations().empty());
}

TEST_CASE_FIXTURE(SmFixture, "[R11.1 lifecycle] instances 1 and 2 can be created and destroyed one after the other on the same port") {
	{
		auto one = makeSm(1);
		REQUIRE(initialize(*one));
	}
	{
		auto two = makeSm(2);
		CHECK(motor_uart.isReserved());
		CHECK(initialize(*two));
	}
	CHECK_FALSE(motor_uart.isReserved());
}

// ---------------------------------------------------------------- R11.2 two instances on one port

TEST_CASE_FIXTURE(SmFixture, "[R11.2 two instances] characterization: a second instance created while the first exists does not get the port, the port stays reserved after it is destroyed") {
	// FW: UARTDevice(port) ignores the result of reservePort(); reservePort refuses a second device; the destructor of the
	// second one calls freePort(second), which refuses as well (UART.cpp:108, :137, :216-222).
	auto first = makeSm(1);
	{
		auto second = makeSm(2);
		CHECK(motor_uart.isReserved());
		PortProbe probe;
		CHECK_FALSE(probe.tryReserve());
	}
	CHECK(motor_uart.isReserved()); // the first one still owns the port
}

TEST_CASE_FIXTURE(SmFixture, "[R11.2 two instances] characterization: the second instance cannot complete a parameter transfer while the first owns the port") {
	// FW: the receive callback of the port goes to the owner (the first instance), so the second one's waiting flag is never
	// cleared and its settings sequence times out.
	auto first = makeSm(1);
	auto second = makeSm(2);
	CHECK_FALSE(initialize(*second));
	CHECK_FALSE(second->motorReady());
}

TEST_CASE_FIXTURE(SmFixture, "[R11.2 two instances] characterization: the first instance still initializes after a second instance was created and destroyed") {
	auto first = makeSm(1);
	{
		auto second = makeSm(2);
	}
	CHECK(initialize(*first));
	CHECK(first->motorReady());
}


// ---------------------------------------------------------------- failed transmit (lead, coverage of the abort paths)

TEST_CASE_FIXTURE(SmFixture, "[R3.8 fast] a rejected transmit of the fast update sends nothing, leaves the write enable pin low and the next fast update goes out") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	HostPlatform::uart().txResult = HAL_ERROR; // HAL_UART_Transmit_IT refuses the transfer (fake: nothing reaches the peer)
	sm->turn(10);
	CHECK(dev.frameCount() == 0);
	CHECK((GPIOE->ODR & DRV_GP1_Pin) == 0); // endUartTransfer(transmit) on the abort path
	HostPlatform::uart().txResult = HAL_OK;
	sm->turn(11);
	REQUIRE(dev.frameCount() == 1); // the port was given back on the abort path
	CHECK(dev.setpoints().back() == 11);
}

TEST_CASE_FIXTURE(SmFixture, "[R5.9 command] a rejected transmit of an instant command sends nothing, leaves the write enable pin low and the next command goes out") {
	auto sm = makeSm(1);
	REQUIRE(initialize(*sm));
	dev.clearFrames();
	HostPlatform::uart().txResult = HAL_ERROR;
	sm->startMotor(); // CB1 write; returns at once because sendCommand failed
	CHECK(dev.frameCount() == 0);
	CHECK((GPIOE->ODR & DRV_GP1_Pin) == 0);
	HostPlatform::uart().txResult = HAL_OK;
	sm->startMotor();
	REQUIRE(dev.frameCount() == 1);
	CHECK(dev.frames()[0].kind == FakeSmDevice::Kind::Instant);
	CHECK(dev.params[FakeSmDevice::SMP_CONTROL_BITS1] == 1);
}

}
