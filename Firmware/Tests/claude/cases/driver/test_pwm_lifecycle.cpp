/*
 * PWM driver, work order PWL (phase P14 T2): identity, chooser, singleton, defaults and life cycle of MotorPWM.
 * Requirements R1.1 - R1.4 and R6.1 - R6.3 of Tests/claude/charters/driver/PWM_REQUIREMENTS.md.
 *
 * Unit under test: MotorPWM::MotorPWM() / ~MotorPWM() / isCreatable() / getInfo() (MotorPWM.cpp:22-55), the entries
 * `add_class<MotorPWM, MotorDriver>(4)` of MotorDriver::all_drivers (MotorDriver.cpp:35) and of Axis::axis1_drivers /
 * Axis::axis2_drivers (Axis.cpp:56 / :83, public static members), and ClassChooser (the real chooser code).
 *
 * Oracles (charter): HD (hand derivations next to the assertions), HAL (the fake TIM1 behaves like stm32f4xx_hal_tim.c:
 * Stop clears CCxE and, when no channel remains enabled, MOE and CEN), IF (MotorDriver.h: getClassType is Motordriver),
 * FW for the chooser contract (ClassChooser::Create returns nullptr when isCreatable() is false). Identity values are
 * plain facts of the source: name "PWM" (MotorPWM.cpp:23), CLSID_MOT_PWM 0x84 (ClassIDs.h:75), selection id 4
 * (MotorDriver.cpp:35), help text (MotorPWM.h:80).
 *
 * Host vs target: nothing here depends on the word size, packing or byte order. The only arithmetic is the integer CCR of
 * the CENTERED mode (int32_t `pval * period`, the product for the values used here (|p| <= 1000, 33767 * 56000 = 1.89e9) fits int32_t on host and target; it does NOT fit
 * for power above 5580 at speed LOW (B-PWM-4, test_pwm_output.cpp), which no test of this file uses).
 * Hand derivation, period 56000 (3 kHz on 168 MHz), CCR = (0x7fff + p) * 56000 / 0xffff (integer division):
 *   p = 0      -> 32767 * 56000 / 65535 = 1834952000 / 65535 = 27999.57 -> 27999
 *   p = +1000  -> 33767 * 56000 / 65535 = 1890952000 / 65535 = 28854.0x -> 28854
 *   p = -1000  -> 31767 * 56000 / 65535 = 1778952000 / 65535 = 27145.x  -> 27145
 *
 * Characterizations of this file (NEEDS REVIEW, charter R6.3): the three cases named "characterization:" about two
 * instances created with `new` directly. KNOWN_ISSUEs: none in this file.
 *
 * Not tested here: R5.3 (no flash write at construction) belongs to the config group (PWC); the default state is checked
 * through the public interface only (getMode, getPwmSpeed, the `dir` command, the output gate of `turn`).
 */
#include "doctest.h"
#include "PwmFixture.h"

#include "Axis.h"
#include "ClassChooser.h"
#include "CommandHelpers.h"
#include "MotorDriver.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr uint16_t kPwmSelectionId = 4; // MotorDriver.cpp:35, Axis.cpp:57 / :84
constexpr uint16_t kPwmClassId = 0x84;  // CLSID_MOT_PWM, ClassIDs.h:75

/** Everything the TIM1 fake and the driver show: used to compare two set-ups */
struct SetUp {
	uint32_t arr, psc;
	uint32_t ccr[4];
	bool on[4];
	bool moe, cen;
	bool operator==(const SetUp &o) const {
		return arr == o.arr && psc == o.psc && std::equal(ccr, ccr + 4, o.ccr) && std::equal(on, on + 4, o.on) &&
			   moe == o.moe && cen == o.cen;
	}
};

SetUp snapshot() {
	SetUp s{};
	s.arr = PwmFixture::arr();
	s.psc = PwmFixture::psc();
	for (int ch = 1; ch <= 4; ++ch) {
		s.ccr[ch - 1] = PwmFixture::ccr(ch);
		s.on[ch - 1] = PwmFixture::channelOn(ch);
	}
	s.moe = PwmFixture::mainOutput();
	s.cen = PwmFixture::counterOn();
	return s;
}

/** The entries of a driver list with the given selection id */
std::vector<const class_entry<MotorDriver> *> entriesWithId(const std::vector<class_entry<MotorDriver>> &list, uint16_t id) {
	std::vector<const class_entry<MotorDriver> *> found;
	for (const auto &e : list) {
		if (e.selectionId == id) {
			found.push_back(&e);
		}
	}
	return found;
}

} // namespace

TEST_SUITE("PWM lifecycle") {

// ---------------------------------------------------------------- R1.1 identity

TEST_CASE_FIXTURE(PwmFixture, "[R1.1 identity] the PWM driver reports the name PWM and the class id 0x84") {
	auto pwm = makePwm();
	const ClassIdentifier id = pwm->getInfo();
	REQUIRE(id.name != nullptr);
	CHECK(std::string(id.name) == "PWM");
	CHECK(id.id == kPwmClassId);
	CHECK(MotorPWM::info.id == kPwmClassId); // the static identity used by the class registries
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.1 identity] the PWM driver is a motor driver class type when asked through the MotorDriver interface") {
	auto pwm = makePwm();
	MotorDriver *drv = pwm.get();
	CHECK(drv->getClassType() == ClassType::Motordriver);
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.1 identity] the help string of the PWM driver is the fixed text") {
	auto pwm = makePwm();
	CHECK(pwm->getHelpstring() == "PWM output motor driver");
}

// ---------------------------------------------------------------- R1.2 chooser lists

TEST_CASE_FIXTURE(PwmFixture, "[R1.2 chooser] MotorDriver::all_drivers holds the PWM driver exactly once under the id 4") {
	auto found = entriesWithId(MotorDriver::all_drivers, kPwmSelectionId);
	REQUIRE(found.size() == 1);
	CHECK(&found.front()->info == &MotorPWM::info);
	REQUIRE(found.front()->info.name != nullptr);
	CHECK(std::string(found.front()->info.name) == "PWM");
	CHECK(found.front()->info.id == kPwmClassId);
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.2 chooser] the driver lists of axis 1 and axis 2 each hold the PWM driver exactly once under the id 4") {
	for (const auto *list : {&Axis::axis1_drivers, &Axis::axis2_drivers}) {
		auto found = entriesWithId(*list, kPwmSelectionId);
		REQUIRE(found.size() == 1);
		CHECK(&found.front()->info == &MotorPWM::info);
		CHECK(found.front()->info.id == kPwmClassId);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.2 chooser] creating the id 4 from MotorDriver::all_drivers builds a MotorPWM that remembers its selection id") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	CHECK(chooser.isValidClassId(kPwmSelectionId));
	std::unique_ptr<MotorDriver> drv(chooser.Create(kPwmSelectionId));
	REQUIRE(drv != nullptr);
	CHECK(dynamic_cast<MotorPWM *>(drv.get()) != nullptr);
	CHECK(drv->getSelectionID() == kPwmSelectionId);
	CHECK(drv->getClassType() == ClassType::Motordriver);
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.2 chooser] creating the id 4 from the axis lists builds a MotorPWM") {
	for (const auto *list : {&Axis::axis1_drivers, &Axis::axis2_drivers}) {
		ClassChooser<MotorDriver> chooser(*list);
		std::unique_ptr<MotorDriver> drv(chooser.Create(kPwmSelectionId));
		REQUIRE(drv != nullptr);
		CHECK(dynamic_cast<MotorPWM *>(drv.get()) != nullptr);
		drv.reset(); // the singleton flag is released again for the second list
		CHECK(MotorPWM::isCreatable());
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.2 chooser] a driver created through the chooser outputs the neutral value of the default mode through the MotorDriver interface") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::unique_ptr<MotorDriver> drv(chooser.Create(kPwmSelectionId));
	REQUIRE(drv != nullptr);
	CHECK(arr() == 56000); // default 3 kHz set-up of the constructor (168000000 / 3000)
	CHECK(psc() == 0);
	drv->startMotor();
	CHECK(ccr(1) == 27999); // CENTERED neutral, hand derivation in the header comment
	drv->turn(1000);
	CHECK(ccr(1) == 28854);
	drv->stopMotor();
	CHECK(ccr(1) == 27999);
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.2 chooser] the chooser lists the PWM driver as id 4, creatable, with the name PWM") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::vector<CommandReply> replies;
	chooser.replyAvailableClasses(replies);
	bool found = false;
	for (const auto &r : replies) {
		if (r.reply == "4:1:PWM") {
			found = true;
		}
	}
	CHECK(found);
}

// ---------------------------------------------------------------- R1.3 singleton

TEST_CASE_FIXTURE(PwmFixture, "[R1.3 singleton] isCreatable is true without an object, false while one exists and true again after its destruction") {
	CHECK(MotorPWM::isCreatable());
	auto pwm = makePwm();
	CHECK_FALSE(MotorPWM::isCreatable());
	pwm.reset();
	CHECK(MotorPWM::isCreatable());
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.3 singleton] the chooser reports the id 4 creatable only while no PWM driver exists") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	CHECK(chooser.isCreatable(kPwmSelectionId));
	std::unique_ptr<MotorDriver> drv(chooser.Create(kPwmSelectionId));
	REQUIRE(drv != nullptr);
	CHECK_FALSE(chooser.isCreatable(kPwmSelectionId));
	std::vector<CommandReply> replies;
	chooser.replyAvailableClasses(replies);
	bool listedBusy = false;
	for (const auto &r : replies) {
		if (r.reply == "4:0:PWM") {
			listedBusy = true;
		}
	}
	CHECK(listedBusy); // "id:creatable:name" with creatable 0
	drv.reset();
	CHECK(chooser.isCreatable(kPwmSelectionId));
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.3 singleton] a second creation through the chooser while the first driver exists returns no object") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::unique_ptr<MotorDriver> first(chooser.Create(kPwmSelectionId));
	REQUIRE(first != nullptr);
	MotorDriver *second = chooser.Create(kPwmSelectionId);
	CHECK(second == nullptr);
	delete second; // nothing to delete if the chooser behaves, keeps the sanitizer build leak free if it does not
	CHECK(MotorPWM::pwmDriverInUse); // the first driver still holds the flag
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.3 singleton] a refused second creation through the chooser leaves the timer set-up of the first driver unchanged") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::unique_ptr<MotorDriver> first(chooser.Create(kPwmSelectionId));
	REQUIRE(first != nullptr);
	first->startMotor();
	first->turn(1000);
	const SetUp before = snapshot();
	MotorDriver *second = chooser.Create(kPwmSelectionId);
	CHECK(snapshot() == before);
	delete second;
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.3 singleton] the driver can be created through the chooser again after the first one was destroyed") {
	ClassChooser<MotorDriver> chooser(MotorDriver::all_drivers);
	std::unique_ptr<MotorDriver> first(chooser.Create(kPwmSelectionId));
	REQUIRE(first != nullptr);
	first.reset();
	std::unique_ptr<MotorDriver> again(chooser.Create(kPwmSelectionId));
	REQUIRE(again != nullptr);
	CHECK(dynamic_cast<MotorPWM *>(again.get()) != nullptr);
	CHECK_FALSE(MotorPWM::isCreatable());
}

// ---------------------------------------------------------------- R1.4 defaults

TEST_CASE_FIXTURE(PwmFixture, "[R1.4 defaults] on an empty flash the mode is CENTERED") {
	auto pwm = makePwm();
	CHECK(pwm->getMode() == ModePWM_DRV::CENTERED_PWM);
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.4 defaults] on an empty flash the speed is LOW") {
	auto pwm = makePwm();
	CHECK(pwm->getPwmSpeed() == SpeedPWM_DRV::LOW);
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.4 defaults] on an empty flash the direction is not inverted") {
	auto pwm = makePwm();
	CHECK(CommandHelpers::get(*pwm, "dir") == 0);
	// the same through the output: CENTERED, +1000 is above the neutral 27999 (inverted it would be 27145)
	pwm->startMotor();
	pwm->turn(1000);
	CHECK(ccr(1) == 28854);
}

TEST_CASE_FIXTURE(PwmFixture, "[R1.4 defaults] on an empty flash the driver is not active so turn writes nothing") {
	auto pwm = makePwm();
	REQUIRE(ccr(1) == 0); // constructor: turn(0) is gated, ConfigChannel loaded Pulse 0
	pwm->turn(20000);
	pwm->turn(-20000);
	for (int ch = 1; ch <= 4; ++ch) {
		CHECK(ccr(ch) == 0);
	}
	pwm->startMotor(); // activates: now the neutral value appears
	CHECK(ccr(1) == 27999);
}

// ---------------------------------------------------------------- R6.1 destruction

TEST_CASE_FIXTURE(PwmFixture, "[R6.1 destruction] destroying a default driver clears all four channel enables, the main output and the counter") {
	auto pwm = makePwm();
	REQUIRE(counterOn()); // precondition: the construction started everything (R2.1)
	pwm.reset();
	for (int ch = 1; ch <= 4; ++ch) {
		CHECK_FALSE(channelOn(ch));
	}
	CHECK_FALSE(mainOutput());
	CHECK_FALSE(counterOn());
}

TEST_CASE_FIXTURE(PwmFixture, "[R6.1 destruction] destroying a started driver in dual PWM mode clears all four channel enables, the main output and the counter") {
	auto pwm = makePwm();
	pwm->setMode(ModePWM_DRV::PWM_DUAL);
	pwm->startMotor();
	pwm->turn(10000);
	REQUIRE(ccr(1) != 0); // the output is active
	pwm.reset();
	for (int ch = 1; ch <= 4; ++ch) {
		CHECK_FALSE(channelOn(ch));
	}
	CHECK_FALSE(mainOutput());
	CHECK_FALSE(counterOn());
}

TEST_CASE_FIXTURE(PwmFixture, "[R6.1 destruction] destroying the driver releases the singleton flag") {
	auto pwm = makePwm();
	REQUIRE(MotorPWM::pwmDriverInUse);
	pwm.reset();
	CHECK_FALSE(MotorPWM::pwmDriverInUse);
}

// ---------------------------------------------------------------- R6.2 create / destroy / create

TEST_CASE_FIXTURE(PwmFixture, "[R6.2 recreate] every create, destroy, create cycle gives the same timer set-up as the first creation") {
	SetUp first{};
	{
		auto pwm = makePwm();
		first = snapshot();
	}
	for (int cycle = 0; cycle < 3; ++cycle) {
		auto pwm = makePwm();
		CHECK(snapshot() == first);
		CHECK(MotorPWM::pwmDriverInUse);
		CHECK(pwm->getMode() == ModePWM_DRV::CENTERED_PWM);
		CHECK(pwm->getPwmSpeed() == SpeedPWM_DRV::LOW);
		pwm.reset();
		CHECK_FALSE(MotorPWM::pwmDriverInUse);
	}
}

TEST_CASE_FIXTURE(PwmFixture, "[R6.2 recreate] a new driver after a destroyed configured and started one starts from the defaults, nothing leaks but the flash") {
	{
		auto old = makePwm();
		old->setMode(ModePWM_DRV::PWM_DIR);
		old->setPwmSpeed(SpeedPWM_DRV::HIGH);
		REQUIRE(CommandHelpers::run(*old, "dir", CMDtype::set, 1) == CommandStatus::OK);
		old->startMotor();
		old->turn(-5000);
		REQUIRE(ccr(3) != ccr(4)); // the output of the old driver is non-neutral
		// no saveFlash(): nothing is stored
	}
	auto pwm = makePwm();
	CHECK(pwm->getMode() == ModePWM_DRV::CENTERED_PWM);
	CHECK(pwm->getPwmSpeed() == SpeedPWM_DRV::LOW);
	CHECK(CommandHelpers::get(*pwm, "dir") == 0);
	CHECK(arr() == 56000); // 168000000 / 3000
	CHECK(psc() == 0);
	for (int ch = 1; ch <= 4; ++ch) {
		CHECK(ccr(ch) == 0); // ConfigChannel loads Pulse 0 into every channel
		CHECK(channelOn(ch));
	}
	CHECK(mainOutput());
	CHECK(counterOn());
	pwm->turn(20000); // an inactive new driver does not inherit the active flag
	CHECK(ccr(1) == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R6.2 recreate] a stored configuration is restored by the next driver with the matching timer period") {
	{
		auto old = makePwm();
		old->setMode(ModePWM_DRV::PWM_DUAL);
		old->setPwmSpeed(SpeedPWM_DRV::VERYHIGH);
		old->saveFlash();
	}
	auto pwm = makePwm();
	CHECK(pwm->getMode() == ModePWM_DRV::PWM_DUAL);
	CHECK(pwm->getPwmSpeed() == SpeedPWM_DRV::VERYHIGH);
	CHECK(arr() == 7000); // 168000000 / 24000
	CHECK(psc() == 0);
}

// ---------------------------------------------------------------- R6.3 two instances created with new

TEST_CASE_FIXTURE(PwmFixture, "[R6.3 two instances] characterization: a second instance created with new leaves the same timer registers as one instance") {
	// NEEDS REVIEW: the constructor does not enforce the singleton, only the chooser does (MotorPWM.cpp:35-46)
	std::unique_ptr<MotorPWM> first(new MotorPWM());
	const SetUp one = snapshot();
	std::unique_ptr<MotorPWM> second(new MotorPWM());
	CHECK(snapshot() == one);
	CHECK(arr() == 56000);
	CHECK(psc() == 0);
	CHECK(MotorPWM::pwmDriverInUse);
	CHECK(HostTarget::state().unmodelledHalCalls.empty()); // Stop, Config, Start in order is accepted by the strict fake
}

TEST_CASE_FIXTURE(PwmFixture, "[R6.3 two instances] characterization: the constructor of a second instance resets the output of the first started instance to 0") {
	// NEEDS REVIEW: both objects share TIM1; pwmInitTimer loads Pulse 0 into every channel
	std::unique_ptr<MotorPWM> first(new MotorPWM());
	first->startMotor();
	first->turn(1000);
	REQUIRE(ccr(1) == 28854);
	std::unique_ptr<MotorPWM> second(new MotorPWM());
	CHECK(ccr(1) == 0);
}

TEST_CASE_FIXTURE(PwmFixture, "[R6.3 two instances] characterization: destroying the first of two instances switches the shared outputs off and releases the flag, the second one is deleted without error") {
	// NEEDS REVIEW: destruction acts on the shared timer and the shared static flag whatever instance is left
	std::unique_ptr<MotorPWM> first(new MotorPWM());
	std::unique_ptr<MotorPWM> second(new MotorPWM());
	first.reset();
	CHECK_FALSE(MotorPWM::pwmDriverInUse);
	for (int ch = 1; ch <= 4; ++ch) {
		CHECK_FALSE(channelOn(ch));
	}
	CHECK_FALSE(mainOutput());
	CHECK_FALSE(counterOn());
	second.reset(); // Stop on stopped channels is accepted by the fake, no leak in the sanitizer build
	CHECK_FALSE(MotorPWM::pwmDriverInUse);
	CHECK(HostTarget::state().unmodelledHalCalls.empty());
}

}
