/*
 * Tests of SelectableInputs (SelectableInputs.cpp: the input source management of the main classes), the ButtonSource
 * base class (ButtonSource.cpp) and the button source registry (ButtonSources.cpp) in ffb_target_tests.
 * SelectableInputs is tested directly with its own class choosers: registries of MockButtonSource / MockAnalogSource
 * entries with chosen selection ids (the main classes use it through FFBHIDMain, see test_ffbhidmain.cpp).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "ButtonSource.h"
#include "CanButtons.h"
#include "LocalButtons.h"
#include "PCF8574.h"
#include "SPIButtons.h"
#include "SelectableInputs.h"
#include "ShifterAnalog.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace {

/** Button registry: mocks with the selection ids 2, 9 and (class id) 0x2F */
const std::vector<class_entry<ButtonSource>> &buttonRegistry() {
	static const std::vector<class_entry<ButtonSource>> registry = {MockButtonSource::classEntry(2), MockButtonSource::classEntry(9),
																	MockButtonSource::classEntry(MockButtonSource::CLASS_ID)};
	return registry;
}
/** Analog registry: mocks with the selection ids 3, 7 and (class id) 0x4F */
const std::vector<class_entry<AnalogSource>> &analogRegistry() {
	static const std::vector<class_entry<AnalogSource>> registry = {MockAnalogSource::classEntry(3), MockAnalogSource::classEntry(7),
																	MockAnalogSource::classEntry(MockAnalogSource::CLASS_ID)};
	return registry;
}

struct InputsFixture : TargetFixture {
	std::unique_ptr<SelectableInputs> inputs = std::make_unique<SelectableInputs>(ClassChooser<ButtonSource>(buttonRegistry()),
																				  ClassChooser<AnalogSource>(analogRegistry()));
	static std::vector<MockButtonSource *> &buttonMocks() { return MockButtonSource::instances(); }
	static std::vector<MockAnalogSource *> &analogMocks() { return MockAnalogSource::instances(); }
	static MockButtonSource &button(size_t i) {
		REQUIRE(i < buttonMocks().size());
		return *buttonMocks()[i];
	}
	static MockAnalogSource &analog(size_t i) {
		REQUIRE(i < analogMocks().size());
		return *analogMocks()[i];
	}
};

/** Smallest possible button source: only the pure virtual functions */
struct MinimalButtonSource : ButtonSource {
	uint8_t readButtons(uint64_t *buf) override {
		*buf = 0;
		return 0;
	}
	const ClassIdentifier getInfo() override { return ButtonSource::info; }
};

} // namespace

TEST_SUITE("SelectableInputs") {

// ---------------------------------------------------------------- button sources

TEST_CASE_FIXTURE(InputsFixture, "setBtnTypes creates one button source for every set bit in selection id order") {
	inputs->setBtnTypes((1 << 9) | (1 << 2));

	REQUIRE(buttonMocks().size() == 2);
	CHECK(button(0).getSelectionID() == 2);
	CHECK(button(1).getSelectionID() == 9);
}

TEST_CASE_FIXTURE(InputsFixture, "setBtnTypes skips bits without a creatable source") {
	inputs->setBtnTypes((1 << 0) | (1 << 2)); // no source with selection id 0
	CHECK(buttonMocks().size() == 1);

	MockButtonSource::creatable() = false;
	inputs->setBtnTypes(1 << 9);
	CHECK(buttonMocks().empty());
}

TEST_CASE_FIXTURE(InputsFixture, "setBtnTypes replaces the existing button sources") {
	inputs->setBtnTypes(1 << 2);
	REQUIRE(buttonMocks().size() == 1);

	inputs->setBtnTypes(1 << 9);

	REQUIRE(buttonMocks().size() == 1);
	CHECK(button(0).getSelectionID() == 9);
}

TEST_CASE_FIXTURE(InputsFixture, "setBtnTypes 0 and clearBtnTypes delete every button source") {
	inputs->setBtnTypes((1 << 2) | (1 << 9));
	inputs->setBtnTypes(0);
	CHECK(buttonMocks().empty());

	inputs->setBtnTypes((1 << 2) | (1 << 9));
	inputs->clearBtnTypes();
	CHECK(buttonMocks().empty());
}

TEST_CASE_FIXTURE(InputsFixture, "getButtonValues places the buttons of each source after those of the previous source") {
	inputs->setBtnTypes((1 << 2) | (1 << 9));
	button(0).setButtonCount(5);
	button(0).buttons = 0b10101;
	button(1).setButtonCount(3);
	button(1).buttons = 0b011;
	uint64_t values = 0;

	CHECK(inputs->getButtonValues(values) == 8);

	CHECK(values == 0b01110101);
	CHECK(button(0).readCalls == 1);
	CHECK(button(1).readCalls == 1);
}

TEST_CASE_FIXTURE(InputsFixture, "getButtonValues uses the count returned by readButtons as offset of the next source") {
	inputs->setBtnTypes((1 << 2) | (1 << 9));
	button(0).setButtonCount(8);
	button(0).returnedCount = 2;
	button(0).buttons = 0b11;
	button(1).buttons = 0b1;

	uint64_t values = 0;
	CHECK(inputs->getButtonValues(values) == 2 + 8);
	CHECK(values == 0b111);
}

TEST_CASE_FIXTURE(InputsFixture, "getButtonValues ORs the buttons into the given value") {
	inputs->setBtnTypes(1 << 2);
	button(0).buttons = 0b1;
	uint64_t values = 0x100;
	inputs->getButtonValues(values);
	CHECK(values == 0x101);
}

TEST_CASE_FIXTURE(InputsFixture, "getButtonValues without button sources reports no buttons") {
	uint64_t values = 0;
	CHECK(inputs->getButtonValues(values) == 0);
	CHECK(values == 0);
}

TEST_CASE_FIXTURE(InputsFixture, "two sources of 32 buttons fill all 64 buttons") {
	inputs->setBtnTypes((1 << 2) | (1 << 9));
	button(0).setButtonCount(32);
	button(0).buttons = 0x80000001;
	button(1).setButtonCount(32);
	button(1).buttons = 0x80000001;
	uint64_t values = 0;

	CHECK(inputs->getButtonValues(values) == 64);
	CHECK(values == 0x8000000180000001);
}

TEST_CASE_FIXTURE(InputsFixture, "buttons of a source beyond the 64th button do not change the first buttons" *
									  KNOWN_ISSUE_UB("(bug 73) getButtonValues shifts the buttons of each source by the sum of the previous counts "
													 "(SelectableInputs.cpp:95) without a limit: with 64 previous buttons the shift is 64, "
													 "undefined for uint64_t (x86 shifts by 0 and ORs the buttons onto buttons 1-8, ARM gives 0). "
													 "The uint8_t shift also wraps above 255 buttons. The CHECK fails on x86")) {
	inputs->setBtnTypes((1 << 2) | (1 << 9));
	button(0).setButtonCount(64);
	button(0).buttons = 0;
	button(1).setButtonCount(8);
	button(1).buttons = 0xFF;
	uint64_t values = 0;
	inputs->getButtonValues(values);
	REQUIRE_ALL_HAL_MODELLED();

	CHECK(values == 0);
}

TEST_CASE_FIXTURE(InputsFixture, "addBtnType adds a button source after the existing ones") {
	inputs->setBtnTypes(1 << 9);

	inputs->addBtnType(2);

	REQUIRE(buttonMocks().size() == 2);
	CHECK(button(1).getSelectionID() == 2);
	button(0).setButtonCount(1);
	button(1).buttons = 0b1;
	uint64_t values = 0;
	inputs->getButtonValues(values);
	CHECK(values == 0b10); // the added source comes second
}

TEST_CASE_FIXTURE(InputsFixture, "addBtnType adds nothing for an id without a creatable source") {
	inputs->addBtnType(4);
	CHECK(buttonMocks().empty());
	MockButtonSource::creatable() = false;
	inputs->addBtnType(2);
	CHECK(buttonMocks().empty());
}

TEST_CASE_FIXTURE(InputsFixture, "addBtnType does not add a second source whose class id equals the requested id") {
	// The duplicate check compares getInfo().id with the requested selection id (bug 38): it only works when both
	// are equal, as for this registry entry
	inputs->addBtnType(MockButtonSource::CLASS_ID);
	REQUIRE(buttonMocks().size() == 1);

	inputs->addBtnType(MockButtonSource::CLASS_ID);

	CHECK(buttonMocks().size() == 1);
}

// ---------------------------------------------------------------- analog sources

TEST_CASE_FIXTURE(InputsFixture, "setAinTypes creates one analog source for every set bit in selection id order") {
	inputs->setAinTypes((1 << 7) | (1 << 3) | (1 << 1));

	REQUIRE(analogMocks().size() == 2);
	CHECK(analog(0).getSelectionID() == 3);
	CHECK(analog(1).getSelectionID() == 7);
}

TEST_CASE_FIXTURE(InputsFixture, "getAnalogValues returns the axes of all analog sources in order") {
	inputs->setAinTypes((1 << 3) | (1 << 7));
	analog(0).axes = {1, -2};
	analog(1).axes = {300};

	CHECK(*inputs->getAnalogValues() == std::vector<int32_t>{1, -2, 300});
	analog(1).axes = {};
	CHECK(*inputs->getAnalogValues() == std::vector<int32_t>{1, -2});
}

TEST_CASE_FIXTURE(InputsFixture, "getAnalogValues without analog sources returns no values") {
	CHECK(inputs->getAnalogValues()->empty());
}

TEST_CASE_FIXTURE(InputsFixture, "setAinTypes replaces the analog sources and setAinTypes 0 and clearAinTypes delete them") {
	inputs->setAinTypes(1 << 3);
	inputs->setAinTypes(1 << 7);
	REQUIRE(analogMocks().size() == 1);
	CHECK(analog(0).getSelectionID() == 7);

	inputs->setAinTypes(0);
	CHECK(analogMocks().empty());

	inputs->setAinTypes(1 << 3);
	inputs->clearAinTypes();
	CHECK(analogMocks().empty());
}

TEST_CASE_FIXTURE(InputsFixture, "setAinTypes skips sources that are not creatable") {
	MockAnalogSource::creatable() = false;
	inputs->setAinTypes(1 << 3);
	CHECK(analogMocks().empty());
}

TEST_CASE_FIXTURE(InputsFixture, "addAinType adds an analog source after the existing ones") {
	inputs->setAinTypes(1 << 7);
	inputs->addAinType(3);
	REQUIRE(analogMocks().size() == 2);
	analog(0).axes = {7};
	analog(1).axes = {3};
	CHECK(*inputs->getAnalogValues() == std::vector<int32_t>{7, 3});
}

TEST_CASE_FIXTURE(InputsFixture, "addAinType adds nothing for an id without a creatable source") {
	inputs->addAinType(5);
	CHECK(analogMocks().empty());
}

TEST_CASE_FIXTURE(InputsFixture, "addAinType does not add a second source whose class id equals the requested id") {
	inputs->addAinType(MockAnalogSource::CLASS_ID);
	inputs->addAinType(MockAnalogSource::CLASS_ID);
	CHECK(analogMocks().size() == 1);
}

TEST_CASE_FIXTURE(InputsFixture, "deleting the selectable inputs deletes all button and analog sources") {
	inputs->setBtnTypes((1 << 2) | (1 << 9));
	inputs->setAinTypes(1 << 3);
	REQUIRE(buttonMocks().size() == 2);
	REQUIRE(analogMocks().size() == 1);

	inputs.reset();

	CHECK(buttonMocks().empty());
	CHECK(analogMocks().empty());
}

} // TEST_SUITE

TEST_SUITE("ButtonSource") {

TEST_CASE_FIXTURE(TargetFixture, "a button source reports no buttons until it sets its button count") {
	MinimalButtonSource source;
	CHECK(source.getBtnNum() == 0);
}

TEST_CASE_FIXTURE(TargetFixture, "getBtnNum returns the button count of the source") {
	MockButtonSource source;
	source.setButtonCount(12);
	CHECK(source.getBtnNum() == 12);
}

TEST_CASE_FIXTURE(TargetFixture, "a button source is a creatable button source class") {
	MinimalButtonSource source;
	CHECK(source.getClassType() == ClassType::Buttonsource);
	CHECK(ButtonSource::isCreatable());
}

TEST_CASE_FIXTURE(TargetFixture, "the base button source info is the hidden class NONE with id 0") {
	CHECK(std::string(ButtonSource::info.name) == "NONE");
	CHECK(ButtonSource::info.id == 0);
	CHECK(ButtonSource::info.visibility == ClassVisibility::hidden);
}

} // TEST_SUITE

TEST_SUITE("ButtonSources") {

TEST_CASE("the F407 button source registry offers local, SPI 1, shifter, PCF8574 and CAN buttons with their selection ids") {
	std::vector<std::pair<uint16_t, uint16_t>> entries; // selection id, class id
	for (const class_entry<ButtonSource> &e : ButtonSource::all_buttonsources) {
		entries.emplace_back(e.selectionId, e.info.id);
	}
	CHECK(entries == std::vector<std::pair<uint16_t, uint16_t>>{
						 {0, CLSID_BTN_LOCAL}, {1, CLSID_BTN_SPI}, {3, CLSID_BTN_SHIFTER}, {4, CLSID_BTN_PCF}, {5, CLSID_BTN_CAN}});
}

TEST_CASE("the selection ids and class ids of the button source registry are unique and selection ids fit the 16 bit btntypes mask") {
	std::set<uint16_t> selectionIds;
	std::set<uint16_t> classIds;
	for (const class_entry<ButtonSource> &e : ButtonSource::all_buttonsources) {
		CHECK(e.selectionId < 16);
		selectionIds.insert(e.selectionId);
		classIds.insert(e.info.id);
	}
	CHECK(selectionIds.size() == ButtonSource::all_buttonsources.size());
	CHECK(classIds.size() == ButtonSource::all_buttonsources.size());
}

TEST_CASE_FIXTURE(TargetFixture, "every registered button source is creatable and creates its class with the selection id") {
	ClassChooser<ButtonSource> chooser(ButtonSource::all_buttonsources);
	for (const class_entry<ButtonSource> &e : ButtonSource::all_buttonsources) {
		CAPTURE(e.selectionId);
		CHECK(chooser.isCreatable(e.selectionId));
		std::unique_ptr<ButtonSource> source(chooser.Create(e.selectionId));
		REQUIRE(source != nullptr);
		CHECK(source->getInfo().id == e.info.id);
		CHECK(std::string(source->getInfo().name) == e.info.name);
		CHECK(source->getSelectionID() == e.selectionId);
		CHECK(source->getClassType() == ClassType::Buttonsource);
	}
}

TEST_CASE_FIXTURE(TargetFixture, "the registry has no button source for the unused selection ids") {
	ClassChooser<ButtonSource> chooser(ButtonSource::all_buttonsources);
	CHECK_FALSE(chooser.isValidClassId(2)); // SPI buttons 2: SPIBUTTONS2 is not defined for the F407
	CHECK(chooser.Create(2) == nullptr);
	CHECK(chooser.Create(15) == nullptr);
}

} // TEST_SUITE
