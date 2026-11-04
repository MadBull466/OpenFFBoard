/*
 * AnalogSource (FFBoard/Src/AnalogSource.cpp): the base class of the analog sources (defaults: class info NONE, the
 * public axis buffer returned by getAxes, flash handler registration) and AnalogSources.cpp: the registry
 * AnalogSource::all_analogsources of the F407 feature set (LocalAnalog 0, CanAnalog<8> 1, ADS111X 2).
 */
#include "doctest.h"
#include "AxisFixture.h"
#include "TestHelpers.h"

#include "ADS111X.h"
#include "AnalogSource.h"
#include "CanAnalog.h"
#include "ClassChooser.h"
#include "LocalAnalog.h"

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace {

/** The smallest analog source: only the pure virtual getInfo */
struct PlainSource : AnalogSource {
	const ClassIdentifier getInfo() override { return AnalogSource::info; }
};

/** Analog source that records that its own destructor ran */
struct FlaggedSource : AnalogSource {
	explicit FlaggedSource(bool &destroyed) : destroyed(destroyed) {}
	~FlaggedSource() override { destroyed = true; }
	const ClassIdentifier getInfo() override { return AnalogSource::info; }
	bool &destroyed;
};

} // namespace

TEST_SUITE("AnalogSource") {

TEST_CASE_FIXTURE(TargetFixture, "the analog source base class is the hidden class NONE with id 0x40") {
	CHECK(std::string(AnalogSource::info.name) == "NONE");
	CHECK(AnalogSource::info.id == CLSID_ANALOG_NONE);
	CHECK(AnalogSource::info.id == 0x40);
	CHECK(AnalogSource::info.visibility == ClassVisibility::hidden);
}

TEST_CASE_FIXTURE(TargetFixture, "by default getAxes returns the public axis buffer of the source") {
	PlainSource source;
	CHECK(source.getAxes() == &source.buf);
	CHECK(source.getAxes()->empty());
	source.buf = {1, -2};
	CHECK(*source.getAxes() == std::vector<int32_t>{1, -2});
}

TEST_CASE_FIXTURE(TargetFixture, "an analog source is creatable and of the class type analog source") {
	PlainSource source;
	CHECK(AnalogSource::isCreatable());
	CHECK(source.getClassType() == ClassType::Analogsource);
}

TEST_CASE_FIXTURE(TargetFixture, "an analog source is a flash handler while it exists") {
	auto &handlers = PersistentStorage::getFlashHandlers();
	const size_t before = handlers.size();
	{
		PlainSource source;
		CHECK(handlers.size() == before + 1);
		CHECK(std::count(handlers.begin(), handlers.end(), static_cast<PersistentStorage *>(&source)) == 1);
	}
	CHECK(handlers.size() == before);
}

TEST_CASE_FIXTURE(TargetFixture, "an analog source deleted through a base class pointer runs the destructor of the source") {
	const size_t before = PersistentStorage::getFlashHandlers().size();
	bool destroyed = false;
	AnalogSource *source = new FlaggedSource(destroyed);
	delete source;
	CHECK(destroyed);
	CHECK(PersistentStorage::getFlashHandlers().size() == before);
}

} // TEST_SUITE

TEST_SUITE("AnalogSources") {

TEST_CASE("the analog source registry lists the local pins, CAN and ADS111X sources with selection ids 0 to 2") {
	const auto &list = AnalogSource::all_analogsources;
	REQUIRE(list.size() == 3);
	CHECK(list[0].selectionId == 0);
	CHECK(list[0].info.id == CLSID_ANALOG_LOCAL);
	CHECK(list[1].selectionId == 1);
	CHECK(list[1].info.id == CLSID_ANALOG_CAN);
	CHECK(list[2].selectionId == 2);
	CHECK(list[2].info.id == CLSID_ANALOG_ADS111X);
}

TEST_CASE("the selection ids of the analog source registry are unique and fit the 16 bit source mask") {
	std::set<uint16_t> ids;
	for (const auto &e : AnalogSource::all_analogsources) {
		CAPTURE(e.selectionId);
		CHECK(e.selectionId < 16);
		CHECK(ids.insert(e.selectionId).second);
	}
}

TEST_CASE("the class ids and names of the analog source registry are unique") {
	std::set<uint16_t> ids;
	std::set<std::string> names;
	for (const auto &e : AnalogSource::all_analogsources) {
		CAPTURE(e.info.name);
		CHECK(ids.insert(e.info.id).second);
		CHECK(names.insert(e.info.name).second);
		CHECK(e.info.id != CLSID_ANALOG_NONE);
		CHECK(e.info.visibility != ClassVisibility::hidden);
	}
}

TEST_CASE_FIXTURE(TargetFixture, "every source of the analog source registry is creatable and created with its class info") {
	for (const auto &e : AnalogSource::all_analogsources) {
		CAPTURE(e.info.name);
		REQUIRE(e.isCreatable());
		std::unique_ptr<AnalogSource> source(e.create());
		REQUIRE(source != nullptr);
		CHECK(source->getInfo().id == e.info.id);
		CHECK(source->getClassType() == ClassType::Analogsource);
	}
}

TEST_CASE_FIXTURE(TargetFixture, "a class chooser on the analog source registry creates the source of a selection id") {
	ClassChooser<AnalogSource> chooser(AnalogSource::all_analogsources);
	for (uint16_t id = 0; id < 3; id++) {
		CAPTURE(id);
		CHECK(chooser.isValidClassId(id));
		std::unique_ptr<AnalogSource> source(chooser.Create(id));
		REQUIRE(source != nullptr);
		CHECK(source->getInfo().id == AnalogSource::all_analogsources[id].info.id);
	}
	CHECK_FALSE(chooser.isValidClassId(3));
	CHECK(chooser.Create(3) == nullptr);
}

} // TEST_SUITE
