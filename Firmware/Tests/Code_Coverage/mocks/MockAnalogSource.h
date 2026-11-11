/*
 * MockAnalogSource.h
 *
 * Analog source implementing the AnalogSource class interface without hardware (ffb_target_tests). The test sets the
 * axis values; the mock records how the firmware reads it.
 *
 * Selectable through a ClassChooser<AnalogSource> like the real sources: registry() is the firmware's registry
 * (AnalogSource::all_analogsources of AnalogSources.cpp) plus the mock with selection id SELECTION_ID (15, unused by
 * the firmware: 0..2). Use with SelectableInputs / FFBHIDMain through support/InputSources.h.
 * Every created mock is listed in instances() (in creation order) until it is deleted.
 */
#ifndef HOSTTEST_MOCKANALOGSOURCE_H_
#define HOSTTEST_MOCKANALOGSOURCE_H_

#include "AnalogSource.h"
#include "ClassChooser.h"

#include <algorithm>
#include <cstdint>
#include <vector>

class MockAnalogSource : public AnalogSource {
public:
	static constexpr uint16_t SELECTION_ID = 15; //!< Selection id in registry() (aintypes bit 15)
	static constexpr uint16_t CLASS_ID = 0x4F;	 //!< getInfo().id: in the analog source range, unused by the firmware (0x40..0x43)

	MockAnalogSource() { instances().push_back(this); }
	~MockAnalogSource() override {
		auto &list = instances();
		list.erase(std::remove(list.begin(), list.end(), this), list.end());
	}

	static inline ClassIdentifier info = {.name = "Mock analog", .id = CLASS_ID, .visibility = ClassVisibility::visible};
	const ClassIdentifier getInfo() override { return info; }
	static bool isCreatable() { return creatable(); }

	// --- AnalogSource interface ---
	/** Copies `axes` into the public buffer `buf` of AnalogSource and returns it (like the real sources do) */
	std::vector<int32_t> *getAxes() override {
		getAxesCalls++;
		buf = axes;
		return &buf;
	}
	void saveFlash() override { saveCalls++; }
	void restoreFlash() override { restoreCalls++; }

	/** Class registry entry for a ClassChooser<AnalogSource> */
	static class_entry<AnalogSource> classEntry(uint16_t selectionId = SELECTION_ID) { return add_class<MockAnalogSource, AnalogSource>(selectionId); }
	/** The firmware's analog source registry plus the mock (selection id SELECTION_ID) */
	static const std::vector<class_entry<AnalogSource>> &registry() {
		static const std::vector<class_entry<AnalogSource>> list = [] {
			std::vector<class_entry<AnalogSource>> entries = AnalogSource::all_analogsources;
			entries.push_back(classEntry());
			return entries;
		}();
		return list;
	}

	/** Existing mocks in creation order */
	static std::vector<MockAnalogSource *> &instances() {
		static std::vector<MockAnalogSource *> list;
		return list;
	}
	/** The most recently created existing mock, nullptr if none */
	static MockAnalogSource *last() { return instances().empty() ? nullptr : instances().back(); }
	/** Result of isCreatable(). Simulates occupied resources */
	static bool &creatable() {
		static bool value = true;
		return value;
	}

	// --- Controlled by the test ---
	std::vector<int32_t> axes; //!< Values returned by getAxes (one per axis)

	// --- Recorded ---
	uint32_t getAxesCalls = 0;
	uint32_t saveCalls = 0;
	uint32_t restoreCalls = 0;
};

#endif
