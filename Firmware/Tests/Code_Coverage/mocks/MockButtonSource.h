/*
 * MockButtonSource.h
 *
 * Button source implementing the ButtonSource class interface without hardware (ffb_target_tests). The test sets the
 * button bits and the amount of buttons; the mock records how the firmware reads it.
 *
 * Selectable through a ClassChooser<ButtonSource> like the real sources: registry() is the firmware's registry
 * (ButtonSource::all_buttonsources of ButtonSources.cpp) plus the mock with selection id SELECTION_ID (15, unused by
 * the firmware: 0..5). Use with SelectableInputs / FFBHIDMain through support/InputSources.h.
 * Every created mock is listed in instances() (in creation order) until it is deleted.
 */
#ifndef HOSTTEST_MOCKBUTTONSOURCE_H_
#define HOSTTEST_MOCKBUTTONSOURCE_H_

#include "ButtonSource.h"
#include "ClassChooser.h"

#include <algorithm>
#include <cstdint>
#include <vector>

class MockButtonSource : public ButtonSource {
public:
	static constexpr uint16_t SELECTION_ID = 15; //!< Selection id in registry() (btntypes bit 15)
	static constexpr uint16_t CLASS_ID = 0x2F;	 //!< getInfo().id: in the button source range, unused by the firmware (0x20..0x25)

	MockButtonSource() {
		btnnum = 8;
		instances().push_back(this);
	}
	~MockButtonSource() override {
		auto &list = instances();
		list.erase(std::remove(list.begin(), list.end(), this), list.end());
	}

	static inline ClassIdentifier info = {.name = "Mock buttons", .id = CLASS_ID, .visibility = ClassVisibility::visible};
	const ClassIdentifier getInfo() override { return info; }
	static bool isCreatable() { return creatable(); }

	// --- ButtonSource interface ---
	/** Writes `buttons` into *buf (no mask: the test decides the bits) and returns `returnedCount` (default: btnnum) */
	uint8_t readButtons(uint64_t *buf) override {
		readCalls++;
		*buf = buttons;
		return returnedCount >= 0 ? static_cast<uint8_t>(returnedCount) : static_cast<uint8_t>(btnnum);
	}
	void saveFlash() override { saveCalls++; }
	void restoreFlash() override { restoreCalls++; }

	// --- Test helpers ---
	/** Amount of buttons reported by getBtnNum() and returned by readButtons() */
	void setButtonCount(uint16_t count) { btnnum = count; }

	/** Class registry entry for a ClassChooser<ButtonSource> */
	static class_entry<ButtonSource> classEntry(uint16_t selectionId = SELECTION_ID) { return add_class<MockButtonSource, ButtonSource>(selectionId); }
	/** The firmware's button source registry plus the mock (selection id SELECTION_ID) */
	static const std::vector<class_entry<ButtonSource>> &registry() {
		static const std::vector<class_entry<ButtonSource>> list = [] {
			std::vector<class_entry<ButtonSource>> entries = ButtonSource::all_buttonsources;
			entries.push_back(classEntry());
			return entries;
		}();
		return list;
	}

	/** Existing mocks in creation order */
	static std::vector<MockButtonSource *> &instances() {
		static std::vector<MockButtonSource *> list;
		return list;
	}
	/** The most recently created existing mock, nullptr if none */
	static MockButtonSource *last() { return instances().empty() ? nullptr : instances().back(); }
	/** Result of isCreatable(). Simulates occupied resources */
	static bool &creatable() {
		static bool value = true;
		return value;
	}

	// --- Controlled by the test ---
	uint64_t buttons = 0;	   //!< Bits written by readButtons
	int returnedCount = -1;	   //!< Return value of readButtons; -1: btnnum

	// --- Recorded ---
	uint32_t readCalls = 0;
	uint32_t saveCalls = 0;
	uint32_t restoreCalls = 0;
};

#endif
