/*
 * InputSources.h
 *
 * Helpers of ffb_target_tests to select MockButtonSource / MockAnalogSource through the real class choosers of
 * SelectableInputs (used by FFBHIDMain: btntypes / aintypes / addbtn / addain commands), like AxisFixture::selectMocks
 * does with the driver and encoder choosers of an axis.
 *
 *   FFBWheel wheel;
 *   InputSources::useMockRegistries(InputSources::of(wheel)); // firmware sources + mocks (selection id 15)
 *   cmd(wheel, "btntypes", CMDtype::set, 1 << MockButtonSource::SELECTION_ID); // the firmware creates the mock
 *   MockButtonSource::last()->buttons = 0b101;
 *
 * SelectableInputs keeps its choosers protected and FFBHIDMain inherits SelectableInputs privately. Both are reached
 * without changing the firmware: the C style cast converts to the private base class ([expr.cast]: a cast to an
 * inaccessible base class is a static_cast that ignores access), and the protected choosers are reached through member
 * pointers formed in a class derived from SelectableInputs. Nothing else of the firmware is accessed this way.
 */
#ifndef HOSTTEST_INPUTSOURCES_H_
#define HOSTTEST_INPUTSOURCES_H_

#include "FFBHIDMain.h"
#include "MockAnalogSource.h"
#include "MockButtonSource.h"
#include "SelectableInputs.h"

namespace InputSources {

struct Access : SelectableInputs {
	static ClassChooser<ButtonSource> SelectableInputs::*buttonChooser() { return &Access::btn_chooser; }
	static ClassChooser<AnalogSource> SelectableInputs::*analogChooser() { return &Access::analog_chooser; }
};

/** The SelectableInputs part of a main class (private base class of FFBHIDMain) */
inline SelectableInputs &of(FFBHIDMain &main) { return *(SelectableInputs *)(&main); }

/** The button source chooser of SelectableInputs (assignable) */
inline ClassChooser<ButtonSource> &buttonChooser(SelectableInputs &inputs) { return inputs.*Access::buttonChooser(); }
/** The analog source chooser of SelectableInputs (assignable) */
inline ClassChooser<AnalogSource> &analogChooser(SelectableInputs &inputs) { return inputs.*Access::analogChooser(); }

/**
 * Replaces both choosers with the firmware registries plus the mocks (MockButtonSource::registry(),
 * MockAnalogSource::registry()). Sources that already exist are kept; the change affects the next creation.
 */
inline void useMockRegistries(SelectableInputs &inputs) {
	buttonChooser(inputs) = ClassChooser<ButtonSource>(MockButtonSource::registry());
	analogChooser(inputs) = ClassChooser<AnalogSource>(MockAnalogSource::registry());
}

} // namespace InputSources

#endif
