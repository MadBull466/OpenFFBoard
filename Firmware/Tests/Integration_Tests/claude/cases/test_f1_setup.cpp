/*
 * F1 setup (WO I0-4): the smoke case of the integration executable. It proves that the real TinyUSB device stack, the
 * firmware and the fake platform link together and that the global reset works. The fixture (IntegrationFixture)
 * comes with WO I0-5; the cases of the flow charter follow in the later work orders.
 */
#include "doctest.h"

#include "HostDcd.h"
#include "HostPlatform.h"
#include "HostTarget.h"
#include "usbd.h"

TEST_SUITE("F1Setup") {

// Oracle: the TinyUSB device API (usbd.h, tud_inited / tud_deinit): tud_deinit shuts the device stack down and after it
// tud_inited() reports false until tud_init is called again. The case does not depend on test order: later F1 cases
// enumerate the device, and tud_deinit does not clear the configuration number (usbd.c, so tud_mounted() would stay
// true), but it does define tud_inited(). The reset order is the one of plan section 3.3 (tud_deinit first).
TEST_CASE("the device stack is not initialised after the global reset") {
	tud_deinit(0);
	HostDcd::reset();
	HostPlatform::reset();
	HostTarget::reset();
	CHECK_FALSE(tud_inited());
}

} // TEST_SUITE
