#include "UsbFixture.h"
#include <cstdlib>
#include <string>

namespace {
// Same ordering as the protected control fixture: enumerate, then clear logs.
struct SetupBoundaryFixture : UsbFixture {
	SetupBoundaryFixture() {
		startDevice();
		// Test-owned opt-in used only by the dedicated subprocess guard check.
		const char *probe = std::getenv("CODEX_USB_SETUP_FAULT_PROBE");
		const bool inject = probe && std::string(probe) == "1";
		if (inject) dcd_remote_wakeup(0);
		enumerate();
		if (inject) MESSAGE("CODEX_USB_DISCARD_REACHED");
		const auto discarded = HostDcd::takeFaults();
		CHECK(discarded.empty());
	}
};
}

TEST_SUITE("CodexUsbCloseout") {
TEST_CASE_FIXTURE(SetupBoundaryFixture, "successful enumeration reaches derived log clearing without a setup fault") {
	CHECK(tud_mounted());
	CHECK(UsbFixture::firstFault(host).empty());
}
}
