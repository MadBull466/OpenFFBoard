/*
 * Tests of the cpp_freertos TickHook wrapper (FFBoard/Src/ctickhook.cpp, FFBoard/Inc/tickhook.hpp; vendor code, tier A:
 * suspected defects are reported, never fixed) on the fake FreeRTOS API F11 (support/HostRtosExt.h).
 *
 * Every simulated 1 ms boundary is one SysTick: vApplicationTickHook() of ctickhook.cpp runs in interrupt context and
 * calls Run() of every registered and enabled hook. The class has no Deregister(): a hook is removed by its destructor.
 * TickHook::Callbacks is a static list: every test destroys the hooks it registered.
 *
 * Host vs target (NEEDS REVIEW 141): the F407 has configUSE_TICK_HOOK 0, so ctickhook.cpp compiles to nothing there;
 * ffb_rtos_tests switches the hook on. No shipped firmware uses this class (latent code).
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"
#include "cmsis_compiler.h"

#include "tickhook.hpp"

#include <cstdint>
#include <string>
#include <vector>

using namespace cpp_freertos;

namespace {

/** Tick hook counting its runs; appends its name to `order` (optional) on every run */
class LoggingTickHook : public TickHook {
public:
	explicit LoggingTickHook(std::vector<std::string> *order = nullptr, std::string name = "", uint32_t *externalRuns = nullptr)
		: order(order), name(std::move(name)), externalRuns(externalRuns) {}
	uint32_t runs = 0;
	uint32_t runsInSysTick = 0;
	std::vector<TickType_t> runTicks;

protected:
	void Run() override {
		runs++;
		if (__get_IPSR() == 15) { // SysTick exception number
			runsInSysTick++;
		}
		runTicks.push_back(xTaskGetTickCountFromISR());
		if (order != nullptr) {
			order->push_back(name);
		}
		if (externalRuns != nullptr) {
			(*externalRuns)++;
		}
	}

private:
	std::vector<std::string> *order;
	std::string name;
	uint32_t *externalRuns;
};

} // namespace

TEST_SUITE("TickHook") {

TEST_CASE_FIXTURE(RtosFixture, "a tick hook does not run before it is registered") {
	LoggingTickHook hook;
	HostPlatform::advanceMs(3);
	CHECK(hook.runs == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a registered tick hook runs on every tick in the SysTick interrupt") {
	LoggingTickHook hook;
	hook.Register();
	HostPlatform::advanceMs(3);
	CHECK(hook.runTicks == std::vector<TickType_t>{1, 2, 3});
	CHECK(hook.runsInSysTick == 3);
}

TEST_CASE_FIXTURE(RtosFixture, "a tick hook does not run without a tick") {
	LoggingTickHook hook;
	hook.Register();
	HostPlatform::advanceUs(999);
	CHECK(hook.runs == 0);
	HostPlatform::advanceUs(1);
	CHECK(hook.runs == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "a disabled tick hook does not run") {
	LoggingTickHook hook;
	hook.Register();
	hook.Disable();
	HostPlatform::advanceMs(4);
	CHECK(hook.runs == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a tick hook disabled before it was registered stays disabled") {
	LoggingTickHook hook;
	hook.Disable();
	hook.Register();
	HostPlatform::advanceMs(2);
	CHECK(hook.runs == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "an enabled tick hook runs again after it was disabled") {
	LoggingTickHook hook;
	hook.Register();
	HostPlatform::advanceMs(2);
	hook.Disable();
	HostPlatform::advanceMs(3);
	hook.Enable();
	HostPlatform::advanceMs(1);
	CHECK(hook.runTicks == std::vector<TickType_t>{1, 2, 6});
}

TEST_CASE_FIXTURE(RtosFixture, "every registered tick hook runs on each tick") {
	LoggingTickHook first;
	LoggingTickHook second;
	first.Register();
	second.Register();
	HostPlatform::advanceMs(4);
	CHECK(first.runs == 4);
	CHECK(second.runs == 4);
}

TEST_CASE_FIXTURE(RtosFixture, "disabling one tick hook does not stop the others") {
	LoggingTickHook first;
	LoggingTickHook second;
	first.Register();
	second.Register();
	first.Disable();
	HostPlatform::advanceMs(2);
	CHECK(first.runs == 0);
	CHECK(second.runs == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "a destroyed tick hook is no longer called and the others still are") {
	uint32_t destroyedRuns = 0;
	LoggingTickHook remaining;
	remaining.Register();
	LoggingTickHook *hook = new LoggingTickHook(nullptr, "", &destroyedRuns);
	hook->Register();
	HostPlatform::advanceMs(1);
	REQUIRE(destroyedRuns == 1);
	delete hook;
	HostPlatform::advanceMs(2);
	CHECK(destroyedRuns == 1);
	CHECK(remaining.runs == 3);
}

TEST_CASE_FIXTURE(RtosFixture, "destroying a tick hook that was never registered leaves the registered hooks") {
	LoggingTickHook registered;
	registered.Register();
	{
		LoggingTickHook unregistered;
	}
	HostPlatform::advanceMs(1);
	CHECK(registered.runs == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "registering, enabling, disabling and destroying a tick hook each use one critical section") {
	{
		LoggingTickHook hook;
		hook.Register();
		CHECK(HostRtos::criticalStats().enter == 1);
		hook.Disable();
		CHECK(HostRtos::criticalStats().enter == 2);
		hook.Enable();
		CHECK(HostRtos::criticalStats().enter == 3);
	}
	CHECK(HostRtos::criticalStats().enter == 4);
	CHECK(HostRtos::criticalStats().exit == 4);
	CHECK(HostRtos::criticalStats().maxNesting == 1);
}

// NEEDS REVIEW 145: Register() uses push_front, so the hook registered last runs first on each tick. FreeRTOS gives no
// order for the application tick hook; is a registration order expected by users of this class?
TEST_CASE_FIXTURE(RtosFixture, "characterization: the tick hook registered last runs first") {
	std::vector<std::string> order;
	LoggingTickHook a(&order, "A");
	LoggingTickHook b(&order, "B");
	a.Register();
	b.Register();
	HostPlatform::advanceMs(1);
	CHECK(order == std::vector<std::string>{"B", "A"});
}

// NEEDS REVIEW 146: Register() does not check whether the hook is already in the list, so a hook registered twice runs
// twice per tick (the destructor removes both entries). Should a second Register() be ignored?
TEST_CASE_FIXTURE(RtosFixture, "characterization: a tick hook registered twice runs twice per tick") {
	LoggingTickHook hook;
	hook.Register();
	hook.Register();
	HostPlatform::advanceMs(2);
	CHECK(hook.runs == 4);
}

}
