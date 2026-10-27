/*
 * RtosFixture.h
 *
 * Fixture of ffb_rtos_tests (the cpp_freertos wrappers on the fake FreeRTOS API F11, see HostRtosExt.h).
 * Use with TEST_CASE_FIXTURE(RtosFixture, "...").
 *
 * Resets before and after every test: simulated time and interrupt context (HostPlatform::reset), the fake kernel
 * (HostRtos::reset: notifications, delay budget, kernel faults, injected create failures, critical section / scheduler
 * suspension state, tick counter, the timer service task with its command queue and active timer list, timers deleted
 * by the service). Kernel objects and tasks owned by firmware objects are not deleted (their destructors do that):
 * destroy the wrappers inside the test. Objects with static storage of the firmware (Thread::StartGuardLock) stay.
 *
 * At the end of every test the fixture CHECKs that no kernel fault was recorded and no critical section / scheduler
 * suspension is left open. In a KNOWN_ISSUE (should_fail) test that check would count as the expected failure, so such
 * tests call REQUIRE_ALL_RTOS_MODELLED() right before the assertion describing the defect. A test that expects a fault
 * removes it with expectKernelFault("part of the text").
 */
#ifndef HOSTTEST_RTOSFIXTURE_H_
#define HOSTTEST_RTOSFIXTURE_H_

#include "HostPlatform.h"
#include "HostRtos.h"
#include "HostRtosExt.h"

#include "doctest.h"

#include <cstdint>
#include <string>
#include <vector>

/**
 * Fails the test (REQUIRE) if the fake recorded a kernel fault (unmodelled call, configASSERT, block forever outside a
 * task), or a critical section / scheduler suspension is still open. Use it in KNOWN_ISSUE tests right before the
 * assertion that describes the defect.
 */
#define REQUIRE_ALL_RTOS_MODELLED()                                                                                         \
	do {                                                                                                                  \
		INFO("first kernel fault: " << (HostRtos::kernelFaults().empty() ? std::string() : HostRtos::kernelFaults().front())); \
		REQUIRE(HostRtos::kernelFaults().empty());                                                                        \
		REQUIRE(HostRtos::criticalStats().nesting == 0);                                                                  \
		REQUIRE(HostRtos::schedulerSuspendNesting() == 0);                                                                \
	} while (0)

struct RtosFixture {
	RtosFixture() { resetAll(); }
	virtual ~RtosFixture() {
		const auto &faults = HostRtos::kernelFaults();
		INFO("first kernel fault: " << (faults.empty() ? std::string() : faults.front()));
		CHECK_MESSAGE(faults.empty(), "the fake FreeRTOS recorded a kernel fault (see HostRtosExt.h)");
		CHECK_MESSAGE(HostRtos::criticalStats().nesting == 0, "a critical section is still open at the end of the test");
		CHECK_MESSAGE(HostRtos::schedulerSuspendNesting() == 0, "the scheduler is still suspended at the end of the test");
		resetAll();
	}

	static void resetAll() {
		HostPlatform::reset();
		HostRtos::reset();
	}

	/**
	 * Lets `ms` ticks of simulated time pass like with a running scheduler: per tick the SysTick (tick hook), then every
	 * ready task runs until it blocks (timer service task, tasks woken by a timeout or a condition).
	 */
	static void advance(uint32_t ms) {
		for (uint32_t i = 0; i < ms; i++) {
			HostPlatform::advanceMs(1);
			HostRtos::runUntilIdle();
		}
	}

	/** Runs every ready task until all are blocked (the scheduler between two ticks) */
	static uint32_t runReady() { return HostRtos::runUntilIdle(); }

	/** Removes the first recorded kernel fault containing `part`. @return true if there was one */
	static bool expectKernelFault(const std::string &part) { return HostRtos::takeKernelFault(part); }

	/**
	 * Keeps an object reachable until the program ends and returns it. For objects that can never be destroyed
	 * (cpp_freertos::MemoryPool has a deleted destructor): created with new, never deleted. Without a reference the
	 * leak sanitizer of the sanitizer builds would report them (and the memory they hold) at exit.
	 */
	template <class T>
	static T *keepAlive(T *object) {
		static auto *objects = new std::vector<void *>(); // Never destroyed: still referenced when the leak check runs
		objects->push_back(object);
		return object;
	}
};

#endif
