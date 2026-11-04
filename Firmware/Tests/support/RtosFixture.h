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
 *
 * Leak check: at the end of every test the fixture also CHECKs that no more tasks, queues, semaphores / mutexes, timers
 * and event groups exist than at its construction (HostRtos::objectCounts; the timer service task and a timer whose
 * delete command is queued do not count). Justified leaks are allowed with allowLeak(kind, n); keepAlive(new MemoryPool)
 * allows the pool's mutex. Destroy wrappers that wait for another task in a task: runInTask(fn, priority).
 */
#ifndef HOSTTEST_RTOSFIXTURE_H_
#define HOSTTEST_RTOSFIXTURE_H_

#include "HostPlatform.h"
#include "HostRtos.h"
#include "HostRtosExt.h"

#include "doctest.h"

#include "mem_pool.hpp"
#include "thread.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <type_traits>
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
	/** Kinds of kernel objects of the leak check (allowLeak) */
	enum class Leak { Tasks, Queues, Semaphores, Timers, EventGroups };

	RtosFixture() {
		resetAll();
		atStart = HostRtos::objectCounts();
	}
	virtual ~RtosFixture() {
		const auto &faults = HostRtos::kernelFaults();
		INFO("first kernel fault: " << (faults.empty() ? std::string() : faults.front()));
		CHECK_MESSAGE(faults.empty(), "the fake FreeRTOS recorded a kernel fault (see HostRtosExt.h)");
		CHECK_MESSAGE(HostRtos::criticalStats().nesting == 0, "a critical section is still open at the end of the test");
		CHECK_MESSAGE(HostRtos::schedulerSuspendNesting() == 0, "the scheduler is still suspended at the end of the test");
		checkNoLeaks();
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
	 * Runs `fn` in a new resumable task of priority `priority` until it returns (all ready tasks run, see runReady) and
	 * REQUIREs that it returned. For code that blocks until another task acts, e.g. destructors that wait for a thread
	 * (~WorkQueue) or would wait for the timer service: on the test thread such a wait is recorded as "would block
	 * forever", because no task runs during a wait of the test thread (see HostRtosExt.h). The task deletes itself.
	 */
	static void runInTask(std::function<void()> fn, UBaseType_t priority = 1) {
		bool done = false;
		FunctionTask task([&] {
			fn();
			done = true;
		}, priority);
		REQUIRE(task.Start());
		HostRtos::makeResumable(task.GetHandle());
		HostRtos::runUntilIdle();
		REQUIRE_MESSAGE(done, "the function run by runInTask is still blocked");
	}

	/**
	 * Allows `count` more objects of `kind` to exist at the end of the test than at its start (a leak that is the
	 * subject of the test or a documented defect). Use sparingly, with a comment.
	 */
	void allowLeak(Leak kind, size_t count = 1) { allowed[(size_t)kind] += count; }

	/**
	 * Keeps an object reachable until the program ends and returns it. For objects that can never be destroyed
	 * (cpp_freertos::MemoryPool has a deleted destructor): created with new, never deleted. Without a reference the
	 * leak sanitizer of the sanitizer builds would report them (and the memory they hold) at exit. A MemoryPool also
	 * keeps its mutex (MemoryPool::Lock): it is allowed in the leak check.
	 */
	template <class T>
	T *keepAlive(T *object) {
		static auto *objects = new std::vector<void *>(); // Never destroyed: still referenced when the leak check runs
		objects->push_back(object);
		if constexpr (std::is_base_of_v<cpp_freertos::MemoryPool, T>) {
			allowLeak(Leak::Semaphores, 1);
		}
		return object;
	}

	/** CHECKs that no more kernel objects exist than at the construction of the fixture (plus allowLeak) */
	void checkNoLeaks() const {
		const HostRtos::ObjectCounts now = HostRtos::objectCounts();
		checkCount("tasks", atStart.tasks, now.tasks, Leak::Tasks);
		checkCount("queues", atStart.queues, now.queues, Leak::Queues);
		checkCount("semaphores / mutexes", atStart.semaphores, now.semaphores, Leak::Semaphores);
		checkCount("timers", atStart.timers, now.timers, Leak::Timers);
		checkCount("event groups", atStart.eventGroups, now.eventGroups, Leak::EventGroups);
	}

	HostRtos::ObjectCounts atStart; //!< Kernel objects at the construction of the fixture

private:
	/** Thread of runInTask */
	class FunctionTask : public cpp_freertos::Thread {
	public:
		FunctionTask(std::function<void()> body, UBaseType_t priority) : Thread("RUNINTASK", 128, priority), body(std::move(body)) {}

	protected:
		void Run() override { body(); }

	private:
		std::function<void()> body;
	};

	void checkCount(const char *what, size_t start, size_t now, Leak kind) const {
		INFO("kernel " << std::string(what) << " at the start of the test: " << start << ", at the end: " << now
					   << ", allowed leaks: " << allowed[(size_t)kind]);
		CHECK_MESSAGE(now <= start + allowed[(size_t)kind], "kernel objects leaked by the test (RtosFixture leak check, see allowLeak)");
	}

	size_t allowed[5] = {};
};

#endif
