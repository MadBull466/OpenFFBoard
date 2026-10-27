/*
 * Tests of cpp_freertos::EventGroup (FFBoard/Src/cevent_groups.cpp, FFBoard/Inc/event_groups.hpp; tier A vendor code:
 * suspected defects are reported, never fixed) on the fake FreeRTOS API F11 (support/HostRtosExt.h).
 *
 * The wrapper forwards to the FreeRTOS event group API; the tests check what a caller of the wrapper observes: the
 * returned bits, the bits left in the group, which tasks continue, timeouts against simulated time, the FromISR
 * variants (on the F407 configUSE_TRACE_FACILITY is 1, so they are deferred to the timer service task) and create
 * failures.
 *
 * Host vs target: the create failures throw EventGroupCreateException on the host; the F407 build defines
 * CPP_FREERTOS_NO_EXCEPTIONS and stops in configASSERT instead (NEEDS REVIEW 142).
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"

#include "event_groups.hpp"
#include "thread.hpp"

#include <cstdint>
#include <functional>
#include <string>

using namespace cpp_freertos;
using HostRtos::RtosObject;

namespace {

/** Thread running a function set by the test (resumable: continues after a blocking call) */
class EgTask : public Thread {
public:
	explicit EgTask(std::function<void()> body, const std::string &name = "EG", UBaseType_t priority = 1)
		: Thread(name, 128, priority), body(std::move(body)) {}

	TaskHandle_t startResumable() {
		REQUIRE(Start());
		HostRtos::makeResumable(GetHandle());
		return GetHandle();
	}

protected:
	void Run() override { body(); }

private:
	std::function<void()> body;
};

/** Fills the command queue of the timer service task (configTIMER_QUEUE_LENGTH entries) with pended no-op calls */
void fillTimerServiceQueue() {
	auto noop = [](void *, uint32_t) {};
	for (int i = 0; i < configTIMER_QUEUE_LENGTH; i++) {
		REQUIRE(xTimerPendFunctionCall(noop, nullptr, 0, 0) == pdPASS);
	}
}

} // namespace

TEST_SUITE("EventGroup") {

// ---------------- Creation and destruction ----------------

TEST_CASE_FIXTURE(RtosFixture, "a new event group has no bits set") {
	EventGroup group;
	CHECK(group.GetBits() == 0);
	CHECK(HostRtos::createCalls(RtosObject::EventGroup) == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "destroying an event group deletes its kernel event group") {
	const size_t before = HostRtos::eventGroupCount();
	auto *group = new EventGroup();
	CHECK(HostRtos::eventGroupCount() == before + 1);
	delete group;
	CHECK(HostRtos::eventGroupCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "an event group whose kernel event group can not be created throws EventGroupCreateException") {
	HostRtos::failCreate(RtosObject::EventGroup);
	CHECK_THROWS_WITH_AS(EventGroup(), "Event Group Constructor Failed", EventGroupCreateException);
	CHECK(HostRtos::eventGroupCount() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "an event group with a static buffer is created with xEventGroupCreateStatic and works like a dynamic one") {
	StaticEventGroup_t buffer{};
	EventGroup group(&buffer);
	CHECK(HostRtos::createCalls(RtosObject::StaticEventGroup) == 1);
	CHECK(HostRtos::createCalls(RtosObject::EventGroup) == 0);
	CHECK(group.SetBits(0x10) == 0x10);
	CHECK(group.GetBits() == 0x10);
}

TEST_CASE_FIXTURE(RtosFixture, "an event group whose static creation fails throws EventGroupCreateException") {
	HostRtos::failCreate(RtosObject::StaticEventGroup);
	StaticEventGroup_t buffer{};
	CHECK_THROWS_AS(EventGroup{&buffer}, EventGroupCreateException);
	CHECK(HostRtos::eventGroupCount() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "destroying a statically created event group deletes its kernel event group") {
	const size_t before = HostRtos::eventGroupCount();
	StaticEventGroup_t buffer{};
	{
		EventGroup group(&buffer);
		CHECK(HostRtos::eventGroupCount() == before + 1);
	}
	CHECK(HostRtos::eventGroupCount() == before);
}

// ---------------- Set, clear, get ----------------

TEST_CASE_FIXTURE(RtosFixture, "SetBits adds the bits and returns all bits of the group") {
	EventGroup group;
	CHECK(group.SetBits(0x1) == 0x1);
	CHECK(group.SetBits(0x4) == 0x5);
	CHECK(group.GetBits() == 0x5);
}

TEST_CASE_FIXTURE(RtosFixture, "ClearBits clears only the given bits and returns the bits before clearing") {
	EventGroup group;
	group.SetBits(0x7);
	CHECK(group.ClearBits(0x5) == 0x7);
	CHECK(group.GetBits() == 0x2);
}

TEST_CASE_FIXTURE(RtosFixture, "GetBitsFromISR returns the bits of the group in an interrupt") {
	EventGroup group;
	group.SetBits(0x9);
	EventBits_t bits = 0;
	{
		HostPlatform::IsrContext isr;
		bits = group.GetBitsFromISR();
	}
	CHECK(bits == 0x9);
}

TEST_CASE_FIXTURE(RtosFixture, "SetBitsFromISR sets the bits when the timer service task runs") {
	EventGroup group;
	BaseType_t woken = pdFALSE;
	BaseType_t result = pdFAIL;
	{
		HostPlatform::IsrContext isr;
		result = group.SetBitsFromISR(0x5, &woken);
	}
	CHECK(result == pdPASS);
	CHECK(group.GetBits() == 0); // Deferred to the timer service task (event_groups.h, configUSE_TRACE_FACILITY 1)
	runReady();
	CHECK(group.GetBits() == 0x5);
}

TEST_CASE_FIXTURE(RtosFixture, "SetBitsFromISR reports that the waiting timer service task has a higher priority than the interrupted code") {
	EventGroup group;
	{
		HostPlatform::IsrContext isr;
		REQUIRE(group.SetBitsFromISR(0x1, nullptr) == pdPASS);
	}
	runReady(); // The timer service task now waits for its next command
	BaseType_t woken = pdFALSE;
	{
		HostPlatform::IsrContext isr;
		REQUIRE(group.SetBitsFromISR(0x2, &woken) == pdPASS);
	}
	CHECK(woken == pdTRUE); // Timer service priority 2 > the interrupted test thread
	runReady();
	CHECK(group.GetBits() == 0x3);
}

TEST_CASE_FIXTURE(RtosFixture, "SetBitsFromISR fails and sets nothing when the timer service queue is full") {
	EventGroup group;
	fillTimerServiceQueue();
	BaseType_t result = pdPASS;
	{
		HostPlatform::IsrContext isr;
		result = group.SetBitsFromISR(0x1, nullptr);
	}
	CHECK(result == pdFAIL);
	runReady();
	CHECK(group.GetBits() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "ClearBitsFromISR clears the bits when the timer service task runs") {
	EventGroup group;
	group.SetBits(0x7);
	BaseType_t result = pdFAIL;
	{
		HostPlatform::IsrContext isr;
		result = group.ClearBitsFromISR(0x3);
	}
	CHECK(result == pdPASS);
	CHECK(group.GetBits() == 0x7);
	runReady();
	CHECK(group.GetBits() == 0x4);
}

TEST_CASE_FIXTURE(RtosFixture, "ClearBitsFromISR fails and clears nothing when the timer service queue is full") {
	EventGroup group;
	group.SetBits(0x1);
	fillTimerServiceQueue();
	BaseType_t result = pdPASS;
	{
		HostPlatform::IsrContext isr;
		result = group.ClearBitsFromISR(0x1);
	}
	CHECK(result == pdFAIL);
	runReady();
	CHECK(group.GetBits() == 0x1);
}

// ---------------- WaitBits without blocking ----------------

TEST_CASE_FIXTURE(RtosFixture, "WaitBits for all bits that are already set returns the bits and clears the awaited ones on exit") {
	EventGroup group;
	group.SetBits(0x7);
	CHECK(group.WaitBits(0x3, true, true, 0) == 0x7); // The bits before clearing
	CHECK(group.GetBits() == 0x4);
}

TEST_CASE_FIXTURE(RtosFixture, "WaitBits without clear on exit leaves the bits set") {
	EventGroup group;
	group.SetBits(0x3);
	CHECK(group.WaitBits(0x3, false, true, 0) == 0x3);
	CHECK(group.GetBits() == 0x3);
}

TEST_CASE_FIXTURE(RtosFixture, "WaitBits for any bit is satisfied by one of the bits and clears all awaited bits on exit") {
	EventGroup group;
	group.SetBits(0x6);
	CHECK(group.WaitBits(0x3, true, false, 0) == 0x6);
	CHECK(group.GetBits() == 0x4);
}

TEST_CASE_FIXTURE(RtosFixture, "WaitBits for all bits with only some set and no timeout returns the bits without clearing them") {
	EventGroup group;
	group.SetBits(0x1);
	CHECK(group.WaitBits(0x3, true, true, 0) == 0x1);
	CHECK(group.GetBits() == 0x1);
}

TEST_CASE_FIXTURE(RtosFixture, "WaitBits on the test thread times out after its timeout of simulated time and returns the current bits") {
	EventGroup group;
	group.SetBits(0x1);
	CHECK(group.WaitBits(0x3, true, true, 10) == 0x1);
	CHECK(HostPlatform::nowUs() == 10000);
	CHECK(group.GetBits() == 0x1);
}

// ---------------- WaitBits in a task ----------------

TEST_CASE_FIXTURE(RtosFixture, "a task waiting for all bits keeps waiting while only some of them are set") {
	EventGroup group;
	EventBits_t result = 0;
	bool returned = false;
	EgTask task([&] {
		result = group.WaitBits(0x3, false, true, portMAX_DELAY);
		returned = true;
	});
	TaskHandle_t handle = task.startResumable();
	runReady();
	group.SetBits(0x1);
	runReady();
	CHECK_FALSE(returned);
	CHECK(HostRtos::isWaiting(handle));
	group.SetBits(0x2);
	runReady();
	CHECK(returned);
	CHECK(result == 0x3);
}

TEST_CASE_FIXTURE(RtosFixture, "a task waiting for any bit continues when the first of them is set") {
	EventGroup group;
	EventBits_t result = 0;
	EgTask task([&] { result = group.WaitBits(0x3, false, false, portMAX_DELAY); });
	task.startResumable();
	runReady();
	group.SetBits(0x2);
	runReady();
	CHECK(result == 0x2);
	CHECK(group.GetBits() == 0x2); // No clear on exit
}

TEST_CASE_FIXTURE(RtosFixture, "a task woken with clear on exit clears only the bits it waited for") {
	EventGroup group;
	group.SetBits(0x4);
	EventBits_t result = 0;
	EgTask task([&] { result = group.WaitBits(0x3, true, true, portMAX_DELAY); });
	task.startResumable();
	runReady();
	group.SetBits(0x3);
	runReady();
	CHECK(result == 0x7); // The bits at the time the task was unblocked
	CHECK(group.GetBits() == 0x4);
}

TEST_CASE_FIXTURE(RtosFixture, "two tasks waiting with clear on exit for the same bit both continue when it is set") {
	EventGroup group;
	EventBits_t first = 0;
	EventBits_t second = 0;
	EgTask a([&] { first = group.WaitBits(0x1, true, true, portMAX_DELAY); }, "EGA");
	EgTask b([&] { second = group.WaitBits(0x1, true, true, portMAX_DELAY); }, "EGB");
	a.startResumable();
	b.startResumable();
	runReady();
	group.SetBits(0x1);
	runReady();
	CHECK(first == 0x1);
	CHECK(second == 0x1);
	CHECK(group.GetBits() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a task waiting with a timeout continues after the timeout with the current bits") {
	EventGroup group;
	group.SetBits(0x1);
	EventBits_t result = 0;
	bool returned = false;
	EgTask task([&] {
		result = group.WaitBits(0x3, true, true, 5);
		returned = true;
	});
	task.startResumable();
	runReady();
	advance(4);
	CHECK_FALSE(returned);
	advance(1);
	CHECK(returned);
	CHECK(result == 0x1);
	CHECK(group.GetBits() == 0x1); // Condition not met: nothing cleared
}

TEST_CASE_FIXTURE(RtosFixture, "a task waiting for bits continues with no bits when the event group is destroyed") {
	auto *group = new EventGroup();
	group->SetBits(0x2);
	EventBits_t result = 0xFFFF;
	bool returned = false;
	EgTask task([&] {
		result = group->WaitBits(0x1, true, true, portMAX_DELAY);
		returned = true;
	});
	TaskHandle_t handle = task.startResumable();
	runReady();
	REQUIRE(HostRtos::isWaiting(handle));
	delete group; // vEventGroupDelete unblocks the waiting task (event_groups.c:627)
	runReady();
	CHECK(returned);
	CHECK(result == 0);
}

// ---------------- Sync ----------------

TEST_CASE_FIXTURE(RtosFixture, "Sync returns at once when its own bits complete the rendezvous and clears the awaited bits") {
	EventGroup group;
	group.SetBits(0x2);
	CHECK(group.Sync(0x1, 0x3, 0) == 0x3);
	CHECK(group.GetBits() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "three parties meeting at a Sync rendezvous all continue") {
	EventGroup group;
	EventBits_t resultA = 0;
	EventBits_t resultB = 0;
	EgTask a([&] { resultA = group.Sync(0x1, 0x7, portMAX_DELAY); }, "EGA");
	EgTask b([&] { resultB = group.Sync(0x2, 0x7, portMAX_DELAY); }, "EGB");
	a.startResumable();
	b.startResumable();
	runReady();
	CHECK(group.GetBits() == 0x3); // Both set their bit and wait
	CHECK(group.Sync(0x4, 0x7, 0) == 0x7);
	runReady();
	CHECK(resultA == 0x7);
	CHECK(resultB == 0x7);
	CHECK(group.GetBits() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "Sync on the test thread times out when the other party does not arrive and leaves its bit set") {
	EventGroup group;
	CHECK(group.Sync(0x1, 0x3, 5) == 0x1);
	CHECK(HostPlatform::nowUs() == 5000);
	CHECK(group.GetBits() == 0x1);
}

TEST_CASE_FIXTURE(RtosFixture, "a task in Sync with a timeout continues after the timeout with the current bits") {
	EventGroup group;
	EventBits_t result = 0;
	bool returned = false;
	EgTask task([&] {
		result = group.Sync(0x1, 0x3, 3);
		returned = true;
	});
	task.startResumable();
	runReady();
	advance(2);
	CHECK_FALSE(returned);
	advance(1);
	CHECK(returned);
	CHECK(result == 0x1);
}

}
