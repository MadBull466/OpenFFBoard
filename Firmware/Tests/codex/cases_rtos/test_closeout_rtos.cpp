/* Closeout CDX-012/013. Public wrapper tests; simulated scheduling only.
 * FreeRTOS V10.3.1 event_groups.c:198-210,531 reserves bits 24..31 in this
 * 32-bit-tick configuration. Fault-state preservation is the fake's recoverable
 * assertion convention, not target continuation after configASSERT.
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"
#include "event_groups.hpp"
#include "tasklet.hpp"
#include "timer.hpp"
#include <functional>
#include <vector>
using namespace cpp_freertos;
namespace {
class PayloadTasklet : public Tasklet {
public:
	~PayloadTasklet() override { CheckForSafeDelete(); }
	std::vector<uint32_t> payloads;
protected:
	void Run(uint32_t p) override { payloads.push_back(p); }
};
class Caller : public Thread {
public:
	explicit Caller(std::function<void()> body) : Thread("CDXCALL", 128, 3), body(std::move(body)) {}
	TaskHandle_t start() { REQUIRE(Start()); HostRtos::makeResumable(GetHandle()); return GetHandle(); }
protected:
	void Run() override { body(); }
private:
	std::function<void()> body;
};
class CountingTimer : public Timer {
public:
	explicit CountingTimer(unsigned &runs) : Timer("CDXTMR", 3, true), runs(runs) {}
protected:
	void Run() override { ++runs; }
private:
	unsigned &runs;
};
}
TEST_SUITE("CodexRtosCloseout") {
TEST_CASE_FIXTURE(RtosFixture, "Sync rejects each reserved set bit without changing group state") {
	for (unsigned bit = 24; bit < 32; ++bit) {
		CAPTURE(bit);
		EventGroup group;
		REQUIRE(group.SetBits(2) == 2);
		CHECK(group.Sync(EventBits_t{1} << bit, 1, 0) == 0);
		CHECK(expectKernelFault("xEventGroupSetBits of control bits"));
		CHECK(HostRtos::kernelFaults().empty());
		CHECK(group.GetBits() == 2);
	}
}
TEST_CASE_FIXTURE(RtosFixture, "Sync rejects each reserved wait bit before setting valid bits") {
	for (unsigned bit = 24; bit < 32; ++bit) {
		CAPTURE(bit);
		EventGroup group;
		REQUIRE(group.SetBits(2) == 2);
		CHECK(group.Sync(4, EventBits_t{1} << bit, 0) == 0);
		CHECK(expectKernelFault("xEventGroupSync: no bits or control bits"));
		CHECK(HostRtos::kernelFaults().empty());
		CHECK(group.GetBits() == 2);
	}
}
TEST_CASE_FIXTURE(RtosFixture, "Sync accepts the highest application bit and clears only rendezvous bits") {
	EventGroup group;
	constexpr EventBits_t highest = EventBits_t{1} << 23;
	REQUIRE(group.SetBits(highest | 2) == (highest | 2));
	CHECK(group.Sync(1, highest | 1, 0) == (highest | 3));
	CHECK(group.GetBits() == 2);
	CHECK(group.Sync(0, 1, 0) == 2); // Empty set mask is valid; unmet rendezvous returns current state.
	CHECK(group.GetBits() == 2);
}
TEST_CASE_FIXTURE(RtosFixture, "Sync rejects an empty wait mask before changing state") {
	EventGroup group;
	REQUIRE(group.SetBits(2) == 2);
	CHECK(group.Sync(4, 0, 0) == 0);
	CHECK(expectKernelFault("xEventGroupSync: no bits or control bits"));
	CHECK(HostRtos::kernelFaults().empty());
	CHECK(group.GetBits() == 2);
}
TEST_CASE_FIXTURE(RtosFixture, "a successful tasklet schedule reports acceptance and delivers exactly its payload") {
	PayloadTasklet tasklet;
	REQUIRE(tasklet.Schedule(0x12345678, 0));
	CHECK(HostRtos::timerCommandsPending() == 1);
	CHECK(tasklet.payloads.empty());
	runReady();
	CHECK(HostRtos::timerCommandsPending() == 0);
	CHECK(tasklet.payloads == std::vector<uint32_t>{0x12345678});
}
// V12 intent only: CmdTimeout's header documents daemon-send time, not a
// general end-to-end deadline. If a repair applies it to DtorLock, failure to
// acquire that lock must reject the callback (bool/queue/payload contract).
TEST_CASE_FIXTURE(RtosFixture, "under V12 zero lock wait intent a pending tasklet rejects a second callback"
	* KNOWN_ISSUE("vendor V12 timeout intent: Schedule waits indefinitely for DtorLock; a timeout repair must reject failed lock acquisition. First failure is returned")) {
	PayloadTasklet tasklet;
	bool first = false, returned = false, accepted = true;
	Caller caller([&] { first = tasklet.Schedule(17, 0); accepted = tasklet.Schedule(29, 0); returned = true; });
	HostRtos::runUntilBlocked(caller.start()); // Priority 3, before the daemon (priority 2).
	REQUIRE(first);
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK(returned);
	CHECK_FALSE(accepted);
	CHECK(HostRtos::timerCommandsPending() == 1);
	runReady(); // Complete current vendor's blocked call before destruction; never exercise UAF.
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK(tasklet.payloads == std::vector<uint32_t>{17});
}
TEST_CASE_FIXTURE(RtosFixture, "a stopped and drained timer is destroyed in task context without later callbacks") {
	unsigned callbacks = 0;
	auto *timer = new CountingTimer(callbacks);
	REQUIRE(timer->Start(0));
	runReady();
	advance(3);
	REQUIRE(callbacks == 1);
	REQUIRE(timer->Stop(0));
	runReady(); // Establish dormant state before any derived object destruction.
	REQUIRE_FALSE(timer->IsActive());
	runInTask([&] { delete timer; }, 3); // Also permits a synchronous destructor to await the daemon.
	runReady();
	CHECK(HostRtos::timerCount() == 0);
	advance(9);
	CHECK(callbacks == 1);
}
}
