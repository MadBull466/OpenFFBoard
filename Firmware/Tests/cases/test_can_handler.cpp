/*
 * CAN port registry (CAN.cpp) and CAN callback handlers (CanHandler.cpp)
 *
 * The HAL CAN interrupt callbacks (global_callbacks.cpp, not compiled for the host) look up the port with
 * CANPort::handleToPort(hcan) and call the matching callback of every registered CanHandler.
 * The dispatch helpers below do exactly that, so the registry and the callbacks are tested the way the
 * firmware uses them.
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "HostPlatform.h"
#include "MockCANPort.h"

#include "CAN.h"
#include "CanHandler.h"

#include <algorithm>
#include <ostream>
#include <memory>
#include <string>
#include <vector>

namespace {

/** Mock port with a selectable HAL handle and access to the waiting flag of the port implementation */
class HandlePort : public MockCANPort {
public:
	explicit HandlePort(void *handle) : handle(handle) {}
	void setWaiting(bool waiting) { isWaitingFlag = waiting; }

protected:
	void *getHandle() override { return handle; }

private:
	void *handle;
};

struct Call {
	std::string type;
	CANPort *port;
	uint32_t value; // fifo, error code, mailbox or message id
	bool operator==(const Call &o) const { return type == o.type && port == o.port && value == o.value; }
};

[[maybe_unused]] std::ostream &operator<<(std::ostream &os, const Call &c) { return os << c.type << " " << c.value << " port " << c.port; }

class RecordingCanHandler : public CanHandler {
public:
	explicit RecordingCanHandler(std::vector<std::string> *log = nullptr, std::string name = "") : log(log), name(std::move(name)) {}
	void canRxPendCallback(CANPort *port, CAN_rx_msg &msg) override { record("rxPend", port, msg.header.id); }
	void canErrorCallback(CANPort *port, uint32_t errcode) override { record("error", port, errcode); }
	void canRxFullCallback(CANPort *port, uint32_t fifo) override { record("rxFull", port, fifo); }
	void canTxCpltCallback(CANPort *port, uint32_t mailbox) override { record("txCplt", port, mailbox); }
	void canTxAbortCallback(CANPort *port, uint32_t mailbox) override { record("txAbort", port, mailbox); }

	std::vector<Call> calls;

private:
	void record(const std::string &type, CANPort *port, uint32_t value) {
		calls.push_back({type, port, value});
		if (log) log->push_back(name + " " + type);
	}
	std::vector<std::string> *log;
	std::string name;
};

// --- Dispatch like the HAL callbacks in global_callbacks.cpp ---
void dispatchRxPend(void *hcan, uint32_t id) {
	CAN_rx_msg msg;
	msg.header.id = id;
	msg.header.length = 0;
	HostPlatform::IsrContext isr;
	CANPort *port = CANPort::handleToPort(hcan);
	for (CanHandler *c : CanHandler::getCANHandlers()) c->canRxPendCallback(port, msg);
}
void dispatchRxFull(void *hcan, uint32_t fifo) {
	HostPlatform::IsrContext isr;
	for (CanHandler *c : CanHandler::getCANHandlers()) c->canRxFullCallback(CANPort::handleToPort(hcan), fifo);
}
void dispatchError(void *hcan, uint32_t err) {
	HostPlatform::IsrContext isr;
	for (CanHandler *c : CanHandler::getCANHandlers()) c->canErrorCallback(CANPort::handleToPort(hcan), err);
}
void dispatchTxCplt(void *hcan, uint32_t mailbox) {
	HostPlatform::IsrContext isr;
	for (CanHandler *c : CanHandler::getCANHandlers()) c->canTxCpltCallback(CANPort::handleToPort(hcan), mailbox);
}
void dispatchTxAbort(void *hcan, uint32_t mailbox) {
	HostPlatform::IsrContext isr;
	for (CanHandler *c : CanHandler::getCANHandlers()) c->canTxAbortCallback(CANPort::handleToPort(hcan), mailbox);
}

void dispatchAll(void *hcan) {
	dispatchRxPend(hcan, 0x123);
	dispatchRxFull(hcan, 1);
	dispatchError(hcan, 0x40);
	dispatchTxCplt(hcan, 2);
	dispatchTxAbort(hcan, 0);
}

bool isRegistered(CanHandler *h) {
	auto &v = CanHandler::getCANHandlers();
	return std::count(v.begin(), v.end(), h) == 1;
}

struct CanFixture {
	CAN_HandleTypeDef hcanA{};
	CAN_HandleTypeDef hcanB{};
	CanFixture() { HostPlatform::reset(); }
	~CanFixture() { HostPlatform::reset(); }
};

} // namespace

TEST_SUITE("CANPort") {

TEST_CASE_FIXTURE(CanFixture, "a port is found by its HAL handle") {
	HandlePort port(&hcanA);
	CHECK(CANPort::handleToPort(&hcanA) == &port);
}

TEST_CASE_FIXTURE(CanFixture, "an unknown handle gives no port") {
	HandlePort port(&hcanA);
	CHECK(CANPort::handleToPort(&hcanB) == nullptr);
	CHECK(CANPort::handleToPort(nullptr) == nullptr);
}

TEST_CASE_FIXTURE(CanFixture, "every port is found by its own handle") {
	HandlePort a(&hcanA);
	HandlePort b(&hcanB);
	CHECK(CANPort::handleToPort(&hcanA) == &a);
	CHECK(CANPort::handleToPort(&hcanB) == &b);
}

TEST_CASE_FIXTURE(CanFixture, "a destroyed port is no longer found") {
	HandlePort stays(&hcanB);
	{
		HandlePort gone(&hcanA);
		REQUIRE(CANPort::handleToPort(&hcanA) == &gone);
	}
	CHECK(CANPort::handleToPort(&hcanA) == nullptr);
	CHECK(CANPort::handleToPort(&hcanB) == &stays);
}

TEST_CASE_FIXTURE(CanFixture, "a new port is not waiting") {
	HandlePort port(&hcanA);
	CHECK_FALSE(port.isWaiting());
}

TEST_CASE_FIXTURE(CanFixture, "isWaiting reports the waiting flag of the port implementation") {
	HandlePort port(&hcanA);
	port.setWaiting(true);
	CHECK(port.isWaiting());
	port.setWaiting(false);
	CHECK_FALSE(port.isWaiting());
}

TEST_CASE_FIXTURE(CanFixture, "giving the port semaphore ends the waiting state") {
	HandlePort port(&hcanA);
	CHECK(port.takeSemaphore(0));
	port.setWaiting(true);
	port.giveSemaphore();
	CHECK_FALSE(port.isWaiting());
	CHECK(port.takeSemaphore(0)); // Was given back
	port.giveSemaphore();
}

} // TEST_SUITE

// ---------------------------------------------------------------------------
TEST_SUITE("CanHandler") {

TEST_CASE("a can handler is registered while it exists") {
	CanHandler *address = nullptr;
	{
		RecordingCanHandler handler;
		address = &handler;
		CHECK(isRegistered(&handler));
	}
	auto &v = CanHandler::getCANHandlers();
	CHECK(std::find(v.begin(), v.end(), address) == v.end());
}

TEST_CASE_FIXTURE(CanFixture, "every callback reaches every registered handler with the port of the handle") {
	HandlePort port(&hcanA);
	RecordingCanHandler a;
	RecordingCanHandler b;
	std::vector<Call> expected;
	SUBCASE("rx pending") {
		dispatchRxPend(&hcanA, 0x321);
		expected = {{"rxPend", &port, 0x321}};
	}
	SUBCASE("rx fifo full") {
		dispatchRxFull(&hcanA, 1);
		expected = {{"rxFull", &port, 1}};
	}
	SUBCASE("error") {
		dispatchError(&hcanA, 0x80);
		expected = {{"error", &port, 0x80}};
	}
	SUBCASE("tx complete") {
		dispatchTxCplt(&hcanA, 2);
		expected = {{"txCplt", &port, 2}};
	}
	SUBCASE("tx abort") {
		dispatchTxAbort(&hcanA, 1);
		expected = {{"txAbort", &port, 1}};
	}
	CHECK(a.calls == expected);
	CHECK(b.calls == expected);
}

TEST_CASE_FIXTURE(CanFixture, "callbacks of an unknown handle are delivered without a port") {
	RecordingCanHandler handler;
	dispatchTxCplt(&hcanB, 0);
	REQUIRE(handler.calls.size() == 1);
	CHECK(handler.calls[0].port == nullptr);
}

TEST_CASE_FIXTURE(CanFixture, "can handlers are called in the order they were created") {
	std::vector<std::string> log;
	RecordingCanHandler first(&log, "first");
	RecordingCanHandler second(&log, "second");
	dispatchError(&hcanA, 1);
	CHECK(log == std::vector<std::string>{"first error", "second error"});
}

TEST_CASE_FIXTURE(CanFixture, "a destroyed can handler receives no more callbacks") {
	std::vector<std::string> log;
	RecordingCanHandler stays(&log, "stays");
	{
		RecordingCanHandler gone(&log, "gone");
	}
	dispatchRxFull(&hcanA, 0);
	CHECK(log == std::vector<std::string>{"stays rxFull"});
}

TEST_CASE_FIXTURE(CanFixture, "the default callbacks of the can handler base class ignore all events") {
	HandlePort port(&hcanA);
	std::vector<std::string> log;
	CanHandler plain; // Registered, overrides nothing
	RecordingCanHandler after(&log, "after");
	CHECK(isRegistered(&plain));
	dispatchAll(&hcanA);
	CHECK(log == std::vector<std::string>{"after rxPend", "after rxFull", "after error", "after txCplt", "after txAbort"});
}

TEST_CASE_FIXTURE(CanFixture, "frames received by the mock port reach all can handlers") {
	MockCANPort port;
	CAN_filter f{};
	f.filter_id = 0x100;
	f.filter_mask = 0x7FF;
	f.active = true;
	port.addCanFilter(f);
	RecordingCanHandler a;
	RecordingCanHandler b;
	CHECK(port.receive(0x100, std::array<uint8_t, 8>{}));
	CHECK(a.calls == std::vector<Call>{{"rxPend", &port, 0x100}});
	CHECK(b.calls == std::vector<Call>{{"rxPend", &port, 0x100}});
}

} // TEST_SUITE
