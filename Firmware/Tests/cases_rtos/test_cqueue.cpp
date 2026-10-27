/*
 * Tests of the cpp_freertos queue wrappers Queue, Deque and BinaryQueue (FFBoard/Src/cqueue.cpp, FFBoard/Inc/queue.hpp;
 * vendor code, tier A: suspected defects are reported, never fixed) on the fake FreeRTOS API F11
 * (support/HostRtosExt.h).
 *
 * No shipped firmware uses these classes (latent code).
 * Host vs target: the host build uses the exception variant (the F407 defines CPP_FREERTOS_NO_EXCEPTIONS and stops in
 * configASSERT when the kernel queue can not be created, NEEDS REVIEW 142). Items are copied byte by byte by the kernel,
 * so the item sizes used here (uint32_t, a 12 byte struct) are the same on host and target.
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"

#include "queue.hpp"
#include "thread.hpp"

#include <cstdint>
#include <cstring>
#include <functional>
#include <string>

using namespace cpp_freertos;

namespace {

/** Thread running a function set by the test */
class QueueTestThread : public Thread {
public:
	explicit QueueTestThread(std::function<void()> body, const std::string &name = "QT", UBaseType_t priority = 1)
		: Thread(name, 128, priority), body(std::move(body)) {}

	/** Starts the task and makes it resumable (it continues after a blocking call, see HostRtos::makeResumable) */
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

struct Item12 {
	uint32_t a;
	uint16_t b;
	uint8_t c[6];
};
static_assert(sizeof(Item12) == 12, "same layout on host and target");

/** Fills a queue with the values 1..n */
void fill(Queue &queue, uint32_t n) {
	for (uint32_t i = 1; i <= n; i++) {
		REQUIRE(queue.Enqueue(&i, 0));
	}
}

} // namespace

TEST_SUITE("Queue") {

// ---------------- Creation and state ----------------

TEST_CASE_FIXTURE(RtosFixture, "a new queue is empty and has its whole capacity left") {
	Queue queue(4, sizeof(uint32_t));
	CHECK(queue.IsEmpty());
	CHECK_FALSE(queue.IsFull());
	CHECK(queue.NumItems() == 0);
	CHECK(queue.NumSpacesLeft() == 4);
}

TEST_CASE_FIXTURE(RtosFixture, "a queue counts its items and the spaces left") {
	Queue queue(4, sizeof(uint32_t));
	fill(queue, 3);
	CHECK(queue.NumItems() == 3);
	CHECK(queue.NumSpacesLeft() == 1);
	CHECK_FALSE(queue.IsEmpty());
	CHECK_FALSE(queue.IsFull());
}

TEST_CASE_FIXTURE(RtosFixture, "a queue with all spaces used is full") {
	Queue queue(2, sizeof(uint32_t));
	fill(queue, 2);
	CHECK(queue.IsFull());
	CHECK(queue.NumSpacesLeft() == 0);
	CHECK(queue.NumItems() == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "the exception of a failed queue creation names the failure") {
	HostRtos::failCreate(HostRtos::RtosObject::Queue);
	std::string what;
	try {
		Queue queue(2, sizeof(uint32_t));
	} catch (const QueueCreateException &e) {
		what = e.what();
	}
	CHECK(what == "Queue Constructor Failed");
}

TEST_CASE_FIXTURE(RtosFixture, "a queue create exception with details appends them to the message") {
	QueueCreateException e("(item queue)");
	CHECK(std::string(e.what()) == "Queue Constructor Failed (item queue)");
}

TEST_CASE_FIXTURE(RtosFixture, "deleting a queue deletes its kernel queue") {
	const size_t before = HostRtos::queueCount();
	Queue *queue = new Queue(2, sizeof(uint32_t));
	CHECK(HostRtos::queueCount() == before + 1);
	delete queue;
	CHECK(HostRtos::queueCount() == before);
}

// ---------------- Enqueue / Dequeue / Peek on the test thread ----------------

TEST_CASE_FIXTURE(RtosFixture, "items leave a queue in the order they were enqueued") {
	Queue queue(3, sizeof(uint32_t));
	fill(queue, 3);
	for (uint32_t expected = 1; expected <= 3; expected++) {
		uint32_t value = 0;
		REQUIRE(queue.Dequeue(&value, 0));
		CHECK(value == expected);
	}
	CHECK(queue.IsEmpty());
}

TEST_CASE_FIXTURE(RtosFixture, "a queue stores a copy of the item") {
	Queue queue(2, sizeof(Item12));
	Item12 item{0x11223344, 0x5566, {1, 2, 3, 4, 5, 6}};
	REQUIRE(queue.Enqueue(&item, 0));
	std::memset(&item, 0, sizeof(item)); // Changing the source after the enqueue does not change the queued item
	Item12 received{};
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received.a == 0x11223344);
	CHECK(received.b == 0x5566);
	CHECK(received.c[0] == 1);
	CHECK(received.c[5] == 6);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue without a timeout argument adds the item to a queue with space") {
	Queue queue(2, sizeof(uint32_t));
	uint32_t value = 77;
	CHECK(queue.Enqueue(&value));
	uint32_t received = 0;
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 77);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue without a timeout argument waits forever on a full queue") {
	Queue queue(1, sizeof(uint32_t));
	fill(queue, 1);
	uint32_t value = 2;
	CHECK_FALSE(queue.Enqueue(&value)); // The test thread can not wait: recorded instead of blocking forever
	CHECK(expectKernelFault("would block forever: xQueueSendToBack"));
	CHECK(queue.NumItems() == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue with a timeout of 0 to a full queue fails at once and keeps the items") {
	Queue queue(2, sizeof(uint32_t));
	fill(queue, 2);
	uint32_t value = 3;
	CHECK_FALSE(queue.Enqueue(&value, 0));
	CHECK(HostPlatform::nowUs() == 0);
	uint32_t received = 0;
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 1);
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 2);
	CHECK(queue.IsEmpty());
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue to a full queue on the test thread fails after its timeout of simulated time") {
	Queue queue(1, sizeof(uint32_t));
	fill(queue, 1);
	uint32_t value = 2;
	CHECK_FALSE(queue.Enqueue(&value, 7));
	CHECK(HostPlatform::nowUs() == 7000);
}

TEST_CASE_FIXTURE(RtosFixture, "a dequeue without a timeout argument returns an available item") {
	Queue queue(2, sizeof(uint32_t));
	fill(queue, 1);
	uint32_t received = 0;
	CHECK(queue.Dequeue(&received));
	CHECK(received == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "a dequeue with a timeout of 0 from an empty queue fails at once") {
	Queue queue(1, sizeof(uint32_t));
	uint32_t received = 5;
	CHECK_FALSE(queue.Dequeue(&received, 0));
	CHECK(received == 5);
	CHECK(HostPlatform::nowUs() == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a peek returns the front item and leaves it in the queue") {
	Queue queue(3, sizeof(uint32_t));
	fill(queue, 2);
	uint32_t peeked = 0;
	CHECK(queue.Peek(&peeked, 0));
	CHECK(peeked == 1);
	CHECK(queue.NumItems() == 2);
	uint32_t received = 0;
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "a peek without a timeout argument returns an available item") {
	Queue queue(1, sizeof(uint32_t));
	fill(queue, 1);
	uint32_t peeked = 0;
	CHECK(queue.Peek(&peeked));
	CHECK(peeked == 1);
	CHECK(queue.NumItems() == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "a peek with a timeout of 0 on an empty queue fails at once") {
	Queue queue(1, sizeof(uint32_t));
	uint32_t peeked = 9;
	CHECK_FALSE(queue.Peek(&peeked, 0));
	CHECK(peeked == 9);
}

TEST_CASE_FIXTURE(RtosFixture, "a peek on an empty queue on the test thread fails after its timeout of simulated time") {
	Queue queue(1, sizeof(uint32_t));
	uint32_t peeked = 0;
	CHECK_FALSE(queue.Peek(&peeked, 3));
	CHECK(HostPlatform::nowUs() == 3000);
}

TEST_CASE_FIXTURE(RtosFixture, "flushing a queue removes all items") {
	Queue queue(3, sizeof(uint32_t));
	fill(queue, 3);
	queue.Flush();
	CHECK(queue.IsEmpty());
	CHECK(queue.NumSpacesLeft() == 3);
	uint32_t received = 0;
	CHECK_FALSE(queue.Dequeue(&received, 0));
}

// ---------------- Blocking tasks ----------------

TEST_CASE_FIXTURE(RtosFixture, "a task waiting in a dequeue continues with the item enqueued later") {
	Queue queue(2, sizeof(uint32_t));
	uint32_t received = 0;
	bool result = false;
	QueueTestThread task([&] { result = queue.Dequeue(&received); });
	TaskHandle_t handle = task.startResumable();
	runReady();
	REQUIRE(HostRtos::isWaiting(handle));
	uint32_t value = 0xBEEF;
	REQUIRE(queue.Enqueue(&value, 0));
	runReady();
	CHECK(result);
	CHECK(received == 0xBEEF);
	CHECK(queue.IsEmpty());
}

TEST_CASE_FIXTURE(RtosFixture, "a task waiting in an enqueue to a full queue continues when an item was dequeued") {
	Queue queue(1, sizeof(uint32_t));
	fill(queue, 1);
	bool result = false;
	QueueTestThread task([&] {
		uint32_t value = 2;
		result = queue.Enqueue(&value);
	});
	TaskHandle_t handle = task.startResumable();
	runReady();
	REQUIRE(HostRtos::isWaiting(handle));
	CHECK_FALSE(result);
	uint32_t received = 0;
	REQUIRE(queue.Dequeue(&received, 0));
	runReady();
	CHECK(result);
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "a dequeue with a timeout in a task fails when the timeout of simulated time has passed") {
	Queue queue(1, sizeof(uint32_t));
	bool done = false;
	bool result = true;
	QueueTestThread task([&] {
		uint32_t received = 0;
		result = queue.Dequeue(&received, 10);
		done = true;
	});
	task.startResumable();
	runReady();
	advance(9);
	CHECK_FALSE(done);
	advance(1);
	CHECK(done);
	CHECK_FALSE(result);
}

TEST_CASE_FIXTURE(RtosFixture, "a dequeue with a timeout in a task returns an item that arrives before the timeout") {
	Queue queue(1, sizeof(uint32_t));
	bool result = false;
	uint32_t received = 0;
	QueueTestThread task([&] { result = queue.Dequeue(&received, 10); });
	task.startResumable();
	runReady();
	advance(4);
	uint32_t value = 31;
	REQUIRE(queue.Enqueue(&value, 0));
	runReady();
	CHECK(result);
	CHECK(received == 31);
	CHECK(HostPlatform::nowUs() == 4000);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue with a timeout in a task fails when the queue stays full for the timeout") {
	Queue queue(1, sizeof(uint32_t));
	fill(queue, 1);
	bool done = false;
	bool result = true;
	QueueTestThread task([&] {
		uint32_t value = 2;
		result = queue.Enqueue(&value, 5);
		done = true;
	});
	task.startResumable();
	runReady();
	advance(4);
	CHECK_FALSE(done);
	advance(1);
	CHECK(done);
	CHECK_FALSE(result);
	CHECK(queue.NumItems() == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "a peek in a task waits for an item and leaves it in the queue") {
	Queue queue(1, sizeof(uint32_t));
	bool result = false;
	uint32_t peeked = 0;
	QueueTestThread task([&] { result = queue.Peek(&peeked); });
	TaskHandle_t handle = task.startResumable();
	runReady();
	REQUIRE(HostRtos::isWaiting(handle));
	uint32_t value = 12;
	REQUIRE(queue.Enqueue(&value, 0));
	runReady();
	CHECK(result);
	CHECK(peeked == 12);
	CHECK(queue.NumItems() == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "flushing a full queue lets a task waiting to enqueue continue") {
	Queue queue(1, sizeof(uint32_t));
	fill(queue, 1);
	bool result = false;
	QueueTestThread task([&] {
		uint32_t value = 2;
		result = queue.Enqueue(&value);
	});
	task.startResumable();
	runReady();
	REQUIRE_FALSE(result);
	queue.Flush();
	runReady();
	CHECK(result);
	uint32_t received = 0;
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 2);
}

// ---------------- Interrupt variants ----------------

TEST_CASE_FIXTURE(RtosFixture, "an enqueue from an interrupt adds the item at the back") {
	Queue queue(3, sizeof(uint32_t));
	fill(queue, 1);
	BaseType_t woken = pdFALSE;
	uint32_t value = 2;
	{
		HostPlatform::IsrContext isr;
		CHECK(queue.EnqueueFromISR(&value, &woken));
	}
	CHECK(woken == pdFALSE); // Nobody waits
	uint32_t received = 0;
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 1);
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue from an interrupt to a full queue fails") {
	Queue queue(1, sizeof(uint32_t));
	fill(queue, 1);
	BaseType_t woken = pdFALSE;
	uint32_t value = 2;
	bool result = true;
	{
		HostPlatform::IsrContext isr;
		result = queue.EnqueueFromISR(&value, &woken);
	}
	CHECK_FALSE(result);
	CHECK(queue.NumItems() == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue from an interrupt reports that a task waiting for the item was woken") {
	Queue queue(1, sizeof(uint32_t));
	uint32_t received = 0;
	QueueTestThread task([&] { queue.Dequeue(&received); }, "QT", 3);
	task.startResumable();
	runReady();
	BaseType_t woken = pdFALSE;
	uint32_t value = 4;
	{
		HostPlatform::IsrContext isr;
		REQUIRE(queue.EnqueueFromISR(&value, &woken));
	}
	CHECK(woken == pdTRUE);
	runReady();
	CHECK(received == 4);
}

TEST_CASE_FIXTURE(RtosFixture, "a dequeue from an interrupt returns the front item") {
	Queue queue(2, sizeof(uint32_t));
	fill(queue, 2);
	BaseType_t woken = pdFALSE;
	uint32_t received = 0;
	bool result = false;
	{
		HostPlatform::IsrContext isr;
		result = queue.DequeueFromISR(&received, &woken);
	}
	CHECK(result);
	CHECK(received == 1);
	CHECK(queue.NumItems() == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "a dequeue from an interrupt on an empty queue fails") {
	Queue queue(1, sizeof(uint32_t));
	BaseType_t woken = pdFALSE;
	uint32_t received = 3;
	bool result = true;
	{
		HostPlatform::IsrContext isr;
		result = queue.DequeueFromISR(&received, &woken);
	}
	CHECK_FALSE(result);
	CHECK(received == 3);
}

TEST_CASE_FIXTURE(RtosFixture, "a dequeue from an interrupt reports that a task waiting to enqueue was woken") {
	Queue queue(1, sizeof(uint32_t));
	fill(queue, 1);
	bool sent = false;
	QueueTestThread task([&] {
		uint32_t value = 2;
		sent = queue.Enqueue(&value);
	}, "QT", 3);
	task.startResumable();
	runReady();
	BaseType_t woken = pdFALSE;
	uint32_t received = 0;
	{
		HostPlatform::IsrContext isr;
		REQUIRE(queue.DequeueFromISR(&received, &woken));
	}
	CHECK(woken == pdTRUE);
	runReady();
	CHECK(sent);
}

TEST_CASE_FIXTURE(RtosFixture, "a peek from an interrupt returns the front item and leaves it in the queue") {
	Queue queue(2, sizeof(uint32_t));
	fill(queue, 2);
	uint32_t peeked = 0;
	bool result = false;
	{
		HostPlatform::IsrContext isr;
		result = queue.PeekFromISR(&peeked);
	}
	CHECK(result);
	CHECK(peeked == 1);
	CHECK(queue.NumItems() == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "a peek from an interrupt on an empty queue fails") {
	Queue queue(1, sizeof(uint32_t));
	uint32_t peeked = 8;
	bool result = true;
	{
		HostPlatform::IsrContext isr;
		result = queue.PeekFromISR(&peeked);
	}
	CHECK_FALSE(result);
	CHECK(peeked == 8);
}

// ---------------- Deque ----------------

TEST_CASE_FIXTURE(RtosFixture, "an item enqueued to the front of a deque is dequeued first") {
	Deque deque(3, sizeof(uint32_t));
	fill(deque, 2);
	uint32_t value = 99;
	CHECK(deque.EnqueueToFront(&value, 0));
	uint32_t received = 0;
	REQUIRE(deque.Dequeue(&received, 0));
	CHECK(received == 99);
	REQUIRE(deque.Dequeue(&received, 0));
	CHECK(received == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue to the front without a timeout argument adds the item to a deque with space") {
	Deque deque(2, sizeof(uint32_t));
	uint32_t value = 5;
	CHECK(deque.EnqueueToFront(&value));
	CHECK(deque.NumItems() == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue to the front of a full deque fails after its timeout") {
	Deque deque(1, sizeof(uint32_t));
	fill(deque, 1);
	uint32_t value = 5;
	CHECK_FALSE(deque.EnqueueToFront(&value, 2));
	CHECK(HostPlatform::nowUs() == 2000);
	uint32_t received = 0;
	REQUIRE(deque.Dequeue(&received, 0));
	CHECK(received == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue to the front from an interrupt puts the item first") {
	Deque deque(3, sizeof(uint32_t));
	fill(deque, 2);
	BaseType_t woken = pdFALSE;
	uint32_t value = 42;
	bool result = false;
	{
		HostPlatform::IsrContext isr;
		result = deque.EnqueueToFrontFromISR(&value, &woken);
	}
	CHECK(result);
	uint32_t received = 0;
	REQUIRE(deque.Dequeue(&received, 0));
	CHECK(received == 42);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue to the front from an interrupt to a full deque fails") {
	Deque deque(1, sizeof(uint32_t));
	fill(deque, 1);
	BaseType_t woken = pdFALSE;
	uint32_t value = 42;
	bool result = true;
	{
		HostPlatform::IsrContext isr;
		result = deque.EnqueueToFrontFromISR(&value, &woken);
	}
	CHECK_FALSE(result);
	uint32_t received = 0;
	REQUIRE(deque.Dequeue(&received, 0));
	CHECK(received == 1);
}

// ---------------- BinaryQueue ----------------

TEST_CASE_FIXTURE(RtosFixture, "a binary queue holds a single item") {
	BinaryQueue queue(sizeof(uint32_t));
	CHECK(queue.NumSpacesLeft() == 1);
	uint32_t value = 1;
	CHECK(queue.Enqueue(&value));
	CHECK(queue.IsFull());
	CHECK(queue.NumItems() == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue to a full binary queue overwrites the item") {
	BinaryQueue queue(sizeof(uint32_t));
	uint32_t first = 1;
	uint32_t second = 2;
	REQUIRE(queue.Enqueue(&first));
	CHECK(queue.Enqueue(&second));
	CHECK(queue.NumItems() == 1);
	uint32_t received = 0;
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue through the Queue interface of a binary queue overwrites the item") {
	BinaryQueue binary(sizeof(uint32_t));
	Queue &queue = binary;
	uint32_t first = 1;
	uint32_t second = 2;
	REQUIRE(queue.Enqueue(&first));
	CHECK(queue.Enqueue(&second)); // Virtual: BinaryQueue::Enqueue, does not wait on the full queue
	uint32_t received = 0;
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "an enqueue from an interrupt to a full binary queue overwrites the item") {
	BinaryQueue queue(sizeof(uint32_t));
	uint32_t first = 1;
	uint32_t second = 2;
	REQUIRE(queue.Enqueue(&first));
	BaseType_t woken = pdFALSE;
	bool result = false;
	{
		HostPlatform::IsrContext isr;
		result = queue.EnqueueFromISR(&second, &woken);
	}
	CHECK(result);
	CHECK(queue.NumItems() == 1);
	uint32_t received = 0;
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 2);
}

// NEEDS REVIEW 144: BinaryQueue overrides only Enqueue(item) and EnqueueFromISR; the timed overload
// Queue::Enqueue(item, Timeout) is not overridden (and hidden on a BinaryQueue object), so through the Queue interface
// a timed enqueue to a full binary queue waits / fails like a normal queue instead of overwriting. Intended?
TEST_CASE_FIXTURE(RtosFixture, "characterization: a timed enqueue through the Queue interface to a full binary queue fails instead of overwriting") {
	BinaryQueue binary(sizeof(uint32_t));
	Queue &queue = binary;
	uint32_t first = 1;
	uint32_t second = 2;
	REQUIRE(queue.Enqueue(&first));
	CHECK_FALSE(queue.Enqueue(&second, 0));
	uint32_t received = 0;
	REQUIRE(queue.Dequeue(&received, 0));
	CHECK(received == 1);
}

}
