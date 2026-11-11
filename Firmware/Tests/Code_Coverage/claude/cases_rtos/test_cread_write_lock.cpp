/*
 * Tests of cpp_freertos::ReadWriteLockPreferReader / ReadWriteLockPreferWriter (FFBoard/Src/cread_write_lock.cpp,
 * FFBoard/Inc/read_write_lock.hpp; tier A vendor code: suspected defects are reported, never fixed) on the fake
 * FreeRTOS API F11 (support/HostRtosExt.h).
 *
 * The locks block indefinitely (no timeouts in the API, read_write_lock.hpp). Contention is tested with resumable
 * tasks that take the lock, record what they did and park on a gate semaphore until the test lets them go on. The
 * fake runs ready tasks in creation order (no priorities), which decides which of several waiting tasks gets a free
 * semaphore first; the tests only rely on orders the lock itself enforces.
 *
 * Host vs target: the create failures throw ReadWriteLockCreateException on the host; the F407 build defines
 * CPP_FREERTOS_NO_EXCEPTIONS and stops in configASSERT instead (NEEDS REVIEW 142).
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"

#include "read_write_lock.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

using namespace cpp_freertos;
using HostRtos::RtosObject;

namespace {

/** Thread running a function set by the test (resumable: continues after a blocking call) */
class RwTask : public Thread {
public:
	explicit RwTask(std::function<void()> body, const std::string &name, UBaseType_t priority = 1)
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

/** Records the order of events of the tasks */
struct Log {
	std::vector<std::string> events;
	void add(const std::string &e) { events.push_back(e); }
	bool has(const std::string &e) const {
		for (const std::string &x : events) {
			if (x == e) {
				return true;
			}
		}
		return false;
	}
};

/** A reader task: lock, log "<name> in", wait for the gate, unlock, log "<name> out" */
std::function<void()> reader(ReadWriteLock &lock, BinarySemaphore &gate, Log &log, const std::string &name) {
	return [&lock, &gate, &log, name] {
		lock.ReaderLock();
		log.add(name + " in");
		gate.Take();
		lock.ReaderUnlock();
		log.add(name + " out");
	};
}

/** A writer task: like reader() with WriterLock / WriterUnlock */
std::function<void()> writer(ReadWriteLock &lock, BinarySemaphore &gate, Log &log, const std::string &name) {
	return [&lock, &gate, &log, name] {
		lock.WriterLock();
		log.add(name + " in");
		gate.Take();
		lock.WriterUnlock();
		log.add(name + " out");
	};
}

} // namespace

TEST_SUITE("ReadWriteLock") {

// ---------------- Creation and destruction ----------------

TEST_CASE_FIXTURE(RtosFixture, "a reader-preferring lock owns one mutex and one binary semaphore for its lifetime") {
	const size_t before = HostRtos::semaphoreCount();
	auto *lock = new ReadWriteLockPreferReader();
	CHECK(HostRtos::semaphoreCount() == before + 2);
	CHECK(HostRtos::createCalls(RtosObject::Mutex) == 1);
	CHECK(HostRtos::createCalls(RtosObject::BinarySemaphore) == 1);
	delete lock;
	CHECK(HostRtos::semaphoreCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "a writer-preferring lock owns two mutexes and two binary semaphores for its lifetime") {
	const size_t before = HostRtos::semaphoreCount();
	auto *lock = new ReadWriteLockPreferWriter();
	CHECK(HostRtos::semaphoreCount() == before + 4);
	CHECK(HostRtos::createCalls(RtosObject::Mutex) == 2);
	CHECK(HostRtos::createCalls(RtosObject::BinarySemaphore) == 2);
	delete lock;
	CHECK(HostRtos::semaphoreCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "a lock whose read mutex can not be created throws ReadWriteLockCreateException") {
	const size_t before = HostRtos::semaphoreCount();
	HostRtos::failCreate(RtosObject::Mutex);
	CHECK_THROWS_WITH_AS(ReadWriteLockPreferReader(), "ReadWriteLock Constructor Failed", ReadWriteLockCreateException);
	CHECK(HostRtos::semaphoreCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "a lock whose resource semaphore can not be created throws and deletes its read mutex") {
	const size_t before = HostRtos::semaphoreCount();
	HostRtos::failCreate(RtosObject::BinarySemaphore);
	CHECK_THROWS_AS(ReadWriteLockPreferReader(), ReadWriteLockCreateException);
	CHECK(HostRtos::semaphoreCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "a writer-preferring lock whose write mutex can not be created throws and deletes the base semaphores") {
	const size_t before = HostRtos::semaphoreCount();
	HostRtos::failCreate(RtosObject::Mutex, 1); // The first mutex (read lock) is created, the second (write lock) fails
	CHECK_THROWS_AS(ReadWriteLockPreferWriter(), ReadWriteLockCreateException);
	CHECK(HostRtos::createCalls(RtosObject::Mutex) == 2);
	CHECK(HostRtos::semaphoreCount() == before);
}

TEST_CASE_FIXTURE(RtosFixture, "a writer-preferring lock whose block-readers semaphore can not be created throws and deletes all semaphores") {
	const size_t before = HostRtos::semaphoreCount();
	HostRtos::failCreate(RtosObject::BinarySemaphore, 1);
	CHECK_THROWS_AS(ReadWriteLockPreferWriter(), ReadWriteLockCreateException);
	CHECK(HostRtos::createCalls(RtosObject::BinarySemaphore) == 2);
	CHECK(HostRtos::semaphoreCount() == before);
}

// ---------------- Reader preference ----------------

TEST_CASE_FIXTURE(RtosFixture, "a reader-preferring lock is free again after a writer unlocked it") {
	ReadWriteLockPreferReader lock;
	lock.WriterLock();
	lock.WriterUnlock();
	lock.ReaderLock(); // Would block forever (kernel fault) if the lock was still held
	lock.ReaderUnlock();
	lock.WriterLock();
	lock.WriterUnlock();
}

TEST_CASE_FIXTURE(RtosFixture, "several readers hold a reader-preferring lock at the same time") {
	ReadWriteLockPreferReader lock;
	BinarySemaphore gate;
	Log log;
	RwTask a(reader(lock, gate, log, "A"), "RA");
	RwTask b(reader(lock, gate, log, "B"), "RB");
	a.startResumable();
	b.startResumable();
	runReady();
	CHECK(log.has("A in"));
	CHECK(log.has("B in"));
	gate.Give();
	runReady();
	gate.Give();
	runReady();
	CHECK(log.has("A out"));
	CHECK(log.has("B out"));
}

TEST_CASE_FIXTURE(RtosFixture, "a reader waits while a writer holds a reader-preferring lock") {
	ReadWriteLockPreferReader lock;
	BinarySemaphore gate(true);
	Log log;
	lock.WriterLock();
	RwTask a(reader(lock, gate, log, "A"), "RA");
	TaskHandle_t handle = a.startResumable();
	runReady();
	CHECK_FALSE(log.has("A in"));
	CHECK(HostRtos::isWaiting(handle));
	lock.WriterUnlock();
	runReady();
	CHECK(log.has("A out"));
}

TEST_CASE_FIXTURE(RtosFixture, "a second writer waits until the first writer unlocks a reader-preferring lock") {
	ReadWriteLockPreferReader lock;
	BinarySemaphore gate;
	Log log;
	RwTask w1(writer(lock, gate, log, "W1"), "W1");
	RwTask w2(writer(lock, gate, log, "W2"), "W2");
	w1.startResumable();
	w2.startResumable();
	runReady();
	CHECK(log.events == std::vector<std::string>{"W1 in"});
	gate.Give();
	runReady();
	CHECK(log.events == std::vector<std::string>{"W1 in", "W1 out", "W2 in"});
	gate.Give();
	runReady();
	CHECK(log.has("W2 out"));
}

TEST_CASE_FIXTURE(RtosFixture, "with reader preference a new reader enters while a writer waits for the active reader") {
	ReadWriteLockPreferReader lock;
	BinarySemaphore gate;
	Log log;
	RwTask r1(reader(lock, gate, log, "R1"), "R1");
	RwTask w(writer(lock, gate, log, "W"), "W");
	RwTask r2(reader(lock, gate, log, "R2"), "R2");
	r1.startResumable();
	runReady();
	w.startResumable();
	runReady();
	r2.startResumable();
	runReady();
	CHECK(log.events == std::vector<std::string>{"R1 in", "R2 in"});
	gate.Give(); // R1 leaves; R2 still reads
	runReady();
	CHECK_FALSE(log.has("W in"));
	gate.Give(); // R2 leaves: the writer enters
	runReady();
	CHECK(log.has("W in"));
	gate.Give();
	runReady();
	CHECK(log.has("W out"));
}

TEST_CASE_FIXTURE(RtosFixture, "the last reader releases a reader-preferring lock even if another reader locked it first") {
	ReadWriteLockPreferReader lock;
	BinarySemaphore gateA;
	BinarySemaphore gateB;
	Log log;
	RwTask a(reader(lock, gateA, log, "A"), "RA"); // First reader: takes the resource semaphore
	RwTask b(reader(lock, gateB, log, "B"), "RB");
	a.startResumable();
	b.startResumable();
	runReady();
	gateA.Give(); // A leaves first
	runReady();
	gateB.Give(); // B (last reader) gives the resource semaphore A took: allowed for a binary semaphore
	runReady();
	CHECK(log.has("B out"));
	lock.WriterLock(); // Free again (a kernel fault "would block forever" otherwise)
	lock.WriterUnlock();
}

// ---------------- Writer preference ----------------

TEST_CASE_FIXTURE(RtosFixture, "a writer-preferring lock is free again after a writer unlocked it") {
	ReadWriteLockPreferWriter lock;
	lock.WriterLock();
	lock.WriterUnlock();
	lock.ReaderLock();
	lock.ReaderUnlock();
	lock.WriterLock();
	lock.WriterUnlock();
}

TEST_CASE_FIXTURE(RtosFixture, "several readers hold a writer-preferring lock at the same time") {
	ReadWriteLockPreferWriter lock;
	BinarySemaphore gate;
	Log log;
	RwTask a(reader(lock, gate, log, "A"), "RA");
	RwTask b(reader(lock, gate, log, "B"), "RB");
	a.startResumable();
	b.startResumable();
	runReady();
	CHECK(log.has("A in"));
	CHECK(log.has("B in"));
	gate.Give();
	runReady();
	gate.Give();
	runReady();
	CHECK(log.has("A out"));
	CHECK(log.has("B out"));
}

TEST_CASE_FIXTURE(RtosFixture, "a writer waits until the last reader has unlocked a writer-preferring lock") {
	ReadWriteLockPreferWriter lock;
	BinarySemaphore gate(true);
	Log log;
	lock.ReaderLock();
	lock.ReaderLock();
	RwTask w(writer(lock, gate, log, "W"), "W");
	w.startResumable();
	runReady();
	lock.ReaderUnlock();
	runReady();
	CHECK_FALSE(log.has("W in"));
	lock.ReaderUnlock();
	runReady();
	CHECK(log.has("W out"));
}

TEST_CASE_FIXTURE(RtosFixture, "a reader waits while a writer holds a writer-preferring lock") {
	ReadWriteLockPreferWriter lock;
	BinarySemaphore gate(true);
	Log log;
	lock.WriterLock();
	RwTask a(reader(lock, gate, log, "A"), "RA");
	TaskHandle_t handle = a.startResumable();
	runReady();
	CHECK_FALSE(log.has("A in"));
	CHECK(HostRtos::isWaiting(handle));
	lock.WriterUnlock();
	runReady();
	CHECK(log.has("A out"));
}

TEST_CASE_FIXTURE(RtosFixture, "with writer preference a new reader waits behind a writer that waits for the active reader") {
	ReadWriteLockPreferWriter lock;
	BinarySemaphore gate;
	Log log;
	RwTask r1(reader(lock, gate, log, "R1"), "R1");
	RwTask w(writer(lock, gate, log, "W"), "W");
	RwTask r2(reader(lock, gate, log, "R2"), "R2");
	r1.startResumable();
	runReady();
	w.startResumable();
	runReady();
	r2.startResumable();
	runReady();
	CHECK(log.events == std::vector<std::string>{"R1 in"});
	gate.Give(); // R1 leaves: the writer enters before the new reader
	runReady();
	CHECK(log.events == std::vector<std::string>{"R1 in", "R1 out", "W in"});
	gate.Give(); // The writer leaves: the reader enters
	runReady();
	CHECK(log.events == std::vector<std::string>{"R1 in", "R1 out", "W in", "W out", "R2 in"});
	gate.Give();
	runReady();
	CHECK(log.has("R2 out"));
}

TEST_CASE_FIXTURE(RtosFixture, "with writer preference queued writers all go before a waiting reader") {
	ReadWriteLockPreferWriter lock;
	BinarySemaphore gate;
	Log log;
	RwTask w1(writer(lock, gate, log, "W1"), "W1");
	RwTask r(reader(lock, gate, log, "R"), "R");
	RwTask w2(writer(lock, gate, log, "W2"), "W2");
	w1.startResumable();
	runReady();
	r.startResumable();
	runReady();
	w2.startResumable();
	runReady();
	CHECK(log.events == std::vector<std::string>{"W1 in"});
	gate.Give(); // W1 leaves: W2 is still counted as a writer, so the reader stays blocked
	runReady();
	CHECK(log.events == std::vector<std::string>{"W1 in", "W1 out", "W2 in"});
	gate.Give(); // The last writer leaves: readers are let in again
	runReady();
	CHECK(log.events == std::vector<std::string>{"W1 in", "W1 out", "W2 in", "W2 out", "R in"});
	gate.Give();
	runReady();
	CHECK(log.has("R out"));
}

TEST_CASE_FIXTURE(RtosFixture, "the last reader releases a writer-preferring lock even if another reader locked it first") {
	ReadWriteLockPreferWriter lock;
	BinarySemaphore gateA;
	BinarySemaphore gateB;
	Log log;
	RwTask a(reader(lock, gateA, log, "A"), "RA");
	RwTask b(reader(lock, gateB, log, "B"), "RB");
	a.startResumable();
	b.startResumable();
	runReady();
	gateA.Give();
	runReady();
	gateB.Give();
	runReady();
	CHECK(log.has("B out"));
	lock.WriterLock();
	lock.WriterUnlock();
}

}
