/*
 * Ring buffer (ringbuffer.c) and its C++ wrapper (ringbufferwrapper.cpp)
 *
 * The C functions do not check anything: the caller has to make sure there is enough data or
 * free space. The wrapper (used by CmdParser) clamps every request to the available data/space.
 * A buffer of length N (power of two) stores at most N-1 bytes (one slot stays free to tell
 * "full" from "empty").
 */
#include "doctest.h"
#include "TestHelpers.h"

#include "ringbuffer.h"
#include "ringbufferwrapper.h"

#include <array>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <vector>

namespace {


/** Raw C ring buffer with its own storage */
struct RawRing {
	std::array<uint8_t, 8> storage{};
	RingBuffer rb{};
	RawRing() { REQUIRE(ringBufferInit(&rb, storage.data(), storage.size()) == 1); }

	void append(const std::string &s) { ringBufferAppendMultiple(&rb, reinterpret_cast<const uint8_t *>(s.data()), s.size()); }
	std::string get(size_t len) {
		std::string out(len, '\0');
		ringBufferGetMultiple(&rb, reinterpret_cast<uint8_t *>(out.data()), len);
		return out;
	}
	std::string peek(size_t len) {
		std::string out(len, '\0');
		ringBufferPeekMultiple(&rb, reinterpret_cast<uint8_t *>(out.data()), len);
		return out;
	}
};

std::string getAll(RingBufferWrapper &w) {
	std::string out(w.length(), '\0');
	size_t n = w.getMultiple(reinterpret_cast<uint8_t *>(out.data()), out.size());
	out.resize(n);
	return out;
}

size_t append(RingBufferWrapper &w, const std::string &s) {
	return w.appendMultiple(reinterpret_cast<const uint8_t *>(s.data()), s.size());
}

} // namespace

TEST_SUITE("ringbuffer") {

TEST_CASE("init accepts only power of two lengths") {
	uint8_t storage[16];
	RingBuffer rb{};
	CHECK(ringBufferInit(&rb, storage, 0) == 0);
	CHECK(ringBufferInit(&rb, storage, 3) == 0);
	CHECK(ringBufferInit(&rb, storage, 6) == 0);
	CHECK(ringBufferInit(&rb, storage, 12) == 0);
	CHECK(ringBufferInit(&rb, storage, 1) == 1);
	CHECK(ringBufferInit(&rb, storage, 2) == 1);
	CHECK(ringBufferInit(&rb, storage, 16) == 1);
}

TEST_CASE("a new buffer is empty and can store one byte less than its length") {
	RawRing r;
	CHECK(ringBufferEmpty(&r.rb) == 1);
	CHECK(ringBufferLen(&r.rb) == 0);
	CHECK(ringBufferMaxSize(&r.rb) == 7);
	CHECK(ringBufferFreeSpace(&r.rb) == 7);
}

TEST_CASE("bytes appended one by one are returned in the same order") {
	RawRing r;
	ringBufferAppendOne(&r.rb, 'a');
	ringBufferAppendOne(&r.rb, 'b');
	CHECK(ringBufferEmpty(&r.rb) == 0);
	CHECK(ringBufferLen(&r.rb) == 2);
	CHECK(ringBufferFreeSpace(&r.rb) == 5);
	CHECK(ringBufferGetOne(&r.rb) == 'a');
	CHECK(ringBufferGetOne(&r.rb) == 'b');
	CHECK(ringBufferEmpty(&r.rb) == 1);
}

TEST_CASE("peekOne returns the oldest byte without removing it") {
	RawRing r;
	r.append("xy");
	CHECK(ringBufferPeekOne(&r.rb) == 'x');
	CHECK(ringBufferPeekOne(&r.rb) == 'x');
	CHECK(ringBufferLen(&r.rb) == 2);
}

TEST_CASE("multiple bytes are appended, peeked and removed in order") {
	RawRing r;
	r.append("hello");
	CHECK(ringBufferLen(&r.rb) == 5);
	CHECK(r.peek(3) == "hel");
	CHECK(ringBufferLen(&r.rb) == 5);
	CHECK(r.get(3) == "hel");
	CHECK(ringBufferLen(&r.rb) == 2);
	CHECK(r.get(2) == "lo");
	CHECK(ringBufferEmpty(&r.rb) == 1);
}

TEST_CASE("a buffer filled to its maximum size has no free space") {
	RawRing r;
	r.append("1234567");
	CHECK(ringBufferLen(&r.rb) == 7);
	CHECK(ringBufferFreeSpace(&r.rb) == 0);
	CHECK(ringBufferEmpty(&r.rb) == 0);
	CHECK(r.get(7) == "1234567");
}

TEST_CASE("appending up to the end of the storage wraps the write position to the start") {
	RawRing r;
	r.append("abc");
	CHECK(r.get(3) == "abc");
	r.append("defgh"); // Positions 3..7, exactly up to the end
	CHECK(ringBufferLen(&r.rb) == 5);
	ringBufferAppendOne(&r.rb, 'i'); // Position 0
	CHECK(r.storage[0] == 'i');
	CHECK(r.get(6) == "defghi");
}

TEST_CASE("multiple bytes wrap around the end of the storage") {
	RawRing r;
	r.append("123456");
	CHECK(r.get(6) == "123456");
	r.append("abcde"); // 6,7 then 0,1,2
	CHECK(std::string(r.storage.begin(), r.storage.begin() + 3) == "cde");
	CHECK(ringBufferLen(&r.rb) == 5);	   // Read position behind the write position
	CHECK(ringBufferFreeSpace(&r.rb) == 2);
	CHECK(r.peek(5) == "abcde");
	CHECK(r.get(5) == "abcde");
	CHECK(ringBufferEmpty(&r.rb) == 1);
}

TEST_CASE("single bytes wrap around the end of the storage") {
	RawRing r;
	for (int round = 0; round < 3; round++) {
		for (uint8_t i = 0; i < 7; i++) {
			ringBufferAppendOne(&r.rb, (uint8_t)(round * 10 + i));
		}
		CHECK(ringBufferLen(&r.rb) == 7);
		for (uint8_t i = 0; i < 7; i++) {
			CHECK(ringBufferGetOne(&r.rb) == round * 10 + i);
		}
	}
}

TEST_CASE("discard removes bytes and clear empties the buffer") {
	RawRing r;
	r.append("abcdef");
	ringBufferDiscardMultiple(&r.rb, 2);
	CHECK(ringBufferPeekOne(&r.rb) == 'c');
	CHECK(ringBufferLen(&r.rb) == 4);
	ringBufferClear(&r.rb);
	CHECK(ringBufferEmpty(&r.rb) == 1);
	CHECK(ringBufferLen(&r.rb) == 0);
}

// NEEDS REVIEW 20: the C functions do not check the free space. Appending to a full buffer moves the write
// position onto the read position, so all stored bytes are lost and the buffer looks empty. The only user
// (RingBufferWrapper) clamps to the free space first, so this is reachable only through the raw C API.
TEST_CASE("characterization: appending one byte to a full raw buffer makes it look empty") {
	RawRing r;
	r.append("1234567");
	REQUIRE(ringBufferFreeSpace(&r.rb) == 0);
	ringBufferAppendOne(&r.rb, '8');
	CHECK(ringBufferEmpty(&r.rb) == 1);
	CHECK(ringBufferLen(&r.rb) == 0);
}

} // TEST_SUITE

// ---------------------------------------------------------------------------
TEST_SUITE("RingBufferWrapper") {

TEST_CASE("a wrapper with a power of two size is valid and empty") {
	RingBufferWrapper w(16);
	CHECK(w.isValid());
	CHECK(w.empty());
	CHECK(w.length() == 0);
	CHECK(w.capacity() == 15);
	CHECK(w.freeSpace() == 15);
}

TEST_CASE("a wrapper with a size that is not a power of two is invalid") {
	// Constructed in own storage and intentionally never destroyed: the destructor of an invalid wrapper
	// created with a size deletes an uninitialised pointer (see the skipped KNOWN ISSUE below).
	alignas(RingBufferWrapper) unsigned char storage[sizeof(RingBufferWrapper)] = {};
	RingBufferWrapper *w = new (storage) RingBufferWrapper(6);
	CHECK_FALSE(w->isValid());
	RingBufferWrapper *zero = new (storage) RingBufferWrapper(0);
	CHECK_FALSE(zero->isValid());
}

TEST_CASE("an invalid wrapper created with a size is empty, has no capacity and can be destroyed" * doctest::skip() *
		doctest::description("KNOWN ISSUE (undefined behaviour): (bug 18) RingBufferWrapper(uint32_t) returns early for sizes that are not a power of two (ringbufferwrapper.cpp:29-34) without initialising the RingBuffer member. head, tail, sizeMask and data stay indeterminate, so capacity() and freeSpace() return garbage, and ~RingBufferWrapper deletes the indeterminate data pointer (ringbufferwrapper.cpp:45-50). Skipped in every build: the test itself would read indeterminate values and can crash the run. Unreachable in the firmware today: CmdParser is only constructed with 512 and 1024/4096")) {
	{
		RingBufferWrapper w(6);
		CHECK_FALSE(w.isValid());
		CHECK(w.capacity() == 0);
		CHECK(w.freeSpace() == 0);
	}
}

TEST_CASE("a wrapper uses the storage it is given and frees it when destroyed") {
	uint8_t *data = new uint8_t[8]();
	{
		RingBufferWrapper w(data, 8); // Takes ownership (delete[] in the destructor; checked by LeakSanitizer)
		CHECK(w.isValid());
		CHECK(w.capacity() == 7);
		CHECK(append(w, "abc") == 3);
		CHECK(data[0] == 'a');
		CHECK(data[2] == 'c');
	}
}

TEST_CASE("a heap allocated wrapper frees its storage when deleted") {
	std::unique_ptr<RingBufferWrapper> w = std::make_unique<RingBufferWrapper>(4);
	CHECK(w->isValid());
	CHECK(append(*w, "abc") == 3);
	w.reset(); // Deleting destructor, storage freed (checked by LeakSanitizer)
	CHECK(w == nullptr);
}

TEST_CASE("a wrapper given storage with a length that is not a power of two is invalid and frees the storage") {
	uint8_t *data = new uint8_t[6]();
	RingBufferWrapper w(data, 6); // Storage is deleted by the constructor (no leak, no double free under ASan)
	CHECK_FALSE(w.isValid());
}

TEST_CASE("bytes are returned in the order they were appended") {
	RingBufferWrapper w(8);
	CHECK(w.appendOne('a') == 1);
	CHECK(append(w, "bc") == 2);
	CHECK_FALSE(w.empty());
	CHECK(w.length() == 3);
	CHECK(w.freeSpace() == 4);
	uint8_t c = 0;
	CHECK(w.getOne(&c) == 1);
	CHECK(c == 'a');
	CHECK(getAll(w) == "bc");
	CHECK(w.empty());
}

TEST_CASE("peek returns data without removing it") {
	RingBufferWrapper w(8);
	append(w, "xyz");
	uint8_t c = 0;
	CHECK(w.peekOne(&c) == 1);
	CHECK(c == 'x');
	uint8_t two[2] = {};
	CHECK(w.peekMultiple(two, 2) == 2);
	CHECK(two[0] == 'x');
	CHECK(two[1] == 'y');
	CHECK(w.length() == 3);
}

TEST_CASE("appending to a full wrapper drops the bytes that do not fit") {
	RingBufferWrapper w(8);
	CHECK(append(w, "0123456789") == 7); // Only the free space is used
	CHECK(w.freeSpace() == 0);
	CHECK(w.appendOne('x') == 0);
	CHECK(getAll(w) == "0123456"); // Stored data is not overwritten
}

TEST_CASE("reading more than is stored returns only the stored bytes") {
	RingBufferWrapper w(8);
	append(w, "ab");
	uint8_t out[8];
	std::memset(out, '-', sizeof(out));
	CHECK(w.peekMultiple(out, 8) == 2);
	CHECK(w.getMultiple(out, 8) == 2);
	CHECK(out[0] == 'a');
	CHECK(out[1] == 'b');
	CHECK(out[2] == '-'); // Not written
	CHECK(w.empty());
}

TEST_CASE("reading from an empty wrapper returns nothing and leaves the destination unchanged") {
	RingBufferWrapper w(8);
	uint8_t c = '-';
	CHECK(w.getOne(&c) == 0);
	CHECK(w.peekOne(&c) == 0);
	CHECK(c == '-');
	CHECK(w.discardMultiple(3) == 0);
	CHECK(w.empty());
}

TEST_CASE("discard removes at most the stored bytes") {
	RingBufferWrapper w(8);
	append(w, "abcd");
	CHECK(w.discardMultiple(1) == 1);
	CHECK(w.discardMultiple(10) == 3);
	CHECK(w.empty());
	append(w, "ef");
	CHECK(getAll(w) == "ef");
}

TEST_CASE("clean empties the wrapper") {
	RingBufferWrapper w(8);
	append(w, "abcdef");
	w.clean();
	CHECK(w.empty());
	CHECK(w.freeSpace() == 7);
}

TEST_CASE("data stays intact over many wrap-arounds") {
	RingBufferWrapper w(8);
	std::string expected;
	for (int i = 0; i < 50; i++) {
		std::string chunk;
		for (int j = 0; j < 5; j++) {
			chunk.push_back((char)('A' + (i * 5 + j) % 26));
		}
		REQUIRE(append(w, chunk) == 5);
		REQUIRE(getAll(w) == chunk);
	}
	CHECK(w.empty());
}

TEST_CASE("typed access reads single bytes and counts elements") {
	// Note: peek_as/get_as only compile for byte types: they use static_cast<uint8_t*>(&data)
	RingBufferWrapper w(8);
	bool ok = true;
	uint8_t v = w.get_as<uint8_t>(&ok);
	CHECK_FALSE(ok); // Empty
	(void)v;

	append(w, "\x11\x22\x33\x44\x55");
	CHECK(w.numElements<uint8_t>() == 5);
	CHECK(w.numElements<uint16_t>() == 2);
	CHECK(w.numElements<uint32_t>() == 1);

	CHECK(w.peek_as<uint8_t>(&ok) == 0x11);
	CHECK(ok);
	CHECK(w.length() == 5);
	CHECK(w.get_as<uint8_t>(&ok) == 0x11);
	CHECK(ok);
	CHECK(w.get_as<uint8_t>(&ok) == 0x22);
	CHECK(w.length() == 3);
}

} // TEST_SUITE
