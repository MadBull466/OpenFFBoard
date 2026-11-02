/*
 * TinyUSB's ring buffer (FFBoard/USB/common/tusb_fifo.c, vendor tier B: tested, never fixed), the FIFO behind the CDC
 * receive / transmit buffers (cdc_device.c: tu_fifo_config, tu_fifo_write_n, tu_fifo_read_n, tu_fifo_peek,
 * tu_fifo_set_overwritable, tu_fifo_clear, the write / read mutexes) and the DWC2 packet FIFO copies
 * (TUP_MEM_CONST_ADDR: tu_fifo_*_const_addr_full_words). Direct tests of the library API; the reference is the
 * documentation of tusb_fifo.h (index scheme, overflow handling) and the comments of tusb_fifo.c.
 *
 * The mutex tests use UsbFixture only for its reset of the fake FreeRTOS and its leak / kernel fault checks (no USB
 * device is started). A FreeRTOS mutex that the test thread already holds cannot be taken again: the fake records
 * "would block forever" (HostRtosExt.h; in ffb_usb_tests also as an unmodelled call) and the take fails; that record
 * shows which mutex a FIFO call takes. The test removes the records it expects.
 */
#include "doctest.h"
#include "UsbFixture.h"

#include "common/tusb_fifo.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

/** A FIFO of `depth` items of `itemSize` bytes with its own buffer */
struct Fifo {
	std::vector<uint8_t> storage;
	tu_fifo_t f;
	Fifo(uint16_t depth, uint16_t itemSize = 1, bool overwritable = false) : storage((size_t)depth * itemSize, 0xEE) {
		std::memset(&f, 0, sizeof(f)); // mutexes NULL: no locking (like a FIFO without tu_fifo_config_mutex)
		REQUIRE(tu_fifo_config(&f, storage.data(), depth, itemSize, overwritable));
	}
	/** Writes the bytes 1..n (or first..first+n-1) one call */
	uint16_t writeSeq(uint8_t first, uint16_t n) {
		std::vector<uint8_t> data(n);
		for (uint16_t i = 0; i < n; i++) {
			data[i] = (uint8_t)(first + i);
		}
		return tu_fifo_write_n(&f, data.data(), n);
	}
	std::vector<uint8_t> readAll() {
		std::vector<uint8_t> out(tu_fifo_depth(&f) * 2u + 4u, 0);
		const uint16_t n = tu_fifo_read_n(&f, out.data(), (uint16_t)out.size());
		out.resize(n);
		return out;
	}
};

std::vector<uint8_t> seq(uint8_t first, size_t n) {
	std::vector<uint8_t> v(n);
	for (size_t i = 0; i < n; i++) {
		v[i] = (uint8_t)(first + i);
	}
	return v;
}

struct Item3 { // an item size that is neither a power of two nor a multiple of 4
	uint8_t b[3];
	bool operator==(const Item3 &o) const { return std::memcmp(b, o.b, 3) == 0; }
};

} // namespace

TEST_SUITE("TusbFifo") {

// ---------------- Configuration and state ----------------

TEST_CASE("a newly configured FIFO is empty with its whole depth remaining") {
	Fifo ff(8);
	CHECK(tu_fifo_depth(&ff.f) == 8);
	CHECK(tu_fifo_empty(&ff.f));
	CHECK_FALSE(tu_fifo_full(&ff.f));
	CHECK(tu_fifo_count(&ff.f) == 0);
	CHECK(tu_fifo_remaining(&ff.f) == 8);
	CHECK_FALSE(tu_fifo_overflowed(&ff.f));
}

TEST_CASE("tu_fifo_config rejects a depth above 0x8000 and leaves the FIFO unchanged") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 3) == 3);
	uint8_t big[4];
	CHECK_FALSE(tu_fifo_config(&ff.f, big, 0x8001, 1, false)); // tusb_fifo.c:75: index space is limited to 2 * depth
	CHECK(tu_fifo_depth(&ff.f) == 8);
	CHECK(tu_fifo_count(&ff.f) == 3);
}

TEST_CASE("tu_fifo_config of a used FIFO empties it") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 5) == 5);
	REQUIRE(tu_fifo_config(&ff.f, ff.storage.data(), 8, 1, false));
	CHECK(tu_fifo_empty(&ff.f));
}

TEST_CASE("tu_fifo_clear empties the FIFO") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 8) == 8);
	REQUIRE(tu_fifo_full(&ff.f));
	CHECK(tu_fifo_clear(&ff.f));
	CHECK(tu_fifo_empty(&ff.f));
	CHECK(tu_fifo_remaining(&ff.f) == 8);
	CHECK(ff.writeSeq(20, 2) == 2);
	CHECK(ff.readAll() == seq(20, 2));
}

// ---------------- Write / read / peek ----------------

TEST_CASE("single items are read in the order they were written") {
	Fifo ff(4);
	for (uint8_t v = 1; v <= 3; v++) {
		REQUIRE(tu_fifo_write(&ff.f, &v));
	}
	CHECK(tu_fifo_count(&ff.f) == 3);
	uint8_t r = 0;
	for (uint8_t v = 1; v <= 3; v++) {
		REQUIRE(tu_fifo_read(&ff.f, &r));
		CHECK(r == v);
	}
	CHECK(tu_fifo_empty(&ff.f));
}

TEST_CASE("reading an empty FIFO returns nothing and leaves the buffer untouched") {
	Fifo ff(4);
	uint8_t r = 0x5A;
	CHECK_FALSE(tu_fifo_read(&ff.f, &r));
	CHECK(r == 0x5A);
	CHECK_FALSE(tu_fifo_peek(&ff.f, &r));
	uint8_t buf[4] = {9, 9, 9, 9};
	CHECK(tu_fifo_read_n(&ff.f, buf, 4) == 0);
	CHECK(tu_fifo_peek_n(&ff.f, buf, 4) == 0);
	CHECK(buf[0] == 9);
	CHECK(tu_fifo_empty(&ff.f));
}

TEST_CASE("a full FIFO that is not overwritable refuses another item and keeps its content") {
	Fifo ff(4);
	REQUIRE(ff.writeSeq(1, 4) == 4);
	CHECK(tu_fifo_full(&ff.f));
	CHECK(tu_fifo_remaining(&ff.f) == 0);
	const uint8_t extra = 99;
	CHECK_FALSE(tu_fifo_write(&ff.f, &extra));
	CHECK(ff.writeSeq(50, 3) == 0);
	CHECK(ff.readAll() == seq(1, 4));
}

TEST_CASE("tu_fifo_write_n into a FIFO that is not overwritable writes only what fits") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 5) == 5);
	CHECK(ff.writeSeq(6, 10) == 3);
	CHECK(tu_fifo_full(&ff.f));
	CHECK(ff.readAll() == seq(1, 8));
}

TEST_CASE("tu_fifo_write_n of zero items writes nothing") {
	Fifo ff(4);
	uint8_t data = 1;
	CHECK(tu_fifo_write_n(&ff.f, &data, 0) == 0);
	CHECK(tu_fifo_empty(&ff.f));
}

TEST_CASE("peek returns the oldest item without removing it") {
	Fifo ff(4);
	REQUIRE(ff.writeSeq(7, 3) == 3);
	uint8_t p = 0;
	REQUIRE(tu_fifo_peek(&ff.f, &p));
	CHECK(p == 7);
	REQUIRE(tu_fifo_peek(&ff.f, &p));
	CHECK(p == 7);
	CHECK(tu_fifo_count(&ff.f) == 3);
}

TEST_CASE("peek_n returns at most the stored items without removing them") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 3) == 3);
	std::array<uint8_t, 6> buf{};
	CHECK(tu_fifo_peek_n(&ff.f, buf.data(), 6) == 3);
	CHECK(std::vector<uint8_t>(buf.begin(), buf.begin() + 3) == seq(1, 3));
	CHECK(tu_fifo_count(&ff.f) == 3);
	CHECK(tu_fifo_peek_n(&ff.f, buf.data(), 2) == 2);
}

TEST_CASE("read_n of more items than stored returns only the stored items") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 5) == 5);
	std::array<uint8_t, 8> buf{};
	CHECK(tu_fifo_read_n(&ff.f, buf.data(), 3) == 3);
	CHECK(std::vector<uint8_t>(buf.begin(), buf.begin() + 3) == seq(1, 3));
	CHECK(tu_fifo_read_n(&ff.f, buf.data(), 8) == 2);
	CHECK(std::vector<uint8_t>(buf.begin(), buf.begin() + 2) == seq(4, 2));
	CHECK(tu_fifo_empty(&ff.f));
}

TEST_CASE("write_n and read_n wrap around the end of the buffer") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 6) == 6);
	REQUIRE(ff.readAll().size() == 6);
	// Write index at buffer position 6: 6 more items wrap (2 linear, 4 at the start)
	REQUIRE(ff.writeSeq(10, 6) == 6);
	CHECK(ff.storage[6] == 10);
	CHECK(ff.storage[7] == 11);
	CHECK(ff.storage[0] == 12);
	CHECK(ff.storage[3] == 15);
	CHECK(tu_fifo_count(&ff.f) == 6);
	CHECK(ff.readAll() == seq(10, 6));
}

TEST_CASE("peek_n of a wrapped FIFO returns the items in order") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 7) == 7);
	std::array<uint8_t, 8> buf{};
	REQUIRE(tu_fifo_read_n(&ff.f, buf.data(), 7) == 7);
	REQUIRE(ff.writeSeq(30, 5) == 5); // positions 7, 0..3
	CHECK(tu_fifo_peek_n(&ff.f, buf.data(), 8) == 5);
	CHECK(std::vector<uint8_t>(buf.begin(), buf.begin() + 5) == seq(30, 5));
}

TEST_CASE("a FIFO with a depth that is not a power of two keeps its order over many wrap-arounds") {
	Fifo ff(5);
	uint8_t next = 0;
	uint8_t expected = 0;
	for (int round = 0; round < 200; round++) {
		const uint16_t n = (uint16_t)(1 + round % 5);
		std::vector<uint8_t> data(n);
		for (auto &d : data) {
			d = next++;
		}
		REQUIRE(tu_fifo_write_n(&ff.f, data.data(), n) == n);
		REQUIRE(tu_fifo_count(&ff.f) == n);
		std::vector<uint8_t> out(n);
		REQUIRE(tu_fifo_read_n(&ff.f, out.data(), n) == n);
		for (uint8_t v : out) {
			REQUIRE(v == expected++);
		}
	}
	CHECK(tu_fifo_empty(&ff.f));
}

TEST_CASE("a FIFO of the largest depth counts correctly across the end of the 16-bit index space") {
	// depth 0x8000: the index space 0..2*depth-1 is the whole uint16_t range (advance_index, tusb_fifo.c:356-369)
	Fifo ff(0x8000);
	tu_fifo_advance_write_pointer(&ff.f, 0x7FFF);
	tu_fifo_advance_read_pointer(&ff.f, 0x7FFF);
	tu_fifo_advance_write_pointer(&ff.f, 0x7FFF);
	tu_fifo_advance_read_pointer(&ff.f, 0x7FFF); // both indices at 0xFFFE
	REQUIRE(tu_fifo_empty(&ff.f));
	REQUIRE(ff.writeSeq(1, 4) == 4); // write index wraps from 0xFFFE to 2
	CHECK(tu_fifo_count(&ff.f) == 4);
	CHECK(ff.storage[0x7FFE] == 1);
	CHECK(ff.storage[0x7FFF] == 2);
	CHECK(ff.storage[0] == 3);
	CHECK(ff.readAll() == seq(1, 4));
	CHECK(tu_fifo_empty(&ff.f));
}

// ---------------- Item sizes ----------------

TEST_CASE("items of 4 bytes are stored and read as whole items, also across the wrap-around") {
	Fifo ff(4, sizeof(uint32_t));
	const uint32_t a[3] = {0x11111111, 0x22222222, 0x33333333};
	REQUIRE(tu_fifo_write_n(&ff.f, a, 3) == 3);
	uint32_t r = 0;
	REQUIRE(tu_fifo_read(&ff.f, &r));
	CHECK(r == 0x11111111);
	REQUIRE(tu_fifo_peek(&ff.f, &r));
	CHECK(r == 0x22222222);
	const uint32_t b[3] = {0x44444444, 0x55555555, 0x66666666};
	REQUIRE(tu_fifo_write_n(&ff.f, b, 3) == 2); // 2 free items
	CHECK(tu_fifo_full(&ff.f));
	uint32_t out[4] = {};
	REQUIRE(tu_fifo_read_n(&ff.f, out, 4) == 4);
	CHECK(out[0] == 0x22222222);
	CHECK(out[1] == 0x33333333);
	CHECK(out[2] == 0x44444444);
	CHECK(out[3] == 0x55555555);
}

TEST_CASE("items of 3 bytes keep their bytes together across the wrap-around") {
	Fifo ff(3, sizeof(Item3));
	const Item3 i1{{1, 2, 3}}, i2{{4, 5, 6}}, i3{{7, 8, 9}}, i4{{10, 11, 12}};
	REQUIRE(tu_fifo_write(&ff.f, &i1));
	REQUIRE(tu_fifo_write(&ff.f, &i2));
	Item3 r{};
	REQUIRE(tu_fifo_read(&ff.f, &r));
	CHECK(r == i1);
	const Item3 more[2] = {i3, i4};
	REQUIRE(tu_fifo_write_n(&ff.f, more, 2) == 2); // positions 2 and 0
	Item3 out[3] = {};
	REQUIRE(tu_fifo_read_n(&ff.f, out, 3) == 3);
	CHECK(out[0] == i2);
	CHECK(out[1] == i3);
	CHECK(out[2] == i4);
}

// ---------------- Overwritable mode ----------------

TEST_CASE("an overwritable FIFO accepts an item when full and then holds the newest items") {
	Fifo ff(4, 1, true);
	REQUIRE(ff.writeSeq(1, 4) == 4);
	const uint8_t v = 5;
	CHECK(tu_fifo_write(&ff.f, &v));
	CHECK(tu_fifo_overflowed(&ff.f));
	CHECK(tu_fifo_count(&ff.f) == 4); // count is limited to the depth
	CHECK(ff.readAll() == seq(2, 4));  // the read corrects the read index (tusb_fifo.c:428-432)
	CHECK_FALSE(tu_fifo_overflowed(&ff.f));
}

TEST_CASE("peek and read of single items correct the read index of an overflowed FIFO") {
	Fifo ff(4, 1, true);
	REQUIRE(ff.writeSeq(1, 4) == 4);
	REQUIRE(ff.writeSeq(5, 2) == 2); // overflowed by 2
	uint8_t r = 0;
	REQUIRE(tu_fifo_peek(&ff.f, &r));
	CHECK(r == 3);
	REQUIRE(tu_fifo_read(&ff.f, &r));
	CHECK(r == 3);
	CHECK(tu_fifo_count(&ff.f) == 3);
}

TEST_CASE("the read index is corrected also when the write index has wrapped below the depth") {
	Fifo ff(4, 1, true);
	tu_fifo_advance_write_pointer(&ff.f, 5);
	tu_fifo_advance_read_pointer(&ff.f, 5); // both at index 5
	REQUIRE(ff.writeSeq(1, 3) == 3);		// write index 8 -> 0
	REQUIRE(ff.writeSeq(4, 2) == 2);		// write index 2 (< depth), 5 items for 4 slots
	REQUIRE(tu_fifo_overflowed(&ff.f));
	CHECK(ff.readAll() == seq(2, 4)); // read index corrected to write index + depth (tusb_fifo.c:408-411)
	CHECK(tu_fifo_empty(&ff.f));
}

TEST_CASE("write_n of more items than the depth into an overwritable FIFO keeps the last depth items") {
	Fifo ff(4, 1, true);
	REQUIRE(ff.writeSeq(1, 2) == 2);
	CHECK(ff.writeSeq(10, 7) == 4);
	CHECK(tu_fifo_full(&ff.f));
	CHECK(ff.readAll() == seq(13, 4));
}

TEST_CASE("write_n of exactly depth items into a used overwritable FIFO replaces its content without an overflow") {
	Fifo ff(4, 1, true);
	REQUIRE(ff.writeSeq(1, 2) == 2);
	CHECK(ff.writeSeq(10, 4) == 4); // written from the read position (tusb_fifo.c:495-511)
	CHECK_FALSE(tu_fifo_overflowed(&ff.f));
	CHECK(tu_fifo_full(&ff.f));
	CHECK(ff.readAll() == seq(10, 4));
}

TEST_CASE("write_n into an overflowed overwritable FIFO keeps it full with the newest items") {
	Fifo ff(4, 1, true);
	REQUIRE(ff.writeSeq(1, 4) == 4);
	REQUIRE(ff.writeSeq(5, 2) == 2); // single overflow: 6 items written, 4 kept
	REQUIRE(tu_fifo_overflowed(&ff.f));
	CHECK(ff.writeSeq(7, 3) == 3); // double overflow: write index re-positioned (tusb_fifo.c:515-525)
	CHECK(tu_fifo_full(&ff.f));
	CHECK(tu_fifo_count(&ff.f) == 4);
	// The data of the double overflow is not checked: tusb_fifo.c:522-525 documents that the oldest item is not
	// shifted out ("result in incorrect data read back")
}

TEST_CASE("an overwritable FIFO written item by item with twice its depth holds the newest items (V25)" * doctest::should_fail() *
		  doctest::description("KNOWN ISSUE V25 (TinyUSB, vendor): tu_fifo_write (tusb_fifo.c:819-845) has no double-overflow "
							   "handling (unlike _tu_fifo_write_n :514-525): after 2 * depth single writes the write index "
							   "wraps onto the read index and the full FIFO reads as empty (tusb_fifo.h:72-77: 'This must be "
							   "prevented')")) {
	Fifo ff(4, 1, true);
	for (uint8_t v = 1; v <= 8; v++) {
		REQUIRE(tu_fifo_write(&ff.f, &v));
	}
	CHECK(tu_fifo_count(&ff.f) == 4); // Defect: 0, the write index equals the read index again
	CHECK(ff.readAll() == seq(5, 4));
}

TEST_CASE("tu_fifo_set_overwritable switches a full FIFO between refusing and overwriting") {
	Fifo ff(4);
	REQUIRE(ff.writeSeq(1, 4) == 4);
	const uint8_t v = 9;
	REQUIRE_FALSE(tu_fifo_write(&ff.f, &v));
	CHECK(tu_fifo_set_overwritable(&ff.f, true));
	CHECK(tu_fifo_write(&ff.f, &v));
	CHECK(tu_fifo_set_overwritable(&ff.f, false));
	CHECK(ff.readAll() == std::vector<uint8_t>{2, 3, 4, 9});
	REQUIRE(ff.writeSeq(1, 4) == 4);
	CHECK_FALSE(tu_fifo_write(&ff.f, &v));
}

TEST_CASE("tu_fifo_correct_read_pointer turns an overflowed FIFO into a full one with the newest items") {
	Fifo ff(4, 1, true);
	REQUIRE(ff.writeSeq(1, 6) == 4); // n >= depth: the last 4 are written at the read position
	REQUIRE(ff.writeSeq(7, 1) == 1); // single overflow
	REQUIRE(tu_fifo_overflowed(&ff.f));
	tu_fifo_correct_read_pointer(&ff.f);
	CHECK_FALSE(tu_fifo_overflowed(&ff.f));
	CHECK(tu_fifo_full(&ff.f));
	CHECK(ff.readAll() == seq(4, 4));
}

// ---------------- Linear buffer access (DMA helpers) ----------------

TEST_CASE("the read info of an empty FIFO has no data") {
	Fifo ff(8);
	tu_fifo_buffer_info_t info;
	std::memset(&info, 0x55, sizeof(info));
	tu_fifo_get_read_info(&ff.f, &info);
	CHECK(info.len_lin == 0);
	CHECK(info.len_wrap == 0);
	CHECK(info.ptr_lin == nullptr);
	CHECK(info.ptr_wrap == nullptr);
}

TEST_CASE("the read info of an unwrapped FIFO is one linear part at the read position") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 5) == 5);
	uint8_t tmp[2];
	REQUIRE(tu_fifo_read_n(&ff.f, tmp, 2) == 2);
	tu_fifo_buffer_info_t info;
	tu_fifo_get_read_info(&ff.f, &info);
	CHECK(info.len_lin == 3);
	CHECK(info.len_wrap == 0);
	CHECK(info.ptr_lin == ff.storage.data() + 2);
	CHECK(info.ptr_wrap == nullptr);
}

TEST_CASE("the read info of a wrapped FIFO has a linear part to the end and a part at the start of the buffer") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 6) == 6);
	REQUIRE(ff.readAll().size() == 6);
	REQUIRE(ff.writeSeq(10, 5) == 5); // positions 6, 7, 0, 1, 2
	tu_fifo_buffer_info_t info;
	tu_fifo_get_read_info(&ff.f, &info);
	CHECK(info.len_lin == 2);
	CHECK(info.len_wrap == 3);
	CHECK(info.ptr_lin == ff.storage.data() + 6);
	CHECK(info.ptr_wrap == ff.storage.data());
	// Consuming through the pointers and tu_fifo_advance_read_pointer
	CHECK(static_cast<uint8_t *>(info.ptr_lin)[0] == 10);
	CHECK(static_cast<uint8_t *>(info.ptr_wrap)[2] == 14);
	tu_fifo_advance_read_pointer(&ff.f, info.len_lin + info.len_wrap);
	CHECK(tu_fifo_empty(&ff.f));
}

TEST_CASE("the read info of a full FIFO covers all items") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 3) == 3);
	REQUIRE(ff.readAll().size() == 3);
	REQUIRE(ff.writeSeq(1, 8) == 8); // write position == read position (3)
	tu_fifo_buffer_info_t info;
	tu_fifo_get_read_info(&ff.f, &info);
	CHECK(info.len_lin == 5);
	CHECK(info.len_wrap == 3);
	CHECK(info.ptr_lin == ff.storage.data() + 3);
}

TEST_CASE("the read info of an overflowed FIFO corrects the read index to the newest items") {
	Fifo ff(4);
	tu_fifo_advance_write_pointer(&ff.f, 6); // e.g. a DMA that wrote 6 items into a FIFO of 4
	REQUIRE(tu_fifo_overflowed(&ff.f));
	tu_fifo_buffer_info_t info;
	tu_fifo_get_read_info(&ff.f, &info);
	CHECK_FALSE(tu_fifo_overflowed(&ff.f));
	CHECK(info.len_lin + info.len_wrap == 4);
	CHECK(info.ptr_lin == ff.storage.data() + 2);
}

TEST_CASE("the write info of a full FIFO has no space") {
	Fifo ff(4);
	REQUIRE(ff.writeSeq(1, 4) == 4);
	tu_fifo_buffer_info_t info;
	std::memset(&info, 0x55, sizeof(info));
	tu_fifo_get_write_info(&ff.f, &info);
	CHECK(info.len_lin == 0);
	CHECK(info.len_wrap == 0);
	CHECK(info.ptr_lin == nullptr);
	CHECK(info.ptr_wrap == nullptr);
}

TEST_CASE("the write info of an empty FIFO is the space to the end of the buffer and the space before the write position") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 3) == 3);
	REQUIRE(ff.readAll().size() == 3);
	tu_fifo_buffer_info_t info;
	tu_fifo_get_write_info(&ff.f, &info);
	CHECK(info.len_lin == 5);
	CHECK(info.len_wrap == 3);
	CHECK(info.ptr_lin == ff.storage.data() + 3);
	CHECK(info.ptr_wrap == ff.storage.data());
	// Filling through the pointer and tu_fifo_advance_write_pointer
	static_cast<uint8_t *>(info.ptr_lin)[0] = 42;
	tu_fifo_advance_write_pointer(&ff.f, 1);
	uint8_t r = 0;
	REQUIRE(tu_fifo_read(&ff.f, &r));
	CHECK(r == 42);
}

TEST_CASE("the write info of a wrapped FIFO is the linear space up to the read position") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 6) == 6);
	uint8_t tmp[4];
	REQUIRE(tu_fifo_read_n(&ff.f, tmp, 4) == 4); // read position 4
	REQUIRE(ff.writeSeq(7, 3) == 3);			 // write position 1 (wrapped)
	tu_fifo_buffer_info_t info;
	tu_fifo_get_write_info(&ff.f, &info);
	CHECK(info.len_lin == 3);
	CHECK(info.len_wrap == 0);
	CHECK(info.ptr_lin == ff.storage.data() + 1);
	CHECK(info.ptr_wrap == nullptr);
}

TEST_CASE("the read and write info of a FIFO with items of 4 bytes point at the item positions (V26)" * doctest::should_fail() *
		  doctest::description("KNOWN ISSUE V26 (TinyUSB, vendor): tu_fifo_get_read_info / tu_fifo_get_write_info "
							   "(tusb_fifo.c:1022, :1077) index the byte buffer with the item position (&f->buffer[rd_ptr]) "
							   "instead of rd_ptr * item_size; the lengths are in items (tusb_fifo.h:131-132), so for items "
							   "larger than one byte the pointers point into the wrong item")) {
	Fifo ff(4, sizeof(uint32_t));
	const uint32_t a[3] = {1, 2, 3};
	REQUIRE(tu_fifo_write_n(&ff.f, a, 3) == 3);
	uint32_t r = 0;
	REQUIRE(tu_fifo_read(&ff.f, &r)); // read position 1 = byte offset 4
	tu_fifo_buffer_info_t info;
	tu_fifo_get_read_info(&ff.f, &info);
	REQUIRE(info.len_lin == 2);
	CHECK(info.ptr_lin == ff.storage.data() + 4); // Defect: storage + 1
	tu_fifo_get_write_info(&ff.f, &info);
	CHECK(info.ptr_lin == ff.storage.data() + 12);
}

// ---------------- Copies from / to a constant address (DWC2 packet FIFO, TUP_MEM_CONST_ADDR) ----------------

TEST_CASE("write_n from a constant address stores the word register bytes in order") {
	// dcd_dwc2 reads the OTG_FS receive FIFO register word by word: each read gives the next 4 bytes of the packet.
	// Here the register holds the same word for every read, so the stored bytes repeat with period 4.
	Fifo ff(16);
	volatile uint32_t reg = 0x44332211;
	CHECK(tu_fifo_write_n_const_addr_full_words(&ff.f, (const void *)&reg, 6) == 6);
	CHECK(ff.readAll() == std::vector<uint8_t>{0x11, 0x22, 0x33, 0x44, 0x11, 0x22});
}

TEST_CASE("write_n from a constant address splits a word at the wrap-around of the buffer") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 6) == 6);
	REQUIRE(ff.readAll().size() == 6); // write position 6: 2 bytes before the end
	volatile uint32_t reg = 0x44332211;
	CHECK(tu_fifo_write_n_const_addr_full_words(&ff.f, (const void *)&reg, 7) == 7);
	CHECK(ff.storage[6] == 0x11);
	CHECK(ff.storage[7] == 0x22);
	CHECK(ff.storage[0] == 0x33); // the rest of the same word continues at the start of the buffer
	CHECK(ff.readAll() == std::vector<uint8_t>{0x11, 0x22, 0x33, 0x44, 0x11, 0x22, 0x33});
}

TEST_CASE("write_n from a constant address wraps at a word boundary without splitting") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 4) == 4);
	REQUIRE(ff.readAll().size() == 4); // write position 4: one full word before the end
	volatile uint32_t reg = 0xDDCCBBAA;
	CHECK(tu_fifo_write_n_const_addr_full_words(&ff.f, (const void *)&reg, 6) == 6);
	CHECK(ff.readAll() == std::vector<uint8_t>{0xAA, 0xBB, 0xCC, 0xDD, 0xAA, 0xBB});
}

TEST_CASE("read_n to a constant address writes the bytes as little-endian words with a zero-padded last word") {
	Fifo ff(16);
	REQUIRE(ff.writeSeq(1, 6) == 6);
	volatile uint32_t reg = 0;
	// Every word write overwrites the register: the last write is the padded rest (bytes 5, 6)
	CHECK(tu_fifo_read_n_const_addr_full_words(&ff.f, (void *)&reg, 6) == 6);
	CHECK(reg == 0x00000605u);
	CHECK(tu_fifo_empty(&ff.f));
}

TEST_CASE("read_n to a constant address joins the bytes of a word that wraps around the buffer") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 6) == 6);
	REQUIRE(ff.readAll().size() == 6);
	REQUIRE(ff.writeSeq(0x10, 4) == 4); // positions 6, 7, 0, 1
	volatile uint32_t reg = 0;
	CHECK(tu_fifo_read_n_const_addr_full_words(&ff.f, (void *)&reg, 4) == 4);
	CHECK(reg == 0x13121110u); // one word of 2 bytes from the end and 2 from the start of the buffer
}

TEST_CASE("read_n to a constant address wraps at a word boundary without joining") {
	Fifo ff(8);
	REQUIRE(ff.writeSeq(1, 4) == 4);
	REQUIRE(ff.readAll().size() == 4);
	REQUIRE(ff.writeSeq(0x20, 6) == 6); // positions 4..7, 0, 1
	volatile uint32_t reg = 0;
	CHECK(tu_fifo_read_n_const_addr_full_words(&ff.f, (void *)&reg, 6) == 6);
	CHECK(reg == 0x00002524u); // last write: the 2 wrapped bytes, zero padded
}

// ---------------- Mutexes (tu_fifo_config_mutex, CFG_FIFO_MUTEX with FreeRTOS) ----------------

namespace {
/** A FIFO with a write and a read mutex (as tu_edpt_stream_init / cdcd_init configure them) */
struct MutexFifo {
	Fifo ff{8};
	osal_mutex_def_t wrDef, rdDef;
	osal_mutex_t wr = osal_mutex_create(&wrDef);
	osal_mutex_t rd = osal_mutex_create(&rdDef);
	MutexFifo() {
		REQUIRE(wr != nullptr);
		REQUIRE(rd != nullptr);
		tu_fifo_config_mutex(&ff.f, wr, rd);
	}
	~MutexFifo() {
		osal_mutex_delete(wr);
		osal_mutex_delete(rd);
	}
	/** True if both mutexes are free (each can be taken without waiting) */
	bool bothFree() {
		bool ok = true;
		for (osal_mutex_t m : {wr, rd}) {
			if (osal_mutex_lock(m, 0)) {
				osal_mutex_unlock(m);
			} else {
				ok = false;
			}
		}
		return ok;
	}
	/**
	 * Runs `op` while the test thread holds `m`. @return true if `op` tried to take `m` (the fake records the
	 * forever-wait of the test thread). The FIFO's unlock afterwards gives the mutex back.
	 */
	template <typename Op> bool takes(osal_mutex_t m, Op op) {
		REQUIRE(osal_mutex_lock(m, 0));
		op();
		const bool took = HostRtos::takeKernelFault("would block forever");
		// ffb_usb_tests also reports kernel faults as unmodelled calls (host_target.cpp): remove the expected one
		auto &unmodelled = HostTarget::state().unmodelledHalCalls;
		if (took && !unmodelled.empty() && unmodelled.back().find("would block forever") != std::string::npos) {
			unmodelled.pop_back();
		}
		if (!took) {
			osal_mutex_unlock(m);
		}
		return took;
	}
};
} // namespace

TEST_CASE_FIXTURE(UsbFixture, "a write takes the write mutex of the FIFO only and releases it") {
	MutexFifo m;
	const uint8_t v = 3;
	CHECK(m.ff.writeSeq(1, 2) == 2);
	CHECK(tu_fifo_write(&m.ff.f, &v));
	CHECK(HostRtos::kernelFaults().empty());
	CHECK(m.bothFree());
	CHECK(m.takes(m.wr, [&] { m.ff.writeSeq(4, 1); }));
	CHECK(m.takes(m.wr, [&] { tu_fifo_write(&m.ff.f, &v); }));
	CHECK_FALSE(m.takes(m.rd, [&] { m.ff.writeSeq(4, 1); }));
	CHECK_FALSE(m.takes(m.rd, [&] { tu_fifo_write(&m.ff.f, &v); }));
	CHECK(m.bothFree());
}

TEST_CASE_FIXTURE(UsbFixture, "read, read_n, peek, peek_n and the read pointer correction take the read mutex of the FIFO only") {
	MutexFifo m;
	REQUIRE(m.ff.writeSeq(1, 8) == 8);
	uint8_t buf[2];
	CHECK(tu_fifo_peek(&m.ff.f, buf));
	CHECK(tu_fifo_peek_n(&m.ff.f, buf, 2) == 2);
	CHECK(tu_fifo_read(&m.ff.f, buf));
	CHECK(tu_fifo_read_n(&m.ff.f, buf, 2) == 2);
	CHECK(HostRtos::kernelFaults().empty());
	CHECK(m.bothFree());
	CHECK(m.takes(m.rd, [&] { tu_fifo_peek(&m.ff.f, buf); }));
	CHECK(m.takes(m.rd, [&] { tu_fifo_peek_n(&m.ff.f, buf, 1); }));
	CHECK(m.takes(m.rd, [&] { tu_fifo_read(&m.ff.f, buf); }));
	CHECK(m.takes(m.rd, [&] { tu_fifo_read_n(&m.ff.f, buf, 1); }));
	CHECK(m.takes(m.rd, [&] { tu_fifo_correct_read_pointer(&m.ff.f); }));
	CHECK_FALSE(m.takes(m.wr, [&] { tu_fifo_read(&m.ff.f, buf); }));
	CHECK_FALSE(m.takes(m.wr, [&] { tu_fifo_peek_n(&m.ff.f, buf, 1); }));
	CHECK(m.bothFree());
}

TEST_CASE_FIXTURE(UsbFixture, "clear, set_overwritable and config take both mutexes of the FIFO and release them") {
	MutexFifo m;
	CHECK(tu_fifo_clear(&m.ff.f));
	CHECK(tu_fifo_set_overwritable(&m.ff.f, true));
	CHECK(tu_fifo_config(&m.ff.f, m.ff.storage.data(), 8, 1, false));
	CHECK(HostRtos::kernelFaults().empty());
	CHECK(m.bothFree());
	for (osal_mutex_t mutex : {m.wr, m.rd}) {
		CHECK(m.takes(mutex, [&] { tu_fifo_clear(&m.ff.f); }));
		CHECK(m.takes(mutex, [&] { tu_fifo_set_overwritable(&m.ff.f, false); }));
		CHECK(m.takes(mutex, [&] { tu_fifo_config(&m.ff.f, m.ff.storage.data(), 8, 1, false); }));
	}
	CHECK(m.bothFree());
}

TEST_CASE("a FIFO without mutexes works without the RTOS") {
	Fifo ff(4); // tu_fifo_config_mutex not called: _ff_lock skips NULL mutexes (tusb_fifo.c:41-49)
	REQUIRE(ff.writeSeq(1, 2) == 2);
	CHECK(ff.readAll() == seq(1, 2));
}

} // TEST_SUITE
