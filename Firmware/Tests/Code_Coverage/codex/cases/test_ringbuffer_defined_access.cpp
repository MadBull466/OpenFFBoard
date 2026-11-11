// CDX-008 replacement: successful typed access and counts, without empty typed-return UB.
#include "doctest.h"
#include "ringbufferwrapper.h"
#include <array>

namespace {
struct DefinedRingFixture {
	RingBufferWrapper ring{8};
};
}
TEST_SUITE("CodexDefinedRingAccess") {
TEST_CASE_FIXTURE(DefinedRingFixture, "populated typed byte access preserves order and element counts") {
	const std::array<uint8_t, 5> bytes{0x11, 0x22, 0x33, 0x44, 0x55};
	REQUIRE(ring.isValid());
	REQUIRE(ring.appendMultiple(bytes.data(), bytes.size()) == bytes.size());
	CHECK(ring.numElements<uint8_t>() == 5);
	CHECK(ring.numElements<uint16_t>() == 2);
	CHECK(ring.numElements<uint32_t>() == 1);
	bool ok = false;
	CHECK(ring.peek_as<uint8_t>(&ok) == 0x11);
	REQUIRE(ok);
	CHECK(ring.length() == 5);
	for (const auto expected : bytes) {
		REQUIRE(ring.length() > 0);
		CHECK(ring.get_as<uint8_t>(&ok) == expected);
		REQUIRE(ok);
	}
	CHECK(ring.empty());
}
TEST_CASE_FIXTURE(DefinedRingFixture, "empty untyped access reports failure without changing its output") {
	uint8_t output = 0xA5;
	CHECK(ring.getOne(&output) == 0);
	CHECK(output == 0xA5);
	CHECK(ring.empty());
}
}
