#include "doctest.h"
#include "../support/NumericAssertions.h"
#include <initializer_list>
#include <limits>

// Pure predicate: no mutable global state or hardware fixture is needed.
TEST_SUITE("CodexNumeric") {
TEST_CASE("finite comparison accepts equal values and the inclusive tolerance boundary") {
	CHECK(CodexNumeric::finiteWithin(-2.0, -2.0, 0.0));
	CHECK(CodexNumeric::finiteWithin(-1.5, -2.0, 0.5));
	CHECK_FALSE(CodexNumeric::finiteWithin(-1.5, -2.0, 0.25));
}
TEST_CASE("finite comparison rejects nonfinite operands and invalid tolerances") {
	const double nan = std::numeric_limits<double>::quiet_NaN();
	const double inf = std::numeric_limits<double>::infinity();
	for (double invalid : {nan, inf, -inf}) {
		CHECK_FALSE(CodexNumeric::finiteWithin(invalid, 1.0, 0.5));
		CHECK_FALSE(CodexNumeric::finiteWithin(1.0, invalid, 0.5));
		CHECK_FALSE(CodexNumeric::finiteWithin(1.0, 1.0, invalid));
	}
	CHECK_FALSE(CodexNumeric::finiteWithin(1.0, 1.0, -0.5));
}
TEST_CASE("finite comparison rejects overflow of the error between finite operands") {
	const double largest = std::numeric_limits<double>::max();
	CHECK_FALSE(CodexNumeric::finiteWithin(largest, -largest, largest));
	CHECK(CodexNumeric::finiteWithin(largest, largest, 0.0));
}
}
