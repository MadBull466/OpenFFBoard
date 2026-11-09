#ifndef CODEX_NUMERIC_ASSERTIONS_H
#define CODEX_NUMERIC_ASSERTIONS_H

#include <cmath>

namespace CodexNumeric {
// Finite-domain scalar oracle. Check every sample before any reduction: a NaN
// must never disappear through std::max or an unordered floating comparison.
inline bool finiteWithin(double actual, double reference, double tolerance) {
	if (!std::isfinite(actual) || !std::isfinite(reference) ||
		!std::isfinite(tolerance) || tolerance < 0.0) {
		return false;
	}
	const double error = std::fabs(actual - reference);
	return std::isfinite(error) && error <= tolerance;
}
}

#endif
