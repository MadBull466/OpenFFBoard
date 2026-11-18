/*
 * FrequencyResponse.cpp: see FrequencyResponse.h.
 */
#include "FrequencyResponse.h"

#include <cmath>

namespace {
const double kPi = 3.14159265358979323846;
}

std::complex<double> FrequencyResponse::bin(const std::vector<double> &x, double sampleRateHz, double frequencyHz) {
	std::complex<double> acc(0.0, 0.0);
	const double w = 2.0 * kPi * frequencyHz / sampleRateHz;
	for (size_t n = 0; n < x.size(); n++) {
		const double a = w * static_cast<double>(n);
		acc += x[n] * std::complex<double>(std::cos(a), -std::sin(a));
	}
	return acc;
}

FrequencyResponse::Result FrequencyResponse::measure(const std::vector<double> &input, const std::vector<double> &output, const Options &opt) {
	Result r;
	auto fail = [&r](const char *text) {
		r.valid = false;
		r.error = text;
		return r;
	};
	if (input.empty() || input.size() != output.size()) {
		return fail("FrequencyResponse: input and output must be non-empty and of equal length");
	}
	if (!(opt.sampleRateHz > 0.0) || !(opt.frequencyHz > 0.0) || opt.minPeriods == 0) {
		return fail("FrequencyResponse: sample rate and frequency must be > 0 and minPeriods >= 1");
	}
	if (opt.frequencyHz >= 0.5 * opt.sampleRateHz) {
		return fail("FrequencyResponse: frequency at or above the Nyquist frequency");
	}
	double sumAbs = 0.0;
	for (size_t i = 0; i < input.size(); i++) {
		if (!std::isfinite(input[i]) || !std::isfinite(output[i])) {
			return fail("FrequencyResponse: NaN or infinite sample");
		}
		sumAbs += std::fabs(input[i]);
	}
	r.periods = static_cast<double>(input.size()) * opt.frequencyHz / opt.sampleRateHz;
	const double whole = std::round(r.periods);
	if (std::fabs(r.periods - whole) > 1e-9 * (whole > 1.0 ? whole : 1.0) || whole < 1.0) {
		return fail("FrequencyResponse: the window is not a whole number of periods");
	}
	if (whole < static_cast<double>(opt.minPeriods)) {
		return fail("FrequencyResponse: the window holds fewer periods than minPeriods");
	}
	if (opt.sampleRateHz / opt.frequencyHz < 4.0) {
		return fail("FrequencyResponse: fewer than 4 samples per period");
	}
	r.inputBin = bin(input, opt.sampleRateHz, opt.frequencyHz);
	r.outputBin = bin(output, opt.sampleRateHz, opt.frequencyHz);
	if (std::abs(r.inputBin) <= 1e-12 * sumAbs || std::abs(r.inputBin) == 0.0) {
		return fail("FrequencyResponse: the input has no energy at the test frequency");
	}
	const std::complex<double> h = r.outputBin / r.inputBin;
	r.gain = std::abs(h);
	r.phaseRad = std::arg(h);
	r.phaseDeg = r.phaseRad * 180.0 / kPi;
	r.valid = true;
	return r;
}
