/*
 * Biquad filter (Filters.cpp, Filters.h)
 *
 * The coefficients are compared with references computed here in double precision:
 *  - lowpass, highpass, bandpass and notch with the formulas of the RBJ "Audio EQ cookbook"
 *    (w0 = 2*pi*Fc, alpha = sin(w0)/(2Q)). The bilinear transform form used by Filters.cpp
 *    (K = tan(pi*Fc), earlevel.com) is algebraically identical for these four types.
 *  - peak, lowshelf and highshelf with the formulas of Filters.cpp itself (earlevel.com / Zoelzer).
 *    They deviate from the cookbook (see the characterization tests), so only the physical
 *    properties (gain at DC, fc and Nyquist) are checked against independent expectations.
 *
 * Filters.cpp has two implementations: CMSIS-DSP (USE_DSP_FUNCTIONS, table based sin/cos,
 * arm_biquad_cascade_df1_f32) and a fallback (tan(), transposed direct form II). This file is
 * compiled in both builds and checks both against the same double precision references, so the
 * two builds agree with each other within twice the tolerances used here.
 *
 * Host vs target: the host uses the portable C version of CMSIS-DSP and x86-64 SSE floats without
 * fused multiply-add. The target compiles with -ffp-contract=fast (gnu++20) and may fuse operations,
 * so results differ in the last bits. All comparisons use tolerances. The Nyquist case below
 * depends on an exact 0 returned by arm_cos_f32 and may behave differently on the target.
 */
#include "doctest.h"
#include "TestHelpers.h"
#include "Filters.h"

#include <cmath>
#include <complex>
#include <vector>

using doctest::Approx;

namespace {

/**
 * Normalised biquad coefficients in the cookbook convention:
 * H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
 */
struct Coeffs {
	double b0, b1, b2, a1, a2;
};

/** Relative tolerance for coefficients (CMSIS-DSP sin/cos tables are accurate to about 2e-5 relative) */
constexpr double COEFF_EPS = 2e-4;

/**
 * Coefficients of a firmware filter converted to the cookbook convention.
 * Filters.cpp stores {b0, b1, b2, -a1, -a2} (CMSIS-DSP order with negated feedback terms).
 */
Coeffs firmwareCoeffs(const Biquad &filter) {
	const float *c = filter.getCoeffs();
	return {c[0], c[1], c[2], -(double)c[3], -(double)c[4]};
}

// ---- RBJ audio EQ cookbook (independent reference) ----
struct Rbj {
	double cw, sw, alpha;
	Rbj(double fc, double q) {
		const double w0 = 2.0 * M_PI * fc;
		cw = std::cos(w0);
		sw = std::sin(w0);
		alpha = sw / (2.0 * q);
	}
	Coeffs norm(double b0, double b1, double b2, double a0, double a1, double a2) const {
		return {b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0};
	}
};

Coeffs rbjLowpass(double fc, double q) {
	Rbj r(fc, q);
	return r.norm((1 - r.cw) / 2, 1 - r.cw, (1 - r.cw) / 2, 1 + r.alpha, -2 * r.cw, 1 - r.alpha);
}
Coeffs rbjHighpass(double fc, double q) {
	Rbj r(fc, q);
	return r.norm((1 + r.cw) / 2, -(1 + r.cw), (1 + r.cw) / 2, 1 + r.alpha, -2 * r.cw, 1 - r.alpha);
}
/** Band pass with constant 0 dB peak gain */
Coeffs rbjBandpass(double fc, double q) {
	Rbj r(fc, q);
	return r.norm(r.alpha, 0, -r.alpha, 1 + r.alpha, -2 * r.cw, 1 - r.alpha);
}
Coeffs rbjNotch(double fc, double q) {
	Rbj r(fc, q);
	return r.norm(1, -2 * r.cw, 1, 1 + r.alpha, -2 * r.cw, 1 - r.alpha);
}
/** Cookbook peaking EQ (A = 10^(dB/40)). Only used to show that Filters.cpp deviates from it */
Coeffs rbjPeak(double fc, double q, double gainDb) {
	Rbj r(fc, q);
	const double A = std::pow(10.0, gainDb / 40.0);
	return r.norm(1 + r.alpha * A, -2 * r.cw, 1 - r.alpha * A, 1 + r.alpha / A, -2 * r.cw, 1 - r.alpha / A);
}

// ---- Formulas of Filters.cpp (earlevel.com / Zoelzer), transcribed in double precision ----
Coeffs zoelzer(BiquadType type, double fc, double q, double gainDb) {
	const double V = std::pow(10.0, std::fabs(gainDb) / 20.0);
	const double K = std::tan(M_PI * fc);
	const double s2 = std::sqrt(2.0), s2V = std::sqrt(2.0 * V);
	double n;
	const bool boost = gainDb >= 0;
	switch (type) {
	case BiquadType::peak:
		if (boost) {
			n = 1 / (1 + K / q + K * K);
			return {(1 + V / q * K + K * K) * n, 2 * (K * K - 1) * n, (1 - V / q * K + K * K) * n, 2 * (K * K - 1) * n, (1 - K / q + K * K) * n};
		}
		n = 1 / (1 + V / q * K + K * K);
		return {(1 + K / q + K * K) * n, 2 * (K * K - 1) * n, (1 - K / q + K * K) * n, 2 * (K * K - 1) * n, (1 - V / q * K + K * K) * n};
	case BiquadType::lowshelf:
		if (boost) {
			n = 1 / (1 + s2 * K + K * K);
			return {(1 + s2V * K + V * K * K) * n, 2 * (V * K * K - 1) * n, (1 - s2V * K + V * K * K) * n, 2 * (K * K - 1) * n, (1 - s2 * K + K * K) * n};
		}
		n = 1 / (1 + s2V * K + V * K * K);
		return {(1 + s2 * K + K * K) * n, 2 * (K * K - 1) * n, (1 - s2 * K + K * K) * n, 2 * (V * K * K - 1) * n, (1 - s2V * K + V * K * K) * n};
	case BiquadType::highshelf:
		if (boost) {
			n = 1 / (1 + s2 * K + K * K);
			return {(V + s2V * K + K * K) * n, 2 * (K * K - V) * n, (V - s2V * K + K * K) * n, 2 * (K * K - 1) * n, (1 - s2 * K + K * K) * n};
		}
		n = 1 / (V + s2V * K + K * K);
		return {(1 + s2 * K + K * K) * n, 2 * (K * K - 1) * n, (1 - s2 * K + K * K) * n, 2 * (K * K - V) * n, (V - s2V * K + K * K) * n};
	default:
		FAIL("no Zoelzer reference for this type");
		return {};
	}
}

/** |H(e^{j 2 pi f})| of a coefficient set, f relative to the sample rate */
double magnitude(const Coeffs &c, double f) {
	const std::complex<double> z1 = std::polar(1.0, -2.0 * M_PI * f); // z^-1
	const std::complex<double> num = c.b0 + c.b1 * z1 + c.b2 * z1 * z1;
	const std::complex<double> den = 1.0 + c.a1 * z1 + c.a2 * z1 * z1;
	return std::abs(num / den);
}

double dbToGain(double db) { return std::pow(10.0, db / 20.0); }

void checkCoeffs(const Coeffs &actual, const Coeffs &expected) {
	CHECK(actual.b0 == Approx(expected.b0).epsilon(COEFF_EPS));
	CHECK(actual.b1 == Approx(expected.b1).epsilon(COEFF_EPS));
	CHECK(actual.b2 == Approx(expected.b2).epsilon(COEFF_EPS));
	CHECK(actual.a1 == Approx(expected.a1).epsilon(COEFF_EPS));
	CHECK(actual.a2 == Approx(expected.a2).epsilon(COEFF_EPS));
}

/** True if all 5 coefficients of two filters are identical */
bool sameCoeffs(const Biquad &a, const Biquad &b) {
	for (int i = 0; i < 5; i++) {
		if (a.getCoeffs()[i] != b.getCoeffs()[i]) return false;
	}
	return true;
}

/** Fc / Q combinations used for the coefficient comparisons (Fc relative to the sample rate) */
struct Design {
	double fc, q;
};
const Design designs[] = {{0.01, 0.707}, {0.05, 0.5}, {0.1, 1.0}, {0.25, 0.707}, {0.4, 2.0}};

/**
 * Amplitude of the output for a sine input of amplitude 1 after the filter settled.
 * Measured by correlation with sin/cos over the measurement window, so it does not depend on
 * where the samples fall on the sine (use windows with an integer number of periods).
 */
double sineGain(Biquad &filter, double f, int settle = 2000, int measure = 2000) {
	std::complex<double> acc = 0;
	for (int n = 0; n < settle + measure; n++) {
		const double phase = 2.0 * M_PI * f * n;
		const float out = filter.process((float)std::sin(phase));
		if (n >= settle) acc += (double)out * std::polar(1.0, -phase);
	}
	return 2.0 * std::abs(acc) / measure;
}

/** Output after a constant input was applied for some samples */
float settle(Biquad &filter, float in, int samples = 3000) {
	float out = 0;
	for (int i = 0; i < samples; i++) out = filter.process(in);
	return out;
}

} // namespace

TEST_SUITE("Filters") {

// ---------------- Coefficients against the RBJ cookbook ----------------

TEST_CASE("lowpass coefficients match the RBJ cookbook") {
	for (const Design &d : designs) {
		CAPTURE(d.fc);
		CAPTURE(d.q);
		Biquad filter(BiquadType::lowpass, (float)d.fc, (float)d.q, 0);
		checkCoeffs(firmwareCoeffs(filter), rbjLowpass(d.fc, d.q));
	}
}

TEST_CASE("highpass coefficients match the RBJ cookbook") {
	for (const Design &d : designs) {
		CAPTURE(d.fc);
		CAPTURE(d.q);
		Biquad filter(BiquadType::highpass, (float)d.fc, (float)d.q, 0);
		checkCoeffs(firmwareCoeffs(filter), rbjHighpass(d.fc, d.q));
	}
}

TEST_CASE("bandpass coefficients match the RBJ cookbook band pass with 0 dB peak gain") {
	for (const Design &d : designs) {
		CAPTURE(d.fc);
		CAPTURE(d.q);
		Biquad filter(BiquadType::bandpass, (float)d.fc, (float)d.q, 0);
		checkCoeffs(firmwareCoeffs(filter), rbjBandpass(d.fc, d.q));
	}
}

TEST_CASE("notch coefficients match the RBJ cookbook") {
	for (const Design &d : designs) {
		CAPTURE(d.fc);
		CAPTURE(d.q);
		Biquad filter(BiquadType::notch, (float)d.fc, (float)d.q, 0);
		checkCoeffs(firmwareCoeffs(filter), rbjNotch(d.fc, d.q));
	}
}

TEST_CASE("coefficients are stored in CMSIS-DSP order with negated feedback terms") {
	// Lowpass at fc = 0.1, Q = 1: cookbook a1 = -1.4074, a2 = 0.6293 (a0 normalised to 1)
	Biquad filter(BiquadType::lowpass, 0.1f, 1.0f, 0);
	const Coeffs ref = rbjLowpass(0.1, 1.0);
	const float *c = filter.getCoeffs();
	CHECK(c[0] == Approx(ref.b0).epsilon(COEFF_EPS));
	CHECK(c[1] == Approx(ref.b1).epsilon(COEFF_EPS));
	CHECK(c[2] == Approx(ref.b2).epsilon(COEFF_EPS));
	CHECK(c[3] == Approx(-ref.a1).epsilon(COEFF_EPS)); // positive for a lowpass
	CHECK(c[4] == Approx(-ref.a2).epsilon(COEFF_EPS)); // negative
	CHECK(c[3] > 0);
	CHECK(c[4] < 0);
}

// ---------------- Peak and shelf filters ----------------

// NEEDS REVIEW: the peak filter uses the earlevel/Zoelzer form with V = 10^(gain/20) in the numerator
// only. The RBJ cookbook uses A = 10^(gain/40) in numerator and denominator. Both have the same gain at
// fc, DC and Nyquist but a different bandwidth for the same Q. Which Q definition is intended?
TEST_CASE("characterization: peak filter coefficients follow the earlevel/Zoelzer design, not the RBJ cookbook") {
	for (double gain : {6.0, -6.0, 12.0, -3.0}) {
		for (const Design &d : designs) {
			CAPTURE(gain);
			CAPTURE(d.fc);
			CAPTURE(d.q);
			Biquad filter(BiquadType::peak, (float)d.fc, (float)d.q, (float)gain);
			const Coeffs fw = firmwareCoeffs(filter);
			checkCoeffs(fw, zoelzer(BiquadType::peak, d.fc, d.q, gain));
			CHECK(std::fabs(fw.a2 - rbjPeak(d.fc, d.q, gain).a2) > 1e-3); // Differs from the cookbook
		}
	}
}

TEST_CASE("peak filter has the requested gain at fc and unity gain at DC and Nyquist") {
	for (double gain : {6.0, -6.0, 12.0, -12.0}) {
		CAPTURE(gain);
		Biquad filter(BiquadType::peak, 0.1f, 0.707f, (float)gain);
		const Coeffs c = firmwareCoeffs(filter);
		CHECK(magnitude(c, 0.1) == Approx(dbToGain(gain)).epsilon(1e-3));
		CHECK(magnitude(c, 0.0) == Approx(1.0).epsilon(1e-3));
		CHECK(magnitude(c, 0.5) == Approx(1.0).epsilon(1e-3));
	}
}

TEST_CASE("peak filter with 0 dB gain passes all frequencies unchanged") {
	Biquad filter(BiquadType::peak, 0.1f, 0.707f, 0);
	const Coeffs c = firmwareCoeffs(filter);
	for (double f : {0.0, 0.05, 0.1, 0.3, 0.5}) {
		CAPTURE(f);
		CHECK(magnitude(c, f) == Approx(1.0).epsilon(1e-4));
	}
}

// NEEDS REVIEW: the shelf filters ignore Q (fixed Q = 1/sqrt(2)) and use the Zoelzer/earlevel design.
// RBJ cookbook shelves with S = 1 give different coefficients (the cookbook fc is the half gain point).
TEST_CASE("characterization: shelf filter coefficients follow the earlevel/Zoelzer design") {
	for (BiquadType type : {BiquadType::lowshelf, BiquadType::highshelf}) {
		for (double gain : {6.0, -6.0, 12.0, -3.0}) {
			for (const Design &d : designs) {
				CAPTURE((int)type);
				CAPTURE(gain);
				CAPTURE(d.fc);
				Biquad filter(type, (float)d.fc, (float)d.q, (float)gain);
				checkCoeffs(firmwareCoeffs(filter), zoelzer(type, d.fc, d.q, gain));
			}
		}
	}
}

TEST_CASE("characterization: Q has no influence on shelf filters") {
	for (BiquadType type : {BiquadType::lowshelf, BiquadType::highshelf}) {
		CAPTURE((int)type);
		Biquad a(type, 0.1f, 0.5f, 6.0f);
		Biquad b(type, 0.1f, 3.0f, 6.0f);
		CHECK(sameCoeffs(a, b));
	}
}

TEST_CASE("lowshelf has the shelf gain at DC and unity gain at Nyquist") {
	for (double gain : {6.0, -6.0, 12.0}) {
		CAPTURE(gain);
		Biquad filter(BiquadType::lowshelf, 0.05f, 0.707f, (float)gain);
		const Coeffs c = firmwareCoeffs(filter);
		CHECK(magnitude(c, 0.0) == Approx(dbToGain(gain)).epsilon(1e-3));
		CHECK(magnitude(c, 0.5) == Approx(1.0).epsilon(1e-3));
	}
}

TEST_CASE("highshelf has unity gain at DC and the shelf gain at Nyquist") {
	for (double gain : {6.0, -6.0, 12.0}) {
		CAPTURE(gain);
		Biquad filter(BiquadType::highshelf, 0.05f, 0.707f, (float)gain);
		const Coeffs c = firmwareCoeffs(filter);
		CHECK(magnitude(c, 0.0) == Approx(1.0).epsilon(1e-3));
		CHECK(magnitude(c, 0.5) == Approx(dbToGain(gain)).epsilon(1e-3));
	}
}

TEST_CASE("shelf cut is the inverse of the shelf boost") {
	for (BiquadType type : {BiquadType::lowshelf, BiquadType::highshelf}) {
		CAPTURE((int)type);
		Biquad boost(type, 0.08f, 0.707f, 9.0f);
		Biquad cut(type, 0.08f, 0.707f, -9.0f);
		for (double f : {0.0, 0.02, 0.08, 0.2, 0.5}) {
			CAPTURE(f);
			CHECK(magnitude(firmwareCoeffs(boost), f) * magnitude(firmwareCoeffs(cut), f) == Approx(1.0).epsilon(1e-3));
		}
	}
}

// ---------------- Frequency response measured with process() ----------------

TEST_CASE("lowpass step response settles at the input value") {
	Biquad filter(BiquadType::lowpass, 0.05f, 0.707f, 0);
	CHECK(settle(filter, 1.0f) == Approx(1.0).epsilon(1e-4));
	CHECK(settle(filter, -250.0f) == Approx(-250.0).epsilon(1e-4));
}

TEST_CASE("lowpass step response starts with b0 and rises smoothly without overshoot for Q 0.5") {
	Biquad filter(BiquadType::lowpass, 0.05f, 0.5f, 0);
	const Coeffs c = firmwareCoeffs(filter);
	float prev = filter.process(1.0f);
	CHECK(prev == Approx(c.b0).epsilon(1e-4));
	for (int i = 0; i < 500; i++) {
		const float out = filter.process(1.0f);
		CHECK(out >= prev - 1e-6f); // Critically damped: monotonic
		CHECK(out <= 1.0f + 1e-5f);
		prev = out;
	}
}

TEST_CASE("highpass blocks a constant input") {
	Biquad filter(BiquadType::highpass, 0.05f, 0.707f, 0);
	CHECK(filter.process(1.0f) == Approx(firmwareCoeffs(filter).b0).epsilon(1e-4)); // The step passes first
	CHECK(std::fabs(settle(filter, 1.0f)) < 1e-4);
}

TEST_CASE("lowpass passes a low frequency sine and attenuates a high frequency sine") {
	Biquad low(BiquadType::lowpass, 0.05f, 0.707f, 0);
	CHECK(sineGain(low, 0.005) == Approx(1.0).epsilon(0.01));
	Biquad high(BiquadType::lowpass, 0.05f, 0.707f, 0);
	const double gain = sineGain(high, 0.25);
	CHECK(gain == Approx(magnitude(rbjLowpass(0.05, 0.707), 0.25)).epsilon(0.01));
	CHECK(gain < 0.05);
}

TEST_CASE("lowpass gain at fc equals Q") {
	for (double q : {0.5, 0.707, 2.0}) {
		CAPTURE(q);
		Biquad filter(BiquadType::lowpass, 0.05f, (float)q, 0);
		CHECK(magnitude(firmwareCoeffs(filter), 0.05) == Approx(q).epsilon(1e-3));
		CHECK(sineGain(filter, 0.05) == Approx(q).epsilon(0.01));
	}
}

TEST_CASE("bandpass has unity gain at fc and blocks DC") {
	Biquad filter(BiquadType::bandpass, 0.1f, 2.0f, 0);
	CHECK(sineGain(filter, 0.1) == Approx(1.0).epsilon(0.01));
	CHECK(magnitude(firmwareCoeffs(filter), 0.0) < 1e-6);
}

TEST_CASE("notch removes a sine at fc and passes other frequencies") {
	Biquad filter(BiquadType::notch, 0.1f, 2.0f, 0);
	CHECK(sineGain(filter, 0.1, 4000) < 0.01);
	Biquad other(BiquadType::notch, 0.1f, 2.0f, 0);
	CHECK(sineGain(other, 0.01) == Approx(1.0).epsilon(0.01));
}

TEST_CASE("peak filter amplifies a sine at fc by the peak gain") {
	Biquad filter(BiquadType::peak, 0.1f, 1.0f, 6.0f);
	CHECK(sineGain(filter, 0.1) == Approx(dbToGain(6.0)).epsilon(0.01));
}

TEST_CASE("process output matches the difference equation of the coefficients") {
	// The DSP build uses direct form I, the fallback transposed direct form II.
	// Both must give the output of the same difference equation.
	for (BiquadType type : {BiquadType::lowpass, BiquadType::highpass, BiquadType::bandpass, BiquadType::notch, BiquadType::peak,
							BiquadType::lowshelf, BiquadType::highshelf}) {
		CAPTURE((int)type);
		Biquad filter(type, 0.07f, 0.9f, 4.0f);
		const Coeffs c = firmwareCoeffs(filter);
		double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
		for (int n = 0; n < 200; n++) {
			const double x = std::sin(0.3 * n) * 100.0 + ((n % 17) - 8) * 7.0; // deterministic, broadband
			const double y = c.b0 * x + c.b1 * x1 + c.b2 * x2 - c.a1 * y1 - c.a2 * y2;
			const float out = filter.process((float)x);
			CAPTURE(n);
			CHECK_NEAR(out, y, 1e-3 + 1e-5 * std::fabs(y));
			x2 = x1;
			x1 = x;
			y2 = y1;
			y1 = y;
		}
	}
}

// ---------------- Construction and setters ----------------

TEST_CASE("default constructed filter configured with setBiquad equals the parametrised constructor") {
	Biquad configured;
	configured.setBiquad(BiquadType::highpass, 0.12f, 0.8f, 0);
	Biquad constructed(BiquadType::highpass, 0.12f, 0.8f, 0);

	CHECK(sameCoeffs(configured, constructed));
	CHECK(configured.getFc() == 0.12f);
	CHECK(configured.getQ() == 0.8f);
	for (int i = 0; i < 50; i++) {
		const float in = (float)((i * 37) % 11) - 5.0f;
		CHECK(configured.process(in) == constructed.process(in));
	}
}

TEST_CASE("a vector of default constructed filters can be configured like AnalogAxisProcessing does") {
	std::vector<Biquad> filters(3);
	for (Biquad &f : filters) f.setBiquad(BiquadType::lowpass, 0.03f, 0.5f, 0.0f);
	for (Biquad &f : filters) {
		checkCoeffs(firmwareCoeffs(f), rbjLowpass(0.03, 0.5));
		CHECK(settle(f, 2.0f) == Approx(2.0).epsilon(1e-4));
	}
}

TEST_CASE("setFc changes the frequency and keeps type and Q") {
	Biquad filter(BiquadType::bandpass, 0.1f, 1.5f, 0);
	filter.setFc(0.2f);
	CHECK(filter.getFc() == 0.2f);
	CHECK(filter.getQ() == 1.5f);
	checkCoeffs(firmwareCoeffs(filter), rbjBandpass(0.2, 1.5));
}

TEST_CASE("setFc clips the frequency to the range 0 to 0.5") {
	Biquad filter(BiquadType::lowpass, 0.1f, 0.707f, 0);
	filter.setFc(0.7f);
	CHECK(filter.getFc() == 0.5f);
	filter.setFc(-0.1f);
	CHECK(filter.getFc() == 0.0f);
	filter.setFc(0.5f);
	CHECK(filter.getFc() == 0.5f);
	filter.setFc(0.0f);
	CHECK(filter.getFc() == 0.0f);
}

TEST_CASE("the constructor and setBiquad clip the frequency like setFc") {
	Biquad high(BiquadType::lowpass, 3.0f, 0.707f, 0);
	CHECK(high.getFc() == 0.5f);
	Biquad low(BiquadType::lowpass, -1.0f, 0.707f, 0);
	CHECK(low.getFc() == 0.0f);
	low.setBiquad(BiquadType::lowpass, 0.9f, 0.707f, 0);
	CHECK(low.getFc() == 0.5f);
}

TEST_CASE("setQ changes the quality factor and getQ returns it") {
	Biquad filter(BiquadType::lowpass, 0.1f, 0.707f, 0);
	CHECK(filter.getQ() == 0.707f);
	filter.setQ(2.5f);
	CHECK(filter.getQ() == 2.5f);
	CHECK(filter.getFc() == 0.1f);
	checkCoeffs(firmwareCoeffs(filter), rbjLowpass(0.1, 2.5));
}

TEST_CASE("setPeakGain changes the gain of a peak filter") {
	Biquad filter(BiquadType::peak, 0.1f, 1.0f, 3.0f);
	filter.setPeakGain(-9.0f);
	checkCoeffs(firmwareCoeffs(filter), zoelzer(BiquadType::peak, 0.1, 1.0, -9.0));
	CHECK(magnitude(firmwareCoeffs(filter), 0.1) == Approx(dbToGain(-9.0)).epsilon(1e-3));
	CHECK(filter.getQ() == 1.0f);
	CHECK(filter.getFc() == 0.1f);
}

TEST_CASE("setPeakGain has no influence on lowpass filters") {
	Biquad a(BiquadType::lowpass, 0.1f, 1.0f, 0.0f);
	Biquad b(BiquadType::lowpass, 0.1f, 1.0f, 0.0f);
	b.setPeakGain(12.0f);
	CHECK(sameCoeffs(a, b));
}

TEST_CASE("setBiquad changes the filter type") {
	Biquad filter(BiquadType::lowpass, 0.1f, 0.707f, 0);
	filter.setBiquad(BiquadType::notch, 0.15f, 3.0f, 0);
	CHECK(filter.getFc() == 0.15f);
	CHECK(filter.getQ() == 3.0f);
	checkCoeffs(firmwareCoeffs(filter), rbjNotch(0.15, 3.0));
}

// ---------------- Reset behaviour ----------------

TEST_CASE("changing a parameter clears the filter state") {
	// After a step the state is non zero. With a cleared state the response to input 0 is exactly 0.
	Biquad f(BiquadType::lowpass, 0.05f, 0.707f, 0);
	settle(f, 100.0f, 200);

	SUBCASE("unchanged filter keeps its state (reference)") {
		CHECK(f.process(0.0f) > 90.0f); // Decays slowly from 100
	}
	SUBCASE("setFc") {
		f.setFc(0.05f);
		CHECK(f.process(0.0f) == 0.0f);
	}
	SUBCASE("setQ") {
		f.setQ(0.707f);
		CHECK(f.process(0.0f) == 0.0f);
	}
	SUBCASE("setPeakGain") {
		f.setPeakGain(0.0f);
		CHECK(f.process(0.0f) == 0.0f);
	}
	SUBCASE("setBiquad") {
		f.setBiquad(BiquadType::lowpass, 0.05f, 0.707f, 0);
		CHECK(f.process(0.0f) == 0.0f);
	}
	SUBCASE("calcBiquad") {
		f.calcBiquad();
		CHECK(f.process(0.0f) == 0.0f);
	}
}

TEST_CASE("a filter after a parameter change behaves like a new filter") {
	Biquad used(BiquadType::lowpass, 0.05f, 0.707f, 0);
	settle(used, 50.0f, 100);
	used.setQ(1.2f);
	Biquad fresh(BiquadType::lowpass, 0.05f, 1.2f, 0);
	for (int i = 0; i < 20; i++) {
		CHECK(used.process(1.0f) == Approx(fresh.process(1.0f)).epsilon(1e-6));
	}
}

// ---------------- Invalid parameters ----------------

// NEEDS REVIEW: a frequency of 0 (or a negative one, clipped to 0) gives an all-zero numerator,
// the filter outputs 0 for every input instead of being a pass-through or rejected.
TEST_CASE("characterization: a lowpass with frequency 0 outputs zero") {
	Biquad filter(BiquadType::lowpass, -0.2f, 0.707f, 0);
	CHECK(filter.getFc() == 0.0f);
	const Coeffs c = firmwareCoeffs(filter);
	CHECK(c.b0 == 0.0);
	CHECK(c.b1 == 0.0);
	CHECK(c.b2 == 0.0);
	CHECK(settle(filter, 1.0f) == 0.0f);
}

// NEEDS REVIEW: Biquad does not validate Q. Q = 0 divides by zero (K/Q = inf) and gives a NaN
// feedback coefficient, the filter output is NaN from then on. EffectsCalculator::checkFilterCoeff
// maps q = 0 to 1, the Axis constants are non zero, so no caller is known to pass 0.
TEST_CASE("characterization: a Q of zero makes the filter output NaN") {
	Biquad filter(BiquadType::lowpass, 0.1f, 0.0f, 0);
	CHECK(std::isnan(filter.getCoeffs()[4]));
	CHECK(std::isnan(settle(filter, 1.0f, 3))); // The fallback build needs 2 samples until the NaN reaches the output
}

#ifdef USE_DSP_FUNCTIONS
#define NYQUIST_ISSUE                                                                                                       \
	*KNOWN_ISSUE("Filters.cpp:102 K = arm_sin_f32(PI*Fc)/arm_cos_f32(PI*Fc) with Fc = 0.5 (allowed by the clip at "        \
				 "Filters.cpp:44/86): arm_cos_f32 returns exactly 0, K = inf and all coefficients are NaN")
#else
#define NYQUIST_ISSUE
#endif

// Suspected bug (DSP build, the configuration of the hardware targets): Fc = 0.5 is accepted by the
// clip in setFc/setBiquad and EffectsCalculator::checkFilterCoeff passes exactly calcfrequency/2
// (also for a stored frequency of 0). With CMSIS-DSP all coefficients become NaN.
// The fallback build computes tan() in double and gives the finite limit {1, 2, 1, -2, -1}, i.e.
// H(z) = 1 with a double pole on the unit circle (z = -1).
// Intended: finite coefficients and a filter that passes DC.
TEST_CASE("lowpass at the highest allowed frequency 0.5 has finite coefficients and passes a constant input" NYQUIST_ISSUE) {
	Biquad filter(BiquadType::lowpass, 0.5f, 0.707f, 0);
	for (int i = 0; i < 5; i++) {
		CAPTURE(i);
		REQUIRE(std::isfinite(filter.getCoeffs()[i]));
	}
	CHECK(settle(filter, 1.0f, 50) == Approx(1.0).epsilon(1e-3));
}

#ifdef USE_DSP_FUNCTIONS
#define COPY_ISSUE                                                                                                          \
	*KNOWN_ISSUE("Filters.h: the CMSIS-DSP instance S stores pointers to pCoeffs/pState of the object it was initialised "   \
				 "in (Filters.cpp:22/30). The implicit copy keeps pointing to the original, so the copy filters with the " \
				 "coefficients and state of the original")
#else
#define COPY_ISSUE
#endif

// Suspected latent bug (DSP build): Biquad has no copy constructor / assignment, but holds an
// arm_biquad_casd_df1_inst_f32 with pointers into itself. No current caller copies a Biquad
// (vectors/arrays are default constructed in place, EffectsCalculator uses unique_ptr), so this is
// only a hazard. The original is kept alive here, so the test has no undefined behaviour.
TEST_CASE("a copied filter works independently of the original" COPY_ISSUE) {
	Biquad original(BiquadType::lowpass, 0.05f, 0.707f, 0);
	Biquad copy = original;
	copy.setFc(0.3f); // Recalculates the coefficients of the copy only
	Biquad reference(BiquadType::lowpass, 0.3f, 0.707f, 0);
	CHECK(copy.process(1.0f) == Approx(reference.process(1.0f)).epsilon(1e-6));
	CHECK(original.process(1.0f) == Approx(firmwareCoeffs(original).b0).epsilon(1e-6)); // original still has a clean state
}

} // TEST_SUITE
