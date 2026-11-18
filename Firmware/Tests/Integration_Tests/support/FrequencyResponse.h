/*
 * FrequencyResponse.h
 *
 * Gain and phase of an output sequence relative to an input sequence at one known frequency, by the single-bin DFT over a whole
 * number of periods (plan section 6.3 step 2, the F1-O1 conformance measurement). Pure host code, self-tested on a synthetic delay
 * and a first-order IIR with their analytic responses.
 *
 * METHOD
 *  For a window of N samples at sample rate fs and test frequency f, with N f / fs = P a whole number of periods:
 *      X = sum_{n=0}^{N-1} x[n] e^{-j 2 pi f n / fs},   Y = the same for the output,   H = Y / X
 *      gain = |H|,   phase = arg(H) in (-pi, pi]
 *  Phase convention: the phase of the output relative to the input; an output that LAGS the input (a delay, a low-pass) has a
 *  NEGATIVE phase: y[n] = x[n - d] gives phase = -2 pi f d / fs (wrapped). The input is a sequence (not assumed to be a perfect
 *  sinusoid), so the measured input amplitude and phase of the actual stimulus enter H.
 *  Whole periods: the DC component and every harmonic of f whose period divides the window are orthogonal to the bin, so a
 *  constant offset and distortion at integer multiples of f do not bias the result; any other frequency leaks.
 *
 * ERRORS (valid = false, error text, gain / phase 0): sizes differ or empty; fs or f <= 0; f at or above fs / 2 (fewer than 2
 * samples per period); N f / fs not a whole number (relative tolerance 1e-9 of one period of the window); fewer than minPeriods periods
 * (Options::minPeriods, required: the caller chooses; the window must also hold at least 4 samples per period); input energy at
 * the bin zero (|X| <= 1e-12 times the sum of |x|, nothing to divide by); NaN / infinite samples.
 *
 * LIMITS: noise and other frequencies are not rejected (the bin only sees what leaks); transients in the window bias the result: the
 * caller discards the settling time first. The result is exact up to floating point only for exactly sinusoidal inputs and a linear
 * time-invariant output.
 */
#ifndef HOSTTEST_INTEGRATION_FREQUENCYRESPONSE_H_
#define HOSTTEST_INTEGRATION_FREQUENCYRESPONSE_H_

#include <complex>
#include <string>
#include <vector>

class FrequencyResponse {
public:
	struct Options {
		double sampleRateHz = 0.0; //!< fs, required
		double frequencyHz = 0.0;  //!< f, required
		size_t minPeriods = 0;	   //!< minimum number of whole periods in the window (required, >= 1)
	};

	struct Result {
		bool valid = false;
		std::string error;
		double gain = 0.0;
		double phaseRad = 0.0;	//!< output lagging = negative
		double phaseDeg = 0.0;
		double periods = 0.0;	//!< N f / fs
		std::complex<double> inputBin;
		std::complex<double> outputBin;
	};

	static Result measure(const std::vector<double> &input, const std::vector<double> &output, const Options &opt);
	/** Single-bin DFT of x at f (no window checks): sum x[n] e^{-j 2 pi f n / fs} */
	static std::complex<double> bin(const std::vector<double> &x, double sampleRateHz, double frequencyHz);
};

#endif
