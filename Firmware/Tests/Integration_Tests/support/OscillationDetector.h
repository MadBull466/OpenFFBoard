/*
 * OscillationDetector.h
 *
 * Measures the oscillation of a sampled signal after an excitation (plan section 6.3 step 3): dominant frequency, the
 * cycle-to-cycle amplitude ratio and a classification decaying / sustained / growing / insufficient. Pure host code (no
 * firmware, no fake): it works on any sequence of doubles, so it is self-tested on synthetic signals A e^{sigma t} sin(2 pi f t + phi).
 *
 * METHOD (documented, strict)
 *  1. Centre: c = Config::center is subtracted (default 0: signals are taken relative to the equilibrium; the detector does
 *     not estimate it).
 *  2. Half-waves with hysteresis (a Schmitt trigger of +-Config::floor): a positive half-wave starts when the signal rises above
 *     +floor, a negative one when it falls below -floor; a half-wave lasts until the signal crosses the opposite threshold.
 *     Excursions that stay inside +-floor (quantisation chatter) therefore neither start nor end a half-wave. The extremum of the
 *     half-wave is its peak; its time and amplitude are refined by a parabola through the extreme sample and its two neighbours
 *     (error O(dt^2) of the sampling for a smooth signal; at the ends of the record the sample itself).
 *  3. Peaks of the same sign form two sequences (positive and negative). The cycle ratio is the ratio of the amplitudes
 *     |peak| of successive same-sign peaks: one full cycle apart. For A e^{sigma t} sin(...) it is exactly e^{sigma / f}.
 *     With a CONSTANT amplitude the ratio does not depend on the centre; with a decaying or growing amplitude and a wrong
 *     Config::center it does: for a true offset d the peaks on the offset's side are A_k + d (ratio pulled TOWARDS 1) and
 *     those on the other side A_k - d (ratio pushed AWAY from 1). Give the true centre for trend measurements.
 *  4. The sequence with more peaks is used (positive if equal). With n peaks there are n - 1 cycles. Result values:
 *       frequencyHz = (n - 1) / (t_last - t_first)   (mean period between same-sign peaks)
 *       ratio       = (A_last / A_first)^(1 / (n - 1))  geometric mean cycle ratio of the WHOLE record (reported, not used to classify)
 *       cycleRatios = the n - 1 individual ratios
 *  5. Classification on the TRAILING WINDOW (lead decision 2026-11-18: a decay that settles into a limit cycle must be
 *     sustained; the state at the END of the record is what the closed-loop tests ask). W = the last min(minCycles, cycles)
 *     cycle ratios; windowRatio = their geometric mean. eps = Config::eps:
 *       cycles < minTrendCycles                                   -> insufficient
 *       cycles >= minCycles (W = the last minCycles ratios):
 *           every ratio of W within 1 +- eps                      -> sustained (plan 6.3: ratio within 1 +- eps for >= 10 cycles
 *                                                                    above the quantisation floor)
 *           else windowRatio > 1 + eps                            -> growing
 *           else windowRatio < 1 - eps                            -> decaying
 *           else (mean inside the band but a single ratio outside)-> insufficient (irregular: the caller inspects cycleRatios)
 *       minTrendCycles <= cycles < minCycles (W = all ratios): windowRatio > 1 + eps -> growing, < 1 - eps -> decaying, else
 *           insufficient (an early trend call, accepted by the lead 2026-11-18; sustained is never called with fewer than minCycles).
 *     "Above the floor": only half-waves whose peak exceeds the floor exist at all (step 2), so a signal that has decayed into
 *     the floor ends the sequence there and a quantised signal that never leaves the floor gives insufficient.
 *
 * LIMITS
 *  - Only the last minCycles cycles decide: an earlier transient is not judged (see `ratio` for the whole record). An amplitude
 *    that beats with a period shorter than the window and a mean inside the band is reported insufficient or sustained by the
 *    window rule; inspect cycleRatios when that matters. Two superposed frequencies are not
 *    separated (the peaks are those of the sum). A signal with an offset needs Config::center; a drifting offset is not handled.
 *  - The peak amplitude of a quantised signal is known to one quantisation step: choose the floor and amplitude so that one
 *    step is much smaller than eps times the amplitude over the cycles used, or the ratio scatters by that much.
 *  - A half-wave cut by the end of the record has a peak too (the extreme so far): the caller should stop the record at the
 *    point of interest; the peak is only used if it is a true local extremum of the samples (not at the first or last sample).
 *  - Peaks need at least 3 samples per half-wave to be a parabola: sample at least about 8 times per period.
 */
#ifndef HOSTTEST_INTEGRATION_OSCILLATIONDETECTOR_H_
#define HOSTTEST_INTEGRATION_OSCILLATIONDETECTOR_H_

#include <cstddef>
#include <string>
#include <vector>

class OscillationDetector {
public:
	enum class Class { Insufficient, Decaying, Sustained, Growing };

	struct Config {
		double samplePeriodS = 0.0; //!< seconds between samples (> 0, required)
		double floor = 0.0;			//!< hysteresis and minimum peak amplitude, in signal units (> 0, required: the quantisation level)
		double eps = 0.05;			//!< tolerance of the cycle ratio for sustained (plan 6.3)
		size_t minCycles = 10;		//!< cycles needed to call a signal sustained (plan 6.3)
		size_t minTrendCycles = 3;	//!< cycles needed to call a trend (decaying / growing)
		double center = 0.0;		//!< value subtracted from the signal
	};

	struct Peak {
		double timeS = 0.0;
		double amplitude = 0.0; //!< |peak| relative to the centre
		bool positive = true;
	};

	struct Result {
		Class cls = Class::Insufficient;
		bool valid = false;		//!< false if the configuration or the input is unusable (see error)
		std::string error;
		double frequencyHz = 0.0;
		double ratio = 0.0;		//!< geometric mean cycle ratio of the whole record, 0 if cycles < 1
		double windowRatio = 0.0; //!< geometric mean of the trailing window of cycle ratios (the classification input)
		size_t windowCycles = 0;  //!< length of that window: min(minCycles, cycles)
		size_t cycles = 0;		//!< same-sign peak pairs used
		std::vector<double> cycleRatios;
		std::vector<Peak> peaks; //!< all peaks above the floor, in time order
		double firstAmplitude = 0.0;
		double lastAmplitude = 0.0;
	};

	static Result analyse(const std::vector<double> &signal, const Config &cfg);
	static const char *name(Class c);
};

#endif
