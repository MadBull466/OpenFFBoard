/*
 * OscillationDetector.cpp: see OscillationDetector.h for the method and its limits.
 */
#include "OscillationDetector.h"

#include <cmath>

const char *OscillationDetector::name(Class c) {
	switch (c) {
	case Class::Decaying: return "decaying";
	case Class::Sustained: return "sustained";
	case Class::Growing: return "growing";
	default: return "insufficient";
	}
}

OscillationDetector::Result OscillationDetector::analyse(const std::vector<double> &signal, const Config &cfg) {
	Result r;
	if (!(cfg.samplePeriodS > 0.0) || !(cfg.floor > 0.0) || !(cfg.eps > 0.0) || cfg.minCycles == 0 || cfg.minTrendCycles == 0) {
		r.error = "OscillationDetector: samplePeriodS, floor and eps must be > 0 and the cycle counts >= 1";
		return r;
	}
	for (double v : signal) {
		if (!std::isfinite(v)) {
			r.error = "OscillationDetector: the signal contains a NaN or infinite sample";
			return r;
		}
	}
	r.valid = true;
	const size_t n = signal.size();
	auto x = [&](size_t i) { return signal[i] - cfg.center; };

	// Half-waves with hysteresis (method step 2)
	int state = 0; // +1 in a positive half-wave, -1 in a negative one, 0 none yet
	size_t bestIdx = 0;
	double bestVal = 0.0;
	auto closeWave = [&]() {
		if (state == 0) {
			return;
		}
		// A true local extremum only: not the first or last sample of the record
		if (bestIdx == 0 || bestIdx + 1 >= n) {
			return;
		}
		const double ym = x(bestIdx - 1);
		const double y0 = x(bestIdx);
		const double yp = x(bestIdx + 1);
		const double denom = ym - 2.0 * y0 + yp;
		double off = 0.0;
		double val = y0;
		if (denom != 0.0) {
			off = 0.5 * (ym - yp) / denom; // vertex of the parabola, in samples relative to bestIdx
			if (off > 1.0) off = 1.0;
			if (off < -1.0) off = -1.0;
			val = y0 - 0.25 * (ym - yp) * off;
		}
		Peak p;
		p.timeS = (static_cast<double>(bestIdx) + off) * cfg.samplePeriodS;
		p.amplitude = std::fabs(val);
		p.positive = state > 0;
		if (p.amplitude > cfg.floor) {
			r.peaks.push_back(p);
		}
	};
	for (size_t i = 0; i < n; i++) {
		const double v = x(i);
		const int want = v > cfg.floor ? 1 : (v < -cfg.floor ? -1 : 0);
		if (want != 0 && want != state) {
			closeWave();
			state = want;
			bestIdx = i;
			bestVal = v;
		} else if (state != 0 && v * state > bestVal * state) {
			bestIdx = i;
			bestVal = v;
		}
	}
	closeWave();

	// Same-sign sequences (step 3, 4)
	std::vector<Peak> pos, neg;
	for (const Peak &p : r.peaks) {
		(p.positive ? pos : neg).push_back(p);
	}
	const std::vector<Peak> &use = pos.size() >= neg.size() ? pos : neg;
	if (use.size() >= 2) {
		r.cycles = use.size() - 1;
		for (size_t i = 1; i < use.size(); i++) {
			r.cycleRatios.push_back(use[i].amplitude / use[i - 1].amplitude);
		}
		r.firstAmplitude = use.front().amplitude;
		r.lastAmplitude = use.back().amplitude;
		r.frequencyHz = static_cast<double>(r.cycles) / (use.back().timeS - use.front().timeS);
		r.ratio = std::pow(use.back().amplitude / use.front().amplitude, 1.0 / static_cast<double>(r.cycles));
	}

	// Classification on the trailing window (step 5)
	r.cls = Class::Insufficient;
	if (r.cycles >= 1) {
		r.windowCycles = r.cycles < cfg.minCycles ? r.cycles : cfg.minCycles;
		double sumLog = 0.0;
		bool allInBand = true;
		for (size_t i = r.cycles - r.windowCycles; i < r.cycles; i++) {
			sumLog += std::log(r.cycleRatios[i]);
			allInBand = allInBand && std::fabs(r.cycleRatios[i] - 1.0) <= cfg.eps;
		}
		r.windowRatio = std::exp(sumLog / static_cast<double>(r.windowCycles));
		if (r.cycles >= cfg.minTrendCycles) {
			if (r.cycles >= cfg.minCycles && allInBand) {
				r.cls = Class::Sustained;
			} else if (r.windowRatio > 1.0 + cfg.eps) {
				r.cls = Class::Growing;
			} else if (r.windowRatio < 1.0 - cfg.eps) {
				r.cls = Class::Decaying;
			}
		}
	}
	return r;
}
