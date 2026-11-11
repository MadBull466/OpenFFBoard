/*
 * FakeTmcPhase.cpp: recorder of the TMC4671 double (see FakeTmcPhase.h for what it models and what not).
 */
#include "FakeTmcPhase.h"

namespace FakeTmcPhase {

State &state() {
	static State s;
	return s;
}

void reset() {
	state() = State();
}

void record(const char *name, int64_t a, int64_t b) {
	Call c;
	c.name = name;
	c.a = a;
	c.b = b;
	state().calls.push_back(c);
}

size_t count(const std::string &name) {
	size_t n = 0;
	for (const Call &c : state().calls) {
		if (c.name == name) {
			++n;
		}
	}
	return n;
}

std::vector<std::string> names() {
	std::vector<std::string> out;
	for (const Call &c : state().calls) {
		out.push_back(c.name);
	}
	return out;
}

} // namespace FakeTmcPhase
