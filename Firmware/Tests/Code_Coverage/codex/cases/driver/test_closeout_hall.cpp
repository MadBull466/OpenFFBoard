#include "TmcFixture.h"

TEST_SUITE("CodexHallCloseout") {
TEST_CASE_FIXTURE(TmcFixture, "public Hall setup packs six distinct signed nondefault angles") {
	auto tmc = makeTmc1();
	TMC4671HALLConf c;
	c.pos0 = -123;
	c.pos60 = 2345;
	c.pos120 = -3000;
	c.pos180 = 4095;
	c.pos240 = 12345;
	c.pos300 = -23456;
	tmc->setup_HALL(c);
	// Supplied Rev2.07 section 7.2 p89: paired signed 16-bit low/high fields.
	// -123 = 65536-123 = FF85; 2345 = 0929.
	// -3000 = F448; 4095 = 0FFF; 12345 = 3039; -23456 = A460.
	CHECK(chip1.reg(0x34) == 0x0929FF85u);
	CHECK(chip1.reg(0x35) == 0x0FFFF448u);
	CHECK(chip1.reg(0x36) == 0xA4603039u);
	REQUIRE_ALL_HAL_MODELLED();
}
}
