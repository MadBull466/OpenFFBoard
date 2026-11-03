/*
 * TMC4671 driver, group G7: encoder selection, alignment and procedures (requirements R90-R94 of
 * Firmware/Tests/driver/TMC4671_REQUIREMENTS.md, lead decisions L1-L18).
 *
 * Datasheet citations "DS 0xNN" = TMC4671-LA datasheet Rev 2.07, section 7.2 register map.
 *
 * PLANTS (fidelity Low everywhere in this file). The FakeTmc4671 has no encoder decoder and no motor. Every procedure
 * test therefore scripts a plant with the fake's onRead / onWrite hooks. The plants are NOT models of the chip, they
 * are the smallest behaviour that lets the firmware procedure run to its end:
 *   AbnPlant   rotor angle follows PHI_E_EXT (0x1C) while the flux target (0x64 [15:0]) is not zero. The ABN
 *              decoder reads ABN_DECODER_PHI_E_PHI_M (0x2A) [31:16] = rotor + encoderOffset + PHI_E_OFFSET (0x29 [31:16])
 *              (assumption: the DS says only that 0x29 is an offset; additive is the assumption). The N pulse
 *              (0x76 bit 2) is static (fake setNPulse).
 *   AencPlant  same for AENC_DECODER_PHI_E_PHI_M (0x46) with PHI_E_OFFSET (0x45 [15:0]), plus a position (0x6B)
 *              that moves by a fixed step per read in the direction of the open loop speed (0x21), plus raw AENC words
 *              (0x02 selected by 0x03 = 2 and 3) that alternate between two fixed extremes on every read.
 *   Flux plant PID_TORQUE_FLUX_ACTUAL (0x69) answers the flux target (0x64 [15:0]) at once, scaled by
 *              (1 + fluxI / 10000) while the target is not zero (an overshoot that grows with the I gain).
 *   Ext plant  the MockEncoder position follows the written PHI_E_EXT: counts = phi_e * cpr / (65535 * pole_pairs).
 * Every test on a plant-driven procedure is a `characterization:` test: the expected values are derived by hand from
 * the firmware algorithm (written next to the assertion) because there is no external specification of the procedures.
 *
 * L11: every procedure is bounded to 60 s of simulated time; a procedure that does not end fails with a message.
 * Procedures use microsPerTransfer = 1000 us where the firmware busy-waits on HAL_GetTick (keeps the frame count low,
 * modelling choice; a real frame takes 7.6 us).
 * Host vs target: no assertion depends on type widths. getPhiEfromExternalEncoder uses int64 and int16 conversions,
 * identical on ARM (int16 conversion of an out-of-range value is implementation defined but modulo 2^16 on GCC/ARM).
 */
#include "doctest.h"
#include "MockCommandInterface.h"
#include "MockEncoder.h"
#include "TmcFixture.h"

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

extern TIM_HandleTypeDef htim6;

namespace {

using VT = FakeTmc4671::ViolationKind;

/** Simulated time in ms */
uint64_t nowMs() { return HostPlatform::nowUs() / 1000; }

/** Runs the driver thread in 10 ms steps until pred() holds. Fails the test (L11) after maxMs of simulated time. @return ms elapsed */
uint64_t runUntilPred(TmcFixture &f, TMC4671 &tmc, const std::function<bool()> &pred, const char *what, uint32_t maxMs = 60000) {
	const uint64_t start = nowMs();
	while (!pred()) {
		if (nowMs() - start > maxMs) {
			FAIL("bounded procedure (L11): '" << std::string(what) << "' did not finish within " << maxMs << " ms of simulated time, state "
										  << static_cast<uint32_t>(tmc.getState()));
		}
		f.runThreadFor(tmc, 10);
	}
	return nowMs() - start;
}

bool hasError(ErrorCode code) {
	for (const Error &e : ErrorHandler::getErrors()) {
		if (e.code == code && e.isError()) {
			return true;
		}
	}
	return false;
}

/** Plant of an ABN encoder, see the header comment */
struct AbnPlant {
	FakeTmc4671 &chip;
	uint16_t encoderOffset; //!< Unknown mechanical offset between the rotor and the encoder zero
	bool moves;				//!< false: the encoder does not move (broken encoder)
	uint16_t cmd = 0;
	uint16_t rotor = 0;
	AbnPlant(FakeTmc4671 &c, uint16_t offset = 0, bool moves = true) : chip(c), encoderOffset(offset), moves(moves) {
		chip.onWrite(0x1C, [this](FakeTmc4671 &, uint32_t d) { cmd = d & 0xFFFF; });
		chip.onRead(0x2A, [this](FakeTmc4671 &c) {
			if (this->moves && (c.reg(0x64) & 0xFFFF) != 0) {
				rotor = cmd;
			}
			const uint16_t phiE = static_cast<uint16_t>(rotor + encoderOffset + (c.reg(0x29) >> 16));
			c.setReg(0x2A, static_cast<uint32_t>(phiE) << 16);
		});
	}
};

/** Plant of an analog encoder (sin/cos), see the header comment */
struct AencPlant {
	FakeTmc4671 &chip;
	uint16_t cmd = 0;
	uint16_t rotor = 0;
	uint32_t readsUx = 0, readsWy = 0;
	explicit AencPlant(FakeTmc4671 &c, bool constantChannels = false) : chip(c) {
		chip.onWrite(0x1C, [this](FakeTmc4671 &, uint32_t d) { cmd = d & 0xFFFF; });
		chip.onRead(0x46, [this](FakeTmc4671 &c) {
			if ((c.reg(0x64) & 0xFFFF) != 0) {
				rotor = cmd;
			}
			c.setReg(0x46, static_cast<uint32_t>(static_cast<uint16_t>(rotor + (c.reg(0x45) & 0xFFFF))) << 16);
		});
		chip.onRead(0x6B, [](FakeTmc4671 &c) {
			const int32_t speed = static_cast<int32_t>(c.reg(0x21));
			const int32_t step = speed > 0 ? 2000 : (speed < 0 ? -2000 : 0);
			c.setReg(0x6B, static_cast<uint32_t>(static_cast<int32_t>(c.reg(0x6B)) + step));
		});
		if (!constantChannels) {
			chip.onRead(0x02, [this](FakeTmc4671 &c) {
				// ADC_RAW_ADDR 2: AENC_UX [31:16], AGPI_B [15:0]; 3: AENC_WY [31:16], AENC_VN [15:0] (DS 0x02/0x03 p77)
				if ((c.reg(0x03) & 3) == 2) {
					c.setReg(0x02, (readsUx++ % 2 ? 0x9000u : 0x7000u) << 16);
				} else if ((c.reg(0x03) & 3) == 3) {
					const uint32_t n = readsWy++;
					const uint32_t wy = n % 2 ? 0x8800u : 0x7800u;
					const uint32_t vn = n % 2 ? 0x8C00u : 0x7400u;
					c.setReg(0x02, (wy << 16) | vn);
				}
			});
		}
	}
};

/** reachShutdown, BLDC motor with 4 pole pairs (encoder procedures only run for a motor type != NONE) */
void prepare(TmcFixture &f, TMC4671 &tmc) {
	f.reachShutdown(tmc);
	tmc.setMotorType(MotorType::BLDC, 4);
}

/** Runs the thread until the phi_e selection 0x52 holds `code` and the state is Shutdown (ends of the successful alignments) */
uint64_t runUntilAligned(TmcFixture &f, TMC4671 &tmc, FakeTmc4671 &chip, uint32_t phiECode) {
	// Running is accepted too: the open loop procedures (runOpenLoop) request the motor start, so EncoderFinished goes on to Running
	return runUntilPred(f, tmc, [&] {
		return chip.reg(0x52) == phiECode && (tmc.getState() == TMC_ControlState::Shutdown || tmc.getState() == TMC_ControlState::Running);
	}, "alignment");
}


/** Plant of the external encoder: MockEncoder counts follow PHI_E_EXT, see the header comment. reversed: counts run the other way */
struct ExtPlant {
	std::shared_ptr<MockEncoder> mock = std::make_shared<MockEncoder>(10000);
	std::shared_ptr<Encoder> enc = mock;
	ExtPlant(FakeTmc4671 &chip, bool reversed = false) {
		MockEncoder *m = mock.get();
		chip.onWrite(0x1C, [m, reversed](FakeTmc4671 &, uint32_t d) {
			const double phiE = static_cast<double>(static_cast<int16_t>(d & 0xFFFF));
			const double counts = phiE * 10000.0 / (65535.0 * 4.0); // cpr 10000, 4 pole pairs
			m->setRawPosition(static_cast<int32_t>(std::lround(reversed ? -counts : counts)));
		});
	}
};

/** Raises the ENC_N flag and the flag interrupt after the open loop has been turning for `afterMs` (call once per 10 ms poll) */
struct IndexTrigger {
	TmcFixture &f;
	TMC4671 &tmc;
	uint32_t afterMs;
	uint32_t turningMs = 0;
	bool raised = false;
	void poll() {
		if (!raised && f.chip1.reg(0x21) != 0 && (f.chip1.reg(0x63) & 0xFF) != 0) {
			turningMs += 10;
			if (turningMs >= afterMs) {
				raised = true;
				f.chip1.raiseFlags(1u << 28); // DS 0x7C bit 28 ENC_N
				tmc.exti(FLAG_Pin);
			}
		}
	}
};

} // namespace

TEST_SUITE("TMC4671 encoder") {

// ---------------------------------------------------------------------------------------------------------------
// R90 encoder type selection
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R90] an unsupported encoder type becomes NONE") {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 9); // 9 is not an EncoderType_TMC (0..5)
	CHECK(CommandHelpers::get(*tmc, "encsrc") == 0);	  // 0 = NONE
	CHECK(tmc->getState() == TMC_ControlState::Shutdown);
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R90] an NONE encoder is aligned at once and the state stays Shutdown") {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 0);
	runThreadFor(*tmc, 500);
	CHECK(tmc->getState() == TMC_ControlState::Shutdown);
	tmc->startMotor();
	CHECK(runUntilState(*tmc, TMC_ControlState::Running, 1000));
	CHECK(tmc->motorReady()); // encoderAligned is true
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R90] ext before the driver allowed external encoders becomes NONE") {
	// setupDriver() (which allows the external encoder) was not called: allowExternalEncoder is false
	auto tmc = makeTmc1();
	std::shared_ptr<Encoder> enc = std::make_shared<MockEncoder>();
	tmc->setEncoder(enc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5);
	CHECK(CommandHelpers::get(*tmc, "encsrc") == 0);
	CHECK_FALSE(HostTarget::timerRunning(&htim6));
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R90] ext without an external encoder keeps the selection but is aligned like NONE and starts no timer") {
	// The charter says "ext without an external encoder becomes NONE". The firmware keeps enctype ext (5) and takes the
	// NONE branch (Shutdown, aligned): see setEncoderType else-branch. Reported as a lead decision.
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5);
	CHECK(CommandHelpers::get(*tmc, "encsrc") == 5);
	CHECK_FALSE(HostTarget::timerRunning(&htim6));
	runThreadFor(*tmc, 500);
	CHECK(tmc->getState() == TMC_ControlState::Shutdown);
	tmc->startMotor();
	CHECK(runUntilState(*tmc, TMC_ControlState::Running, 1000));
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R90] hall is trusted at once: position and velocity from phi_m_hal (12), phi_e from hall (5), Shutdown aligned") {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 4);
	// DS 0x50 p95: VELOCITY_METER_SELECTION [15:8] = 0 (mode 0), VELOCITY_SELECTION [7:0] = 12 phi_m_hal
	CHECK(chip1.reg(0x50) == 12u);
	// DS 0x51: POSITION_SELECTION 12 = phi_m_hal
	CHECK(chip1.reg(0x51) == 12u);
	// DS 0x52: PHI_E_SELECTION 5 = phi_e_hal
	CHECK(chip1.reg(0x52) == 5u);
	CHECK(tmc->getState() == TMC_ControlState::Shutdown);
	tmc->startMotor();
	CHECK(runUntilState(*tmc, TMC_ControlState::Running, 1000));
	CHECK(tmc->motorReady());
}

// ---------------------------------------------------------------------------------------------------------------
// R91 / R92 ABN alignment
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R91] ABN alignment with a following encoder ends with phi_e source abn (0x52 = 3), position and velocity from phi_m_abn (9) and a motor that can run") {
	AbnPlant plant(chip1);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	const uint64_t ms = runUntilAligned(*this, *tmc, chip1, 3);
	CHECK(ms < 60000);
	CHECK(chip1.reg(0x50) == 9u); // DS 0x50: 9 = phi_m_abn
	CHECK(chip1.reg(0x51) == 9u);
	tmc->startMotor();
	CHECK(runUntilState(*tmc, TMC_ControlState::Running, 1000));
	CHECK(tmc->motorReady());
	CHECK_FALSE(hasError(ErrorCode::encoderAlignmentFailed));
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R91] ABN alignment writes the phi_e offset that cancels the mechanical encoder offset into 0x29 [31:16]") {
	// Plant: the encoder reads rotor + 0x1234. bangInitEnc: phiEpos = 0 (0x53 = 0), offset set to 0, rotor driven to
	// phiEpos, phiE_enc = 0 + 0x1234, phiEoffset = phiEpos - phiE_enc = -0x1234 = 0xEDCC (16 bit), written to 0x29 [31:16].
	AbnPlant plant(chip1, 0x1234);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	runUntilAligned(*this, *tmc, chip1, 3);
	CHECK((chip1.reg(0x29) >> 16) == 0xEDCCu);
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R91] a manual encoder alignment (encalign) broadcasts 'Aligned successfully' with value 1") {
	AbnPlant plant(chip1);
	MockCommandInterface observer;
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	CHECK(CommandHelpers::run(*tmc, "encalign", CMDtype::get) == CommandStatus::NO_REPLY);
	observer.replies.clear();
	runUntilAligned(*this, *tmc, chip1, 3);
	const uint32_t id = tmc->getCommandFromName("encalign")->cmdId;
	bool found = false;
	for (const CommandResult &r : observer.allResults()) {
		if (r.type == CommandStatus::BROADCAST && r.originalCommand.cmdId == id) {
			REQUIRE(r.reply.size() == 1);
			CHECK(r.reply[0].reply == "Aligned successfully");
			CHECK(r.reply[0].val == 1);
			found = true;
		}
	}
	CHECK(found);
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R92] ABN alignment with an encoder that does not move ends in HardError with the alignment error after 4 attempts") {
	AbnPlant plant(chip1, 0, false);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	ErrorHandler::clearAll();
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	const size_t abnWrites0 = chip1.writesTo(0x25).size();
	// checkEncoder forces HardError at the end of each failed attempt, but the state machine takes the requested
	// state (EncoderInit) again until the retry limit (enc_retry_max = 3, i.e. the 4th failure) is exceeded.
	// The final HardError is the one that stays: wait until it has lasted 2 s.
	uint64_t hardSince = 0;
	const uint64_t ms = runUntilPred(*this, *tmc, [&] {
		if (tmc->getState() != TMC_ControlState::HardError) {
			hardSince = 0;
			return false;
		}
		if (hardSince == 0) {
			hardSince = nowMs();
		}
		return nowMs() - hardSince >= 2000;
	}, "alignment failure");
	CHECK(ms < 60000);
	CHECK(tmc->getState() == TMC_ControlState::HardError);
	CHECK(hasError(ErrorCode::encoderAlignmentFailed));
	// estimateABNparams (once per attempt) writes 0x25 (updateReg of the direction bit): 4 attempts
	CHECK(chip1.writesTo(0x25).size() - abnWrites0 == 4);
	CHECK_FALSE(tmc->motorReady());
	tmc->startMotor();
	runThreadFor(*tmc, 1000);
	CHECK(tmc->getState() == TMC_ControlState::HardError); // does not start
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R92] a manual alignment (encalign) of an encoder that does not move broadcasts 'Error during check' with value 1") {
	AbnPlant plant(chip1, 0, false);
	MockCommandInterface observer;
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	CommandHelpers::run(*tmc, "encalign", CMDtype::get);
	observer.replies.clear();
	const uint32_t id = tmc->getCommandFromName("encalign")->cmdId;
	bool found = false;
	runUntilPred(*this, *tmc, [&] {
		for (const CommandResult &r : observer.allResults()) {
			if (r.type == CommandStatus::BROADCAST && r.originalCommand.cmdId == id) {
				found = true;
				REQUIRE(r.reply.size() == 1);
				CHECK(r.reply[0].reply == "Error during check");
				CHECK(r.reply[0].val == 1);
			}
		}
		return found;
	}, "failed manual alignment");
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R91] a DC motor with an ABN encoder is aligned without an alignment movement (no phi_e offset is written)") {
	// bangInitEnc and checkEncoder return at once unless the motor is a stepper or BLDC
	AbnPlant plant(chip1, 0x1234); // an offset that bangInitEnc would have cancelled
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	tmc->setMotorType(MotorType::DC, 1);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	runUntilAligned(*this, *tmc, chip1, 3);
	CHECK((chip1.reg(0x29) >> 16) == 0u);
	tmc->startMotor();
	CHECK(runUntilState(*tmc, TMC_ControlState::Running, 1000));
	CHECK(tmc->motorReady());
}

// ---------------------------------------------------------------------------------------------------------------
// R93 procedures
// ---------------------------------------------------------------------------------------------------------------

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] full calibration with an ABN encoder broadcasts the measured polarity, stores the settings and ends aligned in Shutdown") {
	AbnPlant plant(chip1);
	chip1.setNPulse(true); // N high for the whole rotation: highcount = 88 > 44 -> npol = true (HD: p = 0, 47, ... < 0x0fff: 88 steps)
	chip1.setMicrosPerTransfer(1000);
	MockCommandInterface observer;
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	chip1.setMicrosPerTransfer(1000);
	CHECK(CommandHelpers::get(*tmc, "calibrated") == 0); // nothing stored yet
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	runUntilAligned(*this, *tmc, chip1, 3); // first the normal start-up alignment ...
	observer.replies.clear();
	CHECK(CommandHelpers::run(*tmc, "calibrate", CMDtype::get) == CommandStatus::OK);
	runUntilPred(*this, *tmc, [&] { return CommandHelpers::get(*tmc, "calibrated") == 1; }, "full calibration");
	runThreadFor(*tmc, 100);
	runUntilAligned(*this, *tmc, chip1, 3);
	const uint32_t id = tmc->getCommandFromName("abnpol")->cmdId;
	bool found = false;
	for (const CommandResult &r : observer.allResults()) {
		if (r.type == CommandStatus::BROADCAST && r.originalCommand.cmdId == id) {
			REQUIRE(r.reply.size() == 1);
			CHECK(r.reply[0].val == 1);
			found = true;
		}
	}
	CHECK(found);
	uint16_t stored = 0;
	CHECK(Flash_Read(ADR_TMC1_ADC_I0_OFS, &stored)); // saveAdcParams after the ADC calibration
	CHECK(stored == 0x7FFF);						  // ADC words 0x7FFF -> mean 0x7FFF
}

TEST_CASE_FIXTURE(TmcFixture, "[R93] the calibrated flag stays 1 after a full calibration with an ABN encoder has finished its alignment" * KNOWN_ISSUE("B-T29: FullCalibration clears recalibrationRequired (TMC4671.cpp:526) but the following encoderInit sets it again because encHallRestored is false (:1461), so `calibrated` reads 0 after a successful calibration")) {
	AbnPlant plant(chip1);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	chip1.setMicrosPerTransfer(1000);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	runUntilAligned(*this, *tmc, chip1, 3);
	CommandHelpers::run(*tmc, "calibrate", CMDtype::get);
	runUntilPred(*this, *tmc, [&] { return CommandHelpers::get(*tmc, "calibrated") == 1; }, "full calibration");
	runUntilPred(*this, *tmc, [&] { return CommandHelpers::get(*tmc, "calibrated") == 0 || (chip1.reg(0x52) == 3 && tmc->getState() == TMC_ControlState::Shutdown); }, "alignment after calibration");
	runUntilAligned(*this, *tmc, chip1, 3);
	CHECK(CommandHelpers::get(*tmc, "calibrated") == 1);
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] analog (sin/cos) encoder start-up calibrates the AENC offsets and scales and aligns") {
	// calibrateAenc is entered from encoderInit (analog_enc_skip_cal is 0 for the default hardware config).
	// Plant: UX alternates 0x7000 / 0x9000 -> offset (0x9000 + 0x7000) / 2 = 0x8000, scale 0xF6FF00 / (0x9000 - 0x7000) =
	//        16187136 / 8192 = 1975 (integer division, 0x07B7) -> 0x0D = 0x8000 | 0x07B7 << 16
	// WY alternates 0x7800 / 0x8800 -> offset 0x8000, scale 16187136 / 4096 = 3951 (0x0F6F) -> 0x0F = 0x8000 | 0x0F6F << 16
	// (channel VN/AENC1 is only calibrated for uvw: 0x0E keeps the default written by setup_AENC for sin/cos)
	chip1.expectViolation(VT::WriteToReadOnly, 0x41); // charter Q1 / B-T22: the firmware zeroes AENC_DECODER_COUNT
	AencPlant plant(chip1);
	chip1.setMicrosPerTransfer(1000);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	chip1.setMicrosPerTransfer(1000);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 2);
	runUntilAligned(*this, *tmc, chip1, 6); // 6 = phi_e_aenc
	CHECK(chip1.reg(0x0D) == (0x8000u | (0x07B7u << 16)));
	CHECK(chip1.reg(0x0F) == (0x8000u | (0x0F6Fu << 16)));
	CHECK(chip1.reg(0x51) == 11u); // DS 0x51: 11 = phi_m_aenc
	tmc->startMotor();
	CHECK(runUntilState(*tmc, TMC_ControlState::Running, 1000));
}

TEST_CASE_FIXTURE(TmcFixture, "[R93] analog encoder calibration with a constant AENC channel does not divide by zero" * doctest::skip() *
									doctest::description("KNOWN ISSUE (crash): B-T17: calibrateAenc divides 0xF6FF00 by (max - min) of a channel without checking for 0 (TMC4671.cpp:1105). Skipped because it terminates the test run (SIGFPE)")) {
	// Intended behaviour: a disconnected encoder (constant raw words) must not crash the firmware. The body is the reproduction:
	// the position plant moves, the raw words stay at the fake's default 0, so max == min in the first scale computation.
	chip1.expectViolation(VT::WriteToReadOnly, 0x41);
	AencPlant plant(chip1, true);
	chip1.setMicrosPerTransfer(1000);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	chip1.setMicrosPerTransfer(1000);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 2);
	runThreadFor(*tmc, 30000);
	CHECK(tmc->getState() != TMC_ControlState::HardError); // reaching this line at all is the intended behaviour
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] a failing ADC calibration in the full calibration is retried once and then ends in HardError with the calibration error") {
	// ADC words: I0 = 0x7FFF + 6000 = 32767 + 6000 = 38767 = 0x976F (within the 10000 spike filter of the stored offset 0x7FFF), I1 = 0x7FFF.
	// Mean offset error 6000 > TMC_ADCOFFSETFAIL 5000 -> adcCalibrationError. calibrationFailCount = 1: one retry (calibFailCb).
	AbnPlant plant(chip1);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	chip1.setMicrosPerTransfer(1000);
	ErrorHandler::clearAll();
	chip1.setAdcRaw(0, 0x7FFF976F);
	CommandHelpers::run(*tmc, "calibrate", CMDtype::get);
	runUntilPred(*this, *tmc, [&] { return tmc->getState() == TMC_ControlState::HardError; }, "failing full calibration");
	CHECK(hasError(ErrorCode::tmcCalibFail));
	CHECK(hasError(ErrorCode::adcCalibrationError));
	CHECK(CommandHelpers::get(*tmc, "calibrated") == 0);
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] full calibration with a sin/cos encoder runs the analog calibration and ends aligned") {
	chip1.expectViolation(VT::WriteToReadOnly, 0x41);
	AencPlant plant(chip1);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	chip1.setMicrosPerTransfer(1000);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 2);
	CommandHelpers::run(*tmc, "calibrate", CMDtype::get);
	runUntilPred(*this, *tmc, [&] { return CommandHelpers::get(*tmc, "calibrated") == 1; }, "full calibration");
	runUntilAligned(*this, *tmc, chip1, 6);
	CHECK(chip1.reg(0x0D) == (0x8000u | (0x07B7u << 16))); // same plant as the start-up test: HD there
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] analog UVW encoder start-up also calibrates the third channel (AENC1 from VN) and selects uvw mode") {
	// VN alternates 0x7400 / 0x8C00: offset (0x8C00 + 0x7400) / 2 = 0x8000, span 0x1800 = 6144,
	// scale 16187136 / 6144 = 2634 (integer, 2634.75) = 0x0A4A -> 0x0E = 0x8000 | 0x0A4A << 16. 0x3B bit 0 = 1 (uvw), DS 0x3B.
	chip1.expectViolation(VT::WriteToReadOnly, 0x41);
	AencPlant plant(chip1);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	chip1.setMicrosPerTransfer(1000);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 3);
	runUntilAligned(*this, *tmc, chip1, 6);
	CHECK(chip1.reg(0x0E) == (0x8000u | (0x0A4Au << 16)));
	CHECK((chip1.reg(0x3B) & 1u) == 1u);
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] ABN start-up with the index enabled searches the index first and then reports an incremental encoder with index") {
	AbnPlant plant(chip1);
	REQUIRE(Flash_Write(ADR_TMC1_ENC_OFFSET, static_cast<uint16_t>(0))); // B-T28: restoreFlash reads an uninitialised encofs when the key is missing
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	chip1.setMicrosPerTransfer(1000);
	CommandHelpers::run(*tmc, "abnindex", CMDtype::set, 1);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	CHECK(tmc->getEncoderType() == EncoderType::incremental); // index not hit yet
	IndexTrigger index{*this, *tmc, 300};
	runUntilPred(*this, *tmc, [&] {
		index.poll();
		// runOpenLoop of the index search requested the motor start: EncoderFinished goes on to Running
		return chip1.reg(0x52) == 3 && (tmc->getState() == TMC_ControlState::Shutdown || tmc->getState() == TMC_ControlState::Running);
	}, "alignment with index");
	CHECK(index.raised);
	CHECK(tmc->getEncoderType() == EncoderType::incrementalIndex);
	CHECK_FALSE(hasError(ErrorCode::encoderIndexMissed));
	CHECK_FALSE(hasError(ErrorCode::encoderAlignmentFailed));
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] index search finds the index, stops the open loop and raises no error") {
	// autohome -> findEncoderIndex(speed = -10 because posOffsetFromIndex = 0 is not < 0, power = 5000 / 2, ...).
	// The plant raises ENC_N (0x7C bit 28) and calls the flag interrupt once the open loop has been turning for 300 ms.
	AbnPlant plant(chip1);
	// restoreFlash() reads the stored index offset into an UNINITIALISED local when the key is missing
	// (TMC4671.cpp:157-159, suspected defect, lead decision): store 0 so the test does not depend on stack garbage.
	REQUIRE(Flash_Write(ADR_TMC1_ENC_OFFSET, static_cast<uint16_t>(0))); // B-T28: restoreFlash reads an uninitialised encofs when the key is missing
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	ErrorHandler::clearAll();
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	CHECK(CommandHelpers::run(*tmc, "autohome", CMDtype::get) == CommandStatus::OK);
	IndexTrigger index{*this, *tmc, 300};
	runUntilPred(*this, *tmc, [&] {
		index.poll();
		return index.raised && chip1.reg(0x21) == 0;
	}, "index search");
	CHECK_FALSE(hasError(ErrorCode::encoderIndexMissed));
	CHECK(chip1.writesTo(0x21).front() == 0xFFFFFFF6u); // HD: speed -10 as 32 bit two's complement
	CHECK(chip1.writesTo(0x21).back() == 0u);
	CHECK((chip1.reg(0x64) & 0xFFFF) == 0u); // flux ramped down
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] index search without an index pulse ends after the 10 s timeout with the index-missed error") {
	AbnPlant plant(chip1);
	REQUIRE(Flash_Write(ADR_TMC1_ENC_OFFSET, static_cast<uint16_t>(0))); // B-T28: restoreFlash reads an uninitialised encofs when the key is missing
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	ErrorHandler::clearAll();
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 1);
	CommandHelpers::run(*tmc, "autohome", CMDtype::get);
	const uint64_t ms = runUntilPred(*this, *tmc, [&] { return hasError(ErrorCode::encoderIndexMissed); }, "index search timeout");
	CHECK(ms >= 10000); // HD: 1000 polls of Delay(10) = 10 s plus the 500 ms flux ramp
	CHECK(ms <= 12000);
	runThreadFor(*tmc, 500);
	CHECK(chip1.reg(0x21) == 0u); // open loop stopped again
}

// PID autotune. Flux plant: see the header comment. Derivation of the result for the plant with overshoot = I / 10000:
//   targetflux = min(pid_torque_flux 30000, bangInitPower 5000) = 5000, targetflux_p = 5000 * 0.75 = 3750
//   P loop: first read answers 5000 > 3750 -> break with fluxP = 100.
//   I loop (fluxI starts at 100, step_i 10 after the first pass because peak >= 0.95 * target): the peak is
//   5000 + 5000 * I / 10000; the cut-off is 5000 + 5000 * 0.04 = 5200 -> exceeded first for I = 410 (peak 5205);
//   fluxI = 410 - 10 = 400. newpids: flux/torque I = 400, P = 100.
struct FluxPlant {
	FluxPlant(FakeTmc4671 &chip, bool responds) {
		chip.onRead(0x69, [responds](FakeTmc4671 &c) {
			const int32_t target = static_cast<int16_t>(c.reg(0x64) & 0xFFFF);
			const int32_t i = c.reg(0x54) & 0xFFFF;
			const int32_t flux = (!responds || target == 0) ? 0 : target + target * i / 10000;
			c.setReg(0x69, static_cast<uint32_t>(flux) & 0xFFFF);
		});
	}
};

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] PID autotune with a plant that overshoots from I = 410 sets flux and torque PI to I = 400, P = 100 and broadcasts success") {
	FluxPlant plant(chip1, true);
	MockCommandInterface observer;
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	chip1.setMicrosPerTransfer(1000);
	observer.replies.clear();
	CHECK(CommandHelpers::run(*tmc, "pidautotune", CMDtype::get) == CommandStatus::NO_REPLY);
	const uint32_t id = tmc->getCommandFromName("pidautotune")->cmdId;
	bool found = false;
	runUntilPred(*this, *tmc, [&] {
		for (const CommandResult &r : observer.allResults()) {
			if (r.type == CommandStatus::BROADCAST && r.originalCommand.cmdId == id) {
				found = true;
				REQUIRE(r.reply.size() == 1);
				CHECK(r.reply[0].reply == "PID Autotune success");
				CHECK(r.reply[0].val == 1);
			}
		}
		return found;
	}, "pid autotune");
	// DS 0x54 / 0x56: I [15:0], P [31:16]
	CHECK(chip1.reg(0x54) == (400u | (100u << 16)));
	CHECK(chip1.reg(0x56) == (400u | (100u << 16)));
	runUntilState(*tmc, TMC_ControlState::Shutdown, 2000);
	CHECK(tmc->getState() == TMC_ControlState::Shutdown); // returns to the previous state
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] PID autotune with a plant that never produces flux gives up after both loops and broadcasts failure") {
	FluxPlant plant(chip1, false);
	MockCommandInterface observer;
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	chip1.setMicrosPerTransfer(1000);
	observer.replies.clear();
	CommandHelpers::run(*tmc, "pidautotune", CMDtype::get);
	const uint32_t id = tmc->getCommandFromName("pidautotune")->cmdId;
	bool found = false;
	runUntilPred(*this, *tmc, [&] {
		for (const CommandResult &r : observer.allResults()) {
			if (r.type == CommandStatus::BROADCAST && r.originalCommand.cmdId == id) {
				found = true;
				REQUIRE(r.reply.size() == 1);
				CHECK(r.reply[0].reply == "PID Autotune failed");
				CHECK(r.reply[0].val == 0);
			}
		}
		return found;
	}, "pid autotune failure");
}

// ---------------------------------------------------------------------------------------------------------------
// R94 external encoder
// ---------------------------------------------------------------------------------------------------------------

namespace {
/** Driver with an external encoder selected (alignment not run: the thread is not activated after setEncoderType) */
struct ExtSetup {
	std::shared_ptr<MockEncoder> mock = std::make_shared<MockEncoder>(10000);
	std::shared_ptr<Encoder> enc = mock;
};
} // namespace

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R94] selecting ext with an external encoder starts TIM6 with the update interrupt and ARR 250") {
	ExtSetup ext;
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	tmc->setEncoder(ext.enc);
	CHECK_FALSE(HostTarget::timerRunning(&htim6));
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5);
	CHECK(HostTarget::timerRunning(&htim6));
	CHECK(HostTarget::timerUpdateIrqEnabled(&htim6));
	CHECK(htim6.Instance->ARR == 250u); // TIM_TMC_ARR (4 kHz)
}

/** Selects ext, sets the phi_e source to the external encoder and delivers one TIM6 update with the async write completed */
static void deliverTimerUpdate(FakeTmc4671 &chip, TMC4671 &tmc) {
	(void)chip;
	REQUIRE(HostTarget::timerUpdate(&htim6));
	HostRtos::runUntilIdle(); // thread TMCENC: WaitForNotification -> writeRegAsync
	REQUIRE(HostTarget::spiTransferPending(&hspi1));
	REQUIRE(HostTarget::spiComplete(&hspi1));
	(void)tmc;
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R94] a TIM6 update writes PHI_E_EXT = (pos * 65535 / cpr * pole_pairs) & 0xFFFF for a position that gives a positive 16 bit value") {
	ExtSetup ext;
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	tmc->setEncoder(ext.enc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5);
	tmc->setPhiEtype(PhiE::extEncoder);
	// HD: pos 1000, cpr 10000, pole pairs 4: 1000 * 65535 = 65535000; / 10000 = 6553 (6553.5 truncated, int64);
	//     6553 * 4 = 26212 = 0x6664; & 0xFFFF = 0x6664; offset 0
	ext.mock->setRawPosition(1000);
	chip1.clearLog();
	deliverTimerUpdate(chip1, *tmc);
	REQUIRE(chip1.writesTo(0x1C).size() == 1);
	CHECK(chip1.writesTo(0x1C).front() == 0x6664u);
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R94] a TIM6 update writes the low 16 bits 0xFFFC of PHI_E_EXT for position 2500") {
	ExtSetup ext;
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	tmc->setEncoder(ext.enc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5);
	tmc->setPhiEtype(PhiE::extEncoder);
	// HD: 2500 * 65535 = 163837500; / 10000 = 16383 (16383.75 truncated); * 4 = 65532 = 0xFFFC; DS 0x1C: s16(15:0) = -4
	ext.mock->setRawPosition(2500);
	deliverTimerUpdate(chip1, *tmc);
	REQUIRE(chip1.writesTo(0x1C).size() == 1);
	CHECK((chip1.writesTo(0x1C).front() & 0xFFFFu) == 0xFFFCu);
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R94] the upper 16 bits of the PHI_E_EXT word are the sign extension of the int16 angle (0xFFFF for a negative angle)") {
	// getPhiEfromExternalEncoder returns int16_t which writeRegAsync takes as uint32_t (inventory 9.17): -4 -> 0xFFFFFFFC.
	// The DS only defines bits 15:0, so the effect on the chip is undocumented; recorded, not asserted as a requirement.
	ExtSetup ext;
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	tmc->setEncoder(ext.enc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5);
	tmc->setPhiEtype(PhiE::extEncoder);
	ext.mock->setRawPosition(2500);
	deliverTimerUpdate(chip1, *tmc);
	REQUIRE(chip1.writesTo(0x1C).size() == 1);
	CHECK(chip1.writesTo(0x1C).front() == 0xFFFFFFFCu);
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R94] a TIM6 update writes nothing while the phi_e source is not the external encoder") {
	ExtSetup ext;
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	tmc->setEncoder(ext.enc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5); // source stays phi_e_ext (alignment not run)
	HostTarget::timerUpdate(&htim6);
	HostRtos::runUntilIdle();
	CHECK_FALSE(HostTarget::spiTransferPending(&hspi1));
	CHECK(chip1.writesTo(0x1C).empty());
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R94] ext alignment with a following external encoder ends aligned with phi_e from the external encoder (0x52 = 1)") {
	// Plant: ExtPlant (fidelity Low).
	// bangInitEnc of the ext type reads/writes register 0 (phiEoffsetReg = 0 is never set for ext, TMC4671.cpp:977/1007):
	// a write to the read-only CHIPINFO_DATA, reported as a new suspected defect.
	chip1.expectViolation(VT::WriteToReadOnly, 0); // B-T27
	ExtPlant ext(chip1);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	tmc->setEncoder(ext.enc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5);
	runUntilPred(*this, *tmc, [&] { return chip1.reg(0x52) == 1 && tmc->getState() == TMC_ControlState::Shutdown && tmc->getPhiEtype() == PhiE::extEncoder; }, "ext alignment");
	CHECK_FALSE(hasError(ErrorCode::encoderAlignmentFailed));
	CHECK_FALSE(hasError(ErrorCode::encoderReversed));
	tmc->startMotor();
	CHECK(runUntilState(*tmc, TMC_ControlState::Running, 1000));
	CHECK(tmc->motorReady());
}

TEST_CASE_FIXTURE(TmcFixture, "[R94] external encoder alignment writes no read-only register" * KNOWN_ISSUE("B-T27: bangInitEnc leaves phiEoffsetReg = 0 for the ext encoder and does updateReg(0, ...), a write to the read-only CHIPINFO_DATA (TMC4671.cpp:977-1007)")) {
	chip1.expectViolation(VT::WriteToReadOnly, 0); // only so the fixture teardown does not report it a second time
	ExtPlant ext(chip1);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	tmc->setEncoder(ext.enc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5);
	runUntilPred(*this, *tmc, [&] { return chip1.reg(0x52) == 1 && tmc->getState() == TMC_ControlState::Shutdown && tmc->getPhiEtype() == PhiE::extEncoder; }, "ext alignment");
	size_t writesToReg0 = 0;
	for (const auto &v : chip1.violations()) {
		if (v.kind == VT::WriteToReadOnly && v.addr == 0) {
			writesToReg0++;
		}
	}
	CHECK(writesToReg0 == 0); // first assertion: the defect
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R94] ext alignment with an encoder that counts the wrong way raises the encoder-reversed warning and still aligns") {
	// checkEncoder: phiE_enc = -angle for every angle -> err = 2 * angle > nErr = 0 -> revCount counts every step
	// (63 forward + more than 50 in total, HD: angles 0x00ff * k < 0x3fff, k = 1..63) -> revCount > 50 -> direction flipped.
	chip1.expectViolation(VT::WriteToReadOnly, 0); // B-T27
	ExtPlant ext(chip1, true);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	ErrorHandler::clearAll();
	tmc->setEncoder(ext.enc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5);
	runUntilPred(*this, *tmc, [&] { return chip1.reg(0x52) == 1 && tmc->getState() == TMC_ControlState::Shutdown && tmc->getPhiEtype() == PhiE::extEncoder; }, "ext alignment");
	CHECK(hasError(ErrorCode::encoderReversed));
	CHECK_FALSE(hasError(ErrorCode::encoderAlignmentFailed));
}

TEST_CASE_FIXTURE(TmcFixture, "characterization: [R93] full calibration with an external encoder that counts the wrong way detects the direction first, so the alignment raises no reversed warning") {
	// estimateExtEnc: phiE_enc decreases at (nearly) every one of the 88 steps -> rcount > c / 2 -> encoderReversed toggled
	// before the alignment; the alignment then sees a consistent encoder.
	chip1.expectViolation(VT::WriteToReadOnly, 0); // B-T27
	ExtPlant ext(chip1, true);
	auto tmc = makeTmc1();
	prepare(*this, *tmc);
	chip1.setMicrosPerTransfer(1000);
	ErrorHandler::clearAll();
	tmc->setEncoder(ext.enc);
	CommandHelpers::run(*tmc, "encsrc", CMDtype::set, 5);
	CommandHelpers::run(*tmc, "calibrate", CMDtype::get); // FullCalibration replaces the pending ExternalEncoderInit
	runUntilPred(*this, *tmc, [&] { return CommandHelpers::get(*tmc, "calibrated") == 1; }, "full calibration");
	runUntilPred(*this, *tmc, [&] { return chip1.reg(0x52) == 1 && tmc->getState() == TMC_ControlState::Shutdown && tmc->getPhiEtype() == PhiE::extEncoder; }, "ext alignment after calibration");
	CHECK_FALSE(hasError(ErrorCode::encoderReversed));
	CHECK_FALSE(hasError(ErrorCode::encoderAlignmentFailed));
}

TEST_CASE_FIXTURE(TmcFixture, "[R94] reading extphie without an external encoder does not divide by zero" * doctest::skip() *
									doctest::description("KNOWN ISSUE (crash): B-T18: getPhiEfromExternalEncoder divides by the cpr of the dummy encoder (0) when no external encoder is set (TMC4671.cpp:955, command extphie has no guard). Skipped because it terminates the test run (SIGFPE)")) {
	auto tmc = makeTmc1();
	reachShutdown(*tmc);
	std::vector<CommandReply> replies;
	CHECK(CommandHelpers::run(*tmc, "extphie", CMDtype::get, 0, &replies) == CommandStatus::OK);
	CHECK(replies.size() == 1);
}

} // TEST_SUITE
