# WO I1-5: conditions with a scripted position (F1-E5, F1-E6, F1-E7, F1-E8) (as sent, 2026-11-19)

One `integration-test-writer` run, in parallel with other writers. File list: `wo/I1-5.files` (`claude/cases/test_f1_conditions.cpp`, new). **`I1_PARALLEL_COMMON.md` (same folder) is part of this order: read it first and follow it.**

## Order

You write the condition-effect tests (spring, damper, inertia, friction) of the OpenFFBoard integration executable `ffb_integration_tests`: the position is scripted through register 0x6B of the fake TMC4671 with `PositionScript` (open loop, no plant), conditions are sent as PID reports over the real USB stack, the torque target at 0x64 is observed per FFB cycle with `TorqueTrace`. Repository root `/home/ben/Documents/OpenFFBoard/testing/OpenFFBoard`, branch `add-test-suite`, WO id `I1-5`.

Spec: PID 1.01 5.3 (Set Condition: CP Offset, Positive / Negative Coefficient, Positive / Negative Saturation, Dead Band; metrics position / velocity / acceleration; normalisation) and 5.1 (Set Effect, Gain, Direction Enable); datasheet 0x6B (s32 PhiM, 65536 counts per mechanical revolution: cite the page). The order a scripted value is seen: `PositionScript.h` (written in step k, read in the FFB cycle of step k+1).

### Cases

Common: a single condition block (Parameter Block Offset 0 = the first axis), Direction Enable off unless the case needs it, `axis.0.degrees` set explicitly (the range maps PhiM counts to the normalised position: derive q from counts and `degrees` in a comment, MATH; the mapping constant itself, e.g. `getCpr()`, is HD / seam S1, checked in I1-7: here use ratios so S1 does not matter), positions well inside the endstop range (no endstop torque: say so), effects gains `fx.0.spring` / `damper` / `inertia` / `friction` read back and stated.

1. **F1-E5 spring**: force = coefficient x (q - (CP Offset +- Dead Band)) per side (PID 5.3):
   - restoring: the torque at the chip opposes the scripted displacement from CP Offset on both sides (the sign relative to the displacement needs no polar zero: common rule 4); asymmetric coefficients give the ratio of the two sides; inside the dead band 0; linear in displacement outside it (ratio at two displacements); clipped at Positive / Negative Saturation (ratio of the saturated values = ratio of the saturations); CP Offset shifts the zero.
   - absolute scale for one setting: `characterization:` (HD), INR label.
2. **F1-E6 damper** (metric velocity from successive 0x6B values): constant-speed ramps (counts per ms) in both directions: torque opposes the motion; proportional to speed in steady state (two speeds, ratio; derive in a comment why the speed filter and the damper biquad have DC gain 1 so the steady state is the unfiltered product, MATH, and the settling time from their design frequencies); dead band and saturation on the velocity metric as PID 5.3; zero at standstill after the transient.
3. **F1-E7 inertia** (metric acceleration): a parabolic position (constant acceleration) in both signs: torque opposes the acceleration, proportional to it in steady state (two accelerations); zero during constant speed after the transient.
4. **F1-E8 friction**: torque opposes the direction of motion, saturates (independent of speed above the ramp-up speed), ramps up smoothly around zero speed as `fx.0.frictionPctSpeedToRampup` documents (read it back; the ramp shape is HD: `characterization:`); seam S4 (charter: the friction coefficient is stored in a `uint16_t` from a signed field, `EffectsCalculator.cpp:411`): a negative coefficient: PID 5.3 gives the meaning of a negative coefficient (quote it); if the firmware's result differs, mark it per common rule 5.
5. Per condition, the sign relative to the displacement / motion / acceleration is a plain test; nothing depends on the polar zero unless you use Direction Enable (then common rule 4).

### Verification

As in `I1_PARALLEL_COMMON.md` item 3 with `--wo I1-5`.
