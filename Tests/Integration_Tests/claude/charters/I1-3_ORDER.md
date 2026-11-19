# WO I1-3: constant force, ramp, envelope (F1-E1, F1-E3, F1-E4) (as sent, 2026-11-19)

One `integration-test-writer` run, in parallel with other writers. File list: `wo/I1-3.files` (`claude/cases/test_f1_effects.cpp`, new). **`I1_PARALLEL_COMMON.md` (same folder) is part of this order: read it first and follow it.**

## Order

You write the effect-to-torque tests for constant force, ramp and envelope of the OpenFFBoard integration executable `ffb_integration_tests`: a PID report sequence over the real USB stack in, the torque target at the fake TMC4671's register 0x64 out, position held at 0 (no scripted motion needed; keep 0x6B at 0 so the idle spring, endstop and conditions add nothing, and say so). Repository root `/home/ben/Documents/OpenFFBoard/testing/OpenFFBoard`, branch `add-test-suite`, WO id `I1-3`.

Spec sections: PID 1.01 4.1 (normalised force), 5.1 (Set Effect: Duration, Gain, Direction with Direction Enable, Start Delay), 5.2 (envelope), 5.6 (constant force), 5.7 (ramp), 5.10 (Device Gain); datasheet 0x64 (flux [15:0], torque [31:16]); for the flux field the TMC driver charter `Tests/Code_Coverage/claude/charters/driver/TMC4671_REQUIREMENTS.md` R60 / R62 (what `turn()` writes as flux).

### Cases

1. **F1-E1 constant force**:
   - Sign: reversing Direction by 18000 (180 degrees) reverses the torque; `axis.0.invert` toggled reverses it (state the default of `invert` you read back first; `axis.0.invert?` is 1 on a fresh flash per WO I1-1). The direction-to-sign mapping: one `characterization:` case (common rule 4).
   - Linearity, each as a ratio of steady torques with the integer rounding bound derived in a comment: Magnitude (e.g. M and M/2, and -M), Effect Gain (Set Effect Gain 255 vs 128), Device Gain (255 vs 128), `axis.0.fxratio` (two values), `axis.0.power` (two values, both below the clip). Choose values that stay far from the `power` clip and from int16 limits.
   - Flux field: the flux half of 0x64 while a constant force runs equals what R60 / R62 (and the `tmc.0.fluxoffset` setting, read back by command) say; if that needs firmware constants, `characterization:`.
   - Absolute torque count for one setting: `characterization:` with the hand derivation (HD) of the scaling chain in the comment (`EffectsCalculator` -> `Axis::updateTorque` -> `TMC4671::turn`), INR label.
   - Seam S5 (charter: constant-force filter chosen by `filter[0]`, at 1 kHz with fc = 0.5 effectively bypassed): check the filter's influence at the default FFB rate (`fx.0.filterCfFreq?` / `filterCfQ?` read back): if the steady state and the first cycles show the constant force passes unfiltered while a filter is configured, decide from the source whether that is a defect (the setting has no effect) and mark it per common rule 5; otherwise document how many cycles the steady-state rule needs.
   - Settling rule: state from the settings (filter, slew rate `axis.0.slewrate?`) how many FFB cycles after the start you treat as transient; assert the steady value over at least 20 cycles after that.
2. **F1-E3 ramp**: Ramp Start S to Ramp End E over Duration D (ms, PID 5.1 units from the descriptor): the torque at the chip moves monotonically from the value of S to the value of E (values as ratios of a constant force of the same magnitude), linear in time within the derived rounding bound at three interior points, reaches E at D within one FFB cycle, and the effect ends at D (torque back to 0). State IB 1 (1001 us FFB period) where cycle counts and ms differ.
3. **F1-E4 envelope** (PID 5.2) on a constant force of Magnitude M with Duration D: the torque starts at the Attack Level's value, reaches M's value at Attack Time, holds, and fades towards the Fade Level's value over the last Fade Time before D; linear in between (PID 5.2 says "linear" only if it does: quote the sentence you rely on; if it does not specify the shape, assert only the end points and monotonicity). One case for attack only, one for fade only, one for both with an Attack Level above M (PID allows levels above the magnitude: check the text).

### Verification

As in `I1_PARALLEL_COMMON.md` item 3 with `--wo I1-3`.
