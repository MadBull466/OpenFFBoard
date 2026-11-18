# Flow F1: HID PID -> EffectsCalculator (CMSIS-DSP) -> Axis -> TMC4671: requirements charter

Status: **draft by the lead (Opus 5.5), 2026-11-18, waiting for the owner's approval of the plan.** Changes only by the lead, recorded in the decisions log of `Tests/Integration_Tests/claude/plans/PROGRESS.md`.

Plan: `Tests/Integration_Tests/claude/plans/INTEGRATION_TEST_PLAN.md`. Executable: `ffb_integration_tests`, cases in `Firmware/Tests/Integration_Tests/claude/cases/test_f1_*.cpp`. Each test name starts with its requirement id (`[F1-E3] ...`; for characterization `characterization: [F1-E3] ...`).

This charter lists what the **integration** must prove: behaviour that only exists when the real modules are wired together. It builds on, and does not repeat, the unit charters:

| Code | Charter | Used for |
|---|---|---|
| R n | `Tests/Code_Coverage/claude/charters/driver/TMC4671_REQUIREMENTS.md` (R1-R114, lead decisions L1-L34) | the driver's register behaviour; an F1 requirement cites the R ids whose behaviour it relies on, it never re-tests them in isolation |
| C n | `Tests/Code_Coverage/claude/charters/dsp/CMSIS_DSP_REQUIREMENTS.md` (C1-C6) | the library functions are correct in isolation; F1 proves the firmware calls them with the right parameters, rate and state |
| target tests | `Firmware/Tests/Code_Coverage/claude/cases_target/test_ffb_integration.cpp`, `test_axis_torque.cpp` (real `Axis` with `MockMotorDriver` / `MockEncoder`, report structs passed straight to `hidOut`) | F1 replaces the two mocks with the real TMC4671 on `FakeTmc4671` and the struct injection with real USB transfers built from the parsed report descriptor |

## Oracles

| Code | Source | Rule |
|---|---|---|
| PID §n | `Tests/Code_Coverage/shared/specs/USB_PID1_01.pdf` (Device Class Definition for PID 1.0/1.01). Key sections: 4.1 normalised force, 5.1 effect block (direction, gain, duration, sample period), 5.2 envelope, 5.3 condition (force = coefficient x (q - (CP offset +- dead band)), metric position / velocity / acceleration, saturation, one block per axis vs one rotated block), 5.5 periodic (offset +- magnitude, phase, period), 5.6 constant, 5.7 ramp, 5.9 effect operation (start, start solo, stop, loop count), 5.10 device gain, 5.11 block management, 7 report ids | plain test |
| DESC | The firmware's HID report descriptor (`hid_2ffb_desc` / the descriptor the wheel configuration sends), **parsed by the test** (new descriptor parser, I0) | layout, logical ranges and report ids only; the descriptor is the device's published contract to the host, so a host built from it is independent of the firmware's C structs |
| DS §n | TMC4671-LA datasheet Rev 2.07 (`TMC4671-LA_datasheet_Rev2.07.pdf`) | register layout of 0x64 (flux [15:0], torque [31:16]), 0x6B (s32 position, PhiM 16 bit = 65536 counts per revolution), 0x5E limit, 0x76 ENABLE_IN, 0x1C PHI_E_EXT |
| MATH | Mathematical definitions written out in a comment or in a stability derivation: biquad frequency response of a design (type, fc, Q, fs), backward difference, discrete loop transfer functions, Catmull-Rom / natural spline | plain test when the design parameters are stated in a source other than the code under test (spec, header constant named as a design value, user setting); otherwise `characterization:` |
| HD | Hand derivation from the firmware's documented effect formulas (scalers, internal constants), written beside the assertion | always `characterization:` + INR entry (no external spec fixes those constants) |
| PLANT | The documented nominal wheel model (`Tests/Integration_Tests/claude/charters/stability/` once written) | closed-loop expectations; fidelity Low-Medium, stated per test |
| FW | Firmware behaviour only | `characterization:` + INR entry |

Never copy a value the firmware currently produces into an assertion without `characterization:` and an INR row.

## Requirements

Priority: P1 must be proven before the phase closes, P2 should, P3 is recorded but may be left untested with a reason.

### G-A Infrastructure self-checks (I0)

| Id | Pri | The integration must prove | Oracle | Notes |
|---|---|---|---|---|
| F1-A1 | P1 | The fixture brings up the real chain: `FFBWheel` main class, real `USBdevice` + TinyUSB enumerated by the simulated host, axis 0 with driver TMC_1 on `FakeTmc4671` (CS PA4), supply present, TMC state `Running`, `motorReady()` true, within a bounded simulated time | FW sequence (R70, R72) | smoke |
| F1-A2 | P1 | The test's report-descriptor parser finds every PID report the firmware declares (ids 1-6, 0x0A-0x0D output; 0x11-0x13 feature; input 2 state) with field offsets, sizes and logical ranges, and the builder produces reports whose length equals the descriptor's | DESC + PID §7 | protects against host builder errors |
| F1-A3 | P1 | The wheel plant integrates its documented model exactly (energy check for the frictionless case, analytic decay for the viscous case) and writes 0x6B as the integer PhiM count | MATH | plant self-test |
| F1-A4 | P1 | The scheduler runs one FFB cycle per TIM13 update and the TMC thread at its own delays; one simulated ms = one FFB cycle at the default rate | FW (FFBHIDMain timer, `Delay(10)`) | documents the host-vs-target order |
| F1-A5 | P2 | The oscillation detector classifies decaying, sustained and growing synthetic signals correctly (amplitude ratio per cycle, frequency) | MATH | detector self-test |

### G-H HID entry through the real USB stack

| Id | Pri | The integration must prove | Oracle | Builds on |
|---|---|---|---|---|
| F1-H1 | P1 | Create New Effect (feature 0x11) through a control SET_REPORT followed by a Block Load GET_REPORT (0x12) returns success and an Effect Block Index 1..max that later output reports address | PID §5.11.2, DESC | target `test_ffb_integration` (struct path) |
| F1-H2 | P1 | Every output report type the descriptor declares (Set Effect, Envelope, Condition, Periodic, Constant, Ramp, Effect Operation, Block Free, Device Control, Device Gain) sent on the interrupt OUT endpoint changes the torque at 0x64 the way its PID section defines (one case per type, details in G-E) | PID §5.1-5.10 | |
| F1-H3 | P1 | Pool report (0x13) values are consistent with the number of effects the device actually accepts: creating effects until Block Load reports full matches the pool's simultaneous-effects value | PID §5.11.2 | |
| F1-H4 | P2 | The PID State input report (id 2) reflects actuators enabled / device paused / effect playing after Device Control and Effect Operation | PID §5.11.2 (PID State Report), DESC | |
| F1-H5 | P2 | A short, long or unknown-id OUT report does not change any running effect's torque and does not stall the endpoint | PID §7, DESC (report sizes) | robustness |
| F1-H6 | P1 | Reports sent while the FFB loop runs (USB thread work between two FFB cycles) take effect in the next FFB cycle, never half-applied (a Set Effect + Set Constant pair sent in one ms is seen as a pair) | FW ordering | host order fixed by the scheduler; preemption not modelled (limit) |

### G-E Effects to torque at the chip (open loop: position scripted through 0x6B)

| Id | Pri | The integration must prove | Oracle | Builds on |
|---|---|---|---|---|
| F1-E1 | P1 | Constant force: the torque field of 0x64 has the sign the PID direction convention and the axis inversion settings give, and its magnitude scales linearly with Magnitude, Effect Gain, Device Gain, `fxratio` and `power`; flux field = field-weakening value | PID §5.1, 5.6, 5.10; DS 0x64; HD for the scaling chain | R60, R62 |
| F1-E2 | P1 | Periodic (sine, square, triangle, saw up/down): 0x64 torque over time spans offset +- magnitude, with the reported period and phase, sampled at the FFB rate | PID §5.5; MATH | |
| F1-E3 | P1 | Ramp: torque moves from Ramp Start to Ramp End over Duration | PID §5.7 | |
| F1-E4 | P1 | Envelope: attack level/time and fade level/time shape the torque as PID §5.2 defines | PID §5.2 | |
| F1-E5 | P1 | Spring (metric = position from 0x6B through `Axis`): force = coefficient x (q - (CP offset +- dead band)) on each side, clipped at the saturations, restoring (opposes displacement) at the chip after all sign flips | PID §5.3; DS 0x6B; HD for the absolute scale | |
| F1-E6 | P1 | Damper (metric = velocity derived from successive 0x6B values): sign opposes motion, dead band and saturation as PID §5.3, magnitude proportional to speed in steady state | PID §5.3; MATH (steady state of the filters = DC gain 1); HD for scale | |
| F1-E7 | P1 | Inertia (metric = acceleration): opposes acceleration, proportional in steady ramp | PID §5.3; MATH; HD | |
| F1-E8 | P1 | Friction: opposes motion direction, saturates, smooth ramp-up around zero speed | PID §5.3 (friction is listed as a condition); HD for the ramp | candidate seam: `uint16_t coeff` from a negative coefficient |
| F1-E9 | P1 | Two condition blocks address X and Y separately; a single block with a direction acts along that direction (axis 0 gets cos/sin share) | PID §5.3 last paragraphs | target tests (struct path) |
| F1-E10 | P1 | Effect Operation: start, start solo (stops the others), stop; Duration ends the effect; Start Delay delays it; Loop Count repeats the operation | PID §5.9, 5.1 | candidate seam: Loop Count ignored |
| F1-E11 | P1 | Device Control: disable actuators, stop all, reset, pause / continue change the torque at the chip accordingly (pause holds effect time) | PID §5.11.2 (Device Control) | |
| F1-E12 | P1 | Combined effects: constant + spring + damper sum before the axis clip; the sum saturates at `power` without wrapping; the int32 -> int16 narrowing to `turn()` never wraps for any setting reachable by command | HD; DS 0x64 (s16) | bug 32 (power > 0x7fff) is cross-referenced, not re-numbered |
| F1-E13 | P2 | Per-effect Gain applies to every effect type the PID spec lists it for, including conditions | PID §5.1 ("Gain") | candidate seam: gain not applied to conditions (`EffectsCalculator.cpp:314`) |
| F1-E14 | P2 | An effect whose type changes by Set Effect keeps a valid filter state (no crash, no burst) | FW | bug 12 / 124 cross-reference (KNOWN_ISSUE, IB row "same as") |

### G-R Reconstruction, filters and DSP inside the chain

| Id | Pri | The integration must prove | Oracle | Builds on |
|---|---|---|---|---|
| F1-R1 | P1 | Each reconstruction mode (none, linear, Hermite, natural spline via `arm_spline_f32`) turns constant-force reports arriving at a game rate (e.g. 60 / 100 / 250 Hz, report timestamps from `micros()`) into a 1 kHz torque sequence at 0x64 that matches the mode's mathematical definition, including the documented one-interval delay of linear, the clamp to the sample range and the 50 ms hold | MATH (interpolation definitions); FW for the delay policy (characterization) | C1/C3 for `arm_spline_*`; NEEDS REVIEW 2 (float timestamps) cross-ref |
| F1-R2 | P1 | Reconstruction keeps working past 2^24 us uptime and across the `micros()` 32-bit wrap (71.6 min) | MATH | coverage NR 2 |
| F1-R3 | P1 | The condition filters (damper 30 Hz Q 0.40, inertia 15 Hz Q 0.20, friction 50 Hz Q 0.20, profile values) are designed for the actual FFB rate: the measured frequency response from 0x6B input to 0x64 output matches the biquad design at fc and 2fc within a derived tolerance, at 1 kHz and after a rate change | MATH (design values named in `EffectsCalculator.h` as defaults = design data) | C1 biquad; coverage `test_filters.cpp` |
| F1-R4 | P1 | The axis speed / accel filters (profiles 0-3) and the axis-internal damper / inertia / friction filters follow the same rule | MATH | |
| F1-R5 | P1 | Changing a filter setting (`damper_f`, `_q`, profile, FFB rate) while an effect runs produces no torque step larger than a derived bound (state reset transient), and never NaN / inf | MATH; FW | candidate seam: `calcBiquad` resets the state |
| F1-R6 | P2 | Settings that put fc at or above Nyquist (rate divider with a high profile) or Q at its limits do not produce NaN or a runaway output at the chip | MATH | candidate seam: Fc clamped to 0.5, coverage NR on Q = 0 |
| F1-R7 | P2 | The DSP build and the non-DSP build (`FFB_TESTS_USE_DSP=OFF`) give the same torque sequence within a derived float tolerance for the same report sequence | MATH | speed-limiter PID differs between the two builds (bug 158 area): cross-ref |

### G-X Axis between effects and driver

| Id | Pri | The integration must prove | Oracle | Builds on |
|---|---|---|---|---|
| F1-X1 | P1 | The axis limit and the chip limit agree: `power` sets both the axis clip and 0x5E (torque limit); the clip happens before the chip sees a value | DS 0x5E; FW | R42 |
| F1-X2 | P1 | Sign table: every combination of axis `invert`, TMC `invertForce`, encoder direction and PID direction gives a restoring spring and an opposing damper at the chip (no positive feedback in any combination reachable by command) | PID §5.3; DS 0x64 / 0x6B | R62; two independent inversions (seam) |
| F1-X3 | P1 | Position chain: a known 0x6B change gives the expected `Axis` angle, scaled position and endstop onset: one mechanical revolution (65536 PhiM counts) = 360 degrees | DS (PhiM 16 bit per rev); MATH | candidate seam: `getCpr()` returns 0xffff (360.0055 degrees per rev) |
| F1-X4 | P1 | Endstop: beyond the range the torque at the chip pushes back with the documented gain, combined with effects, clipped at `power` | HD | |
| F1-X5 | P1 | 0x6B multi-turn: the position stays continuous over many turns and across the int32 range the chip can report within a session; out-of-range handling stops the motor (`outOfBounds`) | DS 0x6B (s32); FW | |
| F1-X6 | P1 | Idle spring and axis-internal damper / inertia / friction act only in their documented states (idle spring only while FFB is off) with the measured position from the chip | FW (characterization of constants) | target `test_axis_torque` |
| F1-X7 | P1 | Slew limiter, speed limiter, fade-in and expo / equalizer shape the torque at the chip as documented and recover correctly after the driver becomes ready late | FW; HD | candidate seam: `metric.previous.torque` advances while `turn()` drops commands (driver not ready) |

### G-T TMC4671 driver inside the chain

| Id | Pri | The integration must prove | Oracle | Builds on |
|---|---|---|---|---|
| F1-T1 | P1 | Start-up order: selecting TMC_1 in the axis (`drvtype`) runs `setupDriver`, the limits, the flash restore and `setPower` -> 0x5E in that order; no torque target other than 0 reaches 0x64 before the PWM is in FOC mode and the enable pin is set | DS 0x64, 0x1A; R27-R29 | R20-R29 |
| F1-T2 | P1 | Effects started before the driver is ready produce no torque at the chip; once `Running`, the current effect torque appears within one FFB cycle (not delayed by stale limiter state) | FW | R62, R82 |
| F1-T3 | P1 | Emergency stop from the board (E-stop input -> `FFBHIDMain`) zeroes the torque, stops the chip (enable pin low) and keeps it at zero while effects keep arriving; release restores torque without a jump beyond the slew limit | FW + safety | R73; bugs 33, 39, 40 cross-ref |
| F1-T4 | P1 | Hardware enable input (0x76 bit 15 low) while effects run: the driver's emergency stop reaches the axis and the host (error, state report) | FW | R74, bug 163 cross-ref |
| F1-T5 | P1 | Undervoltage and over-temperature during a running effect stop the torque at the chip; recovery behaviour as the driver charter defines | FW | R75, R76; bugs 152, 164 cross-ref |
| F1-T6 | P1 | TMC commands sent over CDC while effects run (`invertForce`, `fluxoffset`, `trqbq_mode` / `_f`, `pwmlim`, `torqueP/I`, `seqpi`, `combineEncoder`) change the chip registers without interrupting or corrupting the torque stream (no lost or interleaved 0x64 write) | DS layouts; R100 | |
| F1-T7 | P2 | USB suspend / resume and USB unplug with effects running: torque 0 at the chip during suspend, effects resume as the firmware documents | FW | bug 42 cross-ref |
| F1-T8 | P2 | A blocking SPI failure (`HAL_ERROR` / timeout) on a 0x6B read or 0x64 write during the loop does not produce a torque jump at the chip on the next cycle | FW | R8 (untestable in the unit charter because of the uninitialised buffer: the integration checks the downstream effect where deterministic) |
| F1-T9 | P2 | With an external encoder (TIM3 quadrature, `EncoderLocal`) instead of the integrated one: the TMCENC thread writes PHI_E_EXT at the TIM6 rate, the axis uses the external position, and the sign conventions of F1-X2 still hold | DS 0x1C; R94 | |

### G-P Persistence across reboot

| Id | Pri | The integration must prove | Oracle | Builds on |
|---|---|---|---|---|
| F1-P1 | P1 | Settings saved with the board-wide save (axis, TMC, effects: power, invert, degrees, `fxratio`, filter profiles, gains, TMC motor / encoder / PI / torque filter) and restored after a simulated reboot produce the same torque at the chip for the same report sequence | FW round trip | R110-R113, coverage P3 tests |
| F1-P2 | P1 | Restore order: the restored `power` reaches 0x5E and the restored inversion settings are active before the first non-zero torque | FW | |

### G-S Timing and update rate

| Id | Pri | The integration must prove | Oracle | Builds on |
|---|---|---|---|---|
| F1-S1 | P1 | Changing the FFB rate (rate divider on TIM13) updates the sample rate of every effect and axis filter and of the speed / accel derivation (speed in deg/s stays correct at each rate) | MATH | seam 16 of the exploration |
| F1-S2 | P1 | One 0x64 write per FFB cycle at most, and none when the torque did not change (driver called only on change); the SPI frame count per cycle stays bounded | FW; DS §4.2.1 | |
| F1-S3 | P2 | Jitter: FFB cycles that arrive 1 tick late or early (injected) change the derived speed by the amount MATH predicts for a fixed-fs derivative, without NaN | MATH | documents the fixed-fs assumption |
| F1-S4 | P2 | A non-zero time per SPI frame (`setMicrosPerTransfer`, real frame ~7.6 us) does not shift the FFB cycle schedule | FW | |

### G-O Stability and oscillation (method: `INTEGRATION_TEST_PLAN.md` section 6)

| Id | Pri | The integration must prove | Oracle | Notes |
|---|---|---|---|---|
| F1-O1 | P1 | Conformance: the measured small-signal transfer function of the real firmware chain (0x6B position in -> 0x64 torque out), for damper, inertia, friction (linearised) and axis-internal paths, matches the analytic model of the stability derivation within its stated tolerance at a set of frequencies, at default and extreme settings | MATH (stability derivation) | catches wrong sample rate, wrong filter, extra delay, sign |
| F1-O2 | P1 | Closed loop with the nominal plant at the default settings: a position disturbance decays (no sustained oscillation) | PLANT + MATH | |
| F1-O3 | P1 | For each analysed setting point (sweep of damper, inertia, axisdamper, axisinertia, Q, fc, profile, power, FFB rate), the closed-loop run agrees with the analytic prediction (stable / unstable), for points at least a stated margin away from the predicted boundary | PLANT + MATH | where the prediction is "unstable inside the user-settable range" the case is an INR item (design question), not a pass |
| F1-O4 | P1 | No firmware-caused instability: parameter changes during a run, filter resets, rate changes and the friction filter freezing inside the dead band do not inject growing or sustained oscillation into an otherwise stable configuration | PLANT + MATH | candidate seams of G-R |
| F1-O5 | P2 | Friction / slew-limit nonlinearities: limit cycles predicted by the describing-function analysis appear (or not) as predicted | PLANT; describing function (Low fidelity) | `characterization:` unless the derivation is exact |

### G-B 32-bit and target differences (applies to every group)

| Id | Pri | The integration must prove | Oracle | Notes |
|---|---|---|---|---|
| F1-B1 | P1 | Every F1 case passes (or fails as its marker says) in both the 64-bit and the `FFB_TESTS_32BIT=ON` build with identical torque sequences, except where a test documents a `size_t` / `long` dependency (`KNOWN_ISSUE_32BIT`) | CLAUDE.md | |
| F1-B2 | P1 | Float-to-int conversions on the chain that can go out of range for reachable settings (condition products, `calculateFFBTorque`, `torqueScaler`) are identified; an out-of-range conversion is `KNOWN_ISSUE_UB` with the target behaviour (ARM `vcvt` saturates) written in the comment | C++ / Armv7-M ARM | L23 of the TMC charter |
| F1-B3 | P2 | Double-precision literals on the chain (`/1000.0`, `/256.0`, `/100.0`) are listed with their effect on the target (software double on the single-precision FPU: timing, not value) | Armv7-M | report item, not a test |

## Candidate seams found during planning (check `BUG_REPORT.md` and the source before recording anything)

From the read-only exploration of 2026-11-18. None is a recorded bug; each must be confirmed by a failing integration test that describes the intended behaviour, then numbered IB n (with "same as" when the coverage register already has it).

| # | Seam | Where | Related coverage item |
|---|---|---|---|
| S1 | Integrated encoder cpr returned as 0xffff, PhiM spans 65536 per revolution | `TMC4671.cpp:1932-1941`, `Encoder.cpp:67-72`, `Axis.cpp:1259-1264` (TODO) | none found |
| S2 | `metric.previous.torque` updated although `turn()` drops the command while the driver is not ready | `Axis.cpp:899-921`, `TMC4671.cpp:1745-1750` | bug 33 (similar, after E-stop) |
| S3 | Two independent force inversions (axis `invert`, TMC `invertForce` / reversed ext encoder) | `Axis.cpp:884`, `TMC4671.cpp:1752-1753` | none |
| S4 | Friction coefficient stored in `uint16_t` from a signed field | `EffectsCalculator.cpp:411` | none found |
| S5 | Constant-force filter chosen by `filter[0]` instead of `filter[axis]`; bypassed at 1 kHz (fc = 0.5) | `EffectsCalculator.cpp:344-348` | none found |
| S6 | Filter state cleared on every `setFc` / `setQ` (two resets per update) | `Filters.cpp` `calcBiquad` | NR 22 (default biquad) |
| S7 | Fc clipped to [0, 0.5]: at 0.5 `tan(pi Fc)` is huge | `Filters.cpp:102-106` | none found |
| S8 | Friction filter not updated inside the dead band (state frozen) | `EffectsCalculator.cpp:416` | none |
| S9 | Per-effect Gain not applied to condition effects | `EffectsCalculator.cpp:314` | NR 351 bundle (check) |
| S10 | Loop Count of Effect Operation not used | `HidFFB.cpp:400` | none found |
| S11 | No lock between `hidOut` (USB thread) and `calculateEffects` (FFBMAIN) | `HidFFB.cpp`, `FFBHIDMain.cpp` | not testable without preemption: limit, reported once |
| S12 | Hands-off detection computed but unused | `Axis.cpp:1332-1392` | bug 35 (cross-reference only) |
