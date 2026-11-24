# WO I2-2: driver start-up and readiness (F1-T1, F1-T2) (as sent, 2026-11-24)

One `integration-test-writer` run, in parallel with the other I2 writers (I2-2, I2-3, I2-4, I2-6). **`I2_PARALLEL_COMMON.md` (same folder) is part of this order: read it first and follow it.** Repository root `/mnt/SATA_SSD1/OpenFFBoard Testing/OpenFFBoard` (space in the path: quote it), branch `add-test-suite`. File list: `wo/I2-2.files` (new file `claude/cases/test_f1_driver.cpp`).

## Order

Requirements F1-T1 and F1-T2 (charter group G-T; I2 work-order row I2-2). Firmware paths to start from (verify, cite what you use): `Axis::setDrvType` `FFBoard/Src/Axis.cpp:512-545` (create driver, encoder, `setupDriver`, then `startMotor` if USB is connected), `TMC4671::setupDriver` `FFBoard/UserExtensions/Src/TMC4671.cpp:243-254` (restoreFlash, setLimits, motion mode, thread start), `initialize` `:260-359` (0x64 = 0 at `:300`, PWM off `:301`), `initializeWithPower` `:423-424` (enable pin, then 0x1A FOC), `startMotor` `:1682-1691`, `setTorqueLimit` 0x5E `:2151`, `Axis::setPower` `Axis.cpp:498-506`.

1. **F1-T1** with `ChipEventLog` attached before the driver is selected (use the fixture's split bring-up, e.g. `reachShutdown` / `selectDriver` / `mountAndRun`, as `test_f1_axis.cpp` LateFixture does; find how a case gets the chain up to the point before `axis.0.drvtype=1`):
   - plain safety property: no 0x64 write with a non-zero torque or flux before 0x1A holds the FOC mode value (DS 0x1A) and the enable pin is high; also none while the PWM is off after a stop;
   - the order setupDriver -> limits -> power at 0x5E -> enable + FOC as the firmware does it: `characterization:` (INR) for the exact sequence; the power value at 0x5E equals `axis.0.power` set by command before the driver is selected (DS 0x5E field layout); with a power that came from flash this hits IB 17 (cite `(IB 17)`, do not re-number);
   - selecting the driver by command while USB is connected vs before USB mount (both orders reachable by the fixture): both reach Running and keep the safety property; anything else is a finding.
   - A power cycle with a stored TMC configuration ends in Shutdown without a USB replug (decisions log 2026-11-24 item 3): that defect belongs to **I2-6**; do not write a case for it here, only mention in "Findings for other WOs" if your cases show more.
2. **F1-T2**: only what the LateFixture cases of `test_f1_axis.cpp` do not prove (read them: constant force before Running, slew bound after Running, IB 24, IB 25, IB 22). With the event log: an effect started before Running gives exactly 0 at 0x64 until Running; the first non-zero torque after Running equals the effect's current torque (the value a reference run already in Running gives for the same report sequence and the same effect time) within one FFB cycle, also with a fade-in or slew setting that would otherwise shape it (cite the I1 slew / fade cases by name for what is already proven).

Name every case with `F1-T1` / `F1-T2`. Answer per the common part item 7.
