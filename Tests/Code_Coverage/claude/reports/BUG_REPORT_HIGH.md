# OpenFFBoard firmware: high severity suspected bugs

Generated 2026-11-19 from the bug table in `Tests/Code_Coverage/claude/plans/PROGRESS.md`. The full list is `BUG_REPORT.md` (unchanged).

Severity is derived from the description text (keyword rules), so it is a triage aid, not a verdict. High: Safety (motor torque, E-stop), device stops (hang, crash, watchdog, blocked port), memory errors, lost or permanent settings and flash data.

74 items: 1, 5, 11, 12, 17, 18, 24, 31, 32, 33, 35, 37, 39, 40, 41, 42, 49, 56, 57, 58, 63, 64, 65, 74, 75, 77, 79, 81, 86, 89, 96, 98, 110, 114, 115, 116, 122, 124, 125, 126, 131, 139, 153, 154, 155, 156, 166, 172, 179, 183, 189, 192, 193, 200, 205, 206, 219, 220, V4, V5, V6, V7, V13, V17, V28, V33, V57, V62, V81, V89, V108, V110, V111, V129

## Bug 1: EffectsCalculator

- **Where:** `FFBoard/Src/EffectsCalculator.cpp:1048` (`effectsForces`, `std::min<uint8_t>(cmd.adr, MAX_AXIS)`), read at line 1053 `effects_statslast[i].current[axis]`
- **What:** Clamps the axis index to `MAX_AXIS` instead of `MAX_AXIS-1`, then reads `current[MAX_AXIS]` (`std::array<int16_t,2>`) out of bounds. Silent under old libstdc++, aborts under GCC 16 `_GLIBCXX_ASSERTIONS` at -O0
- **Test / evidence:** Existing test `requesting the forces of a not existing axis is rejected` (`cases/test_effects_calculator.cpp:1062`, already `KNOWN_ISSUE_UB`). Message: `/usr/include/c++/16/array:210: ... std::array<short int, 2>::operator[] ... Assertion '__n < this->size()' failed.` -> SIGABRT after 192 of 317 cases. Excluded with `--test-case-exclude`

## Bug 5: Command execution

- **Where:** `executeCommands` (find with `grep -rn executeCommands Firmware/FFBoard/Src`)
- **What:** Dereferences the command definition before checking it for nullptr. A command with an unknown id terminates the run **Host crash only:** on the F407 `cmdDef->flags` reads address 0x0C (vector table through the flash alias), so there is no fault and the command most likely gets NOT_FOUND. Reachable only through numeric-id interfaces (CAN/HID).
- **Test / evidence:** **Confirmed in the source (P0):** `FFBoard/Src/FFBoardMainCommandThread.cpp:99-102`: `cmdDef = handler->getCommandFromId(cmd.cmdId)`, then `validFlags = cmdDef != nullptr`, then `if(cmdDef->flags & ...)` dereferences it unconditionally. Existing test `command with an unknown id is rejected` (`cases/test_command_execution.cpp:186`) stays `skip()`ped (known issue 3)

## Bug 11: HidFFB

- **Where:** `HidFFB.cpp:301-305` `set_effect`
- **What:** `set_filters` runs before `effect_p->type` is updated: a type change sets up the filters of the OLD type
- **Test / evidence:** KNOWN_ISSUE "set effect with another effect type sets up the filters of the new type". Confirmed in source

## Bug 12: HidFFB / EffectsCalculator

- **Where:** `HidFFB.cpp:301-305` + `:423`, crash at `EffectsCalculator.cpp:416/425/433`
- **What:** **Device stops:** a running effect without filters (e.g. sine) changed by Set Effect to damper/friction/inertia keeps null filters (423 skips `set_filters` for running effects); the next force calculation uses a null Biquad: SIGSEGV on the host; on the target (DSP on for all targets, `Makefile:156/158`) the vector table is read as a Biquad (`numStages` = reserved vector word 9 = 0), the CMSIS-DSP loop runs about 2^32 stages until a bus fault or watchdog reset (same mechanism as bug 124). Bug 11 (`HidFFB.cpp:301-305` ordering) leads to the same crash
- **Test / evidence:** KNOWN_ISSUE "a running effect changed to a damper by set effect has damper filters" (stops at `REQUIRE(filter[0] != nullptr)`); crash confirmed by the author with a temporary probe

## Bug 17: UART

- **Where:** `UART.cpp:212-222`
- **What:** `~UARTDevice` calls `uartport->freePort(this)` unconditionally; a default-constructed device has `uartport == nullptr` (crash). No current firmware class uses the default constructor
- **Test / evidence:** Skipped KNOWN ISSUE (crash) case "a device created without a port can be destroyed"

## Bug 18: RingBufferWrapper

- **Where:** `ringbufferwrapper.cpp:29-34` (early return in the size constructor), `45-50` (destructor)
- **What:** Size constructor returns before initialising `buffer` for a non-power-of-two size: `capacity()`/`freeSpace()` read garbage, destructor deletes an uninitialised pointer. Unreachable today (CmdParser uses 512/1024/4096)
- **Test / evidence:** Skipped KNOWN ISSUE cases only (review: the former KNOWN_ISSUE_UB read indeterminate memory in the test itself and could flip; made a `skip()` case)

## Bug 24: FFBHIDMain

- **Where:** `Inc/ffb_defs.h:135` (`HID_GamepadReport_base` without virtual destructor), owned by `std::unique_ptr<HID_GamepadReport_base> reportHID` (`UserExtensions/Inc/FFBHIDMain.h:143`)
- **What:** Deleting any FFBHIDMain main class (e.g. on main class change) is UB: ASan new-delete-type-mismatch No practical effect on the target (derived members trivially destructible, base at offset 0, `operator delete` ends in `free`).
- **Test / evidence:** KNOWN_ISSUE on the type trait (no UB executed); tests that delete a main class are marked `DELETES_FFBHIDMAIN` and skipped in sanitizer builds (decided 2026-10-23)

## Bug 31: Axis

- **Where:** `Axis.cpp:294-299` `restoreFlash`
- **What:** Stored filter profile used unchecked as index into the 4-element `filterSpeedCst`/`filterAccelCst`: out-of-bounds read (aborts under `_GLIBCXX_ASSERTIONS`)
- **Test / evidence:** Skipped KNOWN ISSUE (crash) "a stored filter profile out of range is limited to the last profile like the command does"

## Bug 32: Axis

- **Where:** `Axis.cpp:498-501, 892-894`
- **What:** `setPower` accepts any `uint16_t`; the torque is clipped to +-power and passed to `MotorDriver::turn(int16_t)`: power 40000 with full force sends -25536 (**motor reverses**)
- **Test / evidence:** KNOWN_ISSUE "the driver never receives a torque above 0x7fff or with the wrong sign when the power is above 0x7fff" ; KNOWN_ISSUE "power above 0x7fff is limited to 0x7fff" (`test_axis_config.cpp:474`)

## Bug 33: Axis

- **Where:** `Axis.cpp:917-921` with `618`
- **What:** **Safety:** after an emergency stop reset the slew limiter starts from `metric.previous.torque` (torque before the stop), not from 0: first torque after reset 9899 with slew rate 100; bypasses slew limit and fade-in. Review p2r1 confirmed the same after `usbSuspend`/`usbResume` (first torque 9899); KNOWN_ISSUE "after a USB resume the slew rate limiter starts from the zero torque of the stopped motor"
- **Test / evidence:** KNOWN_ISSUE "after an emergency stop reset the slew rate limiter starts from the zero torque of the stopped motor"

## Bug 35: Axis

- **Where:** `Axis.cpp:1332-1392`, `Axis.h:454-456` (documented intent), `:564` (unused `bool handsOff`)
- **What:** Hands-off detection computes `handsOff` but nothing reads it ("Safety override ... cut torque" never happens; unused since commit 40c65391)
- **Test / evidence:** 2 KNOWN_ISSUE (speed override, steady wheel)

## Bug 37: FFBHIDMain

- **Where:** `FFBHIDMain.cpp:250` `setReportRate`
- **What:** `clip(rateidx, 0, ffbrates.dividers.size())` allows index 4 of a 4-element array (command `hidsendspd=4`): out-of-bounds read at 256/260/262 (aborts under `_GLIBCXX_ASSERTIONS`; on target garbage dividers, possible division by 0)
- **Test / evidence:** Skipped KNOWN ISSUE (crash) "hidsendspd with an index beyond the last rate selects the last rate"

## Bug 39: FFBHIDMain

- **Where:** `FFBHIDMain.cpp:361-364` with `298`
- **What:** A new critical error calls `emergencyStop(true)`, but `true` means reset: **a critical error releases an active emergency stop** instead of triggering one. History (review p2r3): commit `8fad1a7b` (2022-06-02) introduced `emergencyStop(bool reset)` and converted this call to `true` while `exti` got `false`; `3105628c` added `control.emergency = !reset`. A regression: before `8fad1a7b` a critical error did stop
- **Test / evidence:** KNOWN_ISSUE "a critical error puts the main class into emergency stop" and "a critical error keeps an active emergency stop"

## Bug 40: FFBHIDMain

- **Where:** `FFBHIDMain.cpp:343` (`exti`, E-stop debounce)
- **What:** **Safety:** an E-stop edge within 1 s of the previous one is dropped and nothing re-reads the pin later (no polling in `updateControl`): pressing the E-stop within 1 s after a release, or within 1 s after boot (`lastEstop` starts at 0), leaves the motor running while the E-stop is held (found by review p2r3 with 2 probe tests)
- **Test / evidence:** 2 KNOWN_ISSUE: "an E-stop pressed within 1 s after a release still stops the motor", "an E-stop pressed within 1 s after start stops the motor" (`test_ffbhidmain.cpp`)

## Bug 41: flash_helpers

- **Where:** `flash_helpers.cpp:298-326` `OTP_Write`/`OTP_Read`
- **What:** Bound check `adr > FLASH_OTP_END` lets offsets 64/65 reach the OTP lock bytes (0x1FFF7A00..0x1FFF7A0F): `OTP_Write(64, x)` would permanently lock OTP blocks Also reachable on the F407VG through `sys.signature` setat: no range check (getat has one at `SystemCommands.cpp:354`) and no `CMDFLAG_DEBUG`, so `signature=x?64` locks OTP without debug mode; `cmd.adr` is truncated to `uint16_t` (`SystemCommands.cpp:75, 319, 361-362`; `SIGBNATUREBASEADR` `target_constants.h:144`). Range part of NEEDS REVIEW 55 merged here (final review FR2).
- **Test / evidence:** 3 KNOWN_ISSUE (`OTP_Write` at offsets 64 and 65, `OTP_Read` at 64) in `test_flash_helpers.cpp`; the write programs the lock bytes

## Bug 42: global_callbacks

- **Where:** `global_callbacks.cpp:365` `tud_suspend_cb()` vs TinyUSB `usbd.h:162` / `usbd.c:659` `tud_suspend_cb(bool)`
- **What:** **USB bus suspend never reaches the main class**: the firmware defines a parameterless overload, TinyUSB keeps calling its own empty weak default (`usbd.c:77`). On a bus suspend (e.g. PC sleeps with the wheel connected) `FFBHIDMain::usbSuspend` is never called: FFB not stopped, effects not reset, axes not suspended (only unmount does it). Mismatch since TinyUSB update `42bf907a` (2021-02-22). Probably safety relevant. Confirmed in source by the main agent
- **Test / evidence:** KNOWN_ISSUE "a USB bus suspend reported by TinyUSB suspends the main class" (`test_global_callbacks.cpp`, weak stand-in for the TinyUSB default)

## Bug 49: voltagesense

- **Where:** `voltagesense.cpp:78/83`
- **What:** **Safety (narrow):** `brakeActiveTime == 0` means "not braking": a brake that starts in the single millisecond where `HAL_GetTick()==0` (first ms after reset, or the ms of each 49.7-day wrap) is never shut down by the 5 s stuck-on protection (the subtraction itself is wrap-safe)
- **Test / evidence:** KNOWN_ISSUE (`test_voltagesense.cpp`)

## Bug 56: SPIButtons

- **Where:** `setSpiSpeed` / `setConfig` (`SPIButtons.cpp:139/141, 274, 115`)
- **What:** Clip to `speedPresets.size()` allows index 3 of a 3-entry array (command and stored `spi_speed`): out-of-bounds read (same class as bug 37)
- **Test / evidence:** 2 skipped KNOWN ISSUE (crash) cases

## Bug 57: SPIDevice

- **Where:** `SPI.cpp:338-340` (destructor)
- **What:** Does not clear the port's `current_device`: deleting a source while its DMA read is pending (e.g. a `btntypes` change) makes the completion call a deleted object (the fixture clears it)
- **Test / evidence:** KNOWN_ISSUE_UB "a device deleted during its transfer is not called back by the port" (`test_spi_port.cpp:1049`)

## Bug 58: I2C

- **Where:** `I2C.cpp:155-159, 169-173, 175-179, 198-212` (`_IT`/`_DMA` wrappers)
- **What:** Take the semaphore before the HAL call and never release it when the HAL returns an error: the port stays blocked (like bug 16). `transmitMasterDMA`/`receiveMasterDMA` always fail on the F407 (no DMA stream, `DMA_PARAM`)
- **Test / evidence:** KNOWN_ISSUE "an interrupt driven transfer rejected by the HAL releases the port", "a DMA transfer on a port without DMA stream releases the port" (`test_i2c_port.cpp:700, 720`)

## Bug 63: AnalogAxisProcessing

- **Where:** `AnalogAxisProcessing.cpp:174/187`
- **What:** `min`/`max` check `adr <= axisAmount`: adr == axisAmount reads/writes one past the vector (reachable by a command; aborts under `_GLIBCXX_ASSERTIONS`). Confirmed in source
- **Test / evidence:** Skipped KNOWN ISSUE (crash)

## Bug 64: AnalogAxisProcessing

- **Where:** `AnalogAxisProcessing.cpp:44-48`
- **What:** Constructor dereferences a null `cmdHandler` for `values`/`rawval` although nullptr is documented to disable commands (SIGSEGV). Latent
- **Test / evidence:** Skipped KNOWN ISSUE (crash)

## Bug 65: ADS111X

- **Where:** `ADS111X.cpp:336-341` with `:296`
- **What:** `setAxes` (`inputs`/`diff` commands) during a conversion read: `rawbuf[lastAxis]` written out of bounds (host abort; on the target into vector spare capacity)
- **Test / evidence:** Skipped KNOWN ISSUE (crash)

## Bug 74: I2C

- **Where:** `I2C.cpp:49` with `I2C.h:37`
- **What:** `I2CPort::setSpeedPreset` clips to `size()`: preset 2 reads `presets[2]` of a 2-element span (reachable via PCF8574 `speed=2`; same pattern as 37/56)
- **Test / evidence:** Skipped KNOWN ISSUE (crash) ; skipped crash "speed with a preset beyond the last selects the last preset" (`claude/cases_target/test_pcf8574.cpp:369`)

## Bug 75: SPIButtons

- **Where:** `SPIButtons.cpp:104, 110`, command `219-224`
- **What:** `btncut` set has no effect: `setConfig` always applies the mode preset's `cutRight` (was NEEDS REVIEW 79; the command exists to change it)
- **Test / evidence:** KNOWN_ISSUE "btncut set cuts the extra bits of 74xx165 buttons on the right"

## Bug 77: EncoderSSI

- **Where:** `EncoderSSI.h:73`, `EncoderSSI.cpp:43-47`
- **What:** `mode` has no initialiser and is set only from a stored `ADR_SSI_CONF1` (indeterminate without flash)
- **Test / evidence:** KNOWN_ISSUE_UB "the mode of an SSI encoder created without a stored configuration is a defined mode"

## Bug 79: EncoderLocal

- **Where:** `EncoderLocal.cpp:85`
- **What:** `% this->cpr` with cpr 0 (the default without flash) after an index hit: SIGFPE on the host; on the Cortex-M4 (no DIV_0_TRP) `x % 0` gives the dividend, the offset becomes the raw distance truncated to int16
- **Test / evidence:** Skipped (host-only crash, UB) "setting the position after the index with 0 counts per turn is safe"

## Bug 81: MtEncoderSPI

- **Where:** `MtEncoderSPI.cpp:113-114`, `301-302`
- **What:** Indices clipped to `size()`: CS index 3 (`cs=4`, `cs=0`, stored 3) dereferences `getCsPin(3)` = nullptr; speed preset 3 (command or stored) reads `spispeeds[3]` of 3 (same pattern as 37/56/74)
- **Test / evidence:** 4 skipped crash cases

## Bug 86: SPIPort

- **Where:** `SPI.cpp:89-156`
- **What:** `_IT`/`_DMA` wrappers ignore the HAL return: on HAL_ERROR or HAL_BUSY no callback follows, so CS stays asserted, the semaphore stays taken and `current_device` stays set. Fixing the return alone is not enough: the HAL leaves State BUSY_* after a failed DMA start (`stm32f4xx_hal_spi.c:1713-1716`, 1823, 1941, 1962) and `configurePort` only reinits on a configuration change
- **Test / evidence:** KNOWN_ISSUE "a DMA transfer that the HAL fails to start releases the CS pin and the port", "an interrupt driven transfer rejected by a busy HAL releases the CS pin and the port"

## Bug 89: I2CPort

- **Where:** `I2C.cpp:141-220`
- **What:** `currentDevice` is set before the semaphore is taken: a waiter using the default `startI2CTransfer` (PCF8574, `PCF8574.cpp:89-99`) replaces the pending device. Target sequence: ADS111X thread (prio 25) starts an IT transfer; pcfbtn (prio 20) sets `currentDevice` and blocks in Take; the ISR ends the PCF transfer with stale `lastData`; ADS111X is never notified (recovers after its 4 s re-init); the PCF transfer then completes to nobody and the semaphore stays taken until a PCF timeout `resetPort`. ADS111X itself takes the semaphore first (`ADS111X.cpp:19-41`)
- **Test / evidence:** KNOWN_ISSUE "the completion of a transfer reaches its device while a second device waits for the port"

## Bug 96: EncoderSSI

- **Where:** `EncoderSSI.cpp:211-216` (`190-197`)
- **What:** The `bits` command does not recompute `transferlen`: the read length stays the old one
- **Test / evidence:** KNOWN_ISSUE "changing the resolution in raw mode changes the number of bytes read"

## Bug 98: I2CPort

- **Where:** `I2C.cpp:118-121`
- **What:** `freePort` by the device of a pending IT/DMA transfer only clears `currentDevice` and gives the semaphore; the HAL transfer keeps running: the ISR writes into the buffer of the device being destroyed (`~PCF8574` with `readByteIT(&lastData)` pending; like bug 57) and the next transfer gets HAL_BUSY (then bug 58)
- **Test / evidence:** KNOWN_ISSUE "freeing the port by the device of a pending transfer stops the transfer"

## Bug 110: CANPort2B

- **Where:** `CANPort2B.cpp:207-214`, `220-221`; `CAN.h:205`
- **What:** The wait for a free mailbox never happens: the port semaphore starts available and the failure path gives it back, so `takeSemaphore` succeeds at once, AddTx fails and the send returns false without waiting for a Tx completion
- **Test / evidence:** KNOWN_ISSUE "a frame sent from a task while all mailboxes are pending waits for a free mailbox and is sent when one completes"

## Bug 114: CANInputMain

- **Where:** `CanInputMain.h:59`, constructor `CanInputMain.cpp:27-36`, used `:112-115` and in `dvals`
- **What:** `digitalBuffer` is never initialised: `dvals` returns garbage and whether the first button frame is sent depends on heap content (indeterminate value, not reported by ASan/UBSan)
- **Test / evidence:** KNOWN_ISSUE "the button state reported by dvals is 0 before the first report"

## Bug 115: CANInputMain

- **Where:** `CanInputMain.cpp:130, 148-151`
- **What:** One `CAN_tx_msg` is reused and cleared only after a send: when an unchanged group is skipped, the next shorter frame carries the skipped group's bytes
- **Test / evidence:** KNOWN_ISSUE "the unused bytes of the last analog frame are zero when the frame before it was not sent"

## Bug 116: CANInputMain

- **Where:** `CanInputMain.cpp:160-162` (also `:65-66`)
- **What:** `setReportRate` clips the index to `report_rates.size()` (7): index 7 (command or stored 3-bit value) reads `report_rates[7]` (same pattern as 37/56/74/81)
- **Test / evidence:** skipped crash "a report rate index beyond the last rate never selects a rate that does not exist"

## Bug 122: SerialFFB

- **Where:** `SerialFFB.cpp:95-97`, `:156`
- **What:** `mag` set passes the 64-bit address to `setMagnitude(uint8_t)`, which checks `idx > effects.size()`: address 256 sets effect 0, address 40 writes `effects[40]` past the array (aborts under `_GLIBCXX_ASSERTIONS`; memory corruption on the target); invalid addresses answer OK while the get answers ERR
- **Test / evidence:** KNOWN_ISSUE "mag with an address beyond 8 bit does not change effect 0"; skipped crash "mag rejects the address equal to the number of effects"

## Bug 124: SerialFFB

- **Where:** `SerialFFB.cpp:76-90`, `:114-122`; `EffectsCalculator.cpp:425/433/416`
- **What:** `setFilters` is never called (HidFFB does, `HidFFB.cpp:276/303/424`): damper and inertia call `process()` on a null filter every cycle, friction once the axis moves. Host: SIGSEGV. Target: UB that reads the vector table as a Biquad; with CMSIS-DSP numStages 0 loops about 2^32 stages (`arm_biquad_cascade_df1_f32.c:534-552`) until a BusFault/HardFault or watchdog reset. **Shipped trigger:** FFBHIDExt over CDC, `fxm.0.new=9`, `state=1?0`, `ffbstate=1`. Constant forces also unfiltered (matters only with `cffilter` below half the update rate, `EffectsCalculator.cpp:345`)
- **Test / evidence:** KNOWN_ISSUE "a damper created and started over fxm has its effect filters like the HID PID path"; skipped crash "a damper created over fxm damps the motion of the axis"

## Bug 125: SerialFFB

- **Where:** `SerialFFB.cpp:76-90`, `:180`; `ffb_defs.h:349`; `EffectsCalculator.cpp:253/276/294`
- **What:** Period 0 reaches the calculator: a new effect keeps period 0 and the `period` command stores 0 (HidFFB clips to 1..0x7fff, `HidFFB.cpp:468`). Triangle/sawtooth compute `% 0`: SIGFPE on the host; on the Cortex-M4 `x % 0 = x`, the slope is infinite and the int32 conversion saturates: **likely full-scale torque (safety)**. Shipped (FFBHIDEXT on F407VG, F407VG_DISCO, F411RE): `new=5`, `mag`, `state=1`, `ffbstate=1` without a `period` command
- **Test / evidence:** KNOWN_ISSUE "setting the period 0 never stores period 0", "a triangle created over fxm has a period of at least 1"; skipped crash "a triangle started over fxm without a period command gives a bounded torque"

## Bug 126: SerialFFB

- **Where:** `SerialFFB.cpp:82`
- **What:** `std::min(instance, MAX_AXIS)` off by one: instance 2 writes `axisMagnitudes[2]` out of bounds; other commands index by instance without a clamp. Not reachable from FFBHIDExt (instances 0, 1)
- **Test / evidence:** KNOWN_ISSUE_UB "a new effect of an effects manager with an instance beyond the axes enables the last axis"

## Bug 131: CANInputMain / targets

- **Where:** `CanInputMain.cpp:122, 149` (only `sendMessage`), `Targets/F407VG_DISCO/Core/Inc/target_constants.h:29, 31, 48` (CANINPUTMAIN and CANBUS, no CANCOMMANDS)
- **What:** `CANInputMain` never starts the CAN port (`takePort`/`start`); only the CAN command interface does, which F407VG_DISCO does not build: a fresh DISCO board sends no frame (unless a CAN config is stored: NR 119). Was NEEDS REVIEW 138 (reclassified by final review FR2)
- **Test / evidence:** code reading (the test feature set has CANCOMMANDS)

## Bug 139: TMC4671 driver: SPI transport

- **Where:** `TMC4671.cpp:2529-2540`, SPI config `:63`
- **What:** Register reads run at SCK 5.25 MHz as one 5-byte transfer; the datasheet allows reads above 2 MHz only with a 500 ns pause after the address byte (DS §4.2.1 Table 2 p18-19). The comment "500ns delay" at `:2533` is not implemented
- **Test / evidence:** KNOWN_ISSUE "[R5] a register read respects the read timing ..." (B-T1)

## Bug 153: TMC4671 driver: State machine

- **Where:** `TMC4671.cpp:313-319, 447, 467-471`
- **What:** ADC calibration failure in `initialize()` requests HardError non-forced and returns; the loop forces uninitialized again: endless re-init instead of HardError
- **Test / evidence:** KNOWN_ISSUE [R77] (B-T15)

## Bug 154: TMC4671 driver: ADC calibration

- **Where:** `TMC4671.cpp:1413-1417`
- **What:** `calibrateAdcOffset` divides by the sample count before checking it: zero accepted samples (ADC words 0, the DS reset value, or a spike-filtered ADC) divide by zero (host SIGFPE; Cortex-M4 returns 0)
- **Test / evidence:** skipped crash [R78] (B-T16), also reachable from `calibrate`

## Bug 155: TMC4671 driver: AENC calibration

- **Where:** `TMC4671.cpp:1105, 1109, 1112, 1146-1152`
- **What:** `calibrateAenc` divides by `max - min`: a constant (disconnected) channel divides by zero; the loop has no timeout
- **Test / evidence:** skipped crash [R93] (B-T17)

## Bug 156: TMC4671 driver: External encoder

- **Where:** `TMC4671.cpp:952-961, 3075-3081`
- **What:** `extphie` / `getPhiEfromExternalEncoder` divide by the cpr of the dummy encoder (0) when no external encoder is set
- **Test / evidence:** skipped crash [R94] and [R100] (B-T18)

## Bug 166: TMC4671 driver: Flash

- **Where:** `TMC4671.cpp:162-164`
- **What:** `restoreFlash` uses the uninitialised `uint16_t encofs` when the flash key is missing (index offset = stack garbage)
- **Test / evidence:** code reading + observed direction change in WO6; nondeterministic, no test

## Bug 172: ODrive driver: Node id

- **Where:** `ODriveCAN.cpp:326` (`canid`), `:69`
- **What:** `canid` is an unvalidated int8: 64, -1, 200 are stored and reported, while the bus uses `node & 0x3F` (11-bit truncation in the CAN peripheral) and the saved value is `& 0x3F`, so a reload changes the node (SPEC: node id max 0x3F)
- **Test / evidence:** KNOWN_ISSUE 4 cases [O16] (B-O4)

## Bug 179: VESC driver: Filters

- **Where:** `VescCAN.cpp:107-110`
- **What:** Changing `offbcanid`/`vesccanid` leaks the old VESC filter (`filterIdVesc = -1` before `removeCanFilter`): old ids stay accepted, one bank lost per change
- **Test / evidence:** KNOWN_ISSUE 5 cases [V2] (B-V1)

## Bug 183: VESC driver: Buffer protocol

- **Where:** `VescCAN.cpp:630, 638`
- **What:** No bound check on FILL_RX_BUFFER (offset up to 255 into a 64-byte buffer), LONG checks `ind` not `ind+len`: FW_VERSION replies longer than 64 bytes (name lengths > 35) are lost and overflow the buffer
- **Test / evidence:** skipped (memory corruption) [V24] (B-V5)

## Bug 189: VESC driver: Encoder

- **Where:** `VescCAN.cpp:152-157`
- **What:** `getEncoderType()` with `useencoder` off and no external encoder dereferences a null encoder
- **Test / evidence:** skipped crash [V63] (B-V11)

## Bug 192: VESC driver: Buffer protocol

- **Where:** `VescCAN.cpp:668-674`
- **What:** PROCESS_SHORT_BUFFER with DLC 0 or 1: `length - 2` underflows, up to 255 bytes past the 8-byte frame are decoded (out-of-bounds read)
- **Test / evidence:** KNOWN_ISSUE_UB [V26] (B-V14)

## Bug 193: VESC driver: Buffer protocol

- **Where:** `VescCAN.cpp:630, 638`
- **What:** FILL_RX_BUFFER with DLC 0 (LONG with DLC < 2): `length - 1` underflows to a ~4 GB `memcpy`
- **Test / evidence:** skipped crash [V26] (B-V15)

## Bug 200: TMCDebugBridge: `mode` command

- **Where:** `TMCDebugBridge.cpp:105`
- **What:** `cmd.val < (uint8_t) MotionMode::NONE` accepts negative `int64_t` values, `MotionMode(val)` truncates to a `uint8_t`: `mode set -1` selects mode 255 and starts the motor, `-256` and `INT64_MIN` alias to mode 0 (a silent stop); intended: rejected like 10 (help text reply, mode unchanged). Fixed underlying type: no UB
- **Test / evidence:** KNOWN_ISSUE (B-DTDC-1, B-DTDC-1b)

## Bug 205: SimpleMotion: device address

- **Where:** `MotorSimplemotion.cpp:30`
- **What:** `address(address + 1)` initialises the member from itself and ignores the `instance` argument: the address is indeterminate and both instances share it (intended `instance + 1`, 1 and 2). Tested in 0xA5-poisoned memory (wire address 0xA6 on both). Owner question 42
- **Test / evidence:** KNOWN_ISSUE_UB (B-SM-1, 2 cases)

## Bug 206: SimpleMotion: position wrap

- **Where:** `MotorSimplemotion.cpp:261-275`
- **What:** `position = value + 0xffff * overflows - 0x7fff` adds 0xffff instead of 65536 per wrap of the shifted counter (at the reports 0x7fff -> 0x8000): one count is lost per revolution (0x7ff0 -> 0x8010 gives +0x1f instead of +0x20; three revolutions give 196605 instead of 196608). The true counter wrap 0xfff0 -> 0x0010 is exact. Owner question 38
- **Test / evidence:** KNOWN_ISSUE (B-SM-2, 3 cases)

## Bug 219: MidiMain: note table

- **Where:** `MidiMain.cpp:107` (`noteToFreq[note->note]`), `MidiHandler.cpp:29-31` (no mask)
- **What:** A note number with bit 7 set (malformed packet, e.g. `09 90 FF 7F`) is stored unmasked and used as an index into the 128 entry `noteToFreq`: out of bounds read. Intended: ignored (lead decision L13). Owner question 59 territory
- **Test / evidence:** KNOWN_ISSUE_UB (B-MIDI-1)

## Bug 220: MidiMain: pitch bend

- **Where:** `MidiMain.cpp:148-155`
- **What:** A pitch bend only changes the notes sounding at that moment; a note started afterwards plays unbent (a pitch bend is channel state, MIDI 1.0 spec). Owner question 55
- **Test / evidence:** KNOWN_ISSUE (B-MIDI-2)

## Vendor V4: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `cthread.cpp:201-211` (adapter `vTaskDelete` at `:211`) and `:184-187` (`~Thread`); also `~WorkQueue` deleting its worker
- **What:** When `Run()` returns, the adapter deletes the task; the Thread destructor then calls `vTaskDelete` again on the freed handle (double delete / use after free on target). The fake ignores it (found by review r2)
- **Test / evidence:** Comment in `test_rtos_wrappers.cpp`; no test (UB)

## Vendor V5: ST EEPROM emulation (**vendor, tier A**)

- **Where:** `eeprom.c:333-356` `EE_VerifyPageFullyErased`
- **What:** Loop `while (Address <= PAGE0_END_ADDRESS)` for any page: for page 1 it never runs and reports "erased"; it also checks only every other half-word. Page 1 is never erased by `EE_Format` / the `EE_Init` RECEIVE/ERASED, VALID/ERASED branches; a VALID page 1 left over then receives writes that `Flash_Read` cannot find (silent loss). Confirmed in source by the main agent
- **Test / evidence:** KNOWN_ISSUE at the firmware layer: "a variable written after Flash_Format following a page transfer is read back" (`test_flash_helpers.cpp`); in `test_eeprom.cpp` 7 KNOWN_ISSUE tagged V5 plus 1 tagged V5+V6 (`:323, 786, 798, 811, 824, 839, 1066, 1175`, incl. "after EE_Init formatted two receiving pages ..."). Reachable on every target through the `format` command when page 1 is active

## Vendor V6: ST EEPROM emulation (**vendor, tier A**)

- **Where:** `eeprom.c:286` then `296` (and `177` then `187`) `EE_Init` recovery
- **What:** The recovery of an interrupted transfer marks the new page VALID **before** erasing the old page (`EE_PageTransfer` does the safe order: erase `715`, then VALID `723`). A power loss during that erase that leaves the old header VALID gives two VALID pages; the next `EE_Init` formats (`233`): **every variable lost**. The 177/187 branch is hidden behind V5. Confirmed in source by the main agent
- **Test / evidence:** KNOWN_ISSUE "a power loss when the EE_Init recovery starts to erase page 0 keeps every variable at its old or new value", the full-erase-cut recovery loop (to page 1) and, with V5, the recovery loop to page 0 (`eeprom.c:177/187`) (`test_eeprom.cpp`). Review p3r1 fix experiments: with V5 and V6 fixed all 10 eeprom KNOWN_ISSUE cases pass and no passing test fails

## Vendor V7: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `cthread.cpp:244-262` (`Thread::Wait`), `ccondition_variable.cpp:86-88`
- **What:** A timed-out `Wait` leaves the thread in the condition variable's wait list: the next `Signal` pops the stale entry and the real waiter misses the signal (lost wakeup); the stale thread's next `Wait` returns at once. Also on the target (`CPP_FREERTOS_CONDITION_VARIABLES` defined for the F407). Latent: no shipped firmware uses condition variables
- **Test / evidence:** KNOWN_ISSUE "a condition variable signal after a timed out wait wakes the thread that still waits" (`test_rtos_setup.cpp`); "a wait after a timed out wait and a broadcast blocks until the next signal" (`test_ccondition_variable.cpp`)

## Vendor V13: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `ctasklet.cpp:77-78`, `:87`
- **What:** The lock is given back only after `Run()`: a `Schedule()` from the tasklet's own `Run()` (or another timer callback while it is pending) blocks the timer service on itself (deadlock, all software timers stop). Separate from V12 (default timeout `portMAX_DELAY`). Latent
- **Test / evidence:** KNOWN_ISSUE "a tasklet that schedules itself again from its run does not block the timer service"

## Vendor V17: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `cmem_pool.cpp:106, 121`
- **What:** The constructor allocates the item memory before `new MutexStandard()`; when the mutex creation throws the block is never freed (leak; exception builds only: the F407 stops in configASSERT) Latent: no firmware user of `MemoryPool`.
- **Test / evidence:** KNOWN_ISSUE "a pool whose lock mutex can not be created frees its memory"

## Vendor V28: TinyUSB (**vendor, tier B**)

- **Where:** `hid_device.c:115-123`
- **What:** An oversized `tud_hid_n_report` returns false while the IN endpoint stays claimed: `tud_hid_n_ready` says true but every later report fails until a bus reset. Latent (fixed report sizes; cf. the "tinyusb will hang" comment at `HidCommandInterface.h:20`)
- **Test / evidence:** KNOWN_ISSUE "an input report is sent after a rejected report that did not fit into the endpoint buffer (V28)"

## Vendor V33: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_biquad_cascade_df1_f32.c:534-687` (portable do/while)
- **What:** numStages 0 runs about 2^32 stages through memory: host SIGSEGV, target fault or watchdog. Reached through firmware bug 124 (null Biquad)
- **Test / evidence:** skipped crash "a biquad cascade with zero stages leaves the output and the coefficient and state memory untouched" (intended: write nothing, like the NEON version)

## Vendor V57: CMSIS-DSP (**vendor, tier C**)

- **Where:** `Include/dsp/utils.h:261` (`arm_div_int64_to_int32`)
- **What:** `arm_div_int64_to_int32(INT64_MIN, -1)` executes INT64_MIN / -1: SIGFPE on x86; on the target the result saturates to the wrong sign (INT32_MIN; DOC says saturated, INT32_MAX expected); also a divide by 0 raises SIGFPE (DOC silent)
- **Test / evidence:** skipped (crash, B-DSQ31-2)

## Vendor V62: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_biquad_cascade_df1_fast_q31.c:130, :156, :182, :214, :260` and the macro `multAcc_32x32_keep32_R` (`dsp/none.h:186`, `((q63_t)a) << 32`, reached from :121, :144, :147, :248)
- **What:** Left shift of a negative accumulator (UB); also a positive int overflow at :260 with full-scale input (DOC requires input in [-0.25, +0.25))
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN (B-DFBQ-2)

## Vendor V81: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_fir_sparse_{f32,q31,q15,q7}.c` (e.g. `arm_fir_sparse_q15.c:173`, `arm_fir_sparse_q7.c:174`)
- **What:** `tapCnt = numTaps - 2U` underflows for numTaps = 1 and the kernel crashes (SIGSEGV); the DOC states no minimum numTaps
- **Test / evidence:** skipped crash cases, 4 (B-DSPAR-4)

## Vendor V89: ST HAL GPIO (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_gpio.c:294` (`HAL_GPIO_DeInit`)
- **What:** DeInit clears MODER, OSPEEDR and PUPDR of every pin to 0, also for the debug pins PA13, PA14, PA15, PB3, PB4, whose RM0090 reset values are not 0 (MODER A 0xA8000000, B 0x280; OSPEEDR A 0x0C000000, B 0xC0; PUPDR A 0x64000000, B 0x100): the pins become plain inputs and the debug port is lost until reset
- **Test / evidence:** KNOWN_ISSUE (3 cases in `test_hal_gpio.cpp`, G7)

## Vendor V108: ST HAL I2C (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_i2c.c:5548,5753-5762` (`I2C_MasterReceive_RXNE`, `I2C_MasterReceive_BTF`)
- **What:** A Size 1 `I2C_LAST_FRAME` after a same-direction `I2C_FIRST_AND_NEXT_FRAME` frame never completes: the BTF handler's else branch reads DR and decrements `XferCount` below 0 on every BTF (reads past `Size`), no `MasterRxCpltCallback`, no STOP. UM1725: LAST_FRAME "manage only the new data to transfer if no direction change and with a final stop condition"; "At reception end of current frame transfer, HAL_I2C_MasterRxCpltCallback() is executed"
- **Test / evidence:** KNOWN_ISSUE (`test_hal_i2c_seq.cpp`, IQ4)

## Vendor V110: ST HAL I2C (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_i2c.c:6486-6491` (`I2C_ITError`, "Store Last receive data if any")
- **What:** **Crash, not tested.** With `HAL_I2C_EnableListen_IT` and no own transfer armed `pBuffPtr` is NULL after `Init`; a byte waiting in DR (RXNE = 1) plus an error (for example OVR with NOSTRETCH) makes `I2C_ITError` write `*pBuffPtr = DR`: NULL pointer write (SIGSEGV on the host). A crash cannot be wrapped by `KNOWN_ISSUE`; the OVR case empties DR first to avoid it
- **Test / evidence:** no test (found by the slave writer, reproduced in gdb); report only

## Vendor V111: ST HAL CAN (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_can.c:1365,1372,1379` (`HAL_CAN_AbortTxRequest`)
- **What:** Each `SET_BIT(TSR, CAN_TSR_ABRQx)` is a read-modify-write of TSR, and RQCPx / TXOKx / ALSTx / TERRx are rc_w1 (RM0090 32.9.2: "Cleared by software writing a 1"): the write-back clears the RQCP flag the previous abort just set (and the result flags of other mailboxes). Aborting three mailboxes in one call leaves only the last mailbox's abort callback (TMEIE handler sees one RQCP); the status flags of earlier transmissions are lost
- **Test / evidence:** KNOWN_ISSUE (`test_hal_can_irq.cpp`, CQ2)

## Vendor V129: ST HAL I2C (**vendor, tier D**, source-level)

- **Where:** `stm32f4xx_hal_i2c.c:6443-6461` (`I2C_ITError`)
- **What:** With DMAEN set the function reads `hi2c->hdmatx->State` unconditionally before it looks at `hdmarx`; `HAL_I2C_Master_Receive_DMA` itself needs only `hdmarx`, so with `hdmatx == NULL` aborting or failing a running receive reads through a null pointer. Distinct from V110 (listen-mode null buffer). Found by the Codex review (P12C-T4C-003) from the source; the reproduction crashes the process (confirmed 2026-11-17 with `--no-skip`: core dump after 5 passing assertions), so the case is skipped
- **Test / evidence:** KNOWN_ISSUE, skipped (crash) (`test_hal_review_vendor_i2c.cpp`, RV21; remove the `doctest::skip()` of RV21 to see the crash)
