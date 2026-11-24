# OpenFFBoard firmware: low severity suspected bugs

Generated 2026-11-19 from the bug table in `Tests/Code_Coverage/claude/plans/PROGRESS.md`. The full list is `BUG_REPORT.md` (unchanged).

Severity is derived from the description text (keyword rules), so it is a triage aid, not a verdict. Low: Latent, host-only, cosmetic or rarely reached behaviour.

183 items: 7, 8, 9, 10, 14, 15, 16, 19, 21, 23, 25, 26, 27, 28, 29, 30, 44, 46, 47, 48, 50, 52, 53, 54, 57 (ext.), 60, 62, 66, 67, 68, 69, 71, 73, 76, 80, 82, 83, 85, 87, 88, 91, 92, 93, 94, 97, 100, 104, 105, 106, 107, 108, 112, 113, 117, 118, 121, 123, 128, 132, 133, 134, 137, 138, 140, 141, 143, 146, 147, 148, 149, 151, 157, 158, 160, 161, 163, 164, 165, 167, 168, 169, 171, 173, 174, 175, 177, 178, 180, 181, 182, 184, 185, 187, 188, 195, 198, 199, 201, 207, 208, 210, 214, 215, 216, 217, 218, 221, 224, 225, V1, V3, V8, V9, V11, V12, V21, V22, V24, V29, V30, V31, V34, V35, V36, V40, V41, V42, V43, V44, V45, V48, V49, V52, V53, V55, V56, V58, V59, V60, V61, V66, V67, V68, V69, V70, V72, V73, V75, V76, V77, V78, V79, V80, V82, V86, V87, V88, V90, V91, V92, V95, V96, V97, V99, V101, V102, V103, V104, V105, V106, V107, V112, V113, V115, V116, V119, V120, V121, V122, V123, V124, V125, V128

## Bug 7: ErrorHandler

- **Where:** `ErrorHandler.cpp:130-143` (`getErrors` + `sortErrors`)
- **What:** Empty slots (code 0) sort to the front; `getErrors()` stops at the first empty slot, so after any `clearError`/`clearTemp` it returns an empty list while errors are still stored. **Visible via the `errors` command after every read** (P4: `replyErrors` ends with `clearTemp()` -> `sortErrors()`, so each `errors` read or error-printer pass hides persistent errors from the next read)
- **Test / evidence:** KNOWN_ISSUE "the other errors stay listed after one error is cleared" (`test_error_handler.cpp`) and "a persistent error is still listed by a second errors command" (`test_system_commands.cpp`). Confirmed in source

## Bug 8: ErrorHandler

- **Where:** `ErrorHandler.cpp:118-124` `clearError(ErrorCode)`
- **What:** The slot is emptied before the callbacks, which receive `errors[i]` = an empty Error instead of the cleared one
- **Test / evidence:** KNOWN_ISSUE "clearing by code tells the handlers which error was cleared". Confirmed in source

## Bug 9: ErrorHandler

- **Where:** `ErrorHandler.cpp:66-72` `clearTemp`
- **What:** `break` after the first temporary error: only one is cleared per call; comment says "Clears errors marked as temporary"
- **Test / evidence:** KNOWN_ISSUE "clearTemp removes all temporary errors". Confirmed in source

## Bug 10: EffectsCalculator

- **Where:** `EffectsCalculator.cpp:235` (square wave)
- **What:** Raw phase (0.01 deg) added as milliseconds instead of `phase*period/36000` like sine/triangle; 90 deg on a 1000 ms period shifts by 984 ms instead of 250 ms
- **Test / evidence:** KNOWN_ISSUE "square phase is a fraction of the period like for the other periodic effects". Confirmed in source

## Bug 14: Filters

- **Where:** `Filters.cpp:44/86` + `:102`, reachable via `EffectsCalculator.cpp:868-872`
- **What:** Clip allows Fc = 0.5 (comment: "Must be lower than 0.5"); with CMSIS-DSP `arm_cos_f32(PI/2)` is exactly 0, K = inf, NaN coefficients, NaN output. DSP off: finite but a double pole on the unit circle. `checkFilterCoeff` maps a stored freq of 0 to Fc = 0.5
- **Test / evidence:** KNOWN_ISSUE in the DSP build only "lowpass at the highest allowed frequency 0.5 ...". Target result may differ (FMA) ; KNOWN_ISSUE (DSP builds) "lowpass at the highest allowed frequency 0.5 has finite coefficients and passes a constant input" (`claude/cases/test_filters.cpp:613`, `NYQUIST_ISSUE`)

## Bug 15: Filters (latent)

- **Where:** `Inc/Filters.h`, `Filters.cpp:22/30`
- **What:** In the DSP build the copied `arm_biquad_casd_df1_inst_f32` still points into the original's coefficients/state: a copy filters with the original's data and corrupts its state. No current caller copies a Biquad
- **Test / evidence:** KNOWN_ISSUE in the DSP build only "a copied filter works independently of the original"

## Bug 16: UART

- **Where:** `UART.cpp:62-106` (`transmit_IT/_DMA`, `receive_IT/_DMA`)
- **What:** The port is taken before the HAL call; on a HAL error no completion releases it. `UART_CommandInterface::Run` then waits forever in `while(uartport->isTaken()) Delay(1)`
- **Test / evidence:** 4 KNOWN_ISSUE tests in `test_uart.cpp`

## Bug 19: UsbHidHandler

- **Where:** `UsbHidHandler.cpp:19-21`
- **What:** Destructor does not clear `globalHidHandler`; TinyUSB callbacks (`global_callbacks.cpp:317, 332, 344`) would use a dangling pointer. Same root cause as the existing HidFFB case (`test_hid_ffb.cpp:51`): one bug
- **Test / evidence:** KNOWN_ISSUE "a registered handler unregisters when destroyed"

## Bug 21: CommandHandler

- **Where:** `CommandHandler.cpp:536` `flags &= flags | CMDFLAG_EXTOVERRIDE`
- **What:** No-op although the comment says "honor flag mask": (re-)registering an overridden command drops the override mask (moved from NEEDS REVIEW #6 after review r1)
- **Test / evidence:** KNOWN_ISSUE "registering an overridden command again keeps the override mask" (`test_command_handler_gaps.cpp`)

## Bug 23: Axis

- **Where:** `Axis.h:504-507`, `Axis.cpp:221-262, 498-501, 733-735`
- **What:** An axis restored from empty flash reports power 5000 / effect ratio 204, but `torqueScaler` and `effectRatioScaler` stay 0 (only computed in `setPower`/`setEffectRatio`, which `restoreFlash` calls only when flash has a value): **no torque until power/ratio are set by a command or flash**
- **Test / evidence:** KNOWN_ISSUE "an axis restored from empty flash applies its default power of 5000 and effect ratio of 204" (`claude/cases_target/test_target_setup.cpp`); first failing check is the driver torque. Confirmed in source

## Bug 25: Axis

- **Where:** `Axis.cpp`
- **What:** No `#include <cmath>`: compiles only with CMSIS-DSP (`-include cmath` added for `ffb_target_tests` with DSP off, F14)
- **Test / evidence:** Build

## Bug 26: Axis

- **Where:** `Axis.cpp:1415` `clip<int8_t>(gain,-120,120)`
- **What:** Narrowing inside `clip()` (`cppmain.h`): ill-formed C++, GCC warns, clang rejects (`-Wno-c++11-narrowing` for clang, `Axis.cpp` only, `ffb_target_tests` only). No runtime effect
- **Test / evidence:** Build

## Bug 27: Axis

- **Where:** `Axis.cpp:519-525` `setDrvType`
- **What:** A valid but not creatable driver id leaves `drv == nullptr`; `getEncType`, `setEncType`, `updateDriveTorque`, `emergencyStop` would dereference it
- **Test / evidence:** KNOWN_ISSUE "selecting a driver that cannot be created keeps a driver on the axis" (`test_axis_config.cpp`)

## Bug 28: Axis

- **Where:** `Axis.cpp:366` `saveFlash`
- **What:** `(uint16_t)(handsOffAccelThreshold*1000.0)` truncates: 985 of the values 0..1999 lose 1 per save (10 is saved as 9)
- **Test / evidence:** KNOWN_ISSUE "a hands-off acceleration threshold set by command survives a flash save and restore"

## Bug 29: Axis

- **Where:** `Axis.cpp:1320`
- **What:** The hands-off accel get reply truncates the float: 251 reads back as 250
- **Test / evidence:** KNOWN_ISSUE "the hands-off acceleration threshold reads back the value that was set"

## Bug 30: Axis

- **Where:** `Axis.cpp:875` vs `Axis.h:356-361, 473`
- **What:** `startForceFadeIn(start, fadeTime)` is documented as "over fadeTime", but `fadeTime/samplerate` is added per cycle: the fade takes (1-start)/fadeTime s (zeroenc 1.5 s, usbResume/estop reset 2 s instead of 0.5 s)
- **Test / evidence:** KNOWN_ISSUE (B) "zeroenc fades the force in within the 0.5 s fade time" and (A) "after a USB resume the torque is back to full within the documented fade time of 0.5 s"

## Bug 44: USBdevice

- **Where:** `USBdevice.cpp:87` `std::min<uint8_t>(31, field.length())`
- **What:** Truncates the length to 8 bits before comparing: a 256-character string becomes 0 characters (latent, no current string that long)
- **Test / evidence:** KNOWN_ISSUE "a string of 256 characters is cut to 31 characters"

## Bug 46: usb_descriptors

- **Where:** `usb_descriptors.cpp:25`
- **What:** `bDeviceClass` 0 with subclass 2 / protocol 1: USB 2.0 9.6.1 requires subclass 0 when the class is 0, the IAD ECN requires class 0xEF (the comment at line 24 states the IAD intent)
- **Test / evidence:** KNOWN_ISSUE (`test_usb_descriptors.cpp`)

## Bug 47: USBdevice

- **Where:** `USBdevice.cpp:81` vs `USBdevice.h:24`
- **What:** Interface strings get the first 4 decimal digits of UID word 0 ("(4259)"); the header documents the last n characters of the serial ("(0032)") (was NEEDS REVIEW 62)
- **Test / evidence:** KNOWN_ISSUE (`test_usb_descriptors.cpp`)

## Bug 48: ledEffects

- **Where:** `ledEffects.cpp:85`
- **What:** `HAL_GetTick() > tick + period` is not wrap-safe: a pulse started 16 ms before the 32-bit tick wraps ends after 1 ms
- **Test / evidence:** KNOWN_ISSUE (`test_led_effects.cpp`)

## Bug 50: voltagesense

- **Where:** `voltagesense.h:28` vs `voltagesense.cpp:40`
- **What:** `setupBrakePin(uint32_t, ...)` declared, `(int32_t, ...)` defined: any call through the header fails to link
- **Test / evidence:** Build (test declares the defined signature)

## Bug 52: voltagesense

- **Where:** `voltagesense.cpp:24, 85-88`
- **What:** A brake-resistor shutdown (stuck-on protection) does not report the defined critical error "Brake resistor stuck on" (the `addError` call is commented out, apparently because of ISR context): over-voltage protection stays off until reboot without indication (was NEEDS REVIEW 73)
- **Test / evidence:** KNOWN_ISSUE "a brake resistor shutdown reports the critical error Brake resistor stuck on" (`test_voltagesense.cpp`)

## Bug 53: I2CHandler

- **Where:** `I2CHandler.cpp:15-17`
- **What:** `~I2CHandler` calls `addCallbackHandler` instead of `removeCallbackHandler`: a deleted handler stays in the list and the next I2C callback calls a dead object. Confirmed in source
- **Test / evidence:** KNOWN_ISSUE "a destroyed I2C handler is removed from the I2C handler list" (`test_input_setup.cpp`); the fixture restores the handler list

## Bug 54: CanButtons

- **Where:** `CanButtons.h:50` with the constructor
- **What:** `mask` defaults to 0xff while `btnnum` is 32: without a stored config only 8 of the 32 reported buttons work. Confirmed in source
- **Test / evidence:** KNOWN_ISSUE "without a stored configuration all 32 reported buttons are received"

## Bug 57 (ext.): SPIDevice / MtEncoderSPI

- **Where:** `MtEncoderSPI.cpp:49-52`, `193`; `SPI.cpp` `beginSpiTransfer`
- **What:** Deleting the MT encoder during its DMA read leaves the port semaphore taken (only the completion through the deleted object gives it back)
- **Test / evidence:** KNOWN_ISSUE "deleting the MT encoder during its DMA read leaves the encoder SPI port free"

## Bug 60: CanAnalog

- **Where:** `CanAnalog.cpp:97`
- **What:** `canid` limit `0x7FF - axes/4` instead of `0x7FF - (packets-1)`: with 8 axes 0x7FE is refused (0x7FD stored), with 4 axes 0x7FF is refused
- **Test / evidence:** KNOWN_ISSUE

## Bug 62: ADS111X

- **Where:** `ADS111X.cpp:24-29` `readReg`
- **What:** Returns the bytes in bus order without swapping (0x8583 reads as 0x8385). Latent: no caller
- **Test / evidence:** KNOWN_ISSUE

## Bug 66: ADS111X

- **Where:** `ADS111X.cpp:31-40` `writeRegIT`/`writeReg`
- **What:** Write the shared `writeItBuffer` before taking the port semaphore: a second write overwrites a pending IT transfer's bytes (the real HAL reads the buffer byte by byte in the ISR; the fake copies at start, so not observable). Latent: `setThresh` not called
- **Test / evidence:** KNOWN_ISSUE "setting the thresholds starts the write of the low threshold register" (`test_ads111x.cpp:272`, fails at `:290`)

## Bug 67: SPIButtons / SPI

- **Where:** `SPIButtons.cpp:29-31` with `SPI.cpp:220-222`, constructor `SPIButtons.cpp:48`
- **What:** `SPI_Buttons_1::isCreatable` uses `hasFreePins`, which counts all CS pins, not the free ones: offered with no free pin, then the constructor reads `getFreeCsPins()[0]` of an empty vector
- **Test / evidence:** KNOWN_ISSUE (`test_spi_buttons.cpp`)

## Bug 68: SPIButtons

- **Where:** `SPIButtons.cpp:94-100`
- **What:** `setConfig` selects the stored CS number without checking it is free (TODO at 46): two devices drive one CS pin; deleting the buttons frees the other device's pin
- **Test / evidence:** KNOWN_ISSUE

## Bug 69: SPIButtons

- **Where:** `SPIButtons.cpp:91, 278`
- **What:** btnnum 0 is accepted but saved as 63 and restores as 64 buttons (CanButtons/PCF8574 limit to 1..64)
- **Test / evidence:** KNOWN_ISSUE

## Bug 71: ShifterAnalog

- **Where:** `ShifterAnalog.cpp:241-242`
- **What:** `cspin` command only stores the value; `setCSPin` is dead code, the G27 mode always uses the first free CS pin
- **Test / evidence:** KNOWN_ISSUE

## Bug 73: SelectableInputs

- **Where:** `SelectableInputs.cpp:95`
- **What:** Button offset shift of 64 or more is UB: with 64 previous buttons x86 ORs the next source onto buttons 1-8, ARM gives 0; the `uint8_t` shift also wraps above 255 (host-vs-target)
- **Test / evidence:** KNOWN_ISSUE_UB

## Bug 76: EncoderSSI

- **Where:** `EncoderSSI.cpp:123-124`
- **What:** Raw mode ORs every byte shifted by `bytes` (8) and loops over all 8 bytes instead of MSB-first over `transferlen`: 0x1234 / 0x1235 decode as 0x3600 / 0x3700
- **Test / evidence:** KNOWN_ISSUE "a raw frame one count higher gives a position one count higher"

## Bug 80: EncoderSSI / SPI

- **Where:** `EncoderSSI.cpp:93`, `:23` (`SPI.cpp:220-222`)
- **What:** `isCreatable` uses `hasFreePins`; the constructor always takes CS pin 0 even if another device holds it; deleting frees the other device's pin (bug 67 pattern)
- **Test / evidence:** KNOWN_ISSUE (2 cases) "an SSI encoder is not created on a CS pin another device holds", "deleting an SSI encoder keeps the CS pin of another device reserved"

## Bug 82: MtEncoderSPI

- **Where:** `MtEncoderSPI.cpp:335` (restore `:59`; effects 281, 188-201, 129-141)
- **What:** Any mode accepted: mode 2 gives `getCpr()` 0, starts no angle transfer (thread waits forever) and `readSpi` falls off its end (UB)
- **Test / evidence:** KNOWN_ISSUE "mode set with an unknown mode keeps the current mode"

## Bug 83: EncoderBissC

- **Where:** `EncoderBissC.cpp:175`, `182`
- **What:** `__builtin_clzll(0)` for an all-zero frame (no encoder answer; at 182 a frame with only the first bit set) is UB; on host and ARM the result still gives a CRC error. Visible only with `--no-skip` on a sanitizer build (UBSan)
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN "a frame of all zeros (no encoder answer) is counted as an error"

## Bug 85: EncoderBissC / SPI

- **Where:** `EncoderBissC.cpp:131` with `:27`
- **What:** `isCreatable` uses `hasFreePins` (bug 67 pattern); the constructor takes `getCsPins()[0]` even if held, then takes the port exclusively
- **Test / evidence:** KNOWN_ISSUE "the BiSS-C encoder is not creatable while another device uses a CS pin of the encoder SPI port"

## Bug 87: SPIPort / I2CPort

- **Where:** `SPI.cpp:240-284`, `I2C.cpp:230-233, 244-247`
- **What:** Completion callbacks clear the current device after calling it: a transfer the device starts in its own callback loses its completion and the semaphore stays taken. Latent (firmware callbacks only notify threads)
- **Test / evidence:** KNOWN_ISSUE "an SPI/I2C transfer that a device starts from its completion callback is completed to that device"

## Bug 88: I2CPort

- **Where:** `I2C.cpp:250-256`
- **What:** `I2cError` has no instance check: an error of any I2C peripheral reaches this port's current device (which resets the port) or resets the port. Latent on the F407 (one I2C port)
- **Test / evidence:** KNOWN_ISSUE "an error of another I2C port is not reported to the device of this port"

## Bug 91: EncoderBissC

- **Where:** `EncoderBissC.cpp:230`
- **What:** `mtpos = newpos / getCpr()` divides by a `uint32_t`: a negative `newpos` gives `mtpos` 1023, `getPos` one turn off after a nonzero `setPos` (reachable via `Axis.cpp:1151`, axis `pos`)
- **Test / evidence:** KNOWN_ISSUE (2 cases) "setPos with a value within the turn makes getPos return that value" (+ dir 0)

## Bug 92: MtEncoderSPI

- **Where:** `MtEncoderSPI.cpp:111-117` (constructor via `restoreFlash` line 67)
- **What:** `setCsPin` takes a pin without checking it is free: shares another device's pin; deleting the encoder frees it
- **Test / evidence:** KNOWN_ISSUE (2 cases) "a new MT encoder does not take a CS pin used by another device", "cs set to a pin used by another device keeps the current pin"

## Bug 93: EncoderLocal

- **Where:** `EncoderLocal.cpp:85`
- **What:** int32 % uint32: a negative distance becomes unsigned; cpr 1000 gives offset 196 instead of 900
- **Test / evidence:** KNOWN_ISSUE "zeroing after the index with a counts per turn that is not a power of two restores the index position modulo a turn"

## Bug 94: EncoderLocal

- **Where:** `EncoderLocal.cpp:85`
- **What:** Subtracts the new position (`idxpos-cnt-pos`): `setPos(500)` 100 counts after the index restores the index at 3496 instead of 400 (mod 4096)
- **Test / evidence:** KNOWN_ISSUE "setting a nonzero position after the index restores the index position modulo a turn"

## Bug 97: EncoderSSI

- **Where:** `EncoderSSI.cpp:58`, `:48`
- **What:** int32 offset saved as uint16, restored unsigned: -5 becomes 65531, position 65536 counts off after a restart
- **Test / evidence:** KNOWN_ISSUE "a negative position offset is restored as the same offset"

## Bug 100: MtEncoderSPI

- **Where:** `MtEncoderSPI.cpp:80` (save), `65` (restore)
- **What:** A negative offset is stored as `uint16_t` and read back unsigned: one turn (cpr) off after a restart (-261,488 instead of 656). Reachable via the axis `pos` command
- **Test / evidence:** KNOWN_ISSUE "a negative offset is restored by the next MT encoder"

## Bug 104: EncoderSSI

- **Where:** `EncoderSSI.cpp:233`
- **What:** `mode` command stores any value 0..15; an unknown mode decodes nothing
- **Test / evidence:** KNOWN_ISSUE "the mode command rejects a mode that does not exist"

## Bug 105: EncoderSSI

- **Where:** `EncoderSSI.cpp:214`, `:105`
- **What:** AMT23 bits clamp 12..14 keeps 13 although only 12 and 14 are valid (comment at 214); at 13 bits `newPos >>= 1` misaligns the check-bit pairs (half of valid frames counted as errors)
- **Test / evidence:** KNOWN_ISSUE "in AMT23 mode the bits command accepts only 12 or 14 bits"

## Bug 106: CANPort2B

- **Where:** `CANPort2B.cpp:233`, `:241` (with `global_callbacks.cpp:155-183`)
- **What:** `txMailboxes &= ~mailbox` clears with the mailbox index 0..2, but `txMailboxes` holds the `1 << index` bits from `HAL_CAN_AddTxMessage`: after mailbox 1 completes, `abortTxRequests()` leaves mailbox 0 pending
- **Test / evidence:** KNOWN_ISSUE (`test_can_setup.cpp`)

## Bug 107: CANPort2B

- **Where:** `CANPort2B.cpp:107-108`
- **What:** `start()` calls `setSpeedPreset` (stops and starts CAN1), then `HAL_CAN_Start` again, which fails (State LISTENING, `ErrorCode |= NOT_READY`): `start()` always returns false and the stale NOT_READY bit appears in the next error callback
- **Test / evidence:** KNOWN_ISSUE

## Bug 108: CANPort2B

- **Where:** `CANPort2B.cpp:295-320` (doc `:259`)
- **What:** With all 14 banks used, `foundId` ends at 13: bank 13 is overwritten and 13 returned instead of the documented -1
- **Test / evidence:** KNOWN_ISSUE

## Bug 112: CANPort2B

- **Where:** `CANPort2B.cpp:376-377`
- **What:** `len=260` is stored as uint8_t 4 before the limit to 8
- **Test / evidence:** KNOWN_ISSUE "a length above 255 is limited to 8 bytes like any other length above 8"

## Bug 113: CANPort2B

- **Where:** `CANPort2B.cpp:314-322` (doc `:258-260`)
- **What:** When `HAL_CAN_ConfigFilter` fails the bank number is still returned instead of -1
- **Test / evidence:** KNOWN_ISSUE "a filter that CAN1 rejects is reported as not added"

## Bug 117: CanBridge

- **Where:** `CanBridge.cpp:156-162`
- **What:** The GVRET start-marker loop uses `i < *Len-pos` while setting `pos=i+1`: only the first half of the transfer is searched (`E7 E7 F1 09` gets no keepalive reply; `E7 E7 07` runs a command without marker)
- **Test / evidence:** KNOWN_ISSUE (2 cases) "a GVRET command after two 0xE7 bytes in the same transfer is answered", "bytes without a 0xF1 start marker are not executed as a command"

## Bug 118: CanBridge

- **Where:** `CanBridge.cpp:171-178`
- **What:** Commands are split at every 0xF1 byte, not by command length: a 0xF1 in the id or data of a build-frame command starts a new command (spurious reply or second frame)
- **Test / evidence:** KNOWN_ISSUE "a data byte 0xF1 of a build-frame command is not read as the start of a new command"

## Bug 121: CanBridge

- **Where:** `CanBridge.cpp:101`
- **What:** The GVRET frame timestamp is the CAN hardware TIME field (16 bit, 0 without time-triggered mode) instead of microseconds as in GVRET and the time sync reply (`:200`)
- **Test / evidence:** KNOWN_ISSUE "the timestamp of a received frame is the time of reception in microseconds"

## Bug 123: SerialFFB

- **Where:** `SerialFFB.cpp:140`
- **What:** `reset?adr` passes the address to `free_effect(uint16_t)`: 65536 frees effect 0; no range check, every address answers OK
- **Test / evidence:** KNOWN_ISSUE "reset with an effect address beyond 16 bit does not free effect 0"

## Bug 128: SerialFFB

- **Where:** `SerialFFB.cpp:76-90`
- **What:** `newEffect` does not reset the slot (HidFFB assigns a fresh `FFB_Effect`, `HidFFB.cpp:268-278`): values written to a free slot carry into the next `new` (`state=1?1`, `mag=7000?1`, `new=1` gives a constant force already running at 7000)
- **Test / evidence:** KNOWN_ISSUE "a new effect in a slot written while free starts stopped with magnitude 0"

## Bug 132: CanBridge

- **Where:** `CanBridge.cpp:192-193`
- **What:** The build-frame command clips the copied data to 8 bytes itself but passes the raw `msgLen` as DLC to the HAL: 9..15 are sent as DLC 9..15 with 8 bytes, 16+ corrupt TDTR. Was NEEDS REVIEW 125 (reclassified by final review FR2)
- **Test / evidence:** characterization case of NR 125 (`test_canbridge.cpp`)

## Bug 133: Axis

- **Where:** `Axis.cpp:535-538`
- **What:** `setDrvType` sets `control->usb_disabled = false` in the branch where USB is NOT connected (looks inverted); only reader is the guard at `FFBHIDMain.cpp:308`. Very low impact. Was NEEDS REVIEW 28 (probable bug, final review FR2)
- **Test / evidence:** KNOWN_ISSUE "selecting a driver while USB is not connected keeps the usb_disabled flag set (bug 133)" (`claude/cases_target/test_axis_config.cpp`; was the characterization of NR 28)

## Bug 134: USBdevice / global_callbacks

- **Where:** `USBdevice.cpp:55-57` (via `global_callbacks.cpp:282-284`); TinyUSB `usbd.c:970-971` only rejects NULL
- **What:** The configuration descriptor callback ignores the index: SET_CONFIGURATION(2) mounts configuration 1 as 2, GET_DESCRIPTOR(CONFIG, 1) returns configuration 0, instead of a Request Error (USB 2.0 9.4.3, 9.4.7)
- **Test / evidence:** KNOWN_ISSUE (3 cases) "SET_CONFIGURATION with a configuration value the device does not have is rejected (bug 134)", "GET_DESCRIPTOR of a configuration index the device does not have is rejected (bug 134)" (`claude/cases_usb/test_usbd_core.cpp`), "the configuration descriptor of an index the device does not have is not available (bug 134)" (`claude/cases_target/test_cppmain.cpp`; the passing case there now covers index 0 only and was renamed "the device descriptor and the configuration descriptor of index 0 are the ones given to the device")

## Bug 137: CDCcomm / global_callbacks

- **Where:** `CDCcomm.cpp:67` (Take), `:31-32` (only Give), `:70`, `:74-77`; `global_callbacks.cpp:301-304, 365-380`
- **What:** `cdcSend` holds `cdcSems` until the Tx-complete callback; a bus reset or unplug drops the running IN transfer without a completion and nothing gives the semaphore back: the CDC thread's next `cdcSend` blocks forever, **no CDC replies until a power cycle** (e.g. the PC restarts while an externally powered wheel has a reply pending) Second part: a pending reply longer than the free FIFO leaves `remainingStrs` and `usb_busy_retry` set, so a semaphore-only fix still loses the next reply (`CDCcomm.cpp:70` skips the flush). Consequence holds for FFBWheel, FFBJoystick, FFBHIDExt and the failsafe class; `CommandHandler::logSerial` (`CommandHandler.cpp:453`) also blocks its caller. CanBridge (`CanBridge.cpp:120, 282`) writes CDC directly, and its Tx completion gives the semaphore back (review p9r2).
- **Test / evidence:** KNOWN_ISSUE (3 cases) "the CDC reply path works again after the host reset the device while a reply was pending (bug 137)", "... while a reply longer than the FIFO was pending (bug 137)", "... after the cable was unplugged while a reply was pending (bug 137)" (`claude/cases_usb/test_cdc_device.cpp`; a full fix must also clear `usb_busy_retry`/`remainingStrs`)

## Bug 138: global_callbacks (HidFFB, HID command interface)

- **Where:** `global_callbacks.cpp:312-325`; TinyUSB `hid_device.c:323-327` vs `:396`; `HidFFB.cpp:113-114`; `HidCommandInterface.h:23`
- **What:** TinyUSB strips the report id from SET_REPORT data but not from interrupt OUT reports; the firmware only fixes the id argument, so an output report sent with SET_REPORT(Output) (e.g. Windows `HidD_SetOutputReport`) is parsed one byte off
- **Test / evidence:** KNOWN_ISSUE (2 cases) "an output report sent with SET_REPORT reaches the HID handler like one sent on the interrupt endpoint (bug 138)", "a PID device control sent with SET_REPORT(Output) enables the actuators (bug 138)" (`claude/cases_usb/test_hid_device.cpp`)

## Bug 140: TMC4671 driver: SPI transport

- **Where:** `TMC4671.cpp:2532` + `SPI.cpp:163,172,181`
- **What:** `readReg` returns the uninitialised `tbuf` when the HAL transfer fails (`SPIPort` ignores the HAL result): stack garbage is used as register data
- **Test / evidence:** code reading; nondeterministic, no test (R8)

## Bug 141: TMC4671 driver: Identification

- **Where:** `TMC4671.cpp:1855`
- **What:** `tmctype` version uses `&&` instead of `&`: any version prints "1.1" (or "0.0")
- **Test / evidence:** KNOWN_ISSUE 2 cases [R11] (B-T3)

## Bug 143: TMC4671 driver: PI setup

- **Where:** `TMC4671.cpp:2131`
- **What:** `getPids()` resets `sequentialPI` to true (aggregate init), so a classic PI setup is persisted as sequential
- **Test / evidence:** KNOWN_ISSUE [R41] (B-T5)

## Bug 146: TMC4671 driver: AENC encoder

- **Where:** `TMC4671.cpp:1340`
- **What:** `setup_AENC` writes phi_e offset to [15:0] and phi_m offset to [31:16] of 0x45; DS 0x45 p91 has phi_m [15:0], phi_e [31:16] (`bangInitEnc` uses the DS layout)
- **Test / evidence:** KNOWN_ISSUE [R51] (B-T8)

## Bug 147: TMC4671 driver: Voltage mode

- **Where:** `TMC4671.cpp:1667-1669`
- **What:** `setUdUq` with negative `ud` sign-extends into the `uq` half of 0x24
- **Test / evidence:** KNOWN_ISSUE [R53] (B-T9)

## Bug 148: TMC4671 driver: GPIO

- **Where:** `TMC4671.cpp:1829-1832`
- **What:** `setGpioPins` masks the pre-shifted value with 0xff: always writes 0 (no caller)
- **Test / evidence:** code reading, dead code, no test (R55)

## Bug 149: TMC4671 driver: Control

- **Where:** `TMC4671.cpp:2077-2079`
- **What:** `getFlux()` uses `&& 0xffff`: returns 0 or 1
- **Test / evidence:** KNOWN_ISSUE 2 cases [R61] (B-T11)

## Bug 151: TMC4671 driver: Readouts

- **Where:** `TMC4671.cpp:235-241`
- **What:** `getTmcVM` below the ADC offset wraps (uint32) and converts an out-of-range float to int32 (UB; x86 INT_MIN, ARM INT_MAX)
- **Test / evidence:** KNOWN_ISSUE_UB [R67] (B-T13)

## Bug 157: TMC4671 driver: Commands

- **Where:** `TMC4671.cpp:3052-3055`
- **What:** `reg` getat reads `readReg(cmd.val)` instead of `cmd.adr` (setat uses `adr`)
- **Test / evidence:** KNOWN_ISSUE [R101] (B-T19)

## Bug 158: TMC4671 driver: Flash

- **Where:** `TMC4671.cpp:110, 154-155`
- **What:** ABN cpr is stored in a 16-bit flash word: cpr above 65535 is truncated and written back to 0x26/0x40
- **Test / evidence:** KNOWN_ISSUE [R112] (B-T20)

## Bug 160: TMC4671 driver: Register map

- **Where:** `TMC4671.cpp:990, 1119` (0x41), `:945` (0x1E), `:991` (0x47), `:2169, 2181` (0x5C)
- **What:** Writes to AENC_DECODER_COUNT 0x41 (read-only in DS §7.1) and use of 0x1E, 0x47, 0x5C, which datasheet Rev 2.07 does not document (older register map?)
- **Test / evidence:** fake records them; no KNOWN_ISSUE (effect undocumented), open question Q1

## Bug 161: TMC4671 driver: Control

- **Where:** `TMC4671.cpp:2102-2113`
- **What:** `rampFlux` stops short of the target when the distance is not a multiple of the step (0 -> 105 in 20 ms ends at 100)
- **Test / evidence:** KNOWN_ISSUE [R66] (B-T23)

## Bug 163: TMC4671 driver: State machine

- **Where:** `TMC4671.cpp:581-592, 658-660, 1701-1724`
- **What:** After a hardware estop the recovery in `Shutdown` (clear `emergencyStop`, re-init) is unreachable: the thread suspends itself, `emergencyStop(true)` goes straight to Running and the critical error stays
- **Test / evidence:** KNOWN_ISSUE [R74] (B-T25)

## Bug 164: TMC4671 driver: State machine

- **Where:** `TMC4671.cpp:609-612, 1671-1680`
- **What:** Over-temperature: `OverTemp` calls `stopMotor()`, which acts only in Running/EncoderFinished, so the driver ends in HardError with PWM still in FOC mode and torque mode on
- **Test / evidence:** KNOWN_ISSUE [R76] (B-T26)

## Bug 165: TMC4671 driver: External encoder

- **Where:** `TMC4671.cpp:977-1007`
- **What:** External-encoder alignment does a read-modify-write of read-only register 0x00 (`phiEoffsetReg` stays 0)
- **Test / evidence:** KNOWN_ISSUE [R94] (B-T27)

## Bug 167: TMC4671 driver: Calibration

- **Where:** `TMC4671.cpp:526` vs `:1461`
- **What:** `calibrated` reads 0 right after a successful full calibration (ABN): the encoder re-init sets `recalibrationRequired` again
- **Test / evidence:** KNOWN_ISSUE [R93] (B-T29)

## Bug 168: TMC4671 driver: Commands

- **Where:** `TMC4671.cpp:2853`
- **What:** `mtype` set: the range check tests `cmd.type` instead of the value; 259 wraps to BLDC through the 8-bit cast
- **Test / evidence:** KNOWN_ISSUE [R102] (B-T30)

## Bug 169: ODrive driver: Commands / flash

- **Where:** `ODriveCAN.cpp:356, 138`
- **What:** `maxtorque` get and save truncate `maxTorque*100`: 282 of the values 0..4095 read back one lower (set 53 -> get 52), and a save persists the lower value
- **Test / evidence:** KNOWN_ISSUE 2 cases [O51] (B-O1)

## Bug 171: ODrive driver: RX parsing

- **Where:** `ODriveCAN.cpp:390-435`
- **What:** No DLC/RTR check: RTR frames and frames shorter than the payload (e.g. another master's RTR 0x09) overwrite position, voltage, error and state with stale/zero data (CSV payload sizes, SPEC call & response)
- **Test / evidence:** KNOWN_ISSUE 6 cases [O25] (B-O3)

## Bug 173: ODrive driver: RX parsing

- **Where:** `ODriveCAN.cpp:390`
- **What:** Extended-id frames whose upper bits match are interpreted as CAN Simple standard frames
- **Test / evidence:** KNOWN_ISSUE 3 cases [O26] (B-O5)

## Bug 174: ODrive driver: State machine

- **Where:** `ODriveCAN.cpp:174-185, 196-199`
- **What:** Set Controller Modes (torque, passthrough) is not sent when the ODrive is already in closed loop at connect (T7 path) or when the closed-loop heartbeat arrives after START_RUNNING (`state=RUNNING` before the check): the driver then sends torque without having selected torque control
- **Test / evidence:** KNOWN_ISSUE 2 cases [O33] (B-O6)

## Bug 175: ODrive driver: Flash

- **Where:** `ODriveCAN.cpp:145-146, 113-118`
- **What:** The stored position offset (`storepos`) is a 16-bit word read back as int16: offsets beyond half a turn (32767 counts) wrap
- **Test / evidence:** KNOWN_ISSUE [O62] (B-O7)

## Bug 177: ODrive driver: Encoder

- **Where:** `ODriveCAN.cpp:253`
- **What:** `getPos_f` measures its 5 ms re-request window from the last reply, not from the request: without replies every call sends an RTR 0x09 (request flood at the axis loop rate)
- **Test / evidence:** KNOWN_ISSUE [O42] (B-O9)

## Bug 178: ODrive driver: Flash

- **Where:** `ODriveCAN.cpp:96-102`
- **What:** `restoreFlash` selects the M1 settings word 0x3D2 only inside the successful read of 0x3D0: without 0x3D0, ODriveCAN2 restores motor 0's settings
- **Test / evidence:** KNOWN_ISSUE [O60] (B-O10)

## Bug 180: VESC driver: TX frames

- **Where:** `VescCAN.cpp:449-459`
- **What:** The values request is sent with DLC 8 and an uninitialised 8th byte (SRC sender: DLC = payload 5 + 2 = 7)
- **Test / evidence:** KNOWN_ISSUE [V13] (B-V2)

## Bug 181: VESC driver: TX frames

- **Where:** `VescCAN.cpp:497`
- **What:** `memcpy(&msg.data, nullptr, 0)` for every POLL_ROTOR_POS request: UBSan "null pointer passed as argument 2" aborts sanitizer runs
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN 5 cases (B-V3)

## Bug 182: VESC driver: Buffer protocol

- **Where:** `VescCAN.cpp:643-667`
- **What:** PROCESS_RX_BUFFER does not check that the buffered byte count equals the announced length: a partial or stale buffer with a matching CRC is decoded (SRC drops it)
- **Test / evidence:** KNOWN_ISSUE 2 cases [V23] (B-V4)

## Bug 184: VESC driver: FW version check

- **Where:** `VescCAN.cpp:529-530`
- **What:** The byte after the UUID is read as the test version, but SRC 7.01 sends `pairing_done` there and the test version next: a paired 5.3 release is rejected, a 5.3.50 test build is accepted (open question 16a)
- **Test / evidence:** KNOWN_ISSUE 2 cases [V30] [V31] (B-V6)

## Bug 185: VESC driver: FW version check

- **Where:** `VescCAN.cpp:526`
- **What:** `memcpy(buffer + ind, uuid, 12)` has source and destination swapped (writes zeros into the receive buffer); not observable
- **Test / evidence:** code reading, no test

## Bug 187: VESC driver: Torque

- **Where:** `VescCAN.cpp:132-139, 380-383`
- **What:** `stopMotor` does not reset `lastTorque`: after stop and start the 500 ms refresh sends the old torque without any `turn()`
- **Test / evidence:** KNOWN_ISSUE [V52] (B-V9)

## Bug 188: VESC driver: Encoder

- **Where:** `VescCAN.cpp:478-483`
- **What:** `prevPos360` starts at 0: a first position sample above 180 deg counts a spurious turn (-1)
- **Test / evidence:** KNOWN_ISSUE [V62] (B-V10)

## Bug 195: TMCDebugBridge: command 143

- **Where:** `TMCDebugBridge.cpp:171-178`
- **What:** Unknown type (address byte other than 3 / 4) answers an all-zero reply with status 0, which is not a TMCL status code (valid: 100 and 1..6, TMCL §2.2.1); opcode 143 itself is not in the supplied documents
- **Test / evidence:** KNOWN_ISSUE (B-DTDF-2)

## Bug 198: TMCDebugBridge: GGP and GIO replies

- **Where:** `TMCDebugBridge.cpp:180, 189`
- **What:** The status byte is 64 (0x40), not 100 (0x64 = "no error", TMCL §2.2 / §3.10 / §3.15 example reply `02 01 64 0A ...`); 64 is not a status code. Looks like a hex / decimal slip (observation)
- **Test / evidence:** KNOWN_ISSUE (B-DTDF-5)

## Bug 199: TMCDebugBridge: command 143 replies

- **Where:** `TMCDebugBridge.cpp:173, 175`
- **What:** The status byte is 40 (0x28), not a TMCL status code (valid: 100 and 1..6); the opcode 143 is not in the supplied documents, so only membership in the valid set is asserted
- **Test / evidence:** KNOWN_ISSUE (B-DTDF-6)

## Bug 201: MotorPWM: RC frame period

- **Where:** `MotorPWM.cpp:129-157, 167`
- **What:** The RC frame is 1.2 % longer than named: the prescaler register is written `timerFreq / 2000000` = 84 but divides by PSC + 1 = 85, so "20ms / 15ms / 10ms / 5ms" are 20.2386 / 15.1791 / 10.1196 / 5.0600 ms (names `RC_SpeedNames`, tolerance 0.1 %). The pulse widths are exact (`tFreq` uses `prescaler + 1`). Owner question 32
- **Test / evidence:** KNOWN_ISSUE (B-PWM-1)

## Bug 207: SimpleMotion: voltage range

- **Where:** `MotorSimplemotion.cpp:239-245`
- **What:** `getVoltage()` returns mV as `int16_t`: 4800 counts (48 V) give -17536, 3277 counts (32.77 V) give -32766; the `voltage` command replies the same. Owner question 39
- **Test / evidence:** KNOWN_ISSUE (B-SM-3, 3 cases)

## Bug 208: SimpleMotion: uninitialised status

- **Where:** `MotorSimplemotion.cpp:170-178`
- **What:** `st` is read uninitialised when the status read fails (`if(!st)`), so the number of requests after a failed status read depends on stack content
- **Test / evidence:** not testable (NEEDS REVIEW 257)

## Bug 210: SimpleMotion: torque range

- **Where:** `MotorSimplemotion.cpp:248-254`
- **What:** `getTorque()` returns mA as `int16_t`: 20000 counts = 35714 mA give -29822 (same shape as B-SM-3; above 18350 counts)
- **Test / evidence:** KNOWN_ISSUE (B-SM-6)

## Bug 214: RMD: ErrorStatus bit field

- **Where:** `RmdMotorCAN.h:69-85`
- **What:** The public `ErrorStatus` flags disagree with the SP 2.14 table from `overpower` on (field at bit 5 / 6 / 7 / 8 / 9 where SP has 0x0040 / 0x0080 / 0x0100 / 0x0800 and 0x1000 / 0x2000); one `overtemp` flag for two SP bits; no flag for 0x4000 (falls into `res2`). Only `asInt` is used: no behavioural effect today. Owner questions 48 and 52
- **Test / evidence:** KNOWN_ISSUE (B-RMD-4, 6 cases)

## Bug 215: RMD: position offset

- **Where:** `RmdMotorCAN.cpp:81, 93-95`
- **What:** The offset (range +-35999) is saved as `uint16_t` and restored as `int16_t`: 35000 comes back -30536 (`getPos` 130536 instead of 65000), -35999 comes back +29537 (70463 instead of 135999). Owner question 47
- **Test / evidence:** KNOWN_ISSUE (B-RMD-5, 2 cases)

## Bug 216: RMD: interpolation dimension

- **Where:** `RmdMotorCAN.cpp:254-256`
- **What:** In the request mode the 0xA1 handler adds `speed / (1e6 * dt_us)` (500 dps, 10 ms: 5e-11 degrees) instead of speed (dps) times dt (s) = 5 degrees; also the condition `ustime - lastTorqueStatusUpdate_us` is a truthiness test. Owner question 49
- **Test / evidence:** KNOWN_ISSUE (B-RMD-6)

## Bug 217: RMD: temperature type

- **Where:** `RmdMotorCAN.h:107`, `RmdMotorCAN.cpp:249, 288`
- **What:** `curTemp` is `uint8_t`, SP 2.14 / 2.19 say int8: -10 degC reads 246. Owner question 48
- **Test / evidence:** KNOWN_ISSUE (B-RMD-7)

## Bug 218: RMD: command status

- **Where:** `RmdMotorCAN.cpp:350-352`
- **What:** `maxtorque` discards the status of `handleGetSet`: an unsupported command type answers OK where every other RMD command answers ERR (theoretical, the parser's flag check normally rejects the type first). Owner question 51
- **Test / evidence:** KNOWN_ISSUE (B-RMD-8)

## Bug 221: MidiHandler: `sysexState`

- **Where:** `MidiHandler.cpp:33-45`
- **What:** `sysexState` is cleared only by a packet whose byte 1 is 0xF7; a SysEx that ends in byte 2 or 3 of the packet (`p6 02 F7 00`, `p7 02 03 F7`, `p6 F0 F7 00`, UM Table 4-2) leaves it true (not read anywhere today). Owner question 58
- **Test / evidence:** KNOWN_ISSUE (B-MIDI-3)

## Bug 224: global_callbacks / MIDI class: no handler

- **Where:** `global_callbacks.cpp:355`, `midi_device.c:93-113`
- **What:** While no `MidiHandler` exists `tud_midi_rx_cb` returns without reading: the first packet leaves 60 free RX FIFO bytes, `_prep_out_transaction` arms the OUT endpoint only with 64 free, so the endpoint NAKs for ever, also after a handler is created. Confirmed by experiment in T2. Owner question 60
- **Test / evidence:** KNOWN_ISSUE (B-MIDI-6)

## Bug 225: SerialFFB: negative effect address on 32 bit

- **Where:** `SerialFFB.cpp:157, 164, 179, 186, 193, 201, 208, 220, 227, 234` (`cmd.adr < effects.size()`), `CommandHandler.h:70` (`int64_t adr`)
- **What:** The command address is `int64_t`, the guard compares it with `size_t`. On the 64-bit host (LP64) the comparison is unsigned and every negative address is rejected; on the 32-bit target (ILP32) `size_t` is converted to `int64_t`, the comparison is signed and any negative address passes: `effects[adr]` is read (`mag`, `state`, `type` get and every get) and written (`period`, `duration`, `deadzone`, `offset`, `sat`, `coeff`, `axisgain` set) outside the array; `state` set truncates to `uint8_t` first and answers OK instead of ERR. Reachable on shipped firmware with `FFBHIDEXT` (F407VG, F407VG_DISCO, F411RE): serial `fxm.0.period=5?-1`, HID command report with addr all ones, CAN. Found by the P18 32-bit build. Suggested fix: `cmd.adr >= 0 && cmd.adr < (int64_t)effects.size()` (a `(size_t)` cast is not a fix on ILP32: 2^32 becomes slot 0)
- **Test / evidence:** `test_serial_ffb_32bit.cpp`: 18 `KNOWN_ISSUE_32BIT_UB` (skipped on ILP32, abort with `--no-skip`), 1 `KNOWN_ISSUE_32BIT` (`state` set); the existing `mag` / `state` rejection cases are excluded in 32-bit builds (`FFB_TARGET_TESTS_EXCLUDE_32BIT`)

## Vendor V1: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `csemaphore.cpp:117-131`
- **What:** `CountingSemaphore(0, x)` or initial > max throws before `handle` is set; `~Semaphore` then deletes an uninitialised handle (UB). Exception builds only (target uses `CPP_FREERTOS_NO_EXCEPTIONS`/`configASSERT`)
- **Test / evidence:** KNOWN_ISSUE_UB "a counting semaphore with a maximum count of 0 is rejected without deleting an unrelated handle", "a counting semaphore with an initial count above its maximum is rejected without deleting an unrelated handle" (`claude/cases_rtos/test_csync_gaps.cpp:337, 353`); exception builds only (NR 142)

## Vendor V3: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `cmutex.cpp:53, 61-63, 91-93`, `csemaphore.cpp:93, 101-103, 135-137`
- **What:** On a kernel create failure (NULL), the destructors call `vSemaphoreDelete(NULL)` (real FreeRTOS asserts)
- **Test / evidence:** KNOWN_ISSUE "a standard mutex whose creation fails does not delete a NULL handle", "a recursive mutex ...", "a binary semaphore ...", "a counting semaphore ..." (`claude/cases_rtos/test_csync_gaps.cpp:204, 215, 315, 326`; configASSERT `queue.c:1979`)

## Vendor V8: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `ctasklet.cpp:124`
- **What:** `ScheduleFromISR` gives `DtorLock` back with `xSemaphoreGive` instead of the FromISR variant when the pend fails (timer service queue full): in an interrupt FreeRTOS stops in configASSERT (`port.c:415`). Latent
- **Test / evidence:** KNOWN_ISSUE "a tasklet that can not be scheduled from an interrupt releases its lock with the interrupt API"

## Vendor V9: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `ctimer.cpp:89`
- **What:** `~Timer` only queues `xTimerDelete`: the timer keeps the destroyed object as its ID and stays active, and the service processes expired timers and queued starts before the delete (`timers.c:548-560`), so it can call `Run()` of the destroyed object (deleting task priority >= 2). Latent
- **Test / evidence:** KNOWN_ISSUE "the timer service can no longer call a destroyed active timer" (`claude/cases_rtos/test_ctimer.cpp`, deleter at priority 3)

## Vendor V11: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `cworkqueue.cpp:72-74`, `:90-92`
- **What:** When a later `new` in the constructor throws, the item queue (and ThreadComplete) leak. Exception builds only (NR 142)
- **Test / evidence:** KNOWN_ISSUE_UB "a work queue whose completion semaphore can not be created leaves no kernel queue behind" (`test_cworkqueue.cpp`; LSan)

## Vendor V12: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `ctasklet.cpp:87` (doc `tasklet.hpp:145-148`)
- **What:** `Schedule` waits for its lock with `portMAX_DELAY` whatever `CmdTimeout` is: `Schedule(p, 0)` blocks while an earlier run is pending. Latent
- **Test / evidence:** KNOWN_ISSUE "a schedule with a timeout of 0 returns at once while an earlier run is still pending" (`test_ctasklet.cpp`)

## Vendor V21: TinyUSB (**vendor, tier B**)

- **Where:** `device/usbd.c:637-638` (only a completion clears `claimed`), `usbd_edpt_stall`/`usbd_edpt_clear_stall`; `class/cdc/cdc_device.c` (`cdcd_control_xfer_cb`, `_prep_out_transaction`)
- **What:** SET_FEATURE(ENDPOINT_HALT) on the CDC data OUT endpoint drops the armed transfer (dwc2 `edpt_disable`) but leaves the endpoint claimed; CLEAR_FEATURE only clears halt and busy and the CDC driver ignores standard requests, so nothing re-arms it and `tud_cdc_n_read` cannot either: the device NAKs every OUT packet until a bus reset or re-enumeration, CDC command input dead (USB 2.0 9.4.5). The HID OUT endpoint is in the same family. Low: hosts rarely halt a bulk endpoint. Only the claim fix together with a CDC re-arm passes
- **Test / evidence:** KNOWN_ISSUE "the CDC data endpoint receives again after the host clears its halt (V21)" (`claude/cases_usb/test_usb_setup.cpp`)

## Vendor V22: TinyUSB (**vendor, tier B**)

- **Where:** `device/usbd.c:876-895`
- **What:** The fallback for standard interface requests acknowledges SET_INTERFACE for any alternate setting (9.4.10) and stalls GET_STATUS(interface), which must return 0x0000 (9.4.5)
- **Test / evidence:** KNOWN_ISSUE (2 cases) "SET_INTERFACE with an alternate setting the interface does not have is rejected (V22)", "GET_STATUS of an interface returns two zero bytes (V22)" (`claude/cases_usb/test_usbd_core.cpp`)

## Vendor V24: TinyUSB (**vendor, tier B**)

- **Where:** `usbd.c:1398-1409`
- **What:** `usbd_edpt_clear_stall` clears busy on an endpoint that is not halted while its transfer is still armed: the endpoint can be claimed and armed twice (e.g. `tud_cdc_n_read` after CLEAR_FEATURE)
- **Test / evidence:** KNOWN_ISSUE "CLEAR_FEATURE(ENDPOINT_HALT) of an endpoint that is not halted keeps its armed transfer busy (V24)"

## Vendor V29: TinyUSB (**vendor, tier B**)

- **Where:** `hid_device.c:110-126`
- **What:** `tud_hid_n_report` while unconfigured (`ep_in` 0) claims EP0 OUT, arms the report buffer on it and returns true although nothing is sent. Latent
- **Test / evidence:** KNOWN_ISSUE "an input report is rejected while the device is not configured (V29)"

## Vendor V30: TinyUSB (**vendor, tier B**)

- **Where:** `device/usbd_control.c:76-80` (and `:156`)
- **What:** A device-to-host request with wLength 0 gets an OUT status stage; USB 2.0 8.5.3 (no-data control) requires IN, so the host NAK-times out (e.g. GET_DESCRIPTOR with wLength 0)
- **Test / evidence:** KNOWN_ISSUE "a device-to-host request with wLength 0 completes with an IN status stage (V30)" (`claude/cases_usb/test_usbd_control.cpp`)

## Vendor V31: TinyUSB (**vendor, tier B**)

- **Where:** `tusb_mcu.h:201` (`TUP_DCD_ENDPOINT_MAX 6` for every STM32F4), `usbd.c:906`
- **What:** Endpoint numbers are only checked against `CFG_TUD_ENDPPOINT_MAX` = 6, but the F407 OTG_FS core has 4: a host SET/CLEAR_FEATURE(HALT, 0x84/0x85/0x04/0x05) reaches dwc2 `edpt_disable`/`dfifo_flush_tx` on registers the core does not have and is not stalled (9.4.9). Low, host-triggered. A config fix (`CFG_TUD_ENDPPOINT_MAX 4`) passes
- **Test / evidence:** KNOWN_ISSUE "SET_FEATURE(ENDPOINT_HALT) of an endpoint number the controller does not have is rejected (V31)" (`claude/cases_usb/test_usbd_core.cpp`)

## Vendor V34: TinyUSB (**vendor, tier B**)

- **Where:** `device/usbd_control.c` (`tud_control_xfer`, `_data_stage_xact`), `class/cdc/cdc_device.c` SET_LINE_CODING
- **What:** A control write with a wLength larger than the class buffer (e.g. SET_LINE_CODING with wLength > 7) ends the data stage early: the host's remaining OUT packets are NAKed until timeout; if the buffer length is a multiple of 64 a 0-byte OUT with a NULL buffer is armed (dwc2 would read into address 0). HID checks wLength (`hid_device.c:314`); CDC only from a non-conforming host. Latent
- **Test / evidence:** code reading (the fake records such a packet as an overrun fault)

## Vendor V35: generate_memory.py (**vendor/tooling**)

- **Where:** `Firmware/scripts/generate_memory.py:137-143`
- **What:** The duplicate check compares address strings; the map mixes decimal and hex, so 16 and 0x10, 0x101 and 0x0101, 0x1A and 0x1a are missed (two variables would share one EEPROM cell)
- **Test / evidence:** `@expectedFailure` `NumericDuplicateTest.test_the_same_address_written_in_decimal_and_in_hex_is_a_duplicate` (check in `setUp`, so a raising fix shows as an error)

## Vendor V36: generate_memory.py (**vendor/tooling**)

- **Where:** `generate_memory.py:111-113` vs `:84`, `:118`
- **What:** `NB_OF_VAR` / `NB_EXPORTABLE_ADR` count enabled rows without a name or address, but no array element is emitted for them: C zero-fills the rest, so virtual address 0 becomes a registered and exported variable
- **Test / evidence:** `@expectedFailure` `IncompleteEnabledRowTest.test_the_variable_count_matches_the_array_elements_also_for_an_enabled_row_without_an_address` (generation in `setUp`)

## Vendor V40: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_vlog_f16.c` (`logf16_scalar`)
- **What:** Positive f16 subnormals are not normalised: log(2^-24) should be -16.6 and is -10.4; log(1.0014e-5) should be -11.5 and is -10.24 (measured while probing). DOC gives no input range, but the input is positive and in the domain
- **Test / evidence:** KNOWN_ISSUE_UB "log of a positive subnormal input ..." (B-DFF16-1; UB because the call also hits V39)

## Vendor V41: CMSIS-DSP (**vendor, tier C**, low, open question 17)

- **Where:** `arm_sqrt_q15.c`
- **What:** 6 of the 32767 positive inputs exceed ARM's own bound of 6 LSB against the rounded double sqrt (x=30840 gives 31796, reference 31789, error 7; also 31373, 31782, 31796, 31797, 31812). ARM's 1024-point pattern does not contain them. Unrounded worst error 7.15 at x=31797. Possibly a tolerance question, not a defect
- **Test / evidence:** KNOWN_ISSUE "B-DFQ15-1" (`[C3 arm_sqrt_q15] ... within ARM's 6 LSB of the rounded sqrt(x)`)

## Vendor V42: CMSIS-DSP (**vendor, tier C**, low, open question 17)

- **Where:** `arm_sqrt_q31.c`
- **What:** Measured over all 2^31-1 positive inputs against round(sqrt(x/2^31)*2^31): worst error 9 LSB (x=2065697199), 39621 inputs exceed ARM's own bound of 7 LSB, 1285810 exceed 6. ARM's pattern does not contain them. Same question as V41 (is ARM's tolerance the contract?)
- **Test / evidence:** KNOWN_ISSUE "a strided sweep over the whole input range stays within ARM's 7 LSB ..." (B-DFQ31-2; the strided sweep first fails at x=2147483393, error 8)

## Vendor V43: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_linear_interp_q31.c:97`
- **What:** `return (y << 1U)` shifts a negative value left (UB; UBSan: "left shift of negative value"); the result is the intended one on the host (L10)
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN C1 / C2 / C3 mixed-sign cases (B-DIQ-1; positive-table twins run under UBSan)

## Vendor V44: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_bilinear_interp_q31.c:114`
- **What:** `return ((q31_t)(acc << 2))` shifts a negative value left (UB), same as V43
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN C1 / C2 / C3 (B-DIQ-2)

## Vendor V45: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_bilinear_interp_f16.c:68-69`, `arm_bilinear_interp_f32.c:116-117`
- **What:** `(int32_t)X` / `(int32_t)Y` truncate toward zero, DOC says XF = floor(x): X or Y in (-1, 0) pass the `< 0` outside-the-table test and the first cell is extrapolated (2x2 table {1,2,3,5}, X=-0.5, Y=0 gives 0.5, DOC: zero). The q31/q15/q7 versions floor correctly
- **Test / evidence:** KNOWN_ISSUE f16 (B-DIH-1) and f32, 3 cases (B-DIF-2)

## Vendor V48: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_pid_init_q31.c:71-76`
- **What:** Every partial sum is clipped on the host path: Kp = Ki = 0.75, Kd = -0.75 gives A0 = 0.25 and A1 = 0.25 instead of 0.75 and 0.75, although all gains are in range. The q15 init saturates once at the end and is right; on the target QADD16 clips partial sums as well
- **Test / evidence:** KNOWN_ISSUE (B-DCP-5)

## Vendor V49: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_sin_cos_q31.c:69, 81, 83, 99, 101`
- **What:** Left shifts of negative values (`Df << 32`, `<< 23` of 384) are UB; UBSan reports them first at line 81; the host result is the intended one (L10)
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN, 13 q31 sin_cos cases (B-DCP-3)

## Vendor V52: CMSIS-DSP (**vendor, tier C**, low)

- **Where:** `controller_functions.h` (`arm_park_q31`, `arm_inv_park_q31`)
- **What:** (-1.0) x (-1.0) = 2^31 wraps to INT32_MIN (Ialpha = INT32_MIN with cos = INT32_MIN gives INT32_MIN, expected INT32_MAX); only when both factors are exactly -1.0
- **Test / evidence:** KNOWN_ISSUE (B-DCT-2)

## Vendor V53: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_max_no_idx_{f16,f32,f64}.c` (`:119`, `:116`, `:119`), `arm_min_no_idx_{f16,f32,f64}.c`
- **What:** The scalar path starts from `F*_MIN` (= -FLT_MAX / -DBL_MAX / -FLT16_MAX) resp. `F*_MAX` instead of the first sample, so a block of only -inf (max) or only +inf (min) returns the largest finite value instead of the infinity; the indexed and absmax variants are right. The DOC text of `arm_min_no_idx_f64` is a copy of the max text
- **Test / evidence:** KNOWN_ISSUE x6 (B-DSF16-1/2, B-DSF32-1/2, B-DSF64-1/2; passing companions at +-FLT_MAX)

## Vendor V55: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_logsumexp_f32.c:251-271`, `arm_logsumexp_f16.c`
- **What:** A maximum of +-inf gives expf(inf - inf) = NaN: lse {-inf,-inf} = NaN (math: -inf), lse {+inf,...} = NaN (math: +inf); {-inf,1} = 1 is right. A log-probability of -inf is a plausible input; `arm_logsumexp_dot_prod_*` inherits it. DOC silent
- **Test / evidence:** KNOWN_ISSUE (B-DSF32-5, B-DSF16-3)

## Vendor V56: CMSIS-DSP (**vendor, tier C**)

- **Where:** `Include/dsp/utils.h:221` (`arm_norm_64_to_32u`, called by `arm_div_int64_to_int32` and `arm_mean_q31` paths)
- **What:** For inputs >= 2^62 the 32-bit shifts use counts 32, 33 or -1 (UB, UBSan: "shift exponent 32 is too large"); on x86 the count is masked (2^62+1 returns 0x40000001, definition 0x40000000); target result unverified (a Cortex-M register shift by >= 32 gives 0, so likely 0 for inputs >= 2^63). ARM's pattern has no such input
- **Test / evidence:** KNOWN_ISSUE_UB (B-DSQ31-1) plus UB_HIDDEN variants in the div cases

## Vendor V58: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_var_q31.c:208`, `arm_std_q31.c:152`, `arm_var_q15.c:223`, `arm_std_q15.c:165`
- **What:** `blockSize * (blockSize - 1U)` in uint32 wraps for n > 65536: q31 var of one sample 0.125 plus 65536 zeros at n = 65537 returns 0 (expected 512 LSB); the reviewer's two-level signal gave -679485184; the DOC of q31 allows large n with a scale-down, the q15 DOC gives no maximum n
- **Test / evidence:** KNOWN_ISSUE x4 (B-DSQ31-3 var and std, B-DSQ15-3 var and std)

## Vendor V59: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_var_q15.c:226`
- **What:** DOC: "truncated to 34.15 format ... and then saturated to yield a result in 1.15 format", the code stores `(meanOfSquares - squareOfMean) >> 15` without saturating: {-32768, 32767} has variance 65534 LSB, the library returns -2 (negative variance), intended 32767; `arm_std_q15.c:169` saturates (`__SSAT`)
- **Test / evidence:** KNOWN_ISSUE x2 (B-DSQ15-1)

## Vendor V60: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_mean_q15.c:120-125`, `arm_mean_q7.c:119-121`, `arm_math_memory.h:178` (host-only `read_q7x4`)
- **What:** Left shifts of values past the sign bit / negative values in the unrolled loops (UB; UBSan first reports `arm_math_memory.h:178` for q7); the host result is the intended one (L10); more UB may hide behind the first UBSan stop
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN, 9 q15 and 10 q7 mean cases (B-DSQ15-2, B-DSQ7-1)

## Vendor V61: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_biquad_cascade_df1_q31.c:384, :400, :416, :432`
- **What:** `acc_h << uShift` with a negative `acc_h` in the unrolled loop (blocks of 4 or more samples): UB; the host result is the intended one (L10); more UB may hide behind the first UBSan stop
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN (B-DFBQ-1)

## Vendor V66: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_fir_decimate_fast_q31.c:140-:328` (`(q63_t)acc << 32`), `:158, :172, :223-:226, :339`
- **What:** Left shift of a negative accumulator (UB); host result is the intended one
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN, 13 cases (B-DDECQ-4)

## Vendor V67: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_fir_q15.c:568` via `arm_math_memory.h:105`
- **What:** Without `ARM_MATH_DSP` and blockSize >= 4 the kernel reads 2 bytes behind a state buffer of the documented length numTaps + blockSize - 1 (needs numTaps + blockSize). The result is unaffected. Evidence from an ASan-only probe (T = 4, B = 4, 7-element state)
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN, q15 and fast q15 (B-DFIRQ-3)

## Vendor V68: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_fir_init_q15.c`
- **What:** DOC: "ARM_MATH_ARGUMENT_ERROR: numTaps is not greater than or equal to 4 and even"; without `ARM_MATH_DSP` an odd numTaps (3) returns `ARM_MATH_SUCCESS` (numTaps = 2 is accepted too, and ARM's own test uses it)
- **Test / evidence:** KNOWN_ISSUE (B-DFIRQ-4)

## Vendor V69: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_fir_interpolate_q15.c:657-658` via `arm_math_memory.h:105`
- **What:** The state copy uses `read_q15x2_ia` (`pQ15[1] << 16`, negative sample): UB when phaseLength >= 5; host-only (the target has `__ARM_FEATURE_UNALIGNED` and uses memcpy). ARM's own C1 pattern hits it
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN, 10 cases (B-DDECQ-5)

## Vendor V70: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_conv_fast_q31.c:162, :196, :268, :457`, `arm_correlate_fast_q31.c:493`, `arm_conv_partial_fast_q31.c:186, :501`
- **What:** Left shift of a negative accumulator in the fast q31 conv / correlate / partial kernels (UB, host result is the intended one)
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN, 10 cases (B-DCVQ31-1)

## Vendor V72: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_levinson_durbin_q31.c:89`
- **What:** `r << (shift + 2)` on a negative `r` (UB), reached by ARM's own patterns
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN, 3 cases (B-DCVQ31-3)

## Vendor V73: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_conv_fast_q15.c:172`, `arm_correlate_fast_q15.c:218` (and `arm_conv_partial_fast_q15.c:454`, whose DOC has no saturation sentence: `characterization:`)
- **What:** DOC: "the 2.30 accumulator is right shifted by 15 bits and then saturated to 1.15", the code stores `(q15_t)(sum >> 15)`: -1.0 * -1.0 gives -32768 instead of 32767, and the sum -65534 (not -65536) wraps to 2 instead of -32768. The `_fast_opt` functions saturate correctly
- **Test / evidence:** KNOWN_ISSUE, 6 cases (B-DCVQ15-1)

## Vendor V75: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_conv_fast_q15.c`, `arm_correlate_fast_q15.c`, `arm_conv_partial_fast_q15.c` scalar tails
- **What:** `__SMLAD(*px++, *py--, sum)` on sign-extended q15 multiplies the upper halves too: +1 (2^-30) per product of two negatives (data {-1,-1} x {-32767,-32767} gives {1,1,1} instead of {0,1,0}); the target SMLAD has the same upper-half behaviour. The two-tap A=256 B=2 case of the std conv gives 8192 instead of 8191 on the target branch (scratch measurement)
- **Test / evidence:** KNOWN_ISSUE_UB (B-DCVQ15-3)

## Vendor V76: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_conv_partial_fast_q15.c:679` (stage 3, count not reduced when firstIndex > srcALen)
- **What:** Over-reads 2 bytes past srcA and returns garbage near the end of the window (A=9 B=3 first=10, 1 point); ARM's own C1 windows first = 9 and 7 of the 6x8 test fail on the host (SNR 56.9 < 65 when the stale element is non-zero). The std `arm_conv_partial_q15` has the same defect on the `ARM_MATH_DSP` branch (scratch measurement)
- **Test / evidence:** KNOWN_ISSUE_UB, 5 cases (B-DCVQ15-4)

## Vendor V77: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_math_memory.h:105` (`read_q15x2`), `dsp/none.h:212` (`__PKHBT`), `:222-230` (`__PACKq7`, call sites `arm_conv_opt_q7.c:305`, `arm_conv_partial_opt_q7.c:324`), `:460` (`__SMLAD`), `:503`
- **What:** Left shifts of negative or large packed values on the host SIMD emulation: q15 conv / correlate / partial (std, opt, fast, fast_opt), q7 opt, fir_lattice q15, fir_sparse q15 / q7 (blockSize >= 4), lms q15 (T >= 4). Extends V64. Host result is the intended one
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN, about 190 cases (B-DCVQ15-5/6/7, B-DCVQ7-1/3/4, B-DSPAR-2/3, B-DLATT-2)

## Vendor V78: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_conv_q7.c:331, :336, :345, :628, :633`, `arm_correlate_q7.c:403, :708, :713`
- **What:** `((q31_t) in2 << 16U)` with a negative q15 (UB, plain variants, unrolled loops)
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN (B-DCVQ7-2)

## Vendor V79: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_conv_opt_q15.c:252`, `arm_correlate_opt_q15.c:228`, `arm_conv_fast_opt_q15.c:256`, `arm_correlate_fast_opt_q15.c:233`, `arm_conv_partial_opt_q15.c:268`, `arm_conv_partial_fast_opt_q15.c:267`, `arm_conv_opt_q7.c:263`, `arm_correlate_opt_q7.c:288`, `arm_conv_partial_opt_q7.c:282`
- **What:** The opt functions read 2 bytes past the DOC-sized scratch1 (`read_q15x2`) when min(A,B) is a multiple of 4 and the other length is 1 mod 4 ((4,5), (8,9), (12,13), (16,17)); the result is unaffected. Evidence is an ASan probe (UBSan stops first in the suite)
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN (B-DCVQ15-8, B-DCVQ7-5)

## Vendor V80: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_fir_sparse_q31.c:329-335, :354`
- **What:** `in = *pOut << 1` of a negative value (UB) for every blockSize
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN, 5 cases (B-DSPAR-1)

## Vendor V82: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_lms_q31.c:162, :194-228`, `arm_lms_norm_q31.c:120, :176, :213-247`, `arm_lms_q15.c:150`, `arm_lms_norm_q15.c:168`
- **What:** Left shifts of negative values (`coef << 1U`, `acc_h << uShift`) and `energy << 32` on a negative energy (norm q31 with numTaps = 1, non-negative data after the first sample): UB, host result is the intended one
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN (B-DLMS-1, B-DLMS-2, B-DLMS-4)

## Vendor V86: TinyUSB MIDI class (**vendor, tier B**)

- **Where:** `midi_device.c:288-312` (`tud_midi_n_stream_write`)
- **What:** Real-time messages F8, F9, FA-FF are written with CIN 5 (`35 F8 00 00` on cable 3) instead of CIN 0xF (`3F F8 xx xx`, USB MIDI class definition 1.0 Table 4-2)
- **Test / evidence:** KNOWN_ISSUE (B-MIDI-V1)

## Vendor V87: TinyUSB MIDI class (**vendor, tier B**)

- **Where:** `midi_device.c:155-160` (`tud_midi_n_stream_read`)
- **What:** A reserved packet (CIN 0 or 1) between valid packets makes the function `return 0`: the bytes already copied are not counted (6 bytes read, 0 returned)
- **Test / evidence:** KNOWN_ISSUE (B-MIDI-V2)

## Vendor V88: TinyUSB MIDI class (**vendor, tier B**)

- **Where:** `midi_device.c:263-286`
- **What:** A real-time byte written inside a SysEx (`F0 01 02 F8 03 F7`, also mid-packet `F0 01 F8 02 F7`) becomes SysEx payload instead of its own CIN 0xF packet (MIDI 1.0: real-time messages may interrupt other messages)
- **Test / evidence:** KNOWN_ISSUE (B-MIDI-V3, two cases)

## Vendor V90: ST HAL core (**vendor, tier D**)

- **Where:** `stm32f4xx_hal.c:390` (`HAL_Delay`)
- **What:** The function adds `uwTickFreq` to a delay below `HAL_MAX_DELAY`; with 100 (10 Hz) and a delay of 0xFFFFFFF0 the sum wraps and the call returns after about 100 ms instead of the requested time
- **Test / evidence:** KNOWN_ISSUE (`test_hal_core.cpp`, H8; would take about 2.7 s if fixed)

## Vendor V91: ST HAL PWR (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_pwr.c:221-244` (`HAL_PWR_EnterSLEEPMode`)
- **What:** UM1725 notes of the function: "In Sleep mode, the systick is stopped ..."; the HAL leaves the SysTick enabled (CTRL.TICKINT and ENABLE stay set) while it waits, so the tick interrupt ends the sleep at once
- **Test / evidence:** KNOWN_ISSUE (`test_hal_pwr.cpp`, PW6)

## Vendor V92: ST HAL RCC (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_rcc_ex.c` (strong `HAL_RCC_DeInit`, F407 branch)
- **What:** UM1725: DeInit "doesn't modify the configuration of the Peripheral clocks, LSI, LSE and RTC clocks"; the function clears RCC_CSR.LSION (and sets RMVF), so the LSI is switched off
- **Test / evidence:** KNOWN_ISSUE (`test_hal_rcc_osc.cpp`, RC2)

## Vendor V95: ST HAL DMA (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_dma.c:610` (`HAL_DMA_PollForTransfer`)
- **What:** After a poll timeout the function sets the state to READY and unlocks but leaves the stream enabled: a second poll returns `HAL_ERROR` (NO_XFER) and the next Start writes NDTR, PAR and M0AR while they are write-protected (RM0090 10.5.5-10.5.9), so the stream can be neither re-polled nor restarted. Weaker than V94: UM1725 gives no state after a timeout
- **Test / evidence:** KNOWN_ISSUE (`test_hal_dma_xfer.cpp`, DS7)

## Vendor V96: ST HAL DMA (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_dma_ex.c:100` (`HAL_DMAEx_MultiBufferStart`)
- **What:** The non-IT function returns `HAL_BUSY` when the state is not READY without `__HAL_UNLOCK`, so the handle stays locked and every later call returns `HAL_BUSY` through the lock (`HAL_DMA_Start`, `HAL_DMA_Start_IT` and `HAL_DMAEx_MultiBufferStart_IT` unlock)
- **Test / evidence:** KNOWN_ISSUE (`test_hal_dma_xfer.cpp`, DX1)

## Vendor V97: ST HAL SPI (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_spi.c:935` (`HAL_SPI_Receive`), `:1467` (`Receive_IT`), `:1748` (`Receive_DMA`)
- **What:** For a 2-line master the three functions set `State = BUSY_RX` before they hand the call to `TransmitReceive*`; when that rejects a NULL buffer, `Size` 0 or a locked handle, the state is not restored, the handle stays BUSY_RX and every later call returns `HAL_BUSY`. Oracle is inference (UM1725 gives no state rule for a rejected call); the sibling functions of the same family restore READY
- **Test / evidence:** KNOWN_ISSUE (`test_hal_spi_poll.cpp` 4 cases, `test_hal_spi_irq.cpp`, `test_hal_spi_dma.cpp`)

## Vendor V99: ST HAL SPI (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_spi.c:2455-2466` (`HAL_SPI_IRQHandler`, abort branch, `SPI_DMAAbortOnError` for both streams)
- **What:** After one overrun in `TransmitReceive_DMA` both stream abort callbacks call `HAL_SPI_ErrorCallback`: the user gets two error callbacks for one error
- **Test / evidence:** KNOWN_ISSUE (`test_hal_spi_dma.cpp`, SD3)

## Vendor V101: ST HAL SPI (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_spi.c:1644` (`HAL_SPI_Transmit_DMA`)
- **What:** **Arguable.** When the DMA stream cannot be started the function returns `HAL_ERROR` with `HAL_SPI_ERROR_DMA` but leaves the state BUSY_TX, so later calls return `HAL_BUSY` (same pattern as V97). UM1725 gives no state rule for a failed DMA start
- **Test / evidence:** KNOWN_ISSUE (`test_hal_spi_dma.cpp`, SD1)

## Vendor V102: ST HAL UART (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_uart.h:862,872` (`UART_DIVFRAQ_SAMPLING16` / `_SAMPLING8`), used by `UART_SetConfig` (`stm32f4xx_hal_uart.c:3744-3751`)
- **What:** `HAL_UART_Init` writes a BRR that is not the nearest value: the macros truncate USARTDIV to hundredths before rounding the fraction. RM0090 30.3.4 ("the nearest real number", examples 25.62 to 0x19A, 50.99 with carry): 42 MHz / 115200 baud (the firmware's clock and rate) gives 0x16C, the nearest is 0x16D (baud error about 0.16 % instead of 0.04 %); also 84 MHz / 230400 (0x16C, expected 0x16D), 42 MHz / 921600 (0x2D, expected 0x2E), OVER8 = 1 at 50 MHz / 460800 (0xD4, expected 0xD5). Arguable add-on: the exact tie 12 MHz / 38400 (0x138 / 0x270; RM0090 Tables 134 / 135 print the rounded-up 0x139 / 0x271)
- **Test / evidence:** KNOWN_ISSUE (`test_hal_uart_init.cpp`, UI2: 4 cases + 2 tie cases)

## Vendor V103: ST HAL UART (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_uart.c:1264` (`HAL_UART_Receive`), `:3217` (`UART_WaitOnFlagUntilTimeout`)
- **What:** **Arguable.** An overrun during a polling `Receive` (timeout not `HAL_MAX_DELAY`) ends the call at once with `HAL_UART_ERROR_ORE` set, but the function maps the wait's `HAL_ERROR` to `HAL_TIMEOUT`, so the caller cannot tell an overrun from a timeout. UM1725 gives no return code for an overrun; the charter's intended value is `HAL_ERROR`
- **Test / evidence:** KNOWN_ISSUE (`test_hal_uart_poll.cpp`, UP6)

## Vendor V104: ST HAL UART (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_uart.c:1543-1568` (`HAL_UART_DMAStop`)
- **What:** **Arguable.** `DMAStop` acts only while DMAT / DMAR is set, and `HAL_UART_DMAPause` clears exactly those bits, so `Transmit_DMA` / `Receive_DMA`, `DMAPause`, `DMAStop` returns `HAL_OK` but leaves the state BUSY_TX / BUSY_RX and the stream enabled. UM1725 says only "Stops the DMA Transfer" (intent inferred)
- **Test / evidence:** KNOWN_ISSUE (`test_hal_uart_dma.cpp`, UD4: Tx and Rx case)

## Vendor V105: ST HAL UART (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_uart.c:2144` (`HAL_UART_Abort_IT`)
- **What:** **Arguable, low (contrived trigger).** With Tx and Rx DMA running and the Rx stream stopped behind the UART's back (DMAR still set), a failing `HAL_DMA_Abort_IT` on Rx sets `AbortCplt = 1` again while the Tx abort is pending: `HAL_UART_AbortCpltCallback` runs at once and a second time when the Tx DMA abort finishes. UM1725: "At abort completion, call user abort complete callback"
- **Test / evidence:** KNOWN_ISSUE (`test_hal_uart_abort.cpp`, UA2)

## Vendor V106: ST HAL I2C (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_i2c.c:4964-4970,5019-5023` (`HAL_I2C_ER_IRQHandler`, `I2C_ITError`)
- **What:** BERR in controller mode aborts the interrupt transfer: the handler clears BERR and calls `I2C_ITError` (ErrorCallback, state READY, interrupts off). ES0182 2.10.1: "Detection of bus error has no effect on the I2C-bus transfer in controller mode ... the ongoing transfer can be handled normally"
- **Test / evidence:** KNOWN_ISSUE (`test_hal_i2c_irq.cpp`, IT3)

## Vendor V107: ST HAL I2C (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_i2c.c:4836` (`HAL_I2C_EV_IRQHandler` BTF branch)
- **What:** **Arguable (depends on interrupt latency above one byte time).** `Mem_Write_IT` with a 16-bit memory address: when BTF is set after the high address byte, the BTF branch tests `State == BUSY_TX` (true for a memory write) and calls `I2C_MasterTransmit_BTF`, which writes the first data byte instead of the low address byte; the memory branch is reached only in BUSY_RX. Expected bus: S A0 12 34 data P, actual S A0 12 data P
- **Test / evidence:** KNOWN_ISSUE (`test_hal_i2c_irq.cpp`, IT1)

## Vendor V112: ST HAL timers (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_hal_tim.c` (`HAL_TIM_Base_Start_DMA` 546-567, `HAL_TIM_OC_Start_DMA` / `HAL_TIM_PWM_Start_DMA`, `HAL_TIM_DMABurst_MultiWriteStart` / `MultiReadStart`, IC / Encoder `Start_DMA`), `stm32f4xx_hal_tim_ex.c` (`HAL_TIMEx_OCN_Start_DMA`, `PWMN_Start_DMA`, `HallSensor_Start_DMA`)
- **What:** The start function sets the handle, channel or burst state to BUSY and returns `HAL_ERROR` when `HAL_DMA_Start_IT` fails without restoring READY, so every later start returns `HAL_BUSY` / `HAL_ERROR`. UM1725 gives no state rule for a failed DMA start; the intent follows the state definitions and the SPI precedent V101
- **Test / evidence:** KNOWN_ISSUE (`test_hal_tim_base.cpp` TB2, `test_hal_tim_ic.cpp` TI, `test_hal_tim_cfg.cpp` TC4, `test_hal_tim_exn.cpp` TN3)

## Vendor V113: ST HAL ADC (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_adc.c:405-454` (`HAL_ADC_DeInit`)
- **What:** UM1725 7.2.7: "Deinitializes the ADCx peripheral registers to their default reset values". The HAL only clears ADON, calls `HAL_ADC_MspDeInit`, clears ErrorCode, sets RESET and unlocks: CR1 (SCAN, RES), CR2 (ALIGN, CONT, DMA) and SQR1 (length) keep their values; RM0090 13.13 gives reset value 0 for all three (a board could reset them in `MspDeInit`)
- **Test / evidence:** KNOWN_ISSUE (`test_hal_adc_base.cpp`, AB1)

## Vendor V115: ST HAL ADC (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_adc_ex.c:884-893` (`HAL_ADCEx_InjectedConfigChannel`)
- **What:** UM1725 8.1.1 `ExternalTrigInjecConv`: "must be modified when ADC is disabled ... If ADC is enabled, this parameter setting is bypassed without error reporting". The HAL writes JEXTSEL / JEXTEN whatever ADON is
- **Test / evidence:** KNOWN_ISSUE (`test_hal_adc_ex.cpp`, AX1)

## Vendor V116: ST HAL ADC (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_adc_ex.c:912-921` (`HAL_ADCEx_InjectedConfigChannel`)
- **What:** UM1725 8.1.1 `InjectedDiscontinuousConvMode`: discarded when continuous mode is enabled and when the sequencer (scan) is disabled. The HAL sets JDISCEN unconditionally. Two cases
- **Test / evidence:** KNOWN_ISSUE (`test_hal_adc_ex.cpp`, AX1)

## Vendor V119: ST HAL USB (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_ll_usb.c` end of `USB_SetCurrentMode`
- **What:** `if (ms == HAL_USB_CURRENT_MODE_MAX_DELAY_MS) return HAL_ERROR` after the loop: the mode reached at the last poll is reported as an error. RM0090 gives no timing (200 ms is HAL policy)
- **Test / evidence:** KNOWN_ISSUE (`test_hal_pcd_init.cpp`, PI4)

## Vendor V120: ST HAL USB (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_ll_usb.c:606-612` (`USB_ActivateEndpoint`)
- **What:** `HAL_PCD_EP_Open(0x80, 8, CTRL)` leaves DIEPCTL0.MPSIZ at 0 (64 bytes): MPSIZ is written only while USBAEP = 0 and endpoint 0 always has USBAEP = 1; RM0090 DIEPCTL0 MPSIZ 11 = 8 bytes
- **Test / evidence:** KNOWN_ISSUE (`test_hal_pcd_ep.cpp`, PE1)

## Vendor V121: ST HAL USB (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_ll_usb.c:606-612, 619-625`
- **What:** Re-Open of an active endpoint keeps the old type and size (same USBAEP guard); RM0090 "Endpoint activation" allows reconfiguring an active endpoint
- **Test / evidence:** KNOWN_ISSUE (`test_hal_pcd_ep.cpp`, PE1)

## Vendor V122: ST HAL USB (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_ll_usb.c:699-702` (`USB_DeactivateEndpoint`), `:967-968` (`USB_EPStopXfer`)
- **What:** Close / Abort of an armed OUT endpoint sets SNAK and EPDIS without the Global OUT NAK (DCTL.SGONAK) that RM0090 "Disabling an OUT endpoint" step 1 requires. Two cases
- **Test / evidence:** KNOWN_ISSUE (`test_hal_pcd_ep.cpp`, PE1, PE4)

## Vendor V123: ST HAL USB (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_pcd.c:1965` (`HAL_PCD_EP_SetStall`), `:2008` (`HAL_PCD_EP_ClrStall`)
- **What:** Only numbers greater than `dev_endpoints` are rejected: with 4 endpoints (0-3), endpoint 4 returns HAL_OK and writes reserved registers. Two cases
- **Test / evidence:** KNOWN_ISSUE (`test_hal_pcd_ep.cpp`, PE4)

## Vendor V124: ST HAL USB (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_hal_pcd.c:2072, 2076` (`HAL_PCD_EP_Flush`)
- **What:** The results of `USB_FlushTxFifo` / `USB_FlushRxFifo` are cast to void and HAL_OK is returned: a flush timeout is hidden (UM1725: "Return values: HAL status")
- **Test / evidence:** KNOWN_ISSUE (`test_hal_pcd_ep.cpp`, PE4)

## Vendor V125: ST HAL USB (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_hal_pcd.c:1479` (`HAL_PCD_IRQHandler`, incomplete isochronous OUT)
- **What:** `(RegVal & (0x1U << 16)) == (hpcd->FrameNumber & 0x1U)` compares the unshifted bit with the frame parity, so it matches only EONUM 0 on an even frame; RM0090 34.17.5: "An endpoint transfer is incomplete if ... EONUM bit (in OTG_FS_DOEPCTLx) = SOFFN[0] (in OTG_FS_DSTS) ... EPENA = 1"
- **Test / evidence:** KNOWN_ISSUE (`test_hal_pcd_irq.cpp`, PQ3)

## Vendor V128: ST HAL UART (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_uart.c:3359-3372` (`UART_DMAAbortOnError`), `:3529-3561` (`UART_Transmit_IT`)
- **What:** An Rx DMA error abort clears `TxXferCount` as well as `RxXferCount`. With an unrelated `HAL_UART_Transmit_IT` running (gState BUSY_TX, TXEIE set) the HAL keeps the Tx process alive but its counter is 0, so the next Tx interrupt does `--TxXferCount` from 0 to 65535 and goes on sending past the buffer (the first byte is read in bounds; later reads are source-inferred, not executed). Found by the Codex review (P12C-T4B-001), re-checked in the source and reproduced
- **Test / evidence:** KNOWN_ISSUE (`test_hal_review_vendor_uart.cpp`, RV20)
