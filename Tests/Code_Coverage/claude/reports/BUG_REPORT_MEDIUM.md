# OpenFFBoard firmware: medium severity suspected bugs

Generated 2026-11-19 from the bug table in `Tests/Code_Coverage/claude/plans/PROGRESS.md`. The full list is `BUG_REPORT.md` (unchanged).

Severity is derived from the description text (keyword rules), so it is a triage aid, not a verdict. Medium: Wrong results or unchecked input that neither stops the device nor moves the motor.

89 items: 2, 13, 20, 22, 36, 38, 43, 45, 51, 55, 59, 61, 70, 72, 78, 84, 90, 95, 99, 101, 102, 103, 109, 111, 119, 120, 127, 129, 130, 142, 144, 145, 150, 152, 159, 162, 170, 176, 186, 190, 191, 194, 196, 197, 202, 203, 204, 209, 211, 212, 213, 222, 223, 226, V2, V10, V16, V18, V23, V25, V26, V27, V32, V37, V38, V39, V46, V47, V50, V51, V54, V63, V64, V65, V71, V74, V83, V84, V85, V93, V94, V98, V100, V109, V114, V117, V118, V126, V127

## Vendor 2: CMSIS-DSP (vendor, Tier C) reached from EffectsCalculator

- **Where:** `Libraries/CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.c:223`, called from `EffectsCalculator.cpp:695` with `blockSize = 1`
- **What:** `while( *pXq <= x[i+1] && blkCnt > 0 )` dereferences `pXq` before checking `blkCnt`, so it reads one float past the 1-element input (`interp_time`, a stack variable). Bug is in the vendor library; the firmware call is valid per the API. On target it reads one adjacent stack word; the result is not used, so likely harmless
- **Test / evidence:** ASan stack-buffer-overflow (GCC and clang), test cases `reconstruction of a constant signal is constant` and `reconstruction of a step never leaves the range of the input values and reaches the target`. Not visible without ASan, not in DSP-OFF build. Possible firmware-side workaround (not to be applied by me): pass a 2-element buffer **Note (2026-11-02):** only "reconstruction of a constant signal is constant" triggers ASan today (GCC 16 and clang 23); "reconstruction of a step ..." passes without a report but stays excluded on purpose (my decision) and is highlighted under ATTENTION in the full test report of sanitizer builds.

## Bug 13: CommandHandler

- **Where:** `CommandHandler.cpp:533-561`
- **What:** The override placeholder from `overrideCommandFlags_INT` lacks `CMDFLAG_EXTOVERRIDE`, so `registerCommand_INT` treats the id as "already present": the real command is never registered (placeholder keeps `cmd == nullptr`; `getCommandFromName` would compare nullptr with a string = UB)
- **Test / evidence:** KNOWN_ISSUE "a command registered after a flag override gets its name and the masked flags". Confirmed in source

## Bug 20: CDCcomm

- **Where:** `CDCcomm.cpp:69`
- **What:** `std::min<uint16_t>(reply->length(), bufferRemaining)` truncates the length to 16 bit. Review r3: after a partial write the remainder can be exactly 65536 (any multiple of 65536), which truncates to 0; the retry writes 0 bytes without flush, no tx-complete follows, `cdcSems[0]` is never given back and every later `cdcSend` blocks: **CDC output dead** **Latent:** unreachable on every shipped target (`malloc` -> `pvPortMalloc`, `cppmain.cpp:138`; `configTOTAL_HEAP_SIZE` 0xffff, so a reply of 65536+ bytes can never be allocated).
- **Test / evidence:** KNOWN_ISSUE "a reply longer than 65535 bytes is sent in fifo sized parts" (`claude/cases/test_cdc.cpp:166`)

## Bug 22: UART

- **Where:** `UART.cpp:97, 104, 116-123`, used at `189-191`
- **What:** `abortReceive()` and a failed `receive_IT/_DMA` never clear `rxbuf_t`: the next single-byte completion armed by `registerInterrupt()` passes the old (possibly dangling) user buffer to `uartRcv` instead of the received byte (found by review r3)
- **Test / evidence:** 2 KNOWN_ISSUE tests in `test_uart.cpp` (after abort, after a failed receive)

## Bug 36: Axis

- **Where:** `Axis.cpp:656` `getLastScaledEnc`
- **What:** Clip in float: 0x7fffffff becomes 2^31; position >= 1.0 converts out of range to int32 (UB). Host gives INT_MIN (opposite end in the HID report), ARM saturates. UBSan does not check float-cast-overflow
- **Test / evidence:** KNOWN_ISSUE_UB "an axis beyond the end of its range reports the largest gamepad position" (host-vs-target)

## Bug 38: SelectableInputs

- **Where:** `SelectableInputs.cpp:44-46, 76-78`
- **What:** `addbtn`/`addain` duplicate check compares the class id (`getInfo().id`) with the selection id: never matches, a second identical source is created
- **Test / evidence:** 2 KNOWN_ISSUE (`test_ffbhidmain.cpp`)

## Bug 43: global_callbacks

- **Where:** `global_callbacks.cpp:340` `tud_hid_report_complete_cb(..., uint8_t len)` vs `hid_device.h:146` (`uint16_t len`)
- **What:** Same kind of mismatch: TinyUSB never calls the firmware version. No effect today (both `transferComplete` implementations are empty)
- **Test / evidence:** Comment only (nothing observable)

## Bug 45: UART / global_callbacks

- **Where:** no `HAL_UART_ErrorCallback`
- **What:** On an overrun or any error during DMA/IT reception the ST HAL aborts the reception (`UART_EndRxTransfer`, `stm32f4xx_hal_uart.c:2410-2457`) and calls only the weak no-op: `UARTPort` never re-arms, the UART command interface stops receiving
- **Test / evidence:** Code reading; not testable until the UART fake can inject errors (P5/P7)

## Bug 51: usb descriptors

- **Where:** `usb_hid_ffb_desc.h:757`, `usb_descriptors.h`, `FFBWheel.cpp:40`
- **What:** The 1-axis 32-bit build (`FFBWHEEL_USE_1AXIS_DESC` + `HIDAXISRES_USE_32B_DESC`) does not compile: wrong size macro name, `usb_cdc_hid_conf_1axis_32b` not declared
- **Test / evidence:** Build (not a shipped configuration)

## Bug 55: SPIButtons

- **Where:** `SPIButtons.h:81` `spi_buf[4]`, `SPIButtons.cpp:125-129, 174`
- **What:** `bytes` can be up to 9 (TM mode above 29 buttons, PISOSR above 32): the DMA writes past the buffer and the `memcpy` reads past it. In `SPI_Buttons_1/2` the overflow lands in 5 bytes of tail padding (host and target), so it is silent today; any derived class whose members reuse the padding is corrupted
- **Test / evidence:** KNOWN_ISSUE_UB "a read of 8 bytes stays inside the receive buffer of the source" (derived class with a guard array)

## Bug 59: CanAnalog

- **Where:** `CanAnalog.cpp:110-121`
- **What:** `extId` never checked: an extended frame whose low 11 bits equal the CAN id (accepted by another filter on the port) becomes axis values
- **Test / evidence:** KNOWN_ISSUE (`test_can_analog.cpp`)

## Bug 61: CanAnalog

- **Where:** `amount` command
- **What:** Does not re-check the id: `canid` 0x7FF then amount 8 needs a second frame id 0x800, axes 5-8 can never be received
- **Test / evidence:** KNOWN_ISSUE

## Bug 70: ShifterAnalog

- **Where:** `ShifterAnalog.cpp:243-246, 55-56`
- **What:** `xchan`/`ychan` unchecked: channel 0 reads `ADC1_BUF[-1]`, channels past `ADC1_CHANNELS` read past the buffer (F407VG >= 9, F407VG_DISCO >= 6, F411RE >= 11)
- **Test / evidence:** KNOWN_ISSUE (the OOB read itself not executed)

## Bug 72: CanButtons

- **Where:** `CanButtons.cpp:137-138`
- **What:** Extended (29-bit) frames whose low 11 bits equal the CAN id set the buttons (`extId` not checked; same as bug 59)
- **Test / evidence:** KNOWN_ISSUE

## Bug 78: EncoderSSI

- **Where:** `EncoderSSI.cpp:110`
- **What:** AMT23 check accepts a frame with only one check bit right (`if(parity)` instead of `== 3`)
- **Test / evidence:** KNOWN_ISSUE "an AMT23 frame with one wrong check bit is counted as an error and keeps the position"

## Bug 84: EncoderBissC

- **Where:** `EncoderBissC.cpp:252` (effects 238, 188, 52, `saveFlash` 91)
- **What:** `bits` not range-checked: >= 32 shifts `getCpr` by >= 32 (`EncoderBissC.cpp:238`, `1<<bits`; the test uses 40, and its "above 32" still accepts 32, which is UB), >= 29 overflows the mask at 188, 0 gives `1 << -1`; `saveFlash` keeps 5 bits (stored 32 also hits the `getCpr` UB)
- **Test / evidence:** KNOWN_ISSUE_UB "bits set above 32 is not accepted", KNOWN_ISSUE "bits set to 0 is not accepted"

## Bug 90: EncoderBissC

- **Where:** `EncoderBissC.cpp:253-255`
- **What:** `bits` above 24 lowers `spiSpeed` to 2 but never calls `configSPI`: the port keeps 2.6 MHz
- **Test / evidence:** KNOWN_ISSUE "bits set above 24 lowers the SPI clock of the encoder SPI port to preset 2"

## Bug 95: EncoderLocal

- **Where:** `EncoderLocal.cpp:80, 84-85`
- **What:** Offset uses the timer count only, ignoring the overflow sum in `pos`: cpr 4000, +40100 counts restores 1436 instead of -100 (mod 4000). Shows only when cpr does not divide 65536, where bug 93 also applies: the case flips only with 93 and 95 fixed
- **Test / evidence:** KNOWN_ISSUE "zeroing more than half a timer range after the index restores the index position modulo a turn"

## Bug 99: SPIPort / I2CPort

- **Where:** `SPI.cpp:189-192`, `I2C.cpp:265-268`
- **What:** `takeSemaphore` ignores a failed `TakeFromISR`: a transfer started in an ISR while the port is taken goes ahead (SPI: second CS asserted during the first transfer, HAL_BUSY, completion goes to the wrong device; I2C: HAL_BUSY and bug 89). Latent
- **Test / evidence:** KNOWN_ISSUE "an SPI/I2C transfer started in interrupt context while the port is taken does not reach the bus"

## Bug 101: MtEncoderSPI

- **Where:** `MtEncoderSPI.cpp:221`, `227` (comment 229)
- **What:** `overspeed` reads raw bit 4 of byte 3 after the shift, i.e. ANGLE[0] per the layout comment. Latent: the flag is never read (NEEDS REVIEW 114)
- **Test / evidence:** code reading only

## Bug 102: EncoderSSI

- **Where:** `EncoderSSI.cpp:212`
- **What:** `bits` command not range-checked (40 accepted; `1<<bits` at 111/126/133/179 undefined)
- **Test / evidence:** KNOWN_ISSUE "the bits command keeps the resolution within 1 to 32 bits"

## Bug 103: EncoderSSI

- **Where:** `EncoderSSI.cpp:44` with `126`, `179`
- **What:** 31 and 32 bits restored from flash (layout allows 1..32): `(1<<31)-1` overflows, `1<<32` in `getCpr` (UBSan: shift exponent 32). x86 masks the shift count, ARM LSL by >= 32 gives 0
- **Test / evidence:** code reading + UBSan finding (the raw read-length case stops at 30 bits)

## Bug 109: CANPort2B

- **Where:** `CANPort2B.cpp:264` (`stm32f4xx_hal_can.c:905-908`)
- **What:** `CAN_FilterTypeDef sFilterConfig;` never sets `SlaveStartFilterBank`; the F407 HAL writes it into FMR CAN2SB, so stack garbage decides which banks belong to CAN1
- **Test / evidence:** code reading (not deterministic; the fake does not apply CAN2SB)

## Bug 111: CANPort2B

- **Where:** `CANPort2B.cpp:351` (check `:137`)
- **What:** `speed=257` is truncated to uint8_t 1 and selects 100k
- **Test / evidence:** KNOWN_ISSUE "a speed preset above 255 is ignored like any other preset that does not exist"

## Bug 119: CanBridge

- **Where:** `CanBridge.cpp:172-177`
- **What:** The search for the next 0xF1 reads `Buf[*Len]`, one byte past the received data, for the last command (ASan heap-buffer-overflow with `--no-skip`; no visible effect)
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN "a GVRET command at the end of the received data is answered without reading past the data"

## Bug 120: CanBridge

- **Where:** `CanBridge.cpp:218-238`
- **What:** GVRET command 5 never stores the speed in `conf1.speed`: command 6 always reports 500000 (also after `main.spd`)
- **Test / evidence:** KNOWN_ISSUE "the CAN config reply reports the speed set by the host"

## Bug 127: SerialFFB

- **Where:** `SerialFFB.cpp:106-108`
- **What:** The "Compatibility for conditional effects" statements only read the coefficients and never assign the magnitude: `mag` has no effect on a conditional effect (assumed intent: magnitude to both coefficients)
- **Test / evidence:** KNOWN_ISSUE "setting the magnitude of a spring sets its coefficients for compatibility"

## Bug 129: CanBridge

- **Where:** `CanBridge.cpp:185-193`
- **What:** The GVRET build-frame length check uses the whole transfer (`*Len >= 8`), not the bytes left for this command: id, bus, length and data are read past `*Len` (`F1 0C F1 00 23 01 00 00` sends a DLC-0 frame with id 0x123 from bytes never received; stale `cdcbuf` on the target)
- **Test / evidence:** KNOWN_ISSUE "a build-frame command cut at the end of a transfer sends no frame"

## Bug 130: CanBridge

- **Where:** `CanBridge.cpp:120`, `:282`
- **What:** The return value of `tud_cdc_n_write` is ignored: with a nearly full CDC Tx FIFO only part of a GVRET frame/reply is sent and the host loses frame boundaries
- **Test / evidence:** KNOWN_ISSUE "a GVRET frame to the host is not cut when the CDC transmit buffer is nearly full"

## Bug 142: TMC4671 driver: State machine

- **Where:** `TMC4671.cpp:266-269, 461`
- **What:** A chip that never answers is never reported: `tmcCommunicationError` is only raised if the second ping in `initialize()` fails right after a successful one
- **Test / evidence:** KNOWN_ISSUE "[R14] a chip that never answers is reported ... after 5 s" (B-T4)

## Bug 144: TMC4671 driver: PWM

- **Where:** `TMC4671.cpp:2477-2482`
- **What:** `setPwmFreq` converts `4*clk/f-1` to uint16 before the clip: undefined for f < 1526 Hz (x86 wraps, ARM saturates so the target is unaffected)
- **Test / evidence:** KNOWN_ISSUE_UB [R43] (B-T6)

## Bug 145: TMC4671 driver: ABN encoder

- **Where:** `TMC4671.cpp:898`
- **What:** The index clear flag is written to bit 9 of ABN_DECODER_MODE 0x25; the datasheet `cln` bit is bit 8 (DS 0x25 p86, bit 9 undefined)
- **Test / evidence:** KNOWN_ISSUE 2 cases [R50] (B-T7)

## Bug 150: TMC4671 driver: Control

- **Where:** `TMC4671.cpp:2102-2113`
- **What:** `rampFlux` reads the start flux as uint16: from a negative flux (field weakening) it ramps the wrong way
- **Test / evidence:** KNOWN_ISSUE [R66] (B-T12)

## Bug 152: TMC4671 driver: State machine

- **Where:** `TMC4671.cpp:643-650, 484-491, 425-434`
- **What:** Undervoltage recovery never clears the `undervoltage` warning and does not restore the motion mode (low practical effect: `turn()` re-selects torque)
- **Test / evidence:** KNOWN_ISSUE 2 cases [R75] (B-T14)

## Bug 159: TMC4671 driver: Flash

- **Where:** `TMC4671.cpp:2716`
- **What:** `restoreEncHallMisc` assigns `velocity_P` twice and never restores `position_P`
- **Test / evidence:** KNOWN_ISSUE [R113] (B-T21)

## Bug 162: TMC4671 driver: Biquad filters

- **Where:** `TMC4671.cpp:2207, 2228, 2249, 2270`
- **What:** The biquad enable is written to bit 0 of CONFIG_DATA; DS §7.2 0x4D p93 defines `biquad_*_enable` as bit 31: by the datasheet the flux filter and torque filter are never enabled
- **Test / evidence:** KNOWN_ISSUE 4 cases [R25] [R47] (B-T24), question Q4

## Bug 170: ODrive driver: Connection

- **Where:** `ODriveCAN.cpp:205-212`
- **What:** `connected` is true during the first 2 s after boot without any frame (`lastCanMessage` starts at 0); encoder estimates are requested from a node never seen
- **Test / evidence:** KNOWN_ISSUE 2 cases [O31] (B-O2)

## Bug 176: ODrive driver: Encoder

- **Where:** `ODriveCAN.cpp:264`
- **What:** `getPos()` converts `65535.0f * pos` to int32: undefined beyond about 32768 turns (x86 INT_MIN, ARM saturates)
- **Test / evidence:** KNOWN_ISSUE_UB [O44] (B-O8)

## Bug 186: VESC driver: State machine

- **Where:** `VescCAN.cpp:676-678, 358-369`
- **What:** An unsolicited PONG sets state PONG, which has no exit: no more requests, the driver never becomes ready again
- **Test / evidence:** KNOWN_ISSUE 2 cases [V42] (B-V8)

## Bug 190: VESC driver: Commands

- **Where:** `VescCAN.cpp:303`
- **What:** `pos` command converts turns * 1e9 to int32: undefined beyond 2.147 turns (x86 INT_MIN, ARM saturates)
- **Test / evidence:** KNOWN_ISSUE_UB [V71] (B-V12)

## Bug 191: VESC driver: Members

- **Where:** `VescCAN.cpp`, `VescCAN.h:117-143`
- **What:** `lastTorque`, `voltage`, `vescErrorFlag`, `buffer_rx` are never initialised (read by `turn`, `Run`, the commands, the CRC over stale bytes; heap reuse made a partial reply decode during writing)
- **Test / evidence:** code reading, nondeterministic, no test

## Bug 194: TMCDebugBridge: TMCL checksum

- **Where:** `TMCDebugBridge.cpp:143-146`
- **What:** A frame with a wrong checksum is dropped silently. TMCL reference v2.22 §2.2 / §2.2.1 (and TOSV / LEV manuals): every command is answered, wrong checksum is status 1
- **Test / evidence:** KNOWN_ISSUE (B-DTDF-1)

## Bug 196: TMCDebugBridge: unknown commands

- **Where:** `TMCDebugBridge.cpp:133-231`
- **What:** A command number without a branch (e.g. 200, 9, 11, 135, 137, 147, 149) is swallowed: no reply and not passed to the text parser; TMCL §2.2: status 2 (invalid command)
- **Test / evidence:** KNOWN_ISSUE (B-DTDF-3)

## Bug 197: TMCDebugBridge: command 136

- **Where:** `TMCDebugBridge.cpp:197-226`
- **What:** Command 136 (get firmware version) with a type other than 0 / 1 is dropped silently; TMCL §2.2.1 status 3 (wrong type)
- **Test / evidence:** KNOWN_ISSUE (B-DTDF-4)

## Bug 202: MotorPWM: `freq set` range

- **Where:** `MotorPWM.cpp:123-163, 254-255`
- **What:** `freq set` with a value above 3 is ignored but answers `OK` (intended: `ERR`, speed unchanged); `cmd.val` (int64) is cast to `uint8_t`, so 256 selects speed 0, -1 selects 255
- **Test / evidence:** KNOWN_ISSUE (B-PWM-2)

## Bug 203: MotorPWM: `mode set` and stored mode range

- **Where:** `MotorPWM.cpp:216, 237-240, 273-274`
- **What:** `mode set` with a value above 3 stores the invalid mode and answers `OK`; `turn` then writes nothing and the output stays frozen at its last value; a stored flash word with mode 4..15 (`var & 0xf`) does the same; int64 truncation as for `freq`
- **Test / evidence:** KNOWN_ISSUE (B-PWM-3)

## Bug 204: MotorPWM: CENTERED overflow

- **Where:** `MotorPWM.cpp:94-95`
- **What:** `(pval * period) / 0xffff` is a signed 32-bit product: at speed LOW (period 56000, the default mode and speed) it overflows for power above 5580 (+10000 gives CCR 4294938304 instead of 36544 on the host; +32767 gives 4294957759 instead of 55999, about 100 % duty). Undefined behaviour on host and target. PWM_DIR / DUAL (max 32768 x 56000) and CENTERED at the faster speeds fit
- **Test / evidence:** KNOWN_ISSUE_UB (B-PWM-4)

## Bug 209: SimpleMotion: rejected reply

- **Where:** `MotorSimplemotion.cpp:381-411`
- **What:** When the receive path rejects a reply (wrong CRC-16 or a reply from another device address) it calls `resetBuffer()`, which clears `waitingReply`: the busy wait of `readParameter` / `writeParameter` then ends as if the reply had arrived, copies the stale `replyvalues[]` and returns true (intended false, charter R5.3)
- **Test / evidence:** KNOWN_ISSUE (B-SM-5, 4 cases)

## Bug 211: RMD: port release

- **Where:** `RmdMotorCAN.cpp:61-64`
- **What:** The destructor removes the filter and stops the motor but never calls `port->freePort()`, although the constructor calls `takePort()` (`ODriveCAN` and `VescCAN` do release): the CAN port keeps one user per driver ever created and is never stopped. Owner question 44
- **Test / evidence:** KNOWN_ISSUE (B-RMD-1)

## Bug 212: RMD: CAN id range

- **Where:** `RmdMotorCAN.cpp:340-344`
- **What:** `canid set` stores `(uint8_t) value` unchecked: 0 is accepted (frames to 0x140, not a motor id; SP: ID 1..32), 256 becomes 0 (261 becomes 5); 33..255 are clamped to 32 by `setCanFilter` (`characterization:`). Owner question 45
- **Test / evidence:** KNOWN_ISSUE (B-RMD-2, 2 cases)

## Bug 213: RMD: maxtorque range

- **Where:** `RmdMotorCAN.cpp:102, 214, 350`
- **What:** `maxtorque` accepts up to 65535 (65536 wraps to 0 with status OK) although the header says "max 0x7fff"; the flash word masks it (40000 gives 7232); `turn(0x7fff)` converts a float above 32767 to `int16_t` (undefined; the host gives -25536). Owner question 46
- **Test / evidence:** KNOWN_ISSUE (B-RMD-3, 4 cases) + KNOWN_ISSUE_UB (1 case)

## Bug 222: MidiHandler: CIN ignored

- **Where:** `MidiHandler.cpp:24-31`
- **What:** The code index number is never looked at: the single byte packets of an unparsed stream (CIN 0xF, UM Table 4-1 Note 2: `0F 90 00 00`) are decoded as events (a `noteOff(0, 0, 0)` for the status byte). Owner question 58
- **Test / evidence:** KNOWN_ISSUE (B-MIDI-4)

## Bug 223: MidiMain: `range` and `power` unchecked

- **Where:** `MidiMain.cpp:102, 116, 160-165`
- **What:** Both commands accept any 32 bit value: `range` above 32767 makes `(int16_t)(sine * movementrange)` an out of range float to int conversion (undefined; x86 keeps the low 16 bits), `power` above 32767 reaches `setUdUq(int16_t, ...)` as a negative voltage (wrap-around, defined). Owner question 59
- **Test / evidence:** KNOWN_ISSUE_UB + KNOWN_ISSUE (B-MIDI-5)

## Bug 226: MidiMain: control change 121

- **Where:** `MidiMain.cpp:144`
- **What:** Control change 121 (Reset All Controllers) clears the held notes of the channel like 120 and 123. MIDI 1.0 Detailed Specification p. 25: it resets the controllers (continuous and switch controllers, pitch bend, pressures); held notes are released by All Notes Off (123). Arguable (the spec does not say in words that notes survive). The pitch bend reset is not done either, but cannot be observed on its own (the notes are cleared first and a bend lives only inside existing notes, B-MIDI-2). From the Codex review NP17-001
- **Test / evidence:** KNOWN_ISSUE (B-MIDI-8, arguable)

## Vendor V2: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `cthread.cpp:184-187`
- **What:** `~Thread` of a never-started thread calls `vTaskDelete` with an uninitialised handle (NULL deletes the calling task on FreeRTOS). Also on target
- **Test / evidence:** Code reading; no test (UB)

## Vendor V10: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `cworkqueue.cpp:78`, `:96`
- **What:** The result of `WorkerThread->Start()` is ignored: if `xTaskCreate` fails, work is queued but never runs and `~WorkQueue` waits forever on `ThreadComplete`. Latent
- **Test / evidence:** KNOWN_ISSUE "a work queue whose worker thread can not be created reports the failure" (`claude/cases_rtos/test_cworkqueue.cpp`)

## Vendor V16: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `cmem_pool.cpp:136-143` (also `:106-119`, `AddMemory :172-204`); `mem_pool.hpp:158-159/178-179`, `:171`
- **What:** `MemoryPool` never aligns the start of its memory: items are aligned to `alignment` only if the memory passed in (or malloc'd: heap_4 aligns to 8 on the F407) already is, although the header promises it; `mem_pool.hpp:171` also says "does not throw" for a constructor that throws Latent: no firmware user of `MemoryPool`.
- **Test / evidence:** KNOWN_ISSUE "the items of a pool are aligned to the requested alignment" (detects both an aligning and a rejecting fix)

## Vendor V18: FreeRTOS wrappers (**vendor, tier A**)

- **Where:** `cmem_pool.cpp:106`, `AddMemory :172`
- **What:** The int product `ItemSize * itemCount` overflows (UB): `1431655768 * 3` wraps to `malloc(8)` with 3 items beyond the block, on the host and on the 32-bit target Latent: no firmware user of `MemoryPool`.
- **Test / evidence:** KNOWN_ISSUE_UB "a pool whose total size does not fit in an int is rejected with MemoryPoolMallocException", "AddMemory with a total size that does not fit in an int is rejected with MemoryPoolMallocException"

## Vendor V23: TinyUSB (**vendor, tier B**)

- **Where:** `usbd.c:901-953`
- **What:** Standard endpoint requests are not checked against the configuration: GET_STATUS of endpoint 0x03 (exists in the core, not in the configuration) answers "not halted"; SET/CLEAR_FEATURE on an endpoint without a driver (0x03, EP0, any endpoint in the address state) gets no status stage, so the host NAK-times out (9.4.1, 9.4.5, 9.4.9)
- **Test / evidence:** KNOWN_ISSUE (3 cases) "... (V23)"

## Vendor V25: TinyUSB (**vendor, tier B**)

- **Where:** `common/tusb_fifo.c:819-845`
- **What:** `tu_fifo_write` in overwrite mode has no double-overflow handling: after 2x depth single writes the FIFO reads as empty. Latent (CDC uses `write_n`)
- **Test / evidence:** KNOWN_ISSUE "an overwritable FIFO written item by item with twice its depth holds the newest items (V25)" (`claude/cases_usb/test_tusb_fifo.cpp`)

## Vendor V26: TinyUSB (**vendor, tier B**)

- **Where:** `tusb_fifo.c:1022, 1077`
- **What:** `tu_fifo_get_read_info` / `get_write_info` pointers ignore `item_size` (wrong pointer for items over 1 byte). Latent (users use item size 1)
- **Test / evidence:** KNOWN_ISSUE "the read and write info of a FIFO with items of 4 bytes point at the item positions (V26)"

## Vendor V27: TinyUSB (**vendor, tier B**)

- **Where:** `class/hid/hid_device.c:299-306` vs the comment at `hid_device.h:126`
- **What:** With a non-zero report id the stack counts the id byte before calling the GET_REPORT callback, so a callback result of 0 is never stalled: the host gets a 1-byte answer instead of the promised STALL
- **Test / evidence:** KNOWN_ISSUE "GET_REPORT for a report the HID handler does not have is stalled (V27)" (`claude/cases_usb/test_hid_device.cpp`)

## Vendor V32: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_sin_f32.c:88`, `arm_cos_f32.c:87`
- **What:** For |x| > 2^31·2π (1.35e10 rad), inf or NaN the float->int conversion is undefined; for negative x `n--` overflows INT32_MIN: results far outside [-1, 1] (host 3.3e10 for sin(2e10); ARM saturates, also wrong). Intended (chosen): clamped within [-1, 1], as C `sinf` is finite for every finite x. Latent: the firmware's angles are bounded
- **Test / evidence:** KNOWN_ISSUE_UB "arm_sin_f32 and arm_cos_f32 of an angle beyond the int32 range stay within -1 and 1" (`claude/cases_target/test_cmsis_dsp.cpp`)

## Vendor V37: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_sin_q31.c:77`, `arm_cos_q31.c:79`
- **What:** `sinVal << 1` / `cosVal << 1` on a negative value: undefined behaviour (UBSan: left shift of negative value). The host result is the intended one. Any negative sin/cos result reaches it
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN "B-DFQ31-1" in 11 cases of `claude/cases_dsp/test_dsp_fastmath_q31.cpp` (skipped in sanitizer builds)

## Vendor V38: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_sin_q15.c:78,81`, `arm_cos_q15.c:79,82`
- **What:** `(q31_t) sinVal << 16` and `sinVal << 1` on a negative value: undefined behaviour, same pattern as V37 (UBSan aborts at the first hit, so further UB inside sin/cos may stay hidden)
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN "B-DFQ15-2" in 9 cases of `claude/cases_dsp/test_dsp_fastmath_q15.cpp`

## Vendor V39: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_vlog_f16.c:83` (`logf16_scalar`)
- **What:** `vecExpUnBiased << 10` is a left shift of a negative value for every input below 1: undefined behaviour
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN "B-DFF16-2" in 6 cases of `claude/cases_dsp/test_dsp_fastmath_f16.cpp`

## Vendor V46: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_pid_init_q31.c:76`
- **What:** `A1 = -clip_q63_to_q31(...)`: for Kp = Kd = -0.75 the clip returns INT32_MIN and the negation overflows (UB), A1 stays INT32_MIN instead of saturating to INT32_MAX; the q15 init handles the case
- **Test / evidence:** KNOWN_ISSUE_UB (B-DCP-1)

## Vendor V47: CMSIS-DSP (**vendor, tier C**)

- **Where:** `controller_functions.h:335-341` (`arm_pid_q31`)
- **What:** The output wraps instead of saturating (DOC line 319: truncated to 1.32 then saturated to 1.31): `out += state[2]` overflows int32 (UB; Ki 0.75, input 0.5 gives 0.375, 0.75, then negative) and the cast `(q31_t)(acc >> 31)` wraps before the saturation (Kp = Kd = 0.45, state[0] = 0.9, input -0.9 gives +0.29, expected -1.0). DOC line 317 says the accumulator wraps, which contradicts line 319. q15 saturates correctly
- **Test / evidence:** KNOWN_ISSUE_UB x2 (B-DCP-2), KNOWN_ISSUE (B-DCP-2b)

## Vendor V50: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_sin_cos_q31.c` (`sinTable_q31[384]` = INT32_MIN)
- **What:** The negation `d1 = -sinTable_q31[indexS]` overflows for table index 383/384: the cosine is wrong by up to 7.8e6 LSB (0.0036) for x in [-90.70, -89.30] degrees (16,770,947 of the 2^32 inputs; cos(-89.99 degrees) returns -3141, expected +3142), DOC says accurate to 5 digits. Sine is fine. Confirmed by a full 2^32 sweep
- **Test / evidence:** KNOWN_ISSUE_UB (B-DCP-4)

## Vendor V51: CMSIS-DSP (**vendor, tier C**)

- **Where:** `controller_functions.h` (`arm_clarke_q31`)
- **What:** `(q31_t)(((q63_t)Ib * 0x49E69D16) >> 30)` is cast to q31 before the saturating add: for |Ib| above about 0.866 full scale the result wraps (Ia = 0, Ib = 0.9 gives -2063236825, expected INT32_MAX); DOC says "there is saturation on the addition, hence there is no risk of overflow"
- **Test / evidence:** KNOWN_ISSUE (B-DCT-1)

## Vendor V54: CMSIS-DSP (**vendor, tier C**, reached from the firmware's `arm_std_f32`)

- **Where:** `arm_std_f32.c:80`, `arm_rms_f32.c:126/187`, `arm_std_f16.c:61`, `arm_rms_f16.c:129` (via `arm_sqrt_f32`, guard `in >= 0.0f`)
- **What:** A NaN variance / mean square fails the guard: the sqrt writes 0 and returns ARGUMENT_ERROR, so std and rms of data with NaN (or inf) return 0 silently (std {1,NaN,3} = 0 while `arm_var_f32` and `arm_std_f64` give NaN; std {inf,1} = 0)
- **Test / evidence:** KNOWN_ISSUE x5 (B-DSF32-3/4, B-DSF16-4/5)

## Vendor V63: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_biquad_cas_df1_32x64_q31.c:248, :257, :594-:777`
- **What:** Left shift of negative values, plus signed int64 overflow at :768 for accumulators above 2^62
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN (B-DFBQ-3)

## Vendor V64: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_math_memory.h:105` (`read_q15x2`, `pQ15[1] << 16`), `dsp/none.h:460, :503, :531` (`__SMLAD` / `__SMUAD` / `__SMLALD` emulation, packed pair `<< 16`)
- **What:** Left shift of a negative int and int overflow on the host SIMD emulation, hit by every fast q15 biquad case, q15 and fast q15 FIR with blockSize >= 4. The target uses real SIMD instructions; the host result is the intended one
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN (B-DFBQ-4, B-DFIRQ-1)

## Vendor V65: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_fir_fast_q31.c:160, :222, :223, :269` and the macro (`none.h:185`)
- **What:** `acc0 << 1` and the macro shift of a negative accumulator (UB); a 2.30 accumulator above 2^30 also overflows int. Signed data is hidden in sanitizer builds
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN (B-DFIRQ-2)

## Vendor V71: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_conv_partial_fast_q31.c:529, :597`
- **What:** `count = srcBLen - 1U` is never reduced when `firstIndex > max(srcALen, srcBLen)`, so stage 3 reads past the longer input (ASan heap-buffer-overflow read at :597; ARM's own window first = 9 of the 6x8 test hits it and passes only because ARM's buffers are zero behind the data). The same stage code in `arm_conv_partial_q31.c` and `arm_conv_partial_f32.c` fails on the target path (`ARM_MATH_DSP`), measured only in a scratch build, not in the suite
- **Test / evidence:** KNOWN_ISSUE_UB, 2 cases (B-DCVQ31-2)

## Vendor V74: CMSIS-DSP (**vendor, tier C**)

- **Where:** `dsp/none.h:503` (`__SMLALD`), `:518/:519` (`__SMLALDX`)
- **What:** The portable emulation adds two int products before widening (two (-32768)^2 overflow int): conv / correlate / partial opt q15 and `arm_lms_q15` with T >= 4 return -32768 instead of 32767. Host-emulation artefact only: Cortex-M `SMLALD` accumulates in 64 bit, firmware unaffected
- **Test / evidence:** KNOWN_ISSUE_UB (B-DCVQ15-2, B-DLMS-6)

## Vendor V83: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_lms_q15.c:147-153`, `arm_lms_norm_q15.c:165-171`
- **What:** Only a 32-bit window of the 34.30 accumulator is extracted and then saturated, so a large sum wraps: T = 5, postShift = 14, all coefficients and inputs 32767 gives y = -32768 (DOC "saturated", intended 32767; norm_q15 also gives e = -32768, DOC "no risk of internal overflow"); smallest wrapping postShift per numTaps: 5 and 8 taps 14, 16 taps 13, 64 taps 11, 256 taps 9, 1024 taps 7, 65535 taps 1
- **Test / evidence:** KNOWN_ISSUE_UB, 4 cases (B-DLMS-5)

## Vendor V84: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_lms_norm_q15.c:182`, `utils.h:139`
- **What:** `(q15_t) energy + DELTA_Q15` wraps negative once the energy reaches full scale (32766 + 5 gives -32765 for `arm_recip_q15`), the negative value is then shifted left, and the coefficient moves the wrong way (falls to -1417 for e = 16000, x = 32767, c = 0, mu = 0.5)
- **Test / evidence:** KNOWN_ISSUE_UB (B-DLMS-7)

## Vendor V85: CMSIS-DSP (**vendor, tier C**)

- **Where:** `arm_fir_lattice_q31.c:109-136, :167-272, :284-349, :397-427`
- **What:** `(fnext0 << 1U) + fcurr0` and `gnext0 = (gnext0 << 1U) + gcurr0` shift a negative value (UB) with ordinary negative data inside the documented range; a signed int overflow also occurs when a sum leaves the int range (outside the DOC input scale)
- **Test / evidence:** KNOWN_ISSUE_UB_HIDDEN (B-DLATT-1)

## Vendor V93: ST HAL RCC (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_rcc.c:591` (`HAL_RCC_ClockConfig`)
- **What:** **Arguable.** The UM1725 note says a clock source that is not ready yet is switched to when it becomes ready; the HAL checks the ready flag at the call, returns `HAL_ERROR` and never writes SW. The note may describe the hardware, UM1725 gives no return value
- **Test / evidence:** KNOWN_ISSUE (`test_hal_rcc_clock.cpp`, RC15)

## Vendor V94: ST HAL DMA (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_dma.c:309` (`HAL_DMA_DeInit`)
- **What:** DeInit requests the stop and writes the reset values at once, without waiting for EN = 0 (unlike `HAL_DMA_Init` and `Abort`); while EN still reads 1 the writes to SxNDTR, SxPAR, SxM0AR, SxM1AR and SxFCR are ignored (RM0090 10.5.5-10.5.10), so a stream that is still running keeps its old configuration. The model's stop delay of 3 ticks is an assumption (Medium)
- **Test / evidence:** KNOWN_ISSUE (`test_hal_dma_init.cpp`, DI7)

## Vendor V98: ST HAL SPI (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_spi.c:1858` (`HAL_SPI_TransmitReceive_DMA`)
- **What:** The Tx stream gets no error callback (`XferErrorCallback = NULL`): a transfer error of the Tx stream is not reported (no `ErrorCallback`, no `HAL_SPI_ERROR_DMA`) and the transfer never ends (UM1725: the error callback runs when a communication error is detected)
- **Test / evidence:** KNOWN_ISSUE (`test_hal_spi_dma.cpp`, SD3)

## Vendor V100: ST HAL SPI (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_spi.c:769` (`HAL_SPI_Transmit`)
- **What:** **Arguable.** The polling function never looks at MODF: after a mode fault clears SPE and MSTR it waits for TXE until the timeout, never sets `HAL_SPI_ERROR_MODF` and never clears the flag (RM0090 28.3.10 sequence). UM1725 does not say that the polling functions report MODF; the oracle is the UM1725 error code list
- **Test / evidence:** KNOWN_ISSUE (`test_hal_spi_poll.cpp`, SP9, 2 cases)

## Vendor V109: ST HAL I2C (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_i2c.c:6013,6038` (`I2C_SlaveTransmit_TXE`, `I2C_SlaveTransmit_BTF`)
- **What:** A Size 1 slave transmit frame (`Slave_Seq_Transmit_IT`) that starts while the controller already waits for data (BTF set, clock stretched) never reports `HAL_I2C_SlaveTxCpltCallback`: the byte goes out through the BTF handler, which has no completion handling, and the state stays BUSY_TX_LISTEN. UM1725: "At transmission end of current frame transfer, HAL_I2C_SlaveTxCpltCallback() is executed". Model dependent (needs the stretched controller); a multi-byte frame works
- **Test / evidence:** KNOWN_ISSUE (`test_hal_i2c_seq.cpp`, IQ3)

## Vendor V114: ST HAL ADC (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_hal_adc.c:1335-1350` (`HAL_ADC_IRQHandler`, OVR branch)
- **What:** The overrun branch sets only `ErrorCode` (HAL_ADC_ERROR_OVR) and never the state bit `HAL_ADC_STATE_REG_OVR` (UM1725: "Overrun occurrence"), while `HAL_ADC_PollForEvent` does set it; the two paths are inconsistent. UM1725 does not say the handler sets the state bit
- **Test / evidence:** KNOWN_ISSUE (`test_hal_adc_irq.cpp`, AQ2)

## Vendor V117: ST HAL ADC (**vendor, tier D**)

- **Where:** `stm32f4xx_hal_adc_ex.c:218-227` (`HAL_ADCEx_InjectedStart`), `:324-333` (`InjectedStart_IT`)
- **What:** RM0090 13.13.16 CCR MULTI: dual mode (00001 to 01001) has ADC1 and ADC2 working together, "ADC3 is independent". With MULTI != 0 the injected start software-starts only ADC1, so an independent ADC3 never starts (corrected 2026-11-17 after the Codex review P12C-T6B-001: `HAL_ADC_Start`, `Start_IT` and `Start_DMA` have the same defect: their `ADC3` condition (`stm32f4xx_hal_adc.c:790-795`, `< ADC_CCR_MULTI_4`) sits inside the `MULTI == 0` branch, where it can only be true for MULTI = 0, and the `MULTI != 0` branch admits only ADC1; so with MULTI = 00110 an independent ADC3 is never software started by the regular start functions either). Two cases for the injected start, three for the regular start
- **Test / evidence:** KNOWN_ISSUE (`test_hal_adc_ex.cpp`, AX2; `test_hal_adc_base.cpp` AB3, `test_hal_adc_irq.cpp` AQ1, AQ3)

## Vendor V118: ST HAL USB (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_ll_usb.c:1013` (`USB_WritePacket`, reached from `HAL_PCD_EP_Transmit` through `PCD_WriteEmptyTxFifo`)
- **What:** The FIFO is filled with `(len + 3) / 4` whole 32-bit reads of the caller's buffer, so a buffer whose length is not a multiple of 4 is over-read by up to 3 bytes. Found by AddressSanitizer (stack-buffer-overflow); harmless on the MCU unless the buffer ends at the end of RAM. No case: the over-read is not observable without the sanitizer; all test source buffers are padded and carry a `V118` comment
- **Test / evidence:** reported, no test

## Vendor V126: ST HAL flash (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_hal_flash_ex.c:160-222` (`HAL_FLASHEx_Erase`)
- **What:** Never reads FLASH_CR.LOCK: on a locked flash RM0090 3.6.1 ignores the CR writes, nothing is erased, yet the HAL returns HAL_OK with `*SectorError` = 0xFFFFFFFF, which UM1725 defines as "all the sectors have been correctly erased"; `HAL_FLASH_Unlock` verifies LOCK, so the erase path is inconsistent
- **Test / evidence:** KNOWN_ISSUE (`test_hal_flash_ex.cpp`, FE1: sector erase and mass erase)

## Vendor V127: ST HAL flash (**vendor, tier D**, arguable)

- **Where:** `stm32f4xx_hal_flash.c:461-475` (`HAL_FLASH_OB_Unlock`)
- **What:** Writes the two OPTKEYR keys and returns HAL_OK without reading OPTLOCK back; after a wrong key sequence OPTCR stays locked until reset (RM0090 3.7.2), so the unlock did not happen; `HAL_FLASH_Unlock` reads LOCK back and returns HAL_ERROR
- **Test / evidence:** KNOWN_ISSUE (`test_hal_flash_base.cpp`, FB1)
