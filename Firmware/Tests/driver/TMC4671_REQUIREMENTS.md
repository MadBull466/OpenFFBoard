# TMC4671 driver: requirements charter (phase D1)

Status: **signed by the lead (Opus 5.5), 2026-11-03.** Changes only by the lead, recorded in the "Lead decisions log" of `Tests/PROGRESS.md`.

Driver under test: `Firmware/FFBoard/UserExtensions/Src/TMC4671.cpp`, `TMC4671_configs.cpp`, `Inc/TMC4671.h` (classes `TMC4671`, `TMC_1`, `TMC_2`).
Executable: `ffb_driver_tests`, tests in `Firmware/Tests/cases/driver/test_tmc4671_*.cpp`.
Inventory (line references used below): `../ffb-test-build/ws/d1/tmc_inventory.md` (not committed; the facts that matter are repeated here).

## Sources and how to cite them

| Code | Source |
|---|---|
| DS | TMC4671-LA datasheet, IC version V1.3, document revision V2.07 (2022-01-12), supplied by the user as `Tests/TMC4671-LA_datasheet_Rev2.07.pdf` (never committed). Cite as `DS §<section> p<page>`; register fields as `DS 0xNN` (= section 7.2 "Register Map - Functional Description", p77-110, and 7.3 defaults, p111-127). |
| RM | RM0090 / ST HAL source (SPI clock, prescaler) |
| HD | Hand derivation written in a comment next to the assertion (show the arithmetic) |
| FW | Firmware behaviour only, no external spec: the test name starts with `characterization:` and gets a NEEDS REVIEW entry in `Tests/PROGRESS.md` |

Datasheet facts every test author needs (verified by the lead in the PDF text):

* **SPI datagram** (DS §4.2.1 p17): 40 bits, MSB first; bit 39 = WRnRD (1 = write); bits 38..32 = address (7 bit, 128 addresses, DS §7 p72); bits 31..0 = data, MSB first. The chip answers **inside the same datagram** for reads and writes (no pipelining: the read data of a datagram belongs to the address of that same datagram). SPI mode 3 (CPOL 1, CPHA 1). Example from the datasheet: write `0x8100000000` (CHIPINFO_ADDR = 0), then read `0x0000000000` returns `0x34363731` (ASCII "4671").
* **SPI timing** (DS §4.2.1 Table 2 and info box p18-19): writes up to 8 MHz; reads up to 8 MHz **only with a pause of at least 500 ns after the address byte**, otherwise reads are limited to **2 MHz**.
* **Stacked registers** (DS §7.1 p73): CHIPINFO_DATA 0x00 selected by CHIPINFO_ADDR 0x01 (0 SI_TYPE, 1 SI_VERSION, 2 SI_DATE, 3 SI_TIME, 4 SI_VARIANT, 5 SI_BUILD); ADC_RAW_DATA 0x02 selected by ADC_RAW_ADDR 0x03 (0: I1 [31:16] & I0 [15:0]; 1: AGPI_A [31:16] & VM [15:0]; 2: AENC_UX [31:16] & AGPI_B [15:0]; 3: AENC_WY [31:16] & AENC_VN [15:0]); CONFIG_DATA 0x4D selected by CONFIG_ADDR 0x4E (1/2/4/5/6/7 biquad_x a1/a2/b0/b1/b2/enable, 9-15 biquad_v, 17-23 biquad_t (torque), 25-31 biquad_f (flux), 62 ADVANCED_PI_REPRESENT: bit0 current I, bit1 current P, bit2 velocity I, bit3 velocity P, bit4 position I, bit5 position P; 1 = Q4.12, 0 = Q8.8); PID_ERROR 0x6C/0x6D; INTERIM 0x6E/0x6F.
* **Register layouts used by the driver** (DS §7.2): 0x08/0x09 offset u16[15:0], scale s16[31:16]; 0x0A ADC_I_SELECT I0_SELECT[7:0], I1_SELECT[15:8], UX[25:24], V[27:26], WY[29:28]; 0x17 PWM_POLARITIES bit0 LS, bit1 HS; 0x18 PWM_MAXCNT u12, fPWM = 100 MHz / (PWM_MAXCNT+1); 0x19 BBM_L[7:0], BBM_H[15:8] in 10 ns; 0x1A PWM_CHOP[7:0] (0 off ... 7 centered FOC), PWM_SV bit 8; 0x1B N_POLE_PAIRS u16[15:0], MOTOR_TYPE u8[23:16] (0 none, 1 DC, 2 stepper, 3 BLDC); 0x25 ABN_DECODER_MODE apol b0, bpol b1, npol b2, use_abn_as_n b3, cln b8, direction b12 (no bit 9); 0x26 PPR u24; 0x29 PHI_M_OFFSET s16[15:0], PHI_E_OFFSET s16[31:16]; 0x45 AENC PHI_M_OFFSET s16[15:0], PHI_E_OFFSET s16[31:16]; 0x50/0x51 selection codes (0 phi_e sel, 1 phi_e_ext, 2 openloop, 3 abn, 5 hal, 6 aenc, 7 phi_a_aenc, 9 phi_m_abn, 10 phi_m_abn_2, 11 phi_m_aenc, 12 phi_m_hal), 0x50 VELOCITY_METER_SELECTION [15:8]; 0x52 PHI_E_SELECTION (1 ext, 2 openloop, 3 abn, 5 hal, 6 aenc, 7 phi_a_aenc); 0x54/56/58/5A I s16[15:0], P s16[31:16]; 0x5D u16 (abs value used); 0x5E u16; 0x63 MODE_MOTION[7:0] (0 stopped, 1 torque, 2 velocity, 3 position, 8 uq_ud_ext), MODE_PID_SMPL [30:24], MODE_PID_TYPE bit31 (1 = sequential/advanced); 0x64 FLUX_TARGET s16[15:0], TORQUE_TARGET s16[31:16]; 0x65 same layout (offsets); 0x69 FLUX_ACTUAL s16[15:0], TORQUE_ACTUAL s16[31:16]; 0x6B PID_POSITION_ACTUAL s32, a write also writes PID_POSITION_TARGET; 0x75 VM_LIMIT_LOW u16[15:0], HIGH u16[31:16]; 0x76 INPUTS_RAW bit 2 N of ABN, bit 15 ENABLE_IN_RAW; 0x7C STATUS_FLAGS (bit 19 not_PLL_locked, 26 adc_i_clipped, 27 aenc_clipped, 28 enc_n, 29 enc_2_n, 30 aenc_n, 31 reserved); 0x7D STATUS_MASK u32.
* **Biquad** (DS §4.8.1 p61): `Y(n) = X(n)·b0 + X(n-1)·b1 + X(n-2)·b2 + Y(n-1)·a1 + Y(n-2)·a2`; coefficients S32 in Q3.29 (`round(x·2^29)`), with `a1 = round(-a1_z·2^29)`, `a2 = round(-a2_z·2^29)` (feedback terms are ADDED, so they are the negated denominator coefficients). Sampling period = PWM period. Consequence used as an oracle: a lowpass has unity DC gain, so `(b0+b1+b2) / (2^29 - a1 - a2) = 1` (HD).
* **Registers the driver uses that Rev 2.07 does not document**: 0x1E (`setPositionExt`), 0x47 (`bangInitEnc`), 0x5C (`setLimits`/`getLimits`, "torque/flux ddt limit"); and the driver writes 0x41 AENC_DECODER_COUNT, which DS §7.1 p75 lists as read-only. Rev 2.01/2.02 removed registers (DS revision history p152). See open question Q1.

## Requirements

Priority: **P1** must be tested; **P2** should be tested (needs a scripted plant or more setup); **P3** characterization only or recorded as untestable.
Oracle column: what the test asserts and where the expected value comes from.

### G1 SPI transport

| Id | Pri | Requirement (observable behaviour) | Source | Oracle |
|---|---|---|---|---|
| R1 | P1 | A register read is one 5-byte transfer with CS (PA4 for TMC_1, PB0 for TMC_2) low for the whole frame and high afterwards; byte 0 = address with bit 7 clear, bytes 1-4 = 0. | DS §4.2.1 p17 | exact tx bytes, `csAtStart/csAtEnd`, selected CS index |
| R2 | P1 | The value returned by a read is MISO bytes 1-4 of the same frame, most significant byte first. | DS §4.2.1 p17 | fake answers 0x12345678 for one address, driver getter returns it (via a public getter, e.g. `getVelocity` 0x6A or the `reg` command) |
| R3 | P1 | A register write is one 5-byte transfer: byte 0 = 0x80 OR address, bytes 1-4 = data MSB first; CS framing as R1. | DS §4.2.1 p17 | exact tx bytes |
| R4 | P1 | The SPI peripheral is configured for mode 3 (CPOL high, CPHA second edge), 8-bit, MSB first, software NSS, and the resulting SCK does not exceed 8 MHz. | DS §4.2.1 p17-18; RM (SCK = 84 MHz / prescaler, SPI1 on APB2) | `HAL_SPI_Init` parameters recorded by the fake SPI; HD of the clock |
| R5 | P1 | Register reads respect the read timing: either SCK <= 2 MHz, or a pause of >= 500 ns after the address byte (i.e. the address byte is a separate transfer). | DS §4.2.1 Table 2 + info box p18-19 | **suspected bug B-T1**: SCK is 5.25 MHz and the read is one 5-byte transfer (comment "500ns delay" at TMC4671.cpp:2533 but no delay). `KNOWN_ISSUE`; first assertion = the SCK/transfer-split condition |
| R6 | P1 | A read-modify-write (`updateReg` paths) changes only the addressed field and keeps the other bits as read from the chip. | DS §7 p72 ("fields need to be composed by masking and shifting") | fake preloaded with a pattern; after e.g. `setSvPwm`, register 0x1A = old with only bit 8 changed |
| R7 | P2 | The asynchronous write path (`writeRegAsync`, used by the external-encoder thread) sends the same frame as R3, keeps CS low until the transfer-complete callback and then releases CS and the port. | DS §4.2.1 p17 | IT transfer pending, `spiComplete`, CS levels |
| R8 | P3 | A read whose SPI transfer fails (HAL error/timeout) is not used as register data. | robustness | **untestable deterministically** on host (uninitialised buffer, inventory 9.1); recorded as suspected bug B-T2, no test |

### G2 Identification and presence

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| R10 | P1 | `pingDriver()` selects CHIPINFO_ADDR = 0 (SI_TYPE) and reports the chip present iff CHIPINFO_DATA = 0x34363731. | DS §4.2.1 p17, DS 0x00/0x01 p77 | write 0x01=0 then read 0x00; true for 0x34363731, false for 0, 0xFFFFFFFF and any other value |
| R11 | P1 | The `tmctype` command reports the SI_TYPE name ("4671") and the SI_VERSION as major.minor (u16.u16) of a present chip, and "No driver connected" when SI_TYPE reads 0 or 0xFFFFFFFF. | DS 0x00 p77 (SI_VERSION u16.u16) | version 0x00010003 -> "1.3"; **suspected bug B-T3** (`&&` instead of `&`, TMC4671.cpp:1855): `KNOWN_ISSUE` for the version digits |
| R12 | P2 | An engineering sample (SI_VERSION = 0x00010000 read after CHIPINFO_ADDR = 1) makes the driver lower the SPI clock to about 1 MHz and skip the PI precision setting. | FW (the datasheet does not name the ES version) | `characterization:` prescaler change, no 0x4E=62 write |
| R13 | P1 | While the chip does not answer the ping, the driver stays `uninitialized`, does not write configuration registers (only the ping), and retries every 250 ms. | FW | `characterization:` state + transfer log over simulated time |
| R14 | P1 | A chip that never answers is reported to the user (communication error) rather than retried silently forever. | intended behaviour (the error `tmcCommunicationError` exists for this) | **suspected bug B-T4** (inventory 9.28): `KNOWN_ISSUE`, first assertion = error present after N ms of silence |

### G3 Initialisation sequence (`initialize()`, `initializeWithPower()`, `setupDriver()`)

All expected values are the register layouts of the DS plus the firmware default configuration (`TMC4671.h` defaults). Where a value is a firmware choice (board specific) the DS only gives the layout: the test asserts layout from DS and value from the header default with a comment naming both.

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| R20 | P1 | After a successful init the PWM is configured: PWM_MAXCNT = 4095 (fPWM = 100 MHz/4096 = 24.414 kHz), BBM_L/BBM_H packed low/high byte, polarities 0x17 = 0. | DS 0x17-0x19 p84; defaults TMC4671.h | register values in the fake |
| R21 | P1 | Motor type and pole pairs are written to 0x1B as `pole_pairs | type<<16`; a DC motor forces 1 pole pair; SVPWM bit (0x1A bit 8) is set only for a BLDC motor with SVPWM enabled. | DS 0x1A/0x1B p84-85 | register values for DC, stepper, BLDC, none |
| R22 | P1 | ADC setup: MDEC 0x07 = mdecA | mdecB<<16, MCLK 0x05/0x06, sel_nclk_mclk_i bits (0x04 bit 4 / bit 20) set when the MCLK is non-zero, ADC_I_SELECT 0x0A = 0x18000100 (I0<-ADC0, I1<-ADC1, UX=I0, V=I2, WY=I1). | DS 0x04-0x07, 0x0A p77-79 | field-by-field decode of the written values; the channel mapping itself is board specific: comment |
| R23 | P1 | ADC offsets and scales go to 0x09 (I0) and 0x08 (I1): offset [15:0], scale [31:16], other half preserved. | DS 0x08/0x09 p78 | register values |
| R24 | P1 | The PI precision bits are written through CONFIG_ADDR 62 / CONFIG_DATA with bit order current I, current P, velocity I, velocity P, position I, position P. | DS 0x4D/0x4E p93-94 | indirect write pair and data bits |
| R25 | P1 | The flux biquad is configured as a lowpass (350 Hz, Q 0.7 at the PWM rate) through CONFIG_ADDR 25/26/28/29/30/31, enabled, with Q3.29 coefficients that satisfy unity DC gain and the DS sign convention. | DS §4.8.1 p61, 0x4E p94 | indirect writes in this order; HD: standard bilinear lowpass with K = tan(pi·fc/fs) computed in the test; DC gain check; tolerance +-2 LSB (truncation vs DS rounding: see Q4) |
| R26 | P1 | The init writes brake chopper limits 0x75 = low | high<<16 from the selected hardware config, a status mask 0x7D (adc_i_clipped | not_PLL_locked = bits 26 and 19), the four PI registers and the controller structure bit (0x63 bit 31). | DS 0x75, 0x7C/0x7D, 0x54-0x5A, 0x63 | register values |
| R27 | P1 | The init leaves no stale torque/flux target: 0x64 = 0 before the PWM is enabled. | DS 0x64 p97 | write order in the transfer log |
| R28 | P1 | `setupDriver()` writes all limits (0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62) with the configured values and starts the "TMC" thread. | DS 0x5D-0x62 p96-97 | register values; thread exists |
| R29 | P1 | The PWM is switched to centered FOC (0x1A [7:0] = 7) and the enable pin (PE7) is set only after power is present and the ADC is calibrated (`initializeWithPower`). | DS §4.9 p63 (mode 7 for operation) | order: enable pin and 0x1A = 7 occur after the power condition; never before |
| R30 | P2 | Warm start: if the chip already has a motion mode != 0 at init, the driver reports a warm start; else cold. | FW | `characterization:` |

### G4 Configuration setters

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| R40 | P1 | `setPids` writes each PI register as I [15:0], P [31:16]; `getPids` decodes the same layout. | DS 0x54-0x5A p96 | values incl. P >= 0x8000 |
| R41 | P1 | Reading the PIDs back does not change the controller structure setting (sequential/classic). | DS 0x63 bit 31 (separate register) | **suspected bug B-T5** (`getPids` resets `sequentialPI` to true, inventory 9.7): `KNOWN_ISSUE` |
| R42 | P1 | `setLimits` / `getLimits`, `setTorqueLimit` (0x5E), `setUqUdLimit` (0x5D), `setPowerLimit` write/read the documented registers. | DS 0x5D-0x62 | values |
| R43 | P1 | `setPwmFreq(f)` writes PWM_MAXCNT = round-down of 100 MHz/f - 1, clipped to the 12-bit range used by the driver (255..4095); `getPwmFreq` = 100 MHz/(MAXCNT+1). | DS 0x18 p84 | HD for 25 kHz (3999), 100 kHz (999), 10 kHz -> clipped 4095; frequencies whose MAXCNT exceeds 65535 (f < ~1526 Hz): **suspected UB B-T6** (float to uint16 before the clip, inventory 9.18) `KNOWN_ISSUE_UB` |
| R44 | P1 | `setBBM(low, high)` writes 0x19 = low | high<<8. | DS 0x19 p84 | values |
| R45 | P1 | Motion mode setter writes MODE_MOTION [7:0] of 0x63 and keeps bit 31 and [30:24]; forced vs deferred mode (deferred applies on the next forced/torque command). | DS 0x63 p97; FW for deferral | values; deferral `characterization:` |
| R46 | P1 | Position/velocity selectors: 0x51 = selection code, 0x50 = code | meter<<8; phi_e source 0x52 = code with `extEncoder` mapped to 1 (phi_e_ext); `getPhiEtype` decodes 0x52. | DS 0x50-0x52 p95 | values |
| R47 | P1 | Torque biquad (`setTorqueFilter`): mode none writes enable = 0; lowpass/notch/peak write Q3.29 coefficients through CONFIG_ADDR 17/18/20/21/22/23 with the DS sign convention; a filter with fc <= 0 is not enabled. | DS §4.8.1 p61, 0x4E p94 | HD as R25; enable word |
| R48 | P1 | Hardware version selection (`setHwType` / `tmcHwType`) applies the documented config of `TMC4671_configs.cpp`: brake limits, BBM, voltage scaler; an unknown version keeps the previous config. | FW (board data, not in DS) | `characterization:` per version (1, 2, 3, 6) and for 4/5/unknown |
| R49 | P1 | ABN encoder setup writes 0x25 (apol b0, bpol b1, npol b2, use_abn_as_n b3, cln b8, direction b12), 0x26 = cpr, 0x29 = phi_e_offset [31:16] | phi_m_offset [15:0]; `setCpr` writes 0x26 and AENC PPR 0x40; cpr 0 becomes 1. | DS 0x25/0x26/0x29 p86-87, 0x40 p90 | values |
| R50 | P1 | Enabling/disabling the encoder index flag clears or sets the documented N-event bit only: `setEncoderIndexFlagEnabled(_, zero)` writes the "write COUNT_N into COUNT at N" (cln) bit. | DS 0x25 bit 8 p86 | **suspected bug B-T7** (bit 9 written, inventory 9.21) `KNOWN_ISSUE` |
| R51 | P1 | AENC setup writes 0x0D-0x0F offset [15:0] / scale [31:16], 0x40 PPR, 0x3E phi_a offset, 0x45 phi_m_offset [15:0] and phi_e_offset [31:16], 0x3C N_THRESHOLD u16 [15:0] (the firmware also writes an `nMask` into [31:16], a field removed in DS rev V2.04: not asserted, see Q1), 0x3B (120deg_n90deg b0 = uvw mode, direction b12). | DS 0x0D-0x0F p79-80, 0x3B-0x45 p90-91 | **suspected bug B-T8** (0x45 halves swapped in `setup_AENC`, inventory 9.20) `KNOWN_ISSUE`; the other registers plain |
| R52 | P1 | Hall setup writes HALL_MODE 0x33 (polarity b0, synchronous PWM sampling b4 (the firmware calls it `filter`), interpolation b8, direction b12, HALL_BLANK u12 [27:16]), 0x34-0x36 positions (0/60, 120/180, 240/300 as low/high halves), 0x37 phi_m_offset [15:0] / phi_e_offset [31:16], 0x38 dPhiMax u16. | DS 0x33-0x38 p89 | values |
| R53 | P2 | `setUdUq(ud, uq)` writes UQ_UD_EXT 0x24 = ud [15:0] | uq [31:16] for negative values too. | DS 0x24 p86 | **suspected bug B-T9** (sign extension of negative `ud`, inventory 9.10) `KNOWN_ISSUE` |
| R54 | P2 | Open loop: `setOpenLoopSpeedAccel` writes 0x21 velocity target and 0x20 acceleration; `runOpenLoop` selects open loop phi_e and voltage mode. | DS 0x1F-0x24 p85-86 | values |
| R55 | P3 | `setGpioPins`, feed-forward setters, `setBiquadPos/Vel`, `setPositionExt`, `setFFMode` have no caller in the firmware build. | FW | not tested (dead code); `setGpioPins` defect (inventory 9.22) recorded as suspected bug B-T10, no test |

### G5 Control: torque/flux, position, readouts

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| R60 | P1 | `setFluxTorque(flux, torque)` writes 0x64 = flux (s16) [15:0] | torque (s16) [31:16] for positive and negative values, and switches the motion mode to torque if it was not. | DS 0x64/0x63 p97 | values: (-1, -1) -> 0xFFFFFFFF; (100, -200) -> 0xFF380064 (HD) |
| R61 | P1 | `setTorque` / `setFlux` update only their half of 0x64; `getTorque` / `getFlux` decode their half as s16. | DS 0x64 p97 | **suspected bug B-T11** (`getFlux` uses `&&`, D0 oddity, TMC4671.cpp:2078) `KNOWN_ISSUE` for `getFlux` |
| R62 | P1 | `turn(power)` does nothing unless the motor is ready and enabled; when ready it writes torque = power (negated if `invertForce`, or external encoder reversed) and flux = -min(|power|, fluxoffset). | FW (field weakening policy) + DS 0x64 layout | `characterization:` for the flux policy; layout DS |
| R63 | P1 | Flux dissipation (`fluxbrake`): when the internal supply exceeds the external one by more than 1 V, `turn` replaces the flux by `clip(vDiff·0.5·torqueLimit·0.0002, 0, torqueLimit)`. | FW | `characterization:` with HD of the formula |
| R64 | P1 | `getActualTorqueFlux` decodes 0x69 as flux [15:0], torque [31:16] (s16); the `acttrq` command reports torque and flux. | DS 0x69 p97 | values incl. negative |
| R65 | P1 | Position: `getPos` reads 0x6B (s32), `setTmcPos` writes 0x6B; target 0x68; velocity actual 0x6A, target 0x66; `reachedPosition(tol)` = |target - actual| < tol. | DS 0x66-0x6B p97-98 | values |
| R66 | P2 | `rampFlux(target, ms)` moves the flux target from its current value to the target in steps, ending exactly at the target, starting from a negative current flux correctly. | DS 0x64 s16 | **suspected bug B-T12** (uint16 start, inventory 9.24) `KNOWN_ISSUE` for a negative start; positive start plain |
| R67 | P2 | Supply voltage readout `getTmcVM` / `vm` command: ADC_RAW_ADDR = 1, VM = ADC_RAW_DATA [15:0], mV = (raw - 0x7FFF - adcOffset)·vmScaler·1000, negative below the offset. | DS 0x02/0x03 p77; FW scaler | **suspected bug B-T13** (unsigned wrap, inventory 9.3) `KNOWN_ISSUE` for raw below the offset; above it `characterization:` of the scaler |
| R68 | P2 | Temperature `getTemp` / `temp`: ADC_RAW_ADDR = 2, raw from [15:0]; NTC formula of the firmware; 0 if the thermistor is disabled. | DS 0x02/0x03 (AGPI_B in [15:0] for address 2) | `characterization:` with HD of the Beta formula |

### G6 State machine, power, errors (thread `Run()`)

Observable: `getState()`, `state` command, registers, enable pin, errors, flash. Time only via simulated time.

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| R70 | P1 | Start-up path with a present chip and no encoder (enctype NONE): `uninitialized` -> `waitPower` -> (power present for > 5 checks) -> `Shutdown`; `motorReady()` false until `startMotor`. | FW | `characterization:` state sequence + timing (250 ms / 100 ms steps) |
| R71 | P1 | Without supply voltage (`hasPower` false: int or ext <= 10 V or int >= 78 V) the driver stays in `waitPower` and never enables the gate driver (PE7) or the PWM. | FW thresholds; safety | `characterization:` thresholds; "never enables" plain |
| R72 | P1 | `startMotor` from `Shutdown` (initialised, encoder aligned) -> `Running`; `stopMotor` -> motion mode stop (0x63 [7:0] = 0), PWM off (0x1A [7:0] = 0), `Shutdown`. | DS 0x63, 0x1A | values + state |
| R73 | P1 | Emergency stop (`emergencyStop(false)`): enable pin low, motion stopped, motor not ready; `emergencyStop(true)` restores the requested enable. | FW + safety | pin, registers, state |
| R74 | P1 | Hardware enable input: in `Running`, if ENABLE_IN_RAW (0x76 bit 15) is low at the periodic check (every > 2 s), the driver triggers an emergency stop and raises the critical `emergencyStop` error; when the input is high again it clears the error and re-initialises. | DS 0x76 bit 15 p103; FW period | state, error, pin |
| R75 | P1 | Undervoltage while running: the driver stops the motion (0x63 = 0), raises the `undervoltage` warning and goes to `waitPower`; when power returns it resumes and clears the warning. | FW | first part `characterization:`; the clearing is **suspected bug B-T14** (inventory 9.29) `KNOWN_ISSUE` |
| R76 | P1 | Over-temperature (thermistor enabled, temperature > 90 C) in `Running` stops the motor and ends in `HardError`. | FW (limit from config) | state; PWM off |
| R77 | P1 | ADC offset calibration failure during `initialize()` ends in `HardError` (not an endless re-init loop) and the enable pin stays low. | FW intent (comment "ADC or shunt amp is broken", `changeState(HardError)`) | **suspected bug B-T15** (inventory 9.4) `KNOWN_ISSUE` |
| R78 | P1 | `calibrateAdcOffset`: averages ADC_RAW_DATA (address 0) samples, writes the means as offsets (R23 layout) and succeeds when both means are within 5000 of 0x7FFF with enough samples; otherwise raises the critical `adcCalibrationError`. A chip returning only 0/0xFFFFFFFF fails cleanly (no division by zero). | DS 0x02/0x03; FW limits | values; the zero-sample case is **suspected bug B-T16** (division by zero, inventory 9.5) `KNOWN_ISSUE_UB` |
| R79 | P1 | Status flags: `readFlags` reads 0x7C (masked by the mask when asked); the FLAG pin interrupt (PE8) runs a status check that clears the flags (0x7C = 0) and, for an `enc_n` flag with an ABN encoder (or `aenc_n` with sincos), records the index hit and disables the index flag in the mask. | DS 0x7C/0x7D p107 | registers, `getEncoderType` incrementalIndex |
| R80 | P1 | A brake resistor failure error (raised elsewhere) makes the driver emergency-stop and enter `HardError`. | FW | state, pin |
| R81 | P2 | Two instances (TMC_1 on CS1, TMC_2 on CS2) address only their own CS; flash addresses differ by 0x40. | FW (board) | CS index; flash keys |
| R82 | P2 | `motorReady()` is true only in `Running` with power initialised, ADC calibrated and encoder aligned. | FW | combinations |

### G7 Encoder alignment and procedures (scripted plant; Medium/Low fidelity)

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| R90 | P2 | Encoder type selection (`encsrc`): unsupported types become NONE; `ext` becomes NONE without the ext-encoder timer or when ext is not allowed, and with ext allowed but no external encoder it keeps type 5 but behaves like NONE (L28); ABN with cpr 0 stays unaligned; hall is trusted immediately (selectors 0x50/0x51 = 12 phi_m_hal, phi_e = 5); NONE -> `Shutdown` aligned. | DS 0x50-0x52; FW policy | `characterization:` for the policy, selector codes DS |
| R91 | P2 | ABN alignment (`EncoderInit`) with a plant whose encoder follows the commanded angle: ends with `encoderAligned`, phi_e source abn (0x52 = 3), state `EncoderFinished`/`Shutdown`, success broadcast. | FW | `characterization:` with the scripted plant |
| R92 | P2 | ABN alignment with an encoder that does not move: `encoderAlignmentFailed` error and `HardError` after the retries. | FW | `characterization:` |
| R93 | P3 | Full calibration, AENC calibration, index search/autohome, ABN parameter estimation, PID autotune: run to completion with a scripted plant without hanging (bounded simulated time), and report via their broadcast. | FW | `characterization:`; AENC/autotune loops without bound (inventory 4.5, 9.6) recorded; division by zero in `calibrateAenc` for a constant channel = suspected bug B-T17 `KNOWN_ISSUE_UB` if reachable |
| R94 | P2 | External encoder mode: with an external encoder (MockEncoder) the ext-encoder timer (TIM6) is started and each update writes PHI_E_EXT 0x1C = (pos·65535/cpr·pole_pairs + offset) & 0xFFFF; without one, `extphie` must not divide by zero. | DS 0x1C p85; FW formula (HD) | value HD; the cpr 0 case is **suspected bug B-T18** (inventory 9.17) `KNOWN_ISSUE_UB` |

### G8 Commands (CommandHandler `tmc.<n>`)

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| R100 | P1 | Every registered command (42, ids 0-41) answers get/set as documented in its help string; set commands with a register effect produce the register write of the matching G3-G5 requirement. | FW + DS layouts | one test per command or table-driven per group; register value from DS layout |
| R101 | P1 | `reg` command: `getat` reads the register at the given address, `setat` writes value to the address. | FW contract (other getat handlers use `adr`) | **suspected bug B-T19** (`getat` uses `val`, inventory 9.12) `KNOWN_ISSUE` |
| R102 | P1 | `mtype` set rejects or maps an invalid motor type value. | FW | `characterization:` (inventory 9.11) |
| R103 | P1 | Commands are only enabled after the constructor finished (no command executes on a half-built object). | FW | `characterization:` |

### G9 Persistent settings (flash)

| Id | Pri | Requirement | Source | Oracle |
|---|---|---|---|---|
| R110 | P1 | `saveFlash` + `restoreFlash` round-trip motor config (type, svpwm, encoder type, pole pairs up to 1023), encoder/hall misc bits, hardware version, torque/flux PI values, encoder offset, flux offset, torque filter (freq 13 bit, mode 3 bit), for both instances at their addresses (TMC1 0x320-0x32C, TMC2 0x360-0x36C). | FW (eeprom_addresses.h) | HD of the bit packing; round trip |
| R111 | P1 | Restoring settings re-applies them to the chip (cpr -> 0x26/0x40, hw type, motor type). | DS layouts | registers after restore |
| R112 | P1 | ABN cpr above 65535 survives save/restore. | DS 0x26 u24 (cpr up to 2^24-1) | **suspected bug B-T20** (16-bit flash field, inventory 9.8) `KNOWN_ISSUE` |
| R113 | P1 | The position-P PI precision bit survives save/restore. | FW intent | **suspected bug B-T21** (inventory 9.9) `KNOWN_ISSUE` |
| R114 | P1 | ADC offsets are stored only after a successful calibration (`saveAdcParams`), and a stored, still valid offset skips the calibration at power-up (`checkAdc`). | FW | `characterization:` |

## Fake peer: `FakeTmc4671` (to be built by a Sonnet worker, reviewed by the lead)

Location `Firmware/Tests/fakes/FakeTmc4671.h/.cpp`, linked only into `ffb_driver_tests`. It plugs into `HostTarget::spiSetResponder(&hspi1, ...)` and selects itself by its CS pin (the responder checks `selectedCs()`; two fakes can share the bus).

Must model (with DS citations in the fidelity header):
1. 128 x 32-bit register file, power-on defaults from DS §7.3 for every documented register (at least the ones in the list above; others 0). Read-only registers (DS §7.1 "R") ignore writes; the write is recorded as a violation.
2. 40-bit datagram decode exactly as DS §4.2.1: transfer size must be 5 (anything else = violation, answer zeros); WRnRD bit; read answer in bytes 1-4 of the same frame; the answer to a write frame is the current content (DS: "immediately responses ... on read and write"; the content of the answer to a write is not used by the driver, answer = register value before the write, documented as an assumption).
3. Stacked registers: CHIPINFO (SI_TYPE 0x34363731, SI_VERSION configurable, default 0x00010003 = V1.3 per DS title "IC Version V1.3"), ADC_RAW (4 selectable 32-bit words, settable by the test), CONFIG (address space 0..63, values readable by the test), PID_ERROR, INTERIM.
4. STATUS_FLAGS: test can raise bits; a write of 0 to 0x7C clears flags (DS 0x7C "also used to reset status flags"; exact clear semantics beyond write-0 not in DS: document as assumption). 0x6B write also sets 0x68 (DS 0x6B).
5. Undocumented addresses (not in DS §7.1: 0x10, 0x14, 0x1D, 0x1E, 0x2B, 0x32, 0x43, 0x44, 0x47-0x4C, 0x4F, 0x55, 0x57, 0x59, 0x5B, 0x5C, 0x5F (documented only in 7.3: treat as documented), 0x70-0x74, 0x7A, 0x7E, 0x7F): accesses are stored but recorded as `undocumented` events.
6. Inputs the driver reads that are hardware state: ENABLE_IN (0x76 bit 15, default high), N pulse, flags, ADC raw words, actual torque/flux/velocity/position, encoder readouts (0x2A, 0x39, 0x3A, 0x46, 0x28): settable by the test. Hooks: `onRead(addr, fn)` and `onWrite(addr, fn)` so a test can script a plant (Low confidence, documented per test).
7. Strictness: `violations()` (wrong frame size, write to read-only, CS not low for the whole frame, transfer while another device's CS is low) and `undocumentedAccesses()`. The driver test fixture fails the test at teardown on any violation that the test did not declare as expected (`expectViolation(...)`).
8. Time: optional `setMicrosPerTransfer(us)` advancing `HostPlatform` time per frame (default 0: blocking SPI takes no simulated time in the fake SPI); needed so busy-wait loops on `HAL_GetTick` end. Documented as a modelling choice (real frame = 40 bits at 5.25 MHz = 7.6 us).
9. Not modelled (state in the header): the FOC control loop, PWM outputs, ADC conversion, encoder decoding, PLL, watchdog, UART/RTMI, step/dir, SPI timing (only checked by R5 through the SPI configuration), ENI/ENO pin effects. Confidence: frame/register file **High**; stacked registers **High**; status flags **Medium**; anything dynamic (plant hooks) **Low**.

Fixture `support/TmcFixture.h` (driver tests only): resets the target (`TargetFixture`), installs one `FakeTmc4671` per CS, sets supply voltages through the fake ADC (helpers `powerOn()`, `powerOff()`), provides `makeTmc1()` / `makeTmc2()` (construct after the fake), `runThreadFor(ms)` (makeResumable "TMC", advance simulated time), command helpers, and the teardown check of R-violations.

## Coverage target

`TMC4671.cpp` line coverage >= 80 % in the GCC coverage build, remainder classified as: dead code (no caller, R55), hardware-only/plant-dependent (G7 parts), unreachable. `TMC4671_configs.cpp`: data, covered by R48.

## Open questions to the user (Q) / lead decisions (L)

* **Q1** Registers 0x1E, 0x47, 0x5C are not in Rev 2.07 and 0x41 is read-only there. Is the driver written against an older register map (e.g. a pre-2.01 datasheet) that you can supply? Until answered: the fake records these as undocumented, the tests do not assert a meaning for them, and the writes to read-only 0x41 are reported (suspected bug B-T22, no `KNOWN_ISSUE` because the effect on the chip is undocumented).
* **Q2** Engineering-sample handling (R12): the datasheet does not give the ES SI_VERSION; tested as characterization only.
* **L1** Board-specific values (ADC channel mapping 0x0A, hardware configs, flux filter 350 Hz, thresholds 10 V / 78 V, 90 C) are firmware/board facts, not chip facts: layout asserted against the DS, the value as `characterization:` where no other source exists, or as a header default with a comment.
* **L2** Biquad rounding: the DS says `round()`, the firmware truncates (`(int32_t)` cast). Tolerance +-1 LSB on each coefficient is accepted as equivalent (Q3.29 LSB = 1.9e-9); not a bug.
* **L3** Concurrency hazards (ISR `statusCheck` vs thread `updateReg`, inventory 9.34) are not testable with the sequential fake RTOS: recorded as a host-vs-target caveat, no test.
* **L4** `changeState` is defined `inline` in the .cpp (inventory 9.30): tests must not call it directly; drive states through public API/commands.

## Lead decisions on the requirement matrix (2026-11-03)

* **L5 (R4/R5)** The SPI configuration is observed through the `hspi1.Init` the fake SPI records (`HAL_SPI_Init`). The 500 ns pause cannot be observed in simulated time; the accepted proxy is "SCK <= 2 MHz, or the address byte is a separate transfer before the 4 data bytes". R5 stays `KNOWN_ISSUE` (B-T1).
* **L6 (R7)** The fake SPI supports IT transfers (`spiTransferPending`, `HostTarget::spiComplete`): R7 is testable.
* **L7 (R100)** A help string is not a value specification: command tests assert the reply value/format and the register effect (from the DS layouts in G3-G5), not the help text.
* **L8 (R103)** No observable exists for "a command during construction" on the sequential host: R103 becomes P3, not tested, reason recorded.
* **L9 (R62/R63/R72-R82)** Shared fixture helpers `reachShutdown`, `reachRunning`, `runUntilState` (in `support/TmcFixture.h`) are used by all writers; no private helpers duplicating them.
* **L10 (R67/R78)** Board/firmware constants (0x7FFF centre, the 5000 tolerance, sample filter 10000, `vmScaler`) are asserted as `characterization:`; the division by zero (B-T16) and the unsigned wrap (B-T13) stay `KNOWN_ISSUE_UB` / `KNOWN_ISSUE`.
* **L11 (R93)** Every procedure test is bounded: it must reach its end state within 60 s of simulated time, otherwise it fails with a message (a hang becomes a failing test, not a stuck run). A procedure that cannot terminate with any plain plant is recorded as a suspected bug instead of a test.
* **L12 (R14/R77)** Stay `KNOWN_ISSUE`: R14 because the firmware defines the error `tmcCommunicationError` ("TMC not responding") for exactly this case; R77 because the firmware explicitly requests `HardError` with the comment "ADC or shunt amp is broken".
* **L13 (R30/R90)** Characterization only is accepted (no external spec).

## Lead decisions on the fake (2026-11-03, after the lead review of `FakeTmc4671`)

* **L14** `TmcFixture` drives SPI1_SS1..SS3 high before the driver is built (modelling choice). Observation: the board init (`Targets/F407VG/Core/Src/main.c:1377`, CubeMX, out of scope) drives SPI1_SS2 and SS3 LOW and nothing in the firmware releases them before the first TMC_1 frame, so a second device on SS2/SS3 would be selected during TMC_1 frames. Not a driver requirement: recorded as an "other" finding and question Q3 for the user, no test.
* **L15** STATUS_FLAGS write semantic for non-zero values is an assumption of the fake; the driver only writes 0 (documented clear), so no test depends on it.
* **L16** ENABLE_IN_RAW defaults to high in the fake (board pin assumption; the DS register default 0 is the reset value of the input register). Tests that need it low set it explicitly.
* **L17** Fake register defaults spot-checked by the lead against DS §7.3 (0x0A, 0x18, 0x19, 0x26, 0x40, 0x5D, 0x5E, 0x60, 0x75): correct. Fake accepted, fidelity: frame/register file High, stacked High, flags Medium, plant hooks Low.
* **L18** B-T16 (division by zero in `calibrateAdcOffset` with ADC raw words 0, the DS default) is confirmed by the fake builder: SIGFPE on host. Its test must be `KNOWN_ISSUE_UB` and skipped-crash style (the process would terminate): use the existing crash-skip convention (`doctest::skip` + "KNOWN ISSUE (crash)") so the run continues.

## Lead decisions during writing (2026-11-03)

* **L19 (replaces the tolerance of L2)** Biquad coefficients: each within 1e-6 in real units (about 537 LSB of Q3.29) of a double-precision oracle; DC gain `(b0+b1+b2)/(2^29-a1-a2)` within 1e-3. Reason: the firmware computes in single precision (`arm_sin_f32`/`arm_cos_f32` on target), so several float ulps (64-128 LSB for |x| in [1,2)) are normal; a wrong sign, frequency or Q deviates far more.
* **L20** Feedback sign of the biquad: correct. `Filters.cpp:234-237` negates `b1`/`b2` after computing them, so the TMC `a1`/`a2` are the negated denominator coefficients, as DS eq 43-44 require. A suspected sign defect raised by two writers came from reading `calcBiquad` only up to the switch; no bug.
* **L21 (R30)** Warm/cold start has no observable (`startupType` is private and never read): untestable, no test.
* **L22** B-T23 = `rampFlux` stops short of the target when the distance is not a multiple of the step (R66, `KNOWN_ISSUE`).
* **L23 (B-T6, B-T13)** Out-of-range float-to-integer conversions are `KNOWN_ISSUE_UB`: x86 wraps or gives INT_MIN, ARM `vcvt` saturates, so the visible effect differs on target (B-T6 would not show on target at all, B-T13 would give INT_MAX). Host-vs-target caveat.
* **L24 / B-T24** Biquad enable bit: DS §7.2 0x4D p93 defines `biquad_x/v/t/f_enable` as **bit(31)** of CONFIG_DATA ("0: off, 1: on"); DS §7.3 p116 only gives the field range 0..1. The driver writes `enable & 0x1` (bit 0, `TMC4671.cpp:2207, 2228, 2249, 2270`), so by the datasheet no biquad (flux filter at init, torque filter) is ever enabled. Tests of an ENABLED filter assert bit 31 with `KNOWN_ISSUE("B-T24 ...")`; tests of a disabled filter (word 0) are plain. Question Q4 for the user (datasheet sections 7.2 vs 7.3 could be read differently; the lead reading is bit 31).
* **L25** B-T25 = the hardware-estop recovery in the `Shutdown` case is unreachable (the thread suspends itself after `emergencyStop(false)`, the critical `emergencyStop` error stays while Running again). B-T26 = over-temperature: `OverTemp` calls `stopMotor()`, which acts only in Running/EncoderFinished, so PWM stays in FOC and the motion mode stays torque in `HardError`. Both `KNOWN_ISSUE`. B-T14 additionally covers the motion mode not restored after undervoltage recovery.
* **L26** `vSenseMult` (changed by `setHwType`) is restored by the target reset of every `TmcFixture` (`host_target_system.cpp:273`); no extra fixture reset needed.
* **L27** Platform fidelity note: the fake `HAL_GPIO_ReadPin` reads IDR, which the fake does not mirror from ODR for output pins (on the real STM32 the IDR of an output pin follows the pin level). Tests read output pins through ODR / `HostTarget::outputPin`. Recorded, no change in D1.
* **L28** R90 wording corrected to the firmware behaviour (characterization). "ABN with cpr 0 stays unaligned" is unreachable through the public API (`setCpr` maps 0 to 1, `abnconf` private): no test.
* **L29** New suspected bugs from WO6: **B-T27** external-encoder alignment does a read-modify-write of read-only register 0x00 (`phiEoffsetReg` stays 0 for ext, `TMC4671.cpp:984, 1005`), `KNOWN_ISSUE`; **B-T28** `restoreFlash` uses the uninitialised `uint16_t encofs` when the flash key is missing (`TMC4671.cpp:162`), nondeterministic, no test (like R8); **B-T29** `calibrated` reads 0 right after a successful full calibration (ABN) because the encoder re-init sets `recalibrationRequired` again (`TMC4671.cpp:526` vs `1461`), `KNOWN_ISSUE`. PID autotune with an ideal no-overshoot plant needs about 100 s simulated: slow, not a defect (NEEDS REVIEW note).

## Lead decisions on the review (2026-11-03)

* **L30** B-T24 stays `KNOWN_ISSUE`: in DS §7.3 every 1-bit field has the value range 0x0..0x1 independent of its position (e.g. `sel_nclk_mclk_i_a`, bit 4 of 0x04), so the range says nothing about the bit position, which DS §7.2 gives as bit 31. Q4 to the user stays open.
* **L31** In `KNOWN_ISSUE` tests a bounded wait must not fail on timeout (a hang would count as the expected failure); the procedure's completion is asserted by a plain sibling test with the same setup.
* **L32** B-T30: `mtype` set with a value that wraps through the 8-bit cast (e.g. 259 -> 3) is accepted as BLDC; out-of-range values must be treated like the other invalid ones (NONE). Root cause: the check tests `cmd.type` instead of the value (`TMC4671.cpp:2853`).
* **L33** Mutation survivors: M58 (R70 window), M73 (0x7C cleared when enabling the index flag), M87 (encoder misc flash word after full calibration), M98 (peak filter gain) get tests; M62 and M75 are equivalent while B-T26 / B-T25 are open; M97 (hw type mask 0x1F vs 0x0F) is equivalent for every existing config (versions < 16).
* **L34** R82 combinations of private flags (`powerInitialized`, `adcCalibrated`, `encoderAligned`) cannot be isolated through the public API while Running: limitation recorded. B-T14 part 2 (motion mode not restored) has low practical effect because `turn()` re-selects torque mode on the next force update.
