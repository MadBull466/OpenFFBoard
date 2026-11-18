# Audit group A: `FakeTmc4671` (WO I0-2, 2026-11-18)

Auditor: `fake-auditor` (Opus 5.5, high), order: `I0-2_AUDIT_ORDERS.md` group A. The auditor stopped at its turn limit, then reported. Mutations ran in `../ffb-test-build/audit/group-a/` only (`base.xml`, `M*.xml`, `ffb_driver_tests -r=xml`; the fake is linked only into `ffb_driver_tests`). The lead re-derived the M1 / M2 case diffs from the XML files and read the ENABLE_IN default in the fake.

**Result: no defects.** Gaps matter for closed-loop / plant and fault cases; one documented assumption (ENABLE_IN high) has wide reach.

## Result

DS = `TMC4671-LA_datasheet_Rev2.07.pdf`; fake = `Firmware/Tests/Code_Coverage/fakes/FakeTmc4671.cpp`; driver = `Firmware/FFBoard/UserExtensions/Src/TMC4671.cpp`.

| Behaviour | Verdict | Fake | Source |
|---|---|---|---|
| 40-bit datagram (bit 39 WRnRD, address, 32 data bits, MSB first); read answer = data of the address in the same frame | ok | `:164-176, 191, 207-210` | DS §4.2.1 p17 |
| MISO byte 0 = 0x00; answer to a write frame = content before the write | ok (assumptions; driver uses neither) | `:191-197, 207` | DS Fig. 9 / 10 p19 ambiguous |
| 0x64 stored as written, flux [15:0], torque [31:16] | ok | `:245` | DS 0x64 p98 |
| 0x6B signed 32-bit, settable; a write also writes 0x68; no other mirror | ok | `:237-240, 265` | DS 0x6B p98-99 |
| **G2** 0x6B as an accumulator of input differences (a write rebases it) | gap | not modelled | DS p99 "Input position differences are accumulated" |
| 0x5E default 0x7FFF, stored | ok | `:72` | DS §7.3 |
| **G1** 0x5E limiting the targets; torque applied only in torque mode (0x63 [7:0] = 1) and only with ENI high | gap | not modelled | DS p97, p98, §4.7.1 p52 |
| CHIPINFO select-then-read, SI_TYPE "4671", SI_VERSION 0x00010003 | ok (SI_VERSION is an assumption; DS §7.3 default 0) | `:120-121, 216` | DS p17, §7.2 p77 |
| 0x7C: tests raise flags, a write ANDs | ok for the driver (it writes only 0) | `:241-243, 278` | DS 0x7C p107 |
| **G3** 0x7C live conditions that set themselves again; 0x7D mask driving the STATUS pin (FLAG / EXTI PE8) | gap | plain store | DS p107, 0x7D p110 |
| 0x76 bit 15 ENABLE_IN default high, settable | ok as documented (**assumption**, DS §7.3 default 0; board schematic not supplied) | `:78, 279` | DS 0x76 p104, §4.7.1 p52 |
| ADC_RAW 0x02 via 0x03; 0x1C PHI_E_EXT (also over the IT path) | ok | `:217, 245, 274` | DS p77-78 |
| Two chips by the single low CS; unknown CS answered with zeros | ok | `:95-101, 139-156` | |
| Write log order per chip; across chips use `HostTarget::spiTransfers(&hspi1)` (equal `timeUs` when time per frame is 0) | ok | `:199` | |
| Read-only list (all 20 "R" entries), writes ignored and recorded as violations; undocumented addresses stored and logged | ok | `:22-33, 181-195` | DS §7.1 Table 29 |
| Power-on defaults | ok (every non-zero DS §7.3 default checked) | `:49-82` | DS §7.3 p111ff |
| Header cites "DS 0x6B p98" for the mirror (it is p99); CHIPINFO selectors compared as 32 bit | doc | header | |

## Assumption mutations (copy only, `ffb_driver_tests`, 1731 cases)

| Mutation | Cases changed (lead re-derived M1, M2 from the XML) |
|---|---|
| M1 ENABLE_IN default 0 (DS value) | 9: `ENABLE_IN_RAW is high by default and follows setEnableIn`; `[R76] a temperature above the limit in Running ends in HardError`; `[R76] a temperature below the limit keeps the driver in Running`; `[R1.2 torque] set -1000 while the driver is Running writes 0xFC18 to bits 31:16`; `[R1.2 torque] set while the driver is Running selects the torque mode and does not start the motor again`; `[R1.2 torque] set while the driver is Running writes the torque to bits 31:16 and zero flux`; `[R1.4 mode] set while the driver is Running answers OK, writes the mode and the PWM start again`; two `characterization: [R1.2 torque] ...` cases |
| M2 SI_VERSION 0 | 2: the fake self-test and `[R100] tmctype get ... SI_VERSION word 0x00010003` |
| M3 write answer = value after the write; M4 0x6B -> 0x68 mirror removed; M8 read-only writes applied | 1 each, the fake's own self-tests only |
| M5 address echoed in MISO byte 0; M7 any 0x7C write clears all; M9 CHIPINFO_ADDR as u8 | 0 |

M1: every "Running" case depends on ENABLE_IN being high, because the driver's 2 s status check reads 0x76 and enters the E-stop path when it is low. The assumption is plausible (DS p52: controllers need ENI high; the board must pull it high to run) but the OpenFFBoard schematic is not supplied. The integration will rely on it in every Running case: recorded as owner question IQ5.

## For the integration (no shared change needed now)

* G1: the plant must not take 0x64 at face value: `WheelPlant` (or a typed helper on the fake) applies the torque only in torque mode, with ENI high, clamped by 0x5E. Otherwise a closed-loop test can pass while the chip would apply no or less torque.
* G2: the plant advances 0x6B by differences with int32 wrap (`onWrite(0x6B)` rebases it), so the driver's own zeroing writes are not overwritten.
* G3: E-stop / fault cases re-raise a held flag from `onWrite(0x7C)` and trigger the FLAG EXTI by hand (with the group B edge helper). FLAG = STATUS on the board: spec not supplied.
* G4 (not verified, risk): `setMicrosPerTransfer` advances the clock from inside the SPI responder; under `HOSTTEST_RTOS_EXT` that runs RTOS ticks inside an SPI call. Keep the time per frame at 0 until a work order tests it.
* `IntegrationFixture` re-installs the responder after `HostTarget::reset`; queued `spiQueueRx` bytes take precedence over the responder and never reach the fake.

Confidence for F1: open-loop register checks high; E-stop / flags medium; closed loop low-medium until G1 / G2 are built into `WheelPlant` (WO I0-7).
