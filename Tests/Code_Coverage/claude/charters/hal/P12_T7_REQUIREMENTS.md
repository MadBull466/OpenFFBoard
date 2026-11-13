# P12 T7 requirements charter: FLASH / FLASH_Ex and IWDG (real ST HAL, F407VG and F407VG_DISCO)

Status: 2026-11-13 (lead). T7 follows T6b (ADC; T6a and T6b are gated and awaiting approval). Platform (`fakes/stm32_hal/HalFlash.{h,c}`, `HalIwdg.{h,c}`, the extended register bus, lead probe `test_hal_flash_probe.cpp`, shared callbacks `FlashTestSupport.h` / `test_hal_flash_support.cpp`) done; tests, review, gate pending.
Plan: `Tests/Code_Coverage/claude/plans/P12_ST_HAL_PLAN.md`. Rules as in the T1-T6 charters. Both targets compile identical files; the same test files run in both executables.

## Oracles

| Topic | Oracle |
|---|---|
| API semantics, parameters, return values, error codes, IT behaviour | UM1725 Rev 8 chapters "HAL FLASH Generic Driver" (from line 23692 of `../ffb-test-build/p12t4c-specs/um1725.txt`), "HAL FLASH Extension Driver" (24329), "HAL IWDG Generic Driver" (37199) |
| Registers, key sequences, program and erase rules, parallelism, error flags, option bytes, OTP | RM0090 Rev 22 (`rm0090.txt`) chapter 3: 3.3 / Table 5 (line 3529), 3.6 (4022), 3.7 option bytes (4239), 3.8 OTP (4688), 3.9 registers (4726); the F405/407 variants only |
| IWDG registers and key protocol | RM0090 chapter 21 (line 38561): 21.3 functional description, 21.4 registers |

Where UM1725 / RM0090 say nothing the case is `characterization:` with a NEEDS REVIEW entry (numbers from 342; writers use `NR_FLASH_<n>` and `V_FLASH_<n>` placeholders).

## Compiled code and instances

`stm32f4xx_hal_flash.c`, `_flash_ex.c`, `stm32f4xx_hal_iwdg.c`. Out of scope: `stm32f4xx_hal_flash_ramfunc.c` (compiled out on the F407: only the F410, F411, F412 and F446), the dual bank, PCROP and boot configuration code (preprocessor for other devices: gcov decides). `USE_HAL_*_REGISTER_CALLBACKS` do not exist for these modules. Weak callbacks `HAL_FLASH_EndOfOperationCallback` and `HAL_FLASH_OperationErrorCallback` are replaced by the strong ones of `FlashTestSupport.h` (never define them in a test file). `HAL_FLASH_Program` of OTP addresses is allowed by the HAL (`IS_FLASH_ADDRESS`).
Target facts: the firmware uses the flash for its emulated EEPROM; sectors, voltage range 3 (2.7 - 3.6 V, x32) are what UM1725 calls `FLASH_VOLTAGE_RANGE_3`.

## Platform (lead): `HalFlash` and `HalIwdg` models

Read the header comments of `HalFlash.h` and `HalIwdg.h`: registers, masks, key sequences, the program / erase / option byte rules, completion by `hostFlashComplete` or by poll reads (`hostFlashSetPollReads`: default completes at the first SR read with BSY set; 0 never, to test the timeout), IRQ pending, counters of ignored writes, the extended bus (`hostRegsLastAccessAddr / Bytes`: the exact address and width of a store; flash memory 0x08000000-0x080FFFFF is mapped and trapped; set-up of contents with `hostFlashFill`, reads with `hostRegsPeek`). A register outside the modelled set (for example FLASH_OPTCR1 at 0x18) is a model fault that fails the test case. The flash is filled with 0xFF at every fixture reset.
Model decisions the manual leaves open (state them in the report if a case depends on them): EOP after a failed operation that started; OPERR only with ERRIE; STRT with neither SER nor MER does nothing without a flag; SNB > 11 does nothing; the erase and program effect is applied at once and BSY only delays completion; a wrong key sequence is counted (`hostFlashBusFaults`) and locks until reset; OTP lock byte 0x00 blocks stores with WRPERR (Low); IWDG update flags clear after the n-th SR read.

## Requirement groups (one writer, one file each)

### FB (`test_hal_flash_base.cpp`): programming, interrupts, locks
`HAL_FLASH_Program / Program_IT / IRQHandler / Unlock / Lock / OB_Unlock / OB_Lock / OB_Launch / GetError`, `FLASH_WaitForLastOperation` (through the public functions), `FLASH_SetErrorCode`, the static `FLASH_Program_*` functions, the callbacks.
- FB1 Unlock / Lock / OB_Unlock / OB_Lock: key sequences (CR.LOCK, OPTCR.OPTLOCK), already unlocked case, `HAL_ERROR` when the unlock fails (after a wrong sequence done by a test with direct KEYR writes), Lock sets LOCK, OB_Lock sets OPTLOCK, return values.
- FB2 Program (poll): byte, half-word, word, double word at several addresses of several sectors and in OTP; the CR bits the HAL sets (PG, PSIZE per type) and clears afterwards, NOR result (programming 1 -> 0 only, data AND old), `HAL_BUSY` / `HAL_ERROR` / `HAL_TIMEOUT` returns (use `hostFlashSetPollReads(0)` for the timeout, a finite tick), lock handling (`pFlash`: process locked, `HAL_ERROR` on re-entry), ProcedureOnGoing / state, error codes: WRP (protected sector via OPTCR nWRP), PGA, PGP, PGS reachable through the HAL (which are, which only by direct register sequences), `HAL_FLASH_GetError` values and the clearing of the error code.
- FB3 Program_IT: EOPIE / ERRIE enabled, `pFlash.ProcedureOnGoing`, the EOP callback with the programmed address, `IRQHandler` branches (error flags first, then EOP; the callbacks' return values; flags cleared by the handler; IT disabled afterwards; the state returns to NONE; double word / word / half / byte), error path callback, `hostFlashIrqPending`.
- FB4 OB_Launch / option byte timing: OPTSTRT, wait, return codes, the commit to the option byte words (`hostRegsPeek(0x1FFFC000)`).

### FE (`test_hal_flash_ex.cpp`): erase and option bytes
`HAL_FLASHEx_Erase / Erase_IT / OBProgram / OBGetConfig / AdvOBProgram / AdvOBGetConfig` (only what is compiled), `FLASH_MassErase`, `FLASH_Erase_Sector`, `FLASH_FlushCaches`, `FLASH_OB_*` static functions (WRP enable / disable, RDP level, user config, BOR level, getters).
- FE1 Erase (poll): sector erase of sectors 0-11 singly and in ranges (NbSectors), the CR bits (SER, SNB, PSIZE per voltage range, STRT) and their restoration, contents become 0xFF only in the erased sectors, `*SectorError` = 0xFFFFFFFF on success and the failing sector otherwise, mass erase (MER, STRT), cache flush (ACR ICRST / DCRST pulse and restore of ICEN / DCEN), return codes, errors from protected sectors (WRPERR: which sector is reported), timeouts, the lock.
- FE2 Erase_IT: EOPIE / ERRIE, state machine (`ProcedureOnGoing` SECTERASE / MASSERASE, `NbSectorsToErase`, `Sector`, `VoltageForErase`), the EOP callback value per sector (the sector number, 0xFFFFFFFF at the end) driven by `hostFlashComplete` + `HAL_FLASH_IRQHandler`, mass erase end, error callback value.
- FE3 OBProgram: OptionType combinations (WRP enable / disable per sector bit, RDP level, USER bits, BOR level), the OPTCR bytes written (low byte USER / BOR, byte 1 RDP, bytes 2 / 3 nWRP), OPTSTRT not set by the function (OB_Launch does it), return codes with OPTLOCK, errors from BSY, `HAL_ERROR` paths.
- FE4 OBGetConfig / AdvOBGetConfig: OptionType, WRPSector, RDPLevel (0xAA, 0xCC, others), USERConfig bits, BORLevel; values read from OPTCR (set with `hostRegsPoke`); the compiled-out advanced functions are reported, not tested.

### IW (`test_hal_iwdg.cpp`): independent watchdog
`HAL_IWDG_Init / Refresh`, the macros `__HAL_IWDG_START`, `IWDG_ENABLE_WRITE_ACCESS`, `__HAL_IWDG_RELOAD_COUNTER` (through the functions).
- IW1 Init: NULL handle, the KR key sequence (0xCCCC, 0x5555) and the PR / RLR values for every prescaler (4 to 256) and reload (0, 1, 0xFFF), the update wait with timeout (`HAL_TIMEOUT` with `hostIwdgSetPollReads(0)`: the tick comes from `HAL_GetTick`), the final reload 0xAAAA, counter value, register state afterwards, running flag.
- IW2 Refresh: key 0xAAAA written, the counter reloaded, return value, `hostIwdgReloads`, the write access disabled afterwards (a later PR / RLR write is ignored); the expiry (`hostIwdgAdvance` to 0) requests a reset only when not refreshed.
- IW3 Register rules through the HAL: PR / RLR writes without 0x5555 ignored, SR flags, the NULL handle of Refresh, the handle with another instance value is not a case.

## Writer rules
As in the T6b brief (`../ffb-test-build/briefs/p12t7_brief.md`): expected values from UM1725 / RM0090 / hand derivation, `characterization:` + `NR_FLASH_<n>`, `KNOWN_ISSUE("V_FLASH_<n> ...")` with the first failing assertion being the defect and a spec quote + HAL line in the report; names carry the requirement id; one behaviour per case; fixture `FlashFixture`; no sleeping; gcov per target.
