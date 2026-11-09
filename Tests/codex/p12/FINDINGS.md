# P12 findings

## P12-TICK-001 — large finite delay loses its minimum interval

Low-impact boundary finding; no affected OpenFFBoard caller has been established. The checked-in STM32F4 HAL 1.8.3 source's HAL_Delay adds uwTickFreq to a finite uint32_t wait to provide its minimum-duration guard. With frequency HAL_TICK_FREQ_10HZ (a 100 ms increment) and Delay=4294967200, the intended wait addition is 4294967300, which wraps modulo 2^32 to 4. The function returns at the first sampled 100 ms instead of waiting at least the requested 4294967200 ms.

This is defined unsigned arithmetic, not UB; the intended public minimum-delay behavior is violated in the bounded host execution. No firmware/vendor repair is applied. The permanent case is named `large finite delay at ten hertz retains its requested minimum interval` with KNOWN_ISSUE(P12-TICK-001), and its first failing assertion is exactly `p12_elapsed_ticks() >= 4294967200ULL`. Lower configuration must succeed and there must be no script exhaustion or extra failed assertion for the result to count.

Source: `Firmware/Targets/F407VG/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c`, HAL_Delay API comment and body (initial snapshot lines 379–407). Local target copies have identical core C source bytes. The observation executable compiles the original C source and substitutes only its public weak GetTick time source; the genuine GetTick is separately covered by tick-state tests.

Independent derivation: 4294967200 is divisible by 100. The scripted clock values 0,0,100,4294967200,4 are consistent with that tick quantum modulo 2^32. Accumulated model time is checked in 64 bits. A large default-frequency request and normal rollover are ordinary positive controls. A scratch widened accumulator must pass the intended assertion and flip only the expected-failure marker; that experiment is a bounded oracle/repair discriminator, not a production patch.

No direct HAL_SetTickFreq call was found in the scanned FFBoard or three board Core directories. This is a lexical observation, not proof no indirect/future caller exists. Default 1 kHz does not produce this particular finite-wait addition overflow. No claim is made about every large-delay boundary, interrupt starvation or actual device execution.

Exact current executions, marker checks, scratch experiment and source hashes are recorded by [TIME_STATE_RESULTS](TIME_STATE_RESULTS.md). This document reports vendor behavior rather than changing vendor code.

## P12-I2C-001 — delayed memory-write interrupt skips an address byte

The real HAL 1.8.3 C implementation of HAL_I2C_Mem_Write_IT can omit the low byte of a 16-bit memory address when the high byte finishes transmitting before the next interrupt service. Request: device address 0x52, memory address 0x1234, payload 0x91,0x42. Observed request trace: 0x52,0x12,0x91,0x42, with a successful memory completion callback. Intended trace: 0x52,0x12,0x34,0x91,0x42. The ordinary prompt-service control sends the complete intended trace.

RM0090 Rev 22 section 27.3.3, controller transmitter (printed page 843), and section 27.6.6, SR1 BTF (printed page 863), permit TXE+BTF when DR remains empty after the preceding byte completes. The peer model's completed-byte state represents delayed ISR service, not a measured time or preemption implementation. NOSTRETCH is disabled. No ARM execution or board reproduction has been performed. F411 results are source/header compatibility evidence; its own hardware manual has not been supplied.

Cause: HAL_I2C_EV_IRQHandler sends a BUSY_TX BTF event to I2C_MasterTransmit_BTF, which transmits payload without completing EventCount==1's remaining memory-address byte. A TXE-only event instead reaches I2C_MemoryTransmit_TXE_BTF and correctly emits that byte. Test setup, prefix/payload guards and callback assertions must all pass; only the complete intended trace assertion may fail. The marked regression is `characterization: I2C memory interrupt write retains both address bytes when BTF precedes service`, using KNOWN_ISSUE and strict native XML validation.

A scratch copy under sibling `codex-p12-native32-i2c-repair/source` dispatches the MEM/EventCount==1 BTF state through the address helper first. Its `evidence/discriminator-corrected` run has all 214 ordinary cases passing and zero raw assertion failures; only the now-satisfied should_fail marker in this regression makes the executable nonzero. The earlier broad scratch guard affected a DMA path and is invalid discrimination evidence; the narrowed run supersedes it. Vendor files are unchanged. This is an isolated repair discriminator, not a proposed production patch.

F407, DISCO and F411 original-source runs reproduce the expected trace failure; native32 UBSan at the 214-case checkpoint reports no undefined-behavior diagnostic. No affected OpenFFBoard application caller has yet been established. Assess impact before assigning application severity. The separate P12-TICK-001 finding remains in the 64-bit real-C time suite; no USB FIFO defect is established.
