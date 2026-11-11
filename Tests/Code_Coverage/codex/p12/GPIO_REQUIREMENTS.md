# P12.1 GPIO software-contract requirements

Current extension: [REGISTER_MODEL_DESIGN](REGISTER_MODEL_DESIGN.md) and [REGISTER_MODEL_RESULTS](REGISTER_MODEL_RESULTS.md) add 36 access-observation/model cases in a separate C++ target. The real-C eight-case evidence below remains valid within its original storage-only scope; earlier missing-manual statements are historical.

Scope: public HAL_GPIO_ReadPin, HAL_GPIO_WritePin and HAL_GPIO_TogglePin from the unchanged checked-in ST source. The oracle observes caller-owned ST GPIO register storage and write requests. It does not claim peripheral side effects, interrupt atomicity or electrical behavior.

| ID | Public contract / independent oracle | Permanent test |
|---|---|---|
| GIO-1 | A specified high input returns the GPIO_PIN_SET enum, not a raw shifted mask. API comment: hal_gpio.c lines 368–374. Exercise all 16 positions. | each selected high input is read as set |
| GIO-2 | Other high inputs do not make a specified low input high. Same API contract; complementary input vectors distinguish wrong pin/register/unmasked reads. | unrelated high inputs do not set the selected low input |
| GIO-3 | A read preserves the observed IDR/ODR/MODER/BSRR words. No assertion about every possible register or access trace. | reading an input leaves register storage unchanged |
| GIO-4 | Setting each pin requests only the associated BSRR BS field. HAL write API plus device-header GPIO_BSRR_BS0..BS15; bit positions come from the supplied header. | setting each output requests only its named BSRR set bit |
| GIO-5 | Resetting each pin requests only its BSRR BR field, including bit 31. API plus GPIO_BSRR_BR0..BR15 definitions; unsigned 32-bit expected words avoid signed-shift UB. | resetting each output requests only its named BSRR reset bit |
| GIO-6 | Toggling a selected currently-low output requests its set bit regardless of unselected high outputs. Toggle API and named BS fields. | toggling a selected low output requests its set bit |
| GIO-7 | Toggling a selected currently-high output requests its reset bit regardless of unselected low outputs. Toggle API and named BR fields. | toggling a selected high output requests its reset bit |
| GIO-8 | For the signed-independent mask of pins 15, 8 and 0 with only pin 15 initially high, request BR15, BS8 and BS0, without pin 7/3 requests. Expected word is composed from named header constants, not a copy of the HAL toggle equation. | a mixed toggle requests distinct set and reset bits only for selected outputs |

Set/reset cases also check observed IDR/ODR/MODER storage. Toggle cases check ODR is not directly overwritten in this storage-only boundary. These are checks of HAL requests; BSRR does not autonomously change ODR/IDR in RAM. No inference is made that physical pin values remain unchanged on hardware.

## Configuration and fidelity

- Real HAL C source, HAL API/definition/legacy headers and selected ST CMSIS device header are compiled. Only core_cm4.h is shadowed with the storage qualifiers required by these device declarations. No CPU instructions, interrupt controller or exclusive accesses are implemented.
- Owned hal_conf enables GPIO alone. F407VG, F407VG_DISCO and F411RE select their checked-in headers/sources, not their complete production-enabled module set. F411 builds establish software/header compatibility only; F411-specific electrical behavior remains unverified.
- Unsupported SYSCFG/EXTI/clock accesses abort with a diagnostic. GPIO Init/DeInit/LockPin/EXTI IRQ/callback bodies compile but are not executed. Lock protocol and W1C semantics are deliberately not modeled; plain RAM would be misleading there.
- Fixtures create fresh zero-initialized storage per test with distinct IDR/ODR/MODER/BSRR sentinels. No shared peripheral state, threads, sleeps or time-dependent successes.
- Default assertion configuration matches the supplied board configuration's disabled USE_FULL_ASSERT. The full-assert build verifies these valid inputs still pass; invalid-input assertion behavior is not yet tested.
- Fixed-width ST registers and unsigned arithmetic are used. Host storage/alignment and portable-C compilation do not establish Cortex-M instruction, bus-ordering, lock or atomicity behavior.
- DS8626 section 3.34 supports high-level GPIO categories. Detailed register write effects require the matching reference manual before a peripheral simulator is accepted.

No new suspected vendor defect is established by this slice. No firmware/vendor sources or shared build files were edited. Self-review and mutation controls are recorded; fresh independent-agent QC has not been performed for P12.1.
