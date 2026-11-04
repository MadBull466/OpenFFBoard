# P11 inventory: CMSIS-DSP (read-only survey, 2026-11-04)

Paths: REPO = /home/ben/Documents/OpenFFBoard/testing/OpenFFBoard, DSP = REPO/Firmware/Libraries/CMSIS-DSP (v1.17.0-22-g1ba19d14, git submodule, URL in REPO/.gitmodules).
Nothing in the repo was edited. Compile experiments (f16/f64) wrote only to the session scratchpad.

## 1. What the firmware calls, which modules are in scope

### 1.1 Call sites (grep of Firmware/FFBoard; Firmware/Targets has none outside vendor drivers)
| Function | Caller file:line |
|---|---|
| arm_sin_f32 | FFBoard/Src/Axis.cpp:717 (ramp-up factor); Src/EffectsCalculator.cpp:19 (MATH_SIN macro); Src/HidFFB.cpp:19 (MATH_SIN); Src/Filters.cpp:102 (biquad coefficient calc) |
| arm_cos_f32 | Src/HidFFB.cpp:20 (MATH_COS); Src/Filters.cpp:102 |
| arm_sqrt_f32 (inline in Include/dsp/fast_math_functions.h:236, not a .c file) | Src/Filters.cpp:168,169,184,185,202,203,218,219 |
| arm_biquad_cascade_df1_init_f32 | Src/Filters.cpp:22, :30 |
| arm_biquad_cascade_df1_f32 | Src/Filters.cpp:76 |
| arm_biquad_casd_df1_inst_f32 (type) | Inc/Filters.h:66 |
| arm_spline_init_f32 | Src/EffectsCalculator.cpp:602 |
| arm_spline_f32 | Src/EffectsCalculator.cpp:695 |
| arm_spline_instance_f32 (type), `arm_initialized` flag is the firmware's own | Inc/ffb_defs.h:215-216, EffectsCalculator.cpp:612,683 |
| arm_pid_init_f32 | Src/Axis.cpp:111 |
| arm_pid_f32 (inline in controller_functions.h:287) | Src/Axis.cpp:945 |
| arm_pid_reset_f32 | Src/Axis.cpp:751, :962 |
| arm_pid_instance_f32 (type) | Inc/Axis.h:489 |
| arm_std_f32 | Src/Axis.cpp:1359 (calls arm_var_f32 -> arm_mean_f32 and arm_sqrt_f32 internally) |
| arm_math_types.h only | Inc/ffb_defs.h:16, Inc/Filters.h:13, EffectsCalculator.cpp:15, HidFFB.cpp:16 |

All uses are f32. The firmware calls no f16, f64, q7, q15 or q31 function. Everything is gated by USE_DSP_FUNCTIONS.

### 1.2 What the firmware build links (REPO/Firmware/Makefile:161-178)
`USE_DSP_FUNCTIONS` adds `-DARM_MATH_LOOPUNROLL -DARM_MATH_CORTEX_M4` and compiles with a wildcard `arm_*.c` over:
BasicMathFunctions, CommonTables, FastMathFunctions, FilteringFunctions, ControllerFunctions, InterpolationFunctions, StatisticsFunctions.
Then `filter-out %_f16.c %_f64.c %_mve.c`. So the shipped firmware contains the f32/q31/q15/q7 sources of those 7 module directories, but SupportFunctions is NOT linked, and no Matrix/Transform/Complex/Window etc.
(Flags: -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard, C: -O2 -std=gnu11.)

### 1.3 Modules in scope (user decision: modules the firmware uses, all functions, all types, + helpers they depend on)
| Module | Why | Source dir |
|---|---|---|
| FastMathFunctions | sin/cos/sqrt | DSP/Source/FastMathFunctions (23 .c) |
| FilteringFunctions | biquad df1 | DSP/Source/FilteringFunctions (116 .c, very large: FIR, conv, correlate, LMS, IIR lattice, decimate, ...) |
| InterpolationFunctions | spline | DSP/Source/InterpolationFunctions (12 .c) |
| ControllerFunctions | PID | DSP/Source/ControllerFunctions (8 .c + 11 inline functions in the header) |
| StatisticsFunctions | arm_std_f32 / arm_var_f32 (the earlier "expected" list did not contain this one; it is needed) | DSP/Source/StatisticsFunctions (93 .c) |
| CommonTables | sinTable_f32/q15/q31, recip tables, atan/log tables | DSP/Source/CommonTables/arm_common_tables.c (45,787 lines) |
| BasicMathFunctions (helper only) | linked in firmware Makefile but not called; needed by: arm_divide_q15/q31 -> arm_abs_q15/q31; arm_logsumexp_dot_prod_f16/f32 -> arm_add_f16/f32 | DSP/Source/BasicMathFunctions (68 .c) |
| SupportFunctions | NOT needed by any in-scope function (checked: no in-scope source calls arm_copy/fill/convert). Not linked in firmware. Out unless you want it | |
| Matrix, Transform, Complex, Window, Bayes, Distance, SVM, Quaternion | not linked by firmware; out | |

### 1.4 Internal dependencies between in-scope sources (computed by script over the .c files)
- arm_std_{f16,f32,f64,q15,q31} -> arm_var_* (and arm_sqrt_* for non-f64). arm_var_{f16,f32,f64} -> arm_mean_*. arm_var_q15, arm_mean_q31 -> arm_div_int64_to_int32 (inline, dsp/utils.h).
- arm_rms_* -> arm_power_*, arm_sqrt_* (arm_sqrt_q15/q31 are .c files in FastMath; arm_sqrt_f32/f16 inline).
- arm_logsumexp_f16/f32 -> arm_max_no_idx_*; arm_logsumexp_dot_prod_* -> arm_add_*, arm_logsumexp_*.
- arm_atan2_{f16,f32,q15,q31}: static helpers arm_atan_*, arm_atan_limited_* inside the same .c; q15/q31 also arm_divide_*; arm_divide_q15/q31 -> arm_abs_q15/q31 (BasicMath).
- arm_sin_f32/arm_cos_f32/arm_sin_cos_f32 -> sinTable_f32, FAST_MATH_TABLE_SIZE (CommonTables); q15/q31 variants -> sinTable_q15/q31.
- arm_lms_norm_* -> recipTable and arm_recip_q15/q31 (Filtering MISC).
- arm_spline_interp_*.c are thin wrappers; real code is in arm_spline_interp_f32.c / ..._init_f32.c; no other dependency.
- arm_vlog_q15/q31 -> arm_scalar_log_q15/q31 (static in the file). arm_vlog_f16 -> typecast inline helpers.
- arm_levinson_durbin_q31 -> arm_divide_q15. arm_conv_*_opt_* / arm_correlate_*_opt_* -> arm_copy_q15 / arm_fill_q15 (SupportFunctions: this is the only place SupportFunctions is needed, and only if the "opt" Filtering variants are tested).
- Compile-time mve/neon branches (`arm_biquad_cascade_df1_mve_init_f32`, `arm_fir_interpolate2_f32_mve`, `arm_biquad_cas_df1_32x64_*`) are not compiled on the host (MVE off).

### 1.5 Public functions per module and type (from the .c file names; `init` files are separate public functions)
(unit = one .c file = one (function,type) pair; "inline" = defined in the header only)
- **FastMath (23 .c + 2 inline = 25 units):**
  - f32: atan2, cos, sin, vexp, vlog; inline: sqrt (header, `__GNUC_PYTHON__` branch uses sqrtf, fast_math_functions.h:258)
  - f64: vexp, vlog
  - f16: atan2, vexp, vinverse, vlog; inline sqrt_f16 (fast_math_functions_f16.h)
  - q31: atan2, cos, divide, sin, sqrt, vlog; q15: atan2, cos, divide, sin, sqrt, vlog
- **Controller (8 .c + 11 inline = 19 units):** .c: pid_init + pid_reset for f32, q31, q15; sin_cos_f32; sin_cos_q31. Inline in controller_functions.h: pid_f32 (:287), pid_q31 (:321), pid_q15 (:368), clarke/inv_clarke/park/inv_park f32 and q31 (:483-:755). No q15 park/clarke, no f16.
- **Interpolation (12 .c):** linear_interp f32/q31/q15/q7/f16 ; bilinear_interp f32/q31/q15/q7/f16 ; spline_interp_init_f32 + spline_interp_f32 (f32 only; header declares arm_spline_init_f32 / arm_spline_f32). No f64.
- **Statistics (93 .c):**
  - f32 (19): absmax, absmax_no_idx, absmin, absmin_no_idx, accumulate, entropy, kullback_leibler, logsumexp, logsumexp_dot_prod, max, max_no_idx, mean, min, min_no_idx, mse, power, rms, std, var
  - f16 (19): same 19
  - f64 (16): same minus logsumexp, logsumexp_dot_prod, rms
  - q31 (14), q15 (14): absmax, absmax_no_idx, absmin, absmin_no_idx, max, max_no_idx, mean, min, min_no_idx, mse, power, rms, std, var
  - q7 (11): absmax, absmax_no_idx, absmin, absmin_no_idx, max, max_no_idx, mean, min, min_no_idx, mse, power (no rms/std/var for q7)
- **Filtering (116 .c):** by type f32 26, q31 30, q15 33, q7 10, f64 7, f16 10 (counts of .c files, init counted). Families:
  biquad_cascade_df1 (f32,f16,q31,q15, fast_q31, fast_q15, 32x64_q31) + init each; biquad_cascade_df2T (f32,f64,f16) + init; stereo_df2T (f32,f16) + init; conv (f32,q31,q15,q7, fast, fast_opt, opt), conv_partial (same set); correlate (f32,f64,f16,q31,q15,q7 + fast/opt); fir (f32,f64,f16,q31,q15,q7, fast_q15/q31) + init; fir_decimate (f32,f64,q31,q15,fast) + init; fir_interpolate (f32,q31,q15) + init; fir_lattice (f32,q31,q15) + init; fir_sparse (f32,q31,q15,q7) + init; iir_lattice (f32,q31,q15) + init; levinson_durbin (f32,f16,q31); lms (f32,q31,q15) + init; lms_norm (f32,q31,q15) + init. Full list: `ls DSP/Source/FilteringFunctions/arm_*.c`.
- **CommonTables:** not functions; data. arm_common_tables.c is ~45k lines (all f32/q31/q15 twiddle, bit-reverse, sin, recip, log tables). The host already compiles it (all 4 executables).
- **BasicMath helper units used:** arm_abs_q15, arm_abs_q31, arm_add_f32, arm_add_f16 (4). Full BasicMath is 68 .c (not requested).

## 1.6 What is compiled into the host test executables today (REPO/Firmware/Tests/CMakeLists.txt)
- `FFB_TESTS_USE_DSP` AUTO/ON/OFF (:15-16); AUTO = ON when `DSP/Include/arm_math_types.h` exists (:107-113); `DSP_DIR` :105; message :114.
- Flags (identical in all 4 blocks): `target_compile_definitions(... USE_DSP_FUNCTIONS ARM_MATH_LOOPUNROLL __GNUC_PYTHON__)` and include dirs `DSP/Include`, `DSP/PrivateInclude`. No ARM_MATH_CORTEX_M4, no ARM_MATH_NEON/MVE/HELIUM, no DISABLEFLOAT16 (so f16 is gated off by default, see section 2). There are no per-source compile options for the DSP files (they get the target's global flags, ASan/UBSan in san builds).
- Source lists (each executable lists its own):
  | Executable | CMake lines | DSP .c files |
  |---|---|---|
  | ffb_unit_tests | 123-129 | arm_common_tables.c, arm_sin_f32.c, arm_cos_f32.c, arm_biquad_cascade_df1_f32.c, arm_biquad_cascade_df1_init_f32.c, arm_spline_interp_f32.c, arm_spline_interp_init_f32.c |
  | ffb_target_tests | 362-374 | same 7 + arm_pid_init_f32.c, arm_pid_reset_f32.c, arm_std_f32.c, arm_var_f32.c |
  | ffb_usb_tests | 715-726 | same 11 |
  | ffb_driver_tests | 826-837 | same 11 |
  ffb_flash_tests / ffb_rtos_tests: no DSP. `test_cmsis_dsp.cpp` lives only in cases_target (so it is built into ffb_target_tests; with USE_DSP off the file is empty).
- Not compiled today: everything else. arm_var_f32.c calls arm_mean_f32 only in its MVE (line 85) and Neon (line 139) branches, so on the host (portable branch, lines 183-290) it is self-contained; that is why the 11-file lists link. For P11 the other var/std/mean units need their own mean/var/sqrt sources (see 1.4).
- Host differences vs target (important for P11): host defines `__GNUC_PYTHON__`, which in arm_math_types.h (:112-,:132-) removes the dependency on CMSIS core, so `ARM_MATH_DSP` (set at :144 only when `!__GNUC_PYTHON__`) is NOT defined and the portable scalar code path (dsp/none.h intrinsics) is used. The target defines ARM_MATH_CORTEX_M4 + __ARM_FEATURE_DSP -> ARM_MATH_DSP, i.e. the SIMD intrinsics (__SMLAD, __QADD16 ...) are used in q15/q7/q31 kernels and the loop-unrolled DSP branches. Fixed-point results should match bit for bit, but the host cannot prove the target path. f32 may differ in the last bits (VFMA contraction, see existing header comment in test_cmsis_dsp.cpp:36-45).
- Known issue: arm_spline_f32 over-read (arm_spline_interp_f32.c:223), ASan; existing cases exclude it in sanitizer builds (CMake F1b :159-171). ARM's own spline tests use ASSERT_EMPTY_TAIL on output only, so ARM's tests do not catch it either.

## 2. float16 and float64 on the host

### 2.1 How CMSIS-DSP gates f16 (Include/arm_math_types_f16.h:44-64)
- `float16_t` is defined (as `typedef __fp16 float16_t;`) and `ARM_FLOAT16_SUPPORTED` is set only if `!DISABLEFLOAT16` and (`__ARM_FP16_FORMAT_IEEE` or `__ARM_FP16_FORMAT_ALTERNATIVE`) is defined by the compiler. (Or `__ARM_FEATURE_MVE & 2`.) Every *_f16.c body is wrapped in `#if defined(ARM_FLOAT16_SUPPORTED)` (e.g. arm_atan2_f16.c:34).
- Constants F16_MAX etc. use `_Float16` / `__FLT16_MAX__` (arm_math_types_f16.h:138-156).
- On the host (x86_64) plain compile: `__ARM_FP16_FORMAT_IEEE` is undefined -> all f16 sources compile to empty objects (verified: 0 symbols, no errors, gcc 16 and clang 23).
- Makefile for the target excludes `%_f16.c %_f64.c` (Makefile:177), so f16/f64 never ship.

### 2.2 Does the host have a 16-bit float?
- GCC 16 and clang 23 on x86_64 Linux both define `__FLT16_MAX__` / support `_Float16` (verified with `-dM -E`). x86 GCC has no `__fp16`.
- **Verified recipe (no source edit, only compile defines):** `-D__ARM_FP16_FORMAT_IEEE=1 -D__fp16=_Float16`. With it, all 51 `*_f16.c` files of FastMath/Filtering/Interpolation/Statistics/BasicMath/Support compile without errors in gcc 16 and clang 23 (-O0, -Wall), and a small program linking arm_std_f16.c + arm_var_f16.c ran correctly with both compilers (std of {1,2,3,4} = 1.29102, sizeof(float16_t)=2).
- Caveat: `_Float16` arithmetic on plain x86-64 (no AVX512-FP16) is evaluated via float with conversion helpers from libgcc/compiler-rt (links fine on this host). Intermediate rounding: with GCC the default `-fexcess-precision=fast`/`standard` for `_Float16` may keep intermediates in float, so results may differ from the target (ARM `__fp16` arithmetic on a Cortex-M4 FPv4-sp is not even available in hardware; the firmware does not ship f16). Treat f16 as "reference semantics of the C code", check with ARM's wide f16 tolerances (SNR 27..60 dB, REL 1e-3..6e-3).
- MinGW/Windows: `_Float16` needs GCC >= 12 (x86); probably available in MinGW GCC 13+. Needs a CMake feature check (`check_c_source_compiles`) and a skip (not a fail) when missing, otherwise the Windows CI could break. Clang/MSVC not in use. If `_Float16` is not available the f16 tests must compile out like `test_cmsis_dsp.cpp` does for DSP-off.
- The test side (C++): use `_Float16` and a pattern loader that converts the 16-bit hex word (f16 patterns store the raw IEEE half bit pattern, e.g. 0x39a8 = 0.707107) with memcpy. Files in C++ need the DSP headers `#include "arm_math_f16.h"` (c++ compile of arm_math_f16.h with the macro hack: untested; the C test compiled fine, C++ untested).
- Sanitizers: untested for f16 (ASan/UBSan should be fine).
- Build recipe: put f16 sources in a separate static library `cmsis_dsp_f16` (only when the check passes) with those two defines PRIVATE; the F16 test file also needs `-D__ARM_FP16_FORMAT_IEEE=1 -D__fp16=_Float16` at its include of `arm_math_f16.h`.

### 2.3 f64
- Plain C `double`; no gating. Verified: arm_vexp_f64, arm_vlog_f64, arm_biquad_cascade_df2T_f64 (+init), arm_std_f64, arm_var_f64 compile without errors (gcc 16, clang 23) with the existing defines. Nothing special needed except adding sources to a CMake list (and ARM reference thresholds for f64 are very tight: SNR 310 dB, REL 2e-16 (FastMath), REL 4e-15 (Stats), 1.2e-3 (BIQUAD F64!)). x87 vs SSE: x86-64 uses SSE2, so no excess precision; MinGW 64-bit is the same; 32-bit x86 builds would differ (not used).
- 64-bit ints: q63 accumulators in stats; fine on 64-bit host. FastMathQ63 patterns (Norm64To32, div) are for helpers `arm_div_int64_to_int32` / `arm_norm_64_to_32u` which are inline in dsp/utils.h (public header functions): consider in scope as "helpers".

## 3. Existing tier C tests
- File: REPO/Firmware/Tests/cases_target/test_cmsis_dsp.cpp, 857 lines, 53 TEST_CASEs, suite name `CmsisDsp`, only with USE_DSP_FUNCTIONS, in `ffb_target_tests`. No other file calls arm_ functions in a DSP-semantic way (cases/test_filters.cpp, test_effects_calculator_gaps.cpp, test_axis_torque.cpp, test_axis_speed_limiter.cpp, test_serial_ffb.cpp, driver tests only exercise the firmware that uses them).
- Coverage (all f32): arm_sin_f32 + arm_cos_f32 (10 cases, lines 263-349: range 0..4pi, negative, odd/even, period, multiples of pi/2, large, just below 0, int32 overflow), arm_sqrt_f32 (5 cases, 360-395), biquad df1 init + run (10 cases, 403-520: reference DF1, sign convention, delays, block sizes, empty block, cascade, state, zero stages), spline init + run (14 cases, 537-669: natural and parabolic runout vs a textbook solver, lines/parabolas, knots, curvature, extrapolation, 2-point cases; two characterization cases = NEEDS REVIEW 161 etc.), PID init/reset/run (7 cases, 681-775), arm_var_f32 (6 cases) and arm_std_f32 (2 cases) (789-850).
- Oracle: independent double-precision references computed in the test (std::sin/cos, hand DF1, dense Gaussian elimination for the spline, positional PID, two-pass variance). No ARM patterns used.
- Tolerances: trig 2.0e-5 + 1.2e-7|x| (table interp error (2pi/512)^2/8 = 1.88e-5); others 2e-6 abs/rel; exact for sqrt (correctly rounded).
- Mutation score from P9: 61/62.
- Not covered: every other function/type (all of sections 1.5).

## 4. ARM reference patterns (DSP/Testing)

### 4.1 Layout (DSP/Testing/Patterns/DSP/<Module>/<Suite>/*.txt), 4,338 files in the repo, 172 MB total (most of it Matrix/Transform), all tracked in the submodule's git
Sizes of the in-scope parts: FastMath 91 files 3.8 MB; Filtering/BIQUAD 49 files 175 KB, FIR 42 files 431 KB, DECIM 48 files 846 KB, MISC 504 files 302 KB; Interpolation 95 files 89 KB; Controller 4 files 22 KB; Stats 175 files 316 KB; (BasicMaths 992 KB, Support 304 KB if wanted). In-scope total about 6 MB; biggest single file 630 KB (FastMathF64/Atan2Input1_f64.txt).
- FastMath/: FastMathF16, F32, F64, Q15, Q31, Q63 (one dir per type; same file stems). Files: Angles1, Sin1, Cos1, SqrtInput1, Sqrt1, LogInput1, Log1, ExpInput1, Exp1 (not in q15/q31), Atan2Input1, Atan2Ref1, Numerator1, Denominator1, DivisionValue1, DivisionShift1_s16, Inverse1 (f16/f32/f64), RecipInput1/RecipRef1/RecipShift1_s16 (q15/q31), Samples1 (benchmark input). FastMathQ63: DivDenInput1_s64, DivNumInput1_s32, DivRef1_s32, Norm64To32_Input1_u64, RefNorm64To32_Norms1_s16, RefNorm64To32_Vals1_s32.
- Filtering/BIQUAD/BIQUADF16|F32|F64|Q15|Q31: BiquadInput1, BiquadCoefs1, BiquadOutput1 (reference test, 3 stages), AllBiquadInputs2 / AllBiquadCoefs2 / AllBiquadRefs2 / AllBiquadConfigs2_s16 (random pattern tests), AllBiquadStereoInputs2 / StereoRefs2 (f32, f16 only), Coefs1, Samples1 (benchmarks). No Q7.
  Filtering/FIR (F16,F32,F64,Q15,Q31,Q7), DECIM (F16,F32,F64,Q15,Q31), MISC (F16..Q7: conv, correlate, lms, iir lattice, levinson, sparse fir, ...: 500+ files `Reference<n>`, `Inputs..`).
- Interpolation/InterpolationF16|F32|Q15|Q31|Q7: Input1 (linear query points; q31 form for the q-types), Input2 (bilinear x/y), Config2_s16 (matrix size), YVals1 (linear table), YVals2 (bilinear matrix), Reference1 (linear ref), Reference2 (bilinear ref), and the spline set 3 (square), 4 (sine), 5 (ramp): InputX<n>/InputY<n> (knots), OutputX<n> (query), Reference<n>. Spline files exist in F32 (used) and in the other type dirs (generated, unused: there is no f16/q spline function; desc_f16.txt Interpolation F16 has only 2 ids, linear and bilinear).
- Controller/ControllerF16|F32|Q15|Q31: only `Samples1_<t>.txt` (benchmark input, 256 random samples). **No references, no ARM correctness tests for arm_pid_*, arm_sin_cos_*, clarke/park.** The generator Controller.py writes only these samples; ARM only benchmarks them (Testing/Source/Benchmarks/ControllerF32.cpp, Q15, Q31).
- Stats/StatsF16|F32|F64|Q15|Q31|Q7: Input1 (max/min), Input2 (mean/power/rms/std/var), MaxVals1/MaxIndexes1_s16, MinVals3/MinIndexes3, MeanVals2, PowerVals4, RmsVals5, StdVals6, VarVals7, Input22/Dims22/RefEntropy22, Input23/Dims23/RefLogSumExp23, InputA24/B24/Dims24/RefKL24, InputA25/B25/Dims25/RefLogSumExpDot25, AbsMaxVals26/Indexes26, AbsMinVals27/Indexes27, MSEVals28, InputNew1/InputNew2 (long/stability tests), InputAccumulate1/Config1_s16/RefAccumulate1, InputBench1/2.
- Not covered by any ARM pattern within the in-scope modules: PID (arm_pid_init/reset/pid, all types), arm_sin_cos_f32/q31, clarke/inv_clarke/park/inv_park, arm_spline in f16 (not existing), arm_atan2 in f64 (no function), arm_biquad_cascade_df1_fast_q15/q31 (desc.txt BIQUAD Q15/Q31 list only `arm_biquad_cascade_df1` and `_32x64`), most Filtering "fast"/"opt" variants are in MISC/FIR (covered by test ids named *_fast_*; see desc.txt MISC lists).
  Bi-directional: arm_sin_f32 / arm_cos_f32 / arm_sqrt_f32 / arm_vlog/vexp / atan2 / divide / sqrt_q15/q31 / sin,cos q15,q31 are covered; arm_spline f32 covered (3 ids).

### 4.2 Text file format (generated by DSP/Testing/PatternGeneration/Tools.py `_writeVector*`, lines 675-1060)
```
<type letter>        W | H | B | D       (32-bit word, 16-bit half, 8-bit byte, 64-bit double word)
<count>              number of values (decimal)
// <human readable value>    one comment line before every value
0x<hex>              the value, one per line, same count times
```
- Total lines = 2 + 2*count (e.g. BiquadInput1_f32: count 100, 202 lines). ASCII, LF endings, no tabs.
- f32: `W`, raw IEEE-754 binary32 bit pattern, minimal hex digits for zero (`0x0`), 8 digits otherwise (e.g. `0x3f3504f3` = 0.707107). Lowercase hex in f32/f64/f16, UPPERCASE hex in q-types and S/U types (`0x5A82799A`, `0x00A4`).
- f64: `D`, raw binary64 bit pattern, 16 hex digits (`0x3fe6a09e667f3bcc`), `0x0` for zero.
- f16: `H`, raw binary16 pattern (`0x39a8`).
- q31: `W`, two's complement 8 digits uppercase (`0x7FFFFFFF`, `0xEEF904F8`); q15: `H` 4 digits (`0x00A4`); q7: `B` 2 digits (`0x7F`); s16/u16 config files `H` decimal values in hex (`0x0003`); s32/u32 `W`; s64/u64 `D` (`0x7FFFFFFFFFFFFFFF`); the `// ` comment holds the decimal value (rounded to 6 decimals for floats, so NEVER parse the comment; parse the hex).
- Q31/Q15 values are produced by `np.round(x * 2^31)` etc. with saturation (Tools.py, FastMath.py `q31accuracy`).
- The bare comment `// 0.707107` makes the files human-checkable but lossy.

### 4.3 How ARM's framework uses them
- DSP/Testing/desc.txt (and desc_f16.txt for the f16 suites) declare `group` / `suite` with `class =` and `folder =` and bind pattern ids to files (e.g. `Pattern ANGLES1_F32_ID : Angles1_f32.txt`, `Output OUT_F32_ID : Output`) and list `Functions { <test name>:<C++ method> }` (an id per line, with options like `nb=3`, `nb=4n`, `nb=4n+1`, `long`, `stability`).
- Test code: DSP/Testing/Source/Tests/<Suite>.cpp (e.g. FastMathF32.cpp, BIQUADF32.cpp, InterpolationTestsF32.cpp, StatsTestsF32.cpp). A test reads `input.ptr()` and `ref`, calls the CMSIS function, writes into `output`, then asserts. Block size variations (3, 8, 11 samples for the f32 stats `nb=3|4n|4n+1`) come from `reload(ID, mgr, nbSamples)` in `setUp` plus a `refOffset` into the reference table (Stats).
- BIQUAD: `blockSize = inputs.nbSamples() >> 1`; init once with 3 stages (coefs 5 per stage: b0 b1 b2 a1 a2, a1/a2 are sign-negated, same CMSIS convention as `-sos[:,4:]` in BIQUAD.py:35-37), filter two blocks in sequence to test state, compare. The "rand" tests use AllBiquadConfigs2_s16 pairs (numStages, blockSize) and pattern arrays.
- Spline (InterpolationTestsF32.cpp): `arm_spline_init_f32(&S, ARM_SPLINE_PARABOLIC_RUNOUT, inpX, inpY, 4, coef, buf); arm_spline_f32(&S, outX, outp, 20);` Note only PARABOLIC_RUNOUT is tested for square, sine, ramp; natural splines have no ARM reference.
- Pattern ids -> test functions are compiled into a generated header; the binaries are produced by processTests.py/preprocess.py with a Python runner (not needed for us).

### 4.4 Tolerances ARM uses (defines in Source/Tests/*.cpp; reference patterns "generated with a double precision computation")
Assertions: ASSERT_SNR (SNR dB = 10 log10(sum ref^2 / sum (ref-out)^2), NaN -> 0 so the test fails, zero error -> 100000, FrameworkSource/Error.cpp:516), ASSERT_REL_ERROR, ASSERT_CLOSE_ERROR(abs, rel) with pass condition `|val-ref| <= abs + rel*|ref|` (Error.cpp:355-410), ASSERT_NEAR_EQ (integer: |a-b| <= thresh, used for q-types), ASSERT_EQ (exact: max/min/indices), ASSERT_EMPTY_TAIL (output buffer overrun check).
| Suite | SNR (dB) | other |
|---|---|---|
| FastMath F32 | 119 (atan2 120, but commented out) | CLOSE: abs 1e-5, rel 1e-6; atan2 abs/rel 5e-7 |
| FastMath F64 | 310 | abs 2e-16, rel 2e-16 |
| FastMath F16 | 60 (log 40, atan2 60) | abs/rel 1e-3; atan2 abs 2e-3, rel 1e-3 |
| FastMath Q31 | 100 | sqrt 7, general ABS_ERROR 2200, div 2, log 2, atan 3, recip 10 (LSB) |
| FastMath Q15 | 69 | sqrt 6, general 10, log 3, atan 3, div 2, recip 2 (LSB) |
| FastMath Q63 | 100 | abs 0 (exact) |
| BIQUAD F32 | 94 | REL 2e-4 |
| BIQUAD F64 | 98 | REL 1.2e-3 |
| BIQUAD F16 | 27 | REL 5e-2, ABS 1e-1 |
| BIQUAD Q31 | 115 | abs 1000 LSB |
| BIQUAD Q15 | 30 | abs 500 LSB |
| Interpolation F32 | 119 | REL 8e-5 |
| Interpolation F16 | 44 | REL 6e-3, ABS 7e-3 |
| Interpolation Q31 / Q15 / Q7 | 100 / 70 / 20 | abs 2000 / 2 / 2 LSB |
| Stats F32 | 120 | REL 1e-5 |
| Stats F64 | 300 | REL 4e-15 |
| Stats F16 | 48 (KL 40) | REL 6e-3, accumulate 7e-3, KL abs 1e-2 rel 5e-3 |
| Stats Q31 / Q15 / Q7 | 100 / 50 / 20 | abs 100 (q31), 100 (q15), 20 (q7); q63 accumulators abs 2^18 / 2^17; q7 q31 results 2^15; MSE variants (100, 100, 6) |
Pattern for the scheme to copy: the pair (SNR threshold on the whole block) + (per-sample abs/rel or integer LSB bound). Index and max/min outputs are exact (ASSERT_EQ). Note the existing own-tolerance of test_cmsis_dsp.cpp (sin/cos 2e-5 + ...) is looser than ARM's FastMath F32 CLOSE (1e-5 abs, 1e-6 rel) because ARM only checks a 23-point pattern (0, pi/4 ... 4pi) where the table error is small. Don't copy the 1e-5 bound to dense sweeps.

### 4.5 Generator scripts (DSP/Testing/PatternGeneration/*.py, run via GenAll.py; deps: numpy 1.24, scipy via statsmodels, in DSP/Testing/requirements.txt; NOT needed to run the tests)
- FastMath.py (numpy: np.sin/np.cos/np.sqrt/np.log/np.exp, np.arctan2 style; division and log reference emulate the fixed point result via `divide(format,x)`/`initLogValues`; FastMath.py:100-175).
- BIQUAD.py (scipy.signal: `signal.zpk2sos` for the coefficient design, `signal.sosfilt` as reference = a double-precision direct computation; `np.random.randn` input, so the patterns are one random draw stored in the file).
- Interpolate.py (scipy.interpolate: `interp1d` for linear, `interp2d` for bilinear, `CubicSpline` for the spline sets; double precision).
- Stats.py (numpy: np.var(ddof=1), np.std(ddof=1), np.mean, np.max, ... Stats.py:355-380; scipy logsumexp, entropy, KL).
- Controller.py: only writes 256 random normalized samples (no reference).
- FIR.py, Decimate.py, Convolutions.py (scipy.signal.lfilter/convolve/correlate) for the other Filtering families.
- Tools.py: `Config.writeInput/writeReference/writeReferenceS16 ...` and the text format in 4.2.

## 5. Size estimate
Public (function,type) units, counting each .c file (init functions and f16/f64 included) plus inline header functions:
| Module | .c units | inline | total | ARM test ids (desc.txt + desc_f16.txt) |
|---|---|---|---|---|
| FastMath | 23 | 2 (sqrt f32/f16) | 25 | 11+12+10+10+2 (F64,F32,Q31,Q15,Q63) + 13 (F16) = 58 |
| Controller | 8 | 11 | 19 | 0 |
| Interpolation | 12 | 0 | 12 | 5+2+2+2 + 2 (F16) = 13 |
| Statistics | 93 | 0 | 93 | 53 (F64) + 53 (F32) + 45 (Q31) + 45 (Q15) + 40 (Q7) + 53 (F16) = 289 |
| Filtering | 116 | 0 | 116 | 401 + 49 (f16) = 450 (BIQUAD alone: F64 2, F32 5, Q31 2, Q15 1, F16 5 = 15; the rest FIR 6, DECIM 7, MISC ~420) |
| BasicMath helpers | 4 | 0 | 4 | (not tested via helpers; 35-59 ids/type exist for full BasicMath) |
| Total | 256 | 13 | ~269 | 810 ids (about 760 in Statistics+Filtering+Interpolation+FastMath) |
Splitting by type over all five modules (.c only): f32 ~57, f16 ~44, f64 ~25, q31 ~55, q15 ~57, q7 ~33 (Filtering 116 includes ~40 init units: a test per init is mostly covered through its run function).
The existing suite covers about 11 units (sin, cos, sqrt, biquad df1 + init, spline + init, pid init, reset, pid, var, std, all f32). Remaining about 258 units. Filtering (116 units, 450 ARM ids) and Statistics (93 units, 289 ids) are 80% of the work. Suggested module groups (stop after each): (1) FastMath, (2) Interpolation + Controller (Controller needs own derivation, no ARM patterns), (3) Statistics, (4) Filtering-BIQUAD/FIR/DECIM, (5) Filtering-MISC (conv, correlate, lms, iir lattice, levinson, sparse, lattice), (6) BasicMath helpers (only 4 units). f16 can be a sub-step per group (feature-checked).

## 6. Practical notes for test writing
- Reading patterns at runtime: add a CMake define `FFB_CMSIS_PATTERN_DIR="${DSP_DIR}/Testing/Patterns/DSP"` (absolute, forward slashes also on Windows/MinGW) to the test target; in the test, `std::ifstream(dir + "/FastMath/FastMathF32/Sin1_f32.txt")`. Parse: first line type letter, second line count, then loop: skip lines beginning with `//`, parse the next token with `std::stoull(tok, nullptr, 16)`. Convert by memcpy into float/double/uint16 (f16 via memcpy to `_Float16`)/int32 etc. Fail the test (not skip) if a file is missing when DSP is on, because CI checks out the submodule.
- Patterns are part of the submodule checkout: the repo's `.gitmodules` points to ARM-software/CMSIS-DSP, the Testing/Patterns files are tracked there (`git ls-files Patterns` = 4,338 files), and CI checks out recursive submodules (`.github/workflows/build-firmware.yml:36 submodules: 'recursive'`; release.yml:32,57 'true'). The Tests CMake guard `EXISTS DSP/Include/arm_math_types.h` should also check that the Patterns directory exists; treat a submodule without Testing (sparse/shallow, if someone removes it) as DSP-tests-off. Which workflow runs the host tests was not re-checked here (the P0 notes in PROGRESS.md record what CI runs).
- Parsing cost: 6 MB of text in total for in-scope modules; each file parses in ms. Load lazily per test case (do not parse all at start). Use a helper in `Firmware/Tests/support/` (new, e.g. `PatternFile.h`) plus a test of the loader itself.
- Windows/MinGW: the path define must not contain a backslash; use `file(TO_CMAKE_PATH)`; open files in binary mode so CRLF checkout (core.autocrlf=true) does not matter, and strip `\r` while parsing.
- Tolerance scheme to copy (section 4.4): per suite one SNR threshold on the full block and one per-sample bound (abs + rel*|ref| for floats, integer LSB for fixed point), exact for index/min/max outputs, a tail-empty check (guard words after the output buffer). Use ARM's thresholds as the starting point, compute SNR in double, NaN -> fail. For f32 tests the host and target both run IEEE binary32; ARM's thresholds were validated on Cortex-M, so they have headroom. Note the ARM F64 biquad REL is 1.2e-3 (poles at radius up to 0.98 amplify error), and ARM's reference is a single random draw (stored), so the test is deterministic.
- Expected values must keep to the project rule: ARM pattern values are the oracle, with a statement in the test header (file path, generator, tolerance). Where ARM has no pattern (Controller, sin_cos, clarke/park, natural spline, df1_fast_*, df1_32x64 edge cases, saturation cases) use hand derivation or math definition in the comment.
- Where ARM tolerance itself looks loose or ARM code is mis-calibrated, record as characterization / NEEDS REVIEW. Potential flags already visible: FastMath Q31 `ABS_ERROR 2200 LSB` for sin/cos is a very loose bound; BIQUAD F64 REL 1.2e-3 is looser than f32's 2e-4; ARM's own spline test does not check over-reads.
- Compile separation: put the new f32/q-type sources in a new executable `ffb_dsp_tests` (separate dir `cases_dsp/`, glob outside `cases/test_*.cpp`, like cases_target) so the 4 existing executables keep their source lists unchanged (their compile commands are checked identical to `../ffb-test-build/d0-ref/`). Build DSP sources as a static library per type group (f32/q, f16 with hack defines, f64) with the same sanitizer flags; Do not use `FIRMWARE_SOURCES`. New CMake options need `FFB_DSP_TESTS_EXCLUDE` / `_SANITIZE` lists like the others.
- Target-vs-host flags for the report (CLAUDE.md "flag whatever could differ on target"): (a) ARM_MATH_DSP intrinsic path absent on host (4 above); (b) VFMA contraction; (c) float->int conversion of out-of-range values (x86 0x80000000 vs ARM saturation): affects arm_sin/cos int32 cast and `arm_float_to_q*` (not in scope); (d) signed shifts of negative numbers in the q15/q31 kernels (UB per C, defined by GCC); (e) host long = 64 bit; (f) `arm_sqrt_f32` returns `ARM_MATH_ARGUMENT_ERROR` for negative input, sqrtf on host vs VSQRT on target (both correctly rounded). Sanitizers: UBSan will likely flag left shift of negative values and overflow in q-type kernels: plan `KNOWN_ISSUE_UB` handling per CLAUDE.md, those are vendor bugs to report not fix.
- Compile warnings: DSP sources compile clean at -Wall with both compilers in my experiments (no `-Wextra` check done).
- The `arm_math.h` / `arm_math_f16.h` include in C++ (the tests are C++): arm_math.h is already included from `test_cmsis_dsp.cpp`; arm_math_f16.h in C++ with `__fp16=_Float16` was not tried (C only).
- Open questions for the lead:
  1. Is Statistics (arm_std/var) the intended in-scope module list? (yes by the calls; the earlier "expected" list omitted it.)
  2. Filtering is 116 units / 450 ARM ids; confirm that the whole module (FIR, conv, LMS, ...) is wanted and not only the biquad family.
  3. f16: OK to add a CMake feature check and the `-D__ARM_FP16_FORMAT_IEEE=1 -D__fp16=_Float16` hack for f16 sources only (no firmware edit, tests only)?
  4. Should Controller sin_cos / clarke / park (not called by the firmware, no ARM reference) count as "functions of the module"? Per the decision "all functions" yes; the oracle then is math definition.
  5. Should the Patterns be read straight from the submodule at configure time (no copy), as assumed here? Yes recommended (no files under Firmware/Tests, nothing duplicated).
