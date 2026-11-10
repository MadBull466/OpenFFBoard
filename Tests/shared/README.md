# Shared documents

`specs/` holds the specification files supplied by the owner. They are git-ignored (`Tests/.gitignore`), never committed and never downloaded by an agent. Both agents read them from here. Without them the tests still build and run (they only cite them).

| File | Used for |
|---|---|
| `TMC4671-LA_datasheet_Rev2.07.pdf` | TMC4671 driver (P10 D1) |
| `TMCL_reference.pdf`, `TMC4671-TOSV-REF_HW_FW_manual_Rev1.1.pdf`, `TMC4671-LEV-REF_datasheet_hw1.00_fw1.00_rev1.00.pdf` | TMCDebugBridge, TMCL frames (P13) |
| `CAN Protocol — ODrive Documentation 0.5.6 documentation.pdf`, `can-protocol.csv` | ODrive driver (P10 D2) |
| `VESC6_CAN_CommandsTelemetry.pdf`, `bldc-master/` (VESC firmware source 7.01) | VESC driver (P10 D3) |
| `SimpleMotionV2-master/` (documentation and reference library; an oracle only, never copied into tests) | SimpleMotion driver (P15) |
| `CAN_BUS_Motor_Motion_Protocol_V4.3.pdf` | MyActuator RMD driver (P16) |
| `midi10.pdf` (USB Device Class Definition for MIDI Devices 1.0) and `MIDI_1.0_Detailed_Specification.pdf` | MIDI (P17, not started) |
| `USB MIDI v2_0.pdf` | no longer needed: the firmware class is USB-MIDI 1.0 style (kept as supplied) |
| `dm00031020-...stm32f405-...pdf` (RM0090), `DS_dm00037051.pdf` (datasheet), `es0182-...errata...pdf` | STM32F407 fakes (HAL, timers, flash, CAN, USB) |

Not supplied by the owner and therefore not in this folder: the USB HID PID 1.01 specification (open question 3), the PWM driver has no protocol specification.
