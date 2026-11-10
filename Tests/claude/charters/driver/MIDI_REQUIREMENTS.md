# MIDI (`MidiMain`, `MidiHandler`, TinyUSB `class/midi`): requirements charter (phase P17, T0)

Status: **T0 approved by the owner 2026-11-10** ("Approve, start T1"; the lead's proposal L1 / question 53 option A is therefore the plan); drafted by the lead (Sonnet 5.5), committed as `f21e631a`. **T1 approved and built 2026-11-10** (see "T1 amendments" at the end): the macro `MIDI` is on in `ffb_usb_tests` only. **T2 done 2026-11-10** (see "T2 amendments": 319 cases, four reviewers, gate), approved by the owner and committed as `fe232b2d` (pushed). Open owner questions start at number 53 (see the end; 53 is settled, 54-61 are open).

Units under test (sources read by the lead on 2026-11-10):
* `Firmware/FFBoard/UserExtensions/Src/MidiMain.cpp` (180 lines, `MM`) and `Inc/MidiMain.h` (78 lines), class `MidiMain : FFBoardMain, MidiHandler, TimerHandler`, class id `CLSID_MAIN_MIDI` (0xD), name "MIDI (TMC)", visibility debug, behind the macro `MIDI`; it plays notes by moving the phase of a TMC4671 (`TMC_1`) on the user timer `TIM_USER` (`htim9`).
* `Firmware/FFBoard/Src/MidiHandler.cpp` (138 lines, `MH`) and `Inc/MidiHandler.h` (33 lines): decodes one 4 byte USB-MIDI event packet into virtual callbacks; the global pointer `midihandler` and the callback `tud_midi_rx_cb` live in `Src/global_callbacks.cpp:349-358` (`GC`).
* TinyUSB MIDI class driver `Firmware/FFBoard/USB/class/midi/midi_device.c` (570 lines, `MD`), `midi_device.h`, `midi.h` (vendor code, tier B: bugs are reported, never fixed), the descriptor macros `TUD_MIDI_DESCRIPTOR` in `USB/device/usbd.h:309-364` and the firmware's `usb_cdc_midi_conf` (`UserExtensions/Src/usb_descriptors.cpp:83-92`: CDC interfaces 0 and 1, MIDI AudioControl interface 2, MIDIStreaming interface 3, OUT endpoint 0x02, IN endpoint 0x83, 64 bytes).

Executable: `ffb_usb_tests` (owner's decision, question 28) with `MIDI` turned on there only; `MidiMain` needs the double of the TMC4671 interface described under "Fake plan" (owner question 53). Plan: `Tests/claude/plans/P14_P17_REMAINING_DRIVERS_PLAN.md`. Tests: `Firmware/Tests/claude/cases_usb/test_midi_*.cpp`, fixture `support/MidiFixture.h` (T1).

## Decision: USB-MIDI version (settles the caveat of decision 27)

The firmware class (`midi.h`, `TUD_MIDI_DESCRIPTOR`) is USB-MIDI **1.0** style: AudioControl interface plus MIDIStreaming interface with `bcdADC` 0x0100 / `bcdMSC` 0x0100, 32 bit event packets with a cable number and a code index number, embedded and external jacks, 9 byte audio endpoints. The oracle is therefore the USB Device Class Definition for MIDI Devices **1.0**. `USB MIDI v2_0.pdf` is not used (its Universal MIDI Packets and group descriptors do not describe these bytes).

## Sources of expected values (oracles)

| Code | Source |
|---|---|
| UM | "Universal Serial Bus Device Class Definition for MIDI Devices, Release 1.0, Nov 1 1999" (`Tests/shared/specs/midi10.pdf`, 43 pages, supplied by the owner, git-ignored, never committed; cited by table number, never copied beyond a short citation). Section 4 event packets: byte 0 = cable number (high nibble) and code index number CIN (low nibble), bytes 1-3 = MIDI_0..MIDI_2, unused bytes are zero; Table 4-1 (CIN 0x2 two byte system common, 0x3 three byte system common, 0x4 SysEx starts or continues, 0x5 single byte system common or SysEx ends with one byte, 0x6 SysEx ends with two bytes, 0x7 with three bytes, 0x8 note off, 0x9 note on, 0xA poly key pressure, 0xB control change, 0xC program change (2 bytes), 0xD channel pressure (2 bytes), 0xE pitch bend, 0xF single byte; 0x0 and 0x1 reserved), Table 4-2 examples (`19 9n kk vv`, `AB Bn pp vv`, real-time `3F F8 xx xx`, SysEx `p4 F0 00 01 / p5 F7 00 00`, `p4 .. / p6 02 F7 00`, `p4 .. / p7 02 03 F7`, `p6 F0 F7 00`, `p7 F0 mm F7`). Section 6 descriptors: Tables 6-1 (MS interface), 6-2 (class-specific header: length 7, subtype MS_HEADER, bcdMSC 0x0100, wTotalLength), 6-3 (IN jack, 6 bytes), 6-4 (OUT jack, 6 + 2 p bytes), 6-6 (standard bulk endpoint, 9 bytes: attributes bulk, bInterval 0, bRefresh 0, bSynchAddress 0), 6-7 (class-specific endpoint, 4 + n bytes: bNumEmbMIDIJack, baAssocJackID), Appendix B (the worked MIDI adapter example: AC interface, AC header wTotalLength 0x0009, MS header wTotalLength 0x0041). Section 7: the class requests are optional, an unsupported request must stall the control pipe (7.2); the MIDIStreaming subclass places no specific requirements on the standard requests (7.1). |
| MS | "MIDI 1.0 Detailed Specification, document version 4.2.1" (`Tests/shared/specs/MIDI_1.0_Detailed_Specification.pdf`, 86 pages, same rules): status byte table (8n note off, 9n note on, An poly pressure, Bn control change, Cn program change, Dn channel pressure, En pitch bend, F8 timing clock sent 24 times per quarter note), "Note On with velocity 0 is used for Note Off", pitch bend 14 bit with the least significant byte first, centre 00 40 (0x2000), channel mode messages 120 all sound off, 121 reset all controllers, 122 local control, 123 all notes off, 124-127 omni / mono / poly (123-127 also act as all notes off, "may be ignored by a receiver"), real-time messages may interrupt other messages, running status for voice messages. The pitch bend sensitivity "is selected in the receiver". The note-number to frequency mapping (A440) is not in this document. |
| HD | Hand derivation in a comment next to the assertion: packet bytes, equal temperament `440 * 2^((n-69)/12)` Hz, the phase value `sin(pi * f * t) * volume / 127 * range` truncated to int16, the timer constants. |
| FW | The firmware's own behaviour where nothing above decides it (the mapping of messages to actions, `MidiMain`'s synthesis, defaults, command ids). Test name starts with `characterization:` and the case gets a NEEDS REVIEW entry. |
| IF | The `FFBoardMain`, `TimerHandler` and `CommandHandler` interfaces and the TinyUSB application API (`tud_midi_*` in `midi_device.h`). |

Rules inherited from `CLAUDE.md` and the P13-P16 charters: fixture per test, simulated time only, never copy a library value into an assertion without `characterization:`, a suspected defect is a `KNOWN_ISSUE("...")` test of the INTENDED behaviour whose first failing assertion is the defect, `KNOWN_ISSUE_UB` for undefined behaviour (skipped in sanitizer builds), no test whose outcome depends on stale memory, tests go through public interfaces (the packets the host sends on endpoint 0x02, the packets read from endpoint 0x83, the virtual callbacks of a probe handler, the commands, the calls recorded by the driver double, the descriptors).

## Facts every test author needs (read from the source by the lead)

* **`MidiHandler::midiRx(itf, packet[4])`** (`MH:24-117`): pulses the system LED, `chan = packet[1] & 0xF`, `packettype = packet[1] & 0xF0`, `b1 = packet[2]`, `b2 = packet[3]`. The cable number and the CIN in `packet[0]` are never looked at. `switch(packet[1])`: 0xF0 sets `sysexState` and calls `otherPacket`; 0xF7 clears `sysexState` and calls `otherPacket`; 0xF8 calls `midiTick()`; any other value goes to `switch(packettype)`: 0x80 `noteOff(chan, b1, b2)`; 0x90 `noteOff` when `b2 == 0` else `noteOn`; 0xB0 `controlChange`; 0xC0 `programChange(chan, b1)`; 0xD0 nothing at all; 0xE0 `pitchBend(chan, ((b1 & 0x7f) | ((b2 & 0x7f) << 7)) - 8192)`; every other value (0xA0 poly pressure, 0xF1-0xF6, 0xF9-0xFF, SysEx data bytes below 0x80) `otherPacket(packet)`. `noteOn` and `noteOff` are pure virtual in the header and also defined empty in the source; the other hooks are empty by default.
* **`GC:349-358`:** `MidiHandler* midihandler = nullptr`; `tud_midi_rx_cb(itf)` returns at once without reading when `midihandler` is null, else `while (tud_midi_n_packet_read(itf, MidiHandler::buf)) midihandler->midiRx(itf, MidiHandler::buf)`. The constructor of `MidiHandler` stores `this` in the global, the destructor sets it to `nullptr` (also when another handler was registered later). `MidiHandler::buf` is one static 4 byte buffer.
* **`MidiMain`** (`MM:24-180`): construction fills `noteToFreq[i] = pow(2, (i-69)/12.0) * 440.0` for 0..127, sets `TIM_USER` (`htim9`) `ARR = 100`, `PSC = SystemCoreClock / 1000000 - 1`, `CR1 = 1`, creates `TMC_1` (`drv`), `setLimits(TMC4671Limits())`, `setEncoderType(NONE)`, pulses the error LED when the motor type is NONE, `setPhiEtype(ext)`, `setUdUq(0, 0)`, `allowSlowSPI = false`, `initialize()` (error LED pulse on failure), `setPhiEtype(ext)`, `setMotionMode(uqudext, true)`, registers the commands `power` (default 2500) and `range` (default 0x3fff) as GET|SET (`handleGetSet` on a `uint32_t`), then `HAL_TIM_Base_Start_IT(htim9)`. The destructor is empty. `update()`: `osDelay(500)`; when `drv->hasPower() && !drv->isSetUp()` it calls `initializeWithPower()`. `timerElapsed(htim)` calls `play()` only for `htim == timer_update`. `usbInit()` creates the `USBdevice` with `usb_devdesc_ffboard_composite`, `usb_cdc_midi_conf` and the default strings and registers it.
* **Notes** (`MM:122-155`): `notes[16]` vectors of `MidiNote {note, volume, counter, pitchbend = 1}`. `noteOn` first calls `noteOff(chan, note, vel)` (a repeated note restarts at counter 0), then appends. `noteOff` erases the first entry with the same note. `controlChange(chan, c, val)` clears `notes[chan]` for c = 120, 121 and 123 only. `pitchBend(chan, val)` sets `pitchbend = 2^(val / 8192)` on the notes that exist at that moment (octave range, a note started later keeps 1).
* **Synthesis** (`MM:91-119`, called every 100 microseconds by the timer): only channel 0 is played. No note: when `active[0]` it clears the flag and calls `setUdUq(0, 0)`. Notes present: when not `active[0]` it calls `setUdUq(power, 0)` and sets the flag; for the last note of the channel: `counter += 100e-6 * pitchbend`, `p = counter * noteToFreq[note]`, `sine = sinf(M_PI * p) * (volume / 127.0f)`, `val = (int16_t)(sine * movementrange)`, `setPhiE_ext(val)`. The argument of `sinf` is `pi * f * t`, i.e. the oscillation frequency is half the note frequency (owner question 54).
* **Class driver** (`MD`): FIFOs of `CFG_TUD_MIDI_RX_BUFSIZE` / `TX` bytes (64 on full speed), endpoint buffers 64 bytes (`CFG_TUD_MIDI_EP_BUFSIZE`), one interface (`CFG_TUD_MIDI` = 1). `midid_open` accepts an AudioControl interface (class 1, subclass 1, protocol 0), skips its class specific descriptors, requires a MIDIStreaming interface (subclass 3, protocol 0) directly after, opens its bulk endpoints (`ep_in`, `ep_out`), skips the class specific endpoint descriptors and arms the OUT endpoint (`_prep_out_transaction` arms it only when the RX FIFO has at least 64 free bytes). `tud_midi_n_mounted` = both endpoints open. `midid_reset` clears the endpoint numbers and the stream state and both FIFOs. `midid_control_xfer_cb` returns false (stall) for every request. `midid_xfer_cb`: OUT completion writes the received bytes to the RX FIFO, calls `tud_midi_rx_cb(itf)` and re-arms; IN completion flushes the TX FIFO or, when nothing is left and the last transfer was a non-zero multiple of 64 bytes, sends a zero length packet.
* **Byte stream to packets** (`tud_midi_n_stream_write`, `MD:241-358`): first byte of an event decides the packet: 0x8-0xB and 0xE: CIN = high nibble, 3 data bytes; 0xC / 0xD: CIN = high nibble, 2 bytes; 0xF0: CIN 4 (SysEx start); 0xF1 / 0xF3: CIN 2; 0xF2: CIN 3; every other 0xF_ value (0xF6, 0xF7, 0xF8-0xFF and the undefined 0xF4 / 0xF5): CIN 5, one byte; a first byte below 0x80: CIN 0xF, one byte. Inside a SysEx (the previous packet had CIN 4) each byte is sysex payload; the byte 0xF7 closes it with CIN 4 + (bytes in the packet) - 1, i.e. 5, 6 or 7. The cable number goes to the high nibble of byte 0. Unused bytes are zero. A full TX FIFO stops the loop (the return value is the number of bytes consumed).
* **Packets to byte stream** (`tud_midi_n_stream_read`, `MD:134-190`): CIN 5 / 0xF give 1 byte, CIN 2 / 6 / 0xC / 0xD give 2, every other CIN except the reserved 0 and 1 gives 3; CIN 0 and 1 make the function `return 0`.
* **Host versus target.** The class driver, the handler and `MidiMain` use no type whose size differs (`uint8_t`, `int16_t`, `uint32_t`, `float`); `sinf(M_PI * p)` multiplies a float by a double (software double on the Cortex-M4F, same IEEE result as the host for this operation); the float to `int16_t` conversion of a value beyond 16 bits is undefined (B-MIDI-5); `noteToFreq` is built with `std::pow` (double) and stored as float. The test hand-derives expected integers and allows one count for the conversion. The TIM_USER interrupt is delivered by the fake timer hook, not preemptively: the `notes` vectors are changed by the USB task and read by the timer interrupt on the target (no host test can show that race, NEEDS REVIEW only).

## Requirements

### R1: identity, chooser, build
| ID | Requirement | Oracle |
|---|---|---|
| R1.1 | `MidiMain::info` name "MIDI (TMC)", id `CLSID_MAIN_MIDI` (0xD), visibility debug; `getInfo()` returns it; `isCreatable()` true; the help string "Plays MIDI via TMC4671" | FW, plain |
| R1.2 | The main class chooser lists `MidiMain` once (id 0xD) when `MIDI` is on, and creating it through the chooser gives a working object | FW, plain |
| R1.3 | The macro `MIDI` is on in `ffb_usb_tests` only: `ffb_driver_tests` (`test_driver_setup.cpp:50` fails on it) and the other executables keep it off, and their compile flags are unchanged | build check |
| R1.4 | `usb_cdc_midi_conf` is selected by `MidiMain::usbInit` (the USBdevice has the composite device descriptor, this configuration and the default strings) | FW, plain |

### R2: `MidiHandler::midiRx`, decoding of the packet
| ID | Requirement | Oracle |
|---|---|---|
| R2.1 | `[09 9n kk vv]` with vv > 0 calls `noteOn(n, kk, vv)` for every channel n = 0..15; vv == 0 calls `noteOff(n, kk, 0)` (Note On with velocity 0 is a Note Off) | MS + UM Table 4-2 |
| R2.2 | `[08 8n kk vv]` calls `noteOff(n, kk, vv)` with the release velocity | MS |
| R2.3 | `[0B Bn cc vv]` calls `controlChange(n, cc, vv)`; `[0C Cn pp 00]` calls `programChange(n, pp)` | MS |
| R2.4 | `[0E En lsb msb]` calls `pitchBend(n, ((lsb & 0x7f) \| ((msb & 0x7f) << 7)) - 8192)`: centre `00 40` gives 0, `00 00` gives -8192, `7F 7F` gives +8191, `01 00` gives -8191 (least significant byte first) | MS (centre and extremes) |
| R2.5 | `[0F F8 00 00]` calls `midiTick()` once and no other hook; `F8` on any cable number | MS (F8) + UM Table 4-2 |
| R2.6 | `[04 F0 d1 d2]` calls `otherPacket` and sets `sysexState`; `[05 F7 00 00]` calls `otherPacket` and clears it; SysEx data packets (CIN 4 with data bytes below 0x80, CIN 6, CIN 7) call `otherPacket` and never a note hook | UM Table 4-2 |
| R2.7 | After a SysEx that ends with CIN 6 or 7 (`p6 02 F7 00`, `p7 02 03 F7`, `p6 F0 F7 00`) `sysexState` is cleared | UM Table 4-2, **`KNOWN_ISSUE` B-MIDI-3** (only a packet whose byte 1 is 0xF7 clears it: the state stays set) |
| R2.8 | Poly key pressure `[0A An kk vv]`, system common (F1, F2, F3, F6) and the real-time messages F9-FF go to `otherPacket` and never to a note hook | FW, plain for the hooks, `characterization:` for the choice |
| R2.9 | Channel pressure `[0D Dn vv 00]` calls no hook (neither a note hook nor `otherPacket`) | FW, `characterization:` (question 56) |
| R2.10 | The cable number is ignored: the same event on cable 0 and cable 5 gives the same call | FW, `characterization:` (question 57); UM 5.1 routes by cable number |
| R2.11 | A data byte that is not a status (CIN 0xF single byte packets of an unparsed stream, Table 4-1 Note 2, e.g. `[0F 90 00 00] [0F 3C 00 00] [0F 64 00 00]`) does not call `noteOff` / `noteOn` for note 0 | UM Table 4-1 Note 2, **`KNOWN_ISSUE` B-MIDI-4** (the CIN is ignored: the first packet is decoded as a note on with note 0, velocity 0 and calls `noteOff(0, 0, 0)`) |
| R2.12 | A note number or velocity with bit 7 set (a malformed packet) is not passed on unchecked: `noteOn(0, 0xFF, 0x7F)` must not reach an out-of-range table index in `MidiMain` | MS (data bytes are 7 bit), **`KNOWN_ISSUE_UB` B-MIDI-1** at the `MidiMain` level (R5.8) |
| R2.13 | Each received packet pulses the system LED (`pulseSysLed`) | FW, plain (observable on the fake LED GPIO) |

### R3: `tud_midi_rx_cb` and the global handler pointer
| ID | Requirement | Oracle |
|---|---|---|
| R3.1 | A host transfer of several packets on endpoint 0x02 is delivered in order: one `midiRx` call per packet, the interface number 0, the payload exactly the 4 bytes | UM section 4 |
| R3.2 | A `MidiHandler` object registers itself in `midihandler`; destroying it clears the pointer; a packet received after the destruction is not delivered and does not crash | FW, plain |
| R3.3 | The constructor of a second handler replaces the first; destroying the first afterwards clears the pointer of the second | FW, `characterization:` (NEEDS REVIEW) |
| R3.4 | Packets that arrive while no handler exists must not stop the endpoint: after a handler is created, the next packets reach it | FW intent, **`KNOWN_ISSUE` B-MIDI-6** if the T2 check confirms it (without a handler nothing reads the RX FIFO, `_prep_out_transaction` never arms the OUT endpoint again) |
| R3.5 | More packets than the 64 byte RX FIFO holds (17 packets in one burst) are delivered completely and in order through repeated transfers | UM + FW |

### R4: `MidiMain` note handling (through the virtual hooks)
| ID | Requirement | Oracle |
|---|---|---|
| R4.1 | `noteOn` adds the note with its velocity to the channel; a second `noteOn` for the same note restarts it (one entry, counter 0); `noteOff` removes it, another note stays; `noteOff` of an unknown note does nothing | FW, plain |
| R4.2 | The notes of the 16 channels are independent | FW, plain |
| R4.3 | Control change 120, 121, 123 clear all notes of that channel and no other channel; other controller numbers change nothing | MS (120 all sound off, 121 reset all controllers, 123 all notes off) |
| R4.4 | Control change 124-127 (omni off / on, mono, poly) do not clear the notes | MS ("123-127 also function as All Notes Off", "may be ignored"), `characterization:` (question 56) |
| R4.5 | `pitchBend` scales the notes that exist at that moment by `2^(val / 8192)` (val -8192: 0.5, 0: 1, +8191: 1.9999); a later `noteOn` starts at 1 | FW, `characterization:` for the octave range (question 55); the second half is **`KNOWN_ISSUE` B-MIDI-2** (a pitch bend is channel state: a note started after the bend keeps the pitch bend of its channel, MS) |
| R4.6 | `programChange`, `midiTick` and `otherPacket` of `MidiMain` do nothing observable | FW, plain |

### R5: `MidiMain::play` (the synthesis, driven by `timerElapsed(htim9)`)
| ID | Requirement | Oracle |
|---|---|---|
| R5.1 | `timerElapsed` with another timer handle does nothing | FW, plain |
| R5.2 | No note: no driver call; after the last `noteOff` / all notes off: one `setUdUq(0, 0)` and no more calls on the following ticks | FW, plain |
| R5.3 | First note: one `setUdUq(power, 0)` (default 2500, the `power` command value), then one `setPhiE_ext(val)` per tick; the call is not repeated while the note continues | FW, plain |
| R5.4 | The value for the note 69 (A4, 440 Hz table value), velocity 127, range 0x3fff, no bend, tick k = 1 and 2: `int16_t(sinf(pi * 440 * k * 100e-6) * 16383)` (HD, within one count) | HD + FW (the formula), `characterization:` for the choice of `pi * f * t` |
| R5.5 | Velocity scales the value linearly (volume = velocity / 127); the `range` command scales it (`range` 0x7fff and 0x1000 give the HD values) | HD + FW |
| R5.6 | Only the last note of channel 0 is played; the notes of channels 1-15 never cause a driver call | FW, `characterization:` (code comment "Take only first channel for now and last note") |
| R5.7 | The counter advances by `100e-6 * pitchbend` per tick: a bend of +1 octave doubles the phase step (HD, after 10 ticks) | HD + FW |
| R5.8 | A note number above 127 (`noteOn(0, 0xFF, 0x7F)` reached through R2.12) must not read outside `noteToFreq` | MS (7 bit data), **`KNOWN_ISSUE_UB` B-MIDI-1** (`noteToFreq[note->note]` with a 128 entry table: out of bounds read for 128..255) |
| R5.9 | The oscillation frequency of the output equals the note frequency: the phase value of A4 completes one period in 1 / 440 s, i.e. its zero crossings are 1 / 880 s apart | FW intent (the note is meant to sound at its pitch), `characterization:` today: question 54 decides whether the half frequency is a defect (B-MIDI-7 candidate); the test asserts the HD value of R5.4, not the intent, until the owner answers |

### R6: `MidiMain` construction, commands, life cycle
| ID | Requirement | Oracle |
|---|---|---|
| R6.1 | Construction sets the user timer: `ARR` 100, `PSC = SystemCoreClock / 1000000 - 1`, `CR1` 1, then starts it with `HAL_TIM_Base_Start_IT`; the update interrupt reaches `timerElapsed` through `TimerHandler` | FW + RM0090 (TIM registers, via the fake) |
| R6.2 | Construction calls the driver in this order: `setLimits`, `setEncoderType(NONE)`, `setPhiEtype(ext)`, `setUdUq(0, 0)`, `initialize()`, `setPhiEtype(ext)`, `setMotionMode(uqudext, true)`; `allowSlowSPI` is false afterwards (a field write, checked as a final value, not as a position in the call order) | FW, `characterization:` (order) |
| R6.3 | `initialize()` failing and a motor type NONE pulse the error LED and construction continues | FW, plain |
| R6.4 | `update()`: waits 500 ms; with power present and not set up it calls `initializeWithPower()` exactly once per call, otherwise nothing | FW, plain |
| R6.5 | Commands `power` and `range`: get returns the value, set stores it (`power` 1, 2500, 0xFFFF; `range` 1, 0x3fff, 0x7fff) and the next tick uses it; an unknown command id is `NOT_FOUND` | FW, plain |
| R6.6 | `range` above 0x7fff is not accepted (the product `sine * range` is converted to `int16_t`) and the value 0x100000 does not cause an undefined conversion in `play` | FW intent, **`KNOWN_ISSUE_UB` B-MIDI-5** (`movementrange` is a `uint32_t` set unchecked; the float to `int16_t` conversion is undefined above 32767; `power` is set unchecked too, T2 checks its use in `setUdUq`) |
| R6.7 | Destroying `MidiMain` clears `midihandler` and the timer handler registration; the timer keeps running (`TIM_USER` is never stopped) | FW, `characterization:` (NEEDS REVIEW) |
| R6.8 | Two `MidiMain` objects at once are not created by the chooser (single main class); `MidiMain` created and destroyed twice leaves no leaked kernel object | FW + leak check, plain |

### R7: descriptors of the class (UM section 6)
| ID | Requirement | Oracle |
|---|---|---|
| R7.1 | `usb_cdc_midi_conf` contains, after the CDC descriptors: the AC interface (length 9, interface 2, 0 endpoints, class 1, subclass 1, protocol 0), the AC header (length 9, CS_INTERFACE, subtype HEADER, bcdADC 0x0100, wTotalLength 0x0009, bInCollection 1, baInterfaceNr = 3), the MS interface (length 9, interface 3, 2 endpoints, class 1, subclass 3, protocol 0), the MS header (length 7, subtype MS_HEADER, bcdMSC 0x0100) | UM Tables 6-1, 6-2, Appendix B (B-3 to B-6) |
| R7.2 | The MS header wTotalLength is 0x0041 (65 = header 7 + one cable of jacks 30 + two class specific endpoint blocks 28), as the spec's worked example Table B-6 | UM Appendix B; the wording of Table 6-2 ("header and all Jack and Element descriptors") would give 37, **NEEDS REVIEW** (question 61) |
| R7.3 | Jacks: embedded IN (id 1), external IN (id 2), embedded OUT (id 3, one input pin: source id 2, pin 1) and external OUT (id 4, source id 1, pin 1): lengths 6, 6, 9, 9; jack types EMBEDDED (1) / EXTERNAL (2); all ids unique and non zero | UM Tables 6-3, 6-4 |
| R7.4 | Endpoints: OUT 0x02 then IN 0x83, length 9, bulk (attributes 0x02, synchronisation none), wMaxPacketSize 64, bInterval 0, bRefresh 0, bSynchAddress 0; each followed by its class specific endpoint descriptor (length 5, CS_ENDPOINT, subtype MS_GENERAL, bNumEmbMIDIJack 1, baAssocJackID = 1 for the OUT endpoint (embedded IN jack) and 3 for the IN endpoint (embedded OUT jack)) | UM Tables 6-6, 6-7 |
| R7.5 | The configuration descriptor is consistent: wTotalLength equals the sum of all descriptors, bNumInterfaces 4, the interface numbers 0-3 are consecutive, the string index 6 of the AC interface exists ("FFBoard MIDI") | UM + USB 2.0 (the sum is a plain count) |
| R7.6 | The descriptors can be walked by `midid_open` (see R8): the returned length covers both interfaces and all endpoint descriptors | MD + UM |

### R8: class driver, open, mount, reset, requests
| ID | Requirement | Oracle |
|---|---|---|
| R8.1 | Enumeration with `usb_cdc_midi_conf` configures the device; `tud_midi_mounted()` is true afterwards and false before the configuration | MD + UM |
| R8.2 | After the configuration the OUT endpoint 0x02 is armed (a host OUT transfer is acknowledged) and IN 0x83 NAKs when there is nothing to send | UM + USB |
| R8.3 | A bus reset or a deconfiguration (`midid_reset`) unmounts the interface and clears both FIFOs and the stream state: queued packets and a half written event are gone after the next enumeration | MD |
| R8.4 | A class specific request to the MIDIStreaming interface (bmRequestType 0x21 / 0xA1, SET_CUR / GET_CUR / GET_STAT) and to a bulk endpoint (0x22 / 0xA2) stalls the control pipe; the device stays configured | UM 7.2 (unsupported requests stall) |
| R8.5 | Standard requests on the MIDI interfaces (GET_INTERFACE, SET_INTERFACE 0) behave as for the other classes (answered, no stall for alternate setting 0) | UM 7.1 + USB 2.0 |
| R8.6 | `midid_open` refuses a configuration whose first MIDI interface is not an AudioControl interface or whose second is not MIDIStreaming (the wrong class, subclass or protocol): the device is not configured, no endpoint is opened | MD (code), `characterization:` (needs a modified descriptor in the test, which is test data) |
| R8.7 | The RX FIFO is full (a handler that does not read): the OUT endpoint is not armed again (NAK) until packets are read; reading re-arms it | MD (documented by the code comment), `characterization:` |

### R9: byte stream to packets (`tud_midi_n_stream_write`)
| ID | Requirement | Oracle |
|---|---|---|
| R9.1 | `90 3C 7F` on cable 0 gives `09 90 3C 7F`; on cable 1 `19 90 3C 7F`; `B1 07 64` on cable 10 gives `AB B1 07 64`; every channel voice status 8n-En (the table of UM 4-1) gives its CIN, the data bytes and padding | UM Tables 4-1, 4-2 |
| R9.2 | `C2 05` gives `0C C2 05 00` and `D3 40` gives `0D D3 40 00` (two byte events padded with zero) | UM Table 4-1 |
| R9.3 | `F1 xx` gives CIN 2, `F3 xx` CIN 2, `F2 ll hh` CIN 3, `F6` CIN 5 with two zero bytes | UM Table 4-1 |
| R9.4 | Real-time `F8` on cable 3 gives `3F F8 00 00` (CIN 0xF); the same for FA, FB, FC, FE, FF | UM Table 4-2, **`KNOWN_ISSUE` B-MIDI-V1** (vendor: CIN 5 with the cable number in the high nibble: `35 F8 00 00`) |
| R9.5 | SysEx: `F0 00 01 F7` gives `p4 F0 00 01` then `p5 F7 00 00`; `F0 00 01 02 F7` gives `p4 F0 00 01`, `p6 02 F7 00`; `F0 00 01 02 03 F7` gives `p4 F0 00 01`, `p7 02 03 F7`; `F0 F7` gives `p6 F0 F7 00`; `F0 mm F7` gives `p7 F0 mm F7`; a long SysEx (7 payload bytes) gives continued CIN 4 packets and the right end CIN | UM Table 4-2 |
| R9.6 | A real-time byte inside a SysEx is sent as its own packet (CIN 0xF) and does not become SysEx payload | MS (real-time messages may interrupt other messages) + UM Table 4-2, **`KNOWN_ISSUE` B-MIDI-V3** (vendor: it is appended to the SysEx as payload) |
| R9.7 | A data byte below 0x80 as the first byte of an event (running status data) gives a single byte packet CIN 0xF | UM Table 4-1 Note 2, `characterization:` (the choice to pack it) |
| R9.8 | The byte stream is cut across calls: `90 3C` then `7F` in two calls gives one packet after the third byte; an event split over three calls works; the stream state is per interface | FW + MD, plain |
| R9.9 | A full TX FIFO: the return value is the number of bytes consumed, the rest is not lost silently (the caller can resend), and nothing is written beyond the FIFO | MD, plain |

### R10: packets to byte stream (`tud_midi_n_stream_read`, `tud_midi_n_available`)
| ID | Requirement | Oracle |
|---|---|---|
| R10.1 | The packets of UM Table 4-2 give the original MIDI bytes (`19 9n kk vv` gives `9n kk vv`, `3F F8 xx xx` gives `F8`, the SysEx pairs give the original SysEx byte sequence) and the right length per CIN (2 for CIN 0xC / 0xD / 0x2 / 0x6, 1 for CIN 0x5 / 0xF, 3 otherwise) | UM Tables 4-1, 4-2 |
| R10.2 | A packet read through a buffer smaller than the event keeps the rest for the next call; `tud_midi_n_available` counts the unread bytes of the current event plus the FIFO content | MD, plain |
| R10.3 | Reserved CIN 0 and 1 are skipped without changing what was read before: a read of `[09 90 3C 7F] [00 ..] [09 90 3D 7F]` returns 6 bytes (the reserved packet is ignored) | UM Table 4-1 (reserved, to be ignored), **`KNOWN_ISSUE` B-MIDI-V2** (vendor: `return 0` at the reserved packet, the bytes already copied are reported as 0) |
| R10.4 | `tud_midi_n_packet_read` returns false and leaves the FIFO empty when fewer than 4 bytes are queued | MD, plain |

### R11: packet API and bulk transfers
| ID | Requirement | Oracle |
|---|---|---|
| R11.1 | `tud_midi_n_packet_write(packet)` queues the 4 bytes; the host reads them on 0x83 as one transfer of 4 bytes; several packets written before the host reads arrive in order (not necessarily as one transfer: the first write is flushed at once as its own 4 byte transfer, the rest follow, `characterization:`) | UM section 4 |
| R11.2 | A full TX FIFO (17 packets unread: one already in the endpoint buffer plus the 16 of the 64 byte FIFO) makes `packet_write` return false and queues nothing; after the host reads, writing works again | MD, plain |
| R11.3 | A transfer of exactly 64 bytes (16 packets) is followed by a zero length packet, a shorter transfer is not; with nothing more to send the endpoint NAKs | USB 2.0 bulk rule (not in the supplied documents), `characterization:` |
| R11.4 | `packet_write` and `stream_write` before the interface is mounted return false / 0 and queue nothing | MD, plain |
| R11.5 | The packets the device sends are well formed for UM section 4: byte 0 high nibble = cable number given, CIN valid, unused bytes zero | UM |

### R12: end to end (host to handler, device to host)
| ID | Requirement | Oracle |
|---|---|---|
| R12.1 | A note on / note off / pitch bend / clock sequence sent by the fake host on 0x02 reaches a probe handler as the matching hooks (chain: dwc2 fake, `midid_xfer_cb`, `tud_midi_rx_cb`, `midiRx`) | UM + MS |
| R12.2 | The same sequence reaches `MidiMain` and produces the driver calls of R5.3 | FW |
| R12.3 | A burst of 16 packets (64 bytes, one full transfer) is delivered completely and the OUT endpoint accepts the next transfer afterwards | UM + MD |
| R12.4 | The main class selection through the USB CDC command interface is not part of this group (the chooser is R1.2) | n/a |

## Defect candidates and decisions the lead made (to be confirmed or struck in T2)

| Id | Where | Candidate defect | Marker | Number (assigned at T2) |
|---|---|---|---|---|
| B-MIDI-1 | `MM:107` | `noteToFreq[note->note]`: a note number 128..255 (malformed packet, `MH` does not mask) reads outside the 128 entry table | `KNOWN_ISSUE_UB` | firmware bug 219 (reserved) |
| B-MIDI-2 | `MM:148-155` | a pitch bend does not apply to notes started afterwards | `KNOWN_ISSUE` | 220 |
| B-MIDI-3 | `MH:33-45` | `sysexState` stays true after a SysEx that ends in byte 2 or 3 of the packet (CIN 6 / 7) | `KNOWN_ISSUE` | 221 |
| B-MIDI-4 | `MH:24-31` | the CIN is ignored: single byte packets (CIN 0xF) with data bytes are decoded as events | `KNOWN_ISSUE` | 222 |
| B-MIDI-5 | `MM:102`, `MM:116`, `MM:160-165` | `range` and `power` unchecked: the float to `int16_t` conversion of `range` is undefined above 32767 (`KNOWN_ISSUE_UB`); `power` above 32767 reaches `setUdUq(int16_t)` as a negative voltage by wrap-around (`KNOWN_ISSUE`, defined behaviour) | `KNOWN_ISSUE_UB` + `KNOWN_ISSUE` | 223 |
| B-MIDI-6 | `GC:355` | without a handler nothing reads the RX FIFO and the OUT endpoint is not armed again (to be verified by a T2 case) | `KNOWN_ISSUE` if confirmed | 224 |
| B-MIDI-7 (candidate) | `MM:115` | `sinf(M_PI * p)` with `p = counter * f` oscillates at f / 2 (an octave below the note); no document decides it | `characterization:` until question 54 | none |
| B-MIDI-V1 | `MD:288-312` | vendor: real-time messages (F8-FF) are written with CIN 5 instead of 0xF | `KNOWN_ISSUE` | vendor bug V86 (reserved) |
| B-MIDI-V2 | `MD:155-160` | vendor: a reserved CIN (0, 1) makes `tud_midi_n_stream_read` return 0 and discards the count of the bytes already copied | `KNOWN_ISSUE` | V87 |
| B-MIDI-V3 | `MD:263-286` | vendor: a real-time byte inside a SysEx becomes SysEx payload | `KNOWN_ISSUE` | V88 |

NEEDS REVIEW candidates (numbers from 263): the choice of `pi * f * t`; the octave pitch bend range; the A440 note table (not in the supplied documents); channel pressure and mode messages 124-127; the cable number ignored; the initialisation order of the driver; the `midihandler` pointer shared by two handlers; the timer that is never stopped; the `wTotalLength` of the MS header (Table 6-2 against Table B-6); the data race between the USB task and the timer interrupt (code reading only); the 64 byte ZLP rule (not in the supplied documents); the single AudioControl / MIDIStreaming layout of `midid_open` (one interface only, `CFG_TUD_MIDI` 1).

## Fake plan (T1)

* **Feature header:** new `platform/usb_midi/target_constants.h` = `platform/target_f407/target_constants.h` plus `#define MIDI`, first in the include path of `ffb_usb_tests` only (the same way `target_f407_drivers` is used by `ffb_driver_tests`); the other executables keep their headers byte for byte.
* **Source list of `ffb_usb_tests`:** add `MidiMain.cpp` and `MidiHandler.cpp`; `global_callbacks.cpp` and `mainclass_chooser.cpp` are already compiled and change behaviour with the macro (`tud_midi_rx_cb`, the chooser entry).
* **Driver double (owner question 53, lead decision L1):** `MidiMain.h` includes `TMC4671.h`, whose real content exists only with `TMC4671DRIVER` (off in `ffb_usb_tests`). CLAUDE.md says: fake a driver that is not built into the executable at its interface. `fakes/FakeTmcPhase.{h,cpp}` provides a shadow header `platform/usb_midi/TMC4671.h` (same include guard `TMC4671_H_` as the real one, only used by `ffb_usb_tests`) with `class TMC4671` / `TMC_1`, `TMC4671Limits`, `EncoderType_TMC`, `PhiE`, `MotionMode`, `MotorType` and the used members (`setLimits`, `setEncoderType`, `conf.motconf.motor_type`, `setPhiEtype`, `setUdUq`, `allowSlowSPI`, `initialize`, `setMotionMode`, `hasPower`, `isSetUp`, `initializeWithPower`, `setPhiE_ext`) that records every call with its arguments in order, can fail `initialize()` and report a motor type, with the fidelity header (models: nothing of the chip; source: the call sites in `MidiMain.cpp`; confidence: Low for the semantics, High for the interface; strict: a call outside the used set cannot compile). The real TMC4671 driver is tested in `ffb_driver_tests` and not re-tested here. If the owner prefers a different place for `MidiMain` (question 53, option B) only this item and the executable of group MMN change.
* **Fake user timer:** `htim9` (`TIM_USER`) with registers `ARR`, `PSC`, `CR1`, strict `HAL_TIM_Base_Start_IT` (the existing model in `host_target.cpp:236`), a test hook that delivers the update interrupt to the `TimerHandler` list (`timerElapsed(&htim9)`), reset by `HostTarget::reset()`. It follows the fake TIM1 of P14 (`platform/src/host_driver_timers.cpp`) and goes into a file of its own, linked in `ffb_usb_tests` only; fidelity header (RM0090 TIM9 / HAL_TIM_Base_Start_IT, no counter).
* **USB host helpers:** `UsbHost` gets `midiInterface()` (AudioControl and MIDIStreaming interface numbers, endpoints 0x02 / 0x83, packet size) next to `cdcControlInterface()` / `hidInterface()` (additive edit of a shared support header, noted in PROGRESS); the transfers use the existing `sendOut` / `receiveIn`.
* **`support/MidiFixture.h`:** `UsbFixture` started with `usb_cdc_midi_conf`; `MidiProbe : MidiHandler` recording every hook call (hook, arguments, order); `MidiMainFixture` (a `MidiMain` with the driver double, the timer hook, `tick(n)`); packet builders (`pkt(cable, cin, b1, b2, b3)`); a raw TX/RX FIFO inspector is NOT added (tests use the public `tud_midi_*` API and the bus). Fails the test on fake controller faults, unmodelled HAL calls, kernel faults and leaked kernel objects like `UsbFixture`.
* **Class tests without a handler:** the class driver tests that call `tud_midi_*` directly run without any `MidiHandler` (so `tud_midi_rx_cb` returns at once), the tests of R3 and R12 create the probe.
* **After turning the macro on:** re-run the existing `ffb_usb_tests` (the chooser and descriptor tests there must be unchanged) and compare `flags.make` of all other executables with `p16t2-default`.

## Groups and writers (T2, Sonnet 5.5, one file each, work orders `P17_WORK_ORDERS.md` written after T1 approval)

| Group | File (in `claude/cases_usb/`) | Requirements | Estimated cases |
|---|---|---|---|
| MHD | `test_midi_handler.cpp` | R2, R3 | 70 |
| MMN | `test_midi_main.cpp` | R1, R4, R5, R6 | 80 |
| MCD | `test_midi_class.cpp` | R7, R8, R11, R12 | 55 |
| MCS | `test_midi_stream.cpp` | R9, R10 | 55 |
| T1 skeleton | `test_midi_setup.cpp` | R1.3, the driver double and timer self-tests, `MidiProbe`, the packet builders, one smoke case per group | 10 |

About 270 cases. Coverage targets: `MidiHandler.cpp` 100 %, `MidiMain.cpp` at least 95 %, `midi_device.c` at least 90 % (it is 14.7 %, 30 / 204 lines today), `tud_midi_rx_cb` 100 %. Four reviewers with mutation checks on scratch copies (each in its own subdirectory under `ffb-test-build/audit/`), full gate (default, sanitizer, clang, DSP off, coverage), `coverage_report.py` gets `MidiMain` / `MidiHandler` out of `OUT_OF_SCOPE`, `COVERAGE_AND_SCOPE.md` and `PROGRESS.md` updated in the same commit.

## Lead decisions (Sonnet 5.5 as lead, T0)

1. **L1 `MidiMain` in `ffb_usb_tests` with a double of the TMC4671 interface** (question 53). Reason: the owner's rule (question 28, `CLAUDE.md`: macro on in `ffb_usb_tests` only; fake drivers that are not built in the executable at their interface); the alternative (real driver and `FakeTmc4671` in `ffb_driver_tests`) would turn `MIDI` on in a second executable.
2. **L2 USB-MIDI 1.0 is the version;** the v2.0 PDF is not used.
3. **L3 `wTotalLength` 0x0041:** plain test against the worked example (Table B-6); the Table 6-2 wording is a NEEDS REVIEW row, not a defect.
4. **L4 notes without a document are `characterization:`:** note frequency table, pitch bend range, the half frequency of the oscillation (B-MIDI-7 only becomes a `KNOWN_ISSUE` after question 54), channel mode 124-127, channel pressure, cable number.
5. **L5 vendor defects (V86-V88) are `KNOWN_ISSUE` against UM Tables 4-1 / 4-2 and the MIDI 1.0 real-time rule;** they are reported, never fixed.
6. **L6 the tests run on full speed only** (CFG sizes 64); no high speed variant exists in the firmware build used.

## Questions for the owner (open questions from 53)

53. **P17 where `MidiMain` is tested.** `MidiMain` creates a `TMC_1` driver, so it cannot be built into `ffb_usb_tests` without the TMC4671 code. Proposal (lead L1): keep `MIDI` in `ffb_usb_tests` only and use a recording double of the TMC4671 interface (shadow header + `FakeTmcPhase`), the real driver stays in `ffb_driver_tests`. Option B: test `MidiMain` in `ffb_driver_tests` with the real TMC4671 and `FakeTmc4671`, which turns `MIDI` on in a second executable. Which one?
54. **P17 note pitch.** `play()` computes `sinf(pi * counter * f)`, so the phase oscillates at half the note frequency (A4 plays at 220 Hz). Is that intended (an octave below on purpose, e.g. because the rotor force is rectified) or a defect (it should be `2 * pi`)? Also: the A440 note table is not in the supplied documents; confirm that note 69 is 440 Hz.
55. **P17 pitch bend.** (a) The bend range is one octave (`2^(val / 8192)`); the MIDI 1.0 document leaves the sensitivity to the receiver (the usual default is two semitones). Intended? (b) A bend only affects the notes sounding at that moment, a later note ignores it (B-MIDI-2): defect (a bend is channel state) or intended?
56. **P17 mode messages and pressure.** Control changes 124-127 do not clear the notes (the MIDI document says 123-127 also act as All Notes Off but a receiver may ignore them); channel pressure is dropped without any hook. Intended?
57. **P17 cable number.** `midiRx` ignores the cable number: all cables are one instrument. Intended (the board has one MIDI port)?
58. **P17 unparsed bytes and `sysexState`.** (B-MIDI-4) Single byte packets (CIN 0xF) with data bytes are decoded as events (note off for note 0); (B-MIDI-3) `sysexState` stays set after a SysEx that ends in byte 2 or 3. Defects (`KNOWN_ISSUE`) or acceptable (no host sends them, `sysexState` is not read anywhere)?
59. **P17 `range` and `power`.** Both commands accept any 32 bit value; `range` above 32767 makes the float to `int16_t` conversion in `play` undefined (B-MIDI-5). Add a limit (defect) or document?
60. **P17 input without a handler.** (B-MIDI-6) If MIDI packets arrive while no `MidiHandler` exists, the OUT endpoint is not armed again and the MIDI input stays dead after a handler appears. Only reachable in a configuration where the MIDI interface is enumerated without `MidiMain`; defect or out of scope?
61. **P17 USB-MIDI descriptor details.** (a) Table 6-2 says wTotalLength covers the header and the jack / element descriptors, the worked example (Table B-6) and TinyUSB include the endpoint descriptors (0x41). The test uses 0x41; agree? (b) Vendor findings V86-V88 (real-time CIN, reserved CIN count, real-time inside SysEx) are reported, never fixed: nothing for you to decide unless you want them upstream.

## T1 amendments (2026-11-10, built by the lead)

What was built (all under `Firmware/Tests/`, nothing under `FFBoard/` changed):
* `platform/usb_midi/target_constants.h`: `platform/target_f407/target_constants.h` plus `MIDI` and `extern htim9`; first feature header in the include path of `ffb_usb_tests` only (the other executables are unchanged).
* **The shadow header cannot work as an include directory.** A quoted `#include "TMC4671.h"` in `MidiMain.h` finds the real header in its own directory first. Fix (lead decision L7): `platform/usb_midi/TMC4671.h` carries the include guard `TMC4671_H_` of the real header and is force-included (`-include`, CMake, C++ sources of `ffb_usb_tests` only), so the real header is skipped. Recording double: `fakes/FakeTmcPhase.{h,cpp}` (fidelity header written).
* `platform/usb_midi/cmsis_os2.h`: the one call `MidiMain.cpp` needs from CMSIS-RTOS2 (`osDelay`, via `vTaskDelay`, 1 kHz tick); the fake platform had no `cmsis_os2.h` (lead decision L8).
* `platform/src/host_midi_timer.cpp`: fake TIM9 (`htim9`), initial state of `MX_TIM9_Init`, generic base-timer fake (`HostTarget::timerUpdate(&htim9)` raises one update event); `MidiMain.cpp` and `MidiHandler.cpp` in `USB_FIRMWARE_SOURCES`.
* `support/UsbHost.h`: `midiStreamingInterface()` (additive); `support/MidiFixture.h`: `MidiProbe`, `MidiFixture` (`startMidi`, `startAndEnumerate`, `sendPackets`, `receivePackets`, packet builders `pkt`, `noteOnPkt`, `noteOffPkt`), `MidiMainFixture` (`createMidiMain`, `tick`).
* `claude/cases_usb/test_midi_setup.cpp`: 12 skeleton cases (macro, driver double, `htim9`, enumeration with the MIDI configuration and endpoints 0x02 / 0x83, packet builders, handler registration, note on end to end, packet write, stream write, `MidiMain` construction, a note and the first tick, `update()` with the 500 ms delay).
Charter change: R6.2 no longer puts `allowSlowSPI` into the call order (it is a field).

## T2 amendments (2026-11-10, lead decisions after the writers' reports)
- **L9 R11.1:** packets written before the host reads arrive in order, not as one transfer (the first write is armed at once as a 4 byte transfer): plain "in order" test plus a `characterization:` test of the split.
- **L10 R11.2:** the TX capacity is 17 packets (1 in the endpoint buffer plus 16 in the 64 byte FIFO), not 16.
- **L11 R8.7 / R3.4:** the OUT endpoint is armed only while the 64 byte RX FIFO is completely empty (`_prep_out_transaction` needs 64 free bytes), so exactly one transfer is acknowledged while nobody reads. B-MIDI-6 is **confirmed by experiment** (a 4 byte transfer without a handler, then a handler, then a packet: NAK): `KNOWN_ISSUE` B-MIDI-6 in `test_midi_handler.cpp`.
- **L12 R8.6:** the refusal of a configuration whose AudioControl interface has a vendor class comes from `usbd.c` (no driver claims the class), not from `midid_open`; kept as a `characterization:` case.
- **Infrastructure fact:** `MidiFixture::sendPackets` always sends a zero length packet after a full 64 byte transfer; with no handler present (RX FIFO full) that packet is NAKed. Tests that send exactly 64 bytes without a handler use `host.sendOut(..., false)`.
- **Observation (not a defect of the project):** the comment at `usbd.h:362` says "88 bytes", the real `TUD_MIDI_DESC_LEN` is 92.
- **L13 R5.8 / B-MIDI-1:** the intended behaviour is "a note number above 127 is ignored" and the test counts the phase calls only (never the value computed from the out of range table read, which depends on stale heap content).
- **L14 R6.8:** the sentence "two `MidiMain` objects at once are not created by the chooser" is struck (`MidiMain` is not a singleton, the chooser would build two); the test covers create / destroy twice with the leak check.
- **L15 R6.2:** the recorded constructor sequence starts with `construct` (the double's own record), then `setLimits`, ...
- **L16 B-MIDI-5 covers `power`:** the writer's candidate B-MIDI-N1 (a `power` above 32767 reaches `setUdUq(int16_t, ...)` as a negative voltage) is part of bug B-MIDI-5 (question 59), marked `KNOWN_ISSUE`, not a new id.
