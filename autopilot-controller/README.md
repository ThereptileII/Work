# SeaTalk autopilot bridge

ESP32 firmware connecting an NMEA 2000 network to a SeaTalk-1 autopilot. The
ST4000 remains responsible for steering the boat. The bridge supplies navigation
and remote key commands and reports **measured pilot state** back to the network.

Confirmed hardware target: **a standard ESP32 development board and the original
Autohelm ST4000**. The existing `esp32dev` build targets the classic ESP32.

This replaces the original transmit-only sketch. Firmware builds and host tests
are provided. Compatibility with this particular ST4000, electrical polarity and timing
must still be checked on the connected hardware; this is not a sea-trial result
or a certified Raymarine product.

## Hardware and wiring

Target: **classic ESP32-WROOM / ESP32 Dev Module**, without PSRAM occupying GPIO16
and GPIO17. The pinned build uses Arduino-ESP32 2.0.17 / ESP-IDF 4.4. Do not select
an ESP32-C3/S3 or upgrade the framework without adapting and testing the RMT driver.

The reported interface is a generic transistor HV/LV converter powered with
**HV = 12 V and LV = 3.3 V**. Firmware now defaults to the common **bidirectional
BSS138 MOSFET circuit**: non-inverted signalling with open-drain TX. The exact
module and its voltage ratings have not been identified; the supply labels alone
do not establish that it uses this circuit or tolerates the installed supply.
Do not infer original-ST4000 command acceptance from ST4000+ examples: confirm
the received status and remote key behavior during the bench procedure below.

| Signal | Default GPIO | Connection |
| --- | --- | --- |
| CAN TX | 5 | CAN transceiver TXD |
| CAN RX | 4 | CAN transceiver RXD |
| SeaTalk TX | 17 | Converter LV1 signal, open-drain output |
| SeaTalk RX | 16 | **Same LV1 signal as GPIO17**, input |
| TX waveform monitor | 18 | **Leave unconnected**; reserved for collision detection |

```text
NMEA 2000 backbone ─ CAN transceiver ─ ESP32
                                      │ TX17 ─┐
                                      │       ├─ LV1 [converter] HV1 ─ yellow
                                      │ RX16 ─┘
                                      └ GPIO18: leave unconnected

ESP32 3.3 V ─ converter LV supply
12 V supply ─ converter HV supply (not the yellow data wire)
Common ground ─ converter GND, ESP32 GND and SeaTalk ground
```

This wiring applies to the usual bidirectional MOSFET module. LV1/HV1 are one
matched signal channel, distinct from the LV/HV **supply** terminals; channel
numbers may differ on your board. Its existing pull-up resistors supply the idle
levels. Both ESP32 signal pins attach to the **same low-voltage channel**, so RX
sees the pilot and the bridge's own transmissions. Do not use a second converter
channel for RX or connect GPIO18 to the converter.

The [SparkFun BSS138 circuit](https://cdn.sparkfun.com/datasheets/BreakoutBoards/Logic_Level_Bidirectional.pdf)
illustrates this topology, but does not identify the generic board. Check its
actual components and ratings before using the 12 V rail. A bare converter is
not an isolated or surge-protected marine interface. Verify LV signal voltage,
rise times and behavior with either side unpowered; firmware cannot provide
electrical protection. For an installed device, use an interface and supply
protection rated for the boat's electrical environment.

- SeaTalk yellow is approximately 12 V when idle and is pulled low for zero.
  **Never connect it directly to an ESP32 input.** Use a protected level-shifting
  receiver or a suitable isolated SeaTalk interface.
- The transmitter must release the wire for logical one. Never drive yellow high
  with a push-pull output. Provide a hardware bias that releases the bus while
  the ESP32 is unpowered, resetting or booting.
- For a non-isolated interface, grounds must be common. For an isolated design,
  preserve its intended isolation and use the appropriate isolated power supply.
- Keep the CAN transceiver, bus power and backbone termination appropriate to
  the existing NMEA 2000 installation. Do not add a third terminator.
- Power the ESP32 through a suitable regulated supply, never from 12 V directly.

### Interface configuration

Edit [`include/Config.h`](include/Config.h) to match the interface:

| Setting | Default | Meaning |
| --- | --- | --- |
| `seaTalkInterface` | `SharedMosfet` | Shared bidirectional converter with open-drain TX |
| `txOpenDrain` | `true` (derived) | GPIO LOW pulls down; GPIO HIGH releases the shared signal |
| `txInverted` | `false` (derived) | Logical HIGH releases yellow |
| `rxInverted` | `false` | RX GPIO HIGH means yellow is idle/high |
| `seaTalkTxMonitor` | `18` | Unconnected output carrying a copy of the TX waveform |
| `controlEnabled` | `true` | Accept remote commands after their prerequisites pass |
| `seaTalkListenOnly` | `false` | Start with SeaTalk transmission enabled for normal operation |
| `publishRudder` | `false` | Publish rudder only after verifying a real measurement |

For a **separate** NPN transmitter and receiver, select `SeparateInvertingTx`.
That profile drives the transmitter input with inverted push-pull logic; it must
not be used with the shared LV connection above. GPIO17 goes to the transistor
input, GPIO16 to the protected receiver output, and GPIO18 remains unconnected.
Set `rxInverted` to match that receiver. The shared MOSFET profile requires
non-inverted RX and rejects an inverted RX configuration at compile time.

The normal-operation build starts with **SeaTalk transmission enabled**. Remote
control, measured feedback and navigation/wind/speed forwarding are available;
each still requires its documented valid inputs and pilot state. Booting does
not send an AUTO or TRACK command. Rudder publication remains off until a real
sensor has been verified.

For initial commissioning of another installation, set `seaTalkListenOnly=true`
and rebuild before connecting. Listen-only receives SeaTalk and publishes measured
feedback on CAN, but rejects remote control and does not forward navigation,
wind or speed to SeaTalk. `listen on` enables this mode and cancels pending
transmissions and commands; `listen off` enables transmission. These serial
choices last until reboot; `seaTalkListenOnly` sets the startup behavior.

`controlEnabled=false` alone disables remote control but still allows data
forwarding when listen-only mode is off; it is not a receive-only setting.

## Behavior

### Reported state and commands

- Decode SeaTalk `0x84` for compass heading, locked heading, mode, off-course and
  wind-shift flags. Rudder bytes are decoded but publication is opt-in.
- At boot, mode and heading are unavailable. AUTO is not inferred from a sent key.
- Feedback expires after 3 seconds. NMEA status then becomes unavailable; the
  bridge does **not** claim the pilot has disengaged.
- AUTO, WIND, TRACK and heading commands require fresh pilot feedback. STANDBY
  may be attempted without it, but still needs feedback to be confirmed.
- Only one control transaction runs at a time. New commands receive a negative
  acknowledgment while busy; STANDBY preempts pending work.
- Absolute headings and +/-1 or +/-10 requests operate in confirmed AUTO. An
  absolute target is approached using individual +/-10 and +/-1 keys, checking
  the reported locked heading after every step. Resolution is one degree, with
  a 0.55-degree confirmation tolerance.
- An unchanged target does not cause repeated keys. Missing echo, lost feedback,
  unexpected mode/heading changes, or timeouts end the transaction.
- A key with uncertain delivery is **never automatically retried**. It may
  already have reached the pilot even when its echo was damaged.
- A physical STANDBY event cancels pending work. The physical pilot remains the
  source of actual operating state.

A NMEA group-function acknowledgment means the bridge accepted the request.
`[CONTROL] confirmed` means a subsequent pilot status matched the requested mode
or heading. SeaTalk does not provide a unique transaction acknowledgment: this
cannot prove which controller caused a matching status, nor whether a mechanical
clutch is engaged. Use one autopilot on this SeaTalk bus.

### Track following and waypoint changes

TRACK requires confirmed AUTO or TRACK plus fresh navigation (including a valid
destination waypoint ID) from one selected NMEA source. The bridge uses **position-to-waypoint bearing**, never locked
heading, COG or origin-to-waypoint bearing as a substitute.

Navigation uses PGNs 129283 and 129284, including navigation-terminated flags and
the bearing's own true/magnetic reference. True bearings require fresh variation.
SeaTalk variation takes precedence over NMEA variation while it remains fresh.
Unavailable or out-of-range data is not replaced by zeros or clamped into a
plausible route. On a waypoint change, XTE must be refreshed before forwarding.

A corrected `0x85` packet is followed by an `0x82` waypoint-change notification.
The four-character SeaTalk waypoint name is a bridge-generated base-36 token,
not the OpenCPN waypoint name. Its sequence is persisted to avoid reusing the
previous token after a normal restart.

Each explicit TRACK request sends **one** track key. If the pilot asks to accept
a turn, inspect its indication and issue another explicit TRACK request or use
the physical control. There is no timed or automatic second press. A matching
TRACK mode alone does not prove that a waypoint-turn prompt has been accepted.

Expired navigation stops navigation transmissions and pending track engagement.
It does not automatically switch the actual pilot to STANDBY or AUTO. The pilot's
own missing-data behavior and alarms must be tested on the installed unit.

### Wind and speed

- Forward fresh NMEA **apparent** wind as SeaTalk `0x10`/`0x11`; true wind is not
  relabelled apparent.
- A fresh local SeaTalk `0x10` from an existing wind instrument also permits WIND
  engagement. Its data is not echoed back onto SeaTalk.
- WIND sends the documented Standby+Auto key combination (`0x23`) once, then
  waits for Wind mode feedback.
- Forward fresh speed through water (128259) as `0x20`.
- Unused COG/SOG parsing was removed. It cannot silently replace a waypoint
  bearing or the pilot's measured heading.

### NMEA 2000 interface

The pinned NMEA2000 library implements fast packets, address claiming, discovery,
PGN lists, configuration strings, ISO requests and heartbeat. The TWAI adapter
adds bounded driver queues, error counters and bus-off recovery.
An additional guard validates the DLC, sequence, destination and a 250 ms total
assembly deadline for control/navigation fast packets before library reassembly.

| Direction | PGNs |
| --- | --- |
| Status out | 127250 heading, 127237 heading/track status, 65379 mode, 65360 locked heading |
| Optional out | 127245 rudder |
| AutoTrack profile out | 65359 measured heading, 126720 Evolution-format measured pilot status |
| Navigation in | 129283 XTE, 129284 navigation |
| Other input | 127258 variation, 130306 apparent wind, 128259 water speed |
| Control in | Addressed, fast-packet 126208 commands targeting 65379 or 65360 |
| Network management | Library-managed claims, requests, PGN lists, product/configuration and 126993 heartbeat |

Commands must be addressed to the bridge's **currently claimed address**.
Broadcast commands, commands for another device, acknowledgments, malformed
fields and ambiguous byte-pattern shortcuts cannot actuate the pilot.

Supported command fields:

- Manufacturer selector 1 = 1851 and industry selector 3 = 4 are required.
- Target 65360: field 6 is a two-byte magnetic heading, in 0.0001 radians.
- Target 65379: field 4 is a two-byte mode: `0000` standby, `0040` auto,
  `0100` wind, `0180` track.
- Target 65379: optional field 5 is accepted only as the two-byte `FFFF`
  placeholder sent by AutoTrackRaymarine; it never constitutes an action.
- Target 65379 only: legacy field 6 is a one-byte button (`00`, `40`, `01`, `80`,
  `51`, `7F`, `D1`, `50`, as in the original sketch). This compatibility mapping
  requires checking with the specific OpenCPN plugin. No heuristic scanning is
  used and multiple actions in one request are rejected.

For example, the documented OpenCPN heading command payload
`01 50 FF 00 F8 03 01 3B 07 03 04 06 F0 3D` is supported.
The payload must be carried in proper NMEA fast-packet frames.

`navigationSource` and `windSource` default to first applicable sender and remain
locked until restart. XTE and navigation must originate from the same sender.
Set explicit source addresses in Config.h if several producers exist. The
optional `controllerSource` restricts remote controls. Source address restrictions
are routing controls, not authentication.

The bridge uses Raymarine manufacturer fields for interoperability, but advertises
its own model/serial and certification level zero. It does not impersonate a
keypad by emitting a fabricated keypad heartbeat.

### SeaTalk transport

RMT hardware transmits 11-bit characters at 4800 baud: start, eight data bits,
explicit command bit, stop. There is no parity-toggle approximation. RMT also
captures received pulse durations; bounded decoding validates framing and length.

Transmissions wait for a high idle bus with randomized backoff. A 50-microsecond
monitor compares the intended waveform on unconnected GPIO18 against RX16,
allowing interface propagation delay, and stops a transmission on sustained
disagreement. Another talker can pull both shared LV pins low, so comparing
GPIO17 against GPIO16 would miss that collision. RMT drives GPIO18 and GPIO17
with the same waveform; GPIO17 remains open drain in the shared profile.
Initialization preloads the release level and enables open drain before output;
the RMT setup's push-pull configuration is applied only to GPIO18. The driver
restores GPIO18's RMT routing after enabling input readback, because ESP-IDF 4.4
resets output routing when changing the GPIO direction. Full packet echo is
also checked. Echo proves the wire transmission, not pilot acceptance.

The monitor is a FreeRTOS timer-task callback, so abort latency is **not a hard
real-time guarantee** under all interrupt/flash loads. Captured echo remains the
second check. Validate collision behavior and CPU-load timing on the actual board.

## Build and test

Install PlatformIO Core, then run from this directory:

```sh
pio run
bash tests/run.sh
```

`pio run` installs the versions pinned in platformio.ini. Tests also need a host
C++17 compiler (`g++`) and use AddressSanitizer and UndefinedBehaviorSanitizer.
Driver tests compile the actual SeaTalk driver against simulated GPIO/RMT APIs
and cover shared-node startup, waveform echo, collisions and failure handling.
They do not simulate transistor behavior or establish hardware timing margins.
The generated firmware is `.pio/build/esp32dev/firmware.bin`.

After wiring and bench checks, upload and open the monitor with:

```sh
pio run --target upload --upload-port /dev/ttyUSB0
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

Replace the port with the actual board port. No firmware was flashed as part of
the software implementation.

Monitor commands: `status`, `raw on`, `raw off`, `can on`, `can off`,
`listen on`, `listen off`, `help`. Logs are buffered and
dropped with a counter if full; they never wait for a slow serial reader.
Status includes feedback age, selected data sources, command outcomes, framing
errors, collisions, echo failures, CAN bus-off/recovery and queue-drop alerts.

With `raw on`, failed transmissions also emit `[ST RESULT]` with the packet,
elapsed microseconds, and one of these reasons:

- `wire-mismatch`: the intended TX waveform and the received line disagreed.
  This can indicate bus contention or an electrical problem; the counter alone
  cannot distinguish them. `monitor_levels=2` means intended HIGH, received LOW.
- `different-packet`: a complete received packet did not match our pending echo.
- `echo-timeout`: no matching echo arrived within 100 ms.
- `timer-start` / `rmt-start`: an ESP32 driver failed to start the operation.
- `cancelled`: software explicitly aborted the pending transmission.

`[ST CAPTURE]` and `[ST PULSES]` report bounded raw pulse timings for a receive
framing error, or a discarded collision capture (`errors=0`).
`max_sample_gap_us` measures the longest interval between wire-monitor samples;
it is not the signal rise time. Successful control-packet echoes log `echo-ok`,
but pilot execution still requires a subsequent `[CONTROL] confirmed`.

Collision and echo-failure counters overlap: one detected collision also counts
as one unsuccessful echo. They must not be added as independent failures.
Uncertain steering keys are never automatically retried. Background data can be
sent again with fresh values after the collision backoff.

## Bench commissioning

### Capture serial diagnostics

On Linux, the capture tool needs only Python 3. Close other serial monitors first:

```sh
python3 tools/capture_serial.py --list
python3 tools/capture_serial.py --port /dev/ttyUSB0 --listen-only
```

It captures for 60 seconds at 115200 baud, displays the output, and saves the
exact received bytes to a new UTC-named file under `logs/`. After two seconds of
startup time, it sends `raw on` and `status`, repeating every five seconds. It
uses `--listen-only` to additionally send `listen on`, disabling remote control
and all SeaTalk forwarding while preserving reception. The mode remains enabled
after capture finishes. Without that option the tool leaves the current mode
unchanged. For the initial receive-only test, leave GPIO17 disconnected
and GPIO18 unconnected, and connect GPIO16 to the converter's LV signal channel.
Opening a USB serial port can reset some ESP32 boards.

Omit `--port` to select the only detected USB serial device automatically. Use
`--seconds 0` to capture until Ctrl+C, or `--output logs/my-test.log` to choose a
new file. Existing files are never overwritten. A USB disconnect ends capture
with an error and preserves the data already saved. Share the log with the
ST4000's displayed mode and heading so we can compare them with the received data.

### OpenCPN / gateway diagnostics

Add `--can` to capture CAN traffic alongside SeaTalk:

```sh
python3 tools/capture_serial.py --port /dev/ttyUSB0 --listen-only --can --seconds 180
```

`[CAN RX]` shows received frames before destination/fragment filtering, including
commands addressed to another device. `[CAN TX_QUEUE]` means the local driver
accepted a frame for transmission, not that OpenCPN received or accepted it;
`[CAN TX_FAIL]` means local queuing failed. Each line includes device uptime,
CAN identifier, PGN, source, destination and unchanged frame bytes, including
fast-packet headers. The trace covers autopilot commands/status, navigation,
wind, water speed, variation and device discovery. It is limited to 60 frames
per second; `status` reports trace-rate drops and serial-log drops separately.
Use `can off` to disable the trace after testing.

`[N2K COMMAND]` reports parsing/acceptance for commands that reach a supported
group-function handler. The corresponding group-function acknowledgment appears
as outgoing PGN 126208 frames. A positive acknowledgment means the request was
accepted; `[CONTROL] confirmed` requires subsequent matching pilot feedback.
Neither a queued CAN frame nor a changed plugin button alone proves pilot action.

The current `autoTrackCompatibility=true` profile targets **AutoTrackRaymarine
2.3 in Evolution mode**, connected through the user's Actisense USB gateway.
It starts by claiming **address 204**, which this plugin hard-codes as its command
destination, rather than restoring the previously saved address 33. Normal NMEA
address arbitration still applies: if the bridge must move to another address,
the plugin's fixed destination will no longer match. Check `status` before testing.

The profile adds measured heading PGN 65359 and the 13-byte Evolution status
wrapper consumed by this plugin in PGN 126720. That wrapper preserves the last
validated SeaTalk `0x84` heading, alarms and other bytes, with its mode byte
adapted to the consumer's `40` standby / `42` auto representation. It is never
generated from a requested or guessed mode, or after feedback expires.
PGN 65360 is suppressed in STANDBY or when its target is unavailable: this plugin
otherwise treats receipt of that PGN itself as AUTO, even if the heading is NA.
The standard status PGNs continue to report unavailable state after feedback loss.

This profile addresses the observed consumer behavior in
[AutoTrackRaymarine 2.3.2 source](https://github.com/douwefokkema/AutoTrackRaymarine_pi/blob/14defb29f82ddaa0bd0b77d446c4b1c1ad493b68/src/AutoTrackRaymarine_pi.cpp).
That plugin's displayed pilot state covers STANDBY/AUTO/its own route tracking;
do not assume its UI represents the bridge's separate WIND/SeaTalk TRACK modes.
Its inspected pilot-discovery flag also has no implemented expiry: plugin
visibility alone does not establish fresh feedback. Verify loss behavior and
actual pilot state separately before relying on its display.

### Hardware checks

1. Confirm model, converter circuit and ratings, power supply, pin availability
   and polarity. Verify GPIO18 is unconnected and LV1 stays within ESP32 limits.
   Leave the drive mechanically disengaged during command tests.
2. With only the receiver connected, enable raw logging. Capture `0x84` in
   Standby, Auto, Wind and Track where available. Compare heading and target with
   the pilot display. No `0x84` means remote engagement remains unavailable.
3. Stop incoming pilot data. After 3 seconds, check that the bridge reports
   UNKNOWN/unavailable rather than STANDBY or the last known target.
4. On an isolated bench bus, scope TX and RX. Verify the explicit ninth bit,
   correct polarity, idle release, echo matching and reset/power-off behavior.
5. Verify one AUTO/STANDBY command, then one +1, -1, +10 and -10. Each must move
   the target once. Remove RX during a change and confirm no repeated steering.
6. Inject known XTE signs, magnetic/true bearings, zero-degree bearing, distance
   boundaries and invalid values. Check decoded packets with an independent
   analyzer. Check the actual direction of correction before track trials.
7. Exercise TRACK engagement and waypoint-change prompts. Validate first and
   second explicit presses for this exact pilot; never assume a matching TRACK
   mode confirms the turn prompt.
8. Interrupt navigation, wind and CAN; create a bench address collision and a
   controlled SeaTalk collision. Confirm cancellation, unavailable status and
   recovery without replaying old keys. Test physical STANDBY during a heading
   sequence.

Only after those checks should supervised on-water testing establish behavior
with the actual drive, instruments and OpenCPN plugin.

## Code and references

- `src/BridgeCore.cpp`: portable decoders, encoders and confirmation state machine.
- `src/SeaTalkBus.cpp`: RMT capture/output, idle/backoff and echo checks.
- `src/NmeaBus.cpp`: TWAI adapter and recovery.
- `src/BridgeApp.cpp`: NMEA integration, scheduling, identity and diagnostics.
- `tests/`: portable regression and real-library transport integration tests.

Protocol sources: [Thomas Knauf SeaTalk reference, revision 3.23](https://plaisance-pratique.com/IMG/pdf/SeaTalk_Technical_Reference.pdf),
[CANboat definitions](https://github.com/canboat/canboat),
[NMEA2000 library](https://github.com/ttlappalainen/NMEA2000),
[OpenCPN Autopilot Route heading-command example](https://opencpn-manuals.github.io/plugins/autopilot_route/index.html).
SeaTalk and proprietary PGN definitions are reverse-engineered and require
hardware validation; the references are not a certification of this bridge.
