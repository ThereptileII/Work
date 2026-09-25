# Beta boat-source inspection — 2026-09-25

This is source inspection, not evidence of firmware currently flashed aboard.
No physical control commands were sent during this work.

## Propulsion bridge

Inspected `ThereptileII/Work` main revision
`9baf01bca09522794a9678dfb3f3c0720d5c9943`,
[`Nissan_ev_to_NMEA_and_wifi.ino`](https://github.com/ThereptileII/Work/blob/9baf01bca09522794a9678dfb3f3c0720d5c9943/Nissan_ev_to_NMEA_and_wifi.ino),
SHA-256 `a7fa8d8ad1c26f5f72401f84e42996242d375822f301f2ad4a930b5e718fc53d`.
The ESP32-C6 dual-CAN firmware keeps EV decoding aboard. OpenNav consumes only
marine messages delivered by the existing OpenCPN bus.

| Marine message | Actual producer behavior | PC interpretation |
| --- | --- | --- |
| 127488 | Instance 0, magnitude RPM, 0.25 RPM resolution | Standard unsigned RPM; no inferred direction |
| 127493 | Instance 0, forward=0/neutral=1/reverse=2/unknown=3 | Copied gear with age/validity; unknown unavailable |
| 127506 | Fast packet, 11 bytes, instance 0 battery, SOC byte; SOH absent | SOC; SOH unavailable, no guessed capacity |
| 127508 | Signed 0.01 V; above 327.65 V uses NA; 0.1 A positive charging | Preserve pinned codec. Never reinterpret unsigned to manufacture HV voltage |
| 127751 | 8-byte DC connection 0, 0.1 V, signed 24-bit 0.01 A positive charging | Wider voltage/current pair; explicit pack identity/sign still required |
| 127489 | Fast packet, 26 bytes, engine-temperature field at byte 5 contains motor temperature in 0.01 K | Generic coolant field until explicit boat-specific interpretation is configured |
| 127505 | **Virtual fuel tank represents SOC**, 100 L fictional capacity | Must not be represented as a physical fuel tank when this boat adapter is selected |
| 61184 | Custom 8 bytes: SID/version=1/regen/flags/current/power, no standard manufacturer header | Disabled by default; requires exact documented adapter identity, not blanket proprietary-PGN decoding |

PGN 127508 does **not** contain SOC (the task's diagnostic example is illustrative).
Whole-pack power derives from coherent voltage/current with configured sign; it
must not be labeled motor or shaft power. Physical pack capacity is user input.

The producer's 127751 voltage uses a signed helper and signed NA. The current
[CANboat field contract](https://canboat.github.io/canboat/canboat.html#pgn-127751)
uses unsigned 0.1 V and signed 0.01 A. Normal boat HV bytes agree; the older
3276.7-V NA encoding is outside the PC battery domain and is invalidated. This
mismatch needs firmware correction/bench verification, not silent reinterpretation.

### Critical upstream freshness limitation

`taskN2kTx` re-emits the retained `Telemetry` every 100 ms. There are no per-field
expiry timestamps; `lastMotorDataMs` is aggregate and not used to invalidate
these transmitted fields. Loss of EV RPM/temperature/power inputs can therefore
leave plausible repeatedly transmitted old values. A changing marine SID does
not establish a fresh underlying sensor sample. PC diagnostics report **receipt
age**, not proof of EV sensor freshness. Boat commissioning must verify proper
producer invalidation or a versioned heartbeat/age contract before relying on
these inputs. Constant values alone cannot safely detect this failure.

## Pinned OpenCPN boundaries

- `model/src/comm_can_util.cpp::IsFastMessagePGN` already includes 127489/127506;
  127751 is single-frame. Existing driver reassembly remains authoritative.
- `model/src/comm_drv_n2k_net.cpp::HandleCanFrameInput` creates normalized owned
  receive envelopes. `MarineBridge` subscribes through `NavMsgBus` on the GUI
  thread; it neither opens a transport nor triggers navigation processing.
- `libs/N2KParser/src/N2kMessages.cpp` has 127489 inside `#if 0`; the new bridge
  copies only its inspected coolant field. The library and Legacy are unchanged.
- Standard 127493 uses the pinned library parser. 127751's small bounded decoder
  is outside upstream and uses the documented marine contract, not EV CAN IDs.
- Selected GPS/SOG/COG continue through the accepted selected-navigation bridge.

## Autopilot translator

Local `/home/standard/Projects/Autopilot controller` is clean at
`a94b816b2d5d8238366c2e7c0fe8ef15f9dde4ee` when inspected. Its actual
`BridgeCore.cpp` structurally parses addressed 126208 requests targeting
65379/65360. Fresh measured SeaTalk pilot feedback is required for commands
other than the special standby request. Feedback expires after 3 seconds.
`publishRudder=false`, `autoTrackCompatibility=true`; preferred address 204 is
configuration, not a trustworthy permanent device identity. The PC adapter must
identify the configured translator/interface and refuse mismatches. TRACK/WIND
remain disabled until the complete physical path is verified. Prior dockside
firmware testing is not acceptance of a new PC output adapter.

## Initial Beta software validation

Byte fixtures cover actual emitted HV/current values, NA/error sentinels,
coherent wide-message precedence, engine-field meaning, standard gear,
truncated envelopes, source identity, rate/dropout and invalid observation
counts. GUI diagnostics retain observation time and show measured/estimated/
invalid state separately. Same-commit Linux and native Windows gates are
required before accepting the increment.
