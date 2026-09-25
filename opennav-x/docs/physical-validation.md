# OpenNav physical validation procedures

**Not executed or accepted.** CI simulators cannot close these gates. Record the
Application commit, Windows build/DPI/GPU, boat equipment/firmware, adapter identity,
date, conditions, screenshots and diagnostics for each session. Do not include
credentials or unrestricted raw voyage logs in a public report.

## NMEA2000 and other vessel sources

Start with XNav control globally disabled. Compare each displayed quantity with
its source instrument and record units, device instance, PGN/message path,
source selection, age, sign and unavailable behavior.

| Group | Required observations |
| --- | --- |
| Navigation | Selected GPS lat/lon, SOG, COG; source change; position loss. Heading reference must be true/magnetic as labelled. STW must remain distinct from SOG. |
| Wind | Apparent speed/angle and valid true-wind reference. Disconnect the instrument; cached values must age and predictions stop. |
| Depth | Below-transducer value; installation offset/draft separately configured. Compare shallow-water alarms with upstream; never label it under-keel without the required offset. |
| Rudder/attitude | A real sensor is required. Verify sign, zero, port/starboard and physical endpoints. Do not enable the boat bridge's rudder publication without a verified measurement. |
| Motor | RPM, actual source temperature and its physical meaning, state/gear, electrical/shaft power and regeneration if supported. Generic coolant temperature is not proof of winding temperature. |
| Battery | Battery identity/instance, voltage, current direction, SOC, SOH, whole-pack power including hotel loads. Compare charging and discharging; verify capacity/reserve configuration before trusting advisory results. |
| Tanks | Physical tank/instance, fluid type, percentage and empty/full/unavailable sentinels. |
| Connectivity | Loss, reconnection, competing sources, explicit source pinning and stale fallback; inspect retained ages after page changes. |

Capture a short explicitly enabled raw diagnostic interval only when needed to
explain a discrepancy. Preserve the original timestamp/source information. Do
not calibrate by replacing missing inputs with guessed constants.

## Energy calibration

Record steady speed, whole-pack net discharge and SOC over representative boat
conditions. State wind, current, sea state, hotel loads and battery conditions.
Import a monotonic speed axis with explicit power semantics; verify interpolation
only inside the calibrated domain. Compare predicted passage energy/arrival SOC
with actual consumption. Mark discrepancies and uncertainty; a desktop
fixture is not a boat-specific propulsion curve.

For Beta, follow the ordered phases in [boat commissioning](boat-commissioning.md).

## Autopilot commissioning

Use a qualified operator at the physical pilot with immediate STANDBY access.
Start dockside with the drive safely disengaged where the equipment permits.
Do not engage steering against moorings or people. Do not enable any command
adapter until its Windows transport and address/capability configuration have
been reviewed for this physical bridge.

1. With XNav output disabled, read physical mode and locked heading. Match fresh
   pilot feedback; an outgoing request or group-function ACK is insufficient.
2. Enable only the intended adapter and one operator. Verify the bridge's current
   claimed address, device identity and feedback timeout. Never broadcast control.
3. Request STANDBY once. Verify subsequent physical feedback and command log.
4. Request AUTO once. Verify the physical display and fresh reported mode/target.
5. Exercise +1, -1, +10 and -10 separately, returning to a known target between
   cases. Verify units/reference, result, timeout and absence of repeated keys.
6. Exercise STANDBY while another command is pending. It must preempt pending
   OpenNav work; the physical state is not labelled disengaged until confirmed.
7. In a controlled test, remove communication/feedback. UI must show stale or
   unavailable, pending work must time out, and no automatic retry may occur.
8. Test command rejection, busy state, external heading change and physical
   STANDBY. Diagnose rather than invent a successful acknowledgement.
9. Test global disable, Legacy/Safe entry and application close. Verify absence
   of OpenNav command output. Disabling software does not prove pilot STANDBY.
10. TRACK is unavailable in Beta. A future supported adapter requires an appropriate supervised underway trial, fresh waypoint/XTE/
    bearing and variation where needed. Inspect the physical turn-acceptance
    prompt; no automatic second TRACK press or SmartNav steering is permitted.
11. WIND is unavailable in Beta. A future supported adapter requires a valid wind source and adapter capability. Verify physical
    Wind mode and loss-of-wind behavior in a supervised trial.

The existing ESP32 bridge's prior dockside work does not accept this new Windows
application/control path. No OpenNav at-sea result is implied by simulator tests.

## Radar

Identify the adapter/plugin, scanner and firmware. Confirm discovery and loss
states with controls disabled. Verify receive/display orientation, scale, range,
latency and overlay registration against independently known targets. Exercise
Off/Overlay/Focus only for advertised capabilities. Any transmit/power controls
need a separate procedure appropriate to the scanner and people nearby. No
synthetic returns may appear in live mode and no radar/AIS fusion is accepted.

## Windows navigation PC and sea trial

On the target PC, verify 100/125/150% DPI, physical touch targets, sunlight/night
readability, keyboard/mouse escape, OpenGL/software fallback, chart zoom/pan and
mode restarts with the installed licensed charts/plugins. Measure startup, CPU,
memory and interaction response with representative traffic. Exercise sensor
loss and Safe recovery before an attended sea trial. Retain normal OpenCPN as
the reference/fallback. OpenNav is not certified or approved for navigation use.
