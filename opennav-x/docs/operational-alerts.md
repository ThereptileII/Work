# Beta operational alerts

## Finding and adjusting alarms

SCRUM-374 (2026-10-17, source only) adds **Alarms & warnings** in two places:

- Bell → Alarms & warnings. The entry remains available with no active alerts.
- Settings → Navigation → Alarms & warnings, as the first navigation setting.

This overview is separate from **Active alerts**. Viewing either changes no
settings and acknowledges nothing. The overview shows actual profile values,
not prototype examples or presumed factory settings.

| Condition | Current behavior | Adjustment path |
| --- | --- | --- |
| AIS close approach | OpenCPN owns CPA/TCPA and alarm eligibility. The overview copies the current enabled flags, thresholds, range limit, target-dialog setting and effective moored-target suppression. Invalid numeric preferences show unavailable. Online AIS is display-only. | AIS alarm settings opens OpenCPN Ships → AIS Targets. Alarm sounds opens User Interface → Sounds. |
| Anchor watch | Requires explicit arming. Existing watch radius is retained; a new SKAGER watch proposes 50 m with adjustment before starting. | Anchor watch. Opening the view does not arm it. |
| Energy reserve / shortfall | Automatic advisory/warning only with valid route, battery and consumption inputs. Capacity and reserve start unconfigured; no default reserve is invented. | Energy reserve settings, using the existing validated energy configuration. |
| Missing/stale sensor | Automatic after usable data has been observed, or an explicit invalid observation. Generic instrument freshness starts at aging 2 s / stale 5 s; configured sources may differ. | Sensor settings → instrument → Configure freshness. This changes data validity and dependent advice, not just notification timing. GPS/route freshness is not editable here. |
| Autopilot feedback / command failure | Automatic fixed safety rules. Missing engaged feedback or an unconfirmed command is critical; other rejected requests remain explicit. | Inspect the pilot/connection. No alarm-delay or mute switch is added. |

There are currently no independent configurable measured shallow-depth,
high-wind, high-motor-temperature or low-battery-voltage threshold alarms.
Chart safety depth controls chart presentation; it does not enable a measured
depth alarm. Do not infer that an empty alert list means these conditions are
being monitored. Acknowledgement neither resolves a condition nor silences
OpenCPN's own alarms.

During replay the overview distinguishes current-installation AIS preferences
from recorded energy assumptions and source timing. Current anchor status is
unavailable; mutation entry points remain disabled. Missing callbacks are
disabled, not replaced by inert controls.

No execution evidence is claimed for this source-only change. Updated fixture
sources cover settings access with empty/live/replay alerts, repeated taps,
the Navigation settings destination, and replay guards. When testing is
authorized, run those focused fixtures and verify real profile readback,
the targeted upstream tabs, returning from settings, and native 1280×800 layout.

## Existing alert semantics

`application::AlertCenter` consumes owned Vessel Data, route-energy, copied AIS,
anchor-watch and pilot views. It has no adapter/command reference. The shell
observes the same coherent tick used for the displayed advice; retained sensor
values are assessed using their original timestamps and source freshness limits.

INFO, ADVISORY, WARNING and CRITICAL are presentation levels. GPS loss, upstream
AIS/anchor alarms, unconfirmed manual pilot commands and loss of previously
engaged pilot feedback receive critical presentation. Energy reserve is advisory;
insufficient route energy is warning. The prediction must reference the exact
current route publication and current calculation tick. Invalid/stale dependent
inputs suppress the prediction, not manufacture an energy result.

Known sensor loss is reported independently for position, heading, measured depth,
apparent wind, rudder, RPM, motor temperature, battery SOC/voltage/current. A
never-installed sensor does not generate an alert. An explicit invalid input is
reportable; valid zero remains valid. Aging is visible in source diagnostics and
is not itself treated as sensor loss. No measured-depth clearance threshold is
invented without a sounder offset contract.

The global 56-DIP strip remains above all center pages while a condition exists.
Menu → Alerts (Ctrl+Shift+F9) opens the actionable list. Acknowledge marks only the
current XNav episode. It neither clears an active condition nor acknowledges
OpenCPN/device alarms. Recovery removes a condition; recurrence creates a new,
unacknowledged episode. Demo, replay and live histories are isolated. Replay
rewind begins a new historical health interval. No automatic steering, retries,
command transmission, audible alarm override or OpenCPN alarm suppression occurs.

AIS alarm state comes from OpenCPN. An empty target list or no received targets
cannot establish receiver failure; source diagnostics retain that uncertainty.
Pilot feedback loss does not imply physical STANDBY. The instruction remains to
check the physical pilot, including physical STANDBY access. XNav control output
may already be disabled while the physical device remains engaged.

Runtime diagnostics expose IDs/severity/episode/acknowledgement. The bounded field
journal records typed transitions; the field bundle omits target/route/device
identities. The current set is bounded by the fixed implemented conditions.

Validation: three portable groups cover sensor loss/zeros/future observations,
upstream flags/physical-feedback/energy route coherence, and mode/replay isolation.
The actual preview smoke requires a persistent strip, acknowledges GPS loss,
changes pages, restores input and proves a new unacknowledged dropout episode.
Native capture and DPI review remain mandatory for acceptance. Acoustic alarms
and target navigation-PC/physical touch operation require separate field review;
existing OpenCPN alarm behavior remains intact.
