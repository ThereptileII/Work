# OpenNav X Beta 2 — desktop and boat-PC checks

Use the installer for the real OpenCPN environment. Use the portable recovery ZIP
for isolated troubleshooting. This checklist records observations; it does not
certify the product for navigation.

Before starting, record the build/commit from **System → Diagnostics**, the
installer or ZIP SHA-256, Windows scaling, screen resolution and whether this is
the installed profile or an isolated recovery profile. Mark each check **Pass**,
**Fail**, **Unavailable** or **Not tested**. A disabled action or missing sensor
is not proof that its underlying hardware works.

## Installation and recovery

1. Back up the real OpenCPN profile and close all modes.
2. Install/update with the supplied Setup. Check the detected OpenCPN and recovery
   location; stop if compatibility fails.
3. Read any expected version-change navigation caution before continuing. Open
   XNav, Legacy and Safe Mode in turn. Confirm the same charts, routes,
   waypoints and settings remain available in installed mode.
4. Test **System → Open Legacy OpenCPN**, then the Legacy menu's
   **Switch to XNav** entry. Also test **System → Safe Mode** and return to XNav.
   Close/restart normally and wait for each startup to finish. Coastlines/chart content must remain
   visible. A blank or all-water image is a failure when land should be in view.
5. Close normally and verify Windows, SSH and RustDesk remain accessible.

During remotely managed boat testing, use the separately prepared guarded
restart procedure. Do not bypass a refused transition with an unreviewed direct
launch. An ordinary close followed by another shortcut is recorded separately
from a successful in-application mode switch.

## Navigation and visual checks

Start at the boat's **1280×800** resolution and current Windows scaling. Exercise
125%/150% in the disposable Windows test environment; change the boat's display
configuration only with a recoverable local test plan.

- Check chart pan, zoom, chart switching and ownship following when GPS is valid.
- Check the four primary data-rail values stay visible when an alert appears.
- Try Day, Dusk and Night on navigation, instruments, energy, AIS, anchor,
  autopilot, settings and diagnostics. Hover Menu and the data rail: Day hints
  may appear; primary Dusk/Night controls should not create bright hover windows.
  Record bright native/Legacy dialogs separately.
- Open/close contextual sheets with their Back/Cancel actions and Escape.
- Review route/waypoint browsing and selection, AIS selection/details, settings
  categories, and source-health details. Note any clipped or unreachable control.
- Check Diagnostics shows Beta 2 and the expected commit from BUILD_INFO.md.
- No normal product launcher, menu or screen should offer synthetic trip scenarios.

Useful paths are **Menu → Vessel instruments**, **Energy**,
**Menu → Settings → SENSORS**, and **Menu → Settings → DISPLAY**.
Display provides **Configure data rail**, **Configure instruments** and
**Fullscreen / window**. Check that the rail contains at most four chosen values,
their order survives a normal restart, and expanded instruments remain reachable
by scrolling. The chart's **North/Course** control changes orientation;
**Center** requires a usable ownship position.

With actual received AIS, select a chart target, review the compact card and open
**Details**. Without received AIS, record **Unavailable** and check the no-data
message. Observe existing SmartNav/anchor alerts only during the read-only boat
pass; do not create alarms, change the anchor watch or acknowledge a real alarm
merely to obtain screenshots.

## Navigation edits

On an isolated desktop recovery profile, right-click/long-press the chart and
choose **Waypoint**, enter a name and **Save**. Use **Menu → Routes → Create route
on chart**, tap three positions, then **Undo** once. **Done** opens naming;
**Cancel** there retains the draft, while the chart's **Cancel** action discards
it after confirmation. Save, rename, reverse and remove only explicit test
objects. Test Go To or route activation/stop only in a disconnected desktop
profile with all output connections and command-capable plugins disabled and a
valid, deliberately supplied read-only position source. If that source is absent,
verify the action is unavailable instead of inventing a successful test.

**On the actual boat PC, do not activate routes or change output settings during
remote read-only testing.** OpenCPN/plugins may emit navigation messages even
while OpenNav autopilot control is disabled. Preserve real user objects.

## Real data, instruments and energy

Observe only sources which are actually connected. Record GPS, heading, wind,
depth, STW, rudder, propulsion, battery and tank availability separately. Compare
values against existing instruments and inspect age/source in Diagnostics.

Missing values must remain unavailable. Stale values must be marked, and
calculations depending on stale SOC/GPS/route inputs must disappear. If no route
is active, arrival SOC should be unavailable. Energy predictions are advisory;
review capacity/reserve/calibration assumptions before assessing accuracy.

Sensor disconnection tests and any physical configuration changes require a safe,
deliberate local test plan. Do not unplug shared vessel equipment remotely merely
to produce an error state.

## Autopilot and other equipment

Keep **Autopilot Control OFF**. Status-only observation is permitted after source
and connection review. Do not press AUTO, STANDBY, ±1, ±10, TRACK or WIND during
remote Beta 2 validation. No propulsion, switching or radar-transmit command is
part of this checklist. Physical command tests require separate explicit approval.
SmartNav advice must never execute a steering command.

## Maintenance checks

In a disposable Windows environment, exercise update, repair, rollback,
uninstall and reinstall. Verify original OpenCPN remains launchable and user data
is unchanged. On the real boat, retain the verified recovery set and do not leave
an unvalidated installation active. Return to known-good Legacy/Safe if needed.

## Reporting

Use **System → Export diagnostic bundle → Export Diagnostic Bundle** for the
product's sanitized operational report. Nothing is uploaded automatically.
Include the exact version/commit, action sequence, expected/actual behavior,
screen resolution/scaling and relevant screenshots. The separate
**Export with selected recording...** action includes only the recording you
choose and confirm; review it before sharing. Never send the complete profile, chart
files, passwords or unrelated Desktop files.

Screenshots can contain vessel position and licensed chart content; review or
redact them before uploading. A missing physical sensor should be reported as
unavailable, not as a successful hardware test.
