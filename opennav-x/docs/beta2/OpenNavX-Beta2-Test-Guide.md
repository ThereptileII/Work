# OpenNav X Beta 2 — desktop and boat-PC checks

Use the installer for the real OpenCPN environment. Use the portable recovery ZIP
for isolated troubleshooting. This checklist records observations; it does not
certify the product for navigation.

## Installation and recovery

1. Back up the real OpenCPN profile and close all modes.
2. Install/update with the supplied Setup. Check the detected OpenCPN and recovery
   location; stop if compatibility fails.
3. Open XNav, Legacy and Safe Mode in turn. Confirm the same charts, routes,
   waypoints and settings remain available in installed mode.
4. Test XNav → Legacy → XNav and Safe → XNav. Coastlines/chart content must remain
   visible. A blank or all-water image is a failure when land should be in view.
5. Close normally and verify Windows, SSH and RustDesk remain accessible.

## Navigation and visual checks

At the physical display resolution, then at supported 125%/150% scaling:

- Check chart pan, zoom, chart switching and ownship following when GPS is valid.
- Check the four primary data-rail values stay visible when an alert appears.
- Try Day, Dusk and Night on navigation, instruments, energy, AIS, anchor,
  autopilot, settings and diagnostics. Record bright native/Legacy dialogs.
- Open/close contextual sheets with their Back/Cancel actions and Escape.
- Review route/waypoint browsing and selection, AIS selection/details, settings
  categories, and source-health details. Note any clipped or unreachable control.
- Check Diagnostics shows Beta 2 and the expected commit from BUILD_INFO.md.
- No normal product launcher, menu or screen should offer synthetic trip scenarios.

## Navigation edits

On an isolated desktop recovery profile, create a waypoint and a short route,
undo a point, cancel an unfinished edit, save, rename, reverse and remove the
explicit test objects. Activate/stop a route only in a disconnected desktop
profile whose output connections are disabled.

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

Use **Export Diagnostic Bundle** for the product's sanitized operational report.
Include the exact version/commit, action sequence, expected/actual behavior,
screen resolution/scaling and relevant screenshots. Select a recording only when
needed and review privacy before sharing. Never send the complete profile, chart
files, passwords or unrelated Desktop files.

Screenshots can contain vessel position and licensed chart content; review or
redact them before uploading. A missing physical sensor should be reported as
unavailable, not as a successful hardware test.
