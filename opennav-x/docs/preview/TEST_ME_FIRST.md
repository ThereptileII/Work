# OpenNav X Developer Preview 0.1

This is a test build, **not approved for navigation**. Use an ordinary Windows
10/11 x64 PC. It preserves OpenCPN's supported 32-bit application/plugin ABI.
No administrator rights or installed OpenCPN are needed for the preview.

## Start here

1. Download the **OpenNavX-DeveloperPreview-win64** artifact from the linked
   successful GitHub Actions run. GitHub's artifact download contains the
   application ZIP and its SHA-256 file.
2. Extract `OpenNavX-DeveloperPreview-win64.zip` into a writable local folder,
   for example `%USERPROFILE%\XNav` (enter `%USERPROFILE%` in File Explorer and
   create an `XNav` folder). Keep the path short and all folders together. Do not
   extract into Program Files or run from inside the ZIP.
3. Open `OpenNavX-DeveloperPreview` and double-click **Run-XNav-Demo.cmd**.
4. Confirm the real OpenCPN chart canvas appears and the top bar visibly says
   **DEMO**. The included coastline is an overview, not a nautical chart.
5. Use **Route**, **Energy**, **Demo** and **System** along the bottom. Zoom and
   pan operate the actual OpenCPN canvas. Demo telemetry is separate: it does
   not drive OpenCPN's own-ship symbol, activate routes or transmit NMEA.

This unsigned developer build may receive Windows reputation prompts. Verify
the run, commit and supplied hash; do not disable Windows security globally.

## Visual checks

Set the window to about **1280×800**, initially at Windows 100% scaling.
Check the status bar, right data rail and chart readability. **Light** cycles
Day → Dusk → Night → Day. Open **Route**, **Energy** and **System → Diagnostics**.
Check for overlapping, clipped or unreadable labels. Content pages scroll on
smaller/scaled windows. Try your normal Windows scaling too and report it.

## Data checks

The deterministic trip runs at 60 times travel speed and finishes in about
three minutes. The displayed 6.3 kn is simulated vessel speed, not playback speed.
Watch changing wind, depth, heading, position, SOC, motor RPM/temperature and
route distance. **Energy** shows estimated range and destination SOC with an
explicit advisory label. **Cruising** in the Demo menu restarts the trip.
Diagnostics lists individual sources, validity and sample ages; scroll for all
items. Demo uses a synthetic 48 kWh pack, 15% reserve and an explicit 0.4 kW
hotel load. These are not settings for your boat.

## Failure states

Open **Demo** and choose:

| Scenario | Expected behavior |
| --- | --- |
| Cruising | Fresh demo values, active route and advisory arrival SOC |
| Sensors stale | Wait six seconds; values show STALE and arrival SOC is unavailable |
| Sensors unavailable | Wind/depth/RPM/net battery power unavailable; arrival SOC unavailable |
| Route inactive | No remaining route distance or arrival SOC; range can remain available |
| Route ending | Short approach, then no active route; never a fabricated zero-distance arrival |
| Low battery | SOC below the configured reserve; depleted/shortfall state |
| High power | Higher motor/net power and lower predicted range |
| Energy shortfall | Insufficient energy; arrival SOC unavailable, shortage reported in kWh |

## Modes and close

Use **System → Open Legacy OpenCPN**, then Legacy's **OpenNav → Switch to XNav**.
The application saves and restarts using the same isolated profile. Demo stops
on a mode restart: select Demo again explicitly. **System → Safe Mode** starts
Legacy recovery with OpenNav modules and plugins inactive. **Restart XNav**
restarts the normal interface. Close all preview windows before using a launcher.

Confirm the coastline remains visible in Legacy, after returning to XNav, and
in Safe Mode. The chart should retain its location/zoom; a blank water-only view
after switching from a coastal view is a failure. This build repairs the earlier
preview's saved default-basemap path; custom chart locations stay unchanged.

- `Run-XNav.cmd`: XNav with live fields unavailable until a supported input exists.
- `Run-XNav-Demo.cmd`: explicitly simulated trip and propulsion/energy data.
- `Run-Legacy.cmd`: original OpenCPN interface, same preview profile.
- `Run-Safe.cmd`: recovery mode, same profile, saved normal preference preserved.

Closing the preview leaves your normal installed OpenCPN and its profile alone.
The preview executable also enforces its local profile when double-clicked
directly. Do not move the executable out of `app/` or replace files in an installed
OpenCPN. Delete the extracted preview folder to remove this test build.

## Keyboard shortcuts

Ctrl+Shift+N: navigation; R: route; E: energy; I: diagnostics; S: system;
T: demo scenarios; D: restart demo cruise; P: pause/resume demo; L: Legacy.

## Report a problem

Use **System → Open diagnostics folder**. Send:

- `logs/opennav-diagnostics.json` and the launcher log from `logs/`;
- `profile/opencpn.log` (the current authoritative OpenCPN log);
- `docs/BUILD_INFO.md`;
- a screenshot and the steps/scenario which caused the problem;
- Windows version, display resolution and scaling percentage.

Logs can contain chart paths or navigation data if you manually add live inputs.
Review them before sharing. No automatic upload or crash-report submission is
required by OpenNav. Keep an untouched ZIP for a clean reset; extract a new copy
instead of copying your production OpenCPN profile into the preview.
