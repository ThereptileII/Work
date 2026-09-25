# OpenNav X Alpha 1 — Windows test guide

**Alpha evaluation only. Not approved for navigation.** Use a desktop PC first.
The package runs the supported OpenCPN **x86 application/plugin ABI on Windows
10/11 x64**. `win64` describes the Windows host.

## 1. Prepare and back up

Close every OpenCPN/XNav instance. Keep your original OpenCPN installer and copy
its configuration/navigation folder to a separate backup location. In normal
OpenCPN, use **Help → About** to find the configuration and log paths. Preserve
all files there, including `opencpn.ini`, `navobj.db` and any adjacent database
files. Your chart files and plugin resources may live elsewhere; preserve those
too. Do not restore an older database over newer voyage data during this test.

Alpha Setup supports only the exact executable hash listed in `BUILD_INFO.md`.
A version label alone is insufficient. Setup will refuse other builds. Do not
rename/copy another executable to bypass this check.

## 2. Try the portable package first

1. Download the `OpenNavX-Alpha1-Windows` GitHub Actions artifact and extract
   that outer archive. It contains the portable ZIP, Setup, this guide and hashes.
   Find `OpenNavX-Alpha1-Portable-win64.zip` and check its entry in
   `SHA256SUMS.txt` if desired (`Get-FileHash` in PowerShell).
2. Extract the entire ZIP to a short, writable path, for example
   `%USERPROFILE%\XNavAlpha`. Do not run files inside the ZIP viewer.
3. Open the extracted folder and run `Run-XNav-Demo.cmd`.
4. Confirm the main chart canvas and a conspicuous **DEMO** indicator. The
   bundled coastline is an overview, not a nautical chart. Demo position/route
   values are synthetic and do not place a real vessel/route on the chart.
5. Pan/zoom the chart. Use **Menu → Settings → Advanced / Legacy Settings** to
   add appropriately licensed test charts to this portable profile.

The portable package uses its own `profile/` and `logs/`. It does not install,
patch or copy your normal OpenCPN profile. Close it before testing installed
Alpha; never run two copies against one profile.

## 3. Install beside normal OpenCPN

Run `OpenNavX-Alpha1-Setup.exe`. An unsigned Alpha may trigger Windows reputation
checks; verify the downloaded artifact/hash before choosing to run it.

Select your original installed OpenCPN executable, or let Setup discover it.
Choose **Install**. Preflight must show a supported configuration; an unsupported
build must fail without changing the installation. Setup stages and verifies
OpenNav under `%LOCALAPPDATA%\OpenNavXAlpha1`, then creates a Start-menu folder.
It requires no administrator rights for OpenNav-owned files.

Launch **OpenNav X** from that folder. Unlike the portable build, installed
XNav/Legacy/Safe use your normal OpenCPN profile. Confirm your chart paths,
routes, waypoints, tracks, connections and plugin preferences are present. The
original OpenCPN executable/shortcut remains unchanged.

## 4. Navigation and display

At 1280×800 and 100% scaling, check the top source state, dominant chart, left
zoom/follow controls, right data rail and bottom navigation actions. Repeat at
125% and 150% scaling if practical. Scroll panels to reach content below the
viewport. Report clipping rather than reducing scaling to hide it.

Use **Settings → Display & layout** for Day, Dusk, Night and fullscreen. Try the
navigation, sailing and energy rail presets; choose individual instruments.
Confirm selection persists after a restart. Hardware brightness stays under
Windows/display control. Check mouse and actual touch if your PC supports it.

For real charts, exercise chart switching, zoom, pan and own-ship following with
valid position input. The portable launchers force software rendering for a
reliable starting point. To test your GPU, close OpenNav, open PowerShell in the
portable folder and run `./app/opencpn.exe --portable --configdir "$PWD/profile" --xnav`
without `--no_opengl`; enable OpenGL in Advanced / Legacy Settings. Use the
normal launcher again to return to software rendering. Record GPU, driver,
chart type and any rendering failure. A plain
all-water image is not evidence that a known coastline/chart loaded correctly.

## 5. Routes, waypoints and AIS

Use **Menu → Routes** to browse, view and inspect legs. With valid live/test
selected GPS, activate a disposable test route, inspect the active waypoint and
remaining distance, then stop navigation. Never activate a real steering route
on connected hardware solely for this desktop test.

Create a test waypoint, edit its name/description, cancel an edit, and delete it
with confirmation. Create a basic route using the chart and **Done**. Inspect
its points, edit points on the chart and reverse an inactive test route. Shared,
active or protected objects may require the preserved Legacy workflow.

Use **Menu → AIS targets** to inspect target cards/list values. Live chart
traffic, CPA/TCPA and alarms come from OpenCPN. DEMO cards are explicitly
synthetic and are not inserted into the chart AIS decoder. No encounter advice
commands steering.

## 6. Instruments, energy and SmartNav

In Demo, confirm motion, wind, depth, SOC and motor values change; remaining
route distance decreases and advisory destination SOC changes. Open Instruments,
Energy and SmartNav. SmartNav shows route/turn timing, energy events and AIS
context where its inputs are valid. These are estimates/advice.

From **Demo**, exercise: Cruising, Sensors stale, Sensors unavailable, Route
inactive, Route ending, Low battery, High power and Energy shortfall. Missing or
stale required inputs must suppress dependent predictions; no active route must
not become a zero-distance arrival. Shortfall must be explicit.

Live battery capacity, reserve, current sign and calibration are unconfigured by
default. Configure them only from known boat/source information under Energy
configuration. Do not copy the Demo assumptions as if they describe your boat.
Data Sources shows source identities, freshness and optional documented
propulsion Signal K mappings. Units/source meanings must match the physical
instrument. See the separate physical-validation checklist before boat tests.

## 7. Adapters and anchor watch

The autopilot panel supports **DEMO-only manual control** in this Alpha. Enable
it explicitly, try AUTO, ±1/±10 and STANDBY, and check fresh feedback confirmation
in the log. Live commands are disabled; unsupported TRACK/WIND stay unavailable.
SmartNav has no direct steering path.

Radar status must remain unavailable without a validated display adapter. No
live radar echoes are fabricated. Anchor watch uses normal OpenCPN position and
anchor-watch behavior. It requires valid live/test navigation; Demo does not
silently create real anchor marks. It is not an intelligent drag detector.

## 8. Legacy, Safe and recovery

Switch XNav → Legacy → XNav and Safe → XNav through the available mode actions.
At each step, check the same known coastline/chart and preserved user objects.
Confirm advanced settings/plugin windows remain available in Legacy. Safe Mode
must avoid OpenNav SmartNav/control services and retain normal plugin preferences.

Portable launchers are `Run-XNav.cmd`, `Run-XNav-Demo.cmd`, `Run-Legacy.cmd` and
`Run-Safe.cmd`. Installed shortcuts have the corresponding mode names. Startup
recovery selects Safe after repeated unfinished XNav starts. Do not force-kill a
real navigation session merely to test this; CI covers disposable crash cases.

## 9. Repair, update, rollback and uninstall

Close all modes first. Open **Maintain OpenNav** in the Start-menu folder.

- **Repair:** restore missing/corrupt OpenNav-owned files from the retained
  verified package. Confirm XNav, charts and your saved test objects still work.
  If the retained package itself is damaged, rerun the original Setup download.
- **Update:** run a newer Alpha Setup and choose Update. Confirm user data,
  custom plugin additions and mode switching remain intact.
- **Rollback:** select Rollback in maintenance. It restores the prior application
  generation; for a first install it unregisters the integration. It preserves
  your newer navigation data.
- **Uninstall:** remove OpenNav registration and shortcuts. Then run the original
  OpenCPN shortcut and confirm charts, routes, tracks, waypoints, connections and
  plugins. OpenCPN may show its normal safety notice again because the build
  version changed; read it and choose Agree to continue. Original executable
  hashes must remain unchanged.

Alpha retains modified/custom files, unpublished staging and logs under
`%LOCALAPPDATA%\OpenNavXAlpha1` after uninstall. Once you have verified original
OpenCPN and saved any diagnostics, you may delete this OpenNav-only folder.
Never delete the normal OpenCPN profile as part of uninstalling OpenNav.

## 10. Report a problem

Open XNav **System → diagnostics** and the diagnostics/log folder. Portable logs
are in `logs/` and `profile/opencpn.log`; installed XNav diagnostic files are in
`opennav-logs/` inside the normal OpenCPN profile. Installer maintenance logs and
its Diagnostics report are in `%LOCALAPPDATA%\OpenNavXAlpha1\logs`.

Send the Alpha commit/build information, Windows version, DPI/GPU, exact steps,
expected/actual result and screenshots. Include the relevant diagnostic JSON and
short log interval. Review logs before sharing: normal OpenCPN logs/configuration
may contain vessel positions, chart paths or connection details. Avoid sending
credentials, proprietary charts or your complete voyage database by default.
