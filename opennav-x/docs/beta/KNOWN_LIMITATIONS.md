# Beta 1 known limitations

**Not approved for navigation or production use.** This is an integrated Beta
for desktop and staged boat evaluation. CI simulations and hosted desktop tests do not establish
boat, at-sea or target navigation-PC acceptance.

- Windows x64 host, supported OpenCPN 5.12.4 **x86/Win32 application/plugin ABI**.
  The `win64` package name does not indicate a new 64-bit plugin ABI.
- Only the exact accepted stock executable hash is supported by Setup. The
  per-user integration runs beside the original application with its normal
  profile. It does not replace Program Files binaries. Other versions/ABIs are
  refused. The portable distribution uses a separate profile.
- Beta is unsigned. App-local MSVC runtime and required DLLs are included.
  Optional OpenCPN embedded web features may need Microsoft's Edge WebView2
  Runtime; core XNav pages do not depend on web content.
- Marine acquisition reuses OpenCPN selected navigation, NMEA 0183, supported
  NMEA 2000 and own-vessel Signal K. The software has loopback/decoder tests;
  physical sensors, gateway behavior, references and instances remain boat gates.
- Boat-specific mapping requires the exact interface and NMEA NAME. The reviewed
  ESP32-C6 producer v2 patch expires four sensor groups independently. It is
  compiled in CI but **not flashed or physically accepted**. Older v1 producer
  state remains uncertain; it cannot supply trusted advisory energy inputs.
- The high-voltage path uses PGN 127751; pinned 127508 cannot represent the whole
  343 V pack. SOC uses standard 127506 or the explicitly bound boat adapter,
  not an invented SOC field in 127508. Coolant-to-motor-temperature meaning and
  vendor regeneration state require explicit mapping. SOH remains unavailable
  when absent. No Leaf EV-CAN decoding runs in the desktop application.
- Signal K external JSON is bounded/validated before parsing and tested over the
  real WebSocket driver. The pinned upstream driver retains its existing TLS
  verification/plain-WebSocket fallback policy. Use a trusted boat network;
  this release does not provide secure remote Internet connectivity.
- Capacity, reserve, pack-current sign and propulsion calibration require real
  boat configuration. Energy assumes present/constant conditions and linear
  SOC-to-energy, with explicit curve domains/loss assumptions. It does not
  forecast weather, currents, battery temperature or aging. Estimates are
  withheld when required inputs are invalid/stale/missing.
- Demo telemetry, route progress and AIS cards are synthetic and clearly marked.
  They do not alter OpenCPN's live GPS/route/AIS state. The chart canvas remains
  real, so simulated route data need not match visible chart objects.
- SmartNav is advisory only. Turn/timeline/energy/AIS context reuse owned
  navigation contracts. The tested hazard-corridor abstraction has no accepted
  live ENC coverage provider; absence of a hazard report never means safe water.
- Live manual autopilot defaults OFF. The implemented ST4000 path is bound to
  exact interface/NAME over bidirectional TCP / Actisense complete-PGN ASCII.
  Serial, UDP and SeaSmart are status-only. Six manual commands require fresh
  physical feedback; sent is not confirmed. TRACK/WIND are unavailable. The
  complete PC → gateway → ESP32 → SeaTalk → ST4000 path still needs dockside
  and underway acceptance. SmartNav cannot command the adapter.
- Radar adapter capability/status architecture exists; no validated live radar
  display/control adapter is included. Existing plugin UIs may be used via
  Legacy. No automatic collision avoidance or radar/AIS fusion is implemented.
- Route/waypoint basic workflows use OpenCPN storage and guards. Advanced route,
  track, chart, connection and plugin settings still use preserved Legacy UI.
  Some protected/active/shared-object edits are deliberately unavailable in XNav.
- Anchor Mode exposes normal OpenCPN anchor-watch state/history. It is not an
  intelligent drag prediction or a substitute for a physical anchor watch.
- Bundled global coastline is an overview, not a nautical chart. No proprietary
  chart material is included. CI uses a hash-pinned public NOAA ENC fixture that
  is not redistributed as a navigation chart in these packages.
- Hosted 100/125/150% DPI and synthetic touch tests supplement mouse tests;
  physical touch, sunlight readability, target GPU/driver OpenGL, large chart
  catalogs and broader third-party plugin combinations require manual testing.
- Use short writable extraction/install paths. Upstream cache filenames can be
  sensitive to long resource paths. Representative bundled Dashboard/WMM/GRIB
  compatibility does not guarantee every plugin works in XNav; Legacy remains
  available for plugin UI compatibility.
- Startup crash-loop protection covers unfinished XNav starts. It does not
  claim universal crash recovery, lossless filesystem power-failure recovery or
  a production updater. Back up the normal OpenCPN profile before Beta tests. No power-loss atomicity
  guarantee is claimed for the filesystem or OpenCPN voyage database.
- Beta uninstall removes registration, shortcuts and verified publisher-owned
  files. Modified/custom files, unpublished staging and logs remain. Delete that OpenNav-only folder manually after
  verifying original OpenCPN. Shared navigation data is never rolled back.

Physical checklists are in `physical-validation.md`. Do not treat a checklist as
accepted until the exact hardware/software path has actually been tested.

Beta keeps Alpha's `%LOCALAPPDATA%\OpenNavXAlpha1`, registry identity and
**OpenNav X Alpha 1** Start-menu folder to preserve update/rollback continuity.
Native file pickers, installer and preserved Legacy/plugin dialogs can remain
bright in Night mode; primary XNav pages are the styled night workflow.
Recordings retain at most three bounded segments; user-selected exports are
not silently deleted. Diagnostic bundles include only the documented whitelist.
Three-hour CI simulations qualify only their measured hosted environment, not
physical touch, arbitrary plugins, all chart catalogs or target-PC endurance.

## Post-build review of a3e6e08

These repository notes follow qualification and do not modify the already
hashed downloadable package or its bundled documents.

- Diagnostics retains an **ALPHA 1 / NOT FOR NAVIGATION** caption above the
  correct Beta 1 version/commit/compiler fields. The caption is cosmetic.
- At 150% the transient System popup partly covers the Alerts button. Critical
  alert text remains visible. Dismiss with Escape or an outside click before
  opening Alerts. Placement refinement is deferred.
- Rails and long pages require Up/Down or pan at larger DPI/limited height.
  With an alert at 100%, the bottom heading card may already need scrolling.
  Use rail configuration to prioritize speed or other instruments.

The authoritative review and physical exclusions are in
[current status](../status.md) and [exact-release evidence](../evidence/beta1-a3e6e08-accepted.json).
