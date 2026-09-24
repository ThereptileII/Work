# Alpha 1 known limitations

**Not approved for navigation or production use.** This is an integrated Alpha
for manual evaluation. CI simulations and hosted desktop tests do not establish
boat, at-sea or target navigation-PC acceptance.

- Windows x64 host, supported OpenCPN 5.12.4 **x86/Win32 application/plugin ABI**.
  The `win64` package name does not indicate a new 64-bit plugin ABI.
- Only the exact accepted stock executable hash is supported by Setup. The
  per-user integration runs beside the original application with its normal
  profile. It does not replace Program Files binaries. Other versions/ABIs are
  refused. The portable distribution uses a separate profile.
- Alpha is unsigned. App-local MSVC runtime and required DLLs are included.
  Optional OpenCPN embedded web features may need Microsoft's Edge WebView2
  Runtime; core XNav pages do not depend on web content.
- Marine input integration reuses OpenCPN NMEA 0183, supported NMEA 2000 and
  own-vessel Signal K messages. Physical instrument/ESP32/N2K validation remains
  open. Unsupported gear/regeneration/device fields stay unavailable.
- The pinned N2K battery-voltage decoder cannot represent a 343 V pack on its
  signed 0.01 V path. Use a documented appropriate source such as Signal K;
  Alpha does not silently reinterpret the field. Unsupported motor-temperature
  or power paths need explicit documented mappings. No Nissan Leaf CAN IDs are
  built into the desktop application.
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
- Live autopilot output is disabled. Only the manual simulator is enabled for
  Alpha testing. Physical feedback, command permissions and at-sea TRACK/WIND
  behavior require separate commissioning. There is no autonomous steering.
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
  a production updater. Back up the normal OpenCPN profile before Alpha tests.
- Alpha uninstall removes registration, shortcuts and verified publisher-owned
  files. Modified/custom files, unpublished staging and logs remain. Delete that OpenNav-only folder manually after
  verifying original OpenCPN. Shared navigation data is never rolled back.

Physical checklists are in `physical-validation.md`. Do not treat a checklist as
accepted until the exact hardware/software path has actually been tested.
