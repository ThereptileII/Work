# SKAGER Beta 2 — release notes

**Version 0.4.0-beta2.1 · Staging candidate · release acceptance pending**

## This Staging increment

- Guided boat setup is available from Preferences → Vessel. Fresh installed
  profiles can complete it on first start; existing installations are preserved.
  Later leaves the draft unsaved. Sensor checks use live observations only.
- Preferences → System can export and restore SKAGER settings. Backups exclude
  charts, routes, connection credentials and autopilot permissions. Restoring
  settings switches pilot control OFF and requires source/calibration review.
- The original OpenCPN navigation warning remains a human decision. The startup
  updater now waits for it within a finite deadline without declaring the
  application healthy before normal initialization finishes.
- The manual pilot path uses the selected OpenCPN Actisense serial connection.
  Control remains OFF until explicitly configured and enabled for the session;
  exact device identity and fresh measured feedback are required. Sending a
  message alone does not confirm a command. TRACK and WIND remain unavailable.
- Private signed-update repository tooling is included in the corresponding
  source. Live signed boat update acceptance remains separate from package
  startup and rollback checks; consult the exact release qualification record.

SKAGER adds a navigation workspace to the supported OpenCPN installation,
with access to its familiar Legacy interface and a Safe Mode recovery path.
Beta 2 is not approved for navigation or production use. A package or successful
build is not proof that its Windows, boat-display or release checks have passed.
Public release remains disabled until the exact candidate is accepted and the
user separately authorizes public opening. Standard delivery is a versioned
GitHub Release, kept in draft while access is closed. Development defaults to
STAGING; Production promotion requires explicit user instruction and preserves
the selected package bytes and embedded version.

## October boat-feedback staging increment

This increment carries the reported boat fixes and requested small workflow
upgrades. Its exact commit and qualification status accompany the release.

- AIS lists support wheel and touch scrolling. Online AIS adds a saved **1–200
  nm radius around the chart center** with an explicit Apply action. Source
  diagnostics now distinguish incoming messages, subscription progress, cached
  traffic and traffic inside that radius. A connected service can legitimately
  have no traffic in an area; the reported missing-target case still requires
  same-area application confirmation.
- Routes and waypoints use focused inline name editing, and new marks can suggest
  a nearby charted name when unambiguous. Existing names are preserved. Route
  selection opens a compact context card; chart information retains the complete
  underlying object details in an expandable SKAGER view.
- Active-route cross-track error comes from OpenCPN's normal navigation update.
  Anchor distance follows the current valid position. Switching between route
  navigation and anchor watch is explicit and preserves saved marks.
- Supported autopilot status can be discovered passively through existing
  OpenCPN connections without configuring a second connection. This does not
  enable equipment control or turn transmitted commands into confirmed state.
- Reported scale-legend, inactive-route, dimension-scaled ownship, light-sector,
  classified lighthouse-tower and anchor-watch presentation gaps are addressed.
  Navigational geometry, alarm meaning and custom symbols are preserved.

Follow the short boat-feedback section in the test guide. Physical touch and
actual chart/data results must be recorded separately from automated tests.

## Changes in this candidate

- A chart workspace with route/passage, traffic, instruments, alerts and source
  health views. Missing or stale input stays visibly unavailable; the installed
  product contains no demonstration vessel-data source.
- Vessel preferences for display name, draft, chart safety depth, usable battery
  capacity and reserve. Unconfigured values remain unconfigured. Chart safety
  depth is separate from the advisory hazard margin.
- Display preferences with 100%, 125% and 150% interface scale, Balanced,
  Chart focus and Instrument focus layouts, and an explicit Apply action.
  Interface scale is separate from Windows display scaling. The primary
  1280×800 layout and additional 1920×1080 layout retain native/boat evidence
  requirements when design review is explicitly requested. Functional usability
  remains a relevant qualification requirement.
- A minimum 48-DIP single-line waypoint/route editor field at the default
  interface scale, with larger fields at the two higher interface scales.
- Maintained OpenSSL, curl and zlib dependencies, verified HTTPS downloads and
  rejection of failed plugin transfers before archive installation. Exact
  dependency, native trust and package test results remain qualification gates.
- Peer-to-peer navigation-object sharing is unavailable in this product
  candidate. Existing stored peer credentials are retained; no sharing setup
  or acceptance is implied.

## Installation and recovery

Use the supplied [installation guide](SKAGER-Beta2-Install-Guide.md) and
[test guide](SKAGER-Beta2-Test-Guide.md). The supported application uses the
OpenCPN 5.12.4 Win32/x86 application and plugin ABI on Windows 10/11 x64.
Setup checks the exact supported OpenCPN executable; do not bypass a refusal.
Back up the real profile and separately stored charts before an authorized
installation or update, and close all application modes first.

Installed modes share the existing OpenCPN navigation profile. Repair and
rollback operate on owned application files; they are not a replacement for
navigation-data backups. The portable recovery ZIP has its own separate profile
and does not automatically load the normal charts or connections. New download
filenames use the `SKAGER-Beta2` prefix. Older accepted downloads retain their
original names and hashes; they are never relabeled or repackaged.

## Limitations to review

- Manual pilot commissioning supports STANDBY, AUTO and ±1°/±10° only after
  explicit configuration, session enablement and fresh device feedback. Default
  control is OFF. SmartNav cannot steer; propulsion, radar transmission and
  switching output are unavailable. Package notes do not authorize physical tests.
- Internet AIS is optional, off by default, and may be delayed or incomplete.
  It supplements onboard observations and does not replace an AIS receiver or
  keeping watch. Keep account keys out of screenshots and support reports.
- Custom radar integration and chart-corridor hazard coverage remain incomplete.
  Guided setup is not vessel commissioning or navigation approval. Missing
  warnings never establish safe water.
- Native typography, layout, 100/125/150% Windows DPI, physical touch and the
  actual boat GPU/display require their own evidence within the applicable review
  scope. Design-only validation is scheduled only on explicit user request;
  functional rendering, control usability and data validity remain required.
  Component checks and injected touch are not physical-device acceptance;
  smaller responsive layouts
  are not generally qualified.
- Setup is unsigned. Verify the complete download against `SHA256SUMS.txt` and
  use only the exact package supplied for the authorized review.

Read [KNOWN_LIMITATIONS.md](KNOWN_LIMITATIONS.md) in the recovery package for
additional boundaries. Its `docs/PRODUCT_BUILD.json` and `docs/BUILD_INFO.md`
identify the exact executable commit and build. The corresponding-source ZIP
contains the same notes and source identity; the installer carries the same
notes in its generation documentation. The outer `QUALIFICATION.txt` describes
build-time candidate status. Preserve that status with the exact artifact;
subsequent qualification belongs in its release record. Endurance testing remains
explicitly skipped until the user changes that instruction; disclose the gap and
never report it as passed. Channel promotion does not rewrite package evidence.
