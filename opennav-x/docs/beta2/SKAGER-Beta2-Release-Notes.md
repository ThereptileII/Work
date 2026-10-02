# SKAGER Beta 2 — release notes

**Version 0.4.0-beta2 · evaluation candidate · release acceptance pending**

SKAGER adds a navigation workspace to the supported OpenCPN installation,
with access to its familiar Legacy interface and a Safe Mode recovery path.
Beta 2 is not approved for navigation or production use. A package or successful
build is not proof that its Windows, boat-display or release checks have passed.
Public release remains disabled until the exact candidate is accepted.

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
  1280×800 layout and additional 1920×1080 layout require native and boat review.
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

- SKAGER equipment output remains status-only. SmartNav is advisory and does
  not steer. No physical actuator operation is authorized by these notes.
- Internet AIS is optional, off by default, and may be delayed or incomplete.
  It supplements onboard observations and does not replace an AIS receiver or
  keeping watch. Keep account keys out of screenshots and support reports.
- Custom radar integration, chart-corridor hazard coverage and the guided
  first-start commissioning flow remain incomplete. Missing warnings never
  establish safe water.
- Native typography, layout, 100/125/150% Windows DPI, physical touch and the
  actual boat GPU/display require their own evidence. Component checks and
  injected touch are not physical-device acceptance; smaller responsive layouts
  are not generally qualified.
- Setup is unsigned. Verify the complete download against `SHA256SUMS.txt` and
  use only the exact package supplied for the authorized review.

Read [KNOWN_LIMITATIONS.md](KNOWN_LIMITATIONS.md) in the recovery package for
additional boundaries. Its `docs/PRODUCT_BUILD.json` and `docs/BUILD_INFO.md`
identify the exact executable commit and build. The corresponding-source ZIP
contains the same notes and source identity; the installer carries the same
notes in its generation documentation. The outer `QUALIFICATION.txt` describes
build-time candidate status. A pending-endurance development artifact remains
pending; later acceptance requires a separate exact-commit review record.
