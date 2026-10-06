# Setup profile preservation (SCRUM-28 / SCRUM-29)

The existing explicit InspectRestore → preservation proposal → reviewed Restore
flow now accepts the following narrow profile differences. It still records
`current-user-state;origin-unverified`, requires a review entry for every exact
before/after byte value, and grants no launch permission. Connection, plugin,
source-priority and navigation protections are unchanged.

| Profile key | Source contract |
| --- | --- |
| `/OpenNav/BoatSetupV1` | `application/BoatSetup.cpp`: only `v1\|pending`, `v1\|complete`, `v1\|existing`. Unknown records and deletion are refused. |
| `/OpenNav/DisplayPreferencesV1` | `application/DisplayPreferences.h`: version 1, scales 100/125/150, layouts balanced/chart/instruments. |
| `/OpenNav/VesselName` | `integration/SettingsStore.cpp::ValidName`: empty or up to 128 UTF-8 bytes, not whitespace-only, no C0/DEL controls. Canonical wxFileConfig backslash/quote encoding is checked; unusual unsupported encodings are refused. |
| `/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR` | `SettingsStore::SaveVessel`, `SaveBoatSetup`, `RestoreBackup`: finite 0–1,000,000 metres; never NaN/infinity, units text or decimal comma. |

For an existing `OpenNavXSettings 1` record containing pilot, source or calibration
configuration, `Assert-PreservedSetupSettingsDelta` compares the entire raw record
before and after while masking only validated draft (0–100 m), capacity
(0.001–100,000 kWh) and reserve (0–100%) fields. Empty means unconfigured, as in
`application/Settings.cpp`. Source mappings, calibration, device identities and
all other bytes must remain identical. The two exact provenance strings written
by the current Vessel UI are admitted: empty and
`User-configured usable battery energy and reserve / OpenCPN profile`.

The sole additional pilot delta is `manual` → `display-only`, matching
`SettingsStore::RestoreBackup`'s revocation of permission while preserving the
local binding. Granting permission, replacing identity, introducing a new pilot
field, duplicate fields, and changing source/calibration data are refused. The
older explicit base-model preservation policy remains available; its supported
fourteen fields are not expanded to arbitrary settings records.

A backup import that actually changes a source mapping, bridge identity,
calibration, or unsupported settings still requires a separate reviewed
preservation policy. Neither this change nor a successful backup import is
permission to load plugins, enable output or bypass fresh commissioning.

`test-setup-preservation.ps1 -PortableContracts` exercises accepted encodings,
Unicode byte bounds, malformed records, source/control refusals, complete
review entry requirements and the exact one-byte COM8 direction inverse.
`test-session-preservation.ps1 -PortableContracts` provides the existing
regression checks. The native `test-commissioning.ps1 -PreservationFixture`
fixture now includes all four new keys in its full preparation/restoration and
interruption paths. Native execution remains the coordinator's Windows gate.
