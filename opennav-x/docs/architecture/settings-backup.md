# SKAGER settings backup and restore

Settings → System exposes **Export settings backup** and **Import settings
backup**. Export writes a `.skager-backup` file through a temporary file and
commit, with an overwrite prompt. Import reads at most 128 KiB plus one byte,
validates the whole record, then shows the vessel, energy, display, source and
calibration summary. Cancel changes nothing; Restore settings is the explicit
commit action. Export a backup first when retaining the previous setup is needed.

The version 1 envelope (`SKAGER_SETTINGS_BACKUP 1`) contains exactly four named,
quoted fields: `vessel`, `settings`, `display`, `chart_safety_depth_m`. Its nested
settings and display records are the existing independently versioned formats.
The allowlist includes vessel name, draft/advisory settings, energy model and
calibration, source priorities/freshness, Signal K scaling/offset mappings,
passive boat-bridge binding, instrument selection, interface scale/layout and
chart safety depth. A missing chart safety depth preserves the host value.

Pilot bindings and control permissions are excluded from export and rejected
on import. Restore retains this computer's pilot identity but sets permission
and the runtime gate OFF. It never issues an actuator command. Credentials,
chart files/licenses, routes/tracks/waypoints, connection definitions, plugins,
interface mode and other OpenCPN files/settings are neither collected nor
restored. This is configuration transfer, not a whole-profile or chart backup.
The shared host chart safety contour is the one explicit host preference in
this format, matching the existing SKAGER Vessel editor.

Unknown, duplicate or missing envelope fields, unsupported versions, invalid
numbers/UTF-8, malformed or oversized records and unsupported nested fields are
rejected before any mutation. Version 1 is the first production backup format;
prototype JSON and arbitrary OpenCPN settings files are incompatible. The only
existing nested migration is the documented old untouched instrument-rail
default in `DecodeSettings`; custom selections remain intact. Future envelope
versions must add an explicit, tested compatibility/migration path.

`SettingsStore::RestoreBackup` validates again, stages all serialized entries,
snapshots their original values and existence, then writes/flushes the bounded
set. Any write/flush failure attempts to restore every original entry; live
state changes only after a successful flush. Rollback failure is reported
explicitly. No crash-atomic transaction across a whole OpenCPN profile is
claimed. Restore is unavailable during replay or restart. Source configuration
and chart presentation are refreshed after commit; no imported configuration
becomes a sensor observation or establishes device identity/freshness.

Focused tests: `settings_backup_contract` covers roundtrip, unconfigured values,
control exclusion/rejection, strict bounds/schema, UTF-8 and nested migration.
`settings_backup_profile_transaction` covers validation without writes,
rollback of existing/missing entries, unrelated values, local control disable,
and reopening the persisted profile. Changed native UI sources also require
compilation. Native Windows dialog selection/cancel/preview/confirm,
permissions/write failure and post-restore display/source behavior remain
integration/release gates; Linux checks do not qualify them.
