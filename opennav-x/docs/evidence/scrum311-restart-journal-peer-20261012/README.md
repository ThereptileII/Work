# SCRUM-311 restart, journal and peer evidence — 2026-10-12

This is component qualification, not an installed updater or release acceptance.

## Exact sources and checks

- Joined generation reopen and permanent execution journal source: `43a9cdb6`.
  Linux race: **18 top-level + 107 nested passed**, zero failures/skips.
  Windows/x86 component executable SHA-256:
  `3a9609c0af441f027421e9573b6329e8f5bfd0d4b88ba59334bc76feda920014`.
  Native generation-reopen: **3 top-level + 14 nested passed**.
  Native execution-journal: **3 top-level + 4 nested passed**.
- Peer executable uses isolated source `42235684b4c9cfe6bb595f1ae44605185baa74bf`,
  integrated as `2689d889`; unchanged executable SHA-256:
  `db384667b7403d40583e6f5f783c3121dd90d7ad4682fdf0c8fb7339148f84ed`.
  The initial SSH/session-0 invocation failed the existing interactive-client
  prerequisite. Its original logs are retained. The SAME binary then passed
  **2 top-level + 7 nested** checks in existing desktop session 1 through the
  limited-current-user dispatcher `c14ae5a4`. The temporary task was removed.
- Actual staging host peer custody and least-rights C++ health client source:
  `204de84b`. The Windows/x86 Go host component compiled; the C++ Windows branch
  and complete sender/receiver integration require native MSVC qualification.
- Isolated generation/journal initial fixture failures and corrected results are
  retained. Their overlapping counts must not be added to the joined totals.

## Scope and limitations

The joined native fixtures exercise original protected generation reopening,
full physical identity and byte checks, readonly loader-compatible custody,
permanent journal reservations, append/flush/readback, mutation/cancellation and
partial-creation refusal. Their disposable current-user factories do not
qualify an actual SCM broker, installed transaction or service permissions.

Peer checks verify actual native process/token/logon identity and disconnect
revocation. They do not grant selection, commissioning, spawn or health authority.

No installed application/profile/service or hardware was changed. OpenCPN
PID1852 remained responsive; no owned fixture process remained. No application
full build, design review or endurance run occurred.

Actual protected selected/pending state, interactive execution, general-profile
commissioning, consumed spawn, authenticated healthy commit and restoration/
fallback remain unfinished. SCRUM-311 remains In Progress. Subsequent execution
transport source and its separate checks do not retroactively change the source
revision of these retained binaries.

`observed-native-counts.json` indexes the original native case results. All
copied JSON/JSONL/log bytes remain unchanged. `SHA256SUMS.txt` covers this record
and every retained file except itself.
