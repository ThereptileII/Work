# SCRUM-98 retained native evidence — run 38160318032

[Run 38160318032](https://github.com/ThereptileII/Work/actions/runs/38160318032)
and [job 114531665650](https://github.com/ThereptileII/Work/actions/runs/38160318032/job/114531665650)
completed successfully on 2026-10-11. The actual final receipt names exactly
two passing groups, `installer` and `charts`, under scope `installer-charts`.
It explicitly records `publicAccess: false` and `releaseQualification: false`.

The helper checkout is `d8bd6c86cd8d6f93d49f8696fc3ef8c3110a540c`.
The frozen Win32 product is `ec51b9261d2854d8600672ea27c97af5abeb3f6d`,
restored from successful producer run `38143022102`, attempt 1,
`windows-integration`, with 4,540 compiled-input files. This was a retained-input
retest, not a new product build. Source and input identities are retained in the
raw staging receipts; the preparation receipt's `qualification: not-run` is its
pre-test state, superseded for the two selected groups by `staging-retest.json`.

## Verified artifact

Artifact `11710123312` is 15,048,995 bytes. Its downloaded ZIP SHA-256 is
`e293ee7524d777cc5c10a61857b5283a5097249da4c6608dbc0516305c55f93e`,
exactly matching the GitHub API digest. All 1,985 ZIP-member CRCs passed.
The producer artifact's API identity/digest also matches the restoration receipt;
that older input archive was not independently downloaded again for this audit.
The raw compiled-input receipt hash matches the installer report's recorded hash.

[`artifact-verification.json`](artifact-verification.json) records verification,
byte-exact retained member hashes and exclusions. [`summary.json`](summary.json)
records the exact named checks, counts, identities and limits. `raw/` contains
compact API receipts, unmodified result/operation reports and selected textual
log evidence. The full ZIP and full job log remain in the private download
directory recorded in the verification receipt. No binaries, chart data,
screenshots, profiles or build trees are committed here.

## Measured passing scope

| Group | Actual report evidence |
| --- | --- |
| Installer | Nine named checks: exact supported stock verification/discovery; ordinary wizard/default path/Cancel; foreign shortcuts, unknown ownership and unknown executable refusal; both unmanaged legacy TLS DLL refusals; actual Beta 2 wizard completion; clean install and first-install rollback preserving stock/profile and rendering coastline. Native Windows / Windows PowerShell 5.1 / NSIS authority. |
| Lifecycle operation reports | Seven recorded operations: passed preflight and rollback, plus five expected failed Install reports for foreign shortcut ownership, unknown root ownership, unsupported executable, `libeay32.dll` and `ssleay32.dll`. These are refusal proofs, not five successful installations. |
| Packaged updater within installer | Four named checks: real launcher/bootstrap authenticated receipt; healthy same-version supervised update finalized after one startup attempt; corrupt candidate refused before process creation and guarded rollback; restored generation authenticated and closed cleanly. |
| Charts | Two requested phases, `software` and `opengl`, with 10 and 8 functional capture metrics respectively. Both loaded the two public NOAA ENC cells, exercised zoom/follow, switched to the adjacent cell and returned, and retained workspace state across XNav/legacy/return transitions. The software phase also records native three-point route creation/edit persistence and opening the native plugin manager. |

The unsupported-executable preservation report verifies unchanged hashes for
the two-file rejected tree and four-file existing profile, with installation
root and all shortcut groups absent. Updater fault injection changed only the
disposable installed candidate's PE header; the report records rejection before
launch and restoration of the verified previous generation without profile
changes. The updater's explicit scope excludes release selection and TUF
acceptance.

## Limits retained from the reports

OpenGL was requested, but the host rejected it and the report verifies software
fallback. Runtime cell-switch observations have `opengl_enabled: false` in both
phases. The report explicitly leaves native screenshot review and physical GPU
qualification separate. No screenshot or design review was performed for this
evidence task; installer `design_validation` is `not requested`.

Chart performance observations cover two ten-second windows on a CI desktop,
two NOAA ENCs, three plugins and synthetic 3.3 Hz RMC input. They exclude
asynchronous chart painting and do not qualify the target navigation PC. The
workspace fixture is explicitly labelled `SIMULATED workspace persistence`.
The private o-charts adapter is unavailable in these chart runtime observations.

The official prerequisite report records exit code `1223`, despite visible
completion and verified executable hash, resources and registration. Its exact
recorded anomaly says Alpha lifecycle still requires zero. This receipt preserves
that fact and does not reinterpret the official exit as zero.

Other AIS/test/build outputs in the artifact are inherited retained material;
they are not additional checks executed by this selected retest. This evidence
does not establish boat operation, live AIS, private charts, physical GPU,
public access, deployment, release acceptance or overall SCRUM-98 completion.
No rerun, build, deployment, boat action or Jira mutation was performed here.
