# Closed-lineage and original-startup retirement — 2026-10-14

SCRUM-27. This is component evidence, not full installed uninstall, maintenance
UI, package or boat acceptance.

## Original native observation

Source `b64e12f37509ab7de87ba36af93e22d0e86a6d74`,
[run 38514939833](https://github.com/ThereptileII/Work/actions/runs/38514939833),
attempt 1, remains **failed**. Linux passes 108 top-level and 324 nested cases.
Windows passes 149 top-level and 397 nested cases, with three failed top-level
cases/seven failed nested cases and three existing non-Windows-boundary skips.
Both original evidence ZIPs are downloaded; provider SHA-256, byte length and
all ZIP CRCs match. Exact metadata is in `original-failure.json`.

The first errors identify three fixture problems:

- `TestNativeStartupInstalledRetirementPreservesOriginalShortcutCustody` tries
  `os.ReadFile` while production retains original DELETE custody. Windows refuses
  that incompatible second open. The corrected assertion reads the same retained
  owner first, then verifies bytes at the fixed path after custody closes.
- `TestNativeRetirementCompletedLineageKeepsOriginalRootsAcrossDrainAndJournal`
  similarly tries an incompatible second open of the still-held started-record
  writer. Its readback must explicitly grant the original writer's sharing.
  Callback failures must return, rather than call `t.Fatal`, so returned partial
  native custody reaches its deterministic close path.
- `TestNativeStartupLinkFieldsRefusesTruncatedGetterResults` tries to generate an
  overlong path through Windows ShellLink; `SetPath` rejects it before the reader
  is exercised. The fixture must construct supported input that actually reaches
  the intended truncated-getter refusal.

Do not remove exclusive original handle ownership or weaken malformed/truncated
field rejection to make these checks green. No application build, installed
maintenance action or boat operation was performed. Corrected native execution
is required before this component is accepted.
