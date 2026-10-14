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
  writer. The second run below disproved that adding sharing to a new reader
  could make this exclusive original writer readable.
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

## Second native observation

Source `3eecfdeb17a5cb2fba4d56cfbc2d844e123fb777`,
[run 38515423088](https://github.com/ThereptileII/Work/actions/runs/38515423088),
attempt 1, remains **failed**. Linux passes 108 top-level / 324 nested cases.
Windows passes 151 top-level / 401 nested cases, fails one top-level / three
nested cases, and retains the three existing non-Windows-boundary skips.
Original artifact length, provider SHA-256 and every ZIP CRC match; metadata and
failed names are preserved in `second-failure.json`.

The original-startup retirement and bounded ShellLink reader corrections pass.
The lineage fixture still opens the durable journal for data reading, but its
production writer intentionally uses `share=0`. Granting every sharing flag to
the second reader does not override that restriction. The lost/cancelled
branches could also mistake this earlier failure for their intended fault.

The next fixture revision observes only metadata before drain (bounded size,
ordinary single-link file and full identity), closes that observer, and reads
bytes through the **same original writer** after commit returns. It verifies the
pre-drain identity/size, exact schema and ordered lineage on every branch. Named
faults must prove their own branch was reached and retain deadline/cancellation
identity. Existing direct-owner native tests already check durable journal bytes
inside the callback. No production lock, sharing, runtime hook or safety
assertion is relaxed. Native evidence for this revision remains pending.
