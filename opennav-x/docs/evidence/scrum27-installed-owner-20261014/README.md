# Same-owner installed cleanup: focused qualification

SCRUM-27, 2026-10-14. Exact source
`b93da59cf0749157dfca84c562cc8c39713ece68` passes
[run 38516605460](https://github.com/ThereptileII/Work/actions/runs/38516605460),
attempt 1, on Linux and native Windows Server 2022 with GOARCH=386.

| Platform | Top-level passed | Nested passed | Failed | Skipped |
|---|---:|---:|---:|---:|
| Linux | 117 | 347 | 0 | 0 |
| Windows | 171 | 448 | 0 | 3 |

The three Windows skips are pre-existing non-Windows API-boundary tests, listed
in `results.json`. All 30 specifically required new/changed Windows controls
actually pass; all 11 portable controls actually pass. Original artifact lengths,
provider SHA-256 digests and every ZIP CRC match downloaded bytes. Broker-source
closure passes on both platforms. The complete required-name set and original
artifact identities are preserved in `results.json`.

## What ran

- Closed completed baseline/update/rollback generation lineage retirement,
  original root identities, callback failure/cancellation, collision and unknown
  child preservation. The metadata-only pre-drain fixture now checks the exact
  journal through its original held writer without relaxing exclusive ownership.
- Real in-memory native ShellLink decoding, exact stock/integration fields,
  malformed/oversized/truncated input refusal and original startup retirement.
- Manifest-derived installed inventory, mutable-state refusal, same-owner
  preparation/final witness/SCM cleanup ordering and sticky failure handling.
- Real native file deletion disposition, original handle identity, cancellation,
  replacement refusal and non-replacing state retirement with recovery bytes.
- Real temporary private HKU registry transactions: deletion is invisible outside
  the transaction before commit; close/abort restores original contents;
  conflicts preserve completed outside writes; successful commit removes only
  the original key; unknown values/subkeys, cancellation, unsupported exports,
  timeout and insufficient rights refuse without a plain-delete fallback.
- CLI input binding, single original owner and broker conversation, suppressed
  success on partial result, close failure or cancellation. The artifact path is
  an approved source input and cannot choose a cleanup destination.

Separate local integration passed 33 top-level/73 nested Linux cases and Win32
cross-compilation of enrollment/provisioner tests. The isolated author passed
Linux race checks. All 69 focused source-package tests passed after requiring
13 previously omitted production sources. Independent read-only review corrected
registry enumeration EOF handling and active-state replacement detection before
integration; no remaining concrete review finding was reported.

## Limits

This is component qualification. It does not prove an actual installed customer
menu, complete approved baseline/update/rollback/uninstall journey, stock profile
preservation across that installed journey, Maintain UI integration, a newly
packaged application, or boat acceptance. The old Lifecycle guard remains;
SCRUM-27 stays open. No full application build, boat action or physical command
was performed. Source/runtime contract: [installed terminal cleanup](../../installer/installed-terminal-cleanup.md).

Original failures are retained under
[closed-lineage evidence](../scrum27-lineage-uninstall-20261014/README.md) and
[two-phase evidence](../scrum27-two-phase-uninstall-20261014/README.md).
