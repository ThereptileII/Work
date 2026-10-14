# Authenticated discovery and bounded code recovery — 2026-10-14

This increment joins signed successor discovery, exact complete-pair DEMAND
recovery and verified-original terminal abort. Customer Setup/Maintain activation
and actual installed-service/desktop/boat acceptance remain separate and open.

## Passing native retest

Exact source `3039528767aa71b801a63f258e4fb3d829e21e6e` passes
[run 38621115904](https://github.com/ThereptileII/Work/actions/runs/38621115904):
**366 Windows x86 / 255 Linux top-level passes**, zero failures. Five existing
explicit Windows skips remain; every one of the **113 Windows / 82 Linux**
mandatory cases ran exactly once and passed without failed/skipped subcases.
Original provider archive hashes, lengths, ZIP CRCs, source/run/attempt and JSON
stream digests were checked. Receipts are retained beside this record.

The corrected empty demand-attempt case passes, including actual descriptor,
zero-length and preserved original identity checks. Native cache identity and
missing-custody cases pass alongside the existing installed completion producer.
Source-closure suites pass 12 broker and 14 provisioner cases. These are native
component results, not an installed service/customer journey or release approval.

## Preserved first native failure

[Run 38619809617](https://github.com/ThereptileII/Work/actions/runs/38619809617),
source `ae5e5e6644494ccf411c262b301a480b99531655`, passed all **253 Linux**
top-level cases. Windows x86 passed **361**, failed **one**, and recorded five
existing explicit skips. The new demand-attempt empty-file fixture failed its
own exact security-descriptor setup before reaching the production reader.
Setting the descriptor after ordinary file creation retained different
inheritance-control flags. The correction uses the existing native CREATE_NEW
helper with the exact descriptor at birth, then flushes and checks the same
actual handle. Zero-length, exact-descriptor and unchanged-native-identity
assertions remain; production acceptance is unchanged.

Original provider archive hashes, lengths, ZIP CRCs and JSON test-stream hashes
were verified. The original failed artifacts remain private; their sanitized
receipt is [first-failure-receipt.json](first-failure-receipt.json). This failed
run does not qualify the increment; the exact-source retest above is separate.

## Original installed cache binding

New enrollment also preserves the original protected TUF cache path and full
native directory identity in schema-2 installed completion. Schema-1 canonical
bytes and ordinary maintenance remain compatible. Missing original cache custody
cannot produce a schema-2 completion; cache reopening cannot infer an old cache
from its current permissions or reset rollback floors. Existing schema-1
automatic discovery requires a separately proven migration and remains unavailable.
See [contract](../../installer/installed-bootstrap-cache.md).

Independent review checked version compatibility, borrowed-owner lifetime,
failure ordering and native identity/ancestry. Portable and native component
checks pass; complete customer cache discovery and service handoff remain open.

## Scope

No full application build, installed service transition, customer UAC journey,
boat installation, physical command or public release is accepted by this record.
The independently running SDK refresh and older frozen application candidate
have separate evidence and cannot qualify these newer source changes.
