# Authenticated discovery and bounded code recovery — 2026-10-14

This increment joins signed successor discovery, exact complete-pair DEMAND
recovery and verified-original terminal abort. Customer Setup/Maintain activation
and actual installed-service/desktop/boat acceptance remain separate and open.

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
receipt is [first-failure-receipt.json](first-failure-receipt.json). Native retest
is required and this failed run does not qualify the increment.

## Original installed cache binding

New enrollment also preserves the original protected TUF cache path and full
native directory identity in schema-2 installed completion. Schema-1 canonical
bytes and ordinary maintenance remain compatible. Missing original cache custody
cannot produce a schema-2 completion; cache reopening cannot infer an old cache
from its current permissions or reset rollback floors. Existing schema-1
automatic discovery requires a separately proven migration and remains unavailable.
See [contract](../../installer/installed-bootstrap-cache.md).

Independent review checked version compatibility, borrowed-owner lifetime,
failure ordering and native identity/ancestry. Portable focused checks and Windows
x86 compilation passed; new native component cases still require execution.

## Scope

No full application build, installed service transition, customer UAC journey,
boat installation, physical command or public release is accepted by this record.
The independently running SDK refresh and older frozen application candidate
have separate evidence and cannot qualify these newer source changes.
