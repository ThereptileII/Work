# Broker code upgrade — focused native evidence

This records operator, publisher and native file/reader checks, not a full
application build, installed SCM/customer journey, boat test or public release.
The [contract](../../installer/broker-code-upgrade.md) retains its explicit
one-successor and recovery-phase limits; normal Setup remains unwired.

## Corrected focused gate passes

Exact source `eceef0843df49d8a930623cf4edaf83406eabec7`,
[run 38597858395](https://github.com/ThereptileII/Work/actions/runs/38597858395),
passes **331 Windows x86 / 232 Linux amd64** top-level cases, with zero failures.
The five existing Windows platform/opt-in skips are not acceptance. All
**78 Windows / 59 Linux mandatory cases** actually ran and passed, with no
skipped or failed required subcases. Source closure checks also passed.

Fixture fix `3449a203` releases the refusing probe's DELETE owner before the
ordinary reread. It additionally requires the foreign file's exact original full
native binding, while preserving both refusal assertions and byte equality.
Production sharing/custody behavior is unchanged. The required-case map now
matches the publisher's pre-existing Linux-only platform contract; all prior
native Windows requirements remain mandatory.

The original authenticated ZIP downloads match GitHub size/digest and ZIP CRCs;
their `tests.json` hashes match the embedded exact-source receipts.
[Windows receipt](windows-receipt.json) and [Linux receipt](linux-receipt.json)
record artifact IDs, counts, sizes and hashes. This gate qualifies the selected
native primitives and readers only. Actual SCM transition, same-account UAC,
installed old/new customer journey, remaining recovery phases, full product
candidate and boat acceptance are still open.

## Preserved first run

Exact source `6b28fccfc5982e8f678c6cca5c4f0e55c67f8f4d`,
[run 38597013498](https://github.com/ThereptileII/Work/actions/runs/38597013498):

| Platform | Top-level pass | Fail | Explicit skip |
| --- | ---: | ---: | ---: |
| Linux amd64 | 232 | 0 | 0 |
| Windows x86 | 330 | 1 | 5 |

The single failed test is
`brokerenroll.TestNativeCodeUpgradeRefusesForeignRecoveryDestination`.
Its expected refusal occurred, but the final `os.ReadFile` assertion encountered
a Windows sharing violation while the foreign file was still held by the native
DELETE-capable observation owner. The refusal and original-byte/identity
assertions must remain; this is not permission to weaken native custody.
The other four new native tests passed, including actual file stage/rename,
each complete rename interruption boundary and successor-reader closure.

The workflow's new required-case map also initially listed the nine Linux-only
publisher/CLI cases for Windows. Their build tags follow the existing Linux-only
repository lock contract. That mapping needs platform-correct selection before
the next gate; no existing Windows requirement is removed. Windows still tests
the signed consumer, operator contract and all native upgrade cases.

Original authenticated artifact downloads were retained without repacking. Their
GitHub sizes/digests and ZIP CRCs match:

| Artifact | ID | Bytes | ZIP SHA-256 |
| --- | --- | ---: | --- |
| Windows | 11911020961 | 62029 | `4538652d250cd5077a3281e8b3b9610e1d235c81c2ff480c111a29dca452e290` |
| Linux | 11908889695 | 64045 | `828bee268cfe80669581113ba40fc6cba562e55b9fd2ecc1e6bcbb587fb09bee` |

Original `tests.json` hashes:

- Windows: `d68cc4d5b4b69ec9edea50790c1270f18ab11db5c8d0512bf85a1e318322bd66`.
- Linux: `6aa1fc78fb70743c45ae48a8fa775dac69f77b87c998e3f5ff7d443b189b6e19`.

Linux's receipt confirms all 59 mandatory cases ran and passed. Windows's
required-case step did not run after the failed test command; no passing Windows
receipt is inferred. Five existing platform/opt-in skips remain unaccepted.
