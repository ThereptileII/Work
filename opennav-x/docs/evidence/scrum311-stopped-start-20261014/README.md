# Exact stopped-successor start fence — 2026-10-14

SCRUM-311 source `9b12442d9de9ad1fd91780f74cdfa43eba261dc6` passes
[native run 38615096156](https://github.com/ThereptileII/Work/actions/runs/38615096156).
Windows x86: **344 top-level passes, zero failures, five existing explicit skips**.
Linux: **240 top-level passes, zero failures or skips**. All 91 Windows / 67 Linux
mandatory maintenance cases ran exactly once and passed, including all four new
required cases. Original provider artifact digests, byte lengths, ZIP CRCs,
receipt source/run and test-stream hashes were verified; receipts are retained
alongside this record. Both corresponding-source suites passed locally: 12 broker
and 14 provisioner cases.

## Qualified scope

The original normal activation and clean AUTO_START/STOPPED recovery consume the
same exclusive, durable exact-original/policy/successor/pair start attempt before
START. Existing or partial attempts refuse another start. The observed normally
RUNNING process, birth identity, token and image remain owned across handoff to
a separate query-only finalizer. A changed non-AUTO configuration refuses before
that handoff. Independent review identified both the missing original-activation
fence and possible configuration downgrade adoption; both were corrected before
native execution.

Three portable tests cover ordered single effects, consumed failure attempts and
complete binding. The new native NTFS fixture covers absent, empty, foreign and
exact attempts, pending/public successor readers, and preservation of the empty
interrupted object. Service and IPC effects in these tests remain seams; these
results do not qualify actual installed SCM, customer UAC or the boat.

## Preserved initial failure

[Run 38614545234](https://github.com/ThereptileII/Work/actions/runs/38614545234),
source `c626dce6f97ccdb0fb804f753e925394bd880faa`, had 343 Windows passes,
one failure and five skips; Linux passed all 240 cases. The new empty-file case
called the normal publication helper with zero bytes; that helper correctly
refused before the case reached its assertions. The fixture now exclusively
creates an actual zero-length file and checks its identity and size survive
refusal. Production code and failure assertions were unchanged. Original failed
artifacts and their hashes remain recorded in `first-failure-receipt.json`.

## Remaining acceptance

Normal Setup/Maintain code-upgrade activation, authenticated early staging and
DEMAND_START verification recovery, actual installed SCM/IPC/customer UAC and
profile/lifecycle qualification remain open. Torn or foreign authority remains
refused; no enrollment, trust, floor or history reset is permitted. A second new
broker-code generation is unsupported. These results do not close SCRUM-311 or
approve public access.

One combined private Staging build at this exact source is
[38615517743](https://github.com/ThereptileII/Work/actions/runs/38615517743).
It was dispatched after this focused gate, with design/extended checks off.
Its outcome is separate; this record does not claim full application acceptance.
