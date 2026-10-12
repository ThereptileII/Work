# Independent review comparison — 2026-10-12

Scope: the exact nine-file frozen source at `2df69903`, with per-file hashes in
`claude-comparison-pending.json`. The user explicitly approved sending this scope
to Anthropic. The read-only wrapper retained its normal snapshot/isolation,
pinned model and budget boundaries.

## Claude result

The attempted Opus 5 review failed with HTTP 429 `usage_limit_reached` before
returning findings. Reset: 2026-10-15 04:00 UTC (06:00 Stockholm). The failure
receipt and sanitized provider result are retained. No retry, alternate account,
model substitution, expanded transfer or raw transcript publication occurred.
There is no completed Claude opinion to compare or credit as acceptance.

## Sol fallback and Codex comparison

The existing GPT-6.1 Sol specialist independently inspected only the same frozen
nine files, read-only, without inspecting prior conclusions or running tests.
It found no confirmed actionable defect in scope. Its conclusions are consistent
with the recorded corrections: immutable original bootstrap identity remains
separate from current rotated trust; pending trust-refresh state retains rollback
floors; registration completion preserves post-commit cleanup uncertainty.

The fallback raised one explicitly unconfirmed native concern: stock validation
hashes the retained image initially, then checks identity, metadata and path. An
already-existing writable mapped view might change bytes without those checks
observing it. Root verified the initial-hash/metadata-only code shape remains in
current `brokerhost/stock_windows.go`; no exploit was reproduced. A final rehash
would add an observation, but cannot establish continuous loader custody. The
existing native stock/loader custody qualification remains open in SCRUM-311;
this hypothesis is not reported as a confirmed defect or a passing gate.

## Current-source boundary

Root compared all nine files with current `ac96ef54`. Six match. Three changed:
`brokerhost/registration.go`, `brokerhost/client_registration.go` and
`brokerhost/client_registration_windows.go` now carry the approved baseline
identity/import and retained baseline closure. The frozen review does not qualify
these additions; their separate integration evidence and focused tests apply.
The new protected-generation implementation was outside this nine-file scope.

Codex retains all decisions. Neither review supplies native execution, installed
update, healthy-start, release or public-launch acceptance.
