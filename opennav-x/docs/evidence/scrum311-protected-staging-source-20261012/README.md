# SCRUM-311 protected staging source — 2026-10-12

Source revision: **3d0b68d3c99118a1f948a1319e56b4a9a4ac5036**. This is a
local source integration, not a qualified package or enabled updater.

| Component | Integrated commit | Scope |
| --- | --- | --- |
| Authenticated extraction | `b6d5ac0b` | Original live acquisition owner; revalidated retained ZIP, inventory, CRC and exact Lifecycle identity; exact installed app/docs set only. |
| Broker pipe | `27751e1f` | Actual local token/process and SCM/module identity, bounded strict framing, cancellation and replay refusal. |
| Protected store | `ade3b817` | Actual non-impersonated virtual account, originally protected fixed machine root, exclusive creation, held physical objects, exact permissions, bounded durable reservations. |
| Production joining operation | `3d0b68d3` | Protected registration → measured stock → fresh real acquisition → protected extraction → artifact cleanup → staging receipt → final validation. SCM-only service entry point. |

The three agents used isolated worktrees and disjoint file ownership. Root alone
integrated the commits. A separate bounded read-only Codex review inspected the
joining code. Its concrete request-ID length disagreement was already identified
by root and assigned to the pipe agent; all components now use exactly 32 lower
hex characters and the returned stage ID must equal the request ID. Root also
required explicit invalid-UTF-8 rejection and ancestor metadata/ACL protection.
The review made no edits and executed no tests. The earlier independent Claude
acquisition/cleanup comparison remains in its [separate exact-scope record](../scrum311-acquisition-claude-comparison-20261012/README.md);
it is not a Claude review of these later broker files.

## Focused results

| Check | Actual result |
| --- | --- |
| Signed local TLS acquisition/extraction | 7 top-level groups / 23 subcases PASS; five groups are new StageRecovery coverage, two are existing acquisition regressions. |
| Joining operation and registration | 8 groups / 35 subcases PASS. Inert private seams check ordering/failures, cleanup before receipt, replay, cancellation, concurrent refusal, redaction and ambiguous/unapproved inputs. |
| Pipe protocol and portable native refusal | 8 groups / 21 subcases PASS. This is protocol/portable evidence, not actual authenticated Windows IPC. |
| Storage state/path/streaming and portable native refusal | 14 groups / 39 subcases PASS. Backend fakes do not prove Windows permissions or custody. |
| Windows/386 host test executable | Cross-compilation PASS. |
| Windows/386 service command | Cross-compilation PASS; binary is private compile-only output, not a release artifact. |
| Windows-targeted vet | PASS for brokerhost, brokerpipe, brokerstore and the service command. |

Total: **37 top-level groups and 118 subcases**, with repeated intermediate
runs excluded. Tests used pinned Go 1.27.1 with `GOTOOLCHAIN=local` and
`-mod=readonly`; Windows checks used `GOOS=windows GOARCH=386 CGO_ENABLED=0`.
The existing CI workflow already enumerates `go test ./...` / `go vet ./...`;
no duplicate workflow or full application build was created. Component agents
also compiled their Windows tests in isolation; those are not additional native
passes. Curated source hashes and focused result-log hashes appear in
[source-receipt.json](source-receipt.json). Raw local logs remain private.

## Boundary and required qualification

The service and launcher do not activate one another yet. No provisioning,
installation, trust reset, selected-generation mutation or code execution occurs
in the staging operation. Registration is chosen by the actual peer SID, not a
client-supplied path. Its protected stock declaration must match the independently
qualified official OpenCPN 5.12.4/x86 hash, then actual retained bytes are hashed.
Pipe consent identifies the exact offer; it does not independently prove a human
gesture. Only signed eligible acquisition can reach extraction.

Candidates and files receive broker/SYSTEM full access and the selected client's
read/execute access at creation. Client access is not claimed to be an OS-wide
execution prohibition. The operation itself never launches candidate code and
its receipt is not a future spawn or no-child proof. The receipt can remain after
a later owner-close/namespace failure, while the response correctly refuses.
All partial candidates remain protected and count toward 128 entries. There is
no automatic eviction, wildcard cleanup or completed orphan reconciliation.

Native qualification must establish actual service identity, cross-account
query/synchronize/token-query rights, pipe behavior, creation-time ACLs, held
object/DOS resolution and cancellation. Missing rights refuse; there is no
fallback to a PID, ordinary-user process, SYSTEM substitute or permissions grant.
Current IPC intentionally supports nonzero interactive sessions and normal
local/domain user SID forms; cloud-only SID forms require separate support.
Provisioning must establish protection originally: present ACLs cannot exclude
earlier writable handles. Directory no-delete sharing alone cannot freeze
reparse metadata; ancestor policy checks now reject untrusted mutation grants.
An already-blocked arbitrary reader cannot be interrupted by a context; the
real recovery reader uses bounded retained local files. Undrained SCM shutdown
exits its own process without closing owners concurrently with active work.

Authenticated service provisioning/upgrades/removal and initial trust,
installed-launcher activation, full loader/plugin custody, installation,
crash/partial-artifact recovery and exact-revision Windows/boat acceptance remain
unfinished under SCRUM-311. No full build, CI dispatch, boat operation, visual or
endurance run, physical command, publication or Production promotion occurred.
