# SCRUM-311 approved client registration integration — 2026-10-12

Product source for the focused combined checks:
`3404a8d210bafa12ff8670e121feef3a1a5c251d`.
Contract documentation: `7f542766`.

## Implemented source

The independently approved exact request now reaches the actual Windows service
through the retained fresh machine provisioner and stopped-only SCM operation.
The optional CLI pair is parsed before network or mutation; exact bootstrap trust
and a real canonical user SID are required. Native request files have final
protected descriptors at creation and retained no-write/no-delete read custody.
The fixed broker-only reader returns owned original bytes. Actual service dispatch
rechecks SID/hash/request and calls the native coordinator before `Running` or a
request pipe. No caller-supplied request path reaches the broker.

The broker creates fresh protected user/trust/artifact state only after a permanent
SID-specific attempt fence. It initializes and validates actual verifier state,
then publishes schema 2 containing exact original approval bytes. Completed native
evidence binds approval and JSON hashes. Partial and existing client state is
refused rather than reset, adopted or retried. This is **fresh enrollment only**;
existing-machine/client migration is not supplied by this API.

Durable publication has an explicit commit point. Late cancellation cannot turn a
successful commit into failure; actual I/O or cleanup uncertainty remains explicit
and may coexist with completed local state. Native stock observation and temporary
probe cleanup now preserve close failures, and repeated close returns the cached
first outcome without retry.

## Review findings and corrections

Independent Codex review found that validating the immutable bootstrap anchor's
current expiry during every protected read would prevent updates after legitimate
root rotation. `988b90d9` separates original identity from current retained trust:
completed reads retain all exact-byte/schema/signature checks; fresh enrollment
still requires an unexpired anchor. The actual verifier still validates current
retained metadata, expiry and rollback floors without resetting state.

The original real-time regression used two short expiry windows. A busy runner
could consume a window during setup. `3404a8d2` replaces sleeps with private,
per-call forward test clocks using go-tuf's documented test hook. Exported APIs,
request/configuration and production wrappers expose no clock override. Each
updater instance is independent; network deadlines remain real. The focused
actual TLS/signature/state regression verifies rotated-root continuity, rollback
refusal, expired current-root refusal and retained floors. Separate three-path
tests prove test/default/test clock isolation. Independent source review found no
concrete production bypass in these paths.

Independent packaging review found that copied-package checks could accept omitted
new runtime sources if both member and inventory entry were removed. `e91f89ff`
requires the complete fixed registration/enrollment source set, applies that
contract to the producer and hashes retained Go members across the module.
Omission and imported-member tampering checks cover the correction. This does not
substitute local hashes for independently authenticated producer/publisher trust.

`e7fabf9b` repairs stock cleanup uncertainty; a separate bounded source review
found no remaining concrete defect. `b8913374` joins the actual stopped dispatcher;
an independent integration review confirmed its early return before service
readiness/pipe serving and its fixed native authority callbacks.

## Focused checks

`source-receipt.json` binds 51 changed source/test/packaging inputs to exact product
source bytes and records the three Windows/x86 test executable hashes. The
original test-fixture compile failures are preserved separately rather than
reported as passes.

Combined affected Linux checks use race detection and one fresh execution:

| Scope | Top-level passes | Nested passes | Failures / skips |
| --- | ---: | ---: | --- |
| Repository, request, host, store, enrollment, provisioner | 115 | 349 | 0 / 0 |
| Actual retained-root rotation and per-call clock isolation | 2 | 3 | 0 / 0 |

The four focused copied-package/producer Python suites passed 12, 14, 32 and 10
tests respectively in the isolated implementation worktree, once, with no failed
attempt. Their seven integrated source files match that agent's committed bytes;
this is delegated focused evidence, not a separate root rerun or native producer.
Windows/x86 test compilation for the joined host, request/enrollment and store
passes. Cross compilation does not qualify native principal, ACL or SCM behavior.

Native Windows execution on the boat passes **14 top-level + 2 nested tests**,
with zero failures and zero skips: client store 9/2, approved request 4/0 and stock
cleanup 1/0. Runner `bd9c05c79626982b85fe66dded0b1e266cd89cd7` has SHA-256
`22a7633bd5a352de039fa84cb562e4561c439cb18dc0519048948160e941585d`.
Its exact discovery/RUN/PASS allowlists, retained executable hashes and bounded
owned-process receipts are preserved under `native-windows/`. Native PowerShell
parsing and all four transferred file hashes passed before test execution. The
initial direct transfer timed out before tests and its missing-file preflight
failed; both are preserved. A compressed transfer passed archive and member hashes.
Only a process-local script policy was used; machine policy was unchanged.

The original OpenCPN PID 1852 remained responsive afterward and no owned test
process remained. Tests used disposable temporary paths/current-user fixture ACLs,
read-only refusal and inert in-memory structures. They did not provision/start a
service or change application/profile state. Temporary-file mechanics do not prove
actual privileged machine enrollment, actual broker principal/SCM authority or
installed application custody; these are not MSVC/application acceptance results.

## Unaccepted boundaries

Independent authentication of first privileged execution/publisher/loader,
actual disposable-machine principal/ACL/SCM creation and installed-byte custody,
healthy-start/installed update handoff, profile preservation, recovery and final
native Windows/boat application qualification remain required. Self-selected
hashes, a protected record, an operator declaration and source checks are not those
authorities. Schema 1 is not adopted; older native completed registrations had no
accepted producer. No old source or package is relabelled by these checks.

Claude's new expanded source review remains pending exact-scope transfer consent.
The earlier completed acquisition review is separate evidence and is not credited
for this increment. No new private source was sent to Anthropic.

No full application build, CI dispatch, public push, release, installed
application/profile change, visual/endurance suite or physical command occurred
for this source increment. SCRUM-311 remains In Progress. Staging is the default;
Production promotion and public access remain separate explicit decisions.
