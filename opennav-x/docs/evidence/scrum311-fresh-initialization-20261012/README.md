# SCRUM-311 fresh initialization — 2026-10-12

Exact integrated source: **e227d0c30299a636e0fe6c3b3a83c9c89f62e1f4**.
Creator integration is `50227181684dce3a3a7e3cc558d1083c4bf49914`; the later
source commit adds actual SCM dispatch and normal startup completion gating.

The native creator uses the actual service principal, full held physical NTFS
ancestry, an independently provisioned protected parent, final creation
descriptors, and a permanent CREATE_NEW started/completed parent fence. Existing
roots or consumed attempts refuse; no adoption, deletion or repair occurs. Normal
SCM startup now requires read-only completed-fence plus exact-tree validation.
Explicit initialization stops before Running/pipe serving. The
[contract](../../installer/broker-bootstrap-policy.md#fresh-native-store-initialization)
records exact boundaries and caller prerequisites.

## Independent comparison

One GPT-6.1 Sol agent implemented the isolated native creator while the root
implemented SCM integration; another GPT-6.1 Sol specialist reviewed both
read-only. The reviewer identified that a consumed fence alone would not
distinguish partial creation from completion. The implementation now writes a
separate synchronized completion state only after exact tree checks, and normal
startup rejects partial state. Final review found no blocking source defect.

The reviewer also confirmed that completed local creation survives a subsequent
close error or cancellation; it is not authenticated enrollment or startup health.
A future provisioner must inspect terminal service status/result instead of
equating StartService acceptance with a ready broker. No reviewer test execution
or native execution is claimed.

Claude's earlier completed acquisition review and Codex's dispositions are in
[the existing comparison](../scrum311-acquisition-claude-comparison-20261012/README.md).
That review does **not** cover this new source. The expanded broker-source
transfer is still awaiting its exact-scope approval after automatic review
rejection. It was not retried and no new Claude review result is claimed.

## Focused results

- Linux Go 1.27.1 `go test -json -race -count=1 ./brokerhost ./brokerstore`:
  **34 top-level tests, 143 pass events including nested cases; 0 failure,
  0 skip**, 6.071 seconds. Raw events and stderr are retained alongside the summary.
- Windows/386 cross-compilation: both affected test executables and the actual
  `cmd/skager-update-broker` component compile; affected Windows vet passes,
  8.143 seconds. This is **not native Windows execution** or application packaging.
- New portable coverage exercises initialization sequencing, all operation/check
  fault positions, retained cleanup uncertainty, non-Windows refusal, exact SCM
  argument dispatch, cancellation and native error redaction.
- New native fixtures compile for exact creation descriptors, existing root/fence,
  root-deletion retry, partial state, missing child, unsafe parent, ordinary-token
  refusal and impersonation refusal. They use inert temporary namespaces and do
  not provision SCM or touch actual ProgramData. Their execution remains pending.

The isolated creator additionally passed portable/race tests and Windows amd64
and 386 test compilation, with amd64 vet. Those agent-reported checks are not
substituted for the retained combined-root results.

## Scope still open

Independent initial verifier/provisioner authentication, parent protection at
original creation, actual SCM enrollment/control DACL, protected client
registration, installed launcher handoff, installation/loader custody,
authenticated healthy receipt and complete recovery/update qualification remain
unfinished. SCRUM-311 remains In Progress.

No full application build, CI dispatch, source push, boat application/profile
change, service activation, public release or physical hardware command occurred.
The source-batch-before-full-build policy remains in force.
