# SCRUM-311 — concrete acquisition and indexed cleanup

Integrated storage `ce226959`, verifier `07ad7922`, and actual broker callsite
`663b8c0d` join a reserved native candidate to authenticated downloads and
acquisition-only startup reconciliation. Original isolated commits and exact
file hashes are recorded in `source-receipt.json`.

The broker creates a private durable lease before metadata networking, binds one
fresh authenticated policy and exact consent, verifies Recovery's Lifecycle
sidecar before downloading Setup, and retains native file ownership through use
and cleanup. Restart cleanup accepts only indexed fixed disposable slots with
matching full native identities. Unknown or ambiguous residue refuses readiness;
payloads and stage reservations are never adopted, swept or reused.

Focused results:

* Storage: **7 Linux groups / 26 test events including subtests** passed.
* Verifier and directly affected existing acquisition checks: **12 Linux groups /
  43 test events including subtests** passed.
* Root host handoff: **6 Linux groups plus 13 fault subcases**, with race detection,
  passed before integration; their exact unchanged host source is inventoried.
* Native boat Windows: **5 groups plus 9 nested cases** passed with the retained
  6,588,928-byte executable, SHA-256
  `7d9bebdfd8822ab301bbe52da57c673a4e4f2d1676f9597747c9215ced7ec879`.
  The passing transcript is `native-tests.txt`; the runner receipt identifies
  Windows PowerShell 5.1 on 64-bit Windows.
* Windows/386 storage, verifier and joined host test-package compilation passed
  using Go 1.27.1. Storage's Windows-targeted vet passed. Cross-compilation is not
  a native MSVC application build or installed-service result.

The first verifier run deliberately remains recorded in the receipt. Its
`TestPrepareBrokerUpdateSetFailuresDrainOwnedFilesAndLease/recovery-layout`
fixture failed at `broker_download_test.go:306` with `protected signed update
check rejected`: it republished changed targets without advancing the retained
TUF version. Advancing that fixture's targets version to 3 corrected the setup;
production verification and assertions were not weakened. The focused rerun had
already started before the coordinator requested a failed-case-only retry. No
additional rerun or application rebuild followed.

Independent Codex source review found no concrete integration defect: the same
reserved owner reaches the downloader and cleanup precedes receipt sealing.
Claude's expanded private-source review remains pending exact-scope consent and
has not run. Do not count Codex review as a completed Claude comparison.

The native fixtures use disposable directories and the current test account.
They do not provision or qualify the actual broker principal, protected
ProgramData initialization, authenticated end-to-end service staging, installation
custody, installed updates or release acceptance. No installed application,
profile, service, remote-access configuration or vessel-control state was changed.
No full application build, CI dispatch or public push occurred. SCRUM-311 remains
In Progress.
