# SCRUM-311 live adapter implementation — 2026-10-14

Helper revision `9352081fedd2e69d774a10b1f3a8752d26ee7619` integrates a bounded read-only GitHub HTTP adapter and
the original limited-token startup route into all four installed updater phases.
The full product remains frozen at `1107c04f55a5a51660e424493bf083fc83106443` in
[run 38549237034](https://github.com/ThereptileII/Work/actions/runs/38549237034).
These helper commits do not relabel or rebuild that application.

The six focused suites listed in `focused-results.json` pass **145 tests**.
The phase tests exercise the actual phase method with native observations
injected, including refusal before button input/readiness and cleanup failure.
HTTP tests use real socketpairs for framing and mocked connection establishment.
Independent read-only source review found no concrete blocking defect.

The exact clean product source also builds the small Linux repository operator
with Go 1.27.1, offline dependency resolution, CGO disabled and unchanged VCS
identity. Its hash and public trust comparison are in `operator-provenance.json`.
The public export equals the committed trust and existing private repository's
public configuration. No signing keys were accessed, no publication occurred.

Still pending: a suitable signed-in disposable Windows host, real TLS/API
transfer, independent live approval producer, one-use challenge/original-context
checks and coherent retained installation across B0 and all four phases. The
current B0 driver ends in uninstall, so the phase driver cannot follow it as-is.
Native startup/UAC, full installed recovery, boat and release acceptance remain
open. No failed hosted-token route was retried and no additional full build,
provider mutation, boat operation or public access was requested here.
