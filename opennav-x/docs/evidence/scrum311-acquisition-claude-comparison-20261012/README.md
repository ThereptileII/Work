# SCRUM-311 acquisition/cleanup Claude comparison — 2026-10-12

At the user's request, the restricted Claude wrapper completed a read-only
**Opus 5** review of 11 selected source/test files at `8d8b6a07` in 453,304 ms.
The wrapper verified the returned model and unchanged selected bytes. Exact
file hashes and review metadata are in [source-receipt.json](source-receipt.json).
No credentials, boat data or raw transcripts are published. No child specialist
result or Claude test execution is claimed.

This reviews the newly integrated actual `PrepareUpdateSet` → progress → launcher
acquisition/cleanup path, distinct from the earlier armed-marker review. Codex
checked findings against the full source, including native files outside Claude's
snapshot. External review is advice, not acceptance authority.

| Claude finding | Codex comparison/disposition |
| --- | --- |
| Both artifacts use one authenticated offer; cleanup failures remain classified | Agree. One freshly checked candidate supplies both downloads. Errors remain `errors.Is`-detectable through acquisition, progress redaction and actual startup refusal. Existing combined-root focused tests support these paths. |
| Repeated `VerifiedArtifact.Close` after failed removal can delete a later file at the same spelling | Accepted as a latent API hazard, not a demonstrated active exploit. Current acquisition callers close once and directory protection is a stated precondition. Fixed in `135cab1f`: one cleanup attempt, with the original result retained on repeated closes. A real failed-removal/replacement regression verifies that later bytes remain intact. |
| Crash/failed cleanup can leave retained artifact residue indefinitely | Real remaining resource/recovery work under SCRUM-311. There is no orphan-artifact reconciliation in the inspected implementation. Do not adopt the suggested blind wildcard deletion: deletion needs independently established ownership and liveness. No recovery-completion claim follows from the current cleanup fix. |
| Armed errors may expose sensitive details | Not established by full native source. Current native checks return fixed diagnostics or Windows error codes; the progress branch forwards that classification. A future platform implementation must preserve redaction. Arbitrary injected errors are not evidence of a present production leak. |
| Timed-out offer prompt is not drained | Rejected for the actual native path. `nativePlatform.wait` kills the inert prompt and drains its completion channel before returning timeout. Generic fake-platform behavior alone does not establish a product bug. |
| Exported set acquisition has no single 30-minute deadline | True distinction, not a violation of the documented per-artifact cap. Each download has its own cap; the actual installed progress caller supplies one 30-minute context across the entire set. Independent API consumers must supply their own total deadline. |
| Non-timeout cancellation may release artifacts while installer runs | Not established for native production. After successful start, cancellation returns `timedOut: true` with the retained completion channel. The actual installer branch drains that channel before deferred artifact cleanup. Native errors before start have no installer to retain. |
| Trust initialization skips pending recovery | Separate installer-only operation. Full source takes the maintenance lock and checks the armed restriction; it does not launch the application. This snapshot does not establish unsafe trust initialization. Further trust/broker qualification remains open. |
| Typed-nil preparation may panic | Test/service-injection concern only. Actual exported `Run` constructs a non-nil owner on success and returns literal nil on failure. No production trigger established. |

## Evidence and limits

The actual acquisition increment's combined-root tests passed five new tests and
nine subcases. The cleanup-classification increment passed three additional
tests and seven subcases. These are focused Linux results, not new native Windows
qualification. The failed-removal fixture explicitly does not prove namespace
custody. The independently integrated direct launch restriction passed 39
portable checks and is outside Claude's selected review scope.

The cleanup hardening's isolated checks passed three focused download tests and
Windows/x86 compilation. Native execution remains pending. This later correction
changes two files after the completed review; the receipt intentionally describes
the original frozen snapshot, not the amended code or release qualification.
Combined-root verification also passed the new replacement-file test, the
two-owner cleanup test and the actual caller's two cleanup-refusal subcases.

No full application build, CI dispatch, boat deployment, visual suite, endurance
test or physical hardware command was performed for this review. Protected broker
creation, native loader custody, orphan-artifact recovery and exact-revision
Windows/boat update qualification remain unfinished. SCRUM-311 stays In Progress.
