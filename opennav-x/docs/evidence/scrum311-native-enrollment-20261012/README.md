# SCRUM-311 native machine enrollment source — 2026-10-12

Product source: `c185300d18e81b317358e172d04a1ee7e412689e`.
Inert native runner: `a5d7b613d2d57a0956139f951ed9e73afb310d44`.

The coordinator joined three isolated implementation increments:

* `84fd669a` and `74c08f41`: exclusive protected fixed KnownFolder creation,
  bounded exact-image copy, retained native ownership and cleanup classification.
* `2add96cc`: actual fixed provisioner producer/verifier, same-commit broker and
  launcher prerequisites, source inventory and strict embedded-build checks.
* `0a6cf9da`: exclusive fixed SCM enrollment under the broker virtual account,
  protected registry/configuration checks and bounded stopped-only service calls.

`c185300d` supplies the actual operator CLI and orchestrates independently pinned
public trust/bootstrap verification, retained exact acquisition, machine creation,
initialization and a separate durable `verify-store` check. Neither special service
invocation announces Running or opens the request pipe. Source inspection and an
independent bounded Codex review found missing preservation of `ErrCleanup` in the
coordinator's initial/final file checks; that was corrected before the joined
commit. The final joining review found no further blocking defect within scope.

The native design is documented in
[broker bootstrap policy](../../installer/broker-bootstrap-policy.md). In
particular, `CreateServiceW` does not take a security descriptor: actual default
creation protection is checked and tightened before any start. No final-DACL-at-
creation claim is made. STOPPED/PID fields are not process-exit proof; a second
start racing process drainage refuses rather than automatically retrying.

## Focused results

[results.json](results.json) and its stdout/stderr files retain the exact commands
and outcomes at the product source revision:

* Linux affected `brokerenroll`, `brokerhost` and provisioner CLI race tests:
  **32 top-level tests / 138 pass events including nested cases**, zero failures
  and zero skips.
* Windows/x86 enrollment and broker-host test cross-compilation: pass.
* Windows/x86 actual broker and provisioner component cross-compilation: pass.
* Windows/x86 affected vet: pass.

[packaging-results.json](packaging-results.json) retains **81 passing inert Python
tests**: launcher producer 16, broker producer 31, broker verifier 11, provisioner
producer 10 and provisioner verifier 13. These tests do not run the actual native
producer, authenticate a publisher or qualify a release package. Cross-compiled
component outputs made while documentation/evidence was uncommitted are not a
clean native producer identity or releasable artifacts.

## Actual boat Windows policy execution

The retained Windows/x86 enrollment test executable was transferred using the
existing `boat` SSH alias into a fresh isolated source-check directory, with the
exact reviewed runner. Local and remote hashes were checked before execution:

* Test executable: 15,013,376 bytes,
  `43d2d3549373af8eb2b4a5ac2754d4a7d15761eb4a706aeb056d3a5381f9cb00`.
* Runner:
  `485aaf2d9554510b7ec5f91e1bddf5aa451a4a1343fe5b5b7f7f7144d403b85b`.

The first attempt failed **before tests executed** because Windows PowerShell
script execution was disabled. Retain
[initial receipt](native-policy-receipt.json) and
[initial output](native-policy-tests.txt). No product failure is inferred from it.
The same reviewed bytes then ran with `-ExecutionPolicy Bypass` for that single
PowerShell process. No permanent execution-policy or remote-access setting changed.

[Native output](native-policy-tests-process-policy.txt) and
[successful receipt](native-policy-process-policy-receipt.json) retain **nine
passing top-level tests**, no application restart, on 64-bit Windows with
PowerShell 5.1.22621.5624:

1. Service SID derivation before service creation.
2. Physical path validation.
3. Code descriptor denies ordinary-user mutation.
4. Ancestor mutation refusal.
5. Initial service descriptor rejects untrusted mutation.
6. Exact final service descriptor with ordinary query-only rights.
7. Resolved registry control-set ancestry validation.
8. Normal/aliased start rejection before any SCM call.
9. Returned service configuration UTF-16 strings stay inside the native buffer.

The runner uses an exact reviewed allowlist; these tests exercise SID derivation,
in-memory descriptors, paths, buffers and refusals only. They do not call actual
SCM provisioning, the coordinator or machine namespace creation. They are native
Go/Windows API tests, **not native MSVC application qualification**. OpenCPN PID
1852 was present and responding before and after. No profile, chart, connection,
installation, hardware output or running application was changed.

## Remaining gates

The first final provisioner still requires independently established executable,
publisher and loader authentication; self-hashes, metadata and UAC are insufficient.
Actual fresh privileged creation/SCM execution requires disposable Windows
qualification before boat use. Protected client registration, installation/loader
custody, installed launcher handoff, healthy-start receipt, recovery and complete
Windows/boat update acceptance remain unfinished. Signing delivery is disabled.
SCRUM-311 remains In Progress, not Done.

The earlier completed Claude acquisition review and confirmed cleanup correction
remain recorded in their own comparison. Claude has not reviewed this increment;
the expanded private broker transfer was rejected by automatic approval and
exact-scope consent remains pending. No additional private source was sent.

No full application build, CI dispatch, public push, release activation, visual
suite, endurance run or physical actuator command occurred in this increment.
