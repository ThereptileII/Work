# Standalone installer-test helper hashing failure

SCRUM-217. Candidate `0da2c64379d5a9cc4b9b2bd068de6e0b69816577`, run
`37207119257`. The retained original installer-failure ZIP is 571,402 bytes,
SHA-256 `fdab34d38224e220e9cbfd61b063fc312166ffcad855e1cd5721e02ae3bbf118`.
The adjacent log is copied without alteration from that archive's retained
extraction; SHA-256
`dee4745c9154002274eb3c2e04d32af294fd2878ef4431e9ade1bd2f955175e8`.

The new `test-installer-missing-dll-selftest.ps1` failed at its first
`Get-FileHash`, before creating its receipt or invoking the AST-loaded production
`SelfTest`. No helper receipt exists. The original lifecycle report (SHA-256
`35de93e1eb053b14cbe70e064c2b9b0dc760601805c2dc247d7f606057f6a315`)
records 35 completed checks, including exact-candidate clean installation,
startup/modes, Beta 1 upgrade, repair/rollback, and rejection of the deliberately
missing wx base DLL before publication. Its final failure is the parent
assertion that this standalone helper did not pass.

The helper now AST-loads unchanged production `Lifecycle.ps1::Hash` and uses it
for all three hashes. That function already uses .NET SHA-256 specifically to
avoid dependence on `Get-FileHash` under an inherited `PSModulePath`. Application,
installer, missing-import refusal, expected loader exit, error-mode restoration,
no-modal and preservation assertions are unchanged.

Dependency inspection found no further `Get-FileHash` call. The helper's other
commands are ordinary PowerShell commands also used by the production path;
`SelfTest` retains its framework-directory scope for `Add-Type`. The helper does
not require an external executable or optional module for hashing. This is a
source inspection, not proof that its unreached commands execute successfully.

No concrete clean-install/startup safety defect follows from this helper failure:
the failure is in a separate deliberately damaged-stage test after those paths
passed. This does not turn the lifecycle suite or CI run into a pass. The direct
missing-DLL `SelfTest` case and subsequent lifecycle cases remain unexecuted.
Exact package identity and existing boat installation/launch guards still apply
to any development handoff; this repair grants no binary or launch attestation.

Validation: diff/whitespace inspection only. No tests, PowerShell execution,
build, workflow dispatch, download or boat operation; no local PowerShell
runtime is available. The repair is not natively verified.
