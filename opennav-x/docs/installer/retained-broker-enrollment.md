# Original broker bytes, separate native SCM retest

This CI-only helper is selected under SCRUM-311/SCRUM-97 after run
[38288556379](https://github.com/ThereptileII/Work/actions/runs/38288556379), exact
`4dcee9b15922e62ea43a97e003e1278eead71b2f`, passed component production but its
SCM step failed resolving a relative output path before the fixture ran.

The original conclusion stays **failure**. The committed selection fixes the
repository, workflow, run/attempt, job, original head, artifact ID/name/digest.
`retained_broker_enrollment.py` reauthenticates those facts through GitHub and
requires every selected producer step to have succeeded and only the named SCM
step to have failed. It verifies the downloaded ZIP hash, bounds and checks its
entire member inventory before extracting only `broker-install/` to a new
location. Existing package verification checks the original native PE, compiled
Go identity, corresponding-source closure and launcher prerequisites. It never
executes or compiles the broker.

Manual `update-verifier-probe.yml` input `retained_enrollment_only=true` selects
one Windows job. Other focused flags must remain false. Normal/full jobs are
unchanged when this input is absent/false. The job gets only contents/actions
read permission. Before actual native effects, the existing fresh-enrollment
wrapper reauthenticates the original producer and external receipt, verifies the
actual helper checkout and original package, then runs the existing actual
fresh-service fixture once on an explicitly opted-in disposable hosted runner.
The fixture records original producer identity separately from helper identity;
it cannot accept an arbitrary retained binary or missing selection. No fixed
service, enrollment reservation, history or trust fence is cleared after failure.

Upload retains the external provenance receipt and actual service-test log. A
passing retest would establish only this original broker's native enrollment
fixture. It does not accept the original failed workflow, a newer broker, installed
no-update/Later/Now/offline behavior, repair/uninstall or the whole product.
Fresh source changes to a packaged component still require their own build.

The role diagnostic in [38295243035](https://github.com/ThereptileII/Work/actions/runs/38295243035)
on helper `b0f5f82a5327748d3ac746574d5645ecbe2e901b` identifies the rejected
original ancestor as `data-root`, component 1, `allowAdds=true`: the actual
ProgramData ancestor grants BUILTIN Users `0x116` with flags `0x02`. Service
initialization did not run. This is a stock-Windows compatibility finding, not a
reason to alter the hosted runner's shared ACLs. A production correction in both
enrollment and broker-store ancestry requires a new same-commit component build;
retesting only the retained original broker cannot qualify that correction.

## ProgramData compatibility boundary

The selected correction is limited to EA/attribute rights on the actual retained
ProgramData ancestor. It must not broadly trust Users, change the system ACL,
permit deletion/ACL changes, or relax protected SKAGER child descriptors.

An attribute-only writer can set a directory reparse point. The
[Windows filesystem contract](https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-fsa/4aeefef8-92c3-4abc-af7a-a610caf8a165)
requires refusal when that directory is nonempty. Consequently the exception
requires a retained existing immediate child opened without delete sharing,
with its original full identity, physical parent, same-volume and ordinary
directory state checked through every privileged boundary. A one-time directory
listing does not provide lasting custody.

The child is derived from the actual CommonStartMenu KnownFolder only when that
resolved path is below the actual ProgramData; no literal `Microsoft` component
is assumed. Its contents confer no trust or recovery authority. Missing,
redirected, reparse, changed, inaccessible or closed custody refuses. Acquisition
is read-only; no directory is created to manufacture a nonempty proof. Every
additional handle belongs to its original owner's error-preserving close path.
Other ancestor policies remain unchanged.

Implementation and native acceptance are separate. The original retained
failures above remain failures; the selected correction requires actual native
attribute/reparse/share-denial tests and fresh same-source service enrollment.

## Registry inheritance-template semantics

Retained-only run [38300951246](https://github.com/ThereptileII/Work/actions/runs/38300951246)
used original broker `0ddb649f` from fresh producer run `38298416414` and helper
`21ad7b27`. Authentication and fresh files passed. The same held `HKLM SYSTEM`
descriptor then refused an Administrators-owned registry key's `S-1-3-0` allow
ACE, flags `0x02`, mask `0x000f003f`. This is the standard CREATOR OWNER
inheritance template, not an actual untrusted token principal. Microsoft describes
[replacement with a descendant owner SID](https://learn.microsoft.com/en-us/windows-server/identity/ad-ds/manage/understand-security-identifiers)
and [effective versus template inheritance](https://learn.microsoft.com/en-us/windows/win32/secauthz/ace-inheritance-rules).

The correction recognizes only that exact allow ACE/SID/CI-only/KEY_ALL_ACCESS
combination, only on the four fixed retained machine registry ancestors or the
fresh service registry key (indices 1–5). Actual descriptor ownership must already
be trusted. It does not trust CREATOR OWNER as an owner or principal, and does
not transfer to SCM/service descriptors, other creator/owner SIDs, masks or flags.
Every other effective ACE and each retained descendant still receives its own
owner, mutation and exact registry-name checks. A materialized untrusted creator
SID or separate untrusted child-creation right still refuses. No machine ACL is
modified. The focused native retest also executes the inert template/diagnostic
and manager-versus-service rights matrices before the actual SCM fixture; these
are not additional privileged installation attempts.

A successful retained test would cover this helper's provisioning path against
the original broker, not a newly compiled provisioner. Final same-commit component
production and installed startup/update qualification remain required.

## Fresh service versus completed initialization

The original exclusive `CreateService` readback and the single read before its
first start may report STOPPED with Win32 exit 1077 (`ERROR_SERVICE_NEVER_STARTED`).
[MS-SCMR requires that code before a start attempt](https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-scmr/cf94d915-b4e1-40e5-872b-a9cb3ad09b46).
Only those two fresh-state checks accept 0 or 1077, still requiring zero accepted
controls and zero service-specific error. They confer no completed-run authority.
Every post-start completion, verification, registration and activation retains
the existing zero-exit rule. There is no retry, existing-service adoption or
process-exit inference from a STOPPED PID. Native read failures retain their
original cause and same-read numeric status for the explicit smoke diagnostic.
The original `38301585978` log did not emit those values; 1077 is a documented
contract correction, not a claimed observation from that failed run.
