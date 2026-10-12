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
