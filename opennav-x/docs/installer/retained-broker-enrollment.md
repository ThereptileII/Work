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
