# Typed authenticated-absence assertion repair

Failed original candidate: `f30b92e70b5357e505925547015155ef38837e73`,
[Staging run 38632343277 attempt 1](https://github.com/ThereptileII/Work/actions/runs/38632343277).
Linux updater job `116087242158` first fails at
`verifier_test.go:174 / TestRollbackAndMissingChannel`: it expects the obsolete
text fragment `target metadata`; the verified return is
`target absent from authenticated nondelegated metadata`.

Original Linux job log: 4,551,377 bytes,
SHA-256 `c0cd6a3a7f9914f86f89bfb441340687be2b3e5af300718be33bf71fc4111dfc`.
The original failed run and private full logs are retained.

Commit `6671af7a` introduced a typed authenticated-absence result and its negative
coverage. The old general test was not updated. Repair uses
`errors.Is(err, ErrAuthenticatedTargetAbsent)` and additionally requires an empty
release and payload. The following rollback rejection remains unchanged. No
runtime behavior or failure assertion is weakened.

Complete corrected Linux updater suite: `go test -count=1 -json ./...`, Go1.27.1,
**790 top-level cases pass, 0 fail, 0 skip; 13 packages pass**. Three command
packages have no tests. Original stream digest and exact changed-input scope:
[local-contracts.json](local-contracts.json). This does not qualify a Windows
application or installed customer journey.

Original Windows updater job `116087242254` was cancelled by matrix fail-fast.
Its original log contains RUN/PASS for both fresh-cache native tests and all five
negative subcases, with no skip/failure. The bounded
[native observation](native-fresh-cache-observation.json) retains exact job/source,
original log digest and terminal records. The whole updater gate did not pass;
the Windows application SDK guard was not reached. No package or boat deployment
is accepted from these partial results.
