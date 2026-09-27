# Expected navigation cautions during installer qualification

Candidate `60cd054712c6a930147247970121a959d48e03bf`, native
[run 36284246369](https://github.com/ThereptileII/Work/actions/runs/36284246369),
passed the clean Beta 2 wizard installation, exact candidate startup with real
coastline, and first-install rollback while preserving stock/profile bytes.
The subsequent genuine accepted Beta 1 installation also committed successfully.
Its first application launch presented **Welcome to OpenCPN** before XNav attached;
the lifecycle harness instead waited for the final XNav frame title and timed out.
This was a missed expected version-change notice, not a demonstrated failed
installation transaction. Later upgrade/repair/uninstall gates were not reached.

The warning screenshot and failed report are retained privately in
`evidence/local/boat-beta2/installer-60cd/`. Artifact `10920653016` has SHA-256
`1f3b17f18560fd52e99419187855cd96b2e354cd15080c371e8e4dfe9ebcf437`.
The complete native artifact is separately retained in `windows-60cd/`.
Fixture UI, production build/package, 100/125/150% DPI/touch, and chart/plugin
gates passed. The installer failure prevented delivery and Windows endurance;
this candidate is not accepted as a release.

Pinned `gui/src/ocpn_app.cpp` calls `ShowNavWarning` whenever the stored full
version/build string changes. This applies to both candidate→accepted Beta 1
and Beta 1→candidate, as well as returning to the official stock build. The
correct lifecycle test captures and acknowledges each actual visible caution;
it does not rewrite `ConfigVersionString` or `NavMessageShown` to suppress it.

The revised disposable CI harness admits only these three explicit transitions.
For installed versions it verifies the exact current owned generation,
version/commit and executable digest; stock must match its official hash after
uninstall. Before acknowledgement, the launched process handle resolves to the
expected executable, and its unique visible welcome/main-frame pair exposes
exactly Agree and Cancel. The existing native dialog handler sends one visible
Agree action and requires dismissal. Unrelated/ambiguous dialogs fail the gate.
Each notice retains its private screenshot and source/executable identity.

Readiness now uses the existing fresh-start/log-rotation observer for each launch
and actual in-app restart, instead of counting markers across changing log files.
Chart content, clean process exit, profile fixtures and installation hashes
remain required. Five policy/regression groups and the existing three startup-log
groups pass locally; full native lifecycle qualification of the revised harness
is pending the next exact-commit product run. No boat action or production UI
change was made for this correction.
