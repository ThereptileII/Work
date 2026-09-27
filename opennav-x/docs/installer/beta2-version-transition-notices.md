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
groups pass locally. Exact native candidate `8e780edc` subsequently passed
39 installer checks with 19 screenshot entries in
[run 36287991989](https://github.com/ThereptileII/Work/actions/runs/36287991989).
All three real notices were observed and acknowledged: candidate to genuine
Beta 1, Beta 1 to candidate, and candidate to exact official stock 5.12.4.
The retained Beta 1 warning screenshot was inspected. Update, repair, rollback,
uninstall and reinstall completed with stock/profile checks preserved.

[The native evidence record](../evidence/beta2-windows-8e780edc.json) identifies
the installer, full artifact and hashes. This successful gate does not qualify
the later Start-menu migration, which was absent from 8e780edc, or the whole
release: its later endurance harness failed before sampling. No boat action or
production UI change was made for this notice-handling correction.

## Prior-package download ordering

Candidate `a896fb5c5fa935b5daa757087ebedc4cd531c1d3` passed native clean
installation, chart startup and initial rollback, then the genuine early Beta 2
rollback fixture refused to download because its read-only Actions credential
had already been removed. The harness now fetches both exact hash-pinned prior
packages before removing that credential in `finally`. Tested installers and
applications still never inherit it. Neither package identity nor rollback
assertions were weakened. Python compilation passes locally; the corrected
full native lifecycle remains a replacement-candidate gate.
