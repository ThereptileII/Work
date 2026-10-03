# SCRUM-285: native AIS project metadata was mistaken for an edge

Frozen remote `4ddf1f383e495150551946dd36103cd77cea85eb`, run `37155858878`,
attempt 1, job `111300070821`, failed step 18 after successful CMake generation.
The original `ais-native-runtime/report.json` reports `KeyError: 'Include'`.
The first traversed project, `ais_session_native.vcxproj`, contains four
`ItemDefinitionGroup/ProjectReference` metadata nodes without an `Include`
attribute. The original `.//m:ProjectReference` selector reaches those before
its four genuine `ItemGroup/ProjectReference` edges.

This is a harness parser defect, not an application crash, TLS failure, or
recurrence of the parent PATH failure. Original OpenSSL parent/child and zlib
parent/child tool-fact reprobes all passed. No AIS client compilation, session,
transport or provider lifecycle execution occurred. The requested counts in the
failed report are intentions, not completed tests. The package remains ineligible.

## Minimal correction

The original traversal is extracted unchanged into `project_closure`, except it
selects `.//m:ItemGroup/m:ProjectReference`. Access to the mandatory `Include`
attribute and to every named project remains strict. `PureWindowsPath` expresses
the existing native MSBuild path semantics while allowing the original backslash
paths to be inspected unchanged on Linux. No source, backend, import, receipt,
PE, runtime, or final identity guard is weakened. Reversing this small extraction
and selector/path-parser change reproduces the entire original wrapper exactly;
`source-inverse.json` records that check.

Seven focused tests pass against the original native project bytes: exact
eight-project closure; original failure reproduction; configuration metadata
ignored even when it carries an `Include`; missing real `Include` rejection;
missing project rejection; unknown real dependency rejection; and unchanged
complete downstream source/runtime guard block (including foreign-source refusal).
This last check is byte preservation, not a newly executed foreign-source build.
No original three-test wrapper suite or broad suite was repeated. Python syntax
and the new workflow YAML parse locally.

## Primary evidence and original identities

Original artifact ID `11287093978` is 58,474,264 bytes, SHA256
`a23b33366a3c119f724e47b77360082068791bd3f8204f8b8104b60db64fd8fd`.
The API digest agrees with the downloaded bytes; all 14,014 ZIP entries passed
CRC checking. The immutable original is retained locally at
`.local/windows-integration-4dd.zip`; its complete entry index is retained locally
with an index hash in `archive-index.json`. No original was rewritten.

The eight-project compressed fixture is a newly assembled subset, **not** the
original GitHub ZIP. Its manifest binds every unmodified member to its original
artifact path, SHA256 and size, and separately binds the original run, commit,
artifact ID/digest and fixture ZIP hash. The retained failed report's wrapper
hash `77332cf80caf68cdab6d405e7926a0c5d4833031e0196e3ed2e2f1666bc1cd47`
matches the frozen local wrapper with Windows CRLF; its LF source SHA is recorded
in the inverse receipt. Primary excerpts contain no credentials or personal data. The raw failed report,
configure log and successful producer reprobe logs are retained byte-identically
in `original-primary-files.zip` (a documentary subset, not the original artifact).

## Prepared short native proof (not dispatched)

Publish reviewed tooling to `skager-ais-project-closure`; the dedicated workflow
runs `python opennav-x/tools/test-ais-projects-native.py` on disposable
`windows-2022`, with a seven-minute job limit. It authenticates the **original**
run/attempt and completed failed native job `111300070821` through GitHub's
read-only Actions API (step 17 completed/success, step 18 completed/failure).
Whole-run completion is not required because unrelated Linux endurance may still
be active. Job SHA/run/attempt are checked, with job attempt checked when present
and the parent run attempt always required. The original artifact is authenticated,
and its exact archive digest/size, all CRCs and eight member hashes are checked.
The GitHub token is not forwarded to the signed storage redirect or written to
evidence. Only those original project files are extracted for the seven project cases.
An eighth focused fake-API case verifies terminal-native/active-sibling acceptance
and wrong identity, nonterminal job, incorrect gates and duplicate-step refusal.
That new case passed locally once; the original seven cases were not repeated.
The receipt distinguishes original application commit/run from current tooling
commit/run, records source/runtime identities, and retains failures. The workflow
uploads compact receipts, logs and the eight original projects, not the 58 MB ZIP.

This is native Python parsing of genuine CMake/MSVC-generated project files.
It compiles no binary, reuses no SDK, and does not rewrite or replay an old
same-job dependency receipt. The existing same-job contract cannot authorize
cross-job maintained-dependency execution. Actual AIS MSVC/TLS/lifecycle execution
therefore remains a later gate using genuinely same-job verified dependencies.
No CI dispatch, full build, source-dependency producer, application, or boat action
was performed for this implementation.
