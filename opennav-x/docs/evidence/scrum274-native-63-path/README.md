# SCRUM-274: native AIS gate stopped at producer environment verification

[Run 37147671879](https://github.com/ThereptileII/Work/actions/runs/37147671879),
native job `111276021896`, exact remote
`63c1029584a325a961fa89794071cbf053c2e966`, mapped local
`d18da7ccc94b175d1f4a09a0019ab7e4b4f55d31`.

At **20:57:42 UTC**, step 18 failed at
`tools/windows-native-tool-facts.ps1:305`:

> Native tool facts changed: openssl-parent

The three offline AIS-runner guards passed. The failing command is the direct
`build-openssl-windows.ps1 -VerifyToolFactsOnly` re-probe, before zlib re-probe
or `test-ais-runtime-windows.py`. There are **no AIS native runtime report,
configure project, compile or runtime results** in this artifact. This is not
an application crash or an observed AIS transport failure.

The original captured and newly observed allowlisted fact JSONs differ in
exactly one field: `environment.PATHSha256`. Every other field, including
producer/helper source identity, PowerShell, Visual Studio and selected tools,
is equal. The exact JSONs and terminal step excerpt are retained unmodified.
The strict verifier correctly refused the changed environment; no captured
identity was refreshed or bypassed.

Source inspection identifies the parent-environment boundary to examine:
`build-pristine-windows.ps1:145,165` prepends selected native Perl and verified
Gettext before invoking the producer. The new workflow directly invokes the
re-probe in another step. This receipt establishes the **PATH hash mismatch**;
it does not reconstruct the entire captured PATH or claim an exact prefix-only
repair has already been proved. Producer verification must remain strict.

Original artifact **11285152311** is **53,015,878 bytes**, SHA256
`a6031fb49dd1c83016431fc904264f5f49116b3d45de4e664033510f33091136`.
All **13,395 ZIP entry CRCs** passed. GitHub artifact metadata and the archived
same-job receipt both bind the exact remote commit/run. The actual producer and
tool-facts helper hashes independently match this local source after Windows
CRLF checkout conversion. `audit.json` records those bindings and the full
terminal decoded job log identity (7,139,617 UTF-8 bytes).

The original ZIP and complete decoded job log remain untracked in the private
worktree `scrum274-native-63-failure/.local/` as `windows-integration-63.zip` and
`native-job-original.log`. Retrieval used one original artifact download and one
terminal job-log request; no multi-megabyte body was printed. The retained step
excerpt is lines 22107–22134 of that decoded log. Intermediate patch-written
log chunks were not used as original evidence because patch application
normalized line endings; the final log was transferred losslessly through
base64 and its byte length verified.

Integrated step 8 and the fixture UI/scenario steps passed before this failure.
Production, public TLS and packaging were skipped; later independent DPI/chart
checks cannot qualify them. No source changes, tests, build, workflow retry,
publication, Jira write or boat action occurred in this investigation.
