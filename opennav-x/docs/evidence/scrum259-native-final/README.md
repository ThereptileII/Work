# SCRUM-259 corrected native short-gate verification

[Run 37098440931](https://github.com/ThereptileII/Work/actions/runs/37098440931),
job `111133035756`, completed successfully at exact remote source
`5d6c5cf44cb72f8166b2ab67115bccd568336e35`, mapped to local
`de26174c6cd801da1411283728aa7c8d19517083`.

The independently downloaded artifact `11265451293` contains 34 files. Its ZIP
is **16,574 bytes**, SHA-256
`0789dc1b6afef740703e59b1e4a55c66b20b3be666dcab24b9abb5137f16e908`,
matching both GitHub API metadata and the independent upload/job log. All ZIP
CRCs passed. Every extracted file is inventoried with size, CRC and SHA-256.
Original artifact and logs are retained here.

Verified actual results:

- **14 preparation tests passed** in 0.513 seconds; the original test log ends OK.
- **16 build-wiring rejection cases passed**, plus PowerShell parsing/order,
  initial orchestration and exact same-job reuse. This is mocked orchestration,
  explicitly not a real private adapter compile.
- **38 distinct native loader groups passed**, counted individually in the
  executable log and cross-checked against its JSON and summary. The actual
  production loader/callback/fallback sources compiled under MSVC Win32 with
  locked wxWidgets 3.2.8 and ran using harmless DLL fixtures.

All 15 recorded native source hashes/sizes match the mapped local revision,
allowing only normal Git checkout CRLF conversion. The remote monorepo workflow
path maps to the local `.github` path. Six preparation/wiring implementation and
test files were additionally fetched directly at the exact remote SHA and
matched local Git bytes without normalization; their hashes are recorded.
The wx archive lock, picosha2 and read-only original vendor DLL identity agree
with the checked-in input locks. Native source and verification records are in
`verification.json`.

Native groups cover copied status, unavailable/Safe/non-main/Standard bypass,
wrong identities/sizes, missing or malformed modules/exports, path and junction
refusal, exclusive locks, changes after compatibility checking, binding/status
rejections, single-module ownership, callback/fallback and injected invalid
handle refusal. Expected Win32 error messages in negative cases are retained;
they are followed by their corresponding PASS groups. DLL event logs contain
no plugin factory calls.

The artifact intentionally excludes the test executable, runtime DLLs and tiny
fixture DLL bytes. Their hashes/PE identities are **runner-recorded receipts**,
not independently rehashed downloaded binaries. The compile commands/logs and
actual execution log were independently inspected. This distinction does not
turn the small guard fixture into full application acceptance.

Remaining gates: actual private adapter/import ABI and renderer, native private
wxCurl TLS, full application mode/lifecycle, encrypted charts/licensing, real
loaded-module unload refusal, final package/runtime and physical boat behavior.
The original o-charts DLL was hash input only; neither it nor its helper/factory
was executed. The unload negative uses a reserved invalid handle. No new tests,
builds, retries, CI dispatches or boat operations were performed during this
verification. The earlier failed candidate remains separate evidence.
