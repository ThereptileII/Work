# SCRUM-259 optional private adapter in the existing Windows build

`tools/build-pristine-windows.ps1 -PrivateOCharts` opts into the private adapter;
it remains off by default. The switch requires `-Integration` and the disposable
`windows-integration` Actions job. No workflow is changed by this increment.

The adapter step follows successful maintained OpenSSL/curl/zlib producer or
same-job reuse verification and cache checks, before the integrated OpenCPN
configure. It generates the current chart resources from that same pinned
integration source, prepares the locked original source/patch/SDK tree, builds
only `skager-ocharts-adapter` with native MSVC Win32, and generates the closed
three-file package. It consumes the existing same-job prefixes directly; it
neither rebuilds maintained dependencies nor substitutes plugin vendor binaries.

The host configure receives `SKAGER_OCHARTS_PACKAGE` explicitly, including an
empty value when the opt-in is absent so an older CMake cache cannot silently
retain the adapter. The host consumer independently verifies matching generated
resources and package identity during configure/build/install.

The production invocation must repeat `-PrivateOCharts`. With the existing
`-Production -Integration -ReuseVerifiedDependencies` guard, the private path
requires the first pass receipt from the same run, attempt, job, source commit
and orchestration script. It checks the exact preparation receipt and all three
package payloads, independently re-derives the prepared source/patch inputs,
compares SDK manifests with current same-job producer manifests, and hashes
every SDK and producer output again. Finally it checks package source/resource
identity against freshly generated current resources. Only these successful
checks permit skipping the private CMake rebuild. Missing bytes, changed input
or a different selection fail closed; a receipt alone cannot supply a prefix.
Fresh builds refuse existing prepared/native/package output paths.

After the production application is installed and the existing test gates pass,
the additional private wxCurl TLS harness runs with `-OChartsPrepared`, the same
integration source, and `production-install`. Its evidence is separate from the
unchanged core harness. All original build/painter/CTest/mode/safety paths remain;
there are no test skips or full-workflow edits in this change.

Focused local validation: the actual PowerShell parser accepts the script;
`tools/test-ocharts-build-wiring.ps1` executes the actual orchestration and hash
functions with only native commands mocked. It proves first-build orchestration,
reuse without CMake calls, dependency-before-host ordering, explicit package
selection, and sixteen refusal cases covering unsupported context, stale output,
job/source drift, package/preparation/SDK/producer mutations, missing SDK bytes,
and source/resource validator failures. This is a local orchestration proof,
not a Windows adapter build, private TLS pass, or product/boat qualification.
The first real native private adapter build remains required.
