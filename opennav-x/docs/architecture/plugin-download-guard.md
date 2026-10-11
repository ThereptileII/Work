# Plugin download failure propagation probe

SCRUM-211's plugin-specific gate exercises the pinned OpenCPN
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7` source with the existing download trust
patch. It changes no production code, plugin ABI, trust roots or archive policy.

## Boundary verified

`PluginHandler::InstallPlugin(PluginMetadata)` obtains a temporary path, calls
`Downloader::download(path)`, and returns its useful failure before the overload
which extracts the archive and writes installation records. This caller uses
Downloader directly, not wxCurl. Core and private o-charts wxCurl remain covered
by their separate real transport/native gates.

The probe compiles the whole patched Downloader plus twelve byte-identical
PluginHandler function bodies: both install overloads, archive extraction and
copying, error checking, and file/directory/version record writers and paths.
The complete patched source hashes gate extraction; every emitted function body
has a recorded digest. Source drift refuses preparation before compilation.
Real libcurl and libarchive perform the transfer and extraction. An observation
wrapper counts archive-reader allocations without substituting archive behavior.

Explicit test adapters replace application paths, temporary allocation, the
metadata container and logging. The destination mapper accepts only two inert
regular-file fixture entries. It does not test production Windows/Linux path
routing, plugin registration/loading, UI, metadata parsing or plugin ABI.
Unexpected archive-error rollback throws; rollback is not qualified here.

## Focused execution

From the product root, with C++17, pkg-config, libcurl/libarchive development
libraries, OpenSSL, Python and the pinned OpenCPN Git object available:

```sh
python3 tools/test-plugin-download-guard.py \
  --upstream /path/to/pristine/OpenCPN \
  --output /path/to/new-disposable-evidence
```

The output directory must not already exist. The harness creates owned loopback
TLS servers and a temporary private CA directory, never changing global trust.
Proxy settings are removed from probe children. Private keys are deleted on exit.
The Linux staging regression job now runs this gate before the application build.

The thirteen cases include direct trusted install and trusted HTTPS redirect
(two actual extraction/record-publication positives), direct and redirected
untrusted/expired/wrong-host certificates, HTTP downgrade, local-file redirect,
initial HTTP, missing CA, and an interrupted response containing an otherwise
complete extractable tar. Every negative requires zero archive allocations and
unchanged file contents, records, directories, symlinks and temporary state.
Invalid TLS endpoints receive no HTTP requests. A separate plain-HTTP control
proves the downgrade target is reachable; forbidden requests never reach it.

A second compiled probe deliberately bypasses the caller's failed-download
return guard. The same partial-transfer assertion rejects that mutation because
it enters archive installation. This prevents transport-only success from being
mistaken for caller-boundary coverage. The mutant is disposable test code only.

## Native preparation and remaining gates

`--prepare-only` emits verified source and slices without reporting test success.
`tests/plugin_download_guard` is a narrow MSVC Win32 CMake target consuming those
sources, maintained curl headers/import library, matching libarchive SDK, and
real wxWidgets. Required CMake inputs are `SOURCE_DIR`, `TOOLS_DIR`,
`CURL_INCLUDE`, `CURL_IMPORT`, `ARCHIVE_INCLUDE`, `ARCHIVE_IMPORT`, plus normal
wxWidgets root/library settings. Paths are normalized at the input boundary.
The target uses `/MD`, refuses other compiler/architecture combinations, and has
no `OPENNAV_DOWNLOADER_TLS_TEST` definition or Linux wx shim. The production
Windows native-CA branch and corrected `TrustProbeConsole` lifetime apply.

Native compilation and execution have **not** run for this probe revision. A
Windows runner must bind matching SDK/runtime bytes to the selected candidate,
provision and remove owned test roots in its disposable trust store, and execute
positive install and negative certificate/partial-transfer assertions. The older
separate package orchestration was not copied because its assumptions need fresh
candidate verification. Preparing a CMake target is not native acceptance.

## Recorded result

[Retained Linux evidence](../evidence/scrum211-plugin-guard-20261011/results.json)
records 13/13 cases and rejected guard-bypass mutation on 2026-10-11, against
product base `5dd3ac94`, libcurl 8.22.0 and libarchive 3.8.9. Source tampering was
also refused; focused CMake checks verified normalized inputs and rejection of a
non-MSVC/non-Win32 configure. The initial attempt's local socket permission
failure is retained beside the successful results; no TLS assertion ran in that
failed attempt. Native execution, installed routing/loading, installer/recovery,
source compliance and exact release acceptance remain open in Jira.
