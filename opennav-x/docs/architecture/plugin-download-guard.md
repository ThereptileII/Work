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
Windows runner must execute the orchestration below with authenticated matching
SDK/runtime inputs and retain positive/negative and owned-root cleanup evidence.
Preparing the target or passing portable orchestration tests is not native
acceptance.

## Recorded result

[Retained Linux evidence](../evidence/scrum211-plugin-guard-20261011/results.json)
records 13/13 cases and rejected guard-bypass mutation on 2026-10-11, against
product base `5dd3ac94`, libcurl 8.22.0 and libarchive 3.8.9. Source tampering was
also refused; focused CMake checks verified normalized inputs and rejection of a
non-MSVC/non-Win32 configure. The initial attempt's local socket permission
failure is retained beside the successful results; no TLS assertion ran in that
failed attempt. Native execution, installed routing/loading, installer/recovery,
source compliance and exact release acceptance remain open in Jira.

## Reviewed native orchestration entry point

`tools/test-plugin-download-guard-windows.py` now orchestrates the narrow target.
The manually dispatched `skager-plugin-download-retained.yml` runs it after
authenticating original capture and helper sources. It refuses execution except on a disposable
GitHub-hosted Windows runner for `ThereptileII/Work`. This is a source-slice probe,
not an installed application, plugin loader, release or boat acceptance test.

The caller supplies two independent authorities:

* `--dependency-bundle` and `--dependency-bundle-provenance`: the existing
  authenticated maintained OpenSSL/zlib/curl bundle and its external provenance.
  Restore it at its required workspace paths first. The runner verifies it and
  repeats the native toolchain reprobes without rebuilding the product.
* `--retained-root`, `--retained-inputs`, `--retained-inputs-sha256`, and
  `--candidate-commit`: candidate SDK/runtime bytes plus a closed JSON inventory.
  The SHA-256 pin must come from independent review of the candidate evidence,
  not from a hash supplied by the same downloaded artifact. Producing or
  authenticating that external candidate pin is a prerequisite, not something
  this runner manufactures. It additionally compares curl SDK and maintained
  TLS runtime bytes with the authenticated dependency producer.

For a retained candidate whose original SDK image differs from the newly
allocated host, the workflow uses two explicit roles. This is a retained-test
policy only; application/dependency build reuse remains governed by the
unchanged strict current-image bundle verifier.

* The **host SDK** still passes normal authenticated fetch, restore, complete
  verification and fresh native toolchain/OpenSSL reprobes on its exact current
  image. Nothing changes the runner environment to impersonate another image.
* `--subject-dependency-bundle`, `--subject-dependency-bundle-provenance` and
  `--retained-capture` must be supplied together. The original capture selects
  only an already reviewed immutable entry in the existing two-image lock.
  GitHub independently authenticates that original producer run, successful job,
  artifact ID/digest and manifest. The subject SDK is never restored or staged
  into the host SDK workspace.
* `plugin_subject_sdk.py` requires the original subject runner identity, complete
  closed payload, source fingerprint/ABI, original toolchain digest, successful
  upstream logs and original producer/package assertions. Subject and host must
  have identical Python identity, all 30 source inputs, all five complete native
  tool-fact documents, and all headers/import libraries. Actual CMake-selected
  compiler, dumpbin, CMake, Python and PowerShell paths/hashes are checked against
  those records before native compilation and hashed again afterward.
* Original curl receipts contain absolute producer paths. The subject helper
  validates both dependency prefixes and the certificate executable against
  that original workspace before making a temporary validation view. Exactly
  those three paths in a copy of `curl-build.json` point to the authenticated
  subject payload, allowing the unchanged package validators to check the
  original output hashes, dependency manifest hashes, PE ABI and certificate
  executable. This derived view is deleted and never becomes artifact authority
  or probe input. Original manifests, files and runtime bytes remain unchanged.
* Retained curl headers/imports and TLS DLLs bind to the **subject**, never the
  host's alternative DLLs. The native probe compiles against retained inputs and
  executes the exact retained runtime. The subject's own authenticated
  `openssl.exe` runs the existing `version -a` identity verifier with explicit
  subject configuration/module paths before generating disposable certificates.
  Both original authorities and all consumed subject bytes are rechecked after
  the unchanged 13 cases, guard-bypass mutant and owned-root cleanup.

The workflow authenticates the new helper before import. Its only additional
existing-source exception pins the exact original e80 runner blob and reviewed
replacement blob; it does not permit arbitrary assertion edits. Existing probe
exceptions, external capture inventory pin, source-slice verification, Win32
`/MD`, import closure and trust cleanup remain mandatory. Results separately
record host authority, original subject authority and fresh subject OpenSSL
observation; none qualifies the final installed application or boat.

Run `python3 tools/test-plugin-subject-sdk.py` for focused inert subject-policy
and path-view negatives, alongside the existing portable orchestration tests.
These checks do not execute Windows binaries. Failed retained run
`38756849052` attempts 1 and 2 remain failed: they correctly refused e80's old
`libcurl.dll` against the newly selected host SDK before reaching native cases.
A fresh exact-candidate native run remains required after this orchestration fix.

The retained inventory has exactly these top-level fields:

```json
{
  "schema": 1,
  "candidateCommit": "<exact 40-character candidate commit>",
  "inputs": {
    "curlInclude": "sdk/curl/include",
    "curlImport": "sdk/curl/lib/libcurl.lib",
    "archiveInclude": "sdk/archive/include",
    "archiveImport": "sdk/archive/lib/archive.lib",
    "wxRoot": "sdk/wx",
    "wxLib": "sdk/wx/lib/vc14x_dll"
  },
  "runtime": {
    "libcurl.dll": "runtime/libcurl.dll",
    "archive.dll": "runtime/archive.dll",
    "libssl-3.dll": "runtime/libssl-3.dll",
    "libcrypto-3.dll": "runtime/libcrypto-3.dll",
    "zlib1.dll": "runtime/zlib1.dll",
    "wxbase32u_vc14x.dll": "runtime/wxbase32u_vc14x.dll",
    "wxmsw32u_core_vc14x.dll": "runtime/wxmsw32u_core_vc14x.dll",
    "msvcp140.dll": "runtime/msvcp140.dll",
    "vcruntime140.dll": "runtime/vcruntime140.dll"
  },
  "files": {
    "<each file below selected input directories, imports and runtime paths>": {
      "bytes": 123,
      "sha256": "<64 lowercase hexadecimal characters>"
    }
  }
}
```

Additional required runtime DLLs belong in `runtime` and `files`; the example is
not a claim that nine DLLs close every candidate. All paths are root-relative
forward-slash paths. Missing, extra, changed, case-aliased, linked or reparse
inputs fail verification. The runner copies only verified inputs into its owned
workspace, verifies the copy, checks actual generated source/import settings,
requires MSVC x86 `/MD`, audits native DLL dependencies, and records tool,
executable and runtime hashes. It checks the inputs and runtime again afterward.

For example, in that authorized disposable Windows job:

```powershell
python tools/test-plugin-download-guard-windows.py `
  --dependency-bundle <authenticated-bundle> `
  --dependency-bundle-provenance <external-provenance> `
  --retained-root <candidate-sdk-and-runtime-root> `
  --retained-inputs <reviewed-inventory.json> `
  --retained-inputs-sha256 <independently-reviewed-inventory-hash> `
  --candidate-commit <exact-candidate-sha> `
  --upstream <pinned-OpenCPN-git-checkout> `
  --patch-program <patch-executable> `
  --output <new-evidence-directory>
```

The generated CA is unique to the owned temporary directory. The trust helper
refuses an already present certificate, atomically records the previously absent
certificate identity and run before import, and removes only that exact owned
identity. Import/removal are bounded; final cleanup retries after partial failure.
The public root and ownership receipt are retained, private keys are deleted,
and a cleanup failure fails the run. No test trust override is compiled into the
native executable. Probe children use only their verified runtime plus System32
on PATH, with proxy and CA environment overrides removed.

The native matrix has the same twelve shared positive/negative cases and the
failed-download guard-bypass mutant. The thirteenth case removes the owned
system root and proves the previously valid endpoint now fails before any HTTP
request or archive allocation. It replaces Linux's missing injected-CA case,
which is not applicable to the real Windows native-store branch. All other
failure assertions remain unchanged. Logs, exact prepared source, generated
projects, import lists, external input inventory and results are retained.

Portable contract tests run with
`python3 tools/test-plugin-download-guard-windows-tests.py`. They validate the
orchestration's input boundaries and shared receipt behavior; they do not execute
Windows trust operations or establish native acceptance. Native compile/runtime,
owned-root cleanup under real Windows faults, matching candidate SDK provenance,
installed routing/loading, installer/recovery and release qualification remain
required before SCRUM-211 can be accepted.

To restore an already authenticated downloaded bundle at its original Windows
workspace location (after the trusted CI downloader has checked GitHub run/job
success and artifact ID/digest, and written its provenance outside the artifact):

```powershell
python tools/windows_dependency_bundle.py restore --root <product-root> `
  --bundle <unpacked-bundle-directory> --provenance <external-provenance.json>
```

The verifier deliberately rejects different runner/Python identities, workspace
relocation, source/recipe fingerprints and toolchain drift. A refusal requires
fresh compatible producer evidence; do not create a replacement provenance file
from the artifact's own assertions. No maintained producer job is run by this
new helper.

To prepare the separate candidate inventory, identify the exact candidate run
and artifact through GitHub's API, verify its digest, and retain that evidence
outside the artifact. Obtain the matching curl, libarchive and wx SDKs from the
candidate's reviewed build inputs (a runtime-only archive is insufficient).
Populate the six `inputs` paths and every required `runtime` DLL mapping above.
Use `windows_dependency_receipt._inventory` on those selected directory/file
roots to populate `files`, preserving bytes and SHA-256 for every selected file.
Review that inventory against the candidate build/SDK provenance, ABI and runtime
artifact; only then pin `(Get-FileHash <inventory.json> -Algorithm SHA256).Hash`
in the trusted invocation. This is an explicit review prerequisite: neither a
self-authored inventory nor this helper can prove wx/libarchive provenance alone.
