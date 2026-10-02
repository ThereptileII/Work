# Plugin download guard regression (SCRUM-211)

The focused probe exercises the patched
`PluginHandler::InstallPlugin(PluginMetadata)` caller, its archive-install
overload, and the real libarchive extraction and installation-record writers.
It closes the gap between observing `Downloader::download` reject a transfer
and observing the plugin caller refuse to enter archive installation.

`tools/test-plugin-download-guard.py` reads three source files directly from
pinned OpenCPN commit `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`, applies the
existing download-trust patch to a new evidence directory, and checks the
resulting full-file SHA-256 values against reviewed constants. Any source or
boundary drift fails before compilation. Twelve function bodies are copied
unchanged into a generated include; each body's source line and hash is
recorded. No production source or patch is modified.

The probe links the actual patched Downloader and real libcurl/libarchive.
An observation wrapper counts archive allocations while delegating to
libarchive. The actual archive extraction guards and record-writing bodies
are unchanged. Fixture adapters supply the metadata container, logging,
temporary allocation and an isolated destination router accepting only two
known inert text entries. Unexpected archive-error cleanup throws and fails
the probe: that rollback branch is not simulated or qualified.

Three owned-loopback TLS cases use the existing Downloader CA-generation
helpers:

- A trusted complete inert tar must enter real extraction, replace an existing
  text payload, add another, and publish the actual file-list, directory and
  version records.
- An untrusted certificate must return the plugin download error, allocate no
  archive reader, leave installed files and all records byte-identical, and
  remove the owned temporary path.
- An interrupted HTTP response delivers the **same complete, extractable tar**
  but declares 19 additional bytes before disconnecting. It must satisfy the
  same refusal and preservation assertions. The successful case proves that
  these exact archive bytes are installable, so a damaged archive cannot
  accidentally explain the rejection.

Run only the focused Linux probe, supplying the existing pinned checkout:

```sh
python tools/test-plugin-download-guard.py \
  --upstream /path/to/pinned/OpenCPN \
  --output evidence/local/plugin-download-guard
```

The output directory must be new. It retains generated sources, source/body
hashes, executable, inert archive, case output, before/after file hashes and
`results.json`, including failed results. CA private keys remain in a temporary
directory and are removed; no global trust store changes occur. Each probe
process has a 30-second deadline. No plugin code or application is launched.

Linux uses the pre-existing test-only CA override and wx filesystem adapters;
this is not native Windows trust evidence. `--prepare-only` emits the reviewed
source slices without compilation or tests, for a later native MSVC Win32
probe using real wxWidgets and maintained installed curl/libarchive. That
follow-up must omit `OPENNAV_DOWNLOADER_TLS_TEST` and the Linux wx shim, and use
the existing disposable-runner native CA procedure. The platform-neutral C++
fixture is retained here for that follow-up; this change adds no native CI.

This qualifies the narrow download-to-archive caller guard and successful
archive/record publication in fixture destinations. It does **not** establish
complete installed PluginHandler integration, platform destination routing,
plugin ABI/loading/UI, archive-error rollback, native Windows, or boat/release
acceptance. Existing application, installer/recovery, source notices and
native trust gates remain required. Separate peer trust remains SCRUM-212.

## Focused Linux result

The 2026-10-02 focused run passed all three cases against host libcurl 8.22.0
and libarchive 3.8.9. The positive case opened one archive reader and wrote two
inert payload entries and all three records. Both rejected cases opened zero
archive readers, preserved every installed-file/record hash, removed the owned
temporary file, and returned the actual Downloader error through PluginHandler.
Retained local evidence is
`evidence/local/plugin-guard-final/results.json`, SHA-256
`eac4366e9e693bcb18a73530fce2d937787b2ca260b4e9d54b26f4b198ddf8b8`.
An earlier fixture-start failure and earlier successful run remain retained
alongside it. Native Windows has not run for this probe.

## Isolated native consumer

`tools/test-plugin-download-guard-windows.py` is restricted to GitHub-hosted
Windows. Its caller must first authenticate the exact candidate through the
GitHub run/artifact API, verify the outer archive and internal checksums, and
verify the installer/recovery counterparts. The peer candidate adapter's
`prepare()` already supplies this boundary. The caller passes its authenticated
manifest hash and candidate commit explicitly; a `verified: true` field is not
accepted as authority. This helper independently rehashes the supplied manifest,
all package files, and the fixture-free/status-only `PRODUCT_BUILD.json` policy,
before and after the probe. It never launches the packaged application.

```sh
python tools/test-plugin-download-guard-windows.py \
  --runtime-dir <verified-runtime-containing-app-and-docs> \
  --package-manifest-path <verified-package.json> \
  --expected-manifest-sha256 <parent-verified-sha256> \
  --expected-commit <exact-candidate-commit> \
  --evidence <new-evidence-directory>
```

The existing curl/OpenSSL/zlib manifests must bind their installed Win32 DLLs
and exact reviewed source identities. The helper does not call, relax, or
pretend to satisfy same-job dependency reuse. Official locked wxWidgets 3.2.8
SDK archives supply real headers/import libraries; both required wx runtime
DLLs must equal the candidate. The libarchive SDK is limited to four pinned
files at OpenCPN support commit
`e90cc5842b02d5502a549f0f90424e3e1614bf67` (resolved upstream `v0.5`), recorded
in `tools/windows-plugin-archive-sdk.lock.json`. Its `archive.dll` must equal
the candidate before linking its import library; the bundle's obsolete curl
is never fetched. Curl 8.22.0 public headers come from the locked source archive
and must individually equal the producer manifest.

When `--curl-import-lib` supplies the original producer library, its hash and
size must match; mismatch aborts. Otherwise the separately authorized fallback
uses MSVC `dumpbin` on the exact candidate DLL and `lib.exe /machine:x86` to
create a disposable import library. It accepts only a complete, unique table
of undecorated `curl_*` names; forwarded/unknown export shapes abort. Evidence
records the DLL, export listing/names, DEF file, both tool hashes and generated
library hash. This is explicitly a **probe-only reconstructed library**, never
the producer library or evidence authorizing dependency reuse.

Only the probe and Downloader translation units compile, with native MSVC
Win32 `/MD`. Neither the Linux wx shim nor the test CA macro is enabled.
Execution uses copied, hash-checked candidate DLLs and a reduced PATH. The
existing CA-generation and three-case logic are shared with Linux, but the
native caller supplies no CA override. `plugin-probe-owned-trust.ps1` checks
that its generated thumbprint was absent, records ownership before importing
into CurrentUser Root, and removes/verifies that exact thumbprint in `finally`.
Uncertain cleanup fails the result. Keys and temporary SDK/runtime files are
deleted afterward; diagnostic sources, hashes, logs, probe and generated
import-library evidence remain. No CI dispatch or native passing result is
implied by this implementation.
