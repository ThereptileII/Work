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
