# Curl import-library name: isolated repair

Frozen native candidate `0e9ec666ab27262a0c246f87f2e0fa5c7ede9fd0`, run
`36969350849`, passed all **1,569/1,569** reported upstream curl tests. Its
retained `windows-xnav-Win32.log` then records installation of
`install/lib/libcurl_imp.lib` and the producer's refusal because
`install/lib/libcurl.lib` was absent. This is a dependency installation-contract
failure, not an application crash or a failing upstream curl test.

The locked curl 8.22.0 archive has SHA-256
`f7ef3ae8a22e521f289803fe93543eb64c329b58aa73a9e224dfd915a2a5f4f7`.
Its `lib/CMakeLists.txt` (SHA-256
`8fa2d3c63eea16e1bf7f759c200b2134f18e7cc89b264089ceebc9901ea0250b`)
sets the default `IMPORT_LIB_SUFFIX` to `_imp` on Windows whenever shared
libraries are enabled and import/static extensions match (lines 64–71). This
applies even when the static library is disabled. The shared target consumes
that supported variable directly in `IMPORT_SUFFIX` (line 213).

The producer now explicitly supplies `-DIMPORT_LIB_SUFFIX:STRING=` while
retaining `BUILD_SHARED_LIBS=ON` and `BUILD_STATIC_LIBS=OFF`. The actual producer
therefore emits the already-contracted `libcurl.lib`; nothing is renamed and
there is no fallback to another library. Installed-output manifests, cache
mapping, package verification, receipt staging, and Downloader consumers keep
their existing truthful filename contracts.

`windows-curl-import-layout.cmake` observes the actual configured shared target
at the end of the top-level CMake directory. It requires native Win32,
shared-only output, the explicit empty suffix and exact target properties,
then generates the resolved import-library path. The producer checks that path
immediately after configuration and checks the nonempty built import library
before the long upstream suite. The same-job receipt binds this new observer
as a producer input.

The existing source diagnostic's `-ProductionOnly` mode adds a bounded native
proof: configure the **actual complete pinned curl source** once with its
default name and once with the explicit suffix override, using its existing
Schannel/no-external-dependency configuration. Compare the real generated
Release/Win32 project import paths and the observer's resolved path; retain
both project XML files and their hashes. This does not compile curl, OpenSSL,
zlib or the application, and does not replace the later upstream suite.

Local checks: the nine same-job reuse tests and fifteen receipt tests pass,
including refusal after modifying the bound observer;
the locked archive/source checks match the hashes above. Native naming proof
remains pending before another integrated build. The successful tests from
`0e9ec666` do not qualify this new producer configuration or a new candidate.
