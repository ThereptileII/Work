# Maintained Win32 zlib source build

`tools/build-zlib-windows.ps1` is a standalone source builder for the
maintained shared zlib prerequisite used by the Windows curl path. The
integrated Windows build calls its `-VerifySourceOnly` mode before expensive
native dependency builds, then runs the normal full builder and upstream tests.

The builder is pinned by `tools/windows-zlib.lock.json` to the zlib 1.3.2
release asset published by the upstream Mark Adler repository at
[`madler/zlib` v1.3.2](https://github.com/madler/zlib/releases/tag/v1.3.2).
The release asset `zlib-1.3.2.tar.gz` is 1,502,830 bytes and has SHA-256
`bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16`. The
lock retains the reviewed signing fingerprint `5ED46A6721D365587791E2AA783FCD8E58BCAFBA`.

The prior zlib.net URL returned an 11,975-byte object in native CI instead of
the reviewed 1,502,830-byte archive (observed SHA-256
`d5b43f44bbe15f74bae56414ae917f7b2ed13ab0df81bff0b845a100efda5681`). The
response body was not characterized. A direct fetch of the upstream release
asset was verified as 1,502,830 bytes with the locked SHA-256. The source lock
test pins this exact release URL and identity; the native source-only job
still downloads it and enforces the same byte-count and digest checks before
any extraction or build.

The source-only mode uses the same archive download and SHA-256/byte-count
guard as the full build and exits before extraction, Visual Studio discovery,
or compilation. Both modes write
`evidence/local/windows-zlib-1.3.2/source-verification.json` with the reviewed
and observed hash and size. A changed, truncated, missing, or failed download
remains a failure; its evidence contains identity metadata, never archive
contents or credentials. The short native CI source job exercises the real
download path and deterministic rejected bodies before any full build rerun.

On a native Windows host with the licensed MSVC x86 tools, the script invokes
`vcvarsall.bat x86`, configures CMake with the Visual Studio generator and
`CMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL`, builds the shared target, runs
the upstream CTest suite with `--no-tests=error`, and installs into its private
build prefix. It does not claim native acceptance from Linux parsing.

zlib 1.3.2's CMake defaults produce `z.dll`. The generated wrapper adds the
verified source tree and sets the target's `OUTPUT_NAME` to `zlib1` before
CMake generation/build. This preserves the import-library identity instead of
renaming a DLL after link. The resulting contract is `zlib1.dll` with the
upstream Win32 export-by-name and CDECL interface described in
`win32/DLL_FAQ.txt`.

The script proves `/MD` twice: it inspects generated MSBuild
`RuntimeLibrary=MultiThreadedDLL`, and it runs `dumpbin /DEPENDENTS` on the
installed DLL, requiring `VCRUNTIME140*.dll` plus a UCRT import
(`api-ms-win-crt-*.dll` or `ucrtbase.dll`). It also checks the installed
header's real `ZLIB_VERSION`, PE machine type, DLL flag, and exact output
hashes and byte counts.

The success manifest contains exactly the contracted top-level fields:
`schemaVersion`, `library`, `version`, `configuration`, `architecture`, `abi`,
`runtime`, `source`, `buildSteps`, and `outputs`. It is written only after all
checks pass. No legacy DLL is removed, and no application, installer, boat, or
stock integration is performed by this helper.

The script was parsed successfully with the repository PowerShell runtime at
`/home/standard/Projects/X-nav/.local/pwsh/pwsh`. After loading the main
repository tool environment, the generated wrapper was configured and built
with Ninja against the pinned extracted source, and CTest discovered and passed
all 14 upstream tests. A second configure/build with the wrapper located under
`/tmp/zlib wrapper & check` passed as well, covering spaces and `&` in the
wrapper path. Logs were retained under `/tmp/zlib-wrapper-check/` for this
review. These are meaningful Linux cross-platform checks and do not qualify the
native Windows build.
