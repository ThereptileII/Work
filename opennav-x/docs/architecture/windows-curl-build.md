# Windows curl source-build boundary

SCRUM-209 introduces a bounded native build script for curl 8.22.0. It does
not yet alter the OpenCPN build pipeline, installer or retained dependency
bundle. `tools/build-curl-windows.ps1` only writes into its owned build roots
and a caller-selected disposable integration source.

The script accepts already verified OpenSSL and zlib prefixes. OpenSSL must be
the project's OpenSSL 3.5.9 Win32 shared build, including its build manifest.
The zlib producer must provide schema version 1 with exactly these top-level
keys: `schemaVersion`, `library`, `version`, `configuration`, `architecture`,
`abi`, `runtime`, `source`, `buildSteps`, and `outputs`. Its identity is zlib
1.3.2, `Win32`, `x86`, `Win32 shared`, and `MultiThreadedDLL (/MD)`. Its
reviewed source identity is also checked. `outputs` must bind SHA-256 and byte size for
`include/zlib.h`, `include/zconf.h`, `lib/zlib1.lib`, and `bin/zlib1.dll`.
The curl script rehashes every dependency input before configuring CMake and
independently checks the zlib PE imports for the modern dynamic VC/UCRT.
It likewise rehashes both OpenSSL runtime DLLs, checks their Win32 machine
type, requires all four OpenSSL build stages, and matches the OpenSSL manifest
source fields to the reviewed project lock before executing dependent code.

The maintained output is a Win32 shared `libcurl.dll` and matching
`libcurl.lib`, built with the dynamic MSVC runtime and OpenSSL 3.5.9. The
build explicitly disables opportunistic TLS, HTTP/2, SSH, PSL, Brotli, Zstd,
c-ares and GSSAPI dependencies. It retains HTTP(S), FTP(S), Telnet, the old
form API and the broader built-in protocol set used by pinned OpenCPN/WXCURL.
The complete generated curl public-header directory replaces the disposable
integration cache header directory; mixing old headers with the new import
library is forbidden.

The upstream CMake `tests` target runs `tests/runtests.pl -a`, using curl's
local test servers. Its actual stdout is retained in
`evidence/local/windows-curl-native-output.log`; the manifest binds that log's
hash and executed/passed counts. Exit zero without a nonzero `TESTDONE` report
is rejected. The manifest is written only after that target, install,
version/protocol checks, Release/Win32-specific `/MD` inspection, PE x86
checking, and `dumpbin /DEPENDENTS` confirmation. The DLL must import `libssl-3.dll`,
`libcrypto-3.dll` and `zlib1.dll`, and must not import `ssleay32.dll` or
`libeay32.dll`.

This increment does not patch `model/cmake/Curl.cmake`, install the new files,
or remove legacy DLLs. Those operations require a clean native dependency
closure, package checks and installer rollback behavior.

Certificate handling remains a separate security gate in SCRUM-211. The curl
build avoids embedding a build-machine CA path and disables unsafe PATH CA
search. OpenCPN currently has call sites which disable peer verification, and
WXCURL uses a relative `curl-ca-bundle.crt` path. A source build does not repair
those application behaviors. TLS acceptance requires an owned CA-bundle policy,
absolute runtime resolution, peer and hostname verification, negative-certificate
tests, and endpoint compatibility evidence.

Native Windows MSVC compilation and the real upstream test target remain the
authority. PowerShell parsing and schema/path checks on Linux only validate the
boundary definition.
