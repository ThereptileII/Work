# Windows maintained dependency integration

SCRUM-209 connects the reviewed OpenSSL 3.5.9, zlib 1.3.2 and curl 8.22.0
source builders to the disposable Win32 integration build. It does not change
the pristine build, package an installer or modify an installed OpenCPN tree.

After the pinned upstream dependency bootstrap,
`build-pristine-windows.ps1 -Integration` builds OpenSSL, then zlib, then curl.
The zlib headers, import
library and DLL are copied into the disposable `cache/buildwin` only after
their hashes and byte sizes match the zlib producer manifest. The curl builder
then independently verifies both dependency manifests and inputs, builds and
tests curl, proves its Win32 import closure with `dumpbin`, and publishes the
matching full header set, import library and DLL to that cache.

The narrow `model/cmake/Curl.cmake` patch preserves the pinned imported-target
layout. It removes only the legacy CA bundle and OpenSSL 1.0 DLLs from that
disposable tree's CMake install list. The maintained curl manifest must already
show imports of `libssl-3.dll`, `libcrypto-3.dll` and `zlib1.dll`, and reject
`ssleay32.dll` and `libeay32.dll`, before application CMake is invoked. The
existing pinned CMake rules install the OpenSSL 3 DLLs and zlib DLL; the curl
rule installs the maintained libcurl DLL.

After installation, orchestration copies the three producer manifests into the
install root and rehashes all four installed DLLs against their manifest
records. It fails if either legacy TLS DLL appears. It never deletes legacy
DLLs from a user installation; package upgrade, rollback and stale-file removal
remain separate installer work.

Source identities remain in `tools/windows-openssl.lock.json`,
`tools/windows-zlib.lock.json` and `tools/windows-curl.lock.json`. Native output
is retained by the producer scripts under `evidence/local`, including
`windows-openssl-native-output.log`, the zlib 1.3.2 evidence directory and
`windows-curl-native-output.log`; the outer integration transcript and curl
orchestration log retain sequencing and failures.

This change has PowerShell syntax and patch-stack validation on Linux only.
Native MSVC compilation, upstream producer tests, application link/import
closure, runtime TLS, installer and release acceptance still require native
Windows evidence. SCRUM-211 separately governs application certificate and
redirect policy; changing the source-built dependency alone does not establish
secure download behavior.
