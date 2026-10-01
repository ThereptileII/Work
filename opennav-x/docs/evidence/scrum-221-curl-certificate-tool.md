# SCRUM-221 — Native curl certificate-tool failure

The frozen source `f1e2cde8fcbf92826d648007b267cc0f5320aa55` progressed beyond
the corrected archive extraction step in native run
[36875830071](https://github.com/ThereptileII/Work/actions/runs/36875830071).
Both composition job `110414881055` and object-flow job `110414880809` failed
while building curl's test certificates, before application qualification.

Composition artifact `11171184345` was downloaded and verified against its
327,021-byte size and SHA-256
`2470cc26c8d3b8ab96d03058860bdcbe3976c530a081a9f706ceb0d7e06cb374`.
All 15 ZIP entries passed CRC, path and size-bound checks before extraction.
The local retained inventory is
`evidence/local/f1e2-native-curl-failure/verified-summary.json` in the main
checkout; raw logs are retained there, not published as product evidence.

The evidence establishes:

- OpenSSL's producer configured, compiled, tested and installed successfully.
  It copied `apps/openssl.exe` to its installation `bin` directory and captured
  successful OpenSSL 3.5.9 / library 3.5.9 / VC-WIN32 version output.
- curl found the explicit OpenSSL and zlib libraries and compiled its Win32
  executable and shared library.
- curl's `tests/certs/genserv.pl` then reported a missing or unsupported
  `openssl` tool, and the generated `build-certs.vcxproj` exited with code 2.
  The printed search path contains the pinned OpenSSL installation directory.
- The full integration job `110417629448` in run
  [36875827855](https://github.com/ThereptileII/Work/actions/runs/36875827855)
  subsequently failed at the same certificate-tool step; its complete log was
  inspected. All three application jobs are terminal failures.

## Confirmed source boundary and repair

The original curl 8.22.0 source archive was verified at 2,953,092 bytes and
SHA-256 `f7ef3ae8a22e521f289803fe93543eb64c329b58aa73a9e224dfd915a2a5f4f7`.
In `tests/certs/genserv.pl`, line 40 selects literal `openssl`. Lines 70–84
scan PATH using a regular-file check against that literal filename. Windows
has `openssl.exe`; this lookup does not add the extension. The retained log's
`PATH used:` diagnostic establishes that this branch failed before the later
OpenSSL version/key-generation commands. This is a test-tool filename defect,
not evidence that the compiled TLS library is broken.

The reviewed helper `tools/patch-curl-test-openssl.py` changes only that
selection to `openssl.exe` on Perl `MSWin32`; other platforms keep `openssl`.
It accepts only original script SHA-256
`d737cbe77e23e275b4fcfcec36e62d49d1d59d9d9fd0013a428b7143ee75c982`,
and verifies the exact resulting SHA-256
`a9aac30978a5c6c670aef643a337a7d2922e9d324f49fb663a41df29e6c44e54`.
Idempotence recognizes only those patched bytes. Four focused tests cover the
patch, idempotence, changed-source refusal and symlink refusal. Helper/test
commit: `b77193dab32ad46a3ceb9d194ad90ef4f11507d6`.

The upstream CMake certificate target still runs `genserv.pl test` against its
normal certificate configuration list. No certificate/TLS tests are skipped.
A short native probe reproduces the original failure and exercises patched
CA/localhost generation and actual certificate/key parsing/chain verification;
its host OpenSSL is a test tool, not pinned producer qualification. The actual
producer must separately bind and test its own pinned OpenSSL executable before
compilation. Native results and full replacement gates remain pending.

## Corresponding source

`tools/source_package.py` includes tracked project files and their hashes in
`SOURCE_REFERENCE.json`. `tools/curl_package.py` contributes the exact original
archive as `third-party-sources/curl-8.22.0.tar.xz`; the committed helper gives
the deterministic local patch recipe. Consumers can extract the original
archive and apply that helper to reproduce the modified test script. Upstream
archive/license provenance remains unchanged. This source-boundary review does
not substitute for release-package verification or legal review.

No application, package, Windows or boat acceptance is claimed from the failed
runs or local helper tests.
