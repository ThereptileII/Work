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

## First native reproduction: rejected

The isolated probe publication `d7000af44ba4da0bda6ca4bac58c82cc51d9452f`
ran at [36886471275](https://github.com/ThereptileII/Work/actions/runs/36886471275).
The original lookup failure and exact source patch were confirmed. Patched
certificate generation hit its 180-second limit; output pipes then failed to
close within 10 seconds, so partial generator output was not retained. This is
not a certificate-generation pass. The host test tool was OpenSSL 3.6.4.
The [verified failure inventory](scrum-221-native-probe-failure.json) records
artifact identity and observed stages.

Subsequent source review found a separate new probe/producer argument error:
`Makefile.inc` lists `test-localhost.prm`; there is no `localhost.prm`. Both
invocations must use the real configuration and `test-localhost.crt/key`
outputs. That error is not yet established as the cause of the hang. Retain
partial output and bounded child-process observations before retrying; do not
increase timeouts or skip upstream certificate tests to obtain a pass.

## Native child trace — 2026-10-01

The corrected diagnostic commit `7897dd1aadbb1f299c226c1ceda01e9348f4665b`
failed in [run 36889948495](https://github.com/ThereptileII/Work/actions/runs/36889948495),
job `110462739693`. The downloaded artifact was verified at 5,907 bytes and
SHA-256 `f2e4cac8278ff3d6ee313cdb716b3378f4da0db34618666999077dd76fd2b399`.
See the [file inventory](scrum-221-native-pipe-failure.json).

The owned process trace at the unchanged 180-second deadline identifies
`openssl.exe x509 -in test-ca.raw-cacert -text -nameopt multiline`, called
from the generator's `redir` at line 100. That routine waits for stderr EOF
before reading stdout; Windows stdout-buffer saturation is therefore a concrete
hypothesis to test. Other branches do not drain hidden stderr at all. The
next repair uses direct duplicated file handles to preserve output semantics
without those pipes. Its native success is not established by this failure.

Output collection also hit a Windows sharing violation after killing the owned
process tree. The diagnostic reader now permits existing writer handles and
reads only a bounded snapshot. Raw output remains excluded from uploaded
artifacts; retained copies redact the generator's PATH line.

## Bounded native certificate probe passed

The follow-up isolated probe passed in run
[36891804473](https://github.com/ThereptileII/Work/actions/runs/36891804473),
job `110468992463`, at source commit `ded9bab3e4ae1600418846989e17ecddee337f2f`.
The downloaded artifact was verified at 8,543 bytes with SHA-256
`4ca753b281a1a302a652c9d53c564ab2444d2d68e95e0678be4ba614064bfedf`.
See the [bounded result inventory](scrum-221-native-certificate-pass.json).

The exact original generator failed as expected on its Windows `openssl`
filename lookup. The new pipe-free helper passed all five regression tests,
then generated the CA and `test-localhost` certificate/key in 717 ms. OpenSSL
parsed the CA, host certificate and private key and verified the certificate
chain. The corresponding-source test also passed using the locked archive and
packaged helper.

This probe used host OpenSSL 3.6.4; it did not qualify the pinned OpenSSL 3.5.9
producer, the full curl build/upstream test suite, application integration,
packaging, or boat acceptance. Earlier failed runs above remain historical
records and are not rewritten by this follow-up.
