# Native 15452e5: expired-certificate fixture preparation failure

[Run 37136712793](https://github.com/ThereptileII/Work/actions/runs/37136712793),
native job `111243795980`, failed in step 18. Exact local source is
`96c0c2705aacae511b6f8c22afaa6618dbabceff`, mapped to remote
`15452e512fd073090b1a4cea7c9010eac0874118`, tree
`3c53cff0332dfc09babb8e5b8daa7986f77a9c84`.

At 18:13:49 UTC, OpenSSL `ca` reported `Problem with index file: .../expired-index.txt
(could not load/parse file)`. `test-downloader-trust-windows.ps1:51` propagated its
exit 1; the actual fixture creation is at lines 94–98. The original production
transcript records the causal message at line 4137. `first-error.log` preserves
the timestamped completed-job excerpt; the full original production transcript
is retained as gzip without changing its bytes.

The preceding fixture and fixture-free builds each passed 139/139 tests. The
repaired private trust probe configured successfully, then both actual
`downloader-trust-probe.exe` and `wxcurl-trust-probe.exe` linked. Their identities,
source hashes and staged runtime hashes are in the original retained
`private-probe-prerequisites.json`. The failure occurs while generating the
expired certificate, before importing the owned CA, starting a TLS server or
running any trust cases. It does not show an application crash or failed TLS
acceptance assertion.

The frozen script initializes the CA index with `Set-Content -Value '' -Encoding
ascii`, which writes a blank line rather than a zero-byte file. This is the
narrow source-based explanation to reproduce before correcting it. The original
index bytes were not retained: the wrapper cleans its disposable work directory
in `finally`. This audit did not execute a replacement fixture or modify source.

Original artifact `11281990645` independently downloaded: 65,648,702 bytes,
SHA256 `1fa9660cbf99b68893cd96b49c6fee22d92dd9f4a4d0d39f3660f970cba43f3b`.
All 18,441 entries passed ZIP CRC checks. Original archive:
`/home/standard/Projects/X-nav-worktrees/scrum273-full-154-watch/.local/windows-integration-154.zip`.
Extracted evidence is in the sibling `.local/failure` directory. The archive
contains no `opencpn.exe`, trust-probe executable, maintained runtime closure or
installer; six CMake compiler-identification executables are not replay payloads.

The retained five-export private DLL/package passes the exact frozen package
verifier against actual production resources. The corresponding-source archive's
trust script matches frozen source after exact newline normalization. These are
static artifact checks only: real-host module loading was skipped. DPI and public
ENC steps completed independently, but both requested software and OpenGL phases
report `opengl_enabled=false`; native GL remains unqualified.

No functional early package was assembled. Product recovery, installer, native
endurance and real-host private-module gates were skipped after the failure.
There is no eligible boat payload, no TLS acceptance and no release qualification.
The independent Linux elapsed-time job continues on this exact source; no final
Linux result is asserted here. No rerun, publication or boat action was taken.
