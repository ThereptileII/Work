# SCRUM-209 installer PE32+ closure proof

The focused installer PE suite passed under native Windows PowerShell
`5.1.20348.5622`, in both 32-bit and 64-bit hosts, on the first attempt of
[run 38740580580](https://github.com/ThereptileII/Work/actions/runs/38740580580).
The tested integrated commit is `6786a1e5b17906f87957c32ffa63704cd6147a55`;
its four changed files are byte-identical to implementation commit
`11a72f5a739170555fbbf14dc1b6f70d62604451`.

The shared reader now handles bounded I386/PE32 and AMD64/PE32+ metadata.
All retained AMD64 DLLs receive the same import, forwarder, known-byte and
legacy-name checks. Each dependency edge must match its importer, including
already visited files. An AMD64 helper uses its own application directory;
the parent application's x86 runtime cannot satisfy its imports. The finite
VC14 exception requires an installed native Microsoft runtime with matching
architecture and a valid Microsoft Authenticode signature. The x86 host ABI
and its application-local runtime policy remain unchanged.

| Host | Job | Result |
| --- | --- | --- |
| x86 Windows PowerShell | [116440929670](https://github.com/ThereptileII/Work/actions/runs/38740580580/job/116440929670) | Passed |
| x64 Windows PowerShell | [116440928900](https://github.com/ThereptileII/Work/actions/runs/38740580580/job/116440928900) | Passed |

[native-receipt.json](native-receipt.json) records authenticated original
attempt/job identities, both downloaded artifact digests, CRC checks, member
hashes, original job-log hashes and the exact pass lines. Both native hosts
passed real system-directory/kernel32 architecture checks and Microsoft
Authenticode checks for `msvcp140.dll`, `vcruntime140.dll` and
`vcruntime140_1.dll`. The 32-bit host exercises the `Sysnative` branch and the
64-bit host exercises `System32`; this is established by the verified host
bitness, exact source and passing assertions. Individual CRT identity log
entries are captured internally by the fixture, not printed in its stdout.

Both jobs also authenticated and inertly parsed the original wxWidgets DLL
from successful producer job `116388013514`, run `38722465573`, attempt 1,
product commit `1ffd09ddf2fa6588def2907485f6e14e89bc3a7d`. Its 7,467,008 bytes
retain SHA-256 `beadbd9824a90d54e25f899afb9cf45611e7f74c9e0a0ceb6531e0bf6d21bdc3`.
The original failed application qualification remains failed; this focused
run neither rebuilds nor accepts that product.

[local-verification.json](local-verification.json) records the preceding Linux
PowerShell 7.6.7 suite pass and private exact-byte parser evidence: 14 retained
helper files plus three native CRT files, all unchanged. Their graph has 17
reachable AMD64 files and 169 import edges; 20 Windows/API-set names remain the
OS resolution boundary. Private binaries, machine paths and original private
observations are excluded from this repository.

The focused fixtures retain malformed-input, work-budget, obsolete-byte,
legacy-import, transitive-forwarder and architecture-mismatch refusals. They
also reject unsigned/missing native VC14 files, helper-local wrong-architecture
copies, out-of-family runtime names and high-image-base delay-VA truncation.
Portable signature-observation negatives supplement the actual native signature
checks; they are not presented as native observations.

No application build, installer transaction, retained-helper execution or
installed-product acceptance occurred. In particular, parsing external-helper
OpenSSL 3.5.0 does not qualify that version or its source/provenance. The actual
retained-plugin installation lifecycle and those security decisions remain
separate gates.
