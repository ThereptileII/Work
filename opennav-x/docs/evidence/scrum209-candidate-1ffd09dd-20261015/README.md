# Native candidate 1ffd09dd: producer passed, installed qualification failed

SCRUM-209 / SCRUM-22 / SCRUM-212. Frozen product source is
`1ffd09ddf2fa6588def2907485f6e14e89bc3a7d`, version `0.4.0-beta2.4`,
[run 38722465573, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38722465573).
This later evidence commit does not rebuild or relabel that candidate.
The run is failed. No installed-product, full Staging, release or boat acceptance
follows from the successful producer components.

[Native producer 116388013514](https://github.com/ThereptileII/Work/actions/runs/38722465573/job/116388013514)
succeeded on Windows 2022 image `20261012.333.1`, retaining the Win32 application
and plugin ABI on Windows x64. SDK authentication, fresh native environment
reprobes, both application builds, native TLS/security checks, recovery packaging,
installer compilation and input sealing passed. Fixture and fixture-free builds
each passed the same **146 distinct CTest names**. AIS output records 251 session
checks, 18 TLS/transport scenarios and eight provider lifecycles. Public Downloader
and wxCurl each passed 12 cases; duplicate/private summaries are not added as
unique cases. Plugin SDK capture is retention only.

[Retained qualifier 116405193249](https://github.com/ThereptileII/Work/actions/runs/38722465573/job/116405193249)
restored the original sealed inputs without compilation. Its thirteen retained
offline component executables passed. The exact fixture-free recovery ZIP passed
its seven recorded package/startup/mode/preservation checks, with 1,046 files
verified. Earlier runtime reports remain scoped to their actual disposable
fixtures and retain any screenshot-review caveats. Design review was not
requested; no such acceptance is inferred here.

The native installer stops at the first ordinary fresh installation. Its original
[engine log](original/installer-engine/20261015-120956-441-Install.log) reports
`Invalid PE export directory` for `app/wxmsw32u_core_vc14x.dll` at
`2026-10-15T12:09:56.4377050Z`. The preceding corrected foreign Start-menu refusal
and both unmanaged stock legacy-TLS DLL refusals pass. The original
[installer report](original/installer-staging.json) retains all seven preceding
checks and the failed wizard result; later lifecycle and public ENC checks were
not reached, and candidate assembly was skipped.

The authenticated plugin-input archive and both compiled install trees contain
the same 7,467,008-byte wx core DLL, SHA-256
`beadbd9824a90d54e25f899afb9cf45611e7f74c9e0a0ceb6531e0bf6d21bdc3`.
Read-only header inspection finds export RVA 5,626,288 and size 1,158,912 bytes.
The candidate's `Lifecycle.ps1` rejects export sizes above 1,048,576 bytes.
This identifies the retained input and rejecting source boundary; changing that
installed engine requires separate focused native evidence and packaging.

The [receipt](native-candidate-receipt.json) binds official API run/job/artifact
metadata, original logs, archive sizes and SHA-256 digests, complete ZIP CRC
checks, selected member hashes and scope-specific outcomes. All 4,578 sealed
input entries and all 13,583 corresponding-source file hashes also match.
Recovery contents
also match all 17,035 inner manifest entries. All 9,180 retained product source
entries and 51 recipe entries match their exact candidate Git object references;
byte comparison explicitly accounts for Windows CRLF checkout conversion.
The 413 AIS local-input receipts independently match that same source. Patched
integrated OpenCPN source is separately retained with its dirty-source inventory;
prepackage recovery remains explicitly unqualified.

Original archives and API/log bytes are retained privately under
`/home/standard/Work/candidate-windows-1ffd-evidence-20261015`.
This inspection ran no application tests/builds, dispatched no CI and performed
no boat, physical-output, publication or private chart transfer action. The
separate Linux ENC readiness failure also remains open. Endurance remains skipped
by user direction. Source/license review and boat acceptance remain separate.

The separately reviewed parser correction (`b138b121`, integrated into harness
`f880ef8f3f20978e251bc9a281c85a194eacdc58`) then passed focused native
[run 38734026147](https://github.com/ThereptileII/Work/actions/runs/38734026147).
Both original x86/x64 artifacts are authenticated by run/source/size/digest and
all member CRCs. Actual Windows PowerShell `5.1.20348.5622` reports 32/64-bit
hosts, all three maintained fixture completion markers, and 28 imports from the
unchanged authenticated wx DLL. [Original receipts and exact scope](corrected-native-pe-receipt.json)
are retained. This is inert parser proof; it neither replaces the original failed
installer qualification nor creates a corrected package. No fixture assertion
count was emitted, so none is invented.

Final-package comparison finds 78 of the 82 prepackage PE files unchanged, four
replaced MSVC runtime DLLs and six additions including the updater launcher.
The ten changed/new PEs passed inert parsing with the corrected source under
Linux PowerShell 7.6.7; the unchanged 78 were not repeated. All ten hashes
independently match the authenticated final recovery ZIP. This local byte/parsing
proof is separate from the native Windows wx-DLL proof and installed acceptance.
