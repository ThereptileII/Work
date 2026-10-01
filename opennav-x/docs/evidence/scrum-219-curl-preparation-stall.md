# Native curl preparation stall — SCRUM-219

Observed 2026-10-01. Candidate:
`e9737d6bad9f3eb3db71877bce0816f59c62b0e7` (local mapped source
`f9396889740e142e6a5c622eaada3a892ec0e348`).

[Prototype run 36850600318](https://github.com/ThereptileII/Work/actions/runs/36850600318),
composition job `110331054362`, started 10:41:48 UTC and completed cancelled
13:42:19 UTC. Its native build step was cancelled; subsequent application,
security and UI gates were skipped. The always-upload evidence step succeeded.

Downloaded artifact `11166793278`, `prototype-native-Windows-e9737d6bad9f3eb3db71877bce0816f59c62b0e7`:

- ZIP size: 312,975 bytes.
- SHA-256: `1544cdd80ec2ce789175923f364da7c6c4743eda998b96d02df4fcefcc823766`.
- GitHub artifact metadata binds run 36850600318 and the candidate above.
- Size/digest and bounded archive paths were verified before extraction.

The actual logs establish:

- OpenSSL 3.5.9 main test harness: 347 files, 4,283 tests, `Result: PASS`,
  1,455 seconds. Its separate non-FIPS preparation harness reports a skipped
  FIPS-only setup with zero tests; this is not the main test result.
- OpenSSL build/install verification completed 11:26:21 UTC.
- zlib 1.3.2 CTest: 13/13 passed, 12.36 seconds; build/install verification
  completed 11:26:56 UTC.
- The curl native log contains only the prerequisite zlib DLL import report.
  Output does not prove that the dumpbin/PowerShell pipeline returned.
  The reviewed script's next operations validate that report, clear its own
  disposable curl build directories, extract the source, then invoke CMake.
- No curl CMake configure output appears before cancellation 13:42:14 UTC.
  The pinned curl CMakeLists emits its CMake version near its beginning.

The exact retained DLL-import text passed both producer runtime regular
expressions in a local PowerShell check (18 ms). The hash-pinned curl archive
was independently fetched and read on Linux: 4,444 entries, 22,239,154 bytes
unpacked, 38 directories and 4,406 regular files, no special entries, maximum
path length 69 characters. These observations narrow the investigation; they
do not prove which native preparation operation stalled.

The running-job log API previously returned an incomplete/stale prefix near
4.21 MB. After terminal completion, the same tool returned the complete
5,512,738-character log through cleanup. A fixed connector output-size cap was
therefore not established. SCRUM-218's unpublished additional collector was
deferred; existing retained artifacts and finalized logs provide the evidence.

SCRUM-219's separate native diagnostic workflow will record stage timestamps,
actual tool identities and bounded child-process results. It does not run or
qualify the application, alter a frozen job, install on the boat, or issue any
hardware command. Product qualification and the requested boat cleanup remain
open. No longer timeout or unverified retry is accepted as a root-cause fix.

The independently cancelled object/pointer job retained artifact `11166409726`
(`prototype-object-flow-Windows-e9737d6bad9f3eb3db71877bce0816f59c62b0e7`),
312,834 bytes, SHA-256
`33d7b6d8a21d01420489e35d856997fe1c0630900ba341e0a43dda9b8cb861ad`.
Its size/digest and bounded entries were also verified. Its curl log is likewise
575 bytes containing only zlib DLL imports. This confirms the same last visible
boundary in two disposable Windows jobs, not the underlying cause. The short
probe tests archive preparation independently; success would rule out that
reproduction only and would not qualify the original integrated build.
