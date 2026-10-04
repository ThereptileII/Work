# SCRUM-211: native console logger ownership

The focused Windows proof passes for remote
`13c51725e78193eee8def2073d2131467a2b22aa`, frozen local
`b00cf0b5b03ba79bbb0bb4b93d7fbd136ff3e12a`:
[run 37191145516, attempt 1](https://github.com/ThereptileII/Work/actions/runs/37191145516),
[job 111403399523](https://github.com/ThereptileII/Work/actions/runs/37191145516/job/111403399523).
This qualifies the narrow wx console ownership correction, not actual TLS.

`original.zip` is the unmodified artifact **11298917603**, 11,214 bytes,
SHA-256 `37cdddd754e40cc652948fba4e27dce9b52999ceef747e93aa6be95c36f649a0`.
`audit.json` records independent CRC, 26 safe unique regular paths, per-file
hashes and all six source comparisons against frozen Git blobs, with LF converted
to Windows CRLF checkout bytes. All match. The retained build command/log confirms
MSVC Win32, Release, `/MD`; the three wx 3.2.8 archive identities match the frozen
lock. Executable and runtime identities remain runner-reported because their
binary bytes are not part of this small artifact.

| Actual case | Observed result |
| --- | --- |
| Original no-init log | Own visible `Message` dialog containing `native-original-log`; stopped after the five-second bound; no after-log marker. |
| Old startup-log ownership | Observer destructor runs between before/after wx initialization. Explicit exit **87** rejects the now-dangling former ownership, releasing that pointer without dereferencing or deleting it again. |
| Fixed log/file path | Exit **0**; message/warning reach stderr, staging and rename complete, and logger deletion/wx cleanup complete. Hash-bound runner checks the payload and absence of partial files. |
| Fixed actual helper lifecycle | Exit **0**; all **16** iterations log, allocate additional strings, and record logger-delete begin/end, wx cleanup, then completed scope in exact order. |
| Fixed assertion | Exit **86** through the actual fail-fast stderr handler; assertion detail retained and no after-assert marker. |

The source basis is upstream [wxWidgets 3.2.8 `src/common/init.cpp`](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/common/init.cpp#L346-L351):
lines 244–249 preserve an already installed custom startup logger, while lines
346–351 delete the active logger after initialization. Cleanup also deletes an
active logger at lines 380–392. The raw source SHA-256 is
`f9fd740f3495d9fce34f80d58d3f6a05dae1dc72cac97061ff901874bc991ff8`.
The prior helper created and owned its logger before wx initialization, leaving a
dangling `unique_ptr`. The corrected helper initializes wx first, then creates its
owned logger, and detaches/deletes that logger before wx cleanup. Both real probes
also flush their final stdout fields before teardown without changing exit-code
acceptance or Downloader/curl behavior.

The [earlier console proof](../scrum211-native-console/README.md) remains valid for
its original modal/file/assert observations, but did not establish correct logger
ownership. Why that smaller program exited successfully despite invalid ownership
is unresolved; allocation reuse is only a hypothesis. This new evidence directly
observes the lifetime defect and corrected cleanup. It contains no access-violation
stack and does not identify the exact crash site of the d29 actual TLS probe.
Actual Downloader/wxCurl trust tests, native candidate/package gates and boat
acceptance remain separate. No application build, TLS call, rerun or boat operation
was performed to prepare this evidence audit.
