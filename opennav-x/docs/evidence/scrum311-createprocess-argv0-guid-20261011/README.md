# SCRUM-311: application/argv0 and held-file GUID diagnostic

This diagnostic-only revision extends coordinator baseline `afa5fbcf33cbf93f0f6c3f74fd72acf0b22086bf`. Production updater source and workflows are unchanged. Native execution of this revision is pending; the portable receipts are not Windows acceptance.

The earlier native run [38165504516](https://github.com/ThereptileII/Work/actions/runs/38165504516) observed all seven DOS application/argv0 pairs succeeding, including GLOBALROOT CWD, and all three GLOBALROOT application/argv0 pairs failing with error 87 regardless of CWD. Application and argv0 were coupled, so this does not identify which caused failure. Root retains that run's raw evidence separately.

The original ten row definitions remain byte-identical. Three crossed argv0 controls retain GLOBALROOT CWD, owner Unicode environment and flags `0x404`: GLOBALROOT application with basename argv0; GLOBALROOT application with physical DOS argv0; and physical DOS application with GLOBALROOT argv0. These first thirteen observations are retained even if subsequent GUID derivation refuses.

Four additional rows use a volume GUID application derived only from the held file using `GetFinalPathNameByHandleW(VOLUME_NAME_GUID)`: GUID argv0 with true NULL, DOS or GLOBALROOT CWD; then GLOBALROOT argv0 and GLOBALROOT CWD. The last row directly probes an application-only change. No GUID CWD or mutable DOS retry is introduced.

The helper strictly validates GUID grammar and the exact held NT suffix, reopens the GUID volume root and image, checks volume/file identity, native names, type, reparse/hardlink policy and size, and keeps both reopened handles plus the original physical ancestry/file pins alive through the cases. It revalidates before each GUID case. These handles do **not** pin a namespace alias (`namespaceLinkPinned=false`); this is a compatibility probe, not a proof that volume GUID naming is immune to local shadowing.

Every child remains suspended, with no resume invocation. Successful calls are checked against the held NT image before terminating only their owned raw child handle and waiting at most ten seconds. Ambiguous ownership, cleanup, namespace or source identity refuses further cases. Existing hosted-repository/PS5.1/non-LocalSystem guards, exact owned alias-stack removal, immediate Win32 error capture and source hashes remain.

Focused Linux checks passed: exact embedded owner/helper compilation; canonical GUID/NT validation; thirteen malformed-name refusals; missing suspended-flag refusal before native calls; a managed mock proving true NULL CWD and independently quoted argv0; and non-Windows refusal with zero rows/alias mutations. `portable-compile.json` and `portable-refusal.json` are byte-exact reports. `receipt.json` records source hashes and limits. No native Windows API or child creation was run locally.

Existing CLI remains:

```powershell
& ./opennav-x/tools/diagnose-native-update-createprocess.ps1 -EvidenceDirectory ./opennav-x/evidence/local/native-update-createprocess
```

`-CompileOnly` performs the managed compilation check without native calls. Native completion requires seventeen uniquely named rows and the named physical DOS baseline; observed API failures remain diagnostic observations, not a product pass.

Primary API contracts: [GetFinalPathNameByHandleW](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-getfinalpathnamebyhandlew), [CreateProcessW](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-createprocessw), and [volume naming](https://learn.microsoft.com/en-us/windows/win32/fileio/naming-a-volume). These contracts do not establish this machine's GUID CreateProcess compatibility; the native matrix must measure it.
