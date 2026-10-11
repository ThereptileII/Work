# Independent Claude / Sol / native comparison — 2026-10-11

Codex GPT-6.1 Sol compared three completed, read-only Claude reviews with
retained source identities and native results. Claude supplied independent
static analysis; Codex owns disposition. No review, native execution, build,
network request or external transfer was initiated by this evidence task.
Only this new evidence directory is changed. `comparison.json` records exact
receipt/review hashes, selected input paths/hashes and the derived native audit;
raw review events, transcripts and source snapshots are not copied.
Inputs were the coordinator's existing `.reviews/<review-name>/receipt.json`
and `review.md`, plus `opennav-x/evidence/local/native-owner-38166517938/`.

## Review identity and scope

All receipts report model **`claude-opus-5`**, observed models
**`["claude-opus-5"]`**, Claude Code **`2.1.292 (Claude Code)`**, status
**`completed`**, and scope **"selected working-tree bytes; not full-repository or
test acceptance"**. These are frozen selected bytes, not full-commit reviews.

| Completed review | Base commit | Duration ms | Cost USD | `sourceChangedAfterCapture` | Selected scope |
| --- | --- | ---: | ---: | --- | --- |
| `native-application-spelling-security` | `afa5fbcf33cbf93f0f6c3f74fd72acf0b22086bf` | 177928 | 0.6353549999999999 | false | `UpdateTransaction.ps1`, `diagnose-native-update-createprocess.ps1` |
| `native-creation-independent-debugging` | `25bb19e45ab68c5040bb66314d3bc0ed4b654da0` | 549930 | 1.8391137499999999 | true | Same two scripts plus the retained ten-row `diagnostic.json` |
| `plugin-forwarder-approved-security` | `afa5fbcf33cbf93f0f6c3f74fd72acf0b22086bf` | 558845 | 2.00916375 | false | `OriginalPluginTlsGuard.h`, `PluginPresentationFallback.h`, `Win32PeImports.h`, `Win32PluginTlsImports.h`, `win32_pe_imports_test.cpp` |

The native reviews captured the same production script hash
`312685e4402c20efded897cb607ae8126643f43a8af34d6684b0494e1ffd4ca4`
and ten-row diagnostic hash
`d6ed35941c46dfbbb7863a88bed690a46cd62f85157486beb88962e7eb30f4ed`.
At native candidate `3d13f1e93f40099df20cc75edb8d77755b9a270a`, production
still matches that capture; the diagnostic has changed to seventeen rows and
requires revalidation. The old ten-row JSON still matches its frozen receipt.
The debugging receipt's `true` flag does not invalidate the immutable capture or
authorize applying its conclusions to changed diagnostic bytes. Later production
and plugin fixes also require their own verification; no frozen review is
approval of those later bytes.

## SCRUM-311: observations and disposition

The retained [run 38166517938](https://github.com/ThereptileII/Work/actions/runs/38166517938),
[job 114550099612](https://github.com/ThereptileII/Work/actions/runs/38166517938/job/114550099612),
attempt 1, used exact candidate `3d13f1e93f40099df20cc75edb8d77755b9a270a`.
Artifact **11712027529**, **3,860 bytes**, matches API SHA-256
`fc1044be9e58f9e77f1f7fef9cf7e199b04fcf2b7140e53dee89a250e56ac0ed`;
all three ZIP members pass CRC. The diagnostic step passed as an observation;
the unchanged real-owner step and overall job/run **failed**. Product acceptance
is false.

| Native application spelling | Rows | Actual result |
| --- | ---: | --- |
| Physical fixture DOS path | 8 | Created suspended; error 0; held NT image matched |
| GLOBALROOT physical path | 5 | Failed with error 87 |
| Held-file-derived, reopened/identity-validated volume GUID path | 4 | Failed with error 87 |

All seventeen rows retained `CREATE_SUSPENDED`; no resume was attempted. Every
row records cleanup confirmed, and fixture cleanup is confirmed. GUID root/file
identity validation succeeded, but its namespace link was explicitly not pinned.

- **Confirmed within this fixture:** the old ten-row application/argv0 coupling
  limited attribution. New crossed rows show GLOBALROOT application still fails
  with basename or DOS argv0, while DOS application succeeds with GLOBALROOT
  argv0 and CWD. GLOBALROOT CWD alone therefore is not a sufficient failure
  cause in these controls. This supports changing only the application spelling
  while retaining physical argv0/CWD, single creation attempt, held ancestry/file
  custody and pre-resume NT-image validation.
- **Contradicted as a remedy here:** handle-derived GUID application spelling
  did not restore creation, including the application-only GUID row. No GUID
  production remedy is accepted from the static proposal. Namespace-shadowing
  security claims also remain hypotheses; reopening an identity does not pin an
  alias link.
- **Rejected factual premise and fallback:** the debugging review equated
  `OriginalImage` with row 2. The frozen diagnostic instead uses `$aExe`, the
  explicitly constructed physical `A\\app\\marker.exe` DOS path (case loop at
  lines 178–180). Owner acquisition uses the disposable alias at line 158;
  `OriginalImage=full` at production line 206 preserves that alias, which is then
  remapped to B. Row 2 does not test an `OriginalImage` fallback. That fallback
  remains rejected; a DOS spelling derived from the single held image is a
  separate proposed implementation, pending real-owner qualification.
- **Overclaims not accepted:** these finite observations do not prove failure
  is a pure function of path spelling for every environment, flag, path length or
  Windows version. Failed rows report PID zero and no usable returned child;
  they do not prove no process object ever existed or locate an undocumented
  internal rejection phase. ABI/reflection and custody explanations are useful
  source-supported checks, not complete product acceptance.

## SCRUM-209: findings disposition

| Claude finding | Sol disposition and evidence |
| --- | --- |
| Reserved DOS stems reach sibling open | Addressed by `4afa74fada6e18c1f073966a6974b2382937720d`: portable normal/delay/forwarder tests prove refusal before any callback. No COM port or native device was opened. |
| Lowercase dedup key also becomes lookup filename | Addressed by the same commit: preserve original spelling plus extensionless `.dll`, retaining lowercase dedup. Exact-string fake-reader probes now reach and refuse the TLS-bearing sibling. A native case-sensitive `LoadLibrary` bypass has not been demonstrated. |
| Blanket sibling reparse refusal may reject legitimate files | Intentional conservative fail-closed policy; not relaxed by this work. Availability cost remains explicit; hydration/dedup compatibility was not natively measured. |
| Root copy and graph queue can amplify memory pressure | Open qualification limit. Approximately 384 MiB is an estimated upper exposure for PE buffers under the existing 128 MiB root / 256 MiB closure caps, not a reproduced crash or measured peak; allocator/host overhead is separate. No queue rewrite or budget weakening is accepted here. |
| Forwarder path untested in the approved five files | Correct about the selected import test's scope, not the whole repository. The separate `win32_pe_forwarders_test.cpp` was absent from Claude's five-file capture. Real native run 38164763565 passed 17,496 reader checks and all 51 loader groups at exact `b8d6dc059deedf3c061681558406a37fd82e7b0b`. |
| No parser bounds/overflow defect found | Useful bounded static result; not a proof against all malformed images or substitute for executed/native gates. |

[The earlier native forwarder receipt](../scrum209-forwarder-native-38164763565/README.md)
also records actual multi-dot target `forward.clean`, value 409, file identity,
hash and unload/lock checks. It does not establish a general separator rule or
permit broadening the conservative forwarder grammar. Other review hypotheses
about unusual directory sizes and unreachable non-standard exceptions remain
unproven; no policy change is made from them.

[The module-name correction receipt](../scrum209-module-name-review-20261011/README.md)
records 9,885 import and 17,602 forwarder portable ASan/UBSan checks and the
before/after callback probe. Native Win32 qualification of the changed header
and the coordinator's changed-source native loader qualification remains pending
at the time of this comparison. Added portable cases do not change the loader's
51 named runtime groups.
Neither the earlier 51-group pass nor these reviews close SCRUM-209, qualify
broader plugin/dependency closure, or accept a product package. SCRUM-311's new
real-owner implementation likewise remains unqualified by this comparison.
