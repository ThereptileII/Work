# SCRUM-272 / SCRUM-273: native source-cache preparation failure

[Run 37135421013](https://github.com/ThereptileII/Work/actions/runs/37135421013),
job `111238859542`, failed in step 4 at 2026-10-03 16:03:38 UTC. Exact remote
`9ec668981b9aa86bcd643b84c6f18cfa002b3f74` maps to local
`9f72d0a797868d84efd9441022220a9b2ca52918`; published tree is
`647ad08dbdf47ebee44cec646af57c480d2f24d2`.

Original artifact `11277952581` is 5,083 bytes, SHA256
`50f25a26e0e1dad379cffced31981caaf41324c55fed6bee0ef705bdbb5a6e3a`.
The independently downloaded ZIP passed all five entry CRCs. Its five original
files are retained verbatim here. The ZIP and full completed job log remain in
the private audit worktree; `first-failure.log` is the unchanged relevant job-log
excerpt, including traceback and upload identity.

The source receipt covers 60 inputs. Eleven match local bytes directly; the
other 49 match precisely after applying Windows CRLF checkout conversion to
local LF text. Both byte length and SHA256 were compared; no mismatches or other
normalizations were accepted. See `audit.json` for the per-file distinction.

The 18 drive/UNC/POSIX path checks, six absent-input checks, and production
boundary checks passed. Pinned OpenCPN preparation also returned successfully,
including its reconstructed-index comparison against all nine reviewed patches.
The subsequent private source download failed before native configure: the
original failing configure control, corrected configure, actual source projects,
private/header inventories and ten x86 object results are **not available**.
There is no compiler, link, TLS, package or application-runtime acceptance.

First causal error: `test-trust-compile-windows.py:71` calls `fetch_sources`;
`prepare-ocharts-adapter.py:94` raises `PermissionError [WinError 5]` replacing a
temporary file into cache key `7d5393a1212da0841e28e2f74883cb40fbd75a1f`.
The pinned lock gives both `COPYING` and `COPYING.gplv2` this same 15,170-byte
blob. Fourteen blob keys have duplicate destinations. The eight-worker fetcher
currently schedules each destination independently and does not serialize
read/check/replace operations by blob key. This establishes an actual race
opportunity at the failed boundary; the precise Windows file-handle interleaving
was not traced. SCRUM-273 owns the coalescing correction and deterministic proof.
This is preparation failure, not an application crash. No rerun was dispatched.
