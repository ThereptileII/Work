# SCRUM-289: original native proof and isolated boat staging

Tool source local `35cd4c98468c57749245d5d5c348f5c7f7b1bab2` maps to published
`47c0a789967c29a618db1f0249a42ade756bf460`. Original run
[37184066301](https://github.com/ThereptileII/Work/actions/runs/37184066301), attempt 1,
completed all four jobs successfully. This audit downloaded the four original
artifacts through the connector. No tests/builds were rerun.

[api.json](api.json) retains original API identities and completed jobs.
[audit.json](audit.json) records the independent checks. All artifact byte sizes,
API SHA256 digests, entry CRCs and safe unique non-symlink paths passed. The four
small original ZIPs are retained unchanged alongside original receipts.

| Original artifact | ID | SHA256 |
|---|---|---|
| Qualified bundle | 11296067491 | `f182084abb099347f5788dd973bef5297cda1ce5541629e98d5b849888d4e76d` |
| Window proof | 11296715299 | `00bad4079ee98f65304c0d967ff6cb7951ad516e82fe890332b97bcffb6b5917` |
| Policy proof | 11295919163 | `76aa5bf21b7b3d0d86505d5f21116c7d975d3c48c46ad8081abc3c2a8a855982` |
| Broker proof | 11296516288 | `855975eb1fe955cbf7b3c9207acdbfbbc4316a041162373a9a946017b97e6131` |

The inner `review-tools.zip` SHA256 is
`8c157723ece86ff63affeb38e0ea15c61077234b264793f549d0b193abd9cf63`;
[manifest.json](manifest.json) SHA256 is
`50425fd96fa29beae2e6381ee45c544dab8929de448ad5faf95faf7fe345aca6`.
All **117** inner files match the exact manifest size/hash, committed source
SHA256 and source bytes with only recorded Git LF/CRLF conversion. The closed
inventory matches `tools/boat-review-composition.json`; no files from the prior
705 bundle were substituted.

The three [qualification](qualification.json) receipt hashes, source inventory,
composition hash and run/attempt/commit agree. Each referenced original report's
bytes match its receipt and report passed with fixture-only scope. Independently
checked **75 distinct native display cases**, including **22 new prototype
zoom/System/recovery success/refusal cases**, all with fixture exit 0, no error
and no cleanup errors. Native helper and fixture source hashes match. The window
report also retains **16** palette cases; broker proof retains **13** cases and
the actual scheduled palette Arm/Collect success with exit 0. These prove the
bounded tools against inert native fixtures, not behavior on the installed boat
application. Earlier base qualification and its limitations remain retained.

## Staged copy, without application or actor execution

Used the existing reviewed `stage-review-tools.ps1` operation documented in
[tool composition](../../installer/skager-boat-tool-composition.md), with its
exact qualified `Common.ps1`, archive and manifest. A fresh read-only check at
07:00:41 UTC found no `commissioning-active.json`, safe existing workspace/scripts
paths, and absent transport/destination. All five transport payload hashes were
verified before helper loading; the no-active guard was repeated immediately
before staging and after verification.

The first invocation stopped before loading `Common.ps1` because the boat's
default PowerShell policy disables scripts. No staged destination was created
by that attempt. The successful retry used process-local `-ExecutionPolicy
Bypass`, as in the native workflow; no persistent execution policy changed.

Exact new directory:

`C:\XNav\scripts\review-47c0a789967c29a618db1f0249a42ade756bf460`

All 117 extracted file sizes/hashes were rechecked on the boat; its complete set
is exactly those files plus `staging-complete.json`, with no child directories or
reparse entries. The original [completion receipt](staging-complete.json) was
retrieved and independently hashed:

`f64bab5e7eba48c9ad151cc842bf0f2be52e26a4b00b38b7f80e733dce878713`

[boat-staging.json](boat-staging.json) records the 07:03:51 UTC successful closed
set verification and absent active review. Staging reports `toolsExecuted=false`,
`applicationChanged=false`, `profileChanged=false`, and
`sourceCheckoutChanged=false`. This means no staged operator/application/review
actor ran; only the transport, existing staging helper and read-only verification
ran. The isolated `incoming-review-47c0a789967c29a618db1f0249a42ade756bf460`
transport directory remains retained under `C:\XNav\scripts`.

## Remaining gates

The tooling copy is ready for a future guarded session. No application was
installed, launched, configured, switched or reviewed; no boat safety command
was issued. Application/package qualification, the exact installed identity,
complete dependency/helper closure, fresh read-only commissioning audit and
actual boat chart/display/font/symbol review remain required. The failed bcc
production trust gate is not waived by this tooling result. Use this one coherent
directory for a newly qualified session; do not mutate an existing pinned session
or mix tool generations.
