# SCRUM-209: original B0 capture failure and helper correction

The frozen B0 run [38442801510, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38442801510)
at `2c61369080c358d5ef9b8952d6f2881b52891b13` remains **failed**. Its
`windows-integration` job `115461450333` passed fixture-free product compilation
(step 15) and compiled-recovery retention (steps 16–18). The plugin input
capture (step 19) failed; this is not a product compilation failure.

The original capture artifact reports:

```text
Plugin input capture refused: Source changed before read: dumpbin.exe
```

The failed-step log records that message at line 11 and exit code 1 at line 12.
The helper refused its path-versus-descriptor complete-stat comparison before
hashing `dumpbin.exe`. Windows path and descriptor stat can expose different
execute bits and ctime semantics; the exact differing field was **not recorded**.
The original evidence does not establish actual tool mutation or a product defect.

Artifact `11840811376` is 283 bytes, SHA-256
`a1bb8f34901d8258cc301134f4af1c9c0539876356ec51a98e26eb2793c4d7c3`.
Authenticated GitHub length/digest and ZIP CRC pass. It contains only the capture
log, with no inventory or retained SDK data. The retained plugin follow-up is
**ineligible** because its required original capture step failed.

Compiled-recovery artifact `11840836434` is 426,988,285 bytes, SHA-256
`fa0357a26605f29403bde6602cdedf5b91f8ac10345956ad44f4cc5ae201a8a7`.
Its authenticated outer ZIP passes CRC and contains `COMPILED_RECOVERY.zip` plus
`receipt.json`. The original receipt says `packaging: not-run`,
`qualification: not-run`, and `trackedSourceDirty: true`. These are diagnostic
compiled bytes, **not packaged or qualified recovery**, and have no supported
reuse path. The updater staging, packaging, installer and Staging seal steps
were skipped. Artifact existence does not qualify the product or authorize reuse.

Helper fix `4cbea64a01c155873319011a0bddf69cba2dfa63` (isolated patch `bc77234e`) matches common physical
identity across stat sources, then preserves each complete source baseline for
mutation checks. Both the hash/read and copy paths keep hardlink, reparse,
metadata, byte-count and content checks. Linux: **34 focused tests passed**;
Python syntax checks passed. The first native run
[38454485025](https://github.com/ThereptileII/Work/actions/runs/38454485025) at
`97879a11` remains **failed**, with four Windows fixture path-substitution
assertions and two existing case-sensitive-fixture skips. Fixture correction
`d43465a1` then passed [38454852384](https://github.com/ThereptileII/Work/actions/runs/38454852384):
34 tests total, 32 passed and the same two skips.

Follow-up `806b80da` adds the system `WLDAP32.dll` allowlist entry while retaining
local-shadow refusal and its negative control. Linux passes 37 capture-helper and
30 runner tests. Native [38455446291](https://github.com/ThereptileII/Work/actions/runs/38455446291)
at `cd8a76e7d0e2a1ab62e1277606e9ccdb4d9d6495` passes 37 tests: 35 passed and the
same two skips. Its downloaded artifact `11842117998` is 2,642 bytes, SHA-256
`5cdfe81864dc5cf389e357cc6b7424ea78cef9ffd68b8b7d9fb5d8008cacfa10`;
GitHub digest/length and ZIP CRC match. The actual native probe observes
`st_mode` differences for `dumpbin.exe` and `st_mode`/`st_ctime_ns` differences
for Python; common identity matches and every complete source baseline stays
stable. These later observations do not reconstruct the exact field in the
original B0 failure. No inspected payload executes.

One replacement private-trust [Staging run 38455915079](https://github.com/ThereptileII/Work/actions/runs/38455915079)
at exact `cd8a76e7d0e2a1ab62e1277606e9ccdb4d9d6495` is **pending** at this update,
with extended tests and design validation off. Runtime protected-retirement Go
inputs changed from B0 and the diagnostic archive has no supported restore path.
The native helper proof does not qualify that application, its plugin/download
integration, installed private-channel B0/B1 behavior or full Lifecycle uninstall.

[results.json](results.json) retains the original producer steps, artifact hashes,
unaltered compiled-recovery receipt and separate helper/native statuses. Subsequent
results are appended without changing the original failed conclusions.

## Same-commit operator kit retained while application qualification continues

The replacement run's operator artifact `11843817697` is authenticated at exact
`cd8a76e7`, attempt 1. Producer job `115498679867` and its packaging step passed.
The downloaded outer ZIP is 159,729,989 bytes, SHA-256
`06f70ad624ff23f05867aef9c76d9b67ea9c76f43a1bfb30ec4b4d2406496520`.
GitHub digest/length, outer/inner/source ZIP integrity, all nine payload hashes,
three Windows/386 executable build identities and their prerequisite chain pass.
The source archive's 30,718 member hashes and 434 project files match the exact
commit, accounting for checkout line endings. No payload was executed.

This is operator-tool provenance only. Application qualification, a same-commit
fixture-free recovery package, launcher pairing and installed acceptance remain
pending. The structured audit is preserved under `replacementStaging` in
[`results.json`](results.json); it does not change the original B0 failure.

## Linux integrated gate passed; native product qualification still pending

The same-commit Linux job `115498678722` passed. Artifact `11845712573`
(14,630,077 bytes, SHA-256
`964e3753acc1f0ec2771782abc4f5604f1f0e895aca38b423c41e7616406e8bc`)
matches GitHub's digest/length and all 3,984 ZIP member CRCs. Original CTest XML
reports contain **154 passed, zero failed/skipped** in each of the fixture and
fixture-free builds. These are the same 154 entries exercised in two builds,
not 308 distinct feature tests. Mode-cycle/profile persistence and the Linux
runtime probes also passed their original job steps. Native Windows compilation,
packaging and installed acceptance remain separate, pending gates at this update.

## Replacement Windows capture refused native wx SDK include files

Run [38455915079, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38455915079)
at exact `cd8a76e7d0e2a1ab62e1277606e9ccdb4d9d6495` is now **failed**.
Original native job `115506154676` completed both application compiles, installed
security checks and both CTest suites: **146 passed, zero failed/skipped** in
each build. Its later capture step refused:

```text
Plugin input capture refused: Unexpected SDK input type: build/integration-source/cache/wxWidgets-3.2.8/include/wx/arrimpl.cpp
```

Original capture artifact `11847718246` is 340 bytes, SHA-256
`138e5c62af9215f68f39beede111ede97b226bef1a68770877ab4636307acf1a`.
GitHub digest/length and ZIP CRC match; it contains only that diagnostic log,
with no SDK inventory. Build-evidence artifact `11847343748` is 65,357,528 bytes,
SHA-256 `d0770cd7924e969e9c7b8b8305a5b087809830fb70065f41541d5ce0df8b2bb2`;
GitHub digest/length and all 18,103 member CRCs match, including the original
CTest XML and capture log. These successes do not override the capture failure.

The exact official archives selected by `tools/windows-wx.lock.json` were
downloaded and checked independently. The headers archive is 1,454,469 bytes,
SHA-256 `996041908057574ee3836a5f5e03f40858ffc61011231f19a5e87297b74b74e2`;
its complete include tree contains 1,509 files: 1,477 `.h`, three `.cpp`, ten
`.cur`, twelve `.ico`, two `.bmp`, three `.manifest`, one `.rc` and one `.xpm`.
The three implementation includes are exactly `wx/arrimpl.cpp`,
`wx/listimpl.cpp` and `wx/thrimpl.cpp`. The Dev archive is 33,824,019 bytes,
SHA-256 `2d1469322e89293e351260954f0575a18629a78d53a39e0545b1594f221f5810`.
It adds no include-tree files; its selected `lib/vc14x_dll/mswu` tree contains
exactly `build.cfg`, `wx/setup.h` and `wx/msw/rcdefs.h`. The existing filter
would also have refused `build.cfg` after the first include failure.

The helper correction permits only those three exact include-relative names
and the exact release-setup-relative `build.cfg`; global `.cpp`/`.cfg` support
is unchanged. All four members are copied and hash-bound through the existing
identity, mutation, reparse, hardlink and size checks. Linux focused checks:
**39 capture tests and 30 retained-runner tests passed, zero failed/skipped**.
The corrected filter also accepted all 1,509 actual pinned include files and
all three actual release-setup files. Wrong-tree, nested and unknown C++ and
configuration members still refuse. This is local helper evidence; native
capture and downstream source-slice qualification remain pending.

Compiled-recovery artifact `11847738253` is retained separately: 427,030,551
bytes, SHA-256 `e75878e80eb476cae58bdb7cd530c7e3e5c4cd55b3bdb7d655e0516f00ac826b`.
GitHub digest/length and outer/inner ZIP CRCs match. Its inner archive SHA-256 is
`a03d1560b64758fa8849d986e30815aeb433083884fa1e080e1a7a9dec069109`;
manifest SHA-256 is
`67c992f6d3394822d4a00f829cd9c9574887556e40570ecd796882bd7cd841ca`.
The manifest retains 16,726 files and 892,198,057 uncompressed bytes, including
both original application installs. It contains no wx SDK cache or production
CMake selection controls. Its receipt remains `packaging: not-run`,
`qualification: not-run`, `trackedSourceDirty: true`. Existing tooling provides
no supported restore path for this diagnostic archive, and the retained plugin
workflow requires the original capture step to have succeeded. The failed run
cannot be relabeled as qualified or used with that workflow's current recipe.
