# Boat recovery inspection — September 28

At 18:36 UTC SSH became reachable. Read-only inspection found no OpenCPN
process, the original stock executable SHA-256
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`, and unchanged
navigation/chart database bytes relative to the prior closed checkpoint.
At that inspection the input-only commissioning transaction was still active.
The INI differed and required review before restoration, preserving user changes.

The local log records clean frame/application exit at 18:34 UTC. The retained
o-charts decoder was started during plugin teardown. Its reviewed executable
hash is `ec27c947fc9ba4ae09345961780b885deddc2e4fcf094805a14cdf19504e0adb`.
Its parent has exited and has no tool-owned launch receipt. The old stock-launch
shutdown tool is therefore inapplicable; no receipt was invented or refreshed.
Cold `InspectRestore` correctly refused the still-running helper.

The new orphan-recovery tool verifies the active hash-bound cold transaction,
actual user/session, exact installed generation, complete plugin inventory,
known decoder bytes, exact PID/creation time, absent parent PID and exact
source-derived pipe arguments. Every other app/helper process still blocks.
It reuses the already tested one-shot chart-decoder CMD_EXIT transport, with
actual pipe-server PID checks, retained process handle, bounded cancellation,
measured exit, no retry and no forced termination. The shared attempt locator
prevents using the older tool to retry an uncertain operation.

This is a local chart-decoder lifecycle operation, not marine equipment output.
It does not mutate a profile or restore plugins. A separate cold inspection,
review/adoption of INI changes and full restore verification are still required.
Native run `36468187212`, source `485a8c47cb33ef975b3b10a9d5f649c702d0021a`
(complete local `e3054865e1394373e317004aa79382a90d79a5fe` mapping), passes
21 orphan-policy checks, 45 chart-helper checks including ten actual disposable
Windows pipe cases, 34 preparation checks and 21 commissioning checks. The
downloaded tooling artifact 10990931545 verifies SHA-256
`800efdcd1c96231f37e77e1eb530055093d2148fe78b15d841b4d654a09aa6ae`, size 24,135
bytes and ZIP CRC. The whole tooling job failed its separate existing GUI label
check (`Pilot`, now `Autopilot`); this is not reported as an all-green run or
prototype UI qualification. Replacement label checks also identify `Center`
renamed to `Follow boat`. Boat GUI automation needs a separate prototype flow
review, including owned floating windows; none of those pointer actions ran.

After verified source-only update, the orphan preflight passed against the
actual cold record. One normal helper shutdown observed the retained exact
process exit code 0, verified pipe-server identity and a three-byte reply.
Reply contents are not interpreted as command success; observed process exit
is the evidence. No force termination, application launch or marine command
was used. Cold InspectRestore then succeeded and saved a fresh private copy.

Eight non-connection changes cover chart/workspace position/size/follow state,
cached ownship position, exact stock build marker and GPU texture budget. The
ninth original-baseline diff is the expected one-byte temporary COM8 input-only
setting. Navigation and chart databases remain unchanged. These preferences
are to be preserved, not overwritten by the older baseline.

The GPU 128-to-64MB reset follows pinned `ocpn_app.cpp:1493` (different build
marker sets upgrade flag), `OCPNPlatform.cpp:634-643` (GL-capable upgrade default)
and `navutil.cpp:2168` (persistence). The narrow policy now additionally accepts
the exact observed September 27 Beta build marker returning to the exact stock
September 12 2025 marker. Other builds/budgets/GL changes remain refused.
Replacement native [run 36469956631](https://github.com/ThereptileII/Work/actions/runs/36469956631)
passes all nine jobs at source `b17813aa5379886df0a51e61c74ad7521b1a8878`
(complete local `c17ee89a90add6510bba968566a3ca4dff34cf9d` mapping). All 23
maintenance suites pass, including 178 native baseline-adoption checks. Its
downloaded artifact 10991372580 verifies 26,445 bytes, ZIP CRC and SHA-256
`bcba39801cf23817f5b1dbdb4302b6e7ae5ed9b288c781d0e50af0925c3da653`.

Following a clean source-only update, actual adoption and restoration succeeded.
An independent post-restore inspection verified:

- No OpenCPN/helper process and no active commissioning marker.
- All five quarantined plugin DLLs restored with original hashes; quarantine
  copies absent.
- Original connection direction restored, preserving the eight reviewed
  preferences and all other profile files. No application was launched.
- Stock executable hash remains the value above.
- INI SHA-256 `f6db33a9f7722e2df2758f932e76b2d598017ef859d9074caa09dc76c4a42300`.
- Unchanged navigation database SHA-256
  `8eada8eb11d17fc91e687307f713a7bdb90a38ade31e7d6f272198fff360ebbc`.
- Unchanged chart database SHA-256
  `1440c8d8ab0a907fb688d19e9d66579eaa84c6eb3318c3a5cdcdc637d11a191b`.
- Durable restore record SHA-256
  `82977eb3be61ee45ce9c5765857c49fe512cde82626cd48d318582cc3d7b8b2c`.
- New adopted-baseline record SHA-256
  `13c03fd622d6105f353db364ef5777dce87054f26fa788e3f4574dad301266f5`.

The deployed Beta executable remains unchanged; no prototype build is deployed.
Because output-capable plugins and original connection settings are restored,
any next application launch requires fresh Inventory/source review/Prepare/Apply
using this new baseline. Old restart receipts are expired. This closes recovery,
not the new prototype UI or boat-display acceptance gate. Private logs, user
paths and navigation data remain outside committed evidence.
