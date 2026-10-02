# SCRUM-224: observe soak viewport inputs

Selected in Jira comment 10524. This increment changes only harness evidence;
it does not change chart controls, navigation, zoom behavior or acceptance
tolerances. The default endurance assertions remain intact.

Each actual zoom/palette input now records before/after chart state, diagnostic
ticks, wall/monotonic observation times, file modification time, advertised
control bounds, input timing, and available bounded fixture pointer traces.
Linux retains the original three-action control snapshot and half-second
delay; fresh bounds are recorded alongside the selected bounds rather than
silently changing the input. Windows retains the existing HWND helpers and
records native candidate bounds separately. Fresh diagnostic ticks alone are
not represented as an activation acknowledgement. Periodic samples include
the viewport; final disposable configuration and pointer trace are retained.

`--viewport-only` performs just the first zoom pair and palette action, with a
screenshot before each input. It reports `status: observed`, no requested
endurance duration and `release_duration: false`. It cannot report an endurance
pass. Normal runs retain the original duration, progress, dropout/recovery,
resource and fixture-preservation assertions.

## One local 120-second observation

Evidence: `evidence/local/viewport-120s-8a938ecc/` in this worktree. The
unchanged existing assertions passed after 120.086 seconds, with 12 samples.
This was an older local Linux executable, **not c95, FFE or current Windows**:

- Reported build source: `8a938eccfe8ca05392d6ebe5e404ad1f702a76e5`.
- Executable: `waypoint-touch-regression/build/xnav-install/bin/opencpn`.
- Executable SHA256: `3e926787e9c345eed75ad8b01518db795d3657e0b0b3115e6adf3a4b20918800`.
- Harness base: `f220b31391815663c442d1f8dcbe98a54c03333c`; run-time harness
  SHA256: `0e6eb225afcf3b561bf873664768e03c1e1a14a51cbf8b432812beac1f8238b0`.
  The short diagnostic option was added after this run.
- The original soak script and zoom bridge are unchanged between the reported
  older source and the harness base. Other product/UI changes exist; this run
  does not qualify the current candidate.

| Input | Sent screen point | Diagnostic ticks | Observed result |
| --- | --- | --- | --- |
| `+` | (648,365) | 13 → 19 | Raw down/up on chart canvas; no button activation; center changed to 59.0584437,18.68273053 |
| `−` | (692,365) | 19 → 23 | Raw down/up on chart canvas; no button activation; center changed to 59.03687386,18.99726669 |
| Day | (1201,34) | 23 → 27 | Palette button activation recorded; Day → Dusk; viewport unchanged |

Scale stayed `0.003000000026` across all three actions and at the end. Final
configuration explicitly records `SmoothPanZoom=0`. Raw chart input identifies
the receiving window at (80,68), size 1014x566. Both the selected and fresh
advertised zoom bounds were (626,343,44,44) and (670,343,44,44).

Visual inspection matters: `start.png` has no painted floating zoom/compass/
follow controls despite their diagnostic visibility flags. `end.png` shows
the zoom tools and compass displaced into the chart, consistent with the
advertised bounds. Therefore this is evidence of an older-build placement/
visibility inconsistency and input reaching the canvas, **not proof that a
click passed through a visibly correctly placed button**.

The exact c95 404.829x scale drift remains unproved by this observation. Its
retained artifacts lack per-action scale/activation history. A fresh-source
short trace remains a separate gate; no product correction, new full build,
CI dispatch or boat operation was performed here.

## One fresh-source three-input observation

The single `--viewport-only` replay used fresh build source
`74761400862bc1d25102b2b741d14247dd343cdc`, confirmed by the executable's own
reported identity. The 25,721,560-byte executable SHA256 was
`233b0d8745c82a91dca7fe8c50ae879a16f1e3cb5475a421fdae322a54821b83`.
Harness commit was separately `1101b00b7fe2f727d4256f5d9cadcfa4f255931d`,
script SHA256 `7cf8e122462d3ea5ec2d360fea1995ced41937e80e583df064a28dcba2aa5773`.
Evidence is `evidence/local/viewport-only-7476140/`. The action phase lasted
2.851 seconds; result is **observed**, with `release_duration: false` and no
endurance pass. Process exit and unchanged navigation-fixture checks passed.

The builder's ready install was invoked directly, with its actual build's
`include/config.h` and the local sysroot libraries. The log confirms the
compiled shared-resource prefix still resolves to
`waypoint-touch-regression/build/xnav-install/share/opencpn`. A read-only
comparison independently found all **608 resource files byte-identical** to
the fresh private install's resources; `resource-identity.json` retains that
comparison. No source or original cache was modified.

Startup ordering is material: the new disposable profile requested 1280x800;
the application log records initial frame creation at **896x560** at
20:35:40.996. The unchanged harness waits for the visible window, sends its
1280x800 resize/move/focus, then waits for deferred canvas finalization
(logged 20:35:43.912) and another 1.5 seconds. Startup observation took 4.977
seconds. The resize itself has no separate timestamp in this revision; do not
infer one. Before input, chart bounds are (80,68,1014,566), while advertised
floating controls retain the smaller layout's coordinates.

| Input | Sent center | Ticks | Recorded outcome |
| --- | --- | --- | --- |
| `+` | (648,365) | 10 → 16 | Raw canvas down/up; no button activation; next diagnostic still reports initial center |
| `−` | (692,365) | 16 → 21 | Raw canvas down/up; no button activation; center now 59.03687386,18.99726669 |
| Day | (1201,34) | 21 → 25 | Raw button down/up and activation; Day → Dusk; viewport unchanged |

All three before-input PNGs and `end.png` show **no painted floating chart
controls**, although diagnostics mark them visible and enabled. Zoom bounds
are still (626,343,44,44)/(670,343,44,44), compass (650,78,68,90), Layers
(662,178,44,44), and Follow (98,347,142,44). Unlike the older 120-second
observation, the fresh short run's palette change **does not reveal them**.
The canvas receives both zoom-labelled inputs at its actual (80,68) origin,
1014x566 size. This is not evidence of clicking through a correctly painted
button, nor merely the harness retaining an older coordinate snapshot:
fresh advertised bounds match the selected bounds.

Scale stays `0.003000000026`, follow remains false, and retained configuration
explicitly records `SmoothPanZoom=0`. The screenshot before the second input
already shows a shifted chart although the preceding diagnostic center had
not advanced; fresh UI ticks are not a chart-settled acknowledgement. Therefore
attribute the final displacement to the observed pair of canvas inputs, not
exclusively to the second gesture.

This confirms the Linux startup/resize visibility-and-bounds discrepancy on
fresh source. It neither reproduces c95's 404.829x scale drift nor establishes
a Windows failure. No further run or product fix is part of this observation;
root retains the dedicated regression and release/native gates in Jira.
