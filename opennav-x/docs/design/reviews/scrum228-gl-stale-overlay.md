# SCRUM-228 — stale-route GTK overlay investigation

Exact executable source: `1356fd1603aacbea04d7081d16331e9a181180bb`; SHA-256 `0aaaf2238828fcf4b42d02c8b7dad7191781ead1bf2ece0c2a9384c1fdc726bf`. Linux Xvfb/Mesa evidence only. This investigation did not change product source or invalidate the original passed software/GL route reports.

The original GL stale screenshot omitted all four chart-control surfaces, despite a later diagnostic reporting visible controls. Root's follow-up probe failed at its first attempted `xwininfo` invocation; both stale screenshots already captured by that probe show all controls, and its burst-0 diagnostic reports them visible/enabled. The exact failed source, images and reports are retained, along with an explicitly attributed error transcription; no original stderr file existed.

One actual application repeat replaced `xwininfo` with read-only libX11 window queries. Two setup failures occurred before executable launch (missing Xvfb PATH entry, then incorrect executable alias); their original errors are retained separately. The corrected repeat passed the unchanged 26 route checks and exact route-ink probes. Six stale screenshots were taken at approximately half-second intervals, with window-tree snapshots before and after each capture and contemporaneous diagnostic publications. All four control rectangles are pixel-identical to the active screenshot in all six images.

However, the X tree proves a transient native stacking inversion:

| Snapshot | Monotonic seconds | Bottom-to-top order |
| --- | ---: | --- |
| Burst 0 before | 40619.3805 | Main frame, Follow boat, Layers, Orientation, Tools |
| Burst 0 after | 40619.5466 | Follow boat, Layers, Orientation, Tools, Main frame |
| Burst 1 before | 40620.0484 | Main frame, Follow boat, Layers, Orientation, Tools |

All five windows remain mapped (`IsViewable`, map_state 2) with unchanged geometry throughout. Diagnostics continue to report all controls visible/enabled. The captured image precedes the post-capture inversion and still shows the controls. By the next sample the owner/surface order is restored, and all later sampled orders are correct. Thus a real transient native stacking state can obscure the controls; it is not evidence that the stale-data branch intentionally hides them. The original missing-controls frame is consistent with this mechanism, but no continuous trace yet identifies the raising caller or establishes the exact duration. Do not call this a proven GL framebuffer defect, a permanent visibility loss, or a harmless screenshot artifact.

`src/ui/FloatingSurface.cpp:15` presents each owned GTK surface; lines 34–40 restack it above its owner. `src/ui/Shell.cpp:1624` chooses availability and calls Present; the normal update invokes this every 250 ms. Recovery is consistent with that existing reconciliation. A faster timer or a global topmost hint would hide the race without explaining it. The next bounded step is to identify the owner-raise caller and repair the supported owner/owned stacking boundary without focus changes, altered visibility requests or navigation changes. An unproven generic GTK event hook should not be introduced.

The [analysis](../../evidence/scrum228-gl-stale-overlay/analysis.json) records exact ordering, geometry, capture-time diagnostics and pixel comparisons. The [identity manifest](../../evidence/scrum228-gl-stale-overlay/identity.json) hashes all retained artifacts. `completed_monotonic_ns` in the read-only helper was evaluated before tree traversal and is not a valid completion timestamp; the analysis uses only the correctly recorded snapshot-start times. Profiles, configuration and navigation databases are excluded. Windows and boat acceptance remain open.
