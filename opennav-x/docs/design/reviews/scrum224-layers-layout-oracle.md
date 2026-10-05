# SCRUM-224: require the bounded prototype Layers control

Selected by Jira comment 10507. Candidate
`88c141ba9fee5c28e12b5f4ac0f7e7550d1a8ca6` remains failed and frozen;
this correction changes the inspection contract, not product geometry.

The failed Linux job [110926582484](https://github.com/ThereptileII/Work/actions/runs/37033702537/job/110926582484)
retained `objects-input-results.json:startup_layout_failure`. Artifact
`11239641523` was independently downloaded and verified by the parent task:
6,345,329 bytes, SHA256
`c700f239ac11d1391a1eb6780aba40380519fd518b7224dcbe653f48466e106a`,
2,627 ZIP entries passed CRC/path checks. The test fixture retains only the
frame/client and layout fields used by this oracle, without the profile or
unrelated diagnostics.

The immutable `docs/design/prototype/index.html:8` specifies a top-right
column 22px from the chart edges, a 68x90 compass, 10px gap and centered
44x44 icon buttons. Line 135 places Layers immediately after the compass.
Thus Layers has bounds `(chart.right - 78, chart.y + 122, 44, 44)`.
`src/ui/Shell.cpp:264` creates that button and line 1628 positions it
accordingly. The actual failed capture reports `(1016, 190, 44, 44)` within
chart `(80, 68, 1014, 566)`; the retained screenshot also shows that placement.

The old six-control oracle rejected this actual capture with
`Unspecified permanent controls must not cover the chart`. The corrected
oracle requires exactly one visible `Layers` control with the stated bounds.
The existing one-pixel rounding allowance, chart containment, other six
controls, rail/footer/recovery checks and rejection of arbitrary chart
overlays remain unchanged. The reported floating-control count is now seven.

Focused validation: the same original capture fails before this change and
passes afterward. `python tests/chart_layout_tests.py` passes three existing
Linux/Windows-client compositions, the retained actual capture, 29 existing
invalid-layout cases, nine additional missing/hidden/duplicate/misidentified/
moved/oversized/arbitrary-control cases, and the existing 54 palette checks.
The new geometry mutations exceed the unchanged tolerance by one pixel.

Shared callers include `tools/smoke-navigation.py:509` (startup) and `:763`
(Settings return), with Win32 frame/client handling at `:497`, plus
`tools/prototype/capture-native.py:351`, which also supports native Windows.
They use this same oracle, so the correction is not Linux-specific. Other
Windows consumers of `chart-render-check.py` use its pixel checks rather
than `navigation_layout`; those checks are unchanged.
`tools/build-pristine-windows.ps1:244` and `:262` invoke the affected objects
flow in targeted and full integrated Windows validation respectively.

No application build, new CI run, native Windows rerun, security-probe
dispatch, or boat operation was performed. The earlier candidate's failed
Linux job continues to make its package ineligible for the security probe.
