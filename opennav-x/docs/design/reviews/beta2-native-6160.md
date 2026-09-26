# Native candidate 6160: failed gate review

Candidate `6160d3e4bcd924853462f96a32f1b502a72a2884` **is not accepted**.
[Native Windows run 36277024981](https://github.com/ThereptileII/Work/actions/runs/36277024981)
compiled with MSVC and passed all 102 native CTest cases plus the initial
21-check navigation-object/AIS scenario. Four later UI gates failed. The exact
[evidence record](../../evidence/beta2-windows-6160d3e4.json) identifies the
results separately and binds the downloaded artifact by SHA-256:
`1bb7eda773c25cd0bbd19d475fd7486001156257c55772f5bb75fe0a714c49fa`.

This is the fixture-enabled disposable CI build. Visible Demo controls in these
images are expected test infrastructure; these are not installed-product or
boat-PC acceptance screenshots.

| Failed gate | Evidence and required follow-up |
|---|---|
| Pointer user flows | After the North click, diagnostics did not expose Course before timeout. Preserve as unresolved for this candidate; trace the native pointer, handler and resulting state. |
| Preview summary | Native children contain both Demo and the new distance/destination summary. The helper still requires the obsolete `NM to destination` caption. Update the predicate without dropping containment assertions, then rerun natively. |
| 150% DPI rail | The assertion paired pre-resize diagnostics with the current native frame. Captured pixels and the later independent HWND/diagnostic inventory show all four values fitting. Synchronize observation times; retain size and clipping checks. |
| Chart/plugin workflow | The test looks for the plugin entry directly on the new grouped Settings page. Correct the interaction path and rerun the remaining software/plugin/OpenGL checks. |

Only these four images were visually inspected for this review:

| Screenshot in the artifact | What is visible | Qualification limit |
|---|---|---|
| `dpi-150-failure.png` | Coastlines, four complete SOG/depth/wind/heading rows, unavailable labels and the full navigation bar at 1280×800 and 144 DPI. Current rail bounds are x1065–1269 and y129–704. | The full 150% suite failed; this image alone cannot pass it. |
| `chart-software-01-loaded.png` | Seattle/Elliott Bay ENC coastline, soundings, named objects, navigation symbols and ownship. The chart dominates the layout; missing sensor values remain unavailable. | The large native `Feet` annotation and crowded chart labels remain prominent. Rendering evidence is not final design acceptance. |
| `chart-software-03a-adjacent-cell.png` | Different chart coverage with detailed soundings, contours and traffic-lane information. This is real ENC content, not a blank all-water frame. | Software-renderer evidence only; no OpenGL claim. |
| `chart-software-overlay.png` | Grouped XNav menu actions fit within the window, with persistent navigation available. | The full-page menu covers the chart; this capture does not prove chart rendering underneath or complete the visual design review. |

The native DPI results completed 100% and 125% sequences, including alert/rail
layout, night workflows, injected touch, fullscreen return and mode transitions.
Physical touch was not tested. At 150%, the retained pre-resize row was x1129
with width204, while the screenshot, subsequent diagnostic and native child
rectangles agree on x1065 with width204. See the
[observation investigation](beta2-dpi-observation-6160.md) for the synchronization
repair and the checks it preserves.

Software ENC images exist for loading, zoom, follow, adjacent-cell selection,
return and menu restoration. The chart result has **zero completed phases**
because it failed before finishing its plugin check; the later OpenGL phase was
not reached. These partial observations must not be reported as a completed
chart/plugin gate.

All original captures and profiles remain in the downloaded, ignored evidence
archive. This public review copies no profile content, personal identifiers,
chart files, license data or real boat configuration. An exact replacement
native run, fixture-free product packaging and boat-display review remain
required before release acceptance.
