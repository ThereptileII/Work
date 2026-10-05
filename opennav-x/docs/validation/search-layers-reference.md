# SCRUM-14 — Search and Layers reference scope

The canonical Search state is the unchanged HTML opened through its actual
top-bar button, `.header-end [data-panel="search"]`. `tools/prototype/render.py`
now exposes this as `--states search`. It leaves the query empty and allows the
original focus timer to run. The existing `layers` state opens the original
map-top Layers button. Neither state substitutes native fixture data into HTML.

## Local Search reference

On 2026-10-02, only Search Day, Dusk and Night were rendered at 1280×800,
DPR 1, using Playwright 1.58.0 / Chromium 145.0.7632.6 on Linux. All 113 original
files passed their immutable manifest checks before and after rendering; the
HTML SHA-256 remains
`b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`.
The renderer rejected external network requests and page errors. All three
images were inspected: each shows the focused empty search field, five original
illustrative results and the original scope note in its selected theme.

Actual platform fonts for the drawer heading, search placeholder and secondary
result text were **Liberation Sans** (`LiberationSans`); the first result name
used **LiberationSans-Bold**. These are Linux fallback measurements, not proof
of Windows typography. The drawer is `(682,80,398,674)` in all three themes;
the input is `(705,191.109375,352,44)` and the first result row is 66px high.

[Compact measured evidence](../evidence/search-reference-linux.json) records
the renderer identity, original identity, browser, fonts, selected computed
styles and hashes of the actual captures. Full PNGs and the complete measurement
record remain under `evidence/local/search-reference/` in the
`scrum14-search-reference` worktree; they are not shipped product assets.
Reproduce with the project-local Playwright environment, or another environment
with the pinned renderer requirements and Chromium installed:

```sh
python tools/prototype/render.py --output evidence/local/search-reference \
  --states search --themes day dusk night
```

No Layers rerender, native application build or Windows run was performed for
this documentation increment.

## Pair states before judging visual parity

| Native surface | Actual HTML reference | Comparison boundary |
| --- | --- | --- |
| Populated saved-object Search | `search-day`, `search-dusk` or `search-night`, matching the native theme | Compare drawer placement, heading, field, row hierarchy, spacing and theme values. HTML results are illustrative coastal places with distances; production results are actual saved OpenCPN routes/waypoints with honest type/scope text. Names, row counts, icons, notes and selection semantics can differ. |
| Empty or unavailable saved-object Search | Same-theme Search for shared structure only | The canonical empty query has five illustrative results. A native empty catalog or no matches is a separate semantic state, not a reference-perfect populated Search. Do not inject example harbours or distances to obtain a match. |
| Layers / Chart presentation, available vector chart | Same-theme `layers` | Compare shared drawer hierarchy, row/segment geometry and theme. Native format is observed data, not the prototype's Vector/Raster concept switch. Managed or unsupported layers must remain explicit; only supported actions use actual readback. |
| Raster chart or unavailable chart/layer/orientation | Same-theme `layers` for shared structure only | The default reference is not the same observed chart state. Disabled ENC controls, unknown selections and unavailable reasons cannot truthfully reproduce its illustrative enabled toggles. Missing charts cannot be reference-perfect. |
| Scrolled Chart presentation bottom or compact Shell | Corresponding actual scroll/viewport capture, if produced | The default `layers` capture is the top at 1280×800. Do not treat it as a matching reference for a different scroll position, viewport or DPI. |

Thus `chart-dusk-raster` and `chart-night-unavailable` component captures retain
their semantic labels; they are not replacements for `layers-dusk` and
`layers-night` references. The Search Night empty fixture likewise is not a
populated `search-night` reference. These distinctions explain differences;
they do not relax geometry assertions, image tolerances or acceptance gates.
Shared visual mismatches still need review against measured reference values.
Native Windows references in the actual font/DPI environment, integrated
application comparisons and physical boat-display review remain required.
