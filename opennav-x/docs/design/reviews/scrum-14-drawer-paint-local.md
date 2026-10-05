# Shared Windows drawer paint — SCRUM-14

Selected scope: Jira comment 10499. Isolated base `632463c`; product changes
are confined to `src/ui/Drawer.cpp` and guarded by `__WXMSW__`.

Direct evidence is native source `29ea06a358ed24cc29eb2be74b67b234a3ecaa6e`,
workflow `37026807977`, artifact `11235841232`, under the integration worktree's
`evidence/local/prototype-native29/files/windows-changed-units/`. Search uses
the canonical Windows reference in
`scrum213-focus-repair/evidence/local/a18-search-reference/files/`;
Settings and Chart use `docs/design/prototype/reference/windows/`.
The inspected source matches the native29 Drawer/Controls source hashes after
Windows line-ending normalization.

At 1280×800, 100% scale, the title ink is consistently four pixels too low:

| Capture | Native ink y | Reference ink y |
| --- | --- | --- |
| `search-component/saved-objects-day.png` | 130–148 | 126–144 |
| `settings-component/settings-day.png` | 130–154 | 126–150 |
| `chart-presentation-component/chart-day-top.png` | 130–154 | 126–150 |

The CSS heading origin is correct: y81 plus title offset 39 equals the
reference h2 origin y120. The reference uses 26px, weight 450, line-height
29.12px. This patch moves only the Windows GDI title drawing origin upward
by four logical pixels. It does not move the eyebrow, divider, children,
close/back controls or change shared `TextTracked` behavior.

The same three components lack the right and bottom outline at (1079,400)
and (900,753). Native Day pixels are background `#152326`; reference pixels
are border `#35464a`. Top and left already agree. Settings and Chart Dusk/Night
also show background instead of the corresponding border at those edges.

`Paint()` previously passed width/height minus one. The wxMSW 3.2.8
`DoDrawRoundedRectangle` forwards these extents to GDI `RoundRect`, placing
the visible right/bottom strokes one pixel inward, underneath children ending
at x1078/y752. This patch passes the full client width/height on Windows,
placing the strokes in the uncovered outer pixel. The GTK paint call, shape,
child bounds, hit areas and drawer layout remain unchanged.
Upstream source: https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/msw/dc.cpp#L1124-L1150

Validation here is source/diff review and `git diff --check`, without a new
build or test run. The integration owner will include this change in the
selected native proof. All XNavDrawer subclasses share these paint paths,
but direct image evidence covers Search, Settings and Chart only. The four-pixel
adjustment is established at the captured font/100% DPI; other DPI scales,
font environments, rounded corners and Back-header states require native
review. No successful post-change Windows, whole-view or boat acceptance is
claimed.
