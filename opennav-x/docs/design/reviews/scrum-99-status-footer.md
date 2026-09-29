# SCRUM-99 — prototype status footer

Parent: SCRUM-14. This is a bounded replacement of the navigation footer, not a
new navigation-data contract. Native Windows and boat-display acceptance remain
required; Linux fallback typography is not their substitute.

## Reference and observed defect

The unchanged `docs/design/prototype/index.html` (SHA-256
`b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`)
defines `.statusbar`. The previous native footer contained the unrelated static
`OpenCPN navigation` caption and a large System button. It omitted position,
course, navigation state and the health entry and did not reproduce the HTML's
three measured-width groups.

The source remains unchanged. Computed CSS, including specificity, requires:

| Property | Exact prototype value |
|---|---|
| Frame | bottom 0, left 0, right 0, 34 px high |
| Layout | flex, vertically centered, space-between, horizontal padding 20 px |
| Background / upper rule | `--bg` / 1 px `--line` |
| Default text | 9 px, weight 400, `--muted` |
| Bold text | 9 px, weight 500, 0.02 em tracking, `--secondary` |
| First bold within each direct span | **7 px**, weight 500, 0.1 em, `--mint` |
| Family | Segoe UI Variable Display, Segoe UI, Arial, sans-serif |
| Group gap | 8 px |
| Separator | `--line`, 5 px margin on either side, in addition to group gap |
| Status dot | 4 × 4 px, `--mint` for current measured data |
| Health link | unboxed text, browser-equivalent 1 px vertical / 6 px horizontal padding |
| Hover | brightness 1.08, 160 ms CSS ease transition |
| Compact breakpoint | middle COG/XTE group hidden at width ≤1100 px |

The 7 px rule applies to both navigation-state and COG values, despite the
later general 9 px declaration. The native implementation follows that actual
computed result, rather than silently redesigning the reference. Below 760 px
the HTML hides its footer; that phone layout is outside the supported native
boat workspace. At 1280×800, the native footer remains 34 DIP; 125/150% Windows
DPI therefore reduces the logical workspace and hides the middle group.

## Native component and owned values

`XNavStatusFooter` draws the three groups with shared theme roles and fonts.
Group placement uses measured text widths and equal free space. The health link
uses the existing XNavButton input semantics, with its actual rendered bounds
available to interaction evidence. No control or OpenCPN object pointer reaches
`FooterView`.

`PresentFooter` consumes the accepted owned Vessel Data, anchor and source-health
presentations. It preserves their existing freshness and coordinate-pair rules.
It does not renew observations when read, invent NMEA 2000 provenance for a
selected navigation sample, or infer vessel motion from a position alone.

| Observation | Presented state |
|---|---|
| Current coherent selected GPS | EXPLORING, actual formatted coordinates |
| Valid current owned route progress | ROUTE ACTIVE |
| Retained/changed/stale route awaiting coherent progress | ROUTE WAITING |
| Existing OpenCPN anchor watch selected, current GPS | ANCHOR WATCH |
| Missing, stale, invalid, estimated or uncertain position | NO POSITION, explicit quality text |
| Measured current COG | actual normalized course; heading is never substituted |
| Stale COG | STALE |
| Missing/invalid/estimated/uncertain COG | unavailable em dash |
| XTE | unavailable em dash in every mode |
| Historical/test input | explicit REPLAY / TEST DATA / HISTORICAL |

Coordinates use locale-independent degrees and thousandths of minutes, with
carry at 60 minutes and true hemisphere formatting. The health summary counts
named onboard **measurements**, not transport connections. It says `Vessel data`
and reports current/stale/aging counts. Online AIS stays independent in the
health drawer and cannot establish onboard receiver health. Illustrative HTML
`UNDERWAY`, `NMEA 2000`, `9 of 10 sources` and `0.02 nm` are never production
values.

### XTE boundary inspected, deliberately not expanded

The pinned OpenCPN 5.12.4 `model/include/model/routeman.h:230–233` has
read-only `GetCurrentXTEToActivePoint()` / `GetXTEDir()` accessors, but the
accepted OpenNav route-progress snapshot contains no XTE. Upstream virtual-leg
origin and waypoint-transition coherence would need to be included in a future
owned XTE contract (`model/src/routeman.cpp:1010` can reset the virtual leg
origin). This footer neither calls navigation processing nor borrows
an unqualified current getter; it keeps XTE unavailable. That future work is not
silently added to this issue.

## Recovery and diagnostics

Visible path: Settings → System → **Interface & recovery** (first item).
Existing Open Legacy OpenCPN, Restart XNav, Safe Mode, Diagnostics and diagnostic
folder controls remain on the System page; Ctrl+Shift+S remains available. The footer
health link opens the existing source-health drawer.

Stable evidence identities:

- Native panel label/name: `OpenNav status footer`.
- Health control label `Source health`, accessible name `Footer source health`.
- `runtime.display.footer_region`: actual physical screen bounds.
- `runtime.display.footer_middle_visible`: actual responsive visibility.
- `runtime.navigation_footer`: presented values, quality states, historical flag
  and source-health summary; no raw OpenCPN handles.

The harness must use actual visible recovery pointer navigation and pair native
footer geometry with diagnostic geometry. Existing chart/coastline/palette
checks are retained. Linux component tests use equivalent logical workspaces
for 125/150%; they are not native Windows DPI acceptance.

## Review status

Implementation and focused Linux validation are recorded below when executed.
Full integrated Linux regression, native Windows MSVC/screenshots/DPI/recovery,
and the physical 1280×800 boat display remain required acceptance gates. No
claim of final prototype conformance or release qualification is made here.


### Focused Linux evidence (working tree; not release acceptance)

- Fixture-free standalone wx build: `build/footer-components`, targets
  `footer_view_tests`, `source_health_view_tests`, `status_footer_test`.
- CTest: **2/2** selected provenance tests passed; footer test contains **44**
  assertions covering invalid/missing/aging/stale/future/estimated positions,
  invalid COG, coordinate boundaries, route invalidation/deactivation, retained
  snapshot lifetime, historical input and independent onboard/online health.
- Dedicated offline footer component: **98 checks**, **10 actual screen captures**,
  with OS pointer hit and real mouse/Enter/Space activation. Equivalent logical
  workspaces 1024×640 and 853×533 keep both outer groups visible and hide COG/XTE.
- Final component evidence:
  `evidence/local/scrum99-footer-components-pass4/capture.json` and `comparison/`.
  This reports the base commit and dirty working-tree status; it is not falsely
  presented as qualification of a committed release executable.
- Fresh immutable reference captures:
  `evidence/local/scrum99-footer-reference/`, all three light modes.
- Paired Day/Dusk/Night comparison uses **literal illustrative HTML values only
  inside the non-installed test executable**. The product's owned model still
  cannot supply synthetic XTE, source counts or an invented UNDERWAY state.
- First test attempt exposed the test frame's automatic single-child resizing;
  its failure remains in `scrum99-footer-components/`. The component fixture was
  corrected to host the footer in its actual bottom region. The next visual
  review found an unwanted auto-focus border; mouse focus now preserves the
  unboxed prototype link, while keyboard focus remains visible.
- Final Linux review: no footer overlap, correct backgrounds/rules and distinct
  stale state; responsive outer groups remain visible. Shared Linux font metrics
  still yield left-group width **210 px vs 201.71875 px** in Chromium and health
  width **180 px vs 175.09375 px**. Middle x is **587 vs 586.515625 px**. Exact
  differences are retained, not masked or accepted using a broad pixel threshold.
  Windows uses the existing DirectWrite fractional measurement path and must be
  measured separately before typography acceptance.
- Coordinated harness-only evidence:
  `evidence/local/scrum99-harness/results.json`: chart geometry 29 rejection cases,
  chart ink 54 checks, Windows summary/page/drawer/footer/pointer checks
  23/18/4/3/10 and diagnostic geometry 19. No application run is implied by those
  pure harness checks.
- New native prototype-workflow step builds/runs `status_footer_test`, renders the
  unchanged reference, asserts actual pixel presence/theme and geometry, and
  uploads reference/current/diff crops with the existing evidence artifact.

**Still pending:** full integrated regressions, exact-commit native MSVC run,
Windows typography and 100/125/150% DPI, installed mode/recovery paths, and boat
1280×800 review. Footer work does not waive any existing chart-content gate.

## Exact integrated failure and correction

Candidate `88b7bbc` reaches 135 Linux integrated cases, then its recording gate
finds the footer is 1279px wide. A separate actual application capture verifies
the containing frame is 1280×800 with no X border. This is a layout defect, not
a serialization offset or a reason to relax the 1280px assertion.

Independent wxGTK 3.2.11 experiments reproduce the fixed-dock trailing stretch
spacer: 16 size/resize/hide-show observations distinguish 1279px fixed docking
from a full 1280px proportional footer. Six further observations prove theme
metric reset/reapplication and exact restoration of the original pane geometry
and perspective. Pinned OpenCPN `MyFrame::SetAndApplyColorScheme` ends with a
6px sash, despite setting it to zero earlier in the same function. XNav now
saves, zeroes, reapplies and restores that metric alongside its border metric;
only `OpenNavActions` changes to proportional docking. No upstream hook changes.

Both native prototype builds also expose a test executable entry-point error.
`status_footer_test` now follows the existing console component test pattern,
retaining stdout and actual wx event processing. Its 98 Linux checks and ten
captures pass after the change; the Windows replacement is still required.
See [retained evidence](../../evidence/prototype-native-88b7bbc-development.json).
