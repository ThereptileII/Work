# Native 29ea06a Search, Shell and Chart presentation review

Read-only review of source `29ea06a358ed24cc29eb2be74b67b234a3ecaa6e`,
Windows workflow `37026807977`, artifact `11235841232`. Evidence root:
`evidence/local/prototype-native29/files/windows-changed-units/` in the
`prototype-followup` worktree. The later System flow commit `27e2d507` is
not represented by these captures.

All nine images were opened and inspected:

- `search-component/saved-objects-day.png`
- `search-component/empty-night.png`
- `search-component/shell-1280.png`
- `search-component/shell-853.png`
- `search-component/shell-853-search.png`
- `chart-presentation-component/chart-day-top.png`
- `chart-presentation-component/chart-day-bottom.png`
- `chart-presentation-component/chart-dusk-raster.png`
- `chart-presentation-component/chart-night-unavailable.png`

Chart comparison uses immutable `docs/design/prototype/index.html:326` and
canonical Windows `reference/windows/layers-{day,dusk,night}.png` and
`capture.json`. Search comparison uses the same immutable HTML at lines
335/347 and the Linux renderer outputs in
`scrum14-search-reference/evidence/local/search-reference/` (HTML SHA-256
`b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`).
That Linux reference records Liberation Sans fallback; it does not qualify
Windows font rendering. Search canonical Windows evidence remains pending.

## Concrete typography findings

1. Chart row supporting text is visibly smaller than the Windows reference,
   including the AIS description around y442 in `chart-day-top.png` and
   `chart-dusk-raster.png`. `ChartPresentationDrawer.cpp:233` renders 9px;
   the active CSS override at HTML line 60 specifies `.row small` at 11px,
   line-height 1.5 and 4px top margin. The earlier 9px rule is superseded.
2. The explanatory notes in `chart-day-bottom.png`, especially the source-health
   note around (705,574), are also undersized. `ChartPresentationDrawer.cpp`
   lines 107–115 use 9px muted text and 15px line spacing. The active HTML
   line 60 specifies `.drawer-body .note` at 11px, line-height 1.65 and
   secondary color. Any correction must preserve readable wrapping and space
   for feedback and the palette action.

## Bounded observations

Chart row boundaries, format track and available switches align closely with
the Windows reference. Managed/Unavailable states, explicit ENC master-text
semantics, observational chart format and the separate palette action are
documented correctness adaptations, not missing decorative controls.

Search retains the intended 398px drawer and 66px result rows; source and
computed reference agree on 12px/550 result names and 9px details. Saved-object
scope, route icons, empty-state copy and differing result counts are truthful
data adaptations. Linux/native header and glyph differences are not a Windows
typography acceptance finding.

The Shell images show separate 44px Layers surfaces and no overlap at the
captured 1280×800 and 853×600 sizes. Compact Search remains within the workspace
and covered chart controls are hidden. The compact sidebar visibly abbreviates
Instruments; no corresponding compact canonical capture was supplied.

The native manifests record 82 Search/Shell checks and 46 fixture-only Chart
checks; this review did not rerun them. Blank chart backgrounds and unavailable
navigation values are intentional fixture conditions. No full-view, release,
or boat-display acceptance is claimed. No product/test code was changed.
