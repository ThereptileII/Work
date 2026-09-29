# SCRUM-100 — prototype navigation horizon

Implementation review; **native Windows and boat conformance remain pending**.
The immutable `docs/design/prototype/index.html` (SHA-256
`b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`)
is unchanged. This correction is limited to the horizon and its existing context
entry points; it does not extend SmartNav navigation calculations or controls.

## Reference and prior defect

At 1280×800 the HTML horizon occupies `(80,634,1014,132)`. Its content has 25 px
side padding and `.8fr 1.12fr 1.12fr 1fr` columns, not four equal cells. The
previous native horizon omitted the Spark heading icon, advisory separator,
Full passage action, vertical event rules and all four event interactions.
It also excluded severity and identity from its invalidation signature.

The final CSS cascade sets heading typography to 9 px / weight 650 / `.13em`
tracking, advisory text to 9 px, Full passage to 11 px, event time to 10 px
(NOW 8 px), event title to 13 px / weight 550, and event details to 10 px. Event
icons are **hidden** in the HTML; the markers remain circles. The narrower and
shorter desktop breakpoints are retained. Computed reference capture metadata
is the independent geometry oracle; pixel differences are retained for review.

## Implementation boundary

`application/HorizonView` owns only text, semantic marker/severity, and copied
route/AIS identities. It preserves the existing SmartNav event order and the
previous bounded three-advisory overview (ArrivalSoc/Reserve remain in their
existing detail views). It does not invent absolute arrival times or assert
that the vessel is on course. Current motion, aging, unavailable and historical
observations remain explicit. No observed timestamp is renewed on read.

The model requires the current advice evaluation and matching route revision
for route events. AIS advice retains the source's existing CPA/TCPA freshness
budgets; a valid advisory with unavailable target position remains visible, with
its context action disabled. Online traffic cannot substitute for the original
onboard target supplying a SmartNav alarm.

Full passage opens the existing passage drawer. NOW invokes the existing follow
callback only with current coherent position. Route cards open passage details.
AIS cards use the current owned local target identity through the existing AIS
selection/details path. Shell re-reads owned input at activation; changed route
revision, stale or ambiguous target, missing position, replay and test data
cannot activate an outdated context. A changed identity between pointer press
and release cancels that activation. No upstream processing is called as a getter.

The native component reuses XNavButton pointer/keyboard behavior, draws the
prototype heading, fractional grid, timeline rules, markers and focus outline,
and includes severity and identity in equality. Identical presentation data
causes no new layout or redraw. It has no live hardware or WebSocket interface.

## Evidence contract

`horizon_view_tests` runs deterministic owned-model and click-revalidation
checks, and is also attached to the integrated upstream test target. The
standalone `horizon_test` process exercises actual pointer, Enter, Space and Tab
input; captures current, unavailable, aging, stale, replay, severity/identity
changes; and verifies 100/125/150-equivalent workspace geometry. Those reduced
workspaces are **not** native Windows DPI or physical-touch qualification.

Literal prototype text appears only in the non-installed component executable,
with `prototype-fixture-day/dusk/night` filenames. The product cannot emit the
illustrative On course assertion, fixed timestamps, destinations or battery
prediction. Each capture records screen-space horizon, heading, Full passage,
and event geometry in `result.json`. Reference/current/diff artifacts preserve
mismatches; no broad image tolerance qualifies a changed layout.

Native Windows exact-font comparison, integrated visible context flows, actual
125/150% DPI, physical boat 1280×800 review and touchscreen remain mandatory
before the Jira issue can be accepted. Existing recovery, installer, chart and
physical-output safety gates remain unchanged.

## Local development validation

The isolated worktree passed 81 portable Horizon model checks and eight related
CTest entries (Horizon, passage, footer, source health, and the four existing
SmartNav suites). The offline native Linux component passed 132 checks and
captured 14 states. Development records are in
`evidence/local/scrum100-contract/` and `evidence/local/scrum100-component-6/`.
They identify the dirty development tree and executable hash; they do not
qualify a release commit.

Reference/current/diff review corrected omitted inline whitespace, the
prototype's final `syncNavigation()` text changes, native drawing offsets that
made 1 px rules occupy two rows, and cumulative heading glyph rounding.
The sampled day rules now exactly match the HTML `(53,70,74)` at rows 0 and 47.
Recorded component/control geometry differs by at most 0.44 physical px from
the independently measured canonical DOM. Linux glyph rasterization/metrics
still differ visibly; native Windows font review and boat capture are pending,
and no visual-conformance PASS is claimed.

## First native replacement failure

Exact remote `ced99ac379e5622c21e48c83772b43aa7f2e8df6` (local `3fb6635`)
fails both native prototype builds at the Windows screenshot branch in
`tests/horizon_test.cpp`: MSVC reports undefined `wxMemoryDC`. The fixture
relied on a transitive include that Linux supplied; its explicit
`wx/dcmemory.h` dependency is now declared. Both downloaded native artifacts
contain build logs and provenance only, with no screenshots. See
[the retained compile failure](../../evidence/scrum-100-horizon-ced99ac-negative.json).
The replacement build, all component captures, typography/DPI review and boat
qualification are still required. This include correction changes no product
behavior or rendering expectations.
