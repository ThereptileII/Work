# Combined SKAGER chart build — 78eccb8

The final clean application source is `78eccb8b7f21b260ded57d3ba763f884d60c8180`,
mapped exactly to GitHub `61a0a7838b56ad841bb458af6fc62651464bdafe`. All 2,472
published blobs and modes were checked against the previous verified tree plus
the exact local delta. The private integrated Linux build passes.

The build includes semantic cardinal and service artwork, land/Night palettes,
readable active-waypoint name cards, normal fixture repaint notifications and
approved SKAGER branding. Nine upstream patches apply to the pinned OpenCPN
revision. The staging guard rejects stale generated build identity; after the
last prose-only commit, explicit CMake configuration regenerated the identity
and the final incremental build completed 27 steps. The rejected stale artifact
was never staged.

`receipt.json` records exact executable, chart manifest and source identities.
`staged-inputs.json` records the development-only fixture flags. Resources were
independently regenerated and byte-compared; the pinned Standard files remain
unchanged. Runtime comparisons and Windows/boat qualification are separate
gates. This directory does not claim a release or whole-chart visual pass.
