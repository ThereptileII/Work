# SCRUM-304 — Anchor watch presentation

The selected upstream anchor watches now use the prototype anchor glyph and a
consistent theme on the existing radius overlay. The current watch alarm has
priority over missing-position styling; a missing position uses attention ink
and a dashed ring. Signed entry-watch semantics remain distinguishable.

The radius, center, projection and selected upstream watch identities are
unchanged. Both software and OpenGL canvases already call the same upstream
ring painter. Legacy and Standard presentation retain the original painter.

Only the exact shipped anchor artwork is eligible for replacement. Loading the
pinned SVG verifies its SHA-256 and decoded pixels; user/plugin replacement
revokes provenance. The actual bitmap identity is checked at paint time.
Custom icons, active/blinking/editing/dragging marks and unrelated anchors
retain upstream rendering. The small bounded icon cache owns only bitmaps.

## Focused verification

The standalone `tests/chart_anchor_watch` fixture passed 1/1 CTest test. It
compiles the production marker painter unchanged and extracts the actual
prepared upstream ring and icon-provenance methods. It compares geometry with
the pinned upstream function, covers both watches across Day/Dusk/Night,
alarm/GPS/entry states and fallback/custom-icon preservation. Actual changed
integration, canvas, route-point and waypoint-manager units passed local syntax
compilation. These checks do not exercise the real OpenGL context or physical
boat display. Native Windows and boat acceptance remain pending.
