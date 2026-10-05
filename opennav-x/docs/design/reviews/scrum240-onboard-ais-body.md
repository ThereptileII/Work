# SCRUM-240 — healthy onboard AIS body presentation

This bounded SCRUM-15 child starts from `1a02ae083501eb05feeef7879e6ee450c076be69`.
The immutable HTML has SHA256
`b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`.
Its AIS body path is `M0-12 6 9 0 5-6 9Z`, 12×21 logical pixels, with
1.6px plum `#916477` stroke and the theme's floating-surface fill. Night's
ancestor brightness(.78) applies only to this new body artwork.

The hook is confined to the existing ordinary ship-body draw in pinned
`gui/src/ais.cpp::AISDrawTarget`. It copies appearance values at paint time;
no upstream target pointer or additional AIS state enters UI/VesselData.
Only verified active SKAGER presentation enables the helper. Standard,
Legacy, Safe and missing/changed presentation-resource fallback retain stock
rendering through the existing `ChartBackground` gate.

Eligibility is deliberately narrow: active, not lost, once-valid position,
not doubtful, valid uncached name, ordinary Class A/B navigation status,
no alert, non-Inland/non-Euro-Inland, no SAR aircraft, follower or blue paddle,
and the upstream renderer must have resolved a valid direction. Class A permits
engine/sailing; Class B also permits its usual unspecified navigation status.
Every status with a native navigation glyph, HSC ship types 40–49, unknown
Class A navigation status and all special target classes retain stock bodies.
When real-time extrapolation is enabled, stock rendering is retained as well,
so the projected ghost and actual body continue to agree. Existing lost-target
suppression occurs before this hook and remains unchanged.

Class B uses the exact prototype notch. **Class A retains a straight stern**:
the prototype depicts every vessel identically, while OpenCPN distinguishes
Class A and Class B by their stern. Removing that distinction would discard
meaning. Both now use the prototype width/height and ink when eligible. User
ship scaling and the 50–100% importance attenuation multiplier remain intact;
the stock physical-size heuristic is replaced only for this base body by
logical prototype pixels multiplied by actual canvas DPI. Real-size outlines,
position, viewport rotation, resolved HDG/COG angle, hit testing, query/SKAGER
selection frames, names, track history, CPA/ROT/navigation overlays and all
predictors remain upstream. Selection framing is retained; introducing the
prototype's selected fill is outside this increment.

The body and centered miter stroke are tessellated explicitly. Software uses
the shared wxGraphicsContext painter, and GL draws the same triangles with the
existing solid-color shader. No four-point strip special case can fill the
concave notch. The helper restores renderer state and updates dirty bounds;
it changes no global AIS semantic palette, configuration or target data.
Optional name styling was excluded because it has separate visibility/font
and density behavior; it is not needed to establish this body-only boundary.

## Focused development evidence

`tests/onboard_ais_body_test.cpp` is a standalone offline executable, also
declared as `onboard_ais_body_test` in the UI test build. Run with one output
PNG path. It tests every class/navigation-status combination across the pinned
range, alarm and HSC status, individual semantic refusal conditions, invalid
geometry, three rotations and four scales against an independent even/odd SVG
interior, and actual wx painter pixels/state in all three themes. Enlarged
proof symbols and default-size samples are explicitly labelled offline.

The [retained evidence](../../evidence/scrum240-onboard-ais/review.json) records 24,751
passing checks, the exact production compile commands and
dependency/source hashes, executable/image hashes and check count. Both the
new helper and patched production `ais.cpp` compile against prepared pinned
headers with `-Werror`, preserving the existing production feature macros.
`prepare-integration.py` verifies the full private patch result. No root/frozen
build output was changed.

A private negative-control copy deliberately fills the Class B notch using
the wrong triangle diagonal. The independent SVG comparison rejects it; the
production source remains unchanged. The retained Day/Dusk/Night PNG was
visually inspected, including default-size and rotated samples.

This is Linux software-painter and geometry evidence. It does **not** qualify
actual OpenGL output, native Windows DPI, live-traffic overlap/selection,
theme/mode-return behavior, physical boat display or whole-chart conformance.
Native/boat comparisons, including fallback and alarm/special-state targets,
remain required. SCRUM-240 stays in Testing and SCRUM-15 remains open.
