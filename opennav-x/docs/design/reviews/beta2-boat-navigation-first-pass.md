# First installed boat navigation review — 2026-09-27

Build: `8e780edc34f68abd693a5d5f6aecdb3ba05a75c4`, fixture-free development
package; no release or physical navigation acceptance.

Reference intent: dominant chart, minimal chrome, four high-value rail items,
cyan navigation accent, technical details confined to Diagnostics.

Observed on the real PC: a 1280×800 frame at DPI144 on a 1920×1080 desktop.
Real detailed chart content, ownship and saved marks are visible. The right rail
keeps SOG, depth, wind and heading within the viewport. Heading says Estimated;
battery/motor data and dependent energy estimates remain unavailable. There are
no Demo controls. The saved Legacy Dashboard floats over approximately the left
320 pixels and continues below the frame, obscuring chart and XNav actions.
The Windows firewall sheet initially obscured the opposite side; one reviewed
Cancel click removed it. No network permission or actuator command was granted.

Changes being validated: bundled Dashboard window registration at the integration
boundary, temporary XNav-only suppression, exact Legacy workspace restoration.
The plugin continues receiving marine data. Unknown plugin windows are untouched.
The System → Legacy path retains access to traditional plugin instruments.

Evidence: `docs/evidence/beta2-boat-first-xnav-8e780.json` contains hashes and
redacted observations. Actual chart screenshots, coordinates, source addresses
and raw diagnostics stay in private local evidence and on the boat PC.

Next pass: replace only after native gates and a fresh installed plugin audit;
review the unobstructed Navigation surface, all primary screens, mode restarts,
chart detail and source-health behavior. This first pass does not close those gates.

The private reviewed Day image also establishes a deterministic same-view pixel
reference using the existing `tools/chart-render-check.py`: land `c8b87a` is
47.4% and water `73b5ee` is 26.8% of its fixed interior samples. Both must remain
present after a same-palette/view restart; all-water/blank captures fail. This
coarse check complements human review and does not override the separately
observed Dashboard obstruction or prove chart correctness. Night/changed-view
comparisons require their own reviewed reference; no color-model substitution.
