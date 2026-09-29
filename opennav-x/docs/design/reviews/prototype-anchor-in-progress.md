# Anchor watch — prototype migration in progress

Authority: unchanged `prototype/index.html`, `case 'anchor'`, and its original
Day/Dusk/Night renders. This is **not** visual or boat acceptance.

Reference intent: a 398px chart-side drawer; AT REST / Anchor watch header;
watch/GPS pills; 230px north-up movement graphic; prominent distance; radius;
two-column depth, true wind, battery and recorded-history values. The body
scrolls to the final watch action in the original 1280×800 reference too.

Changes: `XNavAnchorDrawer` reuses the owned native drawer and drawing primitives.
The measured content positions are pills y=0, graphic y=46, radius label y=292,
value y=318, range y=346 (46 high), statistics y=414/480, action y=560, relative
to the padded body. A shared `XNavRange` supports real pointer dragging and
keyboard steps without performing a navigation command. The chart retains its
original viewport while the sheet is open. Close/Escape return to the chart.

The prototype's fictional trail, 18m distance, 42-minute history and vessel
heading are not product inputs. Distance, movement and history come from copied
OpenCPN observations. The plot uses pinned Mercator bearing/range, not UI
geodesy. A dot represents current position without inventing a heading. Missing
or stale GPS removes the current dot and distance; retained historical points
are muted and explicitly accompanied by the position warning. Gaps over five
seconds are not connected into a fictitious continuous trail.

Safety-required interaction differences:

- Setting/stopping a real watch requires confirmation and existing integration
  validation. Merely reading or changing a proposed radius never arms a watch.
- The range prepares a new watch (20–100m, 5m steps as supplied by the HTML).
  An existing watch is read-only here; stop it before choosing a new radius.
  OpenCPN's exact stored radius remains visible even outside the prototype
  range; the slider is hidden in that case instead of showing a clamped value.
- Negative upstream radii mean an **inside-radius** alarm. The label explicitly
  preserves that meaning. Larger/special watches remain available in Legacy.
- Depth remains below transducer, true wind is never replaced with apparent
  wind, and unavailable data never becomes zero. Replay/fixtures cannot alter
  real watches. Upstream alarms are retained even when GPS becomes stale.

Initial Linux review found a locale-dependent UTF-8 format conversion crash in
the new GPS pill. A live GDB reproduction identified the new Anchor paint path
and `wxFormatString::AsWChar`; explicit UTF-8 conversion fixes the reproduced
failure. The second isolated component run passes 34 checks with five actual
screen captures, including three themes, stale GPS and inactive watch. The source-provenance correction and unified alert entry pass a second
34-check component run and 24-image product run; Windows/boat remain required.

Still pending: final exact-revision integrated captures, independent Windows
font/geometry comparison and correction, real pointer/touch review, and boat
1280×800 comparison. No broad pixel tolerance or false PASS is introduced.

Corrective review: the first integrated pass retains 24 actual product images
and confirms three Anchor themes and Close restoring the chart. The old full-page
Anchor implementation was removed; both the sidebar and an anchor alert route
to the same sheet. Normal explanatory copy uses vessel language. A history
shorter than one minute is shown as `<1 min`, rather than a misleading rounded
zero. The corrective build passes 128 tests and the repeated captures retain the
expected viewport and unavailable state in all three themes. See
[local evidence](../../evidence/prototype-anchor-local.json).
