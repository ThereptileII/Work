# SKAGER integrated chart review — 2026-10-02

This is a development review, not complete prototype conformance.

Source: local `1a02ae083501eb05feeef7879e6ee450c076be69`, published equivalent
[`280d5e2`](https://github.com/ThereptileII/Work/commit/280d5e2e28570fed4f2b84eaafc104bad9ee7c1f).
The [identity manifest](../../evidence/skager-product-fidelity-1a02ae0-linux/identity.json)
ties the six images and actual application diagnostics to their exact bytes.
The [receipt](../../evidence/skager-product-fidelity-1a02ae0-linux/visual-review.json)
records binary, resource, patch and font identities, build flags, clean exits
and the cache-preparation deviation. Both reviewers inspected all six images.

## Reference intent and observed changes

- The approved SKAGER / APP wordmark replaces the development identity. It
  preserves the supplied artwork; its dark rectangular background remains
  visible against the Night header. This is an outstanding visual mismatch.
- Real NOAA US5SEAFL ENC land and built-up areas use the prototype's restrained
  palette in Day, Dusk and Night. Geographic names have lighter, smaller type;
  water names are italic. Standard preserves its original palette and type.
- The real OpenCPN COG predictor now uses the prototype's thin, muted dashes.
  Its stock red endpoint marker remains visible. The prediction still derives
  from OpenCPN's observed motion and configured duration.
- Coastline and nautical objects remain visible in every image. Soundings,
  overlapping light-characteristic labels, chart-selector chrome and several
  symbol/overlay states still look unlike the illustrative HTML. No chart
  data or safety-relevant labels were hidden to obtain a quieter screenshot.

## Evidence limits

The 1280×800 captures use software rendering on Linux with Liberation Sans
fallback, not native Windows Segoe metrics. The exact integrated build passed
and both application runs exited cleanly. Input was explicitly controlled
loopback RMC through the existing OpenCPN navigation path (54 sentences, two
connections, no input errors). No Demo, replay or pilot commands were enabled.
Test fixtures and pilot-loopback support were compiled into this development
binary; it is not an installed-product or release build.

There was no active route or AIS target in this viewport. Route/waypoint/AIS
appearance therefore needs its own capture; these images cannot qualify it.
Native MSVC, OpenGL, installer, Windows DPI and physical boat tests remain open.
All screen-level conformance records remain pending under SCRUM-15.

## Images

| Style | Day | Dusk | Night |
|---|---|---|---|
| SKAGER | [Day](../../evidence/skager-product-fidelity-1a02ae0-linux/SKAGER-Day.png) | [Dusk](../../evidence/skager-product-fidelity-1a02ae0-linux/SKAGER-Dusk.png) | [Night](../../evidence/skager-product-fidelity-1a02ae0-linux/SKAGER-Night.png) |
| Standard | [Day](../../evidence/skager-product-fidelity-1a02ae0-linux/Standard-Day.png) | [Dusk](../../evidence/skager-product-fidelity-1a02ae0-linux/Standard-Dusk.png) | [Night](../../evidence/skager-product-fidelity-1a02ae0-linux/Standard-Night.png) |
