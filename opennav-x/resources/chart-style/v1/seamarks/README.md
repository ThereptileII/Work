# Classified marine seamark artwork

These eleven 24 × 28 RGBA tiles derive from the immutable prototype's original
`chartSymbolGraphic` output. The SVG is kept for inspection; normal resource
builds consume source-locked RGBA rows with the standard library only.
`tools/derive-seamark-art.py` reproduces them with Node, librsvg and Pillow.
Prototype sources and every derivative are hashed in `provenance.json`.

Eight private, eight-character aliases preserve the original ordinary and
preferred-channel source glyphs for inland and Paper Chart users. Only the
sixteen exact uppercase BOYLAT Simplified selectors 1029–1044 redirect. Preferred
channel aliases preserve the selected red/green/red or green/red/green bands.
BOYISD12 and BOYSAW12 have exclusive Simplified users and receive new bitmap
bounds. LIGHTS13 receives the centered circle/rays tile and the explicit
`prefer-bitmap` change from `no` to `yes`; its old vector definition remains.

The prototype uses 27/32 scale for buoys and 25/32 for LIGHTS13. Night RGB inputs
receive brightness .78 once; alpha, stroke geometry, and supplied class marks
are unchanged. These are Simplified classification marks, not assertions of a
physical installed topmark. Actual TOPMAR and light conditional logic remain.

BOYSPP11 is deliberately unchanged. Its eight existing rules do not qualify
COLOUR; white/orange and missing-color buoys must not acquire the prototype's
yellow/X mark. Generic BCNGEN01 also remains unchanged because its shared use
and physical topmark composition are not proved. Other Paper Chart geometry,
sectors, color-specific lights and unmapped glyphs retain their original rules.

The immutable prototype identifies the modern paths as original OpenNav artwork.
The source chart catalog/metadata derives from OpenCPN under GPL-2.0-or-later;
see `docs/design/prototype/SYMBOLS.md` and its retained vendor license. No
illustrative prototype position, range or sector is used as chart data.
