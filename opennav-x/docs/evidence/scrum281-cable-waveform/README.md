# SCRUM-281 cable waveform: rejected retained-scale experiment

Base48c2f8b. No application/resource implementation is committed here. The
resource-only experiment was removed from tracked product files after visual
review exposed a material scale mismatch. Original prototype, source resource,
root checkout and frozen boat candidate remain unchanged.

The exact supplied `line:CBLSUB06` path is
`M-12 0q3-5 6 0t6 0t6 0t6 0`. Its final opaque area stroke is1.3 CSS units,
with round caps/joins; `.chart-symbol-line` opacity.16 belongs to the separate
sample supporting line. Art source SHA256:
`be2c7f44817c29d3714412c7974d3cf07e238f65c7131958b0cb8ff6fbd7816d`.

The experiment kept original vector width2293,height500, origin692,1050,
pivot448,1274 and min/max distance0. It placed an isotropic2293/24 approximation
at origin692,1300 using64 line segments.149 focused resource/analytic/inverse/
negative checks passed, with measured maximum1.220752 HPGL-unit deviation and
analytic bound1.640131. This does NOT qualify a painter or a useful visual size.

At the explicitly illustrative96DPI (96/25.4 pixels/mm), the native width is
86.6646px versus the supplied24CSSpx. Modern repeat2537 yields95.8866px;
legacy repeat2293 yields86.6646px. The original/current-muted stock, rejected
approximation and prototype appear at their actual respective scales in
`comparison.png`; none is resized to disguise that difference. These are SVG
geometry comparisons, not application screenshots or actual HPGL output.

The loader fixture attempt compiled actual shared core/private XML loader
methods but failed while compiling unchanged pugixml at-O3/Werror with GCC's
`maybe-uninitialized` diagnostic. The original log is retained. No compiler
warning was suppressed; no loader execution, actual SW/GL rendering or native
acceptance is claimed. The discarded implementation and fixture source remain
ignored in the isolated worktree `.local/rejected-wide-source`.

## Proposed correction, not yet implemented

Use owned presentation width635HPGL (6.35mm=24px at96DPI), pivot0,0,
origin0,-84,height168. Map x=(prototypeX+12)*635/24 and y=prototypeY*635/24.
The centerline reaches y±66.1458; including half the1.3CSS stroke reaches
±83.34375. This explicit presentation-density change makes both unchanged
repeat formulae yield635 because originX-pivotX becomes0. It does not change
chart geography, masks, priority, tangent selection, clipping, short-segment
branch or remainder policy. It is a proposal requiring review, not a tested
metadata replacement.

Exact width/caps require a bounded painter hook: pinned HPGL `SW` parses a
long, SetPen quantizes widths, and GL uses individual GL_LINES. SW1.3 is not a
solution. All modern/legacy/plugin LC paths already converge on the two
HPGL draw calls in `draw_lc_poly`. A verified-owned exact-rule hook there can
paint the supplied quadratic path at1.3*PPMM/(96/25.4), using original anchor,
tangent and SCAMIN transform, with the original HPGL fallback. A small coverage
image and the existing reviewed RGBA texture-state restoration boundary can
provide fractional width and round joins/caps. Preserve Standard/Legacy,
CATCBL6 lookup709 and every unrelated rule. No hook has been implemented.

Required next proof would cover actual traversal/fallback and painter bounds,
alpha/GL-state restoration and bounded local tile/upload cost. Native actual
chart scales/themes/renderers and boat review remain open. No current artwork
conformance or final completion claim.
