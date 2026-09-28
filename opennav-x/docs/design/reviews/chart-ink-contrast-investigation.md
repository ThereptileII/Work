# General ENC ink on prototype backgrounds — investigation

The native `ad4913f` public US5SEAFL Night capture still has nearly invisible
chart labels. This is not accepted as a navigational presentation. Source
inspection confirms that changing DEPDW without changing CHBLK combines the
prototype's lighter water with the stock library's very dark monochrome ink.

Measured sRGB luminance contrast (a diagnostic, not chart certification):

| Theme | Stock CHBLK on XNav deep water | Prototype chart-text ink on same water |
|---|---:|---:|
| Day | 15.509 | 3.438 |
| Dusk | 1.288 | 4.368 |
| Night | 1.060 | 4.356 |

A blanket substitution is inappropriate: Day's prototype muted text drops to
1.827 on the very-shallow fill, versus 8.240 for pinned Day ink. Dusk's muted
chart-text also loses contrast on the brightest shallow fill. Preserve Day
monochrome ink; investigate Dusk's existing brighter floating-text token and
Night's chart-text token, with independent shallow/land checks. These are
navigation-legibility exceptions to a fictional map, not new symbol meanings.

The same CHBLK role is used by general text and monochrome vector instructions.
Do not change lookups, conditional symbology, chromatic buoy/light meanings or
proprietary charts. Stock raster symbol sheets are a separate limitation: XML
palette changes do not automatically recolor their baked pixels. They require
actual hazard review, not a claim inferred from a contrast number. Standard
presentation must remain byte-identical and available.

The bounded second pass now keeps Day #070707, uses Dusk --float-text and Night
--chart-text. The 377 generated-resource checks pass, including deep-water,
very-shallow and land contrast. Actual public ENC software/GL captures, native
Windows comparison and boat display review remain required before acceptance.
Other chart text scale/density and built-up-area differences remain open.

Third local pass: narrowly derived Dusk/Night neutral raster ink, with all alpha,
geometry and colored pixels unchanged (42,100 pixels each). The 388 resource
checks pass, including independently decoded golden RGBA hashes. Night captures
under `evidence/local/prototype/raster-ink-{software,opengl}` were inspected.
Small monochrome marks are now visible, but the GL cached-text path remains
black while software text uses S-52 ink. Pinned `s52plib::RenderText` confirms
that the GL cached-glyph branch omits software's default-black-to-LUP fallback.
A scoped replacement and further captures are required. Large "Feet" comes
from `ChartCanvas::EmbossDepthScale`, preserving actual quilt depth units.
Built-up areas, density, depth-unit presentation and full hazard review remain
open. Neither renderer is visually accepted.


Fourth local pass scopes the cached GL default-color fallback to the verified
XNav library only. The Night general text is visibly restored; Standard/Legacy
keep their original constructor default and explicit chart-text preferences
remain honored. All 120 integrated Linux tests pass. An initial capture failed
its first zoom assertion; the collector now waits for a post-resize diagnostic
epoch and exact target canvas size. Three subsequent independent captures each
pass real pointer zoom and Day/Dusk/Night/Day with clean exit. Negative evidence
is retained. This is Linux llvmpipe evidence, not a physical GPU or Windows gate.
See [local evidence](../../evidence/prototype-chart-gl-ink-local.json).
