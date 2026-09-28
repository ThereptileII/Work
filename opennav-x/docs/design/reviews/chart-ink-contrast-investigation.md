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
