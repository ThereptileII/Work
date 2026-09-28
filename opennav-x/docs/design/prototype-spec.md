# Prototype specification — supplied design suite v8

The immutable [HTML](prototype/index.html) is the visual contract. Its SHA-256 is
`b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`.
All 113 supplied files match the supplied ZIP and the committed Git blobs;
[original-file manifest](prototype-original.json) records sizes and hashes.
`archive/` is historical, not an alternative current design. Never run the
prototype build in this directory: it rewrites the original HTML.

The cascade in the HTML wins over earlier declarations and the supplied v3
token JSON. [Extracted tokens](prototype-tokens.json) include inherited theme
values. The renderer records computed styles, bounds, visible interactions,
actual platform fonts and screenshot hashes for each state. These measurements
are the exact implementation target; an old Beta screenshot is not a baseline.

`tools/prototype/extract-tokens.py --check` verifies the final computed variables
against every canonical state on both platforms. The supplied HTML appends a
marker stylesheet absent from `src/style.css`; extracting that source fragment
alone misses eight final marker roles. The corrected token file includes these
roles and hashes of both canonical measurements. Original HTML remains unchanged.

## Palette

| Role / CSS variable | Day | Dusk | Night |
|---|---|---|---|
| Background `--bg` | #152326 | #1d282e | #0c1115 |
| Surface `--surface` | #1d2d31 | #25343b | #141c21 |
| Raised `--surface2` | #26393d | #30444b | #1d282f |
| Separator `--line` | #35464a | #405059 | #29353b |
| Primary `--text` | #f3f5ee | #e2e5db | #b8b5a7 |
| Secondary `--secondary` | #aabdbd | #acb9b7 | #91988e |
| Muted `--muted` | #7e9699 | #819394 | #747d77 |
| Active / confirmed `--mint` | #b6efce | #9bc5b1 | #85a995 |
| Navigation context `--cyan` | #7bc8d7 | #82b0c1 | #78989c |
| Advisory `--amber` | #ecc48c | #cfac84 | #aa9170 |
| Critical `--red` | #ec8f87 | #ec8f87 (inherited) | #b77569 |
| AIS `--magenta` | #cd8dac | #cd8dac (inherited) | #a77f8a |
| Floating surface | #f7f8f0 | #243a40 | #152129 |
| Floating primary | #233e3e | #e1e5d8 | #b6c3af |
| Floating secondary | #6b8380 | #a6bcb7 | #869d91 |

Day shadow: `0 6px 30px #13383514`; Dusk: `0 8px 35px #06151b25`;
Night: `0 8px 35px #0003`. Components retain individual transparency and
selection rules; do not replace them with one opaque accent fill.

## Typography

Exact stack: `"Segoe UI Variable Display", "Segoe UI", Arial, sans-serif`.
`font-synthesis:none`; numeric clock/primary measurements use tabular numbers.
Use the Windows-installed font, selecting the same first available family as
the HTML. Do not redistribute Windows font files or copy them onto Linux.
Microsoft's [font redistribution FAQ](https://learn.microsoft.com/en-us/typography/fonts/font-faq)
requires separate rights for bundling; Segoe UI Variable is not licensed there
for non-Windows use. No font binaries are included in OpenNav.

Linux reference captures explicitly record their fallback family and do not
qualify Windows typography. Windows CI and the boat must record the resolved
family; Windows 2022 and the boat can have different installed stacks.

The root eyebrow is 10 px / 650 / 0.13em tracking. Primary numeric rail values
are light, not bold. The final CSS overrides earlier labels: navigation labels
10 px; metric labels 11 px; timeline main 13 px, secondary and time 10 px;
follow 11 px; footer 9 px. Context headings use their computed weight and size
from `capture.json`, including active media rules. Do not interpret CSS pixels
as typographic points. At the native boundary Windows uses the scaled negative
LOGFONT character height; GTK uses the equivalent fractional point em
(`px × 72/96`), since GTK's pixel-size constructor fits the taller line cell.
Uppercase applies only where the reference uses it.

## Geometry at 1280 × 800, scale factor 1

| Region | x | y | width | height |
|---|---:|---:|---:|---:|
| Top bar | 0 | 0 | 1280 | 68 |
| Left navigation | 0 | 68 | 80 | 698 |
| Chart | 80 | 68 | 1014 | 566 |
| Data rail | 1094 | 68 | 186 | 698 |
| Horizon | 80 | 634 | 1014 | 132 |
| Status footer | 0 | 766 | 1280 | 34 |

Context drawer: 398 px; configuration drawer: 432 px; radius 18 px. Chart
context/dashboard cards use radius 14 px. Final base buttons use radius 9 px, icon buttons
9 px (floating map buttons 12 px); selected navigation item radius 10 px.
Final base button/list row minimum height is 48 px; map icon buttons 44×44;
segments minimum height 40 px. Preserve the prototype's actual hit regions,
including the toggle's expanded hit region; do not enlarge all controls in a
way that changes its composition. Physical touch validation remains required.

Spacing is component-specific: top padding 0 18px 0 21px, gap 18px; sidebar
padding 14px 9px, navigation gap 5px; icon size 22px with 1.65px round stroke.
The reference includes optical adjustments outside an 8px grid. Copy them,
rather than rounding everything to the previous Beta spacing constants.

## Motion and input

Base button transitions: background, color and opacity, 0.16s with default
CSS `ease`. Enabled hover uses `brightness(1.08)`; disabled opacity is .38.
Focus ring is 2px mint with 4px offset. The map changes grab/grabbing/crosshair
with operation. Reduced motion disables animations/transitions and pauses the
mock radar sweep. Context navigation and cancellation follow
[prototype interactions](prototype-interactions.md).

## Rendering and acceptance

`tools/prototype/render.py` uses Playwright 1.58.0, its pinned Chromium build,
1280×800, DPR1, offline file loading, a fixed clock and reduced motion. It
checks all original hashes before/after, refuses network requests/page errors,
and asserts exact main-region bounds. It only operates supplied UI controls.
Generated evidence belongs below `prototype/reference/`; originals stay intact.

Native comparisons must use the same OS font environment and content fixtures
in dedicated test executables. Real ENC geography cannot equal the fictional
SVG pixel-for-pixel; compare chart styling separately using documented swatches,
symbol/line probes and actual ENC review. Never mask the surrounding layout or
use a broad pixel tolerance to call a different native screen conformant.

Prototype hardware/radar/sensor simulations, synthetic values, mock installer
progress and optimistic corridor text are design evidence only. The product
continues to withhold unsupported actions and unavailable/uncertain predictions.
