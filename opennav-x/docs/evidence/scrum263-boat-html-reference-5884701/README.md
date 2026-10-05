# SCRUM-263: boat-machine immutable HTML reference

On 2026-10-03, one source-locked headless Edge run produced the original navigation Day, Dusk and Night reference at 1280 × 800, DPR 1. This is the immutable illustrative prototype, including its original **opennavx** logo and invented reference navigation data; it is not a current SKAGER product screen or a native chart acceptance result.

Original PNGs, directly fetched and independently rehashed:

- [Day](capture/navigation-day.png)
- [Dusk](capture/navigation-dusk.png)
- [Night](capture/navigation-night.png)

All three originals were visually reviewed. [capture.json](capture/capture.json) retains complete computed styles, CDP font observations, screenshot hashes and the embedded runtime receipt. [verification.json](verification.json) binds the fetched original bytes and sanitized before/after checks. [completion.json](completion.json) and both logs retain the actual successful run.

## Observed faces

These are actual first-matching-node CDP observations, identical across the three themes. Glyph counts establish that these particular samples rendered; they do not establish all instances or hidden roles.

| Sample | Actual face | Computed size / weight / tracking | Glyphs |
| --- | --- | --- | --- |
| Primary metric value / label | Segoe UI Variable, Display | Original root hierarchy | 5 / 17 |
| `.chart-label` | Segoe UI | 12 px / 400 / 1 px | 11 |
| `.chart-water-label` | Segoe UI Variable, Display | 16 px italic / 400 / 5 px | 10 |
| `.chart-depth` | Segoe UI Variable, Display | 10 px / 400 / normal | 4 |
| `.chart-landmark-label` | Segoe UI Semibold | 8 px / 600 / .12 px | 12 |
| `.map-pop-label` | Segoe UI Variable, Display | 10 px / 400 / normal | 10 |
| `.map-waypoint text` | Segoe UI Variable, Display Bold | 8 px / 650 / normal | 2 |

The `.chart-symbol-label` observation selects the **same dual-class landmark** as `.chart-landmark-label`: the component bounds and styles are identical. Original `index.html` creates `class="chart-symbol-label chart-landmark-label"` for that landmark. Its 600 weight is not evidence of a normal LIGHTS-description defect. No normal-only symbol-label sample was collected. This distinction is already specified in [the LIGHTS typography review](../../design/reviews/scrum248-light-description-typography.md).

The visible vector `.chart-depth` sample does not activate the raster-mode Georgia rule. Hidden `.light-sector-readout small` has zero glyphs and establishes no rendered font. Missing and zero-glyph observations remain in the original JSON. Current native code has separate ordinary chart, geographic land/water, generated LIGHTS and sounding policies; this reference does not observe their actual native resolution. Historical earlier font reviews remain unchanged.

## Identity and preservation

Tooling commit `5884701a7abd58a70c6dc0ffa494bdc262c30ad3` (root integration `3ffcc22`) used the exact 113 original prototype files. HTML SHA-256 is `b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`. The 61,590,914-byte sealed bundle has SHA-256 `2cd479bd71c2e541a487edab63797b7bac872408c94bebbd55f799e7ed1890e5`; all 124 ZIP CRCs and payload hashes were checked before execution. Full dependency wheels and licenses were retained unchanged.

Actual runtime: installed Edge **154.0.4258.48**, Python **3.13.5 x64**, private Playwright **1.58.0**. The existing Edge launcher hash remained unchanged. No browser or fonts were installed. Dependencies were staged only in the owned private tools directory. Browser contexts were offline; each state recorded two identical file-request events for the sole original HTML URL, with no dependency font asset requested by the reference page.

Native exit was 0. The normal OpenCPN profile retained SHA-256 `d891d88c62657139e1b1c6ff7d9e8acdae726a4e116dbc844126992a39adbdb6`; OpenCPN process count was zero before and after. All seven existing Edge process IDs were preserved. After capture, no owned browser descendants or owned browser temporary directories remained. The owned runtime/evidence directory remains retained. The display report remained 1920 × 1080; actual interactive-window DPI was not measured or changed.

The successful run exercised owned Windows Job Object creation and closure. A forced timeout was not injected. Runtime files were remotely checked before/after by the source-locked runner; the downloaded receipts and seven evidence files were independently rehashed locally. Runtime binary bytes were not downloaded for a second independent hash calculation.

## Preserved transport limitation

The initial SCP transfer reached its 180-second wall timeout before any browser/runtime execution. The retained 61,363,200-byte partial SHA-256 matched the exact local ZIP prefix. After an unused fresh-file retransmit was stopped, SFTP resumed the verified owned partial with the remaining 227,714 bytes. An initial SFTP path spelling refused `stat`; the corrected `/C:/` spelling succeeded. The complete remote ZIP was rehashed before the **single** actual capture attempt. This was a transport recovery, not a renderer retry.

No application package, normal browser profile, chart/profile settings, remote-access service, display configuration or hardware state was changed. These reference images do not qualify physical GPU rendering, native application fonts, charts, private renderer loading or boat navigation.
