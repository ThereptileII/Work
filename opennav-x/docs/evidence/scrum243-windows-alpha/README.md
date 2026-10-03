# SCRUM-243 native Windows alpha repair evidence

Only disposable Windows offline fixture inputs are retained; no chart/profile or
physical data was used. Full application/package/runtime acceptance remains open.

| Replay | Exact remote / mapped local | Result |
| --- | --- | --- |
| [Original 37093438232](https://github.com/ThereptileII/Work/actions/runs/37093438232) | b1d65269064a4b5c30b649347d5332a9a126c185 / 7b850013b5eea4910230d9903cd189f1d3a7a7f0 | Failed unchanged alpha assertion |
| [Corrected 37094223771](https://github.com/ThereptileII/Work/actions/runs/37094223771) | 7687ce53c780bdfd53db30bb063f9bcc8f6404b7 / 8b9e0a45c569d75d0c4f4c7a71a263cf5a910f95 | All 2,452 native checks passed |

Original ZIP: artifact 11263777414, 4,221,207 bytes, SHA256
`f5c30900d9819d3d1d335388628667097bdcb412d453b4c7b200d7611ccc93d0`.
Corrected ZIP: artifact 11263384386, 4,224,200 bytes, SHA256
`a5c29544630e24e7f9348b7a3ee6e74d14380f080f6bdac1326f55b362f7aced`.
All 83 / 84 archive entries passed CRC. Independent verification matched all
8 / 9 recorded source inputs to exact mapped commits (Windows CRLF conversion),
and rehashed each executable, 11 runtime DLLs, and PNG. All 12 PE files in each
closure are x86; corrected native compile/link logs show the new helper and
`gdiplus.lib`. Full ZIPs remain private scratch, with per-entry SHA/CRC inventories
retained here. These receipts verify the fixture, not application PE/icon wiring.

Both used GDI+, 96 PPI, Arial 12pt/16px and identical water origin/145x19 logical
extent. The original first Dusk failure was pixel (381,87), RGB (70,108,123), blue
delta 34 > limit 33. Dusk and Night each had 588 offending channels; corrected
has zero in every theme. The maximum RGB deltas changed from (50,43,34) to
(43,38,32) in Dusk and (44,44,36) to (36,37,31) in Night, within unchanged bounds.

The only production change is the fresh translucent Windows GDI+ context's
text hint: system smoothing to grayscale AntiAliasGridFit. Layout/origin/font,
brush colors and opacity92 remain identical. The output comparison found 1,821
changed pixels, all inside the existing water-label oracle. All pixels outside
it, including opaque land, complex-script and title rows, are byte-identical.
Grayscale smoothing's actual ink footprint is 2px narrower on the right, while
native measured extent and placement are unchanged. No oracle was relaxed.

The native fixture checks 2,452 assertions because pixel assertions execute
per painted channel; this count is backend/coverage dependent, not a reduced
assertion contract (Linux Cairo previously exercised 5,776).

The shared header declaration is Windows-only; non-GDI+ backends and opaque
paths remain unchanged. Main S52 and standalone targets explicitly consume the
helper plus GDI+ library. Private adapter integration is tracked separately by
SCRUM-259 and must consume the same helper/link dependency.
