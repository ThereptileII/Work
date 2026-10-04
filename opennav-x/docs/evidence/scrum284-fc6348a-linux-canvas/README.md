# SCRUM-284: actual all-round light circle, Linux canvas

Exact source **fc6348a6df8374878802b3e051a3670d83ed9a56** passed the bounded
same-camera IHO light/fog review: SKAGER and Standard × software and actual
Mesa OpenGL × Day, Dusk, Night and Day-return. This is **16 captures across
four application sessions, four exact whole-chart Day returns and four clean
exits**, with no masks or tolerances. The GL collectors required actual GL;
software fallback was not accepted.

Compared with the sealed 45d before images, all eight Standard chart images
are pixel-identical. SKAGER changes are confined to the old light-ring annulus:

| Renderer | Changed pixels per theme | Changed radius from actual light anchor |
|---|---:|---:|
| Software | 1,920 | 60.80–67.12 px |
| Mesa OpenGL | 3,392–3,400 | 59.03–68.88 px |

The comparison covers the entire chart rectangle `(80,68)-(1094,634)`.
There are no changed centre labels, landmarks or other chart pixels outside
that annulus. Full PNG hashes differ in Standard because the surrounding
application includes the changed build identity; chart pixels remain exact.
`comparison.json` records every original image hash and pixel result.
The eight comparison panels show before on the left and after on the right.

Actual Day/Night full images and comparison panels were inspected. The new
thin yellow circle retains its centre and radius; the former heavy black/yellow
ring is removed. The nearby generic building mark is clearer where the old
thick ring overlapped it. The original LIGHTS lookup 31183 and full-circle CA
instruction remain in actual painter traces alongside FOGSIG lookup 31164.
No chart attributes, coordinates or classifications were modified.

## Inputs and build

- Ordered nine-core-patch tree: `89258f60eb3676eb919e969ecba6bd3ddc4cda12`.
- Resource manifest, unchanged from 45d: `cecfd92eff1b2c9a9c2aa2e64e9a16968a84bdcdee14b77eba337a9277c07185`.
- Installed ELF: `827c630822d0b2ff82c6663a25d0386930bafab75be1c4530702229ca9373b10`.
- Build ELF: `489db41076f5051742d88c98be7f88ae91e4b079de07c2de0e42dd9fc103d568`.
- Original official IHO `GB4X0000.000`: `c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`.
- Camera: latitude -32.3760351, longitude 61.0307025, actual scale 0.5826126536.

The prior 45d build, install, prepared source and resources were independently
copied and hash-verified before advancing the isolated warm builder. The
normal incremental configure/build/install completed with exit 0 and 29 build
steps, using one compiler job. No dependency producer or full suite was run.
There were no new build or capture failures in this stage. The earlier 45d
compiler termination and two collector failures remain in the separate before
evidence; they are not hidden or reclassified by this successful follow-up.

The before evidence is `docs/evidence/scrum279282-45d-linux-canvas`, committed
as `2824c3e405b0f36fb4892b2a43b02c1f8400965d`. That evidence binds the original
45d source, screenshots, collector repairs and clean sessions. Current source
advance, original build logs, collector scripts, frozen/staged input receipts,
full screenshots, diagnostics, actual CA/SY traces, Mesa identity and clean-exit
reports are retained here. `files.json` seals these bytes. ELFs, caches and full
disposable profiles remain local. Standard's copied terminal summary still
says SKAGER; the requested mode and actual per-capture diagnostics correctly
identify Standard, and the report is treated as a Standard control.

## Limits

This scene proves the ordinary white/yellow all-round light correction in the
Linux core renderer only. It does not exercise actual red/green light scenes,
the private o-charts DLL, Windows or boat hardware. The public IHO geography is
presentation-test data, not an operational chart. Controlled loopback position
input is explicitly recorded; no Demo source or synthetic chart objects were
substituted. No physical control output occurred. Parent SCRUM-284's separate
source/fixture matrix covers additional classes, but those results are not
counted as actual scene coverage here. Whole-prototype conformity and physical
legibility acceptance remain open.
