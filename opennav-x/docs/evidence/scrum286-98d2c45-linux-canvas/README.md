# SCRUM-286: actual compact overzoom warning, Linux canvas

Exact source **98d2c459ae0120070a2fa59b87dfa1c004e19394** passed the bounded
same-camera IHO light/fog review: SKAGER and Standard × software and actual
Mesa OpenGL × Day, Dusk, Night and Day-return. This is **16 captures across
four application sessions, four strict whole-chart Day returns and four clean
exits**. Actual GL was required; software fallback was not accepted.

Whole-chart before/after comparison against the preserved fc6348a build shows
all eight Standard chart images are pixel-identical. In SKAGER, the only
changed pixels belong to the old embossed/new compact overzoom warning:

| Renderer | Changed pixels across themes | Chart-relative changed bounds |
|---|---:|---|
| Software | 6,225–6,333 | `(48,0)-(308,55)` |
| Mesa OpenGL | 6,246–6,392 | `(47,0)-(308,55)` |

The comparison examines the entire chart rectangle `(80,68)-(1094,634)`,
without masking pixels. All pixels outside those measured bounds are exact,
including the SCRUM-284 light circle, buoy/fog/building symbols and labels.
All four Day-return checks remain exact. Standard full-screen PNG hashes
can differ because of surrounding application/build identity; chart pixels
are unchanged. `comparison.json` binds all original image hashes and results.

Full Day software and Night GL screenshots and a before/after Day GL panel
were inspected. The large embossed OverZoom wording is replaced by a compact
warning with the same wording, at the upstream warning location. The Night
warning remains visible. The eight comparison panels show before on the left
and after on the right. This is an engineering extension using the prototype's
warning roles; the immutable prototype does not contain an OverZoom component.

## Exact inputs and build

- Source: `98d2c459ae0120070a2fa59b87dfa1c004e19394`.
- Ordered nine-core-patch tree: `62cde4877219dba27be1dae999271609f971775f`.
- Unchanged resource manifest: `cecfd92eff1b2c9a9c2aa2e64e9a16968a84bdcdee14b77eba337a9277c07185`.
- Installed ELF: `477728e54ad7166a2bf5486a13a3cfa578eefdd58dbee93ab51423b0e849ca63`.
- Build ELF: `c78c3984baec112b3b9de1763d9e0c345e1aacb43ccb70728fea6f18f18d3688`.
- Build header: `fa8fd558c53be62db8e514657022e59a6646d843fd4aebafc8faeb673f48aa20`.
- Original official IHO `GB4X0000.000`: `c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`.
- Camera: latitude -32.3760351, longitude 61.0307025, actual scale 0.5826126536.

Before advancing the isolated warm builder, the exact fc6348a build, install
and prepared upstream tree were independently copied and hash-verified:
25 key inputs and 4,396 prepared source files. `prior-fc6348a-seal.json` records
the preserved identities. Both old and new ordered patch trees were verified;
only prepared chcanv.cpp and glChartCanvas.cpp changed. The normal single-job
incremental configure/build/link/install passed exit 0, with 38 build steps.
No new failure, dependency producer, full suite, CI or boat action occurred.

Before evidence is `docs/evidence/scrum284-fc6348a-linux-canvas`, commit
`7948d058b8b1e02be8be3508edf8102ca354702c`. Current original logs, scripts,
frozen/staged input receipts, full images, source-feature crops, diagnostics,
CA/SY traces, Mesa identity and clean-exit reports are retained here and sealed
by `files.json`. ELFs, caches and full disposable profiles remain local.
Standard's copied terminal summary says SKAGER, but its actual requested mode,
per-capture diagnostics and filenames identify Standard; it is counted only
as a Standard control.

## Limits

This qualifies this overzoomed Linux core-renderer scene, not Windows, the
private o-charts DLL, boat legibility or complete prototype conformity.
The unchanged upstream overzoom threshold and missing-warning/fit fallbacks
are source/focused-component responsibilities; this scene exercises the active,
fitting English warning only. No threshold, chart attributes, coordinates,
classifications or parser assertions were changed for these captures. The IHO
cell is presentation-test geography, not an operational chart. Controlled
loopback position input is recorded, with no Demo or synthetic chart objects
and no physical control output. No additional chart scene was run.
