# SCRUM-268: actual elevation TX cache/offset diagnosis

One authorized read-only GDB replay of the unchanged Linux application built from
`326daf73cdbeb0e7219ccb47b88967bc6a67ceaf` establishes the cause of the retained
355-pixel SKAGER lighthouse-scene Day-return difference. No application source,
chart model, binary, resource or navigation setting was changed for diagnosis.
This is failure diagnosis, not corrected acceptance or native Windows proof.

The original build/canvas evidence remains in
[`../scrum275276-326-linux-canvas/README.md`](../scrum275276-326-linux-canvas/README.md),
committed as `7fd0752a1ca355b6f6d62b988fdc7028c19cb903`. That evidence-only checkout
was verified to differ from the actual compiled source only under `docs/evidence`.
The installed ELF remains SHA256
`0617b1380237489d5e9c7ab5a80ea388a5ad1540bf021c63658d494f5fddae4c`.

## Actual observation

The target was confirmed at runtime as **LNDELV**, object index27/source RCID28,
latitude -32.3740304, longitude +61.0376363, Simplified table76/lookup31187,
ordinary TX `ELEVAT,3,2,2,'15110',1,-1,CHBLK,31)`. The locked IHO cell has
ELEVAT8m; the actual TexFont string was **26.2**, consistent with the pinned
feet conversion. This is an elevation TX, not a sounding or light description.

The selected settled draws all had anchor(957,129), canvas1014×566, offsets(+1,-1),
left/center justification, font size10, text height14, glyph extent38×22,
all four scale factors1, and the same actual font pointer and atlas pointer.
`letter_spacing=0`, opacity255, light_label=false, bspecial_char=false,
texobj0 plus actual TexFont calls establish the ordinary glyph-atlas path.

| Actual palette / sequence | Font cache at entry | Width at RenderText entry → return | Actual RenderString / output rect |
|---|---|---|---|
| Day / 2 | no matching key; Build then M extent16 | 9 → 16 | (973,108), 38×22 |
| Dusk / 36 | same cached font/atlas | 9 → 9 | (966,108), 38×22 |
| Night / 49 | same cached font/atlas | 9 → 9 | (966,108), 38×22 |
| Day return / 62 | same cached font/atlas | 9 → 9 | (966,108), 38×22 |

Each selected phase began with `bFText_Added=false` and FText=null. The text was
recreated; allocator reuse of an address in alternate phases does not mean the
old text survived. Initial Day alone recorded Build followed by M measurement16.
Later cache hits never measured M. Every label extent remained38×22 and every
selected draw returned true. Thus the recreated text's different average width
moves the otherwise unchanged glyph run **7px left**, also changing its declutter
rectangle. The actual palette indices were0→3→4→0. Raw trace includes intermediate
repaint anchors during palette changes; the table deliberately compares equal
settled anchors and includes the exact selected sequences in `metric-audit.json`.

Every diagnostic chart screenshot is pixel-identical to its corresponding
original screenshot (Day, Dusk, Night, returned Day). The unchanged unmasked
whole-chart assertion again failed with355 pixels at chart bbox(967,112,1010,125).
This is not inferred antialiasing. It reproduces the original visual failure under
read-only tracing. The collector stopped at that assertion; Standard was excluded
and no normal-exit acceptance is claimed (`inferior_exit.exit_code=null` after
collector cleanup). No second replay was attempted.

## Source boundary

Actual prepared source:
`build/integration-source/libs/s52plib/src/s52plib.cpp`:

- RenderT_All2527–2554 creates/caches text. The fresh-font branch2619–2620
  measures spec-font X and initializes avgCharWidth. The ordinary SKAGER face
  replacement2679–2688 replaces pFont without remeasuring that width.
- RenderText2195–2224 finds an atlas by font-pointer key; only the cache-miss
  branch measures M and overwrites the per-text avgCharWidth.
- RenderText2246 uses xoffs×avgCharWidth;2295 onward assigns the draw/declutter
  rectangle, then passes the same final position to TexFont::RenderString.

The trace directly observes9 at entry, M16 on the miss, the width overwrite,
cache reuse, unchanged label extent and actual final positions. Calling the
initial9 an X measurement is supported by the exact source branch; the separate
wxScreenDC X call itself was not probed. No inference about private DLL/native
font metrics, all label classes or Standard behavior is made here.

## Diagnostic boundary and reproduction

Original local collector/output:
`/home/standard/Projects/X-nav-worktrees/scrum275-linux-e6ef384/evidence/local/scrum268-metric-326`.
`run.sh opengl s64-light-fog` used the established private bwrap mapping to
`scrum259-linux-6dafd29`, read-only source/dependencies, isolated network/Xvfb/Mesa,
fresh profile and the existing explicitly simulated input-only loopback RMC.
No pilot capability, boat connection, inferior calls or model writes were used.
The source cell/resource identity and runtime loader/viewport checks remained.

Only a standalone offsetof helper was compiled using the existing actual include
flags; no production object or debug build was made. Exact mangled entries for
RenderT_All, RenderText and TexFont Build/GetTextExtent/RenderString were identified
from the installed ELF. `entries.json` binds virtual address, file offset and16
instruction bytes; all entries were checked in the live process before probes.
`entry-disassembly.txt` and the actual-header layout receipt are retained.
SysV argument registers, the stacked S57Obj argument, cache/key order and output
rectangle reads were independently source-reviewed before launch.

Probes filter the exact LNDELV geometry. Measurements use normal-return breakpoints
while output pointers remain alive; missing/out-of-scope returns are errors.
There were **zero trace errors**. Bounds were18000 total entry hits and64 nested
events per target RenderText call. The full trace is retained. A setup-only bwrap
symlink-mount refusal was preserved before the helper setup succeeded; it launched
no application and is not counted as a rendering attempt.

`files.json` hashes every retained original receipt/capture/script except itself
and this narrative. Disposable profile/SENC cache and helper/application binaries
remain local; they are not duplicated here. Original full profile evidence remains
in the preceding canvas evidence. Repair and corrected software/GL theme checks,
plus exact native Windows/private-renderer qualification, remain unpassed.
