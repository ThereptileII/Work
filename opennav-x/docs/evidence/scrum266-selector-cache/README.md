# SCRUM-266 — GL chart-selector outline cache

Base: `a3e84771652c920479517f0d16a1dd6133c440d2`. Pinned OpenCPN:
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.

The retained actual GL Day and Day-return screenshots differ at 1,270 chart
pixels, entirely inside the Piano region (inclusive bounds 83,614–685,631).
Of these, 1,192 change from stock CHBLK `(7,7,7)` to the verified Day ink
`(83,100,95)`. `receipt.json` retains image hashes and the comparison rectangle.
These are failure evidence, not corrected application captures.

## Cause and correction

`ChartPresentation.cpp` enables the verified coastline palette before lazy S52
creation. `color_handler.cpp::GetGlobalColor` resolves through S52 when available,
otherwise the user palette. `Piano::BuildGLTexture` bakes CHBLK into the atlas;
`SyncChartPresentation` previously invalidated only when either vector brush
changed. Those brushes were already styled, so the later S52 outline change did
not invalidate the first atlas. A theme cycle rebuilt it, explaining the return
mismatch. Software painting already resolves this pen on each paint.

The appended patch snapshots the **actual resolved outline used to draw the
atlas**, at the normal end of construction after the atlas/icon uploads. Before
`DrawGLSL`'s existing height/rebuild check, a different resolved outline invalidates
the cache. Deferred builds (missing icons) do not commit a new snapshot. This
retains the existing GL upload/error policy; it does not add GPU success reporting.
Only OPENNAV_X builds keep the snapshot. No color, alpha, key geometry, chart
model, click behavior, or persistence setting changes. Standard/Legacy retain
their resolved stock ink; style activation/fallback uses the same actual-ink check.

## Focused checks

- All nine integration patches applied to a private index of the pinned source.
- The fixture executes the actual `BuildGLTexture`, actual `DrawGLSL` rebuild
  preamble, actual brush methods/resolver, and real wx bitmap/DC drawing. GL
  uploads are captured explicitly, not sent to a GPU. The original source fails
  exactly at “lazy S52 color change must replace stale outline.”
- Corrected source: 21 checks, covering initial deferred construction, lazy color
  change, stable atlas reuse, Day/Dusk/Night/Day equality, deferred retry, size
  change, style fallback/reactivation and Legacy stock ink/reuse.
- Existing actual selector brush fixture: 159 checks passed.
- Actual production `piano.cpp` object compiled with the integrated GL compiler
  command, all real dependency headers, and private source/header/output paths.
  `production-command.json` records that command; empty `production.log` means
  exit zero (receipt records the exit and object hash). The first setup attempt
  incorrectly placed the private include path one directory too high, resolving
  the old header; that diagnostic is retained separately. Correcting only the
  include path to `gui/include/gui` produced the successful compile.

Reproduce the cache fixture against a fully patched pinned `piano.cpp`:

```sh
python3 tools/test-chart-selector-cache.py \
  --piano-source /path/to/integration-source/gui/src/piano.cpp \
  --upstream /path/to/pinned/OpenCPN \
  --wx-prefix /path/to/wx/prefix \
  --output /private/output
```

Use the baseline source and `--expect-stale` to reproduce the original rejection.
The tool uses its own short-lived Xvfb for wx bitmap drawing; it launches no
application, chart, navigation transport or helper. Root still owns the actual
integrated GL exact Day-return capture and Standard/software stability checks.
No Windows, boat, release or full application qualification is claimed here.
