# Combined supplied-symbol resource proof

The four separate artwork increments are integrated at `00ffa88`. A single
focused combined resource generation passed the existing strict pinned-source
and whole-XML semantic guard. This is not a full application or native gate.

The prior manifest is the retained `27e93e4`/`48c2f8b` resource set with SHA-256
`15a6bb831392088990737d688bc7716efa42f1d950f051c7a0dd909ee3065e2d`.
Its generated files were independently verified before comparison. The new
manifest is `cecfd92eff1b2c9a9c2aa2e64e9a16968a84bdcdee14b77eba337a9277c07185`.
The original prototype hash remains unchanged.

Exactly **505 pixels per theme** change across the five reviewed tiles:
SMCFAC02 152, UWTROC03 56, UWTROC04 88, WRECKS05 161 and XNFISH03 48. Each
tile's complete alpha equals its source-locked mask. Restoring only those tiles
recovers every prior RGBA byte and PNG metadata item. Atlas dimensions, old
symbols, hidden RGB and all other pixels are identical. The original RLE file
is byte-identical.

Restoring only the four named symbol bitmap metadata sets, the one AP alias and
selector, and CBLSUB06's exact reviewed HPGL/physical metadata recovers the
complete prior parsed XML. No additional lookup/classification change or
unaccounted symbol modification is permitted. Individual source/negative,
loader and painter checks remain in the original increment evidence.

`check-composition.py` is the exact executed local comparison; `composition.json`
binds its input commit, elapsed time and manifests. It completed in 176.3 seconds.
The earlier broad local resource run was deliberately cancelled before
completion, with no assertion failure observed, to avoid repeating the unchanged
suite for every glyph. Its cancellation is retained and is not counted as a
pass. The required broad CI suite and its failure assertions are unchanged.

SCRUM-283 subsequently changes only the private query patch/preparation list;
it does not change these resource inputs. Actual ENC selection, full SW/GL
canvas, native private DLL, Windows DPI/font and boat recognition remain open.
