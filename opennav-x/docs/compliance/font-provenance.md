# Font payload provenance — SCRUM-59

The Windows payload includes **FreeSans.sfn**, despite the native UI using
installed Windows font families. Font-license closure is therefore **pending**;
a no-bundled-font assertion would be incorrect.

## Source and install evidence

Audited product base: `cf27ce18`; pinned OpenCPN source:
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.

| Resource | Verified source identity | Install boundary |
| --- | --- | --- |
| `data/ssfn/FreeSans.sfn` | 372,928 bytes; SHA-256 `4b34341dbfd211b646807bda719e81f2ab2938bcb31f9eaf9c3f26b371566a46`; `SSFN` header | Upstream `CMakeLists.txt:1639` defines `ssfnData`; line 2048 installs it to `${PREFIX_PKGDATA}/ssfndata`. Windows sets the prefix to `.`, so package assembly copies it as `app/ssfndata/FreeSans.sfn`. |
| `data/s57data/Helvetica.txf` | 66,708 bytes; SHA-256 `1c22a8277dd2ef1c0524978f4a3610fd56015107aa90cedd8990129625dde852`; TXF header | Present in the pinned source tree, but absent from the explicit `s57data` list at upstream `CMakeLists.txt:1668`. That list is installed at line 2036. Source inclusion is not package inclusion. |

The font metadata identifies GNU FreeSans Normal 2012, FreeSans Regular,
version 0412.2268, GNU Freefont contributors, and the GNU FreeFont project URL.
This identifies the claimed upstream family; it does not establish the exact
original source archive or the conversion recipe that produced these SSFN bytes.

The supplied older recovery ZIP at
`evidence/local/boat-beta2/final-79-artifacts/beta-candidate-79a95c4f39063c20ca4d5a98c3147080d9813077/OpenNavX-Beta2-Portable-Recovery.zip`
was inspected read-only. Its one font-format path was
`OpenNavX-Beta2-Portable-Recovery/app/ssfndata/FreeSans.sfn`; its size and SHA-256
matched the pinned source above. This is historical package evidence only.
The exact current Staging recovery ZIP was not available locally and was not
downloaded for this bounded task. Exact current/native package inspection remains
pending; neither source review nor the older artifact qualifies a new package.

## What the inventory now enforces

`tools/font_inventory.py` records both pinned font hashes and checks the reviewed
simple CMake install lists. Package inventory generation checks FreeSans source
bytes inside the corresponding-source ZIP and the final packaged font bytes,
then creates a `FreeSans-SSFN` component with
`source-file-matched-license-unresolved`. Missing or substituted FreeSans, changed
font source bytes, ambiguous install lists, or adding Helvetica to that reviewed
install list fail instead of inheriting old evidence.

The generated JSON includes `fontEvidence`, the hashed install recipe, source-only
Helvetica identity, and detected packaged font candidates. FreeSans remains in
`unresolvedFiles` with category `font` even though its source is now identified.
The generated third-party notice index explicitly retains its unresolved license
and conversion limitation. No license expression is assigned to this font.

SSFN/TXF and common outline/webfont magic bytes are recognized even under another
filename. Additional font extensions are candidates, explicitly marked
`extension-only`/unverified when their header is not recognized. They do not
acquire license clearance and are not rejected solely for resembling a font
filename. This is not an exhaustive parser for embedded binary, archive, SVG or
rasterized lettering content. Unexpected font candidates remain visible for
review rather than being classified as an ordinary unidentified file.

## Outstanding provenance and license decision

The pinned `data/copyright` and `LICENSING` files contain no FreeSans/SSFN-specific
attribution found in this review. General GPL notices in the OpenCPN tree do not
by themselves establish all terms applicable to this converted font. The SSFN
renderer header's license is a separate software component and must not be
assigned to the font data.

[GNU FreeFont's license page](https://www.gnu.org/s/freefont/license.html)
describes GPL version 3 or later and a document-embedding exception for its fonts.
That public upstream statement is useful review input, but this task has not
proven the original font/conversion chain or a specific exception grant for the
pinned `.sfn` artifact. A font-specific notice, original-source identity,
conversion provenance and corresponding-source/license review remain required.
Helvetica's source-only license provenance also remains unreviewed. No legal
compatibility or redistribution approval is inferred here.

`src/ui/Controls.cpp:159`, `src/integration/ChartTextFace.h`,
`src/integration/ChartSoundingFont.h`, and `src/integration/ChartPresentation.cpp`
request installed Segoe UI Variable Display, Segoe UI and Arial through wx font
APIs. Those system-font requests do not identify or clear FreeSans, and this
patch copies no Windows system font. Lettering in branding/chart assets remains
a separate unresolved inventory scope. Full SCRUM-59 completion is not claimed.

Focused verification: `python tools/test_component_license_inventory.py`
(19 tests), plus direct read-only checks of the pinned upstream font hashes and
CMake install lists. Fixtures use inert font stand-ins and do not claim to test
font rendering or establish licensing rights.
