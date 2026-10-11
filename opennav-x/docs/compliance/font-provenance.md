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
The independent upstream comparison and reproduction below establish the
original OTF input and a working conversion recipe, beyond this metadata claim.

The supplied older recovery ZIP at
`evidence/local/boat-beta2/final-79-artifacts/beta-candidate-79a95c4f39063c20ca4d5a98c3147080d9813077/OpenNavX-Beta2-Portable-Recovery.zip`
was inspected read-only. Its one font-format path was
`OpenNavX-Beta2-Portable-Recovery/app/ssfndata/FreeSans.sfn`; its size and SHA-256
matched the pinned source above. This is historical package evidence only.
The exact current Staging recovery ZIP was not available locally and was not
downloaded for this bounded task. Exact current/native package inspection remains
pending; neither source review nor the older artifact qualifies a new package.

## Upstream byte match and reproduced conversion

Follow-up evidence collected on 2026-10-11:

- OpenCPN first added these bytes in commit
  [`743870c2ab12a852cc9f23b6e60bd2bc7eb433a9`](https://github.com/OpenCPN/OpenCPN/commit/743870c2ab12a852cc9f23b6e60bd2bc7eb433a9)
  (2024-07-05, “Refine MUIBar chart scale text size/layout.”). The commit adds
  the font, renderer and install rules, but no original font or conversion recipe.
  Its font bytes are unchanged at the pinned OpenCPN revision.
- The original **SSFN v1** project, `bztsrc/scalable-font`, at commit
  [`79d2e48c32a09d514b1b90729f0f3c3978c72e75`](https://gitlab.com/bztsrc/scalable-font/-/tree/79d2e48c32a09d514b1b90729f0f3c3978c72e75)
  contains [`fonts/FreeSans.sfn.gz`](https://gitlab.com/bztsrc/scalable-font/-/raw/79d2e48c32a09d514b1b90729f0f3c3978c72e75/fonts/FreeSans.sfn.gz).
  Decompression produces exactly the pinned OpenCPN font, verified by direct
  byte equality as well as SHA-256. This is not the newer `scalable-font2`
  project, whose format uses `SFN2` magic.
- The same revision's
  [`fonts/README.md`](https://gitlab.com/bztsrc/scalable-font/-/blob/79d2e48c32a09d514b1b90729f0f3c3978c72e75/fonts/README.md)
  identifies that example as converted from GNU FreeFont's `FreeSans.otf` at
  maximum quality. Its
  [`converter documentation`](https://gitlab.com/bztsrc/scalable-font/-/blob/79d2e48c32a09d514b1b90729f0f3c3978c72e75/docs/ecosystem.md)
  identifies `ttf2sfn`, which uses FreeType to convert outline fonts.
- An independent conversion of `freefont-20120503/FreeSans.otf` from GNU's
  [`freefont-otf-20120503.tar.gz`](https://ftp.gnu.org/gnu/freefont/freefont-otf-20120503.tar.gz)
  using the unchanged converter and header at that SSFN revision reproduces
  the OpenCPN font byte-for-byte. This proves a reproducible conversion from
  that input; it does not claim to recover the original converter invocation
  or historical build environment.

| Downloaded input | Bytes | SHA-256 |
| --- | ---: | --- |
| SSFN v1 `fonts/FreeSans.sfn.gz` | 231,476 | `e8f3043bc89b59722248c34f37e37fdf75a4bf6ca58bab8d858c3140a35a2c1b` |
| GNU `freefont-otf-20120503.tar.gz` | 4,487,813 | `3a6c51868c71b006c33c4bcde63d90927e6fcca8f51c965b8ad62d021614a860` |
| Archive member `FreeSans.otf` | 856,800 | `d3e9138ccf76ef516cc6af867e4e7ed8776331947235824f924c5886186b0c80` |
| GNU `freefont-src-20120503.tar.gz` | 6,505,962 | `0d62e2e22655f841c3ac5fb8c8059a52fe2c09c695d9e8eda9903406e942657a` |
| Source archive member `sfd/FreeSans.sfd` | 3,946,801 | `9dfad4bb43eb8c9cd366f33336844f1b6ace870ee7b652af7147bf3cc8a3df1e` |
| SSFN v1 `sfnconv/ttf2sfn.c` | 47,937 | `7886cb314de625e06f1071ccead43a20aac956b2ef883eb6d5dbc587763b82c4` |
| SSFN v1 `ssfn.h` | 57,291 | `f2e179248ba1a313da821fcb8181828a7164871846e2863113548e2bdbb213d2` |

Reproduction used GCC 16.2.1 and FreeType package 2.14.3-1 on Linux, with the
upstream source layout (`sfnconv/ttf2sfn.c` includes `../ssfn.h`):

```sh
gcc -Wall -Wextra -ansi -pedantic -I/usr/include/freetype2 \
  -o ttf2sfn sfnconv/ttf2sfn.c -lfreetype
./ttf2sfn -6 -s FreeSans.otf reproduced.sfn
sha256sum reproduced.sfn
# 4b34341dbfd211b646807bda719e81f2ab2938bcb31f9eaf9c3f26b371566a46
```

The output is 372,928 bytes. `-6` selects maximum quality; `-s` selects the
sans-serif family. A first run without `-s` differed only at header byte 8,
which defaults to the serif family. The corrected invocation needs no manual
output modification. Compilation emitted a C99 integer-constant warning from
the installed FreeType headers; conversion completed successfully. No product
font, package input, or installed application was changed during this check.

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
The generated third-party notice index preserves the unresolved final license
review. It now lists the five GNU-specific notices copied to
`docs/licenses/GNU-FreeFont-20120503/`, and the component records
`originLicense: GPL-3.0-or-later` separately from its unresolved derivative
license status. This origin field does not assert application compatibility or
continuation of the optional embedding exception.

`vendor/freefont-20120503/` contains the exact GNU editable-source archive,
original OTF, unchanged SSFN v1 converter/header (including their MIT notices),
and `REBUILD.txt`. The source producer includes these tracked inputs in the
corresponding-source ZIP. `tools/freefont_payload.py` pins every byte digest;
the source producer, component inventory and final offline notice gate require
them. Missing or substituted source/notice inputs fail, including substitutions
whose surrounding package/source manifests have been rehashed. Packaging adds
no network access and performs no font conversion.

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

The verified original OTF contains an explicit **GPL version 3 or later**
grant and the document-embedding exception in its OpenType name-table entry 13.
The same GNU archive's `README` includes those terms, and its `COPYING` contains
GPLv3. This is direct evidence for the original input's terms, not a license
inferred from the filename. The
[GNU FreeFont license page](https://www.gnu.org/s/freefont/license.html)
confirms that distributing font software and modified fonts remains subject to
the GPL; its document exception does not exempt application font distribution.

The SSFN converter copies selected names and the copyright string, but does
not retain the OTF's full license/exception entry. Neither the matched SSFN
example nor its README establishes an explicit continuation of the optional
embedding exception for that converted artifact. Consequently this evidence
supports GNU FreeFont provenance and the GPL-3.0-or-later origin, while a
font-specific notice is now supplied while the final license-expression decision
for the shipped derivative remains pending. The SSFN project's MIT license covers its software; it does not
replace the GNU font's license.

Release review must address GPLv3 notice/license preservation, identification
of modifications, and corresponding-source delivery under the applicable
distribution method ([GPLv3 sections 1, 4–6](https://www.gnu.org/licenses/gpl-3.0.html)).
GNU supplies a separate
[`freefont-src-20120503.tar.gz`](https://ftp.gnu.org/gnu/freefont/freefont-src-20120503.tar.gz)
with editable FontForge sources; the original OTF plus the converted SSFN alone
must not be asserted to complete the preferred-source obligation. Its
`sfd/FreeSans.sfd` identifies revision 2268, and its Makefiles provide OTF
generation. That SFD-to-OTF stage was inspected but not rebuilt in this review.
The font
source, OTF-generation inputs, conversion recipe and GNU notices are now included
in the offline source-delivery implementation. Exact native package verification
and final source/distribution review remain pending.
[GNU's FAQ](https://www.gnu.org/software/freefont/FAQ.html) also distinguishes
incorporation from distribution as a separate bundled product. Application
compatibility/aggregation and the final distribution's obligations still need
review; no broad redistribution approval is inferred from the byte match.
Helvetica's source-only license provenance also remains unreviewed.

`src/ui/Controls.cpp:159`, `src/integration/ChartTextFace.h`,
`src/integration/ChartSoundingFont.h`, and `src/integration/ChartPresentation.cpp`
request installed Segoe UI Variable Display, Segoe UI and Arial through wx font
APIs. Those system-font requests do not identify or clear FreeSans, and this
patch copies no Windows system font. Lettering in branding/chart assets remains
a separate unresolved inventory scope. Full SCRUM-59 completion is not claimed.

Initial inventory verification: `python tools/test_component_license_inventory.py`
(19 tests), plus direct read-only checks of the pinned upstream font hashes and
CMake install lists. Fixtures use inert font stand-ins and do not claim to test
font rendering or establish licensing rights.

Follow-up verification: direct byte equality across the introducing and pinned
OpenCPN commits, the decompressed SSFN example, and the independent OTF-to-SSFN
conversion; inspection of original OTF license metadata and GNU source archive;
`git diff --check`. This documentation-only follow-up does not qualify a new
package or close SCRUM-59.

Offline payload implementation verification (2026-10-11): real vendored GNU
archive/member and notice equality checks (2 tests); source packaging (13 tests);
component inventory (20 tests); offline notice gate (22 tests). The existing release-manifest entry point also
runs those checks and the real font inputs (61 tests passing total). The vendored
OTF and converter reproduce SHA-256
`4b34341dbfd211b646807bda719e81f2ab2938bcb31f9eaf9c3f26b371566a46`.
No full application build, native package qualification, push or boat operation
was performed. SFD-to-OTF rebuilding, Helvetica/system-font/asset provenance,
MSVC runtime terms and final legal/aggregation review remain open.
