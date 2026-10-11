# Generated component and license inventory (SCRUM-59)

Native package assembly generates `docs/COMPONENT_LICENSE_INVENTORY.json` and
`docs/THIRD_PARTY_NOTICES.md` after the existing dependency validators and exact
corresponding-source archive generator succeed. The portable and installer
payloads retain these under `docs/`; the package file manifest covers both.
No external service or download is used by the generator.

The deterministic JSON contains the exact product commit, corresponding-source
ZIP hash, every package input file's hash and size, component evidence, and an
explicit list of files without reviewed attribution. It excludes the two
outputs and `FILE_SHA256.json` to avoid self-reference. Repeating generation on
the same bytes produces the same output; timestamps and local absolute paths
are not added by the generator. A build manifest's hash still reflects its
actual content, including any toolchain paths recorded by its producer.

The generator checks these declared links and fails on missing, changed or
ambiguous evidence:

- Product and OpenCPN notice bytes against their entries in the exact source
  archive, including the complete copied OpenCPN notice hierarchy.
- OpenSSL, curl and zlib notices against committed provenance; their final DLLs
  and retained manifests against the already verified dependency build records;
  their bundled source archive bytes against the same declared source identity.
- wxWidgets notice bytes against the committed per-file provenance hashes.
- Duplicate JSON keys, duplicate source archive members, Windows case aliases
  and package symlinks are rejected instead of choosing one interpretation.

This is an internal packaging step, not an authenticator for third-party source
archives. Its source ZIP must be produced by `source_package.create_source_archive`
from the same clean, verified checkout. Existing dependency, source, native
Windows, installer and release qualification checks remain necessary.

The status is deliberately `incomplete-review-required`. Matched evidence does
not establish legal compatibility, attribution completeness, or redistribution
rights. OpenCPN embedded third-party code, wxWidgets runtime/source association,
MSVC redistribution terms, updater and o-charts individual terms still require
review. Other runtime files are inventoried without guessed ownership.
`docs/MSVC_RUNTIME_RECEIPT.json` separately records the actual selected Visual
Studio installation directory/version, VC143 x86 redistributable directory/version,
and every copied CRT DLL's name, numeric file/product version, size and SHA-256.
Assembly fails if a required source DLL is absent, the selected directory does
not belong to that Visual Studio redist, a DLL is not x86 PE or lacks version
resources, or source/copy hashes and versions differ. Version resources are read
through the native Windows version API without executing the DLLs. The receipt
is read-only informational evidence and is covered by `FILE_SHA256.json`; it is
not consulted to authorize runtime behavior. Its deterministic content contains
the selected absolute toolchain paths, no timestamp, and a sorted DLL list.
Selection still follows the wrapper's latest installed Visual Studio/CRT policy;
this records that choice rather than pinning future builds to the same runtime.
It neither supplies Microsoft terms nor establishes redistribution rights, so
MSVC legal review and SCRUM-59 remain unresolved.
The [Microsoft terms and provenance audit](msvc-runtime-redistribution.md)
records the official redistribution references, limits of the current receipt,
and edition/entitlement evidence still required for the selected release.

Packaged fonts and image assets are explicitly categorized as unresolved when
no reviewed attribution exists. No packaged font file does not mean that
system fonts or lettering embedded in images have been cleared. These limits
keep SCRUM-59 open; this slice does not produce a complete SPDX/CycloneDX SBOM
or approve public delivery.

Focused check: `python tools/test_component_license_inventory.py`. Fixtures are
inert package/source bytes; they cover deterministic output and rejected
runtime, manifest, notice and source tampering, missing embedded notices,
ambiguous records and paths. Native package assembly still requires the next
coordinator-selected Windows build; a Linux fixture pass is not a native pass.
## Font-source refinement

The inventory now separately identifies the pinned packaged FreeSans SSFN bytes
and source-only Helvetica TXF input, while retaining unresolved font-specific
license/conversion provenance. See [the font provenance audit](font-provenance.md)
for install boundaries, historical package evidence and the pending exact-native
package gate. SSFN/TXF and common font headers/extensions are classified as font
candidates; an extension alone is explicitly not a validated font format.

## MSVC runtime receipt

`python tools/test-msvc-runtime-receipt.py` additionally checks inert CRT fixtures,
deterministic receipts, missing source/versions, architecture, copy corruption,
source changes and copied-version mismatch. Actual Windows version-resource
inspection and the packaged receipt require the next native packaging gate.
