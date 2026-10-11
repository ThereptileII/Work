# MSVC runtime distribution terms and provenance

This bounded SCRUM-59 / SCRUM-57 evidence note was checked on 2026-10-11 against
product source `ec51b9261d2854d8600672ea27c97af5abeb3f6d` and the Microsoft pages
linked below. It identifies the published redistribution conditions and the
evidence still needed to apply them to a release. It does not establish a
Microsoft entitlement, legal compatibility, an accepted native package, or
permission to publish. Both Jira issues remain open.

## What Microsoft publishes

Microsoft says redistribution of the runtime installer, merge modules and
individual binaries requires a licensed Visual Studio user and is subject to
the applicable Microsoft license terms. The installed IDE's About dialog links
to those terms; Microsoft also supplies an online license directory.
[Redistribute Visual C++ files, Redistributable files and licensing](https://learn.microsoft.com/en-us/cpp/windows/redistributing-visual-cpp-files?view=msvc-170#redistributable-files-and-licensing).

The Visual Studio 2022 Distributable List identifies Enterprise, Professional
and Community 2022 and conditions redistribution on a valid license and the
edition's terms. Its Visual C++ Runtime Files section includes files under
`[VisualStudioFolder]\VC\redist`, requires them to remain unmodified, and
explicitly excludes the two `debug_nonredist` trees. Finding a file under that
directory is therefore only one part of the evidence.
[Microsoft Visual Studio 2022 Distributable List](https://learn.microsoft.com/en-us/visualstudio/releases/2022/redistribution#visual-c-runtime-files).

Microsoft documents application-local deployment beside the executable, but
recommends central deployment using its redistributable installer for servicing.
The current SKAGER package uses application-local DLLs. Its maintenance process
must therefore account for updating those shipped files; the installed central
runtime alone does not update a private copy.
[Redistribute Visual C++ files, Install individual redistributable files and servicing discussion](https://learn.microsoft.com/en-us/cpp/windows/redistributing-visual-cpp-files?view=msvc-170#install-individual-redistributable-files).

The official edition-specific entry points are
[Enterprise / Professional / Trial 2022](https://visualstudio.microsoft.com/license-terms/vs2022-ga-proenterprise/),
[Community 2022](https://visualstudio.microsoft.com/license-terms/vs2022-ga-community/),
and [Diagnostic Build Tools 2022](https://visualstudio.microsoft.com/license-terms/vs2022-ga-diagnosticbuildtools/).
Their pages link embedded Word license documents. This audit located those
links, but retrieval did not yield readable document contents: the web reader
does not support that content type and a direct download returned HTTP 403.
Consequently, this note does **not** summarize those license clauses or assume
that any one edition's terms govern the selected runner. The relevant full
terms still need to be retained and reviewed. See also Microsoft's
[license directory](https://visualstudio.microsoft.com/license-terms/).

## What the current packaging code proves

[`package-preview-windows.ps1`](../../tools/package-preview-windows.ps1) runs
`vswhere -latest -products '*'` requiring the C++ x86/x64 tools component. It
passes the selected installation path and installation version to the packager,
then selects the first descending directory path matching
`VC/Redist/MSVC/*/x86/Microsoft.VC143.CRT`. This is the wrapper's selection policy,
not a fixed redist version or an entitlement check. The `-products '*'` selection
also does not restrict the result to a particular Visual Studio edition.

[`copy_msvc_runtime`](../../tools/msvc_runtime_receipt.py) requires the resolved
runtime path to match that selected installation's versioned x86 VC143 CRT
directory. It requires `msvcp140.dll` and `vcruntime140.dll`, rejects ambiguous
DLL names and direct DLL symlinks, checks the x86 PE machine field, reads numeric
file/product versions through the Windows version API, and compares source and
copied bytes, size, hash and versions. The selected DLL set is recorded in
`docs/MSVC_RUNTIME_RECEIPT.json` by
[`package-preview.py`](../../tools/package-preview.py).

The receipt's `visualStudio.installationVersion` is the installation version,
not the product edition or a license. Its `redistributable.version` is the
selected directory name; DLL file/product versions are separately recorded.
The receipt has no Visual Studio `productId`, edition, runner-image identity,
applicable-terms reference/hash, or entitlement-review reference. The helper
does not authenticate Microsoft signatures, verify license ownership, or bind
the selected redistributable to the compiler that produced every shipped DLL.
The wrapper's diagnostic word "licensed" does not perform any of those checks.

The receipt deliberately retains
`legalStatus: redistribution-terms-unresolved-review-required`.
[`component_license_inventory.py`](../../tools/component_license_inventory.py)
hashes the package inputs, but does not interpret this receipt as a reviewed
MSVC component or as legal approval. Covering the JSON in a package hash manifest
protects its recorded bytes within that manifest's trust boundary; it does not
establish the truth of an entitlement claim. No exact native receipt was
qualified by this documentation audit.

## Evidence needed for the selected release

Source refinement after this audit: `c6ae26d1` adds the actual selected
`visualStudio.productId` and bounded catalog metadata to schema 2; `3bc1478b`
associates that receipt with final package bytes in the component inventory.
These changes address the missing product-identity fields described above, not
runner/release qualification or entitlement. The original `ec51b926` audit and
its historical receipt remain unchanged. Native transport/version-resource
verification and applicable full terms still require evidence.

These are unresolved evidence requirements under SCRUM-59 / SCRUM-57, not new
approvals or a second backlog:

1. Retain the selected `vswhere` record's non-secret `productId`, installation
   path/version, and catalog product name/version, together with the exact CI
   run, job, attempt and runner-image identity. Compare the selected record to
   the receipt and actual packaged files. `visualStudio.productId` is the
   concrete missing receipt field that would identify the installed product;
   an edition-looking directory name is weaker evidence. Adding this field
   would improve provenance, not prove licensing rights.
2. Identify who is distributing the package and the licensing basis that
   applies to that party and the selected installation. Retain a non-secret
   review reference to the relevant entitlement or hosted-runner grant. A
   successful hosted build, the availability of a download, or an installed
   product name does not answer this question. Keep credentials, license keys,
   account details and private purchase records out of public artifacts.
3. Obtain the full terms applicable to that edition/acquisition and the matching
   Distributable List; retain their titles, source URLs, retrieval date and
   content hashes in the release evidence. Have the responsible reviewer map
   the actual files and distribution method to those terms, including any
   notice, downstream-distribution or other conditions they contain. Do not
   silently reuse the 2022 list for a different selected product generation.
4. Bind the native receipt, compiler/dependency evidence and final installer/
   recovery hashes to the exact release revision. Verify the actual source
   locations and that the shipped runtime bytes remain unmodified. The current
   fixture checks are not evidence for the runner's real DLLs or licensing.
5. Record the conclusion on the relationship between the Microsoft runtime
   terms and the product's GPL/other component obligations. This audit makes
   no conclusion about system-library exceptions, aggregation, compatibility,
   or whether a particular additional notice is sufficient. Do not place
   proprietary runtime/SDK binaries in a corresponding-source archive merely
   because their provenance was recorded.

No code gate is added here: a new affirmative license flag derived only from
installed metadata would make an unsupported claim. The existing unresolved
status and public-release review boundary remain intact. A reviewed release
evidence contract can later define which machine-verifiable links must fail
closed without turning an installation record into an entitlement.

## Verification boundary

This change is documentation only. Relative source links and whitespace were
checked. The existing inert runtime-receipt test suite was run to confirm the
described checks; its Windows version-API case remains skipped on Linux. No
application build, native runtime inspection, delivery, boat action, license
acceptance or legal clearance is claimed.
