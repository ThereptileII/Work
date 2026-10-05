# SCRUM-278: bounded native parent-context proof

Prepared on helper commit `91a81e362ea30e5b09a69107d2d5bf655686cedd` (base
`593464106bd110248813777a7be87d3727c3187e`). No native run has been dispatched or
claimed by this preparation. The original full-run failure and artifact remain
as recorded in `../scrum278-parent-environment/README.md`.

`tools/test-windows-parent-context.ps1` exercises the actual shared initializer
and unchanged `windows-native-tool-facts.ps1`, using real selected Windows tools.
It captures one fresh **test** parent receipt, rejects omission of the shared
Gettext/native-Perl prefixes, and verifies restoration against that same receipt.
The original failed full-run receipt is never imported, overwritten, refreshed
or relabelled. This test receipt is not a dependency-build receipt.

The producer-local NASM/Strawberry setup is the exact 2,273-byte LF-normalized
span from `build-openssl-windows.ps1`, bounded before parent capture and guarded
by SHA-256 `dc6b184a18f728b98db4cf0ac522b4edb5495cffc8a037eacc3194483fd59749`.
All three contexts execute that same retained source span with
`VerifyToolFactsOnly=true`. Its reviewed source contains no producer, parent
capture or child launch. It preserves the original conditional installed-NASM
selection and verified pinned fallback. The small NASM archive is prepared in
advance; a missing archive causes refusal, never acquisition inside the span.
No OpenSSL source archive or maintained dependency producer is needed.

The negative control resets only the two shared prefixes. It must produce the
original helper's exact rejection. Comparison requires byte-identical JSON
after replacing only the single `environment.PATHSha256` value **in memory**;
selected executable paths, hashes, versions, interpreter, Visual Studio and all
other environment facts must remain identical. Restoration must reproduce the
exact original PATH bytes. The captured JSON, negative observed JSON and verified
Gettext receipt retain their original SHA-256 hashes throughout.

`.github/workflows/skager-parent-context.yml` runs only on its dedicated
`skager-parent-context` push branch or manual dispatch. It uses Windows 2022 and
PowerShell 7, a 15-minute bound, the existing pinned native/MSYS Perl selection,
the 641,314-byte locked NASM host archive, and the maintained Gettext acquisition
and verification helper. The latter may use the existing bounded Poedit install
if its known-path tools are absent. It does not change global PATH or trust.
The normal full-candidate workflow remains separate and unchanged by this commit.

Always-upload retains prerequisite selection, source/input hashes, the extracted
setup span, original capture, negative facts/rejection, Gettext receipt/probe logs,
result/failure details and native transcript. A failure stops this proof; there
is no retry of the native comparison or weakening of exact facts.

Local preparation checks (Linux, no Windows tool qualification):

- 13 PowerShell parser, source-span, comparison and platform-refusal checks passed.
  `local-guards.ps1` and `local-guards.txt` retain the exact check source/output.
- Workflow YAML and embedded prerequisite Python parsed; one native invocation
  and unconditional artifact retention checked. `git diff --check` passed.
- The producer and native tool-facts helper are byte-unchanged from the base.

Reproduce local guards from the repository root with
`pwsh -NoProfile -File docs/evidence/scrum278-parent-context-proof/local-guards.ps1`.

Even a future passing native result proves **parent-context replay only**. It
will not qualify the x86 child, AIS configuration/link/TLS/lifecycle, dependency
build outputs, application, installer, boat or release. Those original full
qualification gates remain mandatory.
