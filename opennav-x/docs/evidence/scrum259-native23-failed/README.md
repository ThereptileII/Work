# SCRUM-259: preserved native 23-unit preflight failure

[Run 37096431092](https://github.com/ThereptileII/Work/actions/runs/37096431092)
failed on attempt 1, 2026-10-03. **Do not promote from this result.** No retry
was requested or performed by this evidence task.

The native compiler reached the newly included `model/src/plugin_loader.cpp`
and reported `model/include/model/plugin_handler.h(38,10): error C1083:
Cannot open include file: 'archive.h': No such file or directory`.
The complete retained `compile.log` has no other compiler error. This demonstrates
an incomplete preflight dependency closure; it does not establish a failure in
the full application's dependency setup.

## Exact inputs and downloaded proof

- Local candidate: `4e2715088361b0068e920f7744b49ab68ab439e1`.
- Remote candidate: `f761bd47a265a1c0c58b33220216972ae825f8ce`.
- Remote tree: `b6639a2220b18a133032f2620bf51402de7d1644`.
- Artifact: `11264223120`, retained as the exact 4,965,368-byte ZIP.
- ZIP SHA-256: `d26a3cc6ec44976abe903b79085a550c3f77402aee83cad4a0f68f496473a35d`,
  independently matched to GitHub's artifact digest. All 505 ZIP entry CRCs pass.

All 360 recorded product inputs match the exact local commit (336 after the
Windows checkout's CRLF conversion, 24 byte-exact). All 1,439 recorded upstream
inputs match an independently reconstructed nine-patch tree at pinned OpenCPN
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7` (1,423 CRLF, 16 exact). That derived
tree is `4efcf110ac10920b0b8cbfbd6726a51c6a3adfac`. No arbitrary content difference
was accepted. The 23 copied translation-unit sources match their consumed
receipts exactly, as do the 23 project files and two generated configuration
headers. All seven downloaded chart resources were independently regenerated
from this candidate and pinned stock resources and matched byte-for-byte.

The artifact contains **22 of the 23 requested objects**, each independently
hashed and inspected as I386 COFF. `plugin_loader.obj` alone is missing. The
additional compiler-ID object is excluded from that count. The successful
objects include `ChartModuleCheck`, `ChartNameAlphaWindows`, `OChartsPresentation`,
`OChartsModuleLoader`, `PluginPresentationLoader` and all 17 prior chart units.
The exact object inventory and hashes are in `verification.json`; no object
success receipt was invented for the failed overall command.

`archive-inventory.json.gz` records each original ZIP entry's CRC and SHA-256.
The complete compilation/configuration logs, failed summary and consumed-input
receipt are also retained individually as gzip files for review. The original
ZIP retains every artifact byte, including actual objects and resource files.

## Limits

The 1,558 SDK header identities are CI-recorded receipts; those header bytes are
outside the artifact and were not independently rehashed from this run. No
application or private adapter was linked or launched. This result gives no
plugin lifecycle, rendering, licensing, TLS, installer or boat acceptance.
The compile-only policy and all existing failure assertions remain unchanged.
