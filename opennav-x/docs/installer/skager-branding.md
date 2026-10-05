# SKAGER software and installer assets — SCRUM-236

The SCRUM-89 attachment 10000 is the sole artwork source. The unchanged file,
SHA-256, crop coordinates and reproducible output hashes live under
`resources/branding/`. The source wordmark uses SKAGER with a crossbar-free A,
APP below, off-white and mint over deep teal. The embedded software wordmark is
an exact pixel crop. The ICO uses a square crop of the same source at nine sizes;
there is no invented boat, compass, initial or substitute typeface.

The explicit integration build verifies those hashes and configures its Windows
resource from `src/integration/Skager.rc.in`. This replaces only the generated
`opencpn.rc` in the disposable build, after upstream's configuration. The pinned
upstream resource and ordinary OpenCPN build remain untouched. Resource ID 0 is
preserved because upstream `MyFrame` loads `wxICON(0)` on Windows. Explorer,
the frame/taskbar and target-derived shortcuts therefore share the product ICO.
The setup and generated maintenance executable use the same ICO through NSIS
`MUI_ICON`/`MUI_UNICON`. Product metadata says SKAGER and retains OpenCPN credit,
its executable name and its file-version/ABI identity. The generated C++ header
is explicitly LF in `.gitattributes` so Windows checkout conversion cannot break
its recorded byte hash.

## Customer-facing package names

New output names are `SKAGER-Beta2-Setup.exe`,
`SKAGER-Beta2-Portable-Recovery.zip`, `SKAGER-Beta2-source.zip`, and
`SKAGER-Beta2-{Install-Guide,Test-Guide,Release-Notes}.md`. The isolated launcher
is `Run-SKAGER.cmd`; it still passes the existing `--xnav` contract.
The three current guide source files are renamed accordingly, preserving exact
release-note bytes through the portable, outer and corresponding-source archives.

The coordinated producer/consumer changes cover:

- `package-preview.py`, its Windows wrapper and portable smoke gate;
- `package-alpha-installer.py`, compatibility metadata and installer smoke gate;
- `alpha-artifacts.py`, baseline upload paths and accepted-handoff workflow;
- `beta2_handoff.py`, its offline tests, distribution inputs and source tests;
- current shipped Beta 2 guides and recovery archive-root handling.

Historical installers, source/download references, locks, fixtures and recorded
hashes are not relabeled. The handoff collector accepts either complete six-file
name family, rejects a mixed family, and applies all existing exact source/hash,
CI and boat acceptance requirements. The recovery extractor accepts either exact
archive root, rejects mixed roots and retains its traversal/link/size/duplicate
checks. Accepting an old name never substitutes for its exact accepted hash.

## Existing installation compatibility

SCRUM-215 already supplies the versioned SKAGER Start-menu group and installed-app
name. This change preserves `OpenNavX.SkagerStartMenu.1`, exact existing shortcut
spellings, both historical layouts, registry/owner identities, installation root,
`opencpn.exe`, immutable generation bytes and mode arguments. Only current
maintenance error wording changes. Rollback still invokes the exact retained
engine and restores the group it understands. The original OpenCPN, normal
navigation profile and Windows Credential Manager identities are unchanged.

## Focused verification and remaining gates

Local checks passed:

- approved source/output hashes and all nine ICO entries; Pillow 12.3.0 byte-for-byte
  regeneration; visual inspection of original, cropped wordmark and icon sizes;
- Windows resource template compiled to `.res` with LLVM windres;
- 14 actual lifecycle shortcut-layout policy checks in PowerShell;
- 18 offline handoff tests, including unchanged historical and mixed-name cases;
- 11 source-package tests, 5 installer welcome tests and 12 completion tests;
- distribution-input verification, modified Python compilation and PowerShell parsing;
- eight disposable checks of the actual archive extraction functions under Linux
  PowerShell with a separator adapter, including both roots and mixed/traversal/
  duplicate-case rejection. This is not native Windows path-policy qualification.

The complete native Windows exact-commit gates are still pending: MSVC resources,
Explorer/frame/taskbar/shortcut icon display at 96/120/144 DPI, actual NSIS wizard,
maintenance/uninstall appearance, clean install/update/repair/rollback/recovery,
COM shortcuts and unchanged stock/profile/credentials. The original full wordmark
is necessarily small at 16 pixels; a new compact symbol would require separate
artwork approval. No running candidate, CI dispatch, boat state or public release
was changed by this bounded implementation. It is ready for integrated Testing,
not a completed Windows or release acceptance claim.
