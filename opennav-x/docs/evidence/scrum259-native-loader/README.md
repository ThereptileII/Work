# SCRUM-259: native private-chart loader guard evidence

[Run 37095201787](https://github.com/ThereptileII/Work/actions/runs/37095201787)
passed on attempt 1, 2026-10-03. The native Win32 build and all 38 loader/fallback
guard groups passed. The build/test step took 32 seconds; no retry was needed.

## Exact scope and publication

- Frozen local source: `6dafd29db791c773793c2b1155c358cc81452fc2`.
- Published commit: `4609f31dba3e970802e6ba3fbe3438e129ab9aad`.
- Published tree: `8cefcfc4f755c932d51f9df5ee4545a43c006fb8`.
- Only new branch `skager-ocharts-loader` was created. Its only matching push
  workflow was the isolated `skager-ocharts-loader.yml`; no application or
  dependency producer workflow was dispatched.
- All 2,786 mapped blob identities and modes were verified before branch
  creation; `.github/*` maps to repository root and other files to `opennav-x/*`.
  The upstream gitlink was excluded. Eight unrelated repository files were
  preserved exactly. The 172-file delta used the previously verified base
  mapping `34c8d3a` → `d7cc9b4`.

`mapping.json` records the comparison; `mapping-inventory.json.gz` retains every
mapped file identity/mode and the preserved files. The complete workflow/job
identity is retained in `run.json` and `github.json`.

## Native result

Windows Server 2022; MSVC 19.44.35229.0; Windows SDK 10.0.26100.0; Win32 x86;
locked wxWidgets 3.2.8. The small project compiled actual production
`OChartsModuleLoader.cpp`, `PluginPresentationLoader.cpp` and the inline fallback
helper, plus five harmless fixture DLLs. The expected fail-fast fixture factory
produced unreachable-return warnings; there were no build errors.

The 38 groups verify source policy boundaries, original/adapter hash and length
refusal, live read/write/delete locks, reparse parents, compatibility-window
mutation detection, missing exports, rejected bind and bad copied status,
occupied-module refusal, clean one-module fallback and residual-module refusal.
The deliberate reserved non-module-address fault caused real `FreeLibrary`
failure with error **126**, preserving ownership and blocking fallback. This is
explicit fault injection, not a genuine loaded-module unload-failure claim.

The accepted vendor DLL was only read/hashed. The tests used benign locally built
DLLs for all module execution and every fallback original. No plugin factory,
chart helper, encrypted chart, application profile or boat was used.

## Download verification and limits

Artifact `11264111012` is retained as its exact 14,674-byte ZIP:
SHA-256 `8913c4a532fa2c815e95be39b240ef2a59d1f5f9949ebd0e7d988c68b35f7329`.
This matches GitHub's artifact digest. All **32 ZIP entry CRCs** passed; the
per-entry CRC/SHA-256 inventory is retained. All **15 consumed source receipts**
match the exact frozen Git blobs after the Windows checkout's CRLF conversion;
no other byte difference was accepted. The complete log contains all 38 passing
groups and expected refusal errors, with no unexpected DLL events.

The artifact intentionally contains logs/receipts, not executable, fixture or
runtime binaries. Its eleven runtime hashes and five fixture PE/hash identities
were checked before and after execution by the exact verified CI wrapper. They
are **CI-recorded receipts**, not independently rehashed current-run binaries.
`verification.json` records this distinction. No replay was performed merely to
add binary payloads.

This qualifies the bounded native host-loader mechanics. It does not qualify the
real adapter's import ABI/package, complete application Standard/Safe selection,
real plugin lifecycle, encrypted-chart rendering, licensing or physical hardware.
Those acceptance gates remain open under SCRUM-259 and its parent work.
