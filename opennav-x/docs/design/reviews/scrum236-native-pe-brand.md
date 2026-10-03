# SCRUM-236 final executable branding resource gate

`tools/verify-skager-pe-brand.py` inspects the final application and NSIS Setup
on native Windows without executing either file. It uses `LoadLibraryExW` with
`LOAD_LIBRARY_AS_DATAFILE_EXCLUSIVE | LOAD_LIBRARY_AS_IMAGE_RESOURCE`, reads
all group-icon language variants and their referenced icon resources, and
queries every version-resource string table. No imports, entry point, installer
or application initialization runs.

Run after the exact candidate has been packaged:

```powershell
python tools/verify-skager-pe-brand.py --application <installed-app>/opencpn.exe --setup <installer>/SKAGER-Beta2-Setup.exe --report evidence/local/skager-pe-brand.json
```

The approved ICO must match its SCRUM-89/236 provenance SHA-256. Both final PEs
must contain exactly its nine declared frame sizes (16, 20, 24, 32, 40, 48, 64,
128 and 256), with each 32-bit bitmap payload byte-identical to that ICO. NSIS
receives the full ICO through `MUI_ICON`; this gate does not silently accept a
compiler-selected smaller set. A native NSIS size discrepancy is a failure to
inspect, not grounds to relax the requirement. Icon resource IDs and harmless
planes-directory normalization may differ; pixels and masks may not.

`ProductName` and `FileDescription` must exactly match `Skager.rc.in` for the
application and `AlphaSetup.nsi` for Setup. Every discovered language/table must
agree. Reports include complete executable size/SHA-256, expected source hashes,
resource IDs/languages, frame hashes and observed version strings. The executable
bytes must remain unchanged during inspection.

Four portable focused checks pass: all nine real approved payloads, a changed
payload byte refusal, a missing-frame refusal, and exact current metadata source
parsing. These tests do not qualify the Win32 resource APIs or final candidate;
the actual native command remains required. This gate does not inspect the
compressed maintenance executable or qualify DPI appearance/installer behavior.
