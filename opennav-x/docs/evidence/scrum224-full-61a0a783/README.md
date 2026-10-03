# Full candidate 61a0a783: Windows painter failure

[Run 37088759582 / Windows job 111105527254](https://github.com/ThereptileII/Work/actions/runs/37088759582/job/111105527254)
failed at 03:16:18 UTC on 2026-10-03. Remote source
`61a0a7838b56ad841bb458af6fc62651464bdafe` maps to local
`78eccb8b7f21b260ded57d3ba763f884d60c8180`.

The real native application compiled and installed. The installed peer CLI
refused both commands successfully. The first offline painter then failed:
`chart_name_text_test.exe`, exit 1, **Water label must not become opaque or ignore alpha**.
The unchanged pixel bound is at `tests/chart_name_text_test.cpp:69`, invoked by
`tools/build-pristine-windows.ps1:245`; line 24 reports the nonzero exit.
This is neither a Gettext acquisition failure nor an application compile failure.

Downloaded artifact **11262911171**: **20,100,275 bytes**, SHA-256
`ddc7e8627e2a6f821053db5a6b808905cad9bfdb578a8788755d36fbd671676d`.
All **5,804 ZIP entries** pass CRC verification; hash equals GitHub metadata.
The exact complete decoded GitHub job log (UTF-8/CRLF) and original PowerShell
transcript are retained losslessly as gzip files. Every ZIP entry's hash, byte
count and CRC is in the compressed inventory; the full ZIP is retained in the
private audit workspace and referenced by GitHub artifact ID.

The integrated Gettext receipt passes: Poedit 3.9.1 acquired on the first attempt,
GNU msgfmt.exe/msgmerge.exe 0.26 from the known Poedit directory, and successful
final verification before CMake. Five native stdout/stderr pairs independently
rehash. Tool-binary hashes remain runner observations, not independently inspected
executables. The separately downloaded pristine Windows artifact also passes
its 387 CRC entries, five Gettext stream pairs and all 60 upstream tests without
failures/skips; its evidence is retained under `windows-pristine/`.

No failed painter image exists: the fixture saved PNG only after the assertion.
No failed channel, RGB, PPI or native text-renderer details were emitted.
Therefore the failure alone does not distinguish native alpha rendering from
fixture-region/DPI contamination. An instrumented standalone native replay is
required before selecting a production correction; the bound must not be loosened.

The uploaded artifact has no application or setup executable, only compiler-ID
executables. Application/setup PE icon and ProductName/FileDescription comparison
and native Night chart-border inspection cannot be performed from this artifact.
The LIGHTS/wordmark/route-label/AIS painters, integration CTest, application GUI,
production package, installer, DPI, chart and Windows endurance gates were not
reached. Linux endurance was still running when this failure was recorded; it
was not cancelled. No full rerun, deployment, physical output or public launch
was performed. This candidate is not accepted.
