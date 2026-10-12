# SCRUM-209 installer PE dependency policy — 2026-10-12

Integrated product source: `87fe6c5dad6ba0875bb13f48bfbb01a6a0e70001`.
Isolated implementation: `3414f67e46532932db1fe41b53fc1c92a3cb5b06`.

The actual installer PE fixture suite passed in native **x86 Windows
PowerShell** on Windows `10.0.22631.0`, starting at
`2026-10-12T05:23:08.0389696Z`. The test process exited `0`, did not time out,
and produced an empty test stderr log. Both source files passed native
PowerShell parsing and their SHA-256 identities were checked before execution.

The suite imports the actual `Lifecycle.ps1` helper function ASTs without
invoking the installer's transaction entry point. It creates inert temporary
PE bytes; it does not load those binaries. Its coverage includes extensionless
normal/delay imports, maintained and legacy forwarded declarations, transitive
and non-DLL helpers, refusal without changing retained source bytes, malformed
export extents/grammar/ordinals, parser budgets, and the existing foreign-DLL
exception and foreign-EXE refusal cases. Existing assertions were retained.

The output contains these two suite completion messages. It does not report an
assertion count, so no test-count claim is made:

```text
Installer extensionless and export forwarder policy fixtures passed.
Installer staged PE import closure fixtures passed.
```

## Exact source and retained evidence

| Input | SHA-256 |
| --- | --- |
| `opennav-x/installer/windows/Lifecycle.ps1` | `b4284a432d8b34842e87c31f66478a587974333573a5af93e60b99fca470c6fb` |
| `opennav-x/tools/test-installer-pe.ps1` | `170507753ecba6db6362879100cebba930f801bd89566841b2ab118554383373` |

A read-only comparison confirmed that these source bytes are identical in the
isolated and integrated commits and match `source-manifest.json`. The original
transferred archive's hash and exact two-member contents were also verified
against that manifest. Source remains available at the pinned Git revisions;
the archive is not duplicated in this evidence directory.

`receipt.json`, `tests-stdout.log` and `tests-stderr.log` are the original native
result and child-process logs. `native-check.ps1` is the exact successful driver.
`verification.json` records hashes of the seven retained original files and the
read-only source comparison; it is not a replacement for the native receipt.
Original Windows BOM/CRLF bytes are preserved. The local `.gitattributes`
prevents checkout line-ending conversion of these retained evidence files.

## Original helper failure and successful retry

The first remote helper attempt failed with exit `1` and **“The command line is
too long.”**, before fixture execution. Its original `native.json` is retained.
This was a command-transport failure, not a failed assertion or product result.

The coordinator retried using the separately transferred `native-check.ps1`
driver. `retry-native.json` records helper exit `0`; its stderr includes the SSH
transport diagnostic and PowerShell progress CLIXML. The separately captured
**test** stderr is empty. The driver checks the archive and source hashes,
parses both sources, starts the x86 child host, bounds execution to 90 seconds,
and requires both completion messages as well as exit `0`.

The focused command executed by that driver was equivalent to:

```powershell
& "$env:WINDIR\SysWOW64\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -NonInteractive -ExecutionPolicy Bypass -File opennav-x/tools/test-installer-pe.ps1
```

The receipt observes the existing OpenCPN process, PID `1852`, as responding
after the check. This single observation is not navigation or endurance
qualification. Evidence documentation and source comparison caused no rerun.

## Remaining gates

This passes the focused native PE parser/policy suite. It does not qualify
actual DLL loading, all Windows search roots, supported live plugin behavior,
a full application build, or installed update/repair/rollback/uninstall behavior.
The broader disposable native filesystem matrix remains a **CI gate**; its
necessary helper-import list update was not exercised by this two-source run.
SCRUM-209's other native, integration and lifecycle acceptance gates remain open.
