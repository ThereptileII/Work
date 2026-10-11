# SCRUM-311 transaction fixture x86 correction

[Run 38167897619](https://github.com/ThereptileII/Work/actions/runs/38167897619)
at `0e808824` remains failed: both native owner suites passed, but the x86
transaction log stopped after suspended-loader checks and final removal of the
first generation-b application failed with AccessDenied. The original exception
was masked. The coordinator retains the complete failed-run evidence separately.

CompileInertApplication did not select a platform. Microsoft's
[compiler documentation](https://learn.microsoft.com/dotnet/csharp/language-reference/compiler-options/output)
states that default AnyCPU executables run on the 64-bit CLR on 64-bit Windows;
[MainModule documentation](https://learn.microsoft.com/en-us/dotnet/api/system.diagnostics.process.mainmodule)
records the x86-to-x64 module-inspection exception. The fixture cleanup uses
MainModule before signaling its inert child. This supports a fixture architecture
mismatch; the masked exception and actual prior emitted PE were not retained,
so native confirmation remains pending.

The test now uses the existing Framework csc compiler with `/platform:x86` and
checks the actual emitted PE machine, PE32 header and CLR 32BITREQUIRED flag
before copying or launching it. All supervisor and refusal assertions remain.
The native application and final-removal cleanup blocks preserve an original
failure while reporting a secondary cleanup failure; cleanup-only failures still
throw. No product code, retry, cleanup delay or production close policy changed.

[Focused results](results.json): the portable transaction entry point passed its
five summaries, including exact production receiver and suspended-fixture C#
compilation. The exact emitted-output check accepted an independently compiled
X86 PE and refused AnyCPU, X64 and truncated PE files. The script parsed. No
emitted executable, Windows compiler or native API was invoked locally. Both
native transaction suites remain pending the coordinator's focused Windows run.
