# Windows dependency receipt boundary (SCRUM-217)

The Windows integration workflow now wires a same-job dependency receipt into
its fixture and production passes. This is implemented orchestration, not yet a
qualified native reuse result.

After the `windows-integration` job's integrated Win32 build and its subsequent
source-package, smoke, recovery and preview checks succeed, the workflow calls
`tools/windows_dependency_reuse.py capture`. Capture requires GitHub Actions,
the `windows-integration` job identity, and an exact match between
`GITHUB_SHA` and the checked-out `HEAD`. It verifies producer manifests,
retained test evidence and producer-time tool-fact records, preserves the
validated first-success zlib source-verification record, then records a
bounded inventory of the selected dependency prefixes and inputs. Receipt
context hashes the producer scripts, locks, patches, source archives, manifests,
retained logs and helper inputs; it also binds the producer-time tool-fact
files, run/attempt/job, Win32 architecture, commit and absolute workspace.

The later `-Production -ReuseVerifiedDependencies` invocation is likewise
restricted to that same CI job. It still runs the upstream stock dependency
preparation. Before the maintained dependency outputs are staged, it verifies
the receipt and producer evidence, asks the OpenSSL, zlib and curl producers to
reprobe their recorded tools, and runs `windows_dependency_stage.py`. Staging
rechecks the evidence and every source-prefix inventory before replacing the
stock Win32 cache payloads. A failed identity, evidence, tool or inventory
check stops the build; this path does not fall back to unverified cached files.
The normal product, package, installer and security checks remain in the
workflow.

`tools/windows_dependency_receipt.py` supplies the bounded file-receipt
primitive. It rejects missing, added, changed, linked, malformed and
unbounded inputs. `tools/windows_dependency_evidence.py` separately validates
the integrated install against the OpenSSL, zlib and curl producer manifests,
checks each declared output, validates curl's dependency-manifest links and
retained imports, and requires successful nonzero upstream test summaries in
the retained producer logs. The zlib source-only preflight runs again in the
production invocation and overwrites its normal source-verification record;
the capture step therefore preserves the earlier verified build record at
`evidence/local/windows-zlib-1.3.2/first-success-source-verification.json` and
binds that file into the receipt. Verification accepts only that explicit
first-success record for this later pass, with the same bounded path and
source checks.

The orchestration and guards are present in
`.github/workflows/opennav-baseline.yml`, `tools/build-pristine-windows.ps1`,
`tools/windows_dependency_reuse.py`, `tools/windows_dependency_evidence.py`,
and `tools/windows_dependency_stage.py`. The producer scripts capture facts
from their relevant native build environments; the reuse context binds those
fact files and the producer entrypoints reprobe tools before staging. Exact
selected-tool records and producer outputs remain subject to the native checks
in those helpers.

Native version probes retain each executable identity and actual exit code,
including nonzero help/version exit codes. Standard output and standard error
are captured and hashed separately as raw bytes, with their byte counts, a
combined 1 MiB limit and a 30-second deadline. This avoids nondeterministic
ordering from PowerShell's merged streams without ignoring either stream.
The diagnostic first line comes from stdout, falling back to stderr. The helper
and producer hashes bind this format into each same-job receipt, so an older
capture cannot be silently accepted under a changed helper. Native qualification
must pass in Windows PowerShell 5.1 and PowerShell 7; the focused test also
requires repeated unchanged observations and refusal of each altered stream hash.

The latest focused contract run, [36895232260](evidence/scrum-217-native-contracts-pass.json),
passed on both Windows and Linux: receipt (15 tests), evidence (14), reuse (9)
and stage (4) on each platform. This updates the earlier failed evidence-parser
contract result, retained in
[scrum-217-native-evidence-parser-failure.json](evidence/scrum-217-native-evidence-parser-failure.json);
that earlier result remains historical evidence. These are contract-suite
results, not a full native producer build or proof that the integrated
same-job production path successfully skipped and restaged dependencies.
Application/package acceptance, native reuse qualification and timing
improvements remain unproven.
