# SCRUM-209 private multi-dot forwarder alternative

[Run 38164290605](https://github.com/ThereptileII/Work/actions/runs/38164290605),
[job 114543504532](https://github.com/ThereptileII/Work/actions/runs/38164290605/job/114543504532),
at `eba68bf7c09560297b4ee57ffee07b13599fa3c3`, attempt 1, **failed**.
The actual native Win32 MSVC fixture project compiled, all nine bounded PE
forwarder constructions passed, and the native pure reader completed 17,496
checks. The loader transcript contains 49 completed groups: the preceding 42
and the first seven forwarder groups. The final multi-dot probe stopped at
`GetProcAddress` with `FAIL native forwarded export resolution failed`.
The later invalid-module-handle group did not execute; the required 51-group
suite did not pass. The failed summary retains native product acceptance false.

The successful explicit `.dll` probe reported the private `forward_clean.dll`
path and value 209. The failed `forward.clean.fixture_value` probe did not report
a mapped target or a Windows error. Its private case prepared `forward.dll` and
`forward.clean.dll`, but no exact `forward.clean` file. A loader split at the
last dot that retains an existing `.clean` extension is a **hypothesis**, not an
observed cause. The failed evidence does not establish that split rule.

This fixture-only change additionally copies the existing inert
`forward.clean.dll` bytes to the explicit private name `forward.clean`.
It adds that file to the same pre-mapping file-identity/hash snapshots, actual
mapped-path allowlist, unload checks, unchanged-byte checks and writable-after-
cleanup checks as the two existing alternatives. The inert value 409 remains
required as one of the existing two multi-dot outcomes (310 or 409); the positive
resolution and all 51 groups remain mandatory. A failed resolver now immediately
captures `GetLastError` and reports the target plus every prepared filename.
Native mapped-path, file-identity, SHA-256 and value output remains the evidence
needed to establish which alternative Windows actually selects.

Unsupported multiple-dot/explicit-extension production grammar remains refused
before original or sibling execution, including repeated attempts and clean
adapter refusal. No production source, fixture constructor, workflow, native
receipt requirement, or positive assertion is relaxed.

`artifact-verification.json` records artifact **11711213121**, **29,119 bytes**,
SHA-256 `26833faa5c6152df7a9c75322f56063a089030d5cf495d2c58639ee3c831fcb5`,
matching the retained API metadata. All **49 ZIP entries** passed CRC checks.
`raw/` preserves 16 byte-exact original records (180,224 bytes): API run/job/
artifact receipts; full compile/configure logs and commands; nine-file construction
receipt; native reader/loader logs and commands; failed summary; and the three
preparation/build-wiring logs. Each retained payload's original name, length and
SHA-256 is recorded. Original CRLF bytes are preserved. The ZIP, binaries and
private native fixture directories are not committed.

Focused local verification: all 15 portable constructor test methods pass, and
`git diff --check` passes. The source diff preserves the existing value, exact
mapped identity, lifecycle and lock assertions. No native Windows execution was
performed locally. **Native compilation and the complete 51-group run for this
change remain pending.** There is no application, vendor factory, chart, boat,
deployment or release acceptance claim.
