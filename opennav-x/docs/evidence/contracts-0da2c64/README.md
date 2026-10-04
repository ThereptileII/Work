# Completed 0da2c64 contract jobs

Read-only audit of [run 37207119257](https://github.com/ThereptileII/Work/actions/runs/37207119257), attempt **1**, candidate **0da2c64379d5a9cc4b9b2bd068de6e0b69816577**, mapped frozen local source **f1ea102cb362ed719cef50bb2c2506f1c470dd12**. Both contract jobs completed successfully; this is not an overall application or release pass.

| Original completed job | Main CTest passed / failed / skipped | Duration | Additional restart executions |
|---|---|---|---|
| Ubuntu 24.04 — **111450516696** | **94 / 0 / 0** | 5.65 s | 10 passes of one test, 19.47 s |
| Windows 2022, MSVC Win32 Release — **111450516760** | **91 / 0 / 0** | 6.68 s | 10 passes of one test, 20.01 s |

Individual result rows and both terminal summaries agree. These are **94 unique CTest names**, **185 main platform executions**, and **20 additional executions** of `restart_lifecycle_contract`, already present in both main suites. Three early-startup isolation tests are explicitly Linux-only in frozen CMake, not Windows skips. Conditional workflow-step skips are separately retained in `audit.json`.

Auxiliary Python suites report 11/9/17 source/OpenSSL/curl cases, 15/14/13/4 dependency cases, and 7/12 peer-boundary/installer-completion cases, all OK on both platforms. Linux navigation-copy adds 11. Windows records 44 filesystem and 238 shortcut-migration checks on each 64-bit and 32-bit PowerShell host. Installer output-policy invocations record 48 checks and explicitly perform no installer operation. These receipts are not added to unique CTest counts or treated as installed-product acceptance.

Published workflow and CMake bytes independently equal the frozen local Git blobs. Workflow: 53,821 bytes, SHA256 `c424ef555047b421a1add197ba9ce3a9869cb3c8f7b6ff1fa1f6950d02b6a52c`; CMake: 33,478 bytes, SHA256 `caf60fdf2f0f47ac7a08b355357ea004ad522ecc5a9d95efbef69ac0d85f153e`. This verifies these two mapped source files, not a reconstruction of the full publication tree.

`api.json` retains authenticated run and selected attempt-specific job records; the complete listing was checked for pagination. Numbered excerpts preserve every CTest result and auxiliary summary. Complete connector-decoded original logs remain ignored at `/home/standard/Projects/X-nav-worktrees/bccdbb1-contracts-restart/.local/0da2c64-audit/`:

- `linux-original.log`: **93,565 bytes**, SHA256 `e6deec5b70cc46a3f86db0d65d3124fa08991528a3b8e063a4eb30d4d7f78871`.
- `windows-original.log`: **131,118 bytes**, SHA256 `b5a3162a52274c46b161d4d2e4ec96cf5af04ac13ce1c4d6ad04bcb4f9c5f9c5`.

These hashes identify UTF-8 connector-decoded logs with original newline/BOM representation, not an unprovided raw-log archive. The frozen contract job has no artifact upload. The [restart receipt](../restart-0da2c64/README.md) is a separate same-run prerequisite. No tests, builds, reruns, source changes or boat operations were performed for this audit. Endurance is outside this task; application runtime, Windows integration, installer/package, visual and actual boat acceptance remain separate.
