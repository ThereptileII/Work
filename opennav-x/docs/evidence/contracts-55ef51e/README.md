# Completed 55ef51e contract jobs

Read-only audit of [run 37199379614](https://github.com/ThereptileII/Work/actions/runs/37199379614), attempt **1**, candidate **55ef51e4944e8570f6a391dad20b0db8447a7443**, mapped frozen local source **e768b06bf5c130038abfd6e1166da0078037cb5b**. Both contract jobs completed successfully; this is not an overall application or release pass.

| Original completed job | Main CTest passed / failed / skipped | Duration | Additional restart executions |
|---|---|---|---|
| Ubuntu 24.04 — **111427724099** | **94 / 0 / 0** | 5.68 s | 10 passes of one test, 19.46 s |
| Windows 2022, MSVC Win32 Release — **111427724201** | **91 / 0 / 0** | 5.19 s | 10 passes of one test, 19.76 s |

Individual result rows and both terminal summaries agree. These are **94 unique CTest names**, **185 main platform executions**, and **20 additional executions** of `restart_lifecycle_contract`, already present in both main suites. Three early-startup isolation tests are explicitly Linux-only in frozen CMake, not Windows skips. Conditional workflow-step skips are separately retained in `audit.json`.

Auxiliary Python suites report 11/9/17 source/OpenSSL/curl cases, 15/14/13/4 dependency cases, and 7/12 peer-boundary/installer-completion cases, all OK on both platforms. Linux navigation-copy adds 11. Windows records 44 filesystem and 238 shortcut-migration checks on each 64-bit and 32-bit PowerShell host. Installer output-policy invocations record 48 checks and explicitly perform no installer operation. These receipts are not added to unique CTest counts or treated as installed-product acceptance.

Published workflow and CMake bytes independently equal the frozen local Git blobs. Workflow: 50,790 bytes, SHA256 `29894c4471453ef0b183513a4dba3b80c20c27b7b2fac0a3980515c61c8a1a8d`; CMake: 33,478 bytes, SHA256 `caf60fdf2f0f47ac7a08b355357ea004ad522ecc5a9d95efbef69ac0d85f153e`. This verifies these two mapped source files, not a reconstruction of the full publication tree.

`api.json` retains authenticated run and selected attempt-specific job records; the complete listing was checked for pagination. Numbered excerpts preserve every CTest result and auxiliary summary. Complete connector-decoded original logs remain ignored at `/home/standard/Projects/X-nav-worktrees/bccdbb1-contracts-restart/.local/55ef51e-audit/`:

- `linux-original.log`: **92,942 bytes**, SHA256 `29ca295d645c06c93ba55395dd601539855251f71b4e7d99c5353d3537e89f08`.
- `windows-original.log`: **129,105 bytes**, SHA256 `a2ffaf20c7f885cc3d3118f3091a0959fe7449c6584de8a62d10f68ed8bf5c52`.

These hashes identify UTF-8 connector-decoded logs with original newline/BOM representation, not an unprovided raw-log archive. The frozen contract job has no artifact upload. The [restart receipt](../restart-55ef51e/README.md) is a separate same-run prerequisite. No tests, builds, reruns, source changes or boat operations were performed for this audit. Endurance is outside this task; application runtime, Windows integration, installer/package, visual and actual boat acceptance remain separate.
