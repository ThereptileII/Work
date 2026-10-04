# Completed contract jobs for bccdbb1

Read-only audit of [run 37177738716](https://github.com/ThereptileII/Work/actions/runs/37177738716), attempt **1**, remote commit **bccdbb11cef827d3b63731fe1e874bc72bf47d2d**. Authenticated run/jobs records bind both completed successful jobs to that exact commit and attempt; each original log independently confirms the checkout and extends through cleanup. The whole run was still in progress at collection.

| Completed job | Main CTest passed / failed / skipped | Main time | Additional restart executions |
| --- | --- | --- | --- |
| [Linux, Ubuntu 24.04 — 111363797659](https://github.com/ThereptileII/Work/actions/runs/37177738716/job/111363797659) | **94 / 0 / 0** | 5.98 s | Ten passes of one test; 19.46 s |
| [Windows 2022, MSVC Win32 Release — 111363797728](https://github.com/ThereptileII/Work/actions/runs/37177738716/job/111363797728) | **91 / 0 / 0** | 6.53 s | Ten passes of one test; 19.96 s |

Every individual CTest result and terminal summary was checked. There are **94 unique CTest names across the two platforms**, **185 main-suite platform executions**, and **20 additional executions** of the same `restart_lifecycle_contract` (which already ran once in each main suite). The repeats add no unique tests. Linux additionally registers `early_startup_fixtures_isolation`, `early_startup_product_isolation`, and `early_startup_msw_excluded_isolation`; the frozen CMake explicitly limits these three registrations to Linux. They are not skipped Windows tests. Conditional workflow-step skips are separately recorded in `audit.json`.

Other receipts remain separate from CTest totals: the source/OpenSSL/curl Python suites report 11/9/17 tests, dependency receipt/evidence/reuse/staging 15/14/13/4, peer-boundary/installer-completion 7/12, all OK on both platforms. Linux navigation-copy additionally reports 11 tests, OK. Windows native filesystem and shortcut suites report 44 and 238 checks on each 64-bit and 32-bit PowerShell host. Output-policy scripts report 48 checks per invocation and explicitly perform no installer operation. These assertion counts do not establish installed-product acceptance.

The exact remote workflow and CMake bytes independently match mapped local candidate **1a6733a1cbcc817aa0f13fa5acc41aac62a54146**. SHA-256 identities are in `audit.json`; larger publication mapping remains root evidence. This audit worktree starts at `543ab192a5419367c48a99bc01b08798367871fc`.

`api.json` retains selected authenticated run/jobs records. Numbered `linux-excerpts.log` and `windows-excerpts.log` preserve original line positions, every CTest result, and selected independent receipts. Complete connector-decoded originals remain in this worktree's ignored `.local/{linux,windows}-bccdbb1-original.log`, preserving original LF/CRLF bytes. Their UTF-8 identities are:

- Linux: **92,935 bytes**, SHA-256 `47b790b5b50d704ef44a90ff345ad6fab45435cd658289cb55ac167e33647c02`.
- Windows: **131,056 bytes**, SHA-256 `5eb31f4df4cc6fa6618cd89a48167db7d357c6037c8b44542d70056375cb99c8`.

These hashes identify the connector-decoded logs, not an unprovided raw log ZIP. The frozen contracts job has no artifact upload step, so no separate contract-result archive is claimed. No tests, build, CI mutation, or boat action was performed for this audit. **Integrated application, native runtime, installer, visual and physical boat acceptance remain separate gates; these successes do not qualify the whole candidate.** The same-run restart prerequisite is recorded separately in `../restart-bccdbb1/`.
