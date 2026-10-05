# Completed ecf7e0c contract jobs

Read-only audit of [run 37191400051](https://github.com/ThereptileII/Work/actions/runs/37191400051), attempt **1**, remote candidate **ecf7e0c46609cf4cb29141964d7c4bde98b002b7**, mapped local frozen source **1988df7a8a0ae8ddc6365ca46026d32fccfa0bdc**. Authenticated attempt-specific job records and the original completed logs agree on identity and success. The overall run was still in progress at collection.

| Completed contract job | Main CTest passed / failed / skipped | Main duration | Additional restart executions |
| --- | --- | --- | --- |
| [Ubuntu 24.04 — 111404167322](https://github.com/ThereptileII/Work/actions/runs/37191400051/job/111404167322) | **94 / 0 / 0** | 5.73 s | 10 passes of one test; 19.48 s |
| [Windows 2022, MSVC Win32 Release — 111404167372](https://github.com/ThereptileII/Work/actions/runs/37191400051/job/111404167372) | **91 / 0 / 0** | 5.8 s | 10 passes of one test; 19.88 s |

Every individual CTest result and both terminal summaries were inspected. There are **94 unique CTest names**, **185 main-suite platform executions**, and **20 additional executions** of `restart_lifecycle_contract`, already counted in each main suite. Repeats add no unique tests. Linux alone registers `early_startup_fixtures_isolation`, `early_startup_product_isolation`, and `early_startup_msw_excluded_isolation`; the frozen CMake explicitly confines these three to Linux. They are not Windows skips. Conditional workflow-step skips are separately listed in `audit.json`.

Separate Python suite receipts report 11/9/17 source/OpenSSL/curl cases, 15/14/13/4 dependency cases, and 7/12 peer-boundary/installer-completion cases, all OK on both platforms; Linux navigation-copy additionally reports 11, OK. Native Windows filesystem and shortcut receipts report 44 and 238 checks on each 64-bit and 32-bit PowerShell host. Installer output-policy scripts report 48 checks per invocation and no installer operation. These counts are not added to the CTest unique total or treated as installed-product acceptance.

Remote workflow and CMake bytes independently match the mapped local frozen source; hashes are retained in `audit.json`. `api.json` preserves selected authenticated records from the complete 16-job attempt listing. Numbered excerpts retain every CTest result and selected auxiliary receipts. Full connector-decoded originals remain in this worktree's ignored `.local/ecf7e0c-audit/{linux,windows}-original.log`, preserving LF/CRLF and BOM bytes:

- Linux: **92,935 bytes**, SHA-256 `1f6a239483b0d503c222e7f9625a7f9965d03821742d76928733006a06a860cc`.
- Windows: **130,181 bytes**, SHA-256 `59d58dde5abc5d43823dc90dd1dd1b50ac6cec74129f8ad03187e48abbaba03e`.

These identify connector-decoded logs, not an unprovided raw log archive. The frozen contracts job has no artifact upload. No tests, builds, CI changes, or boat actions occurred during this audit. The [same-run restart receipt](../restart-ecf7e0c/README.md) supplies a separate prerequisite. **Application runtime, security, installer/package, visual and actual boat acceptance remain separate; these jobs do not qualify the whole candidate.**
