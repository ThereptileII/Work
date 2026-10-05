# Completed d29da37 contract jobs

Read-only audit of [run 37184477492](https://github.com/ThereptileII/Work/actions/runs/37184477492), attempt **1**, remote candidate **d29da372af86c01cb2817f932891fd9408d882fe**, mapped local frozen source **615118f351abdea7b79a042db63e58fe0a635e0a**. Authenticated attempt-specific job records and the original completed logs agree on identity and success. The overall run was still in progress at collection.

| Completed contract job | Main CTest passed / failed / skipped | Main duration | Additional restart executions |
| --- | --- | --- | --- |
| [Ubuntu 24.04 — 111383445869](https://github.com/ThereptileII/Work/actions/runs/37184477492/job/111383445869) | **94 / 0 / 0** | 3.91 s | 10 passes of one test; 19.29 s |
| [Windows 2022, MSVC Win32 Release — 111383445819](https://github.com/ThereptileII/Work/actions/runs/37184477492/job/111383445819) | **91 / 0 / 0** | 6.74 s | 10 passes of one test; 19.98 s |

Every individual CTest result and both terminal summaries were inspected. There are **94 unique CTest names**, **185 main-suite platform executions**, and **20 additional executions** of `restart_lifecycle_contract`, already counted in each main suite. Repeats add no unique tests. Linux alone registers `early_startup_fixtures_isolation`, `early_startup_product_isolation`, and `early_startup_msw_excluded_isolation`; the frozen CMake explicitly confines these three to Linux. They are not Windows skips. Conditional workflow-step skips are separately listed in `audit.json`.

Separate Python suite receipts report 11/9/17 source/OpenSSL/curl cases, 15/14/13/4 dependency cases, and 7/12 peer-boundary/installer-completion cases, all OK on both platforms; Linux navigation-copy additionally reports 11, OK. Native Windows filesystem and shortcut receipts report 44 and 238 checks on each 64-bit and 32-bit PowerShell host. Installer output-policy scripts report 48 checks per invocation and no installer operation. These counts are not added to the CTest unique total or treated as installed-product acceptance.

Remote workflow and CMake bytes independently match the mapped local frozen source; hashes are retained in `audit.json`. `api.json` preserves selected authenticated records from the complete 16-job attempt listing. Numbered excerpts retain every CTest result and selected auxiliary receipts. Full connector-decoded originals remain in this worktree's ignored `.local/d29da37-audit/{linux,windows}-original.log`, preserving LF/CRLF and BOM bytes:

- Linux: **92,938 bytes**, SHA-256 `609a5ee5200a125d508758a9ae678308cceb9c13e5a4574b5f9be2d581d9040b`.
- Windows: **131,055 bytes**, SHA-256 `5e3c153db8f6cff8ba0971ea538f654fb4d2346b6c864b83729f7583563320de`.

These identify connector-decoded logs, not an unprovided raw log archive. The frozen contracts job has no artifact upload. No tests, builds, CI changes, or boat actions occurred during this audit. The [same-run restart receipt](../restart-d29da37/README.md) supplies a separate prerequisite. **Application runtime, security, installer/package, visual and actual boat acceptance remain separate; these jobs do not qualify the whole candidate.**
