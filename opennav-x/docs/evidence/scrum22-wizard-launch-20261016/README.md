# SCRUM-22: checked Finish launch and retained diagnostics

The original 854 qualifier and corrected diagnostic both retained a Finish-page screenshot with **“Open SKAGER” checked**. These are existing native screenshots, reviewed only for functional identity and state. Exact artifact/member hashes are in [receipt.json](receipt.json); no images or large logs are committed.

The screenshots were captured **after** the helper attempted to clear only the obsolete caption `Launch SKAGER Beta 2`, immediately **before** it pressed Finish. The `wizard()` function is byte-identical across frozen product `854078a5`, first diagnostic helper `2ff32b34` and corrected helper `293eae12`; all three original installer reports match their exact Windows checkout source hashes. The packaged frozen `AlphaSetup.nsi` defines `Open SKAGER` and connects Finish to `LaunchXNav`, which uses `ExecShell` on the installed `SKAGER\Skager.lnk`. The helper subsequently launches the application directly and then starts supervised bootstrap. This establishes the caption mismatch and supports an unintended additional Finish launch path.

It **does not identify the transaction-lock holder**. The original qualifier’s application log records startup at 17:35:07, clean close at 17:35:15, another startup at 17:35:32 and a healthy recovery checkpoint at 17:36:08. The retained window snapshot shows an application and a separate startup dialog, but has no process-parent/creation trace. The healthy checkpoint cannot be attributed to a particular launch or treated as successful supervisor receipt authentication.

| Original run | Result retained |
| --- | --- |
| [38765418126/a1](https://github.com/ThereptileII/Work/actions/runs/38765418126) | Product `854078a5`; qualifier failed waiting for bootstrap launcher completion after 240 seconds. |
| [38878511171/a1](https://github.com/ThereptileII/Work/actions/runs/38878511171) | Helper `2ff32b34`; diagnostic failed in host-path validation with relative `WindowsPowerShell\v1.0\powershell.exe`. Supervisor was not invoked. |
| [38880343949/a1](https://github.com/ThereptileII/Work/actions/runs/38880343949) | Helper `293eae12`; WOW64 preflight passed. Original supervisor exited 1 without timeout; complete 704-byte stderr reports `transaction.lock` in use, surfaced through the top-level catch. No receipt was present; observed inputs were unchanged. |

Both diagnostic original artifacts were authenticated against official run/source/name/size/SHA-256 metadata and all member CRCs. Corrected diagnostic bindings match the original Setup, application, launcher and all three installed scripts, using the same I386 host as preflight. Its artifact retains no application startup log. The lock-open error does not establish an underlying receiver error or prove that cleanup masked it.

All failed results remain failed. This is evidence of a helper defect and retained diagnostic output, not corrected-helper acceptance or product qualification. No tests, application execution, new screenshots/downloads, CI retry, product edit, boat action or publication occurred during this bounded review.
