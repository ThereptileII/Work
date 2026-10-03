# Read-only boat readiness for the final d5 candidate

The 2026-10-03 09:50–09:55 UTC inventory used only the configured `ssh boat`
alias and existing audited read-only inventory/recovery code. No installation,
launch, process close, profile/output edit, staging, retirement, remote-access
change or hardware command occurred. [Sanitized receipt](boat-readiness-final-d5-20261003.json)
contains the measured identities; private raw inspection and query evidence stay
in ignored `evidence/local/boat-final-readiness/` in this isolated d5 worktree.

The boat is reachable and idle: SSH, Tailscale and RustDesk are Running/Automatic;
Tailscale reports Running/online with no health warnings. No navigation/helper
process or commissioning marker was present. Stock OpenCPN remains the exact
supported x86 5.12.4 executable. Installed Beta 1 remains `a3e6e081` and matches
its executable ownership record. Normal profile and installed-state hashes are
unchanged (`d891d88c…` / `a6ec7cda…`). Intel UHD Graphics reports 1920×1080,
driver `32.0.101.7088`; stored AppliedDPI is 144, not a rendered-window measurement.

The completed cold-baseline record still hashes to `543f63c…`; its capture,
review, predecessor and saved/live INI all match. All 2,031 recorded live and
saved-profile entries match, with the live tree still exactly 2,031 entries.
The separate October 1 recovery record hashes to `9f945ac…`, and all 2,123
application plus 2,008 profile entries match. These are distinct records, not
the same backup hash. No reparse entries or mismatches were found. The old
Beta 1 portable manifest, actual marker and executable identity remain present.
The cold baseline is preservation-only and explicitly grants no launch permission.
ACL acceptance, fresh plugin/output review and new installed-build commissioning
remain separate gates.

All **117** files in the staged qualified `ffa2de31` bundle match the original
independent deployment manifest; its completion record remains `f62f6274…`.
Its qualification remains native run **37090716935**, whose downloaded-evidence
receipt is retained under SCRUM-258. This refresh rehashed deployed bytes; it did
not rerun those native tests or transfer acceptance to the new application.

## Exact current-tool differences and native coverage

Comparison is qualified local tool source `3da0e563` (published `ffa2de31`)
against final local source `d5d71356` (published `b8cfbf80`). Only the following
nine existing tool files differ, all solely in diagnostic/error-message wording:

| Changed file under tools/boat | Current native gate relevant to its use |
| --- | --- |
| `Preparation.ps1` | Native boat recovery/commissioning: preparation, commissioning and launch suites; actual broker/Prepare/Arm job |
| `RestartWindowNative.cs` | Fixed guarded mode UI actions: actual restart-window fixture |
| `ReviewWindowNative.cs` | Fixed guarded mode UI actions: actual display-window fixture |
| `StockReview.ps1` | Native maintenance stock-review/chart/welcome suites; actual stock close/resize and official welcome jobs |
| `retire-portable.ps1` | Native maintenance boat-tools transaction/refusal suite |
| `smoke-test.ps1` | Underlying launch/review/close components have the above gates; no direct invocation of this orchestration script was found |
| `start-official-upgrade.ps1` | Native maintenance official-upgrade-policy suite, including dispatch boundaries |
| `upgrade-official-opencpn.ps1` | Official prerequisite/policy gates are relevant; no direct invocation of this orchestration script was found |
| `upgrade-stock.ps1` | Native maintenance boat-tools AST/policy coverage; actual boat upgrade remains separate |

The one added operator is `inspect-fonts.ps1`, a read-only installed-family
inventory. No dedicated current-run invocation was found. It is not required to
use the existing guarded review/restore sequence. No files were removed.

The guarded Common/InteractiveJob, commissioning/cold readers, restart
Prepare/Arm/Broker/policy, review policies, run-mode, commission-read-only and
normal-stop modules are unchanged. Sixteen such dependency files were checked
against staged bytes using only the already proven LF-to-Windows-CRLF checkout
normalization. Source inspection found no required API/behavior incompatibility
with retaining `ffa2de31`; actual new-product behavior still requires its own
qualified package and fresh guarded review.

In current candidate run **37114216075**, API observation confirms success for
native recovery/commissioning, restart process boundary, guarded UI actions,
actual broker/Prepare/Arm, Windows contracts and the private-loader refusal gate.
The native application slice is still in progress. These are current job-status
observations, not independently downloaded artifact acceptance by this audit.
The main workflow does not execute the separate cold-baseline/helper/attempt
suites included in the old tool bundle's own qualification. Keep those evidence
sets distinct; the current application run does not silently replace them.

**Recommendation:** retain the exact already qualified staged `ffa2de31` bundle
for the eventual guarded review/restore. Do not stage a new composition merely
for message wording. Wait for root's qualified, independently hash-verified
application artifact before installation or launch. Package/installer/display,
real host adapter loading, actual boat chart/mode review and remaining endurance
acceptance are not established by this readiness check.

A first attempt to import the preserved helper file was refused by the host's
script-file execution policy before importing it. The retry streamed only the
existing audited read-only helper definitions in memory, as the established
inventory queries do; no execution-policy setting or boat file changed. An
unauthenticated local CLI metadata attempt was replaced with the authenticated
GitHub connector. Both limitations are retained in private local evidence.
