# Installed Beta 2 first-start navigation caution

`review-installed-welcome.ps1` provides only two operations: `InspectWelcome`
and `AcknowledgeWelcome`. It is separate from the official stock identity path.
It accepts only the actual owned, fixture-free `0.4.0-beta2` installation and a
recent normal `Launch` receipt for XNav, Legacy or Safe Mode. It cannot authorize
a portable process, a stock process, or a broker-restarted child.

The pinned upstream `ShowNavWarning` runs before `opennav::Attach` (upstream
startup lines 1937 and 1990 respectively in the accepted patch). Thus the
first-start modal still owns the upstream OpenCPN frame in every installed mode.
The installed helper reuses the **same** `StockWelcomeNative` matcher and
capture/hash-bound single-Agree primitive. It adds no generic dialog driver or
new native input operation. After Agree, the resulting frame must have the
existing exact caption for the launched XNav, Legacy or Safe mode.

## Proof before any acknowledgement

New normal-launch receipts contain the exact process creation time, SID/session,
target-record hash, executable hash, generation/commit and commissioning binding.
The existing installed launch audit is unchanged; old receipts missing this
proof are refused by the new warning wrapper. The helper verifies:

- the hash-bound owned launch request/result and exact live PID/start tick;
- current installed state/ownership and owned `PRODUCT_BUILD.json`, with
  `test_fixtures: false` and `build_purpose: INSTALLED PRODUCT`;
- the unchanged target and active prepared/applied commissioning records;
- the exact managed, stock and installed plugin roots, every retained helper and
  resource, copied source-review evidence, and absent quarantined plugins with
  their preserved copies;
- the real shared profile, independent read-only approvals, current input-only
  connections, and unchanged protected chart/source configuration.

This is an audit of an **already running** application. It does not disable the
cold launch verifier's closed-process gate, renew an audit hash, launch another
process, or modify user configuration. Harmless normal UI persistence is checked
against the existing commissioning profile boundary; changing a connection or
chart path still refuses the operation.

## Procedure

Use a fresh normal launch result from `run-xnav.ps1`, `run-legacy.ps1`, or
`run-safe.ps1`. Pass its `result.json`, exact hash and installed commit to:

```powershell
review-installed-welcome.ps1 -Workspace <workspace> `
  -LaunchResult <launch-result.json> -ExpectedLaunchSha256 <sha256> `
  -ExpectedCommit <commit> -Action InspectWelcome
```

Visually review the private `welcome.png`. The body must be the pinned English or Swedish
OpenCPN GPL/no-warranty/navigation caution with Agree and Cancel. The HTML text is
not accessible through ordinary `GetWindowText`; the result explicitly records
that limitation. Do not treat a licensing, purchase, privacy, plugin or other
startup prompt as this caution.

After review, run the same command with `-Action AcknowledgeWelcome`, adding
`-WelcomeInspection <inspection-directory/review.json>` and
`-ExpectedWelcomeInspectionSha256 <its exact hash>`. The inspection expires after
30 minutes. The full installed/runtime proof is repeated immediately before a
new capture. The current pixels, native window identity, buttons, geometry and
DPI must match the reviewed warning. A durable exclusive intent is written
before one click on the existing Agree button. Uncertain delivery cannot be
retried using that inspection.

The receipt distinguishes transmission from confirmed modal dismissal and saves
an after-image of the exact installed mode. A different startup prompt or missing
frame produces an attention result. No other prompt is dismissed, no process is
killed, and no equipment or navigation command is issued. Continue normal
read-only review and clean-close/profile inspection separately.

## Validation scope

`test-installed-welcome.ps1` covers policy, old/cross-kind receipts, exact process
identity, fixture-enabled/incorrect product reports, changed inspection evidence,
and the actual running-proof function against disposable files. The mutation
fixture changes installed ownership/executable, retained helpers, all plugin
roots, quarantine/recovery bytes, source evidence, transaction records and
protected profile inputs. It also proves missing independent read-only approvals
refuse the operation. It launches no application and invokes no Windows API.

The shared actual official-stock wx warning fixture and native installed-mode
review remain separate acceptance gates. A passing pure policy suite is not a
claim that the boat warning was inspected or acknowledged.
