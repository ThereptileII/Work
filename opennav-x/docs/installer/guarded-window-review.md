# Reviewing a commissioned in-app restart

This is an optional boat-development tool, separate from normal product UI. It
does not arm a broker, start an application, update an audit, change an input
connection, or operate equipment. Native qualification and a guarded installed
generation are required before actual boat use.

The underlying commissioning session remains the authority: exact product and
helper bytes, input-only profile, plugin quarantine/source review, current
generation, initial environment, SID/session and bounded lifetime. The ordinary
`ReviewWindow` action remains tied to its original cold-launch evidence.

## One requested mode change

1. Prepare the commissioning session and launch its exact cold child through the
   existing opt-in launch path. For XNav, use separate display-only review actions
   to open **System**, then inspect its private screenshot.
2. Use `RestartCommissioningArm.ps1 -Action Arm` for the intended parent PID,
   creation FILETIME and one of `--xnav`, `--legacy`, `--safe-mode`. Save the exact
   immutable session, Arm and `ready.json` hashes.
3. Invoke `review-restart-window.ps1 -Action RequestMode` with those identities.
   The tool rechecks the live broker image, PID, creation time, SID/session,
   fixed invocation and owned **Running** scheduled task. Readiness must be less
   than 90 seconds old, leaving margin before its 120-second listening bound.
4. The tool captures the actual foreground application frame, then creates
   `ui-intent-consumed.json` before sending one UI event. It never overwrites that
   journal or retries an uncertain action.
5. Inspect broker outcome. Collect its completed task through the existing
   `RestartCommissioningArm.ps1 -Action Collect` path. A UI send is not evidence
   that any child started. Missing/denied/uncertain outcomes require inspection,
   not an unguarded launch or another mode click.

The XNav path requires its visible **System** page and unique enabled **Open
Legacy OpenCPN**, **Restart XNav**, or **Safe Mode** button. Before release it
rechecks the same button handle, process, caption, page and geometry after the
press callback. Legacy/Safe return uses the actual visible native menu's unique
enabled **Switch to XNav** item; the menu's current command identity is resolved
from that exact source-reviewed caption. There is no caller-supplied command ID,
arbitrary caption, coordinate, accelerator or global input injection. A hidden
menu or any modal dialog blocks the operation.

```powershell
.\review-restart-window.ps1 -SessionRecord $sessionRecord `
  -ExpectedSessionSha256 $sessionHash -ExpectedCommit $commit `
  -Action RequestMode -Mode '--legacy' `
  -ParentProcessId $parentPid -ParentCreatedFiletime $parentCreated `
  -ExpectedArmSha256 $armHash -ExpectedReadySha256 $readyHash
```

## Review the actual child

`-Action ReviewChild` takes the exact `completion.json` path and hash. It verifies
every completed transition, consumed permit, original request bytes, native
receipt, post-close profile copy and typed allowed INI delta, then proves the
currently running child PID, creation time, executable, SID/session and mode.
Its result uses `proofKind=completed-guarded-restart`; it never constructs a
replacement `LaunchResult` or calls a process-start API.

```powershell
.\review-restart-window.ps1 -SessionRecord $sessionRecord `
  -ExpectedSessionSha256 $sessionHash -ExpectedCommit $commit `
  -Action ReviewChild -CompletionFile $completion `
  -ExpectedCompletionSha256 $completionHash -ReviewAction Capture
```

Legacy/Safe children permit only capture and physical `Resize1280x800`. XNav
children also permit the existing fixed display-only review actions. No action
enables hardware control or activates a route. Perform each action separately
and review its image before choosing another. Before arming the next transition,
finish display navigation first: an incomplete/pending transition cannot stand
in for a completed child proof. The sixteenth completed child can still be
reviewed; it cannot obtain a seventeenth restart permit.

Screenshots are private, contain only the validated application frame and can
contain boat/chart positions. Foreground, unobscured bounds and DPI are checked
before and after pixel capture without reacquiring focus between those checks.
Windows display resolution/DPI, remote access and unrelated windows are not
changed. Native/Legacy modal notices require a separate deliberate human action;
this tool does not automatically dismiss them.

## Qualification

- `tools/boat/test-restart-window-review.ps1`: 129 portable policy/native-compile
  and actual wire/journal checks, including identity/expiry denial, single-use
  intent, wrong receipt child, incomplete chains and the sixteenth-child limit.
- Existing restart policy (269 local checks at this source boundary) and
  display-only review (145 checks) remain passing.
- `tools/test-restart-window-native.ps1` provides nine native fixed-control
  fixtures: three XNav mode buttons, Legacy/Safe return menu, duplicate button,
  button replacement during press, hidden menu and unexpected modal. It starts
  only a bounded PowerShell/WinForms marker with no OpenCPN/profile/plugins.
- Native execution of these new window fixtures, the actual installed UI path
  and boat mode-restart acceptance are still pending. Earlier native broker and
  marker-process qualification does not establish this UI handoff's acceptance.
