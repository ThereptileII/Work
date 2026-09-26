# Native boat-window review

`tools/boat/review-window.ps1` reviews an already running, audited, fixture-free
installed Beta 2 XNav. It does not launch OpenCPN, change marine connections,
enable equipment control or automate a multi-page tour. Root/operator review
of each screenshot determines the next action.

The input is the successful installed `Launch` job's `result.json`, its exact
SHA-256 and the expected product commit. The wrapper binds its paired request,
installed generation, owned `PRODUCT_BUILD.json`, executable hash and live PID.
Interactive execution repeats those checks and matches the process start time,
logged-in session, native frame and current foreground. Launch evidence expires
after four hours. Legacy, Safe, portable, synthetic and untracked launches are
not accepted by this helper.

Example, after examining the successful launch evidence:

```powershell
.\review-window.ps1 -Workspace C:\XNav `
  -LaunchResult C:\XNav\runs\<launch-run>\result.json `
  -ExpectedLaunchSha256 <exact-launch-result-hash> `
  -ExpectedCommit <exact-product-commit> -Action Capture
```

Each invocation permits exactly one of these actions:

| Action | Required visible control / effect |
|---|---|
| `Capture` | Save actual window pixels without interaction |
| `Resize1280x800` | Set the visible native frame to exactly 1280×800 physical pixels |
| `Menu` | Open the XNav menu |
| `Navigation`, `Route`, `Energy`, `PilotView`, `System` | Corresponding persistent bottom view button |
| `Routes`, `Waypoints`, `AIS`, `Instruments`, `Advice`, `Anchor`, `Settings`, `Alerts` | Matching visible menu entry; first invoke `Menu` separately |
| `Sources` | `SENSORS` on the Settings page |
| `Diagnostics` | `Diagnostics` on the System page |
| `CyclePalette` | The unique visible Day/Dusk/Night button |
| `ZoomIn`, `ZoomOut`, `Center` | Existing visible chart controls |
| `PageUp`, `PageDown` | Existing visible and enabled page scroll controls |
| `Escape` | One Escape press/release to the same process's focused child HWND |

The implementation selects fixed captions reviewed in `Shell.cpp` and
`ProductPanel.cpp`; there is no arbitrary caption, coordinate, key, command ID,
callback or script parameter. Each pointer interaction targets the unique
enabled, fully visible custom control HWND after a native hit-test. Escape is
also HWND-targeted. **No global `SendInput` or keyboard modifiers are used**:
focus theft cannot redirect a review chord into another application. PilotView
only opens the panel; STBY, AUTO, heading buttons, control enablement, route
activation/editing and configuration writers cannot be requested.

The frame must be foreground, enabled, visible and unminimized. Unknown modal
dialogs, ambiguous controls, clipped/offscreen buttons, missing controls or
sharing a foreground with another window cause refusal. The helper does not
scroll automatically to locate a control, dismiss native dialogs, retry clicks,
force-kill applications or restart Windows. Page-changing preferences, palette
and chart viewport may be saved normally by OpenCPN; “read-only” here means no
vessel/equipment command or navigation-object mutation.

Resize uses a temporary per-thread DPI context and DWM visible-frame bounds.
It never changes Windows resolution, DPI, taskbar, monitor configuration or
application DPI awareness. The current monitor's work area must accommodate
1280×800; otherwise it reports refusal rather than clipping content beneath the
taskbar or changing display settings. Exact observed size and DPI are recorded.

Private `before.png` and `after.png` files are stored beneath a new
`runs/<time>-window-review-<id>` directory accessible only to the interactive SID,
administrators and SYSTEM. Screenshots include only the reviewed window region;
foreground, bounds, DPI and overlap checks run immediately before and after
copying pixels, without reacquiring focus after capture. Enumeration errors are
reported after returning from Win32 callbacks. A visible overlapping window
invalidates capture. Results include image hashes, executable/build/generation,
review-helper hashes and the input method. Chart positions may be private:
review/redact deliberately before sharing evidence.

`test-review-window.ps1` exercises the pure policy and native interop compilation,
including denied commands, build/launch/session/PID reuse, fixture flags and
actual source-label mappings. It never sends native actions. Linux and native
PowerShell 5.1 each passed all 93 groups, including compilation of the native
helper. The native run used a separate validation tree on the boat PC; private
result: `evidence/local/boat-beta2/native-window-review-policy.json`.
No Win32 UI actions were executed by these tests. Actual installed-window
interaction and per-step screenshots remain separate acceptance gates.
