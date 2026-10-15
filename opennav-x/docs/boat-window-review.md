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
| `SelectFirstVisibleWaypoint` | First enabled, fully visible waypoint row on the Waypoints list; opens its existing detail page |
| `SelectFirstVisibleAis` | First enabled, fully visible target row on the AIS targets list; opens its existing detail page |
| `OnlineAISSettings` | `Online AIS settings` in the owned traffic drawer's list view |
| `OnlineAISEnable`, `OnlineAISDisable` | `Enabled` or `Off` in that drawer's Online AIS settings view |

The implementation selects fixed captions reviewed in `Shell.cpp` and
`ProductPanel.cpp`; there is no arbitrary caption, coordinate, key, command ID,
callback or script parameter. Each pointer interaction targets the unique
enabled, fully visible custom control HWND after a native hit-test. Escape is
also HWND-targeted. **No global `SendInput` or keyboard modifiers are used**:
focus theft cannot redirect a review chord into another application. PilotView
only opens the panel; STBY, AUTO, heading buttons, control enablement, route
activation/editing and arbitrary configuration writers cannot be requested.

The frame must be foreground, enabled, visible and unminimized. Unknown modal
dialogs, ambiguous controls, clipped/offscreen buttons, missing controls or
sharing a foreground with another window cause refusal. The helper does not
scroll automatically to locate a control, dismiss native dialogs, retry clicks,
force-kill applications or restart Windows. Page-changing preferences, palette
and chart viewport may be saved normally by OpenCPN; “read-only” here means no
vessel/equipment command or navigation-object mutation.

## Bounded Online AIS preference check (SCRUM-301)

After `AIS` opens the current Traffic list, invoke `OnlineAISSettings` separately
and inspect its screenshot before `OnlineAISEnable` or `OnlineAISDisable`.
These three fixed actions require the existing owned `SKAGER vessel traffic`
drawer, its direct `AIS scroll body`, the source's Close/Back heading and one
direct custom button with its exact caption and accessibility name. Clipped,
covered, disabled, missing, foreign, renamed and duplicate targets refuse. There
is no automatic scrolling, key entry/removal, radius adjustment or input retry.

Each action observes fresh exact-build installed diagnostics from the configured
profile's `opennav-logs/opennav-diagnostics.json` before input and requires a newer
publication showing the expected settings page afterward. Enable/disable also
require the matching boolean preference; disable requires the Disabled provider
state. A click without that observation reports failure, even if input was sent.
The private result records the before/after ticks, page, preference and connection
enum without credential contents. No existing launch/hash/PID/session guard is
replaced. These settings persist normally; record and restore the original state.

Use the same launch receipt/hash/product commit for separate `Escape`, `ZoomOut`,
`ZoomIn` and `PanRight` invocations, examining each image and fresh UI ticks, then
return to `AIS` and the settings view to disable or restore. A usable stored key
must already exist for real service acquisition. Enabled alone does not establish
Connected, target receipt or chart marks. Off is an explicit user disable, **not a
network outage**; actual report silence/offline/reconnect acceptance remains open.
Advancing Shell ticks and screenshots provide coarse responsiveness evidence,
not input-to-frame latency or asynchronous chart-paint timing.

The focused portable policy check is `tools/boat/test-review-window.ps1`. The
harmless native fixture can be selected without a navigation application:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/test-display-window-native.ps1 `
  -Evidence <new-private-evidence-directory> -OnlineAisOnly
```

This requires disposable Windows CI, or `-IsolatedLocal` for an explicitly
authorized disposable local desktop. It refuses coexistence with OpenCPN. The
14 native cases cover all three actions, wrong page/owner, duplicate, clipped,
covered, absent, wrong accessibility name/role, replacement during press, stale
publication and mismatched post-click preference. They exercise only inert
Windows Forms controls and synthetic diagnostic observations, never a provider,
credential, physical output or installed product. The existing
`skager-boat-ais-check.yml` runs these cases with the maintained native fixture;
portable passes do not qualify Win32 input or actual installed AIS behavior.

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


## Bounded read-only list selection

Open `Menu`, review the screenshot, then open `Waypoints` or `AIS` and review
that screenshot before using the corresponding `SelectFirstVisible…` action.
Each is a separate invocation using the same exact installed launch evidence.
The helper chooses the topmost fully visible row, then the leftmost row at the
same vertical position. It records that selected label and HWND in the private
review result. Disabled, clipped, nested, duplicated and overlapping candidates
are refused or excluded; absent objects remain absent. The tool neither creates
test objects nor generates AIS reports.

The native selector requires the exact product-list page, immediate page-child
controls and the row-label grammar used by `ProductPanel.cpp` and the AIS bridge.
Waypoints have the ` / mark` or ` / in route` suffix. AIS rows carry the existing
OpenCPN-derived target-health prefix; optional navigation-status text remains
upstream data. It rechecks the chosen page, caption, HWND, location, bounds,
foreground and hit-test immediately before one target-local press/release. No
name, MMSI, waypoint identifier, row index, arbitrary caption or position can be
provided by a caller. Names and positions in the result are private evidence.

Selection opens the existing **detail page**; it does not synthesize a tap on the
chart or claim that the compact chart context-card interaction was exercised.
GO TO, activation, waypoint editing/removal, target tracking, pilot buttons and
connection changes remain inaccessible through this tooling. `Navigation` or
`Menu` remains the existing way back. A missing expected detail page after the
one click reports failure without retrying; inspect the saved pre-action image
and capture the current state separately.

The extension retains the same installed-generation/hash/PID/session checks and
the successful independently audited launch request. It adds no new launch path
and does not fabricate, refresh or substitute a commissioning attestation.

## In-application restart remains outside this helper

`RestartLegacy`, `RestartSafe` and `RestartXNav` are explicitly refused. The
application's `PrepareClose` persists configuration; `CompleteRestart` starts
`opennav-restart.exe`, which waits for exit and launches the next mode. External
review tooling has no pause between persistence and the successor launch. The
full commissioning audit deliberately requires all OpenCPN/helper processes
closed, so it cannot be reused while that automatic sequence is in progress.

A known same-generation child/PID proves process identity, but does not prove
that the post-close configuration still has the reviewed connections, plugin
permissions, chart paths and actuator settings. This extension therefore does
not copy the new INI hash into an audit or accept that child as a newly audited
launch. A safe automated in-application test needs a separately reviewed
interposition/verification boundary and is not implemented here.

The supported commissioning sequence remains: close normally, inspect the
post-close configuration against the strict commissioning baseline, review and
renew the audit, then use the existing guarded mode launcher. That validates
separate mode starts; it must not be reported as an in-application mode-switch
acceptance result. No forced close, repeated restart, restored output plugin or
physical actuator command is used to get past a refusal.

The selection extension's Linux policy/native-code compilation run passed
**145 groups**, retaining all prior checks and adding source grammar, screen
ordering, wrong-page, absent/disabled/clipped/nested rows, duplicate identities,
malformed labels, unsafe actions and restart-refusal coverage. These tests never
execute Win32 UI actions. Native PowerShell 5.1 execution and actual installed
selection screenshots remain pending; the previous 93-group native result above
does not qualify these additions.
