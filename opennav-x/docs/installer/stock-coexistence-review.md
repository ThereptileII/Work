# Actual stock OpenCPN review after uninstall

`run-stock.ps1` and `review-stock.ps1` provide a separate read-only desktop
coexistence check after OpenNav uninstall. They do not invent an installed XNav
generation, use a portable profile, or pass `--legacy` to upstream OpenCPN.
The official executable receives **empty arguments** and its ordinary shared
profile. Stock Safe Mode is not exercised by these tools.

Only the already validated OpenCPN 5.12.4 Win32 executable is accepted:
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`, pinned source
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. No parameter can extend that allowlist.
The exact PE/version check is repeated by commissioning context discovery.
Any OpenNav `state.json`, unfinished `transaction.json`, or uninstall
registration in either current-user registry view blocks this stock path.
Retained ownership/recovery logs are not treated as an active installation.

## Required sequence

1. Close XNav and its plugin helpers normally; restore any active commissioning
   transaction before installer maintenance. Perform the existing verified
   uninstall and inspect its results. Preserve the user's navigation/profile data.
2. Prepare **a new stock-only commissioning transaction** using
   `commission-read-only.ps1`. Its context must have `installation: null` and
   exactly the actual stock and managed plugin roots. An earlier installed
   transaction is refused, even if its executable originated from the same
   OpenCPN version. The existing fixed recovery baseline still applies; these
   tools do not broaden recovery or erase intervening user changes.
3. Review the full inventory/source plan and apply its input-only/quarantine
   preparation. Independently add `stockReadOnlyAudit` to the private
   `boat-target.json`, retaining the normal `stockExecutable` and
   `profileDirectory`. The audit contains:
   - `launchKind: "StockLegacy"`, the exact `executableSha256` above, and the
     pinned `upstreamCommit` above;
   - actual reviewed `profileIniSha256` and `reviewedUtc` (maximum age 24 hours);
   - explicit reviewed booleans `connectionsOutputDisabled`,
     `pluginOutputsReviewed`, `noActiveRouteOutput`;
   - `commissioning.record`, `recordSha256`, and `appliedSha256` from this new
     transaction. No tool manufactures these approvals.
4. Run `run-stock.ps1 -Workspace <workspace>`. Both dispatch and interactive
   execution verify the active prepared/applied journal, source evidence,
   original/input profile byte proof, complete plugin/helper trees, absent
   quarantined DLLs and their preserved recovery copies, source review age, and
   current input-only shared profile. The child receives the reviewed explicit
   application/System32/Windows PATH and working directory; no global environment
   changes occur and no OpenNav restart broker is armed.
5. Preserve the returned private `runs/*-launchstock-*/result.json` and its hash.
   Use `review-stock.ps1 -Workspace <workspace> -LaunchResult <result.json>
   -ExpectedLaunchSha256 <hash> -Action Capture` to inspect the real chart window.
   `-Action Resize1280x800` permits only that fixed physical window size and
   captures its result. If the monitor work area is smaller, it refuses; it does
   not hide the taskbar or change display resolution/scaling. Capture at the
   existing size remains available.
6. After human chart/profile review, use the same launch binding with
   `-Action Close`. It requests normal application shutdown and verifies exit
   code zero. It does not kill a process. Inspect normal profile writes and the
   existing commissioning restore report separately before restoring the
   original output/control configuration. Do not automatically launch afterward.

Ordinary review permits only Capture, Resize1280x800 and Close. No keys, menu navigation,
route activation, plugin controls, radar controls or autopilot commands exist in
this path. Each operation binds the launch request/result hashes, exact PID and
creation tick, executable, current user/session, unchanged target record and
active stock transaction. It rechecks complete plugin trees, source evidence,
quarantine/recovery bytes and input-only navigation configuration. A restored or
changed transaction refuses review. Images go only to a new private directory
for the current SID, administrators and SYSTEM; obscured, partial or changed
window captures are refused. Native pixels require human inspection—successful
capture is not a chart-content or navigation-acceptance assertion.

## First-start navigation caution

The pinned `MyApp::OnInit` calls `ShowNavWarning` when the saved OpenCPN version
changes or the notice has not been shown. A 5.12.2 profile first opened by 5.12.4
therefore presents the upstream **Welcome to OpenCPN** modal before deferred
startup completes. A launch receipt reporting a main window does not prove that
this modal has closed or that charts/data drivers have initialized. Restoring
the earlier commissioning INI can cause the warning to recur.

`InspectWelcome` and `AcknowledgeWelcome` are two separate, source-specific stock
review actions. They do not relax the ordinary enabled-main-frame checks:

1. With the same exact stock launch receipt/hash, run `-Action InspectWelcome`.
   Review the resulting private `welcome.png` and `review.json`. The English or Swedish
   notice must be OpenCPN's GPL/no-warranty and navigation caution, ending in
   **Agree** and **Cancel**. It must not be a chart purchase, licensing, privacy,
   permission or unrelated plugin dialog.
2. Only after visually reviewing the full notice, run `-Action AcknowledgeWelcome
   -WelcomeInspection <that review.json> -ExpectedWelcomeInspectionSha256 <hash>`.
   The inspection must be no older than 30 minutes and match this launch, PID,
   helper revisions and image hash. The helper rechecks the full commissioning
   proof, unique disabled frame/owned modal, exact English or Swedish title/button tuple, native classes,
   button IDs/captions, HTML content surface, complete on-screen bounds/DPI,
   foreground and lack of occlusion. A new capture must hash identically to the
   reviewed image before a durable, exclusive `agree-intent.json` is written.
3. The helper sends exactly one click to that existing Agree button. It has no
   generic key/message/caption/coordinate selector. An uncertain send is never
   retried with the same inspection. Modal dismissal and a fresh normal-frame
   capture are recorded separately; another startup dialog produces an
   attention result, not an automatic dismissal or a chart-acceptance claim.
4. Continue the ordinary Capture/Resize/Close sequence and post-close profile
   review. No configuration flag is edited to bypass the warning.

The warning body is rendered by `wxHtmlWindow`; the helper explicitly records
`bodyTextAccessible: false`. It does **not** pretend that `GetWindowText` proves
the HTML text. Exact executable/source identity, the narrowly matched native
structure and human review of the unchanged pixels form this boundary. Unknown
translations or window structures refuse acknowledgement. Stock-only launch
proof cannot authorize an installed XNav warning; that separate path is not
implemented by this stock helper.

Source boundaries: pinned OpenCPN `gui/src/ocpn_app.cpp` startup version check,
`gui/src/ocpn_frame.cpp::ShowNavWarning`, and
`libs/gui/src/dialog_alert.cpp` / `dialog_base.cpp`. Native IDs follow
[wxWidgets 3.2.8 button creation](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/msw/control.cpp)
and the dialog class follows its
[zeroed Windows dialog template](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/msw/toplevel.cpp).
Actual official-stock Windows fixture evidence remains mandatory before use.

## Verification

`test-stock-review.ps1` exercises the fixed stock identity, empty-argument and
command refusals, expired/changed evidence, exact process creation identity,
native-helper compilation, and a complete disposable stock-only commissioning
fixture. That fixture mutates helpers, retained plugins, quarantine/recovery
bytes, source evidence, applied journals, profile and loader-root context and
requires refusal. Existing installed-verifier tests run unchanged in their
mandatory installed parameter set. The native maintenance jobs include both
suites. Linux portable checks pass: 82 stock groups (70 policy/native-compilation and
12 full-transaction groups), the existing 11 installed-launch groups, and five
portable maintenance groups. These execute no real application. Native Windows
and actual post-uninstall stock launch/capture/normal-close remain separate,
pending gates.

The new `test-stock-welcome.ps1` additionally passes 36 pure policy/interop
checks: changed launch/helper/caption/ID/age evidence, native round-trip metadata,
pixel mismatch before intent, and durable-intent failure before native input.
These checks invoke no Windows API and are not evidence of actual modal handling.
The separate official 5.12.4 native warning fixture is pending; it must capture
the real wx dialog and test the unchanged capture/acknowledgement helper.


## Input-desktop diagnostics on refused warning capture

A foreground refusal retains the exact stock launch/process and modal checks,
then appends a bounded `desktopDiagnostic` JSON object to the error. It reads the
interactive helper's desktop and the current input desktop using
`GetThreadDesktop`, `OpenInputDesktop(0, false, DESKTOP_READOBJECTS)` and
`GetUserObjectInformationW` (`UOI_NAME`, `UOI_IO`). Only the independently opened
input handle is closed. No desktop is activated or switched; access rights,
credentials and acknowledgement behavior are unchanged.

The record contains desktop names, input-state observations, native error codes,
handle-close outcome and numeric foreground HWND/PID. A read-only
`GetGUIThreadInfo` call on that observed foreground thread reports distinct menu,
popup-menu, system-menu and move/size flags, preserving unavailable values as
unknown and recording whether the foreground HWND changed during the query. It does not enumerate
unrelated window titles or capture another application's pixels. Access denied
is reported as unavailable, not as proof that Windows is locked. A disconnected
session can also affect the reported input desktop. Failure of the probe still
refuses the original capture; it cannot change acceptance.

The portable tests cover classification, missing data, escaping and the native
ABI. On Windows the same suite additionally executes only the read-only desktop
queries and checks handle cleanup. This is diagnostic qualification, not proof
that a real boat desktop is unlocked. API contracts: [OpenInputDesktop](https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-openinputdesktop),
[GetUserObjectInformationW](https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-getuserobjectinformationw),
[GetThreadDesktop](https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-getthreaddesktop),
[GetGUIThreadInfo](https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-getguithreadinfo).


The warning focus request now uses `SetForegroundWindow` followed by a bounded
five-second `SendMessageTimeout(WM_NULL)` rendezvous instead of a fixed 250 ms
sleep. The existing exact-modal identity, foreground and occlusion checks still
follow it. The request's boolean result is diagnostic on refusal; it is never
proof of success. A timeout refuses before capture. This follows Microsoft's
[asynchronous activation guidance](https://devblogs.microsoft.com/oldnewthing/20161118-00/?p=94745/)
and adds no click, permission bypass or input-queue attachment. The native
English/Swedish official-stock fixture must qualify this exact revision.


## Separate fixed caption focus

If `InspectWelcome` refuses solely because the verified warning is not the
foreground window after its normal request/rendezvous, `-Action FocusWelcome`
performs one ordinary title-bar click. It uses the same immutable stock launch,
PID/creation tick/session, source/plugin/profile and helper-hash proof. It does
not dismiss another program, hide the taskbar, attach input queues, change focus
policy, switch/unlock desktops, or acknowledge the caution. The installed-product
wrapper has not been extended to this action.

The native helper accepts only PID and exact creation ticks; it does not accept
coordinates, captions, HWNDs, messages or input selectors. It finds the complete
pinned English/Swedish modal, derives the centre of its native title bar, requires
`WindowFromPoint` to identify that same modal and `WM_NCHITTEST` to return
`HTCAPTION`, and repeats all captured fields and geometry before input. Both
helper and input desktops must be the active `Default` desktop. Held mouse
buttons/modifiers, foreground or target mouse capture/menu/move loops, cloaked
targets, covered windows, changed geometry, reused PIDs and ambiguous notices
refuse. Other window titles are never collected. Refusals report bounded numeric
HWND/PID, class, hit-test and DWM cloak observations; a cloak observation does not
relax the existing strict overlap rule.

An exclusive `focus-intent.json` precedes input. One `SendInput` call contains
only absolute movement to that derived point, left-down and left-up. The batch
is not an atomic HWND-targeted operation: a concurrent desktop change can still
invalidate it, and subsequent verification must refuse. Partial delivery never
repeats a press: a positive partial count permits at most one release-only
cleanup, then reports an uncertain result even if cleanup was inserted. A
failed release remains explicitly uncertain; the tool cannot claim a released
button when Windows blocked input. No automatic retry occurs. Successful
submission is followed by a bounded read-only wait for actual activation/release
(sent messages can overtake queued input), the exact-modal rendezvous, PID/start recheck,
unchanged notice, foreground/occlusion checks and released-button observation.

An already foreground, fully verified warning requires no click.

Only then can `focused-warning.png` be captured. Its result says
`acknowledgementSent: false` and **cannot** serve as an acknowledgement inspection.
Run the normal separate `InspectWelcome`, review its pixels and then use its
hash-bound `AcknowledgeWelcome` if appropriate. No `Agree`/`Cancel` event is
part of caption focus.

Local verification now passes 127 warning policy/interop checks and 83 stock
identity/full-transaction checks. These execute no pointer input and are not
native or boat acceptance. The actual official 5.12.4 English/Swedish fixture
must additionally prove the exact caption operation, wrong PID/start refusal,
foreign-overlay refusal and modal retention before separate inspection/agreement.

API boundaries: [GetTitleBarInfo](https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-gettitlebarinfo),
[WM_NCHITTEST](https://learn.microsoft.com/en-us/windows/win32/inputdev/wm-nchittest),
[SendInput](https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-sendinput),
[MOUSEINPUT](https://learn.microsoft.com/en-us/windows/win32/api/winuser/ns-winuser-mouseinput),
[DwmGetWindowAttribute](https://learn.microsoft.com/en-us/windows/win32/api/dwmapi/nf-dwmapi-dwmgetwindowattribute).
