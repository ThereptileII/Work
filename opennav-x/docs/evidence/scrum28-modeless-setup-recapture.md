# Modeless setup recapture guard — SCRUM-28 / SCRUM-313

This product change is queued for the next product-code batch. It does not change
or qualify the retained `0db45cb92509c29ef4d1fc33d0c76dc7cf511b97` candidate.

The pinned OpenCPN canvas-resize/fullscreen path schedules a one-shot recapture
callback after one second. The patched callback skips `Raise()` when
`Shell::HasTransientSurface()` is true. Boat Setup is an owned modeless dialog
(`Show()`, not `ShowModal()`), so the existing modal-dialog scan did not protect
it. The guard now also recognizes the visible `boat_setup_` weak reference;
hidden or destroyed setup dialogs do not suppress recapture. Existing drawer,
card, modal-dialog and Legacy branches are unchanged.

In pinned wxMSW 3.2.8, `wxTopLevelWindowMSW::Raise()` calls
`SetForegroundWindow()`. If that activation removes a pressed setup button's
focus before release, its existing focus-loss handler clears the pressed state.
This is a conditional source-level cancellation path, **not an established cause
of the native5 failure**. The retained successful pointer traces do not establish
that failed interaction's cause. Button cancellation and native smoke assertions
remain unchanged.

Focused validation uses `tools/verify-frame-recapture.py --skip-lifecycle`, with
the actual extracted production recapture callback, transient guard and restack
method, plus an owned modeless wxDialog in the existing native GTK component
fixture. All **41 checks passed**, including visible/hidden/destroyed setup,
Legacy recapture and existing overlay behavior. Removing only the new guard
caused the fixture to fail at `visible modeless setup suppresses recapture`.
These are Linux/Xvfb component results, not Windows, full-application, package or
physical-boat qualification. No application build or CI was run for this change.

## Native Windows red/green component gate

The subsequently retained native6 trace reports button down at `1731220`, focus
at `1731221`, blur while pressed at `1731251`, and release at `1731270` with
pressed/activation both false. The observed foreground changes from Boat Setup
to the main frame while the cursor stays over Later. This proves cancellation
following focus loss; the old trace does not directly identify the recapture
callback as the activation caller.

`tools/test-frame-recapture-windows.py` builds only the isolated component in
`tests/frame_recapture_windows`, using the locked wxMSW 3.2.8 SDK, MSVC Win32 and
app-local x86 runtime through the existing Windows widget-test helper. It
compiles complete production `Controls.cpp` and `VesselState.cpp`, and extracts
the actual patched recapture callback, integration guard and Shell transient
predicate. The original variant removes only the exact new setup-guard line.
No full Shell, OpenCPN, application, package, profile or transport is built/run.

Each variant creates a real owned modeless wxDialog with a production XNavButton,
positions the native cursor over it, sends one native left-down, verifies button
focus and capture, invokes the extracted callback while pressed, then sends one
native left-up. Original must actually raise the owner, lose button focus and
emit no button command; guarded must preserve focus and emit exactly one queued
button command. Unexpected focus, input, timeout or witness states fail. There
are no retries or weakened button focus-loss semantics. Both expected witnesses
are required for the helper to pass; a zero exit alone is insufficient.

Disposable `windows-2022` GitHub-hosted runner recipe (checkout this exact helper
revision with `src/`, `tests/frame_recapture_windows/`,
`patches/opencpn-5.12.4-xnav.patch`, `tools/test-frame-recapture-windows.py`,
`tools/test-windows-changed-units.py`, `tools/windows-wx.lock.json`):

```powershell
python opennav-x/tools/test-frame-recapture-windows.py --evidence opennav-x/evidence/local/frame-recapture-windows
```

Always retain the entire evidence directory, including compile logs, extracted
methods, each interaction log and `summary.json`, even on failure. The helper
refuses non-native or non-disposable execution. Its SDK downloads are pinned
inputs, not application networking. Three portable extraction checks passed;
Native execution subsequently **passed** in
[run 37443443071](https://github.com/ThereptileII/Work/actions/runs/37443443071),
commit `dcab13cb86c91950de753d11777ea6feacc80f7b`, native job `112202461503`.
Downloaded artifact `11401992443` matches SHA-256
`31601d4c4af6a5ea915feb62e6e29f382f15d35f1d57f3b9aa0e8d096ae16f0b`.
Original reports one native down/up, one focus loss and zero activations;
guarded reports one native down/up, zero focus losses during the interaction,
and one queued activation. The final dialog-destruction blur happens after
the guarded PASS witness. The replacement source comparison accounts for
Windows checkout line endings and the intentional `.2` → `.3` version header.
This proves the isolated callback behavior, not full application scheduling
or package acceptance; those remain in the replacement Staging gate.
