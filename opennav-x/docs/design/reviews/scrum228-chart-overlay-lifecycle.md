# SCRUM-228: restore explicitly hidden GTK chart controls

The confirmed defect is native visibility diverging from wx's cached state,
not stale chart-layout calculation. No chart geometry, action, viewport or
navigation semantics change in this correction. Windows code is unchanged.

## Failure evidence and cause

Fresh source `74761400862bc1d25102b2b741d14247dd343cdc` reproduced invisible
floating controls after the Linux startup 896x560 → 1280x800 resize. The
three-input observer sent the advertised zoom centers to the underlying
canvas, changing its center but leaving scale unchanged. It did not reproduce
the older c95 long-run scale drift. See the separate SCRUM-224 observation.

Diagnostic-only source `671578e925fe663bd7c4fada6755aca7919ab732`, based on
`138ebb5`, reproduced the same failure with bounded fixture tracing. Executable
SHA256 was `6f43f9d5353c481acf53f187b98632551db7545caee448cb4241b9b1c138921a`.
The retained 627-record trace is below the existing 2048-record limit:
`soak-viewport-observation/evidence/local/viewport-diagnostic-671578e/`.

- Initial native surfaces map at compact-layout positions.
- While the owner is disabled, Shell correctly hides its chart surfaces.
- Subsequent ticks read the correct (80,68,1014,566) chart and request tools
  (883,545), compass (1004,90), Layers (1016,190), Follow (108,553).
- Cached wx positions update, but native origins remain at compact positions;
  native mapped state is false while wx `IsShown()` is true. This persists.

[wxGTK 3.2.11's map callback](https://github.com/wxWidgets/wxWidgets/blob/v3.2.11/src/gtk/toplevel.cpp)
can set `wxWindowBase::Show(true)` after a queued map event. Its
`ShowWithoutActivating()` checks only cached `m_isShown`, so an intervening
explicit hide can leave future presentation unable to remap the window.
A small real-X11 probe reproduced exactly that Show/Hide/event-loop ordering.
An unmapped-only check would be unsafe: legitimate initial deferred showing
also has a temporary logical/native mismatch.

## Bounded correction

On GTK only, the floating surface remembers explicit `Show(bool)` requests.
A late native map callback bypasses this override, preserving the hide intent.
`Present` reconciles only the combination of explicit hide intent, cached shown
state and a native widget whose visible flag is false. It resets wx's shown
state, uses the existing nonactivating show, reapplies shape, then moves to the
fresh target. Remapping must precede moving because GTK restores the old
native origin during re-show. Normal deferred showing remains untouched.

Existing owner-relative restacking, Shell owner-enabled/iconized/visibility
checks, drawer overlap rules, input callbacks and all size constants remain
unchanged. No raw GTK map/show, global Raise, timer retry or canvas reparenting
is introduced. The temporary diagnostic patch is not in the production fix.

## Focused regression

`floating_surface_test` is an offline native-window test registered beside the
existing component targets. It creates controls at 896x560, presents then
hides them while disabling the owner before the queued map is handled,
resizes to 1280x800, enables the owner and presents at the new target. It checks
native mapping/origin, reported bounds, preserved owner focus, and an actual
native pointer click reaching the button exactly once rather than the canvas.
It also checks an ordinary hide/show cycle after returning native focus to the
owner. No chart model, application profile, network or hardware is involved.

Direct focused compilation uses only this test, `FloatingSurface.cpp`, local
wxGTK 3.2.11 and GTK libraries. Under Xvfb with `GDK_BACKEND=x11`:

- Original `138ebb5` implementation fails at native remapping after observing
  cached shown=true and native visible=false.
- Corrected implementation passes **17 checks**, including native position
  (980,549), pointer recipient and focus retention across both restore paths.

Local failure/positive records are retained in
`evidence/local/floating-lifecycle/`. The first candidate remapped but failed
exact native-origin checking; that finding determined the final ordering.
The initial ordinary-cycle focus check lacked a focused-owner precondition;
the final test explicitly returns native focus and verifies it before showing.
The Linux integration workflow explicitly executes this target after its
normal build in a private 1280x800 Xvfb display with `GDK_BACKEND=x11`, a
30-second limit and a retained log. This is a separate process step, not a
new CTest registration or reported CTest count.
Integrated clean-binary observation, native Windows and release qualification
remain separate gates. No full build, CI dispatch or boat action occurred here.

## Clean integrated replay

One three-input replay against clean source
`a3edcaef2904bf1450e306f3e086164479ec7f6f` completed with observer status
**observed**, action duration 3.688 seconds, clean exit and unchanged disposable
navigation fixtures. The executable was 25,721,672 bytes, SHA256
`cb195d7356f5891aa2c4831b64f3bc8f33f0fe9e4fc4670c696200858b404985`, and
reported that exact source identity. The separate observer was commit
`399fe5775d3e2d88833e52787b4769095400da27`, unchanged script SHA256
`7cf8e122462d3ea5ec2d360fea1995ced41937e80e583df064a28dcba2aa5773`.
No temporary overlay instrumentation is present in this product build.

Evidence: `soak-viewport-observation/evidence/local/viewport-clean-a3edcae/`.
Before the first input, all floating controls are visibly painted at their
full-size positions. The raw input records confirm the actual recipients:

| Input | Raw receiving button bounds | Actual result |
| --- | --- | --- |
| `+` at (1002,571) | (980,549,44,44) | Button activation; scale 0.003000000026 → 0.006000000052 |
| `−` at (1046,571) | (1024,549,44,44) | Button activation; scale 0.006000000052 → 0.003000000026 |
| Day at (1201,34) | (1179,12,44,44) | Button activation; Day → Dusk |

Center remains 59.08,18.5 and follow stays false throughout. The exact existing
seven-control `navigation_layout` check passes on the final diagnostic, and
the existing coastline/palette checks pass on all three before-input images
and the final Dusk image. `geometry-and-coastline-check.json` retains these
results. Final controls remain visible. The observed inverse zoom pair is
not generalized into a new exact-net-zero navigation requirement.

This passes the local prerequisite for root's separately authorized clean
full preview. It is not endurance, native Windows, CI or boat acceptance.
