# Native Windows Course-up failure — candidate 12100a74

Exact failing commit: `12100a74ff619b7268a6e20902bd9d0de3b53b39`.
[Run 36279275114 / native job 108508347077](https://github.com/ThereptileII/Work/actions/runs/36279275114/job/108508347077).
This is failure analysis and a local replacement check, not native acceptance.

The failure artifact SHA-256 is
`06823aa8f2ae24684802ae7f92f6e8f04ec9819601e7e708097d61c24a7e7538`.
Its downloaded bytes match both the GitHub artifact digest and the completed
native job's artifact-upload log.
The ignored copy is under `evidence/local/boat-beta2/windows-12100-user-flows/`.

The actual frame is 1280×800 on a 1920×1080 disposable desktop. Reviewed
`flow-failure.png`: Course is visibly selected, all four rail values and bottom
controls fit, and the Windows taskbar does not cover the frame. This closes the
earlier pointer-workspace obstruction, but does not qualify the stopped data.
The software chart shows an island; no blank/all-water success is inferred.

The isolated source sent 113 GPS batches without transport errors. The last
published snapshot still reported North and tick 13. The bounded native trace
continued through timer tick 17, then recorded `orientation.begin`, upstream
return, a complete update through `tick.end` at 18, and `orientation.end`.
There were no later timer entries. The trace is only 9,812 bytes, below its
event cap. The native timeout probe answered `WM_NULL`; the main window was
foreground, with no mouse capture, menu loop or move/size loop. This is not a
blocked orientation callback or a partial diagnostics write.

Pinned source inspection found a repeatable invalidation cycle in software
`ChartCanvas::OnPaint`: its basemap block changes the live rotation to skew,
renders, then restores rotation. `SetVPRotation` enters `SetViewPoint`, and
`Quilt::IsQuiltDelta` explicitly compares rotation, causing recomposition and
`Refresh(false)` during paint. Windows processes `WM_PAINT` before `WM_TIMER`;
the source cycle explains the otherwise responsive window with starved timers.
[Microsoft message ordering](https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-getmessage).
The inspected wxWidgets 3.2.8 timer implementation allocates separate native
timer IDs; the evidence does not support an ID collision or a stopped timer.

The replacement changes only that drawing block to use an owned viewport copy,
with the same physical size, projection, center, scale and temporary skew
rotation. The existing basemap renderer and pinned `SetBoxes` still determine
the drawing geometry. No canvas setter, quilt recomposition or Refresh is
called to prepare this temporary rendering state.

Local replacement validation: reviewed patch verification and fixture-enabled
and fixture-free integrated Linux builds pass; actual pointer workflow passes seven groups with
nine screenshots and 233 GPS batches. Course-up advances from tick 16 to 20
and receives a newer LIVE GPS observation before returning North. Reviewed
`flow-00-course-up-linux.png` shows coastlines and the complete rail; Linux
rendering does not establish Windows acceptance. The replacement must still
pass the unchanged native functional assertion and native rotated-chart review.

The same native run also failed the separate preview script while locating
`+1°`. Its saved UTF-8 diagnostic report and native window inventory both show
that exact enabled, visible label with fresh AUTO feedback. The script decoded
the report using Windows' locale-default encoding, producing `+1Â°` internally;
the exact-label lookup therefore found no control. It now uses the existing
UTF-8 snapshot reader. The reader regression includes degree, minus and Swedish
text and proves that CP1252 decoding changes otherwise valid JSON identities.
No pilot enablement, feedback requirements or command behavior changed. The
replacement preview workflow still requires a native rerun.
