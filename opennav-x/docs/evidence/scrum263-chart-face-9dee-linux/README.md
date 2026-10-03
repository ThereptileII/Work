# SCRUM-263 ordinary font correction — 9dee Linux proof

Exact source: `9dee9b148f4d6ebdd20bb4c49229fe19340df209`.
Installed ELF SHA-256:
`274ec23155a2fc77ca47022c3b937765da41c8f3cf757aa5dba73fd2440f23d0`.
Chart manifest SHA-256:
`12cfbce4686fce9a89e15f45105195c9f09a51ae4e3e9932aa7ba803ae205a52`.
The resource files remain unchanged from ccc0. All declared hashes, the clean
source revision and executable were independently rechecked after capture.

The preceding ccc0 installation was independently inventoried/copied into a
read-only private snapshot, then reverified after capture. Both full nine-patch
trees were reconstructed; only the changed s52plib.cpp/h upstream files were
advanced. One incremental build/install completed **65 steps**. The existing
suite ran once: **147/147 passed in 22.30 seconds**. No resource/geometry suite,
coast/lateral scene, full theme cycle or application rebuild was repeated.

The first font-component launch failed before assertions because the desktop's
`GDK_BACKEND=wayland` and `WAYLAND_DISPLAY=wayland-1` were inherited by
`xvfb-run`. The raw `build.exit` therefore remains **2**, recording the combined
build-plus-probe shell's failure after its successful build/install. Its exact
GTK initialization error is retained in `font-probe-initial.log`. One corrected
component retry used dedicated Xvfb249, `GDK_BACKEND=x11` and removed
WAYLAND_DISPLAY. It passed, followed by the once-only suite; `checks.exit` is0.
No application or test assertion changed. The actual executed scripts and logs
are retained, including the failing launcher rather than replacing its result.

The updated actual `ui_font_resolution_test` reports all three Windows UI stack
families absent on this Linux host, system face Adwaita Sans, and the ordinary
chart policy explicitly retaining the original template because Segoe UI/Arial
are unavailable. Its source/executable hashes are in `font-probe-receipt.json`.
This is positive evidence for the unavailable-family fallback. It is not evidence
of Windows GDI substitution, actual boat font selection, or the installed-family
positive branch.

Four fresh isolated application sessions captured the unchanged real NOAA Pier57
viewport in SKAGER and Standard, software and actual Mesa llvmpipe OpenGL. All
four exited normally. The existing source/ELF/resource, runtime capability,
NOAA bytes/quilt/viewport, measured fresh simulated loopback input, renderer,
style, symbol preference, palette, wordmark and edge guards remain intact.
Original PNGs, diagnostics and comparison receipts are under `pier57/`.

**Every entire Day chart rectangle `(80,68)–(1094,634)` is pixel-identical to its
exact same-renderer ccc0 original, with no masks or tolerance.** The identity
panel is also exact. Software and GL SKAGER originals were additionally viewed
at native size. Clock/input ages outside the declared chart/identity regions
are retained in originals but are not historical equality claims. No unexpected
chart difference or capture retry occurred. The prior ccc0 16-image theme proof
remains a distinct source receipt; this increment captures Day only.

No private o-charts adapter, Windows application, physical GPU, boat, real sensor
or hardware-control path was exercised. Later land-family corrections are not
included in this exact9dee record. Windows and physical display acceptance remain
open. No CI was dispatched or root worktree edited.
