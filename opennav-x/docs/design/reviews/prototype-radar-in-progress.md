# Radar — prototype migration in progress

Authority: unchanged `radarView` and final `.radar-focus` CSS rules, canonical
radar Day/Dusk/Night views. No visual/Windows/boat conformance PASS is claimed.

Reference intent: full 1014×698 workspace at 1280×800; 32px page inset; 30px
heading; a 657×508 display at content (32,144); a 265px control column after a
28px gap. The 428px scope uses the supplied fixed green/black radial gradient,
subtle rings and a separately scrolling control column. The installed page
must not turn the entire workspace into a growing list of desktop controls.

The native `XNavRadarPanel` holds copied adapter status only. Inspection of the
pinned `include/ocpn_plugin.h` finds generic CPU/GL overlay callbacks, not a
standardized radar receive-image or control interface. The current integration
owns `UnavailableRadar`; unchanged [radar boundaries](../../beta-chart-radar-boundaries.md)
still apply. No scanner/image/control path is added by this design work.

Required safety differences from illustrative HTML: NO RADAR IMAGE; no mock
returns, rpm, heading, calibrated range or gain values. Unknown settings display
an em dash and no slider thumb. Radar active, guard zone and sweep controls are
explicitly disabled, including if a copied status advertises capabilities. A
capability bit does not establish an owned image or verified command path.
The existing OpenCPN plugin entry remains available outside replay. Live status
provenance can be inspected, but cannot turn this view into a radar receiver.

Measurements are independently rendered without editing any original bytes.
The supplemental Linux measurement retains the canonical radar screenshot hash.
Its font metrics place the display 9px above the Windows reference; native
geometry uses the Windows target. Additional selectors are retained in future
canonical Windows evidence. Render/compare/correct and all native gates remain
required.

The first component attempt failed during layout. Its systemd core records a
repeating `XNavRadarPanel::Reflow → XNavScroll::Layout → size event` stack.
Removing the child-to-parent layout call eliminates that recursion; no device,
profile or navigation data was involved. The first complete pass then exposed
the scope 12px too high, an undersized radial gradient and incorrect reference
rings. The corrective pass includes the legend's 18px margin/13.5px line box,
CSS farthest-corner gradient radius, original canvas ring fractions and alpha,
centred caption tracking and 470px legend width. Both images/diffs are retained.

The second Linux component pass passes 64 checks and six images. Exact interior
pixels remain unchanged across missing, capability-only, stale, replay and
Day/Dusk/Night states: no status can create returns or fabricated scanner state.
Resize away/back, no whole-page scrolling, independent control-column scrolling,
disabled controls and the explicit advanced plugin callback are exercised.
Actual fixture-free application captures the three themes and Close restoring
navigation. Windows font/layout replacement and boat review are still required.

Full-application pixel review subsequently found that the Night capture's scope
interior was blank although the isolated component passed. This image is kept
as negative evidence. A strict full-application check now requires the same
fixed-palette interior in all themes. Theme changes update the persistent Radar
component in place, preserving its scroll/paint surfaces rather than rebuilding
them. The corrective full-application capture passes exact interior equality in
all three themes; the reviewed Night image now retains the gradient, rings and
unavailable caption. Review also identifies the sidebar's Radar selection using
the wrong page identifier. The replacement corrects that mapping and requires
visible selected ink in each theme. These are development results only.
