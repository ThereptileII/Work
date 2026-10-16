# Route arrival and passage display — SCRUM-373

Source-only change requested on 2026-10-16. No build, test, patch-application
check, installation or boat operation has been performed for this change.
The installed candidate and its acceptance evidence remain unchanged.

## Operator behavior

Route details expose **Passage settings** before the list of planned legs:

- **Waypoint arrival radius (m)**: an optional distance from the waypoint.
  A configured radius is between 2 and 1,852 metres. Leave it empty to retain
  existing OpenCPN waypoint-arrival rules.
- **Passage width (m)**: the full geographic display width, between 2 and
  20,000 metres. The default is 100 metres, meaning 50 metres on each side.

Navigation must be stopped before editing these route settings. Save is an
explicit action; cancellation does not write. Stale selections, protected
routes and recording replay cannot change navigation policy. Invalid saved
preferences remain visible. Bounded malformed records can be repaired while the
route is editable; oversized or unreadable records require profile repair because
the editor cannot bind a bounded exact identity to them.

The active route's passage is displayed by default in XNav. **Chart layers →
Route corridor** can hide or show it; that choice is retained. The width is a
planning/display preference, not a depth survey, hazard-clearance result,
off-track alarm threshold or autopilot steering limit. It does not change the
route geometry. Arrival rings denote the explicitly configured radial policy;
they do not claim that unchanged stock waypoint rules are radial.

The shared software/OpenGL painter draws dashed geographic boundaries before
route/waypoint ink, without a filled area covering chart hazards. It also honors
OpenCPN's navigation-object visibility. Drawing is bounded to 2,048 route points,
the next 128 legs, 64 samples per leg and 8,192 corridor strokes; the current
waypoint's arrival ring uses 256 segments. A limited passage label identifies
the leg/stroke limit when it fits. Invalid/polar geometry (absolute latitude at
least 89.5 degrees) is withheld. Separate legs have no invented corner join.
These are display bounds, not changes to OpenCPN route processing.

## Deliberate opt-in arrival semantics

Pinned OpenCPN 5.12.4 `RoutemanGui::UpdateProgress` normally compares its
distance to the perpendicular arrival line with the waypoint arrival radius.
Depending on its existing global preference, it may also advance after moving
away. This is not a guarantee of coming within a radial distance of a waypoint.

The new explicit route preference implements the user's proximity requirement
using **OpenCPN's existing `CurrentRngToActivePoint`**, calculated by its normal
navigation pass. For configured routes only, it compares that radial range with
the selected radius and withholds the moving-away fallback. It still uses
OpenCPN's normal arrival and advancement functions. No second geodesic model,
navigation loop or UI-triggered route processing is introduced.

Absent preferences retain the upstream comparisons and fallback. Negative/MOB
never-arrive sentinels remain protected. Invalid explicit preferences withhold
automatic arrival instead of falling back silently. Explicit settings apply
consistently to XNav, Legacy and Safe in the same integrated application/profile;
the visual overlay remains XNav-only. There is no new physical command or
SmartNav-to-autopilot path.

## Ownership and persistence

Only small passage preferences are stored by the existing route GUID in the
shared OpenCPN profile. OpenCPN still owns route identities, geometry, points
and progress. Waypoint radii are not overwritten: a shared waypoint in another
route keeps its previous behavior. The unused upstream route arrival field and
screen-pixel route stroke width are not treated as operational settings.

The store validates finite values and safe keys, reports persistence failures,
and restores prior values on failed writes. A failed rollback latches storage as
unavailable. Owned catalog revisions and route
progress copies include the policy, so a change cannot retain old guidance as
current. Malformed records also carry a bounded exact storage identity so a
repair cannot overwrite an intervening different record. No raw upstream pointer
is stored in a consumer snapshot.

These preferences are **profile metadata**. GPX export/import does not carry
them, and the existing limited SKAGER settings-export format does not include
them. A full profile backup retains them. A separate stock OpenCPN installation
does not implement the optional strict-arrival policy; removing SKAGER restores
stock waypoint behavior without rewriting waypoint radii.

## Pending qualification

Implementation boundaries: `application/RoutePassage.h` owns the copied policy;
`integration/RoutePassageSettings.cpp` owns profile storage and native policy
resolution; `NavigationObjects.cpp` resolves exact inactive-route edits;
`ActiveRoutePassageOverlay.cpp` owns bounded display geometry. The product route
sheet and chart-layer switch consume those boundaries without retaining native
route/waypoint pointers. The existing OpenCPN integration CMake list links the
new store for both the application and the model-only upstream test fixture.

Independent source review corrected a stale-selection gap for different malformed
records before integration. Test source extends policy validation, route revision
invalidation and replay-control refusal. The native fixture now uses an isolated
memory configuration and adds five policy/storage cases: stock/explicit route
policy with shared points, MOB/layer protection, revision changes, corrupt/oversize
records, and injected flush/rollback failure. These model cases do not exercise
the GUI arrival loop or prove filesystem durability. None has been run.

Source review is not execution evidence. Before delivery, qualify numeric and
corrupt-record handling, failed persistence, shared-point isolation, strict vs
stock arrival/overshoot, MOB sentinels, active-route/replay refusal, revision
invalidation and restart/mode persistence. Exercise both native chart render
paths with active/deactivated routes, different widths and zoom, chart rotation,
dateline crossings and invalid geometry. Confirm chart symbols remain readable
and that width labels/rings agree with configured geographic distances. Native
Windows and boat acceptance remain pending; no screenshot or physical-control
pass is claimed here.
