# Passage — native drawer migration in progress

## Reference and boundaries

The unchanged HTML `routePanel()` defines a 398px sheet at (682,80), height674
in the 1280×800 Windows client. It retains the chart and horizon behind it.
Content begins at (705,190): pills, four stats, section label, numbered route
points, callout and actions. Windows point rows are 77px. The reference's
fictional distances, times, battery prediction and editable active points are
not navigation authority. The canonical renderer now records the route-row
selectors explicitly in addition to its existing reference images.

Inspected integration: `NavigationObjects.cpp::Copy(Route*)`, `Resolve`,
`StopRoute`, and the accepted `RouteProgressInput`, `AssessRoute`, `Advise`
and energy wrapper. No new upstream hook or route processing call is added.
Route distance is copied, cumulative leg/timing/turn values come from existing
SmartNav events, and arrival SOC requires the exact current immutable energy
input-route publication. Deletion, stale/future observations and mismatched
revision/observation predictions are withheld. UI never keeps upstream pointers.

## First review and correction

The former full-page Passage view removed the chart and used large destination
cards. The new native drawer preserves the original chart parent and geometry.
Linux product capture verifies the drawer bounds and painted background; the
fixture-free product shows unavailable values with no active route. The
non-installed component process separately covers a four-point passage, all
three themes, stale position and route deletion. It cannot load a profile,
network or marine equipment.

The first component capture failed its theme-pixel assertion: GTK ScreenDC
returned cached Day pixels. Its negative images/log are retained. The capture
uses the existing AIS test's GDK root-window method; the replacement passes
26 checks and five actual-screen captures. This changes the capture mechanism,
not the acceptance pixels. The second visual refinement adds the prototype's
active-pill tint, original edit icon and explicit disabled editing on protected
active-route points. The clock is injectable only through the native component
API so offline fixture images have deterministic arrival times; product calls
use the real local clock. A further capture exposed grey native backgrounds
behind disabled edit icons; the summary panel now exposes its actual theme
background to child controls, with a strict pixel regression check.

## Native Windows development check

Remote `6a75c056e8cfb74c25361726682a539eac8183e1` has the independently
verified tree of local `d205baf9b42dcc4a619e32751ce3b7d71513f288`.
Run 36491018722 passes all seven jobs: 113 integrated native test cases,
26 Passage component checks and the existing actual-object workflow. All eight
artifact hashes/CRCs and 48 recorded native screenshot hashes verify. The
populated Day drawer and product Night unavailable state were reviewed against
the Windows HTML. Stats and 77px rows align; missing route data stays explicit;
Close retains the 1014×566 chart. The source commit also passes all 121 Linux
tests (19.52 seconds). See [native evidence](../../evidence/prototype-native-6a75c05.json).

## Still open

Further native Windows refinement, first and corrective boat captures, exact font
tracking/line heights/shadow, contextual waypoint sheet, saved-passage library
migration and full route-creation/editor conformance remain open. No screen is
accepted. Active navigation cannot be reversed or edited through mock HTML
behavior; those actions remain guarded by the established OpenCPN rules.
The library retains access to existing Alpha/Beta route workflows meanwhile.
