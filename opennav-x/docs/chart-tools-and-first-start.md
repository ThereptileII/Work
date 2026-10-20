# Chart tools and first-start presentation

Initial source-only feedback increment, 2026-10-17: SCRUM-375 and the first-start
portion of SCRUM-341. No execution was performed at that initial source-only
checkpoint. Subsequent integrated and installed Beta 2.6 evidence is recorded in
[status](status.md); it does not qualify the 20 October follow-up below.

## Chart tools

OpenCPN still owns measurement points, bearings, distances, units and rendering
positions. `RouteLegInfo` gets an optional chart-tool style from
`ChartPresentation`: SKAGER typeface, themed rounded surface and text. The
existing computed strings and placement formulas are retained. The native
`ocpnDC` primitive serves software and OpenGL. The 20 October follow-up extends
the same label to route creation; Legacy/Safe retain their stock path.
XNav's Standard chart style still uses native SKAGER tool presentation.

Route and Measure placement now request a cached cyan precision reticle with a
dark keyline instead of OpenCPN's pencil. The click hotspot stays at its center.
The native `ChartCanvas::SetCursor` boundary covers activation, placement and
movement, forwarding to either the software canvas or GL child. It substitutes
only the native pencil in XNav, without an active plugin cursor. Other cursors,
Legacy/Safe, point coordinates and route/measurement edge scrolling are unchanged.
The reticle follows the display preference within native Windows cursor capacity;
an odd image extent below the system dimensions lets wxMSW pad artwork and hotspot
together rather than resize them. A native crosshair is the fallback.

The combined 21-patch stack applies and reconstructs against pinned OpenCPN.
Actual canvas input/cursor methods pass 261 focused checks with `OPENNAV_X` and
261 without it; stock methods reproduce 20 expected edge/pencil regressions.
These inert-sink checks cover mode, tool, plugin, fallback and GL-child dispatch,
not native cursor rasterization or visual acceptance. The follow-up has no new
application build, Windows/CI result or boat installation.

The shell reads copied native tool state on the application thread. It never
infers successful activation from pressing Measure. The fixed horizon area
temporarily shows instructions and a Cancel/Done action, without changing chart
or instrument-rail height. Instructions change from choosing the first
measurement point to choosing the next. Closing a tool clears its temporary
measurement; it does not create a route.

The map's Waypoint action arms a single placement. It no longer substitutes
the viewport center. A classified mouse/touch chart tap supplies copied
coordinates to the existing named-waypoint Save/Cancel sheet. Panning is not
placement. The existing integration owns naming suggestions, validation and
OpenCPN persistence. Replay/simulation cannot create real marks. Escape,
Cancel, page/canvas changes and rearming invalidate pending placement work;
queued work checks its generation and weak owner before opening a sheet.
OpenCPN route creation and object editing are not simultaneous placement modes.

## First start

The frame starts with the SKAGER title in XNav. A native themed safety notice
replaces the positively identified `ShowNavWarning` presentation only. It
retains the complete pinned OpenCPN warranty, GPL, paper-chart and
life/property warning. Agree and Cancel are explicit; Escape and window close
reject. No acceptance flag is written by the UI. The original first-run/version
trigger, persistence and SCRUM-312 updater waiting/acceptance receipts remain
owned by the existing startup call site. Modal waiting is not healthy startup.

The notice uses English consistently with SKAGER's existing UI, avoiding the
old translated instruction saying OK while its action says Agree. It uses a
dark themed surface, a scrollable full notice and separately visible actions.
Localized SKAGER notice variants are not supplied by this increment.
The startup notice currently uses the dark SKAGER Day palette, not the profile's
persisted Dusk/Night selection; that startup-only theme limitation remains open.

Automatic stock first-use wizard launch is deferred in XNav to the existing
SKAGER Boat Setup. This does **not** mark either setup complete or write chart,
connection or unit defaults. Boat Setup is not a full replacement for the stock
wizard: chart directories/coverage and navigation preferences remain available
under Settings → Navigation; source configuration under Settings → Sensors →
Add a sensor; the full editor under System → Advanced / Legacy Settings.
An explicit `--config_wizard`, Legacy and Safe keep the stock wizard/notice.
Plugin, recovery and error dialogs are not blanket-suppressed.

Windows Search/jump-list identity is outside this scoped SCRUM-341 correction.

## Native qualification scenarios

Preserve these checks when qualifying a candidate. The earlier Beta 2.6 results
in the delivery evidence apply only to its exact bytes; the new route label and
reticle still need native Windows qualification in a subsequent candidate:

1. Apply the maintained patch series to the pinned source. Check the three new
   patch inventories and corresponding-source inclusion. The changed dependency
   receipt helper also requires reviewed exact consumer-hash policy refresh
   before SDK reuse; do not bypass its existing fail-closed policy.
2. On native Windows, exercise Measure first/next point, Done, Escape and F4,
   and route first/next point, Finish and Cancel;
   confirm original units/bearings, reverse first-leg text and rounded
   Day/Dusk/Night labels under software and OpenGL. Check Standard chart style
   and Legacy/Safe as separate paths. Confirm the reticle click hotspot at
   100/125/150% Windows scaling, restoration on exit and plugin cursor precedence.
3. Exercise Waypoint tap → naming → Save/Cancel on blank chart and over existing
   marks/routes/AIS, using mouse and touch. Verify stored coordinates equal the
   selected chart position, no existing object moves, dragging pans without
   placing, double click does not create two marks, and stale/pre-arm/rearmed
   delayed releases cannot save. Include invalid coordinates, replay entry,
   drawer/page/canvas replacement and close while a sheet is pending.
4. Check tool instructions/actions fit at the supported 1280×800 display/DPI
   settings; alerts and data rail retain their geometry. Route creation keeps
   its own Finish/Undo/Cancel area and cannot accidentally create a mark.
5. Using an isolated first-run/version-change profile, check SKAGER's title,
   full safety text, Agree, Cancel, Escape and window close. Verify updater
   pending-human-ack, rejection/timeout and accepted normal startup independently.
   Verify explicit stock wizard, Legacy/Safe and settings access, without
   accepting consent automatically or editing acceptance profile keys.

This list is not completed evidence. No physical hardware command is needed.
