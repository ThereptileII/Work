# Notification centre — prototype migration in progress

Authority: unchanged HTML `case 'alerts'`, final `.callout` / `.row` cascade,
canonical Day/Dusk/Night references. Windows and boat conformance remain pending.

Reference intent: NOTIFICATION CENTRE / A watchful eye in the 398px owned drawer;
real conditions presented as compact semantic callouts; source inspection and
acknowledgement beside the chart; four severity meanings and a persistence note.
No illustrative crossing vessel is copied into the installed product.

The old full-page alert list is replaced by `XNavAlertDrawer`, using the shared
native drawer, callout painter, buttons and scroll gestures. All shell and
advanced page entry points lead to the same surface. The chart remains in its
original pane. The global critical strip remains visible. Recovery restores
the top-bar caption to Alerts rather than leaving an obsolete count.

The view owns copies of existing `AlertCenter` episodes. It does not calculate
new CPA/risk, trigger upstream processing, clear an OpenCPN alarm or command any
adapter. Source inspection is a navigation callback. Acknowledgement carries
the exact id/episode displayed when tapped; repeated queued taps are suppressed.
Recovery/recurrence before dispatch cannot acknowledge a different episode.
Dismissal never acknowledges. Historical replay is labelled and cannot invoke
live source inspection or acknowledgement.

Safety-required differences from mock content: measured real alert text;
critical red instead of treating every condition as amber; an honest empty
state; no fabricated AIS encounter. Callout height accommodates full condition
text and 48px actions. Acknowledged conditions remain visible until resolved.

Validation in progress: dedicated non-installed component tests cover themes,
critical persistence, repeated taps, inspect destination, recovery, recurrence,
a delayed acknowledgement, replay and dismissal. The retained fixture journey
continues to verify GPS loss/recovery and episode-specific acknowledgement.
No native or physical PASS is inferred from compilation or Linux screenshots.

First local review: 132 integrated tests and 53 component checks pass, with
seven component states and 30 actual-product captures. The component comparison
exposes native grey backing at the action-button corners. The correction sets
the owned panel backing to the same exact theme/alpha blend as its callout and
adds a pixel assertion in all three themes. Replacement evidence is required;
the first render is retained, not passed as visually complete.

The second application pass stops on Alerts Close: the retained native image
shows the sheet still visible and Close hovered, with live diagnostic ticks.
The X11 harness sent an immediate down/up pair; GTK activation timing is a
possible cause, not a confirmed product diagnosis. Its replacement uses one
50ms held press, matching the existing Windows
harness, without retries or loosening the required closed-sheet assertion.
The failed run is retained; replacement execution is still required.

The replacement application pass completes with 26 primary captures plus four
AIS-settings captures after explicitly awaiting Back's semantic AIS-list
publication. The retained fixture suite reveals that AIS may precede GPS in
severity order. The harness now locates the GPS action by its readable
accessibility name, scrolling the drawer if needed, and still asserts that no
other episode is acknowledged. It does not assume the first button is GPS.

The corrected full fixture journey passes eight check groups / 44 screenshots
and 132 integrated tests, including acknowledgement amid other critical
conditions, new episodes after recovery, and repeated XNav/Legacy/Safe returns
with deterministic coastline and shared-data preservation. Local evidence is
retained in `docs/evidence/prototype-alerts-local.json`. Native replacement and
boat review are mandatory; no conformance category is marked PASS.
