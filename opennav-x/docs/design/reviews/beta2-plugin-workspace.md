# Beta 2 plugin workspace restoration

Reference intent: chart-first XNav with preserved OpenCPN plugin functionality;
Legacy remains the reference for plugin-owned desktop interfaces. The redesign
must not silently discard a user's saved plugin window layout.

Source review found that XNav's temporary panes made OpenCPN reject the entire
saved perspective during deferred startup and locale reload. XNav removes these
panes before saving, so they can never satisfy the upstream completeness check
at the next start. Ignoring their names alone would also be wrong: wxAUI hides
every current pane before it loads the persisted ones.

The repair distinguishes temporary panes by actual owned window pointers,
preserves their pane infos around normal upstream loading, and requires the
same manager instance. It keeps an already-open product page above its hidden
chart when settings/locale work restores a workspace. It does not parse or
rewrite plugin layouts or relax validation of other panes.

The expanded disconnected chart test loads the actual Dashboard plugin with a
visible floating instrument at nondefault position 820,120, size 220×220, and
dock proportion 73121. These values are distinct from Dashboard defaults. Five
checks in each software/OpenGL cycle inspect initial native AUI state, saved
state after XNav close, saved state after Legacy close, returned XNav state and
final saved state. Temporary XNav pane names must not be persisted. All four
primary rail values must remain visible. Native plugin-window captures are
separate from copied pane-state assertions.

Local fixture and fixture-free Linux integrated builds pass. The object suite
passes 22 groups, including a foreign manager whose pane is named `OpenNavTop`:
the name grants no XNav ownership and its normal perspective restore works.
The final extended software/OpenGL chart run passes all five workspace checks
per mode, retains existing chart/data/persistence gates, and captures 22 images.
This includes the real Dashboard native window at startup and after returning
from Legacy in both rendering modes. Its single SOG instrument receives the
isolated loopback GPS fixture. The Menu overlay is opened and dismissed before
the persistence checkpoints; no plugin visibility is lost. Reviewed Linux
startup, Legacy and returned-XNav chart screenshots retain detailed ENC content
and normal chrome. Evidence: `evidence/local/charts-results.json`,
`chart-{software,opengl}-dashboard-{startup,returned}-linux.png` and
`evidence/local/objects-input-profile/objects-fixture-results.json`. A bare Xvfb has no window manager to keep floating plugin
windows above the main frame, so a separate fixture capture raises the verified
plugin window; this is not native Windows stacking acceptance.

Native MSVC, actual Windows floating-window review, and the real boat plugin
workspace remain unaccepted until the exact replacement passes those gates.
