# FFE Preferences selector correction — SCRUM-224

Selected by Jira comment 10521. Isolated base `f220b31391815663c442d1f8dcbe98a54c03333c`;
the frozen candidate remains unchanged. This changes the smoke driver, not product UI.

The retained FFE Linux artifact `11243630817` (SHA-256
`1bcc1b764aa33352995fc95c44d7a29548dcc38fabae08724fe8ec86e47f0153`)
contains `preview-logs/opennav-diagnostics.json`. At the failure it reports
Settings drawer `(648,80,432,674)` and two enabled Diagnostics controls:

- Hidden previous ProductPanel action `(591,261,487,52)`.
- Visible Settings action `(671,664,386,72)`.

The previous selector asserted that the label occurred exactly once before
checking visibility. The correction requires one visible, enabled, fully
contained drawer action first. If none is usable, one enabled, horizontally
contained candidate can guide a bounded scroll; it cannot be clicked until
visible and fully contained. The existing `shell_click` performs its own fresh
uniqueness/containment checks and real pointer input. Duplicate visible actions
still fail. Section settling also excludes the out-of-drawer hidden row.

The diagnostic schema contains no owner HWND or parent path. Horizontal
containment is therefore used only for scroll selection, not asserted as proof
of widget ownership or permission to click hidden controls.

Five focused regression checks pass: exact retained collision, duplicate visible
rejection, below-fold selection for scrolling, disabled-action rejection and
clipped-control rejection. The selector also passes directly against the full
retained snapshot, choosing `(671,664,386,72)`. Python parsing and diff checks pass.

The related caller audit found no additional occurrence in their current paths:
Linux mode-cycle reveals only Interface & recovery (one row in this snapshot);
recording filters visible/enabled controls before uniqueness; Windows DPI scopes
native discovery to Preferences and requests Advanced battery model / Interface
& recovery. The generic DPI publication barrier would reject duplicate labels
if later reused for Diagnostics, but current calls do not do that. No unrelated
caller or assertion was changed.

Five available installed Linux binaries in the primary, security-integration,
scrum212-integration, skager-product-integration and waypoint-touch-regression
build trees lack both the new System intro and unavailable-installer strings.
No equivalent integrated executable was available for this exact interaction.
Live preview re-execution, any next real failure, and native qualification remain
pending. No full application build, suite, CI dispatch or boat operation ran.

## Native Display route follow-up

Jira comment 10523 selects the second concrete obsolete-menu correction found
while auditing the remainder of preview. Both native Display entry points still
requested Settings → Display → Chart presentation. Display no longer contains
that row: Navigation opens the separate Chart presentation drawer, whose
Chart palette preferences button invokes the preserved ProductPage::Display.

Both entry points now follow that actual visible path, waiting for the chart
presentation and Display page identities. The existing native pointer helper
accepts an explicit scroll-surface caption, defaulting to its unchanged
Preferences behavior. The chart entry names Chart presentation and retains the
same foreground, parent containment, fresh geometry, native hit-target and
wheel/down/up checks. No accelerator or HWND command message was introduced.
Original Display palette, rail, instrument and page assertions remain.

The existing Windows layout/helper check script passes, including its four
original delayed-activation failure/success cases plus one Chart palette
scroll/click case. The five recorded-selector regression cases still pass.
This is helper evidence on Linux, not native execution. The remaining menu audit
found current source destinations for Vessel battery/safety, Sensors Manage
sensors, Radar status, and System recovery/Legacy/Safe; no further obsolete
Settings row was found in this preview flow.

Read-only cache inspection found the newest integrated Ninja cache at
`skager-product-integration/build/xnav-linux`; the corresponding waypoint-touch
build path is a symlink. It uses pinned OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`
in `skager-product-integration/build/integration-source`, but OPENNAV_ROOT and
install prefix point to `waypoint-touch-regression` at
`31a761dfe299eb9e848133dac5ba7b83f966c722`. Eighteen application/build files differ
from FFE's local source, including chart bridge and new Search/Chart drawers;
two upstream patch files also differ. This cache may support an incremental
rebuild after reconciling that complete source set, not a Settings-only rebuild.
No cache/source retargeting, build or install was performed.
