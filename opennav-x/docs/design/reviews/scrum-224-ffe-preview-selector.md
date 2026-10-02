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
At the first source-fix commit, live preview re-execution and native qualification
remained pending; no build, suite, CI dispatch or boat operation had run. The
authorized isolated local verification is recorded below.

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
This was the read-only cache assessment before the separately authorized local
verification below.

## Coherent isolated Linux verification

The authorized development build uses a private clean Git clone at
`74761400862bc1d25102b2b741d14247dd343cdc`, a reflink copy of the configured
cache and prepared OpenCPN tree, and a private development install. A read-only
root bubblewrap namespace maps only these copies onto the cache's configured
paths. Original worktrees, cache and frozen FFE are unchanged. The private
prepared source was verified against pinned upstream plus all nine current
patches before and after building; no changed header, dependency or version
input was suppressed. This is a complete integrated source reconciliation, not
a mixed-version Settings build.

Ninja's final dry run required 244 target steps. The two-job `opencpn` build
reached 231/244 at its 900-second bound; the authorized continuation finished
in 53 seconds with 14 remaining steps including the existing libdnet echo.
The installed executable is 25,721,560 bytes, SHA-256
`233b0d8745c82a91dca7fe8c50ae879a16f1e3cb5475a421fdae322a54821b83`.
Its generated build header records source `7476140`, GNU 16.2.1, 64-bit Linux
and local-development authority. No full CTest suite or CI dispatch ran.

The build identity, 1,510 tracked source hashes, logs and isolated binary are
retained locally under `ffe-preview-selector/.local/incremental-preview/`;
`source-identity.json` binds those inputs and executable. These ignored local
artifacts are development evidence, not a distributable product or native
Windows qualification.

The first live preview passed the former Diagnostics collision, continued
through the retained settings and stale-data checks, and visibly acknowledged
the GPS alert. It then ended with wrapper exit 143 and no Python traceback or
final result export. Its 36 screenshots and interruption record are retained
in `preview-interrupted-1/`. The termination cause is unknown; this is not
recorded as a product crash or a completed preview pass. An unchanged-binary
repeat uses a bounded persistent supervisor and task-local temporary storage
so session loss cannot discard its logs.

The bounded repeat completed in 196 seconds with a genuine terminal failure at
Legacy → XNav, not at the corrected selector. Its retained
`app/evidence/local/preview-linux-results.json` binds the same executable SHA,
60 pointer actions, 39 screenshots and six completed check groups. The actual
Diagnostics pointer targeted the visible enabled Settings row
`(671,664,386,72)`. `preview-05-diagnostics-linux.png` was inspected and shows
the Diagnostics destination and exact `7476140` build. All eight fixture
scenarios, stale/unavailable/shortfall assertions, alert acknowledgment, recovery
and recurrence passed. XNav → Legacy exited cleanly and rendered the retained
coastline.

On the return request, the observed Legacy PID 1116 exited zero. The observed
replacement descendant PID 1281 never produced the expected window and was
retained as a zombie with `waitid` code 2/status 9 (SIGKILL), with empty observed
argv. The last application log records Legacy's clean exit; no replacement
startup was logged. `preview-failure-inventory.json` and the visually inspected
black `preview-failure-linux.png` retain this missing-window state. The available
kernel interval has no OOM record, and the bounded supervisor exited with the
Python test's status 1 rather than timing out; the cause of the replacement's
SIGKILL remains unknown. This is not a completed lifecycle/preview pass, and
later Safe/direct-start steps were not reached. No third run was performed.

The corrected native Display route remains native-execution pending: this Linux
preview retains its existing Display fixture accelerator and therefore cannot
qualify the new Windows Preferences → Navigation → Chart presentation → Chart
palette preferences pointer path. Windows typography/DPI and boat acceptance
are also unchanged and pending. The separate viewport observer uses this same
binary; its results belong to SCRUM-228 and do not turn this preview failure into
a pass.
