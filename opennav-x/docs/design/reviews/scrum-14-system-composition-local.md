# System Preferences composition — SCRUM-14

Selected scope: Jira comment 10503, following the composition audit in 10474.
Isolated base `688bb713db885dbe78ca6db861382ce248c231f7`; the frozen candidate
worktree is unchanged. The active immutable `settingLinks()` System branch
at `docs/design/prototype/index.html:465` calls `systemPanel()` at line 469.
The older superseded renderer is not the reference.

This is a **partial System composition increment**, pending native validation
and the missing product capabilities. Disabled rows are truthful interim states;
they do not complete installation, updates, backups, guides, licensing or setup.

The System landing now follows the actual intro and eight-row order. The
eyebrow reads `OPENNAV X ·` followed by `application::Version`, currently
`0.4.0-beta2`, rather than the prototype's simulated package version.
The title and subtitle are the original prototype text. Typography follows
the active line-61 CSS: 10px eyebrow, 23px bold title with −0.6px tracking and
30px line spacing, 13px supporting copy. The title wraps with measured font
width and reserves another 30px per line, avoiding ellipsis on Linux fallback
fonts and narrower drawers. Existing 72px suite rows are reused. Four missing
icon paths are copied unchanged from the immutable prototype icon table;
existing enum values and shared painting behavior are preserved.

| Row | Actual destination or explicit limitation |
| --- | --- |
| Installation & recovery | Disabled: installer unavailable; recovery controls below |
| Updates | Disabled: update controls unavailable |
| Backups | Disabled: backup and restore controls unavailable |
| Diagnostics | Existing diagnostics callback; versions, data quality and source health |
| Plugins | Existing OpenCPN plugin-settings callback |
| Help & guides | Existing Help tab only; subtitle says basic help and guides unavailable |
| About & licenses | Disabled: version shown above; license viewer unavailable |
| Run vessel setup | Disabled: setup wizard unavailable; points to the existing Vessel tab |

The existing Interface & recovery destination and direct Advanced / Legacy
Settings callback follow those eight rows. Recovery still exposes the real
Legacy/Safe/restart actions, commissioning/recordings and diagnostic export.
Those destinations are not duplicated on the landing page. No updater,
backup, installer, license viewer, guide or setup subsystem was introduced.
The Help section and upstream preferences behavior are unchanged.

The existing Settings component harness was extended only for affected flows:
disabled capabilities reject even explicitly delivered command events; scrolling
reaches Advanced; recovery, direct Advanced and limited Help route correctly;
Escape and Alt-Left dismiss the System root and reopening resets its scroll.
The single-shot timer and original assertions remain. The System capture now
uses 100% interface scale to match the primary 432px prototype drawer; the
Display captures still exercise their original applied-scale states.

Focused compilation of the six Settings UI/test units against local wxGTK,
using the cached application/adapter/navigation libraries, succeeded. The
fixture passed **201 checks**. Its original 14 capture identities remain;
`system-bottom-night.png` is an additional diagnostic image. The retained top
and bottom PNGs, result and source hashes are in
`docs/evidence/scrum14-system-composition-linux/`.

Both System PNGs were opened and inspected at 1280×800. Full intro text,
disabled explanations, ordered rows and the two working bottom links remain
readable. Linux wraps the title to two lines, as the canonical Linux reference
does; canonical Windows renders it on one line. Native font metrics, exact
one-line Windows composition, disabled-state contrast, scrolling and keyboard
flows remain exact-source Windows review gates. Boat-display acceptance also
remains open. This is not a full Settings, release or boat pass. No full build,
CI run or publication was performed.

## Affected Windows DPI caller

A separate follow-up changes the two night-workflow entries in
`tools/smoke-dpi-windows.py`. Each now uses the existing native pointer path:
Settings → System → Interface & recovery → Commissioning & recordings or
Export diagnostic bundle. `ui.open_system()` scrolls and checks the real drawer
target; `ui.pointer_text()` retains enabled/containment/hit-target checks for
the destination control. The existing destination identities and night-surface
assertions are unchanged. No hidden accelerator or synthetic command is used.

These two destinations require eight pointer activations per DPI scale, 24
across 100/125/150%, versus six per scale previously; native wheel events needed
to reveal recovery are additional and depend on the viewport. Static Python
compilation passes. This caller-only follow-up does not rerun the component:
its last interaction result remains 201 checks. Native DPI execution is pending.

The brief caller audit confirms `smoke-charts.py` uses Plugins & adapters from
Radar, which remains valid. Windows recording already uses `open_system()`.
It also identifies existing below-fold visibility assumptions in
`smoke-preview.py` section settling, `smoke-modes-linux.py` action/return
predicates, and the Linux branch of `smoke-recording.py` (Up/Down-only scrolling).
Those findings were reported to the integration owner; this follow-up is confined
to the requested Windows DPI caller.


## Remaining affected callers

A second separate caller follow-up repairs the three additional paths found by
the audit. Preview waits for System controls to exist, then reuses its bounded
Preferences wheel/native-pointer path until recovery is visible and contained.
The Linux mode cycle similarly reveals recovery before its existing pointer
action; after Escape it waits for Settings, scrolls, and retains its original
visible/enabled recovery return assertion. Linux recording uses the Preferences
wheel when that drawer is open and retains the product page's existing Up/Down
path elsewhere. Every destination still requires a visible enabled control;
no hidden command or offscreen click was added. Existing destination and
persistence assertions remain. Radar's Plugins & adapters caller is unchanged.

These three paths add only viewport-dependent wheel interactions, with no new
fixed destination clicks. The DPI count above remains eight pointer activations
per scale for its two recovery destinations. All four affected Python files
pass static compilation; no full smoke suite or new mirrored tests were run.
The latest local component result is still 201 checks. Exact-source Windows
interaction/visual qualification and boat-display acceptance remain pending.
