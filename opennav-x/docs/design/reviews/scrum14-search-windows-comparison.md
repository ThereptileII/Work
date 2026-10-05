# SCRUM-14 Search comparison and scoped correction

Selected in Jira comment 10496. The Windows reference is from
[run 37030134184](https://github.com/ThereptileII/Work/actions/runs/37030134184),
revision `a18a197`, artifact `11236538763`, archive SHA-256
`f3b14b845084e8d9dc2a06db1f1ef42f234e48bcdb15ffad7f9647620d71206a`.
Its capture metadata records Windows, Playwright 1.58.0, Chromium 145.0.7632.6,
1280x800, DPR 1, and immutable HTML SHA-256
`b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`.
The three Day/Dusk/Night PNG hashes match that metadata. Actual fonts are
Segoe UI for title/input and Segoe UI Semibold for result titles.

Compared `saved-objects-day.png` and `empty-night.png` from the native component
capture for `29ea06a358ed24cc29eb2be74b67b234a3ecaa6e`. Both hashes match its
manifest; executable SHA-256 is
`d42f6afd369b2bf9b8a1de3846014d1fe0442ae4bc6191f4bf710aa9afb7b022`.
The four concrete Windows findings are:

| Item | Native observation | Reference | Scope |
| --- | --- | --- | --- |
| Result spacing | 66px rows; separators y318/384 | 71px; y323/394 | Corrected here |
| Focused input | Thin accent border only | External 2px accent outline with 3px offset | Corrected here |
| Shared title | Ink y130–148 | Ink y126–144 | Still open; shared Drawer outside this patch |
| Shared frame | Right/bottom outline absent at x1079/y753 | 1px outline | Still open; shared Drawer outside this patch |

Drawer bounds 682,80,398,674, header divider y169, input 352x44, and sampled
Day/Night background, surface and border colors agree. The saved-object labels,
two results, empty-state text and explicit unindexed-place-name explanation
are legitimate scope differences from the illustrative places. No equivalent
native Dusk capture was supplied, so this comparison does not qualify Dusk.

The immutable CSS at `index.html:60` specifies the 2px focus outline and 3px
offset; the reference `.list-card` measures 71px with 16px vertical padding,
12px title and 9px subtitle. Search now reserves a 5px paint margin around the
unchanged 352x44 editor. Its outer paint panel is 700,189,362,54; results remain
at x705/y253, and the next row starts at y324. Only Search adjusts its body
insets for this margin. Shared Drawer painting and heading code are unchanged.

Focused validation recompiled SearchDrawer and its existing component harness,
then linked an isolated executable with the existing local dependency archives.
One private Xvfb execution passed **85 checks**, including the three added exact
geometry observations, existing selection/generation/availability behavior and
the 44px header action. Day and Night captures were visually inspected once.
The Day pixels show accent at y189–190 and x700–701, three background pixels
before the editor border, and row separators y323/394; Night uses its own accent.

Local commands and captures are under ignored `build/search-spacing` and
`evidence/local/search-spacing` in the isolated worktree. This Linux component
result does not qualify Windows text/rendering, whole-view behavior, higher DPI,
or boat acceptance. Fresh native evidence is still required. No full CI,
publication or boat action was performed.
