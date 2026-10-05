# SCRUM-267 — effective symbol table, separate from saved preferences

The retained a3e full-coast images use the upstream default **Paper (82)**.
Updating only Simplified resources cannot make those images or a normal default
installation show the prototype buoy artwork. The new Pier57 evidence separately
uses a disposable explicit Simplified (76) profile; that does not fix the product.

The verified SKAGER S-52 instance now has a private display-only policy, enabled
only after its owned resources pass verification and library creation succeeds.
`GetEffectiveSymbolStyle()` returns Simplified for this instance; otherwise it
returns the unmodified public `m_nSymbolStyle` preference. No profile key or
saved preference is changed. Standard, Legacy and failed-resource fallback
construct an ordinary instance. Style changes retain the existing restart flow.

The core patch uses that effective selection consistently in seven s57chart
render/query/cache paths, two CM93 paths, the plugin render-context/API and both
OpenCPN configuration messages. `UpdateMarinerParams` also uses it, so the existing
conditional-parameter state hash represents the effective table. Boundary style,
depth selection, other mariner parameters and conditional procedures are unchanged.
The inactive old LIGHTS03 source block is not rewritten.

Actual diagnostics copy `saved_point_style` and `effective_point_style` from the
core library on the application thread. They do not expose or retain its pointer.
No new field is invented when the library has not loaded. This core observation
alone is not proof of the separate private o-charts renderer's current state.
The private port must use the same policy before a combined candidate is frozen.

The generic script executes actual getter/enabler/UpdateMarinerParams bodies
with captured parameter writes and the pinned enum declaration: **31 checks**
pass. A negative control that ignores the display policy fails at selection
check7. This is a reduced method fixture, not a substitute for object selection
or chart rendering. All **seven affected real production compilation units**
compile with their original flags and private outputs. The first command-database
setup found only IXWebSocket entries; no compiler ran until the actual Ninja
compilation database was used. Existing build/source/cache files stayed unchanged.

All nine patches reconstruct successfully. Full patched navutil.cpp, ConfigMgr.cpp,
options.cpp and s52cnsy.cpp remain byte-identical to the prior reviewed source.
Saved preference reads/writes and advanced controls are intact. This test does
not claim that an on-disk profile has already passed a full new-app restart:
actual SKAGER→Standard→Legacy→SKAGER profile comparisons, private-renderer
agreement, native Windows and boat gates remain required.
