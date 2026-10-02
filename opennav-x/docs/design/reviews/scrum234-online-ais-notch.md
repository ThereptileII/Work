# SCRUM-234 — preserve the Online AIS stern notch

The immutable prototype's AIS path is `M0-12 6 9 0 5-6 9Z`
(`docs/design/prototype/index.html`, SHA256
`b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`).
Its stern notch is concave. Pinned OpenCPN
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`,
`gui/src/ocpndc.cpp:1318`, special-cases four-point polygons by swapping
vertices 2 and 3, then drawing a `GL_TRIANGLE_STRIP`: indices 0,1,3,2.
The original bow/right/notch/left order therefore fills the empty notch.

`OnlineAisOverlay.cpp` now starts the same polygon at the right vertex:
right/notch/left/bow. The strip's shared diagonal joins notch to bow and
its triangle union matches the prototype. This is solely a cyclic reorder:
all software polygon edges, winding, coordinates, projection, rotation,
rounding, DPI scale, paint state, availability/age handling and hit testing
remain unchanged. No generic drawing code or upstream patch changes.

Focused evidence is in `docs/evidence/scrum-234-ais-notch.json`.
`tests/online_ais_polygon_tests.py` reads the hash-verified immutable SVG,
the actual production vertex arrays, and the exact pinned upstream strip
implementation through Git. It compares an independent even/odd SVG
interior against the strip triangles over 1,326 points across three
rotation/scale cases, checks the notch explicitly, and verifies software
path equality as a cyclic sequence. Replay against original `25297e3`
fails the two GL geometry checks while software equality passes; the
correction passes all three tests.

Reproduce the focused check with:

```sh
python3 tests/online_ais_polygon_tests.py --upstream /path/to/OpenCPN -v
```

Only the changed production `OnlineAisOverlay.cpp` object was compiled,
successfully, using the existing integrated compile flags and pinned
headers read-only. All output went into this worktree's private directory.
The shared integration source and completed 136-step cache were unchanged.
No application rebuild/link, capture, broad suite, CI or boat operation ran.
This proves geometry and Linux compilation; actual GL rendering, native
Windows and target-hardware acceptance remain separate gates. The prior
25297 software screenshots are not evidence of this newer correction.
