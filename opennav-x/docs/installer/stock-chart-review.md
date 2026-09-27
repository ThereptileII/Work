# Fixed official-stock chart zoom

`review-stock.ps1 -Action ZoomOut` performs one display-only operation after the
normal stock launch, process creation time/session, exact official executable,
complete plugin inventory, active input-only commissioning and profile proofs
succeed. Each invocation requires a new private review directory. An exclusive
`zoom-intent.json` records command 2001 before dispatch; uncertain delivery is
never retried. The subsequent screenshot uses the original strict foreground,
geometry and unobscured-frame guards.

The exact reviewed source is OpenCPN commit
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`:

- `model/include/model/idents.h` fixes Zoom Out to ID 2001.
- `gui/src/ocpn_frame.cpp::RegisterGlobalMenuItems` creates the first Navigate
  menu with Zoom In/Out IDs 2000/2001 and `+`/`-` accelerators.
- `MyFrame::OnToolLeftClick` handles 2001 solely through
  `GetFocusCanvas()->ZoomCanvas(1.0/g_plus_minus_zoom_factor, false)`.
- `gui/src/chcanv.cpp::ZoomCanvas` and `DoZoomCanvas` change chart viewport scale.
  They do not activate a route or issue equipment commands.

The native helper accepts only the full English or Swedish tuple from that
source/catalogue, including accelerators. It requires exactly one ID 2001 in a
bounded native menu tree, ordinary enabled text leaves, the exact foreground
frame and idle GUI/mouse/modifier state. It rechecks the same menu handles and
labels immediately before one synchronous bounded `WM_COMMAND`. The caller
cannot supply a command ID, key, coordinate, zoom factor or repetition count.
The implementation remains inside the existing single hash-bound native source.
A hidden/replaced/unexpected menu or notification overlay refuses the operation.

The actual official-stock native fixture seeds a fixed non-follow coastline
camera in a new portable profile with no connections and all bundled plugins
disabled. After the real English/Swedish first-start warning flow, it exercises
the same fixed resize, captures coastline before and after one Zoom Out, requires
substantial land/water areas with unchanged colors, then closes normally and
checks that OpenCPN persisted scale 0.0015 from 0.003. Both complete screenshots
are retained for visual review. The separate pure suite checks exact/mixed locale
rejection, menu states, ABI and the restricted public signature.

These fixtures qualify this helper against the official binary. They do not
establish the boat's chart coverage, chart suitability, receiver data or physical
bus silence. At the original recovered boat camera, very close viewport scale
can explain an all-water 100 m view; zoom alone is not proof that charts loaded.
