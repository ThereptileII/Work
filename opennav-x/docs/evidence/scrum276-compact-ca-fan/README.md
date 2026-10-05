# SCRUM-276 compact ordinary sector fan

This isolated increment paints the prototype's ordinary red/white/green compact
fan over each renderer's **existing** CA geometry. It does not adopt the prototype's
illustrative 44/340 radius, combine records, consume the SCRUM-275 point owner,
change CA strings, evaluate conditional rules as a getter, or alter sector
visibility. Native Windows, actual chart canvas and boat performance remain open.

## Source and boundary

`source-inputs.json` binds the implementation and immutable prototype files on
base `e6ef3843dc604368d32bc823c770bec76d890b69`. Prototype
`src/chart-symbols.css:4–5,10–12` supplies .17 wash, .65 CSS-pixel boundary at
.6 opacity, 1.2 CSS-pixel arc at .8 opacity, and theme inks. `style.css:13`
requires Night brightness .78 once. `light-sectors.js:16` orders wash, boundary,
then arc. SVG default butt caps and miter join/limit 4 are retained. Expanded
selection, pinning and readout are outside this increment.

The shared classifier requires verified-library enablement, the exact Simplified
LIGHTS/31183 `CS(LIGHTS05)` lookup, one ordinary typed R/W/G color, finite complete
sector limits and ordinary CA outline/width/color parameters. Present orientation,
special/directional, visibility, status or quality attributes, other colors,
all-round, malformed inputs and unknown themes remain stock. The per-record test
is independent of the co-location owner inventory.

Both original `RenderCARC_GLSL` and `RenderCARC_VBO` methods retain their own
radius/display/SCAMIN calculations, rounded center and leg endpoints. Core/private
GL scale differences remain intentional upstream behavior. The new successful
paint expands conservative geographic bounds including the center, both legs,
full arc radius and antialias padding; original fallback is preserved. No Rule
cache or cross-frame texture is added.

CSS widths map through `m_ContentScaleFactor / m_dipfactor`, matching existing
S52 raster scaling: core `base_platform.cpp:GetDisplayDIPMult` is inverse Windows
DIP; `ocpn_frame.cpp` installs DIP/content scale and `pluginmanager.cpp` transmits
them to the private renderer. Chart zoom and SCAMIN still control the existing
geometry, not these non-scaling stroke widths.

The GL tile uses the original method's already rotated pixel center. The fixture
executes the actual `PrepareS52ShaderUniforms` matrix prefix and checks equivalent
transforms at positive and negative rotation. Host normal chart viewport is
(0,0,pix_width,pix_height); private rendering uses the supplied viewport. Any
nonzero/mismatched GL viewport conservatively uses the original painter. Android
GLES also remains stock. This does not claim support for arbitrary backing tiles.

## Focused proof

`method-receipt.json` binds extracted **actual core/private CA method bodies**, the
original pinned comparison bodies, actual LIGHTS06, shader sources and matrix
prefix, compiled executables and helper. Both variants pass **576 checks** using
wx native software and Mesa llvmpipe OpenGL 4.6. All nine R/W/G × Day/Dusk/Night
whole images agree within the original one-channel-level quantization tolerance;
independent interior samples require the exact ink and alpha 43/255. Overlapping
paint, wraparound, rotation, scale, .65 boundary coverage and an acute miter are
also checked. Representative originals are `core/DAY_BRIGHT-3-gl.png`,
`core/NIGHT-1-software.png`, and `core/rotated-negative-gl.png`.

Standard/disabled, present ORIENT and uncertain QUAPOS actual CA methods produce
zero-difference images against the pinned original methods and identical recorded
stock leg endpoints. These fixtures execute real arc painters and shaders;
projection and terminal stock leg drawing are bounded recording substitutes,
not an actual chart. The fixture does not establish real chart clipping,
SCAMIN selection, label composition, private DLL loading or native GPU acceptance.

Hostile GL state checks cover borrowed program matrices/sampler, current program,
vertex attributes including integer mode and VBO bindings, array/unpack buffers,
unpack layout, active texture/unit-zero texture and sampler object, blend enable
and separate equations/factors, and culling. Changed state is restored on exit;
viewport, framebuffer, stencil, scissor, depth and color masks are not changed.
Nonzero viewport refusal leaves state intact. No shared shader source changes.

`objects/` records actual core/private S52 translation units compiling at `-O3`
with `-Werror=dangling-pointer`. `patch-proof.json` records all nine core and two
private patches composing and their S52 output matching the tested files. The
new header is in private copy closure; existing native source-header inventory
covers it and existing S52 object gates remain. All 17 private preparation checks
pass. No complete application or broad test suite was run for this increment.

## Preserved failure and cost

The first actual raster oracle failed red opacity. Native wx/Cairo drawing of
low-alpha color into wxImage lost RGB precision while unpremultiplying (182 red
became 177 at alpha 43). Original failed PNGs/logs are retained under
`preserved-alpha-failure/`. The repair draws three opaque white coverage masks
and composes straight RGBA; assertions and tolerances were not weakened.

Memory/work is bounded by 1,048,576 pixels and 2048 per dimension. Oversized or
invalid whole fans fall back stock; legs are never cropped to fit. Final run:

| Observation | Core | Private |
|---|---:|---:|
| Typical tile construction | 0.441 ms | 0.604 ms |
| Typical GL upload plus finish | 0.502 ms | 0.708 ms |
| 1023 × 1023 construction | 41.23 ms | 76.68 ms |
| Near-cap upload plus finish | 14.35 ms | 24.16 ms |
| Observed process peak RSS increase | 16,508 KiB | 16,384 KiB |
| Disabled cached software paint | 0.0164 ms | 0.0215 ms |
| Original cached software paint | 0.0183 ms | 0.0216 ms |

Near-cap RGBA is 4,186,116 bytes; simultaneous native mask/image backing, copies
and GPU storage add to that, so this is not a 4 MiB total allocation claim.
RSS is a process high-water observation, not an allocator bound. Earlier retained
runs in `timing-observations/` measured up to 94.83 ms construction under other
machine load. Timings are local observations, not performance thresholds or boat
qualification. Three per-pixel passes and transient uploads can be material with
several large visible sectors. This cost must be assessed on the next actual
candidate; it is not hidden by a smaller arbitrary cap or reduced visual scope.

After the separate SCRUM-275 lifetime correction `eaff747` was cherry-picked as
`fe44d02`, both actual S52 objects were compiled again at the same O3 warning
settings. Both passed. `objects-with-lifetime-fix/` binds the corrected inventory
header and unchanged fan header; earlier receipts remain unchanged. No method,
resource, canvas or broad suite was repeated for this independent lifetime fix.
