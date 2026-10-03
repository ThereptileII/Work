# SCRUM-252 actual integrated route-label capture — f0976cc

2026-10-03. **OpenGL development evidence passes; software visual acceptance
fails and remains open.** Neither result qualifies native Windows, boat display,
or release acceptance. No product, shared staging, build, or resources changed.

## Exact identity and isolation

- Source: `f0976cc65ea63d3ed6f60ac53ec38a856d11b66b`.
- Installed ELF SHA256: `97c6ee19043a3fe6bd66a099451c26c8a0f5e2f3cf77b8e2fe626f2ab3e38e3e`.
- Resource manifest SHA256: `7c1f9e41f7dbd99b322a5c1cc9a4c0b73b8ad3ba3505c7da89f05ddc5e4abe39`.
- Pinned OpenCPN: `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.

Every run verifies source cleanliness, executable, all nine patches, resource
manifest and individual files before/after. The shared frozen root/cache is
read-only in bwrap. Each run has a private PID/network namespace, temporary
profile, X display and input-only loopback TCP RMC/GGA fixture. Software uses
`--no_opengl`; actual GL reports llvmpipe (LLVM 22.1.8), Mesa 26.2.2, OpenGL 4.6.
This exercises the real GL painter with software GPU implementation, not native
GPU performance. No AIS service, external network, hardware, boat, or CI used.

The copied collector retains all 111 original Python assertion expressions and
requires all 26 original RouteProgressScenario assertions. Added inspection
uses existing owned diagnostics, actual scenario screen projections and normal
Day/Dusk/Night control clicks. It does not inject model pointers, call navigation
processing as a getter, or alter route progress/geometry.

## Results

The definitive runs are `capture-f0976cc-opengl-2` and
`capture-f0976cc-software-2` in the linked evidence directory below.

OpenGL passed all 26 original route checks, input lifecycle, stale-position
checks, hot Day → Dusk → Night → Day label checks, and normal remote-quit exit0.
A real eligible inactive SIM 3 is fully visible with its floating name and 01
circle. Each theme has 431 exact card-fill pixels: Day(247,248,240),
Dusk(36,58,64), Night(16,26,32), and the identical returned Day paint. The actual
active SIM 2 remains stock: no floating-card fill and stock black name pixels
11/11/17/11. Visual inspection confirms both complete names, with no truncation.
The active stock name is low contrast at Night; this is the explicitly retained
stock fallback and remains a visual limitation, not a claim of label-wide
Night acceptance.

The fixture's original arrival/edit/reversal sequence places SIM 1 below the
unchanged chart viewport during the stable capture. It is recorded as offscreen,
not silently counted as a passing visible label. The original early active-route
screenshot is also retained. No synthetic repositioning or zoom was introduced.
The stale screenshot visibly retains its critical warning, stale speed/age,
footer state and lower-right floating controls. The labels remain actual route
names while stale navigation guidance becomes unavailable.

Software fails the added first Day label probe: SIM 3 has **0** expected
floating-card-fill pixels. Independent visual inspection confirms stock names
and absent numbered circles; the teal foreground exists, while the pale
understroke is also absent. This run stops before the hot cycle and full26;
these are **not** claimed passing for software. Its assertion and raw screenshot
are retained unchanged. Read-only source inspection found the software overlay
wraps `wxMemoryDC` (`chcanv.cpp:11987,12162`). Waypoint and understroke painters
create a graphics context; label software painting instead uses an ordinary
bitmap and shares the waypoint ordinal guard. A single graphics-context failure
alone therefore does not yet explain the stock label. Root cause needs actual
guard/context evidence; no guard was relaxed or product changed for capture.

Both initial `*-1` runs stopped on an incorrect new collector expectation of
“pinned symbols”: the original route scenario intentionally has no ENC loaded
and correctly reports “verified palette; ENC not loaded”. Their source and
resource identities matched. Only that added status expectation was corrected;
all original assertions remained intact. Both failed receipts are retained.

## Receipts and reproduction

[Evidence directory](../../evidence/scrum252-integrated-f0976cc/)
contains the exact copied original/adapted collector, helper, identity gate,
preparation proof, per-run logs/identities, original route contract, diagnostics,
full screenshots and crops. `file-identities.json` hashes the receipts.
No private live charts/routes/credentials are included; names and sensor data
are the existing opt-in upstream scenario fixture.

The identity-gated `run.py` is environment-specific to the frozen integrated
cache path recorded in that script. Copy the collector into a private writable
cache directory before reproducing; preserve the frozen cache read-only. Pass
`--expected-commit` and `--expected-exe-sha256` above, `--renderer software` or
`opengl`, and a new unique `--name`. It requires the bundled Python/PIL, sysroot,
bwrap, Xvfb and normal capture tools recorded in the launcher. A failed added
visual assertion remains a failure; do not convert it into a passing oracle.

No broad suite, rebuild, native Windows or boat validation was run here.

## Bounded software runtime diagnosis

One read-only breakpoint probe of the same frozen executable followed. The
first debugger-wrapped launch stopped before the scenario because the collector
searched the debugger PID for the application window. This harness failure is
retained. The corrected launch reads the actual inferior PID from GDB and
reaches the same failing Day0-fill assertion. It is diagnostic-only: debugger
pauses invalidate timing/performance claims and it does not pass the26 scenario.

The ELF lacks RoutePoint/helper DWARF types. The probe therefore reads native
x86-64 call-entry arguments/return registers and the DC vtable, with no inferior
function calls or model writes. Its96 sampled calls establish:

- All24 sampled graphics-context constructions return nonnull. Their callers
  include actual route understroke/waypoint painters from the software overlay;
  their native DC is `wxMemoryDC`.
- All24 waypoint ordinal calls receive `pinned_icon=true`;16 return an eligible
  ordinal1/2/3, while8 return0 for the guarded path.
- All16 sampled eligible label preparations returntrue;8 zero-ordinal
  preparations returnfalse.
- All18 sampled eligible waypoint draw calls returntrue;6 zero-ordinal calls
  returnfalse.

This **disproves null context or shared eligibility rejection for these sampled
calls**; it does not establish every later-frame state. The probe caps each kind
at24 to avoid long debugger interception. The accepted draw calls occur in
`RoutePointGui::Draw` under `RouteGui::Draw` → `DrawOverlayObjects` → `OnPaint`.
No private instrumented build was made. The remaining missing evidence is where
accepted software paint becomes absent from the final image: actual label draw,
DC coordinate/clipping state and subsequent frame/blit ordering. Those require
a new bounded investigation; changing proven factory/custom guards is not a
justified correction. The frozen candidate's software screenshot still fails.

Raw probe program, log, run identities, failed capture, and extracted sampled
calls are under `software-probe/` in the same evidence directory. Shared source,
ELF and resource identities remain exact and unchanged after the probe.
