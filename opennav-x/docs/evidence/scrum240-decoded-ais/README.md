# SCRUM-240 — actual decoded healthy AIS paint

The frozen Linux development executable at `1356fd1603aacbea04d7081d16331e9a181180bb`
(SHA-256 `0aaaf2238828fcf4b42d02c8b7dad7191781ead1bf2ece0c2a9384c1fdc726bf`)
rendered two decoded onboard AIS targets over the real US5SEAFL ENC in both the
software and OpenGL paths. SKAGER uses the expected plum/floating-surface body;
Standard retains the stock green body. All four complete runs exited normally.
No product source, executable, resources or build configuration changed.

## Actual input and observation

A dedicated `bwrap` namespace adds `--unshare-net` to the existing private-cache
wrapper. It has no non-loopback network route. A new loopback TCP port, short
`/tmp` profile, and Xvfb display are owned by each run. No private profile, boat,
hardware, internet AIS service, Demo source, route scenario or model injection is
used. The application still truthfully identifies itself as a developer test
build with test-loopback-only capability; no pilot capability was enabled.

The collector adapts the checksum/bit construction in `tools/smoke-navigation.py`
and the pinned `AisDecoder::Parse_VDXBitstring` fields. It sends:

- MMSI 990000002: Class A type 1 position and multipart type 5 static report,
  `SIM CLASS A`, 47.606/-122.366, 5 kn, northward HDG/COG, underway using engine,
  ship type 30.
- MMSI 990000004: Class B type 18 position and both parts of type 24 static report,
  `SIM CLASS B`, 47.594/-122.366, 5 kn, southward HDG/COG, ship type 36. The
  pinned type 18 decoder supplies the Class B unspecified navigation status.
- RMC at 47.6/-122.36, 3 kn/090°, through the existing selected-navigation path.

These are explicitly simulated test messages entering the **actual** decoder;
they are not live traffic. Signed longitude encoding, message fragment padding,
checksums, complete static names and position fields are retained in each
`controlled-input.json`. Target positions are intentionally separated from the
ownship and each other. Chart geography is the actual public ENC.

The actual owned AIS list visibly reports **2 targets**, both names and onboard
source. Native pointer input opens each actual target card; diagnostic selection
identities must match the intended MMSI. Both cards show Current position and an
enabled `Show on chart` action, which uses the existing `CurrentPosition` guard.
The retained SKAGER age screenshots were visually inspected: software A **1 s**,
software B **0 s**, OpenGL A **0 s**, OpenGL B **0 s**; all identify
`OpenCPN onboard AIS`. The Class A top card reports Active / Underway using Engine,
5.0 kn and 0°. Class B reports Active, 5.0 kn and 180°. After inspection, Back and
Close clear selection; final body captures require MMSI selection zero and zero
AIS encounter events. The collector preserves the original navigation freshness,
position, source, pilot-disabled, renderer, real-ENC, palette and framebuffer
edge assertions.

Per-target internal flags are not newly exported or altered. The initial bounded
read-only debugger attempt could not inspect decoder fields because the frozen
binary has no `AisDecoder`/`AisTargetData` debug types. Its failure is retained in
`retained-attempts/`; it did not inject calls or write target state. The subsequent
list/card path supplies observable freshness/source/state. The exact class input
and body geometry follow the inspected decoder and painter source. Eligibility
of the new paint is an inference from those verified source guards plus the
actual characteristic body pixels, not a claim that every private flag was
independently read. An initial UI-list inspection checkpoint is also retained;
only the final software/OpenGL reports are passing runs.

## Pixel evidence

`check-pixels.py` operates on the untouched captured PNGs. It passes eight
regions (two targets × two styles × two renderers), requiring exact #916477
stroke and #F7F8F0 interior in both SKAGER bodies, neither color in Standard,
and the actual stock green in Standard. Separate interior samples prove the
Class B notch is not filled and the Class A stern remains straight. The sampled
screen regions were located in the actual chart captures, not fabricated from
model pointers. Software antialiasing and GL edge coverage differ; this test does
not demand identical edge rasterization.

| Path | Class A exact stroke / fill pixels | Class B exact stroke / fill pixels |
| --- | ---: | ---: |
| Software SKAGER | 26 / 64 | 33 / 38 |
| OpenGL SKAGER | 96 / 82 | 96 / 58 |
| Both Standard paths | 0 / 0 | 0 / 0 |

OpenGL logs identify **llvmpipe (LLVM 22.1.8), Mesa 26.2.2, GL 4.6**. This is an
actual production GL painter/shader execution on a software GL implementation,
not physical-GPU acceptance. The full capture contains a pointer tooltip outside
the target regions; it is unmodified and does not obscure either AIS body.

## Reproduction and limits

The exact collectors, source/executable/resource hashes, self-test, profiles,
input receipts, final diagnostics, logs and screenshots are retained here. To
repeat, stage these collector files beside the same frozen cache inputs and
invoke `run-ais240-isolated.sh` with the bundled Python (Pillow available), a new
phase name and unused display. Use `--renderer software` or `--renderer opengl`.
`check-pixels.py` can independently recheck these retained screenshots.

The inherited isolated profile has AIS CPA warnings/dialog/audio disabled;
those defaults were not changed to make this test pass. Real-time extrapolation
is explicitly off to exercise the existing bounded body policy. Consequently,
this evidence does **not** validate alarms, CPA calculations, dangerous targets,
lost/stale states, selected appearance, special statuses, real-time prediction,
Dusk/Night AIS, native Windows/DPI, physical GPU, touch, boat display or actual
traffic. Standard comparison is chart presentation only, not a Legacy/Safe mode
qualification. The cards briefly select targets for inspection, but selected
body acceptance is not claimed. SCRUM-240 remains Testing.
