# Supplied artwork: retained public-cell coverage

Read-only source inspection supporting SCRUM-279–282. No application build,
launch, CI dispatch, boat action or chart change occurred. The unchanged
retained NOAA Seattle cell and its update, and official IHO S-64 cell, were
verified against their previously accepted hashes before and after decoding.
GDAL applied adjacent updates. `source-features.json` records every non-null
attribute and geometry bounds for the six relevant classes, without inferring
an OpenCPN rendering decision from their mere presence.

| Cell | SMCFAC | HRBFAC | UWTROC | WRECKS | FSHFAC | CBLSUB |
|---|---:|---:|---:|---:|---:|---:|
| US5SEAFL, real NOAA ENC | 0 | 10 | 1 | 3 | 0 | 1 |
| GB4X0000, official presentation-test geography | 0 | 1 | 50 | 20 | 3 | 20 |

The IHO geography is **not an operational nautical chart**. No proprietary
boat chart or private vessel position was read or included.

## Useful source-backed views

- The existing IHO light/fog view contains nearby unknown-depth rocks
  RCID1/2/3 with WATLEV3. Their bounds give exact positions; RCID38 has WATLEV5
  and no VALSOU. These are candidates for the existing upstream rock procedure,
  not proof it emits a particular final glyph: isolated-danger substitution and
  associated depth-area evaluation still belong to OpenCPN.
- IHO wreck RCID998 is at latitude −32.5319467, longitude61.0135909, CATWRK2,
  WATLEV3, no VALSOU. It is a candidate for the supplied WRECKS05 branch after
  the existing safety-contour logic. Retained contrasting visible-hull wrecks
  include RCID248/402/1885; WRECKS04 candidates include RCID855/1388. They do
  not all fit the same existing close viewport. Do not manufacture nearby
  coordinates merely to put them into one chart screenshot.
- NOAA submarine cable RCID1571 has CATCBL1 and is already covered by the
  Seattle scene. IHO cables include both CATCBL1 and CATCBL6 (mooring/chain),
  providing original-source evidence for the required unchanged class boundary.

## Coverage gaps that must remain explicit

Neither retained cell has SMCFAC or CATHAF5. NOAA's ten HRBFAC points are nine
CATHAF10 and one CATHAF3; IHO's one is CATHAF4. These scenes therefore **cannot
prove the marina replacement is actually selected**, despite containing harbor
facilities.

IHO's two CATFIF1 objects are lines (RCID239/1539). Its sole FSHFAC polygon
(RCID1183) has no CATFIF. The exact fishing-stake area selector requires
CATFIF1; this polygon must not be reclassified just to exercise the new art.
Further suitable licensed/public ENC or explicitly identified test-only
rendering fixtures are needed for that selector. Existing same-name point
symbols and the generic polygon remain unchanged.

No generated image or successful loader result closes these real-chart gaps.
Actual software/OpenGL, native Windows/private renderer and boat recognition
remain separate acceptance requirements.

## Reproduction

Run `read-public-features.py` with the retained `US5SEAFL.000` (adjacent `.001`
required), `GB4X0000.000` and a new output path. The script refuses a changed,
missing or additional numeric chart update and refuses overwriting evidence.
The existing local inspection environment contains pyogrio0.13.0/GDAL3.12.4.
An initial helper filename `inspect.py` shadowed Python's standard library and
failed before decoding; it was renamed. No source bytes changed or failed
rendering evidence was relabelled.
