# SCRUM-283: restore private associated-area hazard query

Independent implementation based on `48c2f8b`; no SCRUM-280 artwork is included.
The original private `_UDWHAZ03` allocates no associated list because its chart
query is commented out. The retained SCRUM-280 failure is unchanged: UWTROC with
missing VALSOU/WATLEV3, safety contour5m and associated DEPARE DRVAL1=10m emits
UWTROC03/Other privately instead of core ISODGR51/DisplayBase. The analogous
CATWRK2/WATLEV3 wreck produces WRECKS05/Other. This is a safety classification
defect independent of artwork.

## Bounded repair

`ocharts-private-hazard-association.patch` is applied after the existing two
private patches by normal preparation and automatically enters its input/source
receipt closure. All new C++ code is guarded by `SKAGER_OCHARTS_ADAPTER`:

- Append one typed callback to the private `chart_context`. Existing fields retain
  their offsets. No exported symbol, API17/PI structure or host header is changed.
- The one active context initializer in `BuildRAZFromSENCFile` binds the callback
  before attaching that context to chart objects. `S57Obj::Init` initializes its
  borrowed context pointer to null until that attachment.
- A file-local wrapper rejects null/nonowned/mismatched context, calls the
  **unchanged** `eSENCChart::GetAssociatedObjects`, and converts its owned WX list
  to an owned std::list of the same borrowed object pointers. RAII releases both
  temporary containers on conversion failure; normal ownership transfers to
  `_UDWHAZ03`, which already deletes the returned list synchronously.
- `_UDWHAZ03` calls that callback and executes its original depth, EXPSOU, WATLEV,
  isolated-danger and DisplayBase logic. No geometry/classifier is recreated.
  Query/allocation exceptions propagate; they are not turned into a normal
  “safe/no hazard” result. Missing/empty/unavailable context retains unavailable
  original behavior and is **not qualified as a safe portrayal**.

Original stock-private conditional code is retained under the macro's else
branch. Disabling the adapter macro leaves the original context size and field
positions and all tested conditional output/categories unchanged.

## Lifetime and source boundary

The chart owns the context and all associated S57 objects. The original destructor
calls `FreeObjectsAndRules` before freeing the context. The private renderer's
stack-copy sounding object borrows the same context synchronously; no query or
borrowed pointer is queued or exported. API17 selection constructs fresh
`PI_S57Obj` values and copies selected fields; it does not copy `m_chart_context`.
The old `#if 0` PI-context initializer remains unchanged and inactive.

The original query, selection function and destructor are byte-identical in
`source-proof.json`; API17 header bytes are identical. All 217 locally retained
source/gitlink blobs match the source lock, and a fresh three-patch preparation
reproduces all 217 derived files exactly. The full lock has 219 entries: this
local subset omits two license-file copies and is not a distributable archive
qualification. Normal production preparation still requires the complete lock.
The source proof establishes ownership/order; it is not a concurrent renderer/
chart-destruction stress test or private native-DLL lifetime acceptance.

The original query tests the transformed **reference point** even for line/area
features. It returns the first containing associable area from priority1 plain
boundaries, otherwise symbolized boundaries. It does not test complete line/area
intersection geometry. That documented upstream limitation remains unchanged;
this repair must not be described as complete line/area hazard qualification.

## Focused evidence

Actual-source fixtures compile the original conditional procedures and callback,
using actual source-specific headers and a deterministic associated-list fixture:

- Core: 37 checks; original private: 37 checks.
- Repaired private: 55 checks, including both affected ISODGR51/DisplayBase cases,
  constructor-null initialization, null/no-callback/nonowned/mismatched contexts,
  empty/null-return query, exception propagation, synchronous borrowed copies,
  and failure of each conversion allocation with temporary-container cleanup.
- Patched source with adapter disabled: 37 checks, original context layout and
  original outputs retained. The original private binary fails the explicit
  required-repair negative after the first 17 cases.
- 26 core/repaired-private conditional outputs **and display categories** agree,
  except the separately asserted original private Wk text alignment:
  `TX('Wk',2,1,2,'15110',1,0,CHBLK,21)` remains private, versus core
  `TX('Wk',3,1,2,'15110',2,0,CHBLK,21)`. No unrelated alignment repair is included.

The list fixture does not simulate geographic intersection. Actual patched
`s52cnsy.cpp` and `eSENCChart.cpp` separately compile to real objects at O3 with
existing adapter/GL flags and `-Werror=dangling-pointer`; receipts retain commands,
object hashes and compiler logs. No warning suppressions were added. The fixture's
initial inline replacement-allocator warning was resolved by moving the paired
new/delete definitions into a separate test translation unit, not by weakening
compiler flags or production behavior.

`verify-private-hazard-association.py` reproduces the four focused casesets with
`--core <pinned-core> --private-original <locked-private> --private-patched
<normally-prepared-private> --output <isolated-proof> --wx-prefix <actual-wx>`.
It refuses unpinned original private source and retains the original failure.
No full suite, application build/link, Windows dispatch, service/network or boat
operation was performed. Native Win32/private-DLL build and runtime, real ENC
point selection/portrayal, line/area limitation review and boat acceptance remain
unpassed. The original SCRUM-283 safety failure stays recorded until those gates
qualify the exact integrated candidate.
