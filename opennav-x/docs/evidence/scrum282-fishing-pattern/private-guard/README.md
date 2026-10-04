# SCRUM-282 private guard correction

Independent parent review found the new private cell hook was guarded by
`OPENNAV_X`, which the owned adapter deliberately does not define. The base
fixture compiled the core method and checked identical private text; that was
insufficient to establish private execution. Its original results remain
retained, but must not be read as a private-adapter guard qualification.

Only the private hook guard changes to `SKAGER_OCHARTS_ADAPTER`. The core guard,
rule gate, inset limits, pattern geometry, resources and fallback are unchanged.
The fixture now separately compiles and executes the complete extracted core
and private cell methods with their respective owner macro. Each passes
1,048,119 assertions, predominantly repeated per-pixel checks, not distinct test
cases. An original-guard negative control uses the real private macro and
fails at the expected no-HPGL assertion; the corrected method passes it.

The complete private `s52plib.cpp` also compiles successfully using the exact
adapter definition list extracted from `Targets.cmake` (no `OPENNAV_X`) and
its relevant target include closure. Source was independently reconstructed
from locked original blobs and both current patches. Object symbol inspection
confirms the two owned pattern functions are compiled in. The recorded command
uses Linux wx/GL headers and compiler; this is neither Win32 compilation nor
private DLL link/runtime acceptance. The object stays untracked locally.

## Allocation and maximum scale

The ppmm guard and dimension cap bound the temporary image arithmetic. At the
maximum accepted input of 24 ppmm the candidate cell is 842×518 and the scaled
source is 152×152. RGB plus alpha would occupy 1,744,624 bytes; integer products
cannot overflow. This extreme scale actually declines before cell allocation:
its required 14-pixel inset exceeds the authorized ceil(24/(96/25.4)) = 7-pixel
limit. The focused test proves that decline leaves the result empty and the
actual cell method takes stock HPGL fallback. Values above 24 remain rejected.
The earlier assumption that maximum scale should compose was rejected by the
fixture; the presentation limit was not broadened to make it pass.

Bounded size does not guarantee successful allocation. New checks reject an
invalid/null RGB cell before `InitAlpha` and reject absent/null alpha before
clearing either plane. Those failures return false to the existing stock
fallback. This source inspection and normal allocation exercise do not claim
an injected out-of-memory test or a guarantee for upstream fallback allocation.

No resource regeneration, full app build, native CI, polygon/GL draw or boat
operation was performed. Actual combined rendering remains an integration gate.
