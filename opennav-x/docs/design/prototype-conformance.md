# Prototype conformance — in progress

Authority: [immutable v8 prototype](prototype/index.html), hash in
[manifest](prototype-original.json). Current deployed Beta `79a95c4` predates
this design lock and is **not visually conformant**. No screen is accepted by
this record. Functional Beta evidence remains valid only for its original scope.

| Screen family | Layout | Type | Color | Spacing | Components | Interaction | Chart | Boat 1280×800 |
|---|---|---|---|---|---|---|---|---|
| Navigation Day/Dusk/Night | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Passage / waypoint | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| AIS list / target / details | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Instruments | Pending | Pending | Pending | Pending | Pending | Pending | N/A | Pending |
| Propulsion / energy | Pending | Pending | Pending | Pending | Pending | Pending | N/A | Pending |
| Autopilot | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Anchor | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Alerts / health | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Settings / display / sensors | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Diagnostics / system | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Radar unavailable | Pending | Pending | Pending | Pending | Pending | Pending | N/A | Pending |

Each PASS requires linked exact-commit reference/current/diff, measured geometry,
resolved fonts, interaction results, a corrective second capture and a boat
review. No averaging across views or broad image tolerance. Physical rendering,
touch, ENC, OpenGL/software and 100/125/150% remain separate gates.

SCRUM-100's [horizon correction](reviews/scrum-100-horizon.md) now has exact
Day/Dusk/Night component reference/current/diff evidence, owned-data freshness
and action guards, and real pointer/keyboard checks. Local geometry passes;
Linux text rasterization differs visibly. Native Windows and physical boat
acceptance remain pending, so the Navigation row above remains Pending.

The boat was unreachable at the September 28 stage transition. It reconnected
and its interrupted read-only commissioning transaction was subsequently
[closed and independently verified](../evidence/boat-prototype-reconnect-20260928.md).
The earlier Beta remains installed and closed. New deployment/capture requires
a fresh read-only audit against the adopted baseline. Do not infer physical
acceptance from Linux reference generation or the previous Windows suite.
