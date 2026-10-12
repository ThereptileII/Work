# SCRUM-308 — guarded chart information observation

Isolated source: `f868ef6fd9cb29157059f553080cb16b99b28a63`.
Integrated source: `8e074bf7`.

The read-only review helper now recognizes the installed chart-position context
and chart-information drawer. `ChartContextInfo` requires the exact owned context
with direct Go to, Waypoint, Measure and Info controls, then sends one targeted
Info input. `ChartInfoRead` reads bounded existing Static text, including hidden
technical details, without sending input. It preserves every returned section;
it does not infer the first object's identity from its name or pixels.

The implementation agent reports **285 portable checks passed**, with the exact
C# source compiled and a clean diff check. Root inspected the changed source
before integration. These are helper/source results, not native Windows UI or
boat acceptance. No application build, deployment, profile change, hardware
command or lighthouse-symbol correction occurred in this increment.

There is deliberately no automatic chart-center context opener. Existing
geometry and route flags cannot exclude measurement, AIS, waypoint, tide or
other overlay selection. An operator must open the intended chart-position
context first. Use a fresh ordinary Launch receipt after proper preserved-profile
recommissioning; an expired receipt must not be relabelled as fresh. The separate
restart-review dispatcher does not collect this chart-information result.

Normal object information can establish chart object class and attributes, but
does not establish the symbol selected after conditional S-52 rendering. The
helper explicitly reports `ResolvedRendererSymbolObserved=false`. Identification
of the actual northern lighthouse and its resolved glyph remains pending; this
increment does not close SCRUM-308 or claim visual acceptance.
