# SCRUM-267 — observed private point table

This isolated increment starts at frozen application source
`d5d71356d806ea8c3518644d10728a24f1334d1d`. It changes diagnostics only; selected
tables, symbol artwork, saved preferences and chart/helper behavior remain intact.
[The receipt](summary.json) pins the reviewed source, lifecycle comparison,
focused results and remaining gates. The frozen d5/b8 candidate was not changed.

## Observation and lifetime

The existing binding/status v1 header prefix remains byte-identical. A separate
`skager_chart_point_style_v1` export accepts an exact version/size, zero-initialized
request and copies scalar availability/effective-table data. It never accepts a
renderer pointer from the host or retains one across calls. Wrong size/version,
reserved/output bytes and off-thread calls are refused without modifying input.

On the application thread it borrows the actual private `ps52plib` and calls
`GetEffectiveSymbolStyle()`, only after Init completion and with selected binding,
non-null renderer and `m_bOK`. It does not infer Simplified from binding status or
from core diagnostics. Init entry, DeInit and plugin destruction revoke activity;
revocation is atomic and off-thread code cannot enable observation. The pinned
plugin's only global-renderer delete is failed initialization, immediately followed
by clearing the pointer on the same thread. Failed private construction occurs
before activity is enabled. Unbound, fallback, failed, absent and inactive states
return unavailable with no table value.

The host independently requires the original plugin identity, current matching
module handle and initialized container before querying. It validates the copied
response and retains no module function pointer. Core fields now live under
`chart_presentation.core`; private fields live under
`chart_presentation.private_ocharts`. Both expose availability. The effective
private field exists only when actually observed; requested style and existing
presentation status stay separate.

All 219 original source/API blobs were rechecked and both patches reconstructed.
Against d5's derived plugin, removing exactly four activity calls restores the
entire plugin source. Every other derived vendor source file is unchanged,
including S52/artwork, licensing/helper streams and shutdown transport. This
extends source lifecycle evidence only; it does not refresh a commissioning
attestation or remove the closed helper/dongle limitation.

## Focused verification

- The unchanged binding contract passes. The new real observation/host-decoder
  methods pass 84 checks using an inert getter fixture, including actual Paper
  versus Simplified values, a changed getter result between reads, lifecycle
  revocation, null/invalid/fallback renderer, off-thread/malformed requests,
  reserved-byte rejection and unavailable output. No renderer is launched.
- Three negative controls are rejected: bypassing activity, substituting a
  constant Simplified value and accepting nonzero host-response reserved bytes.
- Seventeen package/source guards and 21 actual portable PE-parser cases pass.
  Both strict export inventories now require five exact exports; a legacy
  four-export package is refused. The existing native no-factory host check now
  calls the new export and requires valid unavailable data before Init. Its
  actual Windows execution is still pending.
- Four affected Linux objects compile: `OChartsPresentation.cpp`,
  `OpenCPNIntegration.cpp`, `ChartPresentationAdapter.cpp`, `o-charts_pi.cpp`.
  Existing real production headers/flags were reused read-only, with private
  outputs. Initial host compile setup lacked the installed shapelib include and
  selected stale core S52 headers; the corrected setup uses the existing real
  SCRUM-267 patched headers. Failed setup logs and exact commands remain private.

Active native capture reads unchanged requested/status fields. No active tracked
collector depends on the old flat point-style fields. Five historical collectors
listed in the receipt do; their original evidence stays immutable. Any new copy
of those collectors must read `presentation.core` and independently require the
private observation when reviewing o-charts. Root private script inspection found
no additional flat-field consumers.

Fresh exact-revision native MSVC build/link, five-export/import/package checks,
real host pre-Init/Init/fallback/mode/DeInit/unload observations and licensed boat
chart review remain required. Linux methods/objects do not qualify those gates.
No full build, unrelated restart suite, CI dispatch or boat operation was run.
