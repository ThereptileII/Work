# SKAGER presentation for the pinned o-charts renderer

SCRUM-259, implementation in progress. This is not native or boat acceptance.
The frozen 78eccb8/61a0a783 application remains separate; its native chart-text
failure is retained. The adapter is unavailable by default until a qualified
package is supplied during application construction.

## Why a separate adapter is necessary

The pinned o-charts plugin links a private S-52 renderer. Core ENC presentation
changes cannot style the boat's `.oesu` charts. The inspected source, render
boundaries and private-library lifetime constraints are recorded in the
[source audit](../design/reviews/scrum259-ocharts-private-presentation-boundary.md).
Raster MBTiles remain raster content; this mechanism does not recolor them.

## Ownership and loading

The adapter is built from o-charts source
`c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8`, with its pinned API import library,
reviewed presentation changes and the qualified current Windows networking
closure. It is a SKAGER-owned `skager-ocharts-adapter.dll`, outside normal
`*_pi.dll` discovery. Closed helpers and licensed charts are not included.

Only the exact original `o-charts_pi.dll` SHA-256
`99edcfd4419d606ef5c3fd554759e853cfda1fe4f38c05e26266f335a5a5b875`
is eligible. Original discovery identity, enabled preferences, modification
tracking, plugin name, helper location and chart/plugin-data paths remain
unchanged. One module/container is loaded. No second plugin is registered.

At the shared `PluginLoader::LoadPlugIn` boundary, SKAGER may select the private
module only with the selected SKAGER interface, verified SKAGER presentation
resources and an exact compiled adapter digest/size. Missing, changed,
unsupported or incompatible inputs retain the original plugin. The actual
adapter receives its own upstream compatibility check; checking the original
DLL alone is insufficient. File locks and a second hash check bracket module
loading after that compatibility inspection. Failure to unload a rejected
module stops loading instead of mixing original and adapter code.

`OChartsModuleLoader` owns the actual Win32 path/handle/hash/load/bind operations;
the application wrapper supplies observed startup policy and compiled package
identity. `PluginPresentationFallback.h` is shared by production and the native
guard harness. Because wxWidgets `Unload()` returns void and forgets its handle
even on failure, this Windows boundary checks `FreeLibrary` directly and keeps
ownership if it fails. A rejected module cannot be replaced by the original
while still held in the destination. Locks cover loading, binding and the first
status observation; they are released before OpenCPN calls the normal factory.
No vendor factory is invoked by the isolated guard harness.

Model-only tools have a null registration hook. Standard/Legacy do not select
the adapter. Safe Mode retains normal plugin suppression. Ordinary reload
passes through the same selection boundary. Style changes use the existing
controlled restart; private chart lookup/text/GL caches are not replaced live.

## Private binding

`ChartPresentationBindingV1.h` is a private, versioned C ABI. The host resolves
`skager_bind_chart_presentation_v1` only in a verified module, before `create_pi`.
The complete request is copied: structure size/version, a bounded UTF-8 absolute
resource directory and zero reserved bytes. No chart, wx object, allocator or
callback crosses the DLL boundary. Binding is one-shot, and malformed input is
rejected. The original OpenCPN plugin API/import ABI remains unchanged.

The plugin independently verifies compiled resource hashes, constructs its own
library with strict XML/atlas validation and no CWD override, keeps stock CSV
registrar paths, and falls back as a complete resource set. Fonts and soundings
use bounded ports of the same semantic/default-preference rules as core ENC.
They do not rewrite persisted font or mariner settings.

The copied status export distinguishes unbound, bound but awaiting renderer
initialization, verified SKAGER selection, and Standard fallback. Reading status
does not initialize anything. Host diagnostics query only a currently loaded
matching module and retain no plugin/chart pointer after its lifetime. A bound
module alone is never reported as a successfully initialized renderer.

The separate `skager_chart_point_style_v1` copied observation leaves both existing
v1 layouts and reserved fields unchanged. It reads the actual initialized private
library's effective table on the application thread, gated by Init completion,
selected presentation, `m_bOK` and non-null renderer. Init entry, DeInit and plugin
destruction revoke observation. In this pinned source the sole renderer delete is
failed initialization, immediately followed by clearing its pointer on the same
thread. No renderer address or function pointer is retained by diagnostics or
passed across the DLL boundary. The host also requires the matching currently
loaded container to remain initialized. Unavailable states omit the effective table.

Diagnostic JSON explicitly scopes core values to `chart_presentation.core`
(`available`, `saved_point_style`, `effective_point_style`) and the independent
private observation to `chart_presentation.private_ocharts` (`available` and,
only when observed, `effective_point_style`). Requested presentation and existing
initialization status remain separate. Neither core state nor selected status is
used to manufacture private table data. Native exported-call/lifecycle and real
chart observations remain required after this source change.

## Required evidence

Acceptance still needs native Win32 imports and export/ABI checks, binding and
tamper refusals, actual private software/OpenGL drawing, all three palettes,
Standard/Legacy/Safe recovery, normal plugin reload/unload and original helper
operation. Existing shop/authentication and TLS behavior must remain qualified;
the wrapper cannot silently replace them with a nonequivalent fallback API.
Corresponding source and notices must cover all redistributed open components.
The real licensed boat charts must be reviewed privately at the actual display.
No decrypted chart, licence, credential or closed helper enters Git or evidence.
