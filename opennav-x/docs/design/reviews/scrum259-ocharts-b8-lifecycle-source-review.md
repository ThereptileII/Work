# SCRUM-259: bounded adapter lifecycle source review

This review covers application source `d5d71356d806ea8c3518644d10728a24f1334d1d`,
mapped by the candidate owner to `b8cfbf809450208f723095ffb4e00d7800b619a5`.
It closes the incremental **source inspection**, not native or boat acceptance.
No application, plugin, helper, test or boat command was run. The packaged DLL,
generated resource header, dependency outputs and native runtime remain separate
evidence. All reviewed input hashes are in
[the receipt](../../evidence/scrum259-ocharts-b8-lifecycle-source-review.json).

## Exact comparison

Reconstructed all 219 locked source/API files from existing local cached Git
blobs, checking each blob identity and byte count. Applied the two current
patches using the production preparation function to a derived local copy,
with its documented CRLF normalization. This is a source reconstruction only,
not a complete prepared SDK or a compiled package. Eleven upstream files change:
seven private S-52 files, two wxCurl files, `eSENCChart.cpp` and `o-charts_pi.cpp`.
The receipt lists every changed file and original/derived hashes, all owned
build inputs and the supplemental host-loader inputs.

The original plugin source SHA-256 is
`54056c0c8d6d7d32839a0aa0bff63bcfd9a117a90c81ef0e2968eed4e477652c`, exactly
matching the retained vendor source review. The derived plugin source is
`1ad2286fc4d41541e043626716da980b91bf911fd4ecb4fe8a153ed7ac4feb68`.
An exact whole-file comparison proves only an added include and one replaced
renderer-construction expression in that file. Original source line references
below refer to pinned `c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8`.

| Boundary | Source finding |
| --- | --- |
| Constructor/destructor, `src/o-charts_pi.cpp:480–556` | Bodies unchanged: icons/config/EULA handling and possible private-directory creation remain; destructor remains empty. This was never a claim of zero filesystem writes. |
| `Init`, lines 558–769 | Body unchanged, including enabled-plugin checks, chart registration, helper discovery and dongle query. Its call to `init_S52Library` now reaches changed presentation construction. Therefore startup behavior is not wholly unchanged. |
| Renderer initialization, lines 2630–2710 | Only the construction at original line 2656 is replaced; stock CSV registrar paths and following config/palette setup remain. New `ChartPresentationAdapter.cpp:63–88` reads/copies binding state, hashes resources before and after construction, validates XML/PNG atlases, selects display policies/font resolvers or falls back to stock resources. Bound fallback refuses CWD override; unbound retains upstream CWD policy. Main-thread and verification failures cannot report selected presentation. |
| Messaging, overlays, timers and idle | Original plugin message/overlay/color-scheme bodies and event registrations are unchanged. The timer at lines 3623–3660 has Android-only callback work; no timer/idle/network worker is added by owned binding code. Existing callbacks reach changed S-52 drawing and synchronous font resolver callbacks. These change presentation, not navigation output. |
| Helper startup | `Init` retains `GetPlugInPath(this)` and original same-directory helper resolution, with existing PATH fallback if absent. Unchanged `fpr.cpp:75–110` executes the licensing/dongle `-s` query; unchanged `validate_SENC_server` at lines 2819–3033 may start the existing decode helper with a process-derived pipe name. Missing or changed helper/dependency identity must still block the fresh commissioning review. |
| Shutdown | `DeInit:771–790` still saves configuration, removes its options page, clears chart classes and invokes unchanged `shutdown_SENC_server:3035–3049`. Windows `Osenc.cpp:630–738` still sends `CMD_EXIT` using the existing local named pipe, with its existing wait behavior. No new helper stop/force-kill path. Stream SHA-256 remains `6ce66dc6df9c5c066a0533a27288f9b9e8a2bb2ebb870b46fb1bde8cb4ff91b4`. |
| Actuator/output paths | No added NMEA/comm-driver/actuator transmit call in either patch or the owned overlay closure. Existing helper IPC writes, configuration writes, chart cache/license behavior and driver management traffic must not be described as complete I/O silence. Closed helper/dongle trust remains the retained vendor boundary. |
| Networking | `ocharts-wxcurl-trust.patch` changes transfer readiness/failure handling, native CA trust, peer/host verification and redirects (maximum five; HTTPS stays HTTPS). The recipe substitutes maintained verified curl/zlib dependencies. These are real behavior/dependency changes, not covered by byte equality of plugin lifecycle functions. No additional transfer call is introduced; shop/fingerprint/download/license-changing actions remain excluded from the boat review. |

## New host/module boundary

`opencpn-5.12.4-chart-presentation.patch:2172–2195` changes the library-load
expression only; original discovery/container/factory identity remains.
`ChartPresentation.cpp:131–151` registers the hook for the SKAGER interface;
verified selected resources are required. Standard/Legacy do not select it;
Safe retains plugin suppression. `OChartsModuleLoader.cpp:106–169` requires the
main thread, exact original DLL, compiled adapter size/hash, local unredirected
files, actual adapter ABI compatibility, rehash/locks, bind and pending status.
`PluginPresentationFallback.h:11–42` permits original fallback only after a
rejected/prior module unload succeeds. These are new load/unload boundaries.

`BindingState.h:12–59` validates and copies one bounded request under a mutex;
bind/status do not call the plugin, renderer, helper or chart factory. It adds no
thread, timer or I/O worker. `ChartPresentationAdapter.cpp:96–103` catches
exceptions at the private C exports. The rendering callback and private object
lifetime still require the actual native host, including normal destruction and
DLL unload. Unchanged source is not identical generated code or dependency ABI.

## Evidence reuse and remaining proof

Reuse the hash-pinned original source/helper review and shutdown review for the
unchanged boundaries; their five evidence hashes are in the receipt. Do not
renew an old plan's timestamp and call it a new inspection. The adapter lives
beside the application and is absent from `*_pi.dll` commissioning discovery.
The retained o-charts plan entry therefore needs supplemental exact adapter,
resource/package/source and dependency evidence, tied to the installed build.
Its source revision can remain the pinned vendor revision while the supplemental
evidence explicitly identifies the application patches and derived source hash.
Keep the schema-2 shutdown entry's closed-helper limitation and stream hash.

Before launch: verify downloaded same-candidate DLL/import/source/resource
manifests against this input closure, installed owned bytes and native evidence.
The isolated native loader guard expressly uses harmless fixtures and does not
invoke the vendor factory; it cannot stand in for actual startup/idle/DeInit.
Then collect the guarded real-host startup/idle/chart status, normal helper
behavior, palette/mode transitions, normal shutdown and restored profile proof.
No claim here qualifies those observations or authorizes physical commands.
