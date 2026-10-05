# SCRUM-259: incremental private lifecycle source review

Reviewed application source `e1d01367692f7b8c866be369417e68c836c5d563` against
`d5d71356d806ea8c3518644d10728a24f1334d1d`. This supplements the
[earlier bounded source review](scrum259-ocharts-b8-lifecycle-source-review.md);
it is not runtime qualification, a launch authorization, or acceptance.
No build, test, application, plugin, helper, network service or boat command ran.

## Reconstructed source boundary

All 219 original source/API files were independently verified against the unchanged
Git blob/byte lock using the already-downloaded b8 corresponding-source archive.
The production preparation function applied both revisions' patches to separate
private derived copies. The reconstructed d5 hashes match the previous review.

Only three derived upstream files differ since d5: `src/o-charts_pi.cpp`,
`libs/s52plib/src/s52plib.cpp` and its header. Original files, both source pins and
all helper protocol files remain unchanged. The complete file/hash closure and
three exact diffs are in the [receipt](../../evidence/scrum259-ocharts-e1-lifecycle-source-review/receipt.json).

Removing exactly four `SetChartPresentationActive` calls from the new plugin
file recovers the complete d5 derived file byte-for-byte. The derived plugin hash
is `4ca4224a54b4cc1c2740fbdd10695ef883572827698ac448a2532666c01f8e4c`, matching the
existing SCRUM-267 diagnostic evidence. This updates the earlier statements that
Init, DeInit and destructor bodies were wholly unchanged.

| Actual derived boundary | Increment and consequence |
| --- | --- |
| `o-charts_pi.cpp:557`, destructor entry | Revokes copied observation before the original empty destructor completes. |
| `o-charts_pi.cpp:562`, Init entry | Revokes prior observation; original initialization, helper resolution, dongle query and registrations remain byte-identical. |
| `o-charts_pi.cpp:752`, immediately after `init_S52Library()` | Enables observation on the application thread. The observer additionally requires selected binding, a nonnull renderer and `m_bOK`; enabling activity is not proof of successful private chart initialization. |
| `o-charts_pi.cpp:777`, DeInit entry | Revokes observation before unchanged config save, options cleanup, chart-class clearing and helper shutdown. |
| Private observation implementation | `SetChartPresentationActive` only stores an atomic flag. The new export reads binding status and borrows `ps52plib` for one main-thread read of the existing const effective-style accessor. It copies bounded POD data and retains no renderer/function address. No initialization, preference change, timer, worker or helper call is added. |
| Host diagnostics | The existing diagnostic collection checks initialized plugin container, original path and currently loaded module handle before resolving the observation export. It validates the copied result. Core and private fields remain separate; no table is inferred from selected status. |
| Private S52 drawing | Adds the classified yellow body alias and explicit fitted-X topmark draw after existing object visibility and DC setup. Stable library-owned rules reach existing projection/raster rendering. Bounded, typed actual attributes, exact co-location and unique platform guards apply; no object-state or navigation-processing pass is added. |

The canonical empty-instruction update at e1 accepts only an empty string or the
single unit separator appended by the real S52 parsers. It does not accept an
arbitrary prefix or a populated rule. API-17 pointer ownership still requires a
nonnull instruction pointer. This is a render-eligibility correction, not a new
lifecycle or physical-output operation.

## Unchanged helper and operational boundaries

The full reconstructed `fpr.cpp`, `Osenc.cpp`, `oernc_inStream.cpp`, `ochartShop.cpp`
and plugin header hashes match both d5 and the prior receipt. The wxCurl trust
patch, binding/strict resource verification, original module selection and checked
fallback-unload mechanics are unchanged. The plugin constructor, helper startup,
license/shop callbacks, timers, overlays and shutdown bodies differ only by the
four observation calls just identified. No additional actuator/NMEA/device
transmit or helper command was identified in the reviewed delta.

The five retained vendor/shutdown review files were rehashed unchanged. Cached
`oexserverd.exe` remains `ec27c947fc9ba4ae09345961780b885deddc2e4fcf094805a14cdf19504e0adb`
(70,656 bytes), and cached `SglW32.dll` remains
`16c43e7bb4693be10af44e14873ae6f62cb3e07979f53c0ce2d63ba17c8ee655`
(76,288 bytes). These are read-only cached binary checks, not a fresh boat inventory
or closed-helper source review. No original helper is included or modified by the
adapter source-only build closure.

Existing helper named-pipe commands, dongle interactions, configuration writes,
chart caches, licensing and network behavior remain real behavior. This review
makes no claim of complete I/O silence or electrically silent USB.

## Required future binding

The private ABI now has **five** exact exports, adding the copied point-style
observation while preserving existing binding/status layouts. Updated recipe and
PE validators require that exact inventory. The old four-export b8 DLL/source
receipt cannot qualify a rebuilt five-export adapter.

Tie the final application source to a fresh downloaded adapter/package/resource
and maintained-dependency closure before the guarded real-host startup/idle/chart
and normal shutdown review. Verify the installed original helper/runtime identity
again and retain restored profile evidence. New generic/yellow artwork and parser
eligibility require their separate real-canvas evidence; this source review does
not substitute for software/GL, private encrypted-chart or native Windows proof.
Final mapped commit and produced DLL hashes remain to be attached by the candidate
owner after the source freeze; no source or runtime approval is implied here.
