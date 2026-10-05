# Native Win32 chart compile preflight: seventeen units

[Run 37085103227 / job 111093615789](https://github.com/ThereptileII/Work/actions/runs/37085103227/job/111093615789)
passed for remote `9a4231f45d978321c612a6a1de66d735d531bbaa`, mapped to local
`f0976cc65ea63d3ed6f60ac53ec38a856d11b66b`.

Downloaded artifact **11259832267** is **4,743,383 bytes**, SHA-256
`072eebb0bc27f8616d063c4c16e67889d4f25815417b82cceb4be079802c5f3e`,
matching GitHub's recorded digest. All **407 ZIP entries** pass CRC verification.
The first host download returned HTTP403; a fresh connector URL with the usual
browser User-Agent succeeded. No authentication or approval workaround was used.

Independent inspection verified all **17 expected objects**, exact recorded
hashes and COFF machine `0x014c` (x86), seventeen actual MSVC 14.44.35207
HostX64/x86 compile commands, and seventeen successful target builds. This
includes `ChartRouteLabel`, `glChartCanvas`, `s52plib` and `DepthFont`. All
seventeen archived production source units match their declared identities.

All **308 local inputs** match the exact local candidate (21 exact byte matches,
287 Git CRLF-only matches). All **1,438 upstream inputs** match an independently
reconstructed pinned source with the nine reviewed patches (16 exact, 1,422
CRLF-only). The resulting upstream tree is
`b55b3bf72da3690ac2724e6663568d75cad12723`. Immutable prototype bytes, patches and
the LF-attributed embedded brand asset remain exact. The local input set includes
`ChartRouteLabel.cpp`, `chart_service_art.py` and all five new service artwork
assets/provenance files. Their individual identities are retained in the receipt.

Seven generated chart-resource files, two configuration headers and seventeen
MSVC projects were independently rehashed against the artifact manifests. A
separate offline regeneration from the exact local candidate and independently
reconstructed pinned resource bytes matched **all seven generated files byte
for byte**, including XML, three PNG sheets, resource header and manifest.
This covers the current Night palette, land fill and pilot/radar glyph resources.

This establishes **compilation only**. The policy has fixtures and pilot loopback
disabled, GL compilation enabled, no dependency builds and no application link;
`nativeProductAcceptance` remains false. No GUI, GL/software rendering, DPI,
installer, boat, dependency-producer or full-candidate acceptance is implied.
The 1,558 SDK header identities remain in the original artifact manifest; those
headers were not uploaded and could not be independently rehashed.

No workflow was dispatched, rerun, cancelled or pushed. The audit performed
source reconstruction, archive/identity checks and offline resource regeneration;
it did not compile or launch an application. SCRUM-247 remains Testing.
Exact object/source/generated identities, compile-log hashes and limits are in
[verification.json](verification.json).
