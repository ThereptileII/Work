# Final native Win32 chart compile preflight: seventeen units

[Run 37088424481 / job 111103371331](https://github.com/ThereptileII/Work/actions/runs/37088424481/job/111103371331)
passed for remote `61a0a7838b56ad841bb458af6fc62651464bdafe`, tree
`16b058983cad3955ddf04dd2151f9f4167ce33f2`, mapped to local
`78eccb8b7f21b260ded57d3ba763f884d60c8180`.

Independently downloaded artifact **11260809354** is **4,750,589 bytes**,
SHA-256 `8ef242d0b14a7376cd28c9081c35a7b5476642cb4701ec946447641a2dfeca44`,
matching GitHub's digest. All **407 ZIP entries** passed CRC verification.
The download used the connector's fresh signed URL; no credentials or temporary
signed URLs are retained here.

All **17 expected objects** match their recorded sizes/hashes and COFF machine
`0x014c` (x86). The archived command names exactly those seventeen targets;
the log contains seventeen actual MSVC 14.44.35207 HostX64/x86 compile commands
and seventeen successful target builds. All seventeen archived production
source units match their declared input identities, including `ChartRouteLabel`,
`glChartCanvas`, `s52plib` and `DepthFont`.

All **335 local inputs** match the exact mapped commit (22 byte-exact, 313
Git CRLF-only matches). All **1,438 upstream inputs** match an independent
reconstruction of the pinned source plus nine reviewed patches (16 exact,
1,422 CRLF-only). Reconstructed upstream tree:
`20d5c9bdd59332fd6bf58a8b64a4dae3b5f4683c`.
This includes the route-label helper, service artwork/helper, cardinal artwork/
helper, chart resources and applicable immutable source/patch bytes.

Seven generated resources were independently regenerated from exact candidate
source and reconstructed pinned stock resources: XML, three PNG atlases,
S52RAZDS.RLE, manifest and generated header all match the Windows artifact
**byte for byte**. Two generated configuration headers and seventeen MSVC
projects also match their recorded hashes; the projects retain production
fixture/loopback-off definitions and OpenGL compilation.

This is **compile-only evidence**: no dependency build, application link, GUI,
rendering, DPI, installer or boat acceptance. `nativeProductAcceptance` remains
false. The 1,558 SDK header identities are runner observations in the original
artifact manifest; their bytes were not uploaded for independent rehash.
SCRUM-247 remains Testing. No workflow was dispatched, rerun, cancelled or pushed;
no full candidate was published or promoted by this audit.

[verification.json](verification.json) retains source/object/generated identities,
GitHub metadata and limits. [archive-files.json](archive-files.json) records every
archive entry's actual SHA-256, size and CRC. `raw/` retains the original bounded
prepare, configure, compile, resource-generation and verification commands/logs.
The complete artifact remains the referenced GitHub artifact; the local audit did
not compile or launch an application.
