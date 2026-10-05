# SCRUM-271 native notification compile-only evidence

[Run 37132173264](https://github.com/ThereptileII/Work/actions/runs/37132173264)
passed on its first attempt at published revision
`cc9c277c92b77ae44035295541539f28a4ef80bd`, mapped from local
`7c80fc045c891fcd3af2e58e07f7271bda7f2801`. The current full-application
candidate remains separate and unchanged.

The retained `artifact.zip` is the original downloaded artifact **11277387193**:
1,437,939 bytes; SHA-256
`1e6394a6aed96c46885633d41dd1cfc30389c5d57cb03ed419e72fae76d75800`.
Its length/hash match GitHub's artifact metadata. `audit.json` records the
independent inspection; original run/job metadata and decoded logs are retained.

The actual compiler log and generated project show exactly one full production
source, `gui/src/notification_manager_gui.cpp`, compiled using native MSVC
14.44.35207 HostX64/x86 in Release/Win32 with `OPENNAV_X`, GL/GLSL and production
fixtures/loopback disabled. There are zero compiler warnings or errors. The
single native x86 COFF object has 417,479 bytes and SHA-256
`9d68d6ae91c6e26fb8dc94cd369b40a9115a0fb3c747925da69e3b213e9c9d28`.
MSVC's object target also creates a single-object archive with `Lib.exe`;
there is no application link, substitute-source fixture or application runtime.

Independent input audit checked all 490 local inputs, reconstructed all 1,429
prepared upstream inputs from the pinned commit and exact nine-patch series,
and checked all 1,560 SDK header hashes against independently extracted locked
archives and their production patch series. Windows checkout CRLF conversions
were accounted for explicitly. All seven retained generated-resource hashes and
both generated-config hashes match the report. The actual compiler read log
closes over 53 upstream inputs, 315 SDK headers, eight local inputs and one
generated config, plus 317 native compiler/Windows dependencies. Both new
notification headers were actually consumed. No dependency library was built
for this independent audit.

The workflow skipped unrelated runtime, geometry, capture and reference steps.
The original 23-unit gate was not run. This is **native compile-only evidence**:
application link/runtime, real SW/GL pixels, severity/count/click/acknowledgement,
theme return, 1280×800 and 1920×1080 placement, 100/125/150% DPI hit bounds and
boat display/touch/GPU acceptance remain open. No new source change, rerun,
push, root integration or boat access occurred during this evidence audit.
