# Combined 33d90c3 Linux integration and actual IHO failure

Exact application source `33d90c38cd1a628c21d0ca159647012d98649d2e` built and installed successfully. All 147 integrated upstream CTest cases passed in 23.77 seconds. This batch does **not** include the separate chart-resource Python/private-preparation suites. The generated and installed resource manifest matches the independent combined seal `ffa75c9d097a143e58aa13f9300b04bebac0836ab849bd4284401df1866f4b2a`.

A btrfs reflink copy reused the qualified d5 cache through a private mount namespace. `snapshot.json`, `run-build.sh` and `summary.json` record the physical/virtual mapping, independent Git metadata and frozen file identities. The original d5 source pointers, cache and built/installed binaries retained their hashes and mtimes. Only the private copy changed. The exact nine-patch advancement changed `s52plib.cpp` and `s52plib.h`; Ninja completed 66 incremental steps. No old d5 rebuild, CI, boat or hardware action occurred.

## Actual render is blocked

The retained official IHO S-64 cell is test geography, not an operational nautical ENC. Bounded entry probes observed actual BOYSPP254/Index253 at the source coordinates, original `BOYSPP11` lookup and `XNSPPY01` raster painter with actual table76. TOPMAR257/Index256 entered both normal object rendering and the new topmark guard but produced no `XNSPPT01` painter call. The bounded software capture failed; OpenGL and theme acceptance were not attempted afterward.

Two narrow observations retained that failure and identified its cause: actual RCID31314 has empty attributes, null rule list, presentation enabled, and a **one-character U+001F instruction**. The pinned loader appends `\037` even to empty XML instructions (`chartsymbols.cpp:235-236`, copied into the LUP at491). `YellowEmptyInstruction` at `ChartYellowBuoySymbol.h:68-69` requires zero length. It rejects the real loaded empty slot before platform uniqueness is checked. Normal visibility was not bypassed. No product change or rebuild was made to manufacture a passing capture.

Actual diagnostics also confirm the new `core` saved/effective table76 fields and explicitly unavailable `private_ocharts`. This Linux build has no packaged private adapter; it does not prove private native initialization or observation.

The later collector must account for OpenCPN's observed scale constraint (requested0.6, actual0.5826126536), while preserving symbol/visibility checks. The retained collectors include original, boundary-probe and final copied-value versions. The final diagnostic run intentionally remains failed, with screenshot and actual diagnostics. No Day/Dusk/Night/Day-return success is claimed.

Native Windows, packaged private DLL/host and boat gates remain pending. Keep SCRUM-264 In Progress until the real initialized empty-slot contract and actual canvas proof pass.
