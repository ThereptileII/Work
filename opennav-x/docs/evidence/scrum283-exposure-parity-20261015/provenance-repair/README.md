# SCRUM-283 fixture provenance repair

Helper revision: `da19af1cb903bec98ed38c870336b365f045a59d`, based on the
earlier exposure helper. Application/runtime source remains
`1ffd09ddf2fa6588def2907485f6e14e89bc3a7d`; no runtime file changed.

Review found two blockers in the earlier fixture provenance. The patched source
check accepted additional files: an injected `libs/s52plib/src/bbox.h` could
shadow the pinned geoprim header through a quoted include. The corrected private
header also included three owned headers that were absent from the helper receipt:
`ChartLightLabel.h`, `ChartNameText.h`, and `ChartNameSpacing.h`.

The helper now compares the complete patched file path/hash inventory against
the reproduced tree, and rechecks the original and patched inventories after
execution. It rejects symlinks and Windows reparse attributes on each path and
ancestor before resolving them. All three owned headers are hashed before and
after the run. The compiler consumes copied, individually checked fixture and
header files from a closed staging directory, with no original source or broad
repository include directories. Each stage has its own before/after inventory.
Adding an owned include outside this declared closure now fails compilation.

The exact committed Linux replay passed: 398 checks each for core, original
private and adapter-disabled private; 416 for corrected private. The 26 baseline
and 288 exposure records still agree with core after the retained Wk exception.
All 93 original/corrected differences and the exit-1 required-repair negative
remain. New input negatives cover shadow headers, same-byte symlinks at file,
directory and source-root levels, late path additions, and individual changes
to all three owned headers. Original source-tamper and disabled-assertion
negatives also remain covered. See the [replay](input-negative-replay.md) and
`input-negatives.json`.

Independent Codex review found no remaining blocker in these two repairs and
matched all four staged inventories to the retained compiler receipts. This is
fixture provenance and Linux component evidence. Windows reparse behavior,
MSVC execution, real area queries, private-DLL runtime, software/OpenGL charts,
and boat acceptance remain unrun/unpassed here.

For the next focused native lane, the actual external dependency is wx plus
MSVC/Windows SDK/Win32 C++ runtime. The existing `fetch_sources` and
`apply_patches` functions can prepare original/derived source without calling
the product `prepare()` entry point. Existing pinned-archive fetching and
verification can authenticate the three `windows-wx.lock.json` archives into
a fresh fixture directory. Record a fixture-only wx inventory and compiler/
runtime identity; do not call that a production preparation or dependency
producer receipt. No maintained TLS bundle, TLS source checks, chart resources,
MSYS installation, CMake application build or SDK fingerprint change is needed.
