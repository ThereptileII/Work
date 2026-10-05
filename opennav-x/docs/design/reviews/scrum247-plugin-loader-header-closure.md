# SCRUM-247: plugin-loader native preflight header closure

Run `37096431092` failed compiling the newly included actual
`model/src/plugin_loader.cpp`: `model/plugin_handler.h:38` includes `archive.h`,
which the focused SDK did not provide. This was a preflight setup failure,
not a full application build result.

Production's pinned OpenCPN `buildwin/win_deps.bat:51–66` selects
OCPNWindowsCoreBuildSupport v0.5. Root CMake adds `cache/buildwin` and
`cache/buildwin/include` to its package search at 1348–1350;
`cmake/FindOcpnLibarchive.cmake:75–77` exports the selected include directory.
`model/CMakeLists.txt:474` consumes that dependency. The support tag's immutable
commit `e90cc5842b02d5502a549f0f90424e3e1614bf67` is already pinned for the
production GLEW header in `windows-prototype-headers.lock.json`.

The fix locks that same commit's `buildwin/include/archive.h` and
`archive_entry.h` (libarchive 3.3.2), fetches only those files into the focused
SDK's `libarchive` directory, and includes that directory. Their exact sizes and
SHA256 values are in `tools/windows-chart-headers.lock.json`. Existing SDK header
receipts and post-compile drift checks automatically cover both files. No
library, binary support archive, Android header, substitute declaration, compiler
macro workaround, or application source change is introduced.

The complete unit graph was checked beyond the first failing include:

- Actual completed Linux model object dependency record: 666 dependencies,
  read using Ninja's read-only dependency tool in the independent
  `scrum259-linux-6dafd29` build. Its relevant patch files are byte-identical
  between `6dafd29` and this fix's base `4e2715088361b0068e920f7744b49ab68ab439e1`.
- Non-system closure: the model's base platform, catalog, configuration, command
  line, logger, utilities, blacklist/cache/handler/loader/paths/safe-mode/version
  headers; observable headers; `ocpn_plugin.h`; filesystem wrapper; owned
  `PluginPresentationFallback.h` and `PluginPresentationLoader.h`. These already
  resolve through the preflight's real production include directories.
- The only additional external non-wx public header in that consumed graph is
  `archive.h`. The exact support headers depend on normal CRT types and, for
  Windows archive entries, `windows.h`. Their `android_lf.h` includes are
  strictly Android-only. No extra generated libarchive configuration is needed.
- Windows-specific unit includes are `Psapi.h` and the existing wx Windows
  wrapper. Linux ELF/wordexp/cxxabi and Android includes are gated out. The
  actual unit does not include GUI `pluginmanager.h`; its mention in the model
  loader header is documentation, not an additional header edge.

Focused verification passed: all 7 existing preflight guard tests, Python
syntax, exact fetched header size/hash checks, and the real production
`plugin_loader.cpp` syntax-only command with the two locked public headers first
in its include search. That last check used the actual Ninja model command
(the exported compile database omits this object target), retained its Linux
platform flags, and wrote only private scratch diagnostics. It does not claim
a Windows compile. The patched unit's source SHA256 was
`f5be09a3914d05a4dfc6ce6770a311b0048d35e75f2c872318b3e8e955ef4781`.

The corrected native 23-unit run remains required. No CI was dispatched, no
application linked, and no frozen source, existing object, profile or boat state
was modified by this inspection/check.
