# Native wordmark compositing — SCRUM-235/236

Bounded follow-up to frozen local source
`1356fd1603aacbea04d7081d16331e9a181180bb`, kept in a separate private worktree.
No application build, CI dispatch, boat connection, or frozen-cache write occurred.

The approved original, exact crop, embedded crop header and Windows ICO remain
byte-identical. `tools/verify-skager-brand.py` passed. The artifact guard and
raster derivation are documented in `resources/branding/README.md`.

`native-component.png` is the actual wxMemoryDC/wxBitmap drawing output of the
production helper, composited over the real header colors at the shell's existing
180 × 68 DIP footprint and 148 DIP logo width. Columns are Day/Dusk/Night; rows
are 100/125/150% device sizes. Visual inspection confirms no teal rectangle,
unaltered six SKAGER and three APP glyphs, the crossbar-free A, and quieter Night
luminance. APP remains small at the existing 100% size but legible in the fixture.

Focused checks verify all six plus three separated letter columns at each size,
transparent margins and row separation, exact header pixels wherever alpha is
zero, expected theme ink without old teal fringes, substantial per-row coverage,
cache reuse and invalidation, unchanged decoded source, and original-image
fallback for altered identity, dimensions, or alpha. The weakest brightest-core
contrast measured is 5.56:1 (Night APP at 100%); this is a useful raster regression
measurement, not a claim that every antialiased edge meets a contrast threshold.
Full successful output is `component.log` (129262 assertions, nine drawings).

The actual changed `src/ui/Shell.cpp` also compiled separately against the local
wxWidgets headers. The fixture compiled with C++17, `-Wall -Wextra -Werror`.
A negative control substitutes the old opaque-image path by disabling the
coverage branch in a temporary copied helper. It exits 1 with the explicit
missing-coverage failure in `negative.log`. The first draft fixture assumed alpha
before checking it and its negative control crashed; adding the alpha precondition
fixed the fixture, and the retained final control fails normally. Production
compositing passed before and after that test-only repair.

Reproduce the bounded fixture with wxWidgets core/base development flags:

```
c++ -std=c++17 -Wall -Wextra -Werror -Isrc $(wx-config --cxxflags) tests/skager_wordmark_test.cpp -o skager_wordmark_test $(wx-config --libs core,base)
./skager_wordmark_test native-component.png
python3 tools/verify-skager-brand.py
```

Alternatively build only the `skager_wordmark_test` CMake target with UI component
and test options enabled. This evidence uses Linux GTK wxWidgets with explicit
pixel sizes, not an integrated Shell screenshot or Windows per-monitor DPI test.
Runtime coverage is an approximation from the approved flattened RGB raster; the
original attachment contains no recoverable exact alpha and no approved vector
or transparent replacement was found in the inspected Jira attachments. Native
Windows integrated and release/installer gates remain open.
