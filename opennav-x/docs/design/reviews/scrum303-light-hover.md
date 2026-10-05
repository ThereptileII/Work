# SCRUM-303: chart-light hover palette

Pinned OpenCPN builds `gui/src/s57chart.cpp`; its extended-light hover sectors
used fixed red/green/yellow and black line colors. Paint-time mapping now uses
the prototype sector and boundary roles in both software and GL paths, only
with verified active XNav presentation. Standard/Legacy, unknown colors/schemes,
feature selection, bearings, range, geometry and leading-sector behavior remain
unchanged. Red/green/white sector meaning is retained. Current theme resolves
opacity so a retained hover cannot keep its earlier bright palette.

Focused palette tests pass and the actual s57chart translation unit compiles.
The separate selected-object popup is handled by SCRUM-309. Exact identity of
the originally observed boat feature is unproven; native/boat screenshots are
still required for final acceptance.
