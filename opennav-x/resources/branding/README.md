# Approved SKAGER artwork

`skager-wordmark-approved.png` is the unchanged user-supplied SCRUM-89
attachment 10000, approved for software branding. Its SHA-256 is
`dce92b90a8657d045403cf4508d8220cb0306e6a251d58fbf40e6084ea3ee4ee`.
The recorded source is 1,206,475 bytes, 1447 × 1087 pixels. Approval provenance:
[SCRUM-89](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-89);
implementation: SCRUM-236. No artist attribution or trademark clearance is
inferred from supplying this attachment. Domain/legal clearance remains separate.

The software wordmark is a pixel-exact rectangular crop of the original. It
retains the off-white SKAGER lettering, the A without a crossbar, mint APP and
the original deep-teal background. The Windows icon is a square crop of the same
source, resized with Lanczos into 16, 20, 24, 32, 40, 48, 64, 128 and 256 pixel
32-bit DIB entries. No font substitution, retracing, new boat/compass symbol or
invented monogram is used. Small icon sizes preserve the full approved wordmark;
text legibility at 16 pixels necessarily remains limited.

Regenerate with `python tools/generate-skager-brand.py` using Pillow 12.3.0;
`--check` compares every resulting byte. `tools/verify-skager-brand.py` verifies
committed source/output hashes and ICO entries without an imaging dependency.
`provenance.json` records crop coordinates, tool version and output hashes.
Normal native builds and installer packaging verify the committed assets.

The integrated Windows application uses the SKAGER icon and product description;
its internal executable name and OpenCPN version identity remain unchanged.
Stock OpenCPN and Legacy's upstream attribution/about/licensing are preserved.
The NSIS setup and its generated maintenance/uninstall executable share this ICO.
Native Explorer/taskbar/shortcut, setup and maintenance rendering at Windows
96/120/144 DPI remain acceptance gates for the integrated exact commit.
