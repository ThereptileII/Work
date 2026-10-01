# 1920×1080 native display qualification (SCRUM-213)

The immutable HTML at 1280×800 remains the primary visual reference. The
additional 1920×1080 target is a physical Windows desktop at 100% scale. A
large frame must keep the same XNav navigation hierarchy, readable rail,
critical alert and usable panels without changing the chart or marine data.

`tools/smoke-dpi-windows.py` already requires a real 1920×1080 disposable
desktop, runs the application at 100%, 125% and 150% DPI in a 1280×800 frame,
and checks return from fullscreen. At 100% it now exercises the actual
1920×1080 fullscreen frame: paired native/diagnostic chart hit geometry,
navigation controls and footer, four rail values, a visible and uncovered
critical alert, and native Passage, Traffic, Settings and Alerts panels. It
captures visible screen pixels for navigation and each panel. The existing
return path then checks the 1280×800 buttons and rail; the 125% and 150%
cycles remain intact. The large-desktop step does not repeat the full DPI,
theme or mode-switch matrix.

After the next exact-commit Windows run, review `evidence/local/dpi-results.json`
and the five `evidence/local/dpi-100-1920-*.png` captures. Inspect chart
dominance and coastline, rail and alert legibility, panel placement and text,
and any OS-specific clipping or empty area. Compare the restored 1280×800
capture and `restored_rail` record with the primary reference. A passing
geometry assertion or screenshot file alone is not visual acceptance.

Current status: source-only test increment. Native Windows execution,
human review of its actual captures, and the physical boat display check
remain open. This test does not inspect connections, send pilot commands or
change the boat display.
