# Immutable prototype tooling

Install `requirements.txt` in a dedicated Python environment, then:

```
python -m playwright install chromium
python tools/prototype/test_contract.py
python tools/prototype/render.py --output evidence/local/prototype-reference
```

The supplied 113 files are verified byte-for-byte before/after every render.
No source rewriting, network requests, device connections or application launch
occurs. Each state uses a new browser context and real prototype interactions.
The fixed browser clock/reduced-motion setting removes temporal variation.

`capture.json` contains the input hash, browser/Playwright versions, geometry,
computed styles, actual platform fonts, visible actions and PNG hashes. The
Linux set uses Liberation Sans via the prototype fallback stack on this host;
it is not Windows typography acceptance. Windows must render its own reference
with the same installed font environment used by native XNav.

The prototype contains intentionally illustrative data. These files are design
and CI resources only, not installed product resources or a new Demo mode.
Do not execute `prototype/build.mjs` in the immutable evidence directory.

`capture-ais-component.py --client <ais_drawer_test> --output <new-directory>`
runs the dedicated non-installed widget executable, retaining real screen pixels,
interaction results, exact executable hash and drawer reference/current/diff.
The entire test desktop is labelled as an offline synthetic component fixture;
there is no chart/network/hardware integration in this executable. The wrapper
requires a disposable CI desktop on Windows and uses isolated Xvfb on Linux.
Painted-content checks fail black/missing captures. Comparison crops the exact
398×674 drawer for component review; it does not waive full-product comparisons
or fabricate prototype references for supplemental online failure states.

Use `capture-ais-component.py --component passage --client <passage_drawer_test>
--output <new-directory>` for the separate populated Passage widget test. It
verifies five real-screen captures and preserves component reference/current/
diff images. Fixed fixture time is confined to that non-installed test process.
The product capture also checks the Passage theme cycle and chart return.

Use `--component instruments --client <instrument_panel_test>` for the native
wind/heading and instrument grid. It retains five captures, including loss of
heading and stale sensors. Component comparison uses the exact 1014×566 full
view; the original horizon space is reserved. It checks painted heading ink
as well as geometry, so a misplaced drawing transform cannot pass on bounds
alone. Production captures exercise all three themes and Close with real
pointer input; unavailable vessel input stays unavailable.

Additional `render.py --scale 1.25` and `--scale 1.5` captures measure the
original responsive CSS in a physical 1280×800 image. Their logical viewports
are 1024×640 and 853×533, respectively. DPR1 remains the default canonical
reference; no HTML or CSS is rewritten for these extra states.

`capture-dpi-windows.py` is a separate development probe of the fixture-free
application at actual 100/125/150% Windows scaling. It checks GetDpiForWindow,
keeps a 1280×800 client, requires all eight navigation controls and four primary
readings to fit, and exercises Settings/theme/Close with native injected touch.
It captures failure evidence and restores the original disposable desktop DPI.
It cannot run outside native GitHub CI and does not modify the boat's display.
This adds coverage; it does not replace the existing full release DPI, alarm,
mode lifecycle or physical touch gates. No scale is accepted just because
Windows agreed to change its DPI setting.
