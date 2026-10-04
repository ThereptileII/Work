# Font, small logo and customer identity: bounded read-only review

Reviewed root `33f9fd1dc72bd894bea956d14897af358fe3291f` and the original native 4dd artifact identified below. No new normal-product OpenNav-X/XNav branding defect was found in the bounded quoted-string/source review. This is not whole-product visual or release acceptance.

## Font policy and actual evidence

- Immutable `docs/design/prototype/src/style.css:4` defines `"Segoe UI Variable Display", "Segoe UI", Arial, sans-serif`. `src/ui/Controls.cpp:157–176` uses that installed-face order, numeric weight and Windows once-scaled CSS-em pixel height. Linux uses the documented Pango point conversion; Linux appearance cannot prove Windows face resolution.
- Explicit chart labels use Segoe UI/Arial: `src/integration/ChartTextFace.h:7–20`. Ordinary factory chart text receives that face in both core/private presentation; existing custom/role guards remain. `ChartPresentation.cpp:63–88` gives land 12 CSS px with tracking 1, water 16 px italic/root stack with tracking 5 and opacity 92/255, and eligible generated light descriptions 8 px with tracking .12. These roles are distinct from the general UI stack. Final vector depth CSS is 10 px; Georgia belongs only to the prototype raster-mode selector, not every sounding.
- Original 4dd `evidence/local/windows-native-output.log:5425–5432` reports Variable Display unavailable, Segoe UI installed, GDI selected/resolved Segoe UI for UI 11/23/48 px and ordinary-chart policy at 96 DPI. This is an actual native resolver fixture, not a trace of every live chart label. The earlier native-154 review agrees, but the fresh 4dd log is the evidence used here.
- Existing `docs/evidence/scrum263-boat-html-reference-5884701/README.md` records same-machine Edge CDP: UI/water/vector-depth resolve to Segoe UI Variable Display; chart land to Segoe UI, 12 px; the sampled dual-class landmark to Segoe UI Semibold, 8 px/600. Its `.chart-symbol-label` query selected the same landmark, so it does not establish a normal-only light-description face/weight. That evidence is the immutable original HTML with its original logo/data, not a native SKAGER screen. Actual native boat face/hierarchy acceptance remains open; no new boat operation occurred.

## Small approved logo and icon

`src/ui/SkagerWordmark.h:19–24` retains the approved 680×214 raster, renders 124 DIP wide at left 28 DIP inside the unchanged 180×68 DIP slot; `src/ui/Shell.cpp:108–135` preserves aspect ratio (39 physical pixels high at 96 DPI), centers vertically and uses guarded theme coverage. This is the selected smaller Jira logo, not the immutable prototype's old wordmark.

`resources/branding/provenance.json` locks the approved source and nine-size icon. `src/integration/OpenCPN.cmake:239–255` verifies those bytes and wires `Skager.rc.in`; its resource ID 0 carries skager.ico and lines 14–21 use SKAGER product/version metadata while preserving opencpn.exe ABI/file identity. Icon SHA256 is `33867d3f267603668ebc373a1ebf83565b2729817317f3905b4ebe1dc805c1d0`. The original Windows log line 2373 confirms approved source/embedded bytes/nine icon sizes, and line 5424 records 189,932 wordmark checks plus 18 native theme/size drawings.

Personally inspected original artifact PNGs `evidence/local/dpi-100-01-navigation-day.png`, `dpi-150-02-navigation-night.png`, and `dpi-100-night-settings.png`: small SKAGER App wordmark is present, unclipped and correctly placed; the frame icon/title are SKAGER. DPI receipt records actual native 96/120/144 DPI and a pass requiring visual review. These are integrated fixture-enabled application captures, not final fixture-free installer/boat proof. Full-resolution originals are retained in the original archive; selected byte-identical copies are in the isolated review's ignored `.local/identity-review` directory.

## Remaining names and limits

Current normal identity comes from `src/application/Brand.h:6–12`. Drawer/floating surface captions translate historical stable `OpenNav ` selectors through `src/ui/Branding.h:8–11`; `Drawer.cpp:17–21` retains SetName solely as the established selector. `installer/windows/AlphaSetup.nsi:8–17,37–46` exposes SKAGER, and `Lifecycle.ps1:388–425,815` selects SKAGER captions/shortcuts for new owned generations. Old names there are explicit historical-generation rollback/migration branches; protocol magic, persisted `XNav` chart-style values, diagnostic/internal paths and immutable prototype/legal records were excluded. This agrees with the existing `docs/design/reviews/scrum236-final-copy-audit.md`; no actionable normal-product stale-name exception was found, rather than asserting every possible translated/third-party string is audited.

Ordinary GL text-cache consistency is separate from font-family choice and is outside this bounded review. No new typography policy change is proposed by this audit. No additional tests, builds, renderings, application launches or CI/boat actions occurred. The failed 4dd candidate does not qualify a production package or installer icon/shortcut deployment.

## Original evidence identity

Run 37155858878 / native job 111300070821 / artifact 11287093978; original archive SHA256 `a23b33366a3c119f724e47b77360082068791bd3f8204f8b8104b60db64fd8fd`, 58,474,264 bytes, 14,014 entries with CRCs previously verified. Its native AIS step failed before compilation; later DPI/public-ENC evidence is diagnostic evidence only. Current-source inspection and historical Windows/boat evidence are deliberately identified separately.
