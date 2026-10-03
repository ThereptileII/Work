# SCRUM-14: floating toolbar border/divider and complete navigation caption

Baseline: `8e353cfbff4bbb38ccec5ef04da18ac9d9d33261`. These are actual Linux wxGTK component captures, not a new full application/ENC capture or Windows/boat acceptance. The earlier full application finding is visible in `docs/evidence/skager-product-fidelity-78eccb8-linux/{software,opengl}/SKAGER-Night.png`.

The immutable authority is `docs/design/prototype/src/style.css` (`.floating`, `.map-tools`, `.map-tools>span`, `.nav-btn`) and the computed browser measurements in `docs/design/prototype/reference/linux/capture.json`: toolbar 189×52, 11px radius, 1px `#6b8b801c` border, 1×18 `#68837730` divider with 2px margins; navigation button 61×61 and 10px/400 caption using the existing authoritative font stack. The source prototype and selector geometry were not modified.

The baseline native toolbar exposes a (192,192,192) shaped-window outline in every theme and has no divider. The corrected toolbar opts into owned painting; it composites the exact CSS border and divider over the existing floating surface. Explicit graphics coordinates disable wx's extra half-pixel offset, so the straight border occupies exactly one pixel. Only chart tools use this paint; ownership, raising, focus, hiding and hit rectangles are unchanged.

The measured native `Instruments` caption is 54px wide. Its existing 61px button incorrectly allowed only 53px after an extra 8px deduction. Navigation captions now use the existing full button width, with unchanged font, weight, icon, center and geometry. Other button caption budgets are unchanged. This fixes the full word without abbreviation or a global font adjustment.

## Evidence and checks

- `before/{Day,Dusk,Night}.png`: production baseline controls and floating frame, compiled from exact baseline source.
- `after/{Day,Dusk,Night}.png`: actual changed controls/frame. The top-left text is an independent untruncated native paint oracle; the lower icon/caption is the real production button. The oracle is test-only, never added to the application.
- `pixel-checks.json`: all three full captions equal the independent oracle byte-for-byte; baseline captions fail. Straight border pixels equal the CSS composite, the divider is exactly 1×18 with two pixels of margin, all four 44×44 button regions remain byte-identical, and every pixel outside toolbar/caption regions remains byte-identical.
- `lifecycle.log`: existing unchanged lifecycle fixture passes 17 checks, including native GTK mapping/placement, nonactivating restoration and actual button input. It links `FloatingSurface.cpp` directly without `Controls.cpp`, preserving the standalone native preflight boundary.
- `shell-compile.log` and command: actual affected `Shell.cpp` compiles into a private object using the existing configured production command, read-only dependency includes and local changed source. Production `Controls.cpp` and `FloatingSurface.cpp` compile/link into both capture executables. No root/frozen output is modified.
- `receipt.json`: exact source, authority, fixture, executable and evidence hashes.

`tests/chart_chrome_presentation_test.cpp` is a standalone test executable in the existing optional test build. Build baseline against the three baseline `ui/Controls.cpp`, `ui/FloatingSurface.cpp/.h` files without `SKAGER_CHROME_CORRECTION`; build current with that definition (the CMake target supplies it). Both use the same unchanged production font/theme inputs. Run each with its output directory in a private 420×200 Xvfb display, `GDK_BACKEND=x11`, `GSETTINGS_BACKEND=memory`, then run:

```sh
python3 tools/verify-chart-chrome-pixels.py docs/evidence/scrum14-floating-caption
```

The first capture setup inherited the host's forced Wayland backend and produced empty Xvfb-root captures; these invalid captures were rejected before comparison. The final capture explicitly selects X11 and is visually reviewed. An initial object-command path rewrite omitted the existing wx header prefix; the final retained command uses the actual read-only prefix and compiles successfully. Neither setup issue is presented as product evidence.

This bounded change does not implement the prototype's exterior toolbar shadow, restyle navigational symbols, change real geography, or change chart-selector geometry. Windows/DPI, integrated native application and physical boat validation remain open under SCRUM-14/15. No suite, CI workflow, application, chart source or marine transport was launched for these component captures.
