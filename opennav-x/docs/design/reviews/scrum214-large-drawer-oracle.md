# SCRUM-214 large desktop drawer oracle correction

Selected in Jira comment 10491 after the c95d native DPI failure. This change
corrects test geometry only; it does not change the product or original HTML.

The [c95d run](https://github.com/ThereptileII/Work/actions/runs/37017351644)
failed in `fullscreen_1920_review` at the Preferences width assertion. Verified
artifact `11235646271` has 2,224,089 bytes and SHA-256
`2f276dec397f133e175622793a22faadbd97c50cab05644faa4d163505f71b75`.
Its `dpi-results.json` records build
`c95d3a091bb2a0ce0e19d0146ab28dc848ef5290`, 96 DPI, a 1920x1080 frame,
and drawer `{x:1226,y:88,width:460,height:946}`. The reviewed
`dpi-100-failure-visible.png` agrees with this observation.

The immutable `docs/design/prototype/index.html:71` requires a 460px wide
drawer from 1500 CSS pixels. The measured reference table in
`scrum216-wide-geometry.md` and `PrototypeGeometry.h` agree. Commits
`8c5c8d4` and `9dbab37` introduced and applied that native geometry; the
Windows test still expected 432px above 1100 DIP.

`assert_prototype_drawer` now requires 410 DIP through 1100 DIP, 432 DIP below
1500 DIP, and 460 DIP from 1500 DIP for Preferences. Regular drawers remain
398 DIP. The one-pixel tolerance, ownership, containment and uncovered-window
checks are unchanged. Width failures include observed and expected pixels,
client size and DPI. The shared bounds helper also reflects the large desktop
top and rail dimensions.

Focused validation: `python tests/windows_ui_layout_tests.py` passed,
including seven added bounds cases and 45 native-observation drawer checks.
These cover 1100/1101 and 1499/1500 boundaries, physical-versus-logical DPI
scaling, the observed 1920 geometry, rejection of the obsolete 432px width,
one-pixel tolerance and rejection at two pixels, plus existing ownership and
visibility guards. An in-memory execution of the original assertion against
the same 1920/96-DPI/460px fixture reproduced the recorded width failure.
`git diff --check` passed.

These are inert harness checks, not a replacement native result. No application
was launched, no CI was rerun, and no boat was touched. The c95d DPI result
remains failed; fresh native execution of the integrated revision remains
required for acceptance.
