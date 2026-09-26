# Beta 2 native review — 62e28e5d

Development review only. This candidate failed a Windows interaction gate and
is not accepted for deployment or release.

Source: `62e28e5dfe42e96531f00dd88686634c167b0db8`,
[CI 36273516935](https://github.com/ThereptileII/Work/actions/runs/36273516935).
The downloaded native evidence artifact `10917405061` matched ZIP SHA-256
`c9cc2240dd049d444ab85b83aef0bd41aa096a821a8fc082b146ba448e0a5a7d`.
Screens are 1280×800 on native Windows; this fixture-enabled test executable is
separate from the fixture-free installed product. No boat hardware was used.

## Navigation and palettes

Reference intent: chart dominant, restrained chrome, four readable rail values,
consistent navigation accents, genuinely dim Night mode.

Reviewed `instruments-02-live.png`, `route-01-active-route.png` and
`03-xnav-night.png`. All four rail values remain inside the frame. The chart
occupies the dominant area and coastline content is visible. The active-route
test frame itself reads route unavailable and must not be mistaken for visual
acceptance of the destination panel. Night chrome is dim with no white sheet;
its metadata contrast still needs physical-display assessment. Coarse world
coastline is not evidence that the boat's licensed nautical charts work.

## Propulsion and Energy

Reviewed `n2k-01-live-energy.png`. Battery, propulsion and destination groups
have distinct hierarchy. Missing motor power/current suppress dependent values;
there is no invented zero or arrival estimate. The motor Celsius unit contains
an erroneous extra character. The low-level battery-identity explanation also
needs everyday wording. Both are being corrected before the next candidate.

## Autopilot

Reviewed `pilot-failure.png`. Commanded heading is prominent, course controls
have large targets, STANDBY is accessible, and unsupported TRACK/WIND are dim.
However, degree units and button captions render as `Â°`. The strict native
test correctly refused to find `+1°`. This is an actual Windows text conversion
defect, not grounds to loosen the interaction assertion. Explicit UTF-8
conversion is required while retaining the same manual-only callbacks and
acknowledgement checks.

## Remaining review

A corrected native run must verify those glyphs and continue through all
screens, production fixture removal, installer and display gates. The full
boat-PC screen set, actual chart rendering, real source health and physical
display usability remain outstanding.
