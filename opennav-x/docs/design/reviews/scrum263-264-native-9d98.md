# Bounded native review: logo, typography and chart paint

Application `1835d1b` / published `9d98a500`, native run
[37120549213](https://github.com/ThereptileII/Work/actions/runs/37120549213),
artifact 11275626437. The [image receipt](scrum263-264-native-9d98-images.json)
binds seven actual native images and the canonical Windows prototype references.
Application source/artwork is unchanged in the corrected `442960ba` build;
its separate package/runtime gates remain required.

Reference intent: exact prototype font stack and hierarchy, restrained chart
palette, source-classified marine artwork, and the user-requested smaller
approved SKAGER wordmark. Compare the 1280×800 canonical Day/Night references;
the supplied 650-pixel responsive screenshot is not the desktop geometry target.
Real chart geography, missing data and upstream navigation semantics are not
substituted with illustrative prototype content.

## Observed

- The 124-DIP, two-line SKAGER logo fits without visible clipping in the
  reviewed 100%, 150% and 1920×1080 images. The source artwork and icon remain
  unchanged. Scaling at 150% is expected; physical-display readability is open.
- Main numeric typography and hierarchy are consistent with the prototype.
  Separate actual GDI probes confirm the declared Segoe UI fallback on this
  runner. Pixels alone do not establish every geographic label's selected face.
- Visible real-ENC land/structures are neutral beige with subdued outlines.
  No conspicuous old brown structural fill appears in these views. Small and
  incomplete object coverage cannot qualify every structure or hazard class.
- The loaded, zoomed and adjacent NOAA cells contain small/overlapping marks.
  They cannot qualify all new buoy, cardinal, beacon and yellow-topmark paths.
  Long-range light circles and other guarded stock symbol paths remain visible.
  The pending actual IHO-cell package capture provides more suitable coverage.
- Dense Seattle labels/soundings remain busier than the illustrative prototype.
  Geography, units and unchanged S-52 placement contribute; sounding ink and
  geographic-label readability remain open rather than accepted differences.
- Bright magenta remains around the dumping-ground boundary near ownship
  (zoom: roughly x550–790/y273–518), and traffic/restriction ink in the adjacent
  cell (lavender stripe around x402–506/y99–462). These are the previously
  documented chart roles under SCRUM-15. The spoil-ground color-only change
  in SCRUM-262/private `d1cfbba` was deliberately withheld after its shallow-fill
  visibility review; it is not a missing merge or a claimed matching feature.
  Muted ferry/cable lines are
  visibly distinct; no failure of their narrow mappings is established.
- A stock cyan notification bell remains beside chart orientation (roughly
  x947–983/y101–139). **SCRUM-271** records this additional UI defect. Source
  inspection identifies `NotificationButton` and `NotificationsList`, drawn by
  `ChartCanvas` while upstream notifications exist. Its severity, messages and
  GUID-based acknowledgement must remain accessible; hiding it is not a fix.
- The earlier bright GTK frame outline is not apparent in the native Night
  image. This is a Windows observation, not a statement about the Linux defect.

## Scope and next evidence

The four DPI navigation images contain the coastline basemap, not a full ENC.
The ENC images use software: even the separately requested OpenGL phase fell
back. No painter-fixture image is counted as a real chart; no private licensed
chart or boat image is involved. No product change or rerun was made for this
review. All screen-level conformance rows remain pending until the corrected
package and physical boat review pass. SCRUM-263/264 retain those gates;
SCRUM-15 retains unmapped chart-role differences; SCRUM-271 owns the bell.
