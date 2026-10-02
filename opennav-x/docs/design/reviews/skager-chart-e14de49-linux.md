# Chart review: software and OpenGL, e14de49

This is a development comparison, not visual acceptance. Source
`e14de49f444c56de28465416a83337838ad84413` passed the integrated Linux link;
the [identity record](../../evidence/skager-chart-e14de49-linux/identity.json)
ties the executable, public NOAA ENC, source/resource hashes, collector and
screenshots together. All twelve final screenshots were inspected.

## Observed against the prototype

The SKAGER header uses the approved artwork. Geographic names now have the
prototype tracking and subdued water-name opacity. The enabled COG endpoint
uses the active-route color. The chart selector's two vector roles use the
prototype palette while retaining its interaction and chart identity. Real
coastlines, soundings and navigation objects remain present in both styles.

The light descriptions and sounding digits in this binary still use the older
font treatment. SCRUM-248/249 change these in a later source revision. Remaining
magenta symbols require object-specific inspection; changing all magenta would
change unrelated navigation semantics. No chart objects were hidden for these
screenshots. The prototype's illustrative geography and sensor readings are
not a valid replacement for real chart content or unavailable live input.

OpenGL Dusk/Night contain an unacceptable repeated edge strip in **both** styles:
eight top rows repeat bottom chart rows, and two left columns repeat the right
edge. SCRUM-250 tracks the final-layout/framebuffer-size investigation. Initial
Day and software captures do not show this artifact. Passing the coastline and
palette checks did not detect it; visual review did. This gate remains failed.

## Evidence limits and collector corrections

Software and Mesa llvmpipe each captured SKAGER/Standard × Day/Dusk/Night. Four
application runs exited cleanly. The Linux font fallback is not authoritative
for Windows Segoe rendering, and llvmpipe is not the boat GPU. There is no active
route or AIS target in this viewport, so these images cannot qualify their
appearance. Native MSVC, packaging, Windows DPI and boat checks remain open.

Two initial collector failures are retained. The first software remote-close
request timed out with long concurrent profile/socket paths sharing their
first 107 bytes. Truncation/collision is plausible but not syscall-proven.
Unique temporary profiles with socket paths below 100 bytes passed four clean
exits. This evidence does not establish an application crash.

The initial GL color assertion expected the software palette. Pinned
`RenderToGLAC` divides channels by 256, so the GL area-fill oracle now uses the
exact `round(channel * 255 / 256)` conversion. No broad color tolerance was
introduced. The rejected screenshot and original report are retained alongside
the corrected captures and both collector revisions.

## Images

| Renderer / style | Day | Dusk | Night |
|---|---|---|---|
| Software / SKAGER | [Day](../../evidence/skager-chart-e14de49-linux/software/SKAGER-Day.png) | [Dusk](../../evidence/skager-chart-e14de49-linux/software/SKAGER-Dusk.png) | [Night](../../evidence/skager-chart-e14de49-linux/software/SKAGER-Night.png) |
| Software / Standard | [Day](../../evidence/skager-chart-e14de49-linux/software/Standard-Day.png) | [Dusk](../../evidence/skager-chart-e14de49-linux/software/Standard-Dusk.png) | [Night](../../evidence/skager-chart-e14de49-linux/software/Standard-Night.png) |
| OpenGL / SKAGER | [Day](../../evidence/skager-chart-e14de49-linux/opengl/SKAGER-Day.png) | [Dusk](../../evidence/skager-chart-e14de49-linux/opengl/SKAGER-Dusk.png) | [Night](../../evidence/skager-chart-e14de49-linux/opengl/SKAGER-Night.png) |
| OpenGL / Standard | [Day](../../evidence/skager-chart-e14de49-linux/opengl/Standard-Day.png) | [Dusk](../../evidence/skager-chart-e14de49-linux/opengl/Standard-Dusk.png) | [Night](../../evidence/skager-chart-e14de49-linux/opengl/Standard-Night.png) |
