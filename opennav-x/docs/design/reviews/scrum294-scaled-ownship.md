# SCRUM-294: scaled own-ship artwork

Read-only boat inspection on 2026-10-05 established `OwnShipIconType=1`
(dimension-scaled bitmap). The previous prototype painter covered only the
fixed-size bitmap, so this setting bypassed it in both software and OpenGL.

Both bitmap paths now use the prototype chevron. Scaled rendering retains
OpenCPN's calculated length, beam, reference position, heading/COG selection
and rotation. The GPS antenna point remains visible on scaled vessels. Custom
user icons, explicit scaled-vector preference and the small-scale circle retain
upstream rendering. Standard/Legacy remain unchanged.

No current heading/course means an unoriented position marker, not a northward
vessel. Low-accuracy/invalid upstream position uses attention/muted color and
no healthy oriented chevron. A read-only canvas quality accessor supplies this
existing classification; no navigation processing is triggered.

501 focused painter checks pass, including immutable SVG geometry, independent
beam/length, quality, missing direction, invalid numbers and DC restoration.
The actual presentation, software canvas and GL canvas translation units compile
against pinned OpenCPN. Native Windows/boat rendering remains pending.
