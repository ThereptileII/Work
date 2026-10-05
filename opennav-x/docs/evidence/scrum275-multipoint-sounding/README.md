# SCRUM-275 multipoint sounding correction

The actual c24 Mesa capture traced an empty light inventory to SOUNDG index115,
which OpenCPN represents as GEO_POINT although its parent has no scalar position.
Pinned `S57Obj::SetMultipointGeometry` stores its positions/depths in arrays and
never sets parent x/y. The original inventory incorrectly read that parent as
an independent point, saw NaN, and refused all added light-location symbols.

The shared helper now recognizes only a non-clone SOUNDG GEO_POINT with chart
context, both initialized geometry arrays and a positive point count. Both
inventory walks exclude that container before reading scalar coordinates.
Ordinary/malformed independent points still fail closed. Array contents and
OpenCPN sounding/conditional rendering remain untouched; no chart data is
sanitized or invented. Existing duplicate/cycle/object limits still apply.

The new regression first failed against the unchanged helper at `CA check 45`;
its original log and both source hashes are retained. Corrected actual-method
checks pass **147 core, 147 private and 145 core without GL**, using normal
`-O3 -Wall -Wextra -Werror`. Fourteen new assertions cover unused NaN and
coincident finite parent scalars, unchanged depth arrays, missing arrays,
zero point count, clones and a different object class. Existing checks remain.

These focused tests use real production dispatch methods with recorded terminal
painters. They do not qualify an actual canvas, Windows DLL, boat or release.
The combined ordinary fan plus corrected point still needs actual software/GL
capture before the next full Windows candidate.
