# Live status text wrapping

The ProductPanel update path compared a status label's displayed text with its
unwrapped source value. `wxStaticText::Wrap` inserts newlines into that same
label, so unchanged long status text repeatedly looked different. Every 250 ms
update reset the label, wrapped it and ran Layout/FitInside again. This also
affected a retained hidden product page. The behavior follows the pinned
[wxWidgets 3.2.8 implementation](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/common/stattextcmn.cpp#L178-L207).

Each live label now remembers its original text, wrap width, font and DPI. Source values
are still evaluated on every update. A changed value updates immediately; a
resize rewraps from the original value. An unchanged value at an unchanged
width, font and DPI leaves the label and layout alone. Data quality, age and sensor callbacks
are not cached or skipped, and modal rebuild guards remain unchanged.

The regression uses a real ProductPanel on the wx application thread and a
counting wxBoxSizer around its actual content sizer. Before the correction,
eight unchanged updates failed the assertion that no additional layout runs.
The same fixture checks that the long label really wraps, changed text causes
layout, wider and narrower sizes rewrap, a font change at unchanged width reflows,
explicit line breaks and empty values survive, and the subsequent unchanged update
settles. It adds no product test API and issues no navigation or hardware
commands.

Local Linux validation: the final font/DPI-aware implementation passes all 11
new actual-wx assertions, the complete 26-group object/input workflow and the
standard 110-case integrated suite (19.01 seconds). Eight unchanged updates
execute zero additional sizer layouts. The original failure and passing reports
are retained under `evidence/local/boat-beta2/live-text/`.

This fixes avoidable layout work. It does not claim a measured boat-PC frame
rate improvement or qualify native Windows rendering. The same fixture and
native DPI workflow remain required for the next exact product candidate.
