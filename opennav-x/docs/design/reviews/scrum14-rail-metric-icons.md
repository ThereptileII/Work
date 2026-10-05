# SCRUM-14: configured rail metric icons

The verified `488f` Navigation Day/Night captures omit the prototype's static
metric-type symbols. This bounded correction adds the unchanged speed, depth,
wind and battery paths from immutable `index.html` lines 232–235 to the existing
central icon registry. Compact rail rows draw them in muted theme ink at 16×16
DIP, right-aligned within the existing content inset and beginning at the label
row's padding, matching `.metric-label .icon` and its 16px parent box.

Icons follow stable configured keys: `sog`/`stw` use speed; `depth` uses depth;
`aws`/`awa`/`tws`/`twa` use wind; `soc`/`voltage`/`current`/`pack_power` use battery.
Other categories remain without an icon because these reference symbols do not
identify them. The selection does not depend on freshness or substitute any
reference readings. Long labels retain the existing ellipsis treatment with room
reserved for the icon; full accessible names remain unchanged.

The change starts from `af33a0502876f2acea9bc07233222cf133feecc7` in an isolated
worktree. A disposable Linux wx component compiled current Controls/VesselState
source directly against the existing wx runtime, without rebuilding OpenCPN.
Shell's configured-key wiring also passed syntax compilation. Six fresh-process
paired captures cover Day/Night at row sizes 186×124, 156×95 and 120×72 DIP, with
live, unavailable, stale and estimated samples restricted to the offline harness.
All four symbols appeared in every size/theme. Pixel comparison found no changes
outside the label band: values, units, source/age, freshness and row borders were
identical with icons enabled/disabled. Row sizes and accessible reading names
were equal; 11 recognized and four unmatched keys were checked. The four path
strings were checked byte-for-byte against the immutable HTML.

Local before/after images, diffs, component source and source hashes are retained
under ignored `evidence/local/rail-metric-icons/`; `final/review.json` identifies
the accepted local capture set. An earlier in-process resize capture was rejected
because the disposable harness captured stale geometry; fresh processes corrected
the harness. No product row/sizer, chart viewport, freshness, observation or data
logic changed. No permanent test target was added.

This is focused Linux presentation evidence only. Native Windows rendering,
full-shell integration and actual boat review remain pending for the later
integrated candidate; the currently qualifying candidate was not modified.
