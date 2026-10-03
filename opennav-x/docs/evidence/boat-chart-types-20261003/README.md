# Read-only chart availability — 2026-10-03

At 02:47:45 UTC, inspected only the closed profile's configured chart directories
and directory-entry extensions through `ssh boat`. Both roots exist. The first
contains 581 `.oesu` files among 1,344 files in 443 folders; the second contains
two `.mbtiles` files. The bounded walk completed without hitting its time, file
or folder limits and skipped no redirected entries.

The profile SHA-256 before and after was
`d891d88c62657139e1b1c6ff7d9e8acdae726a4e116dbc844126992a39adbdb6`.
No application or chart helper was launched, no chart content was read, and no
configuration, installation, licence or remote-access setting was changed.
The aggregate [record](summary.json) contains no directory names or chart names.

File presence does not prove successful decoding, current licensing, renderer
selection, graphical appearance or availability of particular navigational
objects. The o-charts private-renderer boundary is tracked in SCRUM-259; actual
installed boat rendering remains an acceptance gate.
