# Exact 1107 Linux integration evidence

Candidate `1107c04f55a5a51660e424493bf083fc83106443`,
[run 38549237034, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38549237034),
has a successful Linux integrated job `115803006381`.
Both original CTest reports pass **154 tests**, zero failures/errors/skips:
fixture-enabled and fixture-free executions of the same suite, not 308 unique tests.

Original artifact `11888380195` is 14,628,412 bytes. Its SHA-256
`fa15e7452037417f3e445a89550a3e8e138a3bd78c902d51e80dd13d7adf193a`
matches both GitHub metadata and the original upload log; every one of 3,984 ZIP
members passes CRC verification. The adjacent records retain original identity,
report hashes/counts and sanitized original log excerpts.

The exact-candidate peer containment reports PASS and offline boat-feedback
components report passed. Extended preview scenarios were skipped by workflow
selection. Endurance remains explicitly skipped, and design review was not requested.

The overall run is **failed and unqualified**: its Windows job refused SDK reuse
on runner-image drift before application compilation. This Linux success neither
accepts Windows nor transfers to later helper/source commits. Original failed
Windows evidence is retained separately under `scrum225-image-refresh-20261014`.
