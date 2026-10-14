# SCRUM-225 hosted Windows image refresh — 2026-10-14

Frozen product `1107c04f55a5a51660e424493bf083fc83106443`,
[run38549237034](https://github.com/ThereptileII/Work/actions/runs/38549237034),
failed Windows SDK restore before application compilation. Authenticated old
producer image `20261004.326.1` differs from consumer image `20261012.333.1`.
This is an environment/input provenance failure, not a new product crash,
AIS failure or installer-receipt regression. No Windows application was built.

The original Windows evidence artifact11886229520 is 61,324 bytes, SHA-256
`73cf59108499f346de3646f04377c5efa86c2e9943c56acc9136f1bc604696a4`.
Its original API identity and all45 member CRCs were verified. The sanitized
records here retain the causal error and original input/producer identities;
the original artifact ZIP is retained locally, not duplicated in source.

The only later dependency workflow was verification-only, so no replacement
SDK existed. Root dispatched the existing dependency-only producer at exact
`e80f0b920c71bb7095ef6ff8485a2dcac931bc76`:
[run38552041879](https://github.com/ThereptileII/Work/actions/runs/38552041879).
It builds/tests the maintained libraries and seals their native outputs; it does
not compile SKAGER. At this checkpoint it is running, not accepted or selected.

Independent source review confirms all30 producer fingerprint inputs/workflow
are identical between1107 and e80. Fresh sealing records the current environment;
identical current consumer scripts require no historical producer compatibility
exception. Runner/Python/native tool/recipe/ABI checks remain unchanged.
The exact successful new artifact still needs authentication and a fresh native
consumer reprobe before a new application candidate can use it. The failed
candidate remains unqualified. No boat, access-policy or public-release change.
