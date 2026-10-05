# SCRUM-264: canonical empty TOPMAR lookup instruction

Base: `33d90c38cd1a628c21d0ca159647012d98649d2e`. Source correction: `e423f84`.
The actual combined canvas investigation retained by root in
`docs/evidence/scrum264-combined-33d-linux` observed RCID31314 / Simplified76,
empty attributes, null rule list, and INST length1/codepoint31. Therefore the
previous `empty()`-only guard incorrectly refused this already-authorized
explicit fitted yellow X.

Both pinned `ChartSymbols::ProcessLookups` implementations append `\037` to
`<instruction></instruction>` (core line236, private line243). Their real
`BuildLookup` methods preserve this string by value in core (line493) and via
`new wxString` in the API17 private renderer (line504). The fix accepts exactly
an empty string or one U+001F codepoint. The pointer overload still rejects null.
It does not strip separators or whitespace, accept prefixes, mutate lookups,
or change parsers/resources. Existing ruleList-null, RCID/table, typed TOPSHP7 /
COLOUR6, unique eligible floating platform, position and visibility guards stay
unchanged.

The focused runner compiles and executes verbatim actual ProcessLookups and
BuildLookup plus the unchanged actual Lookup declarations and actual LUPrec /
S57 types for each renderer. Only the ChartSymbols owner declaration, lookup
containers and test object construction are fixtures. The private parser's full
source is checked against the locked Git blob before compilation. The fixture
reads the actual RCID31314 XML node and passes it through each real parser; it
does not simulate the terminator by hand. Both actual ownership forms then
execute the production guard with a typed, exactly co-located platform/topmark.

Coverage includes canonical empty XML, real SY/CS instructions, whitespace,
multiple/prefixed control separators, null pointer, populated rule chain,
Paper table, alternate RCID, alternate shape, missing and ambiguous platform,
and disabled presentation. Three independent mutation controls per renderer
must fail: old empty-only guard, permissive U+001F-prefix acceptance, and removal
of the strict null-rule-list guard. `receipt.json` records results, actual method
and header hashes, compile arguments and limits; per-case logs are retained.

The existing actual-render fixture now uses the parser's canonical U+001F and
also checks populated/prefixed instructions. It was updated for the next combined
loader/canvas validation; it was not represented as executed by this parser-only
run. The first standalone setup attempt caught a fixture comparator signature,
then refused a nonlocked historical source path, then exposed an unrelated
private chartsymbols.h application-config include. The final fixture executes
both complete parser methods and exact Lookup declarations without including
that unused application header. No product include, parser or safety check was
changed to accommodate the test.

No broad resource suite, application build, canvas launch, native Windows gate,
private DLL runtime, CI dispatch or boat action occurred. Actual combined canvas
confirmation is separately owned; Windows and boat acceptance remain open.
