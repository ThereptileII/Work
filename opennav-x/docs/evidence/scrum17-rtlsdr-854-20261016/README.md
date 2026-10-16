# SCRUM-17: retained RTL-SDR dependency refusal

The 2026-10-16 10:51:18 UTC metadata diagnostic of the failed staged generation
for product `854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc` completed with **103 staged
PE files and exactly two missing imports**, both from `plugins/rtlsdr_pi.dll`:

- `wxbase312u_vc_custom.dll`
- `wxmsw312u_core_vc_custom.dll`

The module is 208,384 bytes, SHA-256
`404e69931034140fbda0179964d0a29077a46b78a441c05fc68a4ee0616a64e5`.
The original Update report records the first missing import. The completed
aggregate diagnostic retained both findings. Before/after complete staged PE
inventories and the pinned installer source were unchanged. No other staged PE
blocker was observed by this scan.

[receipt.json](receipt.json) binds the private original Update report, diagnostic
scripts, failed diagnostics, native parse observation and final scan by exact
SHA-256 and byte count. The final private scan receipt SHA-256 is
`de069224ed5724b7143a81fdb2cc79bc4a3f1cf05cb102d68630b31064195692`.
Private paths, complete inventories and profile/navigation data are omitted.

The diagnostic used the exact frozen `Lifecycle.ps1` parser and resolver bodies
under native x86 Windows PowerShell 5.1.22621.6133. It inserted bounded error
collection at the legacy-name, binary and import boundaries so the first failure
would not hide later findings. The source allowlist excluded Lifecycle top-level
execution. Product bytes and the actual installer gate were not changed.

Two diagnostic-helper failures remain preserved. The first constructed resolver
failed to parse before invocation because Windows PowerShell 5.1 did not sort
hashtable offsets numerically with `Sort-Object at`. The parse-only companion
then failed before report output on direct `List[object]` array conversion.
Versioned helpers use `.ToArray()` and explicit numeric sorting, and retain
failure reason/stack trace. The native parse observation showed three errors
with the original ordering and none with numeric ordering. The local syntax and
inverse check against exact CRLF source matched the native numeric construction
and recovered the original resolver text after removing all seven diagnostic
insertions and the function rename. It did not invoke the resolver.

This is diagnostic evidence, with `acceptance=false`: no candidate module was
loaded, application launched or staged content written by the diagnostic. It
inspected native system metadata; this is not a zero-code-execution claim.
Ordinary plugin initialization, chart rendering, helper behavior and runtime TLS
acceptance are outside this scan.

At this evidence snapshot, the exact stock-DLL cold hold, unchanged Update and
stock restoration remain pending. No successful recovery or boat launch is
claimed. A recovery must retain the unmodified installer's full gates, then bind
fresh installed identities and the separate passive commissioning evidence.
