# SCRUM-211: exact 854 retained PluginHandler gate

[Run 38881652663 / attempt 1](https://github.com/ThereptileII/Work/actions/runs/38881652663/attempts/1),
job `116909569359`, passed with helper and frozen product
`854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc`. The subject is the original
successful Windows producer `38765418126/1`, job `116536443214`; its overall
baseline remains failed. This documentation commit does not change that source.

The independently reviewed capture inventory is
`2a1e50a2ba046d337b50075dfc83e86e323f89495a0019b4a78dccf10cdcff10`.
The gate authenticated all 1,585 captured files and mappings. Independent review
checked all 99 consumed Git blobs/modes unchanged, the exact helper-only module,
17 local inputs and the workflow. Three tested production source files match the
packaged source after explicit CRLF-to-LF conversion; runtime comparisons use
exact bytes. Complete identities and outcomes are in [receipt.json](receipt.json).

All **13 native cases passed**: valid HTTPS and HTTPS redirect installed the
fixture archive; HTTP downgrade, local-file redirect, initial HTTP, interrupted
but extractable archive, untrusted/redirect-untrusted, expired/redirect-expired,
wrong-host/redirect-wrong-host and trust-removed cases refused before archive
open and preserved installed files and records. The ignored-download-failure
mutant was rejected for entering archive installation. The HTTP control was
reachable, forbidden requests were zero, and owned trust/server cleanup was
verified without errors. The maintained workflow also passed 58 loader groups,
17,625 export-forwarder checks and 282 TLS byte-policy checks. No vendor DLL or
plugin factory was executed.

Both separately authenticated SDK roles selected producer `38618335277/1`,
artifact `11924018896`, on actual Windows image `20261012.333.1`: **newer host /
newer captured subject**. This run grants no acceptance to another image pairing.
The original result artifact `12037219056` is 12,754,281 bytes, SHA-256
`87b980faf8eb52dc07449f73f25dee75b6b8b94bc57edfcef6bde4338a9ae0ac`;
all 5,284 member CRCs passed. Original metadata, log and report hashes are retained
in the receipt. Originals were collected once and remain private.

Normal and mutant reports bind all 11 DLL hashes to the authenticated capture;
the result archive does not contain DLL copies. Eight DLLs match the final
package exactly, including all four maintained TLS DLLs. Packaging replaces
`concrt140.dll`, `msvcp140.dll` and `vcruntime140.dll`; the receipt records each
before/after size and hash. This PluginHandler result covers the captured set.
The original final package separately passed its post-copy selftest, restart with
System32-only PATH, and seven production-recovery checks over 1,046 files,
including XNav, Legacy and Safe launches. The recovery report is
`84a9238b3b1fdc67cc76b90894db6da14db25c1751ec9841b237d8b687e05912`;
its fixture-free executable is `8ad205a452683316330549bb93a5d0c039b2a2acb4d59eaa6aa4c5d3b85bc511`.
Independent import review found no normal/delay consumer of `concrt140.dll`
and direct app imports of the other two. The coordinator selected no extra CRT
gate. This coverage is not a per-DLL loaded-hash census or a rerun of these 13
cases, and unrelated fixture-enabled peer results are not relabeled as product
evidence.

The [original package custody](../candidate-854078a5-custody-20261016/README.md)
and [wizard diagnostic finding](../scrum22-wizard-launch-20261016/README.md)
remain separate. At this receipt, installer/charts qualification `38882281891/1`
is pending. This component pass grants no overall product, release or boat
acceptance.
