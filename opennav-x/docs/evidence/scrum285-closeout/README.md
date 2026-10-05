# SCRUM-285 issue-specific closeout

**Recommend Done for the generated-project traversal repair**, subject to root
review. This does not close SCRUM-274/224, detailed AIS runtime qualification,
package, native visual/DPI, boat or release gates.

Base `10b77d7c858919733519c1709e9fb964f05c5b12`; current acceptance and all four
Jira comments, including selection `10925`, were read. `audit.json` records this
read-only evidence inspection and exact byte identities.

| Criterion | Evidence and conclusion |
| --- | --- |
| Diagnose and preserve the actual failure | `../scrum285-native-ais-project-closure/` retains original remote `4ddf1f383e495150551946dd36103cd77cea85eb`, run `37155858878`, job `111300070821`: metadata `ProjectReference` nodes caused `KeyError: Include` after configure. Original artifact `11287093978`, 58,474,264 bytes, SHA256 `a23b33366a3c119f724e47b77360082068791bd3f8204f8b8104b60db64fd8fd`. The failed candidate remains failed. |
| Narrow traversal with strict real references | `tools/test-ais-runtime-windows.py:project_closure` selects `ItemGroup/ProjectReference`; mandatory `Include` and named-project lookups remain strict. Metadata is excluded; real dependencies are followed. The retained inverse-source receipt documents that changes outside the extracted traversal are absent. |
| Actual project and malformed-reference regression | Retained native proof reports 8 passed, 0 skipped: complete eight-project closure, original defect reproduction, metadata exclusion even with Include, missing Include/project and unknown dependency refusal, preserved downstream guards, and exact failed-job identity policy. |
| Native proof before full candidate | Local `c552f6158fda4ccdb995f05d876a6bf6107e326d` → remote `ab92dc78cd99e945a08b7a00d2b8166a5841f954`; run `37162505696`, job `111318683379`, completed 2026-10-03 23:40:54 UTC. Artifact `11288127267`, 27,576 bytes, SHA256 `4af23ff5417a38c1a9a916af4145e68e15673f5df1b28faaeeb36cc93f5943f0`. This preceded the combined candidate. |
| Current source and retained proof integrity | Rehashed the existing native artifact and all 11 members, checked CRCs and all eight original project identities: no mismatches. All ten native execution inputs match the proof revision, frozen local `0a52a6cfe3bd3b4a1e6253d609bd9dc046016bd4` and this audit base; reported text identities match Windows CRLF. The production wrapper is byte-identical throughout. |
| Preserve all downstream checks | Entire wrapper identity plus unchanged source-guard hash confirms retained compiler/backend, /MD, source allowlist, private imports, receipt/tool identity and runtime checks. The current workflow matches frozen source and invokes the actual wrapper with nonzero exit propagation. |
| Required actual downstream execution | `../windows-17ab044-interim-stages.json` records remote `17ab044a1e5222dc71791ac8118454219efe8734`, run `37164360050`, attempt 1, native job `111325149619`: same-job dependency capture succeeded, followed by actual maintained-TLS AIS step 18 completed/success at 2026-10-04 01:55:11 UTC. This resolves the narrow repair's pending downstream execution condition. |

The last row is an authenticated **interim API stage observation**, recorded at
01:56:39 UTC. Run/job were still in progress, with fixture-free production step
19 running. It is not a terminal job result or an independently audited detailed
runtime report. No current session/transport/provider counts, runtime files,
package outcome or final artifact identity are inferred. The short parser proof
itself compiled no native binary and performed no TLS/lifecycle execution; its
Python executable hash remains a CI receipt rather than rehashed executable bytes.

No tests, CI polling, downloads, builds, production/tooling changes, system or boat
operations were performed. Existing archives and failure records were preserved.
Only this closeout documentation was added; root owns the Jira transition.
