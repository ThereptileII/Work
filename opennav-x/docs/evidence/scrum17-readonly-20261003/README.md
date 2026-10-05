# SCRUM-17 / SCRUM-259 read-only boat readiness refresh

Observed **2026-10-03 04:47:40 UTC**, through the existing `ssh boat` alias.
`observations.json` is sanitized at source: no profile contents, private paths,
command lines, addresses, credentials, or device identifiers are retained.
`read-only.ps1` is the exact inventory query supplied over SSH standard input to
an in-memory PowerShell script block. It was not installed or saved on the boat.
The existing version-controlled `tools/boat/inspect.ps1` and `Common.ps1` were
reviewed first; the narrower query avoids their broader path/content output.

| Item | Fresh result |
| --- | --- |
| Stock OpenCPN | 5.12.4, SHA256 `7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`; exact supported identity |
| Installed generation | Beta 1 `0.3.0-beta1`, commit `a3e6e0812e01d2aee8f0b83807527b9a0c0fc79a`; executable matches its ownership record |
| Normal profile | 21,492 bytes; SHA256 `d891d88c62657139e1b1c6ff7d9e8acdae726a4e116dbc844126992a39adbdb6` |
| Install state | 461 bytes; SHA256 `a6ec7cda54cf855ef3ce78bbe63090bf1a799345c0713dc545c2ccc0a003952d` |
| Navigation/helper processes | None matching OpenCPN, OpenNav, XNav, SKAGER, oexserverd or oeserverd |
| Display | Current controller reports 1920×1080; current-user AppliedDPI is 144 |
| SSH | Query succeeded; service Running / Automatic |
| Tailscale | Service Running / Automatic; backend Running; self online; zero health warnings |
| RustDesk | Service Running / Automatic and three processes; zero established TCP connections at observation |
| Prior qualified review tools | Expected `review-ffa2de31ae02ca826ec5d3604f57a8c619035bb1` directory present |
| Commissioning marker | Absent |

The profile and install-state hashes match the supplied previous summary and
were unchanged on the post-inspection hash read. This checks the installed
executable against ownership, not every installed file. Tool-directory presence
does not repeat the earlier tool-content qualification. DPI is a registry
setting, not a measurement from a newly launched rendered window. RustDesk's
idle TCP count neither proves nor disproves an interactive connection; no
interactive session or connection attempt was made.

An initial oversized encoded inventory invocation returned SSH exit 1 without
a usable report; its underlying error was not retained, so no cause is claimed.
The same read-only query succeeded when streamed into an in-memory script block.
The separate initial reachability probe had already succeeded.

No application, vendor helper, installer, commissioning tool or scheduled task
was launched. The only executable queries were PowerShell inventory and the
existing Tailscale read-only status CLI. No process was killed, no profile or
installation was written, no backup was created, no remote-access setting was
changed, and no actuator or physical interface was used. No public acceptance
is implied. The pending native candidate is not installed; private adapter
loading, licensed-chart rendering, actual interactive/DPI presentation and boat
acceptance remain subsequent gates after native qualification.
