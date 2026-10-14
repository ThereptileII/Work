# Already-running successor finalization — focused native evidence

Jira SCRUM-311. Source **`39f5b7e60d18498c329c37f34471dcb1a03a3969`**.
[Native gate 38602308806](https://github.com/ThereptileII/Work/actions/runs/38602308806)
passed. Implementation `7a38ad93`, required-source closure `66a669e6`, mandatory
execution map `39f5b7e6`. Independent source review found no blocker in this
bounded scope; root retained the documented unresolved phases.

## Result

| Platform | Passed top-level tests | Failed | Explicit skips | Required maintenance cases |
| --- | ---: | ---: | ---: | ---: |
| Native Windows 2022, Go x86 | 340 | 0 | 5 | 87 |
| Linux 24.04, Go amd64 | 237 | 0 | 0 | 64 |

Every mandatory case ran/passed exactly once with no failed or skipped required
subcase. All prior required controls are preserved. Five new portable cases
cover readiness/identity/custody refusal, one publication, cleanup ordering and
uncertainty, existing/pending state refusal and resume without downloading code.
Four new native cases cover actual NTFS publication/readback and retained process,
token and image identities; all existing marker namespaces; revocation before
and after creation; and exact pending-pair closure. Native service state and IPC
are seams. No service is created or reconfigured by these fixtures.

The existing 12 broker package tests pass in the native workflow; local exact
integration source also passes 12 broker and 14 provisioner closure tests. The
three new runtime files are mandatory in both corresponding-source packages.
Focused agent passes and cross-compiles overlap and are not added to CI totals.

## Artifact custody

Original ZIPs remain unchanged under the private local folder
`/home/standard/Work/skager-code-upgrade-finalize-20261014`. Provider metadata,
SHA-256, length and every ZIP CRC were checked. Embedded receipts bind the exact
run/source and test stream SHA-256. Portable JSON summaries are committed here.

| Platform | Artifact ID | Bytes | Original ZIP SHA-256 |
| --- | --- | ---: | --- |
| Windows | `11913618718` | 66,473 | `f7882d12b686d88e39d0b55486017464c8aaaa63368e0fdaa13fc237ac30afc2` |
| Linux | `11914755536` | 64,475 | `5a4ae712aac3866a199cd355f891006c7efc1cf1b5d0c96fd04a5e13e7eec170` |

## Qualified scope and limits

The explicit resume entry can finalize only the same complete, authenticated
successor pair already running under exact AUTO_START configuration with a
genuinely absent ready marker. It retains original completion, approved policy,
intent/reservation/drain/successor/pair and full fixed identities. After authenticated
readiness it rechecks the same native process/custody immediately before exclusive
publication and after flush/readback. Existing/partial/foreign ready markers are
never adopted or overwritten.

The service handle permits QUERY_CONFIG, QUERY_STATUS and READ_CONTROL only.
Failure cannot configure, start or stop that service. This still has immutable
ready-publication and existing cooperative executor-query/token effects, whose
normal cleanup and abrupt-death limitations are described in the
[contract](../../installer/broker-code-upgrade.md).

Early staging, DEMAND verify-store, AUTO+STOPPED/pending and partial immutable
publication remain unaccepted recovery phases. Actual installed SCM/UAC/IPC,
customer workflow, normal Setup/Maintain activation, full package and boat gates
remain open. No full application build, deployment, profile change or public
acceptance is implied by this focused run.
