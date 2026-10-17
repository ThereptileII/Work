# Installed Beta 2.5 boat check — 17 October 2026

**Result: not accepted.** SCRUM-17 remains open; the captured startup crash is
tracked in [SCRUM-376](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-376).
The user made the desktop available for the previously pending focused checks.
This session performed cold identity/recovery inspection and reviewed actual
earlier runtime evidence. It did not launch the application, send hardware
commands, install anything, rebuild, or run a design/endurance sweep.

## Exact installed candidate

- Commit: `9feae59ce0ef1f54107c3c51f920085297ea7bc8`.
- Version: `0.4.0-beta2.5`, private Staging.
- CI: [38911440527](https://github.com/ThereptileII/Work/actions/runs/38911440527),
  producing/qualifying attempt 2, with original prerequisite evidence retained.
- Generation: `cd6a841a0a574c9f872457e09e82c51f`.
- Executable SHA-256:
  `6cdaa8a705a3f6745ea1085b4975c2f63928d49bdd0b92d092b89ea28120a164`.
- At 12:40:01 UTC, all nine runtime files listed in the
  [installation record](../scrum17-9fe-install-20261016/README.md) again matched
  authenticated recovery bytes. Stock OpenCPN retained its approved hash.

This does not qualify the later unbuilt source batch, including chart-tool hints,
first-start presentation, route editing, alarms, AIS age/name and Track/Wind work.

## Interrupted recovery: preserve current user state

The unlocked desktop, no application/helper process and running remote-access
services were confirmed. The exact retained Restore invocation then refused
`Protected tree changed; retain RTL-SDR journal for review: profile` before
publishing a completion receipt. The stock DLL was already restored: its original
208,384 bytes/hash remain present, the held file is absent, and the existing
restore intent remains intact. No receipt was manufactured or historical plan
changed to bypass this refusal.

Independent comparison reconstructs the hold-time profile inventory from the
earlier authenticated ordinary restore records and reproduces its original
digest. Current stock-minus-that-DLL, managed-plugin and old-generation trees
match their original digests exactly. The profile has 731 added entries, no
removed entries and four changed files: INI, navigation database, log and
diagnostics. Additions comprise 725 icon-cache entries, five crash-report entries
and one plugin load stamp. These changes predate this operator session; their
origin is not automatically attributed to any particular application operation.

The current INI was last written at 11:23:08 UTC and has SHA-256
`945e0ad3237054092ee5efa9ff6b674ffb0d8f5fb27e2f7e52625693db0e244a`.
It contains 33 normalized configuration changes from the previous baseline,
including display preferences, viewport, resource location and Online AIS radius.
It was copied privately for review, not overwritten. The old full-profile
preservation assertion is therefore false; exact DLL restoration and historical
journal completion are different claims. Any new guarded launch needs an
explicitly reviewed current baseline and fresh passive audit, not an old hash
refreshed without reviewing changes.

## Actual runtime evidence, observed retrospectively

The retained log and diagnostics belong to the user's sessions earlier that day,
not a new controlled run and not a live reading at inspection time.

| Boundary | Evidence and result |
| --- | --- |
| Initial startup | Began 11:16:11 UTC; crashed at 11:16:19 UTC. **Failed.** |
| Recovery behavior | Two later sessions skipped the previously failed AIS-VD candidate, reached the startup-healthy marker and exited normally at 11:19:34 and 11:23:09 UTC. Observed recovery, not a clean original-plugin-set pass. |
| Builtin loading | ChartDownloader, Dashboard, GRIB and WMM explicitly report enabled and initialized in the later diagnostics. This closes the earlier missing-builtin observation only for that captured runtime. |
| Additional plugins | AutoTrackRaymarine and o-charts explicitly report initialized. This does not qualify their physical commands or all plugin functions. |
| Charts and GPU | Actual chart initialization records, verified private o-charts presentation, decoder `Check OK`, and Intel UHD OpenGL 4.6 framebuffer initialization. No fresh screenshot/content review or software-renderer comparison was performed. |
| Vessel data | Captured position/SOG, STW, apparent wind, depth, water temperature and fresh-water tank were LIVE; heading ESTIMATED; waste tank AGING. COG, true wind, rudder and propulsion/battery values were UNAVAILABLE. These are capture-time states, not current sensor claims. |
| Online AIS | Captured Connected state and subscription confirmation, one target/chart mark, 31 received messages and seven accepted reports. No new selection, network-loss or reconnect test. |
| Energy/SmartNav | No active route and unavailable dependent estimates. No route/arrival-SOC qualification. |
| Pilot | Captured control and serial session OFF, no recorded SKAGER command, physical Standby feedback. Existing configured permission/capability was present; this is not a permanent output-disable or pilot-identity qualification. No commands were issued by this test session. |
| TLS runtime | Earlier crash dump contains libssl-3, libcrypto-3 and libcurl and no legacy TLS module names. It contains no plugin DLLs and cannot prove the later plugin session's complete loaded dependency closure. |

The captured product identifies the exact installed commit, `INSTALLED PRODUCT`
and `test_fixtures=false`. No retained value was made fresh merely by reading it.

## Captured crash and bounded diagnosis

The private minidump confirms Windows exception `0xc0000005`, an invalid read
from `0xb38120d1`, on thread 1652 at executable RVA `0x7e6d96`. Crash-report XML's
zero exception-code field is not substituted for the actual dump exception.

Exact executable disassembly, captured registers/stack, RTTI and corresponding
source support this path:

`StdSerialIo::Entry → CommDriverN0183Serial::SendMessage(vector) →
NavMsgBus::Notify → Observable::Notify → inline wxQueueEvent`.

The vector overload here handles **received** serial data; its name does not
establish a hardware command. The captured Observable key is `n0183-ALL`.
The nearest exported symbol is not treated as the crashing function: it returns
before the separate routine containing the fault. No PDB-based source-line
claim is made. The listener heap was not captured, so this evidence cannot
distinguish invalidated vector storage from a stale registered handler.

The last log line concerned AIS-VD compatibility inspection, but the exception
was on the serial event-delivery thread. That log line does not prove AIS-VD
caused the crash; skipping it on the next starts does not prove a fix.

The exact observable source also contains a concrete synchronization defect:
different Observable instances share a per-key listener vector while locking
their own instance mutexes. Temporary wrappers used for registration and removal
therefore do not serialize with a different wrapper's notification. This is a
strong candidate mechanism requiring a focused synchronization/lifetime fix and
regression, not proof of the unique origin of this captured invalid pointer.
SCRUM-376 retains this distinction and the native qualification requirement.

## Final state and remaining gates

The installed application remained closed, no active commissioning marker was
present, and SSH, Tailscale and RustDesk remained running at the final cold
observation. Current navigation/profile/plugin files were not changed by this
inspection. The original recovery journal remains incomplete, with its failed
preservation evidence retained. No output setting was changed or restored by
this session. Do not automatically relaunch against an obsolete passive audit.

Required next boundaries remain the crash correction/targeted native regression,
reviewed preservation of current user state, and a fresh guarded passive startup
and shutdown with exact runtime observations. There was no new Legacy/Safe,
restart, physical-touch, design, network-loss or endurance pass. Public release
and Production promotion remain closed.

## Private receipt identities

Original logs, dump, configuration, account paths and chart data remain outside
Git under the private `boat-resume-9fe-20261017` evidence directory. Only these
sanitized conclusions and digest references are committed.

| Private receipt | SHA-256 |
| --- | --- |
| `restore-before.stdout` | `d789e7841cb5e5db82648006f1796d890df311a2a63087c7e7e58163f3277dfd` |
| `restore-resume.stderr` | `3cfa8bf868ee6d9134b11e6e832b45d0f0795946680851f9853e81c284e33d85` |
| `inspect-preserved-trees.stdout` | `cb3727a70e34b46650040fb786f174b343122598e8fce27a8f98e5ecb12a0828` |
| `preserved-tree-comparison.json` | `73890c41ce4e7ed937fc63f567c254816686f699a01210a880f5d721866f60df` |
| `runtime-observation-20261017.stdout` | `f4566432f8c7e50791a6a245779823a984b01b2da9d87ef70c54bb7e08d34cad` |
| `recent-opencpn.private.log` | `629634d6863e7d9c208f898f4d0b36e555441dd72bb42cc45cf9c81566745b7d` |
| `observed-diagnostics.private.json` | `0ce1e1da82646727f7f163fe4f15954891e7fa5a32d6495932c78d018f3f0777` |
| `observed-crash.xml` | `2a4a9873e8d71ce1d128dbafc0ea494f453c06d8af28c1a11fd35166baa6b264` |
| `crashdump.private.dmp` | `2f623b04feb0e2e29558b35ddd5a9d8b336bfd84108f7de7fa3b2475b3a81900` |
| `dump-modules.private.json` | `ed53ec5139fba936ec89b221cd7db441e225cb0a99e904c6d53b816446b5de78` |
