# Concurrent native endurance — SCRUM-287

The native producer retains all dependency, fixture, production, security,
installer, DPI and chart gates in the same job. Its verified dependency closure
is never transferred to another build job. Endurance uses the already-built
fixture-enabled application on a separate native Windows worker.

The scheduling correction follows run `37164360050`: its 360-minute native job
could no longer fit the mandatory 180-minute endurance stage after the remaining
production and package checks. This fact does not identify the production delay
or establish an application crash. The frozen candidate remains unchanged.

## Handoff and scheduling

1. After successful fixture, dependency-receipt and AIS gates, the producer
   copies its complete `build/xnav-install` runtime and original
   `build/xnav-windows/include/config.h` into a new artifact stage. A closed
   manifest records file hashes, sizes, directories, Win32 identity, source
   closure, commit, repository, run and attempt. No earlier profile is included.
2. A separate Ubuntu readiness job shares the producer's prerequisites, but
   does not depend on producer completion. It observes the exact same-run,
   attempt-named immutable artifact and required successful steps, with complete
   API pagination and a bounded 180-minute wait. Failure or timeout is not success.
3. The independent Windows worker downloads that exact artifact ID, verifies
   the manifest and fills only absent build destinations. Artifact transport
   omits empty directories; only safe, manifest-listed directories are restored.
   Files are never synthesized. The original unchanged `soak-runtime.py` then
   creates its own disposable disconnected profile and runs for 10,800 seconds.
4. Afterward, runtime and source hashes must still match. A separate qualification
   receipt requires native Windows authority, the real elapsed duration, exact
   executable bytes/hash, matching build/harness commits and passed raw results.
5. Candidate promotion requires both complete producer success and independent
   endurance success. It copies the six original product payloads and checksums
   unchanged; only the pending-endurance qualification heading changes. Public
   release still requires every previous platform, installer and boat gate.

The readiness wait uses a separate 195-minute job envelope. The native endurance
job has 225 minutes of its own, so waiting for compilation does not consume its
three-hour execution budget. The native producer retains its existing 360-minute
limit. No application is rebuilt by the readiness, endurance or promotion jobs.

## Boundaries and verification

`native-endurance-handoff.py`, `wait-native-endurance.py` and
`promote-endurance-candidate.py` own the three boundaries. They refuse foreign
or stale identities, malformed/aliased paths, changed or missing files, unlisted
entries, profile transfer, short or unqualified results and overwritten outputs.
GitHub credentials remain confined to the readiness metadata process; receipts
and logs contain no credentials or temporary download URLs.

The fixture-enabled runtime is intentional CI input. It is never the installed
product or a public download. The fixture-free recovery ZIP cannot substitute for
it. The simulation and software-chart endurance scope is unchanged: it does not
qualify physical actuators, native boat touch or the actual boat GPU.

Focused host checks cover 13 handoff, 19 readiness, 9 promotion and 18 existing
release-handoff cases. The small `skager-native-endurance-proof` workflow checks
these contracts on Linux and native Windows without building or running OpenCPN.
That proof alone does not accept real artifact transfer or endurance. A later
complete candidate must exercise the full workflow and preserve its original
receipts before this scheduling work is Done.
