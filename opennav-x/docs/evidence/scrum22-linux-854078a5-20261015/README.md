# Original Linux qualification: 854078a5

Candidate `854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc` passed the original
[Linux integration job 116527477400](https://github.com/ThereptileII/Work/actions/runs/38765418126/job/116527477400),
run `38765418126`, attempt 1. This receipt records that run without rerunning
tests. Later documentation commits are not the tested candidate.

| Original evidence | Result |
| --- | --- |
| Fixture-enabled CTest | 154 passed; zero failures, skips or disabled tests |
| Fixture-free CTest | The same 154 names passed again; 308 executions in total |
| Software and OpenGL public-ENC cycles | Both passed XNav → Legacy → XNav, workspace persistence, loaded ENC and rail assertions |
| Native Dashboard visibility in each cycle | Hidden at XNav startup, visible in Legacy, hidden after XNav return; plugin remains loaded |
| Fixture-free loader | Five checks passed; exact candidate commit, `test_fixtures=false`, `INSTALLED PRODUCT`, no profile initialization or plugin loading |
| Fixture-free pilot check | Five checks passed with simulated loopback only; zero commands, wire output or received bytes; screenshot review remains separate |
| Two ordinary peer instances | Profile preservation passed; both observed owned TCP listener lists empty |
| Linux PluginHandler source slice | 13 cases passed; zero forbidden HTTP requests; ignored-download-failure mutant rejected; native retained gate remains separate |

The chart helper waits for a publication newer than the pre-restart baseline,
matching the returned process PID and completed Dashboard initialization. It
asserts ENC, suppressed workspace and visible rail once, then checks native
visibility separately. The original result records returned pane state `2098127`
and saved Legacy/final state `2098125` in both renderer phases. The final retained
diagnostic reports readiness for PID `24638`, publication `2139339`, tick `7`,
and four visible rail regions. That final file is not a separate trace of the
first ready publication; the source hashes and successful original assertion
step bind the implemented check.

Endurance was explicitly **skipped** under the user-directed policy;
`release_duration=false`. Optional preview was skipped and
`SKAGER_DESIGN_VALIDATION=false`. These are not endurance, design, screenshot,
physical-GPU, Windows-installer, boat or release passes.

Artifact `11992076552` contains **14,636,581 bytes** and **3,984 members**. Its
SHA-256 matched the authenticated GitHub digest and every ZIP CRC passed:

`ba1b5ccf2c90ec68d8ae067df051b98cfbe5a085b880404eaaa5f9ea5cd0b073`

The privately retained original job log is **995,477 bytes**, SHA-256:

`12ddee8e64d63a7576db44fe3471cf2f2ba5812f6057d6327a717547260f67fa`

[receipt.json](receipt.json) records original job metadata, selected artifact
member hashes, exact counts, helper source identities and the skip declarations.
Large original logs, screenshots, profiles and the archive remain private.
