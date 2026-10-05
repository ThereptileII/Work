# Native Win32 chart compile preflight

[Run 37079442345 / job 111076503274](https://github.com/ThereptileII/Work/actions/runs/37079442345/job/111076503274)
passed for remote `9fcd3db54ee6913cc144ecfc09ec2152078a6eff`, mapped to
local `1356fd1603aacbea04d7081d16331e9a181180bb`.

Downloaded artifact 11257159820 is 4,606,157 bytes with SHA-256
`78d63ba3fa7b104e5f6a171811a72d6c5e7ccbf7445add36d5381970234f9dd0`,
matching GitHub's recorded digest. All 389 ZIP entries pass CRC verification.

Independent inspection verified all sixteen expected object files, their exact
hashes and COFF machine `0x014c` (x86), plus sixteen actual MSVC compile commands
and successful target builds. This includes `glChartCanvas` and `DepthFont`.
All sixteen archived source units match their declared identities. The 297 local
inputs match the exact local candidate; 1,438 upstream inputs match a newly
reconstructed pinned source plus all nine reviewed patches. Differences are
limited to Git checkout CRLF conversion; immutable prototype bytes and the
LF-attributed embedded brand asset remain exact. Seven generated chart-resource
files, both generated configuration headers and sixteen MSVC projects were
independently rehashed against the recorded manifest.

This establishes **compilation only**. No application/dependency link, GUI
execution, rendering, DPI, installer or boat acceptance is implied. The recorded
policy has fixtures and pilot loopback disabled, GL enabled, no dependency builds
and no application link; `nativeProductAcceptance` remains false. The 1,558 SDK
header hashes are retained in the original harness manifest, but those headers
were not uploaded for an independent byte rehash. No job was dispatched,
restarted or cancelled, and no build/test was executed during this audit.

Exact object hashes, source counts, artifact identity, command-log hashes and
qualification limits are in `verification.json`.
