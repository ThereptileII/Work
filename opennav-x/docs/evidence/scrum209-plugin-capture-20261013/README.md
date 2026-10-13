# SCRUM-209: original B0 capture failure and helper correction

The frozen B0 run [38442801510, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38442801510)
at `2c61369080c358d5ef9b8952d6f2881b52891b13` remains **failed**. Its
`windows-integration` job `115461450333` passed fixture-free product compilation
(step 15) and compiled-recovery retention (steps 16–18). The plugin input
capture (step 19) failed; this is not a product compilation failure.

The original capture artifact reports:

```text
Plugin input capture refused: Source changed before read: dumpbin.exe
```

The failed-step log records that message at line 11 and exit code 1 at line 12.
The helper refused its path-versus-descriptor complete-stat comparison before
hashing `dumpbin.exe`. Windows path and descriptor stat can expose different
execute bits and ctime semantics; the exact differing field was **not recorded**.
The original evidence does not establish actual tool mutation or a product defect.

Artifact `11840811376` is 283 bytes, SHA-256
`a1bb8f34901d8258cc301134f4af1c9c0539876356ec51a98e26eb2793c4d7c3`.
Authenticated GitHub length/digest and ZIP CRC pass. It contains only the capture
log, with no inventory or retained SDK data. The retained plugin follow-up is
**ineligible** because its required original capture step failed.

Compiled-recovery artifact `11840836434` is 426,988,285 bytes, SHA-256
`fa0357a26605f29403bde6602cdedf5b91f8ac10345956ad44f4cc5ae201a8a7`.
Its authenticated outer ZIP passes CRC and contains `COMPILED_RECOVERY.zip` plus
`receipt.json`. The original receipt says `packaging: not-run`,
`qualification: not-run`, and `trackedSourceDirty: true`. These are diagnostic
compiled bytes, **not packaged or qualified recovery**, and have no supported
reuse path. The updater staging, packaging, installer and Staging seal steps
were skipped. Artifact existence does not qualify the product or authorize reuse.

Helper fix `4cbea64a01c155873319011a0bddf69cba2dfa63` (isolated patch `bc77234e`) matches common physical
identity across stat sources, then preserves each complete source baseline for
mutation checks. Both the hash/read and copy paths keep hardlink, reparse,
metadata, byte-count and content checks. Linux: **34 focused tests passed**;
Python syntax checks passed. The first native run
[38454485025](https://github.com/ThereptileII/Work/actions/runs/38454485025) at
`97879a11` remains **failed**, with four Windows fixture path-substitution
assertions and two existing case-sensitive-fixture skips. Fixture correction
`d43465a1` then passed [38454852384](https://github.com/ThereptileII/Work/actions/runs/38454852384):
34 tests total, 32 passed and the same two skips.

Follow-up `806b80da` adds the system `WLDAP32.dll` allowlist entry while retaining
local-shadow refusal and its negative control. Linux passes 37 capture-helper and
30 runner tests. Native [38455446291](https://github.com/ThereptileII/Work/actions/runs/38455446291)
at `cd8a76e7d0e2a1ab62e1277606e9ccdb4d9d6495` passes 37 tests: 35 passed and the
same two skips. Its downloaded artifact `11842117998` is 2,642 bytes, SHA-256
`5cdfe81864dc5cf389e357cc6b7424ea78cef9ffd68b8b7d9fb5d8008cacfa10`;
GitHub digest/length and ZIP CRC match. The actual native probe observes
`st_mode` differences for `dumpbin.exe` and `st_mode`/`st_ctime_ns` differences
for Python; common identity matches and every complete source baseline stays
stable. These later observations do not reconstruct the exact field in the
original B0 failure. No inspected payload executes.

One replacement private-trust [Staging run 38455915079](https://github.com/ThereptileII/Work/actions/runs/38455915079)
at exact `cd8a76e7d0e2a1ab62e1277606e9ccdb4d9d6495` is **pending** at this update,
with extended tests and design validation off. Runtime protected-retirement Go
inputs changed from B0 and the diagnostic archive has no supported restore path.
The native helper proof does not qualify that application, its plugin/download
integration, installed private-channel B0/B1 behavior or full Lifecycle uninstall.

[results.json](results.json) retains the original producer steps, artifact hashes,
unaltered compiled-recovery receipt and separate helper/native statuses. Subsequent
results are appended without changing the original failed conclusions.
