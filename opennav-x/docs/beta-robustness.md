# Beta input, installation and endurance gates

## External input

Marine inputs are untrusted. The bridge bounds NMEA 0183 length/checksum and
printable ASCII, validates N2K envelope length/PGN/source/priority (0..7), and
rejects nonfinite/out-of-domain quantities. Source/device/provenance strings
cannot contain control bytes. Unicode source labels remain supported.

Signal K text has a 256 KiB / 16-level nesting preflight before recursive
parsing. UTF-8 must be well formed; strings, escapes and balanced containers are
checked without recursion. This is a resource/encoding guard, not a replacement
JSON parser. The existing OpenCPN RapidJSON dependency still decides JSON syntax and marine values.
A real loopback test found wxJSON rejecting a valid escaped Unicode source label;
the OpenNav decoder now uses the same RapidJSON dependency as the upstream
Signal K driver, with length-aware strings and validated UTF-8. Escaped/raw
Unicode equivalence and the driver's CRLF framing have a dedicated regression.
Per-update/value/observation bounds in the existing decoder remain in force.

Inspection found OpenCPN `CommDriverSignalKNet::handle_SK_sentence` parses the
message before publishing the marine bus event and accesses `version` / `self`
with unguarded `GetString`. The integrated driver now applies the same bounded
preflight and checks handshake/context field types, length and control bytes
before those accesses. Valid messages retain the original driver/listener path.
The pristine baseline is unchanged. This protects the application parsing
boundary; it is not a claim of an audited WebSocket/TLS implementation. Upstream
Signal K's TLS certificate-validation/fallback behavior is unchanged: use a
trusted boat network/server, not an unauthenticated Internet endpoint.

`OpenNavMarine` adds malformed ASCII, invalid priority, deterministic randomized
envelopes, NaN/infinity/overflow and deeply nested/malformed Unicode JSON cases.
`smoke-signalk.py` sends real loopback WebSocket frames through the actual driver,
including wrong header types, control text, oversized/nested messages and null
measurements, then requires expiry, recovery and clean process exit. It also
checks live depth, voltage, SOC, motor RPM and Unicode provenance. No EV-CAN
decoding or extra production network stack is introduced.

## Installation failures

Beta retains the accepted per-user immutable-generation architecture and exact
stock SHA-256 allowlist. The new matrix supplements existing clean install,
repair, prior-version upgrade, rollback, uninstall and pre/post-commit recovery:

* A held transaction lock must refuse a concurrent update.
* A real NTFS denial of staging-directory creation must preserve the active app.
* A corrupt payload must fail SHA-256 preflight.
* An interruption after extraction starts must never publish an incomplete app.
* A deliberately trusted CI fixture with a required wx DLL absent must fail the
  actual staged-executable loader check even though its fixture hashes match.
* A locked state file must retain the previous atomic state and journal;
  rerunning after release of the lock must recover successfully.

Before the full installer matrix, two native filesystem checks qualify the
permission fixture itself: CreateDirectory must be denied, then the exact DACL entries/protection
and ability to create a directory must be restored. The fixture uses the Windows
PowerShell 5.1 .NET Framework ACL API without depending on inherited PowerShell
module search paths; setup errors are retained separately from engine failures.

Every failure compares the active executable/state and existing stock/profile
hashes. ACL changes affect only a disposable CI directory and are restored.
Atomic JSON failures remove only their own unique temporary record. The engine
never recursively deletes unknown/custom data. Unpublished staging directories
without a complete ownership manifest are retained as diagnostic residue, never
registered or selected. The uninstall assertion therefore checks **all committed
owned generations**, while separately counting deliberately failed unpublished
stages; this is necessary for the expanded partial-extraction/dependency cases.
This does not relax removal of any previously owned, hash-matching app file.

The real missing-DLL case exposed an OS loader dialog retained after timeout,
which later obscured stock OpenCPN. `SelfTest` now uses direct .NET process
creation (`UseShellExecute=false`) with scoped inherited
`SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX`.
The private installer host restores its previous error mode immediately after
creation, then waits/reaps/disposes the child. No global registry/error-reporting
policy is changed. The native interop class is compiled from the trusted .NET
Framework directory, with both caller directories restored: Windows PowerShell
5.1 otherwise resolves its implicit System.dll against NSIS's native plugin of
the same name. An early native fixture deliberately shadows System.dll to
qualify this boundary before the full Setup wizard. Normal application launch is unaffected. Native fixtures
require child inheritance and parent restoration; the actual missing-DLL case
must return a loader exit failure, not timeout or missing-report ambiguity, and
leave no System Error window. The final stock welcome/chart check remains.
[Windows error-mode inheritance](https://learn.microsoft.com/en-us/windows/win32/api/errhandlingapi/nf-errhandlingapi-seterrormode).

## Actual elapsed endurance

`tools/soak-runtime.py --seconds 10800` runs the real application for at least
three hours of monotonic elapsed time. Accelerated DEMO trip time does not count
as endurance. Short 120-second development runs validate the harness only.
The test uses a disposable profile and no device output. Active route progress,
battery/energy, AIS encounters and SmartNav run continuously; six product pages,
chart zoom and palettes are exercised, with stale/unavailable episodes and
recovery. Whole-source staleness must suppress route advice; the instrument-only
unavailable scenario deliberately retains GPS/route, suppresses energy and marks
depth, RPM and battery current unavailable. The earlier blanket route-suppression
assertion was incorrect and is not used as evidence of a product defect.
The existing separate gates exercise live marine transports, repeated
mode restarts, two public ENC cells, plugins and requested OpenGL/fallback.

Ten-second JSONL samples retain process CPU, resident memory, file descriptors
or Windows handles/private bytes/GDI/USER objects and UI update timing. Page
response is measured from input through the next diagnostic observation (which
publishes at 1 Hz), not represented as paint/frame latency. The callback timing
excludes asynchronous chart painting. The gate requires continuing UI ticks,
route progress, AIS context, energy suppression/recovery and a clean shutdown
with unchanged seeded navigation/profile fixtures.

After warm-up, first/last-window median growth must stay below 128 MiB resident
or private memory, 128 Windows handles / 32 Linux descriptors, 64 GDI/USER
objects or eight threads. Every page observation has an eight-second deadline.
These are CI leak/stall tripwires, not a performance promise for boat hardware.
Raw samples, CPU usage and all limits remain visible. Both integration jobs have
a 300-minute timeout; release qualification must select the full duration.
Physical touch, target-PC GPU and at-sea operation remain separate manual gates.

Native restore evidence showed identical inherited ACEs with only Windows adding
`SE_DACL_AUTO_INHERITED` bookkeeping (`D:` → `D:AI`). The permission fixture checks
identical binary DACL entries and all other descriptor flags, including protection;
it records both descriptors. This does not excuse changed permissions, ACE order,
SIDs or inheritance flags. The actual denied operation and restored creation must
both pass. Prior candidates failing the string-only comparison are not accepted.

## Qualified Beta 1 measurements

Exact `a3e6e0812e01d2aee8f0b83807527b9a0c0fc79a` completed every job in
[run 36137990012](https://github.com/ThereptileII/Work/actions/runs/36137990012). Linux ran 10,800.175 seconds and Windows
10,800.109 seconds, each with 1,080 samples and 540 page actions. Sustained
median resident growth was 184 KiB / 2.97 MiB respectively. Windows private
growth was 3.73 MiB; Linux descriptor/thread and Windows handle/GDI/USER
median growth was zero. Mean CPU was 1.65% / 1.10% of one core. Maximum page
observation was 1.25 s / 1.73 s including the 1 Hz diagnostics delay, not paint
latency. Both completed cleanly with unchanged seeded fixtures and continuing
route progress. Raw sample hashes, resource ranges and limits remain in
[Beta 1 acceptance](evidence/beta1-a3e6e08-accepted.json) and its downloadable platform artifacts.

The final native matrix passed 29 installer lifecycle checks, five loader
self-test checks, 39 filesystem checks in each PowerShell 5.1 bitness and three
crash-recovery cycles. Actual accepted Alpha-to-Beta upgrade, locked/denied
files, corrupt/missing dependencies and interrupted transactions are included.
This qualifies the tested hosted environment, not a general leak-free or
power-loss guarantee. Physical target-PC/endurance and boat tests remain open.
