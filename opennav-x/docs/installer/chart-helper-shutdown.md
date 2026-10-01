# Normal shutdown of an orphaned stock chart decoder

`tools/boat/stop-chart-helper.ps1` sends one normal local shutdown request to the
exact reviewed o-charts decoder left by an already exited official stock
OpenCPN session. It does not start or terminate a process, transmit navigation
data, change a profile, restore plugins, or authorize another application launch.
Only stock-session cleanup is supported.

The source boundary is o-charts commit
`c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8`:

- `src/o-charts_pi.cpp:771-789,3035-3048`: normal `DeInit` calls
  `shutdown_SENC_server`, which invokes `Osenc_instream::Shutdown`.
- `src/Osenc.h:644-649,664`: a 1025-byte all-character `fifo_msg` contains command,
  256-byte reserved FIFO name, 256-byte chart filename and 512-byte key; command
  `2` is `CMD_EXIT`.
- `src/Osenc.cpp:630-738`: Windows sends the exit record with empty filename/key
  to the local named pipe, then reads three reply bytes. It does not validate the
  reply contents or wait for process exit. This tool zero-initializes all unused
  fields instead of copying upstream's uninitialized reserved stack bytes.
- `src/o-charts_pi.cpp:2896-2907` derives `OCPN` plus the OpenCPN PID modulo10000,
  padded to four digits, and starts the helper with only `-p` and that name.

Chart initialization calls `validate_SENC_server` again (`eSENCChart.cpp:612,693`),
whose startup path has no shutdown-state guard. Source inspection supports the
observed possibility of a late chart initialization restarting the decoder after
plugin DeInit; it is not proof of every call in a particular shutdown trace.

The only accepted decoder SHA-256 is
`ec27c947fc9ba4ae09345961780b885deddc2e4fcf094805a14cdf19504e0adb`.
This is the already audited vendor binary from the exact package/source binary
blob. Its internals remain the existing closed vendor trust boundary; no CLI
shutdown option is guessed or executed.

Before invoking the tool, collect the exact helper PID and full precision
`Get-Process.StartTime.ToUniversalTime().Ticks`, retaining the original stock
launch result and request hashes. The tool repeats the complete active stock
commissioning/profile/source/plugin-tree checks, requires every OpenCPN process
to be gone, and refuses any other application/plugin helper. The remaining
helper must match the stock parent PID, original user/session, exact creation,
unique cold-inventory path/hash and source-generated command line. This cleanup
does not bypass or alter the general closed-process preparation guard.

```powershell
& .\tools\boat\stop-chart-helper.ps1 -Workspace C:\XNav `
  -LaunchResult '<exact original stock result.json>' `
  -ExpectedLaunchSha256 '<verified result hash>' `
  -ExpectedRequestSha256 '<verified request hash>' `
  -HelperProcessId <observed helper PID> `
  -ExpectedHelperStartedUtcTicks <exact observed UTC ticks>
```

The native operation holds the exact process handle and an image read lock,
verifies the connected pipe's actual server PID, rechecks identity, and writes
one fixed packet. The modulo-derived name alone is insufficient because two
parent PIDs can share a suffix. Windows exposes the connected server PID through
[GetNamedPipeServerProcessId](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-getnamedpipeserverprocessid).

One 20-second operation deadline covers connection, asynchronous write, three-byte
reply and observed exit. A timeout permits at most five additional seconds to
cancel/drain the tool's own pending pipe I/O; later faults remain observed and
never trigger a retry. The reply is recorded as bytes with
`ReplyMeaningValidated=false`; an apparent reply never substitutes for actual
zero exit of the retained process. Timeout, short reply, identity failure or
nonzero exit remains explicit failure. The pipe is disposed on every path;
managed asynchronous operations retain their buffers through completion.
There is no retry, force-kill or helper relaunch.

A durable private intent plus a deterministic atomic create-new locator prevents
concurrent/repeated attempts for the same PID and creation identity. A failed or
interrupted attempt remains for inspection. Successful helper cleanup still
requires a new full cold `InspectRestore`, final INI review and navigation-data
preservation check before any adoption, restore or next launch.

`test-chart-helper-shutdown.ps1` first runs pure policy/packet/compile checks.
On native disposable Windows it additionally starts only small inert PowerShell
pipe fixtures: normal exit, arbitrary three-byte reply with measured exit,
short/missing reply, nonzero exit, reply timeout, wrong creation/hash/path and a
modulo-colliding wrong server PID. Fixtures receive only the exact fixed packet
or zero bytes and end normally; no vendor helper, OpenCPN or hardware is used.
Portable checks do not establish native pipe behavior or boat acceptance.

## Independent cold cleanup of a bounded existing helper set (SCRUM-223)

`recover-cold-chart-helpers.ps1` is a separate cold path for one to three
already-running managed `oexserverd.exe` instances when no commissioning
transaction or stock launch result exists. It never invents either record. A
parent PID and its pipe suffix come from observed Windows process metadata;
neither proves that the parent was a stock OpenCPN launch. The process owner,
session, exact creation time, managed path, known SHA-256, source-shaped command
line and absent parent PID are all checked again before each operation. A reused
parent PID is refused. Every OpenCPN process and every other application/plugin
helper must be absent. The complete initial candidate set is bounded to three
and must remain unchanged except for each measured successful exit.

`Capture` privately copies the entire existing normal profile and records its
ACL, complete loader trees, installed-generation identity if present, and exact
helper set. It does not weaken the normal closed-process guard used by cold
profile capture and preparation. The operator reviews `capture.json`, then
supplies a separate JSON review file with schema `1`, owner
`OpenNavX.ColdChartHelperReview.1`, `captureSha256`, `reviewedUtc`, decision
`approve-exact-cmd-exit-once`, a nonempty reason, and an exact `candidates`
array copied from the reviewed capture. The tool checks the independent file's
caller-supplied SHA-256; it cannot approve itself. Review expires after 24 hours.

```powershell
& .\tools\boat\recover-cold-chart-helpers.ps1 -Action Capture -Workspace C:\XNav
# Review the private capture and prepare an independent exact-candidate JSON.
& .\tools\boat\recover-cold-chart-helpers.ps1 -Action Close -Workspace C:\XNav `
  -CaptureRecord '<exact capture.json>' -ExpectedCaptureSha256 '<verified hash>' `
  -Review '<independent review.json>' -ExpectedReviewSha256 '<verified hash>'
```

Before each native request, `Close` rechecks the account, installation, profile,
loader trees, private copy/ACL, active-transaction absence and complete remaining
process set. It records a private intent, then atomically reserves the exact
PID/creation identity in the workspace-level private
`chart-helper-attempts` ledger. Existing active-session per-run locators are
also checked. One normal `CMD_EXIT` uses the existing native transport and
actual named-pipe server PID check; its three reply bytes have unknown meaning
and success requires observed zero process exit. Each result, or partial failure,
is retained. Any drift, uncertain delivery, short reply, timeout or nonzero exit
stops the sequence with no retry, forced termination, reboot or replacement
launch. The upstream three-byte reply read is in pinned `Osenc.cpp` around line
800; it is not an acknowledgement definition.

After successful closure, run a **new** normal cold profile capture and review.
The helper closure record provides no profile adoption, plugin restore,
commissioning launch or navigation permission. Portable policy checks and
disposable native Windows fixture/ACL checks are required gates before any boat
use; a CI pass is not boat acceptance.
