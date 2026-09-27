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
