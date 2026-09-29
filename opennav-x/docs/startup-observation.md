# Fixture-only early startup observation

SCRUM-98 retains a Linux Legacy-to-XNav restart whose replacement child exited
255 before normal logging. The original failed evidence and the separate
passing syscall-instrumented run remain distinct. This facility makes a future
occurrence diagnosable; it does not fix the failure or qualify a release.

## Scope and data

`src/diagnostics/TestEarlyStartupTrace.h` is compiled only when all of these
hold: `XNAV_ENABLE_TEST_FIXTURES=1`, Linux and wxGTK, with neither Windows nor
wxMSW defined. The product and native Windows entry points remain unchanged.
The integration parser uses no-op macros outside that policy; disabled macros
do not evaluate their arguments.

The fixed stage enum covers entry/return, wx initialization/return,
application initialization/parser return, parser definition/help/error,
OpenNav parsing result, fixed rejection categories, config-directory validation
and log initialization. A record contains only:

```text
OpenNav startup pid=12345 stage=entry.return result=-1
```

This signed return value can distinguish wx returning `-1` from a later OS
exit-status observation of `255`. Neither code alone proves which operation
failed. Trace stages and process-lifecycle evidence must be interpreted
together. No argument, environment value, profile content, position, credential,
bus payload or arbitrary error string is written.

For the inspected wxWidgets 3.2.11 startup, the following observations narrow
the boundary; they do not by themselves establish the root cause:

| Retained stage sequence | Boundary to inspect |
| --- | --- |
| `initialize.return=0`, no `on_init` | wxGTK/base initialization, including `gtk_init_check` |
| `initialize.return=1`, no `on_init`, `entry.return=-1` | Common post-initialization/module setup |
| `parser.return=0` | Help, parse error, OpenNav validation or upstream parser rejection |
| `parser.return=1`, `log_initialize.return=0` | OpenCPN log initialization |
| Missing `entry.return` | Direct exit, exception, interrupted execution or failed observation; no unique cause |

The reference boundaries are wx `src/common/init.cpp` (`wxEntry`,
`DoCommonPostInit`), `src/common/appbase.cpp` (`GetErrorExitCode`, `OnInit`),
`src/gtk/app.cpp` (`Initialize`), and pinned OpenCPN `MyApp::OnInit`. Incomplete
or unwritable traces must not be interpreted as proof that a stage was absent.

## Explicit disposable sink

The test runner must set both variables before launching its owned process:

```text
OPENNAV_TEST_EARLY_STARTUP_TRACE=1
OPENNAV_TEST_EARLY_STARTUP_TRACE_FILE=<absolute disposable directory>/opennav-startup-trace.log
```

That directory must contain a regular, non-symlink `OPENNAV_TEST_PROFILE`
marker. The filename is fixed; relative and other filenames are refused. The
sink opens append-only with close-on-exec, no-follow and nonblocking flags; a
regular-file check precedes each write. New files use mode `0600`. Symlinks,
directories and FIFOs are not written. Existing records survive subsequent
launches. This opt-in development facility is not a permission boundary against
a hostile process which already controls the disposable test directory.

Each process attempts at most 64 fixed records, closing the handle after every
record. There is no fsync, retry, blocking FIFO write or persistent handle.
Observation errors do not escape, alter errno, replace a return value or
prevent the original call. Wrapped operations run exactly once; exceptions
propagate unchanged and therefore do not receive a fabricated return record.
Writing observations can still affect timing. A passing diagnostic run cannot
supersede retained failed qualification evidence.

## Validation and use

The Linux CMake tests exercise fixture, product and wxMSW-excluded compile
policies. Each runs eleven subprocess scenarios: unset/disabled opt-in,
enabled, append, missing marker, wrong filename, relative path, symlink,
directory, FIFO and bounded output. They also check exact stage order, signed
and boolean results, exception propagation, unchanged errno, no disabled
argument evaluation and preservation of unrelated files. The wxMSW-excluded
case is a compile-policy simulation on Linux, not a Windows build.

For an actual diagnostic, retain the exact source manifest, executable hash,
invocation policy, trace and process outcomes. Keep the normal qualification
harness unchanged and label the derived invocation diagnostic-only. Do not
repeat a failed run until it passes, extend timeouts to hide failures or accept
an instrumented success as the repair. Product regression and native Windows
qualification remain mandatory after a real repair is identified.
