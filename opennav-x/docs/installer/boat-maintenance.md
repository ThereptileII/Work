# Boat deployment and maintenance ordering

Complete installation, update or repair before preparing a read-only commissioning
session. After a session, close OpenCPN and its reviewed helpers normally, inspect
the saved differences, and complete the exact commissioning restoration before
any further installation, repair, rollback or uninstall. Changing an installed
generation during quarantine would invalidate the paths and identities required
for restoration.

The boat `install.ps1` and `maintain.ps1` wrappers refuse all mutating actions while
`commissioning-active.json` exists, even if that record is incomplete. They check
before target access or evidence creation and again before invoking maintenance.
Do not delete this marker to bypass the check. Read-only diagnostics remain
available. Operations must be serialized; these wrappers are not a substitute
for coordinating separate Windows setup processes.

`repair.ps1` and `uninstall.ps1` can inspect an owned installation when its managed
`app/opencpn.exe` is missing or corrupt. This exception only permits offline
Repair/Uninstall: it never authorizes running that executable. The state owner,
generation identifier, ownership record and expected executable hash record must
still be valid, and `maintain.ps1` independently verifies the exact retained
`Lifecycle.ps1` before invoking it. A damaged maintenance engine requires the
verified original Setup. Normal launches, captures and commissioning identity
checks continue to require the exact installed executable hash.

The underlying lifecycle engine stages and verifies replacement files, then
publishes a new generation. It retains modified files during uninstall and never
restores old navigation data over the current shared profile. Silent boat setup
does not run the Finish-page launch action. Its staged loader self-test exits
before profile initialization or plugin loading.

`test-boat-tools.ps1 -PortableContracts` passes five Linux maintenance groups.
The complete native suite passes 27 groups, retaining the original 21 and adding six, including
all six mutating wrapper entrypoints refusing active commissioning before any
target/process access. The boat's native temporary-file run is recorded privately
in `evidence/local/boat-beta2/native-maintenance-repair-final.json`; the existing
launch-verification suite also passes eleven Linux groups. These temporary
fixtures do not qualify a real installation or hardware session.
