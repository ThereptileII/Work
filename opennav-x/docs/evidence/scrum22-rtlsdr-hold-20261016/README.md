# SCRUM-22: RTL-SDR cold-hold helper fixtures

The corrected task-action helper passed **49 native checks** in
[run 38891027391, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38891027391/job/116940294357)
at `654119a759a4f0a229a67549e8b70bb53c72c214`. The host was Windows PowerShell
5.1.20348.5622, x64, on Windows 2022 image `20261012.333.1`. The separate installer
retest job was skipped. This is an inert maintenance-helper result, not an
installed application or real boat recovery pass.

The native helper SHA-256 is
`9a3ee3714f45145c7d5151254c928e1d1f27c59a55a8b982c197b3b877271ca9`;
the test SHA-256 is
`3fc521946586d20a65e6b458671274e9ec1a35e3398528a11c923f0d2835de15`.
[receipt.json](receipt.json) binds the original artifact, report, logs and five
recorded Git/Windows-checkout source identities for each native job. The latest
native result SHA-256 is
`2d365cd13cc7cab425e9f5c33ca56b2e9f23e2d5de8a96dc99e385cc7bdabd56`.

The original [job in run 38889555934](https://github.com/ThereptileII/Work/actions/runs/38889555934/job/116935466337)
at `3bec26c322ef017aed6e6890b5cbba07989ebebd` remains failed. It stopped at
`Profile mutation is detected before hold` and emitted no native result receipt.
Source review found that the fixture appended a bare token to an INI, making it
malformed while expecting the later protected-tree mutation refusal. Its original
output did not expose the underlying refusal. The correction in
`78e44f568cff4278f0b3b194668f349dff36c531` checks malformed syntax separately,
then changes a valid INI and checks the intended mutation refusal. It also retains
bounded expected/actual refusal text on future fixture failure.

That fixture correction passed **38 checks** in
[run 38890077375, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38890077375/job/116937159472)
at `c639a3147cc2ee597d555dbe2e0e094e879c5c7c`. The actual helper was identical in
those first two jobs: Windows-checkout SHA-256
`051540ec2de4564ee6a55f7e5811545491029ce3ce1ce85fb7a75072885dabe6`
(14,313 bytes). This historical pass remains bound to that earlier helper.

A later real Inspect stopped before backup or hold because a scheduled-task COM
action lacked an `Execute` property. The helper correction in `654119a7` accepts
only the declared native Exec and COM-handler action shapes, validates applicable
fields, and keeps application task-name, explicit protected-path and process
barriers. Unknown or malformed shapes refuse. The 49-case run retains all 38
earlier check labels and adds 11 action-shape checks, including an actual native
Exec CIM object and a client-only native COM-handler CIM object. The fixture does
not register or run a task, load COM, or audit arbitrary COM registration/behavior.

The native fixture first rejects inert bytes with the production DLL and stock
pins. It then substitutes one test DLL hash, inert stock identity and process/task
inventories. It executes the actual helper path, ACL, journal, mutex,
transaction-lock and move code against temporary files, including interrupted
phases, occupied destinations, profile/helper mutation and hardlink/reparse
refusals. The original **eight portable checks** on PowerShell 7.6.7 remain
separate, with `nativeObservation=not-run`; they cover the original byte pin and
supplied actor/task observations, not native filesystem/locking behavior.

These helper/fixture changes did not rebuild or alter the application or installer
product source. The fixtures do not invoke the real-user entrypoint or execute
vendor binaries. They do not establish successful boat Inspect, Hold, Update or Restore.
Fresh operational receipts and installed/profile identities remain separate.
Raw logs, private paths and profile data remain outside the repository.
