# One-use commissioning restart verifier

Development implementation only. The portable policy/parser/journal suite passes
265 checks, plus the separate 77-check AUI/Dashboard policy suite. Native process/pipe tests, full real-application shutdown tests, and
boat acceptance remain separate gates. These scripts are not enabled in the
currently published application and must not be used to qualify an older build.

This opt-in procedure extends an already prepared, independently reviewed,
read-only commissioning transaction. It does not replace cold-launch safety
checks, restore a profile, modify connections, operate equipment, or instruct the
user to make an unreviewed mode switch.

## Prerequisites

* A fixture-free installed generation whose owned `PRODUCT_BUILD.json` contains
  integer `commissioning_restart_protocol: 1`, qualified against the actual
  application and companion helper capability probes, with both exact hashes. A version string alone is
  insufficient. Older helpers might ignore guard environment variables.
* Exact installed executable, helper, build record, active commissioning record,
  immutable quarantine and complete retained plugin/helper inventories.
* A fresh independent shutdown review covering every retained plugin by exact
  identity and source revision. The copied review retains the closed o-charts
  helper limitation; this is not a claim to have audited that vendor binary.
* Existing connection/output, profile, source review and plugin gates pass.
  No application or chart/plugin helper is running during preparation.

The new session is private, expires after four hours and permits at most sixteen
explicit transitions. The normal `boat-target.json` audit is never rewritten.
Each script dependency is hashed in the immutable session record. Changing any
verifier, policy or launch script requires a new cold session.

## Operator sequence after qualification

1. Close the application normally and finish the ordinary full commissioning
   review. Retain the independently reviewed shutdown-record hash.
2. Prepare, without launching:

   ```powershell
   $prepared = .\RestartCommissioningPrepare.ps1 -Workspace C:\XNav `
       -ShutdownReview $reviewPath -ShutdownReviewSha256 $reviewedShutdownSha |
       ConvertFrom-Json
   ```

3. Launch one cold application through the usual interactive task, now with the
   explicit optional binding:

   ```powershell
   .\run-mode.ps1 -Workspace C:\XNav -Mode XNav `
       -RestartSessionRecord $prepared.record `
       -RestartSessionSha256 $prepared.recordSha256
   ```

   Both ordinary dispatch and actual interactive execution recheck the full
   audit. The optional binding is validated against that result. Only two fixed
   environment variables are added. The private cold-launch journal is consumed
   before `Process.Start`, and its actual child PID/creation time are recorded.
   A failed or uncertain cold launch cannot reuse this session.

4. Before one human mode-switch action, deliberately arm the broker for the
   exact live parent PID and creation time, and one of `--xnav`, `--legacy` or
   `--safe-mode`. The broker must run as the same interactive user/session in
   native `System32\WindowsPowerShell\v1.0\powershell.exe`:

   ```powershell
   .\RestartCommissioningArm.ps1 -SessionRecord $prepared.record `
       -ExpectedSha256 $prepared.recordSha256 `
       -ParentProcessId $inspectedParentPid `
       -ParentCreatedFiletime $inspectedParentCreatedFiletime -Mode --legacy
   ```

   The fixed dispatcher creates one limited task for the already logged-in
   account; it may be called through the existing SSH alias. Wait for
   `listening-for-one-explicit-restart` before the separately reviewed UI action.
   It never sends that action. Running the underlying broker directly in SSH
   session 0 is refused. After the broker finishes, invoke the same arguments
   with `-Action Collect` to remove only the exact owned completed task. No
   listener, expired session, wrong parent or refused configuration diff means
   no replacement child. An ordinary close is not an in-app restart test.

5. Review `child-identity-verified`, the actual native window/chart and the
   private transition evidence. For another transition, explicitly arm a new
   broker for the last verified child. The preceding exact post-close copy is
   the next baseline; a historical parent cannot reuse it.
6. If denied or uncertain, inspect the records and processes. Do not retry,
   force-close, renew an INI hash or bypass a guard. Re-establish a complete cold
   review where appropriate. Finish with the existing commissioning restoration
   procedure only after every application/helper is closed.

## Configuration proof

`Read-RestartIni` preserves values exactly, including whitespace. Duplicate or
case-ambiguous sections/keys, malformed syntax and invalid UTF-8 are refused.
The ordinary compatibility parser cannot silently normalize a protected change
away. Removed keys are always refused.

The closed typed policy permits only these reviewed keys:

| Scope | Keys | Type |
| --- | --- | --- |
| `OpenNav` | `InterfaceMode` | Exact explicitly requested XNav/Legacy mode; Safe keeps the previous value |
| `Settings/GlobalState` | `FrameWinX/Y`, `ClientSzX/Y` | Integer size 1–32768 |
| `Settings/GlobalState` | `FrameWinPosX/Y`, `ClientPosX/Y` | Integer -32768–32768 |
| `Settings/GlobalState` | `FrameMax` | Exact 0/1 |
| `Settings/GlobalState` | `nColorScheme` | Day/Dusk/Night integer 1–3 |
| `Settings/GlobalState` | `OwnShipLatLon` | Finite cached coordinate, latitude ±90 / longitude ±180 |
| `Settings` | `Fullscreen`, `ShowStatusBar`, `ShowMenuBar`, `ShowCompassWindow` | Exact 0/1 |
| Each of the two explicit `Canvas/CanvasConfig1` and `CanvasConfig2` sections | `canvasVPLatLon`, `canvasVPScale`, `canvasVPRotation`, `canvasSizeX/Y` | Finite bounded viewport position/scale and integer rotation/size |
| Those same two sections | `canvasbFollow`, `canvasCourseUp`, `canvasHeadUp`, `canvasLookahead` | Exact 0/1 |
| Those same two sections | `canvasInitialdBIndex` | Cached reference index -1–1000000 |

Source: pinned OpenCPN `gui/src/navutil.cpp`, `SaveConfigCanvas` and
`UpdateSettings`, plus OpenNav `PrepareClose`. The cached ownship coordinate is
normal OpenCPN persistence and does not make a missing live position valid. The
chart index does not change chart sources or grant chart correctness.

The exact `AUI/AUIPerspective` value has a separate bounded wxAUI parser: the
same pane names/captions and capability bits must remain; only reviewed geometry,
dock layout and visibility state can vary. A missing/malformed baseline is
refused. The exact Dashboard distance counter may advance finitely and
monotonically within its bound; the twenty explicit Dashboard pane-size keys have
bounded integer geometry. These helpers do not grant a plugin-prefix exception.

Everything else remains unchanged, including every connection field, other plugin
setting, chart directory, catalog/source, routing/output setting, sound command,
control permission and model setting. Further legitimate shutdown changes can
cause a refusal. Inspect exact upstream writes and add narrowly typed tests
before extending this list; never allow a `Settings/*` prefix. First-run version
migrations require a separately inspected cold warm-up and baseline review; a
restart session never silently authorizes `ConfigVersionString` changes.

## Single-use authorization

The server holds the exact parent process handle and verifies its normal exit.
Its local pipe rejects remote clients and restricts access to the current SID,
SYSTEM and Administrators. The OS-reported client must be the installed helper
in the same session with the exact creation time. The strict bounded UTF-8 JSON
parser rejects duplicate keys, unknown fields, nesting and noncanonical numbers.
The native client verifies the server process identity independently.

After a typed diff, the server uses an in-memory audit copy with the proven new
hash and invokes the full existing cold verifier, including every retained
plugin/helper tree and quarantine. It rechecks all immutable evidence, expiry,
profile hash and peer before issuing a ten-second permit. The exact consumed
journal is durable before `ALLOW`. The helper locks/rechecks critical files and
permits one child only. A missing/failed receipt remains consumed and uncertain;
there is no fallback to an unguarded restart.

Request bytes, profile copies, exact deltas, consumed permit and child receipt
remain in the private session directory. They can contain vessel/user information
and must not be committed or published as generic CI evidence.

## Tests and limits

`test-restart-commissioning.ps1` runs pure policy, strict parser, actual C# framing
and temporary-file journal tests. It never opens a pipe, launches an application,
uses a real profile or emits marine data. Native local execution requires
`-IsolatedLocal`; default Windows execution is CI only.

The separate native helper suite must validate local peer identities, inherited
parent handle, timeout/denial, one-use behavior, exact launch environment and file
locks. The actual application must additionally be tested with reviewed normal
shutdown persistence, retained plugin unload, mode veto, XNav/Legacy/Safe cycles,
missing listener and uncertain receipt. Linux policy tests cannot replace those
Windows and physical-display gates.

### Full native broker fixture

`tools/test-commissioning-broker-windows.ps1` is CI-only. It first requires the
compiled marker-only byte signature and capability probe, then runs the exact
unchanged broker with the actual native companion and marker processes. Copied
dependencies substitute only known-folder/installation identity inside a marked
random TEMP directory. Synthetic ownership and source-review records are test
fixtures, never real boat attestations. The complete quarantine/tree verifier,
strict profile diff, real PID/session/parent handle, pipe, permit and receipt
logic still execute.

Cases cover success, changed output direction, retained plugin bytes, expired
session, previously consumed transition and unavailable durable receipt storage.
The last case must retain an uncertain consumed permit even though one actual
marker child exists. A separate inert ScheduledTask checks the exact native
principal/action representation used by Arm/Collect. This is not a test of an
installed navigation executable or the real boat scheduler/profile environment.
The fixture constructs its immutable session directly; it does not qualify the
production Prepare command or the Arm/Collect process-launch sequence.

Both the helper/marker transport and maintenance suites must pass first in the
same-commit tooling workflow. The full broker harness is
implemented and parses; its native execution is still pending. No result is
inferred from the fixture's construction or from portable parser tests.

### Native Prepare and Arm/Collect fixture

`tools/test-commissioning-prepare-arm-windows.ps1` separately executes the actual
Prepare and Arm/Collect entrypoints. Their copied bytes, and the broker's copied
bytes, must match the source exactly. Prepare creates the real private session,
copies the baseline and shutdown review, and hashes the dependencies. The test
also supplies an owned but incapable build record and requires refusal before
session creation or application launch.

Arm creates the actual limited, interactive scheduled task for the native
PowerShell broker. Tests exercise wrong-parent identity, duplicate arming,
premature collection, successful authorization and output-connection refusal.
Collect must remove only its exact completed task and publish its durable record.
No fixture force-terminates a process or task.

Scheduled tasks do not inherit the worker's temporary environment. Only the
copied test identity adapter therefore contains a fixed fixture digest/seal; the
copied known-folder identity check uses that same synthetic TEMP location. Every
path remains inside the unique marked fixture tree. Actual process identity,
SID/session, task creation, private ACL, full plugin/quarantine audit, typed INI
proof and named-pipe logic remain active.

The initial marker parent and its cold-child journal are still fixture inputs.
This test does not qualify the product's interactive cold-launch screen, real
installation or boat scheduler. The marker's optional scheduler hold is bounded
at 25 seconds; the native helper's 30-second parent timeout is unchanged. The
harness is implemented and parses; native results remain pending.
