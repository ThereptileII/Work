# SCRUM-264 — audited recovery-package native capture supplement

Collector-only change based on application source `1835d1b`. It does not rebuild,
modify or qualify the frozen `9d98a500916e8a7f59dac9735427dde6d3c7d2e5`
application. No native launch, CI dispatch or boat action was performed here.

The existing collector requires `include/config.h`, which the native package
artifacts do not retain. `package-preview.py` already copies its exact version
and date into the hash-bound packaged `profile/opencpn.conf`. The explicit
recovery mode validates the independently audited original ZIP hash, every ZIP
CRC, the ZIP/extracted file inventory, all payload hashes, `FILE_SHA256.json`,
`PRODUCT_BUILD.json`, expected executable and application commit, fixture-off and
status-only policy. Links/reparse paths, path escape/ambiguity, missing or extra
files, duplicate JSON/config keys and mismatched identities are refused. ZIP
entry metadata independently refuses symbolic links, special file types and DOS
directory/reparse attributes; unspecified or explicit regular-file types are
allowed. The portable marker must contain the exact package-preview text (LF or
native CRLF). Non-object product JSON is rejected by the shared status-only guard.
Only `ConfigVersionString` is read from that packaged profile into a NEW test
profile. Connections, profile plugins, credentials, routes and preferences are
not copied. Immutable bundled binaries under `app/plugins` stay byte-identical.
The existing `--build` mode remains available. Recovery mode refuses app/build
overrides and requires disposable Windows plus `--navigation-only`.

## Invocation after independent artifact audit

Run the exact collector checkout on a disposable GitHub Actions Windows desktop,
serially, using the original portable ZIP and its unchanged extracted directory.
All hash arguments must come from the independent same-run artifact audit, not
values inferred by this invocation. `GITHUB_ACTIONS=true` is an existing required
runner observation, not a switch to authorize use on another computer.

```powershell
python tools/prototype/capture-native.py `
  --recovery-package <extracted/SKAGER-Beta2-Portable-Recovery> `
  --recovery-archive <SKAGER-Beta2-Portable-Recovery.zip> `
  --expected-recovery-archive-sha256 <audited-zip-sha256> `
  --expected-file-manifest-sha256 <audited-FILE_SHA256.json-sha256> `
  --expected-executable-sha256 <audited-app/opencpn.exe-sha256> `
  --expected-application-commit 9d98a500916e8a7f59dac9735427dde6d3c7d2e5 `
  --navigation-only --public-enc `
  --chart-style XNav --renderer software --output <new-output-directory>
```

For the official IHO scene replace `--public-enc` with
`--iho-s64 <retained/GB4X0000.000>`. Repeat each scene only for the selected
`XNav`/`Standard` and `software`/`opengl` combinations. Each invocation captures
Day → Dusk → Night → Day return and requires a normal exit. Requested GL must
actually be enabled; software fallback cannot pass. No fixture-enabled app or
synthetic chart object is required. There is no live/simulated input feed.

The IHO input is exact SHA256
`c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`.
It is official S-64 presentation-test geography, **not an operational nautical
ENC**. The retained e1 collector's actual pair is BOYSPP 254 (yellow, shape 3,
CATSPM 27) and TOPMAR 257 (yellow, TOPSHP 7), both at latitude `-32.3471615`, longitude
`61.169588`. Requested scale is 0.6; actual upstream observed scale must remain
`0.5826126536`, with the single exact cell/quilt, canvas 1014×566, no follow and
saved/effective Simplified table 76. Input is copied into a fresh isolated fixture
directory; source and staged cell are rehashed after shutdown.

## Assertions and retained evidence

`capture.json` records collector source SHA separately from the expected and
observed application SHA, executable/package identities, input provenance,
renderer, themes, controls and exit. Every screenshot has copied runtime
observations. Core/private observations are checked separately; this public-ENC
review requires the private adapter to remain explicitly unavailable. The former
incorrect `XNav ` status-prefix assertion now requires the exact existing
`SKAGER presentation v1 / pinned symbols` status and unchanged no-adapter suffix.

Existing native layout/paint/control and renderer assertions remain. Both scenes
now require exact unmasked whole-chart Day return and retain the difference on
failure. IHO additionally retains a 72×72 source-pair crop and compares predeclared
body/head rectangles with source-hashed, renderer-specific retained e1 images.
The fitted-head rectangle excludes the co-located light at the center; no color
search, movable sample or inferred debugger observation is used. Standard uses
its own original-mark reference. These are strict new Windows pixel comparisons:
any platform raster/density difference fails for inspection, rather than becoming
an automatic tolerance. A matching crop does not claim an MSVC renderer-entry
trace or runtime alias-name observation.

Fourteen focused offline unittest cases passed in 0.613 seconds; Python compilation,
CLI help and `git diff --check` also passed. Offline tests exercise valid identity/version-only profile creation, mismatched
external identity, altered fixture/output policy, missing/extra/modified payload,
ZIP versus manifest inconsistency, forged ZIP link/FIFO/device/reparse/directory
metadata, wrong marker, non-object product JSON, path/link/duplicate refusal and wrong cell
refusal. All 12 retained renderer/style/theme probe images pass their exact
reference. Erasing the fitted head or body fails. The known retained Standard GL
whole-chart Day-return shift remains a failing negative control. Tests do not
execute the identity fixture's dummy executable bytes.

## Boundaries before execution

This harness requires the disposable CI desktop and a new disconnected profile;
it is not an OS network/device sandbox. The existing status-only application
policy is checked, and runtime pilot/control state must remain disabled. Bundled
plugin discovery and OpenCPN startup are not a claim of universal I/O silence.
Run serially with other app tests because upstream uses a fixed local REST port.
The audited extracted package remains unchanged between runs. Each capture uses
`output/disposable-package/app/opencpn.exe` with its own sibling `profile` and
`logs`. The marker stays present, so the unchanged application enforces portable
isolation. All immutable copied files/directories are sealed before and after;
only these two fresh runtime trees may change. Links/reparse/hardlinks/special
files and case aliases are refused, including within the mutable trees. The
copied original `FILE_SHA256.json` is documentary: its original profile/log
hashes do not describe the intentionally fresh profile. No install occurs.

The original ZIP and extracted package receive the same complete final checks.
Copied binaries, profiles, logs and chart input/database files are excluded from
the evidence artifact. Only the bounded disconnected-session OpenCPN log and
final diagnostic snapshot are copied out explicitly; normal screenshots and
per-capture observations remain retained.

No generic `_bcngn`/`_slgto` scene or initialized private o-charts canvas is added.
Windows font-face tracing, physical GPU, helper/licensed chart execution and boat
acceptance remain outside this supplement. The native first execution is still
pending; passing offline guards does not establish native acceptance.

## Pre-dispatch production-boundary correction — 2026-10-03

Source inspection caught two deterministic integration errors before the first
native collector dispatch. The former collector pointed at the original
portable executable but requested `output/profile`; production `PreviewProfile`
correctly rejects that external path. It also looked for the diagnostic JSON in
the profile, although portable `ParseCommandLine` explicitly assigns the package
`logs` directory. `OPENNAV_TEST_PROFILE` never bypasses either production rule.

The corrected collector launches the copied payload described above and polls
its actual portable logs path, including timestamp/freshness checks. Generated
notice and 1280×800 defaults are retained by the existing `new_profile` helper;
no old profile settings are transferred. Upstream `BasePlatform` still writes
`opencpn.log` to `GetPrivateDataDir()` / the explicitly requested profile, which
remains the initialization check. Application source is unchanged.

Twenty offline cases pass (the original fourteen plus six copy/immutability
cases). A tiny CMake probe links the **unchanged production PortableProfile.cpp**:
four boundary checks reproduce the former invocation's refusal, accept the
copied executable's explicit/default own profile, still reject another external
profile, and check own logs plus both original/copy immutability. The identity
fixture's dummy executable is never run. The retained
[Linux result](portable-boundary-linux.json) is a path-contract check, not an
OpenCPN launch or native qualification. The dedicated Windows review workflow
compiles this same tiny probe with MSVC Win32 before any actual package capture;
it does not rebuild the application. All renderer/pixel/Day-return gates remain
unchanged. Focused CLI/Python/diff checks also pass. Initial local CMake discovery
needed the existing bundled runtime and library path; no application failure
was involved.

## First native preflight and canonical-path repair

Local `9bdc060cbbb9fb92a5d2e9f53c2c56c4e3436053` was mapped to remote
`27828302a1c83ecc1f187ae1d79ab9ab5ac0fdfd`, independently reconstructed tree
`d66a90bc6b8aba61259243cf6b4b8b4b748dd066` (4,698 entries).
[Run 37130634759](https://github.com/ThereptileII/Work/actions/runs/37130634759)
ran the small collector checks only. It failed before configuring/compiling the
C++ probe or launching OpenCPN: 20 Python cases, two failures and four errors,
zero skips. All six failures stop at the helper's comparison of a resolved
package path to an only-absolute audited receipt path.

The original artifact `11276721827` is retained as
[original.zip](native-27828302/original.zip), 1,977 bytes,
SHA256 `77a8d8c769c95ba9b86aacb95ab38c7fd17653eb0e230ebe6560dbc917f0509a`;
all three entries pass CRC. The [independent audit](native-27828302/audit.json)
checks exact run/commit and all seven source hashes against their actual Windows
CRLF checkout bytes. The original [job log](native-27828302/job.log) remains.
The log does not show both path spellings: Windows short/long TEMP spelling is a
possible cause, not observed proof. The asymmetric canonicalization is the
confirmed code defect.

The narrow correction checks the receipt's original path for links/reparse
points **before** resolving it and then compares both canonical paths. No
case-fold bypass, original audit relaxation or product change is introduced.
A real `package/app/..` spelling reproduces the defect without mocks; different
roots and linked receipt paths remain refused. All seven focused staging cases
and the four production-boundary checks pass locally. The next native suite has
21 cases.

## Corrected native path gate passes

Local `dd58c8d03903b712d9f6b1967db0ac277690b1a3` maps to remote
`3804b1f38143be6229b26d5bb2bca21181ca16dc`, verified tree
`33f506fab7a86e08895917d075bd2ab99fc00be1` (4,701 entries).
[Run 37130959195](https://github.com/ThereptileII/Work/actions/runs/37130959195),
job `111225795821`, attempt 1 passes: 21 Python cases with zero skips and all four
actual MSVC Win32 production path checks. The source/probe compiles with MSVC
19.44 x86; this is not a rebuild or launch of OpenCPN.

The independently downloaded [original artifact](native-3804b1f/original.zip)
`11277045734` is 2,650 bytes, SHA256
`36679a5f5aa7c28d3dd775e18e41607aa0347a8632bc8c2fc65d23f7cf4628ab`.
All seven ZIP entries pass CRC. The [audit](native-3804b1f/audit.json) verifies
all seven recorded source hashes against exact Windows CRLF checkout bytes;
the complete [job log](native-3804b1f/job.log) is retained. The probe hash is a CI
observation because the probe executable itself is not uploaded; no independent
binary rehash is claimed. This closes the collector path defect, not chart
rendering, application installation, physical GPU or boat visual acceptance.
The dedicated actual-package capture still requires the separately audited
candidate and its explicit target record.
