# Two authenticated Windows SDK images — SCRUM-209

Both immutable SDK producers passed native compilation and their upstream tests.
The second producer supports the older Windows image still allocated by GitHub.
This evidence-only change records custody and comparison; it does not build,
relabel or qualify an application candidate, installation or release.

| Windows 2022 image | Exact producer source | Run / attempt | SDK artifact | ZIP bytes |
| --- | --- | --- | --- | ---: |
| `20261012.333.1` | `68c835175bdbc441e57aa1201db8f6ca95a7d1e7` | [38618335277 / 1](https://github.com/ThereptileII/Work/actions/runs/38618335277) | `11924018896` | 117,986,124 |
| `20261004.326.1` | `bfbfdca7573c14f4623d2b8b42159abedb99e351` | [38735490052 / 1](https://github.com/ThereptileII/Work/actions/runs/38735490052) | `11976566007` | 117,986,121 |

The original ZIP SHA-256 values are respectively
`08f4fa9ca058564822ba9b69ef19aca0cbf3ea84953657d838e054709ab83ca9` and
`b41dd68dee3ec29b10aa1cedbde93df258140624f5c127932d1cdecfc104a831`.
The [newer-image audit](newer-image-producer-audit.json) and
[older-image audit](older-image-producer-audit.json) retain each complete immutable
selection, bundle hash, runner identity and toolchain digest.

Each original archive passed all **296 member CRCs** and the complete **295-file**
payload size/hash inventory. Both producers passed **4,283 OpenSSL**, **1,569 curl**
and **13 zlib** upstream tests. Source archives, manifests, Win32 PE identities,
maintained TLS imports, dependency bindings and producer source inputs verified.
The newer pin was reauthenticated before preparing the second pin. Original
archives, API responses and logs remain private at the paths and hashes recorded
in the [custody receipt](custody-receipt.json); no binary payload is committed here.

The [source comparison](source-input-comparison.json) verifies every actual SDK
input: 29 declared files plus the producer workflow. All 30 Git blobs are
byte-identical across both producers and candidate source
`e80cb962e1a32d98bb75b964dca6eb516f7a5e23`. The unchanged fingerprint is
`bee5aafd7e78230afc40fece7d36051294aaae681389cc1908a949770e8c8048`.
The fetch selector, its tests, documentation and selected-artifact lock are outside
these 30 inputs. Their changes do not invalidate this producer fingerprint.

Reviewed selector proposal `78c155fda0274af98a1803cfedc573155929b05b` maps the
complete four-field runner identity to one immutable pin. Unknown/duplicate
identities refuse; all existing source, archive, runner, Python, toolchain and
native-consumer checks remain mandatory. This evidence changes no selector or lock.
The [earlier native consumer audit](newer-image-native-consumer-audit.json) applies
only to the newer SDK. Fresh native consumption of the older SDK by the eventual
candidate remains a separate gate; this Linux custody audit cannot grant it.

## Actual differences and the PluginHandler acceptance boundary

The [native-input comparison](native-input-comparison.json) binds the original
native PluginHandler result from run `38704113599` to both authenticated SDKs.
All five recorded native tool executables, Python identity, and all five complete
producer tool-fact documents match. The curl import library and 12 curl headers
match. Nineteen of 24 declared SDK outputs match exactly.

The four runtime DLLs and `openssl.exe` have different raw hashes. Complete inert
byte comparison [locates every difference](pe-byte-differences.json): curl and
zlib each differ in six timestamp bytes; libssl differs in timestamps and its
CodeView GUID; libcrypto adds its OpenSSL `built on:` date string; `openssl.exe`
differs in timestamps and its CodeView GUID. Every other byte matches. The nine
libcrypto bytes listed outside PE fields are all inside that retained date string.
No production verifier normalizes these fields or accepts substituted hashes.

This is evidence of metadata-only rebuild drift. The earlier native 13-case
PluginHandler result still authenticates its original source/runtime inputs;
it is **not exact-byte native acceptance of the older SDK's four DLLs**.
The changed selector/lock also changes provenance inputs independently of the
unchanged SDK recipe. Neither difference permits relabeling an old retained capture.

## Minimum fresh retained native gate

Use the existing `.github/workflows/skager-plugin-download-retained.yml`, inspected
at `e80cb962e1a32d98bb75b964dca6eb516f7a5e23`, once against the new candidate's
authenticated `plugin-guard-inputs` capture. Its exact dispatch input names are:

| Input | Required value |
| --- | --- |
| `producer_run` | New candidate's `opennav-baseline.yml` run ID |
| `producer_attempt` | That exact producer attempt |
| `producer_commit` | That candidate's full lowercase 40-character source SHA |
| `inventory_sha256` | Independently authenticated SHA-256 of that capture's original `inventory.json` |

The entire selected baseline run and its sole `windows-integration` job must be
completed. The exact steps `Compile fixture-free product and run installed security
and unit checks` and `Retain exact plugin-download SDK inputs for no-rebuild
qualification` must have succeeded. The workflow permits a later producer/run
failure while preserving its conclusion; it does not bypass either prerequisite.
The artifact must be unique, unexpired and bound to that source/run/attempt.

Dispatch from a reviewed helper ref whose immutable consumed Git blobs and modes
match this producer. The fetch helper and lock are immutable consumed inputs;
only the existing two probe-file exceptions are allowed. The workflow-selected
SDK authority and retained TLS bytes must equal the capture. If an older-image
candidate's qualifier is allocated the newer image, this equality refuses.
There is no image-selection dispatch input or automatic retry authorization.

The minimum existing job includes the maintained native loader gate and actual
PluginHandler 13-case trust/download fixture, bypass-mutant rejection, owned-trust
removal/cleanup, and required dependency/toolchain reprobes. It builds the narrow
test slice, not the application, and does not run the AIS suite. Its output still
does not grant installed-product, vendor-plugin Init/DeInit, release or boat
acceptance. This preparation executes none of those gates and dispatches no CI.
