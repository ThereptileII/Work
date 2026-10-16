# Temporary hold of the reviewed incompatible stock RTL-SDR entry DLL

Frozen candidate `854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc` correctly refuses
`rtlsdr_pi.dll` imported from stock OpenCPN: its wxWidgets 3.1 imports are absent
from the candidate's wxWidgets 3.2 closure. The selected old generation does not
contain that entry DLL. This procedure keeps the installer's complete PE checks.
It does not qualify the plugin or its helper executables, or supply old wx DLLs.

`tools/boat/hold-incompatible-rtlsdr.ps1` handles only the reviewed 208,384-byte
stock DLL with SHA-256
`404e69931034140fbda0179964d0a29077a46b78a441c05fc68a4ee0616a64e5`.
The existing preparation validator must establish the official x86 OpenCPN
5.12.4 executable. Arbitrary plugin names, hashes and source paths are unsupported.
See [the RTL-SDR review boundary](boat-rtl-sdr-review-boundary.md).

An operator first closes applications and helpers normally and resolves scheduled
launch tasks. No commissioning/manual session, pending transaction, signed-update
armed marker, custom plugin root or concurrent maintenance process may remain.
The helper acquires the existing commissioning profile mutex and installation
transaction lock; it never creates an installed lock or changes a denial marker.
It does not elevate, stop processes, disable tasks, run Setup, or launch any DLL,
helper or application. Administrative access to the stock DLL must already exist.

Use the maintained private recovery workspace on the same volume as stock, outside
all profile, application, managed-plugin, installed-generation and PATH roots:

```powershell
.\hold-incompatible-rtlsdr.ps1 -Action Inspect -Workspace $workspace
# Retain the returned private plan path and its exact SHA-256.
.\hold-incompatible-rtlsdr.ps1 -Action Hold -Workspace $workspace `
  -Record $plan -ExpectedRecordSha256 $planSha256
# Operator runs the separately authorized maintained Update exactly once.
# Restore immediately after it finishes, whether Update succeeded or failed:
.\hold-incompatible-rtlsdr.ps1 -Action Restore -Workspace $workspace `
  -Record $plan -ExpectedRecordSha256 $planSha256
```

`Inspect` creates a private backup and immutable plan. Compact complete-inventory
hashes cover the profile, stock tree except the single DLL, managed plugins and
original installed generation. `Hold` persists intent before a same-volume,
non-overwriting rename outside import roots. The DLL's bytes and access ACL are
preserved. Its adjacent helper directory, old generation, stock executable and
profile are never changed. The private backup and journal remain for recovery.

`Restore` verifies those same protected trees and the current installed identity;
a legitimate new selected generation is permitted after Update. It never overwrites
an occupied destination. Durable intents plus exact bytes/ACL distinguish a hold
interrupted before its move, a hold interrupted after its move, and restoration
interrupted after its move. Other ambiguity, corruption, changed protected files,
redirected paths, hard links, busy locks and unsafe session state remain failures.
Retain the journal and investigate; do not delete barriers or manually invent a
success receipt. There is no automatic backup-copy fallback or retry.

Do not launch stock or SKAGER while the DLL is held. This cold procedure coordinates
with maintained tools; it is not a security boundary against an administrator
concurrently rewriting the filesystem. If Update finds another incompatible PE,
retain that original refusal, restore stock and review the next actual cause.
Restoration is not acceptance of the old RTL-SDR plugin: take a fresh installed
inventory and apply the separately reviewed read-only commissioning quarantine
before any authorized launch. Existing commissioning attestations do not transfer
to a newly selected generation.

## Focused evidence boundary

`tools/boat/test-hold-incompatible-rtlsdr.ps1 -Evidence <fresh-result.json>` is the
native Windows PowerShell 5.1 fixture. It runs only with disposable CI or explicit
`-IsolatedLocal`, using temporary inert bytes. Production stock and DLL pins must
reject those bytes; a clearly labeled test-only function derivation substitutes
one inert DLL hash and stock identity to exercise the actual journal, hash, ACL,
move, lock and guard functions. No vendor binary is executed. `-PortableContracts`
on Linux covers only the portable refusal checks, never native qualification.
Native validation must pass before this helper is used on the boat.
