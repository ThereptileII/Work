# SCRUM-311 — protected approved-baseline loading view

Production component `de86d6fa541d02f2255bc22a90116d116f5413e0` is integrated as
`d3d653ba`; that integration also contains the exact runner allowlist from
`693fcef5`. Root compared all six component source hashes against the integrated
files before native execution. Test binary SHA-256:
`f5ea9aba6515a9b78f342e82659cae7c013d9d7cec1a7f182d67b523e769c200`.

## Actual boundary

`Store.MaterializeApprovedBaselineFresh` copies the live same-Store completed
initial baseline into a fresh protected public generation derived from its
approved identity. It retains the fixed original completed registration and
marker, exact independent request approval, original inventory/manifest/product
and all three execution helpers. Source/destination full physical identities
and exact contents are checked. Existing or partial destinations refuse.

Distinct typed baseline completion and read-only owners cannot be interpreted
as signed-candidate policy, Recovery or Setup identities. Strict purpose dispatch
accepts only the two implemented provenance shapes and rejects mixed/unknown
records. The existing one-way transition supplies independently read-only
loader-compatible file objects; public custody survives closure of the original
private source. No launch path or native handle is exposed to clients.

## Evidence

- Linux focused generation race: **12 top-level + 65 nested passed**, including
  **4 top-level + 27 nested** new baseline cases; no final failure/skip.
- Windows/x86 component compilation passed. This is not an MSVC application build.
- Native Windows 10.0.22631: **3 top-level + 5 nested passed**, no failure/skip.
  Exact original copies, distinct source IDs, read/execute opens, write/delete
  refusal, strict mixed-purpose history, registration mutation and partial-view
  refusal were exercised on disposable temporary objects.
- Exact binary/runner/archive hashes, native parsing and case discovery,
  bounded execution, result retrieval and transfer hash passed.

The initial portable run panicked in a shared copy helper because its internal
baseline copier had no backend. Binding it to the actual retained baseline
backend repaired the source check. The initial failure log and final check
receipts are retained; native execution used only the corrected exact binary.

Installed OpenCPN PID1852 remained responsive; no owned test process remained.
No installed product, profile, service, remote-access setting or physical output
was changed. Raw Windows line endings are preserved.

## Remaining integration

This view is not a selected or startup-qualified previous version. The three
helpers are not a complete commissioning execution closure. Atomic authenticated
live sibling-history evolution, actual selected/pending state, interactive
executor binding, commissioning/restoration, one-use process creation and
authenticated healthy/fallback transactions remain required. The native
mixed-purpose test closes the first public owner before adding its sibling; it
does not claim simultaneous retained-owner acceptance. Full same-package
Windows/installer/boat qualification remains open. No full application build or
release was dispatched for these component checks.
