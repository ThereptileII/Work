# SCRUM-311 — one-way read-only generation custody

Production component: `fcec6f21433543f3e4f92f84fd451ca7f28e0436`, integrated as
`bb2dde1b`. Exact native runner source: `3235f8513bf87511880dc8abdfb63b0d87499ffc`.
The component receipt records all six source hashes; root verified these against
the integrated files before recording evidence. Test binary SHA-256:
`6f9d715ecc2eac3a3a2757a05a32b1148dca59389ea546a4fe7298898f78f820`.

## Implemented

`InstalledGeneration.TransitionReadOnly` permanently consumes the writer. It
closes each native writer file object and opens a separate read-only file object
while original metadata, parent IDs and birth-protected namespace remain held.
This does not assume reduced-access duplicate handles change sharing semantics.
Original private parent/reservation/completion records also become read-only.
Payload IDs, original bytes, descriptors and exact namespace are revalidated.

The owned `ReadOnlyGeneration` exposes bounded revocable member reads, identity
and completion information, without raw writable/native handles or client paths.
Store/original-owner close revokes it; transition failure poisons and drains once
without deletion, adoption, ID reuse or retry.

## Results

- Linux focused generation race: **8 top-level + 38 nested PASS**, no fail/skip.
- Windows/x86 component compilation: PASS. This is not an MSVC app build.
- Exact native Windows fixtures: **3 top-level + 3 nested PASS**, no fail/skip.
  The copied own test PE maps with `SEC_IMAGE` but is never executed; read/execute
  opens work, final handles cannot write, new write/delete opens refuse, original
  IDs/content remain equal. Actual writer-close cancellation preserves completed
  records/marker, and payload/private-history/namespace mutations refuse transfer.

Native runner parsing, discovery allowlist, exact transferred hashes, bounded
execution and result-transfer hash passed. Only fresh component scratch/temp
files were used. Installed OpenCPN PID1852 remained responsive; no owned test
process remained. Raw Windows line endings are preserved.

## Remaining actual integration

This capability is not selected-version or launch authority. The existing
strict sibling snapshot must support independently authenticated completed
successors while previous custody remains live, without accepting unknown or
partial state. The initial approved baseline still needs a public protected
loading view. Actual protected pending/current selection, interactive identity,
commissioning/restoration, one-use spawn and authenticated startup/fallback
remain required. No service was provisioned, application launched, installed
profile changed, or physical command sent. SCRUM-311 and full release gates
remain open.
