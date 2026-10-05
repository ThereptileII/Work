# SCRUM-247/259 — native chart typeface objects at 9dee9b1

Both bounded native jobs passed on **2026-10-03**. Independently downloaded
artifacts were checked against exact local source
`9dee9b148f4d6ebdd20bb4c49229fe19340df209`, including `ChartTextFace.h` and the
explicit geographic/LIGHTS null-fallback policy. This receipt does not
qualify later source changes or replace a full Windows application gate.

| Scope | Exact run / job | Commit | Artifact / independent verification |
|---|---|---|---|
| Private adapter: path regression plus 70 production objects | [37113412453 / 111175522188](https://github.com/ThereptileII/Work/actions/runs/37113412453/job/111175522188) | `aa95750b0abbd9ffa646407019b3393f2e7b5bce` | 11270921706; 3,225,843 bytes; 443 CRC-clean ZIP entries |
| Core: 23 chart production objects | [37113614021 / 111176073676](https://github.com/ThereptileII/Work/actions/runs/37113614021/job/111176073676) | `44b80a6902c5293913711a9238cb61f7af6b5030` | 11271266499; 5,546,984 bytes; 516 CRC-clean ZIP entries |

Private artifact SHA-256:
`32ba54cd2bd9f0806783abede24ec00d8008b23980835d7e664661eda323825d`.
Core artifact SHA-256:
`3e1037bf3123699a2424b0550b7f9af25f7968a1b3cda3ecb75c334e5183e71a`.
Both match GitHub's separately fetched digests.

The two commit IDs differ. Independently fetched Git commit metadata confirms
`44b80a` is a child of `aa957` and **both have the identical tree**
`ffadb958481063b613d08a20d10bc6e37ce1a5b5`. The child changes only commit metadata
with `[chart-units]` to select the intended workflow mode. The identity receipt
is retained alongside the run evidence.

## Independently inspected evidence

**Private:** all 70 ordinary x86 COFF objects, ten generated project source
lists, actual compiler commands and 70 retained source copies match their
receipts. All 219 pinned original/derived source files were reconstructed and
compared, as were 17 current owned files. Product/probe/lock inputs match the
exact local revision, allowing only known Git LF→CRLF checkout conversion.
The original native backslash/space path still reproduces the original invalid
`\a` escape; normalized untyped and typed paths both pass. The unchanged
`DECL_IMP` macro redefinition remains a warning, not an import/link proof.

**Core:** all 23 x86 objects, generated project sources, source copies, two
configuration headers and seven resources match their retained hashes. All
460 local inputs match (436 known CRLF conversions, 24 exact); all 1,439 patched
upstream inputs match the existing exact 9dee Linux source tree (1,406 CRLF,
33 exact). Actual projects preserve production mode/GL definitions with test
fixtures and pilot loopback disabled, and have no NOMINMAX or forced-include
mask. The command targets exactly the recorded 23 units.

Both use native MSVC 19.44.35229, Windows SDK 10.0.26100.0, Release /MD and
C++17. All seven generated resource/header identities independently match the
completed local build, including manifest
`12cfbce4686fce9a89e15f45105195c9f09a51ae4e3e9932aa7ba803ae205a52`.
No new local build or resource generation was needed for this comparison.

The SDK/header payloads were not downloaded again: the 1,629 private SDK file
hashes and 1,560 core SDK header hashes remain runner evidence. The private
probe uses independently locked actual public headers, never fabricated
maintained dependency or preparation manifests.

## Workflow selection and qualification limits

The earlier [default run 37113413306](https://github.com/ThereptileII/Work/actions/runs/37113413306)
at `aa957` passed but **did not select chart mode**. Updating the branch alone
was insufficient because chart mode requires the dispatch flag or message
marker. Its completed log is retained in `unintended-default`; it contributes
no chart qualification. The corrected chart-mode run above is the relevant
proof. No private job was repeated for the metadata-only commit.

These gates compile actual core/private objects. They do not link the final
private DLL or integrated application, validate a maintained producer package,
resolve real host imports/exports, execute the installed host-module diagnostic,
or accept native typography/rendering or physical boat behavior. Full candidate
and runtime/package gates remain open. Any later two-constructor font-face
choice is outside this exact-source receipt and belongs to its own final build.

Compact logs, reports, inventories and verification receipts are committed.
Original ZIPs/objects remain only under the private audit worktree's `.local/`
directory; they are not duplicated in Git. No workflow dispatch, ref change,
product edit, deployment or boat command was performed by this audit.
