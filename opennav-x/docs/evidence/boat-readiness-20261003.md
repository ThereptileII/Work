# Boat baseline readiness — 3 October 2026

Read-only inspection over the configured `ssh boat` alias, from
2026-10-02 23:55:08 UTC through 2026-10-03 00:00:05 UTC, confirms the existing
baseline and recovery material remain available for a later qualified package.
This is SCRUM-17 / SCRUM-215 preparation, not permission to launch or product
acceptance. [Sanitized measured evidence](boat-readiness-20261003.json).

SSH, Tailscale and RustDesk are Running with Automatic startup. There is one
active interactive session; its Explorer owner matches the SSH account. No
OpenCPN/SKAGER/chart-helper process or active commissioning marker was found.
The Intel display reports 1920×1080. The interactive account's stored
`WindowMetrics/AppliedDPI` is 144 (150%); no application was launched to measure
actual window DPI. The required 1280×800 and 1920×1080 native/boat visual and
physical touch acceptance remain open.

The stock executable remains the exact allowed OpenCPN 5.12.4 binary
(`7c654756…`). The 21,492-byte normal INI still matches the completed cold
baseline (`d891d88c…`). The exact completed record (`543f63c…`), capture, review,
predecessor record and saved INI hashes match. All 2,031 recorded live-profile
entries and all 2,031 saved-profile entries matched; the live tree has exactly
2,031 entries. The October 1 recovery set's 2,123 application files and 2,008
profile files also match every recorded size/hash, without reparse entries.
This preserves existing user bytes; the cold record explicitly grants no launch
permission. Full baseline ACL validation and fresh plugin/output review were not
rerun by this inventory.

All four retained installed-generation executables match their ownership hashes.
The current generation remains Beta 1 commit `a3e6e081…`; the previous generation
is `79a95c4f…`. Qualified boat tooling remains at clean tracked revision
`20765cf374da5a1ab47dff0b3b427cd24f595455`. Preserve that tooling and cold-baseline
lineage when a later application package is qualified; do not substitute an older
reader just because it is bundled with a product source revision.

Two historical Start Menu groups still contain eight links, with existing targets:
`OpenNav X` points to the previous generation and `OpenNav X Alpha 1` to the current
generation. The old Beta 1 portable directory still has the accepted ownership
manifest, actual `app/OPENNAV_PORTABLE_PREVIEW` marker, matching executable and
1,257 entries. Beta 1 portable/source/setup archives and the Developer Preview ZIP
still match their accepted exact hashes. The initial bounded locator tested an
incorrect root-level marker path; the final check used the actual layout from
`retire-portable.ps1`, and the required marker is present. No missing-marker
finding remains. Complete link ownership/external-shortcut checks and full package
identity must still precede any retirement.

No application, deployment, profile, remote-access, hardware-output or retirement
operation was performed. The inspections used service/process/CIM/registry reads,
file hashes, directory enumeration and shortcut reads. The full recorded-profile
and recovery hash pass read boat-local bytes without exporting their contents.
Two oversized inspection invocations were rejected by Windows command-line length
before executing; the same read-only script then ran through an in-memory
compressed transport. No script was installed on the boat.

Next action remains qualification of the exact replacement package, followed by
existing guarded update, fresh installed-build/profile/plugin/output review and
input-only commissioning. Actual XNav/Legacy/Safe chart, display and recovery
checks must pass before retiring identified old launch surfaces. Preserve stock,
all navigation/profile data and recovery generations throughout.
