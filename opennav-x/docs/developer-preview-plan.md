# Developer Preview 0.1 implementation and acceptance

The preceding read-only route contract is accepted at `954b4505` on both
platforms. This milestone ends with an isolated Windows ZIP, not an installer.

Work sequence:

1. Extend owned Vessel Data fields, deterministic demo scenarios and advisory
   presentation inputs. Test absence, age, source separation and energy failure.
2. Add navigation, destination, propulsion/energy and diagnostics views using the
   approved dark marine design. Keep the real OpenCPN chart canvas.
3. Enforce package-local profile selection, add launchers and build provenance,
   package dependencies/resources and write the human test guide.
4. Run Linux contracts, integrated regressions, navigation/mode and preview
   interaction checks. Publish the exact source revision for native MSVC gates.
5. Smoke-test the extracted ZIP without development dependency paths; review
   native 1280×800 captures and archive evidence before accepting the artifact.

Demo data is explicitly synthetic, separate from OpenCPN-selected live data.
It uses the same owned route snapshot type but identifies its source as DEMO;
it does not activate, edit or advance the user's OpenCPN routes or send NMEA.
The preview must not use simulated telemetry as a fallback for missing live data.
Capacity and reserve in Demo are labelled fixture assumptions, never boat defaults.

All five steps are complete at packaged revision
`b21bd05ce75f22c91ac12927207d264b8b3efde4`.
[The exact CI run](https://github.com/ThereptileII/Work/actions/runs/35663416666)
passed all eight jobs, including native extracted-package testing. The final
1280×800 Windows screenshot review and archive audit are recorded in
[status](status.md) and [validation evidence](preview-validation.md).
The downloadable artifact is `OpenNavX-DeveloperPreview-win64`.

The user's manual test on 2026-09-24 found a missing coastline after Legacy → XNav.
The corrected preview passed both platform gates and native review at
`bcc3fa2bd3ec01b3217a530c3079ae82e0683d0c`; [status](status.md) links the replacement
download and reproduction/acceptance evidence.

Next action: user retest of the corrected mode switch on a normal Windows PC.
Do not advance into the production installer automatically. This acceptance
applies to the isolated Developer Preview, not navigation use.
