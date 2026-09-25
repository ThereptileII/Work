# Alpha 1 execution and acceptance plan

The user accepted the Developer Preview direction and authorized integrated
Alpha development, including a version-gated installer. This replaces the
previous instruction to stop before installer development. Alpha is not approved
for navigation or production use. Win32 application/plugin ABI on Windows x64
remains the approved architecture.

## Ordered increments

1. Close the portable chart restart regression with expanded direct-start,
   XNav → Legacy → XNav and Safe → XNav native chart-content checks. Preserve the
   accepted replacement executable evidence; never accept water-only captures.
2. Inspect upstream boundaries and extend owned Vessel Data, source selection,
   calibrated energy configuration and advisory SmartNav contracts with tests.
3. Add independently tested autopilot/radar interfaces, simulator feedback and
   timeouts, hazard-query foundation and anchor observations. Real control stays
   disabled; SmartNav has no control dependency.
4. Integrate modern navigation, route/waypoint, AIS, instruments, energy,
   advisory, adapter, anchor and settings workflows with the upstream models.
   Keep all advanced upstream workflows accessible through Legacy.
5. Add crash recovery, chart/OpenGL/plugin/DPI validation and diagnostic evidence.
6. Build the exact-hash-gated Alpha installer and portable distributions; test
   install, existing profile preservation, repair, update, rollback, uninstall,
   unsupported input and interrupted operations on disposable native Windows.
7. Review the exact packaged revision on both platforms, record limitations and
   physical-hardware gates, deliver downloads and the practical test guide.

Each increment receives coherent commits and applicable automated gates. No
accepted tests are removed to meet a count. Native Windows evidence is mandatory
for UI/platform/installer acceptance. Physical tests remain explicitly open and
do not block independent simulated work. The final report must distinguish an
implemented abstraction from a physically validated integration.

## Current state

Increment 1 passed at `16dbaf72d7923742a5acaadf0fae42ee490e7cf6`, run 36047288188.
The replacement ZIP and expanded chart-startup matrix are verified on both
platforms; all nine relevant native chart captures were reviewed. Increments
2–3 core contracts pass both platform gates at `0e65cd5` and `82efd08`.
The marine decoder/subscription increment passed both platform gates at
`785aa45`. The integrated product screens, settings, adapters, recovery and
installer engine are implemented and undergoing final native qualification.
The current gates retain 90 Linux / 80 Windows integrated cases and 29 portable
suites. Chart editing, actual ENC switching, software fallback and native
100/125/150% DPI have passing candidate evidence. The official stock installer
prerequisite is verified; repeated native recovery and the complete Alpha
installer lifecycle remain required before publication. See [status](status.md)
for exact revisions, failures, replacements and evidence links.
