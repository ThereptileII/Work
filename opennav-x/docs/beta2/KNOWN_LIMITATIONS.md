# Beta 2 limitations and acceptance boundaries

This package is a Beta evaluation build, not a certified or production-approved
navigation system. Its exact commit and CI run are in BUILD_INFO.md. Release
acceptance and actual boat-PC evidence are recorded in the repository status;
building this ZIP alone does not establish acceptance.

- Only the hash-validated OpenCPN 5.12.4 Windows x86 application is supported on
  Windows x64. Other releases/builds are refused. The initially inspected boat
  PC had unsupported 5.12.2; an authorized prerequisite decision is required
  before deployment. Consult the current status for subsequent resolution.
- The installer preserves the historical Alpha ownership/root/Start-menu identity
  to update existing installations safely. This is migration metadata, not a
  second installed application.
- Portable recovery uses its own empty profile. It does not automatically discover
  the normal installation's charts, plugins, routes or data connections.
- No sensor value is supplied artificially. Real input and freshness determine
  availability; navigation or arrival predictions can correctly be unavailable.
- Actual boat battery capacity, current direction, sensor mapping and propulsion
  calibration require comparison with the physical system. Advisory energy
  estimates are not a navigation guarantee.
- Pilot control is disabled by default. The supported adapter still requires
  physical commissioning; no real actuator command is authorized by the remote
  Beta 2 procedure. TRACK/WIND and autonomous steering are unavailable.
- Live chart-corridor hazard queries and the custom Pathfinder radar receive/control
  integration remain incomplete. Lack of a detected hazard never means safe water.
- Native OpenCPN/Legacy/plugin dialogs may retain their desktop styling and bright
  surfaces. XNav primary flows receive separate Night/DPI/touch review.
- Physical touch, actual GPU/OpenGL behavior and boat-source observations must be
  measured on the target machine. CI mouse/injected-touch evidence is additional,
  not a replacement for those checks.
- Setup is unsigned. Obtain it from the supplied exact CI run and verify its hash.
- Repair/uninstall preserve unknown or modified files and recovery diagnostics.
  Inspect retained files rather than deleting a folder based only on its name.

Outstanding target-specific visual or functional findings belong in the final
same-commit review and handoff. They must not be hidden by substituting CI images
for actual boat screenshots.
