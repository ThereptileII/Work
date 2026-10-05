# Portable recovery — start here

This ZIP is an isolated recovery/development copy of **SKAGER Beta 2**.
Use `SKAGER-Beta2-Setup.exe` for integration with your normal OpenCPN profile.

1. Extract the entire recovery ZIP into a **new folder**.
2. Run **Run-SKAGER.cmd**. The real OpenCPN chart canvas should appear.
3. Check the version under System → Diagnostics.
4. Use **Run-Legacy.cmd** or **Run-Safe.cmd** for recovery; close the current
   application before starting another mode.

If OpenCPN presents its normal navigation caution after a version change, read
it and choose Agree to continue or Cancel to close. An unexpected error is a
separate failure; keep its text and the build information.

The recovery copy starts with its own empty profile. It has no real sensor
connections or active route until you deliberately configure them. Unavailable
instrument/energy values are expected. The built-in world coastline is a chart
background, not a nautical-chart substitute. Configure legally available charts
through Advanced OpenCPN settings if needed; do not copy an entire production
profile into this folder for a quick test.

There is no synthetic vessel-data mode in this product build. Deterministic
simulation exists only in separately compiled automated test builds.
Do not look for an old Demo shortcut or enable a sample trip to fill empty
readings. This package deliberately shows the availability of real input.

All recovery modes share this folder's `profile/`. The normal installed OpenCPN
profile is not used. Direct `app/opencpn.exe` startup also follows the recovery
marker. Keep the folder writable and retain all `app/` files together.

Use the supplied test guide, then close normally. Logs are in `logs/` and
`profile/opencpn.log`. Do not share raw logs/profile files without reviewing them
for private coordinates, paths or connection details. The in-app diagnostic export
is the preferred report.

If startup fails, try Safe Mode, retain the error and BUILD_INFO.md, and use the
original installed OpenCPN as your fallback. No administrator rights are needed
merely to run this ZIP. Beta 2 is not approved for navigation.
