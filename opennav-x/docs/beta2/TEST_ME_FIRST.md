# Portable recovery — start here

This ZIP is an isolated recovery/development copy of **OpenNav X Beta 2**.
Use `OpenNavX-Beta2-Setup.exe` for integration with your normal OpenCPN profile.

1. Extract the entire recovery ZIP into a **new folder**.
2. Run **Run-XNav.cmd**. The real OpenCPN chart canvas should appear.
3. Check the version under System → Diagnostics.
4. Use **Run-Legacy.cmd** or **Run-Safe.cmd** for recovery; close the current
   application before starting another mode.

The recovery copy starts with its own empty profile. It has no real sensor
connections or active route until you deliberately configure them. Unavailable
instrument/energy values are expected. The built-in world coastline is a chart
background, not a nautical-chart substitute. Configure legally available charts
through Advanced OpenCPN settings if needed; do not copy an entire production
profile into this folder for a quick test.

There is no synthetic vessel-data mode in this product build. Deterministic
simulation exists only in separately compiled automated test builds.

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
