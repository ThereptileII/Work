# Boat test: staging startup and saved settings

This case covers the installed `0.4.0-beta2` candidate, commit
`c0d8d85fb602e86d40e2f3f1be32307919702408`. Allow about five minutes.
It is a desktop startup check; normal closing saves your configuration. Leave
hardware controls and connection settings unchanged.

1. Open **System / Diagnostics** in the running SKAGER application. Confirm the
   build begins with `c0d8d85`.
2. Confirm your charts and usual instrument choices are present. Open Instruments
   and AIS, then return to Navigation. Missing sensors should say unavailable,
   rather than display invented values.
3. Close SKAGER normally. Open it again using the **SKAGER** desktop shortcut.
4. Confirm it opens once, responds normally, and keeps your chart view and saved
   choices. Leave it running if everything looks right.

**Pass:** normal startup/restart, retained settings and usable navigation pages.
Report any crash, hang, missing chart, changed setting or duplicate window, along
with the step and approximate time. A screenshot of an error is useful; never
include API keys.

**What this does not test:** a live signed update offer. This candidate has no
configured update feed, so no **Update Now / Later** popup is expected. The
private HTTPS service and guarded boat update transition must be qualified before
the separate update test can run. Do not interpret this startup check as a pass
for downloading, installing or rolling back an offered update.
