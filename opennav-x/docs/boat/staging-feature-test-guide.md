# Staging feature check

Use the installed candidate identified in the handoff. These are user checks for
SCRUM-28, SCRUM-29, SCRUM-312 and SCRUM-313; this guide does not record a pass.

1. **Startup warning.** If OpenCPN shows its navigation warning, read it and
   choose whether to accept. Startup must wait for your choice; cancelling must
   not open the helm or count as successful startup. An existing profile may
   already have accepted the warning, so it need not appear on every launch.

2. **Boat setup.** A fresh profile opens **Boat Setup & Sensor Check**. An
   existing profile is preserved: open **Settings → Vessel → Run boat setup**.
   Try **Later** after editing a field: nothing should be saved, and unfinished
   setup should return on the next start. Reopen it and review **Your vessel**,
   **Display**, **Sources**, **Energy**, **Helm control** and **System summary**.
   Use **Check again** for a new sensor reading. Leave unknown assumptions blank;
   blank chart safety depth keeps its existing value. Only **Save & open helm**
   saves the changes. Restart and check the saved values. **Run boat setup**
   starts another review with current values; it does not erase the profile or
   restore factory defaults. Setup must never enable pilot control.

3. **Settings backup.** Under **Settings → System**, choose **Export settings
   backup** and save the `.skager-backup` file. Make one harmless display change,
   then choose **Import settings backup**. At **Review settings restore**, first
   cancel and check that your change remains. Import again, check the vessel,
   display, source and calibration summary, then choose **Restore settings**.
   Verify the saved display returns and pilot control is OFF. Invalid or
   incompatible files must report failure without restoring anything.

   Backups exclude charts, licenses, credentials, routes, tracks, waypoints,
   OpenCPN connections, plugins, pilot identity and control permission. Restore
   retains the local pilot identity but revokes control permission. Check source
   choices and calibration for this boat before relying on them.

4. **Pilot: your checks after the operator test.** Open **Autopilot** and compare
   mode and available heading readings with the physical pilot. Missing or stale
   feedback must remain visibly unavailable, never appear as a measured result.
   After restart, expect **CONTROL OFF** and saved permission returned to
   display-only. The reviewed translator binding may remain for status use.

   The separately authorized operator test covers six manual command types:
   **Standby**, **Auto**, **−1°**, **+1°**, **−10°**, **+10°**. It requires the actual
   observed translator, **Permit manual commissioning... → Save manual
   permission**, then separately **Enable control → Enable manual control**.
   **Auto** also requires **Request AUTO** confirmation. Each response must be
   checked against fresh physical feedback and measured heading; a button click
   or transmitted command is not confirmation. The operator finishes in physical
   STANDBY, disables the session and selects **Return to display-only**. This
   guide does not ask you to repeat those physical commands. TRACK/WIND and
   autonomous steering are outside this test.

These checks do not establish that SCRUM-311's signed update handoff has passed.
Report the screen, action and observed result for any mismatch.
