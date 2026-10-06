# Staging feature check

Use the installed candidate identified in the handoff. These are user checks for
SCRUM-28, SCRUM-29, SCRUM-312 and SCRUM-313; this guide does not record a pass.

Before changing anything, use **Settings → System → Export settings backup**
to save this boat's current `.skager-backup` file. Keep the existing profile;
there is no need to create a fresh profile or reset boat data for these checks.

1. **Startup warning (SCRUM-312).** If OpenCPN shows its navigation warning,
   leave it open briefly, then read it and choose whether to accept. The helm
   must not open before acceptance. During a supervised startup check, the
   decision window is five minutes; acceptance is followed by initialization
   and a separate 30-second readiness check. Cancelling must not let that
   application reach the helm or count as successful startup; record any
   separate recovery or previous-version launch separately. If the warning
   does not appear, record
   **Not shown on this profile**; an existing acceptance can legitimately
   suppress it. Do not reset the profile just to force the warning.

2. **Boat setup (SCRUM-28).** An ordinary update must preserve the existing boat
   profile and must not automatically reopen completed setup. Open **Settings
   → Vessel → Run boat setup**. Temporarily edit the vessel name, then choose
   **Later**. Reopen setup and verify the original name remains: Later discards
   field edits, while the requested setup review remains pending for the next
   start. **Run boat setup** uses current values; it is not a factory reset.

   Review **Your vessel → Display → Sources → Energy → Helm control → System
   summary**. Keep this boat's existing draft, safety depth and battery values;
   do not invent values for an unknown setting. Blank chart safety depth keeps
   its current value. **Check again** refreshes the displayed observations;
   missing or stale sensors must remain identified as such. Make only a small
   display change, check the final summary, then choose **Save & open helm**.
   Restart normally: the display change should persist, completed setup should
   stay closed, and pilot control must remain OFF. On an actual fresh profile,
   setup opens automatically and Later leaves it pending.

3. **Settings backup (SCRUM-29).** Choose **Settings → System → Import settings
   backup** and select the same boat's file saved above. At **Review settings
   restore**, first cancel: the display change from step 2 must remain. Import
   again and review the vessel, display, energy, source and calibration summary.
   Choose **Restore settings**: the original display should return and pilot
   control must be OFF. Restart and verify those restored values persist.

   Restore replaces the settings listed in that summary, including source
   choices, calibration and configured safety assumptions; it is not merely a
   display reset. An unconfigured chart safety depth in the backup preserves
   the current value. Backups exclude charts, licenses, credentials, routes,
   tracks, waypoints, OpenCPN connections, plugins, pilot identity and control
   permission. Restore retains local pilot identity but revokes control
   permission. Check the restored source choices and calibration for this boat.
   If an invalid or incompatible file is selected, expect an error with no
   restoration; cancelling the review must also leave settings unchanged.

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
