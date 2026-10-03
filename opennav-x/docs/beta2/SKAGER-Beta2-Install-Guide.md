# SKAGER Beta 2 — installation

SKAGER uses your existing OpenCPN charts and navigation settings. Beta 2 is
for evaluation; it is not approved for navigation or production use.

## Before installation

- You need the supported **OpenCPN 5.12.4 Windows release**, using its normal
  32-bit application on Windows 10/11 x64. Setup verifies the exact executable;
  another build with the same version label may still be unsupported.
- Back up your OpenCPN profile and any separately stored charts. Keep a copy
  somewhere other than the computer being tested.
- Close OpenCPN, SKAGER, Legacy and Safe Mode, including any older portable copy.
- Download the complete Beta 2 artifact from the provided accepted CI run.
  Check `SHA256SUMS.txt` when validating a transferred download.

If Setup says your OpenCPN is unsupported, stop. Do not rename executables,
overwrite OpenCPN manually or bypass the check. Installing/upgrading the OpenCPN
prerequisite is a separate operation requiring its own backup and approval.

## Install

1. Run **SKAGER-Beta2-Setup.exe**.
2. Confirm the detected OpenCPN location is the installation you normally use.
   Select the original OpenCPN executable, not a SKAGER or portable copy.
   Use **Browse...** if necessary; Setup rechecks the selected file on **Next**.
   Continue only when the compatibility check succeeds.
   The **Installation action** list offers **Install**, **Update** and **Repair**;
   an existing registered SKAGER installation or earlier development version normally selects **Update**.
3. Read the recovery explanation and keep the backup location.
4. Choose Legacy and Safe Mode shortcuts if wanted. The SKAGER shortcut and
   Windows maintenance entry are always created.
5. Review the locations, then install. Wait for validation to finish.
6. On Finish, choose whether to **Launch SKAGER Beta 2**. For remotely managed
   boat commissioning, leave it unchecked until the separate read-only launch
   checks are complete.

SKAGER installs for your Windows user beside the original OpenCPN application;
it normally needs no administrator prompt. The official OpenCPN installer may
need elevation when separately installing its prerequisite. Do not elevate a
failed or unknown SKAGER package to bypass a compatibility error.

## First start

OpenCPN may display its **Welcome to OpenCPN** navigation caution after a version
change. Read it; choose **Agree** to continue or **Cancel** to leave the application
closed. The notice is not an installer error. Do not edit configuration files to
skip it. Unexpected errors or different dialogs should be recorded separately.

Check that your real charts and coastlines load. Confirm ownship and instrument
values only when real sources are available. Missing sensors must show unavailable,
not invented numbers. Check source age in Diagnostics.

Open **System → Open Legacy OpenCPN** if you need the familiar interface or advanced
settings. **Safe Mode** is the recovery path when normal startup or a plugin
fails. All installed modes use the same existing profile. Keep hardware control
disabled during remote/read-only testing.

## Optional Online AIS (prototype-stage builds)

Online AIS uses your own AISStream account key. Open **Traffic → Online AIS
settings → Set AISStream key**, paste your key into the masked field, and choose
**Save key**. Then choose **Enabled** when you want supplemental internet traffic
for the current chart area. It is off by default; saving a key does not turn it on.

The key is stored in Windows Credential Manager for your Windows account, not
in the OpenCPN profile. Updates and repairs preserve it. To delete it, choose
**Remove key** and confirm; Online AIS stops. Uninstall retains the key for
reinstallation, so remove it first if you no longer want it saved.

Internet traffic can be delayed or incomplete. It supplements onboard AIS and
is not a substitute for an AIS receiver or keeping watch. Do not include your
key in screenshots or support reports.

## Update

Close all modes, run the newer Setup and use **Update**. You do not need to delete
the previous installed version first. Your chart configuration,
connections, routes, tracks, waypoints and settings stay in the existing profile.
The previous application generation remains available for rollback.

## Repair

Close all modes. Open **Maintain Skager** from the SKAGER Start-menu folder,
use the application's **Modify** option in Windows program management where
available, or rerun the matching Setup. Choose **Repair**. Maintenance can use
the retained installation package without another download; if it is missing or
damaged, use the verified Setup again. Repair replaces damaged SKAGER-owned files
without replacing navigation data.

## Roll back or remove

Open **Maintain Skager** and choose **Rollback** to return to the previous retained
application generation. Rolling back the first installation removes the
integration. Navigation data is kept at its current state.

Choose **Uninstall** to remove SKAGER's registration, shortcuts and verified
application files. Your original OpenCPN and navigation data remain. Modified
files, user additions and recovery/diagnostic records may be retained deliberately.
Do not delete those without checking their contents.

SKAGER uses the **SKAGER** Start-menu folder with **Skager**, **OpenCPN Legacy**,
**Skager Safe Mode** and **Maintain Skager** shortcuts (depending on your choices).
Updating migrates verified shortcuts from earlier development folders. Rollback restores
that exact retained generation's original folder and labels so its unchanged
maintainer keeps working. The internal installation directory and registration
identity stay compatible with earlier versions; do not rename them manually.
An older independently extracted Demo/portable folder is separate from this
installer. Closing it is required before testing; deleting that folder is not
part of updating the installed application.

## Portable recovery

`SKAGER-Beta2-Portable-Recovery.zip` is a separate recovery/development tool.
Extract it completely into a new folder and use `Run-SKAGER.cmd`, `Run-Legacy.cmd`
or `Run-Safe.cmd`. It uses its own profile and does not automatically load your
normal charts or connections. See `TEST_ME_FIRST.md` inside that ZIP.
