# OpenNav X Beta 2 — installation

OpenNav X uses your existing OpenCPN charts and navigation settings. Beta 2 is
for evaluation; it is not approved for navigation or production use.

## Before installation

- You need the supported **OpenCPN 5.12.4 Windows release**, using its normal
  32-bit application on Windows 10/11 x64. Setup verifies the exact executable;
  another build with the same version label may still be unsupported.
- Back up your OpenCPN profile and any separately stored charts. Keep a copy
  somewhere other than the computer being tested.
- Close OpenCPN, XNav, Legacy and Safe Mode.
- Download the complete Beta 2 artifact from the provided accepted CI run.
  Check `SHA256SUMS.txt` when validating a transferred download.

If Setup says your OpenCPN is unsupported, stop. Do not rename executables,
overwrite OpenCPN manually or bypass the check. Installing/upgrading the OpenCPN
prerequisite is a separate operation requiring its own backup and approval.

## Install

1. Run **OpenNavX-Beta2-Setup.exe**.
2. Confirm the detected OpenCPN location is the installation you normally use.
   Continue only when it shows **Compatible**.
3. Read the recovery explanation and keep the backup location.
4. Choose Legacy and Safe Mode shortcuts if wanted. The OpenNav X shortcut and
   Windows maintenance entry are always created.
5. Review the locations, then install. Wait for validation to finish.
6. Choose **Launch OpenNav X**.

OpenNav installs for your Windows user beside the original OpenCPN application;
it normally needs no administrator prompt. The official OpenCPN installer may
need elevation when separately installing its prerequisite. Do not elevate a
failed or unknown OpenNav package to bypass a compatibility error.

## First start

Check that your real charts and coastlines load. Confirm ownship and instrument
values only when real sources are available. Missing sensors must show unavailable,
not invented numbers. Check source age in Diagnostics.

Open **System → Legacy OpenCPN** if you need the original interface or advanced
settings. **Safe Mode** is the recovery path when normal startup or a plugin
fails. All installed modes use the same existing profile. Keep hardware control
disabled during remote/read-only testing.

## Update

Close all modes, run the newer Setup and use **Update**. Your chart configuration,
connections, routes, tracks, waypoints and settings stay in the existing profile.
The previous application generation remains available for rollback.

## Repair

Close all modes. Open OpenNav X in Windows Installed Apps and choose maintenance,
or rerun the matching Setup and choose **Repair**. Repair replaces damaged
OpenNav-owned files without replacing navigation data.

## Roll back or remove

Open maintenance and choose **Rollback** to return to the previous retained
application generation. Rolling back the first installation removes the
integration. Navigation data is kept at its current state.

Choose **Uninstall** to remove OpenNav's registration, shortcuts and verified
application files. Your original OpenCPN and navigation data remain. Modified
files, user additions and recovery/diagnostic records may be retained deliberately.
Do not delete those without checking their contents.

The historical `OpenNav X Alpha 1` Start-menu folder may remain for upgrade
continuity; the application and Windows Installed Apps show the active version.

## Portable recovery

`OpenNavX-Beta2-Portable-Recovery.zip` is a separate recovery/development tool.
Extract it completely into a new folder and use `Run-XNav.cmd`, `Run-Legacy.cmd`
or `Run-Safe.cmd`. It uses its own profile and does not automatically load your
normal charts or connections. See `TEST_ME_FIRST.md` inside that ZIP.
