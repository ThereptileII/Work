# OpenCPN prerequisite for SKAGER

SKAGER is a Windows navigation interface that works with your existing OpenCPN
installation. The SKAGER public beta supports a **Windows x64 host with the
Win32/x86 OpenCPN 5.12.4 application and plugin interface**. OpenCPN is a
separate prerequisite; SKAGER does not replace it.

Install OpenCPN only from the [official OpenCPN download page](https://opencpn.org/OpenCPN/info/downloadopencpn.html).
The supported 5.12.4 installer and its source are also available from the
[official OpenCPN 5.12.4 release](https://github.com/OpenCPN/OpenCPN/releases/tag/Release_5.12.4).
The [OpenCPN source repository](https://github.com/OpenCPN/OpenCPN) provides the
project source and license information.

After installing OpenCPN, open its About or version information screen and
confirm that it is the supported 5.12.4 release. A version number by itself is
not enough: a modified, portable, or otherwise different build can have the
same version label and still be unsupported. SKAGER Setup performs its own
compatibility check before changing anything.

If Setup reports that OpenCPN is unsupported, stop there. Do not rename the
OpenCPN executable, replace files by hand, or bypass the check. Setup refuses
to continue and leaves the OpenCPN installation, charts, profile, and
navigation data untouched. Use the official prerequisite installer or contact
support with the displayed version information.

Before installing SKAGER:

1. Close OpenCPN and any SKAGER, Legacy, or Safe Mode window, including an
   older portable copy.
2. Back up your OpenCPN profile and any charts stored separately. Keep that
   backup independent of the SKAGER installation.
3. Select the original installed OpenCPN application when Setup asks which
   installation to use.
4. Read the recovery details and keep the backup location shown by Setup.

SKAGER installs alongside the original OpenCPN application. Your charts,
routes, tracks, waypoints, connections, and settings remain in the OpenCPN
profile. **Legacy** opens the familiar OpenCPN interface, and **Safe Mode** is
the normal recovery route if the SKAGER interface or a plugin does not start.
Updates, repairs, rollback, and uninstall operate on SKAGER-owned files; the
original OpenCPN application and navigation data remain available.

The SKAGER public beta is still being qualified. No public download is
available from this page yet. When release qualification is complete, the
release page will provide the supported installer, recovery information, and
version-specific support links. The [technical compatibility manifest](../../installer/windows/compatibility.json)
is maintained for the installer and is not a substitute for that public
release notice.
