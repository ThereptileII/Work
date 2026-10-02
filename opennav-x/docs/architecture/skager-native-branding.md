# SKAGER native identity (SCRUM-235)

The customer-facing application name and modern interface label are SKAGER.
`application/Brand.h` centralizes the repeated product, mode, window-title and
return-to-modern labels. `Shell.cpp` draws the approved SCRUM-89 artwork from
`application/SkagerBrandAsset.h` in the existing 180×68 DIP header allocation;
it preserves the asset proportions and caches the scaled bitmap per device
size. The source artwork and asset provenance belong to the separate asset/installer task.

## Bounded shipped-surface inventory

| Surface | Implementation and result |
| --- | --- |
| Shell header | `src/ui/Shell.cpp`: approved SKAGER App artwork and public accessible label; navigation action preserved. |
| Main, Legacy and Safe windows | `src/integration/OpenCPNIntegration.cpp`: SKAGER, SKAGER Legacy and SKAGER Safe Mode, all with OpenCPN attribution. |
| Legacy menu and CLI help | `patches/opencpn-5.12.4-xnav.patch`: SKAGER menu and interface help; menu lookup follows the caption. `--xnav`, `--legacy`, `--safe-mode` remain unchanged. |
| Settings System, Help and recovery | `src/ui/SettingsDrawer.cpp`, `ProductPanel.cpp`, `ProductSettings.cpp`: SKAGER headings, version line, restart and chart-style labels. |
| Drawer, context and overlay windows | `src/ui/Drawer.cpp`, `FloatingSurface.h`, `ContextCard.cpp`, `Branding.h`: SKAGER native window titles while existing automation selectors stay stable. |
| Diagnostics and About route | `src/integration/PreviewDiagnostics.cpp`, `src/ui/PreviewPanel.cpp`: SKAGER build/mode identity. Existing OpenCPN attribution remains. The About & licenses viewer is still explicitly unavailable; this copy change does not implement it. |
| Alerts and equipment status | `src/application/Alerts.cpp`, `PilotPresentation.cpp`, `src/ui/AlertDrawer.cpp`, `PilotSettings.cpp`, `src/integration/OpenCPNPilot.cpp`: SKAGER copy; unchanged status-only and acknowledgement limits. |
| Chart presentation status | `src/integration/ChartPresentation.cpp`, `src/ui/ChartPresentationDrawer.cpp`: SKAGER visible status and preference hint; saved `XNav` style value unchanged. |
| Recording and exported diagnostics | `src/ui/CommissioningPanel.cpp`, `FieldReportPanel.cpp`, `src/diagnostics/Recorder.cpp`, `FieldReport.cpp`: SKAGER file-filter captions, report heading, suggested ZIP filename and mode label. Existing recording format/extension unchanged. |
| Recovery errors and operational logs | `src/integration/RecoveryStore.cpp`, `SettingsStore.cpp`, `OpenCPNIntegration.cpp`, `OnlineAis.cpp`, `NavigationActions.cpp`, `src/platform/PortableProfile.cpp`: SKAGER prose; existing storage and transport identities preserved. |
| Windows saved credential description | `src/platform/windows/AisCredentials.cpp`: SKAGER descriptive username; target key `OpenNavX/AISStream/v1` preserved. |
| New temporary anchor marks | `src/integration/NavigationObjects.cpp`: SKAGER description visible through OpenCPN. Cleanup still recognizes both exact historical OpenNav descriptions and preserves repurposed/shared marks. Existing saved user data is not rewritten. |

## Intentional compatibility identities

Internal `opennav` namespaces, `XNav` classes, CLI switches, configuration paths
and values, diagnostic JSON keys/enums, recording/calibration/settings wire
headers, restart pipe/protocol, executable/resource/log filenames and fixture
markers retain their established identities. `SetName`/AUI pane selectors stay
stable; corresponding visible titles and labels use SKAGER. The immutable
prototype and historical evidence are unchanged. OpenCPN's own About and
attribution retain its name. Source-protocol names and customer-owned stored
text are not blindly rewritten.

Current-candidate native capture/smoke harnesses, Windows UI geometry helpers,
boat window/restart selectors and their synthetic fixtures follow the new
window/menu/caption/log strings. Historical installer fixtures still require
their historical names. Package/installer smoke consumers are coordinated with
the separate asset work.

## Focused development verification

- Linux production-policy UI library and integrated search component compiled.
- Nine changed integration translation units compiled independently against
  the existing pinned OpenCPN build headers into private outputs, with test
  input and loopback control disabled. This is compile coverage, not a newly
  linked/release-qualified OpenCPN application.
- Ten tests passed: three field-report privacy/bounds/recording cases, startup
  recovery, startup mode, settings roundtrip/invalid/display, portable profile
  and the existing parent-exit/restart-argument lifecycle test.
- Windows UI geometry/ownership/pointer guard contract checks passed with the
  current captions, as did 24 diagnostic geometry, nine preference-pan and five
  preference-selector tests. All changed Python files parse. These checks use
  controlled observations; they do not replace native Windows execution.
- The existing search/owned-drawer component passed 86 checks, including opening
  settings through its retained selector and observing its SKAGER title.
- Linux shell captures at [1280×800](../evidence/scrum-235/linux-shell-1280.png)
  and [853×600](../evidence/scrum-235/linux-shell-853.png) were inspected: the
  approved wordmark fits its header allocation and adjacent controls remain
  separate. This component intentionally has no chart or vessel input.
- An initial component attempt inherited the desktop Wayland backend and
  failed its drawer-position check. The repeat explicitly selected the isolated
  X11 display and passed; the initial result is not Windows/UI qualification.

Native Windows build, screenshots at 1280×800 and 1920×1080, DPI/theme review,
Legacy/Safe/recovery flows and actual boat-display review remain open for the
exact integrated revision. The expanded real-OpenCPN anchor ownership scenario compiled but
must execute successfully in that qualification; local string inspection alone does not prove
navigation-data preservation or shipped-surface completeness.
