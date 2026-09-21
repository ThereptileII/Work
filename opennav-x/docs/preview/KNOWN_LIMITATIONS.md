# Known limitations — Developer Preview 0.1

**Not approved for navigation use.** This is an isolated UI/data-contract preview,
not an installer or a finished navigation product.

- The ZIP targets Windows 10/11 x64 while preserving OpenCPN 5.12.4's supported
  **Win32 application and plugin ABI**. `win64` names the host, not a new x64 ABI.
- Demo propulsion/battery/wind/depth values are synthetic. Live selected GPS,
  SOG/COG and read-only active-route progress retain their accepted integrations.
  Live heading is withheld where measured provenance is insufficient.
- Live propulsion, battery, N2K wind/depth and other sensor adapters are not yet
  integrated. The boat's ESP32 CAN → NMEA 2000 bridge remains the intended hardware
  boundary. No Nissan Leaf CAN IDs are embedded in desktop code.
- Demo route progress is an explicit synthetic timeline using the same snapshot
  contract. It does not change OpenCPN's active route, own-ship position or sensor
  inputs. The real chart canvas is available for pan/zoom independently.
- Included global coastline data is an overview, **not a nautical chart**. No
  commercial chart collection is bundled. Add test charts only to this isolated
  profile using Legacy; production chart configuration is never imported.
- Energy predictions assume constant current speed and whole-pack net discharge,
  linear SOC/energy and the displayed demo capacity/reserve. They do not forecast
  currents, wind, battery temperature, aging or future power changes. Live capacity
  and reserve remain unconfigured; no demo defaults apply to a real vessel.
- No autopilot command output, autonomous steering, radar control/fusion,
  collision avoidance or weather routing has been added.
- AIS workflow redesign, full route editor and settings redesign remain future
  work. Original OpenCPN workflows remain available in Legacy.
- Production detection/install/update/repair/rollback/uninstall, crash recovery,
  broader DPI/touch testing, target navigation-PC validation, physical N2K and
  at-sea validation are not accepted by this preview.
- Native hosted screenshot acceptance is at 1280×800 / 96 DPI / software rendering.
  Higher DPI checks, when recorded, are limited smoke checks. A desktop runner
  cannot establish touch or target-hardware acceptance.
- Run one copy of a given extracted preview profile at a time. Close the preview
  before restarting it from a launcher. Mode switching handles its own restart.
- Use a short writable extraction path, such as `%USERPROFILE%\XNav`. Upstream
  SVG icon cache filenames include the resource path and can exceed Windows
  path limits in deeply nested folders. Native CI observed nonfatal cache-write
  warnings on its long temporary path; broader long-path support remains open.
- The application is unsigned. No OpenNav telemetry uploader is included.
- Optional OpenCPN web content may require the Windows Edge WebView2 runtime;
  the preview screens and demo do not use embedded web content. MSVC runtime and
  required native DLLs are bundled next to the executable.

Use the test guide, report problems, and wait for the next accepted preview.
This milestone does not automatically proceed to a production installer.
