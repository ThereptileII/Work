# OpenNav X — current Beta 2 limitations

Beta 2 is under development. The last qualified downloadable baseline is Beta 1;
see [status](status.md) for exact commits and acceptance evidence.

The SCRUM-212 candidate deliberately makes local OpenCPN peer sharing
unavailable in integrated XNav, Legacy and Safe modes while secure pairing is
unqualified. This means no Send-to-Peer transfers or inbound peer REST service;
use ordinary file export/import for object exchange. It does not change stock
OpenCPN or local navigation storage. The candidate still requires native Windows
and installed/boat acceptance; this statement does not describe a deployment.

- The authorized backed-up stock upgrade to official OpenCPN 5.12.4 x86 is
  complete. The first fixture-free Beta 2 development package is installed.
  Exact replacement `79a95c4` passes all sixteen CI jobs, 110 integrated Linux
  tests, 102 Windows tests, 45 installer checks and both three-hour endurance
  runs. It is installed with the complete real profile preserved. Installed
  mode/maintenance lifecycle and iterative boat review still prevent Beta 2
  release qualification.
- The originally zero-filled INI and its working temporary sibling remain
  backed up. The reviewed working configuration was recovered; a second cold
  backup covers stock 5.12.4 and this recovered profile.
- The first installed read-only session shows real nautical charts and live
  navigation, wind, depth, STW, water temperature and tank inputs. This proves
  acquisition, not physical sensor calibration or navigation approval. Battery,
  motor and rudder inputs were unavailable at observation; dependent estimates
  remain unavailable. Heading was explicitly estimated.
- Normal configuration includes output-capable connections and autopilot
  plugins. The first installed session used a journaled input-only
  commissioning transaction and temporary plugin quarantines. Earlier
  transactions were restored after measured normal exit and source-reviewed
  profile adoption. A fresh transaction for `79a95c4` was applied before its
  September 28 launch. The boat is offline at the latest check, so its current
  process state is unknown and that transaction must be inspected/restored
  after normal close. Never replay the expired launch/restart session. No
  physical actuator command is authorized or included in remote testing.
- The startup caution was acknowledged once. A Windows firewall prompt was
  cancelled and its disappearance visually reviewed; no Allow action was used.
  The floating Dashboard regression is corrected in the replacement; native
  round-trip evidence and the earlier `7827acb` boat screenshots show XNav
  suppression while preserving Legacy layout. The `79a95c4` launch produced
  another Windows network prompt; closing its proxy did not dismiss the
  composited sheet. No network access was granted. Its final dismissal and
  exact replacement boat review are pending.
- Boat Windows reports 1920×1080 at DPI144. The 07:55 UTC read-only
  inventory confirms one active display and Intel graphics at 1920×1080/60 Hz;
  no display or remote-access settings were changed. An exact 1280×800 application frame
  has been captured, but this is not acceptance of a physical 1280×800 display
  or physical touch. CI covers additional scales independently.
- The older Developer Preview folder and six obsolete ZIP downloads are archived
  after ownership/hash checks, with user data preserved. Beta 1 remains available
  until a known-good replacement with Legacy and Safe Mode is verified.

The [distribution limitations](beta2/KNOWN_LIMITATIONS.md) describe hardware,
chart-hazard, radar, physical touch and advisory energy boundaries retained from
Beta 1. Legacy/native plugin dialogs remain the advanced compatibility path.

A fixed-size review-tool defect was found when Windows restored a maximized
XNav frame to an oversized partly offscreen rectangle. The tool refused further
input. The bounded resize repair has eight new disposable native cases; it does
not change application code, display resolution, remote access or navigation
configuration. The boat remains unavailable for the actual recovery at the
latest September 28 check. See [current status](status.md).
