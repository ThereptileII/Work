# OpenNav X — current Beta 2 limitations

Beta 2 is under development. The last qualified downloadable baseline is Beta 1;
see [status](status.md) for exact commits and acceptance evidence.

- The authorized backed-up stock upgrade to official OpenCPN 5.12.4 x86 is
  complete. The first fixture-free Beta 2 development package is installed.
  It is not a qualified Beta 2 release: full replacement Windows gates,
  endurance, installed mode lifecycle and iterative boat review remain open.
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
  commissioning transaction and temporary plugin quarantines. After measured
  normal exit, all temporary changes were restored, preserving the reviewed
  startup migrations. The application is closed; another remote launch needs
  a fresh audit. No physical actuator command is
  authorized or included in remote testing.
- The startup caution was acknowledged once. A Windows firewall prompt was
  cancelled and its disappearance visually reviewed; no Allow action was used.
  The saved floating Dashboard still obscures the chart. The local fix preserves
  that layout for Legacy and suppresses its panes in XNav; native replacement
  and a second boat-screen review are pending.
- Boat Windows reports 1920×1080 at DPI144. An exact 1280×800 application frame
  has been captured, but this is not acceptance of a physical 1280×800 display
  or physical touch. CI covers additional scales independently.
- The older Developer Preview folder and six obsolete ZIP downloads are archived
  after ownership/hash checks, with user data preserved. Beta 1 remains available
  until a known-good replacement with Legacy and Safe Mode is verified.

The [distribution limitations](beta2/KNOWN_LIMITATIONS.md) describe hardware,
chart-hazard, radar, physical touch and advisory energy boundaries retained from
Beta 1. Legacy/native plugin dialogs remain the advanced compatibility path.
