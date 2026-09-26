# OpenNav X — current Beta 2 limitations

Beta 2 is under development. The last qualified downloadable baseline is Beta 1;
see [status](status.md) for exact commits and acceptance evidence. The presence
of a build or screenshot does not establish boat-PC acceptance.

- Boat deployment is blocked: the inspected stock executable is unsupported
  OpenCPN 5.12.2. It has not been patched. The validated prerequisite remains
  the exact official 5.12.4 x86 executable on Windows x64.
- The existing normal INI was already zero-filled. It and its nonzero temporary
  sibling are preserved in a verified cold recovery set; profile recovery has
  not been applied.
- Normal configuration includes output-capable connections and autopilot
  plugins. Real-profile launches require a fresh read-only audit. No physical
  control command is authorized or included in the remote test procedure.
- Boat Windows currently reports 1920×1080; saved Beta 1 diagnostics show 150%
  scaling. The requested 1280×800 physical-display review remains outstanding.
- New primary layouts, reduced rail and contextual workflows require iterative
  native Windows and boat review. Implementation records are not visual signoff.
- The older Developer Preview folder is archived with all user data preserved.
  Beta 1 remains available pending a known-good replacement; remaining old ZIP
  downloads will be retired with inspected ownership.

The [distribution limitations](beta2/KNOWN_LIMITATIONS.md) describe hardware,
chart-hazard, radar, physical touch and advisory energy boundaries retained from
Beta 1. Legacy/native plugin dialogs remain the advanced compatibility path.
