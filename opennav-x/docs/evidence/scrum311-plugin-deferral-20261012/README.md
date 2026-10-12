# SCRUM-311 plugin deferral and native fixture repair — 2026-10-12

The pinned OpenCPN `LoadPluginCandidate` creates a persistent load stamp before
calling `LoadPlugIn`. A qualification guard only in the latter returned null
without clearing that stamp; ordinary startup would then refuse the plugin as
previously crashed. The new candidate-entry guard runs before logging, plugin
path checks and stamp operations. The low-level guard remains for direct callers.

All 13 reviewed hooks apply in order to upstream
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7` using a private temporary Git index.
`patch-check.json` records exact patch hashes. The source probe compiles the
actual resulting `LoadPluginCandidate` prefix before `wxDateTime` with inert
wx/path/log/stamp seams. Restricted repeated discovery performs no bookkeeping;
ordinary candidate, system-plugin and preexisting-stamp behavior remains.
Removing the new guard makes the negative control fail, as required.
This is bookkeeping behavior evidence, not whole-module ABI or packaged acceptance.
To reproduce, reconstruct the integrated pinned source using the recorded patch
order, put its `model/src/plugin_loader.cpp` beside `probe.py`, then run that probe.

Native focused run [38243098619](https://github.com/ThereptileII/Work/actions/runs/38243098619)
on `175449f68d97d5a71794399e4bd9d66c70781291` failed compiling the inert
`update_qualification_native_test` because Windows/Winsock headers preceded wx
initialization. `native-header-failure.log` retains the diagnostics. Isolated
repair `d3d5a8df96980aba1a072f71528974d3bfa557e6`, integrated as `fae99f9c`,
uses the working native-fixture wx-header convention; the test body is byte-identical.
Actual MSVC compilation and native execution remain pending.

An initial malformed hunk count and local compiler `/tmp` quota failure are
retained; neither is reported as a pass. The corrected source probe uses workspace
temporary storage. No installed profile, service, plugin preferences, boat state,
physical output, full application build, design or endurance run changed here.
