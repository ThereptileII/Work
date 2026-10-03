# SCRUM-259 Linux integrated compile and regression receipt

Final qualified Linux source: `dfa7b721ef6eca3f084f77e95dd8e2adc20bde4b`.
Pinned OpenCPN: `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
The independently reconstructed nine-patch Git tree is
`4efcf110ac10920b0b8cbfbd6726a51c6a3adfac`; actual tracked source matched it.
Patch hashes and executable/resource hashes are in `receipt.json`.

A separate clean build started at `6dafd29db791c773793c2b1155c358cc81452fc2`
in `scrum259-linux-6dafd29`. It reused only read-only SDK/locales and copied
source archives, with private upstream Git metadata and build/install paths.
Bubblewrap mounted every other host path read-only, including the frozen
78eccb8 capture cache. No boat, CI dispatch, UI capture or endurance work ran.

The clean build compiled the real chart/module loader objects and linked the
application, then exposed unresolved `TryPluginPresentationLoader` references
in `ipc-client` and `cli-server`. The shared model object target needed its
actual integration callback link closure. The fix uses an INTERFACE LINK_ONLY
dependency in the integration CMake hook, with no test stubs or pristine
upstream behavior change. A first PUBLIC variant leaked compile usage under
upstream's old CMake policy scope; it was replaced before qualification.
At exact `7d9c35a`, both failed targets linked successfully in 19 bounded steps;
the generated model compile command had no leaked integration include or
fixture macros. The targeted log and executable hashes are retained here.

The disposable worktree then advanced to final `dfa7b72` (identical application
source, patch and CMake bytes to the tested 7d9 fix), completed 223 incremental
build steps and installed successfully. The generated build identity embeds
that exact final commit. This is the existing developer integration recipe:
route scenarios, test fixtures and pilot loopback fixtures enabled; private
adapter package absent. No synthetic production input or hardware output ran.

All five original focused drawing fixtures passed:

| Fixture | Checks |
|---|---:|
| Geographic names | 5,776 |
| Light labels | 48 |
| Wordmark | 129,262 |
| Route labels | 93 |
| Onboard AIS body | 24,751 |

The first ctest attempt exposed test-environment restrictions: the read-only
normal home prevented the IPC fixture's `~/.opencpn` socket use, and D-Bus
reported its transient runtime directory read-only. Its output is retained in
`fixtures-and-first-ctest.log.gz`. The test environment was corrected to a fresh
private HOME (including `.opencpn`), XDG configuration/data and mode-0700 runtime
directory, still inside the only writable worktree. No test assertion or source
changed. The existing `ctest --no-tests=error -E '^tests$' --timeout 90` suite
then passed **147/147 in 22.71 seconds** under `dbus-run-session`; its JUnit and
full log are included. The exclusion is the unchanged original script's
aggregate duplicate test, not a newly introduced exclusion.

Build executable SHA-256:
`4ad98de214ce7d7fe617832a308d7c05bb26fe15ce08c1987e1a8b29ef3a8f23`.
Installed executable SHA-256:
`e4825535e767c011d336701ae72abc7a817ad7a2c8100dbe462b94419d738ac5`.
Both are 26,053,888 bytes. Their difference is the recorded CMake install
removal of the build SDK RUNPATH. Resource manifest SHA-256:
`03533971dfba1bb3cb2cba6ab52b1528a56b4f27c8b4fc118b4b503c25b4bdde`.

This qualifies Linux compilation/linking and these regression checks only.
The Windows-only alpha helper is excluded by its intended platform guard;
this validates its Linux consumer path, not GDI+. Native MSVC, actual private
adapter DLL, Windows resource/installer, chart visual and boat acceptance
remain separate gates. Original failure logs are retained, not overwritten by
passing receipts.
