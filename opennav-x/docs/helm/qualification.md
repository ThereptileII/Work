# Helm qualification

A passing unit test is not a phone, Windows or boat acceptance. Record each
result below against an exact commit; never mark a pending item as passed.

## Automated (every push, `.github/workflows/skager-helm.yml`)

| Check | What it proves |
|---|---|
| `go test`, `go test -race`, `go vet` (Linux and Windows) | Pairing (window, attempts, expiry, reuse); signed connect (wrong key, replayed challenge, removed phone); CORS; full WebRTC session: frames, tap/text delivery, late input dropped, disallowed keys refused, held button released on takeover, busy/takeover, revoke ends the session; control channel; tile planner and flow control; Go/TypeScript crypto vectors |
| App type check and unit tests | Protocol parsing, crypto vectors, gesture mapping (tap, drag, hold, pinch, magnifier, cancel releases), screen fit and mapping, escaping, pilot shown read-only |
| App against the real server in Chromium (`tests/e2e`) | Pair by link → live screen with the right pixels; tap, drag, hold, pinch and keyboard reach the PC; pilot status; server restart → automatic reconnect; second phone busy → take over → first phone told, sends nothing; phone removed in SKAGER |
| `helm_tests` (C++) | Server events, supervisor start/restart/give-up/stop, real child process with the built server |
| Windows job | Real GDI capture of the runner's desktop and SendInput cursor placement; `helm_tests` built with MSVC Win32 against `skager-helm.exe` |
| Android `assembleDebug`, iOS simulator build | Native projects build |

## Done locally on Linux (2026-10-26, branch `claude/helm-20261026`)

- All the automated checks above.
- `helm_tests` under ThreadSanitizer: 5 clean runs.
- **Real X11 screen**: an Xvfb display with a Chromium window as the target. The
  server captured it with X11 GetImage, and the app (headless Chromium) showed
  its pixels exactly. A tap arrived as a click at 640,400. Touch and hold arrived
  as a right click. Text typed on the phone appeared in the window's text field
  (XTEST).
- `opennav_ui` with the Helm page builds, and `OpenCPNIntegration.cpp` compiles
  with the full OpenCPN flags (syntax check).

## Pending (owner/boat)

1. **Windows**: native build with `-DSKAGER_BUILD_HELM_SERVER=ON`, or `skager-helm.exe`
   from CI placed next to SKAGER.
   - GDI capture of the SKAGER window at 100% and 200% scaling.
   - SendInput mapping, and the autopilot guard: a tap on the pilot controls from
     the phone must be refused and logged.
   - The firewall prompt or rule for port 43872 on private networks.
2. **Installer packaging**: `skager-helm.exe` and its third-party notices now ship in
   the Windows Staging package (0.4.0-beta2.8). Still open: a firewall rule added by
   the installer (today Windows asks once), and adding Helm to the component licence
   inventory (it is listed as unresolved there).
3. **Phones**: Android and iPhone on the boat Wi-Fi.
   - Pairing by in-app scan, by camera link and by key.
   - Keystore and Keychain.
   - Rotation, keyboard, background and return.
   - Wi-Fi loss and return.
   - iOS local-network permission and WKWebView WebRTC.
4. **Measurements on the boat PC at 1920x1080**:
   - Frame rate while panning.
   - Capture and encode time per frame.
   - Bandwidth: still screen and panning.
   - Tap-to-photon latency (target below 100 ms on good Wi-Fi).
   - Recovery after Wi-Fi loss.
5. **Chart provider and licence review** of screen mirroring (SCRUM-384).
