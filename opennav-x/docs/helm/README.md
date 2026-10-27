# SKAGER Helm: the SKAGER screen on phones and tablets

Jira Epic [SCRUM-379](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-379)
(features SCRUM-380 to SCRUM-384). SKAGER on the boat PC stays the authority for
charts, navigation, Vessel Data, AIS, routes, alerts and SmartNav. Helm has no
chart engine and no navigation database of its own: it shows the SKAGER window
and sends touches to it.

## Parts

| Part | Where | Role |
|---|---|---|
| Helm Server | `tools/helm-server` (Go, `skager-helm[.exe]`) | Captures the SKAGER window, streams it, takes touches, pairs phones. Started by SKAGER. |
| Host in SKAGER | `src/helm/HelmHost.*`, `src/integration/OpenCPNIntegration.cpp` | Starts and supervises the server over stdin/stdout, passes the autopilot status, refuses autopilot commands that come from Helm. |
| Settings page | `src/ui/HelmPage.cpp` (Settings → System → SKAGER Helm) | On/off, pairing QR code and key, paired phones (remove, disconnect). |
| App | `mobile/helm` (Capacitor 8, Android and iOS) | Pairing (QR or key), the live screen, gestures, keyboard, reconnect. Same design system and logotype as SKAGER Companion. |
| CI | `.github/workflows/skager-helm.yml` | Server tests and Windows/Linux binaries, app unit tests, app against the real server in Chromium, Android APK, iOS simulator build. |

## How it works

1. The owner switches Helm on in SKAGER. SKAGER starts `skager-helm` with
   `-control stdio`, the profile's `helm` folder and the SKAGER window handle.
   The server listens on port 43872 on the boat network. When SKAGER exits,
   the pipe closes and the server stops; on Windows a job object also ends it if
   SKAGER crashes.
2. **Pair a phone or tablet** opens a five-minute, single-use pairing window. SKAGER
   draws the QR code (an `app.skager.helm://pair` link with the LAN addresses
   and an 80-bit secret) and shows the same key as text with the address. The
   phone scans it in the app, or with its camera (the link opens the app), or
   the owner types address and key.
3. The phone connects (signed challenge, then WebRTC) and shows the screen.
   One phone or tablet at a time; another paired phone is told who is
   connected and may take over. The first is told who took over.
4. The phone gets changed 64-pixel tiles as JPEG over the WebRTC `screen` data
   channel (see [protocol.md](protocol.md)). Touches go back on the `input`
   channel as normalized coordinates and are injected into the SKAGER window
   (Windows `SendInput`, X11 `XTEST`).

### Gestures on the phone

| Gesture | On SKAGER |
|---|---|
| Tap | Click |
| Drag | Press, move, release (pans the chart, moves sliders) |
| Touch and hold | Secondary click (context menu) |
| Pinch | Mouse wheel at the pinch centre (chart zoom) |
| Magnifier on + two fingers | Enlarges the phone's view only |
| Keyboard | Typed text and the keys Enter, Backspace, Tab, Escape, arrows |

## Safety boundary

- **No autopilot commands from Helm.** Helm marks every input it injects on
  Windows (`dwExtraInfo` = "SKHL", read with `GetMessageExtraInfo`) and reports
  each touch to SKAGER just before injecting it. SKAGER's autopilot command and
  enable paths refuse a request that came from Helm or within 1.5 s of a Helm
  touch, and log it. Switching control off stays allowed. The phone shows the
  pilot's state (mode, heading) read-only. An old reading is shown as unknown.
- **No stale input.** Input older than 400 ms on arrival is dropped (measured
  against the fastest delivery in the session, so clocks need not agree). A
  press that was dropped also drops its drag. A release of a button the
  session holds is always delivered.
- **Disconnect stops input at once.** When the WebRTC link drops, input is
  refused immediately, held buttons are released and the session ends after 8
  s unless the link recovers. Removing a phone in SKAGER ends its session.
- **Only keys from a short list**, no modifiers or shortcuts; text has no
  control characters.

## Security

See [protocol.md](protocol.md). In short: pairing never sends the secret over
the network; each phone gets its own key (HKDF); every connection is a
challenge-response that also signs both WebRTC descriptions, which carry the
DTLS fingerprints. The screen and touches are therefore encrypted end to end
between the phone and the PC it paired with, without TLS certificates on the boat
network. Device keys are stored in the profile (`helm/helm-devices.json`, owner
only) and, on the phone, in the Keychain or Keystore. Nothing goes over the
internet.

This replaces the initial slice's design of TLS certificates plus a short
pairing code and bearer token. A self-signed certificate cannot be verified by a
phone's WebView, and a 6-digit code can be brute-forced offline by anyone who
recorded the pairing.

## Choices made

- **A tile stream, not H.264.** SKAGER's screen is mostly still. Changed tiles
  keep chart text sharp, cost nothing when nothing moves, and need no platform
  video encoder. The stream lowers quality while the chart pans and sends the
  same tiles sharper once it stops. Flow control (at most two frames not yet
  acknowledged) lowers the frame rate on a slow link instead of adding delay.
  The initial slice's H.264 RTP publisher had no encoder behind it and was
  removed. A hardware encoder can be added behind the same `Display` interface
  if measurements require it.
- **Capacitor app instead of React Native.** The phone's WebView provides WebRTC.
  This reuses SKAGER Companion's build, design tokens, icon and CI.
  `react-native-webrtc` and the separate WebView shell were removed.

## Run it

```sh
# Server with the labelled test pattern (development):
cd opennav-x/tools/helm-server
go run ./cmd/helm-server -source test -state-dir /tmp/helm -pair   # prints the QR code
# App in the browser against it:
cd ../../mobile/helm && npm ci && SKAGER_PREVIEW=1 npm run build
```

Then use `tests/e2e/harness.ts` (it is what CI runs), or see
[mobile/helm/README.md](../../mobile/helm/README.md).

In SKAGER: `-DSKAGER_BUILD_HELM_SERVER=ON` builds `skager-helm[.exe]` from the
same source and installs it next to the SKAGER executable, with its licence
notices in `opennav/third-party/helm/`. The Windows Staging build turns it on
(the workflow installs the pinned Go). For development, `SKAGER_HELM_SERVER` can
point at a server binary instead. Without one, the page says Helm is not part of
the installation.

Windows asks once, when Helm first listens, whether to allow it on the network
(Windows Defender Firewall). Allow it on private networks.

## Open gates

See [qualification.md](qualification.md).
