# SKAGER Helm app

The SKAGER screen on your phone or tablet, over the boat's own Wi-Fi (Jira
SCRUM-379, feature SCRUM-382). One TypeScript web codebase packaged for Android
and iOS with Capacitor 8. It shares the design tokens (`../../companion/src/tokens.css`),
the theme logic, the font, the wordmark and the app icon with SKAGER Companion.
Architecture, protocol and safety: [docs/helm](../../docs/helm/README.md).

## Screens

- **Welcome / boats**: how to pair (3 steps), then the paired boats with Connect.
  The last boat connects on start.
- **Pair**: scan the QR code on the SKAGER screen (camera, jsQR), open the
  `app.skager.helm://pair` link from the phone camera, or type address and key.
- **Helm view**: the live SKAGER screen, letterboxed, with a toolbar. The
  toolbar is at the side in landscape and at the top in portrait. It shows:
  - the boat, connection quality and latency;
  - keyboard and magnifier buttons;
  - the read-only autopilot status;
  - disconnect.
  The menu shows latency, frame rate, data rate, Redraw, Gestures and Settings.
- **States**:
  - connecting
  - reconnecting (picture dimmed, touches not sent)
  - can't reach the boat
  - another phone is using it (Take over)
  - taken over by another phone (Take it back)
  - this phone was removed (Pair again)
  - stopped or disconnected on SKAGER
  - SKAGER screen not available (minimized or locked)
- **Settings**: paired boats (Forget), Auto/Day/Dusk/Night, keep the screen on,
  connect on start, about.

## Develop and test

```sh
npm ci
npm run typecheck && npm test                       # unit tests
SKAGER_PREVIEW=1 npm run build                      # browser preview (accepts #pair=<link>)
(cd ../../tools/helm-server && go build -o /tmp/skager-helm ./cmd/helm-server)
SKAGER_HELM_SERVER=/tmp/skager-helm node --test tests/e2e/*.e2e.test.ts   # app + real server in Chromium
HELM_SCREENSHOTS=/tmp/shots SKAGER_HELM_SERVER=/tmp/skager-helm node --test tests/e2e/*.e2e.test.ts
npm run sync                                        # build + copy into android/ and ios/
```

## Download the APK

The **SKAGER Helm (server, Android and iOS)** workflow builds
`skager-helm-<version>-<commit>-debug.apk` with a SHA-256 file, plus the server
binaries (`skager-helm.exe`, `skager-helm`). Debug builds only: store releases
need signing keys and listings.

## Platform notes

- Android: the app talks to the PC by its local address over HTTP
  (`network_security_config.xml`). Everything is authenticated with the device
  key and carried in WebRTC's DTLS (see the protocol). Camera only for the QR
  code.
- iOS: `NSLocalNetworkUsageDescription`, `NSCameraUsageDescription`, ATS
  `NSAllowsLocalNetworking`. WebRTC in WKWebView needs iOS 14.3+.
- `CapacitorHttp` is on, so pairing and connect requests go through the
  native HTTP stack (no WebView mixed-content or CORS limits on the phone).
