# Helm qualification plan

Qualification is performed against one frozen Staging candidate. A passing
unit test is not a mobile, Windows or boat acceptance result.

## Focused Linux checks

From `opennav-x/tools/helm-server`:

```text
/home/standard/.local/share/mise/installs/go/1.27.1/bin/go test ./...
/home/standard/.local/share/mise/installs/go/1.27.1/bin/go test -race ./...
```

These checks cover pairing expiry, token revocation, disconnect input rejection,
normalized coordinate bounds and media-policy validation.

## Required platform evidence

1. Windows capture and H.264 hardware encoder on the supported OpenCPN/Win32
   installation.
2. Linux capture under the supported X11/Wayland path.
3. Android and iOS native clients on local Wi-Fi.
4. FPS, capture-to-send, decode and input-to-photon measurements at 1280x800;
   the initial target is below 100 ms on suitable Wi-Fi.
5. Packet loss, reconnect, pairing revocation, orientation and coordinate
   mapping.
6. No-Internet operation and no residual input after disconnect.

The endurance suite remains a separate explicitly requested gate. Do not call a
pending measurement a pass.
