# SKAGER Helm mobile client

This is the native Android/iOS client shell for Jira Epic SCRUM-379. It uses
`react-native-webrtc`, stores the revocable session token in the platform
keychain/keystore and keeps the PC as the only chart/navigation authority.

The checked-in source covers pairing, signalling, WebRTC playback and
connection-state handling. `android/` is a small native Android preview shell
which loads the Helm browser client from the LAN server and is built by
`.github/workflows/skager-helm-apk.yml`. It intentionally contains no chart or
navigation implementation of its own. The app is status-only for autopilot and
must not be used to send actuator commands.

## Download the APK

Pushes touching the Android shell start the **SKAGER Helm Android APK** GitHub
Actions workflow. Open the completed run, download the artifact named
`SKAGER-Helm-Android-APK-<commit>`, and install the APK whose SHA-256 is listed
in `SHA256SUMS.txt`. The workflow produces a debug preview APK; no signing key
is stored in the repository.

## Use the Android preview

1. Start the Helm server on the SKAGER PC and put the phone on the same LAN.
2. Open **SKAGER Helm** and enter the PC address, for example
   `http://192.168.0.2:43872` (use the configured HTTPS address for a deployed
   server).
3. Tap **Open Helm**, enter the pairing code in the Helm page, and pair the
   phone. The PC remains the sole chart/navigation authority.

The preview allows LAN HTTP for development, but rejects invalid TLS
certificates and does not grant camera, microphone or actuator permissions.
Native iOS packaging, measured latency qualification and production signing
remain separate acceptance work.
