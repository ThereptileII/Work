# SKAGER Helm mobile client

This is the native Android/iOS client shell for Jira Epic SCRUM-379. It uses
`react-native-webrtc`, stores the revocable session token in the platform
keychain/keystore and keeps the PC as the only chart/navigation authority.

The current checked-in slice covers pairing, signalling, WebRTC playback and
connection-state handling. Native Android/iOS build files, platform capture
integration, orientation qualification and final touch-channel wiring remain
open acceptance work. The app is status-only for autopilot and must not be
used to send actuator commands.
