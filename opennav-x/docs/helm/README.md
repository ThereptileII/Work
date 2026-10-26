# SKAGER Helm

SKAGER Helm is the local companion-display workstream tracked by Jira Epic
[`SCRUM-379`](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-379).
SKAGER PC remains authoritative for OpenCPN rendering, navigation semantics,
Vessel Data, AIS, routes, alerts and SmartNav.

The first vertical slice is in `tools/helm-server`:

* a Go Helm Server using Pion WebRTC for the media/session boundary;
* short-lived, explicit pairing and revocation;
* bounded normalized touch and keyboard events;
* a LAN WebRTC signalling endpoint;
* an RTP H.264 sink for the platform capture/encoder boundary;
* a browser probe at `/helm/` for deterministic transport testing.

The browser probe is a diagnostic client, not the final Android/iOS product.
It is deliberately included so the transport can be tested before native mobile
packaging is accepted.

## Safety boundary

The initial Helm slice exposes autopilot status only. It never forwards
autopilot, propulsion, radar-transmitter or switching commands. A disconnect
marks the session disconnected and the server accepts no further input from
that session. Remote control requires a separate safety design and qualification
under the Helm Epic.

## Current limitations

The following are deliberately still open and must not be described as passed:

* native Android/iOS builds and store packaging;
* Windows and Linux platform screen-capture adapters;
* TLS certificate provisioning for non-loopback deployment;
* authenticated local discovery/mDNS;
* real SKAGER frame publication into the RTP sink;
* measured input-to-photon latency and FPS on the boat PC;
* chart-provider/license review for screen mirroring.

Do not put pairing codes, bearer tokens or credentials in source, logs, issue
comments or evidence.
