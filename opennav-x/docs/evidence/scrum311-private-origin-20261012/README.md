# SCRUM-311 private origin activation — 2026-10-12

The retained public trust selects `https://omarchy.tail5878fc.ts.net`. Live local
Tailscale state confirms that exact hostname, a Running backend, and no existing
Serve or Funnel mappings. No hostname, trust root or key was regenerated.

The existing `tools/staging-origin.py` was started in a retained foreground tool
session, bound only to `127.0.0.1:8765`. Its exact source bytes match `18a6296a`.
A loopback request with the selected Host returned HTTP 200, no redirect, and
the exact retained 1,577-byte public root SHA-256
`a041ec78891a5e3d8896385ed939b691a32ed0ec11e291fb25de773754515d57`.
Only the server's fixed publication allowlist is exposed; no signing key,
operator state, trust config, arbitrary directory or release was published.

Private HTTPS activation was attempted with the selected origin proxy. Tailscale
reported that **Serve is not enabled on this tailnet** and requires the account
owner to enable it through its provided setup link. That user step was requested
asynchronously. Serve configuration remained empty after the attempt; no public
Funnel, firewall, SSH, authentication or remote-access service was changed.

This is **not HTTPS acceptance** or a persistent deployment. The foreground
origin's later liveness must be checked from its actual process/session, not this
receipt. After owner authorization, verify the existing pending setup operation
and actual Serve configuration before making another attempt; do not enable
Funnel. Verify strict HTTPS from the boat, exact public-root bytes and absence of
redirects before publishing any qualified signed offer or accepting an installed
updater test. No package build, service installation, key read or actuator output
occurred. The receipt contains curated observed results, not a retained raw CLI
transcript.
