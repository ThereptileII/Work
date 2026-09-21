# Platform boundary

Not implemented. `linux/` and `windows/` will own graceful restart-by-mode,
diagnostic locations and OS integration. OpenCPN remains the authority for
profile/config paths. Restart must wait for OpenCPN to finish saving and release
its process lock; never launch a second chartplotter while the first is saving.

Paths and argument vectors must remain structured (no shell concatenation).
Windows owns registry, elevation, shortcuts and installer execution; no such
code belongs in Vessel Data, SmartNav or UI.
