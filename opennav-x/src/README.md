# Source Layout

Expected modules:

- `ui/` — XNav presentation/components/themes only
- `vessel/` — canonical Vessel Data state, source freshness/precedence, simulation hooks
- `smartnav/` — advisory calculations/predictions only
- `adapters/` — autopilot/radar/future device abstractions
- `integration/` — narrow OpenCPN bridge/hooks
- `platform/linux/` — Linux-specific implementation
- `platform/windows/` — Windows-specific implementation

Do not put navigation algorithms into UI code.
