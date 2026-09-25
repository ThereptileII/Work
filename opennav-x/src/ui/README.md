# UI boundary

XNav frames the existing OpenCPN ChartCanvas through AUI without reparenting or
replacing its renderer. UI consumes owned Vessel Data/route/AIS snapshots and
SmartNav advice. All model changes and human commands pass through integration
or adapter interfaces; no protocol parsing or navigation math belongs here.

Theme tokens, `XNavButton`, `XNavDataValue`, `XNavScroll` and sheets are shared
across primary workflows. Demo and replay remain explicit. Source loss never
turns into a valid zero or renewed age. Operational alerts stay visible above
center pages. See `docs/display-beta-contract.md` for night/touch boundaries.
