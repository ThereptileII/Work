# OpenNav X `/goal`

Build OpenNav X into a polished, production-quality marine navigation interface on top of OpenCPN.

Treat `OpenNavX_Codex_Project_Specification.md` as the authoritative project specification and follow its architecture, visual design system, safety constraints, development workflow, testing requirements, and Definition of Done.

The final product must:

- Preserve OpenCPN's proven chart, route, waypoint, track, AIS, connection, navigation-data, and plugin functionality.
- Provide a completely redesigned, modern, touch-first XNav interface matching the approved OpenNav X visual style and mockups.
- Preserve the normal OpenCPN interface as Legacy Mode.
- Provide XNav, Legacy, and Safe Mode startup options, all operating on the same underlying OpenCPN data and configuration.
- Keep UI, Vessel Data, SmartNav, hardware adapters, and OpenCPN integration cleanly separated.
- Provide unified vessel data for navigation, wind, depth, rudder, propulsion, batteries, tanks, connectivity, and future sensors.
- Support electric propulsion data, including motor and battery information, and provide tested range and arrival-SOC calculations.
- Introduce SmartNav incrementally for route events, turn prediction, navigation timeline, energy prediction, hazard look-ahead, AIS context, and later sailing/anchor intelligence.
- Integrate autopilot and radar through isolated hardware-adapter interfaces rather than embedding device-specific logic in the UI.
- Never fabricate missing sensor, chart, navigation, or device data. Explicitly represent stale, unavailable, estimated, and uncertain information.
- Keep safety-critical control conservative. The initial product must not autonomously steer the vessel.
- Be easy for a non-developer to install, update, repair, diagnose, roll back, and uninstall on Windows after OpenCPN has been installed.
- Detect and verify supported OpenCPN versions before modifying anything and never patch an unknown or unsupported installation.
- Preserve user charts, routes, tracks, waypoints, plugins, connections, and configuration through updates and rollback.
- Be developed primarily in the existing Linux Codex environment while treating native Windows builds and tests as mandatory release gates.
- Keep the project buildable and runnable throughout development.
- Validate meaningful milestones with Linux tests, native Windows MSVC builds, simulator-driven data tests, interaction smoke tests, and 1280×800 Windows screenshots.
- Treat Windows behavior as authoritative for UI rendering, fonts, DPI scaling, wxWidgets behavior, plugin/DLL loading, installer behavior, XNav/Legacy/Safe switching, and release acceptance.
- Prefer small, maintainable integration hooks into OpenCPN over large invasive rewrites so future OpenCPN versions can be merged realistically.

Work incrementally in vertical slices. Before implementing a major subsystem, inspect the relevant OpenCPN source and existing project code rather than assuming APIs or architecture.

Do not declare a feature complete merely because it compiles. A feature is complete only when its relevant automated, integration, visual, Windows, and regression checks pass.

The finished result should feel like a cohesive commercial marine navigation product built on OpenCPN—not a skin, collection of plugins, or development prototype.

Start by following `FIRST_TASK.md` and the “First Instructions to the Agent” section of `OpenNavX_Codex_Project_Specification.md`. Establish the pristine OpenCPN baseline and complete the first runnable XNav vertical slice before expanding into SmartNav or advanced hardware features.
