# OpenNav X Agent Instructions

The authoritative product specification is:

`OpenNavX_Codex_Project_Specification.md`

The approved visual reference is:

`docs/design/OpenNavX_Design_Reference.png`

The project goal is:

`PROJECT_GOAL.md`

## Core rules

- Preserve standard OpenCPN as **Legacy Mode**.
- Implement **XNav**, **Legacy**, and **Safe Mode** against the same underlying OpenCPN navigation/user data.
- The initial mode switch may use a controlled restart. Reliability is more important than live re-parenting.
- Do not rewrite OpenCPN functionality which can be reused safely.
- Keep direct upstream modifications narrow, reviewable, and documented.
- UI, Vessel Data, SmartNav, OpenCPN integration, and hardware adapters must remain separate.
- Never invent unavailable sensor, chart, AIS, device, or navigation data.
- Missing, stale, invalid, estimated, and uncertain data must be represented explicitly.
- No autonomous steering in the initial product.
- Safety-relevant commands require appropriate confirmation/acknowledgement handling.
- Develop primarily on Linux.
- Native Windows MSVC build is mandatory for Windows-facing milestones.
- Windows rendering is authoritative for release UI acceptance.
- Wine or cross-compilation may assist development but does not replace native Windows validation.
- A feature is not complete merely because it compiles.
- Keep the project runnable after every meaningful milestone.
- Work in small vertical slices.
- Inspect actual OpenCPN source before assuming APIs, symbols, class structure, or plugin capabilities.
- Do not silently alter OpenCPN navigation semantics while redesigning UI.
- Do not modify an unsupported or unknown OpenCPN build.
- Preserve charts, routes, tracks, waypoints, plugins, connections, and configuration across XNav/Legacy switching and product updates.

## Before a major change

1. Read the relevant specification section.
2. Inspect the corresponding pinned OpenCPN source.
3. Identify existing functionality that can be reused.
4. Identify whether the work belongs in UI, Vessel Data, SmartNav, an adapter, or the OpenCPN integration bridge.
5. Define how the change will be tested on Linux.
6. Define whether it requires a Windows gate.
7. Implement the smallest maintainable change.
8. Build and test it.
9. Capture evidence where required.
10. Update architecture/upstream-patch documentation if boundaries changed.

## Visual development

Reference resolution: **1280×800**.

Compare implemented XNav screens against:

`docs/design/OpenNavX_Design_Reference.png`

The reference is a visual target, not a literal pixel specification. The written style tokens, safety rules, data-state rules, and component behavior in the project specification take precedence.

Do not approximate XNav using visibly native desktop controls where the design specifies an XNav component.

## Linux / Windows rule

Use Linux for:
- normal Codex work
- source editing
- OpenCPN Linux build
- SmartNav and Vessel Data development
- simulator work
- unit tests
- static analysis

Use native Windows for:
- authoritative MSVC build
- wxWidgets/platform integration
- DLL/plugin loading
- installer/update/repair/rollback/uninstall
- registry/UAC/shortcuts
- fonts and DPI scaling
- authoritative 1280×800 screenshots
- final UI acceptance
- target-hardware testing

## Upstream OpenCPN

Keep the selected upstream revision pinned.

Recommended repository location:

`upstream/OpenCPN/`

Document every direct upstream modification in:

`docs/upstream-patches.md`

Do not add a new OpenCPN version to the supported compatibility manifest until it has passed the required Windows regression gates.
