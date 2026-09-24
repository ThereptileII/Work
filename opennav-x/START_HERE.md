# OpenNav X — Start Here

This repository starter pack contains the project requirements and operating instructions for Codex.

## Source of truth

1. **Authoritative technical/product specification**  
   `OpenNavX_Codex_Project_Specification.md`

2. **Approved visual reference**  
   `docs/design/OpenNavX_Design_Reference.png`

3. **Persistent agent rules**  
   `AGENTS.md`

4. **Project goal**  
   `PROJECT_GOAL.md`

5. **First implementation task**  
   `FIRST_TASK.md`

The Word document under `docs/reference/` is for human reading only. The Markdown specification is authoritative for the agent.

## How to begin

1. Put this folder under Git.
2. Start Codex from the repository root.
3. Give Codex the `/goal` content from `PROJECT_GOAL.md`.
4. Tell Codex to read `START_HERE.md`, `AGENTS.md`, and the full project specification before editing code.
5. Codex must first establish and document a pristine OpenCPN baseline.
6. Do not start SmartNav, installer patching, or hardware-specific integration until the first XNav vertical slice is runnable.

## Development environment

Primary development environment: **Linux**  
Final target and release validation: **Windows x64**

Linux success is a development gate, not a release gate. Any Windows-facing milestone must also pass native Windows validation as defined in the specification and `docs/WINDOWS_VALIDATION.md`.

## Important

Do not bundle an arbitrary OpenCPN source snapshot into this starter pack. Codex must select, pin, and record the exact upstream OpenCPN tag/commit it is building against, then place it under `upstream/OpenCPN/` or use a pinned submodule.
