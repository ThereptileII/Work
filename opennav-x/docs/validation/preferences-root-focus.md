# SCRUM-213 — Preferences root focus

The retained native `609` regression run failed after 218 checks: with the owner
active, Sensors was at y190 and passed the exact HWND hit check; activating
Preferences restored focus to **Advanced battery model**, scrolled seven 24px
units and moved Sensors to y22. The pointer no longer hit Sensors. Immediate
and settled records agreed. This extends the earlier retained-scroll repair:
resetting scroll alone does not replace the native frame's remembered child.

The verified failing artifact is `11238170367`, SHA-256
`69625643992f95eb4fd9fa6ca923c96b0bde2b27946f5e8ab58d097a7f4d7f6c`.
Its focused trace is retained in the `scrum213-focus-repair` worktree under
`evidence/local/609-focus-repro/files/windows-changed-units/settings-component/activation/evidence.jsonl`.

`XNavSettingsDrawer::Open` now checks that the drawer was shown, synchronously
focuses the current concrete section tab when shown and enabled, then resets
the body scroll. This establishes a visible return target for later native
activation. It does not select another section, save preferences or rebuild
the form. Ordinary `Present`, updates and dismissal retain their existing
behavior; no deferred child pointer is introduced.

Focused Linux verification passed **179 checks** with the existing 14 captures.
Only the Settings harness and changed SettingsDrawer source were compiled and
linked against the existing component libraries; no full OpenCPN build ran.
Commands, logs and captures are retained in the `scrum213-settings-activation`
worktree under `evidence/local/settings-focus/`. The Windows-only activation
sequence is excluded on Linux, so this pass does not qualify that regression.

Native Windows positive verification of this change remains pending. The
existing focused regression must retain its explicit owner deactivation after
Open, exact hit/rectangle assertions and physical Sensors click. Root opening
may itself activate the drawer; its incidental foreground state is not the
product contract. No pointer guard or visual tolerance is relaxed.
