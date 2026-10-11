# Staging Linux setup smoke refusal — 2026-10-11

- Product/source commit: `ec51b9261d2854d8600672ea27c97af5abeb3f6d`.
- Run: [38143022102](https://github.com/ThereptileII/Work/actions/runs/38143022102),
  [Linux integrated job](https://github.com/ThereptileII/Work/actions/runs/38143022102/job/114480657720),
  step 12, `Exercise shared profile mode cycle and IPC shutdown`.
- Privately downloaded completed-job log: 625,908 bytes, SHA-256
  `1a1f7d303df50e982c3554bdea350a9c23388c2ab192bcaa83f0da0ade97a22f`.
  Do not publish that raw log as product evidence.

The integrated application compiled, then all **152/152 CTests** passed.
Installed-resource and frame-recapture checks also passed before the mode-cycle
step. The mode-cycle helper stopped before clicking **Later** on the initial boat
setup sheet. Its diagnostic snapshot contained the real visible `Vessel name`
and `Draft · metres` fields, plus one enabled `Later`, one disabled `Back`, and
one enabled `Continue`. The `Chart safety depth · metres` field was below the
initial 1280×800 scroll viewport. The helper required all three fields to be
visible and raised `First-start setup is incomplete; cannot safely defer it`.
This is a source-grounded helper refusal; it is **not** evidence of an
application crash or a qualified mode cycle.

The next-source correction `4fbd8f92` requires both visible identifying fields
and the complete action row. It still rejects a lone identifying field,
ambiguous/disabled/clipped actions, a retained offscreen setup field after the
click, or missing fresh closure evidence. `python3
opennav-x/tools/test-smoke-startup.py` passed **13/13** focused tests locally.
That check does not substitute for a new exact-commit integrated Linux or native
Windows qualification. The boat installation is unchanged.
