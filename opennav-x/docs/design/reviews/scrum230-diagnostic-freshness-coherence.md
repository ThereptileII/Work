# SCRUM-230: coherent diagnostic freshness frame

The retained clean `a3edcae` preview failed its existing stale-data coherence
assertion. Exported samples had age 5038ms against the unchanged 5000ms gate;
route state was `StalePosition` without `remaining_nm`, but energy still had
`arrival_soc: 32.25925926` and `LIMITED / AGING INPUTS`. The screenshot already
showed unavailable Energy values. This evidence establishes an inconsistent
export, not visible stale navigation advice.

Evidence remains under
`ffe-preview-selector/.local/overlay-clean/app/evidence/local/`; the parent
`clean-identity.json` records executable source
`a3edcaef2904bf1450e306f3e086164479ec7f6f`, SHA256
`cb195d7356f5891aa2c4831b64f3bc8f33f0fe9e4fc4670c696200858b404985` and failed
diagnostic SHA256
`64b12a5d8943510485df36e74bae821cc055e03aa9066566608783a11d883194`.

## Cause and correction

`Shell::Tick` captures its evaluation time before computing the energy
prediction and presenting that same frame. `EnergyPrediction::calculated_at`
already retains this time in successful and unavailable outcomes. Later, the
diagnostic callback independently samples live time for publication throttling
and operational observations. The writer previously sampled live time again
for selected sensor and route quality, while serializing the earlier energy
prediction unchanged. Serialization crossing the five-second boundary could
therefore combine stale route/data with a still-admissible earlier estimate.
Replay already assessed its selected state at the prediction timestamp.

The writer now uses `energy.calculated_at` for selected numeric/text values and
route assessment in every mode. It emits that evaluation timestamp separately
from publication's live monotonic timestamp (captured at serialization entry).
Producer observation timestamps remain unchanged. No freshness threshold,
energy formula, route calculation, Shell timing, publication throttle, UI
behavior or acceptance assertion changes.

The scope distinction is explicit: runtime observations retain the diagnostic
callback's independent sampling. Source candidates are collected separately;
their health retains its previous live publication-time assessment, with a
separate timestamp. Replay candidate assessment retains recorded evaluation
time. The export does not claim those collections were captured atomically
with the selected vessel/route/energy frame. The existing `clock` identifies
the evaluation domain; `publication_clock` is always live monotonic.

## Focused verification

The regression invokes the real atomic JSON writer and parses its actual
output, with unchanged prediction/route/sample assessors. It captures a live
publication floor, places retained observations 5038ms earlier, and evaluates
frames at 4999, exactly 5000 and 5038ms. Every serialization is consequently
after the threshold, without sleeps or scheduling assumptions. All three
frames run in selected-live, DEMO and REPLAY modes. Checks cover consistent
selected quality/age, route/arrival availability, preserved producer times,
separate clocks, later source-candidate health and unchanged runtime payload.

- Original writer fails the retained pre-boundary frame: route freshness is
  assessed at serialization time despite an available energy estimate.
- Corrected writer passes **129 coherence checks**.
- Existing energy prediction, energy quality, configured-energy prediction,
  route-progress (28 scenarios), and vessel-quality tests pass.

The focused driver compiles the changed writer and current test source,
reusing local wxJSON and source-unchanged application/smartnav/vessel archives
from the identified `a3edcae` cache. No full application build or preview was
run. Results are retained in this worktree's `evidence/local/coherence/`.
The regression is attached to the existing integrated Google Test target as
`OpenNavDiagnostics.FreshnessFrameCoherence`, so normal native regression
execution includes the real writer. It is not added as a separate CTest count.
Integrated fix compilation and the retained full-preview failure remain open
until their separately authorized verification; no Windows, release or boat
acceptance is claimed here.
