# Alpha profile settings and live energy

## 2026-10-18 rail and display extension — source-only

SCRUM-378 adds optional `display.rail_presets` to the existing versioned settings
record: at most eight uniquely named presets, names at most 64 UTF-8 bytes,
exactly four distinct supported quantities each. Old records without this field
remain readable. Choosing one of the four slots replaces/swaps values without
creating duplicate slots. Restore essentials selects SOG, depth, true wind and
battery SOC; absent sensors remain unavailable. Save/apply/delete preset actions
are explicit. Older fifth/sixth rail selections are retained in Instruments when
the person first chooses the four-slot layout.

The shell routes these operations through a dedicated rail-only integration
callback. It reads current settings and changes only the rail/preset fields and
preserved overflow. It does not rerun source selection or disable an enabled
pilot. Replay/simulation and pending restart cannot write real rail preferences;
queued popup actions carry a generation and compare retained settings before
writing. A failed save retains the running configuration.

Settings → Display applies the 100/125/150% interface/chart preference and chart
layout immediately. Save failure restores the previous choices and displays the
error. This percentage is additional to Windows DPI; chart positions, ranges,
passage widths and anchor radii do not change. Four primary rail values retain
space for numbers, units and source/age; secondary decoration yields first if
the viewport is too small. Depth is explicitly below transducer: it is not
compared to a chart-datum safety threshold without a valid datum relationship.

The Personalise popup follows the supplied four-position design and adds bounded
named presets. Source and test-source changes have been reviewed, but **no build
or test was executed**. Native persistence, popup gestures, scaling and layout
remain pending. See [SCRUM-378 review](design/reviews/scrum378-source-refinement.md).

The XNav settings screens now configure the tested live energy wrapper and
instrument source registry. They reuse the current `wxFileConfig` supplied by
OpenCPN at `Attach`; they do not create another navigation database or change
Legacy's chart, route, connection or plugin settings. Direct upstream hooks are
unchanged by this increment.

## Ownership and persistence

`application::Settings` is an owned value: explicit battery/model parameters,
current sign, per-quantity source policies and future corridor assumptions.
Configuration is not sensor telemetry. A configured capacity is displayed as
configured and never injected into a freshly timestamped sensor-capacity field.
Missing numeric parameters remain unconfigured; missing telemetry remains missing.

`SettingsStore` reads/writes one versioned `/OpenNav/AlphaSettings` profile entry
on the application thread. A malformed/unknown version is retained on disk and
fails closed to an unconfigured live model with diagnostic status. It is not
silently repaired or overwritten on startup. Explicit user saving validates the
whole record, writes and flushes it before applying it. A failed flush restores
the prior value and retains the previous running settings; a failed restoration
is reported distinctly. Other profile entries are preserved.

The record is at most 64 KiB, with bounded quoted UTF-8 key/value strings, strict
known keys, required schema fields and duplicate rejection. Numeric input uses a
decimal dot; nonfinite spellings, trailing units and overflow are rejected.
Multiline imported curve data is escaped by wxFileConfig and round-trips without
retaining a dependency on the original external file.

## Live energy

Settings → Energy configuration provides:

- usable battery capacity over the source's reported 0–100% SOC interval;
- reserve SOC and minimum passage speed;
- an explicitly selected observed battery SOC device identity;
- unconfigured / positive-discharge / positive-charge current convention,
  with confirmation that the measurement includes all pack loads;
- measured whole-pack consumption or imported speed/power calibration;
- curve hotel-load and shaft-efficiency assumptions when relevant.

The integration merges the selected marine observations, then normalizes current
and V × I only for the selected pack and coherent epoch. The shell calculates
one advisory energy result per refresh and passes that same result to the energy
page, route page, diagnostics and SmartNav. No UI paint recalculates an estimate
from different assumptions. Demo uses its separate, explicit fixture model.

Source freshness and the established energy freshness ceiling both apply: the
shorter stale threshold wins. Increasing a source threshold cannot extend the
accepted energy age limit. A shorter source policy cannot be ignored by energy.
Missing/stale SOC, consumption, selected navigation or route data suppress the
corresponding dependent estimates. Pure range does not require an active route.
All curve results remain estimates, with no extrapolation or weather forecast.

## Data sources and vessel/radar settings

Settings → Data Sources lists every supported instrument quantity. Per-quantity
views show candidates, device/message identities, values, validity, selected
state, priority, age and freshness policy. Automatic selection uses the tested
registry precedence; explicit pins fail closed without silent fallback. GPS
position/SOG/COG remain owned by OpenCPN's selected-navigation service.

Draft, safety margin and corridor half-width configure only the advisory
corridor contract. They do not change ENC safety contours or advertise an active
hazard service; live corridor coverage remains unavailable. OpenCPN navigation
units, chart presentation, alarms and connections remain accessible through
Advanced / Legacy Settings. XNav canonical display units are explicitly labelled.

Radar status now comes through `IRadar`'s unavailable adapter, including
capabilities, rather than fabricated telemetry. There is no working radar image
adapter yet; Off is the only effective live presentation. No unavailable control
is presented as a successful action.

## Validation

Portable groups cover blank/partial configuration, Unicode and multiline curve
round trips, duplicates/unknown schema, malformed/nonfinite/out-of-domain input,
source pins, current convention, configured live range, shorter stale policy,
conservative energy age ceiling and V/I epoch mismatch. Four integrated tests
use the real wxFileConfig: persistence with unrelated profile entries preserved,
corrupt input failing closed, failed flush rollback and worker-thread refusal.

The preview interaction gate includes energy/source/vessel/radar settings pages.
Native Windows additionally edits the battery assumption sheet and verifies
that those explicit values survive mode restart without replacing Demo or
inventing a battery identity. An integrated Signal K decoder/registry/configuration/energy test proves that
explicit live battery observations can feed range and route arrival estimates.
Native qualification passes at `7bc36e4`;
[the evidence](evidence/alpha-installer-7bc36e4-qualification.json) and
[status](status.md) identify qualification and final release revisions separately.
Physical capacity/current calibration and
recorded boat power curves remain unaccepted boat tests.

Explicit optional propulsion mappings are documented in
[propulsion-source-mapping.md](propulsion-source-mapping.md). Configuration
validation shares the live registry's source-policy validation: source pins are
at most 512 bytes and stale thresholds at most 300 seconds. The earlier settings
validator's one-hour/1024-byte bounds exceeded the reducer's accepted limits;
those inputs now fail before saving instead of causing an exception while
applying persisted settings. Energy and selected-navigation freshness ceilings
remain unchanged.
