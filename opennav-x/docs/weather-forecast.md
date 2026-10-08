# Forecast weather (GRIBstream) — boat evaluation increment

Jira: SCRUM-324/325 (provider, token), SCRUM-326/327 (normalized layer, cache,
freshness), SCRUM-328/329 (forecast wind at vessel and on the chart),
SCRUM-330/331 (advisory route forecast). SCRUM-332/333 (SmartNav, Energy and
Sailing consumers) remain deferred. The owner pulled this post-beta theme
forward on 2026-10-07 to judge its usefulness on the boat. It is optional,
XNav-only and **off by default**; navigation works fully offline.

Forecast wind is advisory model output, never a measurement. It is labelled
FORECAST (or STALE FORECAST) with provider, model and run age. It is shown apart
from measured onboard wind and never feeds SmartNav, Energy, pilot or route
state.

## Architecture

- `src/weather/Weather.h` is the provider-neutral contract (`ForecastSnapshot`,
  `ForecastQuery`, `application::WeatherActions`). UI and chart code see only
  these owned values.
- `opennav_weather` (network-free, unit tested): GRIBstream request JSON,
  header-driven CSV parser, U/V → speed and direction FROM (true), strict UTC
  parsing, `ForecastSession` freshness/backoff, `QueryBuilder` bounds.
- `opennav_weather_runtime`: `ix::HttpClient` HTTPS POST to
  `https://gribstream.com/api/v2/<model>/timeseries`, model `gfs`. It uses the
  system CA store with hostname validation, no redirects (the bearer token never
  goes to another host), 10 s connect and 30 s transfer timeouts, a 2 MiB
  response cap and a cancellable worker joined on shutdown.
- `integration::OnlineWeather` stores only the enabled flag and model
  (`/OpenNav/Weather/v1/*`) and owns the token actions. `WeatherOverlay`
  paints the chart layer from the owned snapshot; it never fetches.

## Token and privacy

The user enters their own GRIBstream token on the Weather page (Settings →
Navigation → Weather). Windows stores it in Credential Manager as
`OpenNavX/GRIBstream/v1`; Linux development reads `SKAGER_GRIBSTREAM_TOKEN` only.
The token is never redisplayed and is absent from config, settings backup, cache,
logs, diagnostics and status text. 401/403 stop automatic retries until the
token changes or Test connection is pressed.

## Bounds and freshness

- Query: at most 64 points (fresh vessel fix, up to 16 samples of the active
  route or else the route open on the route page, then a coarse 0.25°–45° chart
  grid of at most 6×6 points) and a valid-time window of now to +48 h.
- Fetch: an unchanged area every 30 min (judged on the model grid), a changed
  area after at least 2 min. Failures back off from 1 to 30 min; 429 honours
  Retry-After (default 15 min).
- Live for 90 min after fetch, then Stale; discarded after 24 h. An outage keeps
  the last forecast as Stale/historical, never as live.
- Chart layer: off by default. Spacing follows `ChartDeclutter` (64/88/120 px),
  with at most 48 arrows and knot values only at full detail.

## Not yet established

Live GRIBstream requests, Windows Credential Manager behaviour, native and GL
rendering, and boat usefulness are unverified. The layer toggle is not persisted.
The model is configurable only in the config file.
