#include "application/SourceHealthView.h"
#include "vessel/AisHealth.h"
#include <cmath>

namespace opennav::application {
namespace {
std::string Bounded(std::string text) {
  if (text.size() > 512) text.resize(512);
  for (auto &c : text) if (static_cast<unsigned char>(c) < 32) c = ' ';
  return text;
}
void State(HealthSignal &s, SignalState state) {
  s.state = state;
  switch (state) {
  case SignalState::Current: s.status = "Current"; break;
  case SignalState::Aging: s.status = "Aging"; break;
  case SignalState::Stale: s.status = "Stale"; break;
  case SignalState::Estimated: s.status = "Estimated"; break;
  case SignalState::Uncertain: s.status = "Uncertain"; break;
  case SignalState::Invalid: s.status = "Invalid"; break;
  case SignalState::Unavailable: s.status = "No data"; break;
  }
}
} // namespace
HealthSignal PresentSignalHealth(const vessel::Sample &s, vessel::Time now) {
  HealthSignal v;
  v.source = Bounded(s.source);
  State(v, SignalState::Unavailable);
  if (!s.value) return v;
  if (!std::isfinite(*s.value) || s.validity == vessel::Validity::Invalid ||
      s.freshness.aging_after < vessel::Duration::zero() ||
      s.freshness.stale_after <= s.freshness.aging_after) {
    State(v, SignalState::Invalid); return v;
  }
  const auto a = vessel::Assess(s, now);
  v.age = a.age;
  switch (a.quality) {
  case vessel::Quality::Live: State(v, SignalState::Current); break;
  case vessel::Quality::Aging: State(v, SignalState::Aging); break;
  case vessel::Quality::Stale: State(v, SignalState::Stale); break;
  case vessel::Quality::Estimated: State(v, SignalState::Estimated); break;
  case vessel::Quality::Uncertain: State(v, SignalState::Uncertain); break;
  case vessel::Quality::Unavailable: break;
  }
  return v;
}
HealthSignal PresentPositionHealth(const vessel::Navigation &nav, vessel::Time now) {
  const auto &lat = nav.latitude_deg, &lon = nav.longitude_deg;
  auto v = PresentSignalHealth(lat, now);
  const auto other = PresentSignalHealth(lon, now);
  v.id = "gps"; v.title = "GPS"; v.measurement = "Selected position";
  if (!lat.value || !lon.value) {
    State(v, SignalState::Unavailable); v.age.reset();
  } else if (!std::isfinite(*lat.value) || !std::isfinite(*lon.value) ||
      std::abs(*lat.value) > 90 || std::abs(*lon.value) > 180 ||
      lat.source != lon.source || lat.device_id != lon.device_id ||
      lat.observed_at != lon.observed_at ||
      v.state == SignalState::Invalid || other.state == SignalState::Invalid) {
    State(v, SignalState::Invalid); v.age.reset();
  } else {
    // Pair status is never healthier than either component. An estimate is
    // explicit and cannot be mistaken for selected measured GPS.
    for (const auto state : {SignalState::Unavailable, SignalState::Uncertain,
                            SignalState::Stale, SignalState::Estimated,
                            SignalState::Aging})
      if (v.state == state || other.state == state) { State(v, state); break; }
  }
  v.note = "Position selection and precedence belong to OpenCPN.";
  return v;
}
SourceHealthView PresentSourceHealth(const vessel::VesselState &vessel,
    const std::vector<vessel::SourceHealth> &sources,
    const vessel::AisState &onboard, const OnlineAisState &online,
    const adapters::PilotView &pilot, vessel::Time now) {
  SourceHealthView view;
  view.historical = vessel.replayed || vessel.simulated;
  view.signals.push_back(PresentPositionHealth(vessel.navigation, now));
  const auto add = [&](const char *id, const char *title, vessel::Quantity q) {
    const auto &sample = vessel::Field(vessel, q);
    auto signal = PresentSignalHealth(sample, now);
    const auto &bounds = vessel::Describe(q);
    if (sample.value && (*sample.value < bounds.minimum || *sample.value > bounds.maximum))
      State(signal, SignalState::Invalid);
    signal.id = id; signal.title = title; signal.quantity = q;
    signal.measurement = vessel::Describe(q).name;
    signal.note = "Quality applies to this measurement, not every signal on the connection.";
    const vessel::SourceHealth *selected = nullptr;
    unsigned matches = 0;
    for (const auto &source : sources)
      if (source.quantity == q && source.selected &&
          source.sample.source == sample.source &&
          source.sample.device_id == sample.device_id &&
          source.sample.observed_at == sample.observed_at) {
        selected = &source; ++matches;
      }
    if (matches == 1 && !sample.source.empty()) {
      signal.priority = selected->priority;
      if (selected->frequency_hz && std::isfinite(*selected->frequency_hz) &&
          *selected->frequency_hz > 0)
        signal.frequency_hz = selected->frequency_hz;
    }
    view.signals.push_back(std::move(signal));
  };
  add("heading", "Heading", vessel::Quantity::Heading);
  add("depth", "Depth", vessel::Quantity::Depth);
  add("wind", "Wind", vessel::Quantity::ApparentWindSpeed);
  add("motor", "Motor", vessel::Quantity::MotorRpm);
  add("battery", "Battery", vessel::Quantity::BatterySoc);
  add("rudder", "Rudder", vessel::Quantity::Rudder);
  HealthSignal ais;
  ais.id = "ais"; ais.title = "Onboard AIS";
  ais.source = Bounded(onboard.source); ais.measurement = "Received target reports";
  ais.note = "Target reports do not prove receiver or transport connectivity.";
  vessel::AisState local;
  local.available = onboard.available;
  local.source = onboard.source;
  local.simulated = onboard.simulated;
  if (onboard.targets.size() <= 2000) {
    for (const auto &t : onboard.targets)
      if (t.origin == vessel::AisOrigin::LocalOpenCPN) local.targets.push_back(t);
  } else local.available = false;
  const auto health = vessel::AssessAisReports(local, now);
  State(ais, health == vessel::AisReportHealth::Current ? SignalState::Current
           : health == vessel::AisReportHealth::Stale ? SignalState::Stale
           : SignalState::Unavailable);
  ais.status = vessel::AisReportHealthName(health);
  view.signals.push_back(std::move(ais));
  HealthSignal ap;
  ap.id = "pilot"; ap.title = "Autopilot adapter";
  ap.source = Bounded(pilot.feedback.source); ap.measurement = "Measured pilot feedback";
  State(ap, SignalState::Unavailable);
  if (!pilot.feedback.source.empty() && pilot.feedback.sequence &&
      pilot.feedback.observed_at <= now &&
      pilot.feedback.mode != adapters::PilotMode::Unavailable) {
    ap.age = std::chrono::duration_cast<vessel::Duration>(now - pilot.feedback.observed_at);
    State(ap, *ap.age >= std::chrono::seconds(3) ? SignalState::Stale
              : pilot.fresh ? SignalState::Current : SignalState::Uncertain);
  }
  ap.note = "Status only. Transmitting a command is not proof of pilot state.";
  view.signals.push_back(std::move(ap));
  add("water", "Water tank", vessel::Quantity::FreshWater);
  HealthSignal internet;
  internet.id = "online"; internet.title = "Online AIS";
  internet.source = "AISStream.io / Internet"; internet.measurement = "Supplemental service";
  State(internet, SignalState::Unavailable);
  internet.note = "Internet traffic never establishes onboard AIS health or navigation safety.";
  internet.status = "Off";
  if (online.enabled && !view.historical) {
    switch (online.feed.health.connection) {
    case ais::Connection::CredentialMissing: internet.status = "Key needed"; break;
    case ais::Connection::Connecting: internet.status = "Connecting"; break;
    case ais::Connection::Subscribing: internet.status = "Subscribing"; break;
    case ais::Connection::Connected:
      if (online.feed.health.subscription_confirmed) {
        State(internet, SignalState::Current); internet.status = "Connected";
      } else internet.status = "Subscribing";
      break;
    case ais::Connection::Backoff: internet.status = "Reconnecting"; break;
    case ais::Connection::Offline: internet.status = "Offline"; break;
    case ais::Connection::Disabled: break;
    }
  }
  view.signals.push_back(std::move(internet));
  return view;
}
} // namespace opennav::application
