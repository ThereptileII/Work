#include "diagnostics/FieldReport.h"
#include "smartnav/VesselEnergy.h"
#include "vessel/DataItems.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
namespace opennav::diagnostics {
namespace {
std::string Number(double n) {
  if (!std::isfinite(n))
    return "unavailable";
  std::ostringstream s;
  s.imbue(std::locale::classic());
  s << std::setprecision(7) << n;
  return s.str();
}
std::string Age(vessel::Time at, vessel::Time now) {
  return at <= now
             ? Number(std::chrono::duration<double>(now - at).count()) + " s"
             : "clock mismatch";
}
std::string Text(std::string s) {
  if (s.size() > 512)
    s.resize(512);
  for (auto &c : s)
    if (static_cast<unsigned char>(c) < 32 || c == 127)
      c = ' ';
  return s;
}
std::string Mode(const vessel::VesselState &s) {
  return s.replayed ? "REPLAY" : s.simulated ? "DEMO" : "LIVE";
}
std::string Digits(const std::string &s, const std::string &key) {
  const auto p = s.find(key);
  if (p == std::string::npos)
    return "unavailable";
  auto begin = p + key.size(), end = begin;
  while (end < s.size() && s[end] >= '0' && s[end] <= '9' && end - begin < 10)
    ++end;
  return end > begin ? s.substr(begin, end - begin) : "unavailable";
}
std::string Protocol(const std::string &s) {
  if (s.rfind("NMEA2000", 0) == 0)
    return "NMEA2000 / PGN " + Digits(s, "PGN-") + " / instance " +
           Digits(s, "instance-");
  if (s.rfind("NMEA0183", 0) == 0)
    return "NMEA0183";
  if (s.rfind("SignalK", 0) == 0 || s.rfind("Signal K", 0) == 0)
    return "Signal K";
  if (s.rfind("DEMO", 0) == 0)
    return "DEMO";
  return "OpenCPN / normalized input";
}
const char *Event(smartnav::EventKind k) {
  using K = smartnav::EventKind;
  switch (k) {
  case K::Waypoint:
    return "waypoint";
  case K::Turn:
    return "turn";
  case K::Destination:
    return "destination";
  case K::ArrivalSoc:
    return "arrival SOC";
  case K::Reserve:
    return "reserve";
  case K::EnergyShortfall:
    return "energy shortfall";
  case K::AisEncounter:
    return "AIS encounter";
  }
  return "unknown";
}
void Sample(std::ostream &o, const vessel::Sample &s, vessel::Time now,
            bool hide = false) {
  const auto a = vessel::Assess(s, now);
  o << (hide      ? "withheld"
        : a.value ? Number(*a.value)
                  : "unavailable")
    << " / " << vessel::ValidityName(s.validity) << " / "
    << vessel::QualityName(a.quality) << " / age " << Age(s.observed_at, now)
    << " / aging " << s.freshness.aging_after.count() << " ms / stale "
    << s.freshness.stale_after.count() << " ms";
}
} // namespace
void FieldJournal::Observe(const FieldSnapshot &s, vessel::Time wall) {
  auto transition = [&](const std::string &key, const std::string &value) {
    auto &old = previous_[key];
    if (old == value)
      return;
    old = value;
    entries_.push_back({wall, key + ": " + value});
    if (entries_.size() > 128)
      entries_.pop_front();
  };
  transition("data mode", Mode(s.vessel));
  for (const auto &item : vessel::DataItems(s.vessel))
    transition(
        item.name,
        std::string(vessel::ValidityName(item.sample->validity)) + " / " +
            vessel::QualityName(vessel::Assess(*item.sample, s.now).quality));
  transition(
      "route",
      s.vessel.navigation.route
          ? vessel::RouteStateName(
                vessel::AssessRoute(*s.vessel.navigation.route, s.now).state)
          : "no route");
  transition("pilot mode", adapters::PilotModeName(s.pilot.feedback.mode));
  transition("pilot output", s.pilot.enabled ? "enabled" : "disabled");
  transition("pilot feedback", s.pilot.fresh ? "fresh" : "unavailable/stale");
  transition("pilot command",
             std::to_string(s.pilot.command.request.id) + " / " +
                 adapters::CommandStateName(s.pilot.command.state));
  transition("recording", s.recording_error ? "error"
                          : s.recording     ? "on"
                                            : "off");
  transition("arrival estimate",
             smartnav::EnergyStatus(s.energy.arrival.reason,s.energy.arrival.input));
  std::string events;
  for (const auto &e : s.advice.events) {
    if (events.size() > 1024)
      break;
    events += std::string(Event(e.kind)) + "/" +
              std::to_string(static_cast<int>(e.severity)) + "; ";
  }
  transition("advisory events", events.empty() ? "none" : events);
  std::string alerts;
  for (const auto &a : s.alerts) {
    if(alerts.size() > 2048) break;
    alerts += Text(a.id) + "/" + application::AlertLevelName(a.level) + (a.acknowledged ? "/ack; " : "/new; ");
  }
  transition("alerts", alerts.empty() ? "none" : alerts);
}
std::string FieldJournal::Export(vessel::Time wall) const {
  std::string text =
      "OpenNav bounded transition journal / newest last / age at export\n";
  for (const auto &e : entries_)
    text += Age(e.at, wall) + " ago / " + e.text + "\n";
  return text;
}
std::vector<BundleEntry>
BuildFieldReport(const FieldSnapshot &s, const FieldEnvironment &env,
                 const FieldJournal &journal, vessel::Time wall,
                 const std::optional<std::string> &recording) {
  std::vector<BundleEntry> out;
  auto add = [&](const char *name, const std::string &text) {
    if (text.size() > 256 * 1024)
      throw std::invalid_argument("Diagnostic report exceeds bound");
    out.push_back({name, text});
  };
  add("READ_ME.txt",
      "OpenNav X field diagnostic bundle v1\nReview before sharing. No "
      "automatic upload.\nDefault report omits positions, route/waypoint "
      "identities, AIS identities, device/interface names, full configuration, "
      "raw bus streams and external log files. Source aliases apply within "
      "this snapshot only.\nLogs are the latest 128 normalized health/control "
      "transitions in this process, not raw NMEA. Crash information is "
      "startup-recovery state, not a memory dump.\nAn optional explicitly "
      "selected recording can contain device names and navigation data.\nNot "
      "navigation approval or physical hardware acceptance.\n");
  std::ostringstream o;
  o.imbue(std::locale::classic());
  o << "Interface: XNav\nData: " << Mode(s.vessel)
    << "\nStartup recovery required: " << env.startup_recovery_required << '\n';
  o << "Startup failures observed " << env.startup_failures_observed << "\nPrevious launch unfinished " << env.previous_launch_unfinished << "\n";
  if (env.build.size() > 16 || env.plugins.size() > 128)
    throw std::invalid_argument("Excess diagnostic metadata");
  for (const auto &line : env.build)
    o << Text(line) << '\n';
  for (const auto &line : env.plugins)
    o << "Plugin: " << Text(line) << '\n';
  add("build-and-recovery.txt", o.str());
  o.str("");
  for (const auto &i : vessel::DataItems(s.vessel)) {
    const bool position = i.sample == &s.vessel.navigation.latitude_deg ||
                          i.sample == &s.vessel.navigation.longitude_deg;
    o << i.name << " / " << i.unit << " / ";
    Sample(o, *i.sample, s.now, position);
    o << " / " << Protocol(i.sample->source) << '\n';
  }
  if (s.sources.size() > 1024)
    throw std::invalid_argument("Excess diagnostic source count");
  o << "\nCandidate source aliases (physical identities withheld)\n";
  unsigned alias = 0;
  for (const auto &h : s.sources) {
    o << "source-" << ++alias << " / " << vessel::Describe(h.quantity).name
      << " / " << Protocol(h.sample.source) << " / ";
    Sample(o, h.sample, s.now);
    o << " / priority " << h.priority << " / selected " << h.selected
      << " / Hz " << (h.frequency_hz ? Number(*h.frequency_hz) : "unavailable")
      << " / received " << h.observations << " / invalid "
      << h.invalid_observations << '\n';
  }
  add("source-health.txt", o.str());
  o.str("");
  const auto &e = s.settings.energy;
  o << "Usable kWh " << Number(e.battery.capacity_kwh) << "\nReserve SOC % "
    << Number(e.battery.reserve_soc_percent) << "\nMinimum speed kn "
    << Number(e.battery.minimum_speed_kn) << "\nHotel kW " << Number(e.hotel_kw)
    << "\nShaft efficiency " << Number(e.shaft_efficiency)
    << "\nConsumption model " << static_cast<int>(e.consumption)
    << "\nCurrent convention " << static_cast<int>(s.settings.current)
    << "\nBattery identity configured " << !e.battery_device_id.empty()
    << "\nCurve points " << e.curve.points.size() << " / reference "
    << static_cast<int>(e.curve.reference) << " / power basis "
    << static_cast<int>(e.curve.basis) << '\n';
  o << "Range: " << smartnav::EnergyStatus(s.energy.range.reason,s.energy.range.input)
    << "\nArrival: " << smartnav::EnergyStatus(s.energy.arrival.reason,s.energy.arrival.input)
    << '\n';
  if (s.energy.arrival.estimate) {
    const auto &a = *s.energy.arrival.estimate;
    o << "Advisory arrival SOC "
      << (a.soc_percent ? Number(*a.soc_percent) : "unavailable")
      << " / shortfall kWh " << Number(a.energy_shortfall_kwh) << '\n';
  }
  add("energy-assumptions.txt", o.str());
  o.str("");
  o << "Pilot mode " << adapters::PilotModeName(s.pilot.feedback.mode)
    << " / fresh " << s.pilot.fresh << " / control enabled " << s.pilot.enabled
    << " / manual capability " << s.pilot.capabilities.manual_control
    << "\nCommand " << s.pilot.command.request.id << " / "
    << adapters::CommandStateName(s.pilot.command.state) << " / age "
    << Age(s.pilot.command.updated_at, s.now) << "\nRadar available "
    << s.radar.available << " / receive capability "
    << s.radar.capabilities.receive << "\n";
  add("adapters.txt", o.str());
  o.str("");
  o << "SmartNav advisory only / identities and free text withheld\n";
  if (s.advice.events.size() > 256)
    throw std::invalid_argument("Excess advisory count");
  for (const auto &a : s.advice.events)
    o << Event(a.kind) << " / severity " << static_cast<int>(a.severity)
      << " / age " << Age(a.observed_at, s.now) << " / approximate seconds "
      << (a.seconds_from_now ? Number(*a.seconds_from_now) : "unavailable")
      << " / NM " << (a.distance_nm ? Number(*a.distance_nm) : "unavailable")
      << '\n';
  if(s.alerts.size() > 32) throw std::invalid_argument("Excess alert report");
  for(const auto &a : s.alerts)
    o << "Alert " << Text(a.id) << " / " << application::AlertLevelName(a.level)
      << " / acknowledged " << a.acknowledged << " / age " << Age(a.first_observed,s.now) << '\n';
  add("smartnav-events.txt", o.str());
  add("recent-transitions.log", journal.Export(wall));
  if (recording) {
    const auto parsed = DecodeRecording(*recording);
    out.push_back({"selected-recording.onxr", EncodeRecording(parsed)});
    add("recording-consent.txt",
        std::string("Explicitly selected recording. Navigation included: ") +
            (parsed.navigation_included ? "YES" : "NO") +
            "\nDevice and source names may be present. Review before "
            "sharing.\n");
  }
  return out;
}
} // namespace opennav::diagnostics
