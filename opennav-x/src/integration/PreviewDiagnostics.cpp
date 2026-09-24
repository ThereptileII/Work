#include "integration/PreviewDiagnostics.h"
#include "OpenNavBuild.h"
#include "application/Version.h"
#include "smartnav/VesselEnergy.h"
#include "vessel/DataItems.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <wx/filefn.h>
#include <wx/jsonwriter.h>
#include <wx/utils.h>

namespace opennav::integration {
std::vector<std::string> PreviewBuildInfo(int dpi, const std::string &profile) {
  return {std::string("OpenNav X ") + application::Edition + " / " +
              application::Version + " / Interface: XNav",
          "OpenCPN 5.12.4 / 37fd0cddb7334fe489e9f18aa163977a9c5c84f7",
          "Build: " OPENNAV_BUILD_COMMIT,
          "Compiler: " OPENNAV_BUILD_COMPILER,
          "Built: " OPENNAV_BUILD_DATE " / CI: " OPENNAV_BUILD_RUN,
          "OS: " + wxGetOsDescription().ToStdString(wxConvUTF8) +
              " / DPI: " + std::to_string(dpi),
          "Profile: " + profile};
}
void WritePreviewDiagnostics(const std::string &path,
                             const vessel::VesselState &state,
                             const std::vector<std::string> &info,
                             const smartnav::EnergyPrediction &e,
                             const application::Settings &settings,
                             const std::vector<vessel::SourceHealth> &sources,
                             const std::string &ui_page,
                             const wxJSONValue &runtime) {
  const auto now = vessel::Clock::now();
  wxJSONValue report;
  report["runtime"] = runtime;
  report["ui_page"] = wxString::FromUTF8(ui_page);
  report["version"] = wxString::FromUTF8(application::Version);
  report["data_mode"] =
      wxString(state.simulated ? "DEMO" : "OPENCPN selected navigation");
  report["build_commit"] = wxString(OPENNAV_BUILD_COMMIT);
  for (const auto &line : info)
    report["build_info"].Append(wxString::FromUTF8(line));
  for (const auto &item : vessel::DataItems(state)) {
    wxJSONValue v;
    const auto a = vessel::Assess(*item.sample, now);
    v["name"] = wxString::FromUTF8(item.name);
    v["unit"] = wxString::FromUTF8(item.unit);
    v["source"] = wxString::FromUTF8(item.sample->source);
    v["device_id"] = wxString::FromUTF8(item.sample->device_id);
    v["aging_after_ms"] =
        static_cast<int>(item.sample->freshness.aging_after.count());
    v["stale_after_ms"] =
        static_cast<int>(item.sample->freshness.stale_after.count());
    v["validity"] =
        wxString::FromUTF8(vessel::ValidityName(item.sample->validity));
    v["quality"] = wxString::FromUTF8(vessel::QualityName(a.quality));
    if (a.value)
      v["value"] = *a.value;
    if (a.age)
      v["age_ms"] =
          static_cast<int>(std::min<long long>(a.age->count(), 2147483647));
    v["observed_monotonic_ms"] = wxString::Format(
        "%lld",
        static_cast<long long>(std::chrono::duration_cast<vessel::Duration>(
                                   item.sample->observed_at.time_since_epoch())
                                   .count()));
    report["data"].Append(v);
  }
  for (const auto &item : vessel::TextDataItems(state)) {
    const auto a = vessel::AssessText(*item.sample, now);
    wxJSONValue v;
    v["name"] = wxString::FromUTF8(item.name);
    v["source"] = wxString::FromUTF8(item.sample->source);
    v["validity"] =
        wxString::FromUTF8(vessel::ValidityName(item.sample->validity));
    v["quality"] = wxString::FromUTF8(vessel::QualityName(a.quality));
    if (a.value)
      v["value"] = wxString::FromUTF8(*item.sample->value);
    if (a.age)
      v["age_ms"] =
          static_cast<int>(std::min<long long>(a.age->count(), 2147483647));
    v["observed_monotonic_ms"] = wxString::Format(
        "%lld",
        static_cast<long long>(std::chrono::duration_cast<vessel::Duration>(
                                   item.sample->observed_at.time_since_epoch())
                                   .count()));
    report["text_data"].Append(v);
  }
  if (state.navigation.route) {
    const auto &r = *state.navigation.route;
    const auto a = vessel::AssessRoute(r, now);
    report["route"]["id"] = wxString::FromUTF8(r.route_id);
    report["route"]["waypoint"] = wxString::FromUTF8(r.active_waypoint_id);
    report["route"]["state"] =
        wxString::FromUTF8(vessel::RouteStateName(a.state));
    report["route"]["source"] = wxString::FromUTF8(r.source);
    report["route"]["revision"] = wxString::Format(
        "%llu", static_cast<unsigned long long>(r.route_revision));
    report["route"]["revision_scope"] = wxString::FromUTF8(r.revision_scope);
    report["route"]["position_source"] = wxString::FromUTF8(r.position_source);
    report["route"]["quality"] =
        wxString::FromUTF8(vessel::QualityName(a.quality));
    report["route"]["unit"] = wxString("nautical miles");
    report["route"]["observed_monotonic_ms"] = wxString::Format(
        "%lld",
        static_cast<long long>(std::chrono::duration_cast<vessel::Duration>(
                                   r.observed_at.time_since_epoch())
                                   .count()));
    if (r.position_observed_at)
      report["route"]["position_monotonic_ms"] = wxString::Format(
          "%lld",
          static_cast<long long>(std::chrono::duration_cast<vessel::Duration>(
                                     r.position_observed_at->time_since_epoch())
                                     .count()));
    if (r.active_waypoint_index)
      report["route"]["active_waypoint_index"] =
          static_cast<int>(*r.active_waypoint_index);
    report["route"]["waypoint_count"] = static_cast<int>(r.waypoint_count);
    if (a.remaining_distance_nm)
      report["route"]["remaining_nm"] = *a.remaining_distance_nm;
  }
  report["settings"]["battery_device"] =
      wxString::FromUTF8(settings.energy.battery_device_id);
  report["settings"]["capacity_kwh"] = wxString::FromUTF8(
      application::SettingNumber(settings.energy.battery.capacity_kwh));
  report["settings"]["reserve_percent"] = wxString::FromUTF8(
      application::SettingNumber(settings.energy.battery.reserve_soc_percent));
  report["settings"]["consumption"] = wxString(
      settings.energy.consumption == smartnav::ConsumptionModel::MeasuredPack
          ? "Measured whole pack"
          : "Calibrated curve");
  report["settings"]["curve_source"] =
      wxString::FromUTF8(settings.energy.curve.source);
  for (const auto &key : settings.data_rail)
    report["settings"]["data_rail"].Append(wxString::FromUTF8(key));
  for (const auto &key : settings.instruments)
    report["settings"]["instruments"].Append(wxString::FromUTF8(key));
  report["settings"]["signal_k_mappings"] = wxJSONValue(wxJSONTYPE_ARRAY);
  for (const auto &m : settings.signal_k_mappings) {
    wxJSONValue item;
    item["path"] = wxString::FromUTF8(m.path);
    item["quantity"] = wxString::FromUTF8(vessel::Describe(m.quantity).key);
    item["scale"] = m.scale;
    item["offset"] = m.offset;
    item["canonical_unit"] =
        wxString::FromUTF8(vessel::Describe(m.quantity).unit);
    report["settings"]["signal_k_mappings"].Append(item);
  }
  for (const auto &source : sources) {
    wxJSONValue v;
    const auto a = vessel::Assess(source.sample, now);
    v["quantity"] = wxString::FromUTF8(vessel::Describe(source.quantity).key);
    v["source_id"] = wxString::FromUTF8(source.source_id);
    v["device_id"] = wxString::FromUTF8(source.sample.device_id);
    v["selected"] = source.selected;
    v["priority"] = static_cast<int>(source.priority);
    v["quality"] = wxString::FromUTF8(vessel::QualityName(a.quality));
    v["validity"] =
        wxString::FromUTF8(vessel::ValidityName(source.sample.validity));
    v["aging_after_ms"] =
        static_cast<int>(source.sample.freshness.aging_after.count());
    v["stale_after_ms"] =
        static_cast<int>(source.sample.freshness.stale_after.count());
    if (a.age)
      v["age_ms"] =
          static_cast<int>(std::min<long long>(a.age->count(), 2147483647));
    if (a.value)
      v["value"] = *a.value;
    auto policy = settings.sources.find(source.quantity);
    v["pinned_source"] = wxString::FromUTF8(
        policy == settings.sources.end() ? "" : policy->second.pinned_source);
    report["source_candidates"].Append(v);
  }
  report["energy"]["arrival_validity"] =
      wxString::FromUTF8(smartnav::EnergyReasonName(e.arrival.reason));
  report["energy"]["model"] = wxString::FromUTF8(e.model_source);
  if (e.range.estimate)
    report["energy"]["range_nm"] = e.range.estimate->range_nm;
  if (e.arrival.estimate) {
    if (e.arrival.estimate->soc_percent)
      report["energy"]["arrival_soc"] = *e.arrival.estimate->soc_percent;
    report["energy"]["shortfall_kwh"] =
        e.arrival.estimate->energy_shortfall_kwh;
  }
  wxJSONWriter writer;
  wxString content;
  writer.Write(report, content);
  const auto pending = path + ".pending";
  {
    std::ofstream out(std::filesystem::u8path(pending), std::ios::binary);
    out << content.ToStdString(wxConvUTF8);
    if (!out)
      return;
  }
  wxRenameFile(wxString::FromUTF8(pending), wxString::FromUTF8(path), true);
}
} // namespace opennav::integration
