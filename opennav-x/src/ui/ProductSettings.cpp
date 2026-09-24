#include "ui/ProductPanel.h"
#include "ui/Sheet.h"
#include "vessel/DisplayItems.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <wx/filedlg.h>
namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString N(double n) {
  return std::isfinite(n) ? wxString::Format("%.6g", n) : wxString{};
}
wxString Configured(double n, const wxString &unit) {
  return std::isfinite(n) ? N(n) + " " + unit : "Unconfigured";
}
const vessel::SourceHealth *Find(const ProductState &s, vessel::Quantity q,
                                 const std::string &id) {
  for (const auto &v : s.sources)
    if (v.quantity == q && v.source_id == id)
      return &v;
  return nullptr;
}
wxString Health(const vessel::SourceHealth &h, vessel::Time now) {
  const auto a = vessel::Assess(h.sample, now);
  return (a.value ? N(*a.value) + " " + W(vessel::Describe(h.quantity).unit)
                  : "No data") +
         " / " + W(vessel::QualityName(a.quality)) +
         (a.age ? wxString::Format(" / %.1f s", a.age->count() / 1000.) : "") +
         (h.selected ? " / SELECTED" : "");
}
} // namespace
void ProductPanel::SaveSettings(application::Settings s) {
  if (!actions_.save_settings)
    return;
  try {
    application::ValidateSettings(s);
    auto result = actions_.save_settings(s);
    if (result.ok) {
      state_.settings = s;
      state_.settings_status = result.message;
      Build();
    }
    Result(std::move(result));
  } catch (const std::exception &e) {
    Result({false, e.what()});
  }
}
void ProductPanel::EnergySettings() {
  Heading("Energy configuration",
          "Live vessel / Explicit assumptions / Advisory estimates");
  Action("Back to Settings",
         [this] { ShowPage(ProductPage::Settings, mode_); });
  Text(state_.vessel.simulated ? "DEMO uses separate fixture assumptions. "
                                 "Changes below affect live mode only."
                               : "No boat capacity, reserve, current sign or "
                                 "propulsion curve is guessed.");
  LiveText([](const auto &s) { return W(s.settings_status); });
  const auto config = actions_.settings ? actions_.settings() : state_.settings;
  const auto &e = config.energy;
  Text("Usable energy: " + Configured(e.battery.capacity_kwh, "kWh") +
           " / Reserve: " + Configured(e.battery.reserve_soc_percent, "%"),
       18);
  Text("Selected battery: " +
       W(e.battery_device_id.empty() ? "Unconfigured" : e.battery_device_id));
  BeginActions(2);
  Action("Configure battery & reserve", [this] {
    auto s = actions_.settings();
    auto f = EditSheet(
        *this, mode_, "Battery assumptions",
        W("Capacity is deliverable energy over the reported 0–100% SOC span. "
          "Reserve is an explicit percentage. Blank leaves an input "
          "unconfigured. Use decimal dots."),
        {{"Usable capacity / kWh", N(s.energy.battery.capacity_kwh), 64},
         {"Reserve SOC / %", N(s.energy.battery.reserve_soc_percent), 64},
         {"Minimum passage speed / kn", N(s.energy.battery.minimum_speed_kn),
          64}});
    if (!f)
      return;
    try {
      s.energy.battery.capacity_kwh = application::ParseSettingNumber((*f)[0]);
      s.energy.battery.reserve_soc_percent =
          application::ParseSettingNumber((*f)[1]);
      s.energy.battery.minimum_speed_kn =
          application::ParseSettingNumber((*f)[2]);
      s.energy.battery.source =
          "User-configured usable battery energy and reserve / OpenCPN profile";
      SaveSettings(std::move(s));
    } catch (const std::exception &e) {
      Result({false, e.what()});
    }
  });
  Action("Show energy estimates", actions_.energy);
  EndActions();
  Text("BATTERY IDENTITY", 18);
  Text("Choose the whole propulsion pack's SOC source. Voltage and current "
       "must identify the same pack and observation epoch. Source pins are "
       "configured separately under Data Sources.");
  BeginActions(2);
  std::set<std::string> devices;
  for (const auto &h : state_.sources)
    if (h.quantity == vessel::Quantity::BatterySoc &&
        !h.sample.device_id.empty())
      devices.insert(h.sample.device_id);
  for (const auto &id : devices)
    Action("Use battery: " + W(id), [this, id] {
      auto s = actions_.settings();
      s.energy.battery_device_id = id;
      SaveSettings(std::move(s));
    });
  Action("Clear selected battery", [this] {
    auto s = actions_.settings();
    s.energy.battery_device_id.clear();
    SaveSettings(std::move(s));
  });
  EndActions();
  if (devices.empty())
    Text("No live battery SOC source has been observed. Connect a standard "
         "marine source to select its identity.");
  Text("CURRENT CONVENTION", 18);
  Text(config.current == vessel::CurrentConvention::PositiveCharge
           ? "Configured: positive current charges the whole pack"
       : config.current == vessel::CurrentConvention::PositiveDischarge
           ? "Configured: positive current discharges the whole pack"
           : "Unconfigured: current is not interpreted as net consumption");
  BeginActions(3);
  for (const auto &choice :
       std::vector<std::pair<wxString, vessel::CurrentConvention>>{
           {"Unconfigured", vessel::CurrentConvention::Unconfigured},
           {"Positive discharge", vessel::CurrentConvention::PositiveDischarge},
           {"Positive charge", vessel::CurrentConvention::PositiveCharge}})
    Action(choice.first, [this, choice] {
      if (choice.second != vessel::CurrentConvention::Unconfigured &&
          !ConfirmSheet(*this, mode_, "Confirm pack current sign",
                        "Confirm this source measures all pack loads, "
                        "including hotel loads, and that its reported sign "
                        "matches the selected convention. Motor-only current "
                        "is not a whole-pack consumption input.",
                        choice.first))
        return;
      auto s = actions_.settings();
      s.current = choice.second;
      SaveSettings(std::move(s));
    });
  EndActions();
  Text("CONSUMPTION MODEL", 18);
  Text(W(e.consumption == smartnav::ConsumptionModel::MeasuredPack
             ? "Measured whole-pack V × I / estimated electrical power"
             : "Calibrated speed-to-power curve / estimated consumption"));
  Text("Constant present conditions; no weather, current or route-leg "
       "consumption forecast. Estimates are withheld for stale/missing inputs "
       "and outside the imported speed domain.");
  BeginActions(2);
  Action("Use measured pack consumption", [this] {
    auto s = actions_.settings();
    s.energy.consumption = smartnav::ConsumptionModel::MeasuredPack;
    SaveSettings(std::move(s));
  });
  Action("Import calibration CSV", [this] {
    wxFileDialog file(this, "Import recorded speed / power calibration", {}, {},
                      "CSV files (*.csv)|*.csv|All files|*",
                      wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (file.ShowModal() != wxID_OK)
      return;
    try {
      const auto path =
          std::filesystem::u8path(file.GetPath().ToStdString(wxConvUTF8));
      if (std::filesystem::file_size(path) > 32768)
        throw std::invalid_argument("Curve exceeds 32 KiB");
      std::ifstream input(path, std::ios::binary);
      std::string csv(32769, '\0');
      input.read(csv.data(), csv.size());
      csv.resize(static_cast<std::size_t>(input.gcount()));
      if (input.bad())
        throw std::invalid_argument("Cannot read calibration");
      auto s = actions_.settings();
      s.energy.curve = smartnav::ImportPowerCurve(
          csv, file.GetFilename().ToStdString(wxConvUTF8));
      if (!ConfirmSheet(
              *this, mode_, "Use imported propulsion curve",
              W(s.energy.curve.source) +
                  wxString::Format(
                      " / %u points. This changes live advisory consumption "
                      "estimates; capacity and reserve still require explicit "
                      "configuration.",
                      static_cast<unsigned>(s.energy.curve.points.size())),
              "Use curve"))
        return;
      s.energy.consumption = smartnav::ConsumptionModel::CalibratedCurve;
      SaveSettings(std::move(s));
    } catch (const std::exception &e) {
      Result({false, e.what()});
    }
  });
  Action("Curve efficiency & hotel load", [this] {
    auto s = actions_.settings();
    auto f = EditSheet(
        *this, mode_, "Curve conversion assumptions",
        "Motor-electrical curves need hotel load. Shaft curves also need "
        "efficiency. Whole-pack curves already include all electrical loads; "
        "these fields are then unused.",
        {{"Hotel load / kW", N(s.energy.hotel_kw), 64},
         {W("Shaft-to-pack efficiency / 0–1"), N(s.energy.shaft_efficiency),
          64}});
    if (!f)
      return;
    try {
      s.energy.hotel_kw = application::ParseSettingNumber((*f)[0]);
      s.energy.shaft_efficiency = application::ParseSettingNumber((*f)[1]);
      SaveSettings(std::move(s));
    } catch (const std::exception &e) {
      Result({false, e.what()});
    }
  });
  EndActions();
  if (!e.curve.points.empty()) {
    Text("Calibration: " + W(e.curve.source) +
         wxString::Format(" / %u samples",
                          static_cast<unsigned>(e.curve.points.size())));
    Text(wxString(e.curve.reference == smartnav::SpeedReference::ThroughWater
                      ? "STW"
                      : "SOG") +
         " / " +
         (e.curve.basis == smartnav::PowerBasis::WholePack ? "whole pack"
          : e.curve.basis == smartnav::PowerBasis::MotorElectrical
              ? "motor electrical"
              : "shaft") +
         " / Hotel " + Configured(e.hotel_kw, "kW") + " / Efficiency " +
         Configured(e.shaft_efficiency, ""));
  }
}
void ProductPanel::Sources() {
  Heading("Data Sources",
          "OpenCPN input bus / Owned observations / Source precedence");
  Action("Back to Settings",
         [this] { ShowPage(ProductPage::Settings, mode_); });
  Text("GPS position, SOG and COG use OpenCPN's selected navigation input. "
       "Instrument selection below does not replace that service. DEMO data "
       "never enters this live source registry.");
  Action("Connections / Advanced settings",
         actions_.navigation.legacy_settings);
  Text("Explicit propulsion mappings: " +
       wxString::Format("%u", static_cast<unsigned>(
                                  state_.settings.signal_k_mappings.size())));
  BeginActions(2);
  Action("Import propulsion Signal K mapping", [this] {
    wxFileDialog file(this, "Import documented propulsion mapping", {}, {},
                      "CSV files (*.csv)|*.csv|All files|*",
                      wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (file.ShowModal() != wxID_OK)
      return;
    try {
      const auto path =
          std::filesystem::u8path(file.GetPath().ToStdString(wxConvUTF8));
      if (std::filesystem::file_size(path) > 16384)
        throw std::invalid_argument("Mapping exceeds 16 KiB");
      std::ifstream in(path, std::ios::binary);
      std::string csv(16385, '\0');
      in.read(csv.data(), csv.size());
      csv.resize(static_cast<std::size_t>(in.gcount()));
      if (in.bad())
        throw std::invalid_argument("Cannot read mapping");
      auto mappings = application::ImportSignalKMappings(csv);
      wxString review =
          "Confirm against the bridge's documented fields and units. These are "
          "live input conversions, not simulated values. Old instrument "
          "observations will be cleared.\n";
      for (const auto &m : mappings)
        review += W(m.path) + W(" → ") + W(vessel::Describe(m.quantity).name) +
                  W(" / value × ") + N(m.scale) + " + " + N(m.offset) + " " +
                  W(vessel::Describe(m.quantity).unit) + "\n";
      if (!ConfirmSheet(*this, mode_, "Confirm propulsion mapping", review,
                        "Use mappings"))
        return;
      auto settings = actions_.settings();
      settings.signal_k_mappings = std::move(mappings);
      SaveSettings(std::move(settings));
    } catch (const std::exception &e) {
      Result({false, e.what()});
    }
  });
  Action(
      "Remove custom propulsion mappings",
      [this] {
        if (!ConfirmSheet(*this, mode_, "Remove propulsion mappings",
                          "Standard marine inputs remain available. All "
                          "retained instrument observations will be cleared "
                          "and reacquired from normal input messages.",
                          "Remove mappings"))
          return;
        auto settings = actions_.settings();
        settings.signal_k_mappings.clear();
        SaveSettings(std::move(settings));
      },
      !state_.settings.signal_k_mappings.empty());
  EndActions();
  for (const auto &m : state_.settings.signal_k_mappings)
    Text(W(m.path) + " / " + W(vessel::Describe(m.quantity).name) + W(" / × ") +
         N(m.scale) + " + " + N(m.offset) + " " +
         W(vessel::Describe(m.quantity).unit));
  for (const auto &q : vessel::Quantities()) {
    Action(W(q.name), [this, q] {
      source_quantity_ = q.quantity;
      ShowPage(ProductPage::SourceDetail, mode_);
    });
    LiveText([q](const auto &s) {
      for (const auto &h : s.sources)
        if (h.quantity == q.quantity && h.selected)
          return Health(h, s.now) + " / " + W(h.source_id);
      return wxString("No selected source / NO DATA");
    });
  }
}
void ProductPanel::SourceDetail() {
  const auto q = source_quantity_;
  Heading(
      W(vessel::Describe(q).name),
      "Live source precedence / Observation age is never renewed by reading");
  Action("Back to Data Sources",
         [this] { ShowPage(ProductPage::Sources, mode_); });
  const auto config = actions_.settings();
  const auto it = config.sources.find(q);
  const auto policy =
      it == config.sources.end() ? vessel::SourcePolicy{} : it->second;
  Text(policy.pinned_source.empty()
           ? "Automatic: freshest usable validity, then configured transport "
             "priority"
           : "Pinned source: " + W(policy.pinned_source));
  Text("An explicit pin fails closed. A missing or stale pinned source will "
       "not silently switch devices.");
  BeginActions(3);
  Action("Automatic selection", [this, q] {
    auto s = actions_.settings();
    s.sources[q].pinned_source.clear();
    SaveSettings(std::move(s));
  });
  Action("Configure freshness", [this, q] {
    auto s = actions_.settings();
    const auto f = s.sources[q].freshness;
    auto values = EditSheet(
        *this, mode_, "Source freshness",
        "Milliseconds since the original observation. Choose thresholds "
        "suitable for the actual source. These affect instrument data only; "
        "route/GPS freshness remains unchanged.",
        {{"Aging after / ms",
          wxString::Format("%lld",
                           static_cast<long long>(f.aging_after.count())),
          64},
         {"Stale after / ms",
          wxString::Format("%lld",
                           static_cast<long long>(f.stale_after.count())),
          64}});
    if (!values)
      return;
    try {
      auto duration = [](const std::string &v) {
        double d = application::ParseSettingNumber(v);
        if (!std::isfinite(d) || d < 1 || d > 300000 || std::floor(d) != d)
          throw std::invalid_argument("Use whole milliseconds in 1..300000");
        return vessel::Duration{static_cast<long long>(d)};
      };
      s.sources[q].freshness = {duration((*values)[0]), duration((*values)[1])};
      SaveSettings(std::move(s));
    } catch (const std::exception &e) {
      Result({false, e.what()});
    }
  });
  Action("Refresh observed sources", [this] { Build(); });
  EndActions();
  bool any = false;
  for (const auto &h : state_.sources)
    if (h.quantity == q) {
      any = true;
      const auto id = h.source_id;
      Text(W(id), 18);
      LiveText([q, id](const auto &s) {
        const auto h = Find(s, q, id);
        return h ? Health(*h, s.now) : wxString("Source no longer available");
      });
      Text("Device: " +
           W(h.sample.device_id.empty() ? "Unspecified" : h.sample.device_id));
      Text(wxString::Format("Priority %u / Aging %.1fs / Stale %.1fs",
                            h.priority,
                            h.sample.freshness.aging_after.count() / 1000.,
                            h.sample.freshness.stale_after.count() / 1000.));
      Action("Select this source", [this, q, id] {
        auto s = actions_.settings();
        s.sources[q].pinned_source = id;
        SaveSettings(std::move(s));
      });
    }
  if (!any)
    Text("No live source has been observed for this quantity. Missing data "
         "remains unavailable.");
}
} // namespace opennav::ui

namespace opennav::ui {
void ProductPanel::DisplaySettings() {
  Heading("Display", "Palettes / Touch layout / Instruments");
  Action("Back to Settings",
         [this] { ShowPage(ProductPage::Settings, mode_); });
  BeginActions(3);
  for (const auto &choice :
       std::vector<std::pair<wxString, LightMode>>{{"Day", LightMode::Day},
                                                   {"Dusk", LightMode::Dusk},
                                                   {"Night", LightMode::Night}})
    Action(choice.first, [this, choice] {
      if (actions_.theme)
        actions_.theme(choice.second);
    });
  EndActions();
  Text("Palette changes also use OpenCPN's chart presentation. Hardware screen "
       "brightness is controlled by Windows or the display.");
  BeginActions(2);
  Action("Configure data rail",
         [this] { ShowPage(ProductPage::RailLayout, mode_); });
  Action("Configure instruments",
         [this] { ShowPage(ProductPage::InstrumentLayout, mode_); });
  Action("Fullscreen / window", actions_.navigation.fullscreen);
  EndActions();
  Text(wxString::Format("Current UI DPI: %d. Windows display scaling controls "
                        "text and touch dimensions.",
                        GetDPI().x));
}
void ProductPanel::InstrumentSelection(bool rail) {
  Heading(rail ? "Data rail layout" : "Instrument layout",
          "Selected values retain their original source, validity and age");
  Action("Back to Display", [this] { ShowPage(ProductPage::Display, mode_); });
  const auto config = actions_.settings ? actions_.settings() : state_.settings;
  const auto selected = rail ? config.data_rail : config.instruments;
  Text(rail ? "Choose 1 to 6 values. Presets replace the rail; individual "
              "selections appear in selection order."
            : "Choose the values shown on the instrument page. At least one "
              "value must remain selected.");
  if (rail) {
    BeginActions(3);
    for (const auto &preset :
         std::vector<std::pair<wxString, std::vector<std::string>>>{
             {"Navigation rail", {"sog", "cog", "heading", "depth", "aws"}},
             {"Sailing rail", {"aws", "awa", "tws", "twa", "stw", "depth"}},
             {"Energy rail", {"soc", "pack_power", "rpm", "sog", "depth"}}})
      Action(preset.first, [this, preset] {
        auto s = actions_.settings();
        s.data_rail = preset.second;
        SaveSettings(std::move(s));
      });
    EndActions();
  }
  BeginActions(2);
  for (const auto &item : vessel::DisplayItems(state_.vessel)) {
    const std::string key = item.key;
    const bool included =
        std::find(selected.begin(), selected.end(), key) != selected.end();
    Action((included ? "Shown / " : "Add / ") + W(item.title),
           [this, rail, key] {
             auto s = actions_.settings();
             auto &list = rail ? s.data_rail : s.instruments;
             const auto found = std::find(list.begin(), list.end(), key);
             if (found != list.end())
               list.erase(found);
             else
               list.push_back(key);
             SaveSettings(std::move(s));
           },
           included ? selected.size() > 1 : !rail || selected.size() < 6);
  }
  EndActions();
}
} // namespace opennav::ui
