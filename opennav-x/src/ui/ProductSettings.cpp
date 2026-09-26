#include "ui/ProductPanel.h"
#include "ui/Sheet.h"
#include "integration/BuildFeatures.h"
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
  const auto age = now >= h.sample.observed_at
      ? std::optional<double>{std::chrono::duration<double>(now - h.sample.observed_at).count()}
      : std::nullopt;
  return (a.value ? N(*a.value) + " " + W(vessel::Describe(h.quantity).unit)
                  : "No data") +
         " / " + (h.sample.validity == vessel::Validity::Invalid
                       ? wxString("INVALID") : W(vessel::QualityName(a.quality))) +
         (age ? wxString::Format(" / %.2f s", *age) : " / Clock mismatch") +
         (h.frequency_hz ? wxString::Format(" / %.1f Hz", *h.frequency_hz)
                         : " / Rate unavailable") +
         (h.selected ? " / SELECTED" : "");
}
} // namespace
void ProductPanel::SaveSettings(application::Settings s) {
  if (state_.vessel.replayed) {
    Result({false, "Stop REPLAY before changing live settings"});
    return;
  }
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
  Text("Set your boat's usable battery capacity, reserve and measured consumption.");
#if XNAV_ENABLE_TEST_FIXTURES
  if (state_.vessel.simulated)
    Text("DEMO has separate test assumptions. These settings affect live input only.");
#endif
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
  Heading("Sensors", "Connection health at a glance");
  BeginActions(2);
  Action("Connections / Advanced settings", actions_.navigation.legacy_settings);
  Action("Motor & battery setup", [this] { ShowPage(ProductPage::BoatMapping, mode_); });
  EndActions();
  auto status = [](const vessel::Sample &sample, vessel::Time now) {
    const auto a = vessel::Assess(sample, now);
    if (a.quality == vessel::Quality::Stale) return wxString("Stale");
    if (a.quality == vessel::Quality::Aging) return wxString("Aging");
    if (a.quality == vessel::Quality::Uncertain) return wxString("Check source");
    return wxString(a.value ? "Connected" : "No data");
  };
  BeginActions(2);
  StatusAction("GPS", [status](const auto &s) { return status(s.vessel.navigation.latitude_deg, s.now); },
               [this] { source_quantity_ = vessel::Quantity::Count; ShowPage(ProductPage::SourceDetail, mode_); });
  for (const auto &entry : std::vector<std::pair<wxString, vessel::Quantity>>{
      {"Heading", vessel::Quantity::Heading}, {"Depth", vessel::Quantity::Depth},
      {"Wind", vessel::Quantity::ApparentWindSpeed}, {"Speed through water", vessel::Quantity::WaterSpeed},
      {"Rudder", vessel::Quantity::Rudder}, {"Motor", vessel::Quantity::MotorRpm},
      {"Battery", vessel::Quantity::BatterySoc}, {"Water temperature", vessel::Quantity::WaterTemperature},
      {"Fresh water", vessel::Quantity::FreshWater}, {"Waste tank", vessel::Quantity::Waste}})
    StatusAction(entry.first, [status, q = entry.second](const auto &s) {
      return status(vessel::Field(s.vessel, q), s.now);
    }, [this, q = entry.second] { source_quantity_ = q; ShowPage(ProductPage::SourceDetail, mode_); });
  StatusAction("AIS", [](const auto &s) {
    return wxString(s.ais.available && s.now >= s.ais.observed_at &&
          s.now - s.ais.observed_at < std::chrono::seconds(5) ? "Connected" : "No current data");
  }, [this] { ShowPage(ProductPage::Ais, mode_); });
  EndActions();
  Text("Select a sensor for its value, source and last update. Connection changes apply without restarting XNav.");
  BeginActions(2);
  Action("Advanced source details", [this] { ShowPage(ProductPage::SourcesAdvanced, mode_); });
  Action("System diagnostics", actions_.diagnostics);
  Action("Commissioning & recordings", [this] { ShowPage(ProductPage::Commissioning, mode_); });
  EndActions();
}
void ProductPanel::BoatMapping() {
  Heading("Motor & battery setup", "Advanced / Identify the installed bridge");
  LiveText([](const auto &s) { return W(s.boat_bridge_status); });
  Text("The inspected bridge uses engine coolant for motor temperature and a "
       "virtual fuel tank for SOC. This mapping suppresses that fictional tank "
       "and reads its documented regeneration extension. Freshness requires "
       "the reviewed v2 producer expiry firmware; older input stays uncertain "
       "and cannot support energy predictions.");
  BeginActions(2);
  Action("Bind boat propulsion bridge", [this] {
    auto s=actions_.settings();
    auto values=EditSheet(*this,mode_,"Boat propulsion identity",
        "Copy the exact interface and NAME from an actual address claim. "
        "Verify the commissioning firmware procedure. No control output is enabled.",
        {{"OpenCPN interface",W(s.boat_bridge.interface_id),140},
         {"Observed NAME / lowercase hex",W(s.boat_bridge.name),16}});
    if(!values)return;
    s.boat_bridge={(*values)[0],(*values)[1]};SaveSettings(std::move(s));
  });
  Action("Remove boat mapping",[this] {
    if(!ConfirmSheet(*this,mode_,"Remove boat mapping?",
       "Standard PGN meanings return. The inspected bridge's virtual SOC fuel "
       "tank must not be mistaken for physical fuel. Old samples will be cleared.","Remove mapping"))return;
    auto s=actions_.settings();s.boat_bridge={};SaveSettings(std::move(s));
  },!state_.settings.boat_bridge.interface_id.empty());
  EndActions();
}
void ProductPanel::SourceDetail() {
  if (source_quantity_ == vessel::Quantity::Count) {
    Heading("GPS", "Position and speed selected by OpenCPN");
    LiveText([](const auto &s) {
      const auto position = vessel::Assess(s.vessel.navigation.latitude_deg, s.now);
      return W(vessel::QualityName(position.quality)) +
             (position.age ? wxString::Format(" / Last update %.1f s ago", position.age->count() / 1000.) : wxString(" / No observation"));
    });
    Value("LATITUDE", "°", [](const auto &s) { return s.vessel.navigation.latitude_deg; }, 5);
    Value("LONGITUDE", "°", [](const auto &s) { return s.vessel.navigation.longitude_deg; }, 5);
    Value("SOG", "kn", [](const auto &s) { return s.vessel.navigation.sog_kn; });
    LiveText([](const auto &s) { return "Source: " + W(s.vessel.navigation.latitude_deg.source); });
    Action("Connections / Advanced settings", actions_.navigation.legacy_settings);
    return;
  }
  const auto q = source_quantity_;
  Heading(
      W(vessel::Describe(q).name),
      "Selected source and recent observations");
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
        return h ? Health(*h, s.now) + "\n" +
                       W(vessel::ValidityName(h->sample.validity)) +
                       " / Observations " + W(std::to_string(h->observations)) +
                       " / Invalid " + W(std::to_string(h->invalid_observations)) +
                       "\nObservation stamp (session ms): " +
                       W(std::to_string(std::chrono::duration_cast<vessel::Duration>(
                           h->sample.observed_at.time_since_epoch()).count()))
                 : wxString("Source no longer available");
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
          rail ? "Four essentials, always visible" : "Choose the readings useful to you");
  Action("Back to Display", [this] { ShowPage(ProductPage::Display, mode_); });
  const auto config = actions_.settings ? actions_.settings() : state_.settings;
  auto selected = rail ? config.data_rail : config.instruments;
  if (rail && selected.size() > 4) selected.resize(4);
  const auto normalize = [](application::Settings &settings) {
    for (std::size_t i = 4; i < settings.data_rail.size(); ++i)
      if (std::find(settings.instruments.begin(), settings.instruments.end(), settings.data_rail[i]) == settings.instruments.end())
        settings.instruments.push_back(settings.data_rail[i]);
    if (settings.data_rail.size() > 4) settings.data_rail.resize(4);
  };
  Text(rail ? "Choose up to four readings. Move the most useful one to the top. Additional instruments stay available on the Instruments screen."
            : "Choose the values shown in each instrument group. At least one must remain selected.");
  if (rail) {
    BeginActions(3);
    for (const auto &preset : std::vector<std::pair<wxString, std::vector<std::string>>>{
        {"Navigation rail", {"sog", "depth", "aws", "heading"}},
        {"Sailing rail", {"aws", "awa", "heading", "depth"}},
        {"Energy rail", {"soc", "pack_power", "sog", "depth"}}})
      Action(preset.first, [this, preset, normalize] {
        auto s = actions_.settings(); normalize(s); s.data_rail = preset.second; SaveSettings(std::move(s));
      });
    EndActions();
    for (std::size_t i = 0; i < selected.size(); ++i) {
      wxString title = W(selected[i]);
      for (const auto &item : vessel::DisplayItems(state_.vessel))
        if (selected[i] == item.key) title = W(item.title);
      Text(wxString::Format("%u  ", static_cast<unsigned>(i + 1)) + title, 18);
      BeginActions(2, 140);
      for (int direction : {-1, 1})
        Action(direction < 0 ? "Move up" : "Move down", [this, normalize, i, direction] {
          auto s = actions_.settings(); normalize(s);
          const auto other = static_cast<int>(i) + direction;
          if (i < s.data_rail.size() && other >= 0 && other < static_cast<int>(s.data_rail.size()))
            std::swap(s.data_rail[i], s.data_rail[static_cast<std::size_t>(other)]);
          SaveSettings(std::move(s));
        }, direction < 0 ? i > 0 : i + 1 < selected.size());
      EndActions();
    }
  }
  BeginActions(2);
  for (const auto &item : vessel::DisplayItems(state_.vessel)) {
    const std::string key = item.key;
    const bool included =
        std::find(selected.begin(), selected.end(), key) != selected.end();
    Action((included ? "Shown / " : "Add / ") + W(item.title),
           [this, rail, key, normalize] {
             auto s = actions_.settings();
             if (rail) normalize(s);
             auto &list = rail ? s.data_rail : s.instruments;
             const auto found = std::find(list.begin(), list.end(), key);
             if (found != list.end())
               list.erase(found);
             else
               list.push_back(key);
             SaveSettings(std::move(s));
           },
           included ? selected.size() > 1 : !rail || selected.size() < 4);
  }
  EndActions();
}
} // namespace opennav::ui
