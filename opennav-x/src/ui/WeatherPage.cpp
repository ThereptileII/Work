// SCRUM-325/327/329/331: forecast wind page, GRIBstream settings and the
// advisory route forecast section. Presentation only: no fetching here, no
// credential is ever displayed, and nothing changes a route or navigation.
#include "ui/ProductPanel.h"
#include "ui/DisplaySizing.h"
#include "ui/Sheet.h"
#include "vessel/RouteProgress.h"
#include "weather/ForecastView.h"
#include <wx/datetime.h>
#include <wx/dialog.h>
#include <wx/textctrl.h>
#include <algorithm>
#include <cmath>

namespace opennav::ui {
namespace {
using weather::ForecastDisplay;
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
weather::WallTime WallNow() { return weather::WallTime::clock::now(); }
wxString LocalTime(weather::WallTime t) {
  return wxDateTime(static_cast<time_t>(
      std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch()).count()))
      .Format("%H:%M");
}
const char *StateName(weather::ForecastState state) {
  switch (state) {
  case weather::ForecastState::Disabled: return "Off";
  case weather::ForecastState::NoCredential: return "Needs a token";
  case weather::ForecastState::Loading: return "Loading";
  case weather::ForecastState::Live: return "Live";
  case weather::ForecastState::Stale: return "Stale";
  case weather::ForecastState::Error: return "Error";
  }
  return "Unknown";
}
std::optional<double> Usable(const vessel::Sample &sample, vessel::Time now) {
  const auto a = vessel::Assess(sample, now);
  if (!a.value || (a.quality != vessel::Quality::Live && a.quality != vessel::Quality::Aging))
    return {};
  return a.value;
}
} // namespace

bool ProductPanel::RefreshWeather(bool force) {
  if (!actions_.weather.read) return false;
  const auto now = std::chrono::steady_clock::now();
  if (!force && now - weather_refreshed_at_ < std::chrono::seconds(2)) return false;
  weather_refreshed_at_ = now;
  weather_ = actions_.weather.read(now);
  return true;
}

void ProductPanel::WeatherPage() {
  RefreshWeather(true);
  Heading("Weather", "Forecast wind / advisory, never measured onboard wind");
  if (!actions_.weather.read) {
    Text("Forecast wind is unavailable in this build.");
    return;
  }
  const auto selected = [this]() -> std::optional<weather::WallTime> {
    return actions_.navigation.weather_time ? actions_.navigation.weather_time() : std::nullopt;
  };
  Visual("Forecast wind at vessel", 214, [this, selected](XNavPainter &p, wxDC &, int width) {
    const auto now = WallNow();
    const int split = width * 58 / 100;
    p.Card(0, 0, width, 210, "FORECAST WIND AT VESSEL");
    // Measured onboard wind stays separate and is labelled as such.
    p.Text("MEASURED ONBOARD", split + 12, 16, 11, p.c.secondary, false, width - split - 24);
    const auto measured = vessel::Assess(state_.vessel.wind.true_speed_kn, state_.now);
    const bool measured_ok = measured.value && (measured.quality == vessel::Quality::Live ||
                                                measured.quality == vessel::Quality::Aging);
    p.Text(measured_ok ? wxString::Format("%.1f", *measured.value) : wxString::FromUTF8("\xE2\x80\x94"),
           split + 12, 48, 36, measured_ok ? p.c.primary : p.c.muted, false, width - split - 24);
    p.Text(measured_ok ? wxString("kn true wind speed / sensor")
                       : wxString(measured.quality == vessel::Quality::Unavailable ? "No measured wind"
                                  : W(vessel::QualityName(measured.quality))),
           split + 12, 100, 12, p.c.secondary, false, width - split - 24);
    const auto display = weather::DisplayState(weather_, now);
    if (display == ForecastDisplay::Unavailable) {
      p.Wrapped(W(weather::UnavailableReason(weather_, now)), 24, 56, 14, 22, split - 48,
                p.c.secondary, 4);
      return;
    }
    const bool stale = display == ForecastDisplay::Stale;
    const auto step = weather::ResolveDisplayTime(weather::ValidTimes(weather_), selected(), now);
    const auto lat = Usable(state_.vessel.navigation.latitude_deg, state_.now);
    const auto lon = Usable(state_.vessel.navigation.longitude_deg, state_.now);
    std::optional<weather::ForecastWind> wind;
    if (lat && lon && step) wind = weather::WindNear(weather_, {*lat, *lon}, *step);
    if (!lat || !lon) {
      p.Text("Vessel position unavailable", 24, 56, 18, p.c.muted, false, split - 48);
    } else if (!wind) {
      p.Text("No forecast coverage at the vessel", 24, 56, 18, p.c.muted, false, split - 48);
    } else {
      const double kn = weather::KnotsFromMps(wind->speed_mps);
      p.Text(wxString::Format("%.0f", kn), 24, 40, 48, stale ? p.c.muted : p.c.primary, false, 110);
      p.Text("kn", 134, 72, 14, p.c.secondary);
      const double from = weather::NormalizeDegrees(wind->direction_from_true_deg);
      p.Text(wxString::Format(W("from %03.0f\xC2\xB0 %s true"), std::fmod(std::round(from), 360.0),
                              weather::CompassPoint(from)),
             24, 108, 16, stale ? p.c.muted : p.c.primary, false, split - 48);
    }
    p.Text(W(weather::ProvenanceLabel(weather_, now)), 24, 150, 11,
           stale ? p.c.attention : p.c.secondary, false, width - 48);
    if (step)
      p.Text("Valid " + LocalTime(*step) + " (" + W(weather::StepLabel(*step, now)) + ")", 24, 174,
             11, p.c.secondary, false, width - 48);
  });
  // Forecast time step, shared with the chart wind layer.
  Visual("Forecast time", 40, [this, selected](XNavPainter &p, wxDC &, int width) {
    const auto now = WallNow();
    const auto times = weather::ValidTimes(weather_);
    const auto step = weather::ResolveDisplayTime(times, selected(), now);
    p.Text("FORECAST TIME", 0, 4, 11, p.c.secondary, false, width / 2);
    wxString value = "No forecast steps";
    if (step) {
      value = W(weather::StepLabel(*step, now)) + "  /  " + LocalTime(*step);
      if (!times.empty())
        value += "  /  " + W(weather::StepLabel(times.front(), now)) + " to " +
                 W(weather::StepLabel(times.back(), now));
    }
    p.TextWeight(value, width / 3, 2, 13, p.c.primary, 500, width - width / 3, true);
  });
  const auto step_to = [this, selected](int delta) {
    if (!actions_.navigation.set_weather_time) return;
    const auto times = weather::ValidTimes(weather_);
    if (delta == 0) actions_.navigation.set_weather_time(std::nullopt);
    else if (const auto next = weather::StepDisplayTime(times, selected(), WallNow(), delta))
      actions_.navigation.set_weather_time(next);
    for (auto *visual : visuals_) visual->Refresh(false);
  };
  const bool can_step = bool(actions_.navigation.set_weather_time);
  BeginActions(3, 140);
  Action("Earlier", [step_to] { step_to(-1); }, can_step)->SetRole(ButtonRole::Quiet);
  Action("Now", [step_to] { step_to(0); }, can_step)->SetRole(ButtonRole::Quiet);
  Action("Later", [step_to] { step_to(1); }, can_step)->SetRole(ButtonRole::Quiet);
  EndActions();
  const auto layer = actions_.navigation.chart_presentation
      ? actions_.navigation.chart_presentation().wind_vectors : application::ChartLayerState{};
  const bool shown = layer.visible.value_or(false);
  Action(shown ? "Hide wind arrows on chart" : "Show wind arrows on chart", [this, shown] {
    if (!actions_.navigation.set_chart_wind) return;
    const auto result = actions_.navigation.set_chart_wind(!shown);
    Build();
    Result(result.command);
  }, layer.visible.has_value() && layer.editable && bool(actions_.navigation.set_chart_wind));
  Text("Chart wind arrows are off by default and also available in Chart layers. "
       "Arrows point downwind; numbers are knots. Fewer arrows are drawn when zoomed out.", 12);

  // SCRUM-325: provider settings, modelled on Online AIS.
  Text("GRIBstream provider", 20);
  const bool token = actions_.weather.token_present && actions_.weather.token_present();
  LiveText([this, token](const ProductState &) {
    wxString text = wxString("Forecasts: ") +
        (weather_.state == weather::ForecastState::Disabled ? "Off" : "Enabled") +
        "   /   Token: " + (token ? "Stored securely" : "Not configured") +
        "\nStatus: " + StateName(weather_.state);
    if (!weather_.status.empty()) text += W(" \xE2\x80\x94 ") + W(weather_.status);
    return text;
  });
  BeginActions(2);
  const bool enabled = weather_.state != weather::ForecastState::Disabled;
  const auto toggle = [this](bool on) {
    if (!actions_.weather.enable) return;
    const auto result = actions_.weather.enable(on);
    RefreshWeather(true);
    Build();
    Result(result);
  };
  auto *off = Action("Off", [toggle] { toggle(false); }, bool(actions_.weather.enable));
  off->SetSelected(!enabled);
  auto *on = Action("Enabled", [toggle] { toggle(true); }, bool(actions_.weather.enable));
  on->SetSelected(enabled);
  Action(token ? "Replace token" : "Save token", [this] { StoreWeatherToken(); },
         bool(actions_.weather.store_token));
  Action("Remove token", [this] {
    if (!actions_.weather.remove_token ||
        !ConfirmSheet(*this, mode_, "Remove GRIBstream token",
                      "Forecast wind will stop updating. The saved token is removed "
                      "from this Windows account.", "Remove token", interface_scale_))
      return;
    const auto result = actions_.weather.remove_token();
    RefreshWeather(true);
    Build();
    Result(result);
  }, token && bool(actions_.weather.remove_token))->SetRole(ButtonRole::Critical);
  Action("Test connection", [this] {
    if (!actions_.weather.test_connection) return;
    // Asynchronous: the outcome appears in the status line above.
    Result(actions_.weather.test_connection());
  }, token && bool(actions_.weather.test_connection));
  EndActions();
  Text("Use your own GRIBstream API token. It is stored securely and never shown, "
       "logged or exported. Forecasts are advisory: they do not replace onboard "
       "measurements or official weather information.", 12);
}

void ProductPanel::StoreWeatherToken() {
  if (!actions_.weather.store_token) return;
  // Masked entry; only the bounded Secret crosses to storage.
  wxDialog prompt(this, wxID_ANY, "GRIBstream token", wxDefaultPosition, wxDefaultSize,
                  wxBORDER_NONE | wxTAB_TRAVERSAL);
  prompt.SetName("GRIBstream token");
  prompt.SetBackgroundColour(Colour(Theme(mode_).background));
  auto *layout = new wxBoxSizer(wxVERTICAL);
  auto *title = new wxStaticText(&prompt, wxID_ANY, "GRIBstream token");
  title->SetFont(UiFont(prompt, 22));
  title->SetForegroundColour(Colour(Theme(mode_).primary));
  layout->Add(title, 0, wxALL, FromDIP(20));
  auto *detail = new wxStaticText(&prompt, wxID_ANY,
      "Enter your own GRIBstream API token. It is stored securely for this "
      "Windows account. Saving a token does not enable forecasts.");
  detail->SetFont(UiFont(prompt, 12));
  detail->SetForegroundColour(Colour(Theme(mode_).secondary));
  detail->Wrap(FromDIP(390));
  layout->Add(detail, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(20));
  auto *entry = new wxTextCtrl(&prompt, wxID_ANY, "", wxDefaultPosition,
      prompt.FromDIP(wxSize(390, DisplayFieldHeight(interface_scale_))),
      wxTE_PASSWORD | wxBORDER_NONE);
  entry->SetName("Protected GRIBstream token");
  entry->SetMaxLength(512);
  entry->SetFont(UiFont(prompt, DisplayFieldFont(interface_scale_)));
  entry->SetBackgroundColour(Colour(Theme(mode_).surface));
  entry->SetForegroundColour(Colour(Theme(mode_).primary));
  layout->Add(entry, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(20));
  auto *row = new wxBoxSizer(wxHORIZONTAL);
  XNavButton *save = nullptr;
  for (auto choice : {std::pair<wxString, int>{"Cancel", wxID_CANCEL}, {"Save token", wxID_OK}}) {
    auto *button = new XNavButton(&prompt, wxID_ANY, choice.first, choice.first);
    button->SetLightMode(mode_);
    button->SetRole(choice.second == wxID_OK ? ButtonRole::Primary : ButtonRole::Quiet);
    if (choice.second == wxID_OK) { save = button; save->Disable(); }
    button->Bind(wxEVT_BUTTON, [&prompt, id = choice.second](wxCommandEvent &) {
      prompt.EndModal(id);
    });
    row->Add(button, 1, wxALL, FromDIP(4));
  }
  entry->Bind(wxEVT_TEXT, [entry, save](wxCommandEvent &) {
    const auto value = entry->GetValue();
    bool valid = !value.empty() && value.length() <= 512;
    for (const auto ch : value) valid = valid && ch >= 33 && ch <= 126;
    save->Enable(valid);
  });
  layout->Add(row, 0, wxEXPAND | wxALL, FromDIP(16));
  prompt.SetSizerAndFit(layout);
  auto *frame = wxGetTopLevelParent(this);
  while (frame->GetParent()) frame = wxGetTopLevelParent(frame->GetParent());
  const auto available = frame->GetClientSize(), size = prompt.GetSize();
  prompt.Move(frame->ClientToScreen({(available.x - size.x) / 2, (available.y - size.y) / 2}));
  prompt.Bind(wxEVT_CHAR_HOOK, [&prompt](wxKeyEvent &e) {
    if (e.GetKeyCode() == WXK_ESCAPE) prompt.EndModal(wxID_CANCEL);
    else e.Skip();
  });
  entry->SetFocus();
  application::CommandResult result{false, ""};
  bool attempted = false;
  if (prompt.ShowModal() == wxID_OK) {
    wxString entered = entry->GetValue();
    wxCharBuffer encoded(entered.utf8_str().data());
    ais::Secret key;
    const bool valid = key.Assign(std::string_view(encoded.data(), encoded.length()));
    volatile char *bytes = encoded.data();
    for (std::size_t i = 0; i < encoded.length(); ++i) bytes[i] = 0;
    for (std::size_t i = 0; i < entered.length(); ++i) entered[i] = wxUniChar(0);
    entry->ChangeValue("");
    result = valid ? actions_.weather.store_token(key)
                   : application::CommandResult{false, "Token is empty or invalid"};
    attempted = true;
  }
  entry->ChangeValue("");
  if (!attempted) return;
  RefreshWeather(true);
  Build();
  Result(result);
}

void ProductPanel::RouteForecast() {
  Text("Forecast wind along route", 20);
  if (!actions_.weather.read) {
    Text("Forecast wind is unavailable in this build.");
    return;
  }
  RefreshWeather(true);
  if (actions_.weather.focus_route) {
    std::vector<weather::Coordinate> focus;
    for (const auto &w : route_.points) focus.push_back({w.latitude_deg, w.longitude_deg});
    actions_.weather.focus_route(std::move(focus));  // Coverage for this route.
  }
  const auto rows = std::max<std::size_t>(1, std::min(route_.points.size(),
                                                      weather::kMaxRouteForecastRows));
  Visual("Forecast wind along route", 76 + 48 * static_cast<int>(rows),
         [this](XNavPainter &p, wxDC &, int width) {
    const auto now = WallNow();
    std::vector<weather::PassPoint> points;
    bool from_vessel = false;
    const auto progress = state_.vessel.navigation.route;
    if (route_.active && progress && progress->route_id == route_.id &&
        vessel::AssessRoute(*progress, state_.now).remaining_distance_nm &&
        !progress->remaining_steps.empty()) {
      // Active: remaining waypoints, first leg from the current position.
      from_vessel = true;
      for (const auto &s : progress->remaining_steps)
        points.push_back({s.name, {s.latitude_deg, s.longitude_deg}, s.distance_from_previous_nm});
    } else {
      for (std::size_t i = 0; i < route_.points.size(); ++i) {
        const auto &w = route_.points[i];
        points.push_back({w.name, {w.latitude_deg, w.longitude_deg},
                          i ? w.incoming_nm : std::nullopt});
      }
    }
    const auto sog = Usable(state_.vessel.navigation.sog_kn, state_.now);
    const auto forecast = weather::AssembleRouteForecast(points, from_vessel, weather_, now, sog,
                                                         "current SOG");
    if (forecast.display == ForecastDisplay::Unavailable) {
      p.Wrapped(W(weather::UnavailableReason(weather_, now)), 0, 0, 13, 20, width, p.c.secondary, 3);
      return;
    }
    const bool stale = forecast.display == ForecastDisplay::Stale;
    p.Text(W(weather::ProvenanceLabel(weather_, now)), 0, 0, 11,
           stale ? p.c.attention : p.c.secondary, false, width);
    p.Wrapped(W(forecast.assumption), 0, 22, 11, 17, width, p.c.secondary, 2);
    int y = 64;
    for (const auto &row : forecast.points) {
      p.Text(W(row.name.empty() ? std::string("Unnamed") : row.name), 0, y, 14, p.c.primary,
             false, width / 2 - 8);
      p.TextWeight(row.wind ? W(weather::WindLabel(*row.wind)) : wxString("No forecast coverage"),
                   width / 2, y, 14, row.wind && !stale ? p.c.primary : p.c.muted, 500,
                   width / 2, true);
      const wxString when = row.eta
          ? "Passing about " + LocalTime(*row.eta) + " (" + W(weather::StepLabel(*row.eta, now)) + ")"
          : wxString("Forecast for now (no passing time)");
      p.Text(when, 0, y + 22, 11, p.c.muted, false, width);
      p.Rule(0, y + 42, width);
      y += 48;
    }
    if (forecast.truncated)
      p.Text("Only the first waypoints are shown.", 0, y, 11, p.c.muted, false, width);
  });
  Text("Advisory only. The forecast never changes, activates or reroutes this route.", 12);
}
} // namespace opennav::ui
