#include "ui/ChartInfoDrawer.h"
#include <algorithm>

namespace opennav::ui {
namespace {
wxString W(const std::string &value) { return wxString::FromUTF8(value); }
}
XNavChartInfoDrawer::XNavChartInfoDrawer(wxWindow &owner)
    : XNavDrawer(owner, "SKAGER chart information") {
  SetHeading("CHART OBJECTS", "Chart information", false);
  SetWide(true);
  body_->Bind(wxEVT_SIZE, [this](wxSizeEvent &event) { Wrap(); event.Skip(); });
}
wxStaticText *XNavChartInfoDrawer::Text(const std::string &value, int size,
                                      bool secondary) {
  auto *text = new wxStaticText(body_, wxID_ANY, wxEmptyString,
                                wxDefaultPosition, wxDefaultSize,
                                wxST_NO_AUTORESIZE);
  text->SetLabelText(W(value));
  text->SetFont(UiFont(*text, size));
  text->SetName(W(value));
  EnableScrollGesture(*text);
  text_.push_back({text, W(value), secondary});
  content_->Add(text, 0, wxEXPAND | wxBOTTOM, FromDIP(10));
  return text;
}
void XNavChartInfoDrawer::Open(application::ChartInfo info,
                              const wxRect &workspace, LightMode mode) {
  info_ = std::move(info);
  Build();
  UpdateLight(mode);
  Present(workspace);
  Wrap();
}
void XNavChartInfoDrawer::Build() {
  ClearBody();
  text_.clear();
  buttons_.clear();
  if (info_.position_valid)
    Text(wxString::Format(W("%.5f°, %.5f°"), info_.latitude,
                          info_.longitude).ToStdString(wxConvUTF8), 12, true);
  if (!info_.notice.empty()) Text(info_.notice, 14);
  if (!info_.objects.empty())
    Text(std::to_string(info_.objects.size()) +
         (info_.objects.size() == 1 ? " information section from OpenCPN"
                                   : " information sections from OpenCPN"), 12, true);
  for (std::size_t i = 0; i < info_.objects.size(); ++i) {
    const auto &object = info_.objects[i];
    Text(object.title, 20);
    if (!object.kind.empty() && object.kind != object.title) Text(object.kind, 12, true);
    std::string summary;
    for (const auto &line : object.summary) {
      if (!summary.empty()) summary += '\n';
      summary += line;
    }
    if (!summary.empty()) Text(summary, 14);
    auto *toggle = new XNavButton(body_, wxID_ANY, "Show all chart details",
                                  "Show all chart details for " + W(object.title));
    toggle->SetRole(ButtonRole::Quiet);
    toggle->SetMinSize(FromDIP(wxSize(240, 48)));
    toggle->SetInterfaceScale(InterfaceScale());
    content_->Add(toggle, 0, wxEXPAND | wxBOTTOM, FromDIP(10));
    buttons_.push_back(toggle);
    auto *details = Text(object.details, 12, true);
    details->Hide();
    toggle->Bind(wxEVT_BUTTON, [this, toggle, details](wxCommandEvent &) {
      const bool show = !details->IsShown();
      details->Show(show);
      toggle->SetLabel(show ? "Hide chart details" : "Show all chart details");
      Wrap();
    });
    if (i + 1 < info_.objects.size()) content_->AddSpacer(FromDIP(18));
  }
  Text("Chart information is supplied by OpenCPN and the selected charts. "
       "Additional file references are shown as text in the full details.", 12, true);
}
void XNavChartInfoDrawer::UpdateLight(LightMode mode) {
  SetLight(mode);
  const auto palette = Theme(mode);
  for (auto &row : text_) {
    row.window->SetBackgroundColour(Colour(palette.background));
    row.window->SetForegroundColour(Colour(row.secondary ? palette.secondary : palette.primary));
  }
  for (auto *button : buttons_) button->SetLightMode(mode);
}
void XNavChartInfoDrawer::Wrap() {
  if (wrapping_) return;
  wrapping_ = true;
  const int width = (std::max)(FromDIP(160), body_->GetClientSize().x - FromDIP(48));
  for (auto &row : text_) {
    if (!row.window->IsShown()) continue;
    row.window->SetLabelText(row.original);
    row.window->Wrap(width);
    row.window->SetMinSize(wxDefaultSize);
    row.window->InvalidateBestSize();
    row.window->SetMinSize(wxSize(width, row.window->GetBestSize().y));
  }
  body_->Layout();
  body_->FitInside();
  wrapping_ = false;
}
} // namespace opennav::ui
