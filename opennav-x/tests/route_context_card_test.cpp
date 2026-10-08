// Offline context fixture; route actions are counters, never navigation output.
#include "ui/RouteContextCard.h"
#include <iostream>
#include <stdexcept>
#include <wx/app.h>
#include <wx/frame.h>
#include <wx/log.h>

using namespace opennav;
namespace {
void Check(bool value, const char *why) { if (!value) throw std::runtime_error(why); }
template <typename T> T *Find(wxWindow *parent, const wxString &label) {
  for (auto *child : parent->GetChildren()) {
    if (auto *value = dynamic_cast<T *>(child); value && value->GetLabel() == label) return value;
    if (auto *value = Find<T>(child, label)) return value;
  }
  return nullptr;
}
void Click(wxWindow &window) {
  wxCommandEvent event(wxEVT_BUTTON, window.GetId());
  event.SetEventObject(&window); window.ProcessWindowEvent(event);
}
class TestApp final : public wxApp {
public:
  bool OnInit() override {
    wxLog::SetActiveTarget(new wxLogStderr());
    frame_ = new wxFrame(nullptr, wxID_ANY, "Offline route context fixture", {0,0}, {1280,800});
    frame_->Show();
    route_.id = "fixture-route"; route_.name = "Fixture route";
    route_.active = route_.visible = route_.editable = true;
    application::Waypoint first, last;
    first.name = "Departure"; last.name = "Destination"; route_.points = {first, last};
    CallAfter([this] { Run(); });
    return true;
  }
  int OnRun() override { wxApp::OnRun(); return result_; }
  int OnExit() override { return result_; }
private:
  ui::XNavRouteContextCard *Card() {
    auto *card = new ui::XNavRouteContextCard(*frame_, route_.id,
        [this](ui::RouteContextAction action, const std::string &id) {
          if (id != route_.id) result_ = 1;
          if (action == ui::RouteContextAction::Details) ++details_;
          else if (action == ui::RouteContextAction::Activate) ++activations_;
          else if (action == ui::RouteContextAction::Stop) ++stops_;
          else ++views_;
        }, [this] { ++dismissals_; });
    card->UpdateRoute(route_, ui::LightMode::Day);
    Check(card->Place(frame_->GetScreenRect()), "Compact route card fits within the chart");
    card->ShowWithoutActivating();
    return card;
  }
  void Run() {
    try {
      auto *card = Card();
      auto *details = Find<ui::XNavButton>(card, "Details");
      auto *view = Find<ui::XNavButton>(card, "View on chart");
      Check(details && view && details->IsEnabled() && view->IsEnabled(),
            "Current route offers modern details and chart actions");
      Check(details->GetMinSize().y >= card->FromDIP(48), "Actions have touch-sized targets");
      for (const auto mode : {ui::LightMode::Day, ui::LightMode::Dusk, ui::LightMode::Night}) {
        card->UpdateRoute(route_, mode);
        Check(card->View().name == route_.name && card->View().active,
              "Theme updates preserve actual route identity and active status");
        Check(card->GetBackgroundColour() == ui::Colour(ui::Theme(mode).elevated),
              "Route hover uses the current XNav theme");
      }
      auto *stop = Find<ui::XNavButton>(card, "Stop navigation");
      Check(stop && stop->IsEnabled() && !Find<ui::XNavButton>(card, "Activate route"),
            "Active route offers Stop navigation, never a duplicate activation");
      auto inactive = route_;
      inactive.active = false;
      card->UpdateRoute(inactive, ui::LightMode::Day);
      Check(stop->GetLabel() == "Activate route" && stop->IsEnabled(),
            "Inactive editable route offers Activate route");
      auto protected_route = inactive;
      protected_route.editable = false;
      card->UpdateRoute(protected_route, ui::LightMode::Day);
      Check(!stop->IsEnabled(), "Protected route cannot be activated from the card");
      card->UpdateRoute({}, ui::LightMode::Night);
      Check(!details->IsEnabled() && !view->IsEnabled() && !stop->IsEnabled() &&
                !card->View().available,
            "Removed route immediately disables every action");
      Click(*stop);
      Check(activations_ == 0 && stops_ == 0, "Unavailable route cannot change navigation");
      Click(*details); Click(*view);
      Check(details_ == 0 && views_ == 0, "Unavailable route cannot dispatch actions");
      card->UpdateRoute(route_, ui::LightMode::Day);
      Click(*details);
      frame_->CallAfter([this] { AfterDetails(); });
    } catch (const std::exception &error) { Finish(error.what()); }
  }
  void AfterDetails() {
    try {
      Check(details_ == 1 && views_ == 0 && activations_ == 0 && stops_ == 0,
            "Details dispatches the exact selected route once");
      Check(dismissals_ == 1, "Action dismissal records the hover suppression point once");
      auto *card = Card();
      Click(*Find<ui::XNavButton>(card, "View on chart"));
      frame_->CallAfter([this] {
        try {
          Check(views_ == 1, "View on chart dispatches once after releasing the card");
          auto *escape_card = Card();
          wxKeyEvent key(wxEVT_CHAR_HOOK); key.m_keyCode = WXK_ESCAPE;
          Check(escape_card->FilterEvent(key) == wxEventFilter::Event_Processed &&
                    !escape_card->IsShown(), "Escape dismisses route context");
          auto *close_card = Card();
          auto *close = Find<ui::XNavIconButton>(close_card, "Close");
          Check(close, "Touch close is present"); Click(*close);
          Check(!close_card->IsShown(), "Touch close dismisses route context");
          Check(dismissals_ == 4, "Actions, Escape and touch close all notify dismissal once");
          Finish();
        } catch (const std::exception &error) { Finish(error.what()); }
      });
    } catch (const std::exception &error) { Finish(error.what()); }
  }
  void Finish(const char *error = nullptr) {
    if (error) { result_ = 1; std::cerr << error << '\n'; }
    else std::cout << "Route context card behavior passed\n";
    frame_->Destroy(); ExitMainLoop();
  }
  wxFrame *frame_ = nullptr;
  application::Route route_;
  int result_ = 0, details_ = 0, views_ = 0, dismissals_ = 0, activations_ = 0, stops_ = 0;
};
}
wxIMPLEMENT_APP(TestApp);
