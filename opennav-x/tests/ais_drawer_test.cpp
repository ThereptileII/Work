// Dedicated offline UI fixture. No OpenCPN, network or credential operations.
#include "ui/AisDrawer.h"
#include "ais/TargetCache.h"
#include <wx/app.h>
#include <wx/dcbuffer.h>
#include <wx/dcscreen.h>
#include <wx/dialog.h>
#include <wx/filename.h>
#include <wx/log.h>
#include <wx/timer.h>
#include <wx/textctrl.h>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif

using namespace opennav;
using namespace std::chrono_literals;
namespace {
constexpr int local_id = 265000001, online_id = 265000002;
const vessel::Time stamp{1000s};
template<class T> T *Find(wxWindow *root, const wxString &name) {
  if (root->GetName() == name)
    if (auto *match = dynamic_cast<T *>(root)) return match;
  for (auto *child : root->GetChildren())
    if (auto *match = Find<T>(child, name)) return match;
  return nullptr;
}
void Mouse(wxWindow &window, wxEventType type, int x = 90, int y = 30) {
  wxMouseEvent event(type);
  event.SetPosition({x, y}); event.SetEventObject(&window);
  window.GetEventHandler()->ProcessEvent(event);
}
vessel::Sample Sample(double n) {
  return {n, "TEST ONLY / OpenCPN model copy", stamp, vessel::Validity::Measured,
          {15s, 60s}};
}
vessel::AisState Local() {
  vessel::AisState s;
  s.available = true; s.observed_at = stamp;
  vessel::AisTarget t;
  t.mmsi = local_id; t.name = "Freja"; t.status = "Under way using engine";
  t.source = "TEST ONLY / OpenCPN model copy"; t.active = true;
  t.observed_at = stamp; t.upstream_alarm = true;
  t.latitude_deg = Sample(58.3); t.longitude_deg = Sample(16.8);
  t.sog_kn = Sample(11.2); t.cog_deg = Sample(220);
  t.heading_true_deg = Sample(220); t.range_nm = Sample(1.2);
  t.bearing_true_deg = Sample(71); t.cpa_nm = Sample(.4);
  t.tcpa_minutes = Sample(12); t.length_m = Sample(32); t.beam_m = Sample(8);
  for (auto *s : {&t.range_nm, &t.bearing_true_deg, &t.cpa_nm, &t.tcpa_minutes}) {
    s->validity = vessel::Validity::Estimated;
    s->freshness = {2s, 5s}; // Exact selected-own-position dependency contract.
  }
  t.destination = {"Arkosund", t.source, stamp, vessel::Validity::Measured,
                   {1h, 6h}};
  s.targets.push_back(t);
  return s;
}
class DrawerTest final : public wxApp {
 public:
  bool OnInit() override {
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString &file, int line, const wxString &,
                          const wxString &condition, const wxString &message) {
      std::cerr << "WX ASSERT " << file << ':' << line << ' ' << condition
                << ' ' << message << std::endl;
      std::abort();
    });
    if (argc != 2) { std::cerr << "Required: isolated output directory\n"; return false; }
    output_ = argv[1];
    if (!wxFileName::Mkdir(output_, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL)) {
      std::cerr << "Output must be a new directory\n"; return false;
    }
    wxInitAllImageHandlers();
    frame_ = new wxFrame(nullptr, wxID_ANY, "TEST ONLY - AIS component", {0, 0},
                         {1280, 800}, wxBORDER_NONE);
    frame_->SetClientSize(1280, 800);
    frame_->SetBackgroundStyle(wxBG_STYLE_PAINT);
    frame_->Bind(wxEVT_PAINT, [this](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(frame_);
      ui::XNavPainter p(*frame_, dc, light_);
      dc.SetBackground(wxBrush(ui::Colour(p.c.background))); dc.Clear();
      p.Text("OFFLINE COMPONENT TEST", 80, 80, 22, p.c.attention);
      p.Text("Synthetic fixtures / no chart / no network / no equipment", 80, 126,
             12, p.c.secondary);
    });
    application::OnlineAisActions actions;
    actions.read = [this](vessel::Time) { return online_; };
    actions.store_key = [this](const ais::Secret &key) {
      ++stores_;
      // Only an unmistakable dummy reaches this test boundary. Never read
      // Credential Manager, environment credentials or a live service here.
      Check(key.View() == "TEST-ONLY-NOT-A-SERVICE-KEY", "explicit dummy reaches owned secret boundary");
      if (fail_store_)
        return application::CommandResult{false, "Secure key storage unavailable", {}};
      online_.credential_present = true;
      return application::CommandResult{true, "Key stored securely", {}};
    };
    actions.remove_key = [this] {
      ++removes_;
      online_.credential_present = online_.enabled = false;
      return application::CommandResult{true, "Key removed", {}};
    };
    drawer_ = new ui::XNavAisDrawer(*frame_, std::move(actions), [this](int id) { shown_.push_back(id); });
    drawer_->on_select = [this](int id) { selected_.push_back(id); };
    // Feed the real owned cache and aggregator, never shortcut UI state.
    ais::PositionReport p;
    p.mmsi = online_id; p.latitude = 58.4; p.longitude = 16.9;
    p.sog = 4.8; p.cog = 55; p.observed_at = stamp;
    Check(cache_.Observe(p, stamp), "online position fixture validated");
    ais::StaticReport s;
    s.mmsi = online_id; s.name = "S/Y Liv"; s.callsign = "TEST123";
    s.destination = "Tyrislot"; s.length_m = 11; s.beam_m = 3.6;
    s.observed_at = stamp;
    Check(cache_.Observe(s, stamp), "online static fixture validated");
    frame_->Show(); frame_->Raise();
    Add([this] { Feed(stamp + 2s); drawer_->Present(Workspace()); });
    // Match the application's regular layout tick after the owned window maps
    // (bare X11 has no window manager to honour initial placement hints).
    Add([this] { drawer_->Present(Workspace()); });
    Add([this] {
      const auto bounds = drawer_->GetScreenRect();
      std::cout << "DRAWER " << bounds.x << ' ' << bounds.y << ' '
                << bounds.width << ' ' << bounds.height << std::endl;
      Check(drawer_->GetScreenRect() == wxRect(682, 80, 398, 674), "canonical drawer bounds");
      Capture("traffic-day");
      Check(drawer_->PageTitle() == "AIS targets", "list diagnostic identifies actual view");
      Check(Require<ui::XNavButton>("Sort AIS by closest approach")->IsEnabled(),
            "fresh upstream estimated CPA remains usable");
      Check(Require<ui::XNavButton>("Sort AIS by range")->IsEnabled(),
            "fresh upstream estimated range remains usable");
      auto *list = Require<ui::XNavListView>("Vessel traffic list");
      Mouse(*list, wxEVT_LEFT_DOWN); Mouse(*list, wxEVT_LEFT_UP);
    });
    Add([this] {
      Check(!selected_.empty() && selected_.back() == local_id, "real list click selects copied MMSI");
      Check(drawer_->PageTitle() == "AIS target", "target diagnostic identifies actual view");
      Check(Require<ui::XNavButton>("Show on chart")->IsEnabled(), "current onboard chart action enabled");
      Capture("ais-target-day");
      Command("Show on chart");
    });
    Add([this] {
      Check(shown_ == std::vector<int>{local_id}, "manual chart action emits selected identity once");
      light_ = ui::LightMode::Dusk; Feed(stamp + 2s);
    });
    Add([this] { Capture("ais-target-dusk"); light_ = ui::LightMode::Night; Feed(stamp + 2s); });
    Add([this] { Capture("ais-target-night"); Command("Back"); });
    Add([this] {
      Check(selected_.back() == 0, "back clears target selection"); Capture("traffic-night");
      light_ = ui::LightMode::Day; Feed(stamp + 2s);
      auto *list = Require<ui::XNavListView>("Vessel traffic list");
      Mouse(*list, wxEVT_LEFT_DOWN, 90, 100); Mouse(*list, wxEVT_LEFT_UP, 90, 100);
    });
    Add([this] {
      Check(selected_.back() == online_id, "online row selects its own identity");
      Check(!display_.targets.back().cpa_nm.value && !display_.targets.back().tcpa_minutes.value,
            "online metrics remain unavailable through real aggregator");
      Capture("online-target-day"); Command("Show on chart");
    });
    Add([this] {
      Check(shown_ == std::vector<int>({local_id, online_id}), "manual online chart action emits identity");
      Feed(stamp + 61s);
    });
    Add([this] {
      Check(!Require<ui::XNavButton>("Show on chart")->IsEnabled(), "stale target chart action disabled");
      Capture("online-stale-day"); Command("Show on chart");
    });
    Add([this] {
      Check(shown_.size() == 2, "even queued stale action cannot emit chart selection");
      Feed(stamp + 121s);
    });
    Add([this] {
      Check(display_.targets.back().lost, "lost state propagated");
      Check(!Require<ui::XNavButton>("Show on chart")->IsEnabled(), "lost target chart action disabled");
      Capture("online-lost-day"); Feed(stamp + 601s);
    });
    Add([this] {
      Check(!Require<ui::XNavButton>("Show on chart")->IsEnabled(), "removed target unavailable");
      Capture("target-expired-day");
      drawer_->List(); Feed(stamp + 2s);
    });
    Add([this] {
      // Sorting/update during a touch must not select whichever new target
      // happens to occupy the pressed row when released.
      const auto count = selected_.size();
      auto *list = Require<ui::XNavListView>("Vessel traffic list");
      Mouse(*list, wxEVT_LEFT_DOWN);
      auto empty = Local(); empty.targets.clear();
      auto changed = ais::Aggregate(empty, online_.feed).display;
      drawer_->Update(changed, online_, stamp + 2s, light_);
      Mouse(*list, wxEVT_LEFT_UP);
      deferred_count_ = count;
    });
    Add([this] {
      Check(selected_.size() == deferred_count_, "mid-press row replacement cancels selection");
      Feed(stamp + 6s);
      Check(!Require<ui::XNavButton>("Sort AIS by closest approach")->IsEnabled(),
            "expired own-position dependency disables estimated CPA sort");
      Check(!Require<ui::XNavButton>("Sort AIS by range")->IsEnabled(),
            "expired own-position dependency disables estimated range sort");
      drawer_->Target(local_id);
      Check(Require<ui::XNavButton>("Show on chart")->IsEnabled(),
            "target remains current when only relative estimates expire");
      auto unavailable = display_;
      unavailable.available = false;
      drawer_->Update(unavailable, online_, stamp + 6s, light_);
      Check(!Require<ui::XNavButton>("Show on chart")->IsEnabled(),
            "unavailable container cannot expose retained chart action");
      Feed(stamp + 2s); drawer_->Target(online_id);
      wxKeyEvent key(wxEVT_CHAR_HOOK); key.m_keyCode = WXK_ESCAPE;
      Check(drawer_->FilterEvent(key) == wxEventFilter::Event_Processed, "Escape handled by top drawer");
    });
    Add([this] {
      Check(selected_.back() == 0, "Escape target returns to list");
      wxKeyEvent key(wxEVT_CHAR_HOOK); key.m_keyCode = WXK_ESCAPE;
      drawer_->FilterEvent(key);
      Check(!drawer_->IsShown(), "Escape list dismisses drawer");
      online_.enabled = false;
      online_.feed.health.connection = ais::Connection::Disabled;
      online_.credential_present = false;
      online_.credential_writable = true; // Test capability, even on Linux.
      drawer_->Update(display_, online_, stamp + 2s, light_);
      drawer_->Present(Workspace());
      Command("Online AIS settings");
    });
    Add([this] {
      Check(drawer_->PageTitle() == "Online AIS settings", "user can reach own-key settings");
      Check(!Require<ui::XNavButton>("Remove key")->IsEnabled(), "absent key cannot be removed");
      Command("Set AISStream key");
    });
    Add([this] {
      auto *entry = KeyEntry();
      Check(entry->GetValue().empty(), "entry starts empty without reading a stored secret");
      Check(!Find<ui::XNavButton>(entry->GetParent(), "Save key")->IsEnabled(), "empty save is disabled");
      entry->SetValue("TEST-ONLY-NOT-A-SERVICE-KEY");
    });
    Add([this] { Capture("ais-key-entry-day"); ModalCommand("AISStream key", "Cancel"); });
    Add([this] {
      Check(stores_ == 0 && !online_.credential_present, "cancel never stores a key");
      Command("Set AISStream key");
    });
    Add([this] {
      auto *entry = KeyEntry();
      Check(entry->GetValue().empty(), "cancelled input is not retained on reopening");
      entry->SetValue("invalid key with spaces");
      Check(!Find<ui::XNavButton>(entry->GetParent(), "Save key")->IsEnabled(), "invalid input cannot enable Save");
      // Even an already-queued button event is validated at the secret boundary.
      ModalCommand("AISStream key", "Save key", false);
    });
    Add([this] {
      Check(stores_ == 0, "invalid queued save cannot reach storage");
      Command("Set AISStream key");
    });
    Add([this] {
      KeyEntry()->SetValue("TEST-ONLY-NOT-A-SERVICE-KEY");
      ModalCommand("AISStream key", "Save key");
    });
    Add([this] {
      Check(stores_ == 1 && online_.credential_present && !online_.enabled,
            "successful save records key presence without enabling traffic");
      Check(Require<ui::XNavButton>("Remove key")->IsEnabled(), "saved key removal is available");
      Capture("ais-key-stored-day");
      fail_store_ = true;
      Command("Set AISStream key");
    });
    Add([this] {
      Check(KeyEntry()->GetValue().empty(), "replacement never prefills the existing key");
      KeyEntry()->SetValue("TEST-ONLY-NOT-A-SERVICE-KEY");
      ModalCommand("AISStream key", "Save key");
    });
    Add([this] {
      Check(stores_ == 2 && online_.credential_present && !online_.enabled,
            "failed replacement preserves prior status without enabling traffic");
      Check(wxWindow::FindWindowByLabel("Secure key storage unavailable", drawer_) != nullptr,
            "storage failure is visible without secret contents");
      light_ = ui::LightMode::Night;
      drawer_->Update(display_, online_, stamp + 2s, light_);
      frame_->Refresh(false);
      Command("Set AISStream key");
    });
    Add([this] { KeyEntry()->SetValue("TEST-ONLY-NOT-A-SERVICE-KEY"); });
    Add([this] {
      Capture("ais-key-entry-night");
      wxKeyEvent key(wxEVT_CHAR_HOOK); key.m_keyCode = WXK_ESCAPE;
      KeyEntry()->GetParent()->GetEventHandler()->ProcessEvent(key);
    });
    Add([this] {
      Check(stores_ == 2 && online_.credential_present, "Escape preserves the stored key");
      Check(drawer_->PageTitle() == "Online AIS settings", "modal Escape cannot navigate the underlying drawer");
      Command("Remove key");
    });
    Add([this] { ModalCommand("Remove AISStream key", "Cancel"); });
    Add([this] {
      Check(removes_ == 0 && online_.credential_present, "removal cancellation preserves key");
      Command("Remove key");
    });
    Add([this] { ModalCommand("Remove AISStream key", "Remove key"); });
    Add([this] {
      Check(removes_ == 1 && !online_.credential_present && !online_.enabled,
            "confirmed removal clears presence and disables online traffic");
      Check(!Require<ui::XNavButton>("Remove key")->IsEnabled(), "removal updates the visible capability");
      Finish();
    });
    timer_.SetOwner(this);
    Bind(wxEVT_TIMER, [this](wxTimerEvent &) {
      try { if (step_ < steps_.size()) steps_[step_++](); }
      catch (const std::exception &e) {
        failure_ = e.what(); result_ = 1;
        timer_.Stop();
        // Unwind stack-owned modal loops before destroying their parent.
        for (auto *window : wxTopLevelWindows)
          if (auto *dialog = dynamic_cast<wxDialog *>(window); dialog && dialog->IsModal())
            dialog->EndModal(wxID_CANCEL);
        CallAfter([this] { Finish(); });
      }
    });
    timer_.Start(400);
    return true;
  }
  int OnRun() override { wxApp::OnRun(); return result_; }
 private:
  wxRect Workspace() const { return {80, 68, 1014, 698}; }
  void Check(bool good, const char *name) {
    if (!good) throw std::runtime_error(name);
    ++checks_; std::cout << "PASS " << name << '\n';
  }
  template<class T> T *Require(const wxString &name) {
    auto *window = Find<T>(drawer_, name);
    Check(window != nullptr, ("control exists: " + name).utf8_str());
    return window;
  }
  void Command(const wxString &name) {
    auto *button = Require<ui::XNavButton>(name);
    wxCommandEvent event(wxEVT_BUTTON, button->GetId());
    event.SetEventObject(button); button->GetEventHandler()->ProcessEvent(event);
  }
  wxTextCtrl *KeyEntry() {
    auto *dialog = dynamic_cast<wxDialog *>(wxWindow::FindWindowByName("AISStream key"));
    Check(dialog && dialog->IsModal(), "key entry is an explicit user modal");
    Check(frame_->GetScreenRect().Contains(dialog->GetScreenRect()), "key input remains inside the application");
    auto *entry = Find<wxTextCtrl>(dialog, "Protected AISStream key");
    Check(entry && entry->HasFlag(wxTE_PASSWORD), "own-key field is password masked");
    Check(entry->GetClientSize().y >= entry->FromDIP(48), "key input has touch-sized height");
    return entry;
  }
  void ModalCommand(const wxString &name, const wxString &action, bool enabled = true) {
    auto *dialog = dynamic_cast<wxDialog *>(wxWindow::FindWindowByName(name));
    Check(dialog && dialog->IsModal(), "expected user modal is open");
    auto *button = Find<ui::XNavButton>(dialog, action);
    Check(button && button->IsEnabled() == enabled, "modal action has expected availability");
    wxCommandEvent event(wxEVT_BUTTON, button->GetId());
    event.SetEventObject(button); button->GetEventHandler()->ProcessEvent(event);
  }
  void Add(std::function<void()> step) { steps_.push_back(std::move(step)); }
  void Feed(vessel::Time now) {
    online_.enabled = true;
    online_.feed.targets = cache_.Read(now);
    online_.feed.health.connection = ais::Connection::Connected;
    online_.feed.health.subscription_confirmed = true;
    display_ = ais::Aggregate(Local(), online_.feed).display;
    drawer_->Update(display_, online_, now, light_);
    frame_->Refresh(false);
  }
  void Capture(const wxString &name) {
    Check(frame_->GetClientSize() == wxSize(1280, 800), "1280x800 actual test workspace");
    const auto origin = frame_->ClientToScreen({0, 0});
    const auto file = output_ + "/" + name + ".png";
#ifdef __WXGTK__
    // wxScreenDC::Blit on GTK can succeed with black pixels; read the actual
    // X11 root window through GDK, never repaint controls into a fake screenshot.
    auto *pixels = gdk_pixbuf_get_from_window(gdk_get_default_root_window(),
                                             origin.x, origin.y, 1280, 800);
    Check(pixels != nullptr, "actual screen pixels copied");
    const bool saved = gdk_pixbuf_save(pixels, file.utf8_str(), "png", nullptr, nullptr);
    g_object_unref(pixels);
    Check(saved, "PNG saved");
#else
    wxScreenDC screen; wxBitmap bitmap(1280, 800, 24); wxMemoryDC copy(bitmap);
    Check(copy.Blit(0, 0, 1280, 800, &screen, origin.x, origin.y), "actual screen pixels copied");
    copy.SelectObject(wxNullBitmap);
    Check(bitmap.SaveFile(file, wxBITMAP_TYPE_PNG), "PNG saved");
#endif
    captures_.push_back(name.ToStdString());
  }
  void Finish() {
    timer_.Stop();
    std::ofstream report((output_ + "/result.json").ToStdString());
    report << "{\"scope\":\"offline native component only; no chart or live traffic\","
           << "\"checks\":" << checks_ << ",\"passed\":" << (result_ ? "false" : "true")
           << ",\"captures\":[";
    for (std::size_t i = 0; i < captures_.size(); ++i)
      report << (i ? "," : "") << '"' << captures_[i] << '"';
    report << "]}\n"; report.close();
    if (!failure_.empty()) std::cerr << "FAIL " << failure_ << '\n';
    drawer_->Destroy(); frame_->Destroy(); ExitMainLoop();
  }
  wxFrame *frame_ = nullptr;
  ui::XNavAisDrawer *drawer_ = nullptr;
  ui::LightMode light_ = ui::LightMode::Day;
  wxTimer timer_; wxString output_;
  ais::TargetCache cache_;
  vessel::AisState display_;
  application::OnlineAisState online_;
  std::vector<std::function<void()>> steps_;
  std::vector<int> selected_, shown_;
  std::vector<std::string> captures_;
  std::size_t step_ = 0, deferred_count_ = 0;
  int checks_ = 0, result_ = 0;
  int stores_ = 0, removes_ = 0;
  bool fail_store_ = false;
  std::string failure_;
};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(DrawerTest);
int main(int argc, char **argv) { return wxEntry(argc, argv); }
