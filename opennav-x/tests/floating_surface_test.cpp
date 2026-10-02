// Offline native-window lifecycle regression; no chart model or marine input.
#include "ui/FloatingSurface.h"
#include <wx/app.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/timer.h>
#include <wx/uiaction.h>
#include <iostream>
#include <stdexcept>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif
namespace {
class TestApp final : public wxApp {
 public:
  bool OnInit() override {
    owner_ = new wxFrame(nullptr, wxID_ANY, "TEST floating controls", {0,0},
                         {896,560}, wxBORDER_NONE);
    canvas_ = new wxPanel(owner_, wxID_ANY);
    canvas_->Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent &) { ++canvas_clicks_; });
    auto *layout = new wxBoxSizer(wxVERTICAL);
    layout->Add(canvas_, 1, wxEXPAND); owner_->SetSizer(layout);
    surface_ = new opennav::ui::XNavFloatingSurface(*owner_, "TEST overlay");
    auto *button = new wxButton(surface_, wxID_ANY, "+", wxDefaultPosition, {44,44});
    button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { ++activations_; });
    auto *tools = new wxBoxSizer(wxHORIZONTAL); tools->Add(button);
    surface_->SetSizerAndFit(tools);
    owner_->Show();
    // Match startup: present at the compact size, disable the owner and hide
    // before the queued native map event is handled, then resize and reenable.
    surface_->Present({626,343});
    owner_->Enable(false); surface_->Hide();
    owner_->SetClientSize(1280,800); owner_->Enable(true);
    timer_.SetOwner(this); Bind(wxEVT_TIMER, &TestApp::Step, this);
    timer_.StartOnce(150); return true;
  }
  int OnRun() override { wxApp::OnRun(); return failed_ ? 1 : 0; }
 private:
  void Check(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
    ++checks_;
  }
  void CheckNative() {
    Check(surface_->IsShownOnScreen(), "overlay logically shown");
#ifdef __WXGTK__
    auto *widget = surface_->GetHandle();
    Check(gtk_widget_get_visible(widget) && gtk_widget_get_mapped(widget),
          "explicitly hidden overlay remaps after owner is enabled");
    int x=0,y=0; gdk_window_get_origin(gtk_widget_get_window(widget), &x, &y);
    std::cout << "native origin=" << x << "," << y << "; desired=" << desired_.x << "," << desired_.y << std::endl;
    Check(wxPoint(x,y) == desired_, "native overlay follows resized chart target");
#endif
    Check(surface_->GetScreenPosition() == desired_, "reported position matches target");
  }
  void Step(wxTimerEvent &) {
    try {
      switch (step_++) {
        case 0:
#ifdef __WXGTK__
          std::cout << "after queued map: requested hidden, wx shown=" << surface_->IsShown()
                    << ", GTK visible=" << gtk_widget_get_visible(surface_->GetHandle()) << '\n';
#endif
          canvas_->SetFocus(); surface_->Present(desired_); break;
        case 1: {
          CheckNative(); Check(wxWindow::FindFocus() == canvas_, "restoring overlay does not steal focus");
          surface_->Present(desired_);
          Check(wxWindow::FindFocus() == canvas_, "ordinary placement preserves focus");
          wxUIActionSimulator pointer;
          Check(pointer.MouseMove(desired_ + wxPoint(22,22)) && pointer.MouseClick(),
                "send actual native click at restored button");
          break;
        }
        case 2:
          Check(activations_ == 1 && canvas_clicks_ == 0,
                "restored button receives exactly one activation instead of underlying canvas");
          owner_->Enable(false); surface_->Hide(); break;
        case 3:
          Check(!surface_->IsShown(), "explicit hide remains effective while owner disabled");
#ifdef __WXGTK__
          Check(!gtk_widget_get_visible(surface_->GetHandle()), "native overlay remains hidden");
#endif
          owner_->Enable(true);
          { wxUIActionSimulator pointer;
            Check(pointer.MouseMove({30,30}) && pointer.MouseClick(), "return native focus to owner"); }
          canvas_->SetFocus(); break;
        case 4:
          Check(wxWindow::FindFocus() == canvas_, "owner has focus before ordinary re-show");
          surface_->Present(desired_); break;
        case 5:
          CheckNative(); Check(wxWindow::FindFocus() == canvas_, "normal hide/show also preserves focus");
          std::cout << checks_ << " floating-surface lifecycle checks passed\n";
          Finish(); return;
      }
      timer_.StartOnce(150);
    } catch (const std::exception &error) {
      std::cerr << "FAILED: " << error.what() << '\n'; failed_=true; Finish();
    }
  }
  void Finish() { timer_.Stop(); surface_->Destroy(); owner_->Destroy(); ExitMainLoop(); }
  wxFrame *owner_=nullptr;
  wxPanel *canvas_=nullptr;
  opennav::ui::XNavFloatingSurface *surface_=nullptr;
  wxTimer timer_; wxPoint desired_{980,549};
  int step_=0,checks_=0,activations_=0,canvas_clicks_=0; bool failed_=false;
};
}
// CMake builds a console test executable on Windows. Keep wxEntry responsible
// for OnInit, the event loop, OnExit and cleanup, while returning its test code.
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc, char **argv) { return wxEntry(argc, argv); }
