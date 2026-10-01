#include "ui/Sheet.h"

#include <wx/app.h>
#include <wx/dialog.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/utils.h>

#include <cstdlib>
#include <iostream>
#include <utility>

class SheetSizeTestApp final : public wxApp {
 public:
  bool OnInit() override { return true; }
};
wxIMPLEMENT_APP_NO_MAIN(SheetSizeTestApp);

namespace {
using opennav::ui::EditSheet;
using opennav::ui::LightMode;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "sheet_field_size_test: " << message << '\n';
    std::exit(1);
  }
}

bool CancelActiveModal() {
  for (auto node = wxTopLevelWindows.GetFirst(); node; node = node->GetNext()) {
    if (auto *dialog = wxDynamicCast(node->GetData(), wxDialog);
        dialog && dialog->IsModal()) {
      dialog->EndModal(wxID_CANCEL);
      return true;
    }
  }
  return false;
}

class ModalFieldProbe final : public wxEvtHandler {
 public:
  explicit ModalFieldProbe(wxFrame &frame) : frame_(frame), timer_(this) {
    Bind(wxEVT_TIMER, [this](wxTimerEvent &) { Inspect(); });
  }

  int ObserveFieldHeight() {
    attempts_ = 0;
    found_ = false;
    timer_.StartOnce(20);
    (void)EditSheet(frame_, LightMode::Day, "Create waypoint", "Name the mark",
                    {{"Name", "Touch target", 128}}, "Save", scale_);
    Require(found_, "real EditSheet exposes its named waypoint input to the probe");
    return height_;
  }

  void SetScale(int scale) { scale_ = scale; }
  int MinimumHeight() const { return minimum_height_; }

 private:
  void Inspect() {
    auto *field = wxWindow::FindWindowByName("Name", &frame_);
    if (!field) {
      if (++attempts_ < 50) {
        timer_.StartOnce(20);
        return;
      }
      std::cerr << "sheet_field_size_test: waypoint input was not found before timeout\n";
      if (!CancelActiveModal()) {
        std::cerr << "sheet_field_size_test: no active modal to close; failing process\n";
        std::exit(2);
      }
      return;
    }
    found_ = true;
    height_ = field->GetSize().y;
    minimum_height_ = field->GetMinSize().y;
    if (auto *dialog = wxDynamicCast(wxGetTopLevelParent(field), wxDialog))
      dialog->EndModal(wxID_CANCEL);
  }

  wxFrame &frame_;
  wxTimer timer_;
  int scale_ = 100;
  int attempts_ = 0;
  int height_ = -1;
  int minimum_height_ = -1;
  bool found_ = false;
};
}  // namespace

int main(int argc, char **argv) {
  if (!wxEntryStart(argc, argv) || !wxTheApp->CallOnInit()) return 2;
  auto *frame = new wxFrame(nullptr, wxID_ANY, "Sheet field size test");
  frame->SetSize(800, 600);
  frame->Show();
  wxTheApp->ProcessPendingEvents();

  ModalFieldProbe probe(*frame);
  for (const auto &[scale, logical_height] :
       {std::pair{100, 48}, std::pair{125, 52}, std::pair{150, 56}}) {
    probe.SetScale(scale);
    const int actual_height = probe.ObserveFieldHeight();
    const int expected = frame->FromDIP(logical_height);
    std::cout << "EditSheet scale=" << scale << "% expectedMin=" << expected
              << " allocated=" << actual_height
              << " wxMin=" << probe.MinimumHeight() << '\n';
    Require(actual_height >= expected,
            "real EditSheet text input meets its scale-specific touch target");
    Require(probe.MinimumHeight() >= expected,
            "real EditSheet text input reserves its scale-specific minimum height");
  }

  frame->Destroy();
  wxTheApp->ProcessPendingEvents();
  wxTheApp->OnExit();
  wxEntryCleanup();
  return 0;
}
