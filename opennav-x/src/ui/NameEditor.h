#pragma once
#include "application/NavigationNaming.h"
#include "ui/Controls.h"
#include "ui/Sheet.h"
#include <wx/textctrl.h>

namespace opennav::ui {
// Inline owned draft. Text changes and Cancel never write navigation storage.
class XNavNameEditor final : public wxPanel {
 public:
  using Save = std::function<application::CommandResult(const std::string &)>;
  XNavNameEditor(wxWindow &parent, const wxString &label, std::string initial,
                 LightMode light, int scale, bool editable, Save save,
                 std::function<void(application::CommandResult)> result);
  bool DraftChanged() const { return draft_.Changed(); }
  void Present(LightMode light, bool live);
  // Preserve a dirty field across background object/theme refresh. Commands
  // still validate the captured revision; replay disables editing immediately.
  static bool PreserveDrafts(wxWindow &owner, LightMode light, bool live);
 private:
  void SaveDraft();
  void CancelDraft();
  void Buttons();
  void PaintFrame();
  application::NavigationNameDraft draft_;
  Save save_;
  std::function<void(application::CommandResult)> result_;
  // The prototype's input is a bordered, rounded surface; wxTextCtrl cannot
  // draw one, so a painted frame hosts the borderless control (SCRUM-347).
  wxPanel *frame_ = nullptr;
  wxTextCtrl *input_ = nullptr;
  XNavButton *save_button_ = nullptr, *cancel_button_ = nullptr;
  LightMode light_ = LightMode::Night;
  bool editable_ = false, live_ = true, focused_ = false;
};
// Same themed native sheet used for description editing, with name validation
// and draft retention before any real storage callback is allowed.
std::optional<std::vector<std::string>> EditNavigationNameSheet(
    wxWindow &parent, LightMode light, const wxString &title,
    const wxString &detail, const std::string &name, const std::string &description,
    const wxString &accept = "Save", int scale = -1);
} // namespace opennav::ui
