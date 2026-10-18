#pragma once
#include "application/NavigationObjects.h"
#include "application/Settings.h"
#include "ui/ChoiceField.h"
#include "ui/Drawer.h"
#include <array>

namespace opennav::ui {
// Owned settings copies only. The persistence callback must recheck the live
// application mode; queued edits also check this drawer's current editability.
class XNavRailPersonaliseDrawer final : public XNavDrawer {
 public:
  using Read = std::function<application::Settings()>;
  using Save = std::function<application::CommandResult(const application::Settings &)>;
  XNavRailPersonaliseDrawer(wxWindow &owner, Read read, Save save);
  void Open(const wxRect &workspace, LightMode mode, bool editable);
  void Update(LightMode mode, bool editable);
  void SetInterfaceScale(int percent);

 private:
  void Build();
  void Choose(std::size_t position, const std::string &key);
  void ChoosePreset(std::size_t index);
  void SavePreset();
  void DeletePreset();
  void Commit(const std::function<void(application::Settings &)> &change);
  void Queue(std::function<void()> action);
  wxPanel *Copy(const wxString &text, int height, int size = 12);
  XNavButton *Action(const wxString &label, std::function<void()> action);
  Read read_;
  Save save_;
  application::Settings observed_;
  std::array<XNavChoiceField *, 4> positions_{};
  std::vector<XNavButton *> buttons_;
  std::vector<wxPanel *> copy_;
  XNavChoiceField *presets_ = nullptr;
  XNavButton *delete_ = nullptr;
  std::string selected_preset_, draft_name_;
  wxString feedback_;
  bool editable_ = false;
  unsigned generation_ = 0;
};
} // namespace opennav::ui
