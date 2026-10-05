#pragma once
#include "application/ChartInfo.h"
#include "ui/Drawer.h"
#include <wx/stattext.h>

namespace opennav::ui {
// Owned chart query copy only. No chart pointers, HTML widgets, file launching
// or navigation mutations are retained by this sheet.
class XNavChartInfoDrawer final : public XNavDrawer {
public:
  explicit XNavChartInfoDrawer(wxWindow &owner);
  void Open(application::ChartInfo info, const wxRect &workspace, LightMode mode);
  void UpdateLight(LightMode mode);
  const application::ChartInfo &View() const { return info_; }
private:
  struct TextRow {
    wxStaticText *window = nullptr;
    wxString original;
    bool secondary = false;
  };
  wxStaticText *Text(const std::string &text, int size, bool secondary = false);
  void Build();
  void Wrap();
  application::ChartInfo info_;
  std::vector<TextRow> text_;
  std::vector<XNavButton *> buttons_;
  bool wrapping_ = false;
};
} // namespace opennav::ui
