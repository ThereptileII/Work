#pragma once
#include "ui/Controls.h"
#include <functional>
#include <vector>

namespace opennav::ui {
struct XNavListRowData {
  std::string identity;
  wxString title, subtitle, metric, detail;
  bool attention = false, stale = false;
};
// Bounded owner-drawn, virtual list: only visible rows are painted, even when
// a dense AIS view contains 2,000 targets. Mouse/touch selection binds to the
// identity at press, not a row index which may change during a sensor update.
class XNavListView final : public wxControl {
public:
  explicit XNavListView(wxWindow *parent);
  void Update(std::vector<XNavListRowData> rows, LightMode mode);
  std::function<void(const std::string &)> on_select;

private:
  void Paint(wxPaintEvent &);
  void ScrollPixels(int pixels);
  int RowAt(wxPoint position) const;
  std::vector<XNavListRowData> rows_;
  std::string pressed_;
  int offset_ = 0, hover_ = -1;
  LightMode light_ = LightMode::Day;
};
} // namespace opennav::ui
