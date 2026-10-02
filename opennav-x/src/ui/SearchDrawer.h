#pragma once
#include "application/NavigationObjects.h"
#include "ui/Drawer.h"
#include <wx/textctrl.h>

namespace opennav::ui {
// Presentation values only. OpenCPN remains the navigation-object owner.
struct SearchMatch {
  std::string id;
  bool route = false;
  wxString name, detail;
};
struct SearchMatches {
  std::vector<SearchMatch> rows;
  bool limited = false;
};
SearchMatches FindNavigationObjects(const application::Catalog &, const wxString &query);
bool HasUniqueSearchObject(const application::Catalog &, const SearchMatch &);

class XNavSearchDrawer final : public XNavDrawer {
public:
  XNavSearchDrawer(wxWindow &owner, application::NavigationActions actions);
  void Open(const wxRect &workspace, LightMode mode);
  void Update(LightMode mode);
  std::function<void(const std::string &, bool)> on_select;

private:
  void RefreshResults();
  void Select(SearchMatch match, std::uint64_t generation);
  void PaintNote(wxPaintEvent &);
  application::NavigationActions actions_;
  application::Catalog catalog_; // Owned, per-open snapshot; never authoritative.
  wxTextCtrl *query_ = nullptr;
  wxPanel *input_frame_ = nullptr, *results_ = nullptr, *note_ = nullptr;
  wxBoxSizer *rows_ = nullptr;
  wxString notice_;
  std::uint64_t generation_ = 0;
  bool themed_ = false;
};
} // namespace opennav::ui
