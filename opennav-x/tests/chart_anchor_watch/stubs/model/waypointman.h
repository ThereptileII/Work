#pragma once
#include <wx/bitmap.h>
#include <vector>
struct MarkIcon {
  wxString icon_name="anchor";
  wxBitmap *piconBitmap=nullptr;
  bool skagerPinnedAnchor=false;
};
struct FixtureIconArray {
  std::vector<MarkIcon *> icons;
  unsigned GetCount() const { return static_cast<unsigned>(icons.size()); }
  void *Item(unsigned index) const { return icons.at(index); }
};
class WayPointman {
public:
  FixtureIconArray icons;
  FixtureIconArray *m_pIconArray=&icons;
};
extern WayPointman *pWayPointMan;
