#pragma once
#include <wx/string.h>
#include <wx/bitmap.h>
class RoutePoint {
public:
  const wxString &GetIconName() const { return icon; }
  const wxString &GetName() const { return name; }
  bool IsDragHandleEnabled() const { return dragging; }
  wxString icon = "anchor", name = "100";
  wxBitmap *m_pbmIcon=nullptr;
  double m_lat = 0, m_lon = 0, radius = 0;
  bool m_bIsActive = false, m_bBlink = false, m_bRPIsBeingEdited = false, dragging = false;
};
