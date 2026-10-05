#pragma once
#include <wx/dcmemory.h>
#include <vector>
// Real wx raster painting plus captured commands. This adapter replaces only
// the upstream GL/DC dispatch, never the production anchor artwork or rings.
class ocpnDC {
public:
  struct Circle { int x, y; double radius; wxPen pen; };
  ocpnDC() : bitmap(512, 512), dc(bitmap) { dc.SetBackground(*wxWHITE_BRUSH); dc.Clear(); }
  void SetPen(const wxPen &pen) { dc.SetPen(pen); }
  void SetBrush(const wxBrush &brush) { dc.SetBrush(brush); }
  void StrokeCircle(int x, int y, double radius) {
    circles.push_back({x,y,radius,dc.GetPen()}); dc.DrawCircle(x,y,radius);
  }
  void DrawBitmap(const wxBitmap &source, int x, int y, bool mask) {
    ++marks; last_mark=source.ConvertToImage(); dc.DrawBitmap(source,x,y,mask);
  }
  void CalcBoundingBox(int x, int y) { bounds.push_back({x,y}); dc.CalcBoundingBox(x,y); }
  wxBitmap bitmap;
  wxMemoryDC dc;
  wxImage last_mark;
  std::vector<Circle> circles;
  std::vector<wxPoint> bounds;
  int marks=0;
};
