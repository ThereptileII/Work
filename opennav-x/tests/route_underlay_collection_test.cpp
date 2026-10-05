// Execute the production collector with deterministic projection inputs. These
// tests qualify collection decisions, not the stub projection or a GL driver.
#include "line_clip.h"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <wx/gdicmn.h>
#include <wx/geometry.h>
constexpr double PI = 3.141592653589793, WGS84_semimajor_axis_meters = 6378137,
                 mercator_k0 = .9996;
constexpr int PROJECTION_MERCATOR = 1, PROJECTION_EQUIRECTANGULAR = 2;
struct Leg {
  std::size_t index;
  std::array<double, 4> xy;
  bool start, end;
};
namespace opennav::integration {
struct ChartRouteUnderlay {
  std::vector<Leg> legs;
  void Add(std::size_t i, double a, double b, double c, double d, bool s = true,
           bool e = true) {
    legs.push_back({i, {a, b, c, d}, s, e});
  }
};
} // namespace opennav::integration
struct RoutePoint {
  double m_lat, m_lon;
  bool m_pos_on_screen = true;
};
struct wxRoutePointListNode {
  RoutePoint point;
  wxRoutePointListNode *next = nullptr;
  RoutePoint *GetData() { return &point; }
  auto GetNext() { return next; }
};
struct List {
  std::vector<wxRoutePointListNode> nodes;
  auto GetFirst() { return nodes.data(); }
};
struct Route {
  List *pRoutePointList;
  int GetnPoints() { return pRoutePointList->nodes.size(); }
  auto GetPoint(int i) { return &pRoutePointList->nodes[i - 1].point; }
};
struct LLBBox {
  double a = -80, b = 80, c = -180, d = 180;
  double GetMinLat() { return a; }
  double GetMaxLat() { return b; }
  double GetMinLon() { return c; }
  double GetMaxLon() { return d; }
};
struct ViewPort {
  LLBBox bbox;
  double view_scale_ppm = .00002, clon = 0, rotation = 0;
  int m_projection_type = 1;
  bool idl = false;
  auto GetBBox() { return bbox; }
  bool ContainsIDL() { return idl; }
};
struct ocpnDC {
  std::vector<std::array<double, 4>> lines;
  void DrawLine(double a, double b, double c, double d) {
    lines.push_back({a, b, c, d});
  }
  void GetSize(int *w, int *h) {
    *w = 900;
    *h = 570;
  }
};
struct ChartCanvas {
  void GetDoubleCanvasPointPix(double lat, double lon, wxPoint2DDouble *p) {
    p->m_x = lon * 2 + 450;
    p->m_y = lat * 2 + 285;
  }
};
namespace opennav::integration {
bool DrawChartRouteSegment(ocpnDC &, ChartCanvas &, double, double, double,
                           double, bool, bool) {
  return false;
}
} // namespace opennav::integration
class RouteGui {
public:
  Route &m_route;
  void DrawGLLines(ViewPort &, ocpnDC *, ChartCanvas *, bool,
                   opennav::integration::ChartRouteUnderlay * = nullptr);
  void RenderSegment(ocpnDC &, int, int, int, int, ViewPort &, bool, int,
                     ChartCanvas *, bool, bool, bool,
                     opennav::integration::ChartRouteUnderlay *, std::size_t);
};
#define OPENNAV_X
#define ocpnUSE_GL
#include "production-collection.h"
int checks = 0;
void Check(bool value, const char *message) {
  ++checks;
  if (!value)
    throw std::runtime_error(message);
}
int main() {
  try {
    ChartCanvas canvas;
    ViewPort vp;
    ocpnDC dc;
    for (int projection : {1, 2, 3})
      for (bool idl : {false, true})
        for (double rotation : {0., .7})
          for (double center : {-100., 0., 100.}) {
            vp.m_projection_type = projection;
            vp.idl = idl;
            vp.rotation = rotation;
            vp.clon = center;
            for (const auto &coordinates :
                 std::vector<std::vector<std::array<double, 2>>>{
                     {{0, 20}, {10, 30}, {20, 35}},
                     {{0, 170}, {10, -170}, {20, 175}},
                     {{85, 10}, {86, 20}, {0, 30}},
                     {{0, 200}, {0, 220}, {0, 30}},
                     {{0, 20}, {NAN, NAN}, {20, 35}},
                     {{0, 20}, {0, 20}, {20, 35}}}) {
              List list;
              for (auto p : coordinates)
                list.nodes.push_back({{p[0], p[1], true}, nullptr});
              for (size_t i = 1; i < list.nodes.size(); ++i)
                list.nodes[i - 1].next = &list.nodes[i];
              Route route{&list};
              RouteGui gui{route};
              opennav::integration::ChartRouteUnderlay collected;
              dc.lines.clear();
              gui.DrawGLLines(vp, &dc, &canvas, false, &collected);
              Check(dc.lines.empty(), "collection drew a line");
              for (auto &n : list.nodes)
                Check(n.point.m_pos_on_screen,
                      "collection changed route point state");
              gui.DrawGLLines(vp, &dc, &canvas, false);
              Check(collected.legs.size() == dc.lines.size(),
                    "collector rejected or invented a normal GL leg");
              for (size_t i = 0; i < dc.lines.size(); ++i)
                Check(collected.legs[i].xy == dc.lines[i],
                      "collector changed projected/wrapped GL coordinates");
            }
          }
    // The production RenderSegment prefix, including wxRect early rejection and
    // the pinned integer Cohen-Sutherland implementation, is executed directly.
    List list;
    Route route{&list};
    RouteGui gui{route};
    for (const auto &xy :
         std::vector<std::array<int, 4>>{{20, 80, 120, 80},
                                         {-20, 80, 120, 80},
                                         {20, 80, 1200, 80},
                                         {-50, -50, -20, -20},
                                         {-50, 250, 1000, 350},
                                         {450, -20, 450, 700}}) {
      opennav::integration::ChartRouteUnderlay collected;
      gui.RenderSegment(dc, xy[0], xy[1], xy[2], xy[3], vp, false, 0, &canvas,
                        false, false, false, &collected, 7);
      int a = xy[0], b = xy[1], c = xy[2], d = xy[3];
      auto visible = cohen_sutherland_line_clip_i(&a, &b, &c, &d, 0, 900, 0,
                                                  570) == Visible;
      Check(collected.legs.size() == (visible ? 1u : 0u),
            "software clipping decision changed");
      if (visible) {
        auto &leg = collected.legs[0];
        Check(leg.xy == std::array<double, 4>{double(a), double(b), double(c),
                                              double(d)},
              "software clipped coordinates changed");
        Check(leg.index == 7 && leg.start == (a == xy[0] && b == xy[1]) &&
                  leg.end == (c == xy[2] && d == xy[3]),
              "clip endpoint invented a join");
      }
    }
    std::cout << checks << " production collection checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
