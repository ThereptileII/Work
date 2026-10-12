#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>
#include "vector2D.h"

// Inert wx list and route boundaries. Selection tests below call the actual
// pinned production Select methods and projection/vector routines.
struct RoutePoint {
  bool m_bIsActive = false;
  bool IsVisibleSelectable(double) { return true; }
};
struct Route {
  bool m_bRtIsActive = false;
  bool IsActive() { return m_bRtIsActive; }
};
struct Track {};
struct TrackPoint {};
Route *FindRouteContainingWaypoint(RoutePoint *) { return nullptr; }
struct SelectItem {
  float m_slat = 0, m_slon = 0, m_slat2 = 0, m_slon2 = 0;
  int m_seltype = 0;
  const void *m_pData1 = nullptr;
  void *m_pData3 = nullptr;
};
struct wxSelectableItemListNode {
  SelectItem *data;
  wxSelectableItemListNode *next = nullptr;
  SelectItem *GetData() { return data; }
  wxSelectableItemListNode *GetNext() { return next; }
};
class SelectableItemList {
 public:
  void Append(SelectItem *item) {
    auto node = std::make_unique<wxSelectableItemListNode>();
    node->data = item;
    if (!nodes_.empty()) nodes_.back()->next = node.get();
    nodes_.push_back(std::move(node));
  }
  wxSelectableItemListNode *GetFirst() {
    return nodes_.empty() ? nullptr : nodes_.front().get();
  }
  void DeleteContents(bool owned) { owned_ = owned; }
  void Clear() {
    if (owned_) for (auto &node : nodes_) delete node->data;
    nodes_.clear();
  }
 private:
  bool owned_ = false;
  std::vector<std::unique_ptr<wxSelectableItemListNode>> nodes_;
};
struct Platform { int GetSelectRadiusPix() { return 12; } } platform;
Platform *g_BasePlatform = &platform;
#include "production-route-hit.h"

namespace {
int checks = 0;
void Check(bool condition, const char *why) {
  ++checks;
  if (!condition) throw std::runtime_error(why);
}
// Horizontal equatorial leg, nonzero longitude avoids date-line special cases.
// Invert the pinned Mercator projection to choose a physical pointer offset.
// Actual toSM independently supplies the native segment-distance calculation.
float LatitudeAtPixels(double pixels, double physical_ppm) {
  const double y = pixels / physical_ppm;
  const double z = WGS84_semimajor_axis_meters * mercator_k0;
  return static_cast<float>(std::asin(std::tanh(y / z)) / DEGREE);
}
SelectItem *Add(Select &selection, int type, RoutePoint &point, Route &route) {
  auto *item = new SelectItem;
  item->m_slat = item->m_slat2 = 0;
  item->m_slon = 0.001f;
  item->m_slon2 = 0.003f;
  item->m_seltype = type;
  item->m_pData1 = &point;
  item->m_pData3 = &route;
  selection.GetSelectList()->Append(item);
  return item;
}
void Selected(Select &selection, SelectCtx ctx, SelectItem *item,
              double pixels, bool expected) {
  const float lat = LatitudeAtPixels(pixels, ctx.scale);
  const bool segment = item->m_seltype == SELTYPE_ROUTESEGMENT ||
                       item->m_seltype == SELTYPE_TRACKSEGMENT;
  const float lon = segment ? 0.002f : item->m_slon;
  Check((selection.FindSelection(ctx, lat, lon, item->m_seltype) == item) == expected,
        "native first selection radius");
  auto list = selection.FindSelectionList(ctx, lat, lon, item->m_seltype);
  Check((list.GetFirst() != nullptr) == expected, "native selection-list radius");
  if (segment)
    Check(selection.IsSelectableSegmentSelected(ctx, lat, lon, item) == expected,
          "native retained segment radius");
}
}

int main() {
  try {
    RoutePoint point;
    Route route;
    for (int radius : {12, 50}) {  // Actual native mouse/touch initialization.
      for (double dpi : {1.0, 1.25, 1.5}) {
        CanvasQueryFixture canvas;
        canvas.dpi = dpi;
        auto ctx = canvas.CanvasQuery();
        Select selection;
        selection.SetSelectPixelRadius(radius);
        auto *item = Add(selection, SELTYPE_ROUTESEGMENT, point, route);
        // Legacy ignores the new context scale entirely.
        Selected(selection, ctx, item, radius * .9, true);
        Selected(selection, ctx, item, radius * 1.1, false);
        selection.SetRouteSegmentSelectionScale(1.5f);
        Selected(selection, ctx, item, radius * 1.5 * dpi * .9, true);
        Selected(selection, ctx, item, radius * 1.5 * dpi * 1.1, false);
        // Existing/plugin 3-arg contexts retain the old fixed-pixel contract.
        SelectCtx plugin(true, ctx.scale, ctx.chart_scale);
        Selected(selection, plugin, item, radius * 1.5 * .9, true);
        Selected(selection, plugin, item, radius * 1.5 * 1.1, false);
        for (int type : {SELTYPE_TRACKSEGMENT, SELTYPE_ROUTEPOINT, SELTYPE_AISTARGET,
                         SELTYPE_TIDEPOINT, SELTYPE_CURRENTPOINT}) {
          auto *other = Add(selection, type, point, route);
          Selected(selection, ctx, other, radius * .9, true);
          Selected(selection, ctx, other, radius * 1.1, false);
        }
        // Hidden inactive routes retain their existing list-only visibility rule.
#ifdef HAS_ROUTE_HIT_DIP_SCALE
        SelectCtx hidden(false, ctx.scale, ctx.chart_scale, dpi);
#else
        SelectCtx hidden(false, ctx.scale, ctx.chart_scale);
#endif
        auto inactive = selection.FindSelectionList(hidden, 0, .002f, SELTYPE_ROUTESEGMENT);
        Check(!inactive.GetFirst(), "hidden inactive route not exposed by larger target");
        route.m_bRtIsActive = true;
        auto active = selection.FindSelectionList(hidden, 0, .002f, SELTYPE_ROUTESEGMENT);
        Check(active.GetFirst() != nullptr, "active hidden route remains selectable");
        route.m_bRtIsActive = false;
      }
    }
    Select selection;
    selection.SetRouteSegmentSelectionScale(1.5f);
    auto *item = Add(selection, SELTYPE_ROUTESEGMENT, point, route);
#ifdef HAS_ROUTE_HIT_DIP_SCALE
    for (double invalid : {0.0, -1.0, .5, 4.01,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
      SelectCtx ctx(true, 1, 10000, invalid);
      Selected(selection, ctx, item, 17, true);
      Selected(selection, ctx, item, 19, false);
    }
#endif
    // The real canvas expression includes Retina's separate physical scale.
    CanvasQueryFixture retina;
    retina.m_displayScale = 2;
    auto ctx = retina.CanvasQuery();
    Selected(selection, ctx, item, 34, true);
    Selected(selection, ctx, item, 38, false);
    SelectItem unrelated;
    Check(!selection.IsSelectableSegmentSelected(ctx, 0, .002f, &unrelated),
          "unretained segment remains refused");
    std::cout << checks << " route-hit DPI checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << "route-hit DPI regression: " << error.what() << '\n';
    return 1;
  }
}
