#pragma once
#include "ais/ChartTargets.h"
class ocpnDC;
class ViewPort;
class ChartCanvas;
namespace opennav::integration {
// Application-thread only; retains no OpenCPN objects. Both rendering paths use
// the same owned marks and the pinned canvas's geographic projection.
class OnlineAisOverlay {
public:
  bool Update(const vessel::AisState &display, vessel::Time now, int selected);
  void Clear();
  void Draw(ocpnDC &dc, ViewPort &vp, ChartCanvas &canvas) const;
  int HitTest(ViewPort &vp, ChartCanvas &canvas, int x, int y) const;
  std::size_t Size() const { return targets_.size(); }
private:
  std::vector<ais::ChartTarget> targets_;
};
} // namespace opennav::integration
