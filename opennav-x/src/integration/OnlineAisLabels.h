#pragma once
#include "ais/ChartTargets.h"
#include "ui/Controls.h"
#include "ui/Theme.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace opennav::integration {
// Already projected by the real chart canvas; the label painter invents no
// geography or motion. A wxDC fixture can exercise the same ocpnDC painter.
struct OnlineAisLabelTarget { ais::ChartTarget mark; wxPoint point; };
template<class DC>
unsigned DrawOnlineAisLabels(DC &dc, wxWindow &window, ui::LightMode mode,
                            wxSize viewport,
                            const std::vector<OnlineAisLabelTarget> &targets,
                            double display_scale = 1.0) {
  if (!std::isfinite(display_scale) || display_scale < 1.0 || display_scale > 1.5)
    display_scale = 1.0;
  const auto pixels = [&](int value) {
    return static_cast<int>(std::lround(window.FromDIP(value) * display_scale));
  };
  const auto font=dc.GetFont();const auto text_ink=dc.GetTextForeground();
  auto label_font=ui::UiFont(window,9);
  label_font.SetFractionalPointSize(label_font.GetFractionalPointSize()*display_scale);
  dc.SetFont(label_font);
  dc.SetTextForeground(ui::Colour(ui::OnlineChartTheme(mode).label));
  std::vector<ais::ChartLabelBounds> occupied;
  for(const auto &p:targets) {
    const int radius=pixels(p.mark.selected?19:13);
    occupied.push_back({p.point.x-radius,p.point.y-radius,2*radius,2*radius});
  }
  std::vector<std::size_t> order(targets.size());
  std::iota(order.begin(),order.end(),0);
  std::sort(order.begin(),order.end(),[&](auto a,auto b) {
    const auto &x=targets[a].mark,&y=targets[b].mark;
    return x.selected!=y.selected?x.selected:x.mmsi<y.mmsi;
  });
  unsigned labels=0;
  for(const auto index:order) {
    const auto &p=targets[index];
    if(p.mark.label.empty() || labels==128 ||
       (p.mark.age!=ais::TargetAge::Live && p.mark.age!=ais::TargetAge::Aging))continue;
    const auto label_text=wxString::FromUTF8(p.mark.label);
    if(label_text.empty())continue; // Invalid UTF-8 cannot become a replacement label.
    int width=0,height=0,descent=0;
    dc.GetTextExtent(label_text,&width,&height,&descent);
    // Pinned ocpnDC clamps either extent to 500 but DrawText paints the full
    // string. Saturated metrics cannot establish its bounds: omit the label.
    if(width>=500 || height>=500)continue;
    if(descent<0 || descent>height)continue;
    // SVG y=-4 is the alphabetic baseline, not the top of its text box.
    const ais::ChartLabelBounds label{p.point.x+pixels(14),
        p.point.y-pixels(4)-(height-descent),width,height};
    const auto own_symbol=occupied[index];occupied[index].width=0;
    const bool fits=ais::ChartLabelFits(label,{0,0,viewport.x,viewport.y},occupied);
    occupied[index]=own_symbol;
    if(!fits)continue;
    dc.DrawText(label_text,label.x,label.y);
    const int gap=pixels(2);
    occupied.push_back({label.x-gap,label.y-gap,width+2*gap,height+2*gap});
    ++labels;
  }
  dc.SetFont(font);dc.SetTextForeground(text_ink);
  return labels;
}
} // namespace opennav::integration
