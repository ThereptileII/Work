#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
namespace opennav::integration {
// Ownership is presentation policy, not an inference about user intent. Values
// equal to pinned factory paint are owned by verified SKAGER presentation.
class CogPredictorStyleOwnership {
 public:
  void Capture(int width, int style, bool factory_color) {
    owned_ = width == 3 && style == 105 && factory_color;
    expected_width_ = width;
  }
  bool BeforeDensity(int width, int style, bool factory_color, int density_width) {
    if (!owned_) return false;
    if (width != expected_width_ || style != 105 || !factory_color || density_width < 1) {
      owned_ = false; // A runtime preference change revokes this startup capture.
      return false;
    }
    expected_width_ = (std::max)(width, density_width);
    return true;
  }
 private:
  bool owned_ = false;
  int expected_width_ = 0;
};

inline std::vector<float> ChartCogPredictorMesh(double ax, double ay,
    double bx, double by, double scale, double viewport_width, double viewport_height,
    bool *valid = nullptr) {
  if (valid) *valid = false;
  std::vector<float> triangles;
  for (double v : {ax,ay,bx,by,scale,viewport_width,viewport_height})
    if (!std::isfinite(v)) return triangles;
  if (scale < .25 || scale > 16 || viewport_width <= 0 || viewport_height <= 0 ||
      viewport_width > 65536 || viewport_height > 65536 ||
      (std::max)({std::abs(ax),std::abs(ay),std::abs(bx),std::abs(by)}) > 1e9)
    return triangles;
  const double dx=bx-ax, dy=by-ay, length=std::hypot(dx,dy);
  if (length==0) return triangles;
  if (valid) *valid = true;
  const double radius=.6*scale, dash=5*scale, period=10*scale;
  const double ux=dx/length, uy=dy/length, nx=-uy*radius, ny=ux*radius;
  double begin=0, end=1;
  const auto clip=[&](double p,double q) {
    if(p==0) return q>=0;
    const double t=q/p;
    if(p<0) { if(t>end) return false; begin=(std::max)(begin,t); }
    else { if(t<begin) return false; end=(std::min)(end,t); }
    return true;
  };
  if(!clip(-dx,ax+radius)||!clip(dx,viewport_width+radius-ax)||
     !clip(-dy,ay+radius)||!clip(dy,viewport_height+radius-ay)) return triangles;
  const double first=begin*length, last=end*length;
  // Clip first, but keep the original projected origin's 5/5 dash phase.
  // Work grows with the visible span, not an offscreen predictor's length.
  for(double start=std::floor(first/period)*period; start<last; start+=period) {
    const double a=(std::max)(first,start), b=(std::min)(last,start+dash);
    if(b<=a) continue;
    const double x1=ax+a*ux,y1=ay+a*uy,x2=ax+b*ux,y2=ay+b*uy;
    triangles.insert(triangles.end(), {float(x1+nx),float(y1+ny),
      float(x1-nx),float(y1-ny),float(x2+nx),float(y2+ny),
      float(x2+nx),float(y2+ny),float(x1-nx),float(y1-ny),
      float(x2-nx),float(y2-ny)});
  }
  return triangles;
}
} // namespace opennav::integration
