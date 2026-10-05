// Executes the extracted production painter, with a recording DC backed by wx.
// Caller selection/projection and the real GL driver remain integration gates.
#include "ui/Theme.h"
#include <wx/wx.h>
#include <wx/graphics.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

class App : public wxApp { public: bool OnInit() override { return true; } };
wxIMPLEMENT_APP_NO_MAIN(App);
constexpr int GLOBAL_COLOR_SCHEME_DAY = 0, GLOBAL_COLOR_SCHEME_DUSK = 1,
              GLOBAL_COLOR_SCHEME_NIGHT = 2;
constexpr int SHIP_NORMAL = 0, SHIP_LOWACCURACY = 1, SHIP_INVALID = 2;
class ChartCanvas {
 public:
  int scheme = 0, dpi_percent = 100;
  int m_ownship_state = SHIP_NORMAL;
  int GetOwnShipState() const { return m_ownship_state; }
  int GetColorScheme() const { return scheme; }
  int FromDIP(int value) const { return value * dpi_percent / 100; }
};
class ocpnDC {
 public:
  explicit ocpnDC(wxMemoryDC &target) : target(target) {}
  wxMemoryDC &target;
  wxPen pen{*wxRED, 1}, painted_pen;
  wxBrush brush{*wxBLUE}, painted_brush;
  std::vector<wxPoint> points, bounds;
  int paints = 0;
  int circles = 0;
  wxPen GetPen() const { return pen; }
  wxBrush GetBrush() const { return brush; }
  void SetPen(wxPen value) { pen = value; }
  void SetBrush(wxBrush value) { brush = value; }
  void CalcBoundingBox(int x, int y) { bounds.emplace_back(x, y); }
  void StrokeCircle(double x, double y, double radius) {
    ++paints; ++circles; points.clear(); painted_pen=pen; painted_brush=brush;
    target.SetPen(pen); target.SetBrush(brush); target.DrawCircle(x,y,radius);
  }
  void StrokePolygon(int count, wxPoint *value, int x, int y) {
    ++paints; points.assign(value, value + count);
    painted_pen = pen; painted_brush = brush; bounds.clear();
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(target));
    auto path = gc->CreatePath();
    path.MoveToPoint(value[0].x + x, value[0].y + y);
    for (int i = 1; i < count; ++i) path.AddLineToPoint(value[i].x + x, value[i].y + y);
    path.AddLineToPoint(value[0].x + x, value[0].y + y);
    gc->SetPen(pen); gc->SetBrush(brush); gc->DrawPath(path);
  }
};
namespace opennav::integration {
bool xnav_mode = true, active = true;
wxColour Color(std::uint32_t c) {
  return {static_cast<unsigned char>(c >> 16), static_cast<unsigned char>(c >> 8),
          static_cast<unsigned char>(c)};
}
// Emulate the Windows SDK macro while compiling the actual painter.
#define max(a, b) WINDOWS_MAX_MACRO_MUST_NOT_EXPAND
#include "production-ownship.h"
#undef max
}
#include "prototype-ownship.h"
int checks = 0;
void Check(bool value, const char *reason) {
  ++checks; if (!value) throw std::runtime_error(reason);
}
double Cross(wxPoint a, wxPoint b, wxPoint p) {
  return (b.x-a.x)*double(p.y-a.y)-(b.y-a.y)*double(p.x-a.x);
}
bool Triangle(wxPoint a, wxPoint b, wxPoint c, wxPoint p) {
  const auto u=Cross(a,b,p), v=Cross(b,c,p), w=Cross(c,a,p);
  return !((u<0||v<0||w<0)&&(u>0||v>0||w>0));
}
int main(int argc, char **argv) {
  if (!wxEntryStart(argc, argv) || !wxTheApp->CallOnInit()) return 2;
  int result = 0;
  try {
    using namespace opennav::integration;
    wxInitAllImageHandlers();
    wxBitmap bitmap(640, 390, 24);
    wxMemoryDC target(bitmap);
    target.SetBackground(wxBrush(wxColour(50,60,65))); target.Clear();
    ocpnDC dc(target); ChartCanvas canvas;
    const auto original_pen=dc.GetPen(); const auto original_brush=dc.GetBrush();
    auto draw=[&](double angle=0, double scale=1) {
      return DrawChartOwnship(dc, canvas, 100, 100, angle, scale);
    };
    for (const auto mode : {std::pair{false,false}, {false,true}, {true,false}}) {
      xnav_mode=mode.first; active=mode.second;
      Check(!draw() && dc.paints==0, "inactive/Standard/Legacy fallback painted");
    }
    xnav_mode=active=true;
    for (double scale : {0.,-1.,std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()})
      Check(!draw(0,scale) && dc.paints==0, "invalid scale painted");
    Check(!draw(0,1e100), "unrepresentable scale painted");
    Check(!DrawChartOwnship(dc,canvas,1e100,100,0,1), "unrepresentable coordinate painted");
    canvas.dpi_percent=0;
    Check(!draw(), "zero post-DPI scale painted");
    canvas.dpi_percent=100;
    Check(!draw(std::numeric_limits<double>::quiet_NaN()), "invalid angle painted");
    Check(!DrawChartOwnship(dc,canvas,INFINITY,100,0,1), "invalid coordinate painted");
    const std::array<unsigned,3> fills{0x267C76,0xB0DFC8,0x71937E};
    const std::array<unsigned,3> strokes{0xF7F8F0,0x243A40,0x101A20};
    for (int theme=0; theme<3; ++theme) {
      canvas.scheme=theme;
      for (int dpi : {100,125,150}) for (double degrees : {0.,41.,90.,180.}) {
        canvas.dpi_percent=dpi;
        const double angle=degrees*std::acos(-1)/180;
        Check(draw(angle), "valid painter refused");
        Check(dc.painted_brush.GetColour()==Color(fills[theme]) &&
              dc.painted_pen.GetColour()==Color(strokes[theme]), "palette cache/stale color");
        Check(dc.GetPen()==original_pen && dc.GetBrush()==original_brush, "DC state leaked");
        Check(dc.points.size()==reference.size(), "wrong vertex count");
        for (const auto p : reference) {
          const double x=100+(p.m_x*std::cos(angle)-p.m_y*std::sin(angle))*dpi/100;
          const double y=100+(p.m_x*std::sin(angle)+p.m_y*std::cos(angle))*dpi/100;
          Check(std::any_of(dc.points.begin(),dc.points.end(),[&](wxPoint q) {
            return std::abs(q.x-x)<=.501 && std::abs(q.y-y)<=.501;
          }), "geometry diverges from immutable SVG");
        }
        Check(dc.bounds.size()==2, "missing repaint bounds");
        for (auto p : dc.points)
          Check(p.x>dc.bounds[0].x && p.y>dc.bounds[0].y &&
                p.x<dc.bounds[1].x && p.y<dc.bounds[1].y, "repaint bounds clip glyph");
      }
    }
    canvas.dpi_percent=100; canvas.scheme=0;
    Check(draw(0,2), "user scale refused");
    Check(dc.painted_pen.GetWidth()==6, "user scale did not scale outline");
    Check(draw(), "base draw refused");
    // Actual pinned ocpnDC four-point strip order [0,1,3,2]: ensure the
    // re-ordered concave path does not fill the missing stern notch.
    auto inside=[&](wxPoint p) { return Triangle(dc.points[0],dc.points[1],dc.points[3],p)
                                   || Triangle(dc.points[1],dc.points[3],dc.points[2],p); };
    Check(inside({100,100}), "GL strip misses body");
    Check(!inside({100,115}), "GL strip fills stern notch");
    // The scaled bitmap setting on the boat carries independent physical
    // length/beam. Preserve both dimensions rather than reverting to fixed art.
    Check(DrawChartOwnship(dc,canvas,100,100,0,2,.5,true), "scaled vessel refused");
    int minx=1000,maxx=-1000,miny=1000,maxy=-1000;
    for(auto p:dc.points){minx=std::min(minx,p.x);maxx=std::max(maxx,p.x);
      miny=std::min(miny,p.y);maxy=std::max(maxy,p.y);}
    Check(maxx-minx==22 && maxy-miny==70,"scaled vessel lost beam or length");
    for(double stretch:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
      Check(!DrawChartOwnship(dc,canvas,100,100,0,1,stretch),"invalid beam painted");
    int circles=dc.circles;
    Check(DrawChartOwnship(dc,canvas,100,100,0,1,1,false) && dc.circles==circles+1,
          "missing heading/course must not invent a north-facing vessel");
    for(auto state:{SHIP_LOWACCURACY,SHIP_INVALID}) {
      canvas.m_ownship_state=state;
      circles=dc.circles;
      Check(draw() && dc.circles==circles+1,"unqualified position must not show healthy chevron");
      Check(dc.painted_brush.GetColour()==Color(state==SHIP_LOWACCURACY
                  ? opennav::ui::Theme(opennav::ui::LightMode::Day).attention
                  : opennav::ui::FloatingTheme(opennav::ui::LightMode::Day).secondary),
            "unqualified position lost quality palette");
      Check(dc.GetPen()==original_pen && dc.GetBrush()==original_brush,"quality marker leaked DC state");
    }
    canvas.m_ownship_state=SHIP_NORMAL;
    target.SetBackground(wxBrush(wxColour(50,60,65))); target.Clear();
    const std::array<double,4> angles{0,41,90,180};
    for (int theme=0;theme<3;++theme) {
      canvas.scheme=theme;
      target.SetTextForeground(*wxWHITE);
      target.DrawText(theme==0 ? "Day" : theme==1 ? "Dusk" : "Night", 8, theme*130+8);
      for (int col=0;col<4;++col) {
        canvas.dpi_percent=col==3 ? 150 : 100;
        DrawChartOwnship(dc,canvas,80+160*col,60+130*theme,
                         angles[col]*std::acos(-1)/180,1);
        target.DrawText(wxString::Format("%.0f deg / %d%%",angles[col],canvas.dpi_percent),
                        22+160*col,100+130*theme);
      }
    }
    target.SelectObject(wxNullBitmap);
    auto img=bitmap.ConvertToImage();
    Check(img.GetRed(80,60)==0x26 && img.GetGreen(80,60)==0x7C, "native raster fill wrong");
    Check(img.GetRed(80,76)==50, "native raster notch filled");
    Check(img.SaveFile(wxString::FromUTF8(argv[1]),wxBITMAP_TYPE_PNG), "capture write failed");
    std::cout << checks << " ownship painter checks passed; local wx raster only, native/GL/boat pending\n";
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; result=1; }
  wxTheApp->OnExit(); wxEntryCleanup(); return result;
}
