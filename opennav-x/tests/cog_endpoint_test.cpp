// Actual endpoint geometry/guard/polygon bodies; wx software output is real.
// Recorded DrawPolygon verifies submissions only, never native GL driver
// output.
#include <wx/wx.h>
#include <wx/graphics.h>
#include <array>
#include <vector>
#include <memory>
#include <stdexcept>
#include <iostream>
#include "reference-endpoint.h"
class App : public wxApp {
public:
  bool OnInit() override { return true; }
};
wxIMPLEMENT_APP_NO_MAIN(App);
struct ocpnDC {
  wxDC *dc = nullptr;
  wxGraphicsContext *pgc = nullptr;
  wxPen pen;
  wxBrush brush;
  std::vector<wxPoint> points;
  int polygons = 0;
  void SetPen(const wxPen &p) { pen = p; }
  void SetBrush(const wxBrush &b) { brush = b; }
  wxPen GetPen() { return pen; }
  wxBrush GetBrush() { return brush; }
  void DrawPolygon(int n, wxPoint p[], wxCoord, wxCoord, float) {
    ++polygons;
    points.assign(p, p + n);
  }
  void StrokePolygon(int, wxPoint[], wxCoord = 0, wxCoord = 0, float = 1);
};
#include "production-stroke-polygon.h"
constexpr double PI = 3.141592653589793;
constexpr int SHIP_NORMAL = 0, SHIP_LOWACCURACY = 1, SHIP_INVALID = 2;
int g_cog_predictor_endmarker = 1, g_cog_predictor_width = 3,
    g_OwnShipIconType = 0;
double g_ShipScaleFactorExp = 1, g_scaler = 1;
#include "production-endpoint-shape.h"
wxColour GetGlobalColor(const wxString &name) { if(name != "UBLCK") throw std::runtime_error("unexpected endpoint border role"); return wxColour(0,0,0); }
struct ChartCanvas;
bool draw_success = true, verified = true;
int predictor_calls = 0;
std::array<double, 4> predictor_coordinates{};
namespace opennav::integration {
bool DrawChartCogPredictor(ocpnDC &, ChartCanvas &, double ax, double ay,
                           double bx, double by) {
  ++predictor_calls;
  predictor_coordinates = {ax, ay, bx, by};
  return draw_success;
}
bool ChartActiveRouteInk(ChartCanvas &, wxColour &);
} // namespace opennav::integration
#define OPENNAV_X
struct ChartCanvas {
  int theme = 0, m_ownship_state = SHIP_NORMAL;
  void *m_pos_image_user = nullptr;
  void Paint(ocpnDC &dc, bool stock, bool xnav_cog_style, wxPoint lPredPoint,
             wxPoint GPSOffsetPixels, wxPoint2DDouble lGPSPoint,
             wxColour cPred) {
    wxPoint lShipMidPoint(lGPSPoint.m_x + GPSOffsetPixels.x,
                          lGPSPoint.m_y + GPSOffsetPixels.y);
    if (stock) {
#include "stock-endpoint.h"
    } else {
#include "production-cog-guard.h"
#include "production-endpoint.h"
    }
  }
};
namespace opennav::integration {
bool ChartActiveRouteInk(ChartCanvas &c, wxColour &ink) {
  if (!verified)
    return false;
  const unsigned colors[]{0x267c76, 0xb0dfc8, 0x71937e};
  const auto rgb = colors[c.theme];
  ink = wxColour(rgb >> 16, (rgb >> 8) & 255, rgb & 255);
  return true;
}
} // namespace opennav::integration
int checks = 0;
void Check(bool good, const char *reason) {
  ++checks;
  if (!good)
    throw std::runtime_error(reason);
}
int main(int argc, char **argv) {
  if (!wxEntryStart(argc, argv) || !wxTheApp->CallOnInit())
    return 2;
  try {
    for (int theme = 0; theme < 3; ++theme)
      for (int eligibility = 0; eligibility < 7; ++eligibility)
        for (int endmarker : {0, 1, 2})
          for (int width : {3, 7})
            for (double size : {1., 2.5})
              for (double scaler : {1., 1.5})
                for (wxPoint endpoint :
                     {wxPoint(100, 50), wxPoint(50, 100), wxPoint(100, 100)}) {
                  ChartCanvas canvas;
                  canvas.theme = theme;
                  canvas.m_ownship_state = eligibility == 2   ? SHIP_LOWACCURACY
                                           : eligibility == 6 ? SHIP_INVALID
                                                              : SHIP_NORMAL;
                  canvas.m_pos_image_user =
                      eligibility == 4 ? &canvas : nullptr;
                  g_OwnShipIconType = eligibility == 3 ? 1 : 0;
                  draw_success = eligibility != 5;
                  verified = true;
                  const bool owned = eligibility != 1;
                  g_cog_predictor_width = width;
                  g_cog_predictor_endmarker = endmarker;
                  g_ShipScaleFactorExp = size;
                  g_scaler = scaler;
                  const wxColour original = eligibility == 1
                                                ? wxColour(17, 43, 81)
                                                : wxColour(255, 0, 0);
                  ocpnDC before, after;
                  canvas.Paint(before, true, owned, endpoint, {7, -9}, {50, 50},
                               original);
                  predictor_calls = 0;
                  canvas.Paint(after, false, owned, endpoint, {7, -9}, {50, 50},
                               original);
                  Check(after.polygons == before.polygons &&
                            after.points == before.points,
                        "marker preference or projected geometry changed");
                  Check(predictor_calls ==
                            (eligibility == 0 || eligibility == 5 ? 1 : 0),
                        "actual COG style/health/icon guard changed");
                  if (predictor_calls)
                    Check(predictor_coordinates ==
                              std::array<double, 4>{57, 41,
                                                    double(endpoint.x + 7),
                                                    double(endpoint.y - 9)},
                          "actual guard changed predictor endpoints");
                  if (!endmarker)
                    continue;
                  Check(after.pen == before.pen,
                        "stock endpoint border changed");
                  const auto rgb = reference_route_ink[theme];
                  const wxColour expected =
                      eligibility == 0
                          ? wxColour(rgb >> 16, (rgb >> 8) & 255, rgb & 255)
                          : original;
                  Check(after.brush.GetColour() == expected,
                        "endpoint fill ignored eligibility or prototype ink");
                }
    ChartCanvas canvas;
    g_OwnShipIconType = 0;
    g_cog_predictor_width = 3;
    g_ShipScaleFactorExp = g_scaler = 1;
    g_cog_predictor_endmarker = 1;
    draw_success = true;
    verified = false;
    ocpnDC failed;
    canvas.Paint(failed, false, true, {100, 50}, {0, 0}, {50, 50},
                 wxColour(255, 0, 0));
    Check(failed.brush.GetColour() == wxColour(255, 0, 0),
          "unverified ink replaced stock fill");
    verified = true;
    wxInitAllImageHandlers();
    wxBitmap bmp(480, 240, 24);
    wxMemoryDC native(bmp);
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(native));
    ocpnDC dc;
    dc.dc = &native;
    dc.pgc = gc.get();
    const unsigned waters[]{0xd5e5e5, 0x344f59, 0x0e171c};
    for (int theme = 0; theme < 3; ++theme) {
      canvas.theme = theme;
      int y = theme * 80;
      auto rgb = waters[theme];
      native.SetPen(*wxTRANSPARENT_PEN);
      native.SetBrush(
          wxBrush(wxColour(rgb >> 16, (rgb >> 8) & 255, rgb & 255)));
      native.DrawRectangle(0, y, 480, 80);
      native.SetTextForeground(theme ? wxColour(200, 215, 205)
                                     : wxColour(35, 62, 62));
      native.DrawText("Stock enabled     Styled enabled     Disabled", 10,
                      y + 8);
      canvas.Paint(dc, true, true, {60, y + 50}, {0, 0}, {30, double(y + 50)},
                   wxColour(255, 0, 0));
      canvas.Paint(dc, false, true, {225, y + 50}, {0, 0},
                   {195, double(y + 50)}, wxColour(255, 0, 0));
      g_cog_predictor_endmarker = 0;
      canvas.Paint(dc, false, true, {385, y + 50}, {0, 0},
                   {355, double(y + 50)}, wxColour(255, 0, 0));
      g_cog_predictor_endmarker = 1;
    }
    gc.reset();
    native.SelectObject(wxNullBitmap);
    auto image = bmp.ConvertToImage();
    for (int theme = 0; theme < 3; ++theme) {
      int y = theme * 80 + 50;
      auto color = [&](int x) {
        return (unsigned(image.GetRed(x, y)) << 16) |
               (unsigned(image.GetGreen(x, y)) << 8) | image.GetBlue(x, y);
      };
      Check(color(60) == 0xff0000 && color(225) == reference_route_ink[theme] &&
                color(385) == waters[theme],
            "actual software endpoint fill or disabled preference differs");
    }
    Check(image.SaveFile(argv[1], wxBITMAP_TYPE_PNG),
          "cannot save endpoint fixture");
    std::cout << checks << " actual endpoint/guard/painter checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  wxTheApp->OnExit();
  wxEntryCleanup();
  return 0;
}
