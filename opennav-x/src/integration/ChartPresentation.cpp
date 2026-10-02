#include "integration/ChartPresentation.h"
#include "ui/Controls.h" // Before GL/X11 headers which define None.
#include "XNavChartResources.h"
#include "model/base_platform.h"
#include "picosha2.h"
#include "s52plib.h"
#include "chcanv.h"
#include "chartbase.h"
#include "ocpndc.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <wx/ffile.h>
#include <wx/fileconf.h>
#include <wx/filename.h>
#include <wx/log.h>
#include <wx/thread.h>

extern BasePlatform *g_BasePlatform;
namespace opennav::integration {
namespace {
wxFileConfig *preferences = nullptr;
bool xnav_mode = false, requested = true, active = false;
std::string status = "Standard OpenCPN presentation";
constexpr const char *key = "/OpenNav/ChartPresentationV1";
wxString ResourceDirectory() {
  if (!g_BasePlatform) return {};
  wxFileName folder(g_BasePlatform->GetSharedDataDir(), "");
  folder.AppendDir("opennav");
  folder.AppendDir("chart-style");
  folder.AppendDir("v1");
  return folder.GetPath();
}
wxColour Color(std::uint32_t c) {
  return {static_cast<unsigned char>(c >> 16),
          static_cast<unsigned char>(c >> 8), static_cast<unsigned char>(c)};
}
bool Verify(const wxString &folder) {
  for (const auto &resource : chart_style::generated::resources) {
    wxFFile file(
        wxFileName(folder, wxString::FromUTF8(resource.name)).GetFullPath(),
        "rb");
    if (!file.IsOpened() || file.Length() < 0 ||
        static_cast<std::uint64_t>(file.Length()) != resource.bytes)
      return false;
    picosha2::hash256_one_by_one hash;
    std::array<unsigned char, 8192> buffer{};
    std::uint64_t remaining = resource.bytes;
    while (remaining) {
      const auto count = static_cast<std::size_t>(
          (std::min)(remaining, std::uint64_t(buffer.size())));
      if (file.Read(buffer.data(), count) != count)
        return false;
      hash.process(buffer.begin(), buffer.begin() + count);
      remaining -= count;
    }
    hash.finish();
    if (picosha2::get_hash_hex_string(hash) != resource.sha256)
      return false;
  }
  return true;
}
} // namespace
void ConfigureChartPresentation(wxFileConfig &config, bool xnav) {
  if (!wxIsMainThread())
    return;
  preferences = &config;
  xnav_mode = xnav;
  active = false;
  wxString saved;
  requested = !config.Read(key, &saved) || saved == "XNav";
  if (!saved.empty() && saved != "XNav" && saved != "Standard") {
    requested = false;
    status = "Invalid chart-style setting; using Standard";
  } else if (xnav_mode && requested) {
    // S-52 is lazy-loaded by upstream only when an ENC is encountered. The
    // coastline-only view still needs the same independently verified palette.
    const auto directory = ResourceDirectory();
    active = !directory.empty() && Verify(directory);
    status = active ? "XNav presentation v1 / verified palette; ENC not loaded"
                    : "XNav presentation resources missing or changed; Standard fallback";
  } else {
    status = "Standard OpenCPN presentation";
  }
}
s52plib *CreateChartPresentation(const wxString &stock_path,
                                 bool force_legacy) {
  if (!wxIsMainThread() || !xnav_mode)
    return new s52plib(stock_path, force_legacy);
  active = false;
  if (requested && g_BasePlatform) {
    const auto directory = ResourceDirectory();
    if (Verify(directory)) {
      auto *library = new s52plib(
          wxFileName(directory, "S52RAZDS.RLE").GetFullPath(), false, false, true);
      if (library->m_bOK) {
        active = true;
        status = "XNav presentation v1 / pinned symbols";
        wxLogMessage("OpenNav chart presentation: verified XNav resources");
        return library;
      }
      delete library;
      status = "XNav presentation could not load; Standard fallback";
    } else
      status =
          "XNav presentation resources missing or changed; Standard fallback";
    wxLogWarning("OpenNav %s", wxString::FromUTF8(status));
  } else
    status = "Standard OpenCPN presentation";
  // XNav's explicit Standard fallback is not shadowed by a working-directory
  // chartsymbols.xml. Normal Legacy/Safe retain the pinned loader behavior.
  return new s52plib(stock_path, force_legacy, false);
}
bool ChartBackground(ColorScheme scheme, wxColour &land, wxColour &water) {
  if (!xnav_mode || !active)
    return false;
  const auto index = scheme == GLOBAL_COLOR_SCHEME_NIGHT  ? 2
                     : scheme == GLOBAL_COLOR_SCHEME_DUSK ? 1
                                                          : 0;
  const auto p = chart_style::generated::backgrounds[index];
  land = Color(p.land);
  water = Color(p.water);
  return true;
}
bool XNavChartRequested() { return requested; }
bool ChartActiveRouteInk(ChartCanvas &canvas, wxColour &ink) {
  if (!wxIsMainThread() || !xnav_mode || !active) return false;
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  ink = ui::Colour(ui::ActiveRouteInk(mode));
  return true;
}
bool DrawChartOwnship(ocpnDC &dc, ChartCanvas &canvas, double x, double y,
                      double angle, double scale) {
  if (!wxIsMainThread() || !xnav_mode || !active ||
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(angle) ||
      !std::isfinite(scale) || scale <= 0)
    return false;
  // Immutable prototype index.html ownShipHeading: M0-19 11 16 0 10-11 16Z.
  // These are logical chart SVG pixels, not vessel dimensions or meters.
  // Measure a larger DIP span so fractional Windows DPI is not rounded to 1.
  scale *= canvas.FromDIP(100) / 100.0;
  // Reserve room for rotation, the miter and dirty bounds before narrowing to
  // wx integer coordinates. Refuse unusable geometry and let upstream draw.
  constexpr double safe_limit = (std::numeric_limits<int>::max)() / 2.0;
  if (!std::isfinite(scale) || scale <= 0 ||
      (std::max)(std::abs(x), std::abs(y)) + 64 * scale > safe_limit)
    return false;
  // Start at the right stern: ocpnDC's four-point GL strip then uses the
  // notch-to-bow diagonal, inside this concave polygon. Starting at the bow
  // would select the outside stern-to-stern diagonal and fill the notch.
  const std::array<wxPoint, 4> outline{{{11, 16}, {0, 10}, {-11, 16}, {0, -19}}};
  std::array<wxPoint, 4> points;
  const double c = std::cos(angle), s = std::sin(angle);
  int left = 0, right = 0, top = 0, bottom = 0;
  for (std::size_t i = 0; i < outline.size(); ++i) {
    const double px = outline[i].x * scale, py = outline[i].y * scale;
    points[i] = wxPoint(std::lround(x + px * c - py * s),
                        std::lround(y + px * s + py * c));
    if (!i) {
      left = right = points[i].x;
      top = bottom = points[i].y;
    } else {
      left = (std::min)(left, points[i].x); right = (std::max)(right, points[i].x);
      top = (std::min)(top, points[i].y); bottom = (std::max)(bottom, points[i].y);
    }
  }
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  // The immutable Night chart ancestor applies brightness(.78). Apply it to
  // this new artwork only; do not filter real chart content or route semantics.
  const auto ink = [mode](std::uint32_t value) {
    if (mode == ui::LightMode::Night) {
      const auto dim = [](unsigned channel) {
        return static_cast<unsigned>(std::lround(channel * .78));
      };
      value = (dim((value >> 16) & 255) << 16) |
              (dim((value >> 8) & 255) << 8) | dim(value & 255);
    }
    return Color(value);
  };
  const auto old_pen = dc.GetPen(); const auto old_brush = dc.GetBrush();
  wxPen pen(ink(ui::FloatingTheme(mode).surface),
            (std::max)(1, static_cast<int>(std::lround(3 * scale))));
  pen.SetJoin(wxJOIN_MITER);
  dc.SetPen(pen); dc.SetBrush(wxBrush(ink(ui::ActiveRouteInk(mode))));
  // The shared DC path is native in software and GL; no stock texture tint or
  // ownship texture cache can retain a previous light-mode color.
  dc.StrokePolygon(points.size(), points.data(), 0, 0);
  const int margin = static_cast<int>(std::ceil(6 * scale)); // Miter + rounding.
  dc.CalcBoundingBox(left - margin, top - margin);
  dc.CalcBoundingBox(right + margin, bottom + margin);
  dc.SetBrush(old_brush); dc.SetPen(old_pen);
  return true;
}
bool DrawChartDepthUnit(ocpnDC &dc, ChartCanvas &canvas) {
  if (!wxIsMainThread() || !xnav_mode || !active || !canvas.GetShowDepthUnits())
    return false;
  wxString unit;
  switch (canvas.GetChartDepthUnit()) {
    case DEPTH_UNIT_FEET: unit = _("Feet"); break;
    case DEPTH_UNIT_METERS: unit = _("Meters"); break;
    case DEPTH_UNIT_FATHOMS: unit = _("Fathoms"); break;
    default: return false; // Mixed/unknown units are never guessed.
  }
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  const auto font = dc.GetFont();
  const auto ink = dc.GetTextForeground();
  dc.SetFont(ui::UiFont(canvas, 8)); // Final .map-disclaimer rule in the HTML.
  dc.SetTextForeground(ui::Colour(ui::FloatingTheme(mode).secondary));
  const wxString label = _("Chart depths") + ": " + unit;
  wxCoord width = 0, height = 0;
  dc.GetTextExtent(label, &width, &height);
  const auto size = canvas.GetClientSize();
  const int x = size.x - canvas.FromDIP(22) - width;
  // Keep the actual chart selector accessible until its workflow is redesigned.
  const int inset = (std::max)(canvas.FromDIP(15),
                               canvas.GetPianoHeight() + canvas.FromDIP(4));
  const int y = size.y - inset - height;
  const bool fits = x >= 0 && y >= 0 && width > 0 && height > 0;
  if (fits) dc.DrawText(label, x, y);
  dc.SetFont(font);
  dc.SetTextForeground(ink);
  return fits;
}
std::string ChartPresentationStatus() { return status; }
bool ChartScaleGeometry(ChartCanvas &canvas, int &x, int &y,
                        int &reference_width) {
  if (!wxIsMainThread() || !xnav_mode || !active ||
      canvas.GetClientSize().x < canvas.FromDIP(480)) return false;
  // .map-bottom-left: 28px inset, native Follow boat width 142px, 25px gap.
  x = canvas.FromDIP(28 + 142 + 25);
  y = canvas.GetClientSize().y - canvas.FromDIP(37);
  // Upstream halves this span, selects a nice distance in the user's units,
  // then projects that actual distance back to pixels. Never draw a fixed
  // 65px bar with an independently guessed distance label.
  reference_width = canvas.FromDIP(65 * 2);
  return true;
}
bool DrawChartScale(ocpnDC &dc, ChartCanvas &canvas, const wxString &label,
                    int x, int y, int length, wxRect &bounds) {
  if (!wxIsMainThread() || !xnav_mode || !active || length <= 0 ||
      x < 0 || y < 0 || length > canvas.GetClientSize().x - x) return false;
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  const auto ink = ui::Colour(ui::FloatingTheme(mode).secondary);
  const auto old_font = dc.GetFont(); const auto old_ink = dc.GetTextForeground();
  const auto old_pen = dc.GetPen(); const auto old_brush = dc.GetBrush();
  dc.SetFont(ui::UiFont(canvas, 8)); dc.SetTextForeground(ink);
  int width = 0, height = 0; dc.GetTextExtent(label, &width, &height);
  const int arm = canvas.FromDIP(5), gap = canvas.FromDIP(5);
  const int top = y - arm - gap - height;
  if (top >= 0) {
    // A real ENC can have a sounding directly behind this legend. Give the
    // scale a small neutral backing so charted depth cannot read as scale text.
    // The illustrative HTML never exercises this overlap; distance is still
    // the exact upstream result, and the legend's content geometry is unchanged.
    const int pad = canvas.FromDIP(4);
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.SetBrush(wxBrush(ui::Colour(ui::FloatingTheme(mode).surface)));
    dc.DrawRoundedRectangle(x - pad, top - pad,
        (std::max)(length, width) + 2 * pad, y - top + 1 + 2 * pad,
        canvas.FromDIP(3));
    dc.SetPen(wxPen(ink, canvas.FromDIP(1)));
    dc.DrawText(label, x, top);
    dc.DrawLine(x, y - arm, x, y);
    dc.DrawLine(x, y, x + length, y);
    dc.DrawLine(x + length, y, x + length, y - arm);
    bounds = wxRect(x - pad, top - pad, (std::max)(length, width) + 2 * pad,
                    y - top + 1 + 2 * pad);
  }
  dc.SetBrush(old_brush); dc.SetPen(old_pen); dc.SetFont(old_font); dc.SetTextForeground(old_ink);
  return top >= 0;
}
application::CommandResult SetXNavChartRequested(bool enabled) {
  if (!wxIsMainThread() || !preferences || !xnav_mode)
    return {false, "Chart style is unavailable in this interface"};
  wxString old;
  const bool existed = preferences->Read(key, &old);
  if (!preferences->Write(key, wxString(enabled ? "XNav" : "Standard")) ||
      !preferences->Flush()) {
    if (existed)
      preferences->Write(key, old);
    else
      preferences->DeleteEntry(key);
    const bool restored = preferences->Flush();
    return {false, restored ? "Chart style could not be saved"
                            : "Chart style save failed; check profile storage"};
  }
  requested = enabled;
  return {true, "Chart style saved. Restart XNav to apply."};
}
} // namespace opennav::integration
