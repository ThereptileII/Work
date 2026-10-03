#pragma once
#include <wx/dcmemory.h>
#include <wx/graphics.h>
#include <wx/image.h>
#include <algorithm>
#include <cmath>
#include <memory>

namespace opennav::integration {
inline bool FactoryRouteLabelFont(const wxFont& font, const wxFont& system,
                                  const wxColour& ink) {
  return font.IsOk() && system.IsOk() && ink == *wxBLACK &&
      font.GetFractionalPointSize() == system.GetFractionalPointSize() &&
      font.GetFaceName() == system.GetFaceName() &&
      font.GetStyle() == wxFONTSTYLE_NORMAL &&
      font.GetWeight() == wxFONTWEIGHT_NORMAL && !font.GetUnderlined() &&
      !font.GetStrikethrough();
}
struct RouteLabelColours { wxColour fill, text, border; };
inline RouteLabelColours RouteLabelPalette(int mode) {
  // Immutable HTML: .map-pop-bg/.map-pop-label, including Night's ancestor
  // chart-canvas brightness(.78). Alpha is unaffected by CSS brightness.
  if (mode == 2) return {{16,26,32}, {142,152,136}, {87,111,94,26}};
  if (mode == 1) return {{36,58,64}, {225,229,216}, {112,142,121,26}};
  return {{247,248,240}, {35,62,62}, {112,142,121,26}};
}
// Transient point-owned native raster: never modifies waypoint properties.
// One whole native text run preserves shaping/fallback; no per-character paint.
class ChartRouteLabelRaster {
 public:
  wxImage image;
  wxBitmap bitmap;
  wxRect bounds; // tight painted pixels, relative to the projected waypoint
  wxPoint origin; // raster quad origin; transparent POT padding is not hit/cull area
  bool texture_current = false;
  bool texture_owned = false;
  bool texture_failed = false;
  unsigned builds = 0;
  template<class Delete> bool FailTexture(unsigned int& texture, Delete destroy) {
    if(texture_owned && texture) { destroy(texture);texture=0; }
    texture_owned=texture_current=false;
    texture_failed=true;
    return false;
  }
  bool Build(const wxFont& font, const wxString& text, double scale,
             int mode, bool last) {
    if (!font.IsOk() || text.empty() || text.size() > 256 ||
        text.find('\n') != wxString::npos || text.find('\r') != wxString::npos ||
        text.find('\t') != wxString::npos || !std::isfinite(scale) ||
        scale < .25 || scale > 4 || mode < 0 || mode > 2) return false;
    for (const auto c:text) if (c.GetValue()<32 || c.GetValue()==127) return false;
    const auto description = font.GetNativeFontInfoDesc();
    if (description == font_ && text == text_ && scale == scale_ &&
        mode == mode_ && last == last_ && bitmap.IsOk()) return true;
    wxBitmap probe(1,1,32); wxMemoryDC measure(probe);
    std::unique_ptr<wxGraphicsContext> metrics(wxGraphicsContext::Create(measure));
    if (!metrics) return false;
    metrics->SetFont(font, *wxBLACK);
    double w=0,h=0,descent=0;
    metrics->GetTextExtent(text,&w,&h,&descent);
    if (!std::isfinite(w) || !std::isfinite(h) || w<=0 || h<=0 || w>2048 || h>128)
      return false;
    // JS String.length counts UTF-16 code units, also on wx's UTF-32 hosts.
    size_t units=0; for (const auto c:text) units += c.GetValue()>0xffff ? 2 : 1;
    const double card_w=(std::min)(170.,units*6.+20.)*scale;
    const double left=(last ? -card_w-14*scale : -18*scale);
    const double tx=left+10*scale, ty=34*scale-(h-descent);
    const int pad=static_cast<int>(std::ceil(8*scale));
    const int x=std::floor((std::min)(left-.5*scale,tx))-pad;
    const int y=std::floor((std::min)(17.5*scale,ty))-pad;
    const int right=std::ceil((std::max)(left+card_w+.5*scale,tx+w))+pad;
    const int bottom=std::ceil((std::max)(43.5*scale,ty+h))+pad;
    int tw=1,th=1;
    while(tw<right-x)tw*=2;
    while(th<bottom-y)th*=2;
    // Bound texture upload/storage, even for enlarged or complex native fonts.
    if(tw>4096 || th>256 || tw*th>65536) return false;
    wxImage transparent(tw,th); if(!transparent.IsOk())return false;
    transparent.InitAlpha();
    std::fill_n(transparent.GetData(),tw*th*3,0);
    std::fill_n(transparent.GetAlpha(),tw*th,0);
    wxBitmap result(transparent); wxMemoryDC dc(result);
    {
      std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(dc));
      if(!gc)return false;
      const auto colours=RouteLabelPalette(mode);
      gc->SetBrush(wxBrush(colours.fill));
      gc->SetPen(gc->CreatePen(wxGraphicsPenInfo(colours.border).Width(scale)));
      gc->DrawRoundedRectangle(left-x,18*scale-y,card_w,25*scale,5*scale);
      gc->SetFont(font,colours.text);
      gc->DrawText(text,tx-x,ty-y);
    }
    dc.SelectObject(wxNullBitmap);
    auto pixels=result.ConvertToImage();
    if(!pixels.IsOk() || !pixels.HasAlpha())return false;
    // An unusual font overhang must fall back as a whole, never clip a name.
    const auto* a=pixels.GetAlpha();
    for(int xx=0;xx<tw;++xx)if(a[xx] || a[(th-1)*tw+xx])return false;
    for(int yy=0;yy<th;++yy)if(a[yy*tw] || a[yy*tw+tw-1])return false;
    int min_x=tw,min_y=th,max_x=-1,max_y=-1;
    for(int yy=0;yy<th;++yy)for(int xx=0;xx<tw;++xx)if(a[yy*tw+xx]) {
      min_x=(std::min)(min_x,xx);min_y=(std::min)(min_y,yy);
      max_x=(std::max)(max_x,xx);max_y=(std::max)(max_y,yy);
    }
    if(max_x<min_x || max_y<min_y)return false;
    image=pixels; bitmap=result; origin={x,y};
    bounds={x+min_x,y+min_y,max_x-min_x+1,max_y-min_y+1};
    font_=description;text_=text;scale_=scale;mode_=mode;last_=last;
    texture_current=texture_failed=false;++builds;
    return true;
  }
 private:
  wxString font_,text_;
  double scale_=0;
  int mode_=-1;
  bool last_=false;
};
} // namespace opennav::integration
