#pragma once
#include "integration/ChartNameText.h"
#include <wx/dcmemory.h>
#include <wx/image.h>
#include <algorithm>
#include <cstring>
#include <string_view>
#include <vector>

namespace opennav::integration {
// Exact three LIGHTS06 description suffixes in pinned s52cnsy.cpp. The literal
// content is not parsed, shortened or regenerated. ORIENT TE and OBJNAM stay stock.
inline bool IsGeneratedLightDescription(const char* feature, const char* rule,
                                         bool tx) {
  if (!feature || !rule || !tx || std::strncmp(feature,"LIGHTS",6)) return false;
  const std::string_view all(rule);
  const auto s=all.substr(0,all.find_first_of(";\037"));
  if (s.empty() || s.front() != '\'' || s.size() > 512) return false;
  for (const auto tail : {"',3,3,3,'15110',2,-1,CHBLK,23)",
                          "',3,2,3,'15110',2,0,CHBLK,23)",
                          "',3,2,3,'15110',2,1,CHBLK,23)"}) {
    const std::string_view t(tail);
    if (s.size() >= t.size() && s.substr(s.size()-t.size()) == t) return true;
  }
  return false;
}
inline bool FactoryLightTextFont(const wxFont& font, const wxFont& system,
                                  wxColour ink) {
  return font.IsOk() && system.IsOk() && ink == *wxBLACK &&
      font.GetPointSize()==system.GetPointSize() &&
      font.GetFaceName()==system.GetFaceName() &&
      font.GetStyle()==wxFONTSTYLE_NORMAL &&
      font.GetWeight()==wxFONTWEIGHT_NORMAL && !font.GetUnderlined() &&
      !font.GetStrikethrough();
}

// Shared SW bitmap / GL texture payload. Round mask dilation follows the native
// anti-aliased glyph outline; a fractional disk edge retains the 1.75px radius.
// It is a raster approximation of SVG stroke, not a new symbol or text layout.
class ChartLightLabelRaster {
 public:
  wxImage image;
  wxBitmap bitmap;
  int margin=0;
  unsigned builds=0;
  bool Build(const wxFont& font, const wxString& text, double scale,
             wxColour ink, wxColour water) {
    if (!font.IsOk() || text.empty() || text.size()>256 ||
        !std::isfinite(scale) || scale<.5 || scale>4) return false;
    // FontMgr returns cached font ref-data. Retain it, so native copy-on-write
    // also invalidates changed fonts without formatting keys every chart frame.
    if (font.GetRefData()==font_.GetRefData() && text==text_ && scale==scale_ &&
        ink==ink_ && water==water_ && image.IsOk() && bitmap.IsOk()) return true;
    wxBitmap probe(1,1,24); wxMemoryDC dc(probe); dc.SetFont(font);
    int w,h; dc.GetTextExtent(text,&w,&h);
    ChartNameTextRun run(dc,text,.12*scale); w+=run.extra_width;
    const double radius=1.75*scale;
    const int pad=static_cast<int>(std::ceil(radius+.5));
    if(w<=0 || h<=0 || w>2048 || h>128) return false;
    int tw=1,th=1;
    while(tw<w+2*pad)tw*=2;
    while(th<h+2*pad)th*=2;
    if(tw*th>262144) return false;
    wxBitmap mask(tw,th,24); if(!mask.IsOk())return false;
    dc.SelectObject(mask);
    dc.SetBackground(*wxBLACK_BRUSH);dc.Clear();dc.SetFont(font);
    dc.SetTextForeground(*wxWHITE);run.DrawOpaque(dc,pad,pad);
    dc.SelectObject(wxNullBitmap);
    const auto source=mask.ConvertToImage();
    if(!source.IsOk())return false;
    wxImage result(tw,th);if(!result.IsOk())return false;result.InitAlpha();
    if(!result.HasAlpha())return false;
    const auto* input=source.GetData();auto* rgb=result.GetData();auto* alpha=result.GetAlpha();
    struct Offset {int x,y;double distance;};std::vector<Offset> disk;
    for(int y=-pad;y<=pad;++y)for(int x=-pad;x<=pad;++x) {
      const double distance=std::hypot(x,y);
      if(distance<radius+1)disk.push_back({x,y,distance});
    }
    std::fill(rgb,rgb+tw*th*3,0);std::fill(alpha,alpha+tw*th,0);
    const int fg[]={ink.Red(),ink.Green(),ink.Blue()};
    const int bg[]={water.Red(),water.Green(),water.Blue()};
    for(int y=0;y<h+2*pad;++y)for(int x=0;x<w+2*pad;++x) {
      const int at=y*tw+x;const double glyph=input[3*at]/255.;
      double halo=0;
      for(const auto d:disk) {
        const int xx=x+d.x,yy=y+d.y;
        if(xx>=0&&xx<tw&&yy>=0&&yy<th) {
          const double coverage=input[3*(yy*tw+xx)]/255.;
          // Coverage estimates the subpixel edge. Expanding this edge gives an
          // opaque interior even for thin glyphs with no fully covered pixel.
          if (coverage>0)
            halo=(std::max)(halo,std::clamp(coverage+radius-d.distance,0.,1.));
        }
      }
      const double a=glyph+halo*(1-glyph);
      alpha[at]=static_cast<unsigned char>(std::lround(a*255));
      if(a>0)for(int c=0;c<3;++c)
        rgb[3*at+c]=static_cast<unsigned char>(std::lround((fg[c]*glyph+bg[c]*halo*(1-glyph))/a));
    }
    image=result;bitmap=wxBitmap(image);margin=pad;
    font_=font;text_=text;scale_=scale;ink_=ink;water_=water;++builds;
    return bitmap.IsOk();
  }
 private:
  wxFont font_;
  wxString text_;
  double scale_=0;
  wxColour ink_,water_;
};
} // namespace opennav::integration
