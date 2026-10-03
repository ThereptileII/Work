#include "integration/ChartRouteLabel.h"
#include "integration/ChartRouteLabelRaster.h"
#include "ui/Controls.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include "FontMgr.h"
#include "OCPNPlatform.h"
#include "chcanv.h"
#include "ocpndc.h"
#ifdef ocpnUSE_GL
#include "glChartCanvas.h"
#endif
#include <vector>
extern Routeman* g_pRouteMan;
extern float g_MarkScaleFactorExp;
namespace opennav::integration {
bool PrepareChartRouteLabel(ChartCanvas& canvas, RoutePoint& point, int ordinal) {
  const auto fallback=[&point] {
    if (point.m_skagerLabel) {
      point.m_skagerLabel->image=wxImage();
      point.m_skagerLabel->bitmap=wxBitmap();
      point.m_skagerLabel->texture_current=false;
      if (!point.m_skagerLabel->texture_owned) point.m_skagerLabel.reset();
    }
    return false;
  };
  // ordinal already applies every SCRUM-242 route/icon/navigation-state guard.
  if (!ordinal || !point.m_bShowName || point.GetName().empty() ||
      !point.m_pMarkFont || !point.m_pMarkFont->IsOk() || point.m_NameLocationOffsetX != -10 ||
      point.m_NameLocationOffsetY != 8) return fallback();
  static const wxFont system=*wxNORMAL_FONT;
  const auto* configured=FontMgr::Get().GetFont(_("Marks"));
  if (!configured || !FactoryRouteLabelFont(*configured,system,
          FontMgr::Get().GetFontColor(_("Marks"))) || point.m_FontColor!=*wxBLACK)
    return fallback();
  // Also preserve a point-local/cached custom font, not just current prefs.
  int size=wxMax(8,configured->GetPointSize());
  size/=OCPN_GetWinDIPScaleFactor();
  const auto* expected=FontMgr::Get().FindOrCreateFont(size,configured->GetFamily(),
      configured->GetStyle(),configured->GetWeight(),false,configured->GetFaceName());
  if (!expected || point.m_pMarkFont->GetNativeFontInfoDesc()!=expected->GetNativeFontInfoDesc())
    return fallback();
  const double scale=canvas.FromDIP(100)/100.*g_MarkScaleFactorExp;
  if (!std::isfinite(scale) || scale<.25 || scale>4 || g_MarkScaleFactorExp<=0)
    return fallback();
  auto font=ui::UiFont(canvas,10,false);
  font.SetFractionalPointSize(font.GetFractionalPointSize()*g_MarkScaleFactorExp);
  const int mode=canvas.GetColorScheme()==GLOBAL_COLOR_SCHEME_NIGHT ? 2 :
      canvas.GetColorScheme()==GLOBAL_COLOR_SCHEME_DUSK ? 1 : 0;
  auto* route=g_pRouteMan->GetpActiveRoute();
  if (!route) return fallback();
  const bool last=ordinal>1 && ordinal==route->GetnPoints();
  if(!point.m_skagerLabel)point.m_skagerLabel=std::make_shared<ChartRouteLabelRaster>();
  if (point.m_skagerLabel->Build(font,point.GetName(),scale,mode,last)) return true;
  return fallback();
}
bool DrawChartRouteLabel(ocpnDC& dc, ChartCanvas& canvas, RoutePoint& point,
                         int x, int y, bool prepared) {
  auto* cache=point.m_skagerLabel.get();
  if(dc.GetDC()) {
    if(!prepared || !cache)return false;
    dc.DrawBitmap(cache->bitmap,x+cache->origin.x,y+cache->origin.y,true);
  } else {
#ifdef ocpnUSE_GL
    if(!prepared || !cache) {
      // Re-enter the existing stock texture builder; never reuse styled pixels.
      if(cache && cache->texture_owned) {
        if(point.m_iTextTexture)glDeleteTextures(1,&point.m_iTextTexture);
        point.m_iTextTexture=0;
        cache->texture_owned=cache->texture_current=false;
      }
      point.m_skagerLabel.reset();
      return false;
    }
    if(!cache->texture_current || !point.m_iTextTexture) {
      // Preserve any stock texture until a replacement upload is proven. On a
      // failure discard only our old styled texture and let stock rebuild once.
      const auto failed=[&] {
        return cache->FailTexture(point.m_iTextTexture,
            [](unsigned int texture) { glDeleteTextures(1,&texture); });
      };
      // Retry only after appearance changes. A known unsupported size/allocation
      // must not delete/recreate the stock texture on every chart frame.
      if(cache->texture_failed)return failed();
      const int w=cache->image.GetWidth(),h=cache->image.GetHeight();
      GLint maximum=0;glGetIntegerv(GL_MAX_TEXTURE_SIZE,&maximum);
      if(w>maximum || h>maximum)return failed();
      std::vector<unsigned char> rgba(w*h*4);
      const auto* rgb=cache->image.GetData();const auto* alpha=cache->image.GetAlpha();
      for(int i=0;i<w*h;++i) {
        std::copy_n(rgb+3*i,3,rgba.data()+4*i);rgba[4*i+3]=alpha[i];
      }
      GLuint replacement=0;glGenTextures(1,&replacement);
      if(!replacement)return failed();
      glBindTexture(GL_TEXTURE_2D,replacement);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
      glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,w,h,0,GL_RGBA,GL_UNSIGNED_BYTE,rgba.data());
      GLint allocated_w=0,allocated_h=0;
      glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_WIDTH,&allocated_w);
      glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_HEIGHT,&allocated_h);
      if(allocated_w!=w || allocated_h!=h) {
        glDeleteTextures(1,&replacement);return failed();
      }
      if(point.m_iTextTexture)glDeleteTextures(1,&point.m_iTextTexture);
      point.m_iTextTexture=replacement;
      point.m_iTextTextureWidth=w;point.m_iTextTextureHeight=h;
      cache->texture_owned=cache->texture_current=true;
    }
    const int left=x+cache->origin.x,top=y+cache->origin.y;
    const int right=left+cache->image.GetWidth(),bottom=top+cache->image.GetHeight();
    float coords[]={float(left),float(top),float(right),float(top),
                    float(right),float(bottom),float(left),float(bottom)};
    float uv[]={0,0,1,0,1,1,0,1};
    glBindTexture(GL_TEXTURE_2D,point.m_iTextTexture);
    glEnable(GL_TEXTURE_2D);glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glChartCanvas::RenderSingleTexture(dc,coords,uv,canvas.GetpVP(),0,0,0);
    glDisable(GL_BLEND);glDisable(GL_TEXTURE_2D);
#else
    return false;
#endif
  }
  dc.CalcBoundingBox(x+cache->bounds.x,y+cache->bounds.y);
  dc.CalcBoundingBox(x+cache->bounds.GetRight(),y+cache->bounds.GetBottom());
  return true;
}
} // namespace opennav::integration
