#pragma once
#include "integration/ChartCaFan.h"

namespace opennav::integration {
// Exact owned Rule signature. No class-wide cable or global CHMGD override.
inline constexpr char kCableWaveHpgl[] = "SPA;SW1;PU0,0;PD10,-16;PD20,-29;PD30,-40;PD40,-50;PD50,-57;PD60,-62;PD69,-65;PD79,-66;PD89,-65;PD99,-62;PD109,-57;PD119,-50;PD129,-40;PD139,-29;PD149,-16;PD159,0;PD169,16;PD179,29;PD189,40;PD198,50;PD208,57;PD218,62;PD228,65;PD238,66;PD248,65;PD258,62;PD268,57;PD278,50;PD288,40;PD298,29;PD308,16;PD318,0;PD327,-16;PD337,-29;PD347,-40;PD357,-50;PD367,-57;PD377,-62;PD387,-65;PD397,-66;PD407,-65;PD417,-62;PD427,-57;PD437,-50;PD446,-40;PD456,-29;PD466,-16;PD476,0;PD486,16;PD496,29;PD506,40;PD516,50;PD526,57;PD536,62;PD546,65;PD556,66;PD566,65;PD575,62;PD585,57;PD595,50;PD605,40;PD615,29;PD625,16;PD635,0;";
inline bool CableWaveRule(bool verified,const Rule* r) {
  return verified && r && r->RCID==2012 &&
    !std::memcmp(r->name.LINM,"CBLSUB06",8) && r->vector.LVCT && r->colRef.LCRF &&
    !std::strcmp(r->vector.LVCT,kCableWaveHpgl) && !std::strcmp(r->colRef.LCRF,"AXNCBL") &&
    r->pos.line.bnbox_w.PAHL==635 && r->pos.line.bnbox_h.PAVL==168 &&
    r->pos.line.bnbox_x.SBXC==0 && r->pos.line.bnbox_y.SBXR==-84 &&
    r->pos.line.pivot_x.PACL==0 && r->pos.line.pivot_y.PARW==0 &&
    r->pos.line.minDist.PAMI==0 && r->pos.line.maxDist.PAMA==0;
}
// One CPU tile per synchronous LC invocation. No chart/Rule pointer or GL
// object escapes. Repeated equal-angle motifs reuse coverage within this call.
class CableWaveTile {
 public:
  CaFanTile tile;
  bool Prepare(double scale,double angle,const wxColour& ink,wxPoint anchor) {
    if(!std::isfinite(scale)||scale<.25||scale>8 ||
       !std::isfinite(angle)||std::abs(angle)>6.284 || !ink.IsOk() ||
       std::abs(double(anchor.x))>10000000 || std::abs(double(anchor.y))>10000000)return false;
    if(!tile.image.IsOk() || scale!=lastScale || angle!=lastAngle || ink!=lastInk) {
      tile.image.Destroy();
      // The rotated convex hull of all Bézier control points contains the
      // complete curve. Add round stroke radius and2px antialias guard.
      double left=0,top=0,right=0,bottom=0;
      const double cs=std::cos(angle),sn=std::sin(angle);
      for(int i=0;i<=8;++i) {
        double x=3*i*scale,y=(i%2 ? (i%4==1?-5:5)*scale:0);
        double px=x*cs-y*sn,py=x*sn+y*cs;
        left=std::min(left,px);right=std::max(right,px);
        top=std::min(top,py);bottom=std::max(bottom,py);
      }
      const double pad=.65*scale+2;
      localBounds={int(std::floor(left-pad)),int(std::floor(top-pad)),
        int(std::ceil(right+pad)-std::floor(left-pad)),
        int(std::ceil(bottom+pad)-std::floor(top-pad))};
      const int w=localBounds.width,h=localBounds.height;
      if(w<=0||h<=0||size_t(w)*h>65536)return false;
      wxImage mask(w,h);if(!mask.IsOk())return false;
      mask.SetRGB(wxRect(0,0,w,h),0,0,0);mask.InitAlpha();
      if(!mask.HasAlpha())return false;
      std::fill_n(mask.GetAlpha(),size_t(w)*h,0);
      {
        std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(mask));
        if(!gc || !gc->SetCompositionMode(wxCOMPOSITION_OVER))return false;
        gc->SetAntialiasMode(wxANTIALIAS_DEFAULT);
        gc->Translate(-localBounds.x,-localBounds.y);gc->Rotate(angle);gc->Scale(scale,scale);
        auto path=gc->CreatePath();path.MoveToPoint(0,0);
        for(int i=0;i<4;++i)path.AddQuadCurveToPoint(6*i+3,i%2?5:-5,6*i+6,0);
        gc->SetPen(gc->CreatePen(wxGraphicsPenInfo(*wxWHITE).Width(1.3).Cap(wxCAP_ROUND).Join(wxJOIN_ROUND)));
        gc->StrokePath(path);
      }
      // Preserve native coverage; supply straight RGB independently of backend
      // premultiplication, as in the reviewed CA fan RGBA path.
      auto* rgb=mask.GetData();
      for(size_t i=0;i<size_t(w)*h;++i){rgb[3*i]=ink.Red();rgb[3*i+1]=ink.Green();rgb[3*i+2]=ink.Blue();}
      tile.image=mask;lastScale=scale;lastAngle=angle;lastInk=ink;
    }
    tile.bounds=localBounds;tile.bounds.Offset(anchor);return true;
  }
 private:
  wxRect localBounds;double lastScale=0,lastAngle=0;wxColour lastInk;
};
#ifdef ocpnUSE_GL
inline bool DrawCableWaveGL(const CaFanTile& tile,GLuint textureShader,
                           GLuint lineShader,int width,int height) {
  if(!lineShader || !glIsProgram(lineShader))return false;
  const GLint mv=glGetUniformLocation(lineShader,"MVMatrix"),
              tr=glGetUniformLocation(lineShader,"TransformMatrix");
  if(mv<0||tr<0)return false;
  GLfloat projection[16],transform[16];
  glGetUniformfv(lineShader,mv,projection);glGetUniformfv(lineShader,tr,transform);
  for(int i=0;i<16;++i)if(!std::isfinite(projection[i])||!std::isfinite(transform[i]))return false;
  // Borrow EXACT actual line-painter matrices, including private chart view.
  return DrawCaFanGL(tile,textureShader,width,height,projection,transform);
}
#endif
} // namespace opennav::integration
