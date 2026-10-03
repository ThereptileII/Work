#pragma once
#include "s52s57.h"
#include <cmath>
#include <cstring>
#include "integration/ChartCanvasInk.h"
#include <wx/graphics.h>
#include <wx/dcmemory.h>
#include <algorithm>
#include <memory>
#include <vector>

namespace opennav::integration {
inline const wxString* CaFanInstruction(const wxString& s) { return &s; }
inline const wxString* CaFanInstruction(const wxString* s) { return s; }
// Compact fan paint is per-record, independent of the final-owner point map.
inline unsigned CaFanColor(bool enabled, const ObjRazRules* node) {
  if (!enabled || !node || !node->obj || !node->LUP) return 0;
  const auto* obj=node->obj;const auto* lup=node->LUP;
  if (obj->Primitive_type!=GEO_POINT || std::memcmp(obj->FeatureName,"LIGHTS",6) ||
      lup->RCID!=31183 || lup->TNAM!=SIMPLIFIED ||
      std::memcmp(lup->OBCL,"LIGHTS",6) || !lup->ATTArray.empty() ||
      obj->n_attr<3 || obj->n_attr>4096 || !obj->att_array || !obj->attVal ||
      obj->attVal->GetCount()!=unsigned(obj->n_attr)) return 0;
  const auto instruction=[](const wxString& s) {
    return s=="CS(LIGHTS05)" || s=="CS(LIGHTS05)\037";
  };
  // Core stores INST by value; API17 stores it by pointer.
  const auto* text=CaFanInstruction(lup->INST);
  if (!text || !instruction(*text)) return 0;
  unsigned seen=0,color=0;double start=0,end=0;
  for(int i=0;i<obj->n_attr;++i) {
    const char* name=obj->att_array+i*6;
    for(const char* excluded:{"ORIENT","CATLIT","LITVIS","STATUS","QUAPOS","QUASOU"})
      if(!std::memcmp(name,excluded,6)) return 0;
    unsigned bit=0;
    if(!std::memcmp(name,"COLOUR",6))bit=1;
    else if(!std::memcmp(name,"SECTR1",6))bit=2;
    else if(!std::memcmp(name,"SECTR2",6))bit=4;
    else if(!std::memcmp(name,"VALNMR",6))bit=8;
    if(!bit)continue;
    if(seen&bit)return 0;
    seen|=bit;
    const auto* value=obj->attVal->Item(i);
    if(!value || !value->value)return 0;
    if(bit==1) {
      if(value->valType!=OGR_STR)return 0;
      const char* s=static_cast<const char*>(value->value);
      if(s[0]!='1' && s[0]!='3' && s[0]!='4')return 0;
      if(s[1])return 0;
      color=unsigned(s[0]-'0');
    } else {
      if(value->valType!=OGR_REAL)return 0;
      const double n=*static_cast<const double*>(value->value);
      if(!std::isfinite(n))return 0;
      if(bit==8) {if(n<=0)return 0;}
      else {if(n<0 || n>360)return 0;if(bit==2)start=n;else end=n;}
    }
  }
  const double sweep=start>end ? end-start+360 : end-start;
  return (seen&7)==7 && sweep>=1 && sweep<360 ? color : 0;
}

struct CaFanGeometry {
  wxPoint center, leg1, leg2; // Final original renderer pixels, including rounding.
  double radius=0, start=0, end=0, pixelScale=1;
};
struct CaFanPaint { wxColour fill, stroke; };
inline bool CaFanColors(unsigned color, const wxString& scheme, CaFanPaint& out) {
  const bool day=scheme=="DAY" || scheme=="DAY_BRIGHT";
  const bool dusk=scheme=="DUSK",night=scheme=="NIGHT";
  if(!day && !dusk && !night)return false;
  std::uint32_t fill=0,stroke=0;
  if(color==3)fill=stroke=day?0xb66e6c:dusk?0xd3948c:0xae7870;
  else if(color==4)fill=stroke=day?0x508d78:dusk?0x8fbaa2:0x789d84;
  else if(color==1) {
    fill=day?0xf7f4d8:dusk?0xddd6ae:0xb7ae88;
    stroke=day?0xa1976a:dusk?0xc5bc92:0x9d987a;
  } else return false;
  const auto mode=night?ui::LightMode::Night:dusk?ui::LightMode::Dusk:ui::LightMode::Day;
  const auto ink=[&](std::uint32_t rgb) {
    rgb=ChartCanvasInk(mode,rgb);
    return wxColour((rgb>>16)&255,(rgb>>8)&255,rgb&255);
  };
  out={ink(fill),ink(stroke)};return true;
}
class CaFanTile {
 public:
  // At most 4 MiB RGBA plus bounded wx backing/copy storage. No per-Rule or
  // cross-frame cache, no full-canvas image and no cropped-away sector legs.
  static constexpr size_t kMaxPixels=1024*1024;
  wxImage image;
  wxRect bounds;
  bool Build(const CaFanGeometry& g,const CaFanPaint& paint) {
    image.Destroy();bounds={};
    if(!std::isfinite(g.radius) || !std::isfinite(g.start) || !std::isfinite(g.end) ||
       !std::isfinite(g.pixelScale) || g.radius<=0 || g.radius>32768 ||
       g.pixelScale<.5 || g.pixelScale>8 ||
       std::abs(g.start)>1440 || std::abs(g.end)>1440)return false;
    const auto coordinate=[](int n){return n>=-10000000 && n<=10000000;};
    if(!coordinate(g.center.x)||!coordinate(g.center.y)||
       !coordinate(g.leg1.x)||!coordinate(g.leg1.y)||
       !coordinate(g.leg2.x)||!coordinate(g.leg2.y))return false;
    double start=g.start,end=g.end;
    if(end<=start)end+=360;
    if(end-start<1 || end-start>=360)return false;
    const double padding=std::ceil(1.3*g.pixelScale)+2;
    const int left=int(std::floor(std::min({double(g.center.x)-g.radius,double(g.leg1.x),double(g.leg2.x)})-padding));
    const int top=int(std::floor(std::min({double(g.center.y)-g.radius,double(g.leg1.y),double(g.leg2.y)})-padding));
    const int right=int(std::ceil(std::max({double(g.center.x)+g.radius,double(g.leg1.x),double(g.leg2.x)})+padding));
    const int bottom=int(std::ceil(std::max({double(g.center.y)+g.radius,double(g.leg1.y),double(g.leg2.y)})+padding));
    const int w=right-left+1,h=bottom-top+1;
    if(w<=0 || h<=0 || w>2048 || h>2048 || size_t(w)*h>kMaxPixels)return false;
    try {
      wxImage built(w,h,true);if(!built.IsOk())return false;
      built.InitAlpha();std::memset(built.GetAlpha(),0,size_t(w)*h);
      // Obtain native antialias coverage with opaque white masks, then compose
      // straight RGBA ourselves. Drawing low-alpha RGB directly into wxImage
      // loses several RGB levels during Cairo's premultiply/unpremultiply.
      for(unsigned layer=0;layer<3;++layer) {
        wxImage mask(w,h,true);if(!mask.IsOk())return false;
        mask.InitAlpha();std::memset(mask.GetAlpha(),0,size_t(w)*h);
        {
          std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(mask));
          if(!gc || !gc->SetCompositionMode(wxCOMPOSITION_OVER))return false;
          gc->SetAntialiasMode(wxANTIALIAS_DEFAULT);gc->Translate(-left,-top);
          const double pi=3.14159265358979323846;
          const double a=(start-90)*pi/180,b=(end-90)*pi/180;
          auto path=gc->CreatePath();
          if(layer==0) {
            path.MoveToPoint(g.center.x,g.center.y);
            path.AddLineToPoint(g.center.x+g.radius*std::cos(a),g.center.y+g.radius*std::sin(a));
            path.AddArc(g.center.x,g.center.y,g.radius,a,b,true);path.CloseSubpath();
            gc->SetBrush(*wxWHITE_BRUSH);gc->FillPath(path);
          } else {
            if(layer==1) {
              // One compound fill is the union of both butt-ended segments and
              // the SVG default miter join (limit 4). Native defaults differ.
              if(!Boundary(path,g))return false;
              gc->SetBrush(*wxWHITE_BRUSH);gc->FillPath(path,wxWINDING_RULE);
            } else {
              path.AddArc(g.center.x,g.center.y,g.radius,a,b,true);
              gc->SetPen(gc->CreatePen(wxGraphicsPenInfo(*wxWHITE).Width(1.2*g.pixelScale).Cap(wxCAP_BUTT)));
              gc->StrokePath(path);
            }
          }
        } // Native coverage is committed to mask alpha here.
        const wxColour color=layer?paint.stroke:paint.fill;
        const unsigned ink[]={color.Red(),color.Green(),color.Blue()};
        const double opacity=layer==0?.17:layer==1?.6:.8;
        auto* rgb=built.GetData();auto* alpha=built.GetAlpha();const auto* coverage=mask.GetAlpha();
        for(size_t i=0;i<size_t(w)*h;++i) {
          if(!coverage[i])continue;
          const double source=coverage[i]/255.0*opacity,behind=alpha[i]/255.0*(1-source),combined=source+behind;
          for(unsigned channel=0;channel<3;++channel)
            rgb[i*3+channel]=static_cast<unsigned char>(std::lround((ink[channel]*source+rgb[i*3+channel]*behind)/combined));
          alpha[i]=static_cast<unsigned char>(std::lround(combined*255));
        }
      }
      image=built;bounds=wxRect(left,top,w,h);return true;
    } catch(const std::bad_alloc&) {return false;}
  }
  static bool Boundary(wxGraphicsPath& path,const CaFanGeometry& g) {
    struct P {double x,y;};
    const double x1=g.center.x-g.leg1.x,y1=g.center.y-g.leg1.y;
    const double x2=g.leg2.x-g.center.x,y2=g.leg2.y-g.center.y;
    const double len1=std::hypot(x1,y1),len2=std::hypot(x2,y2),r=.325*g.pixelScale;
    if(len1<=0 || len2<=0)return false;
    const P n1{-y1/len1*r,x1/len1*r},n2{-y2/len2*r,x2/len2*r};
    const auto polygon=[&](std::vector<P> points) {
      double area=0;for(size_t i=0;i<points.size();++i){const auto& a=points[i];const auto& b=points[(i+1)%points.size()];area+=a.x*b.y-a.y*b.x;}
      if(area<0)std::reverse(points.begin(),points.end());
      path.MoveToPoint(points[0].x,points[0].y);
      for(size_t i=1;i<points.size();++i)path.AddLineToPoint(points[i].x,points[i].y);
      path.CloseSubpath();
    };
    polygon({{g.leg1.x+n1.x,g.leg1.y+n1.y},{g.center.x+n1.x,g.center.y+n1.y},
             {g.center.x-n1.x,g.center.y-n1.y},{g.leg1.x-n1.x,g.leg1.y-n1.y}});
    polygon({{g.center.x+n2.x,g.center.y+n2.y},{g.leg2.x+n2.x,g.leg2.y+n2.y},
             {g.leg2.x-n2.x,g.leg2.y-n2.y},{g.center.x-n2.x,g.center.y-n2.y}});
    const double cross=x1*y2-y1*x2;
    if(cross==0)return true;
    const double side=cross>0?-1:1;
    const P a{g.center.x+side*n1.x,g.center.y+side*n1.y};
    const P b{g.center.x+side*n2.x,g.center.y+side*n2.y};
    const double denominator=1+(x1*x2+y1*y2)/(len1*len2);
    if(denominator>0 && 2/denominator<=16)
      polygon({{double(g.center.x),double(g.center.y)},a,
               {g.center.x+side*(n1.x+n2.x)/denominator,g.center.y+side*(n1.y+n2.y)/denominator},b});
    else polygon({{double(g.center.x),double(g.center.y)},a,b});
    return true;
  }
  bool Draw(wxDC& dc) const {
    if(!image.IsOk())return false;
    wxBitmap bitmap(image);if(!bitmap.IsOk())return false;
    dc.DrawBitmap(bitmap,bounds.x,bounds.y,true);return true;
  }
};

#ifdef ocpnUSE_GL
// No program or texture persists. The existing texture shader is borrowed with
// all changed uniforms/state restored; its source and shared caches stay intact.
inline bool DrawCaFanGL(const CaFanTile& tile,GLuint shader,int width,int height) {
#ifdef USE_ANDROID_GLES2
  return false; // Desktop GL state/texture proof only; preserve original GLES path.
#else
  GLint viewport[4];glGetIntegerv(GL_VIEWPORT,viewport);
  if(viewport[0] || viewport[1] || viewport[2]!=width || viewport[3]!=height)return false;
  if(!tile.image.IsOk() || !shader || !glIsProgram(shader) || width<=0 || height<=0)return false;
  const GLint pos=glGetAttribLocation(shader,"position"),uv=glGetAttribLocation(shader,"aUV");
  const GLint mv=glGetUniformLocation(shader,"MVMatrix"),transform=glGetUniformLocation(shader,"TransformMatrix"),sampler=glGetUniformLocation(shader,"uTex");
  if(pos<0 || uv<0 || pos==uv || mv<0 || transform<0 || sampler<0)return false;
  GLint maximum=0;glGetIntegerv(GL_MAX_TEXTURE_SIZE,&maximum);
  if(tile.image.GetWidth()>maximum || tile.image.GetHeight()>maximum)return false;
  std::vector<unsigned char> rgba;
  try {rgba.resize(size_t(tile.image.GetWidth())*tile.image.GetHeight()*4);}catch(const std::bad_alloc&){return false;}
  for(size_t i=0;i<rgba.size()/4;++i) {
    std::copy_n(tile.image.GetData()+3*i,3,rgba.data()+4*i);rgba[4*i+3]=tile.image.GetAlpha()[i];
  }
  struct State {
    struct Attribute {
      GLuint index;GLint enabled,size,type,normalized,stride,buffer,integer=0;void* pointer;
      explicit Attribute(GLuint i):index(i) {
        glGetVertexAttribiv(i,GL_VERTEX_ATTRIB_ARRAY_ENABLED,&enabled);
        glGetVertexAttribiv(i,GL_VERTEX_ATTRIB_ARRAY_SIZE,&size);
        glGetVertexAttribiv(i,GL_VERTEX_ATTRIB_ARRAY_TYPE,&type);
        glGetVertexAttribiv(i,GL_VERTEX_ATTRIB_ARRAY_NORMALIZED,&normalized);
        glGetVertexAttribiv(i,GL_VERTEX_ATTRIB_ARRAY_STRIDE,&stride);
        glGetVertexAttribiv(i,GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING,&buffer);
        glGetVertexAttribPointerv(i,GL_VERTEX_ATTRIB_ARRAY_POINTER,&pointer);
        if(GLEW_VERSION_3_0)glGetVertexAttribiv(i,GL_VERTEX_ATTRIB_ARRAY_INTEGER,&integer);
      }
      void Restore()const {
        glBindBuffer(GL_ARRAY_BUFFER,buffer);
        if(integer)glVertexAttribIPointer(index,size,type,stride,pointer);
        else glVertexAttribPointer(index,size,type,normalized,stride,pointer);
        if(enabled)glEnableVertexAttribArray(index);else glDisableVertexAttribArray(index);
      }
    } a,b;
    GLint program,active,texture,buffer,pbo,alignment,rowLength,skipRows,skipPixels;
    GLint srcRgb,dstRgb,srcAlpha,dstAlpha,eqRgb,eqAlpha,oldSampler,boundSampler=0;
    bool samplerObjects=false;
    GLboolean blend,cull;GLfloat oldMv[16],oldTransform[16];GLuint shader,owned=0;
    GLint mv,transform,sampler;
    State(GLuint s,GLint p,GLint u,GLint m,GLint t,GLint tex):a(p),b(u),shader(s),mv(m),transform(t),sampler(tex) {
      glGetIntegerv(GL_CURRENT_PROGRAM,&program);glGetIntegerv(GL_ACTIVE_TEXTURE,&active);
      glActiveTexture(GL_TEXTURE0);glGetIntegerv(GL_TEXTURE_BINDING_2D,&texture);
      samplerObjects=GLEW_VERSION_3_3 || GLEW_ARB_sampler_objects;
      if(samplerObjects)glGetIntegerv(GL_SAMPLER_BINDING,&boundSampler);
      glGetIntegerv(GL_ARRAY_BUFFER_BINDING,&buffer);glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING,&pbo);
      glGetIntegerv(GL_UNPACK_ALIGNMENT,&alignment);glGetIntegerv(GL_UNPACK_ROW_LENGTH,&rowLength);
      glGetIntegerv(GL_UNPACK_SKIP_ROWS,&skipRows);glGetIntegerv(GL_UNPACK_SKIP_PIXELS,&skipPixels);
      glGetIntegerv(GL_BLEND_SRC_RGB,&srcRgb);glGetIntegerv(GL_BLEND_DST_RGB,&dstRgb);
      glGetIntegerv(GL_BLEND_SRC_ALPHA,&srcAlpha);glGetIntegerv(GL_BLEND_DST_ALPHA,&dstAlpha);
      glGetIntegerv(GL_BLEND_EQUATION_RGB,&eqRgb);glGetIntegerv(GL_BLEND_EQUATION_ALPHA,&eqAlpha);
      blend=glIsEnabled(GL_BLEND);cull=glIsEnabled(GL_CULL_FACE);
      glGetUniformfv(shader,mv,oldMv);glGetUniformfv(shader,transform,oldTransform);glGetUniformiv(shader,sampler,&oldSampler);
    }
    ~State() {
      glUseProgram(shader);glUniformMatrix4fv(mv,1,GL_FALSE,oldMv);glUniformMatrix4fv(transform,1,GL_FALSE,oldTransform);glUniform1i(sampler,oldSampler);
      a.Restore();b.Restore();glBindBuffer(GL_ARRAY_BUFFER,buffer);
      glBindBuffer(GL_PIXEL_UNPACK_BUFFER,pbo);glPixelStorei(GL_UNPACK_ALIGNMENT,alignment);
      glPixelStorei(GL_UNPACK_ROW_LENGTH,rowLength);glPixelStorei(GL_UNPACK_SKIP_ROWS,skipRows);glPixelStorei(GL_UNPACK_SKIP_PIXELS,skipPixels);
      if(samplerObjects)glBindSampler(0,boundSampler);
      glBindTexture(GL_TEXTURE_2D,texture);if(owned)glDeleteTextures(1,&owned);glActiveTexture(active);
      glBlendFuncSeparate(srcRgb,dstRgb,srcAlpha,dstAlpha);glBlendEquationSeparate(eqRgb,eqAlpha);
      if(blend)glEnable(GL_BLEND);else glDisable(GL_BLEND);
      if(cull)glEnable(GL_CULL_FACE);else glDisable(GL_CULL_FACE);
      glUseProgram(program);
    }
  } state(shader,pos,uv,mv,transform,sampler);
  glGenTextures(1,&state.owned);if(!state.owned)return false;
  if(state.samplerObjects)glBindSampler(0,0);
  glBindTexture(GL_TEXTURE_2D,state.owned);glBindBuffer(GL_PIXEL_UNPACK_BUFFER,0);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);glPixelStorei(GL_UNPACK_ROW_LENGTH,0);glPixelStorei(GL_UNPACK_SKIP_ROWS,0);glPixelStorei(GL_UNPACK_SKIP_PIXELS,0);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,tile.image.GetWidth(),tile.image.GetHeight(),0,GL_RGBA,GL_UNSIGNED_BYTE,rgba.data());
  GLint allocated=0;glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_WIDTH,&allocated);if(allocated!=tile.image.GetWidth())return false;
  glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_HEIGHT,&allocated);if(allocated!=tile.image.GetHeight())return false;
  const float left=tile.bounds.x,top=tile.bounds.y,right=left+tile.bounds.width,bottom=top+tile.bounds.height;
  GLfloat vertices[]={left,top,right,top,left,bottom,right,bottom},coords[]={0,0,1,0,0,1,1,1};
  GLfloat ortho[]={2.f/width,0,0,0, 0,-2.f/height,0,0, 0,0,1,0, -1,1,0,1};
  GLfloat identity[]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
  glUseProgram(shader);glUniformMatrix4fv(mv,1,GL_FALSE,ortho);glUniformMatrix4fv(transform,1,GL_FALSE,identity);glUniform1i(sampler,0);
  glBindBuffer(GL_ARRAY_BUFFER,0);glVertexAttribPointer(pos,2,GL_FLOAT,GL_FALSE,0,vertices);glVertexAttribPointer(uv,2,GL_FLOAT,GL_FALSE,0,coords);
  glEnableVertexAttribArray(pos);glEnableVertexAttribArray(uv);glEnable(GL_BLEND);glDisable(GL_CULL_FACE);
  glBlendFuncSeparate(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA,GL_ONE,GL_ONE_MINUS_SRC_ALPHA);glBlendEquationSeparate(GL_FUNC_ADD,GL_FUNC_ADD);
  glDrawArrays(GL_TRIANGLE_STRIP,0,4);return true;
#endif
}
#endif
} // namespace opennav::integration
