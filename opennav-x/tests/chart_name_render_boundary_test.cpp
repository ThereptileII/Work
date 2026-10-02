// Executes actual patched RenderText, overlap and registration methods.
// GL calls record textures only; no driver/context or full chart is simulated.
#include <wx/wx.h>
#include <wx/dcscreen.h>
#include <wx/dcmemory.h>
#include <wx/listimpl.cpp>
#include <GL/gl.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include "s52s57.h"
#include "TexFont.h"
#include "integration/ChartNameText.h"

WX_DECLARE_LIST(S52_TextC, TextObjList);
WX_DEFINE_LIST(TextObjList);
struct VPointCompat { int pix_width=1000, pix_height=800; double rotation=0; wxRect rv_rect; };
struct TexFontCache { wxFont *key=nullptr; TexFont *cache=nullptr; };
#define TXF_CACHE 8
class s52plib {
 public:
  double m_dipfactor=1, m_TextScaleFactor=1, m_ContentScaleFactor=1, m_FinalTextScaleFactor=-1;
  bool m_useS52DefaultTextColor=true, m_bDeClutterText=true;
  VPointCompat vp_plib;
  TexFontCache s_txf[TXF_CACHE];
  TextObjList m_textObjList;
  bool RenderText(wxDC*,S52_TextC*,int,int,wxRect*,S57Obj*,bool);
  bool CheckTextRectList(const wxRect&,S52_TextC*);
  void RegisterText(bool bwas_drawn,S52_TextC *text);
};
static double fontContentScale=1;
static wxColour userInk(101,40,120);
static std::vector<std::unique_ptr<wxFont>> fonts;
wxColour GetFontColour_PlugIn(wxString) { return userInk; }
wxFont *FindOrCreateFont_PlugIn(int size,wxFontFamily family,wxFontStyle style,
    wxFontWeight weight,bool underline=false,const wxString& face=wxEmptyString) {
  fonts.push_back(std::make_unique<wxFont>(fontContentScale*size,family,style,weight,underline,face));
  return fonts.back().get();
}
// These stock glyph-cache functions must not execute in this focused fixture.
TexFont::TexFont() {} TexFont::~TexFont() {}
void TexFont::Build(wxFont&,double,double,bool) { throw std::runtime_error("Unexpected glyph-cache branch"); }
void TexFont::GetTextExtent(const wxString&,int*,int*) { throw std::runtime_error("Unexpected glyph-cache branch"); }

static GLuint nextTexture=1;
static std::vector<GLuint> deleted;
static std::vector<unsigned char> uploaded;
extern "C" {
void glDeleteTextures(GLsizei n,const GLuint *textures) { deleted.insert(deleted.end(),textures,textures+n); }
void glGenTextures(GLsizei n,GLuint *textures) { while(n--) *textures++=nextTexture++; }
void glBindTexture(GLenum,GLuint) {}
void glEnable(GLenum) {} void glDisable(GLenum) {}
void glTexParameteri(GLenum,GLenum,GLint) {}
void glTexImage2D(GLenum,GLint,GLint,GLsizei w,GLsizei h,GLint,GLenum,GLenum,const GLvoid *data) {
  const auto *bytes=static_cast<const unsigned char*>(data);
  uploaded.assign(bytes,bytes+4*w*h);
}
}
#include "chart-name-render-methods.inc"
class FixtureApp : public wxApp { public: bool OnInit() override { return true; } };
wxIMPLEMENT_APP_NO_MAIN(FixtureApp);
static int checks=0;
static void Check(bool value,const char *why) { ++checks;if(!value)throw std::runtime_error(why); }

int main(int argc,char **argv) {
  if(!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit())return 2;
  int result=0;
  try {
    S52color ink{};ink.R=120;ink.G=135;ink.B=140;
    wxFont font(12,wxFONTFAMILY_SWISS,wxFONTSTYLE_ITALIC,wxFONTWEIGHT_NORMAL,false,"Arial");
    wxBitmap bitmap(1000,800,24);wxMemoryDC dc(bitmap);
    dc.SetFont(font);
    for(double content:{1.,2.}) for(double userScale:{.75,1.,1.5})
    for(double dip:{1.,.8,2./3.,.5}) for(double rotation:{0.,.4,1.5707963267948966}) {
      fontContentScale=content;
      font.SetPointSize(12*content);
      s52plib owner;owner.m_dipfactor=dip;owner.m_ContentScaleFactor=content;
      owner.m_TextScaleFactor=userScale;owner.vp_plib.rotation=rotation;
      S52_TextC name;name.frmtd="SEA NAME";name.pFont=&font;name.pcol=&ink;
      name.letter_spacing=5;name.text_opacity=92;name.xoffs=1;name.yoffs=1;
      name.hjust='1';name.vjust='1';dc.GetTextExtent("X",&name.avgCharWidth,nullptr);
      wxRect glRect,softwareRect;
      Check(owner.RenderText(nullptr,&name,450,350,&glRect,nullptr,true),"GL name should draw");
      Check(owner.RenderText(&dc,&name,450,350,&softwareRect,nullptr,true),"Software name should draw");
      Check(glRect.GetSize()==softwareRect.GetSize(),"GL ink bounds must match software at every DPI");
      Check(std::abs(glRect.x-softwareRect.x)<=2&&std::abs(glRect.y-softwareRect.y)<=2,
            "GL/native baseline, offsets and rotated collision coordinates must agree");
      Check(name.text_width<=name.RGBA_width&&name.text_height<=name.RGBA_height,"Glyph bounds fit the unscaled texture quad");
      int maximumAlpha=0;
      for(std::size_t i=0;i<uploaded.size();i+=4) {
        Check(uploaded[i]==userInk.Red()&&uploaded[i+1]==userInk.Green()&&uploaded[i+2]==userInk.Blue(),"Explicit GL user ink preserved");
        maximumAlpha=std::max(maximumAlpha,int(uploaded[i+3]));
      }
      Check(maximumAlpha>0&&maximumAlpha<=92,"GL opacity remains bounded");
      const auto first=name.texobj;
      owner.m_TextScaleFactor=userScale+0.2;
      Check(owner.RenderText(nullptr,&name,450,350,&glRect,nullptr,true),"Scaled cached name should draw");
      Check(name.texobj!=first&&std::find(deleted.begin(),deleted.end(),first)!=deleted.end(),"Scale invalidation releases old texture");
    }
    // LIGHTS uses the same native glyph/halo payload in the actual SW painter
    // and actual GL upload, with an expanded collision footprint.
    fontContentScale=1;userInk=*wxBLACK;
    for (double scale : {1.,1.5,2.}) {
      s52plib owner;owner.m_TextScaleFactor=scale;
      wxFont lightFont(6,wxFONTFAMILY_SWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_NORMAL,false,"Arial");
      wxFont rasterFont=lightFont;rasterFont.SetPointSize(static_cast<int>(6*scale));
      S52_TextC light;light.frmtd="Fl(2) W 10s 25ft 8Nm";light.pFont=&lightFont;light.pcol=&ink;
      light.light_label=true;light.letter_spacing=.12;light.xoffs=2;light.yoffs=-1;
      light.hjust='3';light.vjust='3';dc.SetFont(lightFont);dc.GetTextExtent("X",&light.avgCharWidth,nullptr);
      Check(light.light_raster.Build(rasterFont,light.frmtd,scale,wxColour(104,123,122),wxColour(213,229,229)),"Prepared production light raster");
      wxRect glRect,swRect;
      Check(owner.RenderText(nullptr,&light,450,350,&glRect,nullptr,true),"Actual light GL upload executes");
      const auto& raster=light.light_raster.image;
      Check(uploaded.size()==static_cast<std::size_t>(raster.GetWidth()*raster.GetHeight()*4),"Full halo texture uploaded");
      for(std::size_t i=0;i<uploaded.size()/4;++i) {
        Check(uploaded[i*4]==raster.GetData()[i*3] && uploaded[i*4+1]==raster.GetData()[i*3+1] &&
              uploaded[i*4+2]==raster.GetData()[i*3+2] && uploaded[i*4+3]==raster.GetAlpha()[i],
              "Actual GL upload exactly equals shared software bitmap payload");
      }
      Check(owner.RenderText(&dc,&light,450,350,&swRect,nullptr,true),"Actual software light draw executes");
      Check(glRect==swRect,"Light anchor, offsets and halo bounds agree in SW/GL");
      Check(swRect.width==light.text_width+2*light.light_raster.margin,"Halo included in actual collision bounds");
      const auto firstTexture=light.texobj;
      Check(light.light_raster.Build(rasterFont,light.frmtd,scale,wxColour(173,187,177),wxColour(52,79,89)),"Theme change rebuilds halo and ink");
      Check(owner.RenderText(nullptr,&light,450,350,&glRect,nullptr,true),"Theme change repaints actual GL label");
      Check(light.texobj!=firstTexture && std::find(deleted.begin(),deleted.end(),firstTexture)!=deleted.end(),"Light palette change releases stale texture");
      S52_TextC blocker;blocker.rText=swRect;owner.RegisterText(true,&blocker);
      Check(!owner.RenderText(nullptr,&light,450,350,&glRect,nullptr,true),"GL light overlap is rejected");
      Check(!owner.RenderText(&dc,&light,450,350,&swRect,nullptr,true),"Software light overlap is rejected");
    }
    // Actual RenderText rejection plus the actual caller's registration block:
    // B overlaps visible A; unseen B would overlap otherwise-clear nav label C.
    fontContentScale=1;
    font.SetPointSize(12);
    for(bool styled:{true,false}) {
      s52plib owner;S52_TextC a,b,c;
      b.frmtd="LONG SEA NAME";b.pFont=&font;b.pcol=&ink;b.bspecial_char=true;
      b.letter_spacing=styled?5:0;b.text_opacity=styled?92:255;
      b.xoffs=b.yoffs=0;b.avgCharWidth=8;b.hjust='3';b.vjust='3';
      wxRect bounds;
      Check(owner.RenderText(nullptr,&b,200,200,&bounds,nullptr,false),"Initial measurement should draw");
      a.rText=wxRect(bounds.x-10,bounds.y,20,bounds.height);
      owner.RegisterText(true,&a);
      const bool drawn=owner.RenderText(nullptr,&b,200,200,&bounds,nullptr,true);
      b.rText=bounds;owner.RegisterText(drawn,&b);
      const wxRect nav(bounds.GetRight()-5,bounds.y,20,bounds.height);
      Check(!nav.Intersects(a.rText),"Navigation control does not overlap visible A");
      Check(drawn==!styled,"Styled rejected name reports false; stock behavior retained");
      Check(owner.CheckTextRectList(nav,&c)==!styled,"Invisible styled name must not suppress subsequent navigation label");
      Check(owner.m_textObjList.GetCount()==(styled?1u:2u),"Only actually drawn styled names register");
    }
    dc.SelectObject(wxNullBitmap);fonts.clear();
    std::cout<<checks<<" actual RenderText DPI, overlap, registration, cache and color checks passed\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';result=1;}
  wxTheApp->OnExit();wxEntryCleanup();return result;
}
