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
struct TexFontCache { TexFont *cache=nullptr; wxFont *key=nullptr; };
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
// Deterministic glyph metrics at the dependency boundary, taken from the
// retained LNDELV trace. Cache selection, ownership and all positioning execute
// the actual extracted RenderText body; no positioning algorithm is copied.
static bool glyphCacheFixture=false;
static int glyphBuilds=0, metricQueries=0;
TexFont::TexFont() {} TexFont::~TexFont() {}
void TexFont::Build(wxFont&,double,double,bool) {
  if(!glyphCacheFixture)throw std::runtime_error("Unexpected glyph-cache branch");
  ++glyphBuilds;
}
void TexFont::GetTextExtent(const wxString& text,int* width,int* height) {
  if(!glyphCacheFixture)throw std::runtime_error("Unexpected glyph-cache branch");
  if(text=="M") { ++metricQueries;if(width)*width=16;if(height)*height=22; }
  else if(text=="26.2") { if(width)*width=38;if(height)*height=22; }
  else throw std::runtime_error("Unexpected glyph input");
}

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
  const bool cacheOnly=argc==2 && std::string(argv[1])=="--glyph-cache-only";
  if(!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit())return 2;
  int result=0;
  try {
    S52color ink{};ink.R=120;ink.G=135;ink.B=140;
    wxFont font(12,wxFONTFAMILY_SWISS,wxFONTSTYLE_ITALIC,wxFONTWEIGHT_NORMAL,false,"Arial");
    wxBitmap bitmap(1000,800,24);wxMemoryDC dc(bitmap);
    dc.SetFont(font);
    // Real cache/text phases: each new text represents the ordinary text cache
    // invalidation on a theme change. The atlas remains owned by this renderer.
    glyphCacheFixture=true;
    for(double dip:{1.,.8,.5}) for(bool stockInk:{false,true}) {
      s52plib owner;owner.m_dipfactor=dip;owner.m_useS52DefaultTextColor=!stockInk;
      owner.m_FinalTextScaleFactor=owner.m_TextScaleFactor/dip;
      wxRect first;
      auto* originalFont=&font;
      const int buildsBefore=glyphBuilds;
      for(int phase=0;phase<4;++phase) {
        S52_TextC text;text.frmtd="26.2";text.pFont=originalFont;text.pcol=&ink;
        text.avgCharWidth=9;text.xoffs=1;text.yoffs=-1;
        text.hjust='3';text.vjust='2';
        wxRect bounds;
        const int queriesBefore=metricQueries;
        Check(owner.RenderText(nullptr,&text,957,129,&bounds,nullptr,false),"Ordinary glyph text draws in each cache phase");
        Check(glyphBuilds==buildsBefore+1,"Theme text recreation reuses the existing atlas");
        if(phase==0)first=bounds;
        Check(bounds==first,"Day/Dusk/Night/Day text phases keep identical placement");
        Check(metricQueries==queriesBefore+1,"Every text obtains the existing atlas metric");
        Check(text.avgCharWidth==static_cast<int>(16*dip),"Cache hit retains the original first-draw metric");
        if(dip==1)Check(bounds==wxRect(973,108,38,22),"Original traced first-draw rectangle is preserved");
      }
      // Different font identity must still allocate a separate cache entry.
      wxFont secondFont=font;
      S52_TextC other;other.frmtd="26.2";other.pFont=&secondFont;other.pcol=&ink;
      other.avgCharWidth=9;other.xoffs=1;other.yoffs=-1;other.hjust='3';other.vjust='2';
      wxRect otherBounds;
      Check(owner.RenderText(nullptr,&other,957,129,&otherBounds,nullptr,false),"Different font renders");
      Check(glyphBuilds==buildsBefore+2&&otherBounds==first,"Separate atlas miss preserves equivalent geometry");
      for(auto& entry:owner.s_txf) { delete entry.cache;entry.cache=nullptr;entry.key=nullptr; }
    }
    glyphCacheFixture=false;
    if(!cacheOnly) {
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
    }
    dc.SelectObject(wxNullBitmap);fonts.clear();
    std::cout<<checks<<" actual RenderText DPI, overlap, registration, cache and color checks passed\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';result=1;}
  wxTheApp->OnExit();wxEntryCleanup();return result;
}
