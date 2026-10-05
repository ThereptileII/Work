// Execute the actual sounding painter and DepthFont atlas rasterizer. Native
// wx draws the pixels; recorded GL uploads require no driver or chart context.
#include <wx/wx.h>
#include <wx/dcscreen.h>
#include <wx/dcmemory.h>
#include <GL/gl.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include "s52s57.h"
#include "DepthFont.h"
#include "integration/ChartSoundingFont.h"

struct FixtureViewport { int pix_width=600,pix_height=300; double rotation=0; };
class s52plib {
 public:
  double m_display_size_mm=300,m_dipfactor=1,m_ContentScaleFactor=1;
  double m_SoundingsScaleFactor=1,m_SoundingsFontSizeMM=0,m_soundFontDelta=0;
  int m_SoundingsPointSize=6;
  wxFont *m_soundFont=nullptr;
  wxDC *m_pdc=nullptr;
  FixtureViewport vp_plib;
  DepthFont m_texSoundings;
  using SoundingFontResolver=wxFont (*)(double,double);
  SoundingFontResolver m_soundingFontResolver=nullptr;
  wxFont m_presentationSoundingFont;
  double m_presentationSoundingScale=-1,m_presentationSoundingContent=-1,m_presentationSoundingDip=-1;
  double GetPPMM() {return 96./25.4;}
  std::vector<wxPoint> coordinates;
  void GetPixPointSingle(double x,double y,double *lat,double *lon) {
    coordinates.emplace_back(std::lround(x),std::lround(y));*lat=y;*lon=x;
  }
  void SetSoundingFontResolver(wxFont (*resolver)(double,double));
  bool RenderSoundingSymbol(ObjRazRules*,Rule*,wxPoint&,wxColor,float);
};
static double fontContentScale=1;
static int legacyFontCalls=0;
static std::vector<std::unique_ptr<wxFont>> fonts;
wxFont *FindOrCreateFont_PlugIn(int size,wxFontFamily family,wxFontStyle style,
    wxFontWeight weight,bool underline=false,const wxString& face=wxEmptyString) {
  ++legacyFontCalls;
  fonts.push_back(std::make_unique<wxFont>(fontContentScale*size,family,style,weight,underline,face));
  return fonts.back().get();
}
static GLuint nextTexture=1;
static int uploads=0,deletes=0,atlasWidth=0;
static std::vector<unsigned char> uploaded;
extern "C" {
void glDeleteTextures(GLsizei n,const GLuint*) {deletes+=n;}
void glGenTextures(GLsizei n,GLuint *textures) {while(n--)*textures++=nextTexture++;}
void glBindTexture(GLenum,GLuint) {}
void glTexParameteri(GLenum,GLenum,GLint) {}
void glTexImage2D(GLenum,GLint,GLint,GLsizei w,GLsizei h,GLint,GLenum format,GLenum,const GLvoid *data) {
  if(format!=GL_ALPHA)throw std::runtime_error("Unexpected atlas format");
  ++uploads;atlasWidth=w;
  const auto *bytes=static_cast<const unsigned char*>(data);uploaded.assign(bytes,bytes+w*h);
}
}
#include "sounding-methods.inc"
class FixtureApp:public wxApp { public:bool OnInit()override{return true;} };
wxIMPLEMENT_APP_NO_MAIN(FixtureApp);
static int checks=0;
static void Check(bool ok,const char *why){++checks;if(!ok)throw std::runtime_error(why);}
static int resolverCalls=0;
static wxFont Resolve(double scale,double content){++resolverCalls;return opennav::integration::ChartSoundingFont(scale,content);}
static bool rejectNativeFont=false;
static wxFont FallibleResolve(double scale,double content) {
 ++resolverCalls;
 return rejectNativeFont ? wxNullFont : opennav::integration::ChartSoundingFont(scale,content);
}
int main(int argc,char **argv){
 if(!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit())return 2;
 int result=0;
 wxSetAssertHandler([](const wxString&,int,const wxString&,const wxString&,const wxString&) {
  throw std::runtime_error("Native wx assertion: invalid font must never reach a drawing API");
 });
 try {
  const double prototypePointSize=std::stod(argv[1])*72./96.;
  wxInitAllImageHandlers();
  wxBitmap bitmap(600,300,24);wxMemoryDC dc(bitmap);wxPoint anchor(300,150);
  Rule rule{};std::memcpy(rule.name.SYNM,"SOUNDG00",8);
  for(double content:{1.,2.})for(double dip:{1.,.8,2./3.,.5})
  for(double preference:{.5,1.,1.5,2.,3.}) {
   s52plib owner;owner.m_soundingFontResolver=Resolve;
   owner.m_ContentScaleFactor=content;owner.m_dipfactor=dip;owner.m_SoundingsScaleFactor=preference;
   const double accepted=preference<=2?preference:1;
   const int startCalls=resolverCalls,startLegacy=legacyFontCalls,startUploads=uploads;
   for(int digit=0;digit<10;++digit){
    rule.name.SYNM[7]='0'+digit;
    owner.m_pdc=nullptr;owner.coordinates.clear();
    Check(owner.RenderSoundingSymbol(nullptr,&rule,anchor,wxColour(31,72,110),0),"GL sounding path");
    Check(uploads==startUploads+1,"One cached atlas for all digits");
    Check(resolverCalls==startCalls+1,"One owned font for all digits");
    Check(legacyFontCalls==startLegacy,"Styled raster keeps fractional font, no legacy size conversion");
    Check(std::abs(owner.m_soundFont->GetFractionalPointSize()-prototypePointSize*accepted*content)<.02,"Exact prototype em with preference/content scale");
    Check(owner.m_soundFont->GetWeight()==wxFONTWEIGHT_NORMAL&&owner.m_soundFont->GetStyle()==wxFONTSTYLE_NORMAL,"Normal digit face");
    wxRect tile;owner.m_texSoundings.GetGLTextureRect(tile,digit);
    // Independent native raster of the chosen font must equal every digit's
    // atlas alpha; compare native color channels before any driver rendering.
    wxBitmap one(tile.width,tile.height,24);wxMemoryDC mask(one);
    mask.SetBackground(*wxBLACK_BRUSH);mask.Clear();mask.SetTextForeground(*wxWHITE);mask.SetFont(*owner.m_soundFont);
    mask.DrawText(wxString::Format("%d",digit),0,0);mask.SelectObject(wxNullBitmap);
    auto expected=one.ConvertToImage();int covered=0;
    for(int y=0;y<tile.height;++y)for(int x=0;x<tile.width;++x){
      int alpha=uploaded[(tile.y+y)*atlasWidth+tile.x+x];covered+=alpha;
      Check(alpha==expected.GetRed(x,y),"Actual DepthFont atlas matches software digit raster");
    }
    Check(covered>0,"Digit is visible");
    for(int group=0;group<6;++group){
      rule.name.SYNM[6]='0'+group;owner.m_pdc=&dc;owner.coordinates.clear();
      dc.SetBackground(*wxWHITE_BRUSH);dc.Clear();
      const wxColour ink(31,72,110);
      Check(owner.RenderSoundingSymbol(nullptr,&rule,anchor,ink,0),"Software sounding path");
      Check(dc.GetTextForeground()==ink,"Existing semantic ink retained exactly");
      Check(dc.GetFont()==*owner.m_soundFont,"Software uses same owned font");
      int width,height,descent;dc.GetTextExtent("0",&width,&height,&descent);
      int px=group<4?width*group:group==4?-width:0;
      int py=group<5?(height-descent)/2:(height-descent)/5;
      px*=dip;py*=dip;
      Check(owner.coordinates[0]==wxPoint(anchor.x-px,anchor.y-py+rule.parm3),"Pinned geographic digit-pivot equation retained");
    }
   }
   const int oldUploads=uploads,oldDeletes=deletes;
   owner.m_pdc=nullptr;owner.m_dipfactor=dip*.9;
   owner.RenderSoundingSymbol(nullptr,&rule,anchor,*wxBLACK,0);
   Check(uploads==oldUploads+1&&deletes==oldDeletes+1,"DIP change rebuilds and releases atlas");
   owner.m_ContentScaleFactor=content*1.25;
   owner.RenderSoundingSymbol(nullptr,&rule,anchor,*wxBLACK,0);
   Check(uploads==oldUploads+2&&deletes==oldDeletes+2,"Content change rebuilds and releases atlas");
   owner.m_SoundingsScaleFactor=.8;
   owner.RenderSoundingSymbol(nullptr,&rule,anchor,*wxBLACK,0);
   Check(uploads==oldUploads+3&&deletes==oldDeletes+3,"Preference change rebuilds and releases atlas");
  }
  // No policy means the exact legacy font-selection branch still executes.
  s52plib stock;stock.m_pdc=&dc;int before=legacyFontCalls;
  stock.RenderSoundingSymbol(nullptr,&rule,anchor,*wxBLACK,0);
  Check(legacyFontCalls>before,"Uninstalled policy retains stock selection");
  Check(!stock.m_presentationSoundingFont.IsOk(),"Stock cannot acquire styled cache");
  // A native font can become unavailable after an already-rendered valid
  // presentation. Exercise the actual resolver setter, painter and atlas with
  // the same owner, then prove failed attempts do not churn its stock cache.
  for(bool software:{false,true}) {
   s52plib fallback;fallback.m_pdc=software?&dc:nullptr;
   rejectNativeFont=false;fallback.SetSoundingFontResolver(FallibleResolve);
   fallback.RenderSoundingSymbol(nullptr,&rule,anchor,*wxBLACK,0);
   Check(fallback.m_soundFont==&fallback.m_presentationSoundingFont,"Valid native font uses owned presentation cache");
   rejectNativeFont=true;fallback.m_ContentScaleFactor=1.25;
   const int attempts=resolverCalls,legacyBefore=legacyFontCalls,uploadsBefore=uploads,deletesBefore=deletes;
   std::vector<unsigned char> stockPixels;
   int fallbackLegacyCalls=0;
   for(int digit=0;digit<10;++digit) {
    rule.name.SYNM[7]='0'+digit;
    fallback.RenderSoundingSymbol(nullptr,&rule,anchor,*wxBLACK,0);
    Check(fallback.m_soundFont&&fallback.m_soundFont->IsOk(),"Invalid native font falls back to valid stock font");
    Check(fallback.m_soundFont!=&fallback.m_presentationSoundingFont,"Invalid owned font never reaches drawing/atlas");
    Check(!fallback.m_presentationSoundingFont.IsOk(),"Rejected font is not accepted as presentation");
    Check(resolverCalls==attempts+1,"One failed native font attempt per input tuple");
    Check(legacyFontCalls>legacyBefore,"Invalid resolver executes original stock font path");
    if(!software) {
     if(!digit) {stockPixels=uploaded;fallbackLegacyCalls=legacyFontCalls;}
     Check(uploads==uploadsBefore+1,"Exactly one stock atlas replaces invalidated styled atlas");
     Check(deletes==deletesBefore+1,"Previous styled atlas released exactly once");
     Check(legacyFontCalls==fallbackLegacyCalls,"Stock atlas/font reused across remaining digits");
    }
   }
   if(!software) {
    s52plib reference;reference.m_ContentScaleFactor=fallback.m_ContentScaleFactor;
    reference.RenderSoundingSymbol(nullptr,&rule,anchor,*wxBLACK,0);
    Check(uploaded==stockPixels,"Fallback atlas exactly equals null-policy stock raster");
   }
   fallback.SetSoundingFontResolver(nullptr);const int nullAttempts=resolverCalls;
   fallback.RenderSoundingSymbol(nullptr,&rule,anchor,*wxBLACK,0);
   Check(resolverCalls==nullAttempts&&fallback.m_soundFont->IsOk(),"Null resolver retains valid stock font without native resolution");
   rejectNativeFont=false;fallback.SetSoundingFontResolver(FallibleResolve);
   fallback.RenderSoundingSymbol(nullptr,&rule,anchor,*wxBLACK,0);
   Check(resolverCalls==nullAttempts+1&&fallback.m_presentationSoundingFont.IsOk(),"Explicit reinstall retries identical failed inputs");
   Check(fallback.m_soundFont==&fallback.m_presentationSoundingFont,"Recovered native font replaces stock cache");
  }
  // Review artifact: test digits only, painted by the actual production method.
  dc.SetBackground(*wxWHITE_BRUSH);dc.Clear();
  for(int row=0;row<5;++row) {
    s52plib sample;sample.m_pdc=&dc;
    if(row)sample.m_soundingFontResolver=Resolve;
    sample.m_SoundingsScaleFactor=row==1?.5:row==3?1.5:row==4?2.:1.;
    dc.SetFont(wxFont(10,wxFONTFAMILY_SWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_NORMAL));
    dc.SetTextForeground(*wxBLACK);
    dc.DrawText(row?wxString::Format("SKAGER %.1fx",sample.m_SoundingsScaleFactor):"Pinned default",10,30+row*50);
    for(int digit=0;digit<10;++digit) {
      rule.name.SYNM[6]='0';rule.name.SYNM[7]='0'+digit;
      wxPoint position(175+digit*38,40+row*50);
      sample.RenderSoundingSymbol(nullptr,&rule,position,*wxBLACK,0);
    }
  }
  dc.SelectObject(wxNullBitmap);
  Check(bitmap.SaveFile(wxString::FromUTF8(argv[2]),wxBITMAP_TYPE_PNG),"Native digit fixture saved");
  std::cout<<checks<<" actual sounding painter/atlas assertions passed; 40 scale combinations, 10 digits, 6 pivot groups\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';result=1;}
 fonts.clear();wxTheApp->OnExit();wxEntryCleanup();return result;
}
