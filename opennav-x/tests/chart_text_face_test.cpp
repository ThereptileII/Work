// Source-extracted production font establishment; rendering is not simulated
// as accepted. Actual platform font substitution needs native HDC evidence.
#include <wx/wx.h>
#include <deque>
#include <vector>
#include <iostream>
#include <stdexcept>
#include "integration/ChartTextFace.h"
#include "integration/ChartNameTypography.h"
#include "integration/ChartLightLabel.h"
class App : public wxApp { public: bool OnInit() override { return true; } };
wxIMPLEMENT_APP_NO_MAIN(App);
struct Text { int bsize=12; char weight='5'; wxFont* pFont=nullptr; int avgCharWidth=0; double letter_spacing=0; unsigned char text_opacity=255; };
struct Call { int size; wxFontFamily family; wxFontStyle style; wxFontWeight weight; bool underline; wxString face; };
static std::vector<Call> calls;
static std::deque<wxFont> owned;
static wxFont configured;
static int getterCalls=0, getterSize=-1, failure=0;
static wxString selected;
wxFont* GetOCPNScaledFont_PlugIn(wxString name, int size=0) {
  if(name!=_("ChartTexts")) throw std::runtime_error("Unexpected template getter");
  ++getterCalls;getterSize=size;return &configured;
}
wxFont* FindOrCreateFont_PlugIn(int size,wxFontFamily family,wxFontStyle style,
                               wxFontWeight weight,bool underline=false,const wxString& face={}) {
  calls.push_back({size,family,style,weight,underline,face});
  if(!selected.empty() && face==selected && failure==1) return nullptr;
  if(!selected.empty() && face==selected && failure==2) owned.emplace_back();
  else owned.emplace_back(size,family,style,weight,underline,face);
  return &owned.back();
}
class Renderer {
  wxString m_presentationTextFace;
 public:
#include "chart-text-face-setter.inc"
  void Establish(Text* text,wxFont* styled=nullptr,double tracking=0,unsigned char opacity=255,
                 const char* feature="BOYLAT",const char* instruction="OBJNAM,",bool bTX=true) {
    struct Object { const char* FeatureName; } object{feature};
    struct Rz { Object* obj; } rz{&object}; auto* rzRules=&rz;
    struct Rule { const char* INSTstr; } rule{instruction}; auto* rules=&rule;
#include "chart-text-font-block.inc"
  }
};
static int count=0;
void Check(bool pass) {++count;if(!pass)throw std::runtime_error("chart font check "+std::to_string(count));}
int main(int argc,char** argv) {
  if(!wxEntryStart(argc,argv))return 2;
  wxTheApp->CallOnInit();
  try {
    using opennav::integration::SelectPrototypeChartTextFace;
    for(int mask=0;mask<4;++mask) {
      std::vector<std::string> probes;
      auto face=SelectPrototypeChartTextFace([&](const char* name){probes.push_back(name);return std::string(name)=="Segoe UI"?(mask&1):(mask&2);});
      Check(face==((mask&1)?"Segoe UI":(mask&2)?"Arial":""));
      Check(probes.size()==((mask&1)?1u:2u) && probes[0]=="Segoe UI");
      if(!(mask&1))Check(probes[1]=="Arial");
    }
    auto actual=opennav::integration::PrototypeChartTextFace();
    const wxString expected=wxFontEnumerator::IsValidFacename("Segoe UI")?"Segoe UI":wxFontEnumerator::IsValidFacename("Arial")?"Arial":"";
    Check(actual==expected);Check(opennav::integration::PrototypeChartTextFace()==actual);
    std::cout<<"Installed chart face policy: "<<(actual.empty()?"original template":actual.ToStdString())<<"\n";
    configured=wxFont(19,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_ITALIC,wxFONTWEIGHT_BOLD,true,"Monospace");
    auto original=configured.GetNativeFontInfoDesc();
    for(char weight:{'3','5','7'})for(int body:{8,12,16,20}) {
      Renderer standard,skager,legacy;skager.SetPresentationTextFace("Segoe UI");
      Text a;a.bsize=body;a.weight=weight;
      calls.clear();getterCalls=0;standard.Establish(&a);auto baseline=calls;const auto baselineGetter=getterSize;
      Check(a.pFont && a.pFont->IsOk() && getterCalls==1 && baseline.size()==2);
      auto stock=a.pFont;const auto oldAverage=a.avgCharWidth;
      Text b;b.bsize=body;b.weight=weight;selected="Segoe UI";
      calls.clear();getterCalls=0;skager.Establish(&b);
      Check(getterCalls==1 && getterSize==baselineGetter && calls.size()==3);
      for(int i=0;i<2;++i)Check(calls[i].size==baseline[i].size && calls[i].family==baseline[i].family && calls[i].style==baseline[i].style && calls[i].weight==baseline[i].weight && calls[i].underline==baseline[i].underline && calls[i].face==baseline[i].face);
      Check(calls[2].size==baseline[1].size && calls[2].family==baseline[1].family && calls[2].style==baseline[1].style && calls[2].weight==baseline[1].weight && calls[2].underline==baseline[1].underline && calls[2].face=="Segoe UI");
      Check(b.pFont && b.pFont!=stock && b.pFont->IsOk() && b.avgCharWidth==oldAverage && b.letter_spacing==0 && b.text_opacity==255);
      auto cached=b.pFont;calls.clear();getterCalls=0;skager.Establish(&b);Check(calls.empty() && getterCalls==0 && b.pFont==cached);
      Text c;c.bsize=body;c.weight=weight;calls.clear();legacy.Establish(&c);Check(calls.size()==2 && calls.back().face==baseline.back().face);
      for(failure=1;failure<=2;++failure){Text fallback;fallback.bsize=body;fallback.weight=weight;calls.clear();skager.Establish(&fallback);Check(calls.size()==3 && fallback.pFont && fallback.pFont->IsOk() && fallback.pFont->GetNativeFontInfoDesc()==a.pFont->GetNativeFontInfoDesc());}failure=0;
      for(bool valid:{false,true}) {
        wxFont styled=valid?wxFont(9,wxFONTFAMILY_SWISS,wxFONTSTYLE_ITALIC,wxFONTWEIGHT_LIGHT,false,"Serif"):wxFont();
        Text special;calls.clear();skager.Establish(&special,&styled,5,92);Check(calls.size()==2);
        if(valid)Check(special.pFont==&styled && special.letter_spacing==5 && special.text_opacity==92);
        else Check(special.pFont && special.pFont->IsOk() && special.letter_spacing==0 && special.text_opacity==255);
      }
      Check(configured.GetNativeFontInfoDesc()==original);
    }
    Renderer roleGuard;roleGuard.SetPresentationTextFace("Segoe UI");
    for(const char* feature:{"BUAARE","LNDARE","LNDRGN","SEAARE"}) {
      Text t;calls.clear();roleGuard.Establish(&t,nullptr,0,255,feature,"OBJNAM,",true);
      Check(calls.size()==2 && calls.back().face==configured.GetFaceName());
    }
    for(const char* tail:{"',3,3,3,'15110',2,-1,CHBLK,23)","',3,2,3,'15110',2,0,CHBLK,23)","',3,2,3,'15110',2,1,CHBLK,23)"}) {
      const std::string rule="'Fl G5s"+std::string(tail);Text t;calls.clear();
      roleGuard.Establish(&t,nullptr,0,255,"LIGHTS",rule.c_str(),true);
      Check(calls.size()==2 && calls.back().face==configured.GetFaceName());
    }
    for(bool tx:{false,true}) {
      Text t;calls.clear();roleGuard.Establish(&t,nullptr,0,255,"LIGHTS",tx?"OBJNAM,":"'%03.0lf',ORIENT,",tx);
      Check(calls.size()==3 && calls.back().face=="Segoe UI");
    }
    Renderer fallback;fallback.SetPresentationTextFace({});Text t;calls.clear();fallback.Establish(&t);Check(calls.size()==2 && calls.back().face==configured.GetFaceName());
    std::cout<<count<<" actual font-establishment checks passed\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
  wxTheApp->OnExit();wxEntryCleanup();return 0;
}
