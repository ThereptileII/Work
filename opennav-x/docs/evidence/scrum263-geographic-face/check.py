from pathlib import Path
import hashlib,json,subprocess,shlex,os
r=Path.cwd();o=r/'.local/geographic-face';wx=Path('/home/standard/Projects/X-nav/.local/sysroot/usr');env={**os.environ,'LD_LIBRARY_PATH':str(wx/'lib')};conf=[str(wx/'bin/wx-config'),'--prefix='+str(wx)];flags=shlex.split(subprocess.check_output(conf+['--cxxflags'],text=True));libs=shlex.split(subprocess.check_output(conf+['--libs','core,base'],text=True))
def body(s):
 a=s.index('wxFont *GeographicNameFont(');i=s.index('{',a)+1;depth=1
 while depth:depth+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[a:i]
head=r'''#include <wx/wx.h>
#include <iostream>
#include <stdexcept>
#include "integration/ChartNameTypography.h"
#include "integration/ChartLightLabel.h"
using namespace opennav::integration;
struct App:wxApp {bool OnInit()override{return true;}};wxIMPLEMENT_APP_NO_MAIN(App);
int mask,checks=0,calls=0;bool custom=false;wxString requested;int points;wxFontStyle style;
struct ProbeEnum {static bool IsValidFacename(const char* x){return std::string(x)=="Segoe UI Variable Display"?mask&1:std::string(x)=="Segoe UI"?mask&2:mask&4;}};
wxFont*Normal(){static wxFont font=*wxNORMAL_FONT;return &font;}
struct FontMgr {static FontMgr&Get(){static FontMgr f;return f;}wxFont*GetFont(wxString){static wxFont changed;changed=*wxNORMAL_FONT;changed.SetPointSize(changed.GetPointSize()+1);return custom?&changed:Normal();}wxColour GetFontColor(wxString){return *wxBLACK;}wxFont*FindOrCreateFont(int p,wxFontFamily,wxFontStyle s,wxFontWeight,bool,wxString f){++calls;requested=f;points=p;style=s;return Normal();}};
wxFont*GetOCPNScaledFont_PlugIn(wxString name,int){return FontMgr::Get().GetFont(name);}wxColour GetFontColour_PlugIn(wxString name){return FontMgr::Get().GetFontColor(name);}wxFont*FindOrCreateFont_PlugIn(int p,wxFontFamily f,wxFontStyle s,wxFontWeight w,bool u,wxString face){return FontMgr::Get().FindOrCreateFont(p,f,s,w,u,face);}
#define wxFontEnumerator ProbeEnum
'''
tail=r'''
#undef wxFontEnumerator
void Check(bool ok){++checks;if(!ok)throw std::runtime_error("resolver check "+std::to_string(checks));}
int main(int argc,char**argv){mask=std::stoi(argv[1]);if(!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit())return 2;try{
 const wxString explicitFace=(mask&2)?"Segoe UI":"Arial",rootFace=(mask&1)?"Segoe UI Variable Display":explicitFace;
 for(const char* feature:{"BUAARE","LNDARE","LNDRGN","SEAARE"}){double track=-1;unsigned char opacity=0;bool light=true;calls=0;auto* font=GeographicNameFont(feature,"OBJNAM,",true,&track,&opacity,&light);const bool water=std::string(feature)=="SEAARE";Check(font && calls==1 && !light);Check(requested==(water?rootFace:explicitFace));Check(points==(water?12:9) && style==(water?wxFONTSTYLE_ITALIC:wxFONTSTYLE_NORMAL));Check(track==(water?5:1) && opacity==(water?92:255));}
 for(bool customized:{false,true}){custom=customized;double t=0;unsigned char opacity=0;bool light=false;calls=0;auto* f=GeographicNameFont("LIGHTS","'Fl G5s',3,3,3,'15110',2,-1,CHBLK,23)",true,&t,&opacity,&light);if(customized)Check(!f && !light && calls==0);else {Check(f && light && calls==1);Check(requested==explicitFace && points==6 && t==.12 && opacity==255);}}
 double t=0;unsigned char opacity=255;bool light=false;calls=0;Check(!GeographicNameFont("BOYLAT","OBJNAM,",true,&t,&opacity,&light) && calls==0);Check(!GeographicNameFont("LNDARE","OBJNAM,",false,&t,&opacity,&light) && calls==0);
 std::cout<<mask<<": "<<checks<<" resolver checks passed\n";
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}wxTheApp->OnExit();wxEntryCleanup();return 0;}
'''
result={}
for name,file in [('core','src/integration/ChartPresentation.cpp'),('private','src/plugin-adapters/ocharts/ChartPresentationAdapter.cpp')]:
 b=body((r/file).read_text());cpp=o/(name+'-fixture.cpp');cpp.write_text(head+b+tail);exe=o/(name+'-fixture');cmd=['c++','-std=c++17','-Wall','-Wextra','-Werror',*flags,'-I'+str(r/'src'),str(cpp),*libs,'-o',str(exe)];subprocess.run(cmd,check=True,env=env);logs=[]
 for mask in range(8):
  run=subprocess.run([str(exe),str(mask)],capture_output=True,text=True,env=env);logs.append(run.stdout+run.stderr);run.check_returncode()
 (o/(name+'-fixture.log')).write_text(''.join(logs));print(name,''.join(logs),flush=True)
 # Original Land root-stack choice is the concrete defect, not a generic false value.
 old=b.replace('*light || role == ChartNameRole::Land ? light_face : face','*light ? light_face : face');assert old!=b;cpp.write_text(head+old+tail);subprocess.run(cmd,check=True,env=env);negative=subprocess.run([str(exe),'3'],capture_output=True,text=True,env=env);assert negative.returncode==1 and 'resolver check' in negative.stderr;(o/(name+'-original-negative.log')).write_text(negative.stdout+negative.stderr);cpp.write_text(head+b+tail)
 result[name]={'source':file,'sourceSha256':hashlib.sha256((r/file).read_bytes()).hexdigest(),'actualResolverSha256':hashlib.sha256(b.encode()).hexdigest(),'availabilityMasks':8,'checksPerMask':21,'originalLandChoiceNegative':negative.returncode,'compileCommand':cmd}
(o/'resolver-receipt.json').write_text(json.dumps(result,indent=2)+'\n')
