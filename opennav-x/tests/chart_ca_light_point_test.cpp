// Actual shared classifier and extracted core/private CA wrapper/point method.
// Projection and terminal painters are recorded boundaries, not canvas proof.
#include <wx/wx.h>
#include <chrono>
#include <deque>
#include <iostream>
#include <map>
#include <limits>
#include <memory>
#include <stdexcept>
#include "integration/ChartCaLightPoint.h"
// Exercise the actual nothrow allocation boundary without a production hook.
static bool failNextNothrowAllocation = false;
static unsigned nothrowAllocations = 0;
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  ++nothrowAllocations;
  if (failNextNothrowAllocation) { failNextNothrowAllocation=false; return nullptr; }
  try { return ::operator new(size); } catch (const std::bad_alloc&) { return nullptr; }
}
void operator delete(void* memory, const std::nothrow_t&) noexcept {
  ::operator delete(memory);
}
using namespace opennav::integration;
S57Obj::S57Obj() : Primitive_type(GEO_POINT), att_array(nullptr), attVal(nullptr), n_attr(0), x(0), y(0), m_chart_context(nullptr) { geoPtz=nullptr;geoPtMulti=nullptr;npt=0;bIsClone=false;m_lat=m_lon=0; std::memcpy(FeatureName,"LIGHTS",7); }
S57Obj::~S57Obj() {}
static wxString _LITDSN01(S57Obj*) { return wxString(); }
#define LISTSIZE 20
#include "ca-conditional.inc"
class s52plib {
 public:
  bool m_presentationLightSymbols=true, m_useGLSL=false;
  CaLightPointInventory* m_presentationCaLights=nullptr;
  wxDC* m_pdc=nullptr;
  std::map<wxString,Rule*> dictionary;
  std::map<wxString,Rule*>* _symb_sym=&dictionary;
  int arcs=0, points=0, projected=0;
  Rule* painted=nullptr;
  std::string order;
#include "scope-methods.inc"
  int RenderCARC(ObjRazRules*,Rules*);
  void RenderPresentationCaLightPoint(ObjRazRules*);
  int RenderCARC_GLSL(ObjRazRules*,Rules*) {++arcs;order+='G';return 73;}
  int RenderCARC_VBO(ObjRazRules*,Rules*) {++arcs;order+='D';return 41;}
  void GetPointPixSingle(ObjRazRules*,double,double,wxPoint* point) {++projected;*point=wxPoint(101,202);}
  bool RenderRasterSymbol(ObjRazRules*,Rule* rule,wxPoint& point,float angle) {
    if(point!=wxPoint(101,202)||angle!=0)throw std::runtime_error("projection/angle");
    painted=rule;++points;order+='P';return true;
  }
};
#include "ca-methods.inc"
static unsigned checks=0;
void Check(bool ok) {++checks;if(!ok)throw std::runtime_error("CA check "+std::to_string(checks));}
void SetInst(wxString& destination,wxString& owner,const wxString& value) {owner=value;destination=owner;}
void SetInst(wxString*& destination,wxString& owner,const wxString& value) {owner=value;destination=&owner;}
struct Light : S57Obj {
  wxArrayOfS57attVal values;
  std::deque<S57attVal> entries;
  std::deque<double> numbers;
  std::deque<std::string> strings;
  std::string names;
  wxString instruction;
  LUPrec lookup{};
  ObjRazRules node{};
  Light(chart_context* chart,const char* color="1",double range=20) {
    attVal=&values;m_chart_context=chart;lookup.RCID=31183;lookup.TNAM=SIMPLIFIED;
    std::memcpy(lookup.OBCL,"LIGHTS",7);SetInst(lookup.INST,instruction,"CS(LIGHTS05)\037");
    node.obj=this;node.LUP=&lookup;String("COLOUR",color);Number("VALNMR",range);
  }
  void Add(const char* name,void* value,OGRatt_t type) {names+=name;att_array=names.data();++n_attr;entries.push_back({value,type});values.Add(&entries.back());}
  void String(const char* name,const char* value) {strings.emplace_back(value);Add(name,strings.back().data(),OGR_STR);}
  void Number(const char* name,double value) {numbers.push_back(value);Add(name,&numbers.back(),OGR_REAL);}
};
struct Fog : S57Obj {
  wxString instruction;
  LUPrec lookup{};
  ObjRazRules node{};
  Rules rule{};
  Rule symbol{};
  char ruleText[12]="FOGSIG01)\037";
  char selector[8]="CATFOG1";
  wxArrayOfS57attVal values;
  S57attVal attribute{};
  std::string name, value="1";
  Fog(chart_context* chart) {
    std::memcpy(FeatureName,"FOGSIG",7);m_chart_context=chart;
    lookup.RCID=31164;lookup.FTYP=POINT_T;lookup.DPRI=PRIO_SYMB_AREA;
    lookup.RPRI=RAD_OVER;lookup.TNAM=SIMPLIFIED;lookup.DISC=STANDARD;
    lookup.LUCM=27080;std::memcpy(lookup.OBCL,"FOGSIG",7);
    SetInst(lookup.INST,instruction,"SY(FOGSIG01)\037");
    node.obj=this;node.LUP=&lookup;
    rule.ruleType=RUL_SYM_PT;rule.INSTstr=ruleText;rule.razRule=&symbol;
    symbol.RCID=1338;std::memcpy(symbol.name.SYNM,"FOGSIG01",8);
    symbol.definition.SYDF='R';symbol.pos.symb.bnbox_w.SYHL=12;
    symbol.pos.symb.bnbox_h.SYVL=13;symbol.pos.symb.pivot_x.SYCL=15;
    symbol.pos.symb.pivot_y.SYRW=-3;
  }
  void Attribute(const char* field) {
    name=field;att_array=name.data();n_attr=1;attVal=&values;
    attribute.value=value.data();attribute.valType=OGR_STR;values.Add(&attribute);
  }
};
int main() {
 try {
  chart_context chart{},other{};ObjRazRules* heads[2][2]{};Rules original{};
  s52plib library;Rule alias{},originalRule{};alias.definition.SYDF='R';std::memcpy(alias.name.SYNM,"XNLIT013",8);library.dictionary["XNLIT013"]=&alias;
  original.razRule=&originalRule;const Rule saved=originalRule;
  Light white(&chart);heads[0][0]=&white.node;
  {CaLightPointScope<s52plib> scope(&library,heads,0);Check(library.m_presentationCaLights->size()==1);
    library.m_pdc=reinterpret_cast<wxDC*>(1);Check(library.RenderCARC(&white.node,&original)==41);
    Check(library.order=="DP"&&library.painted==&alias&&library.projected==1);
    library.RenderCARC(&white.node,&original);Check(library.points==1);}
  Check(library.m_presentationCaLights==nullptr&&original.razRule==&originalRule&&!std::memcmp(&saved,&originalRule,sizeof saved));
  // Actual wrapper returns are retained; empty legacy GL gets no new point.
  library.m_pdc=nullptr;library.order.clear();
  {CaLightPointScope<s52plib> scope(&library,heads,0);Check(library.RenderCARC(&white.node,&original)==41);Check(library.order=="D");}
#ifdef ocpnUSE_GL
  library.m_useGLSL=true;library.order.clear();
  {CaLightPointScope<s52plib> scope(&library,heads,0);Check(library.RenderCARC(&white.node,&original)==73);Check(library.order=="GP");}
#endif
  // Short-lived nested scopes restore ownership, including exception exit.
  {CaLightPointScope<s52plib> outer(&library,heads,0);auto* prior=library.m_presentationCaLights;
    try {CaLightPointScope<s52plib> inner(&library,heads,1);Check(library.m_presentationCaLights==nullptr);throw 1;}catch(int){}
    Check(library.m_presentationCaLights==prior);}
  const unsigned beforeDisabled = nothrowAllocations;
  library.m_presentationLightSymbols=false;{CaLightPointScope<s52plib> scope(&library,heads,0);Check(library.m_presentationCaLights==nullptr);}library.m_presentationLightSymbols=true;
  {CaLightPointScope<s52plib> paper(&library,heads,1);Check(!library.m_presentationCaLights);}
  Check(nothrowAllocations==beforeDisabled);
  {CaLightPointScope<s52plib> outer(&library,heads,0);auto* prior=library.m_presentationCaLights;
    [&] {CaLightPointScope<s52plib> inner(&library,heads,0);
      Check(library.m_presentationCaLights && library.m_presentationCaLights!=prior);
      Check(library.m_presentationCaLights->Take(&white)!=nullptr);
      return;
    }();
    Check(library.m_presentationCaLights==prior && prior->size()==1);
    failNextNothrowAllocation=true;
    {CaLightPointScope<s52plib> failed(&library,heads,0);
      Check(!failNextNothrowAllocation && !library.m_presentationCaLights);
      const int before=library.points;
      library.RenderCARC(&white.node,&original);
      Check(library.points==before);
    }
    Check(library.m_presentationCaLights==prior && prior->size()==1);
    Check(prior->Take(&white)!=nullptr);
  }
  Check(library.m_presentationCaLights==nullptr);
  failNextNothrowAllocation=true;
  {CaLightPointScope<s52plib> failed(&library,heads,0);Check(!library.m_presentationCaLights);}
  Check(!failNextNothrowAllocation && !library.m_presentationCaLights);
  // Actual pinned conditional emits CA for admitted records, and its original
  // colour/angle/range instruction is identical before and after inventory.
  for(const char* color:{"1","3","4","6"}) {
    Light actual(&chart,color);heads[0][0]=&actual.node;
    auto* before=static_cast<char*>(LIGHTS06(&actual.node));std::string expected(before);free(before);
    Check(expected.find(";CA(OUTLW, 4,")==0);
    {CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==1);}
    auto* after=static_cast<char*>(LIGHTS06(&actual.node));Check(expected==after);free(after);
    actual.Number("SECTR1",350.);actual.Number("SECTR2",15.);
    auto* sectorText=static_cast<char*>(LIGHTS06(&actual.node));Check(std::string(sectorText).find(";CA(OUTLW, 4,")==0);free(sectorText);
    {CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==1);}
  }
  heads[0][0]=&white.node;
  // Pinned multipoint soundings are GEO_POINT containers; their scalar x/y
  // are not initialized by SetMultipointGeometry. Never inspect those scalars.
  {S57Obj sounding;std::memcpy(sounding.FeatureName,"SOUNDG",7);
  sounding.m_chart_context=&chart;sounding.npt=2;
  double xyz[6]={0,0,4,1,1,5},positions[4]={0,0,1,1};
  sounding.geoPtz=xyz;sounding.geoPtMulti=positions;
  sounding.x=0;sounding.y=std::numeric_limits<double>::quiet_NaN();
  ObjRazRules soundingNode{};soundingNode.obj=&sounding;white.node.next=&soundingNode;
  {CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==1);Check(inventory.Take(&white)!=nullptr);}
  // The second walk must also ignore the parent even when unused scalars happen
  // to coincide with a light. The actual depth arrays remain byte-identical.
  sounding.y=0;
  {CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==1);}
  Check(xyz[2]==4 && xyz[5]==5 && positions[3]==1);
  for(int bad=0;bad<5;++bad) {
    sounding.geoPtz=xyz;sounding.geoPtMulti=positions;sounding.npt=2;sounding.bIsClone=false;
    std::memcpy(sounding.FeatureName,"SOUNDG",7);
    if(bad==0)sounding.geoPtz=nullptr;
    if(bad==1)sounding.geoPtMulti=nullptr;
    if(bad==2)sounding.npt=0;
    if(bad==3)sounding.bIsClone=true;
    if(bad==4)std::memcpy(sounding.FeatureName,"LNDMRK",7);
    sounding.y=std::numeric_limits<double>::quiet_NaN();
    {CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==0);}
    sounding.y=0;
    {CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==0);}
  }
  white.node.next=nullptr;}
  // A real co-located sector group paints only after its last member.
  Light sector(&chart);sector.Number("SECTR1",350);sector.Number("SECTR2",15);white.node.next=&sector.node;
  {CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==1);Check(!inventory.Take(&white));Check(inventory.Take(&sector));}
  // Each otherwise-eligible record retains its original CA. A mixed group
  // uses the prototype generic location point, never the last record's colour.
  for(const char* color:{"3","4","6"}) {
    Light mixed(&chart,color);mixed.Number("SECTR1",10.);mixed.Number("SECTR2",40.);
    white.node.next=&mixed.node;
    auto* before=static_cast<char*>(LIGHTS06(&mixed.node));std::string ca(before);free(before);
    {CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==1);Check(!inventory.Take(&white));Check(std::string(inventory.Take(&mixed))=="XNLIT013");}
    auto* after=static_cast<char*>(LIGHTS06(&mixed.node));Check(ca==after);free(after);
    mixed.node.next=&white.node;white.node.next=nullptr;heads[0][0]=&mixed.node;
    {CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==1);Check(!inventory.Take(&mixed));Check(std::string(inventory.Take(&white))=="XNLIT013");}
    heads[0][0]=&white.node;
  }
  for(const char* color:{"1,3","12"}) {Light mixed(&chart,color);white.node.next=&mixed.node;CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==0);}
  // Exact fog lookup is allowed before and after normal lazy parsing; no
  // parser is invoked by inventory. Invalid existing chains still refuse.
  for(bool loaded:{false,true}) {Fog fog(&chart);if(loaded)fog.lookup.ruleList=&fog.rule;
    white.node.next=&fog.node;CaLightPointInventory inventory(true,heads,0);
    Check(inventory.size()==1);Check(!inventory.Take(&fog));Check(inventory.Take(&white));}
  for(int bad=0;bad<18;++bad) {
    Fog fog(&chart);white.node.next=&fog.node;
    switch(bad) {
      case 0:fog.lookup.RCID=30363;break;
      case 1:fog.lookup.TNAM=PAPER_CHART;break;
      case 2:fog.lookup.FTYP=AREAS_T;break;
      case 3:fog.lookup.DPRI=PRIO_HAZARDS;break;
      case 4:fog.lookup.RPRI=RAD_SUPP;break;
      case 5:fog.lookup.DISC=OTHER;break;
      case 6:fog.lookup.LUCM=0;break;
      case 7:fog.lookup.OBCL[0]='?';break;
      case 8:SetInst(fog.lookup.INST,fog.instruction,"SY(FOGSIG01);SY(QUESMRK1)\037");break;
      case 9:fog.lookup.ATTArray.push_back(fog.selector);break;
      case 10:fog.lookup.ruleList=&fog.rule;fog.rule.razRule=nullptr;break;
      case 11:fog.lookup.ruleList=&fog.rule;fog.rule.next=&fog.rule;break;
      case 12:fog.lookup.ruleList=&fog.rule;fog.symbol.definition.SYDF='V';break;
      case 13:fog.lookup.ruleList=&fog.rule;fog.symbol.pos.symb.pivot_x.SYCL=0;break;
      case 14:fog.lookup.ruleList=&fog.rule;fog.symbol.name.SYNM[0]='?';break;
      case 15:fog.lookup.ruleList=&fog.rule;fog.rule.ruleType=RUL_CND_SY;break;
      case 16:fog.lookup.ruleList=&fog.rule;fog.rule.b_private_razRule=true;break;
      case 17:fog.lookup.ruleList=&fog.rule;fog.ruleText[8]=',';break;
    }
    CaLightPointInventory inventory(true,heads,0);Check(!inventory.size());
  }
  for(const char* field:{"ORIENT","STATUS","QUAPOS","QUASOU"}) {Fog fog(&chart);fog.Attribute(field);white.node.next=&fog.node;CaLightPointInventory inventory(true,heads,0);Check(!inventory.size());}
  {Fog fog(&chart);fog.Attribute("CATFOG");white.node.next=&fog.node;CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==1);}
  Light structure(&chart);std::memcpy(structure.FeatureName,"LNDMRK",7);white.node.next=&structure.node;{CaLightPointInventory inventory(true,heads,0);Check(!inventory.size());}
  white.node.next=nullptr;
  for(const char* name:{"ORIENT","CATLIT","LITVIS","STATUS","QUAPOS"}) {Light refused(&chart);refused.String(name,"1");heads[0][0]=&refused.node;CaLightPointInventory inventory(true,heads,0);Check(!inventory.size());}
  for(double angle:{0.,72.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {Light refused(&chart);refused.Number("ORIENT",angle);heads[0][0]=&refused.node;CaLightPointInventory inventory(true,heads,0);Check(!inventory.size());}
  {Light duplicate(&chart);duplicate.String("COLOUR","1");heads[0][0]=&duplicate.node;CaLightPointInventory inventory(true,heads,0);Check(!inventory.size());}
  {Light partial(&chart);partial.Number("SECTR1",90);heads[0][0]=&partial.node;CaLightPointInventory inventory(true,heads,0);Check(!inventory.size());}
  for(double range:{-1.,0.,9.,std::numeric_limits<double>::infinity()}) {Light shortLight(&chart,"1",range);heads[0][0]=&shortLight.node;CaLightPointInventory inventory(true,heads,0);Check(!inventory.size());}
  for(int bad:{0,1,2}) {Light light(&chart);if(bad==0)light.lookup.RCID=30382;if(bad==1)light.lookup.TNAM=PAPER_CHART;if(bad==2)light.entries[0].valType=OGR_INT;heads[0][0]=&light.node;CaLightPointInventory inventory(true,heads,0);Check(!inventory.size());}
  heads[0][0]=&white.node;white.node.next=&white.node;{CaLightPointInventory inventory(true,heads,0);Check(!inventory.size());}white.node.next=nullptr;
  Light independent(&other);heads[1][0]=&independent.node;{CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==2);}heads[1][0]=nullptr;
  // Sounding multipoints do not turn every chart into an unavailable inventory.
  Light sounding(&chart);sounding.Primitive_type=GEO_META;std::memcpy(sounding.FeatureName,"SOUNDG",7);white.node.next=&sounding.node;{CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==1);}white.node.next=nullptr;
  // Missing/bad aliases never alter the original CA result or dictionary.
  library.m_pdc=reinterpret_cast<wxDC*>(1);library.m_useGLSL=false;
  for(int bad:{0,1,2,3}) {library.dictionary["XNLIT013"]=&alias;alias.definition.SYDF='R';alias.name.SYNM[0]='X';if(bad==0)library.dictionary.erase("XNLIT013");if(bad==1)library.dictionary["XNLIT013"]=nullptr;if(bad==2)alias.definition.SYDF='V';if(bad==3)alias.name.SYNM[0]='?';const int before=library.points;const auto size=library.dictionary.size();CaLightPointScope<s52plib> scope(&library,heads,0);Check(library.RenderCARC(&white.node,&original)==41);Check(library.points==before&&library.dictionary.size()==size);}
  // Bound worst admitted workload; one overflow refuses the entire decoration.
  std::vector<std::unique_ptr<Light>> many;many.reserve(CaLightPointInventory::kMaxObjects+1);
  for(unsigned i=0;i<=CaLightPointInventory::kMaxObjects;++i){many.emplace_back(new Light(&chart));many.back()->x=i;if(i)many[i-1]->node.next=&many[i]->node;}
  heads[0][0]=&many.front()->node;many[many.size()-2]->node.next=nullptr;
  many[511]->node.next=nullptr;
  const auto typicalStart=std::chrono::steady_clock::now();{CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==512);}
  const auto typicalMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-typicalStart).count();
  for(unsigned i=0;i<512;++i)std::memcpy(many[i]->FeatureName,"LNDMRK",7);
  const auto emptyStart=std::chrono::steady_clock::now();{CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==0);}
  const auto emptyMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-emptyStart).count();
  for(unsigned i=0;i<512;++i)std::memcpy(many[i]->FeatureName,"LIGHTS",7);
  many[511]->node.next=&many[512]->node;
  const auto start=std::chrono::steady_clock::now();{CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==CaLightPointInventory::kMaxObjects);}const auto elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  many[many.size()-2]->node.next=&many.back()->node;{CaLightPointInventory inventory(true,heads,0);Check(inventory.size()==0);}
  std::cout<<"checks="<<checks<<" maxInventoryMs="<<elapsed<<" typical512Ms="<<typicalMs<<" noLights512Ms="<<emptyMs<<" objects="<<CaLightPointInventory::kMaxObjects<<"\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
