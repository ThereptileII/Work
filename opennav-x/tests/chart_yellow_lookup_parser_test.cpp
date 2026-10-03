// Real pinned ProcessLookups + BuildLookup; only the owning containers and
// S57 object construction are fixtures. No canvas, profile or plugin is loaded.
#include <wx/wx.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include "integration/ChartYellowBuoySymbol.h"
#include "pugixml.hpp"
#include "yellow-lookup-type.inc"
WX_DEFINE_SORTED_ARRAY(LUPrec*, wxArrayOfLUPrec);
static int Compare(LUPrec* a, LUPrec* b) { return a->RCID-b->RCID; }
class s52plib {
 public:
  wxArrayPtrVoid allocations;
  wxArrayPtrVoid* pAlloc=&allocations;
  wxArrayOfLUPrec lookups{Compare};
  wxArrayOfLUPrec* SelectLUPARRAY(LUPname) { return &lookups; }
  void DestroyLUP(LUPrec*) { throw std::runtime_error("Unexpected duplicate fixture lookup"); }
};
class ChartSymbols {
 public:
  s52plib* plib=nullptr;
  void ProcessLookups(pugi::xml_node&);
  void BuildLookup(Lookup&);
};
#include "yellow-lookup-methods.inc"
S57Obj::S57Obj() : att_array(nullptr), attVal(nullptr), n_attr(0), x(0), y(0) {}
S57Obj::~S57Obj() {}
static int checks=0;
static void Check(bool value) {
  ++checks;
  if (!value) throw std::runtime_error("Yellow parser check " + std::to_string(checks));
}
static const wxString& Instruction(const wxString& value) { return value; }
static const wxString& Instruction(const wxString* value) { Check(value); return *value; }
struct Point : S57Obj {
  wxArrayOfS57attVal values;
  int shape;
  char color[2]={'6',0};
  char names[13];
  S57attVal attrs[2];
  Point(bool topmark,chart_context* context) : shape(topmark?7:4) {
    Primitive_type=GEO_POINT;std::memcpy(FeatureName,topmark?"TOPMAR":"BOYSPP",7);
    std::memcpy(names,topmark?"TOPSHPCOLOUR":"BOYSHPCOLOUR",13);
    attrs[0]={&shape,OGR_INT};attrs[1]={color,OGR_STR};
    values.Add(&attrs[0]);values.Add(&attrs[1]);
    att_array=names;attVal=&values;n_attr=2;m_chart_context=context;
  }
};
int main(int argc,char** argv) {
  try {
    Check(argc==2);
    pugi::xml_document source;
    Check(source.load_file(argv[1]));
    pugi::xml_node selected;
    for (auto lookup:source.child("chartsymbols").child("lookups").children("lookup"))
      if (lookup.attribute("RCID").as_int()==31314) {Check(!selected);selected=lookup;}
    Check(selected && !std::strcmp(selected.attribute("name").value(),"TOPMAR"));
    Check(!std::strcmp(selected.child_value("table-name"),"Simplified"));
    Check(selected.child("instruction") && !*selected.child_value("instruction"));
    chart_context context{};wxArrayPtrVoid platforms;context.pFloatingATONArray=&platforms;
    Point buoy(false,&context),top(true,&context);platforms.Add(&buoy);
    for (const char* xmlInstruction:{"", "SY(TOPMAR65)", "CS(TOPMAR01)", " "}) {
      pugi::xml_document document;
      auto lookups=document.append_child("lookups");
      auto lookup=lookups.append_copy(selected);
      lookup.child("instruction").text().set(xmlInstruction);
      s52plib owner;ChartSymbols parser;parser.plib=&owner;parser.ProcessLookups(lookups);
      Check(owner.lookups.GetCount()==1);
      auto* lup=owner.lookups.Item(0);
      const auto& value=Instruction(lup->INST);
      Check(value==wxString::FromUTF8(xmlInstruction)+wxString('\037'));
      Check(!lup->ruleList && lup->ATTArray.empty());
      ObjRazRules rz{};rz.obj=&top;rz.LUP=lup;
      const bool empty=!*xmlInstruction;
      Check(opennav::integration::YellowEmptyInstruction(value)==empty);
      Check(opennav::integration::YellowEmptyInstruction(&value)==empty);
      Check(opennav::integration::PresentationYellowTopmark(true,&rz)==empty);
      Check(!opennav::integration::PresentationYellowTopmark(false,&rz));
      Rules rule{};lup->ruleList=&rule;
      Check(!opennav::integration::PresentationYellowTopmark(true,&rz));lup->ruleList=nullptr;
      lup->TNAM=PAPER_CHART;Check(!opennav::integration::PresentationYellowTopmark(true,&rz));lup->TNAM=SIMPLIFIED;
      lup->RCID=31539;Check(!opennav::integration::PresentationYellowTopmark(true,&rz));lup->RCID=31314;
      top.shape=6;Check(!opennav::integration::PresentationYellowTopmark(true,&rz));top.shape=7;
      platforms.Clear();Check(!opennav::integration::PresentationYellowTopmark(true,&rz));platforms.Add(&buoy);
      platforms.Add(&buoy);Check(!opennav::integration::PresentationYellowTopmark(true,&rz));platforms.RemoveAt(1);
#ifdef PRIVATE_LOOKUP
      delete lup->INST;
      lup->ATTArray.~vector();std::free(lup);
#else
      delete lup;
#endif
    }
    Check(!opennav::integration::YellowEmptyInstruction(static_cast<const wxString*>(nullptr)));
    for (const char* bad:{"\037\037", "\037SY(TOPMAR65)", "SY(TOPMAR65)\037", "\036", "\n", ";"}) {
      wxString value=wxString::FromUTF8(bad);
      Check(!opennav::integration::YellowEmptyInstruction(value));
      Check(!opennav::integration::YellowEmptyInstruction(&value));
    }
    wxString empty;
    Check(opennav::integration::YellowEmptyInstruction(empty));
    Check(opennav::integration::YellowEmptyInstruction(&empty));
    std::cout<<checks<<" yellow parser checks passed\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
