// Actual pinned conditional bodies and private bridge; only chart query results,
// object construction and mariner settings are fixtures. No geometry is invented.
#include <wx/wx.h>
#include <wx/list.h>
#include <list>
#include <memory>
#include <map>
#include <deque>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <type_traits>
#include "s52s57.h"
#include "s52utils.h"
#ifdef PATCHED_PRIVATE
// Separate translation unit keeps the replacement allocation pair out-of-line.
extern int failAllocation;
extern void* trackedQueryList;
extern bool queryListDeleted;
#endif
static int checks=0;
static bool requireRepair=false;
template<class T,class=void>struct HasCallback:std::false_type{};
template<class T>struct HasCallback<T,std::void_t<decltype(T::get_associated_objects)>>:std::true_type{};
#ifdef PATCHED_PRIVATE
static_assert(HasCallback<chart_context>::value,"Owned context has internal callback");
#else
static_assert(!HasCallback<chart_context>::value,"Stock/host context must not acquire callback");
#endif
static void Check(bool b) {++checks;if(!b)throw std::runtime_error("association check "+std::to_string(checks));}
class s52plib {public:int m_nDepthUnitDisplay=1;};
static s52plib* ps52plib;
double S52_getMarinerParam(S52_MAR_param_t) {return 5.;}
#ifdef PRIVATE_SOURCE
S57Obj::S57Obj(){Init();}
#else
S57Obj::S57Obj():att_array(nullptr),attVal(nullptr),n_attr(0),x(0),y(0){}
#endif
S57Obj::~S57Obj(){}
#ifdef PRIVATE_SOURCE
WX_DECLARE_LIST(S57Obj,ListOfS57Obj);
#include <wx/listimpl.cpp>
WX_DEFINE_LIST(ListOfS57Obj);
class eSENCChart {
 public:
  std::list<S57Obj*> associated;
  int calls=0;
  bool unavailable=false,fail=false;
  int failConversionAllocation=0;
  ListOfS57Obj* GetAssociatedObjects(S57Obj*) {
    ++calls;
    if(fail)throw std::runtime_error("query unavailable");
    if(unavailable)return nullptr;
    auto list=new ListOfS57Obj;
    for(auto* obj:associated)list->Append(obj);
#ifdef PATCHED_PRIVATE
    if(failConversionAllocation){trackedQueryList=list;queryListDeleted=false;failAllocation=failConversionAllocation;}
#endif
    return list;
  }
};
using TestChart=eSENCChart;
#ifdef PATCHED_PRIVATE
#include "bridge.inc"
#endif
#else
class s57chart {
 public:
  std::list<S57Obj*> associated;
  std::list<S57Obj*>* Associated(S57Obj*) {return new std::list<S57Obj*>(associated);}
};
using TestChart=s57chart;
#endif
#define UNKNOWN 1e6
#define LISTSIZE 32
#include "conditional.inc"
#include "private_hazard_cases.inc"

#ifdef PATCHED_PRIVATE
static void CheckBridgeBoundary() {
  HazardObject rock("UWTROC"),area("DEPARE");
  rock.Int("WATLEV",3);area.Primitive_type=GEO_AREA;area.Real("DRVAL1",10);
  rock.chart.associated.push_back(&area);
  auto invoke=[&]() {
    ObjRazRules rz{};rz.obj=&rock;
    std::unique_ptr<wxString> result(_UDWHAZ03(&rock,0,&rz,nullptr));
    return *result;
  };
  Check(invoke().Contains("ISODGR51") && rock.m_DisplayCat==DISPLAYBASE && rock.chart.calls==1);
  rock.m_DisplayCat=OTHER;
  auto* context=rock.m_chart_context;
  rock.m_chart_context=nullptr;Check(invoke().empty() && rock.m_DisplayCat==OTHER && rock.chart.calls==1);
  rock.m_chart_context=context;context->chart=nullptr;Check(invoke().empty() && rock.chart.calls==1);
  context->chart=&rock.chart;context->get_associated_objects=nullptr;Check(invoke().empty() && rock.chart.calls==1);
  context->get_associated_objects=&SkagerAssociatedObjects;
  TestChart other;
  Check(SkagerAssociatedObjects(&other,&rock)==nullptr && other.calls==0 && rock.chart.calls==1);
  Check(SkagerAssociatedObjects(nullptr,&rock)==nullptr && SkagerAssociatedObjects(&rock.chart,nullptr)==nullptr);
  context->get_associated_objects=nullptr;
  Check(SkagerAssociatedObjects(&rock.chart,&rock)==nullptr && rock.chart.calls==1);
  context->get_associated_objects=&SkagerAssociatedObjects;
  rock.chart.unavailable=true;Check(invoke().empty() && rock.m_DisplayCat==OTHER);rock.chart.unavailable=false;
  rock.chart.associated.clear();Check(invoke().empty() && rock.m_DisplayCat==OTHER);
  rock.chart.associated.push_back(&area);rock.chart.fail=true;
  bool threw=false;try{invoke();}catch(const std::runtime_error& e){threw=std::string(e.what())=="query unavailable";}
  Check(threw && rock.m_DisplayCat==OTHER);rock.chart.fail=false;
  // The returned container owns no chart objects and survives only this call.
  {std::unique_ptr<std::list<S57Obj*>> list(SkagerAssociatedObjects(&rock.chart,&rock));Check(list && list->size()==1 && list->front()==&area);}
  Check(area.n_attr==1 && rock.chart.associated.front()==&area);
  // A synchronous stack copy, as in the private sounding renderer, borrows context.
  S57Obj clone=rock;clone.bIsClone=true;
  {std::unique_ptr<std::list<S57Obj*>> list(SkagerAssociatedObjects(&rock.chart,&clone));Check(list && list->front()==&area);}
  Check(invoke().Contains("ISODGR51"));
  for(int allocation:{1,2}) {
    rock.m_DisplayCat=OTHER;rock.chart.failConversionAllocation=allocation;
    bool oom=false;try{invoke();}catch(const std::bad_alloc&){oom=true;}
    Check(oom && queryListDeleted && rock.m_DisplayCat==OTHER && area.n_attr==1);
  }
  rock.chart.failConversionAllocation=0;
  Check(invoke().Contains("ISODGR51"));
  std::cout<<"null/no-callback/nonowned/mismatched/empty/unavailable/error and synchronous borrowed lifetime checks passed; unavailable is NOT safe qualification\n";
}
#endif
int main(int argc,char**argv) {
 try {
  requireRepair=argc>1 && std::string(argv[1])=="require-repair";
  std::cout<<"context layout "<<sizeof(chart_context)<<" "<<offsetof(chart_context,chart_scale)<<"\n";
  s52plib owner;CheckHazardConditionals(owner);
#ifdef PATCHED_PRIVATE
  {S57Obj fresh;Check(fresh.m_chart_context==nullptr);}
  CheckBridgeBoundary();
#endif
  std::cout<<checks<<" focused checks passed\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
