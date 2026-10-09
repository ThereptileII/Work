#include "application/NavigationNaming.h"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace opennav::application;
namespace { int checks=0; void Check(bool ok,const char *why) { ++checks; if(!ok)throw std::runtime_error(why); } }
int main() {
 try {
  const Coordinate position{58.123456,16.123456};
  const auto fallback=SuggestNavigationName(position,false,{});
  Check(fallback.name=="Waypoint 58.12346N 16.12346E" && !fallback.from_chart,"coordinate fallback");
  Check(SuggestNavigationName(position,true,{}).name=="Route 58.12346N 16.12346E","route fallback");
  Check(SuggestNavigationName({-1,-2},false,{}).name=="Waypoint 1.00000S 2.00000W","hemispheres");
  Check(SuggestNavigationName({-0.,-0.},false,{}).name=="Waypoint 0.00000N 0.00000E","negative zero");
  const auto nearest=SuggestNavigationName(position,false,{{"Distant",.3},{"Harbour",.1}});
  Check(nearest.from_chart && nearest.name=="Harbour","nearest real named feature");
  Check(SuggestNavigationName(position,true,{{"Harbour",.1}}).name=="To Harbour","route destination");
  // SCRUM-350: a near tie now resolves instead of falling back to coordinates,
  // which is what left real waypoints unnamed wherever two marks sat together.
  Check(SuggestNavigationName(position,false,{{"East",.1},{"West",.11}}).name=="East","near tie takes the nearer");
  Check(SuggestNavigationName(position,false,{{"East",.1},{"West",.1}}).from_chart,"exact tie still names");
  Check(SuggestNavigationName(position,false,{{"East",.1},{"West",.1}}).name ==
        SuggestNavigationName(position,false,{{"West",.1},{"East",.1}}).name,"exact tie is order independent");
  // Type outranks distance: a harbour further off beats a nearer buoy.
  Check(SuggestNavigationName(position,false,{{"Buoy",.05,"BOYLAT"},{"Harbour",.4,"HRBFAC"}}).name=="Harbour",
        "feature priority beats distance");
  Check(SuggestNavigationName(position,false,{{"Outer",.05,"BOYLAT"},{"Inner",.04,"BOYLAT"}}).name=="Inner",
        "equal class falls back to distance");
  Check(SuggestNavigationName(position,false,{{"Harbour",.1},{"Harbour",.11}}).from_chart,"same name chart overlap");
  Check(SuggestNavigationName(position,false,{{"Harbour",.1},{"Other",.2}}).name ==
        SuggestNavigationName(position,false,{{"Other",.2},{"Harbour",.1}}).name,"query order independent");
  Check(!SuggestNavigationName(position,false,{{"Invalid",-1},{"NaN",std::numeric_limits<double>::quiet_NaN()}}).from_chart,"distance bounds");
  // The caller supplies the step it is currently searching; anything beyond it
  // waits for the next, wider step rather than being named from too far away.
  Check(!SuggestNavigationName(position,false,{{"Far",1.2}},1.).from_chart,"outside the current step");
  Check(SuggestNavigationName(position,false,{{"Far",1.2}},2.).name=="Far","inside the wider step");
  Check(ChartNameSearchSteps().size()>1 && ChartNameSearchSteps().front()==1.,"stepped search starts at one mile");
  Check(ChartNameFeatureRank("HRBFAC")<ChartNameFeatureRank("BOYLAT"),"harbour outranks buoy");
  Check(ChartNameFeatureRank("")>ChartNameFeatureRank("BCNSPP"),"unknown class ranks last");
  // SCRUM-350: routes are named by their endpoints.
  Check(SuggestRouteName("Djurholmen","Linkoping").name=="From Djurholmen to Linkoping","route from and to");
  Check(SuggestRouteName("","Linkoping").name=="To Linkoping","route falls back to destination");
  Check(!SuggestRouteName("","").from_chart && SuggestRouteName("","").name=="Route","route fallback without endpoints");
  Check(!SuggestNavigationName(position,true,{{std::string(127,'a'),.1}}).from_chart,"prefix bound");
  Check(RelevantChartNameFeature("HRBFAC") && RelevantChartNameFeature("LIGHTS"),"real relevant classes");
  Check(!RelevantChartNameFeature("DEPARE") && !RelevantChartNameFeature("WRECKS") && !RelevantChartNameFeature("LNDARE"),"exclude areas hazards");
  Check(ValidNavigationName("Göteborg"),"unicode name");
  for(const auto &name : std::vector<std::string>{"","  ","bad\nname",std::string("bad\0name",8),std::string(129,'x'),"\xc0\xaf","\xed\xa0\x80","\xf4\x90\x80\x80"})
    Check(!ValidNavigationName(name),"invalid name rejected");
  NavigationNameDraft draft("Existing user name"); int saves=0; std::string stored="Existing user name";
  const auto save=[&](const std::string &name){++saves; stored=name;return CommandResult{true,"Saved"};};
  Check(!draft.Changed() && draft.Value()==stored,"existing name unchanged");
  draft.Set(nearest.name); draft.Set("User override"); draft.Cancel();
  Check(!draft.Changed() && draft.Value()==stored && saves==0,"cancel never saves");
  draft.Set("User override"); Check(draft.Save(save).ok && stored=="User override" && saves==1 && !draft.Changed(),"user override saved exactly");
  draft.Set("Retained draft"); Check(!draft.Save([](const std::string&){return CommandResult{false,"Database failure"};}).ok && draft.Changed() && draft.Value()=="Retained draft","save failure retains draft");
  draft.Set("  "); Check(!draft.Save(save).ok && saves==1,"invalid never dispatched");
  draft.Set("Stale intent"); Check(!draft.Save([](const std::string&){return CommandResult{false,"Selection changed"};}).ok && draft.Changed(),"stale failure retains visible intent");
  draft.Cancel(); Check(draft.Value()=="User override" && saves==1,"cancel after failed save uses last accepted name");
  std::cout<<checks<<" navigation naming checks passed\n";
 } catch(const std::exception&e) {std::cerr<<e.what()<<'\n';return 1;}
}
