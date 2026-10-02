#include "ais/ChartTargets.h"
#include "ais/Provider.h"
#include "ui/Theme.h"
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace opennav;
using namespace std::chrono_literals;
namespace {
unsigned checks = 0;
void Check(bool ok, const char *reason) {
  ++checks; if (!ok) throw std::runtime_error(reason);
}
}
int main() {
  try {
    const vessel::Time at{1000s};
    ais::TargetCache cache;
    Check(cache.Observe({265000001,58.1,179.99,6.3,43,41,0,at},at),
          "Actual normalized cache supplies presentation input");
    const auto state = cache.Read(at);
    const auto get = [&](const vessel::AisState &s, vessel::Time now) {
      return ais::OnlineChartTargets(s,now,265000001);
    };
    auto glyphs=get(state,at);
    Check(glyphs.size()==1&&glyphs[0].selected&&glyphs[0].direction_true==41,
          "Current measured heading and selected identity retained");
    Check(glyphs[0].longitude==179.99,"No independent geodesic/projection at data boundary");
    for(const auto &p:std::vector<std::pair<vessel::Duration,ais::TargetAge>>{
      {15s,ais::TargetAge::Aging},{60s,ais::TargetAge::Stale},{120s,ais::TargetAge::Lost}}) {
      glyphs=get(state,at+p.first);
      Check(glyphs.size()==1&&glyphs[0].age==p.second,"Retained position ages without provider/UI renewal");
      if(p.first>=60s)Check(!glyphs[0].selected&&!glyphs[0].direction_true,
        "Stale/lost marks cannot imply current heading or current selection");
    }
    Check(get(state,at+600s).empty(),"Expired targets disappear");
    Check(get(state,at-1ms).empty(),"Future position refused");
    Check(get(state,vessel::Time::max()).empty(),"Extreme clock cannot overflow");
    auto s=state;s.targets[0].heading_true_deg={};
    Check(get(s,at)[0].direction_true==43,"Valid underway COG used when heading absent");
    s.targets[0].sog_kn.value=0;
    Check(!get(s,at)[0].direction_true,"Stopped vessel cannot acquire invented orientation");
    s.targets[0].sog_kn={};
    Check(!get(s,at)[0].direction_true,"Unknown motion remains unoriented");
    for(int invalid=0;invalid<15;++invalid) {
      s=state;auto &t=s.targets[0];
      switch(invalid) {
      case 0:t.latitude_deg.value=91;break;
      case 1:t.longitude_deg.value=-181;break;
      case 2:t.latitude_deg.value=std::numeric_limits<double>::quiet_NaN();break;
      case 3:t.longitude_deg.value=std::numeric_limits<double>::infinity();break;
      case 4:t.latitude_deg.value.reset();break;
      case 5:t.latitude_deg.validity=vessel::Validity::Estimated;break;
      case 6:t.longitude_deg.observed_at+=1ms;break;
      case 7:t.latitude_deg.source="unrelated";break;
      case 8:t.observed_at+=1ms;break;
      case 9:t.time_basis=vessel::AisTimeBasis::OpenCPNReport;break;
      case 10:t.mmsi=0;break;
      case 11:t.doubtful=true;break;
      case 12:t.origin=vessel::AisOrigin::LocalOpenCPN;break;
      case 13:t.active=false;break;
      case 14:t.lost=true;break;
      }
      Check(get(s,at).empty(),"Invalid/untrusted display input cannot create a ship");
    }
    s=state;s.simulated=true;Check(get(s,at).empty(),"Simulation cannot enter live chart bridge");
    s=state;s.available=false;Check(get(s,at).empty(),"Unavailable feed remains unavailable");
    s=state;s.targets.push_back(s.targets[0]);Check(get(s,at).empty(),"Ambiguous identity withheld");
    s=state;s.targets.resize(2001,s.targets[0]);Check(get(s,at).empty(),"Bounded workload");
    s=state;s.targets[0].origin=vessel::AisOrigin::LocalOpenCPN;s.targets[0].lost=true;
    Check(get(ais::Aggregate(s,{state,{}}).display,at).empty(),
          "Even lost onboard identity suppresses supplemental online chart symbol");
    s=state;s.targets[0].longitude_deg.value=-179.99;
    Check(get(s,at)[0].longitude==-179.99,"Antimeridian kept for pinned OpenCPN projection");
    const auto retained=get(state,at);cache=ais::TargetCache{};
    Check(retained.size()==1&&retained[0].mmsi==265000001,"Owned presentation survives source destruction");
    const auto painted=ais::CurrentChartMark(retained[0],at+61s);
    Check(painted&&painted->age==ais::TargetAge::Stale&&!painted->selected&&
          !painted->direction_true&&painted->observed_at==at,
          "Paint-time read ages a retained symbol without refreshing it");
    Check(!ais::CurrentChartMark(*painted,at),"Clock reversal cannot rejuvenate stale cached marks");
    Check(!ais::CurrentChartMark(retained[0],at+600s),"Retained render snapshot expires even if producer has stopped");
    ais::TargetCache named;
    Check(named.Observe({265000002,58.1,16.4,4,90,90,0,at},at),"Named fixture has actual normalized position");
    ais::StaticReport identity;identity.mmsi=265000002;identity.name="S/Y Liv";identity.observed_at=at;
    Check(named.Observe(identity,at),"Validated static name accepted");
    const auto first=ais::OnlineChartTargets(named.Read(at),at,265000002);
    Check(first.size()==1&&first[0].name=="S/Y Liv","Available name copied into owned chart mark");
    identity.name="Skärgård";identity.observed_at=at+1s;
    Check(named.Observe(identity,at+1s),"Static identity update accepted");
    const auto renamed=ais::OnlineChartTargets(named.Read(at+1s),at+1s,265000002);
    Check(!(first==renamed)&&renamed[0].name=="Skärgård"&&renamed[0].observed_at==at,
          "Name-only update invalidates rendering without renewing position");
    named=ais::TargetCache{};
    Check(first[0].name=="S/Y Liv","Owned name survives cache destruction and later rename");
    Check(ais::CurrentChartMark(first[0],at+15s)->name=="S/Y Liv","Aging identity remains with existing aging mark");
    Check(ais::CurrentChartMark(first[0],at+60s)->name.empty(),"Retained stale mark cannot keep live name label");
    for(const auto &name:std::vector<std::string>{"","   ","bad\nname",std::string(129,'x')}) {
      s=state;s.targets[0].name=name;
      const auto marks=get(s,at);
      Check(marks.size()==1&&marks[0].name.empty(),"Absent or invalid name omits label without losing target");
    }
    using B=ais::ChartLabelBounds;
    const B viewport{0,0,100,80};
    Check(ais::ChartLabelFits({14,4,30,10},viewport,{}),"Full label fits its fixed anchor");
    Check(!ais::ChartLabelFits({90,4,30,10},viewport,{})&&
          !ais::ChartLabelFits({14,-1,30,10},viewport,{})&&
          !ais::ChartLabelFits({14,75,30,10},viewport,{}),"Clipped labels are omitted, never shifted");
    Check(!ais::ChartLabelFits({14,4,30,10},viewport,{{20,0,10,20}}),"Label cannot obscure another target");
    Check(!ais::ChartLabelFits({14,4,30,10},viewport,{{40,10,30,10}}),"Accepted label blocks overlapping next name");
    Check(ais::ChartLabelFits({14,4,30,10},viewport,{{44,4,20,10}}),"Disjoint labels remain available");
    Check(!ais::ChartLabelFits({14,4,0,10},viewport,{}),"Empty text metrics cannot reserve a label");
    Check(ui::OnlineChartTheme(ui::LightMode::Day).label==0x835D70&&
          ui::OnlineChartTheme(ui::LightMode::Dusk).label==0x835D70&&
          ui::OnlineChartTheme(ui::LightMode::Night).label==0x664957,
          "Final prototype label includes inherited Night brightness only");
    std::cout<<checks<<" online chart presentation checks passed\n";
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
