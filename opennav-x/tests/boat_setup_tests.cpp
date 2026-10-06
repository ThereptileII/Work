#include "application/BoatSetup.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
void Check(bool ok,const char* why) { if(!ok) throw std::runtime_error(why); }
bool Has(const std::vector<std::string>& rows,const std::string& text) {
  return std::any_of(rows.begin(),rows.end(),[&](const auto& row){return row.find(text)!=std::string::npos;});
}
int main() { try {
  using application::BoatSetupState;
  Check(application::ReadBoatSetupState({},true)==BoatSetupState::Pending,"New profile starts setup");
  Check(application::ReadBoatSetupState({},false)==BoatSetupState::ExistingProfile,"Older profile is never reset by update");
  for(bool fresh:{false,true}) {
    Check(application::ReadBoatSetupState("v1|pending",fresh)==BoatSetupState::Pending,"Interrupted/reset setup resumes");
    Check(application::ReadBoatSetupState("v1|complete",fresh)==BoatSetupState::Complete,"Completion persists across version changes");
    Check(application::ReadBoatSetupState("v2|complete",fresh)==BoatSetupState::Invalid,"Unknown record cannot trigger reset");
  }
  application::BoatSetupDraft draft;
  application::ValidateBoatSetup(draft);
  Check(!draft.settings.pilot.permit_control,"Fresh control permission OFF");
  Check(Has(application::BoatSetupSummary(draft),"Unconfigured"),"Missing assumptions explicit");
  draft.settings.hazard.draft_m=2; draft.safety_depth_m=1;
  bool rejected=false;
  try { application::ValidateBoatSetup(draft); } catch(const std::invalid_argument&) { rejected=true; }
  Check(rejected,"Unsafe depth relationship rejected");
  draft.safety_depth_m=3;application::ValidateBoatSetup(draft);
  const auto now=vessel::Clock::now();vessel::VesselState state;
  Check(Has(application::BoatSetupSensorSummary(state,{},now),"GPS: No data"),"GPS not invented");
  vessel::SourceHealth source{};source.quantity=vessel::Quantity::Depth;source.source_id="test-depth";
  source.sample={4,"test-depth",now,vessel::Validity::Measured,{2s,5s}};source.selected=true;
  auto original=source.sample.observed_at;
  Check(Has(application::BoatSetupSensorSummary(state,{source},now),"Current"),"Fresh observation detected");
  Check(Has(application::BoatSetupSensorSummary(state,{source},now+6s),"Stale"),"Old source not called live");
  source.sample.validity=vessel::Validity::Estimated;
  Check(Has(application::BoatSetupSensorSummary(state,{source},now),"Estimated"),"Estimate distinguished");
  source.sample.validity=vessel::Validity::Invalid;
  Check(Has(application::BoatSetupSensorSummary(state,{source},now),"Invalid"),"Invalid distinguished");
  state.replayed=true;
  Check(!Has(application::BoatSetupSensorSummary(state,{source},now),"test-depth"),"Replay excluded from live inventory");
  Check(source.sample.observed_at==original,"Read does not renew observation");
  std::cout<<"Boat setup policy, validation and sensor evidence passed\n";
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;} }
