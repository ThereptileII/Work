#include "integration/SettingsStore.h"
#include <wx/init.h>
#include <wx/filename.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace opennav;
void Check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
class FailingConfig : public wxFileConfig {
 public:
  FailingConfig(const wxString& path):wxFileConfig("","",path,"",wxCONFIG_USE_LOCAL_FILE){}
  int fail=0;
  bool Flush(bool current=false) override { if(fail>0){--fail;return false;}return wxFileConfig::Flush(current); }
};
int main(){wxInitializer wx; if(!wx)return 1;
  const auto file=wxFileName::CreateTempFileName("boat-setup-test");
  try {
    {
      FailingConfig config(file); config.fail=1;
      integration::SettingsStore store(config,false);
      Check(store.SetupState()==application::BoatSetupState::ExistingProfile && config.fail==1 &&
          !config.HasEntry("/OpenNav/BoatSetupV1"),"Existing profile inspection must not write or flush");
      config.fail=0;
    }
    {
      FailingConfig config(file);integration::SettingsStore store(config,true);
      Check(store.SetupState()==application::BoatSetupState::Pending,"Fresh starts pending");
      config.Write("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR",7.0);
      config.Write("/OpenNav/Unrelated","preserve me");
      auto configured=store.Read();configured.energy.battery_device_id="existing-pack";
      configured.current=vessel::CurrentConvention::PositiveCharge;
      configured.pilot={"can0","c0508700e76004d2",true};
      const auto seeded=store.Save(configured);if(!seeded.ok)throw std::runtime_error(seeded.message);
      application::BoatSetupDraft draft;draft.settings=store.Read();draft.vessel_name="Test boat";
      draft.safety_depth_m=4;draft.settings.hazard.draft_m=2;draft.settings.energy.battery.capacity_kwh=24;
      draft.settings.pilot={}; // Setup has no authority to alter this setting.
      draft.display.scale_percent=125;
      config.fail=1;
      Check(!store.SaveBoatSetup(draft).ok,"Failed flush must not complete setup");
      Check(store.SetupState()==application::BoatSetupState::Pending && store.VesselName().empty(),"In memory rolled back");
      Check(config.ReadDouble("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR",0)==7,"Chart restored");
      Check(store.SaveBoatSetup(draft).ok,"Explicit completion saved");
      Check(store.Read().pilot.permit_control && store.Read().pilot.name==configured.pilot.name,"Existing pilot permission/identity preserved");
      Check(store.Read().energy.battery_device_id=="existing-pack" && store.Read().current==configured.current,"Source/sign preserved");
      Check(config.Read("/OpenNav/Unrelated","")=="preserve me","Unrelated profile unchanged");
    }
    {
      FailingConfig config(file);integration::SettingsStore store(config,false);
      Check(store.SetupState()==application::BoatSetupState::Complete,"Completion survives restart/update");
      Check(store.VesselName()=="Test boat" && store.Display().scale_percent==125,"Preferences persisted");
      Check(store.RequestBoatSetup().ok && store.VesselName()=="Test boat","Explicit reset preserves configuration");
    }
    {
      FailingConfig config(file);integration::SettingsStore store(config,false);
      Check(store.SetupState()==application::BoatSetupState::Pending,"Reset resumes after restart");
    }
    wxRemoveFile(file);std::cout<<"Boat setup persistence, rollback and preservation passed\n";
  }catch(const std::exception& e){wxRemoveFile(file);std::cerr<<e.what()<<'\n';return 1;}
}
