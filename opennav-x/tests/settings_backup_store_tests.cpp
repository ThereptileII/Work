#include "integration/SettingsStore.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <wx/init.h>
#include <wx/sstream.h>
#include <wx/filename.h>
using namespace opennav;
void Check(bool ok,const char *why) { if(!ok)throw std::runtime_error(why); }
struct FailConfig : wxFileConfig {
  explicit FailConfig(wxInputStream &s):wxFileConfig(s){}
  int calls=0, writes=0, fail_write_at=0; bool fail=false, fail_all_flush=false;
  bool Flush(bool=false) override { ++calls; return !fail_all_flush && !(fail && calls==1); }
  bool DoWriteString(const wxString &key,const wxString &value) override {
    if (++writes==fail_write_at) return false;
    return wxFileConfig::DoWriteString(key,value);
  }
};
application::SettingsBackup Backup() {
  application::SettingsBackup b;
  b.vessel_name="Restored boat"; b.settings.energy.battery.capacity_kwh=48;
  b.display={150,application::ChartLayout::InstrumentFocus}; b.chart_safety_depth_m=4.5;
  return b;
}
int main() {
  try {
    wxInitializer init; Check(init.IsOk(),"wx init failed");
    for (const bool existing:{false,true}) {
      wxStringInputStream input(""); FailConfig config(input);
      application::Settings old; old.energy.battery.capacity_kwh=24;
      old.pilot={"local-interface","c0508700e76004d2",true};
      const auto old_record=application::EncodeSettings(old);
      if (existing) {
        config.Write("/OpenNav/AlphaSettings",wxString::FromUTF8(old_record));
        config.Write("/OpenNav/VesselName","Original boat");
        config.Write("/OpenNav/DisplayPreferencesV1","v1|125|chart");
        config.Write("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR",2.5);
      }
      config.Write("/Settings/TestChartPath","existing-charts");
      config.Write("/Settings/ApiKey","private-secret-never-exported");
      config.Write("/OpenNav/InterfaceMode","legacy");
      integration::SettingsStore store(config);
      const int before_writes=config.writes;
      auto invalid=Backup(); invalid.display.scale_percent=999;
      Check(!store.RestoreBackup(invalid).ok && config.calls==0 && config.writes==before_writes,"Invalid import touched storage");
      invalid=Backup(); invalid.settings.pilot=old.pilot;
      Check(!store.RestoreBackup(invalid).ok && config.calls==0 && config.writes==before_writes,"Imported pilot permission touched storage");
      config.fail=true;
      Check(!store.RestoreBackup(Backup()).ok && config.calls==2,"Flush failure did not rollback");
      Check(config.HasEntry("/OpenNav/AlphaSettings")==existing &&
          config.HasEntry("/OpenNav/VesselName")==existing &&
          config.HasEntry("/OpenNav/DisplayPreferencesV1")==existing &&
          config.HasEntry("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR")==existing,"Rollback changed entry existence");
      if(existing) Check(store.Read().energy.battery.capacity_kwh==24 && store.Read().pilot.permit_control &&
          store.VesselName()=="Original boat" && store.Display().scale_percent==125 &&
          config.Read("/OpenNav/AlphaSettings","")==wxString::FromUTF8(old_record),"Failed restore changed original values");
      config.fail=false;
      Check(store.RestoreBackup(Backup()).ok,"Valid restore failed");
      Check(store.Read().energy.battery.capacity_kwh==48 && store.VesselName()=="Restored boat" &&
          store.Display().scale_percent==150 && !store.Read().pilot.permit_control,"Restored state incorrect");
      if(existing) Check(store.Read().pilot.interface_id=="local-interface","Local pilot binding replaced");
      Check(config.Read("/Settings/TestChartPath","")=="existing-charts" &&
          config.Read("/Settings/ApiKey","")=="private-secret-never-exported" &&
          config.Read("/OpenNav/InterfaceMode","")=="legacy","Unrelated data changed");
      const auto exported=application::EncodeSettingsBackup({store.Read(),store.Display(),store.VesselName(),4.5});
      Check(exported.find("private-secret")==std::string::npos && exported.find("existing-charts")==std::string::npos &&
          exported.find("local-interface")==std::string::npos,"Excluded configuration exported");
      auto missing_depth=Backup(); missing_depth.chart_safety_depth_m=std::nan("");
      Check(store.RestoreBackup(missing_depth).ok,"Missing depth restore failed");
      double depth=0; config.Read("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR",&depth);
      Check(depth==4.5,"Missing chart depth replaced existing value");
    }
    for (int fail_at=1;fail_at<=4;++fail_at) {
      wxStringInputStream input(""); FailConfig config(input);
      config.Write("/OpenNav/VesselName","Original boat");
      integration::SettingsStore store(config);
      config.writes=0; config.fail_write_at=fail_at;
      Check(!store.RestoreBackup(Backup()).ok,"Partial write unexpectedly succeeded");
      Check(config.Read("/OpenNav/VesselName","")=="Original boat" &&
          !config.HasEntry("/OpenNav/AlphaSettings") && !config.HasEntry("/OpenNav/DisplayPreferencesV1") &&
          !config.HasEntry("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR") &&
          store.VesselName()=="Original boat","Partial-write rollback lost original profile");
    }
    {
      wxStringInputStream input(""); FailConfig config(input);
      integration::SettingsStore store(config); config.fail_all_flush=true;
      const auto result=store.RestoreBackup(Backup());
      Check(!result.ok && result.message.find("rollback failed")!=std::string::npos &&
          store.VesselName().empty(),"Rollback failure hidden or live state changed");
    }
    const auto path=wxFileName::CreateTempFileName("skager-backup-test");
    {
      wxFileConfig config("","",path,"",wxCONFIG_USE_LOCAL_FILE);
      integration::SettingsStore store(config);
      Check(store.RestoreBackup(Backup()).ok,"Disk restore failed");
    }
    {
      wxFileConfig config("","",path,"",wxCONFIG_USE_LOCAL_FILE);
      integration::SettingsStore store(config);
      Check(store.VesselName()=="Restored boat" && store.Display().scale_percent==150 &&
          store.Read().energy.battery.capacity_kwh==48 && !store.Read().pilot.permit_control,"Reopened profile differs");
    }
    Check(wxRemoveFile(path),"Could not remove temporary profile");
    std::cout<<"Backup store validation, rollback, preservation and disk persistence checks passed\n";
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
