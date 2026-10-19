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
  b.settings.rail_presets={{"Anchor",{"depth","heading","aws","soc"}}};
  return b;
}
void PresetProfileChecks() {
  const auto presets=Backup().settings.rail_presets;
  application::Settings original;
  original.energy.battery.capacity_kwh=37;
  original.sources[vessel::Quantity::Depth]={"measured-depth",{vessel::Duration{1000},vessel::Duration{4000}}};
  const auto legacy_record=application::EncodeSettings(original);
  auto with_presets=original; with_presets.rail_presets=presets;
  const auto full_record=application::EncodeSettings(with_presets);
  {
    wxStringInputStream input(""); FailConfig config(input);
    config.Write("/OpenNav/AlphaSettings",wxString::FromUTF8(legacy_record));
    integration::SettingsStore store(config);
    Check(store.Read().rail_presets.empty(),"Old profiles invented presets");
    Check(store.Save(with_presets).ok,"Preset save failed");
    // Exact old-compatible core bytes: the installed decoder rejects any new
    // field. Changing presets alone must not change this record at all.
    Check(config.Read("/OpenNav/AlphaSettings","")==wxString::FromUTF8(legacy_record),
          "Presets changed the old-compatible AlphaSettings record");
    Check(application::DecodeRailPresets(config.Read("/OpenNav/RailPresetsV1","").ToStdString(wxConvUTF8))==presets,
          "Separate preset record was not saved");
    integration::SettingsStore reopened(config);
    Check(reopened.Read().rail_presets==presets && reopened.Read().energy.battery.capacity_kwh==37 &&
        reopened.Read().sources.at(vessel::Quantity::Depth).pinned_source=="measured-depth",
        "Reopen lost presets or vessel/source settings");
    config.writes=0;config.fail_write_at=2;
    auto changed=with_presets;changed.energy.battery.capacity_kwh=44;changed.rail_presets.clear();
    Check(!store.Save(changed).ok && store.Read().rail_presets==presets &&
        config.Read("/OpenNav/AlphaSettings","")==wxString::FromUTF8(legacy_record),
        "Failed preset write changed core bytes/live state");
    config.fail_write_at=0;
    integration::SettingsStore rolled_back(config);
    Check(rolled_back.Read().rail_presets==presets,"Failed save lost persisted presets");
    Check(store.SaveVessel(with_presets,"Named boat",std::nan("")).ok,
          "Vessel save with presets failed");
    Check(config.Read("/OpenNav/AlphaSettings","")==wxString::FromUTF8(legacy_record),
          "Vessel save reintroduced incompatible inline presets");
    Check(store.RestoreBackup({original,{},"Old backup",std::nan("")}).ok,
          "Backup without presets could not be restored");
    integration::SettingsStore cleared(config);
    Check(cleared.Read().rail_presets.empty(),"Backup without presets failed to clear prior presets");
  }
  {
    wxStringInputStream input(""); FailConfig config(input);
    config.Write("/OpenNav/AlphaSettings",wxString::FromUTF8(full_record));
    const int writes=config.writes;
    integration::SettingsStore inline_store(config);
    Check(inline_store.Read().rail_presets==presets && config.writes==writes,
          "Reading earlier inline presets lost them or wrote the profile");
    Check(inline_store.Save(inline_store.Read()).ok &&
        config.Read("/OpenNav/AlphaSettings","")==wxString::FromUTF8(legacy_record),
        "Explicit save did not migrate inline presets out of AlphaSettings");
    config.Write("/OpenNav/AlphaSettings",wxString::FromUTF8(full_record));
    config.Write("/OpenNav/RailPresetsV1",wxString::FromUTF8(application::EncodeRailPresets({})));
    integration::SettingsStore empty_wins(config);
    Check(empty_wins.Read().rail_presets.empty(),"Explicit empty preset key resurrected inline presets");
    config.Write("/OpenNav/RailPresetsV1","unsupported or corrupt presets");
    const int before=config.writes;
    integration::SettingsStore corrupt(config);
    Check(corrupt.Read().rail_presets.empty() && corrupt.Read().energy.battery.capacity_kwh==37 &&
        corrupt.Read().sources.at(vessel::Quantity::Depth).pinned_source=="measured-depth" &&
        corrupt.Status().find("rail presets rejected")!=std::string::npos && config.writes==before,
        "Corrupt separate presets damaged vessel settings or wrote the profile");
  }
}
int main() {
  try {
    wxInitializer init; Check(init.IsOk(),"wx init failed");
    PresetProfileChecks();
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
          !config.HasEntry("/OpenNav/RailPresetsV1") &&
          config.HasEntry("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR")==existing,"Rollback changed entry existence");
      if(existing) Check(store.Read().energy.battery.capacity_kwh==24 && store.Read().pilot.permit_control &&
          store.VesselName()=="Original boat" && store.Display().scale_percent==125 &&
          config.Read("/OpenNav/AlphaSettings","")==wxString::FromUTF8(old_record),"Failed restore changed original values");
      config.fail=false;
      Check(store.RestoreBackup(Backup()).ok,"Valid restore failed");
      Check(store.Read().energy.battery.capacity_kwh==48 && store.VesselName()=="Restored boat" &&
          store.Display().scale_percent==150 && !store.Read().pilot.permit_control &&
          store.Read().rail_presets==Backup().settings.rail_presets,"Restored state incorrect");
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
    for (int fail_at=1;fail_at<=5;++fail_at) {
      wxStringInputStream input(""); FailConfig config(input);
      config.Write("/OpenNav/VesselName","Original boat");
      integration::SettingsStore store(config);
      config.writes=0; config.fail_write_at=fail_at;
      Check(!store.RestoreBackup(Backup()).ok,"Partial write unexpectedly succeeded");
      Check(config.Read("/OpenNav/VesselName","")=="Original boat" &&
          !config.HasEntry("/OpenNav/AlphaSettings") && !config.HasEntry("/OpenNav/DisplayPreferencesV1") &&
          !config.HasEntry("/OpenNav/RailPresetsV1") &&
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
          store.Read().energy.battery.capacity_kwh==48 && !store.Read().pilot.permit_control &&
          store.Read().rail_presets==Backup().settings.rail_presets,"Reopened profile differs");
    }
    Check(wxRemoveFile(path),"Could not remove temporary profile");
    std::cout<<"Backup store validation, rollback, preservation and disk persistence checks passed\n";
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
