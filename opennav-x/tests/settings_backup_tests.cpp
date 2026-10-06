#include "application/SettingsBackup.h"
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace opennav;
void Check(bool ok,const char *message) { if(!ok)throw std::runtime_error(message); }
void Reject(const std::function<void()> &fn) {
  try { fn(); } catch(const std::invalid_argument &) { return; }
  throw std::runtime_error("Invalid backup accepted");
}
std::string WithSettings(const std::string &settings) {
  std::ostringstream out;
  out<<"SKAGER_SETTINGS_BACKUP 1\n\"vessel\" \"Boat\"\n\"settings\" "<<std::quoted(settings)
     <<"\n\"display\" \"v1|100|balanced\"\n\"chart_safety_depth_m\" \"\"\n";
  return out.str();
}
int main() {
  try {
    application::SettingsBackup b;
    b.vessel_name="Båt \"One\"";
    b.settings.energy.battery={24,20,.5,"Calibration"};
    b.settings.energy.curve=smartnav::ImportPowerCurve("OpenNavXPowerCurve,1\nreference,STW\nbasis,whole-pack\nspeed_kn,power_kw\n2,1\n4,3\n","Measured calibration");
    b.settings.sources[vessel::Quantity::Depth]={"sensor-one",{vessel::Duration{1000},vessel::Duration{3000}}};
    b.settings.signal_k_mappings={{"propulsion.main.electricalPower",vessel::Quantity::MotorPower,.001,0}};
    b.settings.pilot={"excluded-interface","c0508700e76004d2",true};
    b.display={125,application::ChartLayout::ChartFocus}; b.chart_safety_depth_m=3.75;
    const auto encoded=application::EncodeSettingsBackup(b);
    Check(encoded.find("pilot.")==std::string::npos && encoded.find("excluded-interface")==std::string::npos,"Pilot permission exported");
    const auto restored=application::DecodeSettingsBackup(encoded);
    Check(restored.vessel_name==b.vessel_name && restored.display.scale_percent==125 &&
        restored.display.layout==b.display.layout && restored.chart_safety_depth_m==3.75 &&
        restored.settings.energy.battery.capacity_kwh==24 && restored.settings.energy.curve.points.size()==2 &&
        restored.settings.sources.at(vessel::Quantity::Depth).pinned_source=="sensor-one" &&
        restored.settings.signal_k_mappings[0].scale==.001 && !restored.settings.pilot.permit_control,"Round trip lost configuration");
    const auto blank=application::DecodeSettingsBackup(application::EncodeSettingsBackup({}));
    Check(std::isnan(blank.settings.energy.battery.capacity_kwh) && std::isnan(blank.chart_safety_depth_m),"Missing values fabricated");
    for(const auto &bad:{std::string{},"SKAGER_SETTINGS_BACKUP 2\n"+encoded,
          encoded+"\"credentials\" \"secret\"\n",encoded+"\"vessel\" \"duplicate\"\n",
          encoded.substr(0,encoded.size()-3),std::string(application::SettingsBackupLimit+1,'x'),
          encoded+std::string(1,'\0'),std::string("{\"format\":\"OpenNavX-design-backup\",\"version\":1}")})
      Reject([&]{application::DecodeSettingsBackup(bad);});
    Reject([&]{application::DecodeSettingsBackup(WithSettings(application::EncodeSettings(b.settings)));});
    auto illegal=restored; illegal.settings.energy.battery.reserve_soc_percent=101;
    Reject([&]{application::EncodeSettingsBackup(illegal);});
    illegal=restored; illegal.vessel_name=std::string("bad\xff",4);
    Reject([&]{application::EncodeSettingsBackup(illegal);});
    illegal=restored; illegal.settings.energy.battery.source=std::string("bad\xff",4);
    Reject([&]{application::EncodeSettingsBackup(illegal);});
    // Existing nested settings migration remains explicit: only the historical
    // untouched default rail is migrated; custom selection/order is retained.
    auto legacy=application::Settings{}; legacy.data_rail={"aws","depth","sog","cog","heading"};
    Check(application::DecodeSettingsBackup(WithSettings(application::EncodeSettings(legacy))).settings.data_rail==
        std::vector<std::string>({"sog","depth","aws","heading"}),"Known nested migration failed");
    legacy.data_rail={"cog","sog"};
    Check(application::DecodeSettingsBackup(WithSettings(application::EncodeSettings(legacy))).settings.data_rail==legacy.data_rail,"Custom layout migrated");
    std::cout<<"Settings backup roundtrip, exclusions, compatibility, migration and invalid-input checks passed\n";
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
