#include "application/SettingsBackup.h"
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
namespace opennav::application {
namespace {
void Require(bool ok, const char *why) {
  if (!ok) throw std::invalid_argument(why);
}
// Strict UTF-8: reject overlong sequences, surrogates and codepoints > U+10FFFF.
bool ValidUtf8(const std::string &s, bool name = false) {
  if (name && (s.size() > 128 || (!s.empty() && s.find_first_not_of(' ') == std::string::npos))) return false;
  for (std::size_t i=0; i<s.size();) {
    const auto c=static_cast<unsigned char>(s[i++]);
    if (c == 0 || (name && (c < 32 || c == 127))) return false;
    if (c < 128) continue;
    unsigned cp; int n;
    if (c>=0xc2 && c<=0xdf) { cp=c&31; n=1; }
    else if (c>=0xe0 && c<=0xef) { cp=c&15; n=2; }
    else if (c>=0xf0 && c<=0xf4) { cp=c&7; n=3; }
    else return false;
    const int count=n;
    while (n--) {
      if (i==s.size()) return false;
      const auto d=static_cast<unsigned char>(s[i++]);
      if ((d&0xc0)!=0x80) return false;
      cp=(cp<<6)|(d&63);
    }
    if ((count==2 && cp<0x800) || (count==3 && cp<0x10000) ||
        (cp>=0xd800 && cp<=0xdfff) || cp>0x10ffff) return false;
  }
  return true;
}
}
void ValidateSettingsBackup(const SettingsBackup &b) {
  ValidateSettings(b.settings);
  Require(b.settings.pilot.interface_id.empty() && b.settings.pilot.name.empty() &&
          b.settings.pilot.address.empty() &&
      !b.settings.pilot.permit_control, "Pilot binding and control permissions are excluded from backups");
  Require(ValidUtf8(b.vessel_name,true), "Invalid backup vessel name");
  Require(ValidUtf8(EncodeSettings(b.settings)), "Backup settings must use valid UTF-8");
  Require(ValidDisplayPreferences(b.display), "Invalid backup display preferences");
  Require(std::isnan(b.chart_safety_depth_m) ||
      (std::isfinite(b.chart_safety_depth_m) && b.chart_safety_depth_m>=0 &&
       b.chart_safety_depth_m<=1000000), "Invalid backup chart safety depth");
}
std::string EncodeSettingsBackup(SettingsBackup b) {
  b.settings.pilot = {};
  ValidateSettingsBackup(b);
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "SKAGER_SETTINGS_BACKUP 1\n"
      << "\"vessel\" " << std::quoted(b.vessel_name) << '\n'
      << "\"settings\" " << std::quoted(EncodeSettings(b.settings)) << '\n'
      << "\"display\" " << std::quoted(EncodeDisplayPreferences(b.display)) << '\n'
      << "\"chart_safety_depth_m\" " << std::quoted(SettingNumber(b.chart_safety_depth_m)) << '\n';
  Require(out.str().size()<=SettingsBackupLimit, "Backup exceeds 128 KiB");
  return out.str();
}
SettingsBackup DecodeSettingsBackup(const std::string &record) {
  Require(record.size()<=SettingsBackupLimit && record.find('\0')==std::string::npos,
      "Invalid backup size or content (maximum 128 KiB)");
  std::istringstream in(record);
  in.imbue(std::locale::classic());
  std::string header;
  std::getline(in,header);
  Require(header=="SKAGER_SETTINGS_BACKUP 1", "Unsupported backup format or version; nothing changed");
  std::map<std::string,std::string> fields;
  while (!(in>>std::ws).eof()) {
    std::string key,value;
    Require(in.peek()=='"' && bool(in>>std::quoted(key)), "Invalid backup field");
    in>>std::ws;
    Require(in.peek()=='"' && bool(in>>std::quoted(value)), "Invalid backup value");
    Require(fields.emplace(key,value).second && fields.size()<=4, "Duplicate or excessive backup fields");
  }
  Require(fields.size()==4 && fields.count("vessel") && fields.count("settings") &&
      fields.count("display") && fields.count("chart_safety_depth_m"), "Missing or unsupported backup fields");
  SettingsBackup b;
  b.vessel_name=fields.at("vessel");
  b.settings=DecodeSettings(fields.at("settings"));
  b.display=DecodeDisplayPreferences(fields.at("display"));
  b.chart_safety_depth_m=ParseSettingNumber(fields.at("chart_safety_depth_m"));
  ValidateSettingsBackup(b);
  return b;
}
std::string SettingsBackupPreview(const SettingsBackup &b) {
  ValidateSettingsBackup(b);
  const auto number=[](double v) { return std::isnan(v)?std::string("Unconfigured"):SettingNumber(v); };
  return "Compatible SKAGER settings backup · version 1\n\nVessel: "+
      (b.vessel_name.empty()?"Unconfigured":b.vessel_name)+
      "\nDraft: "+number(b.settings.hazard.draft_m)+" m\nBattery: "+
      number(b.settings.energy.battery.capacity_kwh)+" kWh\nReserve: "+
      number(b.settings.energy.battery.reserve_soc_percent)+" %\nDisplay scale: "+
      std::to_string(b.display.scale_percent)+" %\nLayout: "+
      (b.display.layout==ChartLayout::Balanced?"Balanced":b.display.layout==ChartLayout::ChartFocus?"Chart focus":"Instrument focus")+
      "\nSource policies: "+
      std::to_string(b.settings.sources.size())+"\nSignal K mappings: "+
      std::to_string(b.settings.signal_k_mappings.size())+"\nCalibration points: "+
      std::to_string(b.settings.energy.curve.points.size())+
      "\nChart safety depth: "+number(b.chart_safety_depth_m)+
      " m\n\nReplaces vessel, energy, source, calibration and display settings. "
      "An unconfigured chart safety depth keeps the current value. "
      "Pilot control will be OFF. Verify sources and calibration for this vessel before navigation. "
      "Charts, licenses, credentials, routes, tracks, waypoints, connections and plugins are unchanged.";
}
} // namespace opennav::application
