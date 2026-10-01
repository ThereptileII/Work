#include "integration/SettingsStore.h"
#include <cmath>
#include <gtest/gtest.h>
#include <thread>
#include <wx/filename.h>
#include <wx/init.h>
#include <wx/sstream.h>
using namespace opennav;
namespace {
application::Settings Config() {
  application::Settings s;
  s.energy.battery = {24, 20, .5, "Explicit persisted configuration"};
  s.energy.battery_device_id = "pack-test";
  s.current = vessel::CurrentConvention::PositiveDischarge;
  return s;
}
} // namespace
TEST(OpenNavSettings, PersistsInSharedProfileWithoutChangingOtherEntries) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  const auto path = wxFileName::CreateTempFileName("opennav-settings");
  {
    wxFileConfig config("", "", path, "", wxCONFIG_USE_LOCAL_FILE);
    config.Write("/Settings/TestChartPath", "existing-user-charts");
    config.Write("/OpenNav/InterfaceMode", "legacy");
    integration::SettingsStore store(config);
    EXPECT_TRUE(std::isnan(store.Read().energy.battery.capacity_kwh));
    auto s = Config();
    s.signal_k_mappings = {{"propulsion.main.electricalPower",
                            vessel::Quantity::MotorPower, .001, 0}};
    s.sources[vessel::Quantity::Depth] = {
        "specific-source", {vessel::Duration{1200}, vessel::Duration{3500}}};
    ASSERT_TRUE(store.Save(s).ok);
  }
  {
    wxFileConfig config("", "", path, "", wxCONFIG_USE_LOCAL_FILE);
    integration::SettingsStore store(config);
    EXPECT_EQ(store.Read().energy.battery.capacity_kwh, 24);
    ASSERT_EQ(store.Read().signal_k_mappings.size(), 1u);
    EXPECT_EQ(store.Read().signal_k_mappings[0].scale, .001);
    EXPECT_EQ(store.Read().energy.battery_device_id, "pack-test");
    EXPECT_EQ(store.Read().sources.at(vessel::Quantity::Depth).pinned_source,
              "specific-source");
    EXPECT_EQ(config.Read("/Settings/TestChartPath", ""),
              "existing-user-charts");
    EXPECT_EQ(config.Read("/OpenNav/InterfaceMode", ""), "legacy");
  }
  EXPECT_TRUE(wxRemoveFile(path));
}
TEST(OpenNavSettings, CorruptRecordPreservedAndFailsClosed) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  wxStringInputStream input("");
  wxFileConfig config(input);
  config.Write("/OpenNav/AlphaSettings", "unknown future or corrupt record");
  integration::SettingsStore store(config);
  EXPECT_TRUE(std::isnan(store.Read().energy.battery.capacity_kwh));
  EXPECT_TRUE(store.Read().energy.battery_device_id.empty());
  EXPECT_EQ(config.Read("/OpenNav/AlphaSettings", ""),
            "unknown future or corrupt record");
  auto bad = Config();
  bad.energy.battery.reserve_soc_percent = 101;
  EXPECT_FALSE(store.Save(bad).ok);
  EXPECT_TRUE(std::isnan(store.Read().energy.battery.capacity_kwh));
}
TEST(OpenNavSettings, DisplayPreferencesRoundTripWithoutChangingOlderSettings) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  const auto path=wxFileName::CreateTempFileName("opennav-display");
  const auto old=application::EncodeSettings(Config());
  {
    wxFileConfig config("","",path,"",wxCONFIG_USE_LOCAL_FILE);
    ASSERT_TRUE(config.Write("/OpenNav/AlphaSettings",wxString::FromUTF8(old)));
    ASSERT_TRUE(config.Write("/Settings/TestChartPath","existing-chart-path"));
    for (const int scale:{100,125,150})
      for (const auto layout:{application::ChartLayout::Balanced,
                              application::ChartLayout::ChartFocus,
                              application::ChartLayout::InstrumentFocus}) {
        integration::SettingsStore store(config);
        const application::DisplayPreferences wanted{scale,layout};
        ASSERT_TRUE(store.SaveDisplay(wanted).ok);
        EXPECT_EQ(config.Read("/OpenNav/AlphaSettings","").ToStdString(wxConvUTF8),old);
        EXPECT_EQ(config.Read("/Settings/TestChartPath",""),"existing-chart-path");
        wxFileConfig reopened_config("","",path,"",wxCONFIG_USE_LOCAL_FILE);
        integration::SettingsStore reopened(reopened_config);
        EXPECT_EQ(reopened.Display().scale_percent,scale);
        EXPECT_EQ(reopened.Display().layout,layout);
        EXPECT_EQ(reopened_config.Read("/OpenNav/AlphaSettings","").ToStdString(wxConvUTF8),old);
      }
  }
  {
    wxFileConfig config("","",path,"",wxCONFIG_USE_LOCAL_FILE);
    integration::SettingsStore store(config);
    EXPECT_EQ(store.Display().scale_percent,150);
    EXPECT_EQ(store.Display().layout,application::ChartLayout::InstrumentFocus);
    EXPECT_EQ(config.Read("/OpenNav/AlphaSettings","").ToStdString(wxConvUTF8),old);
  }
  EXPECT_TRUE(wxRemoveFile(path));
}
TEST(OpenNavSettings, InvalidDisplayRecordPreservedAndInvalidSaveRejected) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  for (const auto &raw:{"v2|125|chart","v1|125","v1|125|chart|extra",
                        "v1|200|chart","v1|125|unknown"}) {
    wxStringInputStream input("");
    wxFileConfig config(input);
    ASSERT_TRUE(config.Write("/OpenNav/DisplayPreferencesV1",raw));
    integration::SettingsStore store(config);
    EXPECT_EQ(store.Display().scale_percent,100);
    EXPECT_EQ(store.Display().layout,application::ChartLayout::Balanced);
    EXPECT_FALSE(store.DisplayStatus().empty());
    EXPECT_FALSE(store.SaveDisplay({200,application::ChartLayout::ChartFocus}).ok);
    EXPECT_EQ(config.Read("/OpenNav/DisplayPreferencesV1",""),raw);
  }
}
TEST(OpenNavSettings, DisplayFailedFlushRestoresAppliedValue) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  struct FailingConfig : wxFileConfig {
    explicit FailingConfig(wxInputStream &s) : wxFileConfig(s) {}
    int calls=0;
    bool Flush(bool=false) override { return ++calls!=1; }
  };
  wxStringInputStream input("");
  FailingConfig config(input);
  ASSERT_TRUE(config.Write("/OpenNav/DisplayPreferencesV1","v1|125|chart"));
  integration::SettingsStore store(config);
  EXPECT_FALSE(store.SaveDisplay({150,application::ChartLayout::InstrumentFocus}).ok);
  EXPECT_EQ(config.calls,2);
  EXPECT_EQ(store.Display().scale_percent,125);
  EXPECT_EQ(store.Display().layout,application::ChartLayout::ChartFocus);
  EXPECT_EQ(config.Read("/OpenNav/DisplayPreferencesV1",""),"v1|125|chart");
}
TEST(OpenNavSettings, FailedFlushRestoresPreviousValue) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  struct FailingConfig : wxFileConfig {
    explicit FailingConfig(wxInputStream &s) : wxFileConfig(s) {}
    int calls = 0;
    bool Flush(bool = false) override { return ++calls != 1; }
  };
  wxStringInputStream input("");
  FailingConfig config(input);
  auto original = application::EncodeSettings(Config());
  config.Write("/OpenNav/AlphaSettings", wxString::FromUTF8(original));
  integration::SettingsStore store(config);
  auto altered = Config();
  altered.energy.battery.capacity_kwh = 40;
  EXPECT_FALSE(store.Save(altered).ok);
  EXPECT_EQ(config.calls, 2);
  EXPECT_EQ(store.Read().energy.battery.capacity_kwh, 24);
  EXPECT_EQ(config.Read("/OpenNav/AlphaSettings", "").ToStdString(wxConvUTF8),
            original);
}
TEST(OpenNavSettings, VesselProfileKeepsLegacyRecordAndUnconfiguredEnergy) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  wxStringInputStream input("");
  wxFileConfig config(input);
  config.Write("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR", 1.8288);
  integration::SettingsStore store(config);
  auto partial = store.Read();
  partial.hazard.draft_m = 1.4;
  ASSERT_TRUE(store.SaveVessel(partial, "REPTIL", std::nan("")).ok);
  EXPECT_EQ(store.VesselName(), "REPTIL");
  EXPECT_TRUE(std::isnan(store.Read().energy.battery.capacity_kwh));
  EXPECT_TRUE(std::isnan(store.Read().energy.battery.reserve_soc_percent));
  double depth = 0;
  ASSERT_TRUE(config.Read("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR", &depth));
  EXPECT_DOUBLE_EQ(depth, 1.8288); // Unedited stock value is not rounded/replaced.
  const auto record = config.Read("/OpenNav/AlphaSettings", "").ToStdString(wxConvUTF8);
  EXPECT_EQ(application::DecodeSettings(record).hazard.draft_m, 1.4);
  EXPECT_EQ(record.find("VesselName"), std::string::npos); // Old strict decoder still works.
  integration::SettingsStore reopened(config);
  EXPECT_EQ(reopened.VesselName(), "REPTIL");
  EXPECT_TRUE(std::isnan(reopened.Read().energy.battery.capacity_kwh));
}
TEST(OpenNavSettings, VesselSaveFailedFlushRestoresAllThreeEntries) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  struct FailingConfig : wxFileConfig {
    explicit FailingConfig(wxInputStream &s) : wxFileConfig(s) {}
    int calls = 0;
    bool Flush(bool = false) override { return ++calls != 1; }
  };
  wxStringInputStream input("");
  FailingConfig config(input);
  const auto old = application::EncodeSettings(Config());
  config.Write("/OpenNav/AlphaSettings", wxString::FromUTF8(old));
  config.Write("/OpenNav/VesselName", "Old boat");
  config.Write("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR", 2.75);
  integration::SettingsStore store(config);
  auto changed = store.Read();
  changed.hazard.draft_m = 1.5;
  EXPECT_FALSE(store.SaveVessel(changed, "New boat", 3.5).ok);
  EXPECT_EQ(config.calls, 2);
  EXPECT_EQ(store.VesselName(), "Old boat");
  EXPECT_TRUE(std::isnan(store.Read().hazard.draft_m));
  EXPECT_EQ(config.Read("/OpenNav/AlphaSettings", "").ToStdString(wxConvUTF8), old);
  EXPECT_EQ(config.Read("/OpenNav/VesselName", ""), "Old boat");
  double depth = 0;
  ASSERT_TRUE(config.Read("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR", &depth));
  EXPECT_DOUBLE_EQ(depth, 2.75);
}
TEST(OpenNavSettings, VesselSavePersistsEditedChartNameAndBattery) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  wxStringInputStream input("");
  wxFileConfig config(input);
  integration::SettingsStore store(config);
  auto edited = store.Read();
  edited.energy.battery.capacity_kwh = 72;
  edited.energy.battery.reserve_soc_percent = 18;
  edited.energy.battery.source = "User-configured usable battery energy and reserve / OpenCPN profile";
  ASSERT_TRUE(store.SaveVessel(edited, "Current boat", 3.5).ok);
  integration::SettingsStore reopened(config);
  EXPECT_EQ(reopened.VesselName(), "Current boat");
  EXPECT_DOUBLE_EQ(reopened.Read().energy.battery.capacity_kwh, 72);
  EXPECT_DOUBLE_EQ(reopened.Read().energy.battery.reserve_soc_percent, 18);
  double chart = 0;
  ASSERT_TRUE(config.Read("/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR", &chart));
  EXPECT_DOUBLE_EQ(chart, 3.5);
}
TEST(OpenNavSettings, WorkerCannotWriteConfiguration) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  wxStringInputStream input("");
  wxFileConfig config(input);
  integration::SettingsStore store(config);
  application::CommandResult result;
  std::thread worker([&] { result = store.Save(Config()); });
  worker.join();
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(config.HasEntry("/OpenNav/AlphaSettings"));
}
