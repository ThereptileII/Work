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
