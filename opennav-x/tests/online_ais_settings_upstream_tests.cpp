#include "integration/OnlineAis.h"
#include "integration/AisViewport.h"
#include "bbox.h"
#include <array>
#include <gtest/gtest.h>
#include <limits>
#include <thread>
#include <wx/fileconf.h>
#include <wx/init.h>
#include <wx/sstream.h>
using namespace opennav;
namespace {
constexpr const char *key = "/OpenNav/OnlineAIS/v1/Enabled";
struct Credentials : ais::IAisCredentials {
  bool present = false;
  ais::CredentialStatus store = ais::CredentialStatus::Ready;
  ais::CredentialStatus remove = ais::CredentialStatus::Removed;
  ais::CredentialResult Read() const override {
    ais::CredentialResult r;
    if (present) { r.status=ais::CredentialStatus::Ready; r.key.Assign("test-only-never-a-service-key"); }
    return r;
  }
  ais::CredentialStatus Store(const ais::Secret &) override {
    if(store == ais::CredentialStatus::Ready)present=true;
    return store;
  }
  ais::CredentialStatus Remove() override {
    if(remove == ais::CredentialStatus::Removed)present=false;
    return remove;
  }
};
struct Provider : ais::IOnlineAisProvider {
  bool enabled=false, valid=true;
  int observations=0, credentials_changed=0;
  void SetEnabled(bool value) override { enabled=value; }
  bool ObserveViewport(ais::Viewport v) override {
    ++observations; return valid && !ais::SubscriptionArea(v).empty();
  }
  void CredentialChanged() override { ++credentials_changed; }
  ais::ProviderSnapshot Read(vessel::Time) const override { return {}; }
};
struct Config : wxFileConfig {
  explicit Config(wxInputStream &s) : wxFileConfig(s) {}
  bool fail=false;
  int flushes=0;
  bool Flush(bool=false) override { ++flushes; if(fail){fail=false;return false;}return true; }
};
struct Harness {
  wxInitializer init;
  wxStringInputStream input{""};
  Config config{input};
  Credentials *credentials=nullptr;
  Provider *provider=nullptr;
  std::unique_ptr<integration::OnlineAis> Make() {
    auto c=std::make_unique<Credentials>();credentials=c.get();
    auto p=std::make_unique<Provider>();provider=p.get();
    return std::make_unique<integration::OnlineAis>(config,std::move(c),std::move(p));
  }
};
const ais::Viewport view{57,59,16,19};
}
TEST(OpenNavOnlineAIS, DefaultOffAndPersistentOptInNeedsValidLiveViewport) {
  Harness h; ASSERT_TRUE(h.init.IsOk());
  h.config.Write("/Settings/TestChartPath","untouched");
  auto service=h.Make(); EXPECT_FALSE(service->Enabled());
  service->ObserveViewport(view,true); EXPECT_FALSE(h.provider->enabled);
  ASSERT_TRUE(service->Enable(true).ok);
  EXPECT_FALSE(h.provider->enabled); // setting alone does not invent an area
  service->ObserveViewport(view,false); EXPECT_FALSE(h.provider->enabled);
  service->ObserveViewport({},true); EXPECT_FALSE(h.provider->enabled);
  service->ObserveViewport(view,true); EXPECT_TRUE(h.provider->enabled);
  service=h.Make(); EXPECT_TRUE(service->Enabled()); EXPECT_FALSE(h.provider->enabled);
  EXPECT_EQ(h.config.Read("/Settings/TestChartPath",""),"untouched");
}
TEST(OpenNavOnlineAIS, UnknownConfigurationIsPreservedAndDisabled) {
  Harness h; ASSERT_TRUE(h.init.IsOk());
  for(const char *value: {"yes","true","2","future-format",""}) {
    h.config.Write(key,value);
    auto service=h.Make(); EXPECT_FALSE(service->Enabled());
    service->ObserveViewport(view,true); EXPECT_FALSE(h.provider->enabled);
    EXPECT_EQ(h.config.Read(key,""),value);
  }
}
TEST(OpenNavOnlineAIS, InvalidViewportAndReplayStopNetworking) {
  Harness h; auto service=h.Make(); ASSERT_TRUE(service->Enable(true).ok);
  service->ObserveViewport(view,true); EXPECT_TRUE(h.provider->enabled);
  h.provider->valid=false; service->ObserveViewport(view,true);
  EXPECT_FALSE(h.provider->enabled); // cannot retain the last valid area
  h.provider->valid=true;service->ObserveViewport(view,true);
  EXPECT_TRUE(h.provider->enabled);
  service->ObserveViewport(view,false); EXPECT_FALSE(h.provider->enabled);
}
TEST(OpenNavOnlineAIS, FailedEnableRestoresTheExactPreviousPreference) {
  Harness h; auto service=h.Make(); h.config.fail=true;
  EXPECT_FALSE(service->Enable(true).ok); EXPECT_FALSE(service->Enabled());
  EXPECT_FALSE(h.config.HasEntry(key)); EXPECT_EQ(h.config.flushes,2);
  h.config.Write(key,"0");h.config.fail=true;
  EXPECT_FALSE(service->Enable(true).ok); EXPECT_EQ(h.config.Read(key,""),"0");
}
TEST(OpenNavOnlineAIS, DisableStopsImmediatelyEvenWhenPersistenceFails) {
  Harness h; auto service=h.Make();ASSERT_TRUE(service->Enable(true).ok);
  service->ObserveViewport(view,true);ASSERT_TRUE(h.provider->enabled);
  h.config.fail=true; EXPECT_FALSE(service->Enable(false).ok);
  EXPECT_FALSE(service->Enabled());EXPECT_FALSE(h.provider->enabled);
  service->ObserveViewport(view,true);EXPECT_FALSE(h.provider->enabled);
}
TEST(OpenNavOnlineAIS, CredentialIsSeparateAndDoesNotImplicitlyEnable) {
  Harness h; auto service=h.Make();ais::Secret value;ASSERT_TRUE(value.Assign("test-secret"));
  ASSERT_TRUE(service->StoreKey(value).ok); EXPECT_TRUE(service->CredentialPresent());
  EXPECT_FALSE(service->Enabled());EXPECT_EQ(h.provider->credentials_changed,1);
  EXPECT_FALSE(h.config.HasEntry(key));
  wxStringOutputStream output;h.config.Save(output);
  EXPECT_EQ(output.GetString().Find("test-secret"),wxNOT_FOUND);
  EXPECT_FALSE(h.provider->enabled);
}
TEST(OpenNavOnlineAIS, FailedRemovalStillDisablesAndSuccessfulRemovalVerifiesAbsence) {
  Harness h;auto service=h.Make();ais::Secret value;ASSERT_TRUE(value.Assign("test-secret"));
  ASSERT_TRUE(service->StoreKey(value).ok);ASSERT_TRUE(service->Enable(true).ok);
  service->ObserveViewport(view,true);
  h.credentials->remove=ais::CredentialStatus::Unavailable;
  EXPECT_FALSE(service->RemoveKey().ok);EXPECT_FALSE(h.provider->enabled);
  EXPECT_FALSE(service->Enabled());EXPECT_TRUE(service->CredentialPresent());
  h.credentials->remove=ais::CredentialStatus::Removed;
  EXPECT_TRUE(service->RemoveKey().ok);EXPECT_FALSE(service->CredentialPresent());
}
TEST(OpenNavOnlineAIS, WorkerCannotWriteSettingsOrCredentials) {
  Harness h;auto service=h.Make();application::CommandResult enable,store,remove;
  std::thread worker([&]{ais::Secret value;value.Assign("test-secret");
    enable=service->Enable(true);store=service->StoreKey(value);remove=service->RemoveKey();});
  worker.join();EXPECT_FALSE(enable.ok);EXPECT_FALSE(store.ok);EXPECT_FALSE(remove.ok);
  EXPECT_FALSE(h.config.HasEntry(key));EXPECT_FALSE(h.credentials->present);
}
TEST(OpenNavOnlineAIS, PinnedOrderedBoundingBoxPreservesDatelineAndWorldExtent) {
  for(auto bounds: {std::array<double,4>{57,16,59,19}, {57,179,59,181},
                    {57,-181,59,-179}, {-90,-180,90,180}, {85,-210,90,200}}) {
    LLBBox upstream;upstream.Set(bounds[0],bounds[1],bounds[2],bounds[3]);
    auto copied=integration::AisViewport(upstream.GetValid(),upstream.GetMinLat(),
        upstream.GetMaxLat(),upstream.GetMinLon(),upstream.GetMaxLon());
    ASSERT_TRUE(copied);const auto area=ais::SubscriptionArea(*copied);ASSERT_FALSE(area.empty());
    for(const auto &box:area){EXPECT_GE(box.west,-180);EXPECT_LE(box.east,180);EXPECT_LT(box.west,box.east);}
    if(bounds[3]-bounds[1]>=360){ASSERT_EQ(area.size(),1u);EXPECT_EQ(area[0].west,-180);EXPECT_EQ(area[0].east,180);}
    if(bounds[1]==179 || bounds[1]==-181) { EXPECT_EQ(area.size(),2u); }
  }
}
TEST(OpenNavOnlineAIS, InvalidAndOverflowViewportCannotCreateSubscription) {
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  const auto inf=std::numeric_limits<double>::infinity();
  const auto max=std::numeric_limits<double>::max();
  EXPECT_FALSE(integration::AisViewport(false,57,59,16,19));
  for(auto values: {std::array<double,4>{nan,59,16,19},{57,inf,16,19},
      {57,59,nan,19},{57,59,16,inf},{57,57,16,19},{57,59,19,16},
      {91,92,16,19},{57,59,-max,max}})
    EXPECT_FALSE(integration::AisViewport(true,values[0],values[1],values[2],values[3]));
}
