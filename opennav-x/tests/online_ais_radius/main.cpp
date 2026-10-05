// Fully offline integration: real configuration, geometry, session, decoder,
// cache, merge and chart-target projection; fake credentials and transport only.
#include "integration/OnlineAis.h"
#include "integration/AisViewport.h"
#include "ais/AisStreamSession.h"
#include "ais/ChartTargets.h"
#include <wx/fileconf.h>
#include <wx/init.h>
#include <wx/sstream.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
using namespace opennav;
using namespace std::chrono_literals;
namespace {
int checks = 0;
void Check(bool pass, const char *message) {
  if (!pass) throw std::runtime_error(message);
  ++checks;
}
struct Credentials : ais::IAisCredentials {
  ais::CredentialResult Read() const override { return {}; }
  ais::CredentialStatus Store(const ais::Secret &) override { return ais::CredentialStatus::Unavailable; }
  ais::CredentialStatus Remove() override { return ais::CredentialStatus::Removed; }
};
struct Provider : ais::IOnlineAisProvider {
  bool enabled = false;
  int observations = 0;
  ais::Viewport area;
  ais::ProviderSnapshot snapshot;
  void SetEnabled(bool value) override { enabled = value; }
  bool ObserveViewport(ais::Viewport value) override { ++observations; area = value; return true; }
  void CredentialChanged() override {}
  ais::ProviderSnapshot Read(vessel::Time) const override { return snapshot; }
};
struct Config : wxFileConfig {
  explicit Config(wxInputStream &stream) : wxFileConfig(stream) {}
  bool fail = false;
  int writes = 0;
  bool Flush(bool = false) override { ++writes; if (fail) { fail = false; return false; } return true; }
};
const vessel::Time now{100h};
const auto wall = std::chrono::system_clock::from_time_t(1790596800);
std::string Position(const std::string &type, int id, double latitude, double longitude) {
  return "{\"MessageType\":\""+type+"\",\"MetaData\":{\"MMSI\":"+std::to_string(id)+
    "},\"Message\":{\""+type+"\":{\"Valid\":true,\"UserID\":"+std::to_string(id)+
    ",\"Latitude\":"+std::to_string(latitude)+",\"Longitude\":"+std::to_string(longitude)+
    ",\"Sog\":5,\"Cog\":120,\"TrueHeading\":119}}}";
}
const std::string confirmation = "{\"MessageType\":\"SubscriptionConfirmation\",\"Message\":{\"CompressionEnabled\":true}}";
void Connect(ais::AisStreamSession &session, vessel::Time at, const ais::Viewport &expected) {
  Check(session.Connecting(at), "fixture begins connection");
  Check(session.Opened(at), "fixture socket opens");
  const auto actual = session.PendingSubscription(at), desired = ais::SubscriptionArea(expected);
  Check(actual.size() == desired.size() && !actual.empty(), "complete initial subscription ready immediately");
  for (std::size_t i=0;i<actual.size();++i)
    Check(actual[i].south == desired[i].south && actual[i].north == desired[i].north &&
          actual[i].west == desired[i].west && actual[i].east == desired[i].east,
          "initial/reconnected subscription contains latest selected radius");
  Check(session.SubscriptionSent(at), "fixture sends initial subscription");
  session.Receive(confirmation,at,wall,0);
  Check(session.Read(at).health.subscription_confirmed, "service confirmation required");
}
}
int main() {
 try {
  wxInitializer init;
  Check(init.IsOk(), "wx base initialized");
  const auto view = integration::AisViewport(true,58,60,17,19,ais::AreaCenter{59.3,18});
  Check(bool(view), "actual chart center copied");
  auto area = integration::AisRadiusViewport(*view,25);
  Check(area && area->center->latitude == 59.3 && area->exact_area,
        "radius uses projection center rather than bbox midpoint");
  Check(integration::WithinAisRadius(*area->center,25,59.3,18), "center inside radius");
  Check(!integration::WithinAisRadius(*area->center,25,60,18), "outside radius rejected");
  Check(!integration::WithinAisRadius(*area->center,25,area->north,area->east),
        "bounding-box corner does not leak outside circular display filter");
  const auto dateline = integration::AisViewport(true,0,2,179,181,ais::AreaCenter{1,179.9});
  const auto crossing = integration::AisRadiusViewport(*dateline,25);
  Check(crossing && ais::SubscriptionArea(*crossing).size() == 2,
        "radius splits correctly across antimeridian");
  Check(integration::WithinAisRadius(*crossing->center,25,1,-179.9), "dateline neighbor inside radius");
  const auto polar = integration::AisRadiusViewport({89,90,-180,180,ais::AreaCenter{89.9,45}},25);
  Check(polar && ais::SubscriptionArea(*polar).size() == 1 && polar->west == -180 && polar->east == 180,
        "polar radius safely covers all longitudes");
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  Check(!integration::AisRadiusViewport(*view,0) && !integration::AisRadiusViewport(*view,201),
        "unsupported radii rejected");
  Check(!integration::AisViewport(true,58,60,17,19,ais::AreaCenter{nan,18}), "invalid center rejected");
  Check(!integration::WithinAisRadius({59,18},25,nan,18), "invalid position rejected");

  wxStringInputStream empty(""); Config config(empty);
  Provider *provider = nullptr;
  auto make = [&] {
    auto p = std::make_unique<Provider>(); provider = p.get();
    return std::make_unique<integration::OnlineAis>(config,std::make_unique<Credentials>(),std::move(p));
  };
  auto service = make();
  Check(service->RadiusNm() == 25 && !service->Enabled(), "new profile uses 25 nm and stays off");
  Check(service->SetRadiusNm(1).ok && !provider->enabled, "minimum saves without enabling service");
  service = make(); Check(service->RadiusNm() == 1, "minimum survives reload");
  Check(service->SetRadiusNm(200).ok, "maximum accepted");
  service = make(); Check(service->RadiusNm() == 200, "maximum survives reload");
  Check(!service->SetRadiusNm(0).ok && !service->SetRadiusNm(201).ok && service->RadiusNm() == 200,
        "invalid changes preserve persisted selection");
  config.fail = true;
  Check(!service->SetRadiusNm(10).ok && service->RadiusNm() == 200,
        "failed save keeps previous effective radius");
  Check(config.Read("/OpenNav/OnlineAIS/v1/RadiusNm","") == "200", "failed save restores exact config");
  for (const char *invalid : {"0","201","garbage","12.5","9999999999999999999"}) {
    config.Write("/OpenNav/OnlineAIS/v1/RadiusNm",invalid);
    service = make(); Check(service->RadiusNm() == 25, "invalid external preference safely defaults");
  }
  Check(service->SetRadiusNm(20).ok, "valid radius replaces invalid stored preference");
  application::CommandResult worker;
  std::thread thread([&] { worker = service->SetRadiusNm(10); }); thread.join();
  Check(!worker.ok && service->RadiusNm() == 20, "worker cannot mutate profile");
  Check(service->Enable(true).ok, "explicit enable saved");
  service->ObserveViewport(view,true);
  Check(provider->enabled && provider->area.exact_area, "valid live chart enables exact area subscription");
  const auto large_area = provider->area;
  Check(service->SetRadiusNm(5).ok && provider->area.north < large_area.north,
        "radius shrink updates provider intent immediately");
  service->ObserveViewport(view,false);
  Check(!provider->enabled, "replay/live gate still stops provider");
  Check(service->SetRadiusNm(25).ok && !provider->enabled, "radius changes do not bypass live gate");
  service->ObserveViewport(view,true);

  ais::AisStreamSession session;
  session.Enable(true,now);
  Check(session.ObserveViewport(provider->area), "radius enters real provider policy");
  Connect(session,now,provider->area);
  const char *types[] = {"PositionReport","StandardClassBPositionReport","ExtendedClassBPositionReport"};
  for (int i = 0; i < 3; ++i)
    session.Receive(Position(types[i],265000001+i,59.3+i*.01,18),now+1s,wall+1s,0);
  session.Receive(Position("PositionReport",265000004,60.3,18),now+1s,wall+1s,0);
  provider->snapshot = session.Read(now+1s);
  Check(provider->snapshot.targets.targets.size() == 4 && provider->snapshot.health.accepted == 4,
        "confirmed Class A and both Class B reports pass decoder and cache");
  const auto filtered = service->Read(now+1s);
  Check(filtered.targets.targets.size() == 3, "radius removes out-of-area cached targets only");
  Check(filtered.cached_position_count == 4 && filtered.health.accepted == 4,
        "diagnostics preserve pre-radius positioned-cache count and accepted reports");
  Check(service->Read(now+1s).cached_position_count == 4 && provider->snapshot.targets.targets.size() == 4,
        "diagnostic reads never mutate provider cache or radius counts");
  auto display = ais::Aggregate({},filtered).display;
  Check(display.available && display.targets.size() == 3, "online fixtures reach panel display list");
  auto marks = ais::OnlineChartTargets(display,now+1s,0);
  Check(marks.size() == 3, "same Class A/B fixtures produce chart marks");
  for (const auto &t : display.targets)
    Check(t.origin == vessel::AisOrigin::AisStreamOnline && t.source == ais::OnlineSource &&
          !t.cpa_nm.value && !t.tcpa_minutes.value && !t.upstream_alarm,
          "online provenance and no invented collision data preserved");
  vessel::AisState onboard; onboard.available = true;
  auto local = display.targets.front(); local.origin = vessel::AisOrigin::LocalOpenCPN;
  local.source = "onboard"; local.name = "Local vessel wins";
  onboard.targets.push_back(local);
  display = ais::Aggregate(onboard,filtered).display;
  Check(display.targets.size() == 3 && display.targets.front().name == "Local vessel wins" &&
        ais::OnlineChartTargets(display,now+1s,0).size() == 2,
        "local identity retains precedence without duplicate online chart mark");
  Check(service->SetRadiusNm(1).ok, "shrink configured after traffic arrives");
  Check(service->Read(now+1s).cached_position_count == 4,
        "shrinking retains raw cache count separately from in-radius positions");
  Check(service->Read(now+1s).targets.targets.size() == 2,
        "shrinking immediately filters retained cache without waiting for expiration");
  Check(session.ObserveViewport(provider->area), "shrunk radius observed by session");
  Check(session.PendingSubscription(now+4s).empty(), "replacement bounded to existing five-second cadence");
  Check(!session.PendingSubscription(now+5s).empty(), "shrink replaces larger subscription at cadence");
  Check(session.SubscriptionSent(now+5s), "replacement sent");
  session.Receive(confirmation,now+5s,wall+5s,0);
  session.Disconnected(now+6s,0);
  session.Tick(now+10s,0);
  Connect(session,now+10s,provider->area);
  const auto latest = ais::SubscriptionArea(provider->area);
  Check(!latest.empty() && session.PendingSubscription(now+10s).empty(), "reconnect confirms latest radius once");
  provider->snapshot = session.Read(now+11min);
  Check(service->Read(now+11min).targets.targets.empty(), "expired positions never reappear in radius");
  service->ObserveViewport({},true);
  Check(!provider->enabled && service->Read(now).targets.targets.empty(), "missing chart cannot retain prior traffic area");
  std::cout << "PASS " << checks << " offline AIS radius/data-path checks\n";
  return 0;
 } catch (const std::exception &e) { std::cerr << "FAIL " << e.what() << '\n'; return 1; }
}
