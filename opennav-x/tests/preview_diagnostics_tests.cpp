#include "integration/PreviewDiagnostics.h"
#include "smartnav/VesselEnergy.h"
#include "vessel/RouteProgress.h"
#include <wx/filename.h>
#include <wx/jsonreader.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#ifdef OPENNAV_DIAGNOSTICS_GTEST
#include <gtest/gtest.h>
#endif
using namespace opennav;
using namespace std::chrono_literals;
namespace {
int checks=0;
void Check(bool ok,const char *why) {++checks;if(!ok)throw std::runtime_error(why);}
wxString Stamp(vessel::Time t) {return wxString::FromUTF8(std::to_string(
    std::chrono::duration_cast<vessel::Duration>(t.time_since_epoch()).count()));}
wxJSONValue Read(const std::string &path) {
  std::ifstream in(path);const std::string bytes((std::istreambuf_iterator<char>(in)),{});
  wxJSONReader reader;wxJSONValue value;
  Check(reader.Parse(wxString::FromUTF8(bytes),&value)==0,"read actual atomic diagnostic output");return value;
}
}
int RunDiagnosticCoherenceChecks() {
  const auto path=wxFileName::CreateTempFileName("xnav-diagnostic-coherence-").ToStdString(wxConvUTF8);
  try {
    // No sleeps: every write occurs after this captured live publication floor.
    // The retained pre-boundary evaluation is deliberately 39ms earlier.
    const auto publication_floor=vessel::Clock::now();
    const auto observed=publication_floor-5038ms;
    const auto reading=[&](double v) {return vessel::Sample{v,"offline fixture",observed,vessel::Validity::Measured};};
    vessel::VesselState state;
    state.navigation.latitude_deg=reading(59);state.navigation.longitude_deg=reading(18);
    state.navigation.sog_kn=reading(5);state.battery.soc_percent=reading(68);
    state.battery.net_discharge_kw=reading(4);
    auto route=std::make_shared<vessel::RouteProgressSnapshot>();
    route->route_id="fixture-route";route->revision_scope="offline";route->route_revision=1;
    route->active_waypoint_id="finish";route->active_waypoint_index=0;route->waypoint_count=1;
    route->remaining_distance_nm=10;route->observed_at=observed;route->position_observed_at=observed;
    route->state=vessel::RouteState::Valid;route->source="copied progress";route->position_source="offline fixture";
    state.navigation.route=route;
    const smartnav::EnergyModel model{24,20,.5,"offline explicit assumptions"};
    for(int mode=0;mode<3;++mode) {
      state.simulated=mode==1;state.replayed=mode==2;
      for(const auto age:{4999ms,5000ms,5038ms}) {
        const auto evaluated=observed+age;
        const auto energy=smartnav::PredictVesselEnergy(model,state,evaluated);
        const bool stale=age>=5000ms;
        Check(bool(energy.arrival.estimate)!=stale,"existing energy threshold unchanged");
        // Candidate health is a separate later collection, not the selected frame.
        vessel::SourceHealth candidate{};candidate.quantity=vessel::Quantity::BatterySoc;
        candidate.sample={72,"later candidate",publication_floor,vessel::Validity::Measured};
        wxJSONValue runtime;runtime["separate_observation"]=wxString("retained runtime payload");
        integration::WritePreviewDiagnostics(path,state,{},energy,{},
            state.replayed?std::vector<vessel::SourceHealth>{}:std::vector<vessel::SourceHealth>{candidate},
            "Energy",runtime);
        auto report=Read(path);
        Check(report["route"].HasMember("remaining_nm")!=stale,"route matches prediction evaluation, not serialization time");
        Check(report["energy"].HasMember("arrival_soc")!=stale,"arrival estimate agrees with route freshness");
        Check(report["route"]["state"].AsString()==(stale?"StalePosition":"Valid"),"route state matches boundary");
        bool found=false;
        for(int i=0;i<report["data"].Size();++i) {
          auto item=report["data"][i];if(item["name"].AsString()!="Battery SOC")continue;
          found=true;Check(item["age_ms"].AsInt()==age.count(),"selected sample age uses same evaluation instant");
          Check(item["quality"].AsString()==(stale?"STALE":"AGING"),"selected quality agrees with energy frame");
          Check(item["observed_monotonic_ms"].AsString()==Stamp(observed),"producer timestamp unchanged");
        }
        Check(found,"SOC record exists");
        Check(report["evaluation_monotonic_ms"].AsString()==Stamp(evaluated),"explicit evaluation timestamp");
        long long published=0;
        Check(report["publication_monotonic_ms"].AsString().ToLongLong(&published) &&
            published>=std::chrono::duration_cast<vessel::Duration>(publication_floor.time_since_epoch()).count(),
            "publication occurs beyond retained frame boundary");
        Check(report["runtime"]["separate_observation"].AsString()=="retained runtime payload","runtime observation not relabelled or recomputed");
        if(!state.replayed) {
          Check(report["source_candidates"][0]["quality"].AsString()=="LIVE","later candidate health keeps its independent clock");
          Check(report["source_candidates"][0]["observed_steady_ms"].AsString()==Stamp(publication_floor),"candidate producer timestamp unchanged");
        }
        Check(state.battery.soc_percent.observed_at==observed && route->observed_at==observed,
              "export never refreshes retained input timestamps");
      }
    }
    std::filesystem::remove(std::filesystem::u8path(path));
    std::cout<<checks<<" diagnostic freshness coherence checks passed\n";return 0;
  } catch(const std::exception &e) {
    std::filesystem::remove(std::filesystem::u8path(path));
    std::cerr<<"FAILED: "<<e.what()<<'\n';return 1;
  }
}
#ifdef OPENNAV_DIAGNOSTICS_GTEST
TEST(OpenNavDiagnostics, FreshnessFrameCoherence) {EXPECT_EQ(RunDiagnosticCoherenceChecks(),0);}
#else
int main(){return RunDiagnosticCoherenceChecks();}
#endif
