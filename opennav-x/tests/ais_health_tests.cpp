#include "vessel/AisHealth.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace opennav::vessel;
using namespace std::chrono_literals;
namespace {
void Check(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
AisTarget Report(Time at) {
  AisTarget target;
  target.mmsi = 123456789;
  target.active = true;
  target.latitude_deg = {57, "OpenCPN AIS / MMSI 123456789", at,
                         Validity::Measured, {15s, 60s}};
  target.longitude_deg = {16, target.latitude_deg.source, at,
                          Validity::Measured, {15s, 60s}};
  return target;
}
} // namespace
int main() {
  try {
    const Time now{100s};
    AisState state;
    Check(AssessAisReports(state, now) == AisReportHealth::Unavailable,
          "Missing AIS model remains unavailable");
    state.available = true;
    state.observed_at = now;
    Check(AssessAisReports(state, now) == AisReportHealth::Empty,
          "An empty available decoder does not establish receiver connectivity");
    state.targets = {Report(now)};
    Check(AssessAisReports(state, now) == AisReportHealth::Current,
          "A valid received position report establishes current target data");
    Check(AssessAisReports(state, now + 20s) == AisReportHealth::Current,
          "Aging report stays usable within the established AIS threshold");
    state.observed_at = now + 60s;
    state.targets[0].observed_at = state.observed_at;
    Check(AssessAisReports(state, now + 60s) == AisReportHealth::Stale,
          "Fresh model and target copies cannot renew the position report age");
    Check(state.targets[0].latitude_deg.observed_at == now,
          "Assessment does not change retained report timestamps");
    state.targets[0] = Report(now + 60s);
    Check(AssessAisReports(state, now + 60s) == AisReportHealth::Current,
          "Only a newly received valid report restores current target data");
    state.targets[0].lost = true;
    state.targets[0].latitude_deg = {};
    state.targets[0].longitude_deg = {};
    Check(AssessAisReports(state, now + 60s) == AisReportHealth::Lost,
          "Upstream lost target stays explicit after its coordinates are withheld");
    for (int invalid = 0; invalid < 8; ++invalid) {
      auto target = Report(now);
      switch (invalid) {
      case 0: target.doubtful = true; break;
      case 1: target.active = false; break;
      case 2: target.latitude_deg.value = 91; break;
      case 3: target.longitude_deg.value = std::numeric_limits<double>::quiet_NaN(); break;
      case 4: target.longitude_deg.value.reset(); break;
      case 5: target.latitude_deg.source = "another source"; break;
      case 6: target.longitude_deg.observed_at -= 1s; break;
      case 7: target.latitude_deg.validity = Validity::Invalid; break;
      }
      state.targets = {target};
      Check(AssessAisReports(state, now) == AisReportHealth::Unusable,
            "Invalid or incoherent target data is not current reception");
      Check(AssessAisReports(state, now + 120s) == AisReportHealth::Unusable,
            "Invalid or incoherent data does not become a valid stale report");
    }
    state.targets = {Report(now + 1s)};
    Check(AssessAisReports(state, now) == AisReportHealth::Unusable,
          "A future report cannot establish current reception");
    state.targets = {Report(now - 60s), Report(now)};
    state.targets.back().mmsi += 1;
    Check(AssessAisReports(state, now) == AisReportHealth::Current,
          "One current target is sufficient even when other retained targets are stale");
    state.targets.clear();
    Check(AssessAisReports(state, now) == AisReportHealth::Empty,
          "Removed targets do not leave a fabricated connection status");
    state.targets.resize(2001, Report(now));
    Check(AssessAisReports(state, now) == AisReportHealth::Unavailable,
          "An over-bound target set is not assessed as current");
    Check(std::string(AisReportHealthName(AisReportHealth::Empty)) == "No targets received" &&
          std::string(AisReportHealthName(AisReportHealth::Current)) == "Targets current" &&
          std::string(AisReportHealthName(AisReportHealth::Stale)) == "Targets stale",
          "User labels describe report health without claiming a transport connection");
    std::cout << "AIS report health, stale retention and no-connectivity-inference passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
