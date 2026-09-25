#include "diagnostics/Calibration.h"
#include "diagnostics/Commissioning.h"
#include "diagnostics/Recorder.h"
#include "diagnostics/Recording.h"
#include "diagnostics/Replay.h"
#include "vessel/DataItems.h"
#include "vessel/DemoSource.h"
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
using namespace opennav;
using namespace std::chrono_literals;
void Check(bool v, const char *message) {
  if (!v)
    throw std::runtime_error(message);
}
void Reject(const std::function<void()> &f) {
  bool caught = false;
  try {
    f();
  } catch (const std::invalid_argument &) {
    caught = true;
  }
  Check(caught, "Malformed recording admitted");
}
diagnostics::Recording Fixture(bool navigation = true) {
  diagnostics::Recording r;
  r.navigation_included = navigation;
  r.assumptions.energy.battery = {48, 20, .5, "Test pack"};
  r.assumptions.energy.battery_device_id = "pack-one";
  const vessel::Time start{100s};
  auto state = vessel::DemoFixture(vessel::DemoScenario::Cruise, 0, start);
  state.battery.net_discharge_kw.device_id =
      state.battery.soc_percent.device_id = "pack-one";
  state.battery.soc_percent.source = "N2K åäö / battery";
  for (int i = 0; i < 8; ++i)
    r.frames.push_back(
        diagnostics::CaptureFrame(state, start + i * 1s, start, navigation));
  return r;
}
void RoundTrip() {
  auto r = Fixture();
  const auto bytes = diagnostics::EncodeRecording(r);
  auto decoded = diagnostics::DecodeRecording(bytes);
  Check(diagnostics::EncodeRecording(decoded) == bytes,
        "Lossless normalized roundtrip");
  const auto &frame = decoded.frames.back();
  Check(frame.state.navigation.route->route_id ==
            r.frames.back().state.navigation.route->route_id,
        "Route identity");
  Check(frame.state.navigation.route->remaining_steps.size() == 3,
        "Route progress steps");
  Check(frame.state.battery.soc_percent.observed_at == vessel::Time{} &&
            frame.elapsed == 7s,
        "Original observation retained");
  Check(frame.state.battery.soc_percent.source == "N2K åäö / battery",
        "UTF-8 provenance");
  const vessel::Time origin{500s};
  const auto replay = diagnostics::ReplayFrame(frame, origin);
  Check(replay.replayed && replay.simulated,
        "Replay and original simulation identified separately");
  Check(vessel::Assess(replay.battery.soc_percent, origin + frame.elapsed)
                .quality == vessel::Quality::Stale,
        "Replay preserves dropout");
  Check(vessel::AssessRoute(*replay.navigation.route, origin + frame.elapsed)
                .quality == vessel::Quality::Stale,
        "Route age preserved");
  const auto prediction = smartnav::PredictConfiguredEnergy(
      decoded.assumptions.energy, replay, origin + frame.elapsed);
  Check(!prediction.arrival.estimate, "Dropout does not produce arrival SOC");
  decoded.frames[0].state.simulated = false;
  const auto liveReplay = diagnostics::ReplayFrame(decoded.frames[0], origin);
  Check(liveReplay.replayed && !liveReplay.simulated,
        "Recorded live input retains identity requirements");
  auto badConfig = decoded.assumptions.energy;
  badConfig.battery_device_id = "wrong pack";
  Check(!smartnav::PredictConfiguredEnergy(badConfig, liveReplay, origin)
             .arrival.estimate,
        "Replay cannot bypass device identity");
  const auto retained = replay.navigation.route;
  decoded.frames.clear();
  r.frames.clear();
  Check(retained->remaining_steps.size() == 3, "Owned route lifetime");
  Reject([&] { diagnostics::CaptureFrame(replay, origin + 7s, origin, true); });
}
void Privacy() {
  auto r = Fixture(false);
  r.assumptions.sources[vessel::Quantity::Depth].pinned_source =
      "private connection";
  r.assumptions.energy.curve.source = "C:/Personal/curve.csv";
  r.assumptions.pilot = {"private pilot interface", "c0508700e76004d2", true};
  const auto decoded =
      diagnostics::DecodeRecording(diagnostics::EncodeRecording(r));
  Check(!decoded.frames[0].state.navigation.route &&
            !decoded.frames[0].state.navigation.latitude_deg.value &&
            !decoded.frames[0].state.navigation.longitude_deg.value,
        "Navigation omitted by default");
  Check(decoded.frames[0].state.navigation.sog_kn.value.has_value(),
        "Speed retained for calibration");
  Check(decoded.assumptions.sources.empty() &&
            decoded.assumptions.pilot.interface.empty() &&
            !decoded.assumptions.pilot.permit_control &&
            decoded.assumptions.energy.curve.source != "C:/Personal/curve.csv",
        "Only selected model assumptions retained");
  auto bytes = diagnostics::EncodeRecording(Fixture(true));
  bytes[bytes.find('\n') - 1] = '0';
  Reject([&] { diagnostics::DecodeRecording(bytes); });
}
void Malformed() {
  const auto base = diagnostics::EncodeRecording(Fixture());
  for (const auto &bad :
       {std::string{}, base.substr(0, base.size() - 1), base + "extra\n",
        std::string(diagnostics::RecordingByteLimit + 1, 'a')})
    Reject([&] { diagnostics::DecodeRecording(bad); });
  auto mutate = [&](const std::string &from, const std::string &to) {
    auto s = base;
    auto p = s.find(from);
    Check(p != std::string::npos, "Mutation target");
    s.replace(p, from.size(), to);
    Reject([&] { diagnostics::DecodeRecording(s); });
  };
  mutate("OpenNavXRecording\t1", "OpenNavXRecording\t99");
  mutate("F\t1000\t1", "F\t0\t1");
  mutate("F\t0\t1", "F\t9223372036854775808\t1");
  mutate("END\t8", "END\t9");
  mutate("E\n", "E\tbroken\n");
  auto sampleStart = base.find("S\t"), sampleEnd = base.find('\n', sampleStart);
  const auto row = base.substr(sampleStart, sampleEnd - sampleStart + 1);
  mutate(row, row + row);
  mutate(row, "Unknown\t1\n");
  auto fields = row;
  auto nameEnd = fields.find('\t', 2);
  fields.replace(2, nameEnd - 2, "ff");
  mutate(row, fields);
  fields = row;
  auto unitEnd = fields.find('\t', nameEnd + 1);
  fields.replace(nameEnd + 1, unitEnd - nameEnd - 1, "6d");
  mutate(row, fields);
  auto numeric = [&](const std::string &value) {
    auto line = row;
    auto a = line.find('\t', 2);
    a = line.find('\t', a + 1);
    auto b = line.find('\t', a + 1);
    line.replace(a + 1, b - a - 1, value);
    mutate(row, line);
  };
  for (const char *bad : {"nan", "inf", "1e999", "91", "-91"})
    numeric(bad);
  auto mixed = Fixture();
  mixed.frames[1].state.simulated = false;
  Reject([&] { diagnostics::EncodeRecording(mixed); });
  auto r = Fixture();
  r.frames[1].elapsed = 0ms;
  Reject([&] { diagnostics::EncodeRecording(r); });
  r = Fixture();
  r.frames[0].state.battery.voltage_v.value =
      std::numeric_limits<double>::infinity();
  Reject([&] { diagnostics::EncodeRecording(r); });
  r = Fixture();
  auto route = std::make_shared<vessel::RouteProgressSnapshot>(
      *r.frames[0].state.navigation.route);
  route->remaining_steps[1].waypoint_id = route->remaining_steps[0].waypoint_id;
  r.frames[0].state.navigation.route = route;
  Reject([&] { diagnostics::EncodeRecording(r); });
  r = Fixture();
  r.frames[0].state.battery.voltage_v.observed_at = vessel::Time{1s};
  Reject([&] { diagnostics::EncodeRecording(r); });
  r = Fixture();
  r.frames[0].state.battery.voltage_v.freshness = {2s, 1s};
  Reject([&] { diagnostics::EncodeRecording(r); });
  r = Fixture();
  r.frames[0].state.battery.voltage_v.source = std::string(4097, 'a');
  Reject([&] { diagnostics::EncodeRecording(r); });
}
void Replay() {
  const vessel::Time now{500s};
  auto r = Fixture();
  diagnostics::ReplaySession session(r, now);
  Check(session.Read(now).state.replayed, "Replay explicit at start");
  session.Pause(true, now + 1s);
  const auto paused = session.Read(now + 20s);
  Check(paused.paused && paused.elapsed == 1s && paused.now == now + 1s,
        "Pause freezes explicit replay clock");
  Check(paused.state.battery.soc_percent.observed_at == now,
        "Pause does not renew observed time");
  session.Pause(false, now + 20s);
  auto end = session.Read(now + 30s);
  Check(end.ended && end.elapsed == 11s &&
            vessel::Assess(end.state.battery.soc_percent, end.now).quality ==
                vessel::Quality::Stale,
        "End ages retained data");
  session.Seek(0ms, now + 30s);
  Check(session.Read(now + 30s).state.battery.soc_percent.observed_at == now,
        "Seek preserves original relative stamp");
  Reject([&] { session.Seek(-1ms, now + 30s); });
  Reject([&] { session.Seek(20s, now + 30s); });
  Reject([&] { session.Read(now); });
  r.frames.erase(r.frames.begin(), r.frames.begin() + 6);
  diagnostics::ReplaySession segment(r, now);
  const auto first = segment.Read(now);
  Check(first.elapsed == 0ms && first.duration == 1s &&
            first.state.battery.soc_percent.value.has_value(),
        "Rotated segment begins immediately at its first frame");
  Check(vessel::Assess(first.state.battery.soc_percent, first.now).quality ==
            vessel::Quality::Stale,
        "Segment start never renews pre-segment observations");
  segment.Seek(0ms, now + 1s);
  Check(segment.Read(now + 1s).elapsed == 0ms,
        "Segment rewind uses local offset");
}
void Calibration() {
  auto r = Fixture();
  const auto result = diagnostics::ExportCalibration(
      r, {smartnav::SpeedReference::OverGround, smartnav::PowerBasis::WholePack,
          "pack-one"});
  Check(result.pairs == 1 && result.duplicate == 2 && result.rejected == 5,
        "No duplicate or old calibration pairs");
  Check(result.csv.find("basis,whole-pack") != std::string::npos &&
            result.csv.find("reference,SOG") != std::string::npos,
        "Explicit power and speed basis");
  Check(result.csv.find("DEMO") != std::string::npos,
        "Synthetic calibration labelled");
  auto wrong = diagnostics::ExportCalibration(
      r, {smartnav::SpeedReference::OverGround, smartnav::PowerBasis::WholePack,
          "other pack"});
  Check(wrong.pairs == 0 && wrong.rejected == 8, "No mixing devices");
  r.frames[0].state.battery.net_discharge_kw.value = -5;
  r.frames[1].state.battery.net_discharge_kw.observed_at = vessel::Time{-3s};
  r.frames[2].state.battery.net_discharge_kw.validity =
      vessel::Validity::Uncertain;
  Check(diagnostics::ExportCalibration(r, {smartnav::SpeedReference::OverGround,
                                           smartnav::PowerBasis::WholePack,
                                           "pack-one"})
                .pairs == 0,
        "Charging, incoherent and uncertain pairs omitted");
}
void Storage() {
  namespace fs = std::filesystem;
  const auto root =
      fs::temp_directory_path() /
      ("opennav-recording-test-" +
       std::to_string(vessel::Clock::now().time_since_epoch().count()));
  fs::create_directory(root);
  try {
    const vessel::Time start{100s};
    const auto sample =
        vessel::DemoFixture(vessel::DemoScenario::Cruise, 0, start);
    diagnostics::Recorder recorder(root, {}, false, start, {2, 3, 1});
    for (int i = 0; i < 8; ++i) {
      Check(recorder.Capture(sample, start + i * 1s), "Frame accepted");
      const auto deadline = vessel::Clock::now() + 3s;
      while (recorder.Status().published < static_cast<std::size_t>(i + 1) &&
             recorder.Status().error.empty() && vessel::Clock::now() < deadline)
        std::this_thread::sleep_for(1ms);
      Check(recorder.Status().published == static_cast<std::size_t>(i + 1),
            "Async checkpoint published");
      Check(!recorder.Capture(sample, start + i * 1s),
            "Capture cadence bounded");
    }
    recorder.Stop();
    const auto status = recorder.Status();
    Check(status.directory.parent_path() == fs::canonical(root),
          "Session uses the resolved application-owned root");
    Check(!status.active && status.error.empty() && status.published == 8 &&
              status.segments == 3,
          "Bounded rotation finished");
    Check(!fs::exists(status.directory / "segment-0.onxr") &&
              fs::exists(status.directory / "segment-1.onxr"),
          "Oldest owned segment removed");
    const auto latest =
        diagnostics::LoadRecording(status.directory / "segment-3.onxr");
    Check(latest.frames.size() == 2 && latest.frames.back().elapsed == 7s,
          "Committed file replayable");
    Check(!latest.frames.front().state.navigation.route,
          "Disk default privacy");
    diagnostics::Recorder fail(root, {}, false, start);
    auto failureDirectory = fail.Status().directory;
    fs::rename(failureDirectory, root / "moved-by-test");
    Check(fail.Capture(sample, start), "IO failure queued asynchronously");
    fail.Stop();
    Check(!fail.Status().error.empty() && !fail.Status().active,
          "Storage loss stops visibly");
    Check(!fs::exists(failureDirectory),
          "Recorder does not silently recreate a lost session path");
    Reject([&] { diagnostics::LoadRecording(root); });
    {
      std::ofstream truncated(root / "broken.onxr");
      truncated << "OpenNavXRecording\t1\t0\n";
    }
    Reject([&] { diagnostics::LoadRecording(root / "broken.onxr"); });
    Check(fs::exists(status.directory / "segment-1.onxr"),
          "Failed second session preserves previous evidence");
  } catch (...) {
    fs::remove_all(root);
    throw;
  }
  fs::remove_all(root);
}
void ControlBoundary() {
  namespace fs = std::filesystem;
  const auto root =
      fs::temp_directory_path() /
      ("opennav-replay-test-" +
       std::to_string(vessel::Clock::now().time_since_epoch().count()));
  fs::create_directory(root);
  try {
    const vessel::Time now{500s};
    const auto file = root / "record.onxr";
    {
      std::ofstream out(file, std::ios::binary);
      out << diagnostics::EncodeRecording(Fixture());
    }
    diagnostics::Commissioning service(root);
    Check(service.AllowsHardwareControl(),
          "Live state still requires separate adapter enablement");
    Check(service.OpenReplay(file, now).ok && !service.AllowsHardwareControl(),
          "Replay forbids every OpenNav hardware command");
    Check(!service.StartRecording({}, false, now).ok,
          "No relabelling recorded data as new capture");
    Check(!service.OpenReplay(root / "missing", now).ok && service.Replaying(),
          "Invalid replay cannot silently restore live controls");
    int count = 0;
    application::NavigationActions actions;
    actions.activate = [&](const auto &) {
      ++count;
      return application::CommandResult{true, "ok"};
    };
    actions.create_waypoint = [&](auto, const auto &, const auto &) {
      ++count;
      return application::CommandResult{true, "ok"};
    };
    actions.start_route = [&] { ++count; };
    actions.legacy_settings = [&] { ++count; };
    actions = application::GuardNavigationChanges(
        std::move(actions), [&] { return !service.Replaying(); });
    Check(!actions.activate({}).ok && !actions.create_waypoint({}, "", "").ok,
          "Replay blocks real navigation mutations");
    actions.start_route();
    actions.legacy_settings();
    Check(count == 0, "Replay blocks advanced mutation entry");
    service.StopReplay();
    Check(actions.activate({}).ok && count == 1,
          "Explicit replay exit restores normal human actions");
    diagnostics::Commissioning denied(
        root, [] { return "Output-capable connection active"; });
    Check(!denied.OpenReplay(file, now).ok && !denied.Replaying(),
          "Transport preflight refuses replay without altering live state");
  } catch (...) {
    fs::remove_all(root);
    throw;
  }
  fs::remove_all(root);
}
int main(int argc, char **argv) {
  try {
    const std::string test = argc > 1 ? argv[1] : "";
    if (test == "roundtrip")
      RoundTrip();
    else if (test == "privacy")
      Privacy();
    else if (test == "malformed")
      Malformed();
    else if (test == "replay")
      Replay();
    else if (test == "calibration")
      Calibration();
    else if (test == "storage")
      Storage();
    else if (test == "control")
      ControlBoundary();
    else
      throw std::runtime_error("Unknown test");
    std::cout << "Recording " << test << " passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
