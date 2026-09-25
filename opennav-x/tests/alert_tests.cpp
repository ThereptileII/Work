#include "application/Alerts.h"
#include "smartnav/VesselEnergy.h"
#include "vessel/DemoSource.h"
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
void Check(bool ok, const char *m) {
  if (!ok)
    throw std::runtime_error(m);
}
const application::Alert *Find(const application::AlertCenter &c,
                               const char *id) {
  for (const auto &a : c.Current())
    if (a.id == id)
      return &a;
  return nullptr;
}
int main(int argc, char **argv) {
  try {
    Check(argc == 2, "group");
    std::string group = argv[1];
    const vessel::Time now{100s};
    application::AlertCenter center;
    application::AlertInput s;
    s.now = now;
    auto sample = [&](double v) {
      return vessel::Sample{v, "fixture", s.now, vessel::Validity::Measured};
    };
    if (group == "sensors") {
      center.Observe(s);
      Check(center.Current().empty(), "No alarms for never-installed sensors");
      s.vessel.navigation.latitude_deg = sample(50);
      s.vessel.navigation.longitude_deg = sample(5);
      s.vessel.navigation.heading_true_deg = sample(50);
      s.vessel.environment.depth_below_transducer_m = sample(0);
      s.vessel.wind.apparent_speed_kn = sample(0);
      s.vessel.wind.apparent_angle_deg = sample(0);
      s.vessel.propulsion.motor_rpm = sample(0);
      s.vessel.propulsion.motor_temperature_c = sample(22);
      s.vessel.battery.soc_percent = sample(0);
      s.vessel.battery.voltage_v = sample(340);
      s.vessel.battery.current_native_a = sample(0);
      s.vessel.rudder.angle_deg = sample(0);
      center.Observe(s);
      Check(center.Current().empty(), "Valid zero is not sensor loss");
      s.now += 2s;
      center.Observe(s);
      Check(center.Current().empty(), "Aging accepted");
      s.now += 4s;
      center.Observe(s);
      Check(center.Current().size() == 11,
            "Independent dropout conditions plus one GPS alert");
      Check(center.Current().front().level == application::AlertLevel::Critical,
            "GPS priority");
      const auto a = *Find(center, "depth");
      Check(center.Acknowledge(a.id, a.episode), "Ack episode");
      center.Observe(s);
      Check(Find(center, "depth")->acknowledged, "Ack retained while active");
      s.vessel.environment.depth_below_transducer_m = sample(4);
      center.Observe(s);
      Check(!Find(center, "depth"), "Recovery resolves");
      s.vessel.environment.depth_below_transducer_m.value.reset();
      center.Observe(s);
      Check(Find(center, "depth") && !Find(center, "depth")->acknowledged,
            "New episode unacknowledged");
      Check(!center.Acknowledge(a.id, a.episode),
            "Old event cannot ack new fault");
      s.vessel.environment.depth_below_transducer_m = sample(4);
      s.vessel.environment.depth_below_transducer_m.observed_at += 1s;
      center.Observe(s);
      Check(Find(center, "depth"), "Future timestamp rejected");
    } else if (group == "alarms") {
      s.ais.available = true;
      center.Observe(s);
      Check(center.Current().empty(),
            "No traffic does not prove receiver failure");
      vessel::AisTarget t;
      t.upstream_alarm = true;
      s.ais.targets.push_back(t);
      s.anchor.alarm = true;
      center.Observe(s);
      Check(Find(center, "ais-alarm") && Find(center, "anchor-alarm"),
            "Uses upstream flags");
      center.Acknowledge("ais-alarm", Find(center, "ais-alarm")->episode);
      Check(s.ais.targets[0].upstream_alarm && s.anchor.alarm,
            "UI ack never clears upstream");
      s.pilot.fresh = true;
      s.pilot.feedback.mode = adapters::PilotMode::Auto;
      center.Observe(s);
      s.pilot.fresh = false;
      s.pilot.enabled = false;
      center.Observe(s);
      Check(Find(center, "pilot-feedback"),
            "Lost engaged pilot remains critical even after output disabled");
      s.pilot.command.state = adapters::CommandState::TimedOut;
      center.Observe(s);
      Check(Find(center, "pilot-timeout"),
            "Unconfirmed command outcome explicit");
      s.pilot.fresh = true;
      s.pilot.feedback.mode = adapters::PilotMode::Standby;
      s.pilot.command.state = adapters::CommandState::Confirmed;
      center.Observe(s);
      Check(!Find(center, "pilot-feedback") && !Find(center, "pilot-timeout"),
            "New physical standby feedback resolves");
      vessel::DemoSource demo(now);
      demo.Select(vessel::DemoScenario::Insufficient, now);
      s.vessel = demo.Read(now);
      s.energy = smartnav::PredictVesselEnergy(
          smartnav::PreviewEnergyModel(true), s.vessel, now);
      center.Observe(s);
      Check(Find(center, "energy-shortfall"), "Actual tested model shortage");
      s.vessel.navigation.route.reset();
      center.Observe(s);
      Check(!Find(center, "energy-shortfall"),
            "Old route cannot produce alert");
    } else if (group == "lifecycle") {
      s.vessel.environment.depth_below_transducer_m = sample(5);
      center.Observe(s);
      s.now += 6s;
      center.Observe(s);
      Check(Find(center, "depth"), "Known loss");
      s.vessel = {};
      s.vessel.simulated = true;
      center.Observe(s);
      Check(center.Current().empty(), "Live history does not leak to Demo");
      s.vessel.environment.depth_below_transducer_m = sample(5);
      center.Observe(s);
      s.now += 6s;
      center.Observe(s);
      Check(Find(center, "depth"), "Demo also ages honestly");
      s.vessel = {};
      s.vessel.replayed = true;
      s.pilot.command.state = adapters::CommandState::TimedOut;
      center.Observe(s);
      Check(center.Current().empty(), "Replay isolates pilot and histories");
      s.vessel.environment.depth_below_transducer_m = sample(5);
      center.Observe(s);
      s.now += 6s;
      center.Observe(s);
      Check(Find(center, "depth"), "Replay dropout");
      s.now = now;
      s.vessel = {};
      s.vessel.replayed = true;
      center.Observe(s);
      Check(center.Current().empty(), "Replay rewind resets health episode");
      for (int i = 0; i < 100000; ++i)
        center.Observe(s);
      Check(center.Current().empty(), "Repeated reads do not invent events");
    } else
      Check(false, "unknown group");
    std::cout << "Alert " << group << " passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
