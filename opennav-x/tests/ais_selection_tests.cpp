#include "vessel/AisSelection.h"
#include <iostream>
#include <stdexcept>
using namespace opennav::vessel;
using namespace std::chrono_literals;
void Check(bool b, const char *s) {
  if (!b)
    throw std::runtime_error(s);
}
int main() {
  try {
    const Time now{100s};
    AisTarget t;
    t.mmsi = 123456789;
    t.active = true;
    t.latitude_deg = {10, "upstream AIS", now, Validity::Measured};
    t.longitude_deg = {179.9, "upstream AIS", now, Validity::Measured};
    AisState s;
    s.available = true;
    s.targets.push_back(t);
    AisSelection selection;
    Check(selection.Select(t.mmsi, s, now) && selection.Selected(now) == t.mmsi,
          "Valid live target");
    s.targets.clear();
    Check(selection.Selected(now) == t.mmsi,
          "Retained copy survives original deletion");
    selection.Observe(s, now);
    Check(!selection.Selected(now), "Reported deletion clears selection");
    s.targets = {t};
    selection.Select(t.mmsi, s, now);
    Check(!selection.Selected(now + 5s), "Reading cannot renew age");
    t.latitude_deg.observed_at += 1s;
    s.targets = {t};
    Check(!selection.Select(t.mmsi, s, now + 1s),
          "Incoherent position rejected");
    t.latitude_deg.observed_at = now;
    t.longitude_deg.value = -179.9;
    s.targets = {t};
    Check(selection.Select(t.mmsi, s, now),
          "Antimeridian coordinate copied not reinterpreted");
    s.targets.push_back(t);
    Check(!selection.Select(t.mmsi, s, now), "Duplicate MMSI ambiguity");
    s.targets = {t};
    s.simulated = true;
    Check(!selection.Select(t.mmsi, s, now),
          "Demo cannot select real chart AIS");
    s.simulated = false;
    for (int mode = 0; mode < 3; ++mode) {
      s.targets = {t};
      if (mode == 0)
        s.targets[0].lost = true;
      if (mode == 1)
        s.targets[0].doubtful = true;
      if (mode == 2)
        s.targets[0].active = false;
      Check(!selection.Select(t.mmsi, s, now), "Invalid target state");
    }
    s.targets = {t};
    selection.Select(t.mmsi, s, now);
    s.targets[0].latitude_deg.observed_at -= 1s;
    s.targets[0].longitude_deg.observed_at -= 1s;
    selection.Observe(s, now);
    Check(!selection.Selected(now),
          "Out-of-order target cannot replace selected copy");
    s.targets = {t};
    s.targets[0].latitude_deg.value = 91;
    Check(!selection.Select(t.mmsi, s, now), "Invalid latitude");
    s.targets = {t};
    s.targets[0].longitude_deg.value.reset();
    Check(!selection.Select(t.mmsi, s, now), "Missing longitude");
    std::cout << "AIS selection lifetime and validity passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
