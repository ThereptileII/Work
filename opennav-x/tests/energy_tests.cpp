#include "smartnav/Energy.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace opennav;
using namespace std::chrono_literals;
using smartnav::EnergyReason;
static void Require(bool pass, const char* why) { if (!pass) throw std::runtime_error(why); }
static bool Near(double a, double b) { return std::abs(a - b) <= 1e-9 * std::max(1.0, std::abs(b)); }

int main() {
  try {
    const vessel::Time now{100s};
    const auto measured = [now](double value) {
      return vessel::Sample{value, "synthetic fixture", now, vessel::Validity::Measured};
    };
    const smartnav::EnergyModel model{20, 20, .5, "synthetic 20 kWh linear model"};
    const smartnav::EnergyInputs base{measured(80), measured(5), measured(2), measured(10)};
    const auto predict = [&](const smartnav::EnergyInputs& in) { return smartnav::PredictEnergy(model, in, now); };
    auto p = predict(base);
    Require(p.range.estimate && p.arrival.estimate, "Complete valid inputs produce estimates");
    Require(p.calculated_at == now && p.model_source == model.source, "Calculation time and model source retained");
    Require(Near(p.range.estimate->energy_above_reserve_kwh, 12), "Reserve energy deducted");
    Require(Near(p.range.estimate->endurance_hours, 6) && Near(p.range.estimate->range_nm, 30),
            "kWh / kW gives hours; knots * hours gives nautical miles");
    Require(Near(p.arrival.estimate->passage_hours, 2) && Near(p.arrival.estimate->energy_required_kwh, 4),
            "Distance and total discharge produce passage energy");
    Require(Near(*p.arrival.estimate->soc_percent, 60) && !p.arrival.estimate->below_reserve,
            "Arrival SOC uses capacity, not remaining reserve energy as denominator");
    auto in = base;
    in.distance_remaining_nm = measured(30);
    p = predict(in);
    Require(Near(*p.arrival.estimate->soc_percent, 20) && !p.arrival.estimate->below_reserve,
            "Arrival exactly at reserve boundary");
    in.distance_remaining_nm = measured(35);
    p = predict(in);
    Require(p.arrival.estimate->soc_percent && p.arrival.estimate->below_reserve,
            "Arrival below reserve stays an explicit advisory");
    in.distance_remaining_nm = measured(45);
    p = predict(in);
    Require(!p.arrival.estimate->soc_percent && Near(p.arrival.estimate->energy_shortfall_kwh, 2),
            "Exhaustion before arrival reports deficit, not negative or invented arrival SOC");
    in = base; in.soc_percent = measured(10);
    p = predict(in);
    Require(p.range.estimate->range_nm == 0 && p.arrival.estimate->below_reserve,
            "Below reserve gives zero budget, never negative range");
    in = base; in.sog_kn = measured(0);
    Require(predict(in).range.reason == EnergyReason::NotUnderway, "Stationary is not an infinite ETA");
    in.distance_remaining_nm = measured(0); in.total_discharge_kw = {};
    p = predict(in);
    Require(p.arrival.estimate && p.arrival.estimate->passage_hours == 0 &&
            p.arrival.estimate->soc_percent == 80, "Already-arrived case needs no speed or power forecast");
    for (double power : {0.0, -2.0}) {
      in = base; in.total_discharge_kw = measured(power);
      Require(predict(in).range.reason == EnergyReason::NotDischarging,
              "Zero discharge or regeneration does not imply infinite range");
    }
    for (double soc : {-1.0, 101.0, std::numeric_limits<double>::infinity()}) {
      in = base; in.soc_percent = measured(soc);
      Require(predict(in).range.reason == EnergyReason::InvalidInput, "Invalid SOC rejected");
    }
    in = base; in.distance_remaining_nm = {};
    p = predict(in);
    Require(p.range.estimate && p.arrival.reason == EnergyReason::MissingInput, "Range does not invent a route");
    in = base; in.distance_remaining_nm = measured(-1);
    Require(predict(in).arrival.reason == EnergyReason::InvalidInput, "Negative route distance rejected");
    in = base; in.distance_remaining_nm.observed_at -= 5s;
    p = predict(in);
    Require(p.range.estimate && p.arrival.reason == EnergyReason::StaleInput, "Stale route stops arrival only");
    in = base; in.total_discharge_kw.observed_at -= 5s;
    Require(predict(in).range.reason == EnergyReason::StaleInput, "Stale power suppresses range");
    in = base; in.soc_percent.validity = vessel::Validity::Uncertain;
    Require(predict(in).arrival.reason == EnergyReason::UncertainInput, "Uncertain SOC cannot support arrival");
    in = base; in.sog_kn.observed_at += 1s;
    Require(predict(in).range.reason == EnergyReason::UncertainInput, "Future input time rejected");
    in = base; in.total_discharge_kw.source.clear();
    Require(!predict(in).range.estimate, "Unsourced power is not silently used");
    Require(smartnav::PredictEnergy({}, base, now).range.reason == EnergyReason::InvalidModel,
            "No default boat capacity or reserve assumption");
    auto bad = model; bad.reserve_soc_percent = -1;
    Require(smartnav::PredictEnergy(bad, base, now).range.reason == EnergyReason::InvalidModel,
            "Invalid reserve rejected");
    bad = model; bad.capacity_kwh = 0;
    Require(!smartnav::PredictEnergy(bad, base, now).range.estimate, "Zero capacity cannot be used");
    bad = model; bad.reserve_soc_percent = 101;
    Require(!smartnav::PredictEnergy(bad, base, now).range.estimate, "Reserve above full capacity rejected");
    bad = model; bad.source.clear();
    Require(!smartnav::PredictEnergy(bad, base, now).range.estimate, "Capacity model must have provenance");
    in = base; in.sog_kn = measured(.49);
    Require(predict(in).range.reason == EnergyReason::NotUnderway, "Below model speed floor is suppressed");
    in.sog_kn = measured(.5);
    Require(predict(in).range.estimate.has_value(), "Speed at configured floor is allowed");
    in = base; in.total_discharge_kw = measured(std::numeric_limits<double>::denorm_min());
    Require(predict(in).range.reason == EnergyReason::ArithmeticLimit, "Overflow cannot become infinite range");
    in = base; in.distance_remaining_nm = measured(std::numeric_limits<double>::max());
    in.total_discharge_kw = measured(100);
    Require(predict(in).arrival.reason == EnergyReason::ArithmeticLimit, "Overflow cannot become an arrival");

    // Conservation and monotonicity across a grid, independent of UI formatting.
    for (int capacity = 1; capacity <= 100; capacity += 3) {
      for (int soc = 21; soc <= 100; soc += 7) {
        const smartnav::EnergyModel m{double(capacity), 20, .5, "grid"};
        in = base; in.soc_percent = measured(soc);
        const auto a = smartnav::PredictEnergy(m, in, now);
        in.total_discharge_kw = measured(4);
        const auto b = smartnav::PredictEnergy(m, in, now);
        Require(Near(a.range.estimate->range_nm, 2 * b.range.estimate->range_nm),
                "Doubling total discharge halves constant-condition range");
        Require(Near(a.range.estimate->endurance_hours * 2,
                     capacity * (soc - 20) / 100.0), "Range obeys energy conservation");
      }
    }
    std::cout << "Advisory range, reserve, arrival SOC and failure-state contracts passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
