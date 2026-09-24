#include "smartnav/EnergyConfiguration.h"
#include "vessel/RouteProgress.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace opennav::smartnav {
namespace {
const char *Reference(SpeedReference r) {
  return r == SpeedReference::ThroughWater ? "STW" : "SOG";
}
const char *Basis(PowerBasis b) {
  return b == PowerBasis::WholePack         ? "whole-pack"
         : b == PowerBasis::MotorElectrical ? "motor-electrical"
                                            : "shaft";
}
bool Fresh(const vessel::Sample &s, vessel::Time now) {
  const auto q = vessel::Assess(s, now).quality;
  return q == vessel::Quality::Live || q == vessel::Quality::Aging ||
         q == vessel::Quality::Estimated;
}
} // namespace
bool ValidPowerCurve(const PowerCurve &c) {
  if (c.source.empty() || c.source.size() > 1024 || c.points.size() < 2 ||
      c.points.size() > 512 ||
      (c.reference != SpeedReference::ThroughWater &&
       c.reference != SpeedReference::OverGround) ||
      (c.basis != PowerBasis::WholePack &&
       c.basis != PowerBasis::MotorElectrical && c.basis != PowerBasis::Shaft))
    return false;
  double previous = -1;
  for (const auto &p : c.points) {
    if (!std::isfinite(p.speed_kn) || !std::isfinite(p.power_kw) ||
        p.speed_kn < 0 || p.speed_kn > 200 || p.power_kw <= 0 ||
        p.power_kw > 10000 || p.speed_kn <= previous)
      return false;
    previous = p.speed_kn;
  }
  return true;
}
PowerCurve ImportPowerCurve(const std::string &csv, const std::string &source) {
  if (csv.size() > 32768)
    throw std::invalid_argument("Power curve exceeds 32 KiB");
  std::istringstream input(csv);
  input.imbue(std::locale::classic());
  auto line = [&] {
    std::string value;
    if (!std::getline(input, value))
      throw std::invalid_argument("Incomplete power curve");
    if (!value.empty() && value.back() == '\r')
      value.pop_back();
    return value;
  };
  if (line() != "OpenNavXPowerCurve,1")
    throw std::invalid_argument("Unknown power curve version");
  PowerCurve c;
  c.source = source;
  const auto r = line();
  if (r == "reference,STW")
    c.reference = SpeedReference::ThroughWater;
  else if (r == "reference,SOG")
    c.reference = SpeedReference::OverGround;
  else
    throw std::invalid_argument("Power curve must specify STW or SOG");
  const auto b = line();
  if (b == "basis,whole-pack")
    c.basis = PowerBasis::WholePack;
  else if (b == "basis,motor-electrical")
    c.basis = PowerBasis::MotorElectrical;
  else if (b == "basis,shaft")
    c.basis = PowerBasis::Shaft;
  else
    throw std::invalid_argument("Power curve must specify its power basis");
  if (line() != "speed_kn,power_kw")
    throw std::invalid_argument("Power curve units must be knots and kW");
  std::string row;
  while (std::getline(input, row)) {
    if (!row.empty() && row.back() == '\r')
      row.pop_back();
    std::istringstream fields(row);
    fields.imbue(std::locale::classic());
    CurvePoint p{};
    char comma = 0;
    if (!(fields >> p.speed_kn >> comma >> p.power_kw) || comma != ',' ||
        !(fields >> std::ws).eof())
      throw std::invalid_argument("Invalid speed/power curve row");
    c.points.push_back(p);
  }
  if (!ValidPowerCurve(c))
    throw std::invalid_argument("Curve needs 2..512 finite positive power "
                                "samples with strictly increasing speeds");
  return c;
}
std::string ExportPowerCurve(const PowerCurve &c) {
  if (!ValidPowerCurve(c))
    throw std::invalid_argument("Cannot export invalid curve");
  std::ostringstream s;
  s.imbue(std::locale::classic());
  s << std::setprecision(17);
  s << "OpenNavXPowerCurve,1\nreference," << Reference(c.reference)
    << "\nbasis," << Basis(c.basis) << "\nspeed_kn,power_kw\n";
  for (const auto &p : c.points)
    s << p.speed_kn << ',' << p.power_kw << '\n';
  return s.str();
}
std::optional<double> InterpolatePower(const PowerCurve &c, double speed) {
  if (!ValidPowerCurve(c) || !std::isfinite(speed) ||
      speed < c.points.front().speed_kn || speed > c.points.back().speed_kn)
    return {};
  const auto hi = std::lower_bound(
      c.points.begin(), c.points.end(), speed,
      [](const CurvePoint &p, double s) { return p.speed_kn < s; });
  if (hi == c.points.begin() || hi->speed_kn == speed)
    return hi->power_kw;
  const auto lo = hi - 1;
  const double fraction =
      (speed - lo->speed_kn) / (hi->speed_kn - lo->speed_kn);
  const double power = lo->power_kw + fraction * (hi->power_kw - lo->power_kw);
  return std::isfinite(power) && power > 0 ? std::optional<double>{power}
                                           : std::nullopt;
}
EnergyPrediction PredictConfiguredEnergy(const EnergyConfiguration &c,
                                         const vessel::VesselState &s,
                                         vessel::Time now) {
  EnergyInputs input{s.battery.soc_percent,
                     s.navigation.sog_kn,
                     s.battery.net_discharge_kw,
                     {}};
  if (s.navigation.route)
    input.distance_remaining_nm =
        vessel::RouteDistanceSample(*s.navigation.route, now);
  auto fail = [&](EnergyReason reason) {
    EnergyPrediction p;
    p.range.reason = p.arrival.reason = reason;
    p.model_source = c.battery.source;
    p.calculated_at = now;
    return p;
  };
  if (!s.simulated && (c.battery_device_id.empty() ||
                       input.soc_percent.device_id != c.battery_device_id))
    return fail(EnergyReason::MissingInput);
  auto model = c.battery;
  if (c.consumption == ConsumptionModel::CalibratedCurve) {
    if (!ValidPowerCurve(c.curve))
      return fail(EnergyReason::InvalidModel);
    const auto &speed = c.curve.reference == SpeedReference::ThroughWater
                            ? s.navigation.stw_kn
                            : s.navigation.sog_kn;
    if (!Fresh(speed, now))
      return fail(vessel::Assess(speed, now).quality == vessel::Quality::Stale
                      ? EnergyReason::StaleInput
                      : EnergyReason::MissingInput);
    auto power = InterpolatePower(c.curve, *speed.value);
    if (!power)
      return fail(EnergyReason::InvalidInput);
    if (c.curve.basis != PowerBasis::WholePack) {
      if (!std::isfinite(c.hotel_kw) || c.hotel_kw < 0 || c.hotel_kw > 10000)
        return fail(EnergyReason::InvalidModel);
      if (c.curve.basis == PowerBasis::Shaft) {
        if (!std::isfinite(c.shaft_efficiency) || c.shaft_efficiency <= 0 ||
            c.shaft_efficiency > 1)
          return fail(EnergyReason::InvalidModel);
        *power /= c.shaft_efficiency;
      }
      *power += c.hotel_kw;
    }
    if (!std::isfinite(*power))
      return fail(EnergyReason::ArithmeticLimit);
    input.total_discharge_kw = speed;
    input.total_discharge_kw.value = power;
    input.total_discharge_kw.validity = vessel::Validity::Estimated;
    input.total_discharge_kw.source = c.curve.source + " / current " +
                                      Reference(c.curve.reference) +
                                      " / configured consumption estimate";
    model.source += "; curve: " + c.curve.source + "; " +
                    Reference(c.curve.reference) + "; " + Basis(c.curve.basis);
  } else if (c.consumption != ConsumptionModel::MeasuredPack)
    return fail(EnergyReason::InvalidModel);
  else if (!s.simulated &&
           input.total_discharge_kw.device_id != c.battery_device_id)
    return fail(EnergyReason::MissingInput);
  auto prediction = PredictEnergy(model, input, now);
  prediction.input_route = s.navigation.route;
  return prediction;
}
} // namespace opennav::smartnav
