#pragma once
#include "smartnav/Energy.h"
#include <vector>

namespace opennav::smartnav {
enum class SpeedReference { ThroughWater, OverGround };
enum class PowerBasis { WholePack, MotorElectrical, Shaft };
struct CurvePoint {
  double speed_kn, power_kw;
};
struct PowerCurve {
  SpeedReference reference = SpeedReference::ThroughWater;
  PowerBasis basis = PowerBasis::WholePack;
  std::vector<CurvePoint> points;
  std::string source;
};
// Strict, bounded and locale-independent. Throws for malformed/ambiguous data.
PowerCurve ImportPowerCurve(const std::string &csv, const std::string &source);
std::string ExportPowerCurve(const PowerCurve &curve);
bool ValidPowerCurve(const PowerCurve &curve);
std::optional<double> InterpolatePower(const PowerCurve &curve,
                                       double speed_kn);

enum class ConsumptionModel { MeasuredPack, CalibratedCurve };
struct EnergyConfiguration {
  EnergyModel battery;
  ConsumptionModel consumption = ConsumptionModel::MeasuredPack;
  std::string battery_device_id;
  PowerCurve curve;
  double hotel_kw = std::numeric_limits<double>::quiet_NaN();
  double shaft_efficiency = std::numeric_limits<double>::quiet_NaN();
};
// Live identity checks precede the existing tested core. No inferred battery,
// capacity, reserve, efficiency, current sign, curve or hotel-load defaults.
EnergyPrediction PredictConfiguredEnergy(const EnergyConfiguration &config,
                                         const vessel::VesselState &state,
                                         vessel::Time now);
} // namespace opennav::smartnav
