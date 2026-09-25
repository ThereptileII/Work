#pragma once
#include "diagnostics/Recording.h"
namespace opennav::diagnostics {
struct CalibrationSelection {
  smartnav::SpeedReference speed = smartnav::SpeedReference::ThroughWater;
  smartnav::PowerBasis power = smartnav::PowerBasis::WholePack;
  std::string
      device; // Explicit power device; never average different motors/packs.
};
struct CalibrationExport {
  std::string csv;
  std::size_t pairs = 0, rejected = 0, duplicate = 0;
};
// Reviewable observations, not an automatically installed/claimed boat curve.
CalibrationExport ExportCalibration(const Recording &,
                                    const CalibrationSelection &);
} // namespace opennav::diagnostics
