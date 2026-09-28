#pragma once
#include "vessel/VesselState.h"
#include <vector>

namespace opennav::vessel {
enum class AisOrigin { LocalOpenCPN, AisStreamOnline };
enum class AisTimeBasis { OpenCPNReport, OnlineService, OnlineReceipt };
// Owned reports. Local metrics are copies of OpenCPN's result, not a second
// CPA/TCPA calculator. Supplemental online reports never fabricate these metrics.
struct AisTarget {
  int mmsi = 0;
  std::string name, status, source;
  bool active = false, lost = false, doubtful = false, upstream_alarm = false;
  Sample latitude_deg, longitude_deg, sog_kn, cog_deg, heading_true_deg;
  Sample range_nm, bearing_true_deg, cpa_nm, tcpa_minutes;
  Time observed_at{};
  AisOrigin origin = AisOrigin::LocalOpenCPN;
  // Online timestamps are service ingestion/receipt, never proof of actual
  // transponder observation time or Internet latency. Details must say so.
  AisTimeBasis time_basis = AisTimeBasis::OpenCPNReport;
  TextSample callsign, destination;
  Sample ship_type, navigation_status, length_m, beam_m;
};
struct AisState {
  std::vector<AisTarget> targets;
  std::string source;
  Time observed_at{};
  // Decoder/model availability only. Neither this flag nor the copy timestamp
  // establishes receiver connectivity or freshness of any target report.
  bool available = false, simulated = false;
};
} // namespace opennav::vessel
