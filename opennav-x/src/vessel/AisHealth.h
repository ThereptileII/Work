#pragma once
#include "vessel/AisState.h"

namespace opennav::vessel {
// Report health is not evidence of receiver/transport connectivity. Copying or
// reading an AIS model must never refresh the age of its received reports.
enum class AisReportHealth { Unavailable, Empty, Current, Stale, Lost, Unusable };
AisReportHealth AssessAisReports(const AisState &, Time now);
const char *AisReportHealthName(AisReportHealth);
} // namespace opennav::vessel
