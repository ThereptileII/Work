#pragma once
#include "vessel/VesselState.h"

namespace opennav::ais {
// Supplemental Internet-traffic display policy, not onboard AIS or navigation
// sensor freshness. Service/receipt age does not prove transponder position age.
// All consumers retain the original observation; reads/static reports never
// restart these intervals. Transport health remains separate and immediate.
inline constexpr vessel::Duration OnlineAgingAfter = std::chrono::minutes(3);
inline constexpr vessel::Duration OnlineStaleAfter = std::chrono::minutes(10);
inline constexpr vessel::Duration OnlineLostAfter = std::chrono::minutes(20);
inline constexpr vessel::Duration OnlineExpireAfter = std::chrono::minutes(30);
inline constexpr vessel::Freshness OnlinePositionFreshness{
    OnlineAgingAfter, OnlineStaleAfter};
} // namespace opennav::ais
