#pragma once
#include "vessel/VesselState.h"
namespace opennav::test {
void EnableObjectScenario(const std::string &profile);
void ObjectScenarioStep(const vessel::Navigation &selected);
} // namespace opennav::test
