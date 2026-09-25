#include "BridgeCore.h"
#include "adapters/St4000Pilot.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace opennav::adapters;
void Check(bool v, const char *why) {
  if (!v)
    throw std::runtime_error(why);
}
int main() {
  try {
    for (const auto action :
         {PilotAction::Standby, PilotAction::Auto, PilotAction::AlterCourse}) {
      const std::vector<double> deltas =
          action == PilotAction::AlterCourse
              ? std::vector<double>{-10, -1, 1, 10}
              : std::vector<double>{0};
      for (const auto delta : deltas) {
        const auto bytes = EncodeSt4000Command({1, action, delta, {}});
        bridge::Command result;
        Check(bridge::parseCommand(bytes.data(), bytes.size(), result),
              "Actual boat firmware must accept each emitted command");
        if (action == PilotAction::AlterCourse)
          Check(result.type == bridge::CommandType::Step &&
                    result.value == delta,
                "Firmware interprets exactly the requested manual increment");
        else
          Check(result.type == bridge::CommandType::Mode &&
                    result.mode == (action == PilotAction::Standby
                                        ? bridge::Mode::Standby
                                        : bridge::Mode::Auto),
                "Firmware interprets exactly the requested mode");
        for (std::size_t n = 0; n < bytes.size(); ++n)
          Check(!bridge::parseCommand(bytes.data(), n, result),
                "Every truncated command is rejected by the firmware");
        auto wrong = bytes;
        wrong[7] ^= 1;
        Check(!bridge::parseCommand(wrong.data(), wrong.size(), result),
              "Manufacturer is structural, not an incidental byte pattern");
        wrong = bytes;
        wrong[10] = 5;
        Check(!bridge::parseCommand(wrong.data(), wrong.size(), result),
              "Wrong industry is rejected");
      }
    }
    std::cout
        << "PASS exact pinned ST4000 firmware parser, six manual commands\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
