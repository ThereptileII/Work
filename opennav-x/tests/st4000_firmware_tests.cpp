#include "BridgeCore.h"
#include "adapters/St4000Pilot.h"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace opennav::adapters;
void Check(bool v, const char *why) {
  if (!v)
    throw std::runtime_error(why);
}
using Status = std::array<std::uint8_t, 9>;
// Independent SeaTalk 0x84 fixtures: actual heading 90 degrees, and locked
// headings 80/89/90/91/100. These are host inputs, not captured boat evidence.
const Status standby{0x84, 0x16, 0x40, 0, 0, 0, 0, 0, 0};
const Status automatic{0x84, 0x16, 0x40, 0, 2, 0, 0, 0, 0};
void Receive(bridge::Controller &controller, const Status &status,
             std::uint32_t at) {
  controller.receivePilot(status.data(), status.size(), at);
}
void PhysicalStyleResponse(const bridge::Command &command, unsigned key,
                           const Status &response, double target) {
  bridge::Controller controller;
  const auto &before = command.type == bridge::CommandType::Mode &&
                               command.mode == bridge::Mode::Auto
                           ? standby : automatic;
  Receive(controller, before, 100);
  Check(controller.pilot.valid && controller.pilot.heading == 90.,
        "Physical-style status fixture provides measured heading");
  Check(controller.request(command, 110) && controller.completed == 0,
        "Parsed request queues without claiming physical success");
  bridge::Datagram output;
  Check(controller.next(120, output) && output.size == 4 &&
            output.data[0] == 0x86 && output.data[1] == 0x21 &&
            output.data[2] == key && output.data[3] == (key ^ 0xff),
        "Each parsed command generates the corresponding SeaTalk key");
  controller.started(120);
  auto no_echo = controller;
  no_echo.transmitted(false, 130);
  Check(no_echo.failed == 1 && no_echo.completed == 0 &&
            !no_echo.next(140, output),
        "Failed wire echo does not confirm or retry a command");
  controller.transmitted(true, 130);
  Check(controller.phase == bridge::Phase::Confirming &&
            controller.completed == 0 && !controller.next(140, output),
        "Wire echo alone never confirms or emits another key");
  Receive(controller, before, 150);
  Check(controller.completed == 0 && controller.phase == bridge::Phase::Confirming,
        "New unchanged feedback does not confirm the requested change");
  auto malformed = response;
  malformed[0] = 0;
  Receive(controller, malformed, 160);
  Check(controller.completed == 0 && controller.phase == bridge::Phase::Confirming,
        "Malformed matching-looking feedback cannot confirm");
  auto missing = controller;
  missing.tick(4000);
  Check(missing.completed == 0 && missing.failed == 1 &&
            !missing.next(4010, output),
        "Lost physical feedback fails without retry");
  Receive(controller, response, 170);
  Check(controller.completed == 1 && controller.failed == 0 &&
            controller.phase == bridge::Phase::Idle,
        "Only subsequent matching SeaTalk status confirms each command");
  Check(controller.pilot.mode == (command.type == bridge::CommandType::Mode
                                      ? command.mode : bridge::Mode::Auto),
        "Confirmed mode comes from received status");
  Check(std::isnan(target) ? std::isnan(controller.pilot.target)
                           : controller.pilot.target == target,
        "Confirmed locked heading comes from received status");
  Check(!controller.next(180, output), "Confirmed command sends no extra key");
}

void ModesNeedingBridgeInputs() {
  for (const auto action : {PilotAction::Track, PilotAction::Wind}) {
    // SKAGER now sends these; the firmware, not SKAGER, owns readiness.
    const auto bytes = EncodeSt4000Command({1, action, 0, {}});
    Check(bytes == std::vector<std::uint8_t>{
                       1, 0x63, 0xff, 0, 0xff, 3, 1, 0x3b, 7, 3, 4, 6,
                       static_cast<std::uint8_t>(action == PilotAction::Track ? 0x80 : 1)},
          "SKAGER TRACK/WIND match the firmware's legacy button field");
    bridge::Command command;
    Check(bridge::parseCommand(bytes.data(), bytes.size(), command) &&
              command.type == bridge::CommandType::Mode &&
              command.mode == (action == PilotAction::Track ? bridge::Mode::Track
                                                           : bridge::Mode::Wind),
          "Actual firmware distinguishes TRACK and WIND requests");
    bridge::Controller controller;
    Receive(controller, automatic, 100);
    Check(!controller.request(command, 110) && controller.rejected == 1 &&
              controller.completed == 0,
          "TRACK/WIND cannot engage without fresh navigation/wind prerequisites");
    bridge::Datagram output;
    Check(!controller.next(120, output), "Rejected mode emits no SeaTalk command");
  }
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
        if (action == PilotAction::Standby)
          PhysicalStyleResponse(result, 2, standby, NAN);
        else if (action == PilotAction::Auto)
          PhysicalStyleResponse(result, 1, automatic, 90.);
        else if (delta == -10)
          PhysicalStyleResponse(result, 6, {0x84, 0x16, 0, 160, 2, 0, 0, 0, 0}, 80.);
        else if (delta == -1)
          PhysicalStyleResponse(result, 5, {0x84, 0x16, 0, 178, 2, 0, 0, 0, 0}, 89.);
        else if (delta == 1)
          PhysicalStyleResponse(result, 7, {0x84, 0x16, 0x40, 2, 2, 0, 0, 0, 0}, 91.);
        else
          PhysicalStyleResponse(result, 8, {0x84, 0x16, 0x40, 20, 2, 0, 0, 0, 0}, 100.);
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
    ModesNeedingBridgeInputs();
    std::cout
        << "PASS pinned ST4000 parser/controller: six manual commands, synthetic SeaTalk feedback, TRACK/WIND boundaries\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
