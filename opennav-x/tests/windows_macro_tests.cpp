#include "adapters/Autopilot.h"
#include <array>
#include <functional>
// The Windows SDK defines this COM convenience macro even in unrelated core
// translation units. Portable-only compilation previously missed the collision.
#define interface struct
#include "adapters/St4000Pilot.h"
int main() {
  opennav::adapters::St4000Binding binding;
  binding.interface_id="observed OpenCPN transport";
  opennav::adapters::PilotN2kFrame frame;
  frame.interface_id=binding.interface_id;
  return frame.interface_id==binding.interface_id?0:1;
}
