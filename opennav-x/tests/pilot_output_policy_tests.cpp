#include "integration/PilotOutputPolicy.h"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace opennav::integration;
namespace {
void Check(bool value, const char *why) {
  if (!value) throw std::runtime_error(why);
}
}
int main() {
  try {
    PilotOutputEndpoint local{true,true,true,true,true,"127.0.0.1","127.0.0.1",32000,32000};
    int sends = 0;
    auto send = [&] { ++sends; return true; };
    const bool test = PilotLoopbackTestsEnabled();
    Check(DispatchPilotOutput(local, send) == test, "Product must reject even a fully valid loopback endpoint");
    Check(sends == (test ? 1 : 0), "Denied sink never invokes callback");
    Check(HardwareOutputPolicy() == (test ? "test-loopback-only" : "status-only"), "Executed capability is explicit");
    if (test) {
      Check(TestFixturesEnabled() && BuildPurpose() == "DEVELOPER TEST BUILD", "Output test build cannot impersonate product");
      Check(!DispatchPilotOutput(local, [] { return false; }), "Transport failure is not upgraded to success");
    }
    std::vector<PilotOutputEndpoint> denied;
    for (int flag = 0; flag < 5; ++flag) {
      auto e = local;
      switch (flag) {
      case 0: e.tcp=false; break;
      case 1: e.bidirectional=false; break;
      case 2: e.enabled=false; break;
      case 3: e.connected=false; break;
      case 4: e.actisense=false; break;
      }
      denied.push_back(e);
    }
    // No DNS, interface-name tricks, LAN address, alternate loopback spelling,
    // mapped IPv6 or stale socket peer can upgrade the test transport.
    for (const auto *address : {"", "localhost", "127.0.0.2", "127.1", "::1", "::ffff:127.0.0.1",
                                "192.168.0.10", "100.64.0.1", "TCP:127.0.0.1:32000", "127.0.0.1 "}) {
      auto e = local; e.configured_address=address; denied.push_back(e);
      e=local; e.peer_address=address; denied.push_back(e);
    }
    for (int port : {-1,0,65536,32001}) {
      auto e=local; e.peer_port=port; denied.push_back(e);
      e=local; e.configured_port=port; denied.push_back(e);
    }
    const int before=sends;
    for (const auto &e : denied) {
      Check(!PilotOutputPermitted(e), "Malformed/remote/changed endpoint denied");
      Check(!DispatchPilotOutput(e, send), "Sink independently rejects invalid endpoint");
    }
    Check(sends == before, "No callbacks for any denied endpoint");
    for (const auto *variable : {"XNAV_ENABLE_PILOT_LOOPBACK_TESTS", "OPENNAV_HARDWARE_OUTPUT", "pilot.permission"}) {
#ifdef _WIN32
      Check(_putenv_s(variable,"1") == 0,"Test environment set");
#else
      Check(setenv(variable,"1",1) == 0,"Test environment set");
#endif
      Check(PilotOutputPermitted(local) == test,"Runtime environment cannot upgrade build policy");
    }
    std::cout << HardwareOutputPolicy() << ": transport sink denial matrix passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
