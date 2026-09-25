#include "integration/N2kSourceIdentity.h"
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace integration;
using namespace std::chrono_literals;
const vessel::Time epoch{100s};
void Check(bool x, const char *why) {
  if (!x)
    throw std::runtime_error(why);
}
std::vector<unsigned char> Name(std::uint64_t n) {
  std::vector<unsigned char> data(8);
  for (unsigned i = 0; i < 8; ++i)
    data[i] = (n >> (i * 8)) & 255;
  return data;
}
int main() {
  try {
    N2kSourceIdentity ids;
    Check(ids.Label("bus", 35) == "bus/address-only",
          "Unknown NAME explicitly address-only");
    Check(ids.Observe("bus", 35, Name(0xc0508700ffc12345), epoch, epoch) ==
              ClaimResult::Changed,
          "Actual claim establishes NAME");
    Check(ids.Label("bus", 35) == "bus/NAME-c0508700ffc12345",
          "Unsigned full-width NAME preserved");
    const auto copy = ids.Observations();
    Check(ids.Observe("bus", 35, Name(0xc0508700ffc12345), epoch + 1s,
                      epoch + 1s) == ClaimResult::Unchanged,
          "Repeated identity is not a new device");
    Check(ids.Observe("bus", 35, Name(99), epoch, epoch + 1s) ==
              ClaimResult::Ignored,
          "Old claim cannot revert identity");
    Check(ids.Observe("bus", 35, Name(99), epoch + 2s, epoch + 2s) ==
              ClaimResult::Changed,
          "Address reuse must invalidate previous samples");
    Check(ids.Label("bus", 35) == "bus/NAME-0000000000000063",
          "New claim label");
    Check(copy[0].name == "c0508700ffc12345", "Owned previous copy survives");
    Check(ids.Observe("other", 35, Name(99), epoch + 2s, epoch + 2s) ==
              ClaimResult::Changed,
          "Different bus is isolated");
    Check(ids.Observe("bus", 36, Name(99), epoch + 3s, epoch + 3s) ==
                  ClaimResult::Conflict &&
              ids.Label("bus", 35).empty() && ids.Label("bus", 36).empty(),
          "Duplicate NAME fails closed");
    for (const auto address : {254u, 255u, 999u})
      Check(ids.Observe("bus", address, Name(5), epoch, epoch) ==
                ClaimResult::Ignored,
            "Reserved address");
    for (unsigned length = 0; length < 24; ++length) {
      if (length == 8)
        continue;
      Check(ids.Observe("bus", 10, std::vector<unsigned char>(length, 1), epoch,
                        epoch) == ClaimResult::Ignored,
            "Malformed claim length");
    }
    for (const auto n : {std::uint64_t(0), UINT64_MAX})
      Check(ids.Observe("bus", 10, Name(n), epoch, epoch) ==
                ClaimResult::Ignored,
            "Unavailable NAME");
    Check(ids.Observe("bus", 10, Name(3), epoch + 1s, epoch) ==
                  ClaimResult::Ignored &&
              ids.Observe("bus", 10, Name(3), epoch, epoch + 3s) ==
                  ClaimResult::Ignored,
          "Future and delayed claims rejected");
    ids.Clear();
    Check(ids.Label("bus", 35) == "bus/address-only" &&
              ids.Observations().empty(),
          "Driver lifecycle clears identity ownership");
    for (unsigned i = 0; i < 256; ++i)
      Check(ids.Observe("bus" + std::to_string(i), 1, Name(i + 1), epoch,
                        epoch) == ClaimResult::Changed,
            "Bounded identity table admission");
    Check(ids.Observe("overflow", 1, Name(999), epoch, epoch) ==
                  ClaimResult::Ignored &&
              ids.Observations().size() == 256,
          "Untrusted network cannot grow unbounded state");
    std::cout << "PASS N2K source identity\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
