#include "integration/PilotTrafficDiagnostics.h"
#include <iostream>
#include <stdexcept>

using namespace opennav;
using namespace opennav::integration;
using namespace std::chrono_literals;
namespace {
const vessel::Time start{100s};
void Check(bool value, const char *why) {
  if (!value) throw std::runtime_error(why);
}
std::vector<std::uint8_t> Envelope(std::uint32_t pgn, std::uint8_t source = 204) {
  const unsigned length = pgn == 126720 ? 13 : 8;
  std::vector<std::uint8_t> bytes(14 + length, 0);
  bytes[0] = 0x93;
  bytes[1] = 11 + length;
  bytes[2] = 3;
  bytes[3] = pgn & 255;
  bytes[4] = (pgn >> 8) & 255;
  bytes[5] = (pgn >> 16) & 255;
  bytes[6] = 255;
  bytes[7] = source;
  bytes[12] = length;
  return bytes;
}
void DiscoveryEvidence() {
  PilotTrafficDiagnostics traffic;
  for (const auto pgn : {127250u, 65379u, 65359u, 126720u, 65360u})
    Check(traffic.Observe("COM8", pgn, Envelope(pgn), start, start),
          "Known received envelopes count without a NAME or decoded mode");
  auto before_claim = traffic.GetSnapshot();
  Check(before_claim.sources.size() == 5 && before_claim.accepted_count == 5,
        "Vendor status traffic remains distinguishable from heading alone");
  for (const auto &source : before_claim.sources)
    Check(source.pgn != 60928, "Missing claim remains explicitly absent");
  auto claim = Envelope(60928);
  Check(traffic.Observe("COM8", 60928, claim, start + 1ms, start + 1ms),
        "An envelope can be counted without inventing or validating its NAME");
  claim.assign(100, 255);
  Check(before_claim.sources.size() == 5 && traffic.GetSnapshot().sources.size() == 6,
        "Snapshots own their data and the collector does not retain input buffers");
  Check(traffic.Observe("COM8", 65379, Envelope(65379, 105), start, start),
        "A foreign valid source is counted separately, not assumed to be the pilot");
  Check(traffic.Observe("other", 65379, Envelope(65379), start, start),
        "Identical source numbers on different interfaces remain separate");
  Check(traffic.Observe("COM8", 65379, Envelope(65379, 253), start, start),
        "Envelope diagnostics follow the declared source less than 254 boundary");
  Check(traffic.GetSnapshot().sources.size() == 9,
        "Interface, PGN and CAN source all participate in the diagnostic key");
}
void EnvelopeRejections() {
  PilotTrafficDiagnostics traffic;
  auto bytes = Envelope(65379);
  bytes[0] = 0x94;
  Check(!traffic.Observe("COM8", 65379, bytes, start, start), "TX echo rejected");
  bytes = Envelope(65379); bytes.pop_back();
  Check(!traffic.Observe("COM8", 65379, bytes, start, start), "Truncation rejected");
  bytes = Envelope(65379); bytes.push_back(0);
  Check(!traffic.Observe("COM8", 65379, bytes, start, start), "Extra bytes rejected");
  bytes = Envelope(65379); bytes[1] = 0;
  Check(!traffic.Observe("COM8", 65379, bytes, start, start), "Wrong packet length rejected");
  bytes = Envelope(65379); bytes[12] = 0;
  Check(!traffic.Observe("COM8", 65379, bytes, start, start), "Zero data rejected");
  Check(!traffic.Observe("COM8", 65379, {}, start, start), "Empty envelope rejected");
  Check(!traffic.Observe("COM8", 65379, Envelope(65360), start, start),
        "PGN does not match listener rejected");
  Check(!traffic.Observe("COM8", 65379, Envelope(65379, 254), start, start),
        "Invalid CAN source rejected");
  Check(!traffic.Observe("COM8", 65379, Envelope(65379, 255), start, start),
        "Broadcast address cannot be a received CAN source");
  Check(!traffic.Observe("COM8", 123, Envelope(123), start, start), "Unknown PGN rejected");
  Check(!traffic.Observe("bad\niface", 65379, Envelope(65379), start, start),
        "Invalid interface rejected");
  const auto snapshot = traffic.GetSnapshot();
  Check(snapshot.sources.empty() && snapshot.accepted_count == 0 &&
            snapshot.rejected.invalid_type == 1 && snapshot.rejected.invalid_length == 5 &&
            snapshot.rejected.pgn_mismatch == 1 && snapshot.rejected.invalid_source == 2 &&
            snapshot.rejected.unsupported_pgn == 1 && snapshot.rejected.invalid_interface == 1,
        "Each malformed observation has one explicit rejection reason and no accepted bucket");
}
void TimeBounds() {
  PilotTrafficDiagnostics traffic;
  const auto bytes = Envelope(65379);
  Check(traffic.Observe("COM8", 65379, bytes, start, start), "Initial RX accepted");
  Check(!traffic.Observe("COM8", 65379, bytes, start + 10s, start + 1s),
        "Future timestamp rejected before it can refresh a bucket");
  Check(!traffic.Observe("COM8", 65379, bytes, start - 1ms, start + 1s),
        "Out-of-order observation rejected");
  Check(!traffic.Observe("COM8", 65379, bytes, start, start + 1s),
        "Duplicate timestamp rejected");
  Check(!traffic.Observe("COM8", 65379, bytes, start, start + 3s),
        "Exactly three seconds old is stale");
  Check(!traffic.Observe("COM8", 65379, bytes, vessel::Time{} - 1ms, start),
        "Negative observation time rejected");
  auto snapshot = traffic.GetSnapshot();
  Check(snapshot.sources[0].last_observed == start && snapshot.accepted_count == 1 &&
            snapshot.rejected.future == 1 && snapshot.rejected.out_of_order == 2 &&
            snapshot.rejected.stale == 1 && snapshot.rejected.invalid_time == 1,
        "Rejected timestamps never change accepted counters or observation time");
  Check(traffic.Observe("COM8", 65379, bytes, start + 1s, start + 3999ms),
        "A valid observation still works after a rejected future timestamp");
  Check(traffic.GetSnapshot().sources[0].last_observed == start + 1s,
        "Only an accepted observation refreshes last time");
}
void BoundedFlood() {
  PilotTrafficDiagnostics traffic;
  for (unsigned source = 0; source < 65; ++source)
    Check(traffic.Observe("COM8", 65379, Envelope(65379, source), start, start) ==
              (source < 64), "At most 64 source/PGN buckets are retained");
  Check(traffic.Observe("COM8", 65379, Envelope(65379, 0), start + 1ms, start + 1ms),
        "Capacity exhaustion does not prevent updates to an existing bucket");
  const auto snapshot = traffic.GetSnapshot();
  Check(snapshot.sources.size() == 64 && snapshot.accepted_count == 65 &&
            snapshot.sources[0].accepted_count == 2 && snapshot.rejected.capacity == 1,
        "Flood overflow is counted explicitly without unbounded allocation");
  std::uint64_t counter = std::numeric_limits<std::uint64_t>::max() - 1;
  pilot_traffic_detail::Increment(counter);
  pilot_traffic_detail::Increment(counter);
  Check(counter == std::numeric_limits<std::uint64_t>::max(),
        "Diagnostic counters saturate instead of wrapping");
}
} // namespace
int main() {
  try {
    DiscoveryEvidence();
    EnvelopeRejections();
    TimeBounds();
    BoundedFlood();
    std::cout << "Passive pilot traffic diagnostics tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
