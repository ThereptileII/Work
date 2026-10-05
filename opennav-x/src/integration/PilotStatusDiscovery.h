#pragma once
#include "adapters/St4000Pilot.h"
#include <map>
#include <memory>
#include <stdexcept>

namespace opennav::integration {
// Passive status from OpenCPN's existing receive connections. Each observed
// NAME still passes the strict boat adapter's identity, epoch, source and PGN
// checks. Discovery never grants output permission or transmits a request.
class PilotStatusDiscovery final : private adapters::IN2kPilotTransport {
public:
  explicit PilotStatusDiscovery(adapters::IN2kPilotTransport &transport)
      : transport_(transport) {}

  void Observe(const adapters::PilotN2kFrame &frame, vessel::Time now) {
    if (frame.interface_id.empty() || frame.interface_id.size() > 200 ||
        frame.source >= 254 ||
        frame.observed_at < vessel::Time{} || frame.observed_at > now ||
        now - frame.observed_at >= std::chrono::seconds(3) ||
        !Status(frame.interface_id).connected)
      return;
    for (const unsigned char c : frame.interface_id)
      if (c < 32 || c == 127) return;
    if (frame.pgn == 60928 && frame.data.size() == 8) {
      std::uint64_t name = 0;
      for (unsigned i = 0; i < 8; ++i)
        name |= std::uint64_t(frame.data[i]) << (8 * i);
      const auto text = adapters::FormatPilotName(name);
      try { adapters::ParsePilotName(text); }
      catch (const std::invalid_argument &) { name = 0; }
      if (name) {
        const auto key = std::make_pair(frame.interface_id, text);
        if (!candidates_.count(key)) {
          // Bound unsolicited discovery. An overflow remains unavailable until
          // a new bridge session rather than arbitrarily selecting a device.
          if (candidates_.size() >= 32) {
            overflow_ = true;
            return;
          }
          auto candidate = std::make_unique<Candidate>(
              static_cast<adapters::IN2kPilotTransport &>(*this));
          candidate->pilot.Configure({frame.interface_id, text, false});
          candidates_.emplace(key, std::move(candidate));
        }
      }
    }
    for (auto &[key, candidate] : candidates_) {
      if (key.first != frame.interface_id) continue;
      candidate->pilot.Observe(frame, now);
      const auto feedback = candidate->pilot.GetState();
      if (Fresh(feedback, now)) candidate->last_observed = feedback;
    }
  }

  void Poll(vessel::Time now) {
    for (auto &[key, candidate] : candidates_) candidate->pilot.Poll(now);
  }

  adapters::PilotFeedback GetState(vessel::Time now) const {
    if (overflow_) return {};
    const Candidate *selected = nullptr;
    for (const auto &[key, candidate] : candidates_) {
      const auto feedback = candidate->pilot.GetState();
      if (!Fresh(feedback, now)) continue;
      if (selected) return {}; // Multiple physical pilots: no inferred choice.
      selected = candidate.get();
    }
    if (selected) return selected->pilot.GetState();
    // Preserve the fact of previously accepted feedback for a degraded display,
    // but never retain its mode/headings as current after connection loss.
    for (const auto &[key, candidate] : candidates_) {
      if (!candidate->last_observed.sequence) continue;
      if (selected) return {};
      selected = candidate.get();
    }
    if (!selected) return {};
    auto lost = selected->last_observed;
    lost.mode = adapters::PilotMode::Unavailable;
    lost.heading_magnetic_deg = {};
    lost.locked_heading_magnetic_deg = {};
    return lost;
  }

  std::string Description(vessel::Time now) const {
    if (overflow_) return "Pilot discovery limit exceeded; status unavailable";
    unsigned fresh = 0;
    for (const auto &[key, candidate] : candidates_)
      if (Fresh(candidate->pilot.GetState(), now)) ++fresh;
    if (fresh > 1)
      return "Multiple pilots provide live feedback; pilot status is ambiguous";
    const auto state = GetState(now);
    if (Fresh(state, now)) return "Live pilot feedback / " + state.source;
    if (state.sequence)
      return "Pilot feedback stale, invalid or connection lost / " + state.source;
    return "Waiting for a compatible address claim and physical pilot feedback on an OpenCPN receive connection";
  }

private:
  struct Candidate {
    explicit Candidate(adapters::IN2kPilotTransport &transport) : pilot(transport) {}
    adapters::St4000Pilot pilot;
    adapters::PilotFeedback last_observed;
  };
  static bool Fresh(const adapters::PilotFeedback &feedback, vessel::Time now) {
    return feedback.mode != adapters::PilotMode::Unavailable &&
           feedback.sequence && !feedback.source.empty() &&
           feedback.observed_at <= now &&
           now - feedback.observed_at < std::chrono::seconds(3);
  }
  adapters::PilotTransportStatus Status(const std::string &iface) const override {
    auto status = transport_.Status(iface);
    status.writable = false;
    return status;
  }
  bool Send(const std::string &, std::uint8_t, std::uint32_t, std::uint8_t,
            const std::vector<std::uint8_t> &) override { return false; }
  adapters::IN2kPilotTransport &transport_;
  std::map<std::pair<std::string, std::string>, std::unique_ptr<Candidate>> candidates_;
  bool overflow_ = false;
};
} // namespace opennav::integration
