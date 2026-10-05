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
  struct Diagnostics {
    unsigned verified_identities = 0;
    unsigned fresh_mode_sources_without_identity = 0;
    unsigned stale_mode_sources = 0;
    unsigned identity_conflicts = 0;
    bool traffic_limit_exceeded = false;
    std::string conflict_source;
  };

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
    if (ValidMode(frame)) {
      const auto key = std::make_pair(frame.interface_id, frame.source);
      auto found = mode_traffic_.find(key);
      const auto epoch = Status(frame.interface_id).epoch;
      if (found != mode_traffic_.end()) {
        if (found->second.epoch != epoch ||
            frame.observed_at > found->second.observed_at)
          found->second = {frame.observed_at, epoch};
      } else if (mode_traffic_.size() < 32) {
        mode_traffic_.emplace(key, ModeTraffic{frame.observed_at, epoch});
      } else {
        traffic_limit_exceeded_ = true;
      }
    }
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
      // Separate a transport reset from an identity conflict. Only the adapter
      // decides whether a claim revokes an accepted address.
      candidate->pilot.Poll(now);
      const auto previous_address = candidate->pilot.Address();
      candidate->pilot.Observe(frame, now);
      if (frame.pgn == 60928 && previous_address &&
          !candidate->pilot.Address()) candidate->identity_conflict = true;
      if (candidate->pilot.Address())
        candidate->identity_epoch = Status(key.first).epoch;
      const auto feedback = candidate->pilot.GetState();
      if (Fresh(feedback, now)) candidate->last_observed = feedback;
    }
  }

  void Poll(vessel::Time now) {
    for (auto &[key, candidate] : candidates_) candidate->pilot.Poll(now);
  }

  Diagnostics GetDiagnostics(vessel::Time now) const {
    Diagnostics result;
    result.traffic_limit_exceeded = traffic_limit_exceeded_;
    for (const auto &[key, candidate] : candidates_) {
      const auto status = Status(key.first);
      if (candidate->pilot.Address() && status.connected &&
          candidate->identity_epoch == status.epoch)
        ++result.verified_identities;
      if (candidate->identity_conflict) {
        ++result.identity_conflicts;
        if (result.conflict_source.empty())
          result.conflict_source = key.first + "/NAME-" + key.second;
      }
    }
    for (const auto &[key, traffic] : mode_traffic_) {
      const auto status = Status(key.first);
      if (!status.connected || status.epoch != traffic.epoch ||
          traffic.observed_at > now) continue;
      if (now - traffic.observed_at >= std::chrono::seconds(3)) {
        ++result.stale_mode_sources;
        continue;
      }
      bool identified = false;
      for (const auto &[identity, candidate] : candidates_)
        if (identity.first == key.first &&
            candidate->identity_epoch == traffic.epoch &&
            candidate->pilot.Address() == key.second) identified = true;
      if (!identified) ++result.fresh_mode_sources_without_identity;
    }
    return result;
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
    const auto diagnostics = GetDiagnostics(now);
    if (diagnostics.identity_conflicts)
      return "Pilot address/NAME conflict; identity verification required / " +
             diagnostics.conflict_source;
    if (diagnostics.fresh_mode_sources_without_identity)
      return "Compatible pilot mode traffic received without a verified address claim; pilot identity and status remain unavailable";
    if (state.sequence)
      return "Pilot feedback stale, invalid or connection lost / " + state.source;
    if (diagnostics.verified_identities)
      return "Compatible address claim observed; waiting for valid physical pilot feedback";
    if (diagnostics.stale_mode_sources)
      return "Previously observed pilot mode traffic is stale; waiting for a compatible address claim and fresh physical feedback";
    if (diagnostics.traffic_limit_exceeded)
      return "Pilot traffic diagnostic limit exceeded; waiting for a compatible address claim and physical feedback";
    return "Waiting for a compatible address claim and physical pilot feedback on an OpenCPN receive connection";
  }

private:
  struct Candidate {
    explicit Candidate(adapters::IN2kPilotTransport &transport) : pilot(transport) {}
    adapters::St4000Pilot pilot;
    adapters::PilotFeedback last_observed;
    std::uint64_t identity_epoch = 0;
    bool identity_conflict = false;
  };
  struct ModeTraffic {
    vessel::Time observed_at;
    std::uint64_t epoch;
  };
  static bool ValidMode(const adapters::PilotN2kFrame &frame) {
    if (frame.pgn != 65379 || frame.data.size() != 8 ||
        frame.data[0] != 0x3b || frame.data[1] != 0x9f) return false;
    const unsigned mode = unsigned(frame.data[2]) | (unsigned(frame.data[3]) << 8);
    return mode == 0 || mode == 0x40 || mode == 0x100 || mode == 0x180;
  }
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
  // Diagnostics never create candidates, infer identity or enable output.
  std::map<std::pair<std::string, std::uint8_t>, ModeTraffic> mode_traffic_;
  bool traffic_limit_exceeded_ = false;
  bool overflow_ = false;
};
} // namespace opennav::integration
