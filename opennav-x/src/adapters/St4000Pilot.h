#pragma once
#include "adapters/Autopilot.h"
#include <array>
#include <functional>

namespace opennav::adapters {
// The explicitly configured boat translator, not arbitrary Raymarine equipment.
// NAME is the canonical 16-digit hex value from PGN 60928, not an OpenCPN
// message-address label. No CAN or SeaTalk transport is owned by this adapter.
struct St4000Binding {
  std::string interface, name;
  bool permit_control = false;
};
void ValidateSt4000Binding(const St4000Binding &binding);
std::uint64_t ParsePilotName(const std::string &name);
std::string FormatPilotName(std::uint64_t name);
struct PilotTransportStatus {
  bool connected = false, writable = false;
  std::uint64_t epoch = 0;
  std::string detail;
};
class IN2kPilotTransport {
public:
  virtual ~IN2kPilotTransport() = default;
  virtual PilotTransportStatus Status(const std::string &interface) const = 0;
  // true means attempted/accepted transport only, NEVER physical confirmation.
  virtual bool Send(const std::string &interface, std::uint8_t destination,
                    std::uint32_t pgn, std::uint8_t priority,
                    const std::vector<std::uint8_t> &data) = 0;
};
struct PilotN2kFrame {
  std::string interface;
  std::uint32_t pgn = 0;
  std::uint8_t source = 255;
  std::vector<std::uint8_t> data;
  vessel::Time observed_at{};
};
// Pure protocol encoder, validated against the pinned boat firmware parser.
// Deliberately has no TRACK/WIND command representation in the live contract.
std::vector<std::uint8_t> EncodeSt4000Command(const PilotRequest &request);

class St4000Pilot final : public IAutopilot {
public:
  explicit St4000Pilot(IN2kPilotTransport &transport) : transport_(transport) {}
  void Configure(const St4000Binding &binding);
  void Observe(const PilotN2kFrame &frame, vessel::Time now);
  PilotCapabilities Capabilities() const override;
  PilotFeedback GetState() const override { return feedback_; }
  void Poll(vessel::Time now) override;
  bool Send(const PilotRequest &request) override;
  // Human-initiated, read-only ISO address-claim request. Does not enable pilot
  // control. Rate limited and only on the explicitly configured connection.
  bool RequestIdentity(vessel::Time now);
  std::string Status() const;
  std::optional<std::uint8_t> Address() const { return address_; }

private:
  void Invalidate(const std::string &why);
  std::string Source() const;
  IN2kPilotTransport &transport_;
  St4000Binding binding_;
  std::uint64_t wanted_name_ = 0, transport_epoch_ = 0;
  PilotFeedback feedback_;
  std::optional<std::uint8_t> address_;
  std::array<std::optional<vessel::Time>, 254> claims_{};
  std::optional<vessel::Time> last_send_, last_identity_request_;
  bool connected_ = false, conflict_ = false;
  std::string reason_ = "Translator not configured; control OFF";
};
} // namespace opennav::adapters
