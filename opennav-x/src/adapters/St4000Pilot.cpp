#include "adapters/St4000Pilot.h"
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace opennav::adapters {
namespace {
constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
unsigned U16(const std::vector<std::uint8_t> &p, unsigned i) {
  return unsigned(p[i]) | (unsigned(p[i + 1]) << 8);
}
bool ValidName(std::uint64_t n) {
  // Exact class/function/manufacturer/industry set by BridgeApp.cpp. Device
  // instance and unique number remain boat-specific; never guess the latter.
  return ((n >> 21) & 0x7ff) == 1851 && ((n >> 40) & 0xff) == 135 &&
         ((n >> 49) & 0x7f) == 40 && ((n >> 60) & 7) == 4;
}
bool VendorHeader(const std::vector<std::uint8_t> &p) {
  return U16(p, 0) == (1851 | (3u << 11) | (4u << 13));
}
vessel::Sample Angle(unsigned raw, const std::string &source, vessel::Time at) {
  vessel::Sample sample{{}, source, at, vessel::Validity::Invalid};
  if (raw <= 62832) {
    sample.value = std::fmod(raw * .0001 * radians_to_degrees, 360.0);
    sample.validity = vessel::Validity::Measured;
  }
  sample.freshness = {std::chrono::seconds(1), std::chrono::seconds(3)};
  return sample;
}
} // namespace
std::uint64_t ParsePilotName(const std::string &name) {
  if (name.size() != 16)
    throw std::invalid_argument(
        "Pilot NAME must contain 16 hexadecimal digits");
  std::uint64_t n = 0;
  for (const auto c : name) {
    unsigned digit;
    if (c >= '0' && c <= '9')
      digit = c - '0';
    else if (c >= 'a' && c <= 'f')
      digit = c - 'a' + 10;
    else
      throw std::invalid_argument("Pilot NAME must use lowercase hexadecimal");
    n = (n << 4) | digit;
  }
  if (!ValidName(n))
    throw std::invalid_argument(
        "NAME does not identify the supported ST4000 translator class");
  return n;
}
std::string FormatPilotName(std::uint64_t name) {
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << name;
  return out.str();
}
void ValidateSt4000Binding(const St4000Binding &b) {
  if (b.interface.empty() && b.name.empty() && !b.permit_control)
    return;
  if (b.interface.empty() || b.interface.size() > 200)
    throw std::invalid_argument("Select an existing OpenCPN N2K interface");
  for (const unsigned char c : b.interface)
    if (c < 32 || c == 127)
      throw std::invalid_argument("Invalid pilot interface text");
  ParsePilotName(b.name);
}
std::vector<std::uint8_t> EncodeSt4000Command(const PilotRequest &r) {
  if (!r.id || !std::isfinite(r.delta_deg))
    return {};
  unsigned key;
  if (r.action == PilotAction::Standby && r.delta_deg == 0)
    key = 0;
  else if (r.action == PilotAction::Auto && r.delta_deg == 0)
    key = 0x40;
  else if (r.action == PilotAction::AlterCourse) {
    if (r.delta_deg == 1)
      key = 0x51;
    else if (r.delta_deg == -1)
      key = 0x7f;
    else if (r.delta_deg == 10)
      key = 0xd1;
    else if (r.delta_deg == -10)
      key = 0x50;
    else
      return {};
  } else
    return {};
  // PGN126208 command, target65379, priority keep, three explicit parameters:
  // manufacturer1851, industry4, legacy button field6. Never scan offsets.
  return {1,
          0x63,
          0xff,
          0,
          0xff,
          3,
          1,
          0x3b,
          7,
          3,
          4,
          6,
          static_cast<std::uint8_t>(key)};
}
void St4000Pilot::Invalidate(const std::string &why) {
  const auto next = feedback_.connection_epoch + 1;
  feedback_ = {};
  feedback_.connection_epoch = next;
  if (address_)
    feedback_.source = Source();
  reason_ = why;
}
void St4000Pilot::Configure(const St4000Binding &binding) {
  ValidateSt4000Binding(binding);
  if (binding.interface == binding_.interface &&
      binding.name == binding_.name) {
    binding_.permit_control = binding.permit_control;
    return; // A permission toggle must not clear an identity conflict.
  }
  binding_ = binding;
  wanted_name_ = binding.name.empty() ? 0 : ParsePilotName(binding.name);
  address_.reset();
  claims_ = {};
  conflict_ = false;
  connected_ = false;
  last_send_.reset();
  last_identity_request_.reset();
  Invalidate(wanted_name_ ? "Waiting for translator address claim"
                          : "Translator not configured; control OFF");
}
void St4000Pilot::Poll(vessel::Time) {
  if (!wanted_name_)
    return;
  const auto status = transport_.Status(binding_.interface);
  if (connected_ != status.connected || transport_epoch_ != status.epoch) {
    connected_ = status.connected;
    transport_epoch_ = status.epoch;
    address_.reset();
    claims_ = {};
    Invalidate("Connection changed; new translator identity required");
  }
}
std::string St4000Pilot::Source() const {
  return "ST4000 / NMEA2000 / " + binding_.interface + "/NAME-" +
         binding_.name + "/source-" +
         (address_ ? std::to_string(*address_) : "unavailable");
}
void St4000Pilot::Observe(const PilotN2kFrame &f, vessel::Time now) {
  Poll(now);
  if (!wanted_name_ || !connected_ || conflict_ ||
      f.interface != binding_.interface || f.source >= 254 ||
      f.observed_at < vessel::Time{} || f.observed_at > now ||
      now - f.observed_at >= std::chrono::seconds(3))
    return;
  if (f.pgn == 60928) {
    if (f.data.size() != 8 ||
        (claims_[f.source] && f.observed_at <= *claims_[f.source]))
      return;
    claims_[f.source] = f.observed_at;
    std::uint64_t name = 0;
    for (unsigned i = 0; i < 8; ++i)
      name |= std::uint64_t(f.data[i]) << (i * 8);
    if ((name == wanted_name_ && address_ && *address_ != f.source) ||
        (name != wanted_name_ && address_ && *address_ == f.source)) {
      conflict_ = true;
      address_.reset();
      Invalidate("Address/NAME conflict; remove and verify translator binding "
                 "before reconfiguring");
    } else if (name == wanted_name_ && !address_) {
      address_ = f.source;
      Invalidate("Identity observed; waiting for physical pilot feedback");
    }
    return;
  }
  if (!address_ || f.source != *address_ || !claims_[f.source] ||
      f.observed_at <= *claims_[f.source])
    return;
  const auto source = Source();
  if (f.pgn == 65379) {
    if (feedback_.sequence && f.observed_at <= feedback_.observed_at)
      return;
    PilotMode mode = PilotMode::Unavailable;
    if (f.data.size() == 8 && VendorHeader(f.data)) {
      switch (U16(f.data, 2)) {
      case 0:
        mode = PilotMode::Standby;
        break;
      case 0x40:
        mode = PilotMode::Auto;
        break;
      case 0x100:
        mode = PilotMode::Wind;
        break;
      case 0x180:
        mode = PilotMode::Track;
        break;
      }
    }
    feedback_.mode = mode;
    feedback_.source = source;
    feedback_.observed_at = f.observed_at;
    ++feedback_.sequence;
    if (mode == PilotMode::Standby || mode == PilotMode::Unavailable)
      feedback_.locked_heading_magnetic_deg = {};
    reason_ = mode == PilotMode::Unavailable ? "Pilot mode invalid/unavailable"
                                             : "Physical pilot status observed";
  } else if (f.pgn == 65360) {
    auto &s = feedback_.locked_heading_magnetic_deg;
    if (f.observed_at <= s.observed_at)
      return;
    s = Angle(f.data.size() == 8 && VendorHeader(f.data) ? U16(f.data, 5)
                                                         : 65535,
              source + "/PGN-65360", f.observed_at);
  } else if (f.pgn == 127250) {
    auto &s = feedback_.heading_magnetic_deg;
    if (f.observed_at <= s.observed_at)
      return;
    s = Angle(f.data.size() == 8 && (f.data[7] & 3) == 1 ? U16(f.data, 1)
                                                         : 65535,
              source + "/PGN-127250 magnetic", f.observed_at);
  }
}
PilotCapabilities St4000Pilot::Capabilities() const {
  const auto s = transport_.Status(binding_.interface);
  const bool verified = wanted_name_ && address_ && !conflict_ && connected_ &&
                        s.connected && s.epoch == transport_epoch_;
  return {false,
          true,
          true,
          false,
          false,
          true,
          verified && s.writable && binding_.permit_control};
}
bool St4000Pilot::Send(const PilotRequest &r) {
  Poll(r.issued_at);
  if (!Capabilities().manual_control || !address_)
    return false;
  // Bound even duplicate STANDBY touch events. A new STANDBY can preempt a
  // different in-flight action, but never more than one packet per 250 ms.
  if (last_send_ &&
      (r.issued_at < *last_send_ ||
       r.issued_at - *last_send_ < std::chrono::milliseconds(250)))
    return false;
  const auto data = EncodeSt4000Command(r);
  if (data.empty())
    return false;
  last_send_ = r.issued_at;
  return transport_.Send(binding_.interface, *address_, 126208, 3, data);
}
bool St4000Pilot::RequestIdentity(vessel::Time now) {
  Poll(now);
  const auto s = transport_.Status(binding_.interface);
  if (!wanted_name_ || !s.connected || !s.writable || conflict_ ||
      (last_identity_request_ &&
       (now < *last_identity_request_ ||
        now - *last_identity_request_ < std::chrono::seconds(5))))
    return false;
  last_identity_request_ = now;
  return transport_.Send(binding_.interface, 255, 59904, 6, {0, 0xee, 0});
}
std::string St4000Pilot::Status() const {
  const auto s = transport_.Status(binding_.interface);
  return reason_ + " / " + s.detail +
         (binding_.permit_control ? " / configured control permission"
                                  : " / display-only permission");
}
} // namespace opennav::adapters
