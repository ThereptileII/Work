#include "integration/MarineDecoder.h"
#include "nmea0183.h"
#include <cctype>
#include <cmath>
#include <limits>

namespace opennav::integration {
using vessel::Quantity;
const std::vector<std::string> &InstrumentSentences() {
  static const std::vector<std::string> types = {"HDT", "MWV", "VHW", "DPT",
                                                 "DBT", "MTW", "RSA"};
  return types;
}
std::vector<vessel::SensorObservation>
Decode0183Instruments(const std::string &text, const std::string &iface,
                      vessel::Time at) {
  std::vector<vessel::SensorObservation> out;
  if (iface.empty() || iface.size() > 200 || text.size() < 10 ||
      text.size() > 256 || text[0] != '$' || text[6] != ',')
    return out;
  const auto star = text.find('*');
  if (star == std::string::npos || star + 3 > text.size())
    return out;
  if (text.substr(star + 3) != "" && text.substr(star + 3) != "\r\n" &&
      text.substr(star + 3) != "\n")
    return out;
  // NMEA 0183 is printable ASCII. Reject embedded controls/NUL and malformed
  // UTF-8 before the wx/upstream text parser can truncate or replace bytes.
  for (std::size_t i = 0; i < star; ++i)
    if (static_cast<unsigned char>(text[i]) < 0x20 ||
        static_cast<unsigned char>(text[i]) > 0x7e)
      return out;
  for (std::size_t i = star + 1; i < star + 3; ++i)
    if (!std::isxdigit(static_cast<unsigned char>(text[i])))
      return out;
  SENTENCE sentence;
  sentence = wxString::FromUTF8(text);
  if (sentence.IsChecksumBad(sentence.GetNumberOfDataFields() + 1) != NFalse)
    return out;
  const auto type = text.substr(3, 3),
             device = "NMEA0183/" + iface + "/talker-" + text.substr(1, 2);
  const auto source = device + "/" + type;
  const double unavailable = std::numeric_limits<double>::quiet_NaN();
  auto number = [&](int field) {
    double d;
    return sentence.Field(field).ToCDouble(&d) && std::isfinite(d)
               ? d
               : unavailable;
  };
  auto field = [&](int n) { return sentence.Field(n).ToStdString(wxConvUTF8); };
  auto put = [&](Quantity q, double value,
                 vessel::Validity validity = vessel::Validity::Measured,
                 const std::string &suffix = "") {
    vessel::Sample s{{}, source + suffix, at, vessel::Validity::Invalid};
    s.device_id = device;
    if (std::isfinite(value)) {
      s.value = value;
      s.validity = validity;
    }
    out.push_back({q, source + suffix, std::move(s), 20});
  };
  const auto count = sentence.GetNumberOfDataFields();
  // Field semantics mirror the pinned OpenCPN core/Dashboard sentence parsers.
  // Use its SENTENCE checksum/field parser, with stricter missing-number
  // checks.
  if (type == "HDT" && count == 2)
    put(Quantity::Heading, field(2) == "T" ? number(1) : unavailable);
  else if (type == "DPT" && (count == 2 || count == 3))
    put(Quantity::Depth, number(1));
  else if (type == "DBT" && count == 6)
    put(Quantity::Depth, field(4) == "M" ? number(3) : unavailable);
  else if (type == "MTW" && count == 2)
    put(Quantity::WaterTemperature, field(2) == "C" ? number(1) : unavailable);
  else if (type == "VHW" && count == 8) {
    put(Quantity::Heading, field(2) == "T" ? number(1) : unavailable);
    put(Quantity::WaterSpeed, field(6) == "N" ? number(5) : unavailable);
  } else if (type == "RSA" && count == 4) {
    put(Quantity::Rudder, field(2) == "A" ? number(1) : unavailable,
        vessel::Validity::Measured, "/starboard-or-single");
    // Port sensor is a distinct selectable source, never averaged with
    // starboard.
    if (!field(3).empty() || field(4) == "A")
      put(Quantity::Rudder, field(4) == "A" ? number(3) : unavailable,
          vessel::Validity::Measured, "/port");
  } else if (type == "MWV" && count == 5 &&
             (field(2) == "R" || field(2) == "T")) {
    double speed = number(3), angle = number(1);
    if (field(4) == "M")
      speed *= 3600.0 / 1852.0;
    else if (field(4) == "K")
      speed /= 1.852;
    else if (field(4) != "N")
      speed = unavailable;
    if (angle < 0 || angle > 360)
      angle = unavailable;
    else if (angle > 180)
      angle -= 360;
    if (field(5) != "A")
      speed = angle = unavailable;
    const bool apparent = field(2) == "R";
    const auto validity =
        apparent ? vessel::Validity::Measured : vessel::Validity::Estimated;
    put(apparent ? Quantity::ApparentWindSpeed : Quantity::TrueWindSpeed, speed,
        validity, apparent ? "/relative" : "/true");
    put(apparent ? Quantity::ApparentWindAngle : Quantity::TrueWindAngle, angle,
        validity, apparent ? "/relative" : "/true");
  }
  return out;
}
} // namespace opennav::integration
