#include "ais/AisStreamCodec.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
int checks = 0;
#define CHECK(value)                                                           \
  do {                                                                         \
    ++checks;                                                                  \
    if (!(value))                                                              \
      throw std::runtime_error("AIS codec check failed at line " +             \
                               std::to_string(__LINE__));                      \
  } while (false)
const auto now = vessel::Time{} + 100h;
const auto wall =
    std::chrono::system_clock::from_time_t(1790596800); // 2026-09-28 12:00 UTC
std::string Envelope(const std::string &type, const std::string &body,
                     const std::string &time = "") {
  return "{\"MessageType\":\"" + type + "\",\"MetaData\":{\"MMSI\":123456789" +
         time + "},\"Message\":{\"" + type + "\":{" + body + "}}}";
}
const std::string identity = "\"Valid\":true,\"UserID\":123456789,";
const std::string motion =
    "\"Latitude\":59.1,\"Longitude\":18.2,\"Sog\":4.5,\"Cog\":120.2,"
    "\"TrueHeading\":119,\"NavigationalStatus\":0";
auto Decode(const std::string &s) { return ais::DecodeAisStream(s, now, wall); }
int main() {
  try {
    for (const char *type : {"PositionReport", "StandardClassBPositionReport",
                             "ExtendedClassBPositionReport"}) {
      auto d = Decode(Envelope(type, identity + motion));
      CHECK(d.kind == ais::DecodeKind::Reports);
      CHECK(d.position);
      CHECK(d.position->mmsi == 123456789);
      CHECK(d.position->sog == 4.5);
      CHECK(d.position->cog == 120.2);
      CHECK(d.position->heading == 119);
      CHECK(d.position->observed_at == now);
      CHECK(d.position->receipt_time_only);
    }
    auto d =
        Decode(Envelope("PositionReport", identity + motion,
                        ",\"time_utc\":\"2026-09-28 11:59:58.500 +0000 UTC\""));
    CHECK(d.position && d.position->observed_at == now - 1500ms);
    CHECK(!d.position->receipt_time_only);
    ais::TargetCache cache;
    CHECK(cache.Observe(*d.position, now));
    auto copy = cache.Read(now);
    CHECK(copy.targets[0].time_basis == vessel::AisTimeBasis::OnlineService);
    auto receipt = Decode(Envelope("PositionReport", identity + motion));
    CHECK(cache.Observe(*receipt.position, now));
    CHECK(cache.Read(now).targets[0].time_basis ==
          vessel::AisTimeBasis::OnlineReceipt);
    CHECK(copy.targets[0].latitude_deg.observed_at == now - 1500ms);
    for (const char *timestamp :
         {"2026-09-28T12:00:00Z", "2026-09-28T11:59:59.123456789Z"})
      CHECK(Decode(Envelope("PositionReport", identity + motion,
                            ",\"time_utc\":\"" + std::string(timestamp) + "\""))
                .position);
    for (const char *timestamp :
         {"2026-09-28T12:00:01Z", "2026-09-28T11:49:59Z",
          "2026-02-30T12:00:00Z", "2026-09-28T12:00:00.1234567890Z",
          "2026-09-28T12:00:00+02:00", "2026-09-28T12:00:60Z",
          "2026-09-28T12:00:00.Z", "invalid"})
      CHECK(Decode(Envelope("PositionReport", identity + motion,
                            ",\"time_utc\":\"" + std::string(timestamp) + "\""))
                .kind == ais::DecodeKind::Invalid);
    d = Decode(Envelope(
        "ShipStaticData",
        identity + "\"Name\":\"TEST VESSEL@@ \u0040\",\"CallSign\":\" TEST "
                   "\",\"Destination\":\"PORT\",\"Type\":70,\"Dimension\":{"
                   "\"A\":12,\"B\":8,\"C\":3,\"D\":4}"));
    CHECK(d.static_data && !d.position);
    CHECK(d.static_data->name == "TEST VESSEL");
    CHECK(d.static_data->callsign == "TEST");
    CHECK(d.static_data->length_m == 20 && d.static_data->beam_m == 7);
    CHECK(cache.Observe(*d.static_data, now));
    CHECK(cache.Read(now + 16s).targets[0].latitude_deg.observed_at == now);
    CHECK(
        vessel::Assess(cache.Read(now + 16s).targets[0].latitude_deg, now + 16s)
            .quality == vessel::Quality::Aging);
    auto a = Decode(Envelope(
        "StaticDataReport",
        identity + "\"PartNumber\":false,\"ReportA\":{\"Valid\":true,\"Name\":"
                   "\"CLASS B\"},\"ReportB\":{\"Valid\":false}"));
    auto b = Decode(Envelope(
        "StaticDataReport",
        identity +
            "\"PartNumber\":true,\"ReportB\":{\"Valid\":true,\"CallSign\":"
            "\"CLASSB\",\"ShipType\":36},\"ReportA\":{\"Valid\":false}"));
    CHECK(a.static_data && b.static_data);
    CHECK(cache.Observe(*a.static_data, now));
    CHECK(cache.Observe(*b.static_data, now));
    CHECK(cache.Read(now).targets[0].name == "CLASS B");
    CHECK(cache.Read(now).targets[0].callsign.value == "CLASSB");
    CHECK(Decode(Envelope("StaticDataReport",
                          identity + "\"PartNumber\":false,\"ReportA\":{"
                                     "\"Valid\":false,\"Name\":\"REJECT\"}"))
              .kind == ais::DecodeKind::Invalid);
    d = Decode(Envelope(
        "PositionReport",
        identity + "\"Latitude\":59,\"Longitude\":18,\"Sog\":102.3,\"Cog\":360,"
                   "\"TrueHeading\":511,\"NavigationalStatus\":15"));
    CHECK(d.position && !d.position->sog && !d.position->cog &&
          !d.position->heading && !d.position->navigation_status);
    for (const std::string bad : std::vector<std::string>{
             "{", "null", "[]", "{\"a\":1,\"a\":2}",
             "{\"MessageType\":\"PositionReport\",\"MessageType\":"
             "\"SubscriptionConfirmation\",\"Message\":{\"CompressionEnabled\":"
             "true}}",
             std::string(65537, ' '),
             std::string(17, '[') + std::string(17, ']')})
      CHECK(Decode(bad).kind == ais::DecodeKind::Invalid);
    for (const char *bad :
         {"91", "-91", "NaN", "Infinity", "1e400", "\"59\"", "null"})
      CHECK(Decode(Envelope("PositionReport", identity + "\"Latitude\":" + bad +
                                                  ",\"Longitude\":18"))
                .kind == ais::DecodeKind::Invalid);
    for (const char *bad : {"181", "-181", "1e400"})
      CHECK(Decode(Envelope("PositionReport",
                            identity + "\"Latitude\":59,\"Longitude\":" + bad))
                .kind == ais::DecodeKind::Invalid);
    for (const char *bad : {"-1", "102.4", "\"4\""})
      CHECK(Decode(
                Envelope("PositionReport",
                         identity +
                             "\"Latitude\":59,\"Longitude\":18,\"Sog\":" + bad))
                .kind == ais::DecodeKind::Invalid);
    CHECK(Decode(Envelope("PositionReport",
                          "\"Valid\":false,\"UserID\":123456789," + motion))
              .kind == ais::DecodeKind::Invalid);
    CHECK(Decode(Envelope("PositionReport",
                          "\"Valid\":true,\"UserID\":223456789," + motion))
              .kind == ais::DecodeKind::Invalid);
    CHECK(Decode(Envelope("PositionReport", identity + "\"Sog\":4",
                          ",\"Latitude\":59,\"Longitude\":18"))
              .kind == ais::DecodeKind::Invalid);
    for (const std::string name :
         {std::string(129, 'A'), std::string("bad\\u0000name"),
          std::string("bad\\ud800name"), std::string("bad\xc0\xaf")})
      CHECK(Decode(Envelope("ShipStaticData",
                            identity + "\"Name\":\"" + name + "\""))
                .kind == ais::DecodeKind::Invalid);
    d = Decode("{\"MessageType\":\"SubscriptionConfirmation\",\"Message\":{"
               "\"CompressionEnabled\":true}}");
    CHECK(d.kind == ais::DecodeKind::Confirmation && d.compression);
    CHECK(Decode("{\"MessageType\":\"SubscriptionConfirmation\",\"Message\":{"
                 "\"CompressionEnabled\":\"true\"}}")
              .kind == ais::DecodeKind::Invalid);
    CHECK(Decode("{\"error\":\"redaction-test-key\"}").kind ==
          ais::DecodeKind::ServiceError);
    CHECK(
        Decode("{\"MessageType\":\"BaseStationReport\",\"Message\":{}}").kind ==
        ais::DecodeKind::Ignored);
    CHECK(!Decode(Envelope("ShipStaticData", identity + "\"Name\":\"STATIC\"",
                           ",\"Latitude\":59,\"Longitude\":18"))
               .position);
    auto boxes = ais::SubscriptionArea({58, 59, 17, 18});
    auto wire =
        ais::AisStreamSubscription("redaction-test-key", boxes, {123456789});
    CHECK(wire &&
          wire->find("\"APIKey\":\"redaction-test-key\"") != std::string::npos);
    CHECK(wire->find("\"BoundingBoxes\"") != std::string::npos);
    CHECK(wire->find("\"FiltersShipMMSI\":[\"123456789\"]") !=
          std::string::npos);
    CHECK(wire->find("StandardClassBPositionReport") != std::string::npos);
    CHECK(wire->find("StaticDataReport") != std::string::npos);
    CHECK(!ais::AisStreamSubscription("", boxes));
    CHECK(!ais::AisStreamSubscription("key", {}));
    CHECK(!ais::AisStreamSubscription("bad\nkey", boxes));
    CHECK(!ais::AisStreamSubscription("key", {{59, 18, 58, 17}}));
    CHECK(!ais::AisStreamSubscription(
        "key", {{58, 17, 59, std::numeric_limits<double>::infinity()}}));
    CHECK(!ais::AisStreamSubscription("key", boxes, {123}));
    CHECK(!ais::AisStreamSubscription("key", boxes, {123456789, 123456789}));
    CHECK(!ais::AisStreamSubscription("key", boxes,
                                      std::vector<int>(201, 123456789)));
    std::cout << checks << " AISStream codec checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
