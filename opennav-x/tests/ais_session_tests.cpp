#include "ais/AisStreamSession.h"
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
int checks = 0;
#define CHECK(v)                                                               \
  do {                                                                         \
    ++checks;                                                                  \
    if (!(v))                                                                  \
      throw std::runtime_error("AIS session check at line " +                  \
                               std::to_string(__LINE__));                      \
  } while (false)
const auto start = vessel::Time{} + 100h;
const auto wall = std::chrono::system_clock::from_time_t(1790596800);
const std::string confirmation =
    R"({"MessageType":"SubscriptionConfirmation","Message":{"CompressionEnabled":true}})";
const std::string position =
    R"({"MessageType":"PositionReport","MetaData":{"MMSI":123456789},"Message":{"PositionReport":{"Valid":true,"UserID":123456789,"Latitude":59.1,"Longitude":18.2,"Sog":4.5,"Cog":120.2,"TrueHeading":119,"NavigationalStatus":0}}})";
void connect(ais::AisStreamSession &session, vessel::Time at) {
  CHECK(session.NeedsConnection(at));
  CHECK(session.Connecting(at));
  session.Opened(at);
  CHECK(!session.PendingSubscription(at).empty());
  CHECK(session.SubscriptionSent(at));
  CHECK(session.PendingSubscription(at).empty());
  session.Receive(confirmation, at, wall, 0);
  CHECK(session.Read(at).health.subscription_confirmed);
  CHECK(session.Read(at).health.compression_enabled);
  CHECK(session.Read(at).health.connection == ais::Connection::Connected);
}
void observation_checks() {
  const ais::ConnectionObservation observed{
      ais::AddressFamily::IPv4, {{{127, 0, 0, 1}}, 45000, 0},
      {{{127, 0, 0, 1}}, 443, 0}, 7, start};
  ais::AisStreamSession session;
  CHECK(!session.Opened(start, observed));
  CHECK(session.Read(start).connection.family == ais::AddressFamily::Unavailable);
  CHECK(session.Enable(true, start));
  CHECK(session.ObserveViewport({59, 60, 18, 19}));
  CHECK(!session.Opened(start, observed)); // no accepted Connecting yet
  CHECK(session.Connecting(start));
  CHECK(session.Opened(start, observed));
  auto retained = session.Read(start);
  CHECK(retained.connection.generation == 7);
  CHECK(retained.connection.local.address == observed.local.address);
  CHECK(!session.Enable(true, start + 1s)); // recurring application tick is a no-op
  CHECK(!session.Opened(start + 1s, {}));   // duplicate Open cannot overwrite
  CHECK(session.Read(start + 1h).connection.captured_at == start);
  CHECK(session.SubscriptionSent(start + 1s));
  session.Receive(confirmation, start + 1s, wall, 0);
  CHECK(session.ObserveViewport({40, 41, 10, 11}));
  CHECK(session.SubscriptionSent(start + 6s));
  CHECK(session.Read(start + 6s).connection.generation == 7);
  session.Disconnected(start + 7s, 0);
  CHECK(session.Read(start + 7s).connection.family == ais::AddressFamily::Unavailable);
  CHECK(retained.connection.generation == 7 && retained.connection.captured_at == start);
  CHECK(!session.Opened(start + 7s, observed)); // late Open in backoff
  CHECK(session.Enable(false, start + 8s));
  CHECK(session.Enable(true, start + 8s));
  CHECK(!session.Opened(start + 8s, observed)); // old callback after quick enable
  CHECK(session.Connecting(start + 8s));
  auto future = observed;
  future.captured_at = start + 9s;
  CHECK(session.Opened(start + 8s, future)); // unavailable must not fail connection
  CHECK(session.Read(start + 8s).connection.family == ais::AddressFamily::Unavailable);
  CHECK(session.SubscriptionSent(start + 8s));

  // Every route out of a usable transport clears immediately, without a later Read
  // mutating state. These are the real session entry points used by the worker.
  for (int ending = 0; ending != 7; ++ending) {
    ais::AisStreamSession current;
    current.Enable(true, start);
    current.ObserveViewport({59, 60, 18, 19});
    CHECK(current.Connecting(start));
    CHECK(current.Opened(start, observed));
    switch (ending) {
    case 0: current.Enable(false, start + 1s); break;
    case 1: current.RetryCredentials(start + 1s); break;
    case 2: current.CredentialMissing(start + 1s); break;
    case 3: current.Disconnected(start + 1s, 0); break; // close/error/send failure
    case 4: current.Tick(start + 2500ms, 0); break;    // initial subscription timeout
    case 5:
      CHECK(current.SubscriptionSent(start));
      current.Tick(start + 10s, 0); break;            // acknowledgement timeout
    case 6: current.Receive(R"({"error":"service failure"})", start + 1s, wall, 0); break;
    }
    CHECK(current.ShouldClose());
    CHECK(current.Read(start + 20s).connection.family == ais::AddressFamily::Unavailable);
    CHECK(!current.Opened(start + 20s, observed));
  }
}
void diagnostic_checks() {
  ais::AisStreamSession session;
  auto read=session.Read(start);
  CHECK(read.health.received_messages==0 && read.health.ignored_messages==0);
  CHECK(!read.health.subscription_pending && !read.health.subscription_awaiting_confirmation);
  CHECK(read.cached_position_count==0);
  CHECK(session.Enable(true,start));
  CHECK(!session.Read(start).health.subscription_pending); // no area is not global
  CHECK(session.ObserveViewport({59,60,18,19}));
  CHECK(session.Read(start).health.subscription_pending);
  CHECK(session.Connecting(start));
  CHECK(session.Opened(start));
  CHECK(session.Read(start).health.subscription_pending);
  CHECK(!session.Read(start).health.subscription_awaiting_confirmation);
  CHECK(session.SubscriptionSent(start));
  read=session.Read(start);
  CHECK(!read.health.subscription_pending && read.health.subscription_awaiting_confirmation);
  CHECK(!read.health.subscription_confirmed);
  const std::string unsupported=R"({"MessageType":"SafetyRelatedBroadcastMessage","Message":{}})";
  session.Receive(unsupported,start+1s,wall,0);
  read=session.Read(start+1s);
  CHECK(read.health.received_messages==1 && read.health.ignored_messages==1);
  CHECK(read.health.accepted==0 && read.health.rejected==0); // zero reports is not zero messages
  session.Receive(confirmation,start+2s,wall,0);
  read=session.Read(start+2s);
  CHECK(read.health.received_messages==2 && read.health.ignored_messages==1);
  CHECK(read.health.subscription_confirmed && !read.health.subscription_pending &&
        !read.health.subscription_awaiting_confirmation);
  session.Receive(position,start+3s,wall,0);
  session.Receive("{invalid",start+4s,wall,0);
  read=session.Read(start+4s);
  CHECK(read.health.received_messages==4 && read.health.ignored_messages==1);
  CHECK(read.health.accepted==1 && read.health.rejected==1 && read.cached_position_count==1);
  CHECK(session.ObserveViewport({40,41,10,11}));
  read=session.Read(start+4s);
  CHECK(read.health.subscription_pending && read.health.subscription_confirmed &&
        !read.health.subscription_awaiting_confirmation); // old area confirmed; next waits for cadence
  CHECK(session.PendingSubscription(start+4s).empty());
  CHECK(session.SubscriptionSent(start+5s));
  read=session.Read(start+5s);
  CHECK(!read.health.subscription_pending && !read.health.subscription_confirmed &&
        read.health.subscription_awaiting_confirmation);
  CHECK(session.ObserveViewport({30,31,10,11}));
  read=session.Read(start+5s);
  CHECK(read.health.subscription_pending && read.health.subscription_awaiting_confirmation);
  session.Receive(confirmation,start+6s,wall,0);
  read=session.Read(start+6s);
  CHECK(read.health.subscription_confirmed && read.health.subscription_pending &&
        !read.health.subscription_awaiting_confirmation);
  const auto before=session.Read(start+6s);
  const auto expired=session.Read(start+11min);
  CHECK(expired.cached_position_count==0 && expired.health.accepted==1);
  CHECK(expired.health.received_messages==before.health.received_messages);
  CHECK(session.Read(start+6s).cached_position_count==1); // diagnostic reads never advance session
  session.Disconnected(start+7s,0);
  read=session.Read(start+7s);
  CHECK(read.health.subscription_pending && !read.health.subscription_confirmed &&
        !read.health.subscription_awaiting_confirmation);
  CHECK(session.Enable(false,start+8s));
  read=session.Read(start+8s);
  CHECK(!read.health.subscription_pending && !read.health.subscription_awaiting_confirmation);
  CHECK(read.cached_position_count==0 && read.health.received_messages==5);
  session.Receive(unsupported,start+9s,wall,0);
  CHECK(session.Read(start+9s).health.received_messages==5); // disabled callback does not count
  CHECK(session.Enable(true,start+10s));
  CHECK(session.Read(start+10s).health.subscription_pending); // reconnect even when last area sent
}
int main() {
  try {
    observation_checks();
    diagnostic_checks();
    ais::AisStreamSession session;
    CHECK(!session.NeedsConnection(start));
    CHECK(session.ShouldClose());
    CHECK(!session.Read(start).targets.available);
    session.Enable(true, start);
    CHECK(
        !session.NeedsConnection(start)); // no viewport: no global subscription
    CHECK(!session.ObserveViewport({91, 92, 18, 19}));
    CHECK(session.ObserveViewport({59, 60, 18, 19}));
    CHECK(session.NeedsConnection(start));
    session.CredentialMissing(start);
    CHECK(!session.NeedsConnection(start + 1h));
    CHECK(session.Read(start).health.connection ==
          ais::Connection::CredentialMissing);
    session.RetryCredentials(start + 1s);
    connect(session, start + 1s);
    session.Receive(position, start + 2s, wall, 0);
    auto retained = session.Read(start + 2s);
    CHECK(retained.targets.targets.size() == 1);
    CHECK(retained.health.accepted == 1);
    CHECK(retained.targets.targets[0].observed_at == start + 2s);
    session.Receive(position, start + 1500ms, wall,
                    0); // old callback timestamp
    CHECK(session.Read(start + 3s).health.accepted == 1);
    session.Receive("{broken", start + 3s, wall, 0);
    CHECK(session.Read(start + 3s).health.rejected == 1);
    CHECK(session.ObserveViewport({59.1, 59.9, 18.1, 18.9}));
    CHECK(session.PendingSubscription(start + 6s).empty()); // within margin
    CHECK(session.ObserveViewport({40, 41, 10, 11}));
    CHECK(session.PendingSubscription(start + 5s)
              .empty()); // 5s cadence since send
    CHECK(!session.PendingSubscription(start + 6s).empty());
    CHECK(session.SubscriptionSent(start + 6s));
    CHECK(session.PendingSubscription(start + 100s).empty()); // one in flight
    session.Receive(confirmation, start + 7s, wall, 0);
    CHECK(session.Read(start + 7s).health.connection ==
          ais::Connection::Connected);
    session.Disconnected(start + 8s, 0);
    CHECK(session.ShouldClose());
    CHECK(!session.NeedsConnection(start + 9s));
    CHECK(session.NeedsConnection(start + 10s));
    CHECK(session.Read(start + 8s).targets.targets.size() == 1);
    CHECK(session.Read(start + 8s).targets.targets[0].observed_at ==
          start + 2s);
    CHECK(!session.Read(start + 8s).health.subscription_confirmed);
    connect(session,
            start +
                10s); // complete subscription even though viewport unchanged
    CHECK(session.Read(start + 11s).health.reconnects == 1);
    CHECK(
        session.Read(start + 62s).targets.targets[0].latitude_deg.observed_at ==
        start + 2s);
    CHECK(ais::Age(start + 2s, start + 62s) == ais::TargetAge::Stale);
    CHECK(session.Read(start + 123s).targets.targets[0].lost);
    CHECK(session.Read(start + 602s).targets.targets.empty());
    session.Receive(
        R"({"error":"deliberate-server-text-must-not-be-retained"})",
        start + 13s, wall, 0);
    CHECK(session.Read(start + 13s).health.connection ==
          ais::Connection::Backoff);
    CHECK(session.Read(start + 13s).targets.targets.size() == 1);
    session.Enable(false, start + 14s);
    CHECK(session.Read(start + 14s).targets.targets.empty());
    CHECK(session.Read(start + 14s).health.connection ==
          ais::Connection::Disabled);
    CHECK(retained.targets.targets.size() ==
          1); // lifetime independent of disabled cache
    session.Receive(position, start + 15s, wall, 0);
    CHECK(session.Read(start + 15s).targets.targets.empty());
    session.Enable(true, start + 16s);
    CHECK(session.Connecting(start + 16s));
    session.Opened(start + 16s);
    session.Receive(confirmation, start + 16s, wall,
                    0); // unsolicited; no sent subscription
    CHECK(!session.Read(start + 16s).health.subscription_confirmed);
    session.Receive(position, start + 17s, wall, 0);
    CHECK(session.Read(start + 17s).targets.targets.empty());
    session.Tick(start + 18500ms,
                 0); // initial subscription was not sent promptly
    CHECK(session.Read(start + 18500ms).health.connection ==
          ais::Connection::Backoff);
    CHECK(session.Connecting(start + 21s));
    session.Tick(start + 33s, 0); // stuck connection timeout
    CHECK(session.Read(start + 33s).health.connection ==
          ais::Connection::Backoff);
    CHECK(session.Connecting(start + 40s));
    session.Opened(start + 40s);
    CHECK(session.SubscriptionSent(start + 40s));
    session.Tick(start + 50s, 0); // absent subscription acknowledgement
    CHECK(session.Read(start + 50s).health.connection ==
          ais::Connection::Backoff);
    auto time = start + 1h;
    for (unsigned i = 0; i < 12; ++i) {
      CHECK(session.Connecting(time));
      session.Disconnected(time, 0);
      const auto retry = session.Read(time).health.retry_at;
      CHECK(retry > time);
      CHECK(retry <= time + 15min);
      CHECK(!session.NeedsConnection(retry - 1ms));
      time = retry;
    }
    CHECK(session.Read(time).health.retry_at == time);
    CHECK(session.Connecting(time));
    session.Opened(time);
    CHECK(session.SubscriptionSent(time));
    session.Receive(confirmation, time, wall, 0);
    session.Disconnected(time, 0);
    CHECK(session.Read(time).health.retry_at ==
          time + 2s); // confirmed connection resets backoff
    std::cout << checks << " session checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
