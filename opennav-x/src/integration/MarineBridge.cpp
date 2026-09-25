#include "integration/MarineBridge.h"
#include "model/comm_drv_n2k_net.h"
#include "model/comm_drv_registry.h"
#include "model/comm_navmsg.h"
#include <stdexcept>
#include <wx/thread.h>

namespace opennav::integration {
namespace {
std::optional<vessel::Time>
Receipt(const NavMsg &msg, vessel::Time now,
        std::chrono::system_clock::time_point wall) {
  const auto age = wall - msg.created_at;
  if (age < std::chrono::system_clock::duration::zero() ||
      age > std::chrono::hours(24))
    return {};
  return now - std::chrono::duration_cast<vessel::Clock::duration>(age);
}
} // namespace
MarineBridge::MarineBridge() {
  if (!wxIsMainThread())
    throw std::logic_error(
        "Marine subscriptions require the application thread");
  auto changes = std::make_unique<ObsListener>();
  changes->Init(CommDriverRegistry::GetInstance().evt_driverlist_change,
                [this](ObservedEvt &) {
                  identities_.Clear();
                  network_generations_.clear();
                  sources_.Clear();
                  last_received_.reset();
                });
  listeners_.push_back(std::move(changes));
  auto claims = std::make_unique<ObsListener>();
  claims->Init(Nmea2000Msg(60928), [this](ObservedEvt &event) {
    const auto m = UnpackEvtPointer<Nmea2000Msg>(event);
    if (!m || !m->source || m->payload.size() != 22 || m->payload[0] != 0x93 ||
        m->payload[3] != 0 || m->payload[4] != 0xee || m->payload[5] != 0 ||
        m->payload[12] != 8)
      return;
    const auto now = vessel::Clock::now();
    const auto at = Receipt(*m, now, std::chrono::system_clock::now());
    if (!at || !CheckConnection(m->source->iface, *at))
      return;
    const auto result =
        identities_.Observe(m->source->iface, m->payload[7],
                            std::vector<unsigned char>(m->payload.begin() + 13,
                                                       m->payload.begin() + 21),
                            *at, now);
    if (result == ClaimResult::Changed || result == ClaimResult::Conflict) {
      // Conservative invalidation: no old-address samples survive reassignment.
      // Normal incoming messages repopulate the bounded reducer.
      sources_.Clear();
      last_received_.reset();
    }
  });
  listeners_.push_back(std::move(claims));
  for (const auto pgn : InstrumentPgns()) {
    auto listener = std::make_unique<ObsListener>();
    listener->Init(Nmea2000Msg(pgn), [this](ObservedEvt &event) {
      const auto m = UnpackEvtPointer<Nmea2000Msg>(event);
      if (!m || !m->source || m->source->iface.empty() || m->payload.size() < 8)
        return;
      const auto now = vessel::Clock::now();
      const auto wall = std::chrono::system_clock::now();
      const auto at = Receipt(*m, now, wall);
      if (!at || !CheckConnection(m->source->iface, *at))
        return;
      Accept(DecodeN2kInstruments(
                 m->PGN.pgn, m->payload,
                 identities_.Label(m->source->iface, m->payload[7]), *at),
             now);
    });
    listeners_.push_back(std::move(listener));
  }
  for (const auto &type : InstrumentSentences()) {
    auto listener = std::make_unique<ObsListener>();
    listener->Init(Nmea0183Msg(type), [this](ObservedEvt &event) {
      const auto m = UnpackEvtPointer<Nmea0183Msg>(event);
      if (!m || !m->source || m->source->iface.empty())
        return;
      const auto now = vessel::Clock::now();
      const auto wall = std::chrono::system_clock::now();
      const auto at = Receipt(*m, now, wall);
      if (!at)
        return;
      Accept(Decode0183Instruments(m->payload, m->source->iface, *at), now);
    });
    listeners_.push_back(std::move(listener));
  }
  auto listener = std::make_unique<ObsListener>();
  listener->Init(SignalkMsg(), [this](ObservedEvt &event) {
    const auto m = UnpackEvtPointer<SignalkMsg>(event);
    if (!m || !m->source || m->context_self != m->context)
      return;
    const auto now = vessel::Clock::now();
    const auto wall = std::chrono::system_clock::now();
    if (!Receipt(*m, now, wall))
      return;
    Accept(DecodeSignalKInstruments(m->raw_message, m->context_self,
                                    m->source->iface, now, wall, bindings_),
           now);
  });
  listeners_.push_back(std::move(listener));
}
bool MarineBridge::CheckConnection(const std::string &iface, vessel::Time at) {
  for (const auto &driver : CommDriverRegistry::GetInstance().GetDrivers()) {
    if (driver->bus != NavAddr::Bus::N2000 || driver->iface != iface)
      continue;
    const auto *network = dynamic_cast<const CommDriverN2KNet *>(driver.get());
    if (!network || network->GetParams().NetProtocol != TCP)
      return true;
    const auto generation = network->GetConnectionGeneration();
    auto old = network_generations_.find(iface);
    if (old == network_generations_.end() || old->second != generation) {
      identities_.Clear();
      sources_.Clear();
      last_received_.reset();
      network_generations_[iface] = generation;
    }
    return network->GetSock() && network->GetSock()->IsConnected() &&
           at >= network->GetConnectionChangedAt();
  }
  return true; // Other producers retain explicit address-only provenance.
}
void MarineBridge::Accept(std::vector<vessel::SensorObservation> observations,
                          vessel::Time now) {
  if (!wxIsMainThread())
    throw std::logic_error(
        "Marine observation requires the application thread");
  for (auto &observation : observations) {
    const auto time = observation.sample.observed_at;
    const auto admission = sources_.Observe(std::move(observation), now);
    if ((admission == vessel::Admission::Accepted ||
         admission == vessel::Admission::InvalidValue) &&
        (!last_received_ || time > *last_received_))
      last_received_ = time;
  }
}
vessel::VesselState MarineBridge::Merge(vessel::VesselState state,
                                        vessel::Time now) const {
  auto result = sources_.Merge(std::move(state), now);
  if (last_received_)
    result.connectivity.status = {"Observed marine instrument input",
                                  "OpenCPN input bus", *last_received_,
                                  vessel::Validity::Measured};
  return result;
}
} // namespace opennav::integration
