#pragma once
#include "application/SourceHealthView.h"
#include "smartnav/Advisories.h"
#include <array>
#include <tuple>

namespace opennav::application {
enum class HorizonActionKind { None, Follow, Passage, Ais };
struct HorizonAction {
  HorizonActionKind kind = HorizonActionKind::None;
  std::string route_id, revision_scope, identity, source;
  std::uint64_t route_revision = 0;
  int mmsi = 0;
  bool operator==(const HorizonAction &b) const {
    return std::tie(kind,route_id,revision_scope,identity,source,route_revision,mmsi)==
           std::tie(b.kind,b.route_id,b.revision_scope,b.identity,b.source,b.route_revision,b.mmsi);
  }
};
enum class HorizonMarker { Now, Event, Traffic, Arrival, Unavailable };
struct HorizonItem {
  std::string time, secondary_time, title, detail, detail_accent;
  std::string event_identity, event_source;
  HorizonMarker marker = HorizonMarker::Unavailable;
  smartnav::Severity severity = smartnav::Severity::Information;
  HorizonAction action;
  bool operator==(const HorizonItem &b) const {
    return std::tie(time,secondary_time,title,detail,detail_accent,event_identity,event_source,marker,severity)==
           std::tie(b.time,b.secondary_time,b.title,b.detail,b.detail_accent,b.event_identity,b.event_source,b.marker,b.severity) && action==b.action;
  }
};
struct HorizonView {
  std::array<HorizonItem,4> items;
  std::string advisory_label = "SmartNav · advisory";
  bool historical = false;
  bool operator==(const HorizonView &b) const {
    return items==b.items && advisory_label==b.advisory_label && historical==b.historical;
  }
};
// Formats bounded copied observations and the existing ordered SmartNav stream.
// No navigation calculations, retained upstream handles, or wall-clock ETAs.
HorizonView PresentHorizon(const vessel::VesselState &, const smartnav::NavigationAdvice &,
                          const vessel::AisState &onboard, vessel::Time now);
// Re-evaluate against current copied inputs at activation, independently of the
// presentation refresh. A retained control cannot select an old/replaced target.
bool HorizonActionAllowed(const HorizonAction &,const vessel::VesselState &,
                          const vessel::AisState &onboard,vessel::Time now);
} // namespace opennav::application
