#include "ais/Subscription.h"
#include <algorithm>
#include <cmath>

namespace opennav::ais {
namespace {
bool Finite(double n) { return std::isfinite(n); }
bool Same(const std::vector<BoundingBox> &a, const std::vector<BoundingBox> &b) {
  if(a.size()!=b.size())return false;
  for(std::size_t i=0;i<a.size();++i)
    if(a[i].south!=b[i].south||a[i].west!=b[i].west||a[i].north!=b[i].north||a[i].east!=b[i].east)return false;
  return true;
}
bool Contains(const std::vector<BoundingBox> &area, const std::vector<BoundingBox> &view) {
  if(view.empty()||area.empty())return false;
  for(const auto &v:view) {
    bool contained=false;
    for(const auto &a:area)contained|=v.south>=a.south&&v.north<=a.north&&v.west>=a.west&&v.east<=a.east;
    if(!contained)return false;
  }
  return true;
}
std::vector<BoundingBox> Boxes(Viewport v,double margin) {
  if(!Finite(v.south)||!Finite(v.north)||!Finite(v.west)||!Finite(v.east)||
     v.south< -90||v.north>90||v.south>=v.north||v.west< -180||v.west>180||v.east< -180||v.east>180||v.west==v.east)return {};
  const double longitude_span=v.east>=v.west?v.east-v.west:360-v.west+v.east;
  const double latitude_margin=(v.north-v.south)*margin;
  const double south=std::max(-90.0,v.south-latitude_margin),north=std::min(90.0,v.north+latitude_margin);
  if(longitude_span*(1+2*margin)>=360)return {{south,-180,north,180}};
  double west=v.west-longitude_span*margin,east=v.east+longitude_span*margin;
  if(west< -180)west+=360;
  if(east>180)east-=360;
  if(west>east)return {{south,west,north,180},{south,-180,north,east}};
  return {{south,west,north,east}};
}
} // namespace
std::vector<BoundingBox> SubscriptionArea(Viewport v) { return Boxes(v,v.exact_area ? 0 : .15); }
bool SubscriptionPolicy::ObserveViewport(Viewport viewport) {
  auto visible=Boxes(viewport,0);
  if(visible.empty())return false;
  if(!viewport.exact_area && Contains(sent_,visible)) {
    desired_=sent_; // coalesced pan came back inside the already subscribed margin
    return true;
  }
  desired_=SubscriptionArea(viewport);
  return true;
}
std::vector<BoundingBox> SubscriptionPolicy::Pending(vessel::Time now,bool connection) const {
  if(desired_.empty())return {};
  if(connection)return desired_;
  if(awaiting_confirmation_)return {}; // one replacement in flight, no ambiguous acknowledgement
  const auto cadence=std::chrono::seconds(5);
  if(!sent_.empty()&&(now<last_sent_||last_sent_>vessel::Time::max()-cadence||now<last_sent_+cadence))return {};
  return Same(desired_,sent_)?std::vector<BoundingBox>{}:desired_;
}
bool SubscriptionPolicy::HasPendingChange() const {
  return !desired_.empty() && !Same(desired_,sent_);
}
void SubscriptionPolicy::Sent(vessel::Time at) {
  sent_=desired_;last_sent_=at;confirmed_=false;awaiting_confirmation_=true;
}
vessel::Duration ReconnectDelay(unsigned failures,unsigned entropy) {
  if(failures>=10)return std::chrono::minutes(15);
  const auto milliseconds=std::min(300000u,2000u*(1u<<std::min(failures,8u)));
  return vessel::Duration(milliseconds+entropy%(milliseconds/4+1));
}
} // namespace opennav::ais
