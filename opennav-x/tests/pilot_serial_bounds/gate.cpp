// Actual OpenCPNPilot final sink/session methods with real ST4000 identity and
// command adapter. Only OpenCPN registry/driver/framework collaborators are fake.
#include "adapters/St4000Pilot.h"
#include "integration/PilotOutputPolicy.h"
#include "integration/PilotStatusDiscovery.h"
#include "model/comm_drv_n2k_serial_state.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
using namespace opennav;
struct NavAddr2000 {
  std::string iface; unsigned char address;
  NavAddr2000(std::string i,unsigned char a):iface(i),address(a){}
};
struct Nmea2000Msg {
  unsigned pgn; std::vector<unsigned char> data; int priority;
  Nmea2000Msg(unsigned p,const std::vector<unsigned char>& d,std::shared_ptr<const NavAddr2000>,int q):pgn(p),data(d),priority(q){}
};
struct CommDriverN2K {
  virtual ~CommDriverN2K()=default;
  virtual bool SendMessage(std::shared_ptr<Nmea2000Msg>,std::shared_ptr<NavAddr2000>){return false;}
};
struct CommDriverN2KSerial : CommDriverN2K {
  N2kSerialState state;
  unsigned attempts=0;
  std::vector<std::shared_ptr<Nmea2000Msg>> accepted;
  auto GetSerialState(){return state.Get();}
  void AddTxPGN(unsigned){} // OpenCPN management collaborator, not route receipt.
  void EnablePilotOutput(bool enabled){state.EnablePilot(enabled);}
  bool SendPilotMessage(std::shared_ptr<Nmea2000Msg> msg,std::shared_ptr<NavAddr2000>,uint64_t epoch,uint64_t* ticket=nullptr){
    ++attempts;
    if(!state.Enqueue(msg->data,true,epoch,N2kSerialState::Clock::now(),msg->pgn!=59904,msg->pgn==126208&&msg->data.back()==0,ticket,msg->pgn==126208&&msg->data.back()==0x80))return false;
    accepted.push_back(msg);return true;
  }
  void ClearNavigationOutput(){state.ClearNavigation();}
  bool SendNavigationBatch(const std::vector<std::shared_ptr<const Nmea2000Msg>>& frames,uint64_t epoch,uint16_t token,N2kSerialState::Clock::time_point source_at){
    // Framework double only: the other target executes the actual serial
    // batch validator/encoder. Keep its real state and physical-write gates.
    return !frames.empty() && state.EnqueueNavigation({0x99},epoch,token,source_at);
  }
};
namespace opennav::integration {
CommDriverN2KSerial serial;
PilotOutputEndpoint endpoint;
void MainThread(){}
CommDriverN2K* Driver(const std::string& iface){return iface=="fake-serial" ? &serial : nullptr;}
PilotOutputEndpoint Endpoint(CommDriverN2K*){return endpoint;}
#include "actual_gate_types.inc"
class OpenCPNPilot : public adapters::IN2kPilotTransport {
public:
  adapters::St4000Binding binding_;
  adapters::St4000Pilot pilot_{*this};
  PilotStatusDiscovery status_{*this};
  bool allowed=true,session_enabled_=false,awaiting_serial_write_=false;
  uint64_t session_epoch_=0,awaited_write_ticket_=0;
  uint64_t control_generation_=0;
  uint16_t navigation_token_=0;
  vessel::Time navigation_source_at_{};
  std::string navigation_blocker_="Waiting for a coherent active route";
  std::function<bool()> navigation_current_;
  std::optional<vessel::Time> last_discovery_;
  std::function<bool()> output_allowed_=[this]{return allowed;};
  adapters::PilotTransportStatus status{true,true,1,"FAKE ONLY"};
  adapters::PilotTransportStatus Status(const std::string&)const override {auto s=status;s.writable=s.writable&&allowed;return s;}
  bool Send(const adapters::PilotRequest&);
  bool Send(const std::string&,uint8_t,uint32_t,uint8_t,const std::vector<uint8_t>&)override;
  void SetControlEnabled(bool);
  bool RequestIdentity(vessel::Time);
  adapters::PilotFeedback GetState() const;
  std::optional<NavigationOutputSession> NavigationSession() const;
  bool OwnsNavigationInterface(const std::string&) const;
  void SetNavigationValidator(std::function<bool()>);
  bool SendNavigationBatch(const NavigationOutputSession&,const std::vector<NavigationOutputFrame>&,uint16_t,vessel::Time);
  void InvalidateNavigationOutput(const std::string&);
  std::string NavigationBlocker() const;
};
#include "actual_gate.inc"

void Check(bool b,const char* why){if(!b)throw std::runtime_error(why);}
int main(){try{
  using namespace opennav::integration;
  using namespace std::chrono_literals;
  serial.state.Connection(true);
  endpoint.serial=endpoint.enabled=endpoint.bidirectional=endpoint.connected=endpoint.actisense=true;
  OpenCPNPilot gate;
  const uint64_t name=(uint64_t(0xc0)<<56)|(uint64_t(80)<<48)|(uint64_t(135)<<40)|(uint64_t(1851)<<21)|1234;
  const auto now=vessel::Clock::now();
  const std::vector<uint8_t> iso{0,0xee,0};
  gate.binding_={"fake-serial","",false};gate.pilot_.Configure(gate.binding_);
  Check(gate.RequestIdentity(now),"selected connection can request NAME with control OFF and no guessed identity");
  Check(!gate.RequestIdentity(now+1s),"identity rate limit");
  Check(serial.state.WriteOne([](auto& p){return p.size();})==1,"one discovery item");
  gate.binding_.name=adapters::FormatPilotName(name);gate.pilot_.Configure(gate.binding_);
  adapters::PilotRequest request{1,adapters::PilotAction::Standby,0,now};
  auto standby=adapters::EncodeSt4000Command(request);
  auto track=standby;track.back()=0x80;
  // These payload shapes isolate the integration/session boundary. The other
  // target executes the actual serial payload validator and wire encoder.
  const std::vector<NavigationOutputFrame> frames{{129285,{1}},{129284,{2}},{129283,{3}}};
  Check(!gate.Send("fake-serial",42,126208,3,standby),"no identity or permission cannot reach driver");
  gate.binding_.permit_control=true;gate.pilot_.Configure(gate.binding_);
  std::vector<uint8_t> claim(8);for(unsigned i=0;i<8;++i)claim[i]=(name>>(i*8))&255;
  gate.pilot_.Observe({"fake-serial",60928,42,claim,now-400ms},now);
  gate.pilot_.Observe({"fake-serial",65379,42,{0x3b,0x9f,0x40,0,255,255,255,255},now-300ms},now);
  gate.SetControlEnabled(true);
  Check(!gate.Send("fake-serial",42,126208,3,adapters::EncodeSt4000Command({1,adapters::PilotAction::Auto,0,now})),"AUTO requires actual fresh magnetic heading at final sink");
  Check(!gate.Send("fake-serial",42,126208,3,adapters::EncodeSt4000Command({1,adapters::PilotAction::AlterCourse,1,now})),"course change requires fresh locked heading at final sink");
  gate.SetNavigationValidator([]{return true;});
  const auto without_heading=gate.NavigationSession();
  Check(without_heading && gate.SendNavigationBatch(*without_heading,frames,6,vessel::Clock::now()),"confirmed AUTO can supply navigation without a heading substitute");
  Check(serial.state.WriteOne([](auto& p){return p.size();})==1 && gate.Send("fake-serial",42,126208,3,track),"TRACK uses confirmed mode and current route receipt, not locked heading");
  Check(serial.state.WriteOne([](auto& p){return p.size();})==1,"TRACK still needs its actual writer");
  const auto wind=adapters::EncodeSt4000Command({1,adapters::PilotAction::Wind,0,now});
  Check(gate.Send("fake-serial",42,126208,3,wind),"WIND uses fresh physical feedback without inventing a heading prerequisite");
  Check(serial.state.WriteOne([](auto& p){return p.size();})==1,"WIND still needs the actual writer");
  gate.SetControlEnabled(false);
  gate.pilot_.Observe({"fake-serial",127250,42,{1,0x10,0x27,255,255,255,255,0xfd},now-200ms},now);
  gate.pilot_.Observe({"fake-serial",65360,42,{0x3b,0x9f,1,255,255,0x10,0x27,255},now-100ms},now);
  Check(gate.pilot_.Capabilities().manual_control,"exact observed identity and saved permission");
  Check(!gate.Send(request),"saved permission alone cannot enable a session");
  gate.SetControlEnabled(true);
  Check(gate.session_enabled_,"explicit session enabled with fresh feedback");
  auto reject=[&](std::string iface,unsigned dest,unsigned pgn,unsigned priority,std::vector<uint8_t> data,const char* why){
    const auto before=serial.attempts;
    Check(!gate.Send(iface,dest,pgn,priority,data),why);
    Check(serial.attempts==before,"final denial never invokes driver");
  };
  reject("wrong",42,126208,3,standby,"wrong interface");
  reject("fake-serial",204,126208,3,standby,"never fixed destination 204");
  reject("fake-serial",42,126208,6,standby,"wrong priority");
  reject("fake-serial",42,126208,3,track,"TRACK without current route receipt refused");
  auto unsupported=standby;unsupported.back()=2;
  reject("fake-serial",42,126208,3,unsupported,"unsupported command remains refused");
  auto malformed=standby;malformed[4]=0;reject("fake-serial",42,126208,3,malformed,"only exact known command encoding");
  gate.allowed=false;reject("fake-serial",42,126208,3,standby,"replay/source isolation guard");gate.allowed=true;
  endpoint.bidirectional=false;reject("fake-serial",42,126208,3,standby,"actual endpoint direction");endpoint.bidirectional=true;
  ++gate.status.epoch;reject("fake-serial",42,126208,3,standby,"connection epoch changed");--gate.status.epoch;
  bool route_current=true;
  gate.SetNavigationValidator([&]{return route_current;});
  const auto session=gate.NavigationSession();
  Check(session.has_value() && gate.OwnsNavigationInterface("fake-serial") && !gate.OwnsNavigationInterface("wrong"),"navigation owns only the explicitly selected connection");
  auto wrong_session=*session;wrong_session.interface_id="wrong";
  Check(!gate.SendNavigationBatch(wrong_session,frames,7,vessel::Clock::now()),"navigation interface custody");
  wrong_session=*session;++wrong_session.epoch;
  Check(!gate.SendNavigationBatch(wrong_session,frames,7,vessel::Clock::now()),"navigation epoch custody");
  wrong_session=*session;++wrong_session.generation;
  Check(!gate.SendNavigationBatch(wrong_session,frames,7,vessel::Clock::now()),"navigation control-session custody");
  auto wrong_frames=frames;std::swap(wrong_frames[0],wrong_frames[1]);
  Check(!gate.SendNavigationBatch(*session,wrong_frames,7,vessel::Clock::now()),"integration keeps exact batch order");
  Check(!gate.SendNavigationBatch(*session,frames,7,vessel::Clock::now()+1s) && !gate.SendNavigationBatch(*session,frames,7,vessel::Clock::now()-3s),"integration retains original source-age bounds");
  route_current=false;
  Check(!gate.SendNavigationBatch(*session,frames,7,vessel::Clock::now()),"route invalidation refuses batch");
  route_current=true;
  Check(gate.SendNavigationBatch(*session,frames,7,vessel::Clock::now()) && !gate.NavigationBlocker().empty(),"queued batch is not a local-write receipt");
  reject("fake-serial",42,126208,3,track,"TRACK cannot use a merely queued route batch");
  Check(serial.state.WriteOne([](auto&) -> size_t {throw std::runtime_error("invalidated queued route written");})==0,"early TRACK refusal invalidates unsent route");
  Check(gate.SendNavigationBatch(*session,frames,7,vessel::Clock::now()),"fresh route can be supplied after refusal");
  const auto previous_pilot_ticket=serial.state.Get().pilot_written_ticket;
  Check(serial.state.WriteOne([](auto& p){return p.size();})==1 && gate.NavigationBlocker().empty(),"only completed local route write opens TRACK sink");
  Check(serial.state.Get().pilot_written_ticket==previous_pilot_ticket,"navigation is not a pilot command acknowledgement");
  Check(gate.Send("fake-serial",42,126208,3,track),"TRACK admitted from confirmed AUTO with current navigation receipt");
  Check(serial.state.WriteOne([](auto& p){return p.size();})==1,"TRACK still requires its own completed write");
  Check(gate.SendNavigationBatch(*session,frames,7,vessel::Clock::now()) && !gate.NavigationBlocker().empty(),"new sample with the same waypoint token cannot borrow the previous write receipt");
  Check(serial.state.WriteOne([](auto& p){return p.size();})==1 && gate.NavigationBlocker().empty(),"refreshed sample requires its own local write");
  route_current=false;
  reject("fake-serial",42,126208,3,track,"route changed after receipt refuses TRACK");
  Check(serial.state.Get().navigation_written_token==0,"route change revokes receipt");
  route_current=true;
  gate.SetControlEnabled(false);gate.SetControlEnabled(true);
  Check(!gate.SendNavigationBatch(*session,frames,7,vessel::Clock::now()),"old control generation cannot republish after disable/re-enable");
  const auto new_session=gate.NavigationSession();
  Check(new_session && new_session->generation!=session->generation && serial.state.Get().navigation_written_token==0,"new control session starts without navigation receipt");
  auto observe_mode=[&](unsigned mode){
    const auto at=vessel::Clock::now();
    gate.pilot_.Observe({"fake-serial",65379,42,{0x3b,0x9f,uint8_t(mode),uint8_t(mode>>8),255,255,255,255},at},at);
    // STANDBY clears the old lock. Supply a new physical lock sample so the
    // following refusal specifically distinguishes the confirmed-mode guard.
    const auto locked_at=vessel::Clock::now();
    gate.pilot_.Observe({"fake-serial",65360,42,{0x3b,0x9f,1,255,255,0x10,0x27,255},locked_at},locked_at);
  };
  for(unsigned mode:{0u,0x100u,0x180u}) {
    observe_mode(mode);
    reject("fake-serial",42,126208,3,adapters::EncodeSt4000Command({1,adapters::PilotAction::AlterCourse,1,now}),"course changes require confirmed AUTO, even with locked heading");
    if(mode!=0x180u) {
      Check(!gate.NavigationSession(),"STANDBY/WIND cannot publish native route output");
      reject("fake-serial",42,126208,3,track,"TRACK requires confirmed AUTO or TRACK");
    } else {
      const auto in_track=gate.NavigationSession();
      Check(in_track && gate.SendNavigationBatch(*in_track,frames,8,vessel::Clock::now()),"confirmed TRACK can refresh native navigation");
      Check(serial.state.WriteOne([](auto& p){return p.size();})==1 && gate.Send("fake-serial",42,126208,3,track),"TRACK remains permitted with a current route write");
      Check(serial.state.WriteOne([](auto& p){return p.size();})==1,"drain repeated TRACK once");
    }
  }
  observe_mode(0x40);
  for(const auto& r:std::vector<adapters::PilotRequest>{{1,adapters::PilotAction::Standby,0,now},{2,adapters::PilotAction::Auto,0,now},{3,adapters::PilotAction::AlterCourse,-1,now},{4,adapters::PilotAction::AlterCourse,1,now},{5,adapters::PilotAction::AlterCourse,-10,now},{6,adapters::PilotAction::AlterCourse,10,now}}){
    Check(gate.Send("fake-serial",42,126208,3,adapters::EncodeSt4000Command(r)),"all six exact manual packets reach qualified sink");
    Check(gate.awaiting_serial_write_,"queue acceptance waits for actual serial write provenance");
    Check(serial.state.WriteOne([](auto& p){return p.size();})==1,"drain fake command once");
  }
  // Reproduce the interleaving behind the old counter watermark race: the
  // previous AUTO completes before STANDBY is enqueued. Its completion and a
  // fresh matching mode must not count as STANDBY's serial write.
  const auto auto_command=adapters::EncodeSt4000Command({7,adapters::PilotAction::Auto,0,now});
  Check(gate.Send("fake-serial",42,126208,3,auto_command),"queue preceding AUTO");
  const auto auto_ticket=gate.awaited_write_ticket_;
  Check(serial.state.WriteOne([](auto& p){return p.size();})==1,"preceding AUTO completes");
  Check(gate.Send("fake-serial",42,126208,3,standby),"enqueue STANDBY after preceding completion");
  const auto standby_ticket=gate.awaited_write_ticket_;
  Check(standby_ticket>auto_ticket && serial.state.Get().pilot_written_ticket==auto_ticket,"enqueue ticket is assigned atomically and identifies this command");
  const auto between_writes=vessel::Clock::now();
  gate.pilot_.Observe({"fake-serial",65379,42,{0x3b,0x9f,0,0,255,255,255,255},between_writes},between_writes);
  Check(!gate.GetState().command_confirmation_allowed,"actual GetState rejects post-AUTO/pre-STANDBY matching feedback");
  Check(serial.state.WriteOne([](auto& p){return p.size();})==1,"the exact STANDBY packet completes");
  Check(gate.GetState().command_confirmation_allowed && serial.state.Get().pilot_written_ticket==standby_ticket,"confirmation is eligible only for exact written ticket");
  Check(gate.GetState().observed_at<=gate.GetState().command_written_at,"pre-write sample still fails controller's write-time boundary");
  Check(gate.Send("fake-serial",42,126208,3,standby),"queue before disable");
  gate.SetControlEnabled(false);
  Check(serial.state.WriteOne([](auto&){throw std::runtime_error("disabled command transmitted");return size_t(0);})==0,"session disable cancels unsent command");
  reject("fake-serial",42,126208,3,standby,"disabled session final guard");
  reject("fake-serial",42,126208,3,track,"TRACK cannot bypass disabled session");
  reject("fake-serial",42,126208,3,wind,"WIND cannot bypass disabled session");
  const auto unavailable_at=vessel::Clock::now();
  gate.pilot_.Observe({"fake-serial",65379,42,{0x3b,0x9f,0xff,0xff,255,255,255,255},unavailable_at},unavailable_at);
  gate.SetControlEnabled(true);
  Check(!gate.session_enabled_,"unavailable physical feedback cannot enable session");
  std::cout<<"PASS actual OpenCPNPilot final sink/session/identity gate\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
