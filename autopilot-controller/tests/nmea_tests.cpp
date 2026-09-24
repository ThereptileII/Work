#include <NMEA2000.h>
#include <N2kGroupFunction.h>
#include <N2kMessages.h>
#include "BridgeCore.h"
#include <deque>
#include <vector>
#include <cassert>
#include <cstring>
#include <cstdio>
#include <thread>
#include <chrono>

struct Frame { unsigned long id; uint8_t length; uint8_t data[8]; };
class FakeBus : public tNMEA2000 {
 public:
  std::deque<Frame> incoming;
  std::vector<Frame> outgoing;
 protected:
  bool CANOpen() override { return true; }
  bool CANSendFrame(unsigned long id,unsigned char n,const unsigned char *p,bool) override {
    Frame f={};f.id=id;f.length=n;memcpy(f.data,p,n);outgoing.push_back(f);return true;
  }
  bool CANGetFrame(unsigned long &id,unsigned char &n,unsigned char *p) override {
    if(incoming.empty())return false;
    auto f=incoming.front();incoming.pop_front();id=f.id;n=f.length;memcpy(p,f.data,n);return true;
  }
};
class Handler : public tN2kGroupFunctionHandler {
 public:
  unsigned count=0;
  explicit Handler(FakeBus &bus):tN2kGroupFunctionHandler(&bus,65360) {}
 protected:
  bool HandleCommand(const tN2kMsg &msg,uint8_t,uint8_t,int) override {
    bridge::Command command;
    if(bridge::parseCommand(msg.Data,msg.DataLen,command)) { assert(command.type==bridge::CommandType::Heading);++count; }
    return true;
  }
};
class ModeHandler : public tN2kGroupFunctionHandler {
 public:
  FakeBus &bus; bool allow=false; unsigned count=0;
  explicit ModeHandler(FakeBus &bus):tN2kGroupFunctionHandler(&bus,65379),bus(bus) {}
 protected:
  bool HandleCommand(const tN2kMsg &msg,uint8_t,uint8_t fields,int device) override {
    bridge::Command command;
    assert(bridge::parseCommand(msg.Data,msg.DataLen,command));
    assert(command.type==bridge::CommandType::Mode && command.mode==bridge::Mode::Standby);
    ++count;
    SendAcknowledge(&bus,msg.Source,device,PGN,
      allow?N2kgfPGNec_Acknowledge:N2kgfPGNec_PGNTemporarilyNotAvailable,
      N2kgfTPec_Acknowledge,fields,
      allow?N2kgfpec_Acknowledge:N2kgfpec_TemporarilyUnableToComply);
    return true;
  }
};
static std::vector<Frame> frames(uint8_t sequence,uint8_t source,uint8_t dest=33,
    const std::vector<uint8_t> &payload={1,0x50,0xff,0,0xf8,3,1,0x3b,7,3,4,6,0,0x40}) {
  std::vector<Frame> result;
  size_t pos=0;unsigned index=0;
  while(pos<payload.size()) {
    Frame f;f.id=0x0ded0000u|(uint32_t(dest)<<8)|source;f.length=8;memset(f.data,0xff,8);
    f.data[0]=(sequence<<5)|index;
    unsigned begin=index==0?2:1;if(index==0)f.data[1]=payload.size();
    for(unsigned j=begin;j<8 && pos<payload.size();++j)f.data[j]=payload[pos++];
    result.push_back(f);++index;
  }
  return result;
}
int main() {
  // Like firmware, the NMEA2000 object has process lifetime. The library keeps
  // its startup buffers/handlers for that lifetime and has no teardown API.
  static FakeBus bus;static Handler handler(bus);static ModeHandler modeHandler(bus);
  bus.SetDeviceInformation(1234,135,40,1851,4);
  bus.SetMode(tNMEA2000::N2km_ListenAndNode,33);bus.EnableForward(false);
  bus.SetN2kCANMsgBufSize(16);bus.AddGroupFunctionHandler(&handler);
  bus.AddGroupFunctionHandler(&modeHandler);
  bus.Open();
  for(unsigned i=0;i<7;++i) { std::this_thread::sleep_for(std::chrono::milliseconds(100));bus.ParseMessages(); }
  assert(bus.Open());
  for(unsigned seq=0;seq<8;++seq) {
    auto a=frames(seq,7),b=frames(seq,8);
    for(size_t i=0;i<a.size();++i) { bus.incoming.push_back(a[i]);bus.incoming.push_back(b[i]); }
    bus.ParseMessages();assert(handler.count==(seq+1)*2);
  }
  auto addressedElsewhere=frames(0,7,34);
  for(auto f:addressedElsewhere)bus.incoming.push_back(f);
  bus.ParseMessages();assert(handler.count==16);
  const std::vector<uint8_t> modePayload={1,0x63,0xff,0,0xf8,4,1,0x3b,7,3,4,4,0,0,5,0xff,0xff};
  for(unsigned accepted=0;accepted<2;++accepted) {
    modeHandler.allow=accepted;bus.outgoing.clear();
    for(auto f:frames(4+accepted,7,33,modePayload))bus.incoming.push_back(f);
    bus.ParseMessages();
    std::vector<uint8_t> ack;
    for(const auto &f:bus.outgoing) {
      if(((f.id>>8)&0x1ff00)!=126208 || ((f.id>>8)&255)!=7)continue;
      assert((f.id&255)==33);
      unsigned start=(f.data[0]&31)==0?2:1;
      if(start==2)assert(f.data[1]==8);
      for(unsigned i=start;i<f.length && ack.size()<8;++i)ack.push_back(f.data[i]);
    }
    const std::vector<uint8_t> expected={2,0x63,0xff,0,uint8_t(accepted?0:2),4,uint8_t(accepted?0:0x22),uint8_t(accepted?0:0x22)};
    assert(ack==expected && modeHandler.count==accepted+1);
  }
  auto broadcast=frames(1,7,255);for(auto f:broadcast)bus.incoming.push_back(f);
  bus.ParseMessages();assert(handler.count==16);
  // Missing middle frame must never become a command.
  auto missing=frames(2,7);bus.incoming.push_back(missing[0]);bus.incoming.push_back(missing[2]);
  bus.ParseMessages();assert(handler.count==16);
  tN2kMsg msg;
  SetN2kPGN127237(msg,N2kOnOff_Unavailable,N2kOnOff_Off,N2kOnOff_Unavailable,N2kOnOff_Unavailable,
    N2kSM_HeadingControl,N2kTM_Unavailable,N2khr_magnetic,N2kRDO_Unavailable,
    N2kDoubleNA,5.0,N2kDoubleNA,N2kDoubleNA,N2kDoubleNA,N2kDoubleNA,N2kDoubleNA,N2kDoubleNA,5.1);
  assert(msg.DataLen==21);assert(bridge::u16(msg.Data+5)==50000);assert(bridge::u16(msg.Data+13)==0x7fff);
  SetN2kPGN129283(msg,12,N2kxtem_Autonomous,false,-1852.0);
  bridge::Navigation nav;assert(nav.receive(129283,7,msg.Data,msg.DataLen,0));assert(nav.xte.value==-1);
  puts("PASS: NMEA2000 transport, AutoTrack mode command/positive-negative ACKs and standard field encoding");
}
