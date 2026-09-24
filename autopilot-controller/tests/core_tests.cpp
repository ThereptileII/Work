#include "BridgeCore.h"
#include "Config.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include <random>

using namespace bridge;
static unsigned checks=0;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); abort(); } } while(0)
static void near(double a,double b,double tolerance=0.01) { CHECK(fabs(a-b)<=tolerance); }
static void put32(uint8_t *p,uint32_t v) { for(unsigned i=0;i<4;++i) p[i]=v>>(8*i); }
static Datagram status(Mode mode,double heading=90,double target=100) {
  Datagram d; d.size=9; d.data[0]=0x84;
  unsigned h=unsigned(lround(wrap(heading))), t=unsigned(lround(wrap(target)*2));
  unsigned u=h/90, rem=h%90;
  if(rem&1) u|=4;
  d.data[1]=(u<<4)|6;
  d.data[2]=((t/180)<<6)|(rem/2); d.data[3]=t%180;
  d.data[4]=mode==Mode::Standby?0:mode==Mode::Wind?6:mode==Mode::Track?10:2;
  d.data[6]=0xfe; d.data[8]=5; return d;
}
static void receive(Controller &c,Mode mode,double target,uint32_t now) {
  auto d=status(mode,90,target); c.receivePilot(d.data,d.size,now);
}
static Navigation readyNav(uint32_t now) {
  Navigation nav;
  uint8_t data[34]={}; data[5]=1; put32(data+1,950076); // 5.13 nm
  put16(data+14,uint16_t(lround(230*pi/180*10000))); put32(data+20,42);
  CHECK(nav.receive(129284,7,data,sizeof(data),now));
  uint8_t xte[8]={12,0}; CHECK(nav.receive(129283,7,xte,8,now));
  CHECK(nav.ready(now)); return nav;
}
static std::vector<uint8_t> headingCommand(uint16_t raw) {
  return {1,0x50,0xff,0,0xf8,3,1,0x3b,7,3,4,6,uint8_t(raw),uint8_t(raw>>8)};
}
static void testCommands() {
  Command cmd;
  // Every valid angle, including bytes that formerly meant STANDBY/AUTO/steps.
  for(unsigned raw=0;raw<=62832;++raw) {
    auto p=headingCommand(raw); CHECK(parseCommand(p.data(),p.size(),cmd));
    CHECK(cmd.type==CommandType::Heading); near(cmd.value,angle(raw));
  }
  auto p=headingCommand(0x4000); CHECK(parseCommand(p.data(),p.size(),cmd)); near(cmd.value,93.8734);
  for(size_t n=0;n<p.size();++n) CHECK(!parseCommand(p.data(),n,cmd));
  p[0]=2; CHECK(!parseCommand(p.data(),p.size(),cmd)); p[0]=0;CHECK(!parseCommand(p.data(),p.size(),cmd));
  p=headingCommand(0xffff); CHECK(!parseCommand(p.data(),p.size(),cmd));
  p=headingCommand(100); p[7]=0;CHECK(!parseCommand(p.data(),p.size(),cmd));
  p=headingCommand(100); p[10]=0;CHECK(!parseCommand(p.data(),p.size(),cmd));
  p=headingCommand(100); p.push_back(0);CHECK(!parseCommand(p.data(),p.size(),cmd));
  p=headingCommand(100);p[4]=0xf3;CHECK(!parseCommand(p.data(),p.size(),cmd));
  uint8_t mode[]={1,0x63,0xff,0,0xff,3,1,0x3b,7,3,4,4,0x80,1};
  CHECK(parseCommand(mode,sizeof(mode),cmd)); CHECK(cmd.type==CommandType::Mode && cmd.mode==Mode::Track);
  mode[12]=0;mode[13]=1; CHECK(parseCommand(mode,sizeof(mode),cmd)); CHECK(cmd.mode==Mode::Wind);
  uint8_t step[]={1,0x63,0xff,0,0xff,3,1,0x3b,7,3,4,6,0xd1};
  CHECK(parseCommand(step,sizeof(step),cmd)); CHECK(cmd.type==CommandType::Step && cmd.value==10);
  step[1]=0x50;CHECK(!parseCommand(step,sizeof(step),cmd));
  // Exact mode command emitted by AutoTrackRaymarine 2.3 (four parameters).
  uint8_t evo[]={1,0x63,0xff,0,0xf8,4,1,0x3b,7,3,4,4,0,0,5,0xff,0xff};
  CHECK(parseCommand(evo,sizeof(evo),cmd)); CHECK(cmd.mode==Mode::Standby);
  evo[12]=0x40;CHECK(parseCommand(evo,sizeof(evo),cmd)); CHECK(cmd.mode==Mode::Auto);
  for(size_t n=0;n<sizeof(evo);++n) CHECK(!parseCommand(evo,n,cmd));
  evo[15]=0;CHECK(!parseCommand(evo,sizeof(evo),cmd));evo[15]=0xff;
  evo[14]=4;CHECK(!parseCommand(evo,sizeof(evo),cmd));evo[14]=5;
  evo[7]=0;CHECK(!parseCommand(evo,sizeof(evo),cmd));
  std::mt19937 rng(42); uint8_t fuzz[223];
  for(unsigned i=0;i<30000;++i) {
    for(auto &v:fuzz) v=rng();
    parseCommand(fuzz,rng()%224,cmd);
  }
  FastPacketGuard guard;
  uint8_t first[]={0,14,1,0x50,0xff,0,0xf8,3};
  uint8_t second[]={1,1,0x3b,7,3,4,6,0};uint8_t third[]={2,0x40};
  CHECK(!guard.accept(126208,7,33,first,2,0));
  CHECK(guard.accept(126208,7,33,first,8,0));
  CHECK(!guard.accept(126208,7,33,second,7,1));
  CHECK(!guard.accept(126208,7,33,third,2,2));
  CHECK(guard.accept(126208,7,33,first,8,0));
  CHECK(guard.accept(126208,7,33,second,8,1));
  CHECK(guard.accept(126208,7,33,third,2,2));
  CHECK(guard.accept(126208,7,33,first,8,0));
  CHECK(!guard.accept(126208,7,33,second,8,251));
  CHECK(guard.accept(126208,7,33,first,8,300));
  CHECK(!guard.accept(126208,7,34,second,8,301));
}
static void testNavigation() {
  auto nav=readyNav(100);
  const uint8_t expected[]={0x85,0x06,0x00,0x42,0x16,0x20,0x17,0,0xe8};
  auto packet=navigation(nav,100); CHECK(packet.size==9);CHECK(!memcmp(packet.data,expected,9));
  nav.xte.set(-2.61,100); packet=navigation(nav,100);
  CHECK(packet.data[1]==0x16 && packet.data[2]==5); CHECK((packet.data[6]&0x40)!=0); CHECK(packet.data[6]&8);
  nav.bearing.set(89.9,100);packet=navigation(nav,100);CHECK(packet.data[3]==1 && (packet.data[4]&15)==0);
  nav.bearing.set(359.9,100);packet=navigation(nav,100);CHECK(packet.data[3]==0 && (packet.data[4]&15)==0);
  nav.distance.set(51.3,100);packet=navigation(nav,100);CHECK(packet.data[4]>>4==1);CHECK(packet.data[5]==0x20);CHECK(!(packet.data[6]&0x10));
  nav.distance.set(409.6,100); CHECK(!nav.ready(100));
  nav=readyNav(100);nav.waypoint=0xffffffff;CHECK(!nav.ready(100));
  nav=readyNav(100); nav.reference=0; CHECK(!nav.ready(100));
  nav.variation.set(10,100);near(nav.magneticBearing(100),220,0.01); CHECK(nav.ready(100));
  CHECK(!nav.ready(3100)); CHECK(navigation(nav,3100).size==0);
  nav=readyNav(100);
  uint8_t xte[8]={12,0};put32(xte+2,uint32_t(-185200));
  CHECK(nav.receive(129283,7,xte,8,101));near(nav.xte.value,-1);
  CHECK(!nav.receive(129283,8,xte,8,102));near(nav.xte.value,-1);
  put32(xte+2,0x7fffffff);CHECK(nav.receive(129283,7,xte,8,103));CHECK(!nav.ready(103));
  put32(xte+2,0); xte[1]=0x40;nav.receive(129283,7,xte,8,104);CHECK(!nav.ready(104));
  xte[1]=7;nav.receive(129283,7,xte,8,104);CHECK(!nav.ready(104));
  nav=readyNav(0xffffff00);CHECK(nav.ready(20));CHECK(!nav.ready(3000));
  const auto token=waypointToken(1);
  const uint8_t wp[]={0x82,5,0,0xff,0,0xff,4,0xfb};
  CHECK(token.size==8);CHECK(!memcmp(token.data,wp,8));
  CHECK(memcmp(waypointToken(35).data,waypointToken(36).data,8)!=0);
  nav=readyNav(100);const auto epoch=nav.epoch;
  uint8_t changed[34]={};changed[5]=1;put32(changed+1,185200);put16(changed+14,10000);put32(changed+20,43);
  CHECK(nav.receive(129284,7,changed,34,101));CHECK(nav.epoch==epoch+1);CHECK(!nav.ready(101));
  uint8_t freshXte[8]={};nav.receive(129283,7,freshXte,8,102);CHECK(nav.ready(102));
  Wind wind;uint8_t w[8]={};w[5]=2;put16(w+1,1000);put16(w+3,5000);
  CHECK(wind.receive(9,w,8,100));CHECK(wind.ready(101));near(wind.angle.value,28.6479);
  w[5]=0;CHECK(!wind.receive(9,w,8,200));CHECK(!wind.ready(3100));
  wind.localAngle.set(40,3200);CHECK(wind.ready(3200));CHECK(!wind.nmeaReady(3200));
}
static void testPilot() {
  for(unsigned heading=0;heading<360;++heading) {
    for(unsigned t=0;t<720;t+=17) {
      Pilot p; auto d=status(Mode::Auto,heading,t*0.5); CHECK(p.receive(d.data,d.size,0));
      near(p.heading,heading);near(p.target,t*0.5);near(p.rudder,-2);CHECK(p.alive(0)); CHECK(!p.alive(3000));
    }
  }
  Pilot p;CHECK(!p.alive(0));auto d=status(Mode::Wind);CHECK(p.receive(d.data,9,10));CHECK(p.mode==Mode::Wind);
  d=status(Mode::Track);CHECK(p.receive(d.data,9,20));CHECK(p.mode==Mode::Track);
  d=status(Mode::Standby);CHECK(p.receive(d.data,9,30));CHECK(isnan(p.target));
  d.data[4]=14;CHECK(!p.receive(d.data,9,40));CHECK(p.at==30);
  for(size_t n=0;n<9;++n)CHECK(!p.receive(d.data,n,40));
}
static void testController() {
  Controller c; Datagram d;Command autoCmd{CommandType::Mode,Mode::Auto,0};
  CHECK(!c.request(autoCmd,0));CHECK(c.pilot.mode==Mode::Unknown);
  receive(c,Mode::Standby,100,10);CHECK(c.request(autoCmd,20));CHECK(c.next(20,d));CHECK(d.data[2]==1);
  c.started(20);c.transmitted(true,30);CHECK(c.phase==Phase::Confirming);
  receive(c,Mode::Standby,100,40);CHECK(c.busy()); // old standby report while engaging
  receive(c,Mode::Auto,100,50);CHECK(!c.busy());CHECK(c.completed==1);
  Command heading{CommandType::Heading,Mode::Unknown,111};
  CHECK(c.request(heading,60));CHECK(c.next(60,d));CHECK(d.data[2]==8);
  c.started(60);c.transmitted(true,70);receive(c,Mode::Auto,100,80);CHECK(c.phase==Phase::Confirming);
  receive(c,Mode::Auto,110,90);CHECK(c.phase==Phase::Queued);
  CHECK(c.next(90,d));CHECK(d.data[2]==7);c.started(90);c.transmitted(true,100);
  receive(c,Mode::Auto,111,110);CHECK(!c.busy());CHECK(c.completed==2);
  CHECK(c.request(heading,120));CHECK(!c.next(120,d));CHECK(!c.busy());
  heading.value=121;CHECK(c.request(heading,130));CHECK(c.next(130,d));c.started(130);c.transmitted(false,140);
  CHECK(!c.busy());CHECK(!c.next(150,d)); // never repeat an uncertain +10
  receive(c,Mode::Auto,359,200);heading.value=0;CHECK(c.request(heading,201));CHECK(c.next(201,d));CHECK(d.data[2]==7);
  c.started(201);c.transmitted(true,210);receive(c,Mode::Standby,0,220);CHECK(!c.busy());
  receive(c,Mode::Auto,100,300);heading.value=110;CHECK(c.request(heading,301));CHECK(c.next(301,d));c.started(301);c.transmitted(true,310);
  receive(c,Mode::Auto,95,320);CHECK(!c.busy()); // external heading takes precedence
  CHECK(c.request(heading,321));c.tick(3321);CHECK(!c.busy()); // stale feedback
  Command standby{CommandType::Mode,Mode::Standby,0};CHECK(c.request(standby,4000));CHECK(c.next(4000,d));CHECK(d.data[2]==2);
  c.started(4000);c.transmitted(true,4010);c.tick(7600);CHECK(!c.busy());CHECK(c.pilot.mode==Mode::Auto);CHECK(!c.pilot.alive(7600));
  CHECK(c.request(standby,7700));CHECK(c.next(7700,d));c.started(7700);c.transmitted(true,7710);c.standbyKey();CHECK(c.busy());
  receive(c,Mode::Standby,100,7720);CHECK(!c.busy());
  receive(c,Mode::Auto,100,8000);c.nav=readyNav(8000);Command track{CommandType::Mode,Mode::Track,0};
  CHECK(c.request(track,8001));CHECK(c.next(8001,d));CHECK(d.data[2]==3);c.started(8001);c.transmitted(true,8010);
  receive(c,Mode::Track,100,8020);CHECK(!c.busy());
  c.nav.xte.clear();CHECK(!c.request(track,8030));
}
static void callback(void *ctx,const uint8_t *p,size_t n) {
  auto &v=*static_cast<std::vector<std::vector<uint8_t>> *>(ctx);v.emplace_back(p,p+n);
}
static void encodeRuns(WireDecoder &decoder,const Datagram &packet) {
  std::vector<bool> bits;
  for(unsigned i=0;i<packet.size;++i) {
    bits.push_back(false);for(unsigned b=0;b<8;++b)bits.push_back(packet.data[i]&(1<<b));
    bits.push_back(i==0);bits.push_back(true);
  }
  bits.insert(bits.end(),12,true);
  for(size_t a=0;a<bits.size();) {
    size_t b=a+1;while(b<bits.size() && bits[b]==bits[a])++b;
    decoder.run(bits[a],unsigned(lround((b-a)*1000000.0/4800)));a=b;
  }
  decoder.endCapture(true);
}
static void testWire() {
  std::vector<std::vector<uint8_t>> output; WireDecoder decoder(callback,&output);
  for(unsigned code=0;code<256;++code) {
    auto d=key(code);encodeRuns(decoder,d);CHECK(output.back()==std::vector<uint8_t>(d.data,d.data+d.size));
  }
  auto nav=readyNav(0);encodeRuns(decoder,navigation(nav,0));CHECK(output.size()==257);CHECK(decoder.errors==0);
  auto d=status(Mode::Auto);encodeRuns(decoder,d);CHECK(output.back()[0]==0x84);
  // Truncated packet must never survive into a later idle-separated capture.
  auto bad=key(1);bad.size=2;size_t count=output.size();encodeRuns(decoder,bad);CHECK(output.size()==count);CHECK(decoder.errors>0);
  encodeRuns(decoder,key(2));CHECK(output.size()==count+1);
  decoder.run(false,20);decoder.run(true,900000);decoder.endCapture(true);
}
static void testEvolutionFeedback() {
  Pilot p;
  CHECK(!evolutionStatus(p,0).size);
  CHECK(!publishLockedHeading(p,0,true));
  const uint8_t observed[]={0x84,0x36,0x1e,0,0,0,0xfc,0,8};
  CHECK(p.receive(observed,sizeof(observed),100));
  auto d=evolutionStatus(p,101);
  const uint8_t expected[]={0x3b,0x9f,0xf0,0x81,0x84,0x36,0x1e,0,0x40,0,0xfc,0,8};
  CHECK(d.size==sizeof(expected) && !memcmp(d.data,expected,sizeof(expected)));
  CHECK(!publishLockedHeading(p,101,true));CHECK(publishLockedHeading(p,101,false));
  // AutoTrack receives a 13-byte transport header and a trailing checksum.
  std::vector<uint8_t> pluginMessage(13,0);
  pluginMessage.insert(pluginMessage.end(),d.data,d.data+d.size);pluginMessage.push_back(0);
  CHECK(pluginMessage.size()==27 && pluginMessage[21]==0x40);
  auto actual=status(Mode::Auto,330,331);
  CHECK(p.receive(actual.data,actual.size,200));
  d=evolutionStatus(p,201);CHECK(d.size==13 && d.data[8]==0x42);
  CHECK(publishLockedHeading(p,201,true));
  CHECK(!evolutionStatus(p,3200).size);CHECK(!publishLockedHeading(p,3200,true));
  // Invalid status cannot replace the last validated physical mode.
  actual.data[4]=0x40;CHECK(!p.receive(actual.data,actual.size,202));
  CHECK(evolutionStatus(p,203).data[8]==0x42);
}
int main() {
  testCommands();testNavigation();testPilot();testController();testWire();testEvolutionFeedback();
  printf("PASS: %u checks (commands, navigation, feedback, controller, wire decoder)\n",checks);
}
