#include "BridgeCore.h"
#include "Config.h"
#include <string.h>
#include <algorithm>

namespace bridge {
const char *modeName(Mode m) {
  switch(m) { case Mode::Standby:return "STANDBY"; case Mode::Auto:return "AUTO";
    case Mode::Wind:return "WIND"; case Mode::Track:return "TRACK"; default:return "UNKNOWN"; }
}
bool Pilot::alive(uint32_t now) const { return valid && fresh(now,at,config::feedbackTimeoutMs); }
bool Pilot::receive(const uint8_t *p, size_t n, uint32_t now) {
  if (!p || n!=9 || p[0]!=0x84 || (p[1]&15)!=6 || (p[4]&0xf0) || (p[5]&0xf0)) return false;
  const uint8_t u=p[1]>>4, v=p[2]>>4, flags=p[4];
  // Auto must be set for Wind/Track. Mutually contradictory modes are invalid.
  if (((flags&12) && !(flags&2)) || (flags&12)==12) return false;
  heading=wrap((u&3)*90+(p[2]&0x3f)*2+((u&0xc)==0xc?2:((u&0xc)?1:0)));
  target=wrap((v>>2)*90+p[3]*0.5);
  mode=!(flags&2)?Mode::Standby:((flags&8)?Mode::Track:((flags&4)?Mode::Wind:Mode::Auto));
  if(mode==Mode::Standby) target=NAN;
  int r=static_cast<int8_t>(p[6]); rudder=abs(r)<=90?r:NAN;
  offCourse=p[5]&4; windShift=p[5]&8;
  memcpy(seaTalkStatus,p,sizeof(seaTalkStatus));
  at=now; valid=true; ++generation;
  return true;
}
Datagram evolutionStatus(const Pilot &pilot,uint32_t now) {
  Datagram d;
  if(!pilot.alive(now)) return d;
  d.size=13;
  d.data[0]=0x3b;d.data[1]=0x9f;d.data[2]=0xf0;d.data[3]=0x81;
  memcpy(d.data+4,pilot.seaTalkStatus,9);
  // AutoTrack 2.3's EV-1 status consumer expects 0x40 (standby) / 0x42 (auto)
  // at payload byte 8. Preserve measured mode flags, alarms and heading bytes.
  d.data[8]|=0x40;
  return d;
}
bool publishLockedHeading(const Pilot &pilot,uint32_t now,bool autoTrackCompatibility) {
  // AutoTrack treats receipt of PGN 65360 itself as engagement, ignoring NA.
  return !autoTrackCompatibility || (pilot.alive(now) && pilot.mode!=Mode::Standby && isfinite(pilot.target));
}
bool Navigation::acceptSource(uint8_t sa) {
  if(sa>251 || (config::navigationSource!=255 && sa!=config::navigationSource)) return false;
  if(source==255) source=sa;
  return source==sa;
}
bool Navigation::receive(uint32_t pgn, uint8_t sa, const uint8_t *p, size_t n, uint32_t now) {
  if(!p) return false;
  if(pgn==129283) {
    if(n!=8) return false;
    const uint8_t mode=p[1]&15, term=(p[1]>>6)&3;
    const uint32_t raw=u32(p+2);
    if(source==255 && (term!=0 || mode>1 || (raw>=0x7ffffffd && raw<=0x7fffffff))) return false;
    if(!acceptSource(sa)) return false;
    terminated=term!=0; xteSeen=true;
    // Simulator/manual/error modes do not qualify as live navigation.
    if(term!=0 || mode>1 || raw==0x7fffffff || raw==0x7ffffffe || raw==0x7ffffffd) xte.clear();
    else xte.set(static_cast<int32_t>(raw)*0.01/1852.0,now);
    return true;
  }
  if(pgn==129284) {
    if(n!=34) return false;
    if(source==255 && (u32(p+1)>=0xfffffffd || !angleValid(u16(p+14)) || (p[5]&3)>1)) return false;
    if(!acceptSource(sa)) return false;
    const uint32_t wp=u32(p+20);
    if(wp!=waypoint) { if(waypoint!=0xffffffff) xte.clear(); waypoint=wp; ++epoch; }
    reference=p[5]&3;
    const uint32_t raw=u32(p+1);
    if(raw>=0xfffffffd) distance.clear(); else distance.set(raw*0.01/1852.0,now);
    const uint16_t brg=u16(p+14);
    if(!angleValid(brg) || reference>1) bearing.clear(); else bearing.set(angle(brg),now);
    // Never fall back to origin->destination bearing or vessel heading.
    return true;
  }
  return false;
}
double Navigation::magneticBearing(uint32_t now) const {
  if(!bearing.alive(now,config::navigationTimeoutMs)) return NAN;
  if(reference==1) return bearing.value;
  if(reference==0 && variation.alive(now,config::variationTimeoutMs)) return wrap(bearing.value-variation.value);
  return NAN;
}
bool Navigation::ready(uint32_t now) const {
  return waypoint<0xfffffffd && !terminated && xteSeen && xte.alive(now,config::navigationTimeoutMs)
    && fabs(xte.value)<=40.95 && distance.alive(now,config::navigationTimeoutMs)
    && distance.value>=0 && distance.value<=409.5 && isfinite(magneticBearing(now));
}
bool Wind::receive(uint8_t sa, const uint8_t *p, size_t n, uint32_t now) {
  if(!p || n!=8 || sa>251 || (p[5]&7)!=2 || (config::windSource!=255 && sa!=config::windSource)) return false;
  if(source==255) source=sa;
  if(source!=sa) return false;
  const uint16_t sp=u16(p+1), an=u16(p+3);
  if(sp>=0xfffd) speed.clear(); else speed.set(sp*0.01*1.943844492,now);
  if(!angleValid(an)) angle.clear(); else angle.set(bridge::angle(an),now);
  return true;
}
bool Wind::ready(uint32_t now) const {
  return localAngle.alive(now,config::windTimeoutMs) || nmeaReady(now);
}
bool Wind::nmeaReady(uint32_t now) const {
  return angle.alive(now,config::windTimeoutMs) && speed.alive(now,config::windTimeoutMs) && speed.value<=127.9;
}
Datagram key(uint8_t code) { Datagram d; d.size=4; d.data[0]=0x86; d.data[1]=0x21; d.data[2]=code; d.data[3]=~code; return d; }
Datagram navigation(const Navigation &nav, uint32_t now) {
  Datagram d;
  if(!nav.ready(now)) return d;
  const auto x=uint16_t(lround(fabs(nav.xte.value)*100));
  // Quantize BEFORE splitting quadrants so 89.9 degrees rounds to 90 correctly.
  const unsigned half=unsigned(lround(nav.magneticBearing(now)*2))%720;
  const uint8_t quadrant=half/180, within=half%180;
  const bool fine=nav.distance.value<10;
  const auto dist=uint16_t(lround(nav.distance.value*(fine?100:10)));
  const uint8_t y=(fine?1:0)|(nav.xte.value<0?4:0);
  const uint8_t f=7|(fabs(nav.xte.value)>=0.3?8:0);
  d.size=9; d.data[0]=0x85; d.data[1]=((x>>8)<<4)|6; d.data[2]=x;
  d.data[3]=((within&15)<<4)|quadrant;
  d.data[4]=((dist&15)<<4)|(within>>4); d.data[5]=dist>>4;
  d.data[6]=(y<<4)|f; d.data[7]=0; d.data[8]=~d.data[6];
  return d;
}
Datagram windAngle(double degrees) {
  Datagram d; if(!isfinite(degrees)) return d;
  auto a=uint16_t(lround(wrap(degrees)*2))%720;
  d.size=4; d.data[0]=0x10; d.data[1]=1; d.data[2]=a>>8; d.data[3]=a; return d;
}
Datagram waypointToken(uint32_t epoch) {
  // SeaTalk carries four characters. Use a monotonically changing base-36
  // bridge token, avoiding collisions from truncating 32-bit waypoint IDs.
  uint8_t chars[4];
  for(int i=3;i>=0;--i) { unsigned c=epoch%36; epoch/=36; chars[i]=c<10?c:c+7; }
  Datagram d; d.size=8; d.data[0]=0x82; d.data[1]=5;
  d.data[2]=chars[0]|((chars[1]&3)<<6); d.data[3]=~d.data[2];
  d.data[4]=(chars[1]>>2)|((chars[2]&15)<<4); d.data[5]=~d.data[4];
  d.data[6]=(chars[2]>>4)|(chars[3]<<2); d.data[7]=~d.data[6];
  return d;
}
Datagram windSpeed(double knots) {
  Datagram d; if(!isfinite(knots)||knots<0||knots>127.9) return d;
  auto tenth=unsigned(lround(knots*10));
  d.size=4; d.data[0]=0x11; d.data[1]=1; d.data[2]=tenth/10; d.data[3]=tenth%10; return d;
}
Datagram waterSpeed(double knots) {
  Datagram d; if(!isfinite(knots)||knots<0||knots>6553.2) return d;
  d.size=4; d.data[0]=0x20; d.data[1]=1; put16(d.data+2,uint16_t(lround(knots*10))); return d;
}

static bool modeValue(uint16_t v, Mode &m) {
  switch(v) { case 0:m=Mode::Standby;return true; case 0x40:m=Mode::Auto;return true;
    case 0x100:m=Mode::Wind;return true; case 0x180:m=Mode::Track;return true; default:return false; }
}
void FastPacketGuard::clear() { for(auto &s:slots) s={}; }
bool FastPacketGuard::accept(uint32_t pgn,uint8_t source,uint8_t destination,const uint8_t *p,size_t n,uint32_t now) {
  if(!p || n<2 || n>8) return false;
  Slot *slot=nullptr,*freeSlot=nullptr;
  for(auto &s:slots) {
    if(s.active && !fresh(now,s.at,250)) s.active=false;
    if(s.active && s.pgn==pgn && s.source==source) slot=&s;
    if(!s.active && !freeSlot) freeSlot=&s;
  }
  if((p[0]&31)==0) {
    if(slot) slot->active=false;
    if(p[1]==0 || p[1]>223 || n<std::min<size_t>(8,p[1]+2)) return false;
    if(!slot) slot=freeSlot;
    if(!slot) return false;
    slot->pgn=pgn;slot->source=source;slot->destination=destination;slot->at=now;slot->next=p[0]+1;
    slot->remaining=p[1]>6?p[1]-6:0;slot->active=slot->remaining!=0;
    return true;
  }
  if(!slot) return false;
  if(slot->destination!=destination || slot->next!=p[0] || n<std::min<size_t>(8,slot->remaining+1)) { slot->active=false;return false; }
  ++slot->next;slot->remaining-=std::min<uint8_t>(7,slot->remaining);slot->active=slot->remaining!=0;
  return true;
}
bool parseCommand(const uint8_t *p, size_t n, Command &out) {
  out={};
  if(!p || n<6 || p[0]!=1 || (p[4]&0xf0)!=0xf0) return false;
  const uint8_t priority=p[4]&15;
  if(priority!=8 && priority!=9 && priority!=15) return false;
  const uint32_t target=p[1]|uint32_t(p[2])<<8|uint32_t(p[3])<<16;
  if(target!=65360 && target!=65379) return false;
  bool mfg=false, industry=false, action=false;
  uint32_t fields=0; size_t pos=6;
  Command cmd;
  for(unsigned i=0;i<p[5];++i) {
    if(pos>=n) return false;
    const uint8_t field=p[pos++];
    if(field>7 || !field || (fields&(1u<<field))) return false;
    fields|=1u<<field;
    unsigned width=0;
    if(field==1) width=2;
    else if(field==3) width=1;
    else if(target==65360 && field==6) width=2;
    else if(target==65379 && field==4) width=2;
    else if(target==65379 && field==5) width=2;
    else if(target==65379 && field==6) width=1;
    else return false; // No scanning past fields of unknown size.
    if(n-pos<width) return false;
    const uint16_t value=width==2?u16(p+pos):p[pos]; pos+=width;
    if(field==1) { if((value&0x7ff)!=config::manufacturer) return false; mfg=true; }
    else if(field==3) { if(value!=4) return false; industry=true; }
    else if(target==65379 && field==5) { if(value!=0xffff) return false; }
    else {
      if(action) return false;
      action=true;
      if(target==65360) {
        if(!angleValid(value)) return false;
        cmd.type=CommandType::Heading; cmd.value=angle(value);
      } else if(field==4) {
        cmd.type=CommandType::Mode; if(!modeValue(value,cmd.mode)) return false;
      } else {
        // Explicit legacy button field, confined to PGN 65379.
        cmd.type=CommandType::Mode;
        switch(value) {
          case 0:cmd.mode=Mode::Standby;break; case 0x40:cmd.mode=Mode::Auto;break;
          case 1:cmd.mode=Mode::Wind;break; case 0x80:cmd.mode=Mode::Track;break;
          case 0x51:cmd.type=CommandType::Step;cmd.value=1;break;
          case 0x7f:cmd.type=CommandType::Step;cmd.value=-1;break;
          case 0xd1:cmd.type=CommandType::Step;cmd.value=10;break;
          case 0x50:cmd.type=CommandType::Step;cmd.value=-10;break;
          default:return false;
        }
      }
    }
  }
  if(pos!=n || !mfg || !industry || !action) return false;
  out=cmd; return true;
}
bool Controller::reject(const char *why) { ++rejected; result=why; return false; }
void Controller::cancel(const char *why) { if(busy()) ++failed; phase=Phase::Idle; result=why; }
void Controller::finish() { phase=Phase::Idle; result="confirmed"; ++completed; }
bool Controller::request(const Command &cmd, uint32_t now) {
  if(!config::controlEnabled) return reject("control disabled");
  if(cmd.type==CommandType::None) return reject("unsupported command");
  // Standby may preempt, even if status is missing. It is still not reported as
  // confirmed until fresh pilot feedback arrives. All other commands need it.
  bool standby=cmd.type==CommandType::Mode && cmd.mode==Mode::Standby;
  if(!standby && !pilot.alive(now)) return reject("pilot feedback missing");
  if(busy() && !standby) return reject("command already pending");
  if(cmd.type==CommandType::Mode) {
    if(cmd.mode==Mode::Unknown) return reject("unknown mode");
    if(cmd.mode==Mode::Track && (!nav.ready(now) || (pilot.mode!=Mode::Auto && pilot.mode!=Mode::Track))) return reject("TRACK needs AUTO/TRACK and fresh navigation");
    if(cmd.mode==Mode::Wind && !wind.ready(now)) return reject("WIND needs fresh apparent wind");
  } else if(pilot.mode!=Mode::Auto || !isfinite(pilot.target) || !isfinite(cmd.value)) return reject("heading changes need confirmed AUTO");
  if(busy()) cancel("preempted by STANDBY");
  command=cmd;
  if(command.type==CommandType::Step) {
    if(cmd.value!=1 && cmd.value!=-1 && cmd.value!=10 && cmd.value!=-10) return reject("invalid heading step");
    command.type=CommandType::Heading; command.value=wrap(pilot.target+cmd.value);
  }
  if(command.type==CommandType::Heading) command.value=wrap(command.value);
  if(cmd.type==CommandType::Mode && cmd.mode!=Mode::Track && pilot.alive(now) && cmd.mode==pilot.mode) { finish(); return true; }
  requestedAt=phaseAt=now; routeEpoch=nav.epoch; phase=Phase::Queued; result="queued";
  return true;
}
bool Controller::next(uint32_t now, Datagram &out) {
  tick(now); out={}; if(phase!=Phase::Queued) return false;
  if(command.type==CommandType::Mode) {
    expectedMode=command.mode;
    switch(command.mode) {
      case Mode::Standby:out=key(2);break;
      case Mode::Auto:out=key(1);break;
      case Mode::Wind:out=key(0x23);break;
      // Exactly one TRACK press per explicit request. Never automatically
      // repeat a potentially large waypoint-turn confirmation.
      case Mode::Track:out=key(3);break;
      default:cancel("unsupported mode");return false;
    }
  } else {
    const double delta=difference(command.value,pilot.target);
    if(fabs(delta)<=0.55) { finish(); return false; }
    const double step=(fabs(delta)>=9.5?10:1)*(delta<0?-1:1);
    expectedMode=Mode::Auto; previousHeading=pilot.target;
    expectedHeading=wrap(pilot.target+step);
    out=key(step==10?8:step==1?7:step==-10?6:5);
  }
  return true;
}
void Controller::started(uint32_t now) {
  if(phase!=Phase::Queued) return;
  beforeGeneration=pilot.generation; phase=Phase::Transmitting; phaseAt=now; result="awaiting wire echo";
}
void Controller::transmitted(bool echoOk, uint32_t now) {
  if(phase!=Phase::Transmitting) return;
  if(!echoOk) { cancel("SeaTalk transmission unconfirmed; not retried"); return; }
  phase=Phase::Confirming; phaseAt=now; result="awaiting pilot confirmation";
}
void Controller::standbyKey() {
  if(!(busy() && command.type==CommandType::Mode && command.mode==Mode::Standby)) cancel("physical STANDBY key");
}
void Controller::receivePilot(const uint8_t *p, size_t n, uint32_t now) {
  const Mode previousMode=pilot.mode;
  const double previousTarget=pilot.target;
  if(!pilot.receive(p,n,now)) return;
  if(!busy()) return;
  if(command.type==CommandType::Heading && phase==Phase::Queued &&
     (pilot.mode!=Mode::Auto || (isfinite(previousTarget) && fabs(difference(pilot.target,previousTarget))>0.55))) {
    cancel("pilot changed before next heading step"); return;
  }
  if(pilot.mode==Mode::Standby && previousMode!=Mode::Standby && !(command.type==CommandType::Mode && command.mode==Mode::Standby)) {
    cancel("physical STANDBY or pilot disengagement"); return;
  }
  if(phase!=Phase::Confirming || pilot.generation==beforeGeneration) return;
  if(command.type==CommandType::Mode) {
    if(pilot.mode==expectedMode) finish();
  } else {
    if(pilot.mode!=Mode::Auto) { cancel("pilot mode changed"); return; }
    if(fabs(difference(pilot.target,expectedHeading))<=0.55) {
      if(fabs(difference(command.value,pilot.target))<=0.55) finish();
      else { phase=Phase::Queued; phaseAt=now; }
    } else if(fabs(difference(pilot.target,previousHeading))>0.55) cancel("heading changed externally");
  }
}
void Controller::tick(uint32_t now) {
  if(!busy()) return;
  const bool standby=command.type==CommandType::Mode && command.mode==Mode::Standby;
  if(!standby && !pilot.alive(now)) { cancel("pilot feedback timed out"); return; }
  if(!fresh(now,requestedAt,config::commandTimeoutMs)) { cancel("command timed out"); return; }
  if(command.type==CommandType::Mode && command.mode==Mode::Track && (!nav.ready(now)||routeEpoch!=nav.epoch)) { cancel("navigation lost or waypoint changed"); return; }
  if(command.type==CommandType::Mode && command.mode==Mode::Wind && !wind.ready(now)) { cancel("wind data lost"); return; }
  if(phase==Phase::Queued && !fresh(now,phaseAt,config::queueTimeoutMs)) cancel("SeaTalk bus busy");
  else if(phase==Phase::Transmitting && !fresh(now,phaseAt,250)) cancel("SeaTalk transmitter timed out");
  else if(phase==Phase::Confirming && !fresh(now,phaseAt,config::confirmationTimeoutMs)) cancel("pilot did not confirm command");
}

void WireDecoder::reset() { packet={}; expected=0; character=0; bit=0; }
void WireDecoder::feed(bool high) {
  if(bit==0) { if(!high) { bit=1; character=0; } return; }
  if(bit<=9) { if(high) character|=1u<<(bit-1); ++bit; return; }
  bit=0;
  if(!high) { ++errors; reset(); return; }
  const uint8_t value=character;
  if(character&0x100) {
    if(packet.size) ++errors;
    packet={}; expected=0; packet.data[packet.size++]=value;
  } else if(packet.size) {
    if(packet.size>=sizeof(packet.data)) { ++errors; reset(); return; }
    packet.data[packet.size++]=value;
    if(packet.size==2) expected=3+(value&15);
    if(expected && packet.size==expected) {
      callback(context,packet.data,packet.size); packet={}; expected=0;
    }
  }
}
void WireDecoder::run(bool high, uint32_t us) {
  if(!us) return;
  // 4800 baud. Bound input and reject pulse widths far from integer bits.
  unsigned bits=(uint64_t(us)*4800+500000)/1000000;
  if(bits>200) { ++errors; reset(); return; }
  const double error=fabs(double(us)*4800/1000000-bits);
  if(!bits || error>0.35) { ++errors; reset(); return; }
  for(unsigned i=0;i<bits;++i) feed(high);
}
void WireDecoder::endCapture(bool idleHigh) {
  // RMT may omit the terminal idle run. Only supply a missing stop bit,
  // never missing data/command bits.
  if(bit==10 && idleHigh) feed(true);
  if(bit || packet.size) ++errors;
  reset();
}
}
