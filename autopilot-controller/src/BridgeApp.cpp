#include <Arduino.h>
#include <Preferences.h>
#include <stdarg.h>
#include <algorithm>
#include <N2kMessages.h>
#include <N2kGroupFunction.h>
#include "BridgeApp.h"
#include "BridgeCore.h"
#include "Config.h"
#include "NmeaBus.h"
#include "SeaTalkBus.h"

using namespace bridge;
static Controller controller;
static NmeaBus nmea;
static Preferences preferences;
static bool seaTalkReady=false, rawLog=false, canLog=false, nvsReady=false;
static Sample speedThroughWater, localVariation, externalVariation;
enum class TxKind { Other, Control, Nav, Waypoint, WindAngle, WindSpeed, WaterSpeed };
static TxKind txKind=TxKind::Other;
static uint32_t txEpoch=0, navEchoEpoch=0, announcedEpoch=0;
static bool navEchoValid=false, waypointDirty=false, addressDirty=false;
static uint8_t stwSource=255, variationSource=255;
static uint32_t publishAt=0, navAt=0, windAngleAt=0, windSpeedAt=0, stwAt=0, diagnosticsAt=0;
static uint32_t nmeaSendFailures=0, rejectedMessages=0, logDrops=0, lastRecovery=0;
static uint32_t canTraceDrops=0;
static const char *lastResult=nullptr;
static uint8_t sid=0;

// Bounded serial queue: even a full diagnostic report never waits on USB/UART.
static char logBuffer[2048];
static size_t logRead=0, logWrite=0;
static void queueLog(const char *text,size_t n) {
  const size_t free=(logRead+sizeof(logBuffer)-logWrite-1)%sizeof(logBuffer);
  if(n>free) { ++logDrops; return; }
  for(size_t i=0;i<n;++i) { logBuffer[logWrite]=text[i];logWrite=(logWrite+1)%sizeof(logBuffer); }
}
static void logf(const char *format,...) {
  char text[768];va_list args;va_start(args,format);
  int n=vsnprintf(text,sizeof(text),format,args);va_end(args);
  if(n<0 || size_t(n)>=sizeof(text)) { ++logDrops; return; }
  queueLog(text,n);
}
static void flushLog() {
  if(logRead==logWrite) return;
  size_t n=logWrite>logRead?logWrite-logRead:sizeof(logBuffer)-logRead;
  n=std::min(n,size_t(std::max(0,Serial.availableForWrite())));
  if(n) { Serial.write(reinterpret_cast<uint8_t *>(logBuffer+logRead),n);logRead=(logRead+n)%sizeof(logBuffer); }
}
static void logPacket(const char *direction,const uint8_t *p,size_t n) {
  if(!rawLog) return;
  char line[100]; int used=snprintf(line,sizeof(line),"[ST %s]",direction);
  for(size_t i=0;i<n && used<int(sizeof(line)-5);++i) used+=snprintf(line+used,sizeof(line)-used," %02X",p[i]);
  line[used++]='\n';
  queueLog(line,used);
}
static void logCanFrame(const char *direction,unsigned long id,unsigned char n,const unsigned char *p) {
  if(!canLog) return;
  const uint8_t pf=(id>>16)&255;
  const uint32_t pgn=((id>>8)&0x1ffff)&(pf<240?0x1ff00:0x1ffff);
  switch(pgn) {
    case 59392: case 59904: case 60928: case 126208: case 126464:
    case 126720: case 126996: case 65359: case 65360: case 65379:
    case 127237: case 127245: case 127250: case 127258: case 128259:
    case 129283: case 129284: case 130306: break;
    default:return;
  }
  static uint32_t window=0; static unsigned frames=0;
  const uint32_t now=millis();
  if(uint32_t(now-window)>=1000) { window=now; frames=0; }
  if(frames++>=60) { ++canTraceDrops; return; }
  // Nonblocking and bounded. TX_QUEUE means accepted by the local CAN driver,
  // not acknowledged or consumed by OpenCPN. Fast-packet bytes remain intact.
  char line[160];
  int used=snprintf(line,sizeof(line),"[CAN %s] ms=%lu id=%08lX pgn=%lu src=%u dst=%u data=",
    direction,(unsigned long)now,id,(unsigned long)pgn,unsigned(id&255),pf<240?unsigned((id>>8)&255):255u);
  if(used<0 || used>=int(sizeof(line)-26)) { ++canTraceDrops; return; }
  for(unsigned i=0;i<n && i<8;++i) used+=snprintf(line+used,sizeof(line)-used,"%s%02X",i?" ":"",p[i]);
  line[used++]='\n';queueLog(line,used);
}
static void onSeaTalk(const uint8_t *p,size_t n) {
  logPacket("RX",p,n);
  controller.receivePilot(p,n,millis());
  // A physical key cancels work but never substitutes for confirmed mode.
  if(n==4 && p[0]==0x86 && (p[1]&15)==1 && uint8_t(p[2]^p[3])==0xff) {
    if(p[2]==2) controller.standbyKey();
  }
  // SeaTalk variation is positive west; NMEA variation is positive east.
  if(n==3 && p[0]==0x99 && p[1]==0 && abs(int(static_cast<int8_t>(p[2])))<=90)
    localVariation.set(-double(static_cast<int8_t>(p[2])),millis());
  if(n==4 && p[0]==0x10 && p[1]==1) {
    const uint16_t a=uint16_t(p[2])<<8|p[3];
    if(a<=720) controller.wind.localAngle.set(wrap(a*0.5),millis());
  }
}
static void onSeaTalkComplete(bool ok,bool control) {
  const uint32_t now=millis();
  if(control) controller.transmitted(ok,now);
  if(ok) {
    switch(txKind) {
      case TxKind::Nav:navAt=now;navEchoEpoch=txEpoch;navEchoValid=true;break;
      case TxKind::Waypoint:announcedEpoch=txEpoch;break;
      case TxKind::WindAngle:windAngleAt=now;break;
      case TxKind::WindSpeed:windSpeedAt=now;break;
      case TxKind::WaterSpeed:stwAt=now;break;
      default:break;
    }
  }
  txKind=TxKind::Other;
}
static SeaTalkBus seatalk(onSeaTalk,onSeaTalkComplete);
static void logSeaTalkTrace(const char *reason,const Datagram &packet,uint32_t elapsed,uint32_t levels,uint32_t gap) {
  if(!rawLog) return;
  logf("[ST RESULT] ms=%lu reason=%s cmd=%02X elapsed_us=%lu monitor_levels=%lu max_sample_gap_us=%lu\n",
    (unsigned long)millis(),reason,packet.data[0],(unsigned long)elapsed,(unsigned long)levels,(unsigned long)gap);
  logPacket("DETAIL",packet.data,packet.size);
}
static void logSeaTalkCapture(const rmt_item32_t *items,size_t n,uint32_t errors) {
  if(!rawLog) return;
  logf("[ST CAPTURE] ms=%lu errors=%lu items=%u (level:microseconds)\n",
    (unsigned long)millis(),(unsigned long)errors,unsigned(n));
  // Bound diagnostic output even if a noisy wire fills the RMT memory.
  for(size_t start=0;start<n && start<48;start+=16) {
    char line[384]; int used=snprintf(line,sizeof(line),"[ST PULSES]");
    for(size_t i=start;i<n && i<start+16;++i)
      used+=snprintf(line+used,sizeof(line)-used," %u:%u %u:%u",unsigned(items[i].level0),unsigned(items[i].duration0),unsigned(items[i].level1),unsigned(items[i].duration1));
    line[used++]='\n'; queueLog(line,used);
  }
}
static bool transmit(const Datagram &packet,TxKind kind) {
  if(!packet.size) return false;
  txKind=kind; txEpoch=controller.nav.epoch;
  if(!seatalk.send(packet,kind==TxKind::Control)) { txKind=TxKind::Other; return false; }
  logPacket("TX",packet.data,packet.size); return true;
}

static double angleRadians(double value) { return isfinite(value)?value*pi/180:N2kDoubleNA; }
static bool sendMessage(const tN2kMsg &msg) {
  if(!nmea.healthy() || !nmea.SendMsg(msg)) { ++nmeaSendFailures; return false; }
  return true;
}
static uint16_t rmMode(Mode m) {
  switch(m) { case Mode::Standby:return 0;case Mode::Auto:return 0x40;case Mode::Wind:return 0x100;case Mode::Track:return 0x180;default:return 0xffff; }
}
static bool publishPGN(unsigned long pgn) {
  const uint32_t now=millis(); const Pilot &p=controller.pilot;
  const bool live=p.alive(now); const Mode mode=live?p.mode:Mode::Unknown;
  tN2kMsg msg;
  if(pgn==126720 && config::autoTrackCompatibility) {
    const auto data=evolutionStatus(p,now);
    if(!data.size) return false; // Never synthesize a pilot mode after feedback loss.
    msg.SetPGN(pgn);msg.Priority=3;
    for(unsigned i=0;i<data.size;++i) msg.AddByte(data.data[i]);
  } else if(pgn==127250) {
    double variation=controller.nav.variation.alive(now,config::variationTimeoutMs)?angleRadians(controller.nav.variation.value):N2kDoubleNA;
    SetN2kPGN127250(msg,sid,live?angleRadians(p.heading):N2kDoubleNA,N2kDoubleNA,variation,N2khr_magnetic);
  } else if(pgn==127237) {
    // Standard PGN has no dedicated WIND code; proprietary PGN has exact mode.
    auto steering=static_cast<tN2kSteeringMode>(mode==Mode::Standby?0:mode==Mode::Auto?4:mode==Mode::Track?5:7);
    SetN2kPGN127237(msg,N2kOnOff_Unavailable,
      live?(p.offCourse?N2kOnOff_On:N2kOnOff_Off):N2kOnOff_Unavailable,
      N2kOnOff_Unavailable,N2kOnOff_Unavailable,steering,N2kTM_Unavailable,N2khr_magnetic,
      N2kRDO_Unavailable,N2kDoubleNA,live?angleRadians(p.target):N2kDoubleNA,
      N2kDoubleNA,N2kDoubleNA,N2kDoubleNA,N2kDoubleNA,N2kDoubleNA,N2kDoubleNA,live?angleRadians(p.heading):N2kDoubleNA);
  } else if(pgn==127245 && config::publishRudder) {
    SetN2kPGN127245(msg,live?angleRadians(p.rudder):N2kDoubleNA);
  } else if(pgn==65379 || pgn==65360 || (pgn==65359 && config::autoTrackCompatibility)) {
    if(pgn==65360 && !publishLockedHeading(p,now,config::autoTrackCompatibility)) return false;
    if(pgn==65359 && !live) return false; // Plugin's discovery consumer ignores NA.
    msg.SetPGN(pgn); msg.Priority=3;
    msg.Add2ByteUInt(config::manufacturer|(3u<<11)|(4u<<13));
    if(pgn==65379) {
      msg.Add2ByteUInt(rmMode(mode)); msg.Add2ByteUInt(0xffff);
      msg.AddByte(mode==Mode::Standby?0:mode==Mode::Auto?0x40:mode==Mode::Wind?1:mode==Mode::Track?0x80:0xff);
      msg.AddByte(0xff);
    } else {
      msg.AddByte(sid);
      double magnetic=live?(pgn==65359?p.heading:p.target):NAN, trueHeading=NAN;
      if(isfinite(magnetic) && controller.nav.variation.alive(now,config::variationTimeoutMs)) trueHeading=wrap(magnetic+controller.nav.variation.value);
      msg.Add2ByteUDouble(angleRadians(trueHeading),0.0001); msg.Add2ByteUDouble(angleRadians(magnetic),0.0001); msg.AddByte(0xff);
    }
  } else return false;
  return sendMessage(msg);
}

class PilotGroupHandler : public tN2kGroupFunctionHandler {
 public:
  explicit PilotGroupHandler(unsigned long pgn):tN2kGroupFunctionHandler(&nmea,pgn) {}
 protected:
  bool HandleCommand(const tN2kMsg &msg,uint8_t,uint8_t count,int device) override {
    Command cmd;
    bool addressed=msg.Destination==nmea.GetN2kSource(device) && msg.Source<=251 && msg.Source!=nmea.GetN2kSource();
    bool authorized=config::controllerSource==255 || msg.Source==config::controllerSource;
    bool parsed=addressed && authorized && parseCommand(msg.Data,msg.DataLen,cmd);
    if(parsed && cmd.type==CommandType::Mode && cmd.mode==Mode::Standby) seatalk.abort();
    bool accepted=parsed && !seatalk.listeningOnly() && seaTalkReady && nmea.healthy() && controller.request(cmd,millis());
    if(canLog) logf("[N2K COMMAND] src=%u dst=%u target=%lu parsed=%u accepted=%u listen-only=%u\n",
      unsigned(msg.Source),unsigned(msg.Destination),PGN,unsigned(parsed),unsigned(accepted),unsigned(seatalk.listeningOnly()));
    if(!accepted) ++rejectedMessages;
    SendAcknowledge(&nmea,msg.Source,device,PGN,
      accepted?N2kgfPGNec_Acknowledge:(parsed?N2kgfPGNec_PGNTemporarilyNotAvailable:N2kgfPGNec_RequestOrCommandNotSupported),
      N2kgfTPec_Acknowledge,count,
      accepted?N2kgfpec_Acknowledge:(parsed?N2kgfpec_TemporarilyUnableToComply:N2kgfpec_InvalidRequestOrCommandParameterField));
    return true;
  }
  bool HandleRequest(const tN2kMsg &msg,uint32_t interval,uint16_t offset,uint8_t count,int device) override {
    bool valid=msg.DataLen>=11; size_t pos=11; uint8_t seen=0;
    for(unsigned i=0;valid && i<count;++i) {
      if(pos>=size_t(msg.DataLen)) { valid=false; break; }
      uint8_t field=msg.Data[pos++];
      if(field==1 && !(seen&1) && pos+2<=size_t(msg.DataLen)) { valid=(u16(msg.Data+pos)&0x7ff)==config::manufacturer; pos+=2;seen|=1; }
      else if(field==3 && !(seen&2) && pos<size_t(msg.DataLen)) { valid=msg.Data[pos++]==4;seen|=2; }
      else valid=false;
    }
    valid=valid && pos==size_t(msg.DataLen);
    const bool timing=(interval==0 || interval==0xffffffff || interval==0xfffffffe) && (offset==0 || offset==0xffff);
    if(valid && timing) publishPGN(PGN);
    else SendAcknowledge(&nmea,msg.Source,device,PGN,
      valid?N2kgfPGNec_Acknowledge:N2kgfPGNec_RequestOrCommandNotSupported,
      timing?N2kgfTPec_Acknowledge:N2kgfTPec_TransmitIntervalOrPriorityNotSupported,
      count,valid?N2kgfpec_Acknowledge:N2kgfpec_InvalidRequestOrCommandParameterField);
    return true;
  }
};
static PilotGroupHandler modeHandler(65379), headingHandler(65360);
static bool isoRequest(unsigned long pgn,unsigned char,int) { return publishPGN(pgn); }
static void onNmea(const tN2kMsg &msg) {
  if(msg.Source==nmea.GetN2kSource() || msg.Source>251 || !nmea.healthy()) return;
  const auto *p=msg.Data; const auto n=msg.DataLen; const auto now=millis();
  switch(msg.PGN) {
    case 129283: case 129284: {
      const uint32_t before=controller.nav.epoch;
      controller.nav.receive(msg.PGN,msg.Source,p,n,now);
      if(before!=controller.nav.epoch) waypointDirty=true;
      break;
    }
    case 130306: controller.wind.receive(msg.Source,p,n,now); break;
    case 127258:
      if(n==8 && (variationSource==255 || variationSource==msg.Source)) {
        int raw=static_cast<int16_t>(u16(p+4));
        if(abs(raw)<=15708) { variationSource=msg.Source; externalVariation.set(raw*0.0001*180/pi,now); }
        else if(variationSource==msg.Source) externalVariation.clear();
      }
      break;
    case 128259:
      if(n==8 && (stwSource==255 || stwSource==msg.Source)) {
        uint16_t raw=u16(p+1);
        if(raw<0xfffd) { stwSource=msg.Source; speedThroughWater.set(raw*0.01*1.943844492,now); }
        else speedThroughWater.clear();
      }
      break;
    default:break;
  }
}
static void diagnostics() {
  const uint32_t now=millis();
  logf("SeaTalk interface: %s; TX=%d RX=%d monitor=%d (leave unconnected)\n",
    config::txOpenDrain?"shared MOSFET, open drain":"separate inverting transmitter",
    config::seaTalkTx,config::seaTalkRx,config::seaTalkTxMonitor);
  logf("SeaTalk listen-only: %s\n",seatalk.listeningOnly()?"ON (transmission disabled)":"OFF");
  logf("Remote control: %s; startup SeaTalk transmission: %s; rudder publication: %s\n",
    config::controlEnabled?"ON":"OFF",config::seaTalkListenOnly?"OFF":"ON",config::publishRudder?"ON":"OFF");
  logf("CAN trace: %s; trace rate-limit drops %lu\n",canLog?"ON":"OFF",(unsigned long)canTraceDrops);
  logf("AutoTrack compatibility: %s; expected address %u\n",config::autoTrackCompatibility?"ON":"OFF",unsigned(config::preferredAddress));
  logf("\nPilot: %s; heading %.1f; target %.1f; feedback age %lu ms\n",
    modeName(controller.pilot.alive(now)?controller.pilot.mode:Mode::Unknown),
    controller.pilot.alive(now)?controller.pilot.heading:NAN,controller.pilot.alive(now)?controller.pilot.target:NAN,
    controller.pilot.valid?static_cast<unsigned long>(now-controller.pilot.at):0ul);
  logf("Command: %s; confirmed %lu; failed %lu; rejected %lu\n",controller.result,
    (unsigned long)controller.completed,(unsigned long)controller.failed,(unsigned long)rejectedMessages);
  logf("Pilot alarms: off-course=%s wind-shift=%s; invalid CAN fragments=%lu\n",
    !controller.pilot.alive(now)?"unknown":controller.pilot.offCourse?"yes":"no",
    !controller.pilot.alive(now)?"unknown":controller.pilot.windShift?"yes":"no",(unsigned long)nmea.invalidFrames);
  logf("Navigation: %s (source %u); wind: %s (source %u); CAN address %u\n",
    controller.nav.ready(now)?"fresh":"unavailable",controller.nav.source,
    controller.wind.ready(now)?"fresh":"unavailable",controller.wind.source,nmea.GetN2kSource());
  logf("SeaTalk RX %lu TX %lu framing %lu collisions %lu echo failures %lu; CAN bus-off %lu recoveries %lu RX-drop alerts %lu TX-fail alerts %lu send failures %lu; log drops %lu\n",
    (unsigned long)seatalk.received,(unsigned long)seatalk.sent,(unsigned long)seatalk.framingErrors(),
    (unsigned long)seatalk.collisions,(unsigned long)seatalk.echoFailures,(unsigned long)nmea.busOffCount,
    (unsigned long)nmea.recoveries,(unsigned long)nmea.receiveDrops,(unsigned long)nmea.transmitFailures,
    (unsigned long)nmeaSendFailures,(unsigned long)logDrops);
}
static void console() {
  static char command[32]; static unsigned used=0;
  for(unsigned budget=0;budget<32 && Serial.available();++budget) {
    char c=Serial.read(); if(c=='\r') continue;
    if(c=='\n') {
      command[used]=0; used=0;
      if(!strcmp(command,"status")) diagnostics();
      else if(!strcmp(command,"raw on")) rawLog=true;
      else if(!strcmp(command,"raw off")) rawLog=false;
      else if(!strcmp(command,"can on")) canLog=true;
      else if(!strcmp(command,"can off")) canLog=false;
      else if(!strcmp(command,"listen on")) {
        seatalk.setListenOnly(true);
        if(controller.busy()) controller.cancel("listen-only enabled");
        navEchoValid=false;
        logf("SeaTalk listen-only ON: all SeaTalk transmission disabled\n");
      } else if(!strcmp(command,"listen off")) {
        seatalk.setListenOnly(false);
        logf("SeaTalk listen-only OFF: transmission enabled\n");
      } else logf("Commands: status | raw on | raw off | can on | can off | listen on | listen off | help\n");
    } else if(used<sizeof(command)-1) command[used++]=c;
  }
}
void bridgeSetup() {
  Serial.begin(115200);
  Serial.println("\nSeaTalk bridge 2.0: waiting for measured pilot state");
  seatalk.setListenOnly(config::seaTalkListenOnly);
  nmea.setTrace(logCanFrame);
  seatalk.setTrace(logSeaTalkTrace,logSeaTalkCapture);
  seaTalkReady=seatalk.begin();
  if(!seaTalkReady) Serial.println("ERROR: SeaTalk hardware initialization failed; control disabled");
  nvsReady=preferences.begin("stbridge",false);
  if(nvsReady) controller.nav.epoch=preferences.getULong("wpseq",0);
  uint8_t address=nvsReady?preferences.getUChar("address",config::preferredAddress):config::preferredAddress;
  // AutoTrack 2.3 hard-codes destination 204. Start by claiming that address;
  // normal NMEA address arbitration still applies if another device owns it.
  if(config::autoTrackCompatibility) address=config::preferredAddress;
  if(address>251) address=config::preferredAddress;
  const uint64_t mac=ESP.getEfuseMac();
  char serial[24]; snprintf(serial,sizeof(serial),"ST-%012llX",(unsigned long long)mac);
  const uint32_t unique=uint32_t(mac^(mac>>21)^(mac>>42))&0x1fffff;
  nmea.SetN2kCANMsgBufSize(24); nmea.SetN2kCANSendFrameBufSize(80); nmea.SetN2kCANReceiveFrameBufSize(160);
  nmea.SetProductInformation(serial,20001,"SeaTalk AP Bridge","2.0.0","ESP32 ST bridge",3,2101,0);
  nmea.SetDeviceInformation(unique,135,40,config::manufacturer,4);
  nmea.SetConfigurationInformation("Independent SeaTalk bridge; Raymarine compatible","ESP32 CAN5/4 SeaTalk17/16","Feedback-confirmed control");
  nmea.SetMode(tNMEA2000::N2km_ListenAndNode,address); nmea.EnableForward(false);
  static unsigned long tx[9]={127237,127250,65360,65379};
  unsigned txCount=4;
  if(config::autoTrackCompatibility) { tx[txCount++]=65359;tx[txCount++]=126720; }
  if(config::publishRudder) tx[txCount++]=127245;
  tx[txCount]=0;
  static const unsigned long rx[]={126208,127258,128259,129283,129284,130306,0};
  static const unsigned long rm[]={65359,65360,65379,0};
  nmea.ExtendTransmitMessages(tx); nmea.ExtendReceiveMessages(rx); nmea.ExtendSingleFrameMessages(rm);
  nmea.SetHeartbeatIntervalAndOffset(2000,0);
  nmea.AddGroupFunctionHandler(&modeHandler); nmea.AddGroupFunctionHandler(&headingHandler);
  nmea.SetMsgHandler(onNmea); nmea.SetISORqstHandler(isoRequest);
  nmea.SetOnOpen([]() { nmea.SetHeartbeatIntervalAndOffset(2000,0); });
  nmea.Open(); // Opening and address claiming progress asynchronously in loop().
}
void bridgeLoop() {
  nmea.service();
  if(!nmea.healthy()) {
    controller.cancel("CAN unavailable"); seatalk.abort();
    controller.nav.xte.clear(); controller.nav.bearing.clear(); controller.nav.distance.clear();
    controller.wind.angle.clear(); controller.wind.speed.clear(); speedThroughWater.clear();
  }
  if(nmea.recoveries!=lastRecovery) { lastRecovery=nmea.recoveries; nmea.SendIsoAddressClaim(); }
  seatalk.poll(); nmea.ParseMessages(); seatalk.poll();
  const uint32_t now=millis(); controller.tick(now);
  controller.nav.variation=localVariation.alive(now,config::variationTimeoutMs)?localVariation:externalVariation;
  if(nmea.ReadResetAddressChanged()) addressDirty=true;
  if(!seatalk.busy() && nvsReady) {
    if(addressDirty) { preferences.putUChar("address",nmea.GetN2kSource());addressDirty=false; }
    if(waypointDirty) { preferences.putULong("wpseq",controller.nav.epoch);waypointDirty=false; }
  }
  if(!seatalk.listeningOnly() && !seatalk.busy() && seaTalkReady && nmea.healthy()) {
    Datagram packet; const bool control=controller.next(now,packet);
    const bool navFresh=controller.nav.ready(now);
    const bool needNav=navFresh && (!navEchoValid || navEchoEpoch!=controller.nav.epoch || uint32_t(now-navAt)>=1000);
    const bool needWaypoint=navFresh && navEchoValid && navEchoEpoch==controller.nav.epoch && announcedEpoch!=navEchoEpoch;
    const bool track=control && packet.data[2]==3;
    const bool wind=control && packet.data[2]==0x23;
    // The waypoint token follows an echoed NAV packet. An explicit TRACK key
    // waits for both; the bridge never auto-confirms the pilot's turn prompt.
    if(track && needNav) transmit(navigation(controller.nav,now),TxKind::Nav);
    else if(track && needWaypoint) transmit(waypointToken(controller.nav.epoch),TxKind::Waypoint);
    else if(wind && controller.wind.nmeaReady(now) && uint32_t(now-windAngleAt)>=750)
      transmit(windAngle(controller.wind.angle.value),TxKind::WindAngle);
    else if(control) {
      if(transmit(packet,TxKind::Control)) controller.started(now);
    } else if(needWaypoint) transmit(waypointToken(controller.nav.epoch),TxKind::Waypoint);
    else if(needNav) transmit(navigation(controller.nav,now),TxKind::Nav);
    else if(controller.wind.nmeaReady(now) && uint32_t(now-windAngleAt)>=1000)
      transmit(windAngle(controller.wind.angle.value),TxKind::WindAngle);
    else if(controller.wind.nmeaReady(now) && uint32_t(now-windSpeedAt)>=1000)
      transmit(windSpeed(controller.wind.speed.value),TxKind::WindSpeed);
    else if(speedThroughWater.alive(now,3000) && uint32_t(now-stwAt)>=1000)
      transmit(waterSpeed(speedThroughWater.value),TxKind::WaterSpeed);
  }
  if(uint32_t(now-publishAt)>=1000 && nmea.healthy()) {
    publishPGN(127250); publishPGN(127237); publishPGN(65379); publishPGN(65360);
    if(config::autoTrackCompatibility) { publishPGN(65359);publishPGN(126720); }
    if(config::publishRudder) publishPGN(127245);
    sid=(sid+1)%253; publishAt=now;
  }
  if(controller.result!=lastResult) {
    logf("[CONTROL] %s\n",controller.result); lastResult=controller.result;
  }
  console();
  if(uint32_t(now-diagnosticsAt)>=5000) {
    diagnosticsAt=now;
    logf("[HEALTH] pilot=%s nav=%s CAN=%s\n",
      controller.pilot.alive(now)?modeName(controller.pilot.mode):"UNKNOWN",
      controller.nav.ready(now)?"fresh":"unavailable",nmea.healthy()?"up":"down");
  }
  flushLog();
  delay(1);
}
