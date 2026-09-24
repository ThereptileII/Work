#pragma once
#include <NMEA2000.h>
#include <driver/twai.h>
#include "BridgeCore.h"

// NMEA2000 handles transport, claims, requests and heartbeat. This small adapter
// uses Espressif's TWAI driver, including its documented bus-off recovery.
class NmeaBus : public tNMEA2000 {
 public:
  using Trace=void (*)(const char *,unsigned long,unsigned char,const unsigned char *);
  void setTrace(Trace callback) { trace=callback; }
  void service();
  bool healthy() const { return running; }
  uint32_t busOffCount=0, receiveDrops=0, transmitFailures=0, recoveries=0, invalidFrames=0;
 protected:
  bool CANOpen() override;
  bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *data, bool wait=true) override;
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *data) override;
 private:
  Trace trace=nullptr;
  bool installed=false, running=false;
  bridge::FastPacketGuard guard;
};
