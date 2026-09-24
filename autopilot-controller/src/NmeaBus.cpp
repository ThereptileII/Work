#include "NmeaBus.h"
#include "Config.h"
#include <string.h>
#include <Arduino.h>

bool NmeaBus::CANOpen() {
  if(installed) return running;
  twai_general_config_t g=TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(config::canTx),gpio_num_t(config::canRx),TWAI_MODE_NORMAL);
  twai_timing_config_t t=TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f=TWAI_FILTER_CONFIG_ACCEPT_ALL();
  g.rx_queue_len=160; g.tx_queue_len=48;
  g.alerts_enabled=TWAI_ALERT_BUS_OFF|TWAI_ALERT_BUS_RECOVERED|TWAI_ALERT_RX_QUEUE_FULL|TWAI_ALERT_TX_FAILED;
  if(twai_driver_install(&g,&t,&f)!=ESP_OK) return false;
  installed=true;
  running=twai_start()==ESP_OK;
  return running;
}
bool NmeaBus::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *data, bool) {
  if(!running || !data || len>8) return false;
  twai_message_t msg={}; msg.identifier=id; msg.extd=1; msg.data_length_code=len;
  memcpy(msg.data,data,len);
  const bool queued=twai_transmit(&msg,0)==ESP_OK;
  if(trace) trace(queued?"TX_QUEUE":"TX_FAIL",id,len,data);
  return queued;
}
bool NmeaBus::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *data) {
  if(!running) return false;
  twai_message_t msg;
  // Bound work on unrelated standard-ID/RTR traffic.
  for(unsigned i=0;i<32;++i) {
    if(twai_receive(&msg,0)!=ESP_OK) return false;
    if(!msg.extd || msg.rtr || msg.data_length_code==0 || msg.data_length_code>8) continue;
    // Trace before destination/fragment filtering: wrong-address commands are
    // useful evidence during gateway/plugin commissioning.
    if(trace) trace("RX",msg.identifier,msg.data_length_code,msg.data);
    const uint8_t pf=(msg.identifier>>16)&255;
    const uint32_t pgn=((msg.identifier>>8)&0x1ffff)&(pf<240?0x1ff00:0x1ffff);
    const uint8_t destination=pf<240?(msg.identifier>>8)&255:255;
    if(pgn==126208 && destination!=255 && destination!=GetN2kSource()) continue;
    if((pgn==126208 || pgn==129284) && !guard.accept(pgn,msg.identifier&255,destination,msg.data,msg.data_length_code,millis())) { ++invalidFrames;continue; }
    id=msg.identifier; len=msg.data_length_code; memcpy(data,msg.data,len); return true;
  }
  return false;
}
void NmeaBus::service() {
  if(!installed) return;
  uint32_t alerts=0;
  twai_read_alerts(&alerts,0);
  if(alerts&TWAI_ALERT_RX_QUEUE_FULL) ++receiveDrops;
  if(alerts&TWAI_ALERT_TX_FAILED) ++transmitFailures;
  if(alerts&TWAI_ALERT_BUS_OFF) {
    running=false; ++busOffCount;
    guard.clear();
    // Drop queued application/system frames instead of replaying stale state.
    CANSendFrameBufferRead=CANSendFrameBufferWrite;
    twai_clear_receive_queue();
    if(twai_initiate_recovery()!=ESP_OK) ++transmitFailures;
  }
  if(alerts&TWAI_ALERT_BUS_RECOVERED) {
    running=twai_start()==ESP_OK;
    if(running) ++recoveries;
  }
  // Retry recovery/start after a transient API failure.
  if(!running) {
    twai_status_info_t state;
    if(twai_get_status_info(&state)==ESP_OK) {
      if(state.state==TWAI_STATE_BUS_OFF) twai_initiate_recovery();
      else if(state.state==TWAI_STATE_STOPPED) { running=twai_start()==ESP_OK; if(running) ++recoveries; }
    }
  }
}
