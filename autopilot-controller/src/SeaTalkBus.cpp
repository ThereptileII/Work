#include "SeaTalkBus.h"
#include "Config.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <string.h>

bool SeaTalkBus::idle() const { return bool(gpio_get_level(gpio_num_t(config::seaTalkRx)))!=config::rxInverted; }
void IRAM_ATTR SeaTalkBus::edge(void *ctx) { static_cast<SeaTalkBus *>(ctx)->lastEdgeUs=uint32_t(esp_timer_get_time()); }
bool SeaTalkBus::begin() {
  // Preload the release level before enabling output. gpio_config enables OD
  // before enabling output, so a shared LV node is never driven push-pull HIGH.
  gpio_config_t output={};
  output.pin_bit_mask=1ULL<<config::seaTalkTx;
  output.mode=GPIO_MODE_INPUT;
  if(gpio_config(&output)!=ESP_OK ||
     gpio_set_level(gpio_num_t(config::seaTalkTx),!config::txInverted)!=ESP_OK) return false;
  esp_rom_gpio_connect_out_signal(config::seaTalkTx,SIG_GPIO_OUT_IDX,false,false);
  output.mode=config::txOpenDrain?GPIO_MODE_INPUT_OUTPUT_OD:GPIO_MODE_INPUT_OUTPUT;
  if(gpio_config(&output)!=ESP_OK) return false;
  rmt_config_t rx=RMT_DEFAULT_CONFIG_RX(gpio_num_t(config::seaTalkRx),rxChannel);
  rx.clk_div=80; rx.mem_block_num=4; // 1 us ticks, whole longest datagram fits.
  rx.rx_config.filter_en=true; rx.rx_config.filter_ticks_thresh=80; // 1 us APB filter
  rx.rx_config.idle_threshold=2500;
  if(rmt_config(&rx)!=ESP_OK || rmt_driver_install(rxChannel,8192,0)!=ESP_OK) return false;
  if(rmt_get_ringbuf_handle(rxChannel,&ring)!=ESP_OK) return false;
  // RMT config selects push-pull output: configure only the unconnected monitor
  // pad with it, then fan out the waveform to our explicitly configured TX pad.
  rmt_config_t tx=RMT_DEFAULT_CONFIG_TX(gpio_num_t(config::seaTalkTxMonitor),txChannel);
  tx.clk_div=80; tx.mem_block_num=2;
  tx.tx_config.idle_output_en=true;
  tx.tx_config.idle_level=config::txInverted?RMT_IDLE_LEVEL_LOW:RMT_IDLE_LEVEL_HIGH;
  if(rmt_config(&tx)!=ESP_OK || rmt_driver_install(txChannel,0,0)!=ESP_OK) return false;
  if(gpio_set_direction(gpio_num_t(config::seaTalkTxMonitor),GPIO_MODE_INPUT_OUTPUT)!=ESP_OK) return false;
  // IDF 4.4 gpio_set_direction enables output by routing SIG_GPIO_OUT_IDX.
  // Restore the RMT waveform after enabling input readback on the monitor pad.
  esp_rom_gpio_connect_out_signal(config::seaTalkTxMonitor,
                                 RMT_SIG_OUT0_IDX+int(txChannel),false,false);
  pinMode(config::seaTalkRx,INPUT);
  attachInterruptArg(config::seaTalkRx,edge,this,CHANGE);
  esp_timer_create_args_t args={}; args.callback=monitorWire; args.arg=this; args.name="seatalk_echo";
  if(esp_timer_create(&args,&monitor)!=ESP_OK) return false;
  if(rmt_rx_start(rxChannel,true)!=ESP_OK) return false;
  esp_rom_gpio_connect_out_signal(config::seaTalkTx,
                                 RMT_SIG_OUT0_IDX+int(txChannel),false,false);
  lastEdgeUs=micros(); initialized=true; return true;
}
void SeaTalkBus::monitorWire(void *ctx) {
  auto &bus=*static_cast<SeaTalkBus *>(ctx);
  // Hardware emits bits; this timer only detects sustained bus disagreement.
  // Allow interface propagation delay. Full echo validation is also mandatory.
  // Another talker pulls both shared LV pins low. Read the unconnected waveform
  // copy to retain the intended level and detect that collision.
  const bool tx=bool(gpio_get_level(gpio_num_t(config::seaTalkTxMonitor)))!=config::txInverted;
  const bool rx=bus.idle();
  const uint32_t now=uint32_t(esp_timer_get_time());
  const uint32_t previous=bus.monitorAt.exchange(now);
  if(previous && uint32_t(now-previous)>bus.monitorGapUs.load()) bus.monitorGapUs=now-previous;
  const uint32_t since=bus.mismatchAt.load();
  if(tx==rx) bus.mismatchAt=0;
  else if(!since) bus.mismatchAt=now;
  else if(uint32_t(now-since)>=60 && !bus.mismatch.exchange(true)) {
    bus.mismatchLevels=(uint32_t(tx)<<1)|uint32_t(rx);
    rmt_tx_stop(txChannel);
  }
}
bool SeaTalkBus::send(const bridge::Datagram &d, bool control) {
  if(!initialized || listenOnly || awaiting || d.size<3 || d.size>18 || (d.data[1]&15)+3!=d.size) return false;
  const uint32_t now=micros();
  if(!idle() || uint32_t(now-lastEdgeUs)<backoffUs) return false;
  // RX remains running throughout transmission to capture our own echo.
  memset(items,0,sizeof(items));
  unsigned bitIndex=0;
  for(unsigned c=0;c<d.size;++c) {
    uint16_t frame=(uint16_t(d.data[c])<<1)|(c==0?0x200:0)|0x400;
    for(unsigned b=0;b<11;++b,++bitIndex) {
      bool level=bool(frame&(1u<<b))!=config::txInverted;
      unsigned duration=((bitIndex+1)*1000000u+2400)/4800-(bitIndex*1000000u+2400)/4800;
      auto &item=items[bitIndex/2];
      if(bitIndex&1) { item.level1=level; item.duration1=duration; }
      else { item.level0=level; item.duration0=duration; }
    }
  }
  outbound=d; controlPacket=control; awaiting=true; txDone=false;
  mismatch=false; mismatchAt=0; startedUs=now;
  monitorAt=0; monitorGapUs=0; mismatchLevels=0; failureReason="wire-mismatch";
  // A final idle half-item avoids a zero-duration half terminating before stop.
  if(bitIndex&1) { items[bitIndex/2].level1=!config::txInverted; items[bitIndex/2].duration1=208; ++bitIndex; }
  // Defer failures until poll(), so the caller can first mark the command as
  // transmitted. Callbacks must never run synchronously inside send().
  if(esp_timer_start_periodic(monitor,50)!=ESP_OK) { failureReason="timer-start"; mismatch=true; }
  else if(rmt_write_items(txChannel,items,bitIndex/2,false)!=ESP_OK) { failureReason="rmt-start"; mismatch=true; }
  return true;
}
void SeaTalkBus::finish(bool ok) {
  if(!awaiting) return;
  esp_timer_stop(monitor);
  if(!ok) rmt_tx_stop(txChannel);
  const bool wasControl=controlPacket;
  awaiting=false; txDone=false;
  if(ok) ++sent; else ++echoFailures;
  if(trace && (!ok || wasControl)) trace(ok?"echo-ok":failureReason,outbound,
    uint32_t(micros()-startedUs),mismatchLevels.load(),monitorGapUs.load());
  backoffUs=(ok?3000:250000)+(esp_random()%4000);
  lastEdgeUs=micros();
  complete(ok,wasControl);
}
void SeaTalkBus::abort() { if(awaiting) { failureReason="cancelled"; finish(false); } }
void SeaTalkBus::onDecoded(void *ctx,const uint8_t *p,size_t n) {
  auto &bus=*static_cast<SeaTalkBus *>(ctx);
  if(bus.mismatch.load()) return; // Never interpret a collided capture as pilot state.
  if(bus.awaiting) {
    if(n==bus.outbound.size && !memcmp(p,bus.outbound.data,n)) { bus.finish(true); return; }
    // A different packet before our echo means the transmission is uncertain.
    bus.failureReason="different-packet";
    if(bus.trace) {
      bridge::Datagram seen; seen.size=n; memcpy(seen.data,p,n);
      bus.trace("unexpected-rx",seen,uint32_t(micros()-bus.startedUs),0,0);
    }
    bus.mismatch=true; bus.finish(false); return;
  }
  ++bus.received; bus.receive(p,n);
}
void SeaTalkBus::poll() {
  if(!initialized) return;
  if(awaiting && mismatch.load()) {
    if(!strcmp(failureReason,"wire-mismatch")) ++collisions;
    finish(false);
  }
  if(awaiting && !txDone && rmt_wait_tx_done(txChannel,0)==ESP_OK) {
    esp_timer_stop(monitor); txDone=true;
  }
  size_t bytes=0;
  for(unsigned batch=0;batch<8;++batch) {
    auto *rx=static_cast<rmt_item32_t *>(xRingbufferReceive(ring,&bytes,0));
    if(!rx) break;
    if(!mismatch.load()) {
      const uint32_t errorsBefore=decoder.errors;
      for(size_t i=0;i<bytes/sizeof(*rx);++i) {
        if(rx[i].duration0) decoder.run(bool(rx[i].level0)!=config::rxInverted,rx[i].duration0);
        if(rx[i].duration1) decoder.run(bool(rx[i].level1)!=config::rxInverted,rx[i].duration1);
      }
      decoder.endCapture(idle());
      if(captureTrace && decoder.errors!=errorsBefore) captureTrace(rx,bytes/sizeof(*rx),decoder.errors-errorsBefore);
    } else if(captureTrace) captureTrace(rx,bytes/sizeof(*rx),0); // discarded collision waveform
    vRingbufferReturnItem(ring,rx);
    // After discarding collision debris, normal listening resumes. No automatic
    // retry of keystrokes: a damaged echo cannot prove the pilot ignored them.
    if(!awaiting) mismatch=false;
  }
  if(awaiting && uint32_t(micros()-startedUs)>100000) { failureReason="echo-timeout"; finish(false); }
}
