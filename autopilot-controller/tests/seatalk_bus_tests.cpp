#include "SeaTalkBus.h"
#include "Config.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {
uint32_t now=1000;
int levels[40]={}, latch[40]={}, routes[40]={};
gpio_mode_t modes[40]={};
esp_timer_create_args_t timer;
bool timerRunning=false, txFinished=false, captureAvailable=false;
bool failTimerStart=false;
unsigned stops=0, completions=0, received=0;
bool completedOk=false;
const char *lastReason=nullptr;
void trace(const char *reason,const bridge::Datagram &,uint32_t,uint32_t,uint32_t) { lastReason=reason; }
std::vector<rmt_item32_t> waveform;
void checkTxMode(int pin,gpio_mode_t mode) {
  if(pin!=config::seaTalkTx) return;
  // A push-pull configuration of the shared node is a regression, even if
  // another call would later change it to open drain.
  assert(mode==GPIO_MODE_INPUT || mode==GPIO_MODE_INPUT_OUTPUT_OD);
  if(mode==GPIO_MODE_INPUT_OUTPUT_OD) assert(latch[pin]==1);
}
void complete(bool ok,bool control) { ++completions; completedOk=ok; assert(control); }
void receive(const uint8_t *,size_t) { ++received; }
void tick(uint32_t us) { now+=us; if(timerRunning) timer.callback(timer.arg); }
}
int gpio_get_level(gpio_num_t pin) { return levels[pin]; }
esp_err_t gpio_set_direction(gpio_num_t pin,gpio_mode_t mode) {
  checkTxMode(pin,mode); modes[pin]=mode;
  // IDF 4.4 gpio_output_enable also resets the output matrix to GPIO. Merely
  // adding input readback to an RMT output therefore disconnects its waveform.
  if(mode & GPIO_MODE_OUTPUT) routes[pin]=SIG_GPIO_OUT_IDX;
  return ESP_OK;
}
esp_err_t gpio_config(const gpio_config_t *c) {
  for(int pin=0;pin<40;++pin) if(c->pin_bit_mask&(1ULL<<pin)) gpio_set_direction(pin,c->mode);
  return ESP_OK;
}
esp_err_t gpio_set_level(gpio_num_t pin,int level) { latch[pin]=level; return ESP_OK; }
void esp_rom_gpio_connect_out_signal(uint32_t pin,uint32_t signal,bool inv,bool oenInv) {
  assert(!inv && !oenInv); routes[pin]=signal;
}
void pinMode(int pin,int mode) { assert(mode==INPUT); gpio_set_direction(pin,GPIO_MODE_INPUT); }
void attachInterruptArg(int,void (*)(void *),void *,int) {}
uint32_t micros() { return now; }
int64_t esp_timer_get_time() { return now; }
uint32_t esp_random() { return 0; }
esp_err_t esp_timer_create(const esp_timer_create_args_t *args,esp_timer_handle_t *handle) {
  timer=*args; *handle=&timer; return ESP_OK;
}
esp_err_t esp_timer_start_periodic(esp_timer_handle_t,uint64_t) {
  if(failTimerStart) return ESP_FAIL;
  timerRunning=true; return ESP_OK;
}
esp_err_t esp_timer_stop(esp_timer_handle_t) { timerRunning=false; return ESP_OK; }
esp_err_t rmt_config(const rmt_config_t *c) {
  // Model IDF's push-pull side effect so a future rmt_config(TX17) fails here.
  gpio_set_direction(c->gpio_num,c->rmt_mode==RMT_MODE_TX?GPIO_MODE_OUTPUT:GPIO_MODE_INPUT);
  if(c->rmt_mode==RMT_MODE_TX) {
    assert(c->tx_config.idle_output_en && c->tx_config.idle_level==RMT_IDLE_LEVEL_HIGH);
    routes[c->gpio_num]=RMT_SIG_OUT0_IDX+int(c->channel);
  }
  return ESP_OK;
}
esp_err_t rmt_driver_install(rmt_channel_t,size_t,int) { return ESP_OK; }
esp_err_t rmt_get_ringbuf_handle(rmt_channel_t,RingbufHandle_t *ring) { *ring=&waveform; return ESP_OK; }
esp_err_t rmt_rx_start(rmt_channel_t,bool) { return ESP_OK; }
esp_err_t rmt_tx_stop(rmt_channel_t) { ++stops; txFinished=true; return ESP_OK; }
esp_err_t rmt_wait_tx_done(rmt_channel_t,uint32_t) { return txFinished?ESP_OK:ESP_FAIL; }
esp_err_t rmt_write_items(rmt_channel_t,const rmt_item32_t *p,int n,bool) {
  waveform.assign(p,p+n); txFinished=false; return ESP_OK;
}
void *xRingbufferReceive(RingbufHandle_t,size_t *bytes,uint32_t) {
  if(!captureAvailable) return nullptr;
  captureAvailable=false; *bytes=waveform.size()*sizeof(rmt_item32_t); return waveform.data();
}
void vRingbufferReturnItem(RingbufHandle_t,void *) {}

int main() {
  static_assert(config::txOpenDrain && !config::txInverted && !config::rxInverted,
                "This hardware regression suite exercises the shared MOSFET profile");
  SeaTalkBus bus(receive,complete);
  bus.setTrace(trace,nullptr);
  levels[config::seaTalkRx]=levels[config::seaTalkTx]=levels[config::seaTalkTxMonitor]=1;
  assert(bus.begin());
  assert(modes[config::seaTalkTx]==GPIO_MODE_INPUT_OUTPUT_OD);
  assert(modes[config::seaTalkTxMonitor]==GPIO_MODE_INPUT_OUTPUT);
  assert(routes[config::seaTalkTx]==routes[config::seaTalkTxMonitor]);
  assert(routes[config::seaTalkTx]==RMT_SIG_OUT0_IDX+int(RMT_CHANNEL_4));
  const auto packet=bridge::key(0x01);
  assert(!bus.send(packet,true)); // Observe bus quiet time at startup.
  tick(4000);
  levels[config::seaTalkRx]=0;
  assert(!bus.send(packet,true)); // Never start on a dominant bus.
  levels[config::seaTalkRx]=1;
  assert(bus.send(packet,true));
  assert(!bus.send(packet,true));

  // Actual emitted bits must decode to the original datagram, command marker
  // included, before the completion callback can report successful delivery.
  txFinished=true; captureAvailable=true; bus.poll();
  assert(completions==1 && completedOk && bus.sent==1 && !bus.busy());
  assert(received==0); // Own echo must not be reported as pilot feedback.
  assert(!strcmp(lastReason,"echo-ok"));

  tick(4000); assert(bus.send(packet,true));
  // Short propagation disagreement is tolerated and clears on agreement.
  levels[config::seaTalkRx]=0;
  tick(50); levels[config::seaTalkRx]=1; tick(50);
  assert(stops==0);
  // Another talker now pulls BOTH shared LV pads low while our intended bit
  // is HIGH/release. Comparing TX17 with RX16 would miss this collision.
  levels[config::seaTalkRx]=levels[config::seaTalkTx]=0;
  tick(50); tick(50); assert(stops==0); tick(50);
  assert(stops==1);
  captureAvailable=true; bus.poll(); // Discard the damaged capture.
  assert(completions==2 && !completedOk && bus.collisions==1 && !bus.busy());
  assert(received==0 && bus.sent==1);
  assert(!strcmp(lastReason,"wire-mismatch"));
  levels[config::seaTalkRx]=levels[config::seaTalkTx]=1;
  tick(4000); assert(!bus.send(packet,true)); // Collision backoff.
  tick(250000); assert(bus.send(packet,true));
  tick(100001); bus.poll(); // No echo: outcome remains uncertain.
  assert(completions==3 && !completedOk && !bus.busy());
  assert(!strcmp(lastReason,"echo-timeout"));

  tick(260000); failTimerStart=true;
  assert(bus.send(packet,true));
  assert(completions==3); // Driver failure must complete asynchronously.
  bus.poll(); assert(completions==4 && !completedOk && !bus.busy());
  assert(!strcmp(lastReason,"timer-start") && bus.collisions==1);
  // Listen-only must block even non-control forwarding while preserving RX.
  failTimerStart=false;
  bus.setListenOnly(true);
  tick(260000);
  assert(!bus.send(packet,true) && !bus.send(packet,false));
  captureAvailable=true; bus.poll(); // Discard debris from the previous failure.
  captureAvailable=true; bus.poll();
  assert(received==1 && bus.received==1 && !bus.busy());
  bus.setListenOnly(false);
  assert(bus.send(packet,true));
  bus.setListenOnly(true);
  assert(completions==5 && !completedOk && !bus.busy());
  assert(!bus.send(packet,false));
  assert(!strcmp(lastReason,"cancelled"));
  bus.setListenOnly(false);
  tick(260000); assert(bus.send(packet,true));
  waveform[0].level1^=1; // A valid but different command byte is not our echo.
  txFinished=true; captureAvailable=true; bus.poll();
  assert(completions==6 && !completedOk && !bus.busy());
  assert(!strcmp(lastReason,"different-packet"));
  std::puts("SeaTalk driver: shared-node startup, waveform echo, collision, failure and listen-only tests passed");
}
