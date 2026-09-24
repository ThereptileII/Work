#pragma once
#include "BridgeCore.h"
#include <driver/rmt.h>
#include <esp_timer.h>
#include <freertos/ringbuf.h>
#include <atomic>

class SeaTalkBus {
 public:
  using Receive=void (*)(const uint8_t *,size_t);
  using Complete=void (*)(bool,bool); // success, control packet
  using Trace=void (*)(const char *,const bridge::Datagram &,uint32_t,uint32_t,uint32_t);
  using CaptureTrace=void (*)(const rmt_item32_t *,size_t,uint32_t);
  SeaTalkBus(Receive rx, Complete tx) : decoder(onDecoded,this), receive(rx), complete(tx) {}
  bool begin();
  void poll();
  bool send(const bridge::Datagram &packet, bool control);
  bool busy() const { return awaiting; }
  void abort();
  void setListenOnly(bool enabled) { listenOnly=enabled; if(enabled) abort(); }
  bool listeningOnly() const { return listenOnly; }
  void setTrace(Trace tx, CaptureTrace rx) { trace=tx; captureTrace=rx; }
  uint32_t received=0, sent=0, collisions=0, echoFailures=0;
  uint32_t framingErrors() const { return decoder.errors; }
 private:
  static constexpr rmt_channel_t rxChannel=RMT_CHANNEL_0, txChannel=RMT_CHANNEL_4;
  bridge::WireDecoder decoder;
  Receive receive;
  Complete complete;
  RingbufHandle_t ring=nullptr;
  esp_timer_handle_t monitor=nullptr;
  bridge::Datagram outbound;
  rmt_item32_t items[100]={}; // 18 chars * 11 bits = 99 RMT items.
  bool initialized=false, awaiting=false, controlPacket=false, txDone=false;
  bool listenOnly=false;
  std::atomic<bool> mismatch{false};
  std::atomic<uint32_t> mismatchAt{0};
  std::atomic<uint32_t> monitorAt{0}, monitorGapUs{0}, mismatchLevels{0};
  const char *failureReason="unknown";
  Trace trace=nullptr;
  CaptureTrace captureTrace=nullptr;
  uint32_t startedUs=0, backoffUs=3000;
  volatile uint32_t lastEdgeUs=0;
  static void onDecoded(void *context,const uint8_t *p,size_t n);
  static void monitorWire(void *context);
  static void IRAM_ATTR edge(void *context);
  void finish(bool ok);
  bool idle() const;
};
