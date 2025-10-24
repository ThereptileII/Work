/*
  ESP32 N2K Switch Bank (manual PGNs, onboard TWAI)
  - Bank Instance 0, 8 channels
  - Periodic 127501 status (fast @ boot, then 1 Hz)
  - Receives 127502 control (set/clear bitmasks)
  - Sends 127504 labels (short form) at boot & every 30 s
  - Minimal 60928 Address Claim + reply to 59904 ISO Request
  - Arduino-ESP32 core 3.x+, uses <driver/twai.h>
*/

#include <Arduino.h>
#include "driver/twai.h"

// ---- TWAI wiring (change to your board) ----
static const int TWAI_TX = 5;
static const int TWAI_RX = 4;

// ---- Relay outputs ----
static const uint8_t CHANNEL_COUNT = 8;                   // up to 28 per spec
static const int RELAY_PINS[CHANNEL_COUNT] = {13,12,14,27,26,25,33,32};
static const bool RELAY_ACTIVE_HIGH = true;               // set false for active-LOW boards

// ---- N2K basics ----
static const uint8_t SRC_ADDR         = 0x23;             // pick a free source address
static const uint8_t PRIORITY_DEFAULT = 3;
static const uint8_t BANK_INSTANCE    = 0;

// ---- Cadence ----
static const uint32_t BOOT_BURST_MS          = 10000;     // first 10s: fast status
static const uint32_t BOOT_BURST_PERIOD_MS   = 250;
static const uint32_t NORMAL_STATUS_PERIODMS = 1000;      // 1 Hz after burst
static const uint32_t LABEL_RESEND_PERIOD_MS = 30000;     // send labels every 30 s

// ---- PGNs ----
static const uint32_t PGN_ISO_REQUEST    = 0x00EA00;  // 59904 (PDU1)
static const uint32_t PGN_ADDR_CLAIM     = 0x00EE00;  // 60928 (PDU2)
static const uint32_t PGN_SWITCH_STATUS  = 0x01F20D;  // 127501 (PDU2)
static const uint32_t PGN_SWITCH_CONTROL = 0x01F20E;  // 127502 (PDU2)
static const uint32_t PGN_SWITCH_LABEL   = 0x01F210;  // 127504 (PDU2)

// ---- Labels (<=6 chars each for short-frame form) ----
static const char* SWITCH_LABELS[CHANNEL_COUNT] = {
  "NAV", "CABIN", "ANCHOR", "BILGE", "WIPER", "STEREO", "DECK", "SPARE"
};

// ---- State ----
static uint32_t channelMask = 0;     // ON/OFF bits (bit i = channel i)
static uint32_t lastStatusMs = 0;
static uint32_t lastLabelMs  = 0;
static uint32_t bootMs       = 0;
static uint32_t statusPeriod = NORMAL_STATUS_PERIODMS;

// ---- CAN ID helpers ----
static inline uint32_t make_can_id(uint8_t prio, uint32_t pgn, uint8_t src) {
  return ((uint32_t)prio << 26) | (pgn << 8) | src;
}
static inline uint32_t pgn_from_id(uint32_t id) {
  uint8_t PF = (id >> 16) & 0xFF, PS = (id >> 8) & 0xFF;
  return (PF >= 240) ? ((uint32_t)PF << 8) | PS : ((uint32_t)PF << 8);
}
static inline uint8_t src_from_id(uint32_t id){ return id & 0xFF; }
static inline uint8_t dst_from_id(uint32_t id){
  uint8_t PF = (id >> 16) & 0xFF, PS = (id >> 8) & 0xFF;
  return (PF < 240) ? PS : 0xFF;
}

// ---- Minimal 64-bit NAME (for 60928) ----
struct N2kName { uint64_t v; };
static N2kName make_name(uint32_t unique_id = 0x00ABCDE) {
  uint64_t name = 0;
  const uint8_t industry_group = 4;  // Marine
  const uint8_t system_instance = 0;
  const uint8_t device_class    = 30; // Electrical Gen/Switching-like
  const uint8_t device_function = 130;
  const uint8_t device_instance = 0;
  const uint16_t manufacturer   = 2046; // arbitrary
  name |= ((uint64_t)(unique_id & 0x1FFFFF)) << 43; // 21b Identity
  name |= ((uint64_t)(manufacturer & 0x7FF)) << 32; // 11b Mfg
  name |= ((uint64_t)(device_instance & 0x7F)) << 25;
  name |= ((uint64_t)(device_function))     << 17;
  name |= ((uint64_t)0)                     << 16;  // reserved
  name |= ((uint64_t)(device_class & 0x7F)) << 9;
  name |= ((uint64_t)(system_instance & 0x0F)) << 5;
  name |= ((uint64_t)(industry_group & 0x07)) << 2;
  name |= 0x03; // arbitrary address capable
  return N2kName{ name };
}

// ---- TWAI bring-up ----
bool twai_init() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX, (gpio_num_t)TWAI_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 16; g.rx_queue_len = 64;
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS(); // N2K @ 250k
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  return (twai_driver_install(&g,&t,&f) == ESP_OK) && (twai_start() == ESP_OK);
}

// ---- Send 1 frame ----
bool send_pgn(uint32_t pgn, const uint8_t* d, uint8_t len, uint8_t prio = PRIORITY_DEFAULT, uint8_t src = SRC_ADDR) {
  twai_message_t m = {0};
  m.extd = 1; m.identifier = make_can_id(prio, pgn, src);
  m.data_length_code = len;
  for (int i = 0; i < len && i < 8; ++i) m.data[i] = d[i];
  return twai_transmit(&m, pdMS_TO_TICKS(20)) == ESP_OK;
}

// ---- 60928 Address Claim ----
void send_address_claim() {
  uint8_t d[8]; uint64_t v = make_name((uint32_t)esp_random() & 0x1FFFFF).v;
  for (int i = 0; i < 8; ++i) { d[i] = (uint8_t)(v & 0xFF); v >>= 8; }
  send_pgn(PGN_ADDR_CLAIM, d, 8, 6);
}

// ---- 127501 Status ----
// Byte0: Bank Instance
// Byte1..4: bitmask (Status1..Status32). 1=ON
// Byte5..7: 0xFF
void send_switch_status() {
  uint8_t d[8];
  d[0] = BANK_INSTANCE;
  d[1] = (uint8_t)(channelMask & 0xFF);
  d[2] = (uint8_t)((channelMask >> 8) & 0xFF);
  d[3] = (uint8_t)((channelMask >> 16) & 0xFF);
  d[4] = (uint8_t)((channelMask >> 24) & 0xFF);
  d[5] = d[6] = d[7] = 0xFF;
  send_pgn(PGN_SWITCH_STATUS, d, 8);
}

// ---- 127504 Labels (short form, single frame)
// Byte0=Bank, Byte1=Channel, Byte2..7=ASCII (6 chars, 0-padded)
void send_switch_label_short(uint8_t bank, uint8_t ch, const char* label) {
  uint8_t d[8] = {0};
  d[0] = bank; d[1] = ch;
  for (int i = 0; i < 6 && label && label[i]; ++i) d[2+i] = (uint8_t)label[i];
  send_pgn(PGN_SWITCH_LABEL, d, 8, 6); // low-ish priority OK
}
void send_all_labels() {
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ++ch) {
    const char* name = SWITCH_LABELS[ch] ? SWITCH_LABELS[ch] : "SW";
    send_switch_label_short(BANK_INSTANCE, ch, name);
    delay(5);
  }
}

// ---- 127502 Control handler (mask model) ----
// Byte0: Bank
// Byte1..4: SET mask (1 bits => ON)
// Byte5..7: CLEAR mask (1 bits => OFF)   [short variant]
void handle_switch_control(const uint8_t* d, uint8_t len) {
  if (len < 8 || d[0] != BANK_INSTANCE) return;
  uint32_t setMask = (uint32_t)d[1] | ((uint32_t)d[2]<<8) | ((uint32_t)d[3]<<16) | ((uint32_t)d[4]<<24);
  uint32_t clrMask = (uint32_t)d[5] | ((uint32_t)d[6]<<8) | ((uint32_t)d[7]<<16);

  uint32_t mask = (CHANNEL_COUNT >= 32) ? 0xFFFFFFFFu : ((1u << CHANNEL_COUNT) - 1u);
  setMask &= mask; clrMask &= mask;

  uint32_t newMask = (channelMask | setMask) & ~clrMask;
  if (newMask != channelMask) {
    channelMask = newMask;
    // Drive GPIOs
    for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
      bool on = (channelMask >> i) & 1u;
      if (RELAY_ACTIVE_HIGH) digitalWrite(RELAY_PINS[i], on ? HIGH : LOW);
      else                    digitalWrite(RELAY_PINS[i], on ? LOW  : HIGH);
    }
    // Immediately announce new status
    send_switch_status();
  }
}

// ---- 59904 ISO Request (reply with 60928 when requested) ----
void handle_iso_request(uint32_t id, const uint8_t* d, uint8_t len) {
  if (len < 3) return;
  uint32_t req_pgn = (uint32_t)d[0] | ((uint32_t)d[1]<<8) | ((uint32_t)d[2]<<16);
  if (req_pgn == PGN_ADDR_CLAIM) send_address_claim();
}

// ---- RX loop ----
void process_rx() {
  twai_message_t rx;
  if (twai_receive(&rx, pdMS_TO_TICKS(5)) != ESP_OK) return;
  if (!rx.extd) return;
  uint32_t id  = rx.identifier;
  uint32_t pgn = pgn_from_id(id);

  if (pgn == PGN_SWITCH_CONTROL)   handle_switch_control(rx.data, rx.data_length_code);
  else if (pgn == PGN_ISO_REQUEST) handle_iso_request(id, rx.data, rx.data_length_code);

  // Debug
  Serial.printf("[RX] PGN %06lX len=%d SA=%u\n", pgn, rx.data_length_code, src_from_id(id));
}

// ---- Setup/Loop ----
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32 N2K Switch Bank (manual PGNs, TWAI)");

  for (uint8_t i=0;i<CHANNEL_COUNT;i++){
    pinMode(RELAY_PINS[i], OUTPUT);
    if (RELAY_ACTIVE_HIGH) digitalWrite(RELAY_PINS[i], LOW);
    else                    digitalWrite(RELAY_PINS[i], HIGH);
  }

  if (!twai_init()) { Serial.println("TWAI init FAILED"); while(true) delay(1000); }
  Serial.println("TWAI started @250k");

  bootMs = millis();
  statusPeriod = BOOT_BURST_PERIOD_MS;     // fast during boot window

  send_address_claim();
  send_switch_status(); delay(50);
  send_all_labels();    // announce names early
}

void loop() {
  process_rx();

  uint32_t now = millis();
  if (now - bootMs > BOOT_BURST_MS) statusPeriod = NORMAL_STATUS_PERIODMS;

  if (now - lastStatusMs >= statusPeriod) {
    send_switch_status();
    lastStatusMs = now;
  }
  if (now - lastLabelMs >= LABEL_RESEND_PERIOD_MS) {
    send_all_labels();
    lastLabelMs = now;
  }
}
