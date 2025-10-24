/*
  XIAO ESP32-C6 — Dual TWAI (EV-CAN + NMEA2000) + Responsive Web UI + Switch Bank

  EV-CAN  @500k (Bus A): TX=GPIO2 (D2),  RX=GPIO3 (D3)
  N2K     @250k (Bus B): TX=GPIO20(D20), RX=GPIO21(D21)

  Engine:
    * 127488 (RPM), 127493 (Gear; fast-path), 127508 (V/A), 127506 (SOC),
      127489 (Engine Dynamic; fast-packet, coolant temp from motor temp),
      61184 (Proprietary A; regen)

  Switch Bank:
    * 60928  ISO Address Claim (we claim at boot and respond to ISO Request)
    * 59904  ISO Request (we handle requests for 60928, 127501, 127504)
    * 127501 Binary Switch Bank Status (boot storm then steady; also on-change)
    * 127502 Switch Bank Control (single-frame masks; up to 24 ch handled here)
    * 127504 Switch Bank Label (short 6-char labels, single-frame)

  Web:
    - SoftAP "XIAO-MOTOR" / "12345678"
    - /          : dashboard (engine)
    - /switches  : switch control page (toggle buttons)
    - WebSocket :81 JSON stream + commands

  Hardware:
    - One CAN transceiver per bus; proper termination per bus (120 Ω at ends)
*/

#include <Arduino.h>
#include <driver/twai.h>
#include <math.h>
#include <string.h>

// ---------- WiFi / Web ----------
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WebSocketsServer.h> // arduinoWebSockets by Markus Sattler

// ====== WiFi Config ======
const char* AP_SSID = "XIAO-MOTOR";
const char* AP_PASS = "12345678"; // >= 8 chars
const byte  DNS_PORT = 53;
const int   AP_CHAN  = 6;

// ====== Web globals ======
DNSServer        dnsServer;
WebServer        http(80);
WebSocketsServer ws(81);
uint32_t         broadcastIntervalMs = 1000;
uint32_t         lastBroadcastMs = 0;
bool             anyClientConnected = false;

// ====== Responsive Fullscreen HTML (dashboard) ======
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover"/>
<meta name="theme-color" content="#0b1220"/>
<title>XIAO Motor Data</title>
<style>
:root{--bg:#0b1220;--card:#0d1526;--text:#e6eefc;--muted:#8aa0c3;--accent:#56c0ff;--pad:clamp(10px,2.5vw,24px);--tile-gap:clamp(8px,1.5vw,16px)}
*{box-sizing:border-box}html,body{height:100%}
body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Inter,Arial;color:var(--text);background:var(--bg);min-height:100svh;min-height:100dvh;min-height:100vh;padding:calc(env(safe-area-inset-top)+var(--pad)) var(--pad) calc(env(safe-area-inset-bottom)+var(--pad)) var(--pad);display:flex;align-items:stretch;justify-content:center}
.app{width:min(1200px,100%);height:100%;display:flex;flex-direction:column;gap:var(--tile-gap)}
.header{display:flex;align-items:center;justify-content:space-between;gap:12px}
.title{font-size:clamp(18px,2.6vw,28px);font-weight:700;margin:0}
.badge{background:#0f2236;border:1px solid #20334d;color:var(--accent);padding:6px 12px;border-radius:999px;font-weight:600;white-space:nowrap}
.header-right{display:flex;gap:8px;align-items:center;color:var(--muted);font-size:clamp(12px,1.6vw,14px)}
.card{background:var(--card);border:1px solid #1b2740;border-radius:16px;box-shadow:0 8px 30px rgba(0,0,0,.35);width:100%;height:100%;display:flex;flex-direction:column;padding:var(--pad)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:var(--tile-gap);margin-top:var(--tile-gap);flex:1 1 auto;min-height:0}
.tile{background:#0f1726;border:1px solid #1d2a44;border-radius:12px;padding:clamp(10px,1.8vw,14px);display:flex;flex-direction:column;justify-content:center;min-height:90px}
.label{color:var(--muted);font-size:clamp(11px,1.6vw,13px);margin-bottom:6px}
.kv{display:flex;align-items:baseline;gap:8px}
.v{font-size:clamp(22px,5vw,40px);font-weight:800;letter-spacing:.3px;line-height:1.1}
.u{color:var(--muted);font-size:clamp(12px,1.8vw,14px)}
.footer{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin-top:var(--tile-gap)}
.controls{display:flex;gap:8px;align-items:center}
button,input{background:#0e2337;border:1px solid #1d3653;color:var(--text);padding:10px 12px;border-radius:10px;font-size:clamp(12px,1.8vw,14px)}
input{width:min(140px,35vw)}
a{color:var(--accent);text-decoration:none;font-size:clamp(12px,1.8vw,14px)}
@media (max-width:380px){.tile{min-height:70px}}
</style></head><body>
<div class="app">
  <div class="header">
    <h1 class="title">XIAO Motor Data</h1>
    <div class="header-right"><span class="badge" id="status">Connecting…</span><span id="ts">--</span> <a href="/switches">Switches</a></div>
  </div>
  <div class="card">
    <div class="grid">
      <div class="tile"><div class="label">RPM</div><div class="kv"><div class="v" id="rpm">--</div><div class="u">rpm</div></div></div>
      <div class="tile"><div class="label">Voltage</div><div class="kv"><div class="v" id="v">--</div><div class="u">V</div></div></div>
      <div class="tile"><div class="label">Current (chg +)</div><div class="kv"><div class="v" id="a">--</div><div class="u">A</div></div></div>
      <div class="tile"><div class="label">Power</div><div class="kv"><div class="v" id="p">--</div><div class="u">kW</div></div></div>
      <div class="tile"><div class="label">SOC</div><div class="kv"><div class="v" id="soc">--</div><div class="u">%</div></div></div>
      <div class="tile"><div class="label">Motor Temp</div><div class="kv"><div class="v" id="mt">--</div><div class="u">°C</div></div></div>
      <div class="tile"><div class="label">Gear</div><div class="kv"><div class="v" id="gear">--</div></div></div>
      <div class="tile"><div class="label">Regen</div><div class="kv"><div class="v" id="regen">--</div></div></div>
      <div class="tile"><div class="label">Clients</div><div class="kv"><div class="v" id="clients">0</div></div></div>
    </div>
    <div class="footer">
      <div class="controls">
        <label for="rate" class="label" style="margin:0">Interval (ms)</label>
        <input id="rate" type="number" min="50" step="50" value="1000" inputmode="numeric"/>
        <button id="set">Set</button>
      </div>
      <a href="/">Reload</a>
    </div>
  </div>
</div>
<script>
const $=id=>document.getElementById(id);
const statusEl=$("status"), tsEl=$("ts");
const rpmEl=$("rpm"), vEl=$("v"), aEl=$("a"), pEl=$("p"), socEl=$("soc"), mtEl=$("mt"), gearEl=$("gear"), regenEl=$("regen"), clientsEl=$("clients");
const rateInput=$("rate"), setBtn=$("set");
const host = location.hostname || "192.168.4.1";
const ws = new WebSocket(`ws://${host}:81/`);
ws.onopen=()=>{statusEl.textContent="Live"}; ws.onclose=()=>{statusEl.textContent="Disconnected"}; ws.onerror=()=>{statusEl.textContent="Error"};
ws.onmessage=(ev)=>{try{const j=JSON.parse(ev.data);
  if(j.ts_ms!==undefined) tsEl.textContent=new Date(j.ts_ms).toLocaleTimeString();
  if(j.clients!==undefined) clientsEl.textContent=j.clients;
  if(j.rpm!==undefined) rpmEl.textContent=j.rpm.toFixed?j.rpm.toFixed(0):j.rpm;
  if(j.v!==undefined) vEl.textContent=(+j.v).toFixed(1);
  if(j.a!==undefined) aEl.textContent=(+j.a).toFixed(1);
  if(j.p_kw!==undefined) pEl.textContent=(+j.p_kw).toFixed(1);
  if(j.soc!==undefined) socEl.textContent=(+j.soc).toFixed(1);
  if(j.mt_c!==undefined) mtEl.textContent=(+j.mt_c).toFixed(1);
  if(j.gear_str)  gearEl.textContent=j.gear_str;
  if(j.regen_str) regenEl.textContent=j.regen_str;
  if(j.status) statusEl.textContent=j.status;
}catch(e){}};
setBtn.onclick=()=>{const v=parseInt(rateInput.value||"1000",10); if(isFinite(v)&&v>=50) ws.send(JSON.stringify({cmd:"set_rate",ms:v}));};
</script>
</body></html>
)HTML";

// ====== Switches page HTML ======
const char SWITCHES_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover"/>
<meta name="theme-color" content="#0b1220"/>
<title>XIAO Switches</title>
<style>
:root{--bg:#0b1220;--card:#0d1526;--text:#e6eefc;--muted:#8aa0c3;--accent:#56c0ff;--pad:clamp(10px,2.5vw,24px);--gap:clamp(8px,1.5vw,16px)}
*{box-sizing:border-box}html,body{height:100%}
body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Inter,Arial;color:var(--text);background:var(--bg);min-height:100svh;min-height:100dvh;min-height:100vh;padding:calc(env(safe-area-inset-top)+var(--pad)) var(--pad) calc(env(safe-area-inset-bottom)+var(--pad)) var(--pad);display:flex;align-items:stretch;justify-content:center}
.app{width:min(1200px,100%);height:100%;display:flex;flex-direction:column;gap:var(--gap)}
.header{display:flex;align-items:center;justify-content:space-between;gap:12px}
.title{font-size:clamp(18px,2.6vw,28px);font-weight:700;margin:0}
.badge{background:#0f2236;border:1px solid #20334d;color:var(--accent);padding:6px 12px;border-radius:999px;font-weight:600;white-space:nowrap}
a{color:var(--accent);text-decoration:none;font-size:clamp(12px,1.8vw,14px)}
.card{background:var(--card);border:1px solid #1b2740;border-radius:16px;box-shadow:0 8px 30px rgba(0,0,0,.35);width:100%;height:100%;display:flex;flex-direction:column;padding:var(--pad)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:var(--gap);margin-top:var(--gap);flex:1 1 auto;min-height:0}
.switch{background:#0f1726;border:1px solid #1d2a44;border-radius:12px;padding:clamp(12px,2vw,16px);display:flex;flex-direction:column;gap:8px}
.label{color:var(--muted);font-size:clamp(11px,1.6vw,13px)}
.btn{display:flex;align-items:center;justify-content:center;border-radius:10px;padding:12px;font-weight:700;cursor:pointer;border:1px solid #1d3653;background:#0e2337;color:var(--text);font-size:clamp(14px,2vw,16px)}
.btn.on{background:#1b3b1f;border-color:#2a5c35}
.footer{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin-top:var(--gap)}
.small{color:var(--muted);font-size:clamp(12px,1.6vw,13px)}
</style></head><body>
<div class="app">
  <div class="header">
    <h1 class="title">Switches</h1>
    <div class="small"><span class="badge" id="status">Connecting…</span> <a href="/">Dashboard</a></div>
  </div>
  <div class="card">
    <div id="grid" class="grid"></div>
    <div class="footer small">
      <div>Bank <span id="bank">0</span>, Channels: <span id="count">--</span></div>
      <div id="ts">--</div>
    </div>
  </div>
</div>
<script>
const $=id=>document.getElementById(id);
const statusEl=$("status"), grid=$("grid"), tsEl=$("ts"), bankEl=$("bank"), countEl=$("count");
const host=location.hostname||"192.168.4.1"; const ws=new WebSocket(`ws://${host}:81/`);
let labels=[], states=0, n=0, bank=0;
function draw(){
  grid.innerHTML="";
  for(let i=1;i<=n;i++){
    const lab=(labels[i]||("SW "+i));
    const on=((states>>(i-1))&1)==1;
    const el=document.createElement("div"); el.className="switch";
    const L=document.createElement("div"); L.className="label"; L.textContent=lab; el.appendChild(L);
    const B=document.createElement("button"); B.className="btn"+(on?" on":""); B.textContent=on?"ON":"OFF";
    B.onclick=()=>{ ws.send(JSON.stringify({cmd:"switch", ch:i, on:!on})); };
    el.appendChild(B);
    grid.appendChild(el);
  }
  countEl.textContent = n;
  bankEl.textContent = bank;
}
ws.onopen = ()=>{statusEl.textContent="Live"; ws.send(JSON.stringify({cmd:"switches_sub"}));};
ws.onclose=()=>{statusEl.textContent="Disconnected"}; ws.onerror=()=>{statusEl.textContent="Error"};
ws.onmessage=(ev)=>{ try{ const j=JSON.parse(ev.data);
  if(j.ts_ms!==undefined) tsEl.textContent=new Date(j.ts_ms).toLocaleTimeString();
  if(j.bank!==undefined) bank=j.bank;
  if(j.sw_count!==undefined) n=j.sw_count;
  if(j.sw_state!==undefined) states=j.sw_state>>>0;
  if(j.labels) labels=j.labels;
  if(j.status) statusEl.textContent=j.status;
  draw();
}catch(e){} };
</script>
</body></html>
)HTML";

// ---------- HTTP helpers (captive portal) ----------
bool captivePortal() {
  if (!http.hostHeader().equals(WiFi.softAPIP().toString())) {
    http.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
    http.send(302, "text/plain", "");
    return true;
  }
  return false;
}
void handleRoot(){ http.send_P(200, "text/html; charset=utf-8", INDEX_HTML); }
void handleSwitches(){ http.send_P(200, "text/html; charset=utf-8", SWITCHES_HTML); }
void handleNotFound(){ if (captivePortal()) return; http.send(404, "text/plain", "Not found"); }

// ---------- WebSocket helpers ----------
String gearToStr(uint8_t g){ return (g==1)?"DRIVE":(g==2)?"NEUTRAL":(g==3)?"REVERSE":"UNKNOWN"; }
String regenToStr(uint8_t r){ return (r==0)?"Off":(r==1)?"OneBar":(r==2)?"TwoBar":"Unknown"; }

// ---------- EV-CAN / N2K Pins (dual-controller) ----------
#define EV_TX  GPIO_NUM_2    // Bus A TX (XIAO D2)
#define EV_RX  GPIO_NUM_3    // Bus A RX (XIAO D3)
#define N2K_TX GPIO_NUM_20   // Bus B TX (XIAO D20 / RX silk)
#define N2K_RX GPIO_NUM_21   // Bus B RX (XIAO D21 / TX silk)

// ---------- EV-CAN IDs ----------
static const uint16_t ID_PWR  = 0x1DB; // V/I/SOC
static const uint16_t ID_RPM  = 0x1DA; // RPM
static const uint16_t ID_TEMP = 0x55A; // Motor temp
static const uint16_t ID_GEAR = 0x539; // Gear + Regen

// ---------- N2K addressing & PGNs ----------
static const uint8_t  N2K_SA   = 0x23; // Source Address
static const uint8_t  N2K_PRIO = 6;
static const uint8_t  N2K_BCAST= 0xFF;

static const uint32_t PGN_ISO_ADDRESS_CLAIM = 60928;
static const uint32_t PGN_ISO_REQUEST       = 59904;

static const uint32_t PGN_ENG_RAPID   = 127488;
static const uint32_t PGN_TRANS_DYN   = 127493;
static const uint32_t PGN_BATT_STATUS = 127508;
static const uint32_t PGN_DC_STATUS   = 127506;
static const uint32_t PGN_ENG_DYNAMIC = 127489; // Fast-Packet
static const uint32_t PGN_PROP_A      = 61184;  // Proprietary A (PDU1, DA required)

// Switch bank PGNs
static const uint32_t PGN_SWITCH_STATUS = 127501;
static const uint32_t PGN_SWITCH_CONTROL= 127502;
static const uint32_t PGN_SWITCH_LABEL  = 127504;

// ---------- Telemetry ----------
typedef struct {
  float   packV = NAN;   // V
  float   packA = NAN;   // A (Leaf decode DISCHARGE POS; invert when publishing)
  float   socPct = NAN;  // %
  int32_t rpm = 0;       // rpm
  float   motorTempC = NAN;
  uint8_t gear = 0;      // 1=Drive,2=Neutral,3=Reverse; else 0
  uint8_t regen = 0xFF;  // 0..2; else 0xFF
  uint8_t sid = 0;
} Telemetry;

Telemetry telem;

// ---------- Switch Bank State ----------
static const uint8_t SW_BANK_INSTANCE = 0;   // Bank 0
static const uint8_t SW_CHANNELS      = 16;  // You can set up to 28; status/control masks handle 24 in single frame here
volatile uint32_t sw_state_bits = 0;         // bit 0 => channel 1
char sw_labels[28][7] = {
  "","NAV","ANCHOR","CABIN","DECK","BILGE","WIPER","FAN","PUMP","AUX1","AUX2","AUX3","AUX4","AUX5","AUX6","AUX7","AUX8",
  "A17","A18","A19","A20","A21","A22","A23","A24","A25","A26","A27","A28"
};

// Boot-storm timer for 127501
uint32_t sw_bootStormEndMs = 0;

// ---------- Concurrency ----------
SemaphoreHandle_t telemMtx;
static inline void TEL_LOCK(){ xSemaphoreTake(telemMtx, portMAX_DELAY); }
static inline void TEL_UNLOCK(){ xSemaphoreGive(telemMtx); }

// ---------- TWAI Instances ----------
static bool g_hasEV  = false; // EV-CAN @500k
static bool g_hasN2K = false; // N2K   @250k

// ---------- Helpers ----------
#if !defined(TWAI_CONTROLLER_ID_0)
  #define TWAI_CONTROLLER_ID_0 0
  #define TWAI_CONTROLLER_ID_1 1
#endif

bool twaiInitPinsSpeedCtrl(uint8_t tx, uint8_t rx, int kbps, int controller_id) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx, (gpio_num_t)rx, TWAI_MODE_NORMAL);
  // Best-effort set controller_id (newer cores have this field)
  #ifdef __cplusplus
  *(volatile uint32_t*)((uint8_t*)&g + offsetof(twai_general_config_t, controller_id)) = controller_id;
  #endif
  twai_timing_config_t  t = (kbps==500) ? TWAI_TIMING_CONFIG_500KBITS() : TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  twai_reconfigure_alerts(TWAI_ALERT_BUS_OFF|TWAI_ALERT_RX_DATA|TWAI_ALERT_TX_FAILED, nullptr);
  return true;
}

static inline void setU16(uint8_t *d, uint16_t v){ d[0]=v&0xFF; d[1]=v>>8; }
static inline void setS16(uint8_t *d, int16_t  v){ d[0]=v&0xFF; d[1]=(uint16_t)v>>8; }

// Build 29-bit ID; handle PDU1 (PF<240, DA) vs PDU2
static uint32_t n2kId(uint8_t prio, uint32_t pgn, uint8_t sa, uint8_t da_for_pdu1 = 0xFF) {
  uint8_t PF = (pgn >> 8) & 0xFF;
  uint8_t DP = (pgn >> 16) & 0x01;
  if (PF >= 240) { // PDU2
    return ((uint32_t)prio << 26) | ((uint32_t)DP << 24) | ((uint32_t)PF << 16) | ((uint32_t)(pgn & 0xFF) << 8) | sa;
  } else {         // PDU1 (needs DA)
    return ((uint32_t)prio << 26) | ((uint32_t)DP << 24) | ((uint32_t)PF << 16) | ((uint32_t)da_for_pdu1 << 8) | sa;
  }
}

// Extract PGN from 29-bit ID
static inline uint32_t idToPGN(uint32_t id){
  uint8_t PF = (id >> 16) & 0xFF;
  if (PF < 240){
    // PDU1: PGN has PS zeroed
    return ((id >> 8) & 0x1FFFF00);
  }else{
    // PDU2: PGN uses PS as low byte
    return ((id >> 8) & 0x1FFFF);
  }
}
static inline uint8_t idToDA(uint32_t id){ return (id >> 8) & 0xFF; }
static inline uint8_t idToSA(uint32_t id){ return id & 0xFF; }

// ---------- N2K TX ----------
bool n2kTransmit(uint32_t pgn, const uint8_t *payload, uint8_t len, uint8_t da_for_pdu1 = 0xFF) {
  if (!g_hasN2K) return false;
  twai_message_t msg; memset(&msg,0,sizeof(msg));
  msg.extd = 1;
  msg.identifier = n2kId(N2K_PRIO, pgn, /*SA*/0x23, da_for_pdu1);
  msg.data_length_code = len;
  memcpy(msg.data, payload, len);
  return (twai_transmit(&msg, pdMS_TO_TICKS(20)) == ESP_OK);
}

// Fast-Packet TX helper (for 127489 or longer messages later)
static uint8_t fp_seq = 0;  // rolls 0..31
bool n2kFastPacketTransmit(uint32_t pgn, const uint8_t *payload, uint8_t len, uint8_t da_for_pdu1 = 0xFF) {
  if (!g_hasN2K) return false;
  if (len <= 8) return n2kTransmit(pgn, payload, len, da_for_pdu1);

  uint8_t seq = (fp_seq++ & 0x1F);

  // Frame 0: [seq, len, data0..5]
  {
    twai_message_t m; memset(&m, 0, sizeof(m));
    m.extd = 1;
    m.identifier = n2kId(N2K_PRIO, pgn, /*SA*/0x23, da_for_pdu1);
    m.data_length_code = 8;
    m.data[0] = seq;
    m.data[1] = len;
    uint8_t copy = min<uint8_t>(6, len);
    memcpy(&m.data[2], &payload[0], copy);
    if (twai_transmit(&m, pdMS_TO_TICKS(20)) != ESP_OK) return false;
  }

  // Frames 1..N: [seq, frame#, next 7 bytes]
  uint8_t sent = 6;
  uint8_t frameNo = 1;
  while (sent < len) {
    twai_message_t m; memset(&m, 0, sizeof(m));
    m.extd = 1;
    m.identifier = n2kId(N2K_PRIO, pgn, /*SA*/0x23, da_for_pdu1);
    m.data_length_code = 8;
    m.data[0] = seq;
    m.data[1] = frameNo++;
    uint8_t remain = len - sent;
    uint8_t take = min<uint8_t>(7, remain);
    memcpy(&m.data[2], &payload[sent], take);
    if (twai_transmit(&m, pdMS_TO_TICKS(20)) != ESP_OK) return false;
    sent += take;
  }
  return true;
}

// ---------- ISO Address Claim (60928) ----------
uint64_t N2K_NAME = 0;
// Helper to build a simple NAME: industry=4(marine), manufacturer=2046, device_class=25, function=130(“SwitchBank”), unique=0x123456
uint64_t buildNAME(){
  // NAME: 64-bit per ISO 11783-3 (NMEA2000 uses same): identity fields packed
  // This is a simple, valid (not unique in real products!) example
  uint64_t name = 0;
  uint32_t manufacturer = 2046;      // 11 bits
  uint32_t unique_id    = 0x123456;  // 21 bits
  uint8_t  device_instance = 0;      // 7 bits (lower+upper)
  uint8_t  device_function = 130;    // 8 bits
  uint8_t  device_class    = 25;     // 7 bits (Electrical Generation? Using 25 as example)
  uint8_t  system_instance = 0;      // 4 bits
  uint8_t  industry_group  = 4;      // 3 bits (Marine)
  uint8_t  arbitrary       = 0;      // 1 bit

  name |= ((uint64_t)unique_id & 0x1FFFFF);                 // bits 0..20
  name |= ((uint64_t)manufacturer & 0x7FF) << 21;           // bits 21..31
  name |= ((uint64_t)device_instance & 0x7F) << 32;         // bits 32..38
  name |= ((uint64_t)device_function & 0xFF) << 39;         // bits 39..46
  name |= ((uint64_t)device_class & 0x7F) << 47;            // bits 47..53
  name |= ((uint64_t)system_instance & 0x0F) << 54;         // bits 54..57
  name |= ((uint64_t)industry_group & 0x07) << 58;          // bits 58..60
  name |= ((uint64_t)arbitrary & 0x01) << 63;               // bit 63
  return name;
}

void n2kSend60928_AddressClaim(){
  uint8_t d[8];
  uint64_t n = N2K_NAME;
  for(int i=0;i<8;i++){ d[i] = (uint8_t)((n >> (i*8)) & 0xFF); } // LSB first
  n2kTransmit(PGN_ISO_ADDRESS_CLAIM, d, 8);
}

// ---------- 127488 Engine Rapid (RPM) ----------
void n2kSend127488(int32_t rpm){
  uint8_t d[8]; memset(d,0xFF,8); d[0]=0; // instance 0
  if (rpm >= 0) { uint16_t raw = (uint16_t)constrain((int32_t)lroundf(rpm/0.25f), 0, 0xFFFE); setU16(&d[1], raw); }
  n2kTransmit(PGN_ENG_RAPID, d, 8);
}

// ---------- 127493 Transmission (gear) ----------
void n2kSend127493(uint8_t leafGear){
  // N2K: 0=Neutral,1=Forward,2=Reverse,3=Unknown
  uint8_t gearEnum = 3;
  if (leafGear==1) gearEnum=1; else if (leafGear==2) gearEnum=0; else if (leafGear==3) gearEnum=2;
  uint8_t d[8]; memset(d,0xFF,8); d[0]=0; d[1]=gearEnum;
  n2kTransmit(PGN_TRANS_DYN, d, 8);
}

// ---------- 127508 Battery (V/A; +A = charging) ----------
void n2kSend127508(float packV, float packA_leaf){
  float A_charge = -packA_leaf; // invert Leaf convention
  uint8_t d[8]; memset(d,0xFF,8); d[0]=0; // batt instance 0
  if (isfinite(packV) && packV>=0){ uint16_t vr=(uint16_t)constrain((long)lroundf(packV/0.01f),0,0xFFFE); setU16(&d[1],vr); }
  if (isfinite(A_charge)){ int16_t ar=(int16_t)constrain((long)lroundf(A_charge/0.1f),-32768,32767); setS16(&d[3],ar); }
  d[7]=0; n2kTransmit(PGN_BATT_STATUS, d, 8);
}

// ---------- 127506 DC Detailed (SOC) ----------
void n2kSend127506(float soc){
  uint8_t d[8]; memset(d,0xFF,8); d[0]=0; d[1]=0; d[2]=0;
  if (isfinite(soc) && soc>=0){ uint16_t sr=(uint16_t)constrain((long)lroundf(soc*10.0f),0,1000); setU16(&d[3],sr); }
  n2kTransmit(PGN_DC_STATUS, d, 8);
}

// ---------- 127489 Engine Dynamic — motor temp -> Coolant Temp (0.1 K) ----------
void n2kSend127489_EngineTemp(float motorTempC) {
  if (!isfinite(motorTempC)) return;
  uint8_t d[26]; memset(d, 0xFF, sizeof(d));
  d[0] = 0; // Engine Instance 0
  float kelvin_tenths = (motorTempC + 273.15f) * 10.0f;
  if (kelvin_tenths >= 0.0f && kelvin_tenths <= 65534.0f) {
    uint16_t raw = (uint16_t)lroundf(kelvin_tenths);
    d[5] = raw & 0xFF; d[6] = raw >> 8; // Engine Coolant Temp
  }
  n2kFastPacketTransmit(PGN_ENG_DYNAMIC, d, sizeof(d));
}

// ---------- 61184 Proprietary A — Regen ----------
void n2kSend61184(uint8_t sid, uint8_t regenLeaf, float packV, float packA_leaf){
  if (!g_hasN2K) return;
  uint8_t state = (regenLeaf<=2)? regenLeaf : 0xFF;
  float I_charge = isfinite(packA_leaf) ? -packA_leaf : NAN; // + = charging
  float P_kW = (isfinite(packV) && isfinite(I_charge)) ? (packV*I_charge/1000.0f) : NAN;
  uint8_t flags=0; if (state!=0xFF) flags|=0x01; if (isfinite(I_charge)) flags|=0x02; if (isfinite(P_kW)) flags|=0x04;

  uint8_t d[8]; memset(d,0xFF,8);
  d[0]=sid; d[1]=0x01; d[2]=(state==0xFF)?0xFF:state; d[3]=flags;
  if (isfinite(I_charge)){ int16_t ir=(int16_t)constrain((long)lroundf(I_charge/0.1f),-32768,32767); setS16(&d[4],ir); }
  if (isfinite(P_kW)){ int16_t pr=(int16_t)constrain((long)lroundf(P_kW/0.1f),-32768,32767); setS16(&d[6],pr); }

  twai_message_t msg; memset(&msg,0,sizeof(msg));
  msg.extd = 1;
  msg.identifier = n2kId(N2K_PRIO, PGN_PROP_A, /*SA*/0x23, N2K_BCAST); // PDU1 with DA
  msg.data_length_code = 8;
  memcpy(msg.data, d, 8);
  twai_transmit(&msg, pdMS_TO_TICKS(20));
}

// ---------- 127501 Switch Status ----------
void n2kSend127501_Status(uint8_t bankInstance, uint32_t bits){
  // Single-frame flavor:
  // Byte0: Bank Instance
  // Bytes1..4: State mask LSB.. (bits 0..27 => channels 1..28)
  // Bytes5..7: Reserved (0xFF)
  uint8_t d[8]; memset(d,0xFF,8);
  d[0] = bankInstance;
  d[1] = (uint8_t)( bits        & 0xFF);
  d[2] = (uint8_t)((bits >> 8)  & 0xFF);
  d[3] = (uint8_t)((bits >> 16) & 0xFF);
  d[4] = (uint8_t)((bits >> 24) & 0xFF);
  n2kTransmit(PGN_SWITCH_STATUS, d, 8);
}

// ---------- 127504 Switch Label (short, 6 chars) ----------
void n2kSend127504_LabelShort(uint8_t bankInstance, uint8_t channel, const char* six){
  // Single-frame flavor (compact):
  // Byte0: Bank Instance
  // Byte1: Channel (1..28)
  // Bytes2..7: ASCII label (6 chars, pad with 0x00 or space)
  uint8_t d[8]; memset(d,0x00,8);
  d[0] = bankInstance;
  d[1] = channel;
  for(int i=0;i<6;i++) d[2+i] = six[i] ? (uint8_t)six[i] : ' ';
  n2kTransmit(PGN_SWITCH_LABEL, d, 8);
}

// ---------- 127502 Switch Control (single frame handler) ----------
void handle127502_Control(const uint8_t* db, uint8_t len){
  if (len < 8) return;
  uint8_t bank = db[0];
  if (bank != SW_BANK_INSTANCE) return; // only our bank
  // Bytes1..4 = SetMask (LSB), Bytes5..7 = ClearMask (24-bit LSB)
  uint32_t setMask = (uint32_t)db[1] | ((uint32_t)db[2]<<8) | ((uint32_t)db[3]<<16) | ((uint32_t)db[4]<<24);
  uint32_t clrMask = (uint32_t)db[5] | ((uint32_t)db[6]<<8) | ((uint32_t)db[7]<<16);
  // Apply to local state (limit to configured channels)
  uint32_t limitMask = (SW_CHANNELS>=32)?0xFFFFFFFFu:((1u<<SW_CHANNELS)-1u);
  setMask &= limitMask; clrMask &= limitMask;

  uint32_t prev = sw_state_bits;
  sw_state_bits |= setMask;
  sw_state_bits &= ~clrMask;

  if (sw_state_bits != prev){
    // Immediately publish status on change
    n2kSend127501_Status(SW_BANK_INSTANCE, sw_state_bits);
  }
}

// ----- Serial readout -----
String gearToStr(uint8_t g);
String regenToStr(uint8_t r);
void printStatusLine(const Telemetry &t){
  float a_charge = isfinite(t.packA) ? -t.packA : NAN; // + = charging
  float p_kw = (isfinite(t.packV) && isfinite(a_charge)) ? (t.packV * a_charge / 1000.0f) : NAN;

  Serial.print("RPM:"); Serial.print(t.rpm);
  Serial.print(" V:");  if (isfinite(t.packV)) Serial.print(t.packV,1); else Serial.print("--");
  Serial.print(" A(+):"); if (isfinite(a_charge)) Serial.print(a_charge,1); else Serial.print("--");
  Serial.print(" kW:"); if (isfinite(p_kw)) Serial.print(p_kw,1); else Serial.print("--");
  Serial.print(" SOC%:"); if (isfinite(t.socPct)) Serial.print(t.socPct,1); else Serial.print("--");
  Serial.print(" TempC:"); if (isfinite(t.motorTempC)) Serial.print(t.motorTempC,1); else Serial.print("--");
  Serial.print(" Gear:"); Serial.print(gearToStr(t.gear));
  Serial.print(" Regen:"); Serial.print(regenToStr(t.regen));
  Serial.print(" | SW bank:"); Serial.print((int)SW_BANK_INSTANCE);
  Serial.print(" bits:0x"); Serial.print(sw_state_bits, HEX);
  Serial.println();
}

// ---------- EV-CAN RX task (low-latency gear TX) ----------
void taskEvRx(void*){
  static uint8_t  lastGear = 0xFF;
  for(;;){
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(10)) == ESP_OK){
      if (m.extd || m.rtr) continue;
      uint16_t id = m.identifier & 0x7FF;
      const uint8_t* b = m.data;

      bool gearJustChanged = false;
      uint8_t gearNow = 0;

      TEL_LOCK();
      if (id==ID_PWR){
        uint16_t rawV = (uint16_t)((b[2]<<2)|(b[3]>>6));
        float V = rawV*0.5f;
        int16_t rawA = (int16_t)((b[0]<<3)|(b[1]>>5)); if (rawA & 0x0400) rawA |= 0xF800;
        float A = -(rawA/2.0f); // Leaf decode: discharge positive
        uint16_t rawSOC = (uint16_t)(b[5]<<8)|b[4];
        telem.packV = V; telem.packA = A; telem.socPct = (float)rawSOC;
      }
      else if (id==ID_RPM){
        uint16_t r = (uint16_t)((b[4]<<8)|b[5]);
        telem.rpm = (int32_t)lroundf(r*0.5f);
      }
      else if (id==ID_TEMP){
        telem.motorTempC = b[5]*0.5f;
      }
      else if (id==ID_GEAR){
        uint8_t g=b[0]; if (g<1||g>3) g=0;
        uint8_t rg=b[5]; if (rg>2) rg=0xFF;
        telem.gear=g; telem.regen=rg;
        gearNow = g;
        gearJustChanged = (g != lastGear);
      }
      TEL_UNLOCK();

      // FAST-PATH: push gear & regen immediately to N2K + Serial gear line
      if (id==ID_GEAR && g_hasN2K){
        n2kSend127493(gearNow);
        TEL_LOCK(); Telemetry t = telem; TEL_UNLOCK();
        n2kSend61184(t.sid++, t.regen, t.packV, t.packA);

        if (gearJustChanged){
          lastGear = gearNow;
          Serial.print("[GEAR] -> ");
          Serial.print(gearToStr(gearNow));
          Serial.print(" | Regen: ");
          Serial.println(regenToStr(t.regen));
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------- N2K TX periodic task ----------
void taskN2kTx(void*){
  const TickType_t period = pdMS_TO_TICKS(100); // 10 Hz
  for(;;){
    if (g_hasN2K){
      TEL_LOCK(); Telemetry t = telem; TEL_UNLOCK();

      n2kSend127488(t.rpm);
      n2kSend127493(t.gear);
      if (isfinite(t.packV) && isfinite(t.packA)) n2kSend127508(t.packV, t.packA);
      if (isfinite(t.socPct)) n2kSend127506(t.socPct);
      if (isfinite(t.motorTempC)) n2kSend127489_EngineTemp(t.motorTempC);
      n2kSend61184(t.sid++, t.regen, t.packV, t.packA);

      // Switch bank status steady rate (1 Hz) plus boot storm (4 Hz for first 10 s)
      uint32_t now = millis();
      if (now < sw_bootStormEndMs) {
        static uint32_t lastStorm=0;
        if (now - lastStorm >= 250){ lastStorm = now; n2kSend127501_Status(SW_BANK_INSTANCE, sw_state_bits); }
      }
    }
    vTaskDelay(period);
  }
}

// ---------- N2K RX task (ISO Req + 127502 Control) ----------
void taskN2kRx(void*){
  for(;;){
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(10)) == ESP_OK){
      if (!m.extd) continue;
      uint32_t pgn = idToPGN(m.identifier);
      uint8_t  da  = idToDA(m.identifier);
      const uint8_t *db = m.data;
      uint8_t len = m.data_length_code;

      // 59904 ISO Request (PDU1: DA is target)
      if (pgn == PGN_ISO_REQUEST && (da==0x23 || da==N2K_BCAST)){
        if (len >= 3){
          uint32_t reqPGN = (uint32_t)db[0] | ((uint32_t)db[1]<<8) | ((uint32_t)db[2]<<16);
          if (reqPGN == PGN_ISO_ADDRESS_CLAIM){
            n2kSend60928_AddressClaim();
          } else if (reqPGN == PGN_SWITCH_STATUS){
            n2kSend127501_Status(SW_BANK_INSTANCE, sw_state_bits);
          } else if (reqPGN == PGN_SWITCH_LABEL) {
            // send a burst of short labels (channels 1..SW_CHANNELS)
            for (uint8_t ch=1; ch<=SW_CHANNELS; ch++){
              n2kSend127504_LabelShort(SW_BANK_INSTANCE, ch, sw_labels[ch]);
            }
          }
        }
      }
      // 127502 Switch Bank Control — single-frame control we defined
      else if (pgn == PGN_SWITCH_CONTROL){
        handle127502_Control(db, len);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------- Serial ticker (2 Hz) ----------
void taskSerialTicker(void*){
  const TickType_t period = pdMS_TO_TICKS(500);
  for(;;){
    TEL_LOCK(); Telemetry t = telem; TEL_UNLOCK();
    printStatusLine(t);
    vTaskDelay(period);
  }
}

// ---------- WebSocket server ----------
void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED: {
      anyClientConnected = true;
      String hello = String("{\"status\":\"Client ") + num + " connected\",\"clients\":" + ws.connectedClients() + ",\"ts_ms\":" + String(millis()) + "}";
      ws.sendTXT(num, hello);
      break;
    }
    case WStype_DISCONNECTED: {
      anyClientConnected = (ws.connectedClients() > 0);
      break;
    }
    case WStype_TEXT: {
      String msg((char*)payload, len);
      String low = msg; low.toLowerCase();

      // set_rate
      if (low.indexOf("set_rate") >= 0) {
        int i = low.indexOf("\"ms\"");
        if (i >= 0) {
          int colon = low.indexOf(":", i);
          if (colon >= 0) {
            int start = colon + 1;
            while (start < (int)low.length() && (low[start]==' '||low[start]=='\"')) start++;
            int end = start;
            while (end < (int)low.length() && isDigit(low[end])) end++;
            uint32_t v = low.substring(start,end).toInt();
            if (v >= 50 && v <= 60000) {
              broadcastIntervalMs = v;
              String ack = String("{\"status\":\"interval set to ") + v + " ms\",\"clients\":" + ws.connectedClients() + ",\"ts_ms\":" + String(millis()) + "}";
              ws.sendTXT(num, ack);
            }
          }
        }
      }

      // switches_sub -> push snapshot
      if (low.indexOf("switches_sub") >= 0) {
        // send snapshot of bank state + labels
        String json = "{";
        json += "\"ts_ms\":" + String(millis());
        json += ",\"bank\":" + String((int)SW_BANK_INSTANCE);
        json += ",\"sw_count\":" + String((int)SW_CHANNELS);
        json += ",\"sw_state\":" + String(sw_state_bits);
        json += ",\"labels\":[";
        for (uint8_t i=1;i<=SW_CHANNELS;i++){
          json += "\""; json += sw_labels[i]; json += "\"";
          if (i<SW_CHANNELS) json += ",";
        }
        json += "]}";
        ws.sendTXT(num, json);
      }

      // {"cmd":"switch","ch":N,"on":true/false}
      if (low.indexOf("\"cmd\":\"switch\"") >= 0){
        // crude parse 'ch' and 'on'
        int j = low.indexOf("\"ch\"");
        int k = low.indexOf("\"on\"");
        if (j>=0 && k>=0){
          int cj = low.indexOf(":", j); int co = low.indexOf(":", k);
          int sj = cj+1; while(sj<(int)low.length() && (low[sj]==' '||low[sj]=='\"')) sj++;
          int so = co+1; while(so<(int)low.length() && (low[so]==' '||low[so]=='\"')) so++;
          int ej=sj; while(ej<(int)low.length() && isDigit(low[ej])) ej++;
          int eo=so; while(eo<(int)low.length() && (isalpha(low[eo])||low[eo]=='t'||low[eo]=='f')) eo++;
          uint8_t ch = (uint8_t)low.substring(sj,ej).toInt();
          bool on = (low.substring(so,eo).indexOf("true")>=0);
          if (ch>=1 && ch<=SW_CHANNELS){
            uint32_t mask = (1u<<(ch-1));
            uint32_t prev = sw_state_bits;
            if (on) sw_state_bits |= mask; else sw_state_bits &= ~mask;
            if (sw_state_bits != prev){
              // publish status immediately
              n2kSend127501_Status(SW_BANK_INSTANCE, sw_state_bits);
            }
          }
        }
      }
      break;
    }
    default: break;
  }
}

// ---------- WebSocket broadcast ----------
void broadcastNow(){
  TEL_LOCK(); Telemetry t=telem; TEL_UNLOCK();
  float a_charge = isfinite(t.packA)? -t.packA : NAN;
  float p_kw = (isfinite(t.packV)&&isfinite(a_charge))? (t.packV*a_charge/1000.0f) : NAN;

  // Dashboard payload
  String json = "{";
  json += "\"ts_ms\":" + String(millis());
  json += ",\"clients\":" + String(ws.connectedClients());
  if (isfinite(t.packV)) json += ",\"v\":" + String(t.packV,1);
  if (isfinite(a_charge)) json += ",\"a\":" + String(a_charge,1);
  if (isfinite(p_kw)) json += ",\"p_kw\":" + String(p_kw,1);
  if (isfinite(t.socPct)) json += ",\"soc\":" + String(t.socPct,1);
  json += ",\"rpm\":" + String(t.rpm);
  if (isfinite(t.motorTempC)) json += ",\"mt_c\":" + String(t.motorTempC,1);
  json += ",\"gear_str\":\"" + gearToStr(t.gear) + "\"";
  json += ",\"regen_str\":\"" + regenToStr(t.regen) + "\"";
  // Switch snapshot (helps /switches too)
  json += ",\"bank\":" + String((int)SW_BANK_INSTANCE);
  json += ",\"sw_count\":" + String((int)SW_CHANNELS);
  json += ",\"sw_state\":" + String(sw_state_bits);
  json += "}";

  ws.broadcastTXT(json);
}

// ---------- Setup / Loop ----------
void setup(){
  Serial.begin(115200);
  delay(200);
  telemMtx = xSemaphoreCreateMutex();

  // NAME & Address Claim
  N2K_NAME = buildNAME();

  // WiFi SoftAP + servers
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, AP_CHAN, false, 8);
  Serial.println(ok ? "SoftAP started" : "SoftAP failed");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  http.on("/", HTTP_GET, handleRoot);
  http.on("/switches", HTTP_GET, handleSwitches);
  http.on("/index.html", HTTP_GET, handleRoot);
  http.onNotFound(handleNotFound);
  http.begin();
  ws.begin();
  ws.onEvent(onWsEvent);

  // Bring up BOTH TWAI controllers
  g_hasEV  = twaiInitPinsSpeedCtrl(EV_TX, EV_RX, 500, TWAI_CONTROLLER_ID_0);
  Serial.println(g_hasEV ? "EV-CAN @500k ready (CTRL0)" : "EV-CAN init FAILED");

  g_hasN2K = twaiInitPinsSpeedCtrl(N2K_TX, N2K_RX, 250, TWAI_CONTROLLER_ID_1);
  Serial.println(g_hasN2K ? "N2K @250k ready (CTRL1)" : "N2K init FAILED");

  // Kick out Address Claim and initial bank status/labels
  if (g_hasN2K){
    n2kSend60928_AddressClaim();
    n2kSend127501_Status(SW_BANK_INSTANCE, sw_state_bits);
    // Push a few labels (all, but it's lightweight)
    for (uint8_t ch=1; ch<=SW_CHANNELS; ch++){
      n2kSend127504_LabelShort(SW_BANK_INSTANCE, ch, sw_labels[ch]);
    }
  }

  // Boot-storm: 10 s at ~4 Hz
  sw_bootStormEndMs = millis() + 10000;

  // Tasks
  xTaskCreatePinnedToCore(taskEvRx,        "EV-RX",   4096, nullptr, 20, nullptr, 0);
  xTaskCreatePinnedToCore(taskN2kTx,       "N2K-TX",  6144, nullptr, 10, nullptr, 1);
  xTaskCreatePinnedToCore(taskN2kRx,       "N2K-RX",  4096, nullptr, 12, nullptr, 1);
  xTaskCreatePinnedToCore(taskSerialTicker,"SERIAL",  3072, nullptr,  2, nullptr, 0);
}

void loop(){
  dnsServer.processNextRequest();
  http.handleClient();
  ws.loop();

  uint32_t now = millis();
  if (anyClientConnected && (now - lastBroadcastMs >= broadcastIntervalMs)) {
    lastBroadcastMs = now;
    broadcastNow();
  }

  // Steady 127501 at 1 Hz outside storm
  if (g_hasN2K && now >= sw_bootStormEndMs){
    static uint32_t lastSteady=0;
    if (now - lastSteady >= 1000){ lastSteady = now; n2kSend127501_Status(SW_BANK_INSTANCE, sw_state_bits); }
  }
}
