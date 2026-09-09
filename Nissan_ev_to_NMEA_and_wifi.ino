/*
  XIAO ESP32-C6 — Dual TWAI v2 (two controllers) + NMEA2000 + Web UI + Switch Bank
  - EV-CAN  @500 kbit/s on TWAI controller 0  (Leaf bus)
  - NMEA2000@250 kbit/s on TWAI controller 1  (N2K bus)
  - Web UI: responsive, two rows, gear/regen show "--" when unknown
  - Switch UI updates ONLY when PGN 127501 is received

  Requirements:
    * Arduino core: esp32 by Espressif Systems 3.2.x (IDF >= 5.4)
    * Library: WebSockets by Markus Sattler
    * Two external CAN transceivers (one per TWAI controller), proper termination
*/

#include <Arduino.h>
#include <math.h>
#include <string.h>

extern "C" {
  #include "driver/twai.h"   // v2 API
}

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WebSocketsServer.h> // arduinoWebSockets

// ======== WiFi / Web ========
const char* AP_SSID = "Reptil_II";
const char* AP_PASS = "12345678"; // >= 8
const byte  DNS_PORT = 53;
const int   AP_CHAN  = 6;

DNSServer        dnsServer;
WebServer        http(80);
WebSocketsServer ws(81);
uint32_t         broadcastIntervalMs = 1000;
uint32_t         lastBroadcastMs = 0;
uint32_t         lastHealthBroadcastMs = 0;
bool             anyClientConnected = false;

// ======== Responsive dashboard (two rows) ========
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover"/>
<meta name="theme-color" content="#0b1220"/>
<meta http-equiv="Cache-Control" content="no-store, no-cache, must-revalidate"/>
<title>XIAO Motor Data</title>
<style>
:root{--bg:#0b1220;--card:#0d1526;--text:#e6eefc;--muted:#8aa0c3;--accent:#56c0ff;--pad:clamp(10px,2.5vw,24px);--gap:clamp(8px,1.5vw,16px)}
*{box-sizing:border-box}html,body{height:100%}
body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Inter,Arial;color:#e6eefc;color:var(--text);background:#0b1220;background:var(--bg);min-height:100vh;padding:12px;padding:calc(env(safe-area-inset-top)+var(--pad)) var(--pad) calc(env(safe-area-inset-bottom)+var(--pad)) var(--pad);display:-webkit-flex;display:flex;align-items:stretch;justify-content:center}
.app{width:100%;max-width:1200px;width:min(1200px,100%);height:100%;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;gap:var(--gap)}
.header{display:-webkit-flex;display:flex;align-items:center;justify-content:space-between;gap:12px}
.title{font-size:clamp(18px,2.6vw,28px);font-weight:700;margin:0}
.badge{background:#0f2236;border:1px solid #20334d;color:var(--accent);padding:6px 12px;border-radius:999px;font-weight:600;white-space:nowrap}
.header-right{display:flex;gap:8px;align-items:center;justify-content:flex-end;flex-wrap:wrap;color:var(--muted);font-size:clamp(12px,1.6vw,14px)}
.motor-alert{background:#351417;border:1px solid #9c343c;color:#ffd9dc;padding:10px 14px;border-radius:8px;font-weight:700}
.net-alert{background:#352610;border:1px solid #9b6a22;color:#ffe3ad;padding:5px 9px;border-radius:6px;font-weight:700;white-space:nowrap}
.motor-alert[hidden],.net-alert[hidden]{display:none}
.card{background:#0d1526;background:var(--card);border:1px solid #1b2740;border-radius:16px;box-shadow:0 8px 30px rgba(0,0,0,.35);width:100%;height:100%;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;padding:12px;padding:var(--pad)}
.grid{
  display:-webkit-flex;
  display:flex;
  -webkit-flex-wrap:wrap;
  flex-wrap:wrap;
  display:grid;
  grid-auto-flow:column;
  grid-template-rows:repeat(2,1fr);
  grid-auto-columns:minmax(180px,1fr);
  gap:var(--gap);
  flex:1 1 auto; min-height:0; height:100%;
}
.tile{background:#0f1726;border:1px solid #1d2a44;border-radius:12px;padding:12px;padding:clamp(10px,1.8vw,14px);display:-webkit-flex;display:flex;-webkit-flex:1 1 180px;flex:1 1 180px;-webkit-flex-direction:column;flex-direction:column;justify-content:center;min-height:90px}
.label{color:var(--muted);font-size:clamp(11px,1.6vw,13px);margin-bottom:6px}
.kv{display:flex;align-items:baseline;gap:8px}
.v{font-size:clamp(22px,5vw,40px);font-weight:800;letter-spacing:.3px;line-height:1.1}
.u{color:var(--muted);font-size:clamp(12px,1.8vw,14px)}
.footer{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin-top:var(--gap)}
.controls{display:flex;gap:8px;align-items:center}
button,input{background:#0e2337;border:1px solid #1d3653;color:var(--text);padding:10px 12px;border-radius:10px;font-size:clamp(12px,1.8vw,14px)}
input{width:min(140px,35vw)}
a{color:var(--accent);text-decoration:none;font-size:clamp(12px,1.8vw,14px)}
@media (max-width:620px){.header{flex-direction:column;align-items:flex-start}.header-right{width:100%;justify-content:flex-start}}
@media (max-width:420px){.grid{grid-auto-columns:minmax(150px,1fr)} .tile{min-height:72px}}
</style></head><body>
<div class="app">
  <div class="header">
    <h1 class="title">XIAO Motor Data</h1>
    <div class="header-right"><span class="net-alert" id="n2k-alert" role="status" hidden>NMEA offline</span><span class="badge" id="status">Connecting…</span><span id="ts">--</span> <a href="/switches">Switches</a></div>
  </div>
  <div class="motor-alert" id="motor-alert" role="alert" hidden>Motor data unavailable</div>
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
(function(){
  function byId(id){ return document.getElementById(id); }
  var statusEl=byId("status"), tsEl=byId("ts"), motorAlert=byId("motor-alert"), n2kAlert=byId("n2k-alert");
  var rpmEl=byId("rpm"), vEl=byId("v"), aEl=byId("a"), pEl=byId("p"), socEl=byId("soc"), mtEl=byId("mt"), gearEl=byId("gear"), regenEl=byId("regen");
  var rateInput=byId("rate"), setBtn=byId("set");
  var host=window.location.hostname||"192.168.4.1";
  var ws=null, pollTimer=null, connectTimer=null;

  function setVisible(el,visible){
    if(!el) return;
    if(visible){ el.removeAttribute("hidden"); el.style.display=""; }
    else{ el.setAttribute("hidden","hidden"); el.style.display="none"; }
  }

  function applyData(j,transport){
    if(typeof j.ts_ms!=="undefined") tsEl.textContent=new Date(j.ts_ms).toLocaleTimeString();
    if(j.rpm_valid===false) rpmEl.textContent="--";
    else if(typeof j.rpm!=="undefined") rpmEl.textContent=Number(j.rpm).toFixed(0);
    if(typeof j.v!=="undefined") vEl.textContent=Number(j.v).toFixed(1);
    if(typeof j.a!=="undefined") aEl.textContent=Number(j.a).toFixed(1);
    if(typeof j.p_kw!=="undefined") pEl.textContent=Number(j.p_kw).toFixed(1);
    if(typeof j.soc!=="undefined") socEl.textContent=Number(j.soc).toFixed(1);
    if(typeof j.mt_c!=="undefined") mtEl.textContent=Number(j.mt_c).toFixed(1);
    if(typeof j.gear_str!=="undefined") gearEl.textContent=(j.gear_str&&j.gear_str.length)?j.gear_str:"--";
    if(typeof j.regen_str!=="undefined") regenEl.textContent=(j.regen_str&&j.regen_str.length)?j.regen_str:"--";
    if(typeof j.health_ready!=="undefined"){
      var ready=!!j.health_ready;
      setVisible(motorAlert,ready&&j.motor_can_ok!==true);
      setVisible(n2kAlert,ready&&j.n2k_ok!==true);
    }
    statusEl.textContent=j.status?j.status:transport;
  }

  function pollOnce(){
    var xhr=new XMLHttpRequest();
    xhr.onreadystatechange=function(){
      if(xhr.readyState!==4) return;
      if(xhr.status===200||xhr.status===0){
        try{ applyData(JSON.parse(xhr.responseText),"Live (HTTP)"); }
        catch(e){ statusEl.textContent="Bad data"; }
      }else statusEl.textContent="Reconnecting...";
    };
    try{
      xhr.open("GET","/api/data?_="+(new Date().getTime()),true);
      xhr.send(null);
    }catch(e){ statusEl.textContent="Reconnecting..."; }
  }

  function startPolling(){
    if(pollTimer!==null) return;
    statusEl.textContent="Connecting (HTTP)...";
    pollOnce();
    pollTimer=window.setInterval(pollOnce,1000);
  }

  function stopPolling(){
    if(pollTimer!==null){ window.clearInterval(pollTimer); pollTimer=null; }
  }

  function connectWebSocket(){
    var SocketClass=window.WebSocket||window.MozWebSocket;
    if(!SocketClass){ startPolling(); return; }
    try{
      ws=new SocketClass("ws://"+host+":81/");
      connectTimer=window.setTimeout(function(){ if(!ws||ws.readyState!==1) startPolling(); },3500);
      ws.onopen=function(){ window.clearTimeout(connectTimer); stopPolling(); statusEl.textContent="Live"; };
      ws.onclose=function(){ startPolling(); };
      ws.onerror=function(){ startPolling(); };
      ws.onmessage=function(ev){ try{ applyData(JSON.parse(ev.data),"Live"); }catch(e){} };
    }catch(e){ startPolling(); }
  }

  function sendRate(ms){
    if(ws&&ws.readyState===1){ ws.send(JSON.stringify({cmd:"set_rate",ms:ms})); return; }
    var xhr=new XMLHttpRequest();
    try{
      xhr.open("POST","/api/rate",true);
      xhr.setRequestHeader("Content-Type","application/x-www-form-urlencoded");
      xhr.send("ms="+encodeURIComponent(ms));
    }catch(e){}
  }

  setBtn.onclick=function(){
    var value=parseInt(rateInput.value||"1000",10);
    if(isFinite(value)&&value>=50&&value<=60000) sendRate(value);
  };
  connectWebSocket();
})();
</script>
</body></html>
)HTML";

// ======== Switches page ========
const char SWITCHES_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover"/>
<meta name="theme-color" content="#0b1220"/>
<meta http-equiv="Cache-Control" content="no-store, no-cache, must-revalidate"/>
<title>XIAO Switches</title>
<style>
:root{--bg:#0b1220;--card:#0d1526;--text:#e6eefc;--muted:#8aa0c3;--accent:#56c0ff;--pad:clamp(10px,2.5vw,24px);--gap:clamp(8px,1.5vw,16px)}
*{box-sizing:border-box}html,body{height:100%}
body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Inter,Arial;color:#e6eefc;color:var(--text);background:#0b1220;background:var(--bg);min-height:100vh;padding:12px;padding:calc(env(safe-area-inset-top)+var(--pad)) var(--pad) calc(env(safe-area-inset-bottom)+var(--pad)) var(--pad);display:-webkit-flex;display:flex;align-items:stretch;justify-content:center}
.app{width:100%;max-width:1200px;width:min(1200px,100%);height:100%;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;gap:var(--gap)}
.header{display:flex;align-items:center;justify-content:space-between;gap:12px}
.title{font-size:clamp(18px,2.6vw,28px);font-weight:700;margin:0}
.badge{background:#0f2236;border:1px solid #20334d;color:var(--accent);padding:6px 12px;border-radius:999px;font-weight:600;white-space:nowrap}
.header-meta{display:flex;gap:8px;align-items:center;justify-content:flex-end;flex-wrap:wrap}
.net-alert{background:#352610;border:1px solid #9b6a22;color:#ffe3ad;padding:5px 9px;border-radius:6px;font-weight:700;white-space:nowrap}
.net-alert[hidden]{display:none}
a{color:var(--accent);text-decoration:none;font-size:clamp(12px,1.8vw,14px)}
.card{background:#0d1526;background:var(--card);border:1px solid #1b2740;border-radius:16px;box-shadow:0 8px 30px rgba(0,0,0,.35);width:100%;height:100%;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;padding:12px;padding:var(--pad)}
.grid{display:-webkit-flex;display:flex;-webkit-flex-wrap:wrap;flex-wrap:wrap;display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:var(--gap);margin-top:var(--gap);flex:1 1 auto;min-height:0}
.switch{background:#0f1726;border:1px solid #1d2a44;border-radius:12px;padding:14px;padding:clamp(12px,2vw,16px);display:-webkit-flex;display:flex;-webkit-flex:1 1 140px;flex:1 1 140px;-webkit-flex-direction:column;flex-direction:column;gap:8px}
.label{color:var(--muted);font-size:clamp(11px,1.6vw,13px)}
.btn{display:flex;align-items:center;justify-content:center;border-radius:10px;padding:12px;font-weight:700;cursor:pointer;border:1px solid #1d3653;background:#0e2337;color:var(--text);font-size:clamp(14px,2vw,16px)}
.btn.on{background:#1b3b1f;border-color:#2a5c35}
.footer{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin-top:var(--gap)}
.small{color:var(--muted);font-size:clamp(12px,1.6vw,13px)}
@media (max-width:620px){.header{flex-direction:column;align-items:flex-start}.header-meta{width:100%;justify-content:flex-start}}
</style></head><body>
<div class="app">
  <div class="header">
    <h1 class="title">Switches</h1>
    <div class="small header-meta"><span class="net-alert" id="n2k-alert" role="status" hidden>NMEA offline</span><span class="badge" id="status">Connecting…</span><a href="/">Dashboard</a></div>
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
(function(){
  function byId(id){ return document.getElementById(id); }
  var statusEl=byId("status"), grid=byId("grid"), tsEl=byId("ts"), bankEl=byId("bank"), countEl=byId("count"), n2kAlert=byId("n2k-alert");
  var host=window.location.hostname||"192.168.4.1";
  var ws=null, pollTimer=null, connectTimer=null;
  var labels=[], states=0, n=0, bank=0;

  function setVisible(el,visible){
    if(!el) return;
    if(visible){ el.removeAttribute("hidden"); el.style.display=""; }
    else{ el.setAttribute("hidden","hidden"); el.style.display="none"; }
  }

  function sendSwitch(channel,on){
    if(ws&&ws.readyState===1){ ws.send(JSON.stringify({cmd:"switch",ch:channel,on:on})); return; }
    var xhr=new XMLHttpRequest();
    try{
      xhr.open("POST","/api/switch",true);
      xhr.setRequestHeader("Content-Type","application/x-www-form-urlencoded");
      xhr.send("ch="+encodeURIComponent(channel)+"&on="+(on?"1":"0"));
    }catch(e){}
  }

  function switchHandler(channel,on){ return function(){ sendSwitch(channel,!on); }; }

  function draw(){
    grid.innerHTML="";
    var i,lab,on,el,labelEl,buttonEl;
    for(i=1;i<=n;i++){
      lab=labels[i-1]||("SW "+i);
      on=((states>>(i-1))&1)===1;
      el=document.createElement("div"); el.className="switch";
      labelEl=document.createElement("div"); labelEl.className="label"; labelEl.textContent=lab; el.appendChild(labelEl);
      buttonEl=document.createElement("button"); buttonEl.className="btn"+(on?" on":""); buttonEl.textContent=on?"ON":"OFF";
      buttonEl.onclick=switchHandler(i,on);
      el.appendChild(buttonEl); grid.appendChild(el);
    }
    countEl.textContent=n; bankEl.textContent=bank;
  }

  function applyData(j,transport){
    if(typeof j.ts_ms!=="undefined") tsEl.textContent=new Date(j.ts_ms).toLocaleTimeString();
    if(typeof j.bank!=="undefined") bank=j.bank;
    if(typeof j.sw_count!=="undefined") n=j.sw_count;
    if(typeof j.sw_state!=="undefined") states=j.sw_state>>>0;
    if(j.labels) labels=j.labels;
    if(typeof j.health_ready!=="undefined") setVisible(n2kAlert,!!j.health_ready&&j.n2k_ok!==true);
    statusEl.textContent=j.status?j.status:transport;
    draw();
  }

  function pollOnce(){
    var xhr=new XMLHttpRequest();
    xhr.onreadystatechange=function(){
      if(xhr.readyState!==4) return;
      if(xhr.status===200||xhr.status===0){
        try{ applyData(JSON.parse(xhr.responseText),"Live (HTTP)"); }
        catch(e){ statusEl.textContent="Bad data"; }
      }else statusEl.textContent="Reconnecting...";
    };
    try{ xhr.open("GET","/api/switches?_="+(new Date().getTime()),true); xhr.send(null); }
    catch(e){ statusEl.textContent="Reconnecting..."; }
  }

  function startPolling(){
    if(pollTimer!==null) return;
    statusEl.textContent="Connecting (HTTP)...";
    pollOnce(); pollTimer=window.setInterval(pollOnce,1000);
  }

  function stopPolling(){ if(pollTimer!==null){ window.clearInterval(pollTimer); pollTimer=null; } }

  function connectWebSocket(){
    var SocketClass=window.WebSocket||window.MozWebSocket;
    if(!SocketClass){ startPolling(); return; }
    try{
      ws=new SocketClass("ws://"+host+":81/");
      connectTimer=window.setTimeout(function(){ if(!ws||ws.readyState!==1) startPolling(); },3500);
      ws.onopen=function(){ window.clearTimeout(connectTimer); stopPolling(); statusEl.textContent="Live"; ws.send(JSON.stringify({cmd:"switches_sub"})); };
      ws.onclose=function(){ startPolling(); };
      ws.onerror=function(){ startPolling(); };
      ws.onmessage=function(ev){ try{ applyData(JSON.parse(ev.data),"Live"); }catch(e){} };
    }catch(e){ startPolling(); }
  }

  connectWebSocket();
})();
</script>
</body></html>
)HTML";

// ---------- Captive portal helpers ----------
bool captivePortal() {
  if (!http.hostHeader().equals(WiFi.softAPIP().toString())) {
    http.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
    http.send(302, "text/plain", "");
    return true;
  }
  return false;
}
void sendHtmlNoCache(const char *html){
  http.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  http.sendHeader("Pragma","no-cache");
  http.sendHeader("Expires","0");
  http.send_P(200,"text/html; charset=utf-8",html);
}
void handleRoot(){ sendHtmlNoCache(INDEX_HTML); }
void handleSwitches(){ sendHtmlNoCache(SWITCHES_HTML); }
void handleNotFound(){ if (captivePortal()) return; http.send(404, "text/plain", "Not found"); }

// ======== CAN pins (Seeed XIAO ESP32-C6 header labels) ========
// Seeed pin map: D2=GPIO2, D3=GPIO21, D9=GPIO20, D10=GPIO18.
// Raw GPIO3 is not header pin D3 on this board; it is used for RF switch power.
// EV-CAN (TWAI controller 0) - currently using the verified working CAN board
#define EV_TX  ((gpio_num_t)D9)
#define EV_RX  ((gpio_num_t)D10)
// NMEA2000 (TWAI controller 1) - moved off D9/D10 so the controllers do not share pins
#define N2K_TX ((gpio_num_t)D2)
#define N2K_RX ((gpio_num_t)D3)

// ======== EV-CAN IDs ========
static const uint16_t ID_PWR  = 0x1DB; // V/I/SOC
static const uint16_t ID_RPM  = 0x1DA; // RPM
static const uint16_t ID_TEMP = 0x55A; // Motor temp
static const uint16_t ID_GEAR = 0x539; // Gear + Regen

// ======== N2K PGNs ========
static const uint8_t  N2K_SA   = 0x23;
static const uint8_t  N2K_PRIO = 6;
static const uint8_t  N2K_BCAST= 0xFF;

static const uint32_t PGN_ISO_ADDRESS_CLAIM = 60928;
static const uint32_t PGN_ISO_REQUEST       = 59904;
static const uint32_t PGN_ENG_RAPID   = 127488;
static const uint32_t PGN_TRANS_DYN   = 127493;
static const uint32_t PGN_FLUID_LEVEL = 127505;
static const uint32_t PGN_BATT_STATUS = 127508;
static const uint32_t PGN_DC_STATUS   = 127506;
static const uint32_t PGN_DC_VI       = 127751; // DC Voltage/Current (0.1 V, 0.01 A)
static const uint32_t PGN_ENG_DYNAMIC = 127489; // fast-packet (Engine Dynamic)
static const uint32_t PGN_PROP_A      = 61184;  // Proprietary A (regen)
static const uint32_t PGN_SWITCH_STATUS = 127501;
static const uint32_t PGN_SWITCH_CONTROL= 127502;
static const uint32_t PGN_SWITCH_LABEL  = 127504;

// Virtual fuel tank for MFDs that render tank level more readily than battery SOC.
static const uint8_t  SOC_FUEL_TANK_INSTANCE = 0;
static const float    SOC_FUEL_TANK_CAPACITY_L = 100.0f;
static const uint32_t SOC_FUEL_TANK_PERIOD_MS = 1000;

// ======== Telemetry ========
typedef struct {
  float   packV = NAN;
  float   packA = NAN;    // Leaf decode DISCHARGE POS; invert when publishing charge
  float   socPct = NAN;
  int32_t rpm = 0;
  bool    rpmValid = false;
  float   motorTempC = NAN;
  uint8_t gear = 0;       // 1=Drive,2=Neutral,3=Reverse; else 0
  uint8_t regen = 0xFF;   // 0..2; else 0xFF
  uint8_t sid = 0;
} Telemetry;
Telemetry telem;

// ======== Switch Bank ========
static const uint8_t  SW_BANK_INSTANCE = 0;
static const uint8_t  SW_CHANNELS      = 16;
static const uint8_t  SW_CONTROL_CHANNELS = (SW_CHANNELS > 24) ? 24 : SW_CHANNELS;
static const uint8_t  SW_TARGET_SA     = N2K_SA; // change if your bank is another SA
volatile uint32_t sw_state_bits = 0;             // UI-visible (only from 127501)

char sw_labels[29][7] = {
  "", "NAV","ANCHOR","CABIN","DECK","BILGE","WIPER","FAN","PUMP",
  "AUX1","AUX2","AUX3","AUX4","AUX5","AUX6","AUX7","AUX8",
  "A17","A18","A19","A20","A21","A22","A23","A24","A25","A26","A27","A28"
};

uint32_t bank_state_bits = 0;   // if this node acts as bank too
uint32_t sw_bootStormEndMs = 0;

// ======== Concurrency ========
SemaphoreHandle_t telemMtx;
SemaphoreHandle_t n2kTxMtx;
static inline void TEL_LOCK(){ xSemaphoreTake(telemMtx, portMAX_DELAY); }
static inline void TEL_UNLOCK(){ xSemaphoreGive(telemMtx); }
uint8_t nextSid(){ TEL_LOCK(); uint8_t sid=telem.sid++; TEL_UNLOCK(); return sid; }

// ======== TWAI v2 handles ========
twai_handle_t hEV  = nullptr;  // controller 0
twai_handle_t hN2K = nullptr;  // controller 1

// ======== Bus health ========
static const uint32_t HEALTH_STARTUP_GRACE_MS = 5000;
static const uint32_t MOTOR_DATA_TIMEOUT_MS   = 2500;
static const uint32_t N2K_ACTIVITY_TIMEOUT_MS = 3000;
static const uint32_t HEALTH_BROADCAST_MS     = 1000;

volatile uint32_t lastMotorDataMs   = 0;
volatile uint32_t lastN2kActivityMs = 0;
volatile bool n2kControllerRunning  = false;
volatile uint32_t n2kTxQueuedFrames = 0;
volatile uint32_t n2kTxDroppedFrames = 0;
volatile uint32_t n2kFuelTankTxFrames = 0;
volatile uint32_t n2kRxFrames = 0;
volatile uint32_t n2kHwTxFailedCount = 0;
volatile uint32_t n2kHwBusErrorCount = 0;
volatile uint32_t n2kHwTxErrorCounter = 0;
volatile uint32_t n2kHwRxErrorCounter = 0;
volatile uint32_t n2kBusOffCount = 0;
uint32_t healthStartMs = 0;
bool n2kRecoveryPending = false;

// ======== Helpers ========
static inline void setU16(uint8_t *d, uint16_t v){ d[0]=v&0xFF; d[1]=v>>8; }
static inline void setS16(uint8_t *d, int16_t  v){ d[0]=v&0xFF; d[1]=(uint16_t)v>>8; }
static inline void setS24(uint8_t *d, int32_t v){ uint32_t u=(uint32_t)v&0xFFFFFFu; d[0]=u&0xFF; d[1]=(u>>8)&0xFF; d[2]=(u>>16)&0xFF; }
static inline void setU32(uint8_t *d, uint32_t v){ d[0]=v&0xFF; d[1]=(v>>8)&0xFF; d[2]=(v>>16)&0xFF; d[3]=(v>>24)&0xFF; }
String gearToStr(uint8_t g){ return (g==1)?"DRIVE":(g==2)?"NEUTRAL":(g==3)?"REVERSE":"UNKNOWN"; }
String regenToStr(uint8_t r){ return (r==0)?"Off":(r==1)?"OneBar":(r==2)?"TwoBar":"Unknown"; }
bool healthChecksReady(uint32_t now){ return (uint32_t)(now-healthStartMs)>=HEALTH_STARTUP_GRACE_MS; }
bool motorDataHealthy(uint32_t now){ uint32_t last=lastMotorDataMs; return last!=0 && (uint32_t)(now-last)<=MOTOR_DATA_TIMEOUT_MS; }
bool n2kNetworkHealthy(uint32_t now){ uint32_t last=lastN2kActivityMs; return n2kControllerRunning && last!=0 && (uint32_t)(now-last)<=N2K_ACTIVITY_TIMEOUT_MS; }

bool decodeLeafMotorRpm(const uint8_t *b, int32_t &rpm){
  // DBC: 0x1DA bits 39|15, Motorola, signed, 1 rpm/bit. The unused LSB is discarded.
  uint16_t raw=(uint16_t)(((uint16_t)b[4]<<7)|(b[5]>>1));
  if(raw==0x7FFF || raw==0x7FFE){ rpm=0; return false; } // startup/unavailable values
  int16_t signedRaw=(raw&0x4000)?(int16_t)(raw|0x8000):(int16_t)raw;
  rpm=(int32_t)signedRaw;
  return true;
}

// 29-bit ID builders/parsers
static uint32_t n2kId(uint8_t prio, uint32_t pgn, uint8_t sa, uint8_t da_for_pdu1 = 0xFF) {
  uint8_t PF = (pgn >> 8) & 0xFF;
  uint8_t DP = (pgn >> 16) & 0x01;
  if (PF >= 240) { // PDU2
    return ((uint32_t)prio<<26)|((uint32_t)DP<<24)|((uint32_t)PF<<16)|((uint32_t)(pgn&0xFF)<<8)|sa;
  } else { // PDU1
    return ((uint32_t)prio<<26)|((uint32_t)DP<<24)|((uint32_t)PF<<16)|((uint32_t)da_for_pdu1<<8)|sa;
  }
}
static inline uint32_t idToPGN(uint32_t id){
  uint8_t PF = (id >> 16) & 0xFF;
  if (PF < 240) return ((id >> 8) & 0x1FF00);
  else          return ((id >> 8) & 0x1FFFF);
}
static inline uint8_t idToDA(uint32_t id){ return (id >> 8) & 0xFF; }

// ======== N2K TX (uses hN2K) ========
uint8_t n2kPriorityForPgn(uint32_t pgn){
  if(pgn==PGN_ENG_RAPID || pgn==PGN_ENG_DYNAMIC || pgn==PGN_TRANS_DYN) return 2;
  if(pgn==PGN_SWITCH_STATUS || pgn==PGN_SWITCH_CONTROL) return 3;
  return N2K_PRIO;
}

bool n2kQueueFrameUnlocked(twai_message_t &msg){
  bool ok=twai_transmit_v2(hN2K,&msg,pdMS_TO_TICKS(20))==ESP_OK;
  if(ok) n2kTxQueuedFrames=n2kTxQueuedFrames+1; else n2kTxDroppedFrames=n2kTxDroppedFrames+1;
  return ok;
}

bool n2kTransmitFrameUnlocked(uint32_t pgn,const uint8_t *payload,uint8_t len,uint8_t da_for_pdu1){
  twai_message_t msg{}; msg.extd=1; msg.rtr=0; msg.ss=0;
  msg.identifier=n2kId(n2kPriorityForPgn(pgn),pgn,N2K_SA,da_for_pdu1);
  msg.data_length_code=len;
  memcpy(msg.data,payload,len);
  return n2kQueueFrameUnlocked(msg);
}

bool n2kTransmit(uint32_t pgn, const uint8_t *payload, uint8_t len, uint8_t da_for_pdu1 = 0xFF) {
  if(!hN2K || !payload || len>8) return false;
  if(n2kTxMtx && xSemaphoreTake(n2kTxMtx,pdMS_TO_TICKS(100))!=pdTRUE){ n2kTxDroppedFrames=n2kTxDroppedFrames+1; return false; }
  bool ok=n2kTransmitFrameUnlocked(pgn,payload,len,da_for_pdu1);
  if(n2kTxMtx) xSemaphoreGive(n2kTxMtx);
  return ok;
}

static uint8_t fp_seq = 0;
bool n2kFastPacketTransmit(uint32_t pgn, const uint8_t *payload, uint8_t len, uint8_t da_for_pdu1 = 0xFF) {
  if(!hN2K || !payload || len==0) return false;
  if(len<=8) return n2kTransmit(pgn,payload,len,da_for_pdu1);
  if(len>223) return false;
  if(n2kTxMtx && xSemaphoreTake(n2kTxMtx,pdMS_TO_TICKS(100))!=pdTRUE){ n2kTxDroppedFrames=n2kTxDroppedFrames+1; return false; }

  const uint8_t sequence=(fp_seq++&0x07);
  const uint32_t identifier=n2kId(n2kPriorityForPgn(pgn),pgn,N2K_SA,da_for_pdu1);
  bool ok=true;

  twai_message_t frame{}; frame.extd=1; frame.rtr=0; frame.ss=0;
  frame.identifier=identifier; frame.data_length_code=8;
  memset(frame.data,0xFF,8);
  frame.data[0]=(uint8_t)(sequence<<5); // sequence in bits 7..5, frame counter 0
  frame.data[1]=len;
  memcpy(&frame.data[2],payload,6);
  ok=n2kQueueFrameUnlocked(frame);

  uint8_t sent=6;
  uint8_t frameNumber=1;
  while(ok && sent<len){
    memset(frame.data,0xFF,8);
    frame.data[0]=(uint8_t)((sequence<<5)|(frameNumber&0x1F));
    uint8_t count=min<uint8_t>(7,(uint8_t)(len-sent));
    memcpy(&frame.data[1],payload+sent,count);
    ok=n2kQueueFrameUnlocked(frame);
    sent+=count;
    frameNumber++;
  }

  if(n2kTxMtx) xSemaphoreGive(n2kTxMtx);
  return ok;
}

// ======== NAME / Address Claim ========
uint64_t N2K_NAME=0;
uint64_t buildNAME(){
  uint64_t name=0;
  uint32_t manufacturer=2046, unique_id=0x123456;
  uint8_t device_instance_lower=0, function_instance=0, device_function=130;
  uint8_t device_class=25, system_instance=0, industry_group=4, arbitrary=0;
  name |= ((uint64_t)unique_id & 0x1FFFFF);
  name |= ((uint64_t)manufacturer & 0x7FF) << 21;
  name |= ((uint64_t)device_instance_lower & 0x07) << 32;
  name |= ((uint64_t)function_instance & 0x1F) << 35;
  name |= ((uint64_t)device_function & 0xFF) << 40;
  name |= ((uint64_t)device_class & 0x7F) << 49;
  name |= ((uint64_t)system_instance & 0x0F) << 56;
  name |= ((uint64_t)industry_group & 0x07) << 60;
  name |= ((uint64_t)arbitrary & 0x01) << 63;
  return name;
}
void n2kSend60928_AddressClaim(){
  uint8_t d[8]; uint64_t n=N2K_NAME; for(int i=0;i<8;i++) d[i]=(uint8_t)((n>>(i*8))&0xFF);
  n2kTransmit(PGN_ISO_ADDRESS_CLAIM, d, 8);
}

// ======== Engine PGNs ========
void n2kSend127488(int32_t rpm,bool valid){
  uint8_t d[8]; memset(d,0xFF,8); d[0]=0;
  if(valid){
    uint32_t magnitude=(rpm<0)?(uint32_t)(-(int64_t)rpm):(uint32_t)rpm;
    uint32_t raw=(uint32_t)lroundf((float)magnitude/0.25f);
    if(raw<=0xFFFEu) setU16(&d[1],(uint16_t)raw);
  }
  n2kTransmit(PGN_ENG_RAPID,d,8);
}

void n2kSend127493(uint8_t leafGear){
  uint8_t gear=3; // NMEA: Forward=0, Neutral=1, Reverse=2, Unknown=3
  if(leafGear==1) gear=0;
  else if(leafGear==2) gear=1;
  else if(leafGear==3) gear=2;
  uint8_t d[8]; memset(d,0xFF,8); d[0]=0; d[1]=(uint8_t)(0xFC|gear);
  n2kTransmit(PGN_TRANS_DYN,d,8);
}

void n2kSend127505_FuelSoc(float soc){
  if(!isfinite(soc)) return;
  if(soc<0) soc=0; else if(soc>100) soc=100;

  uint8_t d[8]; memset(d,0xFF,8);
  d[0]=SOC_FUEL_TANK_INSTANCE&0x0F; // upper nibble 0 = Fuel
  setS16(&d[1],(int16_t)lroundf(soc/0.004f));
  setU32(&d[3],(uint32_t)lroundf(SOC_FUEL_TANK_CAPACITY_L/0.1f));
  if(n2kTransmit(PGN_FLUID_LEVEL,d,8)) n2kFuelTankTxFrames=n2kFuelTankTxFrames+1;
}

void n2kSend127508(float V,float A_leaf){
  float A_ch=-A_leaf;
  uint8_t d[8]; memset(d,0xFF,8); d[0]=0;
  setS16(&d[1],INT16_MAX); // signed 0.01 V field; Nissan HV exceeds its valid range
  setS16(&d[3],INT16_MAX);
  if(isfinite(V) && V>=-327.68f && V<=327.65f) setS16(&d[1],(int16_t)lroundf(V/0.01f));
  if(isfinite(A_ch)){
    int32_t raw=lroundf(A_ch/0.1f);
    if(raw<-32768) raw=-32768; else if(raw>32765) raw=32765;
    setS16(&d[3],(int16_t)raw);
  }
  d[7]=nextSid();
  n2kTransmit(PGN_BATT_STATUS,d,8);
}

void n2kSend127751(float V,float A_leaf){
  float A_ch=-A_leaf;
  uint8_t d[8]; memset(d,0xFF,8); d[0]=nextSid(); d[1]=0;
  setS16(&d[2],INT16_MAX);
  setS24(&d[4],0x7FFFFF);
  if(isfinite(V)){
    int32_t raw=lroundf(V/0.1f);
    if(raw>=-32768 && raw<=32765) setS16(&d[2],(int16_t)raw);
  }
  if(isfinite(A_ch)){
    int32_t raw=lroundf(A_ch/0.01f);
    if(raw<-8388608) raw=-8388608; else if(raw>8388605) raw=8388605;
    setS24(&d[4],raw);
  }
  n2kTransmit(PGN_DC_VI,d,8);
}

void n2kSend127506(float soc){
  uint8_t d[11]; memset(d,0xFF,sizeof(d));
  d[0]=nextSid(); d[1]=0; d[2]=0; // SID, DC instance 0, Battery
  if(isfinite(soc) && soc>=0){
    int32_t raw=lroundf(soc);
    if(raw<0) raw=0; else if(raw>100) raw=100;
    d[3]=(uint8_t)raw;
  }
  n2kFastPacketTransmit(PGN_DC_STATUS,d,sizeof(d));
}

void n2kSend127489_EngineTemp(float mtC){
  if(!isfinite(mtC)) return;
  uint8_t d[26]; memset(d,0xFF,sizeof(d)); d[0]=0;
  float rawK=(mtC+273.15f)*100.0f;
  if(rawK>=0 && rawK<=65534){ uint16_t raw=(uint16_t)lroundf(rawK); setU16(&d[5],raw); }
  n2kFastPacketTransmit(PGN_ENG_DYNAMIC,d,sizeof(d));
}
// Proprietary A (regen + Icharge + PkW)
void n2kSend61184(uint8_t sid,uint8_t regen,float V,float A_leaf){
  uint8_t state=(regen<=2)?regen:0xFF; float A_ch=isfinite(A_leaf)?-A_leaf:NAN; float P_kW=(isfinite(V)&&isfinite(A_ch))?(V*A_ch/1000.0f):NAN;
  uint8_t flags=0; if(state!=0xFF)flags|=0x01; if(isfinite(A_ch))flags|=0x02; if(isfinite(P_kW))flags|=0x04;
  uint8_t d[8]; memset(d,0xFF,8); d[0]=sid; d[1]=0x01; d[2]=(state==0xFF)?0xFF:state; d[3]=flags;
  if(isfinite(A_ch)){int16_t ir=(int16_t)constrain((long)lroundf(A_ch/0.1f),-32768,32767); setS16(&d[4],ir);}
  if(isfinite(P_kW)){int16_t pr=(int16_t)constrain((long)lroundf(P_kW/0.1f),-32768,32767); setS16(&d[6],pr);}
  n2kTransmit(PGN_PROP_A,d,8,N2K_BCAST);
}

// ======== Switch PGNs ========
void n2kSend127501_Status(uint8_t bank,uint32_t bits){ uint8_t d[8]; memset(d,0xFF,8); d[0]=bank; d[1]=bits&0xFF; d[2]=(bits>>8)&0xFF; d[3]=(bits>>16)&0xFF; d[4]=(bits>>24)&0xFF; n2kTransmit(PGN_SWITCH_STATUS,d,8); }
void n2kSend127504_LabelShort(uint8_t bank,uint8_t ch,const char* six){ uint8_t d[8]; memset(d,0,8); d[0]=bank; d[1]=ch; for(int i=0;i<6;i++) d[2+i]=six[i]? (uint8_t)six[i] : ' '; n2kTransmit(PGN_SWITCH_LABEL,d,8); }
void n2kSend127502_Control(uint8_t bank,uint32_t setMask,uint32_t clrMask,uint8_t targetSA){
  uint32_t controlLimit=(SW_CONTROL_CHANNELS>=32)?0xFFFFFFFFu:((1u<<SW_CONTROL_CHANNELS)-1u);
  setMask&=controlLimit; clrMask&=controlLimit;
  uint8_t d[8]; memset(d,0,8); d[0]=bank; d[1]=setMask&0xFF; d[2]=(setMask>>8)&0xFF; d[3]=(setMask>>16)&0xFF; d[4]=(setMask>>24)&0xFF; d[5]=clrMask&0xFF; d[6]=(clrMask>>8)&0xFF; d[7]=(clrMask>>16)&0xFF;
  n2kTransmit(PGN_SWITCH_CONTROL,d,8,targetSA);
}

// UI push on 127501
void pushSwitchStateToWeb(uint32_t bits){
  String json=String("{\"ts_ms\":")+String(millis())+
              ",\"bank\":"+String((int)SW_BANK_INSTANCE)+
              ",\"sw_count\":"+String((int)SW_CHANNELS)+
              ",\"sw_state\":"+String(bits)+"}";
  ws.broadcastTXT(json);
}
void on127501_StatusRx(uint8_t bank,uint32_t bits){ if(bank!=SW_BANK_INSTANCE) return; sw_state_bits=bits; pushSwitchStateToWeb(bits); }

// If we act as BANK, apply 127502 and publish 127501
void applyControlAsBankAndPublish(const uint8_t* db,uint8_t len){
  if(len<8||db[0]!=SW_BANK_INSTANCE) return;
  uint32_t setMask=(uint32_t)db[1]|((uint32_t)db[2]<<8)|((uint32_t)db[3]<<16)|((uint32_t)db[4]<<24);
  uint32_t clrMask=(uint32_t)db[5]|((uint32_t)db[6]<<8)|((uint32_t)db[7]<<16);
  uint32_t limit=(SW_CONTROL_CHANNELS>=32)?0xFFFFFFFFu:((1u<<SW_CONTROL_CHANNELS)-1u);
  setMask&=limit; clrMask&=limit;
  uint32_t prev=bank_state_bits; bank_state_bits|=setMask; bank_state_bits&=~clrMask;
  if(bank_state_bits!=prev){ n2kSend127501_Status(SW_BANK_INSTANCE,bank_state_bits); on127501_StatusRx(SW_BANK_INSTANCE,bank_state_bits); }
}

// ======== Serial readout ========
void printStatusLine(){
  TEL_LOCK(); Telemetry t=telem; TEL_UNLOCK();
  float A_ch=isfinite(t.packA)? -t.packA : NAN;
  float p_kw=(isfinite(t.packV)&&isfinite(A_ch))? (t.packV*A_ch/1000.0f):NAN;
  Serial.print("RPM:"); if(t.rpmValid) Serial.print(t.rpm); else Serial.print("--");
  Serial.print(" V:"); if(isfinite(t.packV)) Serial.print(t.packV,1); else Serial.print("--");
  Serial.print(" A(+):"); if(isfinite(A_ch)) Serial.print(A_ch,1); else Serial.print("--");
  Serial.print(" kW:"); if(isfinite(p_kw)) Serial.print(p_kw,1); else Serial.print("--");
  Serial.print(" SOC%:"); if(isfinite(t.socPct)) Serial.print(t.socPct,1); else Serial.print("--");
  Serial.print(" TempC:"); if(isfinite(t.motorTempC)) Serial.print(t.motorTempC,1); else Serial.print("--");
  Serial.print(" Gear:"); Serial.print((t.gear>=1&&t.gear<=3)? gearToStr(t.gear) : String("--"));
  Serial.print(" Regen:"); Serial.print((t.regen<=2)? regenToStr(t.regen) : String("--"));
  uint32_t now=millis(); bool ready=healthChecksReady(now);
  Serial.print(" | SW bits:0x"); Serial.print(sw_state_bits,HEX);
  Serial.print(" MotorCAN:"); Serial.print(!ready?"WAIT":(motorDataHealthy(now)?"OK":"NO DATA"));
  Serial.print(" N2K:"); Serial.print(!ready?"WAIT":(n2kNetworkHealthy(now)?"OK":"OFFLINE"));
  Serial.printf(" [txq:%lu drop:%lu tank:%lu rx:%lu hwfail:%lu buserr:%lu TEC:%lu REC:%lu BO:%lu]\n",
    (unsigned long)n2kTxQueuedFrames,(unsigned long)n2kTxDroppedFrames,(unsigned long)n2kFuelTankTxFrames,(unsigned long)n2kRxFrames,
    (unsigned long)n2kHwTxFailedCount,(unsigned long)n2kHwBusErrorCount,
    (unsigned long)n2kHwTxErrorCounter,(unsigned long)n2kHwRxErrorCounter,(unsigned long)n2kBusOffCount);
}

// ======== EV-CAN RX task (uses hEV) ========
void taskEvRx(void*){
  static uint8_t lastGear=0xFF;
  for(;;){
    twai_message_t m;
    if(hEV && twai_receive_v2(hEV,&m,pdMS_TO_TICKS(10))==ESP_OK){
      if(m.extd || m.rtr || m.data_length_code<6) continue;
      uint16_t id = m.identifier & 0x7FF;
      if(id!=ID_PWR && id!=ID_RPM && id!=ID_TEMP && id!=ID_GEAR) continue;
      lastMotorDataMs=millis();
      const uint8_t* b = m.data;
      bool gearChanged=false; uint8_t gearNow=0;

      TEL_LOCK();
      if(id==ID_PWR){
        uint16_t rawV=(uint16_t)((b[2]<<2)|(b[3]>>6)); float V=rawV*0.5f;
        int16_t rawA=(int16_t)((b[0]<<3)|(b[1]>>5)); if(rawA&0x0400) rawA|=0xF800;
        float A=-(rawA/2.0f); uint8_t rawSOC=b[4]&0x7F;
        telem.packV=V; telem.packA=A; telem.socPct=(rawSOC<=100)?(float)rawSOC:NAN;
      } else if(id==ID_RPM){
        int32_t rpm=0; telem.rpmValid=decodeLeafMotorRpm(b,rpm); telem.rpm=rpm;
      } else if(id==ID_TEMP){
        telem.motorTempC=b[5]*0.5f;
      } else if(id==ID_GEAR){
        uint8_t g=b[0]; if(g<1||g>3) g=0; uint8_t rg=b[5]; if(rg>2) rg=0xFF;
        telem.gear=g; telem.regen=rg; gearNow=g; gearChanged=(g!=lastGear);
      }
      TEL_UNLOCK();

      if(id==ID_GEAR && hN2K){
        n2kSend127493(gearNow);
        TEL_LOCK(); Telemetry t=telem; TEL_UNLOCK();
        n2kSend61184(nextSid(), t.regen, t.packV, t.packA);
        if(gearChanged){ lastGear=gearNow; Serial.printf("[GEAR] -> %s | Regen: %s\n",
          (gearNow?gearToStr(gearNow):String("--")).c_str(), (t.regen<=2?regenToStr(t.regen):String("--")).c_str()); }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ======== N2K TX periodic (uses hN2K) ========
void taskN2kTx(void*){
  const TickType_t period=pdMS_TO_TICKS(100);
  uint32_t lastFuelTankTxMs=0;
  for(;;){
    if(hN2K){
      TEL_LOCK(); Telemetry t=telem; TEL_UNLOCK();
      n2kSend127488(t.rpm,t.rpmValid);
      n2kSend127493(t.gear);
      if(isfinite(t.packV)&&isfinite(t.packA)){
        n2kSend127508(t.packV,t.packA);
        n2kSend127751(t.packV,t.packA);
      }
      if(isfinite(t.socPct)){
        n2kSend127506(t.socPct);
        uint32_t now=millis();
        if((uint32_t)(now-lastFuelTankTxMs)>=SOC_FUEL_TANK_PERIOD_MS){
          lastFuelTankTxMs=now;
          n2kSend127505_FuelSoc(t.socPct);
        }
      }
      if(isfinite(t.motorTempC)) n2kSend127489_EngineTemp(t.motorTempC);
      n2kSend61184(nextSid(), t.regen, t.packV, t.packA);
    }
    vTaskDelay(period);
  }
}

// ======== N2K RX task (uses hN2K) ========
void taskN2kRx(void*){
  for(;;){
    twai_message_t m;
    if(hN2K && twai_receive_v2(hN2K,&m,pdMS_TO_TICKS(10))==ESP_OK){
      n2kRxFrames=n2kRxFrames+1;
      lastN2kActivityMs=millis();
      if(!m.extd) continue;
      uint32_t pgn = idToPGN(m.identifier);
      uint8_t  da  = idToDA(m.identifier);
      const uint8_t* db = m.data; uint8_t len = m.data_length_code;

      if(pgn==PGN_ISO_REQUEST && (da==N2K_SA || da==N2K_BCAST)){
        if(len>=3){
          uint32_t reqPGN=(uint32_t)db[0]|((uint32_t)db[1]<<8)|((uint32_t)db[2]<<16);
          if(reqPGN==PGN_ISO_ADDRESS_CLAIM) n2kSend60928_AddressClaim();
          else if(reqPGN==PGN_SWITCH_STATUS) n2kSend127501_Status(SW_BANK_INSTANCE, bank_state_bits);
          else if(reqPGN==PGN_SWITCH_LABEL)  for(uint8_t ch=1; ch<=SW_CHANNELS; ch++) n2kSend127504_LabelShort(SW_BANK_INSTANCE,ch,sw_labels[ch]);
        }
      } else if(pgn==PGN_SWITCH_STATUS){
        if(len>=5){
          uint8_t bank=db[0];
          uint32_t bits=(uint32_t)db[1]|((uint32_t)db[2]<<8)|((uint32_t)db[3]<<16)|((uint32_t)db[4]<<24);
          on127501_StatusRx(bank,bits);
        }
      } else if(pgn==PGN_SWITCH_CONTROL){
        applyControlAsBankAndPublish(db,len);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ======== Serial ticker ========
void taskSerialTicker(void*){
  const TickType_t period=pdMS_TO_TICKS(500);
  for(;;){ printStatusLine(); vTaskDelay(period); }
}

String healthJson(uint32_t now){
  bool ready=healthChecksReady(now);
  String json=String("{\"health_ready\":")+(ready?"true":"false");
  json+=String(",\"motor_can_ok\":")+(motorDataHealthy(now)?"true":"false");
  json+=String(",\"n2k_ok\":")+(n2kNetworkHealthy(now)?"true":"false");
  json+=String(",\"n2k_tx_queued\":")+String(n2kTxQueuedFrames);
  json+=String(",\"n2k_tx_dropped\":")+String(n2kTxDroppedFrames);
  json+=String(",\"n2k_fuel_tank_tx\":")+String(n2kFuelTankTxFrames);
  json+=String(",\"n2k_rx_frames\":")+String(n2kRxFrames);
  json+=String(",\"n2k_hw_tx_failed\":")+String(n2kHwTxFailedCount);
  json+=String(",\"n2k_bus_errors\":")+String(n2kHwBusErrorCount);
  json+=String(",\"n2k_bus_off\":")+String(n2kBusOffCount);
  json+=String(",\"ts_ms\":")+String(now)+"}";
  return json;
}

void broadcastHealth(uint32_t now){ String json=healthJson(now); ws.broadcastTXT(json); }

String telemetryJson(){
  TEL_LOCK(); Telemetry t=telem; TEL_UNLOCK();
  uint32_t now=millis();
  float A_ch=isfinite(t.packA)?-t.packA:NAN;
  float p_kw=(isfinite(t.packV)&&isfinite(A_ch))?(t.packV*A_ch/1000.0f):NAN;
  String gearS=(t.gear>=1&&t.gear<=3)?gearToStr(t.gear):String("--");
  String regenS=(t.regen<=2)?regenToStr(t.regen):String("--");

  String json; json.reserve(420); json="{";
  json+="\"ts_ms\":"+String(now);
  if(isfinite(t.packV)) json+=",\"v\":"+String(t.packV,1);
  if(isfinite(A_ch)) json+=",\"a\":"+String(A_ch,1);
  if(isfinite(p_kw)) json+=",\"p_kw\":"+String(p_kw,1);
  if(isfinite(t.socPct)) json+=",\"soc\":"+String(t.socPct,1);
  json+=",\"rpm\":"+String(t.rpm);
  json+=String(",\"rpm_valid\":")+(t.rpmValid?"true":"false");
  if(isfinite(t.motorTempC)) json+=",\"mt_c\":"+String(t.motorTempC,1);
  json+=",\"gear_str\":\""+gearS+"\"";
  json+=",\"regen_str\":\""+regenS+"\"";
  json+=String(",\"health_ready\":")+(healthChecksReady(now)?"true":"false");
  json+=String(",\"motor_can_ok\":")+(motorDataHealthy(now)?"true":"false");
  json+=String(",\"n2k_ok\":")+(n2kNetworkHealthy(now)?"true":"false");
  json+=",\"n2k_tx_dropped\":"+String(n2kTxDroppedFrames);
  json+=",\"n2k_hw_tx_failed\":"+String(n2kHwTxFailedCount);
  json+=",\"n2k_bus_errors\":"+String(n2kHwBusErrorCount);
  json+="}";
  return json;
}

String switchesJson(){
  uint32_t now=millis();
  String json; json.reserve(300); json="{";
  json+="\"ts_ms\":"+String(now);
  json+=",\"bank\":"+String((int)SW_BANK_INSTANCE);
  json+=",\"sw_count\":"+String((int)SW_CHANNELS);
  json+=",\"sw_state\":"+String(sw_state_bits);
  json+=",\"labels\":[";
  for(uint8_t i=1;i<=SW_CHANNELS;i++){
    json+="\""; json+=sw_labels[i]; json+="\"";
    if(i<SW_CHANNELS) json+=",";
  }
  json+="]";
  json+=String(",\"health_ready\":")+(healthChecksReady(now)?"true":"false");
  json+=String(",\"n2k_ok\":")+(n2kNetworkHealthy(now)?"true":"false");
  json+="}";
  return json;
}

void sendJsonNoCache(const String &json){
  http.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  http.sendHeader("Pragma","no-cache");
  http.sendHeader("Expires","0");
  http.send(200,"application/json; charset=utf-8",json);
}

void handleApiData(){ sendJsonNoCache(telemetryJson()); }
void handleApiSwitches(){ sendJsonNoCache(switchesJson()); }

void handleApiRate(){
  if(!http.hasArg("ms")){ http.send(400,"application/json","{\"error\":\"missing ms\"}"); return; }
  uint32_t value=(uint32_t)http.arg("ms").toInt();
  if(value<50||value>60000){ http.send(400,"application/json","{\"error\":\"invalid ms\"}"); return; }
  broadcastIntervalMs=value;
  sendJsonNoCache(String("{\"ok\":true,\"ms\":")+String(value)+"}");
}

void handleApiSwitch(){
  if(!http.hasArg("ch")||!http.hasArg("on")){ http.send(400,"application/json","{\"error\":\"missing argument\"}"); return; }
  int channel=http.arg("ch").toInt();
  String onText=http.arg("on"); onText.toLowerCase();
  bool on=(onText=="1"||onText=="true"||onText=="on");
  if(channel<1||channel>SW_CONTROL_CHANNELS){ http.send(400,"application/json","{\"error\":\"invalid channel\"}"); return; }
  uint32_t setMask=on?(1u<<(channel-1)):0;
  uint32_t clrMask=on?0:(1u<<(channel-1));
  n2kSend127502_Control(SW_BANK_INSTANCE,setMask,clrMask,SW_TARGET_SA);
  sendJsonNoCache("{\"ok\":true}");
}

// ======== WebSocket ========
void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  switch(type){
    case WStype_CONNECTED:{
      anyClientConnected=true;
      String hello=String("{\"status\":\"Client ")+num+" connected\",\"ts_ms\":"+String(millis())+"}";
      ws.sendTXT(num,hello);
      String health=healthJson(millis());
      ws.sendTXT(num,health);
      break;
    }
    case WStype_DISCONNECTED:{ anyClientConnected=(ws.connectedClients()>0); break; }
    case WStype_TEXT:{
      String msg((char*)payload,len); String low=msg; low.toLowerCase();

      if(low.indexOf("set_rate")>=0){
        int i=low.indexOf("\"ms\""); if(i>=0){ int c=low.indexOf(":",i); if(c>=0){ int s=c+1; while(s<(int)low.length()&&(low[s]==' '||low[s]=='\"')) s++; int e=s; while(e<(int)low.length()&&isDigit(low[e])) e++;
          uint32_t v=low.substring(s,e).toInt(); if(v>=50&&v<=60000){ broadcastIntervalMs=v; String ack=String("{\"status\":\"interval set to ")+v+" ms\",\"ts_ms\":"+String(millis())+"}"; ws.sendTXT(num,ack); } } }
      }

      if(low.indexOf("switches_sub")>=0){
        String json=switchesJson(); ws.sendTXT(num,json);
      }

      if(low.indexOf("\"cmd\":\"switch\"")>=0){
        int j=low.indexOf("\"ch\""), k=low.indexOf("\"on\""); if(j>=0&&k>=0){
          int cj=low.indexOf(":",j), co=low.indexOf(":",k);
          int sj=cj+1; while(sj<(int)low.length()&&(low[sj]==' '||low[sj]=='\"')) sj++;
          int so=co+1; while(so<(int)low.length()&&(low[so]==' '||low[so]=='\"')) so++;
          int ej=sj; while(ej<(int)low.length()&&isDigit(low[ej])) ej++;
          int eo=so; while(eo<(int)low.length()&&(isalpha(low[eo])||low[eo]=='t'||low[eo]=='f')) eo++;
          uint8_t ch=(uint8_t)low.substring(sj,ej).toInt(); bool on=(low.substring(so,eo).indexOf("true")>=0);
          if(ch>=1&&ch<=SW_CONTROL_CHANNELS){ uint32_t setMask= on?(1u<<(ch-1)):0; uint32_t clrMask= on?0:(1u<<(ch-1)); n2kSend127502_Control(SW_BANK_INSTANCE,setMask,clrMask,SW_TARGET_SA); }
        }
      }
      break;
    }
    default: break;
  }
}

// ======== TWAI v2 bring-up ========
bool twaiInit_Controller(twai_handle_t &handle, int controller_id,
                         gpio_num_t tx, gpio_num_t rx, int kbps) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
  g.controller_id = controller_id; // 0 or 1
  g.tx_queue_len = (controller_id==1)?32:5;
  g.rx_queue_len = (controller_id==0)?64:32;

  twai_timing_config_t t;
  if (kbps == 500) {
    t = TWAI_TIMING_CONFIG_500KBITS();
  } else {
    t = TWAI_TIMING_CONFIG_250KBITS();
  }

  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install_v2(&g, &t, &f, &handle) != ESP_OK) return false;
  if (twai_start_v2(handle) != ESP_OK) return false;

  // If available in your core:
  twai_reconfigure_alerts_v2(handle,
      TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_RX_DATA |
      TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED, nullptr);

  return true;
}

void serviceN2kHealth(uint32_t now){
  static uint32_t lastPollMs=0;
  if(!hN2K || (uint32_t)(now-lastPollMs)<100) return;
  lastPollMs=now;

  uint32_t alerts=0;
  if(twai_read_alerts_v2(hN2K,&alerts,0)==ESP_OK &&
     (alerts&(TWAI_ALERT_TX_SUCCESS|TWAI_ALERT_RX_DATA))){
    lastN2kActivityMs=now;
  }

  twai_status_info_t status{};
  if(twai_get_status_info_v2(hN2K,&status)!=ESP_OK){
    n2kControllerRunning=false;
    return;
  }
  n2kControllerRunning=(status.state==TWAI_STATE_RUNNING);
  n2kHwTxFailedCount=status.tx_failed_count;
  n2kHwBusErrorCount=status.bus_error_count;
  n2kHwTxErrorCounter=status.tx_error_counter;
  n2kHwRxErrorCounter=status.rx_error_counter;

  if(status.state==TWAI_STATE_BUS_OFF && !n2kRecoveryPending){
    if(twai_initiate_recovery_v2(hN2K)==ESP_OK){
      n2kBusOffCount=n2kBusOffCount+1;
      n2kRecoveryPending=true;
      n2kControllerRunning=false;
      Serial.println("[N2K] Bus off; recovery started");
    }
  } else if(status.state==TWAI_STATE_STOPPED && n2kRecoveryPending){
    if(twai_start_v2(hN2K)==ESP_OK){
      n2kRecoveryPending=false;
      n2kControllerRunning=true;
      Serial.println("[N2K] Recovery complete; controller restarted");
    }
  }
}



// ======== Web broadcast (engine snapshot) ========
void broadcastNow(){ String json=telemetryJson(); ws.broadcastTXT(json); }

// ======== Setup / Loop ========
void setup(){
  Serial.begin(115200);
  delay(200);
  healthStartMs=millis();
  telemMtx = xSemaphoreCreateMutex();
  n2kTxMtx = xSemaphoreCreateMutex();
  N2K_NAME = buildNAME();

  bool ap = WiFi.softAP(AP_SSID, AP_PASS, AP_CHAN, false, 8);
  Serial.println(ap ? "SoftAP started" : "SoftAP failed");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  http.on("/", HTTP_GET, handleRoot);
  http.on("/switches", HTTP_GET, handleSwitches);
  http.on("/index.html", HTTP_GET, handleRoot);
  http.on("/api/data", HTTP_GET, handleApiData);
  http.on("/api/switches", HTTP_GET, handleApiSwitches);
  http.on("/api/rate", HTTP_POST, handleApiRate);
  http.on("/api/switch", HTTP_POST, handleApiSwitch);
  http.onNotFound(handleNotFound);
  http.begin();
  ws.begin();
  ws.onEvent(onWsEvent);

  // TWAI v2: two controllers
  bool okEV  = twaiInit_Controller(hEV,  0, EV_TX,  EV_RX, 500);
  Serial.println(okEV ? "EV-CAN @500k ready (TWAI#0)" : "EV-CAN init FAILED");
  bool okN2K = twaiInit_Controller(hN2K, 1, N2K_TX, N2K_RX, 250);
  Serial.println(okN2K ? "N2K  @250k ready (TWAI#1)"  : "N2K init FAILED");

  if (okN2K){
    n2kSend60928_AddressClaim();
    n2kSend127501_Status(SW_BANK_INSTANCE, bank_state_bits);
    on127501_StatusRx(SW_BANK_INSTANCE, bank_state_bits);
    for (uint8_t ch=1; ch<=SW_CHANNELS; ch++) n2kSend127504_LabelShort(SW_BANK_INSTANCE, ch, sw_labels[ch]);
  }

  sw_bootStormEndMs = millis() + 10000;

  // NOTE: ESP32-C6 is single-core → use xTaskCreate (no pinning)
  xTaskCreate(taskEvRx,        "EV-RX",   4096, nullptr, 20, nullptr);
  xTaskCreate(taskN2kTx,       "N2K-TX",  6144, nullptr, 10, nullptr);
  xTaskCreate(taskN2kRx,       "N2K-RX",  4096, nullptr, 12, nullptr);
  xTaskCreate(taskSerialTicker,"SERIAL",  3072, nullptr,  2, nullptr);
}

void loop(){
  dnsServer.processNextRequest();
  http.handleClient();
  ws.loop();

  uint32_t now=millis();
  serviceN2kHealth(now);
  if(anyClientConnected && (now-lastBroadcastMs>=broadcastIntervalMs)){ lastBroadcastMs=now; broadcastNow(); }
  if(anyClientConnected && (now-lastHealthBroadcastMs>=HEALTH_BROADCAST_MS)){ lastHealthBroadcastMs=now; broadcastHealth(now); }

  static uint32_t lastStorm=0,lastSteady=0;
  if(hN2K){
    if(now<sw_bootStormEndMs){ if(now-lastStorm>=250){ lastStorm=now; n2kSend127501_Status(SW_BANK_INSTANCE,bank_state_bits);} }
    else{ if(now-lastSteady>=1000){ lastSteady=now; n2kSend127501_Status(SW_BANK_INSTANCE,bank_state_bits);} }
  }
}
