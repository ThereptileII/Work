#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WebSocketsServer.h>   // Install "arduinoWebSockets" by Markus Sattler
#include <math.h>

// ====== Config ======
const char* AP_SSID     = "XIAO-DATA";
const char* AP_PASS     = "12345678";      // >=8 chars
const byte  DNS_PORT    = 53;
const int   AP_CHANNEL  = 6;

// Adjust this to a real LED/control pin on your XIAO ESP32-C6:
#ifndef LED_BUILTIN
#define LED_BUILTIN 2   // CHANGE to your board’s LED/relay pin
#endif

// ====== Globals ======
DNSServer        dnsServer;
WebServer        http(80);
WebSocketsServer ws(81);                   // ws://<ip>:81/
uint32_t         broadcastIntervalMs = 1000;
uint32_t         lastBroadcastMs = 0;
bool             anyClientConnected = false;

bool             running = false;
float            targetValue = 50.0f;      // 0..100 from UI

// ====== Single-file web app (no "clients" tile) ======
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>XIAO Live Data (WebSocket + Controls)</title>
<style>
:root{--bg:#0b1220;--card:#121a2c;--text:#e6eefc;--muted:#8aa0c3;--accent:#56c0ff}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Inter,Arial;background:var(--bg);color:var(--text);display:flex;min-height:100vh;align-items:center;justify-content:center;padding:16px}
.card{background:var(--card);border-radius:16px;box-shadow:0 8px 30px rgba(0,0,0,.35);max-width:560px;width:100%;padding:20px}
h1{margin:0 0 8px 0;font-size:1.35rem}.muted{color:var(--muted);font-size:.9rem;margin-bottom:14px}
.row{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:10px 0}
.pill{background:#0f2236;border:1px solid #20334d;color:var(--accent);padding:8px 12px;border-radius:999px;font-weight:600}
.value{font-size:2.4rem;font-weight:700;letter-spacing:.5px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:12px}
.tile{background:#0f1726;border:1px solid #1d2a44;border-radius:12px;padding:12px}
.label{font-size:.82rem;color:var(--muted);margin-bottom:6px}
button{background:#0e2337;border:1px solid #1d3653;color:var(--text);padding:8px 12px;border-radius:10px;cursor:pointer}
button:active{transform:translateY(1px)}
input[type=number], input[type=range]{accent-color:#56c0ff;background:#0e2337;border:1px solid #1d3653;color:var(--text);padding:8px 10px;border-radius:10px}
.range-wrap{display:flex;align-items:center;gap:8px;width:100%}
small{color:var(--muted)}
pre{white-space:pre-wrap;word-wrap:break-word;background:#0f1726;border:1px solid #1d2a44;border-radius:12px;padding:10px;max-height:140px;overflow:auto}
.footer{display:flex;justify-content:space-between;align-items:center;margin-top:14px;gap:8px;flex-wrap:wrap}
</style></head><body>
<div class="card">
  <h1>XIAO Live Data</h1>
  <div class="muted">Direct AP + WebSocket push + Controls</div>

  <div class="row">
    <span class="pill" id="status">Connecting…</span>
    <small id="serverMsg"></small>
  </div>

  <div class="value" id="primary">--</div>
  <div class="grid">
    <div class="tile"><div class="label">Secondary</div><div class="value" id="secondary">--</div></div>
    <div class="tile"><div class="label">Timestamp</div><div class="value" id="ts">--</div></div>
  </div>

  <div class="row">
    <button id="btnStart">Start</button>
    <button id="btnStop">Stop</button>
    <button id="btnLedOn">LED ON</button>
    <button id="btnLedOff">LED OFF</button>
    <button id="btnMinus">– rate</button>
    <button id="btnPlus">+ rate</button>
  </div>

  <div class="row range-wrap">
    <div style="min-width:88px" class="label">Target</div>
    <input id="target" type="range" min="0" max="100" step="1" value="50" style="flex:1"/>
    <span id="targetVal">50</span>
    <button id="btnSendTarget">Send</button>
  </div>

  <div class="footer">
    <small>Tip: connect to AP, then open <code>http://192.168.4.1</code></small>
    <a href="/">Home</a>
  </div>

  <div class="tile" style="margin-top:12px">
    <div class="label">Debug</div>
    <pre id="debug"></pre>
  </div>
</div>

<script>
const $=(id)=>document.getElementById(id);
const statusEl=$("status"), primaryEl=$("primary"), secondaryEl=$("secondary"), tsEl=$("ts"), dbg=$("debug"), serverMsg=$("serverMsg");

const btnStart=$("btnStart"), btnStop=$("btnStop"),
      btnLedOn=$("btnLedOn"), btnLedOff=$("btnLedOff"),
      btnMinus=$("btnMinus"), btnPlus=$("btnPlus"),
      target=$("target"), targetVal=$("targetVal"), btnSendTarget=$("btnSendTarget");

target.addEventListener("input", ()=>{ targetVal.textContent = target.value; });

const host = location.hostname || "192.168.4.1";
const ws = new WebSocket(`ws://${host}:81/`);

function log(s){ dbg.textContent = (new Date().toLocaleTimeString()) + " " + s + "\n" + dbg.textContent; }

ws.onopen   = ()=>{statusEl.textContent="Live"; log("WS open");};
ws.onclose  = ()=>{statusEl.textContent="Disconnected"; log("WS closed");};
ws.onerror  = (e)=>{statusEl.textContent="Error"; log("WS error");};

ws.onmessage = (ev)=>{
  try{
    const j = JSON.parse(ev.data);
    if (j.primary   !== undefined) primaryEl.textContent = (typeof j.primary==="number") ? j.primary.toFixed(2) : j.primary;
    if (j.secondary !== undefined) secondaryEl.textContent = j.secondary;
    if (j.ts_ms     !== undefined) tsEl.textContent = new Date(j.ts_ms).toLocaleTimeString();
    if (j.status) { statusEl.textContent = j.status; serverMsg.textContent = j.status; }
    if (j.rate_ms) log("Rate: "+j.rate_ms+" ms");
    if (j.target   !== undefined) { target.value = j.target; targetVal.textContent = j.target; }
    if (j.echo) log("Echo: "+JSON.stringify(j.echo));
  }catch(e){ log("Parse error"); }
};

function sendCmd(cmd, extra={}) {
  const msg = JSON.stringify({cmd, ...extra});
  ws.send(msg);
  log("TX: "+msg);
}

btnStart.onclick     = ()=>sendCmd("start");
btnStop.onclick      = ()=>sendCmd("stop");
btnLedOn.onclick     = ()=>sendCmd("led", {state:1});
btnLedOff.onclick    = ()=>sendCmd("led", {state:0});
btnMinus.onclick     = ()=>sendCmd("set_rate_rel", {delta:-100});
btnPlus.onclick      = ()=>sendCmd("set_rate_rel", {delta:100});
btnSendTarget.onclick= ()=>sendCmd("set_target", {value: parseFloat(target.value)});
</script>
</body></html>
)HTML";

// --- HTTP helpers (captive portal) ---
bool captivePortal() {
  if (!http.hostHeader().equals(WiFi.softAPIP().toString())) {
    http.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
    http.send(302, "text/plain", "");
    return true;
  }
  return false;
}
void handleRoot(){ http.send_P(200, "text/html; charset=utf-8", INDEX_HTML); }
void handleNotFound(){ if (captivePortal()) return; http.send(404, "text/plain", "Not found"); }

// --- Your data source (replace with real sensors/CAN/etc.) ---
void readLiveData(float &primary, uint32_t &secondary) {
  static uint32_t ctr = 0; ctr++;
  float sine = sinf(millis()/1000.0f)*5.0f;
  primary = (running ? 42.0f + sine : targetValue + sine*0.2f);
  secondary = ctr % 100;
}

// --- send helpers ---
void sendJsonTo(uint8_t num, const String& key, const String& value) {
  String msg = "{\"" + key + "\":" + value + ",\"ts_ms\":" + String(millis()) + "}";
  ws.sendTXT(num, msg);
}
void sendStatusTo(uint8_t num, const String& s) {
  String v = "\"" + s + "\"";
  sendJsonTo(num, "status", v);
}

// --- WS: broadcast JSON to all clients ---
void broadcastNow() {
  float primary; uint32_t secondary;
  readLiveData(primary, secondary);

  String json = "{";
  json += "\"primary\":"   + String(primary,2)           + ",";
  json += "\"secondary\":" + String(secondary)           + ",";
  json += "\"target\":"    + String(targetValue,1)       + ",";
  json += "\"rate_ms\":"   + String(broadcastIntervalMs) + ",";
  json += "\"ts_ms\":"     + String(millis());
  json += "}";
  ws.broadcastTXT(json);
}

// --- tiny JSON helpers (no extra libs) ---
bool extractNumber(const String& src, const char* key, int &out) {
  String k = "\"" + String(key) + "\"";
  int i = src.indexOf(k); if (i < 0) return false;
  int c = src.indexOf(':', i); if (c < 0) return false;
  int s = c + 1;
  while (s < (int)src.length() && (src[s]==' '||src[s]=='\"')) s++;
  int e = s;
  while (e < (int)src.length() && (isDigit(src[e]) || src[e]=='-' )) e++;
  out = src.substring(s, e).toInt();
  return true;
}
bool extractFloat(const String& src, const char* key, float &out) {
  String k = "\"" + String(key) + "\"";
  int i = src.indexOf(k); if (i < 0) return false;
  int c = src.indexOf(':', i); if (c < 0) return false;
  int s = c + 1;
  while (s < (int)src.length() && (src[s]==' '||src[s]=='\"')) s++;
  int e = s;
  while (e < (int)src.length() && (isDigit(src[e]) || src[e]=='-' || src[e]=='.')) e++;
  out = src.substring(s, e).toFloat();
  return true;
}
bool hasCmd(const String& src, const char* name) {
  String k = "\"cmd\"";
  int i = src.indexOf(k); if (i < 0) return false;
  int q1 = src.indexOf('\"', i+5); if (q1 < 0) return false;
  int q2 = src.indexOf('\"', q1+1); if (q2 < 0) return false;
  String val = src.substring(q1+1, q2);
  val.toLowerCase();
  return val == name;
}

// --- WS events ---
void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED: {
      anyClientConnected = true;
      pinMode(LED_BUILTIN, OUTPUT);
      sendStatusTo(num, String("Client ") + num + " connected");
      // initial snapshot
      String snap = String("{\"target\":") + String(targetValue,1) +
                    ",\"rate_ms\":" + String(broadcastIntervalMs) +
                    ",\"ts_ms\":" + String(millis()) + "}";
      ws.sendTXT(num, snap);
      break;
    }
    case WStype_DISCONNECTED: {
      anyClientConnected = (ws.connectedClients() > 0);
      break;
    }
    case WStype_TEXT: {
      String msg((char*)payload, len);
      String msgLower = msg; msgLower.toLowerCase();

      if (hasCmd(msgLower, "start")) {
        running = true;
        sendStatusTo(num, "started");
      }
      else if (hasCmd(msgLower, "stop")) {
        running = false;
        sendStatusTo(num, "stopped");
      }
      else if (hasCmd(msgLower, "led")) {
        int state=0;
        if (extractNumber(msgLower, "state", state)) {
          digitalWrite(LED_BUILTIN, state ? HIGH : LOW);
          sendStatusTo(num, String("led ") + (state ? "on" : "off"));
        }
      }
      else if (hasCmd(msgLower, "set_rate")) {
        int ms=0; if (extractNumber(msgLower, "ms", ms)) {
          if (ms >= 50 && ms <= 60000) {
            broadcastIntervalMs = (uint32_t)ms;
            sendStatusTo(num, String("interval set to ") + ms + " ms");
          }
        }
      }
      else if (hasCmd(msgLower, "set_rate_rel")) {
        int d=0; if (extractNumber(msgLower, "delta", d)) {
          long next = (long)broadcastIntervalMs + d;
          if (next < 50) next = 50;
          if (next > 60000) next = 60000;
          broadcastIntervalMs = (uint32_t)next;
          sendStatusTo(num, String("interval set to ") + next + " ms");
        }
      }
      else if (hasCmd(msgLower, "set_target")) {
        float v=0; if (extractFloat(msgLower, "value", v)) {
          if (v < 0) v = 0; if (v > 100) v = 100;
          targetValue = v;
          String echo = String("{\"target\":") + String(targetValue,1) +
                        ",\"ts_ms\":" + String(millis()) + "}";
          ws.sendTXT(num, echo);
        }
      }
      else {
        String echo = String("{\"echo\":") + msg + ",\"ts_ms\":" + String(millis()) + "}";
        ws.sendTXT(num, echo);
      }
      break;
    }
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Start SoftAP
  bool ap = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, false, 4);
  if (!ap) {
    Serial.println("SoftAP start failed");
  } else {
    Serial.print("AP SSID: "); Serial.println(AP_SSID);
    Serial.print("AP IP:   "); Serial.println(WiFi.softAPIP());
  }

  // DNS for captive-portal style redirect
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  // HTTP (serves the page + captive)
  http.on("/", HTTP_GET, handleRoot);
  http.on("/index.html", HTTP_GET, handleRoot);
  http.onNotFound(handleNotFound);
  http.begin();
  Serial.println("HTTP server started");

  // WebSocket
  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println("WebSocket server on :81 started");

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  dnsServer.processNextRequest();
  http.handleClient();
  ws.loop();

  uint32_t now = millis();
  if (anyClientConnected && (now - lastBroadcastMs >= broadcastIntervalMs)) {
    lastBroadcastMs = now;
    broadcastNow();
  }
}
