/*
  ESP32 dual CAN + Nextion + History + Wind math
  - EV-CAN (TWAI @ 500kbps): 0x1DB, 0x1DA, 0x55A, 0x539
  - N2K (MCP2515 @ 250kbps, extended): PGN 128259 (STW), PGN 130306 (Wind)
  - Compute AWS/AWA, TWS/TWA (true wind through water)
  - Ring buffers + min/max inside ESP32
  - Nextion updates over Serial2 (adjust component IDs to your HMI)
*/

#include <Arduino.h>
#include <driver/twai.h>
#include <SPI.h>
#include <mcp_can.h>

// ================= USER CONFIG =================
#define KW_FACTOR            28.0f
#define USB_BAUD             115200
#define OUT_HZ               10                      // serial+Nextion refresh Hz
#define SAMPLE_HZ            5                       // history sampling Hz
#define HISTORY_LEN          600                     // samples per series (e.g. 600 @ 5Hz ~ 2 min)
#define NEXTION_BAUD         115200                  // your Nextion baud
#define SEND_WAVEFORMS       1                       // 1=push samples to Nextion waveforms

// EV-CAN (TWAI) pins
#define EV_TX GPIO_NUM_5
#define EV_RX GPIO_NUM_4

// MCP2515 (N2K)
#define MCP_CS   15
#define MCP_INT  27
#define MCP_SCK  18
#define MCP_MOSI 23
#define MCP_MISO 19
#define MCP_CLOCK MCP_8MHZ    // change to MCP_16MHZ if your board has 16MHz crystal
#define MCP_BAUD  CAN_250KBPS

// Nextion UART (Serial2) pins
#define NX_TX 17
#define NX_RX 16

// ================= IDs / PGNs ==================
// EV-CAN
static const uint16_t ID_PWR  = 0x1DB; // V/I/SoC
static const uint16_t ID_RPM  = 0x1DA; // RPM
static const uint16_t ID_TEMP = 0x55A; // Motor temp
static const uint16_t ID_GEAR = 0x539; // Gear+Regen

// N2K PGNs
static const uint32_t PGN_STW = 128259; // Speed (Water)
static const uint32_t PGN_WIND= 130306; // Wind data (AWS/AWA or True)

// ================= Globals =====================
MCP_CAN MCP(MCP_CS);

struct MinMax {
  bool init=false; float mn=0, mx=0;
  void update(float v){ if(!init){mn=mx=v;init=true;} else { if(v<mn) mn=v; if(v>mx) mx=v; } }
};

struct Ring {
  float buf[HISTORY_LEN]; uint16_t head=0; bool filled=false;
  void push(float v){ buf[head]=v; head=(head+1)%HISTORY_LEN; if(head==0) filled=true; }
};

struct Telemetry {
  float V=0, SOC=0, kW=0, kWh=0, MT=0;
  int32_t RPM=0;
  uint8_t gear=0, regen=0xFF;
  float STW_kn=0;
  // wind
  float AWS_kn=0, AWA_deg=0;  // from PGN130306 when ref=Apparent
  float TWS_kn=0, TWA_deg=0;  // computed true wind (through water)
  // timestamps
  uint32_t t_1DB=0, t_1DA=0, t_55A=0, t_539=0, t_STW=0, t_WIND=0;
  // min/max
  MinMax vMM, kWMM, rpmMM, stwMM, awsMM, awaMM, twsMM, twaMM;
  // history (ring buffers)
  Ring vHist, kWHist, rpmHist, stwHist, awsHist, twsHist;
} telem;

SemaphoreHandle_t mtx;

static inline uint32_t nowMs(){ return millis(); }

// ================= Nextion helpers =================
HardwareSerial Nextion(2); // Serial2

void nxSendRaw(const char* s){ Nextion.print(s); Nextion.write(0xFF); Nextion.write(0xFF); Nextion.write(0xFF); }
void nxSetNum(const char* obj, float val){ char buf[64]; snprintf(buf,sizeof(buf),"%s.val=%ld",obj,(long)lroundf(val)); nxSendRaw(buf); }
void nxSetTxt(const char* obj, const char* txt){ char buf[128]; snprintf(buf,sizeof(buf),"%s.txt=\"%s\"",obj,txt); nxSendRaw(buf); }
// Waveform add: add <objid>,<channel>,<value>  (value 0..255)
void nxWaveAdd(uint8_t objId, uint8_t ch, uint8_t v){ char b[32]; snprintf(b,sizeof(b),"add %u,%u,%u",objId,ch,v); nxSendRaw(b); }

// Map gear/regen to strings
const char* gearStr(uint8_t g){ return (g==1)?"DRIVE":(g==2)?"NEUTRAL":(g==3)?"REVERSE":"UNKNOWN"; }
const char* regenStr(uint8_t r){ return (r==0)?"RegenOff":(r==1)?"RegenOneBar":(r==2)?"RegenTwoBar":"Unknown"; }

// ================= EV-CAN (TWAI) =================
bool twaiInit(){
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(EV_TX, EV_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g,&t,&f)!=ESP_OK) return false;
  if (twai_start()!=ESP_OK) return false;
  twai_reconfigure_alerts(TWAI_ALERT_BUS_OFF|TWAI_ALERT_RX_DATA|TWAI_ALERT_RX_QUEUE_FULL,nullptr);
  return true;
}
void twaiRecover(){ twai_initiate_recovery(); vTaskDelay(pdMS_TO_TICKS(200)); twai_start(); }

void taskEvRx(void*){
  for(;;){
    uint32_t alerts;
    if (twai_read_alerts(&alerts,pdMS_TO_TICKS(1))==ESP_OK){
      if (alerts & TWAI_ALERT_BUS_OFF) twaiRecover();
    }
    twai_message_t m;
    while (twai_receive(&m,0)==ESP_OK){
      if (m.extd||m.rtr) continue;
      const uint8_t* b=m.data;
      uint16_t id = m.identifier & 0x7FF; uint32_t t=nowMs();

      if (id==ID_PWR){
        uint16_t rawV = (b[2]<<2)|(b[3]>>6); float V = rawV*0.5f;
        int16_t rawA = (b[0]<<3)|(b[1]>>5); if (rawA & 0x0400) rawA|=0xF800;
        float A = -(rawA/2.0f);
        float kW = (A*V)/1000.0f;
        uint16_t rawSOC = (b[5]<<8)|b[4]; float SOC = rawSOC*1.0f;
        float kWh = (SOC*KW_FACTOR)/100.0f;

        xSemaphoreTake(mtx,portMAX_DELAY);
        telem.V=V; telem.kW=kW; telem.SOC=SOC; telem.kWh=kWh; telem.t_1DB=t;
        telem.vMM.update(V); telem.kWMM.update(kW);
        xSemaphoreGive(mtx);
      }
      else if (id==ID_RPM){
        uint16_t r=((uint16_t)b[4]<<8)|b[5]; int32_t RPM=(int32_t)lroundf(r*0.5f);
        xSemaphoreTake(mtx,portMAX_DELAY);
        telem.RPM=RPM; telem.t_1DA=t; telem.rpmMM.update(RPM);
        xSemaphoreGive(mtx);
      }
      else if (id==ID_TEMP){
        float MT=b[5]*0.5f;
        xSemaphoreTake(mtx,portMAX_DELAY);
        telem.MT=MT; telem.t_55A=t;
        xSemaphoreGive(mtx);
      }
      else if (id==ID_GEAR){
        uint8_t g=b[0]; if (g<1||g>3) g=0; uint8_t r=b[5]; if (r>2) r=0xFF;
        xSemaphoreTake(mtx,portMAX_DELAY);
        telem.gear=g; telem.regen=r; telem.t_539=t;
        xSemaphoreGive(mtx);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ================= N2K (MCP2515) =================
static inline uint32_t n2kId(uint8_t prio,uint32_t pgn,uint8_t sa){ return ((uint32_t)prio<<26)|(pgn<<8)|sa; }
static const uint8_t N2K_PRIO=6;

bool n2kInit(){
  SPI.begin(MCP_SCK,MCP_MISO,MCP_MOSI);
  if (MCP.begin(MCP_ANY,MCP_BAUD,MCP_CLOCK)!=CAN_OK) return false;
  // Filters: mask priority+PGN, ignore SA; use one mask per PGN
  const uint32_t MASK = 0x1FFFFF00UL;
  const uint32_t ID_STW_BASE  = n2kId(N2K_PRIO,PGN_STW,0)&0x1FFFFFFF;
  const uint32_t ID_WIND_BASE = n2kId(N2K_PRIO,PGN_WIND,0)&0x1FFFFFFF;

  MCP.init_Mask(0,1,MASK);
  MCP.init_Filt(0,1,ID_STW_BASE);
  MCP.init_Filt(1,1,ID_STW_BASE);

  MCP.init_Mask(1,1,MASK);
  MCP.init_Filt(2,1,ID_WIND_BASE);
  MCP.init_Filt(3,1,ID_WIND_BASE);
  MCP.init_Filt(4,1,ID_WIND_BASE);
  MCP.init_Filt(5,1,ID_WIND_BASE);

  MCP.setMode(MCP_NORMAL);
  pinMode(MCP_INT,INPUT);
  return true;
}

void n2kHandleSTW(const uint8_t* d,uint8_t len){
  if (len<3) return;
  uint16_t raw = (uint16_t)d[1]|((uint16_t)d[2]<<8);     // 0.01 m/s
  if (raw==0xFFFF) return;
  float mps = raw*0.01f; float kn = mps*1.94384449f;
  telem.STW_kn = kn; telem.t_STW=nowMs(); telem.stwMM.update(kn); telem.stwHist.push(kn);
}

void computeTrueWind(){
  // Requires AWS/AWA and STW; computes TWS/TWA (through water, boat-axis relative)
  float aws = telem.AWS_kn;
  float awa_deg = telem.AWA_deg;
  float stw = telem.STW_kn;
  if (aws<=0 || stw<0) { return; }

  float aws_ms = aws / 1.94384449f;
  float stw_ms = stw / 1.94384449f;
  float awa_rad = awa_deg * DEG_TO_RAD;

  // V_true = V_app + V_boat (boat x forward, awa measured from bow, starboard positive)
  float vax = aws_ms * cosf(awa_rad);
  float vay = aws_ms * sinf(awa_rad);
  float vtx = vax + stw_ms;
  float vty = vay;

  float tws_ms = sqrtf(vtx*vtx + vty*vty);
  float twa_rad = atan2f(vty, vtx);
  telem.TWS_kn = tws_ms * 1.94384449f;
  telem.TWA_deg = twa_rad * RAD_TO_DEG;

  telem.twsMM.update(telem.TWS_kn);
  telem.twaMM.update(telem.TWA_deg);
  telem.twsHist.push(telem.TWS_kn);
}

void n2kHandleWIND(const uint8_t* d,uint8_t len){
  if (len<6) return;
  // PGN 130306: Byte0 SID
  uint16_t spd_raw = (uint16_t)d[1]|((uint16_t)d[2]<<8);        // 0.01 m/s
  uint16_t ang_raw = (uint16_t)d[3]|((uint16_t)d[4]<<8);        // 0.0001 rad
  uint8_t  ref     = d[5];                                      // 0=True(North),1=Mag,2=Apparent

  if (spd_raw==0xFFFF || ang_raw==0xFFFF) return;
  float mps = spd_raw * 0.01f;
  float rad = ang_raw * 0.0001f;
  // Normalize angle to [-pi, pi] then to degrees
  while (rad >  M_PI) rad -= 2*M_PI;
  while (rad < -M_PI) rad += 2*M_PI;
  float deg = rad * RAD_TO_DEG;

  if (ref==2) {
    telem.AWS_kn  = mps * 1.94384449f;
    telem.AWA_deg = deg;
    telem.awsMM.update(telem.AWS_kn);
    telem.awaMM.update(telem.AWA_deg);
    telem.awsHist.push(telem.AWS_kn);
    telem.t_WIND = nowMs();
    computeTrueWind(); // uses latest STW + AWA/AWS
  } else if (ref==0) {
    // True wind provided by instrument; still store
    telem.TWS_kn  = mps * 1.94384449f;
    telem.TWA_deg = deg;
    telem.twsMM.update(telem.TWS_kn);
    telem.twaMM.update(telem.TWA_deg);
    telem.t_WIND = nowMs();
    telem.twsHist.push(telem.TWS_kn);
  }
}

void taskN2k(void*){
  for(;;){
    // Drain RX
    while (digitalRead(MCP_INT)==LOW){
      long unsigned int id; unsigned char len; unsigned char db[8];
      if (MCP.readMsgBuf(&id,&len,db)!=CAN_OK) break;
      if (!MCP.isExtendedFrame()) continue;
      uint32_t pgn = (id>>8)&0x1FFFF;
      if (pgn==PGN_STW)  n2kHandleSTW(db,len);
      if (pgn==PGN_WIND) n2kHandleWIND(db,len);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ================= Sampling & Output =================
void taskSampler(void*){
  const uint32_t period = 1000/SAMPLE_HZ;
  for(;;){
    // push selected signals into history (some already pushed in handlers)
    telem.vHist.push(telem.V);
    telem.kWHist.push(telem.kW);
    telem.rpmHist.push((float)telem.RPM);
    vTaskDelay(pdMS_TO_TICKS(period));
  }
}

void sendToNextion(){
  // Numbers
  nxSetNum("nV",   telem.V);
  nxSetNum("nSOC", telem.SOC);
  nxSetNum("nRPM", telem.RPM);
  nxSetNum("nK",   telem.kWh);
  nxSetNum("nP",   telem.kW);
  nxSetNum("nMT",  telem.MT);
  nxSetNum("nS",   telem.STW_kn);
  nxSetTxt("tGear",  gearStr(telem.gear));
  nxSetTxt("tRegen", regenStr(telem.regen));
  nxSetNum("nAWS", telem.AWS_kn);
  nxSetNum("nAWA", telem.AWA_deg);
  nxSetNum("nTWS", telem.TWS_kn);
  nxSetNum("nTWA", telem.TWA_deg);

#if SEND_WAVEFORMS
  // Example: push scaled values (0..255) to waveforms with objId
  // Map ranges as needed
  uint8_t p255  = (uint8_t)constrain(lroundf((telem.kW + 50.0f)*255.0f/100.0f),0,255); // -50..+50 kW -> 0..255
  uint8_t rpm255= (uint8_t)constrain(lroundf((float)telem.RPM/8000.0f*255.0f),0,255);  // 0..8000 rpm
  uint8_t aws255= (uint8_t)constrain(lroundf(telem.AWS_kn/40.0f*255.0f),0,255);        // 0..40 kn
  uint8_t tws255= (uint8_t)constrain(lroundf(telem.TWS_kn/40.0f*255.0f),0,255);

  // change objId/channel to your HMI waveform ids
  nxWaveAdd(1,0,p255);   // wP ch0
  nxWaveAdd(2,0,rpm255); // wRPM ch0
  nxWaveAdd(3,0,aws255); // wAWS ch0
  nxWaveAdd(4,0,tws255); // wTWS ch0
#endif
}

void taskOutput(void*){
  const uint32_t period = 1000/OUT_HZ;
  for(;;){
    // CSV line (keeps your old format, with wind fields appended)
    Serial.print("V:");  Serial.print(telem.V);       Serial.print(",");
    Serial.print("SOC:");Serial.print(telem.SOC);     Serial.print(",");
    Serial.print("RPM:");Serial.print(telem.RPM);     Serial.print(",");
    Serial.print("G:");  Serial.print(gearStr(telem.gear));  Serial.print(",");
    Serial.print("K:");  Serial.print(telem.kWh);     Serial.print(",");
    Serial.print("P:");  Serial.print(telem.kW);      Serial.print(",");
    Serial.print("MT:"); Serial.print(telem.MT);      Serial.print(",");
    Serial.print("U:");  Serial.print(regenStr(telem.regen)); Serial.print(",");
    Serial.print("S:");  Serial.print(telem.STW_kn);  Serial.print(",");
    Serial.print("AWS:");Serial.print(telem.AWS_kn);  Serial.print(",");
    Serial.print("AWA:");Serial.print(telem.AWA_deg); Serial.print(",");
    Serial.print("TWS:");Serial.print(telem.TWS_kn);  Serial.print(",");
    Serial.print("TWA:");Serial.print(telem.TWA_deg);
    Serial.println();

    // Nextion updates
    sendToNextion();

    vTaskDelay(pdMS_TO_TICKS(period));
  }
}

// ================= Setup / Loop =================
void setup(){
  Serial.begin(USB_BAUD);
  Nextion.begin(NEXTION_BAUD, SERIAL_8N1, NX_RX, NX_TX);
  mtx = xSemaphoreCreateMutex();

  if (!twaiInit()){ Serial.println("EV-CAN init FAILED"); for(;;) delay(1000); }
  if (!n2kInit()){  Serial.println("N2K init FAILED");    for(;;) delay(1000); }

  // Tasks
  xTaskCreatePinnedToCore(taskEvRx,   "EV-RX",   4096, nullptr, 20, nullptr, 1);
  xTaskCreatePinnedToCore(taskN2k,    "N2K-RX",  4096, nullptr, 10, nullptr, 1);
  xTaskCreatePinnedToCore(taskSampler,"SAMPLE",  4096, nullptr,  5, nullptr, 0);
  xTaskCreatePinnedToCore(taskOutput, "OUTPUT",  6144, nullptr,  5, nullptr, 0);

  // Optional: tell Nextion to go to a specific page
  // nxSendRaw("page 0");
}

void loop(){ /* all work in tasks */ }
