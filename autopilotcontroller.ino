/*
  ESP32 OpenCPN ⇄ NMEA2000 ⇄ SeaTalk-1 bridge for Raymarine ST4000
  Focus: follow-track + full AP controls, minimal logs with raw SeaTalk bytes.

  Logs:
    [AP RX] ...     ← N2K autopilot commands received (126208, etc.)
    [ST TX] ...     ← SeaTalk datagrams sent (raw bytes in hex)

  Pins:
    CAN (TWAI):  TXD=GPIO5, RXD=GPIO4  → SN65HVD230, 250 kbps, proper termination.
    SeaTalk-1:   TX = GPIO17 → level shifter/transistor → yellow. GND common.
*/

#include <Arduino.h>
#include <driver/twai.h>
#include <driver/uart.h>
#include <math.h>
#include <string.h>

static const gpio_num_t CAN_TX = GPIO_NUM_5;
static const gpio_num_t CAN_RX = GPIO_NUM_4;

static const int ST_UART_NUM = UART_NUM_2;
static const int ST_TX_PIN   = 17;
static const int ST_BAUD     = 4800;

HardwareSerial &dbg = Serial;

#define APLOG(...) do{ dbg.printf(__VA_ARGS__); }while(0)
#define STLOG(...) do{ dbg.printf(__VA_ARGS__); }while(0)

#ifndef ESP_IDF_VERSION_MAJOR
  #include <esp_idf_version.h>
#endif
#if ESP_IDF_VERSION_MAJOR >= 5
  #define TWAI_TX_ERR(s)   ((s).tx_error_counter)
  #define TWAI_RX_ERR(s)   ((s).rx_error_counter)
  #define TWAI_BUS_ERR(s)  ((s).bus_error_count)
  #define TWAI_ARB_LOST(s) ((s).arb_lost_count)
#else
  #define TWAI_TX_ERR(s)   ((s).tx_err_cnt)
  #define TWAI_RX_ERR(s)   ((s).rx_err_cnt)
  #define TWAI_BUS_ERR(s)  ((s).bus_error_cnt)
  #define TWAI_ARB_LOST(s) ((s).arb_lost_cnt)
#endif

// Modes/refs
enum SteeringMode : uint8_t { SM_STANDBY=0, SM_AUTO=1, SM_WIND=2, SM_TRACK=3 };
enum HeadingRef   : uint8_t { REF_TRUE=0,   REF_MAG=1 };
enum PilotMode    : uint8_t { PM_STBY=0,    PM_AUTO=1, PM_WIND=2, PM_TRACK=3 };

// Current state reflected to both sides
static volatile PilotMode currentMode = PM_STBY;
static volatile HeadingRef headingRef = REF_MAG;

// Live data for 0x85 frame
static float hdg_deg_now = 0.0f;       // from 127250
static float locked_hdg_deg = NAN;     // from 65360 / set at AUTO/TRACK
static float xte_nm = 0.0f;            // from 129283
static float dtw_nm = 0.0f;            // (optional) from 129284 if decoded
static float cog_mag_deg=0.0f, sog_kn=0.0f;
static float awa_deg=0.0f, aws_kn=0.0f, stw_kn=0.0f;

static uint32_t lastRM=0, lastAP=0, lastST=0, lastAnnounce=0, lastHB=0, lastPI=0;

// N2K PGNs
static const uint32_t PGN_ADDR_CLAIM   = 60928;
static const uint32_t PGN_ISO_REQUEST  = 59904;
static const uint32_t PGN_ISO_ACK      = 59392;
static const uint32_t PGN_CMD_GROUP    = 126208;
static const uint32_t PGN_PGN_LIST     = 126464;
static const uint32_t PGN_HEARTBEAT    = 126993;
static const uint32_t PGN_PRODUCT_INFO = 126996;
static const uint32_t PGN_CONFIG_INFO  = 126998;

static const uint32_t PGN_AP_STATUS    = 127237;
static const uint32_t PGN_VESSEL_HEAD  = 127250;
static const uint32_t PGN_STW          = 128259;
static const uint32_t PGN_COGSOG       = 129026;
static const uint32_t PGN_XTE          = 129283;
static const uint32_t PGN_NAVDATA      = 129284;
static const uint32_t PGN_ROUTE_WP     = 129285;
static const uint32_t PGN_WIND         = 130306;

// Raymarine proprietary (Seatalk NG / N2K)
static const uint32_t PGN_RM_PILOT_MODE     = 65379; // 0x00FF63
static const uint32_t PGN_RM_LOCKED_HEADING = 65360; // 0x00FF50
static const uint32_t PGN_RM_KEYPAD_HEART   = 65374; // 0x00FF5E

// Device identity (mimic Raymarine AP so plugin "sees" a pilot)
static const uint8_t  SA             = 33;
static const uint16_t N2K_MFG        = 1851;    // Raymarine
static const uint8_t  N2K_IND_GR     = 4;
static const uint8_t  N2K_DEV_CLASS  = 40;      // Steering
static const uint8_t  N2K_FUNC       = 135;     // Autopilot
static const uint8_t  N2K_FUNC_INST  = 0;
static const uint8_t  N2K_DEV_INST   = 0;
static const bool     N2K_AAC        = true;
static const uint32_t N2K_UNIQUE     = 0x37AC5;

static const uint16_t N2K_VERSION    = 1301;
static const uint16_t PRODUCT_CODE   = 20001;

static const char     MODEL_ID[]     = "E70010";
static const char     SW_VERSION[]   = "v1.0.0";
static const char     MODEL_VER[]    = "Raymarine AP (emu)";
static const char     SERIAL_NO[]    = "RM-EMU-001";
static const uint8_t  CERT_LEVEL     = 1;
static const uint8_t  LEN_UNITS_50mA = 3;

// utils
static inline void put_u16le(uint8_t *p, uint16_t v){ p[0]=v&0xFF; p[1]=(v>>8)&0xFF; }
static inline void put_i16le(uint8_t *p, int16_t v){ p[0]=v&0xFF; p[1]=(v>>8)&0xFF; }
static inline void put_u32le(uint8_t *p, uint32_t v){ p[0]=v&0xFF; p[1]=(v>>8)&0xFF; p[2]=(v>>16)&0xFF; p[3]=(v>>24)&0xFF; }
static inline float  clampf(float v, float a, float b){ return v<a?a:(v>b?b:v); }

// ---------------- N2K helpers ----------------
static inline uint32_t n2k_id(uint8_t prio, uint32_t pgn, uint8_t src){
  uint32_t PF = (pgn >> 8) & 0xFF;
  uint32_t PDU_SPEC = (PF < 240) ? 0xFF : (pgn & 0xFF); // avoid PS macro clash
  uint32_t DP = (pgn >> 16) & 0x01;
  return ((uint32_t)(prio & 7) << 26) | (DP << 24) | (PF << 16) | (PDU_SPEC << 8) | src;
}
static inline uint32_t extractPGN(const twai_message_t &m){
  uint32_t id = m.identifier;
  uint8_t PF = (id >> 16) & 0xFF;
  uint8_t DP = (id >> 24) & 0x01;
  uint8_t PDU_SPEC = (id >> 8)  & 0xFF;
  if (PF < 240) return ((uint32_t)DP<<16) | ((uint32_t)PF<<8);
  return ((uint32_t)DP<<16) | ((uint32_t)PF<<8) | PDU_SPEC;
}
static bool twai_tx(uint32_t id, const uint8_t *data, uint8_t len){
  twai_message_t msg = {};
  msg.identifier = id; msg.extd = 1; msg.data_length_code = len;
  if (data) memcpy(msg.data, data, len);
  return twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK;
}
static void n2k_send_single(uint32_t pgn, const uint8_t *payload, uint8_t len, uint8_t prio=6){
  twai_tx(n2k_id(prio, pgn, SA), payload, len);
}
static void n2k_send_fast(uint32_t pgn, const uint8_t *payload, uint16_t len, uint8_t prio=6){
  uint8_t seq = (uint8_t)((millis()>>5) & 0x07);
  uint32_t id = n2k_id(prio, pgn, SA);
  uint8_t f[8]; uint16_t sent=0; uint8_t idx=1;

  memset(f,0xFF,8);
  f[0]=(uint8_t)((seq<<5)|0);
  f[1]=(uint8_t)len;
  uint8_t take = (len>6)?6:len;
  memcpy(f+2, payload, take);
  twai_tx(id, f, 8); sent=take;

  while (sent < len){
    memset(f,0xFF,8);
    f[0]=(uint8_t)((seq<<5)|(idx & 0x1F));
    uint8_t t = (len - sent > 7) ? 7 : (uint8_t)(len - sent);
    memcpy(f+1, payload+sent, t);
    twai_tx(id, f, 8);
    sent += t; idx++; delay(3);
  }
}

// --------------- Identity frames ---------------
static uint64_t buildNAME(){
  uint64_t name=0;
  name |= ((uint64_t)(N2K_UNIQUE    & 0x1FFFFF)) << 0;
  name |= ((uint64_t)(N2K_MFG       & 0x07FF))   << 21;
  name |= ((uint64_t)(N2K_DEV_INST  & 0x07))     << 32;
  name |= ((uint64_t)(N2K_FUNC_INST & 0x1F))     << 35;
  name |= ((uint64_t)(N2K_FUNC      & 0xFF))     << 40;
  name |= ((uint64_t)0x1)                        << 48;
  name |= ((uint64_t)(N2K_DEV_CLASS & 0x7F))     << 49;
  name |= ((uint64_t)0x0)                        << 56;
  name |= ((uint64_t)(N2K_IND_GR    & 0x07))     << 60;
  name |= ((uint64_t)(N2K_AAC ? 1:0))            << 63;
  return name;
}
static void send_60928_AddressClaim(){
  uint8_t p[8]; uint64_t nm=buildNAME();
  for(int i=0;i<8;i++) p[i]=(uint8_t)((nm>>(8*i))&0xFF);
  n2k_send_single(PGN_ADDR_CLAIM, p, 8, 6);
}
static void pad32(char *dst, const char *src){ memset(dst,0,32); size_t n=strnlen(src,32); memcpy(dst,src,n); }
static void send_126996_Product(){
  uint8_t p[2+2 + 32+32+32+32 + 1+1]; memset(p,0,sizeof p);
  put_u16le(p+0, N2K_VERSION);
  put_u16le(p+2, PRODUCT_CODE);
  pad32((char*)p+4,   MODEL_ID);
  pad32((char*)p+36,  SW_VERSION);
  pad32((char*)p+68,  MODEL_VER);
  pad32((char*)p+100, SERIAL_NO);
  p[132]=CERT_LEVEL; p[133]=LEN_UNITS_50mA;
  n2k_send_fast(PGN_PRODUCT_INFO, p, sizeof p, 6);
}
static void send_126998_Config(){
  auto emitLAU=[&](const char* s, uint8_t *out)->int{
    size_t sl=strlen(s); out[0]=(uint8_t)(sl+1); out[1]=0x01; memcpy(out+2,s,sl); return (int)(2+sl);
  };
  uint8_t buf[160]; int o=0;
  o+=emitLAU("Install: ESP32 OCPN->ST4000 bridge", buf+o);
  o+=emitLAU("Mfr: Raymarine (emu)",               buf+o);
  n2k_send_fast(PGN_CONFIG_INFO, buf, o, 6);
}
static void send_126464_PGN_List(){
  const uint32_t TX[] = { PGN_AP_STATUS, PGN_VESSEL_HEAD, PGN_PRODUCT_INFO, PGN_CONFIG_INFO,
                          PGN_RM_PILOT_MODE, PGN_RM_LOCKED_HEADING, PGN_RM_KEYPAD_HEART };
  uint8_t a[sizeof(TX)]; for(size_t i=0;i<sizeof(TX)/4;i++) put_u32le(a+4*i, TX[i]);
  n2k_send_fast(PGN_PGN_LIST, a, sizeof a, 6);

  const uint32_t RX[] = { PGN_CMD_GROUP, PGN_WIND, PGN_STW, PGN_COGSOG, PGN_XTE, PGN_NAVDATA, PGN_ROUTE_WP,
                          PGN_ISO_REQUEST, PGN_VESSEL_HEAD };
  uint8_t b[sizeof(RX)]; for(size_t i=0;i<sizeof(RX)/4;i++) put_u32le(b+4*i, RX[i]);
  n2k_send_fast(PGN_PGN_LIST, b, sizeof b, 6);
}

// --------------- Raymarine proprietary ---------------
static inline uint16_t rm_mfg_word_only(){ return (uint16_t)(1851 & 0x07FF); }
static uint16_t rmModeField2(PilotMode m){
  switch(m){case PM_STBY:return 0x0000; case PM_AUTO:return 0x0040; case PM_WIND:return 0x0100; case PM_TRACK:return 0x0180;}
  return 0;
}
static uint8_t rmModeDataByte(PilotMode m){
  switch(m){case PM_STBY:return 0x00; case PM_AUTO:return 0x40; case PM_WIND:return 0x01; case PM_TRACK:return 0x80;}
  return 0x00;
}
static uint16_t deg_to_rad1e4_u16(float deg){
  while (deg<0) deg+=360.0f; while(deg>=360) deg-=360.0f;
  float rad = deg * (float)M_PI / 180.0f;
  uint32_t v = (uint32_t)roundf(rad*10000.0f); if (v>0xFFFF) v=0xFFFF; return (uint16_t)v;
}
static void send_65360_locked_heading(float hdg){
  if (isnan(hdg)) return;
  uint8_t p[8]; memset(p,0xFF,8);
  uint16_t comp=rm_mfg_word_only(), ang=deg_to_rad1e4_u16(hdg);
  p[0]=(uint8_t)(comp&0xFF); p[1]=(uint8_t)(comp>>8);
  p[3]=(uint8_t)(ang&0xFF);  p[4]=(uint8_t)(ang>>8); // True
  p[5]=(uint8_t)(ang&0xFF);  p[6]=(uint8_t)(ang>>8); // Magnetic
  n2k_send_single(PGN_RM_LOCKED_HEADING, p, 8, 3);
}
static void send_65379_mode(PilotMode m){
  uint8_t p[8]; memset(p,0xFF,8);
  uint16_t comp=rm_mfg_word_only(), mode2=rmModeField2(m);
  p[0]=(uint8_t)(comp&0xFF); p[1]=(uint8_t)(comp>>8);
  p[2]=(uint8_t)(mode2&0xFF); p[3]=(uint8_t)(mode2>>8);
  p[6]=rmModeDataByte(m);
  n2k_send_single(PGN_RM_PILOT_MODE, p, 8, 3);
}
static void send_65374_keypad_heartbeat(){
  uint8_t p[8]; memset(p,0xFF,8);
  uint16_t comp=rm_mfg_word_only(); p[0]=(uint8_t)(comp&0xFF); p[1]=(uint8_t)(comp>>8);
  p[2]=0x00; p[3]=0x01; p[4]=0x00;
  n2k_send_single(PGN_RM_KEYPAD_HEART, p, 8, 6);
}
static void send_127237(SteeringMode mode, HeadingRef ref, float hts_deg){
  uint8_t p[21]; memset(p,0xFF,21);
  p[0]=0x00; // SID
  uint8_t steer=0; switch(mode){case SM_STANDBY:steer=0;break;case SM_AUTO:steer=4;break;case SM_WIND:steer=4;break;case SM_TRACK:steer=5;break;}
  uint8_t refBits=(ref==REF_TRUE?0:1);
  p[1]=(uint8_t)((steer&0x7) | ((refBits&0x3)<<6));
  float h=fmodf(hts_deg,360.0f); if(h<0)h+=360.0f;
  int16_t q=(int16_t)lrintf((h*(float)M_PI/180.0f)*10000.0f);
  put_i16le(p+13, q); // Heading-To-Steer
  n2k_send_fast(PGN_AP_STATUS, p, 21, 2);
}
static void send_127250(float heading_deg_now, HeadingRef ref){
  uint8_t d[8]; for(int i=0;i<8;i++) d[i]=0xFF;
  d[0]=0x00; // SID
  float hdg=fmodf(heading_deg_now,360.0f); if(hdg<0)hdg+=360.0f;
  int16_t q=(int16_t)lrintf((hdg*(float)M_PI/180.0f)*10000.0f);
  put_i16le(d+1, q); // heading at bytes 1..2
  d[7]=(ref==REF_TRUE)?0:1;
  n2k_send_single(PGN_VESSEL_HEAD, d, 8, 2);
}

// --------------- SeaTalk TX (UART2 parity flip) + hex log ---------------
static void st_begin(){
  uart_config_t cfg{};
  cfg.baud_rate = ST_BAUD;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity    = UART_PARITY_EVEN; // data bytes EVEN; flip to ODD for header
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_APB;
  uart_driver_install((uart_port_t)ST_UART_NUM, 256, 0, 0, NULL, 0);
  uart_param_config((uart_port_t)ST_UART_NUM, &cfg);
  uart_set_pin((uart_port_t)ST_UART_NUM, ST_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
static void print_hex_bytes(const uint8_t* d, size_t n){
  for(size_t i=0;i<n;i++){ dbg.printf("%s%02X", (i?" ":""), d[i]); }
  dbg.println();
}
static inline void st_write_datagram(const uint8_t *data, size_t len){
  if (!data || !len) return;
  uart_set_parity((uart_port_t)ST_UART_NUM, UART_PARITY_ODD);
  uart_write_bytes((uart_port_t)ST_UART_NUM, (const char*)data, 1);
  uart_wait_tx_done((uart_port_t)ST_UART_NUM, pdMS_TO_TICKS(10));
  if (len>1){
    uart_set_parity((uart_port_t)ST_UART_NUM, UART_PARITY_EVEN);
    uart_write_bytes((uart_port_t)ST_UART_NUM, (const char*)(data+1), len-1);
    uart_wait_tx_done((uart_port_t)ST_UART_NUM, pdMS_TO_TICKS(6));
  }
}

// ---- SeaTalk keystrokes (0x86)
#define ST_CODE_AUTO      0x01
#define ST_CODE_STANDBY   0x02
#define ST_CODE_PLUS1     0x07
#define ST_CODE_PLUS10    0x08
#define ST_CODE_MINUS1    0x05
#define ST_CODE_MINUS10   0x06
static const uint8_t ST_SENDER_NIB = 0x2;

static void st_send_keystroke(uint8_t code){
  const uint8_t b0=0x86, b1=(ST_SENDER_NIB<<4)|0x01, b2=code, b3=(uint8_t)~code;
  uint8_t p[4]={b0,b1,b2,b3};
  st_write_datagram(p,4);
  const char* name = (code==ST_CODE_AUTO)?"AUTO":
                     (code==ST_CODE_STANDBY)?"STBY":
                     (code==ST_CODE_PLUS10)?"+10":
                     (code==ST_CODE_MINUS10)?"-10":
                     (code==ST_CODE_PLUS1)?"+1":
                     (code==ST_CODE_MINUS1)?"-1":"?";
  STLOG("[ST TX] Key %-4s bytes: ", name);
  print_hex_bytes(p,4);
}
static void st_press(uint8_t code, uint8_t repeats=2, uint16_t spacing_ms=120){
  for (uint8_t i=0;i<repeats;i++){ st_send_keystroke(code); delay(spacing_ms); }
}

// ---- SeaTalk NAV 0x85 (XTE/BRG/DST/Track flag)
static void st_send_nav_0x85(float xteNm, float bearingMagDeg, float distNm, bool enterTrack){
  float xabs=fabsf(xteNm);
  uint16_t X=(uint16_t)roundf(clampf(xabs,0.0f,40.95f)*100.0f);
  uint8_t X_hi=(X>>8)&0x0F, X_lo=X&0xFF;

  float brg=fmodf(bearingMagDeg+360.0f,360.0f);
  uint8_t quad=(uint8_t)floorf(brg/90.0f)&0x03; uint8_t U=quad; // magnetic variant
  float rem=brg - (float)quad*90.0f;
  uint8_t WV=(uint8_t)roundf(rem*2.0f); uint8_t V=WV&0x0F, W=(WV>>4)&0x0F;

  bool lt10=(distNm<10.0f);
  uint16_t D=(uint16_t)roundf(clampf(distNm,0.0f,409.5f)*(lt10?100.0f:10.0f));
  uint8_t Z_hi=(D>>8)&0x0F, ZZ=(D>>4)&0xFF, Y=D&0x0F;
  if (lt10) Y|=0x01;
  if (xteNm<0) Y|=0x04;             // NEGATIVE = steer RIGHT
  uint8_t F = enterTrack ? 0x07 : 0x05;

  uint8_t p[9]={
    0x85,
    (uint8_t)(0x60 | X_hi),
    X_lo,
    (uint8_t)((V<<4)|U),
    (uint8_t)((Z_hi<<4)|W),
    ZZ,
    (uint8_t)((Y<<4)|F),
    0x00,
    (uint8_t)((Y<<4)|F)
  };
  st_write_datagram(p,9);
  STLOG("[ST TX] Nav 0x85 XTE=%.3f nm BRG=%.1f° DST=%.2f nm TRACK=%s bytes: ",
        xteNm, bearingMagDeg, distNm, enterTrack?"yes":"no");
  print_hex_bytes(p,9);
}

// ---------------- 126208 handling ----------------
struct FPBuf {
  bool active=false; uint8_t sa=0xFF, seq=0xFF, total=0, next=0, filled=0;
  uint8_t buf[120];
} grp;

static void gf_reset(){ grp={}; grp.active=false; }

static void handle_126208_request(uint32_t target){
  switch(target){
    case 65379: { uint8_t p[8]; memset(p,0xFF,8); n2k_send_single(65379,p,8,3);} return;
    case 65360: if (!isnan(locked_hdg_deg)) send_65360_locked_heading(locked_hdg_deg); return;
    case 126996: send_126996_Product(); return;
    case 60928:  send_60928_AddressClaim(); return;
    default: return;
  }
}
static bool gf65360_extractMagHeading(const uint8_t* b, int n, float &deg_out){
  for (int i=0;i<=n-3;i++){
    if (b[i]==0x06){
      uint16_t raw=(uint16_t)b[i+1]|((uint16_t)b[i+2]<<8);
      deg_out = (raw*0.0001f)*180.0f/(float)M_PI; return true;
    }
  }
  return false;
}
static bool apply_rm_selector_or_button(const uint8_t* b, int n){
  bool did=false;
  // selector: 04 04 <sel>
  for (int i=0; i<=n-3; i++){
    if (b[i]==0x04 && b[i+1]==0x04){
      uint8_t sel=b[i+2];
      PilotMode nm=currentMode;
      if      (sel==0x00) nm=PM_STBY;
      else if (sel==0x40) nm=PM_AUTO;
      else if (sel==0x01) nm=PM_WIND;
      else if (sel==0x80) nm=PM_TRACK;

      if (nm!=currentMode){
        currentMode=nm; headingRef=REF_MAG;
        const char* nms = (nm==PM_STBY)?"STBY":(nm==PM_AUTO)?"AUTO":(nm==PM_WIND)?"WIND":"TRACK";
        APLOG("[AP RX] Mode=%s (126208 04 04 %02X)\n", nms, sel);

        if (nm==PM_AUTO){ st_press(ST_CODE_AUTO);  locked_hdg_deg=hdg_deg_now; }
        if (nm==PM_STBY){ st_press(ST_CODE_STANDBY); locked_hdg_deg=NAN; }
        if (nm==PM_TRACK){ st_press(ST_CODE_AUTO); locked_hdg_deg=hdg_deg_now; } // accept track

        send_65379_mode(currentMode);
      }
      did=true;
    }
  }
  // buttons: 06 <code>
  for (int i=0; i<=n-3; i++){
    if (b[i]==0x06){
      uint8_t code=b[i+1];
      switch(code){
        case 0x00: currentMode=PM_STBY; st_press(ST_CODE_STANDBY); locked_hdg_deg=NAN; APLOG("[AP RX] STBY (126208 06 00)\n"); send_65379_mode(currentMode); did=true; break;
        case 0x40: currentMode=PM_AUTO; st_press(ST_CODE_AUTO);    locked_hdg_deg=hdg_deg_now; APLOG("[AP RX] AUTO (126208 06 40)\n"); send_65379_mode(currentMode); did=true; break;
        case 0x01: currentMode=PM_WIND; APLOG("[AP RX] WIND (126208 06 01)\n"); send_65379_mode(currentMode); did=true; break;
        case 0x80: currentMode=PM_TRACK;st_press(ST_CODE_AUTO);    locked_hdg_deg=hdg_deg_now; APLOG("[AP RX] TRACK(126208 06 80)\n"); send_65379_mode(currentMode); did=true; break;
        case 0x51: st_press(ST_CODE_PLUS1);   APLOG("[AP RX] +1   (126208 06 51)\n"); did=true; break;
        case 0x7F: st_press(ST_CODE_MINUS1);  APLOG("[AP RX] -1   (126208 06 7F)\n"); did=true; break;
        case 0xD1: st_press(ST_CODE_PLUS10);  APLOG("[AP RX] +10  (126208 06 D1)\n"); did=true; break;
        case 0x50: st_press(ST_CODE_MINUS10); APLOG("[AP RX] -10  (126208 06 50)\n"); did=true; break;
        default: break;
      }
    }
  }
  return did;
}
static void handle_126208_full(const uint8_t* b, int n){
  if (n<4) return;
  uint8_t func = b[0];
  uint32_t target = (uint32_t)b[1] | ((uint32_t)b[2]<<8) | ((uint32_t)b[3]<<16);

  if (func==0x01){ handle_126208_request(target); return; }

  if (target==PGN_RM_PILOT_MODE || target==PGN_RM_LOCKED_HEADING){
    bool acted = apply_rm_selector_or_button(b+4, n-4);
    if (acted){ APLOG("[AP RX] 126208 bytes: "); for(int i=0;i<n;i++) dbg.printf("%s%02X",(i?" ":""), b[i]); dbg.println(); }
    if (target==PGN_RM_LOCKED_HEADING){
      float newH;
      if (gf65360_extractMagHeading(b+4, n-4, newH)){
        while (newH<0)newH+=360.0f; while(newH>=360)newH-=360.0f;
        locked_hdg_deg=newH;
        APLOG("[AP RX] Set Locked Heading %.1f° (65360)\n", locked_hdg_deg);
      }
    }
  }
}
static void feed_126208(uint8_t sa, const uint8_t* d, int n){
  if (n<2) return;
  uint8_t fi=d[0]&0x1F, seq=(d[0]>>5)&0x07;
  if (fi==0){
    gf_reset(); grp.active=true; grp.sa=sa; grp.seq=seq; grp.next=1; grp.filled=0;
    grp.total=d[1]; if (grp.total>sizeof(grp.buf)) { gf_reset(); return; }
    int take=min(6,(int)grp.total); memcpy(grp.buf,d+2,take); grp.filled=take;
    if (grp.filled>=grp.total){ handle_126208_full(grp.buf,grp.total); gf_reset(); }
  }else if (grp.active && grp.sa==sa && grp.seq==seq && fi==grp.next){
    grp.next++; int remain=(int)grp.total-(int)grp.filled; int take=min(7,remain);
    memcpy(grp.buf+grp.filled,d+1,take); grp.filled+=take;
    if (grp.filled>=grp.total){ handle_126208_full(grp.buf,grp.total); gf_reset(); }
  }else gf_reset();
}
// Detect single-frame vs fast-packet 126208
static void handle_126208_entry(uint8_t sa, const uint8_t* d, int n){
  if (n<=0) return;
  if ((d[0] & 0xE0) >= 0xA0) { // A0..E0.. → single-frame
    handle_126208_full(d, n);
  } else {
    feed_126208(sa, d, n);
  }
}

// ---------------- Setup / Loop ----------------
void setup(){
  dbg.begin(115200); delay(150);
  dbg.println("\nESP32 OCPN<->N2K<->ST4000 (track-ready, AP RX + ST TX hex)");

  // SeaTalk
  st_begin();

  // TWAI
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
  ESP_ERROR_CHECK(twai_start());

  // Identity + capabilities (silent, for OpenCPN to recognize pilot)
  send_60928_AddressClaim();
  send_126996_Product();
  send_126998_Config();
  send_126464_PGN_List();

  // Initial status
  currentMode=PM_STBY; headingRef=REF_MAG; locked_hdg_deg=NAN;
  send_127237(SM_STANDBY, headingRef, 0.0f);
  send_127250(hdg_deg_now, headingRef);
}

void loop(){
  // N2K RX
  twai_message_t m;
  if (twai_receive(&m, pdMS_TO_TICKS(10)) == ESP_OK && m.extd){
    uint32_t pgn=extractPGN(m); uint8_t *b=m.data; int n=m.data_length_code;
    switch(pgn){
      case PGN_CMD_GROUP:   handle_126208_entry((uint8_t)(m.identifier & 0xFF), b, n); break;
      case PGN_ISO_REQUEST: if (n>=3){ uint32_t req=(uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16); handle_126208_request(req);} break;

      case PGN_VESSEL_HEAD: {
        if (n>=3){ uint16_t q=(uint16_t)b[1]|((uint16_t)b[2]<<8); hdg_deg_now=(q*0.0001f)*180.0f/(float)M_PI; if(!isfinite(hdg_deg_now)) hdg_deg_now=0.0f; }
        break;
      }
      case PGN_WIND: {
        if (n>=6){
          uint16_t sp=(uint16_t)(b[1]|(b[2]<<8));
          uint16_t an=(uint16_t)(b[3]|(b[4]<<8));
          aws_kn=(sp==0xFFFF)?0.0f:(sp*0.01f*1.94384f);
          awa_deg=(an==0xFFFF)?0.0f:fmodf(an*0.0001f*180.0f/(float)M_PI,360.0f);
        }
        break;
      }
      case PGN_STW: {
        if (n>=3){ uint16_t raw=(uint16_t)b[1]|((uint16_t)b[2]<<8); float mps=(raw==0xFFFF)?0.0f:(raw*0.01f); stw_kn=mps*1.94384f; }
        break;
      }
      case PGN_COGSOG: {
        if (n>=7){
          uint16_t cogr=(uint16_t)b[1]|((uint16_t)b[2]<<8);
          uint16_t sogc=(uint16_t)b[5]|((uint16_t)b[6]<<8);
          cog_mag_deg=fmodf((cogr*0.0001f*180.0f/(float)M_PI),360.0f);
          sog_kn=(sogc==0xFFFF)?0.0f:(sogc*0.01f*1.94384f);
        }
        break;
      }
      case PGN_XTE: {
        if (n>=8){ float meters = 0.0f; memcpy(&meters, b+0, sizeof(float)); xte_nm = meters / 1852.0f; }
        break;
      }
      case PGN_NAVDATA: {
        // Optional: If you want DTW/BRG from 129284, decode here (spec offsets differ by variants).
        // This sketch keeps DTW=0 and uses locked_hdg/cog/hdg for BRG to build 0x85 reliably.
        break;
      }
      case PGN_ROUTE_WP: {
        // Optional: could push ST 0x82 waypoint name here if desired.
        break;
      }
      default: break;
    }
  }

  // Periodic N2K TX (silent)
  uint32_t now=millis();
  if (now - lastRM > 1000){
    if (currentMode!=PM_STBY && !isnan(locked_hdg_deg)) send_65360_locked_heading(locked_hdg_deg);
    lastRM=now;
  }
  if (now - lastAP > 1000){
    float hts = isnan(locked_hdg_deg) ? hdg_deg_now : locked_hdg_deg;
    send_127237((SteeringMode)currentMode, headingRef, hts);
    send_127250(hdg_deg_now, headingRef);
    lastAP=now;
  }
  if (now - lastHB > 3000){ send_65374_keypad_heartbeat(); lastHB=now; }
  if (now - lastAnnounce > 15000){
    send_60928_AddressClaim(); send_126996_Product(); send_126998_Config(); send_126464_PGN_List();
    lastAnnounce=now;
  }
  if (now - lastPI > 2000){ uint8_t hb[8]={0}; n2k_send_single(PGN_HEARTBEAT,hb,8,6); lastPI=now; }

  // Periodic SeaTalk NAV 0x85 (track engine)
  if (now - lastST > 1000){
    // Use locked heading (AUTO/TRACK) or fall back to COG/HDG for the BRG we report
    float brg = (!isnan(locked_hdg_deg)) ? locked_hdg_deg
                 : (cog_mag_deg>0 ? cog_mag_deg : hdg_deg_now);
    st_send_nav_0x85(xte_nm, brg, dtw_nm, currentMode==PM_TRACK);
    lastST=now;
  }
}
