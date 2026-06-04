// =============================================================================
//  round_obd_gauge  -  ESP32-2424S012 (1.28" round GC9A01) OBD-II BLE gauge
//  Multi-gauge + capacitive touch.  Swipe left/right to switch gauges.
//
//  Target board : ESP32-C3-MINI-1U  (SINGLE CORE, no PSRAM, no HW-FPU)
//  Display      : GC9A01 240x240 round IPS, SPI 80MHz
//                 SCLK=6 MOSI=7 DC=2 CS=10 RST=-1   Backlight=GPIO3 (HIGH=on)
//  Touch        : CST816D  SDA=4 SCL=5 INT=0 RST=1  addr 0x15
//  OBD adapter  : ELM327-style BLE UART, name OBD_BLE_NAME (Vgate/vLinker/mock)
//
//  Gauges (swipe to cycle): RPM | SPEED | COOLANT | BOOST | TIMING
//  Reused 1:1 from obd_drive.ino : Kalman filter, OBD parser, BLE client.
//  8-bit full canvas sprite (57.6KB) required to coexist with BLE on the C3.
// =============================================================================

#define LGFX_USE_V1
#include <Arduino.h>
#include <Wire.h>
#include <LovyanGFX.hpp>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <math.h>
#include <stdarg.h>
#include <Preferences.h>
#include "logo_openclaw.h"   // OpenClaw boot logo (140x140 RGB565)

// ---------- user config ------------------------------------------------------
#define OBD_BLE_NAME   "IOS-Vlink"
#define PIN_BL         3
#define RENDER_HZ      60   // upper cap; actual fps is render-bound
#define SHOW_FPS       true
#define DEMO_AFTER_MS  60000   // demo face after 1 min with no BLE
#define ZB_TEST        0       // set 1 only to auto-enter 0-100 mode for bench testing
#define OBD_DEBUG      1       // verbose BLE/OBD diagnostics over serial (set 0 for release)

// 0-100 km/h (제로백) launch-timer mode: double-tap on the SPEED gauge to enter.
#define GAUGE_SPEED    1
#define SETTINGS_PAGE  5       // 6th swipe page (after the 5 gauges) = settings menu
#define ZB_TARGET      100.0f   // finish speed (km/h)
#define ZB_ARM         2.0f     // speed below this = "at zero" (armable)
#define ZB_LAUNCH      2.0f     // crossing this from armed starts the clock

// Fuel-grade inference via KNOCK RETARD under load (not absolute advance, which only
// reflects load state). Physics: high-octane fuel resists knock so the ECU holds timing
// advance even under high MAP/load; low-octane knocks, so the ECU yanks timing back
// (retards) under load. We learn the no-load advance as a baseline, then measure how
// much advance is lost under load. Load is detected from MAP/boost. All tunable.
#define LOAD_TH_BAR    -0.05f   // boost(bar) >= this => "under load" (MAP ~96kPa+)
#define RETARD_REG      6.0f    // knock-retard (deg) >= this => REGULAR
#define RETARD_LOW     16.0f    // >= this => LOW octane

// Real-CAN safety: hard floor on time between OBD requests => aggregate <= 10 req/s.
// Combined with request-response pacing (send next only after prior response/timeout),
// this prevents flooding the ELM327 / injecting excess traffic on the vehicle bus.
#define MIN_REQ_INTERVAL_MS  100
#define REQ_TIMEOUT_MS       250

// ---------- touch (CST816D) pins / regs --------------------------------------
#define TP_SDA   4
#define TP_SCL   5
#define TP_INT   0
#define TP_RST   1
#define TP_ADDR  0x15
enum { G_NONE=0x00, G_SLIDE_DOWN=0x01, G_SLIDE_UP=0x02, G_SLIDE_LEFT=0x03,
       G_SLIDE_RIGHT=0x04, G_TAP=0x05, G_DOUBLE=0x0B, G_LONG=0x0C };

// ---------- BLE UART profile (ELM327 over BLE) -------------------------------
#define SERVICE_UUID   "000018F0-0000-1000-8000-00805f9b34fb"
#define CHAR_RX_UUID   "00002AF0-0000-1000-8000-00805f9b34fb"
#define CHAR_TX_UUID   "00002AF1-0000-1000-8000-00805f9b34fb"

// ---------- gauge type (declared early: .ino auto-prototypes reference it) ---
struct GaugeDef {
  const char* name;
  const char* unit;
  float minv, maxv;
  bool  hasWarn;     // color zones (green/orange/red) vs flat accent
  float warnv;       // value at/above which -> red
  uint8_t decimals;
  float labelDiv;    // divide end-scale labels (RPM shows x1000)
  bool  centerZero;  // true => 0 sits at 12 o'clock, fills both ways (boost)
};

// =============================================================================
//  Display driver (GC9A01)
// =============================================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI      _bus;
public:
  LGFX() {
    { auto c = _bus.config();
      c.spi_host=SPI2_HOST; c.spi_mode=0; c.freq_write=80000000; c.freq_read=20000000;
      c.spi_3wire=true; c.use_lock=true; c.dma_channel=SPI_DMA_CH_AUTO;
      c.pin_sclk=6; c.pin_mosi=7; c.pin_miso=-1; c.pin_dc=2;
      _bus.config(c); _panel.setBus(&_bus); }
    { auto c = _panel.config();
      c.pin_cs=10; c.pin_rst=-1; c.pin_busy=-1;
      c.memory_width=240; c.memory_height=240; c.panel_width=240; c.panel_height=240;
      c.offset_x=0; c.offset_y=0; c.offset_rotation=0;
      c.readable=false; c.invert=true; c.rgb_order=false; c.dlen_16bit=false; c.bus_shared=false;
      _panel.config(c); }
    setPanel(&_panel);
  }
};
LGFX        tft;
LGFX_Sprite canvas(&tft);

// ---------- trig LUT (C3 has no HW-FPU) --------------------------------------
static float cosLut[360], sinLut[360];
static void initLUT() { for (int i=0;i<360;i++){ float r=radians(i); cosLut[i]=cosf(r); sinLut[i]=sinf(r);} }
static inline int wrap360(int a){ a%=360; return a<0?a+360:a; }

// =============================================================================
//  1D Kalman filter (reused from obd_drive.ino)
// =============================================================================
struct KalmanFilter1D {
  float x,v,P[2][2],Q_x,Q_v,R;
  void init(float i,float qx,float qv,float r){ x=i;v=0;P[0][0]=1000;P[0][1]=0;P[1][0]=0;P[1][1]=1000;Q_x=qx;Q_v=qv;R=r; }
  void predict(float dt){
    x+=v*dt;
    float p00=P[0][0]+dt*P[1][0]+dt*(P[0][1]+dt*P[1][1])+Q_x*dt;
    float p01=P[0][1]+dt*P[1][1], p10=P[1][0]+dt*P[1][1], p11=P[1][1]+Q_v*dt;
    P[0][0]=p00;P[0][1]=p01;P[1][0]=p10;P[1][1]=p11;
  }
  void update(float z){
    float y=z-x,S=P[0][0]+R,K0=P[0][0]/S,K1=P[1][0]/S;
    x+=K0*y; v+=K1*y;
    float t=1.0f-K0;
    float p00=t*P[0][0],p01=t*P[0][1],p10=P[1][0]-K1*P[0][0],p11=P[1][1]-K1*P[0][1];
    P[0][0]=p00;P[0][1]=p01;P[1][0]=p10;P[1][1]=p11;
  }
};
KalmanFilter1D kf_rpm, kf_speed, kf_coolant, kf_boost, kf_timing;

// =============================================================================
//  Shared state (BLE task <-> render loop)
// =============================================================================
SemaphoreHandle_t xMutex;
volatile bool bleConnected = false;
volatile int  deviceCount  = 0;
volatile int  currentGauge = 0;   // active gauge; written by touch (loop), read by bleTask
volatile int   raw_rpm=0, raw_speed=0, raw_map=101;
volatile float raw_coolant=0, raw_boost=0, raw_timing=0;
volatile bool  nd_rpm=false, nd_speed=false, nd_coolant=false, nd_boost=false, nd_timing=false;

BLEClient*               pClient=nullptr;
BLERemoteCharacteristic* pTxChar=nullptr;
BLERemoteCharacteristic* pRxChar=nullptr;
BLEAdvertisedDevice*     targetDevice=nullptr;
String obdResponse="";
volatile bool gotResponse=false;   // set when a full ELM327 response ('>') arrives
volatile uint32_t lastRespMs=0;    // millis of last OBD response (disconnect watchdog)
volatile uint32_t respCount=0;     // total OBD responses (debug)

// ---------- NVS-backed diagnostic log ----------------------------------------
// Captures the first ~60s of connection diagnostics to a RAM buffer, then persists to
// NVS. On the next boot the stored log is dumped to serial -> read it at your desk
// without a laptop in the car. Capture is gated by the user-toggled `logEnabled` flag.
Preferences   prefs;
static char   logBuf[3000];
static size_t logLen = 0;
bool          logEnabled = false;   // persisted in NVS (settings menu toggles it)
bool          logCapturing = false;
bool          logFlushed = false;
uint32_t      logStartMs = 0;
#define LOG_WINDOW_MS 60000

static void logAppend(const char* s){
  if (!logCapturing) return;
  while (*s && logLen < sizeof(logBuf)-1) logBuf[logLen++] = *s++;
}
// print to serial AND (when capturing) append to the NVS buffer
void dlog(const char* fmt, ...){
  char b[200]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  Serial.print(b); logAppend(b);
}

// =============================================================================
//  OBD parsing (all 5 PIDs)
// =============================================================================
static int hexToDec(String h){ return (int)strtol(h.c_str(),NULL,16); }

void parseOBDResponse(String r){
  if (r.length()<4) return;
  // Find the "41XX" mode-01 response anywhere in the string (not just at the start) so a
  // real ELM327's "SEARCHING..." prefix, header bytes, or stray text don't break parsing.
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    int i;
    if      ((i=r.indexOf("410C"))>=0 && r.length()>=i+8){ raw_rpm=(hexToDec(r.substring(i+4,i+6))*256+hexToDec(r.substring(i+6,i+8)))/4; nd_rpm=true; }
    else if ((i=r.indexOf("410D"))>=0 && r.length()>=i+6){ raw_speed=hexToDec(r.substring(i+4,i+6)); nd_speed=true; }
    else if ((i=r.indexOf("4105"))>=0 && r.length()>=i+6){ raw_coolant=hexToDec(r.substring(i+4,i+6))-40; nd_coolant=true; }
    else if ((i=r.indexOf("410B"))>=0 && r.length()>=i+6){ raw_map=hexToDec(r.substring(i+4,i+6)); raw_boost=(raw_map-101)/100.0f; nd_boost=true; }
    else if ((i=r.indexOf("410E"))>=0 && r.length()>=i+6){ raw_timing=hexToDec(r.substring(i+4,i+6))/2.0f-64.0f; nd_timing=true; }
    xSemaphoreGive(xMutex);
  }
}

// =============================================================================
//  BLE client (reused from obd_drive.ino)
// =============================================================================
static void notifyCallback(BLERemoteCharacteristic*, uint8_t* d, size_t len, bool){
  { String s; for (size_t i=0;i<len;i++){ char c=(char)d[i]; s += (c>=32&&c<127)?c:'.'; } dlog("[rx %u] %s\n", (unsigned)len, s.c_str()); }
  for (size_t i=0;i<len;i++){ char c=(char)d[i];
    if (c=='>'){ parseOBDResponse(obdResponse); obdResponse=""; gotResponse=true; lastRespMs=millis(); respCount++; }
    else if (c!='\r' && c!=' ') obdResponse+=c; }
}
class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) override {
    deviceCount = deviceCount + 1;
    if (device.getName()==OBD_BLE_NAME){
      dlog("[scan] MATCH %s name='%s'\n", device.getAddress().toString().c_str(), device.getName().c_str());
      targetDevice=new BLEAdvertisedDevice(device); device.getScan()->stop();
    }
#if OBD_DEBUG
    else Serial.printf("[scan] %s name='%s'\n", device.getAddress().toString().c_str(), device.getName().c_str());
#endif
  }
};
class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient*) override {}
  void onDisconnect(BLEClient*) override { bleConnected=false; }   // peer gone / out of range
};
void sendOBD(const char* cmd){ if (pTxChar && bleConnected){ String s=String(cmd)+"\r"; pTxChar->writeValue((uint8_t*)s.c_str(), s.length()); } }
bool connectToOBD(){
  targetDevice=nullptr; deviceCount=0;
  BLEScan* scan=BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new ScanCallbacks());
  scan->setActiveScan(true); scan->setInterval(100); scan->setWindow(99);
  scan->start(10, false);  // blocks until target found (onResult stops it) or 10s timeout.
  scan->stop();            // (no leftover delay -- the old delay(11000) wasted ~11s/connect)
  if (!targetDevice) return false;
  if (!pClient){                                  // create once, reuse on reconnect (no leak)
    pClient=BLEDevice::createClient();
    static ClientCallbacks ccb;
    pClient->setClientCallbacks(&ccb);
  }
  if (!pClient->connect(targetDevice)){ dlog("[ble] connect() failed\n"); return false; }
  dlog("[ble] connected, discovering GATT...\n");
  if (auto* svcs = pClient->getServices()){            // dump everything the adapter exposes
    for (auto& s : *svcs){ BLERemoteService* sv=s.second;
      dlog("[gatt] svc %s\n", sv->getUUID().toString().c_str());
      if (auto* chs = sv->getCharacteristics()) for (auto& c : *chs){ BLERemoteCharacteristic* ch=c.second;
        dlog("[gatt]   chr %s R%d W%d Wnr%d N%d I%d\n", ch->getUUID().toString().c_str(),
             ch->canRead(), ch->canWrite(), ch->canWriteNoResponse(), ch->canNotify(), ch->canIndicate()); }
    }
  }
  BLERemoteService* svc=pClient->getService(SERVICE_UUID);
  if (!svc){ dlog("[ble] service 18F0 NOT FOUND\n"); return false; }
  pTxChar=svc->getCharacteristic(CHAR_TX_UUID);
  pRxChar=svc->getCharacteristic(CHAR_RX_UUID);
  if (!pTxChar || !pRxChar){ dlog("[ble] TX/RX char NOT FOUND\n"); return false; }
  // subscribe: notifications if supported, else indications (real adapters vary)
  if      (pRxChar->canNotify())   { pRxChar->registerForNotify(notifyCallback, true);  dlog("[ble] subscribed (notify)\n"); }
  else if (pRxChar->canIndicate()) { pRxChar->registerForNotify(notifyCallback, false); dlog("[ble] subscribed (indicate)\n"); }
  else                             { dlog("[ble] RX char has NO notify/indicate!\n"); }
  respCount=0; lastRespMs=millis();                                   // fresh per connection
  return true;
}
void initOBD(){ sendOBD("ATZ");delay(1000); sendOBD("ATE0");delay(200); sendOBD("ATL0");delay(200); sendOBD("ATS0");delay(200); sendOBD("ATSP0");delay(200); }

void bleTask(void*){
  BLEDevice::init("Round-OBD");
  delay(1000);
  Serial.println("[ble] scanning for " OBD_BLE_NAME " ...");
  bleConnected=connectToOBD();
  Serial.printf("[ble] devices seen: %d, connected: %s\n", deviceCount, bleConnected?"YES":"NO");
  if (bleConnected){ initOBD(); Serial.println("[ble] OBD init done"); }

  // Active-gauge-priority schedule: 2 of every 3 requests poll the PID for the gauge
  // currently ON SCREEN (so it refreshes ~2x faster); the 3rd cycles all PIDs to keep
  // the others warm. Total request rate is unchanged -> CAN-safe (still <=10 req/s).
  const char* gaugePid[] = {"010C","010D","0105","010B","010E"};   // index == gauge index
  const int NG = sizeof(gaugePid)/sizeof(gaugePid[0]);
  int bgIdx=0, slot=0; unsigned long lastReq=0,lastRetry=0; bool awaiting=false;
  while (true){
    // disconnect watchdog: link reported closed, OR (only AFTER data has started flowing)
    // no response for >5s. Gating on respCount avoids killing a real ELM327 during its
    // initial protocol search, which can take several seconds before the first reply.
    bool linkDown = !pClient || !pClient->isConnected();
    bool stalled  = (respCount>0) && (millis()-lastRespMs > 5000);
    if (bleConnected && (linkDown || stalled)){
      Serial.printf("[ble] connection lost (%s)\n", linkDown?"link":"stalled");
      if (pClient) pClient->disconnect();
      bleConnected=false; awaiting=false;
    }
    if (bleConnected){
      unsigned long now=millis();
      // advance once the prior request is answered (or timed out)
      if (awaiting && (gotResponse || now-lastReq>REQ_TIMEOUT_MS)){ awaiting=false; slot++; }
      // send next only after response AND >=MIN_REQ_INTERVAL_MS since last send (<=10 req/s)
      if (!awaiting && now-lastReq>=MIN_REQ_INTERVAL_MS){
        const char* pid;
        int ag = (currentGauge<NG)?currentGauge:0;                  // settings page -> default RPM
        if (slot%3==2){ pid=gaugePid[bgIdx]; bgIdx=(bgIdx+1)%NG; }   // background sweep
        else          { pid=gaugePid[ag]; }                         // active gauge (priority)
        gotResponse=false; sendOBD(pid); lastReq=now; awaiting=true;
      }
    }
    if (!bleConnected && millis()-lastRetry>2000){
      bleConnected=connectToOBD();
      if (bleConnected) initOBD();
      lastRetry=millis();
    }
    vTaskDelay(5/portTICK_PERIOD_MS);
  }
}

// =============================================================================
//  Touch (CST816D) - minimal inline driver, returns raw gesture
// =============================================================================
static uint8_t tpRead(uint8_t reg){
  uint8_t v=0, n;
  do { Wire.beginTransmission(TP_ADDR); Wire.write(reg); Wire.endTransmission(false);
       n=Wire.requestFrom((int)TP_ADDR,1); } while (n==0);
  while (Wire.available()) v=Wire.read();
  return v;
}
static void tpWrite(uint8_t reg, uint8_t val){ Wire.beginTransmission(TP_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission(); }
void touchBegin(){
  Wire.begin(TP_SDA, TP_SCL);
  pinMode(TP_INT, OUTPUT); digitalWrite(TP_INT,HIGH); delay(1); digitalWrite(TP_INT,LOW); delay(1);
  pinMode(TP_RST, OUTPUT); digitalWrite(TP_RST,LOW); delay(10); digitalWrite(TP_RST,HIGH); delay(300);
  tpWrite(0xFE, 0xFF);   // disable auto low-power
}
// returns gesture id (0 if none)
// One-shot touch event. Slides report while the finger is moving (finger present); taps
// and long-press only resolve AFTER the finger lifts, so we detect those on the release
// edge (the old finger-gated read missed every tap). Returns a gesture once per touch.
uint8_t touchEvent(){
  static uint8_t prevFinger=0, slideLatch=0;
  static bool ready=false;                       // ignore power-up garbage until idle seen
  uint8_t finger = tpRead(0x02);
  uint8_t g      = tpRead(0x01);
  if (!ready){ if (finger==0) ready=true; prevFinger=finger; return G_NONE; }
  uint8_t ev = G_NONE;
  if (finger){
    if (!slideLatch && (g==G_SLIDE_LEFT||g==G_SLIDE_RIGHT||g==G_SLIDE_UP||g==G_SLIDE_DOWN)){ ev=g; slideLatch=1; }
  } else {
    if (prevFinger && !slideLatch) ev = (g==G_LONG) ? G_LONG : G_TAP;  // release w/o slide = tap (or long)
    slideLatch = 0;
  }
  prevFinger = finger;
  return ev;
}

// =============================================================================
//  Gauge definitions
// =============================================================================
static const GaugeDef GAUGES[] = {
  {"RPM",     "x1000", 0,    8000, true,  6500, 0, 1000, false},
  {"SPEED",   "km/h",  0,    220,  false, 0,    0, 1,    false},
  {"COOLANT", "\xB0""C", 40, 120,  true,  105,  0, 1,    false},
  {"BOOST",   "bar",   -1.0, 2.0,  true,  1.5,  1, 1,    true },
  {"TIMING",  "BTDC",  -20,  40,   false, 0,    0, 1,    true },
};
static const int NUM_GAUGES = sizeof(GAUGES)/sizeof(GAUGES[0]);

// disp_val = Kalman output (target);  view_val = render-side eased value actually drawn.
// The easing layer guarantees per-frame motion regardless of the ~500ms PID cadence,
// so the needle never freezes-then-jumps between measurements.
#define VIEW_EASE 0.30f          // 0..1 per frame @30Hz  (snappier tracking)
float disp_val[5] = {0,0,0,0,0};
float view_val[5] = {0,0,0,0,0};
bool     demoMode  = false;        // synthetic animated face when no BLE for >60s
uint32_t noBleSince= 0;            // millis of last connected moment

// 0-100 launch timer (N-mode style). Entered by double-tap on the SPEED gauge.
enum ZBState { ZB_WAIT, ZB_ARMED, ZB_RUNNING, ZB_DONE };
bool     zbActive  = false;
ZBState  zbState   = ZB_WAIT;
uint32_t zbT0 = 0, zbElapsed = 0;
float peak[5]     = {0,0,0,0,0};   // max value seen per gauge (peak-hold)
bool  peakHold    = false;         // tap toggles; long-press resets
bool  g_alert     = false;         // any warn-gauge over its threshold
int   g_alertGauge= 0;
float baseAdv     = 25.0f;         // learned timing advance at low load (no-knock ref)
float knockRetard = 0.0f;          // worst advance pulled back under load (deg)
int   fuelGrade   = 2;             // 0=LOW 1=REG 2=PREM

// ---------- gauge geometry ----------
// Math angle convention: 0deg = east, CCW positive, screen y inverted. A_START=225
// (lower-left); sweep CW (decreasing angle) by 270deg over the top to lower-right(-45),
// leaving a 90deg gap at the bottom. A_STEP oversampled so radial strokes overlap (no comb).
static const int CX=120, CY=120, R_OUT=116, R_IN=88;   // thick band -> "filling gauge"
static const float A_START=225.0f, A_SWEEP=270.0f, A_STEP=0.2f;

static inline void polarM(float aDeg,int r,int&x,int&y){ float rr=radians(aDeg); x=CX+(int)lroundf(r*cosf(rr)); y=CY-(int)lroundf(r*sinf(rr)); }

// Gauge geometry is fixed -> precompute the inner/outer point of every slice boundary
// ONCE at boot, so the per-frame fill does ZERO trig (C3 has no FPU; runtime cosf/sinf
// was the main render cost). 3deg slices keep the gradient smooth.
#define GSLICES 120                                  // 270deg / 2.25deg (smoother edges + gradient)
static int16_t sIX[GSLICES+1], sIY[GSLICES+1], sOX[GSLICES+1], sOY[GSLICES+1];
static void initGaugeGeom(){
  for (int i=0;i<=GSLICES;i++){
    float rr = radians(A_START - (A_SWEEP*i)/GSLICES);
    float c = cosf(rr), s = sinf(rr);
    sIX[i]=CX+(int)lroundf(R_IN *c); sIY[i]=CY-(int)lroundf(R_IN *s);
    sOX[i]=CX+(int)lroundf(R_OUT*c); sOY[i]=CY-(int)lroundf(R_OUT*s);
  }
}

// ===== brand design-system palette (hex RGB888) =====
#define BR_GRAY_700   0x666666
#define BR_GRAY_850   0x333333
#define BR_VIOLET_400 0xE6A1D2
#define BR_VIOLET_500 0xB31983
#define BR_VIOLET_600 0x850E60
#define BR_BLUE_100   0xDFF1FF
#define BR_BLUE_300   0x2496FF
#define BR_BLUE_600   0x0F7EE4
#define BR_RED_500    0xFF334B
#define BR_RED_600    0xE40000
#define BR_YELLOW_500 0xFFE400
#define BR_GREEN_500  0x02BC58
#define BR_SLATE_250  0xC0C1D1
#define BR_SLATE_300  0x8E90A8
#define BR_SLATE_400  0x585A71

static inline uint16_t hex565(uint32_t h){ return canvas.color565((h>>16)&0xFF, (h>>8)&0xFF, h&0xFF); }

// per-gauge 3-stop gradient built from the brand palette (distinct identity each)
struct Pal { uint32_t c0,c1,c2; };
static const Pal PALS[] = {
  /* RPM     */ { BR_GREEN_500, BR_YELLOW_500, BR_RED_600 },   // green  -> yellow -> red (redline)
  /* SPEED   */ { BR_BLUE_600,  BR_BLUE_300,   BR_BLUE_100 },  // deep   -> mid    -> light blue
  /* COOLANT */ { BR_BLUE_300,  BR_GREEN_500,  BR_RED_600 },   // cold   -> ok     -> hot
  /* BOOST   */ { BR_SLATE_300, BR_YELLOW_500, BR_RED_600 },   // slate  -> amber  -> red
  /* TIMING  */ { BR_VIOLET_400,BR_VIOLET_500, BR_VIOLET_600 },// violet ramp
};
static uint16_t grad3(uint32_t c0, uint32_t c1, uint32_t c2, float frac){
  frac = constrain(frac, 0.0f, 1.0f);
  uint32_t a,b; float t;
  if (frac < 0.5f){ a=c0; b=c1; t=frac*2; }
  else            { a=c1; b=c2; t=(frac-0.5f)*2; }
  int r=((a>>16)&0xFF)+(int)((((int)((b>>16)&0xFF))-((int)((a>>16)&0xFF)))*t);
  int g=((a>> 8)&0xFF)+(int)((((int)((b>> 8)&0xFF))-((int)((a>> 8)&0xFF)))*t);
  int bl=( a     &0xFF)+(int)((((int)( b     &0xFF))-((int)( a     &0xFF)))*t);
  return canvas.color565(r,g,bl);
}
static uint16_t palColor(int gi, float frac){ const Pal& p = PALS[gi]; return grad3(p.c0,p.c1,p.c2,frac); }

// value -> position [0,1] along the sweep. center-zero gauges map 0 to 0.5 (12 o'clock),
// compressing negative range into the left half and positive into the right half.
static float valuePos(int gi, float value){
  const GaugeDef& g = GAUGES[gi];
  if (g.centerZero){
    if (value <= 0) return constrain(0.5f*(value - g.minv)/(0 - g.minv), 0.0f, 0.5f);
    else            return constrain(0.5f + 0.5f*value/g.maxv,           0.5f, 1.0f);
  }
  return constrain((value - g.minv)/(g.maxv - g.minv), 0.0f, 1.0f);
}

void drawGauge(int gi, float value, int fps){
  const GaugeDef& g = GAUGES[gi];
  canvas.fillSprite(TFT_BLACK);

  const int capR=(R_OUT-R_IN)/2, midR=(R_IN+R_OUT)/2;
  const uint16_t TRACK=hex565(BR_GRAY_850);
  float p = valuePos(gi, value);                          // value -> [0,1] along sweep
  float tintFrac = g.centerZero ? fabsf(p-0.5f)*2.0f : p; // palette fraction at the value

  // --- fill: angular quads from precomputed points (no per-frame trig; solid by
  //     construction). Normal gauges fill 0..p; center-zero (boost) fills outward
  //     from the 12-o'clock center to p, colored by distance from center.
  for (int i=0;i<GSLICES;i++){
    float sf = (i + 0.5f) / GSLICES;
    bool filled; float cf;
    if (g.centerZero){ filled = (p>=0.5f) ? (sf>=0.5f && sf<=p) : (sf<=0.5f && sf>=p); cf = fabsf(sf-0.5f)*2.0f; }
    else             { filled = (sf <= p); cf = sf; }
    uint16_t col = filled ? palColor(gi,cf) : TRACK;
    canvas.fillTriangle(sIX[i],sIY[i], sOX[i],sOY[i], sOX[i+1],sOY[i+1], col);
    canvas.fillTriangle(sIX[i],sIY[i], sOX[i+1],sOY[i+1], sIX[i+1],sIY[i+1], col);
  }
  // --- anti-aliased rounded caps
  { int x,y;
    polarM(A_START,        midR,x,y); canvas.fillCircle(x,y,capR,TRACK);       // track ends: plain
    polarM(A_START-A_SWEEP,midR,x,y); canvas.fillCircle(x,y,capR,TRACK);
    float startAng = g.centerZero ? (A_START-0.5f*A_SWEEP) : A_START;
    polarM(startAng,          midR,x,y); canvas.fillSmoothCircle(x,y,capR,palColor(gi,0));      // fill caps: AA
    polarM(A_START-p*A_SWEEP, midR,x,y); canvas.fillSmoothCircle(x,y,capR,palColor(gi,tintFrac)); }

  // --- peak-hold marker: bright tick at the peak position (tap toggles)
  if (peakHold){
    float pp = valuePos(gi, peak[gi]); float pang = A_START - pp*A_SWEEP;
    int ix,iy,ox,oy; polarM(pang,R_IN-3,ix,iy); polarM(pang,R_OUT+3,ox,oy);
    uint16_t w=hex565(0xFFFFFF);
    canvas.drawLine(ix,iy,ox,oy,w); canvas.drawLine(ix+1,iy,ox+1,oy,w); canvas.drawLine(ix-1,iy,ox-1,oy,w);
  }

  // gauge name (top of center)
  canvas.setFont(&fonts::Font2); canvas.setTextColor(hex565(BR_SLATE_250)); canvas.setTextDatum(middle_center);
  canvas.drawString(g.name, CX, CY-58);

  // big value readout, tinted by current level (per-gauge palette)
  canvas.setTextColor(palColor(gi,tintFrac)); canvas.setFont(&fonts::Font7); canvas.setTextSize(1);
  if (g.decimals>0) canvas.drawFloat(value, g.decimals, CX, CY-8);
  else              canvas.drawNumber((long)lroundf(value), CX, CY-8);

  // unit
  canvas.setFont(&fonts::Font2); canvas.setTextColor(hex565(BR_SLATE_300)); canvas.setTextDatum(middle_center);
  canvas.drawString(g.unit, CX, CY+34);

  // page dots
  int dotsY=CY+62, sp=14, x0=CX-(SETTINGS_PAGE)*sp/2;   // NUM_GAUGES gauges + settings page
  for (int i=0;i<=SETTINGS_PAGE;i++){
    if (i==gi) canvas.fillCircle(x0+i*sp, dotsY, 3, hex565(BR_BLUE_300));
    else       canvas.drawCircle(x0+i*sp, dotsY, 2, hex565(BR_SLATE_400));
  }

  // status / fps
  canvas.setTextDatum(top_center);
  if (demoMode){ canvas.setTextColor(hex565(BR_BLUE_300)); canvas.drawString("DEMO", CX, 16); }
  else { canvas.setTextColor(bleConnected?hex565(BR_GREEN_500):hex565(BR_RED_500));
         canvas.drawString(bleConnected?"BLE":"NO BLE", CX, 16); }
  if (peakHold){ canvas.setTextDatum(top_left); canvas.setTextColor(hex565(0xFFFFFF)); canvas.drawString("PK", 84, 16); }

  // fuel-grade badge (on EVERY gauge), inferred from ignition timing advance.
  if (bleConnected){
    const char* fg = fuelGrade==2?"FUEL PREM":fuelGrade==1?"FUEL REG":"FUEL LOW";
    uint16_t    fc = fuelGrade==2?hex565(BR_GREEN_500):fuelGrade==1?hex565(BR_YELLOW_500):hex565(BR_RED_500);
    canvas.setFont(&fonts::Font2); canvas.setTextDatum(top_center); canvas.setTextColor(fc);
    canvas.drawString(fg, CX, 34);
  }
#if SHOW_FPS
  if (fps>=0){ canvas.setTextDatum(bottom_center); canvas.setTextColor(hex565(BR_GRAY_700)); canvas.drawString(String(fps)+"fps", CX, 232); }
#endif

  // --- alert overlay: pulsing red ring if any warn-gauge exceeds its threshold.
  //     Shown on EVERY gauge so e.g. overheat is never missed; names the gauge to
  //     swipe to when it isn't the one on screen.
  if (g_alert){
    if ((millis()/350)%2==0){
      uint16_t rd=hex565(BR_RED_600);
      canvas.drawCircle(CX,CY,119,rd); canvas.drawCircle(CX,CY,118,rd); canvas.drawCircle(CX,CY,117,rd);
    }
    if (g_alertGauge != gi){
      canvas.setFont(&fonts::Font2); canvas.setTextColor(hex565(BR_RED_500)); canvas.setTextDatum(bottom_center);
      canvas.drawString(String(GAUGES[g_alertGauge].name)+" !", CX, 214);
    }
  }

  canvas.pushSprite(0,0);
}

// 0-100 launch-timer screen: aggressive red (N-mode) arc showing speed 0..100, big
// stopwatch in the center. State machine is driven in loop(); this only renders.
void drawZB(){
  canvas.fillSprite(TFT_BLACK);
  const uint16_t TRACK = hex565(0x2A0000);                 // dark-red track
  const uint32_t R0=0x4A0000, R1=0xE40000, R2=0xFF334B;    // deep -> bright red gradient
  const int capR=(R_OUT-R_IN)/2, midR=(R_IN+R_OUT)/2;
  float sp = view_val[1];
  float p  = constrain(sp/ZB_TARGET, 0.0f, 1.0f);

  for (int i=0;i<GSLICES;i++){
    float sf=(i+0.5f)/GSLICES;
    uint16_t col = (sf<=p) ? grad3(R0,R1,R2,sf) : TRACK;
    canvas.fillTriangle(sIX[i],sIY[i], sOX[i],sOY[i], sOX[i+1],sOY[i+1], col);
    canvas.fillTriangle(sIX[i],sIY[i], sOX[i+1],sOY[i+1], sIX[i+1],sIY[i+1], col);
  }
  { int x,y;
    polarM(A_START,        midR,x,y); canvas.fillCircle(x,y,capR,TRACK);
    polarM(A_START-A_SWEEP,midR,x,y); canvas.fillCircle(x,y,capR,TRACK);
    polarM(A_START,          midR,x,y); canvas.fillSmoothCircle(x,y,capR,grad3(R0,R1,R2,0));
    if (p>0.002f){ polarM(A_START-p*A_SWEEP,midR,x,y); canvas.fillSmoothCircle(x,y,capR,grad3(R0,R1,R2,p)); } }

  canvas.setTextDatum(top_center); canvas.setFont(&fonts::Font4); canvas.setTextColor(hex565(R2));
  canvas.drawString("0-100", CX, 26);

  float secs = (zbState==ZB_DONE)? zbElapsed/1000.0f : (zbState==ZB_RUNNING)? (millis()-zbT0)/1000.0f : 0.0f;
  if (secs>99.99f) secs=99.99f;
  canvas.setTextDatum(middle_center);
  if (zbState==ZB_WAIT){
    canvas.setFont(&fonts::Font4); canvas.setTextColor(hex565(BR_SLATE_250)); canvas.drawString("SLOW TO 0", CX, CY-2);
  } else {
    canvas.setFont(&fonts::Font7); canvas.setTextSize(1);
    canvas.setTextColor(zbState==ZB_DONE?hex565(0xFFFFFF):hex565(R2));
    canvas.drawFloat(secs, 2, CX, CY-6);
    canvas.setFont(&fonts::Font2); canvas.setTextColor(hex565(BR_SLATE_300));
    canvas.drawString(zbState==ZB_ARMED?"READY":zbState==ZB_RUNNING?"GO!":"sec", CX, CY+34);
  }
  canvas.setFont(&fonts::Font2); canvas.setTextDatum(bottom_center); canvas.setTextColor(hex565(0xFF8593));
  canvas.drawString(String((int)lroundf(sp))+" km/h", CX, 224);
  canvas.pushSprite(0,0);
}

// toggle logging + persist to NVS; (re)arm or stop the capture buffer
void setLogging(bool on){
  logEnabled = on;
  prefs.putBool("logEn", on);
  if (on){ logLen=0; logFlushed=false; logCapturing=true; logStartMs=millis(); }
  else   { logCapturing=false; }
  Serial.printf("[log] logging %s (saved)\n", on?"ENABLED":"disabled");
}

// Settings page (6th swipe): logging on/off + status.
void drawSettings(){
  canvas.fillSprite(TFT_BLACK);
  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font2); canvas.setTextColor(hex565(BR_SLATE_250));
  canvas.drawString("SETTINGS", CX, 24);

  // LOG toggle (big, tappable)
  canvas.setTextDatum(middle_center); canvas.setFont(&fonts::Font4);
  canvas.setTextColor(logEnabled?hex565(BR_GREEN_500):hex565(BR_SLATE_400));
  canvas.drawString(logEnabled?"LOG: ON":"LOG: OFF", CX, CY-18);

  canvas.setFont(&fonts::Font2); canvas.setTextColor(hex565(BR_SLATE_300));
  canvas.drawString("tap to toggle", CX, CY+18);

  // status line: capturing / saved bytes
  char st[40];
  if (logCapturing)      snprintf(st, sizeof(st), "capturing %u B", (unsigned)logLen);
  else if (logEnabled)   snprintf(st, sizeof(st), "saved (reboot=dump)");
  else                   snprintf(st, sizeof(st), "off");
  canvas.setTextColor(hex565(BR_GRAY_700)); canvas.drawString(st, CX, CY+44);

  // page dots
  int dotsY=CY+62, sp=14, x0=CX-(SETTINGS_PAGE)*sp/2;
  for (int i=0;i<=SETTINGS_PAGE;i++){
    if (i==SETTINGS_PAGE) canvas.fillCircle(x0+i*sp, dotsY, 3, hex565(BR_BLUE_300));
    else                  canvas.drawCircle(x0+i*sp, dotsY, 2, hex565(BR_SLATE_400));
  }
  canvas.pushSprite(0,0);
}

void drawBootScreen(const char* msg){
  canvas.fillSprite(TFT_BLACK); canvas.setTextColor(TFT_WHITE); canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font4); canvas.drawString(msg, CX, CY); canvas.pushSprite(0,0);
}

// Boot ceremony: gauge self-test sweep (fill rushes up to full then back, like a cluster
// needle sweep) with the title in the center. ~1.3s, blocking (runs before BLE task).
// Boot ceremony: OpenClaw logo scales up (grow-in) with anti-aliased zoom, holds, fades.
void bootCeremony(){
  LGFX_Sprite logo(&canvas);          // parent = canvas, so it composites onto our buffer
  logo.setColorDepth(16);             // 16-bit so the dark outline isn't crushed to black
  bool ok = logo.createSprite(LOGO_W, LOGO_H);
  Serial.printf("[boot] logo sprite %dx%d: %s\n", LOGO_W, LOGO_H, ok?"OK":"FAILED");
  if (!ok){ drawBootScreen("OBD GAUGE"); delay(800); return; }

  logo.pushImage(0, 0, LOGO_W, LOGO_H, LOGO_IMG);
  logo.setPivot(LOGO_W/2.0f, LOGO_H/2.0f);

  const uint32_t grow = 850, hold = 450;
  uint32_t t0 = millis();
  for (;;){
    uint32_t el = millis() - t0; if (el >= grow + hold) break;
    float z;
    if (el < grow){ float e = (float)el/grow; z = 0.05f + 0.95f*(1.0f - (1.0f-e)*(1.0f-e)); } // ease-out
    else            z = 1.0f;
    canvas.fillSprite(TFT_BLACK);
    logo.pushRotateZoomWithAA(&canvas, CX, CY, 0.0f, z, z);   // grow-in, anti-aliased
    canvas.pushSprite(0, 0);
    delay(16);
  }
  logo.deleteSprite();            // free 39KB before BLE starts
}

// =============================================================================
//  setup / loop
// =============================================================================
void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("\n[boot] round_obd_gauge  ESP32-C3 (multi-gauge)");
  Serial.printf("[boot] free heap: %u B\n", ESP.getFreeHeap());

  // NVS: load logging flag, dump the previous session's saved log, arm a fresh capture
  prefs.begin("obdgauge", false);
  logEnabled = prefs.getBool("logEn", false);
  { String prev = prefs.getString("log", "");
    if (prev.length()){ Serial.println("\n===== STORED LOG (previous session) ====="); Serial.println(prev); Serial.println("===== END STORED LOG =====\n"); }
    else Serial.println("[log] no stored log"); }
  Serial.printf("[log] logging %s\n", logEnabled?"ENABLED":"disabled");
  if (logEnabled){ logCapturing=true; logStartMs=millis(); logLen=0; }

  pinMode(PIN_BL, OUTPUT); digitalWrite(PIN_BL, HIGH);

  initLUT();
  initGaugeGeom();
  tft.init(); tft.setRotation(0);
  canvas.setColorDepth(8);    // 16-bit won't fit: 115KB contiguous alloc fails on C3 (fragmented SRAM)
  bool ok = canvas.createSprite(240,240);
  Serial.printf("[boot] canvas alloc: %s, free heap: %u B\n", ok?"OK":"FAILED", ESP.getFreeHeap());

  touchBegin();
  Serial.println("[boot] touch (CST816D) init");

  // Kalman tuning (from obd_drive.ino)
  kf_rpm.init    (0.0f,  500.0f, 30.0f,  2000.0f);
  kf_speed.init  (0.0f,  15.0f,  0.8f,   80.0f);
  kf_coolant.init(20.0f, 0.01f,  0.002f, 2.0f);
  kf_boost.init  (0.0f,  0.1f,   0.02f,  5.0f);
  kf_timing.init (0.0f,  1.0f,   0.3f,   10.0f);

  bootCeremony();

  xMutex = xSemaphoreCreateMutex();
  xTaskCreate(bleTask, "BLE", 8192, NULL, 1, NULL);
}

void loop(){
  static uint32_t last=0, lastFps=0;
  static int frames=0, fps=-1;
  static uint32_t lastTapMs=0, pendingTapMs=0;
  static bool pendingSingle=false;

  uint32_t nowT = millis();
  const uint32_t DTAP_MS = 400;   // two taps within this = double-tap

  // --- touch events (one-shot). swipe = switch gauge; double-tap on SPEED = 0-100 mode;
  //     single-tap = peak-hold; long-press = reset peaks. Double-tap is synthesized from
  //     two taps so it doesn't depend on the chip's (unreliable) double-click register.
  const int NPAGES = SETTINGS_PAGE + 1;   // 5 gauges + settings
  uint8_t ev = touchEvent();
  if (ev==G_SLIDE_LEFT)  { currentGauge=(currentGauge+1)%NPAGES; zbActive=false; pendingSingle=false; }
  else if (ev==G_SLIDE_RIGHT){ currentGauge=(currentGauge-1+NPAGES)%NPAGES; zbActive=false; pendingSingle=false; }
  else if (ev==G_LONG)   { for (int i=0;i<NUM_GAUGES;i++) peak[i]=view_val[i]; pendingSingle=false; Serial.println("[touch] peak reset"); }
  else if (ev==G_TAP){
    if (nowT - lastTapMs < DTAP_MS){          // ---- DOUBLE TAP ----
      pendingSingle=false; lastTapMs=0;
      if (currentGauge==GAUGE_SPEED && bleConnected){
        zbActive=!zbActive; zbState=(disp_val[1]<ZB_ARM?ZB_ARMED:ZB_WAIT);
        Serial.printf("[touch] 0-100 mode=%d\n", zbActive);
      }
    } else {                                   // first tap: defer (might become a double)
      pendingSingle=true; pendingTapMs=nowT; lastTapMs=nowT;
    }
  }
  // resolve a lone single-tap once the double-tap window passes
  if (pendingSingle && nowT - pendingTapMs >= DTAP_MS){
    pendingSingle=false;
    if (currentGauge==SETTINGS_PAGE) setLogging(!logEnabled);       // settings: toggle logging
    else if (zbActive) zbState=(disp_val[1]<ZB_ARM?ZB_ARMED:ZB_WAIT); // re-arm
    else { peakHold=!peakHold; Serial.printf("[touch] peakHold=%d\n", peakHold); }
  }

  uint32_t now=millis();
  const uint32_t period=1000/RENDER_HZ;
  if (now-last >= period){
    float dt=(now-last)/1000.0f; last=now;

    if (bleConnected){
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2))){
        if (nd_rpm)    { kf_rpm.update((float)raw_rpm);        nd_rpm=false; }
        if (nd_speed)  { kf_speed.update((float)raw_speed);    nd_speed=false; }
        if (nd_coolant){ kf_coolant.update(raw_coolant);       nd_coolant=false; }
        if (nd_boost)  { kf_boost.update(raw_boost);           nd_boost=false; }
        if (nd_timing) { kf_timing.update(raw_timing);         nd_timing=false; }
        xSemaphoreGive(xMutex);
      }
      kf_rpm.predict(dt); kf_speed.predict(dt); kf_coolant.predict(dt); kf_boost.predict(dt); kf_timing.predict(dt);
      disp_val[0]=max(0.0f, kf_rpm.x);
      disp_val[1]=max(0.0f, kf_speed.x);
      disp_val[2]=kf_coolant.x;
      disp_val[3]=kf_boost.x;
      disp_val[4]=kf_timing.x;
    } else {
      // NO BLE: drop to default 0. Fully re-init the filters (x=0 AND P=large uncertainty)
      // so the FIRST measurement after reconnect snaps back fast -- just zeroing x while
      // leaving a collapsed P makes the filter distrust new data and crawl up from 0.
      kf_rpm.init    (0.0f, 500.0f, 30.0f,  2000.0f);
      kf_speed.init  (0.0f, 15.0f,  0.8f,   80.0f);
      kf_coolant.init(0.0f, 0.01f,  0.002f, 2.0f);
      kf_boost.init  (0.0f, 0.1f,   0.02f,  5.0f);
      kf_timing.init (0.0f, 1.0f,   0.3f,   10.0f);
      for (int i=0;i<NUM_GAUGES;i++){ disp_val[i]=0; peak[i]=0; }
    }

    // render-side easing: every frame nudge the drawn value toward the target,
    // decoupling smooth motion from the ~500ms measurement cadence (all gauges
    // eased so any is ready instantly on swipe).
    for (int i=0;i<NUM_GAUGES;i++) view_val[i] += (disp_val[i]-view_val[i])*VIEW_EASE;

    // demo face: if BLE hasn't been up for >60s, animate synthetic values (phase-shifted
    // sine sweeps) so every gauge looks alive. Swipe still works to browse the styles.
    if (bleConnected){ noBleSince = now; demoMode = false; }
    else if (now - noBleSince > DEMO_AFTER_MS) demoMode = true;
    if (demoMode){
      float t = now/1000.0f;
      for (int i=0;i<NUM_GAUGES;i++){
        float f = 0.5f + 0.5f*sinf(t*0.6f + i*1.2f);
        view_val[i] = GAUGES[i].minv + f*(GAUGES[i].maxv - GAUGES[i].minv);
      }
    }

    // peak-hold tracking + global warn/alert scan (coolant takes priority; off in demo)
    for (int i=0;i<NUM_GAUGES;i++) if (view_val[i] > peak[i]) peak[i] = view_val[i];
    g_alert=false;
    if (!demoMode) for (int i=0;i<NUM_GAUGES;i++) if (GAUGES[i].hasWarn && view_val[i] >= GAUGES[i].warnv){ g_alert=true; g_alertGauge=i; if (i==2) break; }

    // fuel grade via knock-retard under load: learn no-load advance, measure retard
    // when boost/MAP says we're under load (high-octane holds advance, low yanks it back)
    if (bleConnected){
      float ta=view_val[4], ld=view_val[3];
      if (ld >= LOAD_TH_BAR){
        float r = baseAdv - ta; if (r<0) r=0;
        float k = (r > knockRetard) ? 0.5f : 0.05f;     // rise fast (catch knock), fall slower
        knockRetard += (r - knockRetard)*k;             // re-evaluate every loaded frame
      } else {
        if (ta > 5.0f) baseAdv += (ta - baseAdv)*0.02f; // learn advance at low load
        // hold knockRetard between load events (octane ~ constant)
      }
      baseAdv = constrain(baseAdv, 10.0f, 40.0f);
      fuelGrade = (knockRetard>=RETARD_LOW)?0 : (knockRetard>=RETARD_REG)?1 : 2;
    }

#if ZB_TEST
    { static bool zbForced=false; if (bleConnected && !zbForced){ currentGauge=GAUGE_SPEED; zbActive=true; zbState=ZB_WAIT; zbForced=true; } }
#endif
    // 0-100 launch timer state machine (uses Kalman speed disp_val[1] for low latency)
    if (zbActive && (currentGauge!=GAUGE_SPEED || !bleConnected)) zbActive=false;
    if (zbActive){
      float sp = disp_val[1];
      switch (zbState){
        case ZB_WAIT:    if (sp < ZB_ARM) zbState=ZB_ARMED; break;
        case ZB_ARMED:   if (sp > ZB_LAUNCH){ zbState=ZB_RUNNING; zbT0=now; } break;
        case ZB_RUNNING: if (sp < 1.0f) zbState=ZB_ARMED;
                         else if (sp >= ZB_TARGET){ zbElapsed=now-zbT0; zbState=ZB_DONE;
                           Serial.printf("[zb] 0-100 = %.2fs\n", zbElapsed/1000.0f); } break;
        case ZB_DONE:    if (sp < ZB_ARM) zbState=ZB_ARMED; break;   // re-arm at standstill
      }
    }

    // persist the first-minute diagnostic log to NVS (read it on next boot, no laptop needed)
    if (logCapturing && !logFlushed && millis()-logStartMs >= LOG_WINDOW_MS){
      logBuf[logLen]=0; prefs.putString("log", logBuf);
      logCapturing=false; logFlushed=true;
      Serial.printf("[log] saved %u bytes to NVS\n", (unsigned)logLen);
    }

    frames++;
    if (now-lastFps>=1000){ fps=frames; frames=0; lastFps=now;
      Serial.printf("[run] gi=%d ble=%s%s rpm=%.0f SPD=%.1f cool=%.0f boost=%.2f tim=%.0f fps=%d\n",
                    currentGauge, bleConnected?"1":"0", demoMode?" DEMO":"",
                    view_val[0], view_val[1], view_val[2], view_val[3], view_val[4], fps); }

    if (currentGauge==SETTINGS_PAGE) drawSettings();
    else if (zbActive)               drawZB();
    else                             drawGauge(currentGauge, view_val[currentGauge], fps);
  }
  vTaskDelay(1);
}
