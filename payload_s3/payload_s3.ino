/*
  =====================================================================
   HAB-RECON  -  PAYLOAD ("the balloon")         board: Seeed XIAO ESP32-S3
  =====================================================================
   Two independent sensor chains, compared all the time:
     A) GY-91 (MPU-9250 IMU + BMP280 barometer) read directly by the XIAO
     B) Matek F405-WING V2 running INAV 7.1.2 (ArduPilot also works)
        (ICM42688 IMU, DPS310 baro) - read over MAVLink telemetry
   For every shared quantity the XIAO estimates bias, each sensor's noise,
   the live error between them and a fault status, fuses both into a best
   estimate, and (outdoors, with GPS) splits altitude error between
   BMP280 / DPS310 / GPS with the three-cornered hat.

   Sends 60-byte telemetry 5x/s and a 189-byte sensor-health packet 1x/s
   over ESP-NOW. Indoors the position is a simulated 20 km flight.

   WIRING (XIAO 5V -> breadboard + rail, XIAO GND -> - rail)
     GY-91   VIN -> + rail (5 V)       GND -> - rail
             SDA -> D4                 SCL -> D5
     Camera  UART TX -> D7             UART RX <- D6       (3.3 V logic only!)
             5V  <- + rail             GND -> - rail
     F405    TX1 -> D0                 RX1 <- D1           GND -> - rail   (MAVLink)
             F405 power: USB-C from the power bank (no LiPo needed for the demo;
             its 5 V pads and servo rail stay off without a battery - that's fine)
     GPS (optional, later): to the F405 TX3/RX3, powered from the + rail
     ANTENNA: clip the antenna onto the XIAO S3's U.FL connector.

   INAV 7.1.2 (INAV Configurator 7.1.x)
     Ports tab: UART1 -> Telemetry = MAVLink, 115200. Nothing else on UART1
       (not MSP, not Serial RX) - a shared port only sends when armed.
     CLI:  feature TELEMETRY
           set mavlink_extra2_rate = 5     (altitude 5x/s, optional)
           set mavlink_extra3_rate = 2     (temperature 2x/s, optional)
           save
     INAV sends attitude + gyro rates, altitude and temperature over
     MAVLink, but no raw accelerometer and no pressure. So:
       roll, pitch, gyro X/Y/Z, temperature  -> compared directly
       pressure  -> INAV's baro altitude turned back into pressure
       accel     -> not available from INAV (shown as "no data")
     Heading is only compared when the F405 reports a compass.

   ARDUPILOT (alternative)
     SERIAL1_PROTOCOL = 2 (MAVLink2)   SERIAL1_BAUD = 115
     Stream rates are requested by this code automatically.

   ARDUINO IDE
     Board: "XIAO_ESP32S3" (esp32 by Espressif, 3.x), USB CDC On Boot: Enabled
     Library: "Adafruit BMP280 Library"

   ON BOOT keep the payload STILL: 2 s gyro calibration, then a 20 s
   sensor-comparison calibration starts automatically once the F405 talks.
  =====================================================================
*/
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <Adafruit_BMP280.h>
#include "protocol.h"
#include "nav_math.h"
#include "mpu9250.h"
#include "mavlink_lite.h"
#include "sensor_health.h"

// ============================ CONFIG ==================================
#define GPS_MODE            0        // 0 = always simulated flight (indoor demo)
                                     // 1 = real position when the F405 GPS has a 3D fix
#define SIM_SPEED           20.0f    // 20x faster than real: whole flight takes ~5 min
#define LAUNCH_LAT          42.6567  // default launch site (TU Sofia) - movable from the dashboard
#define LAUNCH_LON          23.3553
#define LAUNCH_ALT_M        590.0f   // ground altitude at the launch site (m MSL)
#define BURST_ALT_M         20000.0f
#define ASCENT_RATE_MS      5.0f
#define MAG_DECLINATION_DEG 5.5f     // Sofia ~ +5.5 deg E
#define CAM_HFOV_DEG        60.0f    // camera field of view - set to your lens
#define CAM_VFOV_DEG        40.0f
#define CAM_BAUD            115200
#define FC_BAUD             115200   // must match SERIAL1_BAUD on the F405
#define FC_LATENCY_MS       20       // MAVLink delay used to line up the two sensors
#define USE_CUTDOWN_SERVO   0        // 1 = Cut down also moves a servo on F405 output CUTDOWN_SERVO
#define CUTDOWN_SERVO       7        //     (then set SERVO7_FUNCTION = 0 in ArduPilot)
#define CUTDOWN_PWM_HOLD    1100
#define CUTDOWN_PWM_RELEASE 1900
#define SENSOR_CAL_MS       20000    // sensor comparison calibration length
#define FAKE_DETECTIONS     1        // invent detections if the camera sends nothing
#define TELEMETRY_HZ        5
#define GY91_FLIP           2        // 0 = GY-91 chip facing up, 1 = chip facing down, 2 = detect at boot
#define I2C_HZ              100000   // GY-91 bus speed; 100 kHz is the most robust on a breadboard
// ======================================================================

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const char *LABELS[] = {"object", "person", "vehicle", "drone", "target"};
static const int N_LABELS = 5;
static const int BOOT_BTN = 0;  // XIAO S3 BOOT button = GPIO0

HardwareSerial CamSerial(1);
HardwareSerial FcSerial(2);
Adafruit_BMP280 bmp(&Wire);
Preferences prefs;
QueueHandle_t cmdQ;
health::Monitor mon;
mav::Parser mavParser;
mav::Msg mavMsg;
uint8_t mavSeq = 0;
uint32_t fcBytes = 0, fcByteRate = 0;   // raw bytes from the F405, for diagnosing the link

// ------------------------------ state ---------------------------------
struct Flight {
  uint8_t phase = PHASE_PAD;
  double  launchLat = LAUNCH_LAT, launchLon = LAUNCH_LON;  // where the next flight starts
  double  originLat = LAUNCH_LAT, originLon = LAUNCH_LON;  // where this flight started
  float   groundAlt = LAUNCH_ALT_M;
  float   alt = LAUNCH_ALT_M;   // m MSL
  double  n = 0, e = 0;         // drift from origin, m
  float   vz = 0;               // m/s
  double  met = 0;              // mission elapsed time, s
  bool    real = false;         // position from the real GPS
  double  lat = LAUNCH_LAT, lon = LAUNCH_LON;
} fl;

// what the flight controller tells us (MAVLink)
struct FcState {
  uint8_t sysid = 1, compid = 1;
  uint32_t lastHb = 0, lastData = 0, msgCount = 0;
  uint8_t msgRate = 0;
  float roll = NAN, pitch = NAN, yaw = NAN;              // deg
  float gyro[3] = {NAN, NAN, NAN};                       // deg/s, body FRD
  float acc[3] = {NAN, NAN, NAN};                        // g, body FRD
  float pressPa = NAN, tempC = NAN, p0 = NAN;
  uint8_t gpsFix = 0, sats = 0;
  bool hasMag = false;                                   // SYS_STATUS says a compass is present
  double gpsLat = 0, gpsLon = 0;
  float gpsAlt = NAN;                                    // m MSL
  double posLat = 0, posLon = 0;
  float posAlt = NAN, posVz = 0;                         // from GLOBAL_POSITION_INT
  uint16_t battMv = 0;
  int16_t battCa = 0;
  // INAV: no pressure over MAVLink -> its baro altitude is turned back into a pressure
  bool inav = false;                                     // HEARTBEAT autopilot != ArduPilot
  uint32_t lastRealPress = 0;                            // last SCALED_PRESSURE with a real pressure
  float relAlt = NAN;                                    // VFR_HUD altitude (m, relative to F405 boot)
  float altRefPa = NAN;                                  // pressure that INAV altitude 0 is mapped to
  // INAV gyro axes are forward-left-up; ours are forward-right-down -> Y and Z flip.
  // Checked live while the payload moves and corrected if the boards are mounted differently.
  float gSign[3] = {1, -1, -1};
  uint8_t gSame[3] = {0, 0, 0}, gDiff[3] = {0, 0, 0};
  bool link() const { return lastHb && millis() - lastHb < 3000; }
} fc;

struct Detection {
  bool valid = false;
  uint8_t label = 0, conf = 0;
  uint32_t t = 0;
  int32_t lat_e7 = 0, lon_e7 = 0;
} det;

bool baroOk = false;
uint8_t baroAddr = 0x76, baroId = 0x58;  // where the BMP280 answered at boot
uint32_t lastGyOk = 0, lastBaroOk = 0;   // last good GY-91 IMU / BMP280 reading
float pressPa = 0, tempC = 0, baroAlt = 0, p0 = 101325, p0Fused = 101325;

// GY-91 attitude (rad) from the complementary filter, and fused outputs (deg)
float roll = 0, pitch = 0, yaw = 0, accelG = 1;
float gyLastGyro[3] = {0, 0, 0}, gyLastAcc[3] = {0, 0, -1};
float outRoll = 0, outPitch = 0, outHdg = 0, outPress = 0;
bool hdgValid = false;   // heading comes only from the GY-91 (the F405 has no compass)
bool attInit = false, yawInit = false;
uint32_t freefallUntil = 0, shockUntil = 0, ffStart = 0;

uint32_t lastCamLine = 0, lastFakeDet = 0;
char camBuf[96];
uint8_t camLen = 0;

uint16_t seq = 0, healthSeq = 0, lastCmdSeq = 0;
bool haveCmdSeq = false;

bool calRunning = false;  // compass calibration
uint32_t calEnd = 0;
float calMin[3], calMax[3];

// ---- GY-91 history, so FC samples are compared with the GY-91 at the same moment ----
struct GySample { uint32_t t; float roll, pitch, yaw, g[3], a[3]; };
static GySample gyHist[64];
static uint8_t gyHead = 0;
static bool gyFilled = false;

void gyPush(uint32_t t) {
  GySample &s = gyHist[gyHead];
  s.t = t;
  s.roll = roll * nav::RAD2DEG;
  s.pitch = pitch * nav::RAD2DEG;
  s.yaw = yaw * nav::RAD2DEG;
  for (int i = 0; i < 3; i++) { s.g[i] = gyLastGyro[i]; s.a[i] = gyLastAcc[i]; }
  gyHead = (gyHead + 1) & 63;
  if (gyHead == 0) gyFilled = true;
}

// sample closest to time t (ms)
const GySample *gyAt(uint32_t t) {
  int n = gyFilled ? 64 : gyHead;
  if (n == 0) return nullptr;
  const GySample *best = nullptr;
  uint32_t bestD = 0xFFFFFFFF;
  for (int i = 0; i < n; i++) {
    uint32_t d = (uint32_t)abs((int32_t)(gyHist[i].t - t));
    if (d < bestD) { bestD = d; best = &gyHist[i]; }
  }
  return bestD < 100 ? best : nullptr;
}

// ============================ helpers =================================
bool gyFresh() { return imu::ok && millis() - lastGyOk < 300; }
bool baroFresh() { return baroOk && millis() - lastBaroOk < 500; }
bool fcFresh() { return fc.link() && millis() - fc.lastData < 1000; }

static inline int32_t clamp32(double v, double lo, double hi) {
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return (int32_t)lround(v);
}

// free a stuck I2C bus (a chip holding SDA low after a glitch) and set the GY-91 up again
void i2cRecover() {
  Wire.end();
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9; i++) {      // 9 clock pulses let a stuck chip finish its byte
    digitalWrite(SCL, LOW); delayMicroseconds(10);
    digitalWrite(SCL, HIGH); delayMicroseconds(10);
  }
  Wire.begin();
  Wire.setClock(I2C_HZ);
  bool imuBack = imu::begin();       // keeps the gyro calibration
  bool baroBack = baroOk && bmp.begin(baroAddr, baroId);
  if (baroBack)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_1);
  Serial.printf("I2C restarted: IMU %s, barometer %s\n", imuBack ? "back" : "still silent",
                baroBack ? "back" : "still silent");
}

void currentPosition(double &lat, double &lon) {
  if (fl.real) { lat = fl.lat; lon = fl.lon; return; }
  nav::offsetLatLon(fl.originLat, fl.originLon, fl.n, fl.e, lat, lon);
}

void zeroBaro() {
  if (baroOk) {
    float s = 0;
    int n = 0;
    for (int i = 0; i < 40 && n < 10; i++) {   // only plausible readings count
      float p = bmp.readPressure();
      if (!isnan(p) && p > 30000 && p < 110000) { s += p; n++; }
      delay(50);
    }
    if (n) p0 = s / n;
  }
  if (!isnan(fc.pressPa)) fc.p0 = fc.pressPa;
  p0Fused = mon.fused(HC_PRESS, p0);
}

void flightReset() {
  fl.phase = PHASE_PAD;
  fl.originLat = fl.launchLat;
  fl.originLon = fl.launchLon;
  fl.alt = fl.groundAlt;
  fl.n = fl.e = 0;
  fl.vz = 0;
  fl.met = 0;
  det.valid = false;
}

void loadMagCal() {
  prefs.begin("habrecon", true);
  float buf[6];
  if (prefs.getBytes("magcal", buf, sizeof buf) == sizeof buf) {
    for (int i = 0; i < 3; i++) { imu::magOff[i] = buf[i]; imu::magScale[i] = buf[i + 3]; }
    Serial.println("Compass calibration loaded");
  }
  prefs.end();
}

void startMagCal() {
  if (!imu::magOk) return;
  calRunning = true;
  calEnd = millis() + 15000;
  for (int i = 0; i < 3; i++) { calMin[i] = 1e9; calMax[i] = -1e9; }
  Serial.println("Compass calibration: rotate the payload in a figure-8 for 15 s");
}

void finishMagCal() {
  calRunning = false;
  float range[3], avg = 0;
  for (int i = 0; i < 3; i++) { range[i] = calMax[i] - calMin[i]; avg += range[i] / 3.0f; }
  if (range[0] < 30 || range[1] < 30 || range[2] < 30) {
    Serial.println("Compass calibration FAILED - rotate more in all directions");
    return;
  }
  float buf[6];
  for (int i = 0; i < 3; i++) {
    imu::magOff[i] = buf[i] = (calMax[i] + calMin[i]) / 2.0f;
    imu::magScale[i] = buf[i + 3] = avg / range[i];
  }
  prefs.begin("habrecon", false);
  prefs.putBytes("magcal", buf, sizeof buf);
  prefs.end();
  yawInit = false;
  Serial.printf("Compass calibrated: offsets %.1f %.1f %.1f uT\n", buf[0], buf[1], buf[2]);
}

// ======================== MAVLink (flight controller) =================
void mavSend(uint32_t id, const uint8_t *payload, uint8_t len) {
  uint8_t buf[280];
  size_t n = mav::pack(buf, id, payload, len, 200, 191, mavSeq);  // sysid 200, onboard computer
  FcSerial.write(buf, n);
}

void fcRequestStreams() {
  uint8_t p[8];
  // MAV_DATA_STREAM: 1 RAW_SENSORS (RAW_IMU, SCALED_PRESSURE), 2 EXTENDED_STATUS (SYS_STATUS, GPS_RAW_INT),
  //                  6 POSITION (GLOBAL_POSITION_INT), 10 EXTRA1 (ATTITUDE)
  const uint8_t streams[4] = {1, 2, 6, 10};
  const uint16_t rates[4] = {10, 2, 5, 10};
  for (int i = 0; i < 4; i++) {
    uint8_t n = mav::requestStream(p, fc.sysid, fc.compid, streams[i], rates[i]);
    mavSend(mav::REQUEST_DATA_STREAM, p, n);
  }
}

void fcSetServo(uint16_t pwm) {
  if (!USE_CUTDOWN_SERVO) return;
  uint8_t p[40];
  uint8_t n = mav::commandLong(p, fc.sysid, fc.compid, 183 /* MAV_CMD_DO_SET_SERVO */, CUTDOWN_SERVO, pwm);
  mavSend(mav::COMMAND_LONG, p, n);
  Serial.printf("F405: servo %d -> %u us\n", CUTDOWN_SERVO, pwm);
}

bool fcRealPressure() { return fc.lastRealPress && millis() - fc.lastRealPress < 3000; }

// INAV altitude 0 m is mapped to the GY-91's current pressure, so both
// "pressures" start equal and the difference shows the altitude error
void fcAnchorAlt() {
  if (isnan(fc.relAlt)) return;
  float pRef = (baroFresh() && pressPa > 30000 && pressPa < 110000) ? pressPa : 101325.0f;
  fc.altRefPa = pRef / powf(fmaxf(1.0f - fc.relAlt / 44330.0f, 0.01f), 5.255f);
}

// INAV gyro: while both sensors see a clear rotation on an axis, check they
// agree on its direction; flip the F405 axis if they clearly don't
void checkGyroSign(int i, float a, float b) {
  if (fabsf(a) < 30.0f || fabsf(b) < 30.0f) return;
  if ((a > 0) == (b > 0)) fc.gSame[i]++; else fc.gDiff[i]++;
  if (fc.gSame[i] + fc.gDiff[i] < 20) return;
  if (fc.gDiff[i] >= 16) {
    fc.gSign[i] = -fc.gSign[i];
    health::Chan &c = mon.ch[HC_GX + i];
    if (c.cal && c.cN) {   // redo the bias for the flipped axis from the calibration sums
      float mA = c.a0 + (float)(c.cSa / c.cN), mB = c.b0 + (float)(c.cSb / c.cN);
      c.bias = -mB - mA;
    }
    c.live.reset(); c.rLP = 0; c.overFaultSince = 0;
    Serial.printf("F405 gyro %c axis turns the other way - flipped it\n", "XYZ"[i]);
  }
  fc.gSame[i] = fc.gDiff[i] = 0;
}

void handleMav(const mav::Msg &m) {
  uint32_t now = millis();
  uint32_t tCmp = now - FC_LATENCY_MS;  // moment the FC sample was taken
  fc.msgCount++;
  switch (m.id) {
    case mav::HEARTBEAT:
      if (m.payload[5] != 8) {  // ignore non-autopilot heartbeats (MAV_AUTOPILOT_INVALID)
        bool inav = m.payload[5] != 3;   // 3 = ArduPilot, INAV says 0 (generic)
        if (!fc.link()) Serial.printf("F405 connected (sysid %u, %s)\n", m.sysid, inav ? "INAV" : "ArduPilot");
        fc.inav = inav;
        fc.sysid = m.sysid; fc.compid = m.compid; fc.lastHb = now;
      }
      break;
    case mav::SYS_STATUS:
      fc.hasMag = mav::u32(m, 0) & 0x04;   // MAV_SYS_STATUS_SENSOR_3D_MAG
      fc.battMv = mav::u16(m, 14);
      fc.battCa = mav::i16(m, 16);
      break;
    case mav::GPS_RAW_INT:
      fc.gpsLat = mav::i32(m, 8) / 1e7;
      fc.gpsLon = mav::i32(m, 12) / 1e7;
      fc.gpsAlt = mav::i32(m, 16) / 1000.0f;
      fc.gpsFix = m.payload[28];
      fc.sats = m.payload[29];
      break;
    case mav::GLOBAL_POSITION_INT:
      fc.posLat = mav::i32(m, 4) / 1e7;
      fc.posLon = mav::i32(m, 8) / 1e7;
      fc.posAlt = mav::i32(m, 12) / 1000.0f;
      fc.posVz = -mav::i16(m, 24) / 100.0f;  // MAVLink vz is positive down
      break;
    case mav::ATTITUDE: {
      fc.lastData = now;
      fc.roll = mav::f32(m, 4) * nav::RAD2DEG;
      fc.pitch = mav::f32(m, 8) * nav::RAD2DEG;
      fc.yaw = mav::f32(m, 12) * nav::RAD2DEG;
      if (fc.inav)   // INAV puts its gyro here in deg/s (not rad/s), forward-left-up axes
        for (int i = 0; i < 3; i++) fc.gyro[i] = mav::f32(m, 16 + 4 * i) * fc.gSign[i];
      const GySample *g = gyAt(tCmp);
      if (g && gyFresh()) {
        mon.update(HC_ROLL, g->roll, fc.roll, now);
        mon.update(HC_PITCH, g->pitch, fc.pitch, now);
        if (imu::magOk && fc.hasMag) mon.update(HC_HEADING, g->yaw, fc.yaw, now);
        if (fc.inav) {
          for (int i = 0; i < 3; i++) {
            checkGyroSign(i, g->g[i], fc.gyro[i]);
            fc.gyro[i] = mav::f32(m, 16 + 4 * i) * fc.gSign[i];   // sign may just have flipped
            mon.update(HC_GX + i, g->g[i], fc.gyro[i], now);
          }
        }
      }
      break;
    }
    case mav::VFR_HUD: {   // INAV: baro altitude in m, relative to where the F405 powered up
      float h = mav::f32(m, 8);
      if (!fc.inav || fcRealPressure() || isnan(h) || fabsf(h) > 40000.0f) break;
      fc.relAlt = h;
      fc.lastData = now;
      if (isnan(fc.altRefPa)) fcAnchorAlt();
      fc.pressPa = fc.altRefPa * powf(fmaxf(1.0f - h / 44330.0f, 0.01f), 5.255f);  // altitude -> pressure
      if (isnan(fc.p0)) fc.p0 = fc.pressPa;
      if (baroFresh()) {
        mon.update(HC_PRESS, pressPa, fc.pressPa, now);
        if (fc.gpsFix >= 3 && !isnan(fc.gpsAlt)) {
          float hA = fl.groundAlt + nav::baroAltitude(pressPa, p0);
          float hB = fl.groundAlt + nav::baroAltitude(fc.pressPa, fc.p0);
          mon.updateAltitude3(hA, hB, fc.gpsAlt, now);
        }
      }
      break;
    }
    case mav::RAW_IMU: {  // ArduPilot sends calibrated values: mG, mrad/s
      fc.lastData = now;
      for (int i = 0; i < 3; i++) {
        fc.acc[i] = mav::i16(m, 8 + 2 * i) / 1000.0f;
        fc.gyro[i] = mav::i16(m, 14 + 2 * i) * 0.001f * nav::RAD2DEG;
      }
      const GySample *g = gyAt(tCmp);
      if (g && gyFresh()) {
        for (int i = 0; i < 3; i++) {
          mon.update(HC_GX + i, g->g[i], fc.gyro[i], now);
          mon.update(HC_AX + i, g->a[i], fc.acc[i], now);
        }
      }
      break;
    }
    case mav::SCALED_PRESSURE: {
      fc.lastData = now;
      float pa = mav::f32(m, 4) * 100.0f, t = mav::i16(m, 12) / 100.0f;
      if (t > -40.0f && t < 85.0f && t != 0.0f) {
        fc.tempC = t;
        if (baroFresh()) mon.update(HC_TEMP, tempC, fc.tempC, now);
      }
      if (!(pa > 30000.0f && pa < 110000.0f)) break;   // INAV sends 0 here -> pressure comes from VFR_HUD
      fc.lastRealPress = now;
      fc.pressPa = pa;
      if (isnan(fc.p0)) fc.p0 = fc.pressPa;
      if (baroFresh()) {
        mon.update(HC_PRESS, pressPa, fc.pressPa, now);
        // three independent altitudes: BMP280, DPS310, GPS (only with a 3D fix)
        if (fc.gpsFix >= 3 && !isnan(fc.gpsAlt)) {
          float hA = fl.groundAlt + nav::baroAltitude(pressPa, p0);
          float hB = fl.groundAlt + nav::baroAltitude(fc.pressPa, fc.p0);
          mon.updateAltitude3(hA, hB, fc.gpsAlt, now);
        }
      }
      break;
    }
    case mav::COMMAND_ACK:
      Serial.printf("F405: command %u result %u\n", mav::u16(m, 0), m.payload[2]);
      break;
  }
}

void pollFc() {
  while (FcSerial.available()) {
    mavParser.push((uint8_t)FcSerial.read());
    fcBytes++;
    while (mavParser.next(mavMsg)) handleMav(mavMsg);
  }
}

// ======================== attitude filter (GY-91) =====================
void updateAttitude(const float a[3], const float g[3], float dt, bool haveMag, const float m[3]) {
  float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  accelG = an;
  float ra, pa;
  nav::rollPitchFromAccel(a[0], a[1], a[2], ra, pa);
  if (!attInit) { roll = ra; pitch = pa; attInit = true; }

  float sr = sinf(roll), cr = cosf(roll), cp = cosf(pitch);
  if (fabsf(cp) < 0.05f) cp = copysignf(0.05f, cp);
  float tp = sinf(pitch) / cp;
  roll  += (g[0] + (g[1] * sr + g[2] * cr) * tp) * dt;
  pitch += (g[1] * cr - g[2] * sr) * dt;
  yaw   += ((g[1] * sr + g[2] * cr) / cp) * dt;

  if (fabsf(an - 1.0f) < 0.2f) {
    roll  += 0.02f * nav::wrapPi(ra - roll);
    pitch += 0.02f * (pa - pitch);
  }
  roll = nav::wrapPi(roll);

  if (haveMag) {
    float hm = nav::tiltCompassHeading(m[0], m[1], m[2], roll, pitch) + MAG_DECLINATION_DEG * nav::DEG2RAD;
    float k = yawInit ? 0.02f : 1.0f;
    yaw += k * nav::wrapPi(hm - yaw);
    yawInit = true;
  }
  yaw = nav::wrapPi(yaw);

  for (int i = 0; i < 3; i++) { gyLastGyro[i] = g[i] * nav::RAD2DEG; gyLastAcc[i] = a[i]; }

  uint32_t now = millis();
  if (an < 0.35f) {
    if (!ffStart) ffStart = now;
    else if (now - ffStart > 80) freefallUntil = now + 2000;
  } else ffStart = 0;
  if (an > 2.5f) shockUntil = now + 2000;
}

// best estimates from both chains
void updateFusedOutputs() {
  float gyRoll = roll * nav::RAD2DEG, gyPitch = pitch * nav::RAD2DEG, gyYaw = yaw * nav::RAD2DEG;
  if (!gyFresh() && fcFresh()) {   // GY-91 missing or unplugged: roll and pitch from the flight controller alone
    outRoll = isnan(fc.roll) ? 0 : fc.roll;
    outPitch = isnan(fc.pitch) ? 0 : fc.pitch;
  } else {
    outRoll = mon.fused(HC_ROLL, gyRoll);
    outPitch = mon.fused(HC_PITCH, gyPitch);
  }
  // Heading only from the GY-91: its compass (AK8963) + declination keeps it on true north. The F405-WING has
  // no compass, so its yaw starts at a random angle - it is not used. No GY-91 -> no heading.
  hdgValid = gyFresh();
  if (hdgValid) outHdg = gyYaw;
  if (baroFresh()) outPress = fcFresh() ? mon.fused(HC_PRESS, pressPa) : pressPa;
  else if (!isnan(fc.pressPa)) outPress = fc.pressPa;
  if (outPress > 0) baroAlt = nav::baroAltitude(outPress, p0Fused > 0 ? p0Fused : outPress);
}

// ======================== flight model ================================
void simSub(float dt) {
  switch (fl.phase) {
    case PHASE_PAD:     fl.alt = fl.groundAlt; fl.n = fl.e = 0; fl.vz = 0; return;
    case PHASE_LANDED:  fl.vz = 0; return;
    case PHASE_ASCENT:  fl.vz = ASCENT_RATE_MS + 0.4f * sinf(fl.met / 45.0f); break;
    case PHASE_DESCENT: fl.vz = -nav::descentRate(fl.alt); break;
  }
  fl.met += dt;
  float vn, ve;
  nav::windAt(fl.alt, vn, ve);
  fl.n += vn * dt;
  fl.e += ve * dt;
  fl.alt += fl.vz * dt;
  if (fl.phase == PHASE_ASCENT && fl.alt >= BURST_ALT_M) {
    fl.phase = PHASE_DESCENT;
    Serial.println("SIM: balloon burst");
  }
  if (fl.phase == PHASE_DESCENT && fl.alt <= fl.groundAlt) {
    fl.alt = fl.groundAlt;
    fl.vz = 0;
    fl.phase = PHASE_LANDED;
    Serial.println("SIM: landed");
  }
}

void simStep(float dtReal) {
  float dt = dtReal * SIM_SPEED;
  while (dt > 0) {
    float h = std::min(dt, 0.5f);
    simSub(h);
    dt -= h;
  }
}

bool realGpsUsable() {
#if GPS_MODE == 1
  return fc.link() && fc.gpsFix >= 3 && !isnan(fc.posAlt);
#else
  return false;
#endif
}

void realStep(float dt) {
  static uint32_t calmSince = 0;
  fl.lat = fc.posLat;
  fl.lon = fc.posLon;
  fl.alt = fc.posAlt;
  fl.vz = fc.posVz;
  if (fl.phase != PHASE_PAD) fl.met += dt;
  switch (fl.phase) {
    case PHASE_PAD:
      fl.groundAlt = fl.alt;
      fl.launchLat = fl.originLat = fl.lat;
      fl.launchLon = fl.originLon = fl.lon;
      if (fl.vz > 1.5f) fl.phase = PHASE_ASCENT;
      break;
    case PHASE_ASCENT:
      if (fl.vz < -3.0f) fl.phase = PHASE_DESCENT;
      break;
    case PHASE_DESCENT:
      if (fabsf(fl.vz) < 0.5f) { if (!calmSince) calmSince = millis(); }
      else calmSince = 0;
      if (calmSince && millis() - calmSince > 20000) fl.phase = PHASE_LANDED;
      break;
  }
}

// ========================= detections =================================
void registerDetection(uint8_t label, uint8_t conf, float u, float v) {
  det.valid = true;
  det.label = label;
  det.conf = conf;
  det.t = millis();
  det.lat_e7 = det.lon_e7 = 0;  // 0,0 = could not place it on the map
  float n, e;
  if (nav::projectToGround(u, v, CAM_HFOV_DEG * nav::DEG2RAD, CAM_VFOV_DEG * nav::DEG2RAD,
                           outRoll * nav::DEG2RAD, outPitch * nav::DEG2RAD, outHdg * nav::DEG2RAD,
                           fl.alt - fl.groundAlt, n, e) && hdgValid) {   // no heading -> can't place it
    double lat, lon, dlat, dlon;
    currentPosition(lat, lon);
    nav::offsetLatLon(lat, lon, n, e, dlat, dlon);
    det.lat_e7 = (int32_t)lround(dlat * 1e7);
    det.lon_e7 = (int32_t)lround(dlon * 1e7);
  }
  Serial.printf("DETECTION %s %u%%\n", LABELS[label], conf);
}

// camera sends lines like:  DET,vehicle,87,512,300   (label, conf %, x 0-1000, y 0-1000)
void handleCamLine(char *line) {
  if (strncmp(line, "DET,", 4) != 0) return;
  char *save = nullptr;
  strtok_r(line, ",", &save);
  char *lab = strtok_r(nullptr, ",", &save);
  char *cf = strtok_r(nullptr, ",", &save);
  char *cx = strtok_r(nullptr, ",", &save);
  char *cy = strtok_r(nullptr, ",", &save);
  if (!lab || !cf || !cx || !cy) return;
  uint8_t idx = 0;
  for (int i = 0; i < N_LABELS; i++) if (strcasecmp(lab, LABELS[i]) == 0) idx = i;
  int conf = constrain(atoi(cf), 0, 100);
  float u = constrain(atoi(cx), 0, 1000) / 1000.0f - 0.5f;
  float v = constrain(atoi(cy), 0, 1000) / 1000.0f - 0.5f;
  registerDetection(idx, conf, u, v);
}

void pollCamera() {
  while (CamSerial.available()) {
    char c = CamSerial.read();
    if (c == '\n' || c == '\r') {
      if (camLen) {
        camBuf[camLen] = 0;
        lastCamLine = millis();
        handleCamLine(camBuf);
        camLen = 0;
      }
    } else if (camLen < sizeof(camBuf) - 1) {
      camBuf[camLen++] = c;
    }
  }
#if FAKE_DETECTIONS
  bool airborne = fl.phase == PHASE_ASCENT || fl.phase == PHASE_DESCENT;
  if (airborne && millis() - lastCamLine > 10000 && millis() - lastFakeDet > 7000) {
    lastFakeDet = millis();
    static const uint8_t pick[] = {1, 2, 2, 4};
    uint8_t lab = pick[esp_random() % 4];
    uint8_t conf = 55 + esp_random() % 41;
    float u = (esp_random() % 1000) / 1000.0f - 0.5f;
    float v = (esp_random() % 1000) / 1000.0f - 0.5f;
    registerDetection(lab, conf, u, v);
  }
#endif
}

// ========================= commands ===================================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len == sizeof(CmdPacket) && data[0] == CMD_MAGIC) {
    CmdPacket c;
    memcpy(&c, data, sizeof c);
    xQueueSend(cmdQ, &c, 0);
  }
}

void handleCommand(const CmdPacket &c) {
  if (haveCmdSeq && c.cmd_seq == lastCmdSeq) return;  // duplicate (sent 3x / via relay)
  haveCmdSeq = true;
  lastCmdSeq = c.cmd_seq;
  switch (c.cmd) {
    case CMD_LAUNCH:
      if (fl.phase == PHASE_PAD || fl.phase == PHASE_LANDED) {
        flightReset();
        fl.phase = PHASE_ASCENT;
        Serial.println("CMD: launch");
      }
      break;
    case CMD_CUTDOWN:
      if (fc.link()) fcSetServo(CUTDOWN_PWM_RELEASE);
      if (fl.phase == PHASE_ASCENT) {
        fl.phase = PHASE_DESCENT;
        Serial.println("CMD: cut-down -> descent");
      }
      break;
    case CMD_RESET:
      flightReset();
      zeroBaro();
      if (fc.link()) fcSetServo(CUTDOWN_PWM_HOLD);
      Serial.println("CMD: reset");
      break;
    case CMD_CAL_MAG:
      startMagCal();
      break;
    case CMD_SET_LAUNCH:
      fl.launchLat = c.lat_e7 / 1e7;
      fl.launchLon = c.lon_e7 / 1e7;
      if (fl.phase == PHASE_PAD) flightReset();
      Serial.printf("CMD: launch site %.5f, %.5f\n", fl.launchLat, fl.launchLon);
      break;
    case CMD_CAL_SENSORS:
      if (fc.inav && !fcRealPressure()) fcAnchorAlt();
      mon.startCal(millis(), SENSOR_CAL_MS);
      Serial.println("CMD: sensor comparison calibration - keep the payload still for 20 s");
      break;
  }
}

// ========================= radio ======================================
void radioBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  Serial.printf("ESP-NOW up on channel %d, MAC %s\n", ESPNOW_CHANNEL, WiFi.macAddress().c_str());
}

void sendTelemetry() {
  Telemetry t = {};
  t.magic = TLM_MAGIC;
  t.hops = 0;
  t.version = PROTO_VERSION;
  uint32_t now = millis();
  uint8_t f = fl.real ? FLAG_GPS_REAL : FLAG_GPS_SIM;
  if (imu::magOk && gyFresh()) f |= FLAG_MAG_OK;   // the GY-91 compass works: heading is on true north (compass + declination)
  if (baroOk || !isnan(fc.pressPa)) f |= FLAG_BARO_OK;
  if (imu::ok || fc.link()) f |= FLAG_IMU_OK;
  if (calRunning) f |= FLAG_MAG_CAL;
  if (now < freefallUntil) f |= FLAG_FREEFALL;
  if (now < shockUntil) f |= FLAG_SHOCK;
  t.flags = f;
  t.seq = seq++;
  t.uptime_ms = now;
  t.met_s = (uint32_t)fl.met;
  double lat, lon;
  currentPosition(lat, lon);
  t.lat_e7 = (int32_t)lround(lat * 1e7);
  t.lon_e7 = (int32_t)lround(lon * 1e7);
  t.alt_cm = clamp32(fl.alt * 100.0, -2e9, 2e9);
  t.baro_alt_cm = clamp32(baroAlt * 100.0, -2e9, 2e9);
  t.press_pa = (uint32_t)lround(outPress);
  t.temp_c100 = (int16_t)clamp32((baroFresh() ? tempC : (isnan(fc.tempC) ? 0 : fc.tempC)) * 100.0, -32768, 32767);
  t.roll_c100 = (int16_t)clamp32(outRoll * 100.0, -32768, 32767);
  t.pitch_c100 = (int16_t)clamp32(outPitch * 100.0, -32768, 32767);
  float hdg = outHdg < 0 ? outHdg + 360.0f : outHdg;
  t.heading_c100 = hdgValid ? (uint16_t)clamp32(hdg * 100.0, 0, 35999) : 0xFFFF;   // 0xFFFF = no heading (HEADING_UNKNOWN)
  t.vz_cms = (int16_t)clamp32(fl.vz * 100.0, -32768, 32767);
  t.accel_mg = gyFresh() ? (uint16_t)clamp32(accelG * 1000.0, 1, 65535) : 0;   // 0 = unknown (INAV sends no accel)
  t.phase = fl.phase;
  if (det.valid) {
    t.det_label = det.label;
    t.det_conf = det.conf;
    t.det_age_ds = (uint8_t)std::min<uint32_t>(254, (now - det.t) / 100);
    t.det_lat_e7 = det.lat_e7;
    t.det_lon_e7 = det.lon_e7;
  } else {
    t.det_age_ds = 255;
  }
  t.ack_cmd_seq = lastCmdSeq;
  esp_now_send(BCAST, (const uint8_t *)&t, sizeof t);
}

void sendHealth() {
  HealthPacket h = {};
  uint32_t now = millis();
  h.magic = HEALTH_MAGIC;
  h.version = PROTO_VERSION;
  uint8_t f = 0;
  if (fc.link()) f |= HFLAG_FC_LINK;
  if (gyFresh() && baroFresh()) f |= HFLAG_GY_OK;
  if (mon.calibrated) f |= HFLAG_CALIBRATED;
  if (mon.calibrating) f |= HFLAG_CALIBRATING;
  if (mon.tchValid(now)) f |= HFLAG_TCH_VALID;
  if (fc.gpsFix >= 3) f |= HFLAG_FC_GPS3D;
  if (fc.hasMag) f |= HFLAG_FC_MAG;
  if (fc.inav) f |= HFLAG_FC_INAV;
  h.flags = f;
  h.seq = healthSeq++;
  h.uptime_ms = now;
  h.cal_progress = mon.calProgress(now);
  h.fc_gps_fix = fc.gpsFix;
  h.fc_sats = fc.sats;
  h.fc_msg_rate = fc.msgRate;
  h.fc_batt_mv = fc.battMv;
  h.fc_batt_ca = fc.battCa;
  if (mon.tchValid(now)) {
    float s[3];
    mon.tchSigmas(s);
    for (int i = 0; i < 3; i++) h.tch_sigma_cm[i] = (int16_t)clamp32(s[i] * 100.0, 0, 32767);
  } else {
    h.tch_sigma_cm[0] = h.tch_sigma_cm[1] = h.tch_sigma_cm[2] = -1;
  }
  mon.pack(h.ch);
  esp_now_send(BCAST, (const uint8_t *)&h, sizeof h);
}

// ============================ setup ===================================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== HAB-RECON payload ===");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // LED off (active low)
  pinMode(BOOT_BTN, INPUT_PULLUP);

  cmdQ = xQueueCreate(8, sizeof(CmdPacket));

  Wire.begin();  // D4 = SDA, D5 = SCL
  Wire.setClock(I2C_HZ);
  Serial.print("I2C devices found:");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
  }
  Serial.println(found ? "" : " none - check SDA (D4), SCL (D5), VIN and GND");

  const uint8_t addrs[] = {0x76, 0x77};
  const uint8_t ids[] = {0x58, 0x60, 0x56, 0x57};
  for (uint8_t a : addrs) {
    for (uint8_t id : ids) {
      if (!baroOk && bmp.begin(a, id)) {
        baroOk = true;
        baroAddr = a; baroId = id;
        Serial.printf("Barometer OK at 0x%02X (chip id 0x%02X)\n", a, id);
      }
    }
  }
  if (baroOk) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_1);
    delay(1500);   // let the BMP280's IIR filter settle before zeroing
    zeroBaro();
  } else {
    Serial.println("Barometer NOT found - check wiring (SDA D4, SCL D5)");
  }

  if (imu::begin()) {
    const char *chip = imu::whoami == 0x71 ? "MPU-9250" : imu::whoami == 0x73 ? "MPU-9255"
                     : imu::whoami == 0x70 ? "MPU-6500 (NO COMPASS - fake GY-91?)" : "unknown";
    Serial.printf("IMU at 0x%02X, WHO_AM_I = 0x%02X -> %s, magnetometer %s\n", imu::MPU, imu::whoami, chip,
                  imu::magOk ? "OK" : "NOT FOUND");
#if GY91_FLIP == 2
    {   // which way up is the GY-91? at rest body Z (down) reads -1 g; chip-down it reads +1 g
      float a[3], g[3], zs = 0;
      int n = 0;
      for (int i = 0; i < 20; i++) { if (imu::readAG(a, g)) { zs += a[2]; n++; } delay(5); }
      imu::flip = n && zs / n > 0.6f;
    }
#else
    imu::flip = GY91_FLIP;
#endif
    if (imu::flip) Serial.println("GY-91 is mounted chip-down - turning its axes over in software");
    Serial.println("Calibrating gyro - keep still...");
    imu::calibrateGyro();
    loadMagCal();
  } else {
    Serial.println("IMU NOT found at 0x68/0x69 - check wiring; if the barometer works, wire GY-91 NCS to 3V3");
  }

  CamSerial.begin(CAM_BAUD, SERIAL_8N1, D7, D6);
  FcSerial.begin(FC_BAUD, SERIAL_8N1, D0, D1);   // RX = D0 (from F405 TX1), TX = D1 (to F405 RX1)

  radioBegin();
  flightReset();
  Serial.println("Ready. Waiting for the F405 on D0/D1 (MAVLink telemetry). Press BOOT or Launch to fly.");
}

// ============================= loop ===================================
void loop() {
  static uint32_t lastImuUs = 0, lastBaro = 0, lastFlight = 0, lastTx = 0, lastHealth = 0,
                  lastDbg = 0, ledOff = 0, btnDown = 0, lastHb = 0, lastReq = 0, lastRate = 0,
                  linkSince = 0;
  static bool autoCalDone = false, servoInit = false;
  uint32_t now = millis();

  CmdPacket c;
  while (xQueueReceive(cmdQ, &c, 0) == pdTRUE) handleCommand(c);

  // BOOT button = launch (or reset if the flight is over)
  if (digitalRead(BOOT_BTN) == LOW) {
    if (!btnDown) btnDown = now;
  } else if (btnDown) {
    if (now - btnDown > 50) {
      if (fl.phase == PHASE_PAD) { fl.phase = PHASE_ASCENT; Serial.println("BUTTON: launch"); }
      else if (fl.phase == PHASE_LANDED) { flightReset(); Serial.println("BUTTON: reset"); }
    }
    btnDown = 0;
  }

  // GY-91 IMU @ 100 Hz
  uint32_t us = micros();
  if (imu::ok && us - lastImuUs >= 10000) {
    float dt = lastImuUs ? (us - lastImuUs) / 1e6f : 0.01f;
    lastImuUs = us;
    float a[3], g[3], m[3], raw[3];
    if (imu::readAG(a, g)) {
      lastGyOk = now;
      bool haveMag = imu::readMag(m, raw);
      if (haveMag && calRunning) {
        for (int i = 0; i < 3; i++) { calMin[i] = std::min(calMin[i], raw[i]); calMax[i] = std::max(calMax[i], raw[i]); }
      }
      updateAttitude(a, g, dt, haveMag && !calRunning, m);
      gyPush(now);
    }
  }
  if (calRunning && now > calEnd) finishMagCal();

  // BMP280 @ 20 Hz
  if (baroOk && now - lastBaro >= 50) {
    lastBaro = now;
    float p = bmp.readPressure(), tc = bmp.readTemperature();
    if (!isnan(p) && p > 500 && p < 120000) { pressPa = p; tempC = tc; lastBaroOk = now; }
  }

  // GY-91 watchdog: if it stops answering, say so and restart the I2C bus
  static uint32_t lastI2cTry = 0;
  bool imuDead = imu::ok && now - lastGyOk > 1000, baroDead = baroOk && now - lastBaroOk > 1000;
  if ((imuDead || baroDead) && now > 5000 && now - lastI2cTry > 3000) {
    lastI2cTry = now;
    Serial.printf("GY-91 not answering (%s%s) - check its VIN, GND, SDA -> D4, SCL -> D5. Restarting I2C...\n",
                  imuDead ? "IMU " : "", baroDead ? "barometer" : "");
    i2cRecover();
  }

  // flight controller
  pollFc();
  if (now - lastHb >= 1000) {                       // our heartbeat, 1 Hz
    lastHb = now;
    uint8_t p[9];
    mavSend(mav::HEARTBEAT, p, mav::heartbeat(p));
  }
  if (fc.link() && (int32_t)(now - fc.lastData) > 2000 && now - lastReq > 3000) {  // no data -> ask for it
    lastReq = now;
    fcRequestStreams();
  }
  if (now - lastRate >= 1000) {
    lastRate = now;
    fc.msgRate = (uint8_t)std::min<uint32_t>(255, fc.msgCount);
    fc.msgCount = 0;
    fcByteRate = fcBytes;
    fcBytes = 0;
    // explain a missing F405 link every 10 s
    static uint32_t lastHint = 0;
    if (!fc.link() && now > 15000 && now - lastHint > 10000) {
      lastHint = now;
      if (fcByteRate == 0)
        Serial.println("F405: no bytes on D0 - check F405 TX1 -> XIAO D0, the GND wire, F405 on USB power; "
                       "in INAV: UART1 Telemetry = MAVLink 115200 and CLI 'feature TELEMETRY' + 'save'");
      else
        Serial.printf("F405: %lu bytes/s on D0 but no MAVLink - in INAV Ports tab UART1 must be ONLY "
                      "Telemetry = MAVLink at 115200 (MSP off), then Save and Reboot\n",
                      (unsigned long)fcByteRate);
    }
  }
  if (fc.link()) {
    if (!linkSince) linkSince = now;
    if (!servoInit) { servoInit = true; fcSetServo(CUTDOWN_PWM_HOLD); }
    // first sensor-comparison calibration starts by itself 5 s after the F405 starts talking
    if (!autoCalDone && now - linkSince > 5000 && fc.lastData && !mon.calibrating) {
      autoCalDone = true;
      if (isnan(fc.p0) || p0Fused <= 0) zeroBaro();
      if (fc.inav && !fcRealPressure()) fcAnchorAlt();
      mon.startCal(now, SENSOR_CAL_MS);
      Serial.println("Sensor comparison calibration started - keep still for 20 s");
    }
  } else {
    linkSince = 0;
  }
  // re-zero the barometer once, 10 s after boot, in case the first readings were off
  static bool baroRezeroed = false;
  if (!baroRezeroed && now > 10000 && baroFresh() && fl.phase == PHASE_PAD) {
    baroRezeroed = true;
    p0 = pressPa;
    p0Fused = mon.fused(HC_PRESS, pressPa);
  }
  static bool wasCalibrating = false;
  mon.tick(now);
  if (wasCalibrating && !mon.calibrating) {   // calibration just ended: re-zero all barometers here
    if (baroOk) p0 = pressPa;
    if (!isnan(fc.pressPa)) fc.p0 = fc.pressPa;
    p0Fused = mon.fused(HC_PRESS, pressPa);
    Serial.println("Sensor comparison calibrated");
  }
  wasCalibrating = mon.calibrating;

  pollCamera();

  // flight model + fused outputs @ 20 Hz
  if (now - lastFlight >= 50) {
    float dt = lastFlight ? (now - lastFlight) / 1000.0f : 0.05f;
    lastFlight = now;
    updateFusedOutputs();
    fl.real = realGpsUsable();
    if (fl.real) realStep(dt);
    else simStep(dt);
  }

  if (now - lastTx >= 1000 / TELEMETRY_HZ) {
    lastTx = now;
    sendTelemetry();
    digitalWrite(LED_BUILTIN, LOW);
    ledOff = now + 15;
  }
  if (now - lastHealth >= 1000) {
    lastHealth = now;
    sendHealth();
  }
  if (ledOff && now >= ledOff) { digitalWrite(LED_BUILTIN, HIGH); ledOff = 0; }

  if (now - lastDbg >= 1000 && Serial) {
    lastDbg = now;
    static const char *PH[] = {"PAD", "ASCENT", "DESCENT", "LANDED"};
    static const char *ST[] = {"--", "uncal", "OK", "WARN", "FAULT"};
    Serial.printf("%s alt %.0f m | R %.1f P %.1f H %.1f%s | %.2f hPa baro %.2f m | GY-91 %s | F405 %s %u msg/s %lu B/s | "
                  "dP %.1f Pa (%s) dRoll %.2f (%s) %s\n",
                  PH[fl.phase], fl.alt, outRoll, outPitch, outHdg < 0 ? outHdg + 360 : outHdg, hdgValid ? (imu::magOk ? "" : " (gyro)") : " (none: no GY-91)",
                  outPress / 100.0f, baroAlt, gyFresh() && baroFresh() ? "OK" : gyFresh() ? "NO BARO" : baroFresh() ? "NO IMU" : "LOST",
                  fc.link() ? "OK" : "--", fc.msgRate, (unsigned long)fcByteRate,
                  mon.ch[HC_PRESS].rLP, ST[mon.ch[HC_PRESS].status],
                  mon.ch[HC_ROLL].rLP, ST[mon.ch[HC_ROLL].status],
                  mon.calibrating ? "CALIBRATING" : "");
  }
}
