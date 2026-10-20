// =====================================================================
//  HAB-RECON radio protocol  (same file in payload_s3, ground_c6, relay_8266)
//  If you change anything here, copy the file to all three folders AND
//  update parseTelemetry() / encodeTelemetry() in dashboard.html.
//
//  All fields are little-endian, structs are packed (no padding).
//  The same packets will later go over LoRa unchanged - only the radio
//  driver changes, not the format.
// =====================================================================
#pragma once
#include <stdint.h>

#define ESPNOW_CHANNEL 1          // all boards must use the same Wi-Fi channel

#define TLM_MAGIC 0xB1            // telemetry: payload -> ground
#define CMD_MAGIC 0xC1            // command:   ground  -> payload
#define PROTO_VERSION 1

// ---- telemetry flags ----
#define FLAG_GPS_REAL   0x01      // position comes from a real GPS fix
#define FLAG_GPS_SIM    0x02      // position comes from the flight simulator
#define FLAG_MAG_OK     0x04      // magnetometer present (real MPU-9250)
#define FLAG_BARO_OK    0x08      // BMP280 working
#define FLAG_IMU_OK     0x10      // accel/gyro working
#define FLAG_MAG_CAL    0x20      // compass calibration running
#define FLAG_FREEFALL   0x40      // free-fall detected in the last 2 s
#define FLAG_SHOCK      0x80      // shock (> 2.5 g) detected in the last 2 s

// ---- flight phases ----
enum : uint8_t { PHASE_PAD = 0, PHASE_ASCENT = 1, PHASE_DESCENT = 2, PHASE_LANDED = 3 };

// ---- detection labels (index sent over the air) ----
//   0 object, 1 person, 2 vehicle, 3 drone, 4 target

// ---- commands ----
enum : uint8_t {
  CMD_LAUNCH = 1,       // start the (simulated) flight
  CMD_CUTDOWN = 2,      // terminate flight -> descent
  CMD_RESET = 3,        // back to the pad, re-zero barometer
  CMD_CAL_MAG = 4,      // 15 s compass calibration (rotate the payload in a figure-8)
  CMD_SET_LAUNCH = 5    // move the launch site to lat/lon in the packet
};

#define CMD_CAL_SENSORS 6 // 20 s sensor comparison calibration (keep the payload still)

// ---- sensor health (GY-91 vs flight controller) ----
#define HEALTH_MAGIC 0xB2         // health: payload -> ground, 1x per second
#define N_HEALTH_CH 11
enum : uint8_t { HC_PRESS = 0, HC_TEMP, HC_ROLL, HC_PITCH, HC_HEADING,
                 HC_GX, HC_GY, HC_GZ, HC_AX, HC_AY, HC_AZ };
enum : uint8_t { HS_NODATA = 0, HS_UNCAL = 1, HS_OK = 2, HS_WARN = 3, HS_FAULT = 4 };
#define HFLAG_FC_LINK     0x01    // MAVLink heartbeat from the flight controller in the last 3 s
#define HFLAG_GY_OK       0x02    // GY-91 IMU + barometer working
#define HFLAG_CALIBRATED  0x04    // sensor comparison has been calibrated
#define HFLAG_CALIBRATING 0x08    // calibration running now
#define HFLAG_TCH_VALID   0x10    // three-cornered-hat altitude sigmas are valid
#define HFLAG_FC_GPS3D    0x20    // flight controller GPS has a 3D fix
#define HFLAG_FC_MAG      0x40    // flight controller has a compass (else heading isn't compared)
#define HEADING_UNKNOWN   0xFFFF  // Telemetry.heading_c100 when there is no heading
#define HFLAG_FC_INAV     0x80    // flight controller runs INAV (no accel, pressure from its altitude)
// Scaling of the int16 fields per channel  (value = a, b ; error = bias, resid, sigmas)
//   pressure: value hPa*10,   error Pa*10        temperature: value/error C*100
//   roll/pitch/heading: deg*100 (heading -180..180)
//   gyro: value deg/s*10, error deg/s*100        accel: value g*1000, error g*10000

#pragma pack(push, 1)
struct HealthChan {         // 15 bytes
  int16_t a;                // GY-91 reading
  int16_t b;                // flight controller reading
  int16_t bias;             // mean (b - a) measured during calibration
  int16_t resid;            // current (b - a) - bias, low-passed
  int16_t sig_live;         // standard deviation of the residual, last ~10 s
  int16_t sig_a;            // GY-91 noise (std at rest, from calibration)
  int16_t sig_b;            // flight controller noise (std at rest)
  uint8_t status;           // HS_*
};

struct HealthPacket {       // 189 bytes
  uint8_t  magic;           // HEALTH_MAGIC
  uint8_t  hops;
  uint8_t  version;
  uint8_t  flags;           // HFLAG_*
  uint16_t seq;
  uint32_t uptime_ms;
  uint8_t  cal_progress;    // 0-100 while calibrating
  uint8_t  fc_gps_fix;      // MAVLink GPS fix type (3 = 3D)
  uint8_t  fc_sats;
  uint8_t  fc_msg_rate;     // MAVLink messages per second from the flight controller
  uint16_t fc_batt_mv;      // flight controller battery voltage
  int16_t  fc_batt_ca;      // battery current, 0.01 A
  int16_t  tch_sigma_cm[3]; // altitude noise of BMP280, DPS310, GPS (three-cornered hat), cm; -1 = n/a
  HealthChan ch[N_HEALTH_CH];
};

struct Telemetry {          // 60 bytes
  uint8_t  magic;           // TLM_MAGIC
  uint8_t  hops;            // 0 = straight from payload, +1 per relay
  uint8_t  version;         // PROTO_VERSION
  uint8_t  flags;           // FLAG_*
  uint16_t seq;             // packet counter (for packet-loss %)
  uint32_t uptime_ms;       // payload uptime
  uint32_t met_s;           // mission elapsed time (simulated time in SIM mode)
  int32_t  lat_e7;          // latitude  * 1e7
  int32_t  lon_e7;          // longitude * 1e7
  int32_t  alt_cm;          // altitude above sea level, cm
  int32_t  baro_alt_cm;     // BMP280 altitude relative to reset point, cm (real sensor)
  uint32_t press_pa;        // BMP280 pressure, Pa
  int16_t  temp_c100;       // BMP280 temperature, 0.01 C
  int16_t  roll_c100;       // 0.01 deg
  int16_t  pitch_c100;      // 0.01 deg
  uint16_t heading_c100;    // 0.01 deg, true north (GY-91 compass + declination); HEADING_UNKNOWN = no GY-91
  int16_t  vz_cms;          // vertical speed, cm/s
  uint16_t accel_mg;        // total acceleration, milli-g
  uint8_t  phase;           // PHASE_*
  uint8_t  det_label;       // last detection label index
  uint8_t  det_conf;        // last detection confidence 0-100
  uint8_t  det_age_ds;      // age of last detection in 0.1 s (255 = none)
  int32_t  det_lat_e7;      // ground position of the detection
  int32_t  det_lon_e7;
  uint16_t ack_cmd_seq;     // seq of the last command the payload executed
};

struct CmdPacket {          // 13 bytes
  uint8_t  magic;           // CMD_MAGIC
  uint8_t  hops;
  uint16_t cmd_seq;         // payload ignores repeats of the same seq
  uint8_t  cmd;             // CMD_*
  int32_t  lat_e7;          // used by CMD_SET_LAUNCH
  int32_t  lon_e7;
};
#pragma pack(pop)

static_assert(sizeof(Telemetry) == 60, "Telemetry must be 60 bytes");
static_assert(sizeof(CmdPacket) == 13, "CmdPacket must be 13 bytes");
static_assert(sizeof(HealthChan) == 15, "HealthChan must be 15 bytes");
static_assert(sizeof(HealthPacket) == 189, "HealthPacket must be 189 bytes");
