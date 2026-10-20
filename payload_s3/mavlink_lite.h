// =====================================================================
//  Minimal MAVLink v1/v2 reader + v2 writer - just the messages we use
//  from INAV or ArduPilot. No library needed. Frames are CRC-checked;
//  frames we can't check (unknown IDs) or that fail are re-scanned.
// =====================================================================
#pragma once
#include <stdint.h>
#include <string.h>

namespace mav {

// message IDs (common.xml)
enum : uint32_t {
  HEARTBEAT = 0, SYS_STATUS = 1, GPS_RAW_INT = 24, RAW_IMU = 27, SCALED_PRESSURE = 29,
  ATTITUDE = 30, GLOBAL_POSITION_INT = 33, RC_CHANNELS_RAW = 35, GPS_GLOBAL_ORIGIN = 49,
  REQUEST_DATA_STREAM = 66, VFR_HUD = 74, COMMAND_LONG = 76, COMMAND_ACK = 77,
  BATTERY_STATUS = 147, STATUSTEXT = 253
};

inline bool crcExtra(uint32_t id, uint8_t &ex) {
  switch (id) {
    case HEARTBEAT: ex = 50; return true;
    case SYS_STATUS: ex = 124; return true;
    case GPS_RAW_INT: ex = 24; return true;
    case RAW_IMU: ex = 144; return true;
    case SCALED_PRESSURE: ex = 115; return true;
    case ATTITUDE: ex = 39; return true;
    case GLOBAL_POSITION_INT: ex = 104; return true;
    case RC_CHANNELS_RAW: ex = 244; return true;     // INAV sends these; we only skip them cleanly
    case GPS_GLOBAL_ORIGIN: ex = 39; return true;
    case REQUEST_DATA_STREAM: ex = 148; return true;
    case VFR_HUD: ex = 20; return true;
    case COMMAND_LONG: ex = 152; return true;
    case COMMAND_ACK: ex = 143; return true;
    case BATTERY_STATUS: ex = 154; return true;
    case STATUSTEXT: ex = 83; return true;
    default: return false;
  }
}

// CRC-16/MCRF4XX (X.25) as used by MAVLink
inline void crcAccum(uint8_t b, uint16_t &crc) {
  uint8_t t = b ^ (uint8_t)(crc & 0xFF);
  t ^= (uint8_t)(t << 4);
  crc = (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (t >> 4));
}

struct Msg {
  uint32_t id;
  uint8_t sysid, compid, len;
  uint8_t payload[256];   // zero-filled after len (MAVLink 2 trims trailing zeros)
};

// little-endian field readers
inline float    f32(const Msg &m, int o) { float v; memcpy(&v, m.payload + o, 4); return v; }
inline int32_t  i32(const Msg &m, int o) { int32_t v; memcpy(&v, m.payload + o, 4); return v; }
inline uint32_t u32(const Msg &m, int o) { uint32_t v; memcpy(&v, m.payload + o, 4); return v; }
inline int16_t  i16(const Msg &m, int o) { int16_t v; memcpy(&v, m.payload + o, 2); return v; }
inline uint16_t u16(const Msg &m, int o) { uint16_t v; memcpy(&v, m.payload + o, 2); return v; }

class Parser {
 public:
  uint32_t good = 0, bad = 0;

  // Feed one byte; returns true when m holds a complete, CRC-valid message.
  // After a CRC failure the bytes of the broken frame are scanned again, so a
  // stray 0xFD/0xFE in line noise can't swallow the real frame behind it.
  bool feed(uint8_t c, Msg &m) { push(c); return next(m); }

  // queue one received byte
  void push(uint8_t c) {
    if (qn >= sizeof(q)) { qn = qi = 0; }
    q[qn++] = c;
  }

  // process queued bytes; call repeatedly until it returns false
  bool next(Msg &m) {
    while (qi < qn) {
      int r = step(q[qi++], m);
      if (r < 0) {                          // replay raw[1..rawLen) before the rest of the queue
        uint8_t tmp[sizeof(q)];
        uint16_t n = 0;
        for (uint16_t i = 1; i < rawLen && n < sizeof(tmp); i++) tmp[n++] = raw[i];
        for (uint16_t i = qi; i < qn && n < sizeof(tmp); i++) tmp[n++] = q[i];
        memcpy(q, tmp, n);
        qn = n; qi = 0; rawLen = 0;
        continue;
      }
      if (r > 0) {                          // keep unprocessed bytes for the next call
        uint16_t rest = qn - qi;
        memmove(q, q + qi, rest);
        qn = rest; qi = 0;
        return true;
      }
    }
    qn = qi = 0;
    return false;
  }

 private:
  uint8_t q[600];
  uint16_t qn = 0, qi = 0;
  uint8_t raw[300];
  uint16_t rawLen = 0;
  uint8_t st = 0, len = 0, idx = 0, incompat = 0, idb = 0, sigLeft = 0;
  bool v2 = false, pendingOk = false;
  uint16_t crc = 0, ck = 0;
  uint32_t msgid = 0;

  // 1 = message ready, 0 = need more bytes, -1 = broken frame (rescan it)
  int step(uint8_t c, Msg &m) {
    if (st != 0 && rawLen < sizeof(raw)) raw[rawLen++] = c;
    switch (st) {
      case 0:
        if (c == 0xFD) { v2 = true; st = 1; rawLen = 0; raw[rawLen++] = c; }
        else if (c == 0xFE) { v2 = false; st = 1; rawLen = 0; raw[rawLen++] = c; }
        return 0;
      case 1: len = c; crc = 0xFFFF; crcAccum(c, crc); idx = 0; incompat = 0; st = v2 ? 2 : 4; return 0;
      case 2:
        incompat = c; crcAccum(c, crc);
        if (incompat & 0xFE) { st = 0; return -1; }  // only the signing flag exists
        st = 3; return 0;
      case 3: crcAccum(c, crc); st = 4; return 0;                       // compat flags
      case 4: crcAccum(c, crc); st = 5; return 0;                       // sequence
      case 5: m.sysid = c; crcAccum(c, crc); st = 6; return 0;
      case 6: m.compid = c; crcAccum(c, crc); st = 7; msgid = 0; idb = 0; return 0;
      case 7:
        msgid |= (uint32_t)c << (8 * idb);
        crcAccum(c, crc);
        if (++idb == (v2 ? 3 : 1)) st = len ? 8 : 9;
        return 0;
      case 8:
        m.payload[idx++] = c;
        crcAccum(c, crc);
        if (idx >= len) st = 9;
        return 0;
      case 9: ck = c; st = 10; return 0;
      case 10: {
        ck |= (uint16_t)c << 8;
        bool hasSig = v2 && (incompat & 0x01);
        sigLeft = 13;
        uint8_t ex;
        if (!crcExtra(msgid, ex)) { st = 0; pendingOk = false; return -1; }  // can't verify: rescan its bytes
        crcAccum(ex, crc);
        if (crc != ck) { bad++; st = 0; pendingOk = false; return -1; }
        good++;
        m.id = msgid;
        m.len = len;
        memset(m.payload + len, 0, sizeof(m.payload) - len);
        if (!hasSig) { st = 0; return 1; }
        st = 11;
        pendingOk = true;
        return 0;
      }
      case 11:
        if (--sigLeft == 0) {
          st = 0;
          if (pendingOk) { pendingOk = false; return 1; }
        }
        return 0;
    }
    st = 0;
    return 0;
  }
};

// Build a MAVLink 2 frame into buf (needs len + 12 bytes). Returns frame length.
inline size_t pack(uint8_t *buf, uint32_t msgid, const uint8_t *payload, uint8_t len,
                   uint8_t sysid, uint8_t compid, uint8_t &seq) {
  while (len > 1 && payload[len - 1] == 0) len--;   // MAVLink 2 payload truncation
  buf[0] = 0xFD; buf[1] = len; buf[2] = 0; buf[3] = 0; buf[4] = seq++;
  buf[5] = sysid; buf[6] = compid;
  buf[7] = msgid & 0xFF; buf[8] = (msgid >> 8) & 0xFF; buf[9] = (msgid >> 16) & 0xFF;
  memcpy(buf + 10, payload, len);
  uint16_t crc = 0xFFFF;
  for (int i = 1; i < 10 + len; i++) crcAccum(buf[i], crc);
  uint8_t ex = 0;
  crcExtra(msgid, ex);
  crcAccum(ex, crc);
  buf[10 + len] = crc & 0xFF;
  buf[11 + len] = crc >> 8;
  return 12 + len;
}

// payload builders --------------------------------------------------------
inline uint8_t heartbeat(uint8_t *p) {           // we are an onboard computer
  memset(p, 0, 9);
  p[4] = 18;    // MAV_TYPE_ONBOARD_CONTROLLER
  p[5] = 8;     // MAV_AUTOPILOT_INVALID
  p[7] = 4;     // MAV_STATE_ACTIVE
  p[8] = 3;     // mavlink version
  return 9;
}
inline uint8_t requestStream(uint8_t *p, uint8_t tsys, uint8_t tcomp, uint8_t stream, uint16_t hz) {
  memcpy(p, &hz, 2); p[2] = tsys; p[3] = tcomp; p[4] = stream; p[5] = 1;
  return 6;
}
inline uint8_t commandLong(uint8_t *p, uint8_t tsys, uint8_t tcomp, uint16_t cmd,
                           float p1, float p2, float p3 = 0, float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0) {
  float f[7] = {p1, p2, p3, p4, p5, p6, p7};
  memcpy(p, f, 28);
  memcpy(p + 28, &cmd, 2);
  p[30] = tsys; p[31] = tcomp; p[32] = 0;
  return 33;
}

}  // namespace mav
