// =====================================================================
//  Sensor comparison: GY-91 (a) vs flight controller (b)
//
//  For every shared quantity (pressure, temperature, roll, pitch, heading,
//  3 gyro axes, 3 accel axes) this estimates:
//    bias      mean of (b - a) while the payload is still  (calibration)
//    sig_a/b   noise of each sensor on its own while still
//    resid     current (b - a) - bias, low-passed
//    sig_live  standard deviation of the residual over the last ~10 s
//    status    OK / WARN (> 3 sigma) / FAULT (> 6 sigma for 1 s)
//  and fuses a and b by inverse-variance weighting.
//
//  Altitude has a third, independent source (GPS), so the "three-cornered
//  hat" splits the error into each sensor's own noise:
//    var(A) = ( var(A-B) + var(A-C) - var(B-C) ) / 2
//
//  Pure maths, no Arduino calls - unit-testable on a PC.
// =====================================================================
#pragma once
#include <math.h>
#include <stdint.h>
#include "protocol.h"

namespace health {

struct Spec {
  const char *name;
  float warnFloor, faultFloor;  // minimum thresholds (sensor units)
  bool angle;                   // wrap differences to +-180
  float valScale, errScale;     // packing into int16 (see protocol.h)
  float sigFloor;               // smallest noise assumed when fusing
};

// units inside: Pa, C, deg, deg/s, g
static const Spec SPEC[N_HEALTH_CH] = {
  {"pressure",    40.0f, 120.0f, false, 0.1f,   10.0f,    1.0f},
  {"temperature",  2.0f,   5.0f, false, 100.0f, 100.0f,   0.05f},
  {"roll",         3.0f,   8.0f, true,  100.0f, 100.0f,   0.1f},
  {"pitch",        3.0f,   8.0f, true,  100.0f, 100.0f,   0.1f},
  {"heading",     15.0f,  40.0f, true,  100.0f, 100.0f,   0.5f},
  {"gyro X",       5.0f,  15.0f, false, 10.0f,  100.0f,   0.05f},
  {"gyro Y",       5.0f,  15.0f, false, 10.0f,  100.0f,   0.05f},
  {"gyro Z",       5.0f,  15.0f, false, 10.0f,  100.0f,   0.05f},
  {"accel X",      0.05f,  0.15f, false, 1000.0f, 10000.0f, 0.002f},
  {"accel Y",      0.05f,  0.15f, false, 1000.0f, 10000.0f, 0.002f},
  {"accel Z",      0.05f,  0.15f, false, 1000.0f, 10000.0f, 0.002f},
};

inline float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d <= -180.0f) d += 360.0f;
  return d;
}

// exponentially-weighted mean / variance
struct EW {
  float mean = 0, var = 0;
  uint32_t n = 0;
  void add(float x, float alpha) {
    if (n == 0) { mean = x; var = 0; }
    else {
      float d = x - mean, inc = alpha * d;
      mean += inc;
      var = (1.0f - alpha) * (var + d * inc);
    }
    n++;
  }
  float sd() const { return sqrtf(var > 0 ? var : 0); }
  void reset() { mean = var = 0; n = 0; }
};

struct Chan {
  // latest values
  float a = NAN, b = NAN;
  uint32_t tUpd = 0;
  // calibration accumulators (Welford-free: sums relative to first sample)
  uint32_t cN = 0;
  double cSa = 0, cSa2 = 0, cSb = 0, cSb2 = 0, cSr = 0, cSr2 = 0;
  float a0 = 0, b0 = 0;
  // results
  bool cal = false;
  float bias = 0, sigA = 0, sigB = 0, sigR0 = 0;
  // live
  EW live;
  float rLP = 0;
  uint32_t overFaultSince = 0;
  uint8_t status = HS_NODATA;
};

class Monitor {
 public:
  Chan ch[N_HEALTH_CH];
  bool calibrating = false;
  uint32_t calStart = 0, calEnd = 0;
  bool calibrated = false;

  // three-cornered hat on altitude: A = BMP280, B = DPS310, C = GPS
  EW dAB, dAC, dBC;
  uint32_t tchLast = 0;

  void startCal(uint32_t now, uint32_t durationMs = 20000) {
    calibrating = true;
    calStart = now;
    calEnd = now + durationMs;
    for (auto &c : ch) {
      c.cN = 0; c.cSa = c.cSa2 = c.cSb = c.cSb2 = c.cSr = c.cSr2 = 0;
    }
  }

  uint8_t calProgress(uint32_t now) const {
    if (!calibrating) return calibrated ? 100 : 0;
    float f = (float)(now - calStart) / (float)(calEnd - calStart);
    return (uint8_t)(f < 0 ? 0 : f > 1 ? 100 : f * 100);
  }

  // one comparison sample: a = GY-91, b = flight controller (same instant)
  void update(int k, float a, float b, uint32_t now) {
    if (isnan(a) || isnan(b)) return;
    Chan &c = ch[k];
    const Spec &s = SPEC[k];
    c.a = a; c.b = b; c.tUpd = now;
    float rr = b - a;
    if (s.angle) rr = wrap180(rr);

    if (calibrating) {
      if (c.cN == 0) { c.a0 = a; c.b0 = b; }
      float da = a - c.a0, db = b - c.b0;
      if (s.angle) { da = wrap180(da); db = wrap180(db); }
      c.cN++;
      c.cSa += da; c.cSa2 += (double)da * da;
      c.cSb += db; c.cSb2 += (double)db * db;
      c.cSr += rr; c.cSr2 += (double)rr * rr;
    }

    float r = rr - (c.cal ? c.bias : 0.0f);
    if (s.angle) r = wrap180(r);
    c.live.add(r, 0.01f);                // ~100 samples = ~10 s at 10 Hz
    c.rLP += 0.2f * (r - c.rLP);         // ~0.5 s low-pass for the fault test
    if (s.angle) c.rLP = wrap180(c.rLP);

    if (!c.cal) { c.status = HS_UNCAL; return; }
    float warnT = fmaxf(3.0f * c.sigR0, s.warnFloor);
    float faultT = fmaxf(6.0f * c.sigR0, s.faultFloor);
    float m = fabsf(c.rLP);
    if (m > faultT) {
      if (!c.overFaultSince) c.overFaultSince = now ? now : 1;
      c.status = (now - c.overFaultSince > 1000) ? HS_FAULT : HS_WARN;
    } else {
      c.overFaultSince = 0;
      c.status = m > warnT ? HS_WARN : HS_OK;
    }
  }

  void finishCal() {
    calibrating = false;
    bool any = false;
    for (int k = 0; k < N_HEALTH_CH; k++) {
      Chan &c = ch[k];
      if (c.cN < 10) continue;   // INAV sends temperature only 1x/s
      double n = c.cN;
      double ma = c.cSa / n, mb = c.cSb / n, mr = c.cSr / n;
      c.bias = (float)mr;
      c.sigA = (float)sqrt(fmax(0.0, c.cSa2 / n - ma * ma));
      c.sigB = (float)sqrt(fmax(0.0, c.cSb2 / n - mb * mb));
      c.sigR0 = (float)sqrt(fmax(0.0, c.cSr2 / n - mr * mr));
      c.cal = true;
      c.live.reset();
      c.rLP = 0;
      c.overFaultSince = 0;
      any = true;
    }
    calibrated = calibrated || any;
  }

  // call often; marks stale channels and ends the calibration
  void tick(uint32_t now) {
    if (calibrating && (int32_t)(now - calEnd) >= 0) finishCal();
    // signed: a channel may have been updated a few ms AFTER 'now' was read
    for (auto &c : ch) if (c.tUpd == 0 || (int32_t)(now - c.tUpd) > 3000) c.status = HS_NODATA;
  }

  // best estimate of a channel: inverse-variance weighting of both sensors
  float fused(int k, float fallbackA) const {
    const Chan &c = ch[k];
    const Spec &s = SPEC[k];
    if (c.status == HS_NODATA || isnan(c.b)) return fallbackA;     // only the GY-91
    if (!c.cal) return c.b;                                         // trust the flight controller
    float aCorr = fallbackA + c.bias;                               // GY-91 in the FC's reference
    if (c.status == HS_FAULT) return c.b;                           // can't isolate with 2 -> FC
    float va = fmaxf(c.sigA, s.sigFloor), vb = fmaxf(c.sigB, s.sigFloor);
    float wa = 1.0f / (va * va), wb = 1.0f / (vb * vb);
    if (s.angle) {
      float d = wrap180(c.b - aCorr);
      return wrap180(aCorr + d * wb / (wa + wb));
    }
    return (aCorr * wa + c.b * wb) / (wa + wb);
  }

  // altitudes in metres from the three independent sources
  void updateAltitude3(float hBmp, float hDps, float hGps, uint32_t now) {
    if (isnan(hBmp) || isnan(hDps) || isnan(hGps)) return;
    dAB.add(hBmp - hDps, 0.002f);   // ~500 samples (~50 s at 10 Hz): the method needs long windows
    dAC.add(hBmp - hGps, 0.002f);
    dBC.add(hDps - hGps, 0.002f);
    tchLast = now;
  }
  bool tchValid(uint32_t now) const { return dAB.n > 300 && now - tchLast < 5000; }
  // sigma of each source (m); negative variance (happens with correlated errors) -> 0
  void tchSigmas(float out[3]) const {
    float vab = dAB.var, vac = dAC.var, vbc = dBC.var;
    float va = 0.5f * (vab + vac - vbc), vb = 0.5f * (vab + vbc - vac), vc = 0.5f * (vac + vbc - vab);
    out[0] = sqrtf(va > 0 ? va : 0);
    out[1] = sqrtf(vb > 0 ? vb : 0);
    out[2] = sqrtf(vc > 0 ? vc : 0);
  }

  void pack(HealthChan *dst) const {
    for (int k = 0; k < N_HEALTH_CH; k++) {
      const Chan &c = ch[k];
      const Spec &s = SPEC[k];
      HealthChan &h = dst[k];
      auto q = [](float v, float sc) -> int16_t {
        if (isnan(v)) return 0;
        float x = v * sc;
        if (x > 32767) x = 32767;
        if (x < -32768) x = -32768;
        return (int16_t)lrintf(x);
      };
      h.a = q(c.a, s.valScale);
      h.b = q(c.b, s.valScale);
      h.bias = q(c.bias, s.errScale);
      h.resid = q(c.rLP, s.errScale);
      h.sig_live = q(c.live.sd(), s.errScale);
      h.sig_a = q(c.sigA, s.errScale);
      h.sig_b = q(c.sigB, s.errScale);
      h.status = c.status;
    }
  }
};

}  // namespace health
