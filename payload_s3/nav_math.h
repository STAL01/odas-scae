// =====================================================================
//  Navigation maths - attitude, compass, camera ground projection,
//  flight simulator wind model. Pure functions, no Arduino calls,
//  so it can be unit-tested on a PC.
//
//  Frames: body frame is NED-style -> X forward, Y right, Z down.
//          world frame is North, East, Down.
// =====================================================================
#pragma once
#include <math.h>

namespace nav {

constexpr float  DEG2RAD = 0.017453292519943295f;
constexpr float  RAD2DEG = 57.29577951308232f;
constexpr double EARTH_R = 6371000.0;

// wrap angle to [-pi, pi)
inline float wrapPi(float a) {
  while (a >= (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// Roll / pitch from the accelerometer (specific force, body frame, any unit).
// At rest and level the accelerometer reads (0, 0, -1 g) in this frame.
inline void rollPitchFromAccel(float fx, float fy, float fz, float &roll, float &pitch) {
  roll  = atan2f(-fy, -fz);
  pitch = atan2f(fx, sqrtf(fy * fy + fz * fz));
}

// Tilt-compensated magnetic heading (rad, 0 = magnetic north, +clockwise).
// m = magnetic field in body frame (after hard-iron calibration).
inline float tiltCompassHeading(float mx, float my, float mz, float roll, float pitch) {
  const float sr = sinf(roll), cr = cosf(roll), sp = sinf(pitch), cp = cosf(pitch);
  const float xh = mx * cp + my * sr * sp + mz * cr * sp;
  const float yh = my * cr - mz * sr;
  return atan2f(-yh, xh);
}

// Rotate a body-frame vector to the world (N, E, D) frame. ZYX Euler angles.
inline void bodyToWorld(float bx, float by, float bz, float roll, float pitch, float yaw,
                        float &n, float &e, float &d) {
  const float sr = sinf(roll), cr = cosf(roll);
  const float sp = sinf(pitch), cp = cosf(pitch);
  const float sy = sinf(yaw), cy = cosf(yaw);
  n = cp * cy * bx + (sr * sp * cy - cr * sy) * by + (cr * sp * cy + sr * sy) * bz;
  e = cp * sy * bx + (sr * sp * sy + cr * cy) * by + (cr * sp * sy - sr * cy) * bz;
  d = -sp * bx + sr * cp * by + cr * cp * bz;
}

// Where on the ground is the thing the camera sees?
//   Camera is mounted looking straight DOWN out of the bottom of the payload,
//   with the top edge of the image towards the payload's "forward" (X) side.
//   u = horizontal image position -0.5 (left) .. +0.5 (right)
//   v = vertical image position   -0.5 (top)  .. +0.5 (bottom)
//   hfov / vfov in rad, heightAGL in metres above the ground.
// Returns false if the pixel looks above the horizon.
inline bool projectToGround(float u, float v, float hfov, float vfov,
                            float roll, float pitch, float yaw, float heightAGL,
                            float &north, float &east) {
  if (heightAGL <= 1.0f) return false;
  const float bx = -2.0f * v * tanf(vfov * 0.5f);   // image up    -> forward
  const float by =  2.0f * u * tanf(hfov * 0.5f);   // image right -> right
  const float bz = 1.0f;                            // optical axis -> down
  float n, e, d;
  bodyToWorld(bx, by, bz, roll, pitch, yaw, n, e, d);
  if (d < 0.05f) return false;                      // (almost) horizontal ray
  const float k = heightAGL / d;
  north = n * k;
  east  = e * k;
  return true;
}

// Move a lat/lon (deg) by north/east metres (flat-earth, fine for < 200 km).
inline void offsetLatLon(double lat, double lon, double north, double east,
                         double &outLat, double &outLon) {
  const double d2r = 0.017453292519943295;
  outLat = lat + (north / EARTH_R) / d2r;
  outLon = lon + (east / (EARTH_R * cos(lat * d2r))) / d2r;
}

// Barometric altitude difference (m) relative to reference pressure p0.
inline float baroAltitude(float p, float p0) {
  return 44330.0f * (1.0f - powf(p / p0, 0.190295f));
}

// ---------------- flight simulator ----------------
// Simple wind model (metres MSL -> wind blowing TOWARDS north/east, m/s).
// Jet stream around 11 km, calmer in the stratosphere. Same model is in
// dashboard.html (windAt) so the landing prediction matches the simulation.
inline void windAt(float h, float &vn, float &ve) {
  float s;
  if (h < 11000.0f)      s = 4.0f + 26.0f * (h / 11000.0f);
  else if (h < 20000.0f) s = 30.0f - 22.0f * ((h - 11000.0f) / 9000.0f);
  else                   s = 8.0f;
  const float dir = (70.0f + 20.0f * sinf(h / 3000.0f)) * DEG2RAD;  // towards ENE
  vn = s * cosf(dir);
  ve = s * sinf(dir);
}

// Descent speed under parachute: ~5 m/s at sea level, faster in thin air.
inline float descentRate(float h) { return 5.0f * expf(h / 15200.0f); }

}  // namespace nav
