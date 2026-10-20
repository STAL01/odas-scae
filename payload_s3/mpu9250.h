// =====================================================================
//  Minimal MPU-9250 / MPU-6500 driver (accel, gyro, AK8963 magnetometer)
//  Outputs are already rotated to the body frame: X fwd, Y right, Z down.
//  Mount the GY-91 flat in the payload with its X arrow pointing "forward".
// =====================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "nav_math.h"

namespace imu {
uint8_t MPU = 0x68;               // 0x69 if the board's SDO/SAO pin is pulled high
const uint8_t MAG = 0x0C;
bool ok = false, magOk = false;
bool flip = false;                // GY-91 mounted chip-down: turned 180 deg about X (arrow still forward)
uint8_t whoami = 0;
float asa[3] = {1, 1, 1};
float gBias[3] = {0, 0, 0};
float magOff[3] = {0, 0, 0}, magScale[3] = {1, 1, 1};

void wr(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
bool rd(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool begin() {
  MPU = 0x68;
  if (!rd(MPU, 0x75, &whoami, 1)) {
    MPU = 0x69;
    if (!rd(MPU, 0x75, &whoami, 1)) return false;
  }
  wr(MPU, 0x6B, 0x80); delay(100);   // reset
  wr(MPU, 0x6B, 0x01); delay(10);    // wake, PLL clock
  wr(MPU, 0x6C, 0x00);               // all axes on
  wr(MPU, 0x1A, 0x03);               // gyro DLPF 41 Hz
  wr(MPU, 0x19, 0x04);               // 200 Hz sample rate
  wr(MPU, 0x1B, 0x08);               // gyro +-500 dps
  wr(MPU, 0x1C, 0x10);               // accel +-8 g
  wr(MPU, 0x1D, 0x03);               // accel DLPF 41 Hz
  wr(MPU, 0x6A, 0x00);               // internal I2C master off
  wr(MPU, 0x37, 0x02);               // bypass -> magnetometer appears at 0x0C
  delay(10);
  uint8_t wia = 0;
  magOk = rd(MAG, 0x00, &wia, 1) && wia == 0x48;
  if (magOk) {
    uint8_t a[3];
    wr(MAG, 0x0A, 0x00); delay(10);
    wr(MAG, 0x0A, 0x0F); delay(10);  // fuse ROM access
    rd(MAG, 0x10, a, 3);
    for (int i = 0; i < 3; i++) asa[i] = (a[i] - 128) / 256.0f + 1.0f;
    wr(MAG, 0x0A, 0x00); delay(10);
    wr(MAG, 0x0A, 0x16); delay(10);  // 16-bit, continuous 100 Hz
  }
  ok = true;
  return true;
}

// accel (g) and gyro (rad/s) in body frame X fwd, Y right, Z down
bool readAG(float a[3], float g[3]) {
  uint8_t b[14];
  if (!rd(MPU, 0x3B, b, 14)) return false;
  bool all00 = true, allFF = true;   // a lost or glitching chip reads as all 0x00 or all 0xFF
  for (int i = 0; i < 14; i++) { all00 &= b[i] == 0x00; allFF &= b[i] == 0xFF; }
  if (all00 || allFF) return false;
  const float AS = 1.0f / 4096.0f;
  const float GS = (1.0f / 65.5f) * nav::DEG2RAD;
  int16_t ax = (b[0] << 8) | b[1], ay = (b[2] << 8) | b[3], az = (b[4] << 8) | b[5];
  int16_t gx = (b[8] << 8) | b[9], gy = (b[10] << 8) | b[11], gz = (b[12] << 8) | b[13];
  // chip frame is X fwd, Y left, Z up  ->  flip Y and Z (chip-down mounting: they are already right)
  const float s = flip ? 1.0f : -1.0f;
  a[0] = ax * AS;  a[1] = s * ay * AS;  a[2] = s * az * AS;
  g[0] = gx * GS - gBias[0];  g[1] = s * gy * GS - gBias[1];  g[2] = s * gz * GS - gBias[2];
  return true;
}

// magnetometer in body frame (uT). raw[] = before calibration (for cal routine)
bool readMag(float m[3], float raw[3]) {
  if (!magOk) return false;
  uint8_t st1;
  if (!rd(MAG, 0x02, &st1, 1) || !(st1 & 0x01)) return false;  // no new data
  uint8_t b[7];
  if (!rd(MAG, 0x03, b, 7)) return false;                       // reading ST2 ends the cycle
  if (b[6] & 0x08) return false;                                // overflow
  int16_t hx = (b[1] << 8) | b[0], hy = (b[3] << 8) | b[2], hz = (b[5] << 8) | b[4];
  float x = hx * asa[0] * 0.15f, y = hy * asa[1] * 0.15f, z = hz * asa[2] * 0.15f;
  // AK8963 axes: X = chip Y, Y = chip X, Z = -chip Z  ->  body X fwd, Y right, Z down
  raw[0] = y;  raw[1] = flip ? x : -x;  raw[2] = flip ? -z : z;
  for (int i = 0; i < 3; i++) m[i] = (raw[i] - magOff[i]) * magScale[i];
  return true;
}

void calibrateGyro() {
  float sum[3] = {0, 0, 0}, a[3], g[3];
  gBias[0] = gBias[1] = gBias[2] = 0;
  int n = 0;
  for (int i = 0; i < 300; i++) {
    if (readAG(a, g)) { for (int k = 0; k < 3; k++) sum[k] += g[k]; n++; }
    delay(5);
  }
  if (n) for (int k = 0; k < 3; k++) gBias[k] = sum[k] / n;
}
}  // namespace imu
