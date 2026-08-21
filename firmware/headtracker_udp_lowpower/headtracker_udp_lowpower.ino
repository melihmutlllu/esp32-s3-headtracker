#include <Arduino.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

// ESP32-S3 Mini + MPU9250 head-tracker fusion test.
// Serial commands:
//   c = recenter current head pose
//   b = recalibrate gyro bias and recenter (keep still)
//   m = toggle magnetometer yaw correction
//   s = scan I2C bus
//   h = print help

static constexpr uint8_t SDA_PIN = 8;
static constexpr uint8_t SCL_PIN = 9;
static constexpr uint8_t MPU_ADDR = 0x68;

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t I2C_CLOCK_HZ = 400000;
static constexpr uint32_t SAMPLE_RATE_HZ = 125;
static constexpr uint32_t PRINT_RATE_HZ = 1;
static constexpr uint32_t UDP_SEND_RATE_HZ = 60;
static constexpr uint16_t CALIBRATION_SAMPLES = 750;
static constexpr uint32_t POST_WIFI_CALIBRATION_SETTLE_MS = 5000;
static constexpr uint32_t MAX_FILTER_DT_US = 10000;
static constexpr uint32_t ECO_CPU_MHZ = 80;
static constexpr bool SERIAL_PLOTTER_MODE = false;
static constexpr bool ENABLE_WIFI_UDP = true;
static constexpr bool ENABLE_AP_FALLBACK = false;
static constexpr bool SEND_ROLL_TO_OPENTRACK = false;
static constexpr bool ENABLE_OUTPUT_SMOOTHING = false;
static constexpr bool ENABLE_AUTO_GYRO_BIAS = true;
static constexpr bool WIFI_LOW_LATENCY_MODE = true;
static constexpr bool USE_UNICAST_TARGET = true;
static constexpr bool ENABLE_MAG_YAW_CORRECTION = true;
static constexpr uint32_t MPU_BASE_SAMPLE_RATE_HZ = 1000;
static constexpr uint8_t MPU_SAMPLE_RATE_DIVIDER =
  static_cast<uint8_t>((MPU_BASE_SAMPLE_RATE_HZ / SAMPLE_RATE_HZ) - 1);
static constexpr wifi_power_t WIFI_TX_POWER = WIFI_POWER_2dBm;

static_assert(MPU_BASE_SAMPLE_RATE_HZ % SAMPLE_RATE_HZ == 0,
              "SAMPLE_RATE_HZ must divide the MPU 1 kHz base rate");

#ifndef HEADTRACKER_WIFI_SSID
#define HEADTRACKER_WIFI_SSID ""
#endif

#ifndef HEADTRACKER_WIFI_PASSWORD
#define HEADTRACKER_WIFI_PASSWORD ""
#endif

#ifndef HEADTRACKER_OPENTRACK_PC_IP
#define HEADTRACKER_OPENTRACK_PC_IP 192, 168, 1, 3
#endif

#ifndef HEADTRACKER_AP_SSID
#define HEADTRACKER_AP_SSID "ESP32-HeadTracker"
#endif

#ifndef HEADTRACKER_AP_PASSWORD
#define HEADTRACKER_AP_PASSWORD "headtrack123"
#endif

// Fill secrets.h for normal WiFi station mode. Leave HEADTRACKER_WIFI_SSID
// empty to make the ESP32 create its own test access point instead.
static const char WIFI_SSID[] = HEADTRACKER_WIFI_SSID;
static const char WIFI_PASSWORD[] = HEADTRACKER_WIFI_PASSWORD;
static const char AP_SSID[] = HEADTRACKER_AP_SSID;
static const char AP_PASSWORD[] = HEADTRACKER_AP_PASSWORD;

static constexpr uint16_t OPENTRACK_UDP_PORT = 4242;
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000;
static constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 5000;
static constexpr uint8_t MAX_CONSECUTIVE_I2C_ERRORS = 8;
static constexpr uint16_t CALIBRATION_MIN_GOOD_SAMPLES = (CALIBRATION_SAMPLES * 9) / 10;
static constexpr uint8_t CALIBRATION_MAX_ATTEMPTS = 3;
static constexpr float CALIBRATION_ACCEL_MIN_G = 0.80f;
static constexpr float CALIBRATION_ACCEL_MAX_G = 1.20f;
static constexpr float CALIBRATION_MAX_ACCEL_STD_G = 0.05f;
static constexpr float CALIBRATION_MAX_GYRO_STD_DPS = 0.80f;
static constexpr float AUTO_BIAS_ACCEL_MIN_G = 0.90f;
static constexpr float AUTO_BIAS_ACCEL_MAX_G = 1.10f;
static constexpr float AUTO_BIAS_GYRO_XY_MAX_DPS = 0.35f;
static constexpr float AUTO_BIAS_GYRO_Z_MAX_DPS = 0.80f;
static constexpr float AUTO_BIAS_ALPHA_XY = 0.00015f;
static constexpr float AUTO_BIAS_ALPHA_Z = 0.00055f;
static constexpr uint16_t AUTO_BIAS_SETTLE_SAMPLES = static_cast<uint16_t>((SAMPLE_RATE_HZ * 3) / 2);
static constexpr float OUTPUT_SMOOTH_TAU_SLOW_S = 0.090f;
static constexpr float OUTPUT_SMOOTH_TAU_FAST_S = 0.018f;
static constexpr float OUTPUT_SMOOTH_SPEED_SLOW_DPS = 20.0f;
static constexpr float OUTPUT_SMOOTH_SPEED_FAST_DPS = 180.0f;
static constexpr float OUTPUT_MAX_RATE_DPS = 900.0f;
static constexpr float MAG_YAW_SIGN = 1.0f;
static constexpr float MAG_YAW_CORRECTION_GAIN = 0.25f;
static constexpr float MAG_YAW_MAX_CORRECTION_DPS = 2.0f;
static constexpr float MAG_YAW_FILTER_TAU_S = 0.70f;
static constexpr float MAG_FIELD_MIN_UT = 10.0f;
static constexpr float MAG_FIELD_MAX_UT = 90.0f;
static constexpr float MAG_MAX_TRUSTED_ERROR_DEG = 45.0f;
static constexpr float MAG_CORRECTION_MAX_GYRO_DPS = 160.0f;
static constexpr float MAG_BIAS_X_UT = 0.0f;
static constexpr float MAG_BIAS_Y_UT = 0.0f;
static constexpr float MAG_BIAS_Z_UT = 0.0f;
static constexpr float MAG_SCALE_X = 1.0f;
static constexpr float MAG_SCALE_Y = 1.0f;
static constexpr float MAG_SCALE_Z = 1.0f;
static IPAddress OPENTRACK_PC_IP(HEADTRACKER_OPENTRACK_PC_IP);

// MPU9250 full scale settings used below:
// accel +/-4 g => 8192 LSB/g, gyro +/-500 dps => 65.5 LSB/dps.
static constexpr float ACCEL_LSB_PER_G = 8192.0f;
static constexpr float GYRO_LSB_PER_DPS = 65.5f;
static constexpr float TEMP_LSB_PER_C = 333.87f;
static constexpr float TEMP_OFFSET_C = 21.0f;
static constexpr float DEG_TO_RAD_F = 0.01745329251994329577f;
static constexpr float RAD_TO_DEG_F = 57.295779513082320876f;

// Register map subset.
static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
static constexpr uint8_t REG_CONFIG = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
static constexpr uint8_t REG_INT_PIN_CFG = 0x37;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t REG_USER_CTRL = 0x6A;
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
static constexpr uint8_t REG_WHO_AM_I = 0x75;

// AK8963 magnetometer inside MPU9250. It is reached through I2C bypass.
static constexpr uint8_t MAG_ADDR = 0x0C;
static constexpr uint8_t MAG_REG_WIA = 0x00;
static constexpr uint8_t MAG_REG_ST1 = 0x02;
static constexpr uint8_t MAG_REG_HXL = 0x03;
static constexpr uint8_t MAG_REG_ST2 = 0x09;
static constexpr uint8_t MAG_REG_CNTL1 = 0x0A;
static constexpr uint8_t MAG_REG_CNTL2 = 0x0B;
static constexpr uint8_t MAG_REG_ASAX = 0x10;
static constexpr uint8_t MAG_MODE_POWER_DOWN = 0x00;
static constexpr uint8_t MAG_MODE_FUSE_ROM = 0x0F;
static constexpr uint8_t MAG_MODE_CONT_100HZ_16BIT = 0x16;

// Map sensor axes into a right-handed head/game frame:
//   logical X = physical roll axis  = sensor Y
//   logical Y = physical pitch axis = sensor X
//   logical Z = physical yaw axis   = -sensor Z
// The signs can still be adjusted later if OpenTrack direction feels reversed.
enum AxisIndex : uint8_t { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };
static constexpr AxisIndex MAP_OUT_X = AXIS_Y;
static constexpr AxisIndex MAP_OUT_Y = AXIS_X;
static constexpr AxisIndex MAP_OUT_Z = AXIS_Z;
static constexpr float SIGN_OUT_X = 1.0f;
static constexpr float SIGN_OUT_Y = 1.0f;
static constexpr float SIGN_OUT_Z = -1.0f;

static constexpr float OUTPUT_YAW_SIGN = -1.0f;
static constexpr float OUTPUT_PITCH_SIGN = -1.0f;
static constexpr float OUTPUT_ROLL_SIGN = 1.0f;

// Mahony 6DOF gain. Higher follows accelerometer faster; lower is smoother.
static constexpr float MAHONY_TWO_KP = 2.0f * 1.0f;
static constexpr float MAHONY_TWO_KI = 2.0f * 0.02f;

static_assert(sizeof(double) == 8, "OpenTrack UDP packet needs 64-bit doubles");

struct RawImu {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Quat {
  float w;
  float x;
  float y;
  float z;
};

struct EulerDeg {
  float yaw;
  float pitch;
  float roll;
};

struct CalibrationResult {
  bool stable;
  uint16_t good;
  Vec3 gyroMeanDps;
  Vec3 accelMeanG;
  float gyroStdDps;
  float accelMeanMagG;
  float accelStdG;
};

static Vec3 gyroBiasDps = {0.0f, 0.0f, 0.0f};
static Vec3 integralFeedback = {0.0f, 0.0f, 0.0f};
static Quat attitude = {1.0f, 0.0f, 0.0f, 0.0f};
static Quat centerAttitude = {1.0f, 0.0f, 0.0f, 0.0f};
static EulerDeg smoothedEuler = {0.0f, 0.0f, 0.0f};
static uint32_t i2cErrors = 0;
static uint32_t lateLoops = 0;
static uint32_t lastSampleUs = 0;
static uint32_t nextSampleUs = 0;
static WiFiUDP udp;
static IPAddress udpTargetIp(255, 255, 255, 255);
static bool udpReady = false;
static uint32_t udpPacketsOk = 0;
static uint32_t udpPacketsFail = 0;
static uint32_t wifiReconnects = 0;
static uint8_t consecutiveI2cErrors = 0;
static uint32_t autoBiasUpdates = 0;
static uint16_t stationarySamples = 0;
static int lastUdpResult = 0;
static bool outputFilterInitialized = false;
static bool magReady = false;
static bool magYawCorrectionEnabled = ENABLE_MAG_YAW_CORRECTION;
static bool magProbeAck = false;
static bool magYawReferenceValid = false;
static bool magYawFilterInitialized = false;
static uint8_t magFailStage = 0;
static uint8_t magWhoAmI = 0x00;
static Vec3 magAsaScale = {1.0f, 1.0f, 1.0f};
static float magCenterYawDeg = 0.0f;
static float magFilteredRelativeYawDeg = 0.0f;
static float magYawCorrectionOffsetDeg = 0.0f;
static float magLastFieldUt = 0.0f;
static float magLastYawErrorDeg = 0.0f;
static uint32_t magReadOk = 0;
static uint32_t magReadFail = 0;
static uint32_t magRejected = 0;

static void resetMagYawReference();

static bool writeReg(uint8_t reg, uint8_t value) {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission(true) == 0) {
      return true;
    }
    delay(2);
  }
  return false;
}

static bool readBytes(uint8_t reg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(MPU_ADDR, static_cast<uint8_t>(length), static_cast<uint8_t>(true));
  if (received != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

static bool readReg(uint8_t reg, uint8_t &value) {
  return readBytes(reg, &value, 1);
}

static bool i2cAddressResponds(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true) == 0;
}

static void scanI2cBus() {
  if (SERIAL_PLOTTER_MODE) {
    return;
  }

  writeReg(REG_USER_CTRL, 0x00);
  delay(10);
  writeReg(REG_INT_PIN_CFG, 0x02);
  delay(10);

  Serial.println("# i2c_scan_start");
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission(true);
    if (err == 0) {
      Serial.print("# i2c_device=0x");
      if (addr < 0x10) {
        Serial.print('0');
      }
      Serial.println(addr, HEX);
    }
  }
  Serial.println("# i2c_scan_done");
}

static bool magWriteReg(uint8_t reg, uint8_t value) {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission(true) == 0) {
      return true;
    }
    delay(2);
  }
  return false;
}

static bool magReadBytes(uint8_t reg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(MAG_ADDR, static_cast<uint8_t>(length), static_cast<uint8_t>(true));
  if (received != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

static bool magReadReg(uint8_t reg, uint8_t &value) {
  return magReadBytes(reg, &value, 1);
}

static int16_t makeI16(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

static bool readRawImu(RawImu &raw) {
  uint8_t b[14];
  if (!readBytes(REG_ACCEL_XOUT_H, b, sizeof(b))) {
    ++i2cErrors;
    return false;
  }

  raw.ax = makeI16(b[0], b[1]);
  raw.ay = makeI16(b[2], b[3]);
  raw.az = makeI16(b[4], b[5]);
  raw.temp = makeI16(b[6], b[7]);
  raw.gx = makeI16(b[8], b[9]);
  raw.gy = makeI16(b[10], b[11]);
  raw.gz = makeI16(b[12], b[13]);
  return true;
}

static Vec3 mapAxes(const Vec3 &v) {
  const float in[3] = {v.x, v.y, v.z};
  return {
    SIGN_OUT_X * in[MAP_OUT_X],
    SIGN_OUT_Y * in[MAP_OUT_Y],
    SIGN_OUT_Z * in[MAP_OUT_Z],
  };
}

static Vec3 accelGFromRaw(const RawImu &raw) {
  const Vec3 v = {
    static_cast<float>(raw.ax) / ACCEL_LSB_PER_G,
    static_cast<float>(raw.ay) / ACCEL_LSB_PER_G,
    static_cast<float>(raw.az) / ACCEL_LSB_PER_G,
  };
  return mapAxes(v);
}

static Vec3 gyroDpsFromRaw(const RawImu &raw) {
  const Vec3 v = {
    static_cast<float>(raw.gx) / GYRO_LSB_PER_DPS,
    static_cast<float>(raw.gy) / GYRO_LSB_PER_DPS,
    static_cast<float>(raw.gz) / GYRO_LSB_PER_DPS,
  };
  return mapAxes(v);
}

static bool configureMag() {
  magFailStage = 0;
  magProbeAck = false;

  if (!ENABLE_MAG_YAW_CORRECTION) {
    magReady = false;
    return true;
  }

  bool ok = true;
  ok &= writeReg(REG_USER_CTRL, 0x00);    // disable MPU internal I2C master
  delay(10);
  ok &= writeReg(REG_INT_PIN_CFG, 0x02);  // enable bypass to AK8963
  delay(10);

  magProbeAck = i2cAddressResponds(MAG_ADDR);
  if (!magProbeAck) {
    magWhoAmI = 0x00;
    magReady = false;
    magFailStage = 1;
    return false;
  }

  delay(10);
  if (!magWriteReg(MAG_REG_CNTL2, 0x01)) {  // soft reset
    magReady = false;
    magFailStage = 2;
    return false;
  }
  delay(100);
  if (!magWriteReg(MAG_REG_CNTL1, MAG_MODE_POWER_DOWN)) {
    magReady = false;
    magFailStage = 3;
    return false;
  }
  delay(10);

  if (!magReadReg(MAG_REG_WIA, magWhoAmI)) {
    magWhoAmI = 0x00;
    magReady = false;
    magFailStage = 4;
    return false;
  }

  if (magWhoAmI != 0x48) {
    magReady = false;
    magFailStage = 5;
    return false;
  }

  if (!magWriteReg(MAG_REG_CNTL1, MAG_MODE_FUSE_ROM)) {
    magReady = false;
    magFailStage = 6;
    return false;
  }
  delay(10);

  uint8_t asa[3] = {128, 128, 128};
  if (magReadBytes(MAG_REG_ASAX, asa, sizeof(asa))) {
    magAsaScale = {
      (static_cast<float>(asa[0]) - 128.0f) / 256.0f + 1.0f,
      (static_cast<float>(asa[1]) - 128.0f) / 256.0f + 1.0f,
      (static_cast<float>(asa[2]) - 128.0f) / 256.0f + 1.0f,
    };
  }

  if (!magWriteReg(MAG_REG_CNTL1, MAG_MODE_POWER_DOWN)) {
    magReady = false;
    magFailStage = 7;
    return false;
  }
  delay(10);
  if (!magWriteReg(MAG_REG_CNTL1, MAG_MODE_CONT_100HZ_16BIT)) {
    magReady = false;
    magFailStage = 8;
    return false;
  }
  delay(10);

  magReady = ok;
  magFailStage = ok ? 0 : 9;
  magYawReferenceValid = false;
  magYawFilterInitialized = false;
  return ok;
}

static bool readMagUt(Vec3 &magUt) {
  if (!magReady) {
    return false;
  }

  uint8_t st1 = 0x00;
  if (!magReadReg(MAG_REG_ST1, st1)) {
    ++magReadFail;
    return false;
  }

  if ((st1 & 0x01) == 0) {
    return false;
  }

  uint8_t b[7];
  if (!magReadBytes(MAG_REG_HXL, b, sizeof(b))) {
    ++magReadFail;
    return false;
  }

  if ((b[6] & 0x08) != 0) {
    ++magRejected;
    return false;
  }

  const int16_t rawX = makeI16(b[1], b[0]);
  const int16_t rawY = makeI16(b[3], b[2]);
  const int16_t rawZ = makeI16(b[5], b[4]);

  const Vec3 sensorUt = {
    static_cast<float>(rawX) * 0.15f * magAsaScale.x,
    static_cast<float>(rawY) * 0.15f * magAsaScale.y,
    static_cast<float>(rawZ) * 0.15f * magAsaScale.z,
  };

  Vec3 mapped = mapAxes(sensorUt);
  mapped.x = (mapped.x - MAG_BIAS_X_UT) * MAG_SCALE_X;
  mapped.y = (mapped.y - MAG_BIAS_Y_UT) * MAG_SCALE_Y;
  mapped.z = (mapped.z - MAG_BIAS_Z_UT) * MAG_SCALE_Z;

  const float field = sqrtf(mapped.x * mapped.x + mapped.y * mapped.y + mapped.z * mapped.z);
  magLastFieldUt = field;
  if (field < MAG_FIELD_MIN_UT || field > MAG_FIELD_MAX_UT) {
    ++magRejected;
    return false;
  }

  magUt = mapped;
  ++magReadOk;
  return true;
}

static float tempCFromRaw(const RawImu &raw) {
  return static_cast<float>(raw.temp) / TEMP_LSB_PER_C + TEMP_OFFSET_C;
}

static Vec3 vecAdd(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 vecSub(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec3 vecScale(const Vec3 &v, float s) {
  return {v.x * s, v.y * s, v.z * s};
}

static float vecDot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static Vec3 vecCross(const Vec3 &a, const Vec3 &b) {
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x,
  };
}

static float vecMagnitude(const Vec3 &v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

static float wrapAngle180(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }
  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }
  return angleDeg;
}

static float angleDeltaDeg(float fromDeg, float toDeg) {
  return wrapAngle180(toDeg - fromDeg);
}

static Vec3 vecNormalize(const Vec3 &v, const Vec3 &fallback) {
  const float mag = vecMagnitude(v);
  if (mag <= 1.0e-6f) {
    return fallback;
  }
  return vecScale(v, 1.0f / mag);
}

static Quat quatNormalize(Quat q) {
  const float mag = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (mag <= 1.0e-6f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }
  const float inv = 1.0f / mag;
  return {q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

static Quat quatConjugate(const Quat &q) {
  return {q.w, -q.x, -q.y, -q.z};
}

static Quat quatMultiply(const Quat &a, const Quat &b) {
  return {
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

static Quat quatFromTwoVectors(Vec3 from, Vec3 to) {
  from = vecNormalize(from, {0.0f, 0.0f, 1.0f});
  to = vecNormalize(to, {0.0f, 0.0f, 1.0f});

  const float dot = vecDot(from, to);
  if (dot < -0.9999f) {
    Vec3 axis = vecCross({1.0f, 0.0f, 0.0f}, from);
    if (vecMagnitude(axis) < 0.01f) {
      axis = vecCross({0.0f, 1.0f, 0.0f}, from);
    }
    axis = vecNormalize(axis, {1.0f, 0.0f, 0.0f});
    return {0.0f, axis.x, axis.y, axis.z};
  }

  const Vec3 cross = vecCross(from, to);
  return quatNormalize({1.0f + dot, cross.x, cross.y, cross.z});
}

static EulerDeg eulerFromQuat(const Quat &qIn) {
  const Quat q = quatNormalize(qIn);

  const float sinRollCosPitch = 2.0f * (q.w * q.x + q.y * q.z);
  const float cosRollCosPitch = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  const float roll = atan2f(sinRollCosPitch, cosRollCosPitch);

  const float sinPitch = 2.0f * (q.w * q.y - q.z * q.x);
  float pitch;
  if (fabsf(sinPitch) >= 1.0f) {
    pitch = copysignf(PI / 2.0f, sinPitch);
  } else {
    pitch = asinf(sinPitch);
  }

  const float sinYawCosPitch = 2.0f * (q.w * q.z + q.x * q.y);
  const float cosYawCosPitch = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  const float yaw = atan2f(sinYawCosPitch, cosYawCosPitch);

  return {
    OUTPUT_YAW_SIGN * yaw * RAD_TO_DEG_F,
    OUTPUT_PITCH_SIGN * pitch * RAD_TO_DEG_F,
    OUTPUT_ROLL_SIGN * roll * RAD_TO_DEG_F,
  };
}

static bool configureMpu() {
  bool ok = true;

  ok &= writeReg(REG_PWR_MGMT_1, 0x80);  // reset
  delay(100);
  ok &= writeReg(REG_PWR_MGMT_1, 0x01);  // clock source: PLL with X gyro
  delay(10);
  ok &= writeReg(REG_PWR_MGMT_2, 0x00);  // enable accel and gyro axes
  delay(10);

  ok &= writeReg(REG_USER_CTRL, 0x00);     // keep internal I2C master disabled for bypass
  ok &= writeReg(REG_CONFIG, 0x03);        // gyro DLPF around 44 Hz
  ok &= writeReg(REG_SMPLRT_DIV, MPU_SAMPLE_RATE_DIVIDER);
  ok &= writeReg(REG_GYRO_CONFIG, 0x08);   // +/-500 dps
  ok &= writeReg(REG_ACCEL_CONFIG, 0x08);  // +/-4 g
  ok &= writeReg(REG_ACCEL_CONFIG2, 0x03); // accel DLPF around 44 Hz
  ok &= writeReg(REG_INT_PIN_CFG, 0x02);   // bypass enable for AK8963 magnetometer

  return ok;
}

static bool recoverMpu() {
  if (!SERIAL_PLOTTER_MODE) {
    Serial.println("# i2c_recover_mpu");
  }

  Wire.end();
  delay(10);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(20);

  consecutiveI2cErrors = 0;
  const bool mpuOk = configureMpu();
  const bool magOk = configureMag();
  return mpuOk && magOk;
}

static const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_watchdog";
    case ESP_RST_TASK_WDT: return "task_watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
  }
}

static bool hasStationCredentials() {
  return WIFI_SSID[0] != '\0';
}

static IPAddress subnetBroadcastAddress() {
  const uint32_t ip = static_cast<uint32_t>(WiFi.localIP());
  const uint32_t mask = static_cast<uint32_t>(WiFi.subnetMask());
  return IPAddress(ip | ~mask);
}

static void printWifiStatus(const char *modeName) {
  if (SERIAL_PLOTTER_MODE) {
    return;
  }

  Serial.print("# wifi_mode=");
  Serial.println(modeName);
  Serial.print("# local_ip=");
  if (WiFi.getMode() & WIFI_AP) {
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(WiFi.localIP());
  }
  Serial.print("# udp_target=");
  Serial.print(udpTargetIp);
  Serial.print(':');
  Serial.println(OPENTRACK_UDP_PORT);
  Serial.print("# wifi_status=");
  Serial.println(WiFi.status());
}

static void setupWifiUdp() {
  if (!ENABLE_WIFI_UDP) {
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    udpReady = false;

    if (!SERIAL_PLOTTER_MODE) {
      Serial.println("# wifi_disabled_safe_mode");
    }
    return;
  }

  WiFi.persistent(false);
  WiFi.setTxPower(WIFI_TX_POWER);
  WiFi.setSleep(!WIFI_LOW_LATENCY_MODE);
  esp_wifi_set_ps(WIFI_LOW_LATENCY_MODE ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM);

  if (hasStationCredentials()) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    if (!SERIAL_PLOTTER_MODE) {
      Serial.print("# wifi_connecting_ssid=");
      Serial.println(WIFI_SSID);
    }

    const uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMs < WIFI_CONNECT_TIMEOUT_MS) {
      delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
      WiFi.setTxPower(WIFI_TX_POWER);
      WiFi.setSleep(!WIFI_LOW_LATENCY_MODE);
      esp_wifi_set_ps(WIFI_LOW_LATENCY_MODE ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM);
      udpTargetIp = USE_UNICAST_TARGET ? OPENTRACK_PC_IP : subnetBroadcastAddress();
      udpReady = udp.begin(0);
      printWifiStatus(USE_UNICAST_TARGET ? "station_unicast_low_latency" : "station_subnet_broadcast_lowpower");
      return;
    }

    if (!SERIAL_PLOTTER_MODE) {
      Serial.println("# wifi_station_failed");
      Serial.print("# wifi_status=");
      Serial.println(WiFi.status());
    }
  }

  if (!ENABLE_AP_FALLBACK) {
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    udpReady = false;

    if (!SERIAL_PLOTTER_MODE) {
      Serial.println("# wifi_disabled_no_ap_fallback");
    }
    return;
  }

  WiFi.disconnect(true);
  delay(200);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_TX_POWER);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  udpTargetIp = IPAddress(192, 168, 4, 255);
  udpReady = udp.begin(0);
  printWifiStatus("softap_broadcast");

  if (!SERIAL_PLOTTER_MODE) {
    Serial.print("# ap_ssid=");
    Serial.println(AP_SSID);
    Serial.print("# ap_password=");
    Serial.println(AP_PASSWORD);
  }
}

static void serviceWifiReconnect() {
  if (!ENABLE_WIFI_UDP || !hasStationCredentials()) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!udpReady) {
      udpReady = udp.begin(0);
    }
    return;
  }

  udpReady = false;

  static uint32_t lastReconnectMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastReconnectMs < WIFI_RECONNECT_INTERVAL_MS) {
    return;
  }
  lastReconnectMs = nowMs;
  ++wifiReconnects;

  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_TX_POWER);
  WiFi.setSleep(!WIFI_LOW_LATENCY_MODE);
  esp_wifi_set_ps(WIFI_LOW_LATENCY_MODE ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  if (!SERIAL_PLOTTER_MODE) {
    Serial.print("# wifi_reconnect_attempt=");
    Serial.println(wifiReconnects);
  }
}

static void sendOpenTrackPose(const EulerDeg &euler) {
  if (!udpReady || WiFi.status() != WL_CONNECTED) {
    return;
  }

  const double pose[6] = {
    0.0,
    0.0,
    0.0,
    static_cast<double>(euler.yaw),
    static_cast<double>(euler.pitch),
    SEND_ROLL_TO_OPENTRACK ? static_cast<double>(euler.roll) : 0.0,
  };

  if (udp.beginPacket(udpTargetIp, OPENTRACK_UDP_PORT) != 1) {
    ++udpPacketsFail;
    lastUdpResult = -1;
    return;
  }

  udp.write(reinterpret_cast<const uint8_t *>(pose), sizeof(pose));
  lastUdpResult = udp.endPacket();
  if (lastUdpResult == 1) {
    ++udpPacketsOk;
  } else {
    ++udpPacketsFail;
  }
}

static void recenter() {
  centerAttitude = attitude;
  smoothedEuler = {0.0f, 0.0f, 0.0f};
  outputFilterInitialized = false;
  resetMagYawReference();
  if (!SERIAL_PLOTTER_MODE) {
    Serial.println("# recentered");
  }
}

static void initializeAttitudeFromAccel(const Vec3 &accelAvg) {
  const Vec3 acc = vecNormalize(accelAvg, {0.0f, 0.0f, 1.0f});
  attitude = quatFromTwoVectors(acc, {0.0f, 0.0f, 1.0f});
  centerAttitude = attitude;
  integralFeedback = {0.0f, 0.0f, 0.0f};
  smoothedEuler = {0.0f, 0.0f, 0.0f};
  outputFilterInitialized = false;
  resetMagYawReference();

  if (!SERIAL_PLOTTER_MODE) {
    Serial.print("# initial_accel_g=");
    Serial.print(acc.x, 5);
    Serial.print(',');
    Serial.print(acc.y, 5);
    Serial.print(',');
    Serial.println(acc.z, 5);
  }
}

static CalibrationResult collectCalibrationAttempt(uint8_t attempt) {
  Vec3 gyroSum = {0.0f, 0.0f, 0.0f};
  Vec3 gyroSqSum = {0.0f, 0.0f, 0.0f};
  Vec3 accelSum = {0.0f, 0.0f, 0.0f};
  float accelMagSum = 0.0f;
  float accelMagSqSum = 0.0f;
  uint16_t good = 0;
  const uint32_t calibrationDelayUs = 1000000UL / SAMPLE_RATE_HZ;

  if (!SERIAL_PLOTTER_MODE) {
    Serial.print("# calibration_attempt=");
    Serial.println(attempt);
  }

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    RawImu raw;
    if (readRawImu(raw)) {
      const Vec3 gyro = gyroDpsFromRaw(raw);
      const Vec3 accel = accelGFromRaw(raw);
      const float accelMag = vecMagnitude(accel);

      if (accelMag > 0.50f && accelMag < 1.50f) {
        gyroSum = vecAdd(gyroSum, gyro);
        gyroSqSum = vecAdd(gyroSqSum, {gyro.x * gyro.x, gyro.y * gyro.y, gyro.z * gyro.z});
        accelSum = vecAdd(accelSum, accel);
        accelMagSum += accelMag;
        accelMagSqSum += accelMag * accelMag;
        ++good;
      }
    }
    delayMicroseconds(calibrationDelayUs);
  }

  CalibrationResult result = {};
  result.good = good;

  if (good == 0) {
    return result;
  }

  const float invGood = 1.0f / static_cast<float>(good);
  result.gyroMeanDps = vecScale(gyroSum, invGood);
  result.accelMeanG = vecScale(accelSum, invGood);
  result.accelMeanMagG = accelMagSum * invGood;

  const float gyroVarX = fmaxf(0.0f, gyroSqSum.x * invGood - result.gyroMeanDps.x * result.gyroMeanDps.x);
  const float gyroVarY = fmaxf(0.0f, gyroSqSum.y * invGood - result.gyroMeanDps.y * result.gyroMeanDps.y);
  const float gyroVarZ = fmaxf(0.0f, gyroSqSum.z * invGood - result.gyroMeanDps.z * result.gyroMeanDps.z);
  const float accelMagVar = fmaxf(0.0f, accelMagSqSum * invGood - result.accelMeanMagG * result.accelMeanMagG);
  result.gyroStdDps = sqrtf((gyroVarX + gyroVarY + gyroVarZ) / 3.0f);
  result.accelStdG = sqrtf(accelMagVar);

  result.stable =
    good >= CALIBRATION_MIN_GOOD_SAMPLES &&
    result.accelMeanMagG >= CALIBRATION_ACCEL_MIN_G &&
    result.accelMeanMagG <= CALIBRATION_ACCEL_MAX_G &&
    result.accelStdG <= CALIBRATION_MAX_ACCEL_STD_G &&
    result.gyroStdDps <= CALIBRATION_MAX_GYRO_STD_DPS;

  return result;
}

static float calibrationScore(const CalibrationResult &result) {
  if (result.good == 0) {
    return 1.0e9f;
  }
  return result.gyroStdDps + (result.accelStdG * 20.0f) +
    fabsf(result.accelMeanMagG - 1.0f) * 5.0f;
}

static void applyCalibration(const CalibrationResult &result) {
  gyroBiasDps = result.gyroMeanDps;
  initializeAttitudeFromAccel(result.accelMeanG);
  stationarySamples = 0;
  autoBiasUpdates = 0;
}

static void printCalibrationResult(const CalibrationResult &result, bool accepted) {
  if (SERIAL_PLOTTER_MODE) {
    return;
  }

  Serial.print("# calibration_good=");
  Serial.println(result.good);
  Serial.print("# calibration_stable=");
  Serial.println(result.stable ? "true" : "false");
  Serial.print("# calibration_accepted=");
  Serial.println(accepted ? "true" : "false");
  Serial.print("# calibration_gyro_std_dps=");
  Serial.println(result.gyroStdDps, 5);
  Serial.print("# calibration_accel_mean_g=");
  Serial.println(result.accelMeanMagG, 5);
  Serial.print("# calibration_accel_std_g=");
  Serial.println(result.accelStdG, 5);
  Serial.print("# gyro_bias_dps=");
  Serial.print(gyroBiasDps.x, 5);
  Serial.print(',');
  Serial.print(gyroBiasDps.y, 5);
  Serial.print(',');
  Serial.println(gyroBiasDps.z, 5);
}

static void calibrateGyroAndInitAttitude() {
  if (!SERIAL_PLOTTER_MODE) {
    Serial.println("# Keep the sensor still: gyro calibration starts");
  }

  CalibrationResult best = {};
  float bestScore = 1.0e9f;
  bool acceptedStable = false;

  for (uint8_t attempt = 1; attempt <= CALIBRATION_MAX_ATTEMPTS; ++attempt) {
    const CalibrationResult result = collectCalibrationAttempt(attempt);
    const float score = calibrationScore(result);

    if (score < bestScore) {
      best = result;
      bestScore = score;
    }

    if (!SERIAL_PLOTTER_MODE) {
      Serial.print("# calibration_attempt_stable=");
      Serial.println(result.stable ? "true" : "false");
    }

    if (result.stable) {
      best = result;
      acceptedStable = true;
      break;
    }

    delay(300);
  }

  if (best.good > 0) {
    applyCalibration(best);
  } else {
    gyroBiasDps = {0.0f, 0.0f, 0.0f};
    attitude = {1.0f, 0.0f, 0.0f, 0.0f};
    centerAttitude = attitude;
    stationarySamples = 0;
    autoBiasUpdates = 0;
  }

  printCalibrationResult(best, best.good > 0);

  if (!SERIAL_PLOTTER_MODE) {
    if (!acceptedStable) {
      Serial.println("# warning: calibration was noisy; keep sensor still and send 'b' to recalibrate");
    }
    Serial.println("# calibration done");
  }
}

static void updateMahony(float gxRad, float gyRad, float gzRad, const Vec3 &accelG, float dt) {
  float ax = accelG.x;
  float ay = accelG.y;
  float az = accelG.z;

  const float accelMag = sqrtf(ax * ax + ay * ay + az * az);
  if (accelMag > 0.50f && accelMag < 1.50f) {
    const float invAccelMag = 1.0f / accelMag;
    ax *= invAccelMag;
    ay *= invAccelMag;
    az *= invAccelMag;

    const float halfVx = attitude.x * attitude.z - attitude.w * attitude.y;
    const float halfVy = attitude.w * attitude.x + attitude.y * attitude.z;
    const float halfVz = attitude.w * attitude.w - 0.5f + attitude.z * attitude.z;

    const float halfEx = (ay * halfVz - az * halfVy);
    const float halfEy = (az * halfVx - ax * halfVz);
    const float halfEz = (ax * halfVy - ay * halfVx);

    if (MAHONY_TWO_KI > 0.0f) {
      integralFeedback.x += MAHONY_TWO_KI * halfEx * dt;
      integralFeedback.y += MAHONY_TWO_KI * halfEy * dt;
      integralFeedback.z += MAHONY_TWO_KI * halfEz * dt;
      gxRad += integralFeedback.x;
      gyRad += integralFeedback.y;
      gzRad += integralFeedback.z;
    } else {
      integralFeedback = {0.0f, 0.0f, 0.0f};
    }

    gxRad += MAHONY_TWO_KP * halfEx;
    gyRad += MAHONY_TWO_KP * halfEy;
    gzRad += MAHONY_TWO_KP * halfEz;
  }

  gxRad *= 0.5f * dt;
  gyRad *= 0.5f * dt;
  gzRad *= 0.5f * dt;

  const Quat q = attitude;
  attitude.w += (-q.x * gxRad - q.y * gyRad - q.z * gzRad);
  attitude.x += ( q.w * gxRad + q.y * gzRad - q.z * gyRad);
  attitude.y += ( q.w * gyRad - q.x * gzRad + q.z * gxRad);
  attitude.z += ( q.w * gzRad + q.x * gyRad - q.y * gxRad);
  attitude = quatNormalize(attitude);
}

static void serviceAutoGyroBias(const Vec3 &rawGyroDps, const Vec3 &correctedGyroDps, const Vec3 &accelG) {
  if (!ENABLE_AUTO_GYRO_BIAS) {
    stationarySamples = 0;
    return;
  }

  const float accelMag = vecMagnitude(accelG);
  const bool stationary =
    accelMag >= AUTO_BIAS_ACCEL_MIN_G &&
    accelMag <= AUTO_BIAS_ACCEL_MAX_G &&
    fabsf(correctedGyroDps.x) <= AUTO_BIAS_GYRO_XY_MAX_DPS &&
    fabsf(correctedGyroDps.y) <= AUTO_BIAS_GYRO_XY_MAX_DPS &&
    fabsf(correctedGyroDps.z) <= AUTO_BIAS_GYRO_Z_MAX_DPS;

  if (!stationary) {
    stationarySamples = 0;
    return;
  }

  if (stationarySamples < UINT16_MAX) {
    ++stationarySamples;
  }

  if (stationarySamples < AUTO_BIAS_SETTLE_SAMPLES) {
    return;
  }

  const Vec3 biasError = vecSub(rawGyroDps, gyroBiasDps);
  gyroBiasDps.x += biasError.x * AUTO_BIAS_ALPHA_XY;
  gyroBiasDps.y += biasError.y * AUTO_BIAS_ALPHA_XY;
  gyroBiasDps.z += biasError.z * AUTO_BIAS_ALPHA_Z;
  ++autoBiasUpdates;
}

static void resetMagYawReference() {
  magYawReferenceValid = false;
  magYawFilterInitialized = false;
  magCenterYawDeg = 0.0f;
  magFilteredRelativeYawDeg = 0.0f;
  magYawCorrectionOffsetDeg = 0.0f;
  magLastYawErrorDeg = 0.0f;
}

static bool magHeadingFromAccel(const Vec3 &magUt, const Vec3 &accelG, float &headingDeg) {
  const Vec3 up = vecNormalize(accelG, {0.0f, 0.0f, 1.0f});
  const Vec3 mag = vecNormalize(magUt, {1.0f, 0.0f, 0.0f});
  const Vec3 horizontal = vecSub(mag, vecScale(up, vecDot(mag, up)));

  if (vecMagnitude(horizontal) < 0.10f) {
    return false;
  }

  headingDeg = wrapAngle180(MAG_YAW_SIGN * atan2f(horizontal.y, horizontal.x) * RAD_TO_DEG_F);
  return true;
}

static EulerDeg applyMagYawCorrection(EulerDeg euler, const Vec3 &accelG, const Vec3 &gyroDps, float dt) {
  if (!magYawCorrectionEnabled || !magReady) {
    return euler;
  }

  Vec3 magUt;
  float headingDeg = 0.0f;
  if (readMagUt(magUt) && magHeadingFromAccel(magUt, accelG, headingDeg)) {
    if (!magYawReferenceValid) {
      magCenterYawDeg = headingDeg;
      magFilteredRelativeYawDeg = euler.yaw;
      magYawCorrectionOffsetDeg = 0.0f;
      magYawReferenceValid = true;
      magYawFilterInitialized = true;
      magLastYawErrorDeg = 0.0f;
    } else {
      const float magRelativeYaw = wrapAngle180(headingDeg - magCenterYawDeg);

      if (!magYawFilterInitialized) {
        magFilteredRelativeYawDeg = magRelativeYaw;
        magYawFilterInitialized = true;
      } else {
        const float alpha = 1.0f - expf(-dt / fmaxf(MAG_YAW_FILTER_TAU_S, 0.001f));
        magFilteredRelativeYawDeg = wrapAngle180(
          magFilteredRelativeYawDeg +
          angleDeltaDeg(magFilteredRelativeYawDeg, magRelativeYaw) * alpha);
      }

      const float correctedYaw = wrapAngle180(euler.yaw + magYawCorrectionOffsetDeg);
      const float yawError = angleDeltaDeg(correctedYaw, magFilteredRelativeYawDeg);
      magLastYawErrorDeg = yawError;

      if (fabsf(yawError) <= MAG_MAX_TRUSTED_ERROR_DEG &&
          vecMagnitude(gyroDps) <= MAG_CORRECTION_MAX_GYRO_DPS) {
        const float maxStep = MAG_YAW_MAX_CORRECTION_DPS * dt;
        const float correctionStep = clampFloat(
          yawError * MAG_YAW_CORRECTION_GAIN * dt,
          -maxStep,
          maxStep);
        magYawCorrectionOffsetDeg = wrapAngle180(magYawCorrectionOffsetDeg + correctionStep);
      } else {
        ++magRejected;
      }
    }
  }

  euler.yaw = wrapAngle180(euler.yaw + magYawCorrectionOffsetDeg);
  return euler;
}

static float smoothAngleAxis(float currentDeg, float targetDeg, float dt) {
  float delta = angleDeltaDeg(currentDeg, targetDeg);
  const float maxStep = OUTPUT_MAX_RATE_DPS * dt;
  delta = clampFloat(delta, -maxStep, maxStep);

  const float speedDps = fabsf(delta) / fmaxf(dt, 0.001f);
  const float speedMix = clampFloat(
    (speedDps - OUTPUT_SMOOTH_SPEED_SLOW_DPS) /
      (OUTPUT_SMOOTH_SPEED_FAST_DPS - OUTPUT_SMOOTH_SPEED_SLOW_DPS),
    0.0f,
    1.0f);
  const float tau = OUTPUT_SMOOTH_TAU_SLOW_S +
    (OUTPUT_SMOOTH_TAU_FAST_S - OUTPUT_SMOOTH_TAU_SLOW_S) * speedMix;
  const float alpha = 1.0f - expf(-dt / fmaxf(tau, 0.001f));

  return wrapAngle180(currentDeg + delta * alpha);
}

static EulerDeg filterOutputEuler(const EulerDeg &rawEuler, float dt) {
  if (!ENABLE_OUTPUT_SMOOTHING) {
    return rawEuler;
  }

  if (!outputFilterInitialized) {
    smoothedEuler = rawEuler;
    outputFilterInitialized = true;
    return smoothedEuler;
  }

  smoothedEuler.yaw = smoothAngleAxis(smoothedEuler.yaw, rawEuler.yaw, dt);
  smoothedEuler.pitch = smoothAngleAxis(smoothedEuler.pitch, rawEuler.pitch, dt);
  smoothedEuler.roll = smoothAngleAxis(smoothedEuler.roll, rawEuler.roll, dt);
  return smoothedEuler;
}

static void printHeader(uint8_t whoAmI) {
  if (SERIAL_PLOTTER_MODE) {
    return;
  }

  Serial.println();
  Serial.println("# headtracker_udp_lowpower");
  Serial.print("# WHO_AM_I=0x");
  if (whoAmI < 0x10) {
    Serial.print('0');
  }
  Serial.println(whoAmI, HEX);
  Serial.print("# I2C_HZ=");
  Serial.print(I2C_CLOCK_HZ);
  Serial.print(", SAMPLE_HZ=");
  Serial.print(SAMPLE_RATE_HZ);
  Serial.print(", PRINT_HZ=");
  Serial.println(PRINT_RATE_HZ);
  Serial.print("# CPU_MHZ=");
  Serial.println(getCpuFrequencyMhz());
  Serial.print("# UDP_HZ=");
  Serial.println(UDP_SEND_RATE_HZ);
  Serial.print("# POST_WIFI_CALIBRATION_SETTLE_MS=");
  Serial.println(POST_WIFI_CALIBRATION_SETTLE_MS);
  Serial.print("# MPU_SMPLRT_DIV=");
  Serial.println(MPU_SAMPLE_RATE_DIVIDER);
  Serial.println(USE_UNICAST_TARGET ? "# WIFI_UDP=enabled_station_unicast_low_latency" : "# WIFI_UDP=enabled_station_subnet_broadcast_lowpower");
  Serial.println("# WIFI_TX_POWER=2dBm");
  Serial.print("# WIFI_LOW_LATENCY_MODE=");
  Serial.println(WIFI_LOW_LATENCY_MODE ? "true" : "false");
  Serial.print("# USE_UNICAST_TARGET=");
  Serial.println(USE_UNICAST_TARGET ? "true" : "false");
  Serial.print("# OPENTRACK_PC_IP=");
  Serial.println(OPENTRACK_PC_IP);
  Serial.print("# SEND_ROLL_TO_OPENTRACK=");
  Serial.println(SEND_ROLL_TO_OPENTRACK ? "true" : "false");
  Serial.print("# ENABLE_OUTPUT_SMOOTHING=");
  Serial.println(ENABLE_OUTPUT_SMOOTHING ? "true" : "false");
  Serial.print("# ENABLE_AUTO_GYRO_BIAS=");
  Serial.println(ENABLE_AUTO_GYRO_BIAS ? "true" : "false");
  Serial.print("# AUTO_BIAS_GYRO_XY_MAX_DPS=");
  Serial.println(AUTO_BIAS_GYRO_XY_MAX_DPS, 2);
  Serial.print("# AUTO_BIAS_GYRO_Z_MAX_DPS=");
  Serial.println(AUTO_BIAS_GYRO_Z_MAX_DPS, 2);
  Serial.print("# AUTO_BIAS_ALPHA_XY=");
  Serial.println(AUTO_BIAS_ALPHA_XY, 5);
  Serial.print("# AUTO_BIAS_ALPHA_Z=");
  Serial.println(AUTO_BIAS_ALPHA_Z, 5);
  Serial.print("# AUTO_BIAS_SETTLE_SAMPLES=");
  Serial.println(AUTO_BIAS_SETTLE_SAMPLES);
  Serial.print("# ENABLE_MAG_YAW_CORRECTION=");
  Serial.println(ENABLE_MAG_YAW_CORRECTION ? "true" : "false");
  Serial.print("# MAG_YAW_RUNTIME_ENABLED=");
  Serial.println(magYawCorrectionEnabled ? "true" : "false");
  Serial.print("# MAG_AK8963_WHO_AM_I=0x");
  if (magWhoAmI < 0x10) {
    Serial.print('0');
  }
  Serial.println(magWhoAmI, HEX);
  Serial.print("# MAG_READY=");
  Serial.println(magReady ? "true" : "false");
  Serial.print("# MAG_PROBE_0x0C_ACK=");
  Serial.println(magProbeAck ? "true" : "false");
  Serial.print("# MAG_FAIL_STAGE=");
  Serial.println(magFailStage);
  Serial.print("# MAG_YAW_CORRECTION_GAIN=");
  Serial.println(MAG_YAW_CORRECTION_GAIN, 3);
  Serial.print("# MAG_YAW_MAX_CORRECTION_DPS=");
  Serial.println(MAG_YAW_MAX_CORRECTION_DPS, 2);
  Serial.print("# OUTPUT_SMOOTH_TAU_SLOW_S=");
  Serial.println(OUTPUT_SMOOTH_TAU_SLOW_S, 3);
  Serial.print("# OUTPUT_SMOOTH_TAU_FAST_S=");
  Serial.println(OUTPUT_SMOOTH_TAU_FAST_S, 3);
  Serial.print("# OUTPUT_MAX_RATE_DPS=");
  Serial.println(OUTPUT_MAX_RATE_DPS, 1);
  Serial.println("# idle_wait=delay_yield");
  Serial.print("# reset_reason=");
  Serial.println(resetReasonName(esp_reset_reason()));
  Serial.println("# mapped frame: X=sensorY, Y=sensorX, Z=-sensorZ");
  Serial.println("# commands: c=recenter, b=gyro calibration + recenter, m=toggle mag yaw correction, s=scan I2C, h=help");
  Serial.println("# columns:");
  Serial.println("YPR,t_us,dt_us,yaw_deg,pitch_deg,roll_deg,ax_g,ay_g,az_g,accel_mag_g,gx_dps,gy_dps,gz_dps,temp_c,i2c_errors,late_loops,udp_ok,udp_fail,udp_last,wifi_rssi,wifi_reconnects,i2c_consecutive,auto_bias_updates,stationary_samples,mag_ok,mag_fail,mag_reject,mag_field_ut,mag_err_deg,mag_offset_deg");
}

static void idleUntilNextSample(uint32_t nowUs) {
  const uint32_t waitUs = nextSampleUs - nowUs;
  if (waitUs > 2000) {
    delay(1);
  } else if (waitUs > 200) {
    delayMicroseconds(100);
  } else {
    yield();
  }
}

static void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == 'c' || c == 'C') {
      recenter();
    } else if (c == 'b' || c == 'B') {
      calibrateGyroAndInitAttitude();
      nextSampleUs = micros();
      lastSampleUs = nextSampleUs;
    } else if (c == 'm' || c == 'M') {
      magYawCorrectionEnabled = !magYawCorrectionEnabled;
      resetMagYawReference();
      if (!SERIAL_PLOTTER_MODE) {
        Serial.print("# mag_yaw_correction_enabled=");
        Serial.println(magYawCorrectionEnabled ? "true" : "false");
      }
    } else if (c == 's' || c == 'S') {
      scanI2cBus();
    } else if (c == 'h' || c == 'H') {
      if (!SERIAL_PLOTTER_MODE) {
        Serial.println("# commands: c=recenter, b=gyro calibration + recenter, m=toggle mag yaw correction, s=scan I2C, h=help");
      }
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 3000) {
    delay(10);
  }

  setCpuFrequencyMhz(ECO_CPU_MHZ);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(100);

  const bool configOk = configureMpu();
  const bool magConfigOk = configureMag();

  uint8_t whoAmI = 0x00;
  if (!readReg(REG_WHO_AM_I, whoAmI)) {
    ++i2cErrors;
  }

  printHeader(whoAmI);
  if (!SERIAL_PLOTTER_MODE) {
    Serial.print("# config_ok=");
    Serial.println(configOk ? "true" : "false");
    Serial.print("# mag_config_ok=");
    Serial.println(magConfigOk ? "true" : "false");
  }

  if (!SERIAL_PLOTTER_MODE && !(whoAmI == 0x71 || whoAmI == 0x73 || whoAmI == 0x70)) {
    Serial.println("# warning: unexpected WHO_AM_I value; check address/wiring/module variant");
  }

  setupWifiUdp();

  if (!SERIAL_PLOTTER_MODE) {
    Serial.print("# post_wifi_calibration_settle_ms=");
    Serial.println(POST_WIFI_CALIBRATION_SETTLE_MS);
  }
  delay(POST_WIFI_CALIBRATION_SETTLE_MS);
  calibrateGyroAndInitAttitude();

  lastSampleUs = micros();
  nextSampleUs = lastSampleUs;
}

void loop() {
  handleSerialCommands();
  serviceWifiReconnect();

  const uint32_t now = micros();
  if (static_cast<int32_t>(now - nextSampleUs) < 0) {
    idleUntilNextSample(now);
    return;
  }

  const uint32_t samplePeriodUs = 1000000UL / SAMPLE_RATE_HZ;
  if (now - nextSampleUs > samplePeriodUs) {
    ++lateLoops;
    nextSampleUs = now;
  } else {
    nextSampleUs += samplePeriodUs;
  }

  RawImu raw;
  if (!readRawImu(raw)) {
    ++consecutiveI2cErrors;
    if (consecutiveI2cErrors >= MAX_CONSECUTIVE_I2C_ERRORS) {
      recoverMpu();
    }
    return;
  }
  consecutiveI2cErrors = 0;

  const uint32_t tUs = micros();
  const uint32_t rawDtUs = tUs - lastSampleUs;
  lastSampleUs = tUs;
  uint32_t dtUs = rawDtUs;
  if (dtUs == 0 || dtUs > MAX_FILTER_DT_US) {
    dtUs = samplePeriodUs;
    ++lateLoops;
  }
  const float dt = static_cast<float>(dtUs) * 1.0e-6f;

  const Vec3 accel = accelGFromRaw(raw);
  const Vec3 rawGyroDps = gyroDpsFromRaw(raw);
  Vec3 gyro = vecSub(rawGyroDps, gyroBiasDps);
  serviceAutoGyroBias(rawGyroDps, gyro, accel);
  gyro = vecSub(rawGyroDps, gyroBiasDps);

  updateMahony(
    gyro.x * DEG_TO_RAD_F,
    gyro.y * DEG_TO_RAD_F,
    gyro.z * DEG_TO_RAD_F,
    accel,
    dt);

  const Quat relative = quatMultiply(quatConjugate(centerAttitude), attitude);
  const EulerDeg rawEuler = eulerFromQuat(relative);
  const EulerDeg magCorrectedEuler = applyMagYawCorrection(rawEuler, accel, gyro, dt);
  const EulerDeg euler = filterOutputEuler(magCorrectedEuler, dt);

  static uint32_t lastUdpUs = 0;
  const uint32_t udpPeriodUs = 1000000UL / UDP_SEND_RATE_HZ;
  if (tUs - lastUdpUs >= udpPeriodUs) {
    lastUdpUs += udpPeriodUs;
    if (tUs - lastUdpUs >= udpPeriodUs) {
      lastUdpUs = tUs;
    }
    sendOpenTrackPose(euler);
  }

  static uint32_t lastPrintUs = 0;
  const uint32_t printPeriodUs = 1000000UL / PRINT_RATE_HZ;
  if (tUs - lastPrintUs >= printPeriodUs) {
    lastPrintUs = tUs;

    if (SERIAL_PLOTTER_MODE) {
      Serial.print("Yaw:");
      Serial.print(euler.yaw, 2);
      Serial.print('\t');
      Serial.print("Pitch:");
      Serial.print(euler.pitch, 2);
      Serial.print('\t');
      Serial.print("Roll:");
      Serial.println(euler.roll, 2);
      return;
    }

    Serial.print("YPR,");
    Serial.print(tUs);
    Serial.print(',');
    Serial.print(dtUs);
    Serial.print(',');
    Serial.print(euler.yaw, 3);
    Serial.print(',');
    Serial.print(euler.pitch, 3);
    Serial.print(',');
    Serial.print(euler.roll, 3);
    Serial.print(',');
    Serial.print(accel.x, 5);
    Serial.print(',');
    Serial.print(accel.y, 5);
    Serial.print(',');
    Serial.print(accel.z, 5);
    Serial.print(',');
    Serial.print(vecMagnitude(accel), 5);
    Serial.print(',');
    Serial.print(gyro.x, 4);
    Serial.print(',');
    Serial.print(gyro.y, 4);
    Serial.print(',');
    Serial.print(gyro.z, 4);
    Serial.print(',');
    Serial.print(tempCFromRaw(raw), 2);
    Serial.print(',');
    Serial.print(i2cErrors);
    Serial.print(',');
    Serial.print(lateLoops);
    Serial.print(',');
    Serial.print(udpPacketsOk);
    Serial.print(',');
    Serial.print(udpPacketsFail);
    Serial.print(',');
    Serial.print(lastUdpResult);
    Serial.print(',');
    Serial.print(WiFi.isConnected() ? WiFi.RSSI() : 0);
    Serial.print(',');
    Serial.print(wifiReconnects);
    Serial.print(',');
    Serial.print(consecutiveI2cErrors);
    Serial.print(',');
    Serial.print(autoBiasUpdates);
    Serial.print(',');
    Serial.print(stationarySamples);
    Serial.print(',');
    Serial.print(magReadOk);
    Serial.print(',');
    Serial.print(magReadFail);
    Serial.print(',');
    Serial.print(magRejected);
    Serial.print(',');
    Serial.print(magLastFieldUt, 2);
    Serial.print(',');
    Serial.print(magLastYawErrorDeg, 3);
    Serial.print(',');
    Serial.println(magYawCorrectionOffsetDeg, 3);
  }
}
