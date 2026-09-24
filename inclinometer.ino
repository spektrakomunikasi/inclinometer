#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <esp_system.h>

// =========================
// Hardware pin configuration
// =========================
struct PinConfig {
  int i2cSda = 21;
  int i2cScl = 22;
  int tftSck = 18;
  int tftMiso = 19;
  int tftMosi = 23;
  int tftCs = 5;
  int tftDc = 2;
  int tftRst = 4;
};

PinConfig pins;
SPIClass tftSpi(VSPI);
Adafruit_ILI9341 tft(&tftSpi, pins.tftCs, pins.tftDc, pins.tftRst);

Adafruit_MPU6050 mpu6050;
Adafruit_ADXL345_Unified adxl345(12345);
WebServer server(80);
Preferences prefs;

static const char *PREF_NS = "inclino";
static const float DEG_PER_RAD = 57.2957795131f;
static const char *ADMIN_TOKEN_PREFIX = "ADM-";
static const uint8_t ADMIN_TOKEN_RANDOM_BYTES = 32;

// =========================
// Config
// =========================
struct AxisMap {
  int8_t x = 0;
  int8_t y = 1;
  int8_t z = 2;
  int8_t sx = 1;
  int8_t sy = 1;
  int8_t sz = 1;
};

struct Config {
  char apSsid[32] = "SHIP-INCLINOMETER";
  char apPassword[32] = "ShipInclinometer";
  bool startupCalibration = true;

  float rollWarning = 8.0f;
  float rollDanger = 15.0f;
  float pitchWarning = 8.0f;
  float pitchDanger = 15.0f;
  float sensorDiffWarning = 5.0f;

  float accelLpfAlpha = 0.15f;
  float compBaseGain = 0.04f;
  float linearAccelRejectG = 0.25f;

  uint16_t sensorMs = 20;
  uint16_t filterMs = 20;
  uint16_t tftMs = 180;
  uint16_t serialMs = 500;
  uint16_t webMs = 250;

  uint16_t sensorTimeoutMs = 2000;
  uint8_t maxConsecutiveFails = 6;

  AxisMap adxlAxis;
};

Config cfg;
Config cfgDefaults;

// =========================
// Runtime state
// =========================
struct SensorState {
  bool present = false;
  bool healthy = false;
  bool warning = false;
  bool plausible = false;
  uint32_t lastGoodReadMs = 0;
  uint16_t consecutiveFails = 0;

  float ax = 0, ay = 0, az = 0;      // g
  float gx = 0, gy = 0, gz = 0;      // deg/s
  float axLpf = 0, ayLpf = 0, azLpf = 0;
  float roll = 0, pitch = 0;
};

SensorState mpuState;
SensorState adxlState;

struct CalibrationState {
  float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
  float zeroRollRef = 0;
  float zeroPitchRef = 0;
  float adxlRollAlign = 0;
  float adxlPitchAlign = 0;
  float mpuGravityRefX = 0;
  float mpuGravityRefY = 0;
  float mpuGravityRefZ = 1;
  float adxlGravityRefX = 0;
  float adxlGravityRefY = 0;
  float adxlGravityRefZ = 1;
  bool calibrated = false;
};

CalibrationState cal;

enum CalibrationPhase : uint8_t {
  CAL_IDLE = 0,
  CAL_STILLNESS = 1,
  CAL_SAMPLING = 2
};

struct CalibrationJob {
  bool active = false;
  CalibrationPhase phase = CAL_IDLE;
  uint16_t sampleIndex = 0;
  uint32_t nextSampleMs = 0;

  float sumMpuNorm = 0, sumMpuNorm2 = 0;
  float sumMpuGyro = 0, sumMpuGyro2 = 0;
  uint16_t mpuStillN = 0;
  float sumAdxlNorm = 0, sumAdxlNorm2 = 0;
  uint16_t adxlStillN = 0;

  float sumGx = 0, sumGy = 0, sumGz = 0;
  float sumMpuAx = 0, sumMpuAy = 0, sumMpuAz = 0;
  uint16_t mpuCalN = 0;
  float sumAdxlAx = 0, sumAdxlAy = 0, sumAdxlAz = 0;
  uint16_t adxlCalN = 0;
};

CalibrationJob calJob;

struct StatusState {
  bool tftOk = false;
  bool wifiOk = false;
  bool calibrating = false;
  String calibrationMessage = "idle";
  String systemStatus = "BOOT";
  uint32_t bootMs = 0;
  uint32_t dataFrame = 0;
};

StatusState sysState;

String adminToken;
bool showCommissioningSecrets = false;

float fusedRoll = 0.0f;
float fusedPitch = 0.0f;
float outputRollDeg = 0.0f;
float outputPitchDeg = 0.0f;
float accelConfidence = 0.0f;
uint32_t lastSensorMs = 0;
uint32_t lastFilterMs = 0;
uint32_t lastTftMs = 0;
uint32_t lastSerialMs = 0;
uint32_t lastWebFrameMs = 0;
uint32_t lastFilterMicros = 0;

float safeAtan2Deg(float y, float x) {
  float v = atan2f(y, x) * DEG_PER_RAD;
  if (!isfinite(v)) return 0.0f;
  if (v > 180.0f) v = 180.0f;
  if (v < -180.0f) v = -180.0f;
  return v;
}

float adxlRollAligned() {
  return adxlState.roll + cal.adxlRollAlign;
}

float adxlPitchAligned() {
  return adxlState.pitch + cal.adxlPitchAlign;
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

String fmtf(float v, uint8_t p = 2) {
  if (!isfinite(v)) v = 0.0f;
  return String(v, p);
}

void applyAdxlAxisMap(float inX, float inY, float inZ, float &outX, float &outY, float &outZ) {
  const float src[3] = {inX, inY, inZ};
  int xi = constrain(cfg.adxlAxis.x, 0, 2);
  int yi = constrain(cfg.adxlAxis.y, 0, 2);
  int zi = constrain(cfg.adxlAxis.z, 0, 2);
  outX = src[xi] * (cfg.adxlAxis.sx >= 0 ? 1.0f : -1.0f);
  outY = src[yi] * (cfg.adxlAxis.sy >= 0 ? 1.0f : -1.0f);
  outZ = src[zi] * (cfg.adxlAxis.sz >= 0 ? 1.0f : -1.0f);
}

String uptimeString() {
  uint32_t sec = (millis() - sysState.bootMs) / 1000UL;
  uint32_t h = sec / 3600UL;
  uint32_t m = (sec % 3600UL) / 60UL;
  uint32_t s = sec % 60UL;
  char b[20];
  snprintf(b, sizeof(b), "%02lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m, (unsigned long)s);
  return String(b);
}

String rollDirection(float roll) {
  if (roll > 0.5f) return "RIGHT";
  if (roll < -0.5f) return "LEFT";
  return "LEVEL";
}

String pitchDirection(float pitch) {
  if (pitch > 0.5f) return "FORWARD";
  if (pitch < -0.5f) return "BACKWARD";
  return "LEVEL";
}

void updateOutputAngles() {
  outputRollDeg = fusedRoll;
  outputPitchDeg = fusedPitch;
  if (cal.calibrated) {
    outputRollDeg -= cal.zeroRollRef;
    outputPitchDeg -= cal.zeroPitchRef;
  }
  if (!isfinite(outputRollDeg) || fabsf(outputRollDeg) > 180.0f) outputRollDeg = 0.0f;
  if (!isfinite(outputPitchDeg) || fabsf(outputPitchDeg) > 180.0f) outputPitchDeg = 0.0f;
}

bool tokenOkAdminPage() {
  if (!server.hasArg("token")) return false;
  return server.arg("token") == adminToken;
}

bool tokenOkApi() {
  return server.hasHeader("X-Admin-Token") && (server.header("X-Admin-Token") == adminToken);
}

bool originAllowed() {
  if (!server.hasHeader("Origin")) return false;
  String origin = server.header("Origin");
  if (origin.length() == 0) return false;
  String ipOrigin = String("http://") + WiFi.softAPIP().toString();
  if (origin == ipOrigin || origin == ipOrigin + "/" || origin == ipOrigin + ":80" || origin == ipOrigin + ":80/") return true;
  return false;
}

void sanitizeConfig() {
  if (cfg.rollWarning < 0.5f) cfg.rollWarning = 0.5f;
  if (cfg.pitchWarning < 0.5f) cfg.pitchWarning = 0.5f;
  if (cfg.rollDanger < cfg.rollWarning + 0.1f) cfg.rollDanger = cfg.rollWarning + 0.1f;
  if (cfg.pitchDanger < cfg.pitchWarning + 0.1f) cfg.pitchDanger = cfg.pitchWarning + 0.1f;
  cfg.sensorDiffWarning = clampf(cfg.sensorDiffWarning, 0.5f, 45.0f);
  cfg.accelLpfAlpha = clampf(cfg.accelLpfAlpha, 0.01f, 0.95f);
  cfg.compBaseGain = clampf(cfg.compBaseGain, 0.001f, 0.25f);
  cfg.linearAccelRejectG = clampf(cfg.linearAccelRejectG, 0.05f, 1.5f);

  cfg.sensorMs = constrain(cfg.sensorMs, 5, 200);
  cfg.filterMs = constrain(cfg.filterMs, 5, 60);
  cfg.tftMs = constrain(cfg.tftMs, 50, 1000);
  cfg.serialMs = constrain(cfg.serialMs, 100, 3000);
  cfg.webMs = constrain(cfg.webMs, 50, 2000);
  cfg.sensorTimeoutMs = constrain(cfg.sensorTimeoutMs, 300, 10000);
  cfg.maxConsecutiveFails = constrain(cfg.maxConsecutiveFails, 1, 50);

  cfg.adxlAxis.x = constrain(cfg.adxlAxis.x, 0, 2);
  cfg.adxlAxis.y = constrain(cfg.adxlAxis.y, 0, 2);
  cfg.adxlAxis.z = constrain(cfg.adxlAxis.z, 0, 2);
  cfg.adxlAxis.sx = cfg.adxlAxis.sx >= 0 ? 1 : -1;
  cfg.adxlAxis.sy = cfg.adxlAxis.sy >= 0 ? 1 : -1;
  cfg.adxlAxis.sz = cfg.adxlAxis.sz >= 0 ? 1 : -1;

  if (strlen(cfg.apSsid) == 0) strlcpy(cfg.apSsid, cfgDefaults.apSsid, sizeof(cfg.apSsid));
  if (strlen(cfg.apPassword) < 8) strlcpy(cfg.apPassword, cfgDefaults.apPassword, sizeof(cfg.apPassword));
}

void saveConfig() {
  sanitizeConfig();
  prefs.begin(PREF_NS, false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.putBytes("cal", &cal, sizeof(cal));
  prefs.end();
}

void loadConfig() {
  cfgDefaults = cfg;
  prefs.begin(PREF_NS, true);
  if (prefs.getBytesLength("cfg") == sizeof(cfg)) {
    prefs.getBytes("cfg", &cfg, sizeof(cfg));
  }
  if (prefs.getBytesLength("cal") == sizeof(cal)) {
    prefs.getBytes("cal", &cal, sizeof(cal));
  }
  prefs.end();
  sanitizeConfig();
}

void resetDefaults() {
  cfg = cfgDefaults;
  cal = CalibrationState();
  sanitizeConfig();
  saveConfig();
}

void updateHealthState(SensorState &s, bool readOk, bool plausibleNow) {
  if (readOk && plausibleNow) {
    s.lastGoodReadMs = millis();
    s.consecutiveFails = 0;
    s.plausible = true;
  } else {
    if (s.consecutiveFails < 60000) s.consecutiveFails++;
    s.plausible = false;
  }

  bool timeout = (millis() - s.lastGoodReadMs) > cfg.sensorTimeoutMs;
  s.healthy = s.present && !timeout && s.consecutiveFails < cfg.maxConsecutiveFails;
}

bool initSensors() {
  Wire.begin(pins.i2cSda, pins.i2cScl);

  mpuState.present = mpu6050.begin();
  if (mpuState.present) {
    mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuState.healthy = true;
    mpuState.lastGoodReadMs = millis();
  }

  adxlState.present = adxl345.begin();
  if (adxlState.present) {
    adxl345.setRange(ADXL345_RANGE_16_G);
    adxlState.healthy = true;
    adxlState.lastGoodReadMs = millis();
  }

  return mpuState.present || adxlState.present;
}

void initDisplay() {
  tftSpi.begin(pins.tftSck, pins.tftMiso, pins.tftMosi, pins.tftCs);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 8);
  tft.print("SHIP INCLINOMETER");
  sysState.tftOk = true;
}

void initWiFiAP() {
  WiFi.mode(WIFI_AP);
  IPAddress apIP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, gateway, subnet);
  WiFi.softAP(cfg.apSsid, cfg.apPassword);
  sysState.wifiOk = true;
}

String statusLevel() {
  bool sensorError = (!mpuState.present && !adxlState.present) || (!mpuState.healthy && !adxlState.healthy);
  if (sensorError) return "SENSOR_ERROR";

  float ar = fabsf(outputRollDeg);
  float ap = fabsf(outputPitchDeg);
  if (ar >= cfg.rollDanger || ap >= cfg.pitchDanger) return "DANGER_TILT";

  bool tiltWarn = (ar >= cfg.rollWarning || ap >= cfg.pitchWarning);

  float diffR = NAN;
  float diffP = NAN;
  bool bothValid = mpuState.healthy && adxlState.healthy;
  if (bothValid) {
    diffR = fabsf(mpuState.roll - adxlRollAligned());
    diffP = fabsf(mpuState.pitch - adxlPitchAligned());
  }

  float dynamicScale = 1.0f + (1.0f - accelConfidence) * 1.5f;
  bool sensorWarn = bothValid && (diffR > cfg.sensorDiffWarning * dynamicScale || diffP > cfg.sensorDiffWarning * dynamicScale);

  if (sensorWarn) return "SENSOR_WARNING";
  if (!mpuState.healthy || !adxlState.healthy) return "SENSOR_WARNING";
  if (tiltWarn) return "WARNING_TILT";
  return "NORMAL";
}

bool readMPU() {
  if (!mpuState.present) return false;

  sensors_event_t a, g, t;
  mpu6050.getEvent(&a, &g, &t);

  mpuState.ax = a.acceleration.x / 9.80665f;
  mpuState.ay = a.acceleration.y / 9.80665f;
  mpuState.az = a.acceleration.z / 9.80665f;

  mpuState.gx = g.gyro.x * DEG_PER_RAD;
  mpuState.gy = g.gyro.y * DEG_PER_RAD;
  mpuState.gz = g.gyro.z * DEG_PER_RAD;

  if (mpuState.axLpf == 0 && mpuState.ayLpf == 0 && mpuState.azLpf == 0) {
    mpuState.axLpf = mpuState.ax;
    mpuState.ayLpf = mpuState.ay;
    mpuState.azLpf = mpuState.az;
  }

  float aLpf = cfg.accelLpfAlpha;
  mpuState.axLpf += aLpf * (mpuState.ax - mpuState.axLpf);
  mpuState.ayLpf += aLpf * (mpuState.ay - mpuState.ayLpf);
  mpuState.azLpf += aLpf * (mpuState.az - mpuState.azLpf);

  float norm = sqrtf(mpuState.ax * mpuState.ax + mpuState.ay * mpuState.ay + mpuState.az * mpuState.az);
  bool plausible = isfinite(norm) && isfinite(mpuState.gx) && isfinite(mpuState.gy) && isfinite(mpuState.gz) && norm > 0.2f && norm < 4.0f;
  updateHealthState(mpuState, true, plausible);
  return plausible;
}

bool readADXL() {
  if (!adxlState.present) return false;
  sensors_event_t event;
  adxl345.getEvent(&event);

  float x = event.acceleration.x / 9.80665f;
  float y = event.acceleration.y / 9.80665f;
  float z = event.acceleration.z / 9.80665f;
  applyAdxlAxisMap(x, y, z, adxlState.ax, adxlState.ay, adxlState.az);

  if (adxlState.axLpf == 0 && adxlState.ayLpf == 0 && adxlState.azLpf == 0) {
    adxlState.axLpf = adxlState.ax;
    adxlState.ayLpf = adxlState.ay;
    adxlState.azLpf = adxlState.az;
  }

  float aLpf = cfg.accelLpfAlpha;
  adxlState.axLpf += aLpf * (adxlState.ax - adxlState.axLpf);
  adxlState.ayLpf += aLpf * (adxlState.ay - adxlState.ayLpf);
  adxlState.azLpf += aLpf * (adxlState.az - adxlState.azLpf);

  float norm = sqrtf(adxlState.ax * adxlState.ax + adxlState.ay * adxlState.ay + adxlState.az * adxlState.az);
  bool plausible = isfinite(norm) && norm > 0.2f && norm < 4.0f;
  updateHealthState(adxlState, true, plausible);
  return plausible;
}

void computeAnglesFromAccel(SensorState &s) {
  s.roll = safeAtan2Deg(s.ayLpf, s.azLpf);
  float denom = sqrtf(s.ayLpf * s.ayLpf + s.azLpf * s.azLpf);
  if (denom < 0.0001f) denom = 0.0001f;
  s.pitch = safeAtan2Deg(-s.axLpf, denom);
}

void clearCalibrationJob() {
  calJob = CalibrationJob();
}

bool beginCalibrationAsync() {
  clearCalibrationJob();
  if (!mpuState.present && !adxlState.present) {
    sysState.calibrating = false;
    sysState.calibrationMessage = "failed_no_sensor";
    return false;
  }
  sysState.calibrating = true;
  sysState.calibrationMessage = "checking_stillness";
  calJob.active = true;
  calJob.phase = CAL_STILLNESS;
  calJob.nextSampleMs = millis();
  return true;
}

bool evaluateStillnessAndAdvance() {
  bool ok = true;
  if (calJob.mpuStillN > 6) {
    float meanNorm = calJob.sumMpuNorm / calJob.mpuStillN;
    float varNorm = fabsf((calJob.sumMpuNorm2 / calJob.mpuStillN) - (meanNorm * meanNorm));
    float meanGyro = calJob.sumMpuGyro / calJob.mpuStillN;
    float varGyro = fabsf((calJob.sumMpuGyro2 / calJob.mpuStillN) - (meanGyro * meanGyro));
    float stdNorm = sqrtf(varNorm);
    float rmsGyro = sqrtf(varGyro + meanGyro * meanGyro);
    ok &= (fabsf(meanNorm - 1.0f) < 0.15f) && (stdNorm < 0.03f) && (rmsGyro < 1.5f);
  }
  if (calJob.adxlStillN > 6) {
    float meanNorm = calJob.sumAdxlNorm / calJob.adxlStillN;
    float varNorm = fabsf((calJob.sumAdxlNorm2 / calJob.adxlStillN) - (meanNorm * meanNorm));
    float stdNorm = sqrtf(varNorm);
    ok &= (fabsf(meanNorm - 1.0f) < 0.20f) && (stdNorm < 0.035f);
  }
  if (!ok) {
    sysState.calibrationMessage = "failed_motion_detected";
    sysState.calibrating = false;
    clearCalibrationJob();
    return false;
  }

  calJob.phase = CAL_SAMPLING;
  calJob.sampleIndex = 0;
  calJob.nextSampleMs = millis();
  sysState.calibrationMessage = "sampling_bias";
  return true;
}

void finalizeCalibration() {
  if (calJob.mpuCalN > 0) {
    cal.gyroBiasX = calJob.sumGx / calJob.mpuCalN;
    cal.gyroBiasY = calJob.sumGy / calJob.mpuCalN;
    cal.gyroBiasZ = calJob.sumGz / calJob.mpuCalN;

    cal.mpuGravityRefX = calJob.sumMpuAx / calJob.mpuCalN;
    cal.mpuGravityRefY = calJob.sumMpuAy / calJob.mpuCalN;
    cal.mpuGravityRefZ = calJob.sumMpuAz / calJob.mpuCalN;

    float rr = safeAtan2Deg(cal.mpuGravityRefY, cal.mpuGravityRefZ);
    float denom = sqrtf(cal.mpuGravityRefY * cal.mpuGravityRefY + cal.mpuGravityRefZ * cal.mpuGravityRefZ);
    if (denom < 0.0001f) denom = 0.0001f;
    float pp = safeAtan2Deg(-cal.mpuGravityRefX, denom);
    cal.zeroRollRef = rr;
    cal.zeroPitchRef = pp;
  }

  if (calJob.adxlCalN > 0) {
    cal.adxlGravityRefX = calJob.sumAdxlAx / calJob.adxlCalN;
    cal.adxlGravityRefY = calJob.sumAdxlAy / calJob.adxlCalN;
    cal.adxlGravityRefZ = calJob.sumAdxlAz / calJob.adxlCalN;
  }

  cal.adxlRollAlign = 0;
  cal.adxlPitchAlign = 0;
  if (calJob.mpuCalN > 0 && calJob.adxlCalN > 0) {
    float adxlRoll = safeAtan2Deg(cal.adxlGravityRefY, cal.adxlGravityRefZ);
    float denom = sqrtf(cal.adxlGravityRefY * cal.adxlGravityRefY + cal.adxlGravityRefZ * cal.adxlGravityRefZ);
    if (denom < 0.0001f) denom = 0.0001f;
    float adxlPitch = safeAtan2Deg(-cal.adxlGravityRefX, denom);

    float mpuRoll = safeAtan2Deg(cal.mpuGravityRefY, cal.mpuGravityRefZ);
    denom = sqrtf(cal.mpuGravityRefY * cal.mpuGravityRefY + cal.mpuGravityRefZ * cal.mpuGravityRefZ);
    if (denom < 0.0001f) denom = 0.0001f;
    float mpuPitch = safeAtan2Deg(-cal.mpuGravityRefX, denom);

    cal.adxlRollAlign = mpuRoll - adxlRoll;
    cal.adxlPitchAlign = mpuPitch - adxlPitch;
  }

  cal.calibrated = true;
  saveConfig();
  updateOutputAngles();
  sysState.calibrationMessage = "ok";
  sysState.calibrating = false;
  clearCalibrationJob();
}

void processCalibrationTask() {
  if (!calJob.active) return;
  if (millis() < calJob.nextSampleMs) return;

  if (calJob.phase == CAL_STILLNESS) {
    if (mpuState.present) {
      sensors_event_t a, g, t;
      mpu6050.getEvent(&a, &g, &t);
      float ax = a.acceleration.x / 9.80665f;
      float ay = a.acceleration.y / 9.80665f;
      float az = a.acceleration.z / 9.80665f;
      float norm = sqrtf(ax * ax + ay * ay + az * az);
      float gyroMag = sqrtf(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z) * DEG_PER_RAD;
      if (isfinite(norm) && isfinite(gyroMag)) {
        calJob.sumMpuNorm += norm;
        calJob.sumMpuNorm2 += norm * norm;
        calJob.sumMpuGyro += gyroMag;
        calJob.sumMpuGyro2 += gyroMag * gyroMag;
        calJob.mpuStillN++;
      }
    }
    if (adxlState.present) {
      sensors_event_t aev;
      adxl345.getEvent(&aev);
      float ax = aev.acceleration.x / 9.80665f;
      float ay = aev.acceleration.y / 9.80665f;
      float az = aev.acceleration.z / 9.80665f;
      applyAdxlAxisMap(ax, ay, az, ax, ay, az);
      float norm = sqrtf(ax * ax + ay * ay + az * az);
      if (isfinite(norm)) {
        calJob.sumAdxlNorm += norm;
        calJob.sumAdxlNorm2 += norm * norm;
        calJob.adxlStillN++;
      }
    }
    calJob.sampleIndex++;
    calJob.nextSampleMs = millis() + 10;
    if (calJob.sampleIndex >= 120) {
      evaluateStillnessAndAdvance();
    }
    return;
  }

  if (calJob.phase == CAL_SAMPLING) {
    if (mpuState.present) {
      sensors_event_t a, g, t;
      mpu6050.getEvent(&a, &g, &t);
      float ax = a.acceleration.x / 9.80665f;
      float ay = a.acceleration.y / 9.80665f;
      float az = a.acceleration.z / 9.80665f;
      float gx = g.gyro.x * DEG_PER_RAD;
      float gy = g.gyro.y * DEG_PER_RAD;
      float gz = g.gyro.z * DEG_PER_RAD;
      if (isfinite(ax) && isfinite(ay) && isfinite(az) && isfinite(gx) && isfinite(gy) && isfinite(gz)) {
        calJob.sumMpuAx += ax;
        calJob.sumMpuAy += ay;
        calJob.sumMpuAz += az;
        calJob.sumGx += gx;
        calJob.sumGy += gy;
        calJob.sumGz += gz;
        calJob.mpuCalN++;
      }
    }
    if (adxlState.present) {
      sensors_event_t aev;
      adxl345.getEvent(&aev);
      float ax = aev.acceleration.x / 9.80665f;
      float ay = aev.acceleration.y / 9.80665f;
      float az = aev.acceleration.z / 9.80665f;
      applyAdxlAxisMap(ax, ay, az, ax, ay, az);
      if (isfinite(ax) && isfinite(ay) && isfinite(az)) {
        calJob.sumAdxlAx += ax;
        calJob.sumAdxlAy += ay;
        calJob.sumAdxlAz += az;
        calJob.adxlCalN++;
      }
    }
    calJob.sampleIndex++;
    calJob.nextSampleMs = millis() + 6;
    if (calJob.sampleIndex >= 250) {
      finalizeCalibration();
    }
    return;
  }
}

void readSensorsTask() {
  readMPU();
  readADXL();

  if (mpuState.healthy) computeAnglesFromAccel(mpuState);
  if (adxlState.healthy) {
    computeAnglesFromAccel(adxlState);
  }
}

void filterTask() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastFilterMicros) / 1000000.0f;
  lastFilterMicros = nowUs;
  dt = clampf(dt, 0.001f, 0.06f);

  float accelRoll = fusedRoll;
  float accelPitch = fusedPitch;
  bool hasAccel = false;

  if (mpuState.healthy) {
    accelRoll = mpuState.roll;
    accelPitch = mpuState.pitch;
    hasAccel = true;
  } else if (adxlState.healthy) {
    accelRoll = adxlRollAligned();
    accelPitch = adxlPitchAligned();
    hasAccel = true;
  }

  if (!isfinite(fusedRoll) || !isfinite(fusedPitch)) {
    fusedRoll = accelRoll;
    fusedPitch = accelPitch;
  }

  if (mpuState.healthy) {
    fusedRoll += (mpuState.gx - cal.gyroBiasX) * dt;
    fusedPitch += (mpuState.gy - cal.gyroBiasY) * dt;
  }

  float norm = 1.0f;
  bool accelSourceValid = false;
  if (mpuState.healthy) {
    norm = sqrtf(mpuState.axLpf * mpuState.axLpf + mpuState.ayLpf * mpuState.ayLpf + mpuState.azLpf * mpuState.azLpf);
    accelSourceValid = true;
  } else if (adxlState.healthy) {
    norm = sqrtf(adxlState.axLpf * adxlState.axLpf + adxlState.ayLpf * adxlState.ayLpf + adxlState.azLpf * adxlState.azLpf);
    accelSourceValid = true;
  }

  if (!accelSourceValid || !isfinite(norm)) {
    accelConfidence = 0.0f;
  } else {
    float linearDev = fabsf(norm - 1.0f);
    accelConfidence = 1.0f - clampf(linearDev / cfg.linearAccelRejectG, 0.0f, 1.0f);
  }

  if (hasAccel && accelConfidence > 0.02f) {
    float corrGain = cfg.compBaseGain * accelConfidence;
    fusedRoll += corrGain * (accelRoll - fusedRoll);
    fusedPitch += corrGain * (accelPitch - fusedPitch);
  }

  if (!isfinite(fusedRoll) || fabsf(fusedRoll) > 360.0f) fusedRoll = accelRoll;
  if (!isfinite(fusedPitch) || fabsf(fusedPitch) > 360.0f) fusedPitch = accelPitch;

  updateOutputAngles();
  sysState.systemStatus = statusLevel();
}

uint16_t statusColor(const String &st) {
  if (st == "DANGER_TILT" || st == "SENSOR_ERROR") return ILI9341_RED;
  if (st == "WARNING_TILT" || st == "SENSOR_WARNING") return ILI9341_YELLOW;
  return ILI9341_GREEN;
}

void updateTftTask() {
  if (!sysState.tftOk) return;

  tft.fillRect(0, 36, 320, 204, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(8, 40);
  tft.print("ROLL : ");
  tft.print(fmtf(outputRollDeg));
  tft.print((char)247);
  tft.setCursor(8, 62);
  tft.print("PITCH: ");
  tft.print(fmtf(outputPitchDeg));
  tft.print((char)247);

  tft.setCursor(8, 86);
  tft.print("R DIR: ");
  tft.print(rollDirection(outputRollDeg));
  tft.setCursor(8, 108);
  tft.print("P DIR: ");
  tft.print(pitchDirection(outputPitchDeg));

  tft.setCursor(8, 136);
  tft.print("MPU  ");
  tft.print(mpuState.healthy ? "OK" : (mpuState.present ? "WARN" : "ERR"));
  tft.setCursor(8, 158);
  tft.print("ADXL ");
  tft.print(adxlState.healthy ? "OK" : (adxlState.present ? "WARN" : "ERR"));

  tft.fillRect(8, 188, 304, 40, statusColor(sysState.systemStatus));
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(12, 200);
  tft.print(sysState.systemStatus);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
}

void serialTask() {
  Serial.println(F("----- Inclinometer Telemetry -----"));
  Serial.printf("ROLL: %.2f deg (%s) | PITCH: %.2f deg (%s)\n", outputRollDeg, rollDirection(outputRollDeg).c_str(), outputPitchDeg, pitchDirection(outputPitchDeg).c_str());

  Serial.printf("MPU6050 present=%d healthy=%d acc[g]=(%.3f,%.3f,%.3f) gyro[dps]=(%.3f,%.3f,%.3f) roll=%.2f pitch=%.2f\n",
                mpuState.present, mpuState.healthy, mpuState.ax, mpuState.ay, mpuState.az, mpuState.gx, mpuState.gy, mpuState.gz, mpuState.roll, mpuState.pitch);
  Serial.printf("ADXL345 present=%d healthy=%d acc[g]=(%.3f,%.3f,%.3f) roll=%.2f pitch=%.2f\n",
                adxlState.present, adxlState.healthy, adxlState.ax, adxlState.ay, adxlState.az, adxlRollAligned(), adxlPitchAligned());

  float diffR = NAN;
  float diffP = NAN;
  if (mpuState.healthy && adxlState.healthy) {
    diffR = fabsf(mpuState.roll - adxlRollAligned());
    diffP = fabsf(mpuState.pitch - adxlPitchAligned());
  }
  Serial.printf("DIFF roll=%.2f pitch=%.2f accel_conf=%.2f status=%s uptime=%s\n", diffR, diffP, accelConfidence, sysState.systemStatus.c_str(), uptimeString().c_str());
}

String basePageHtml(bool adminMode) {
  String h;
  h.reserve(9000);
  h += F(
      "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Ship Inclinometer</title><style>"
      "body{margin:0;background:#0b1220;color:#e6edf7;font-family:Arial,sans-serif}"
      ".top{display:flex;justify-content:space-between;align-items:center;padding:12px 16px;background:#121b2e;position:sticky;top:0}"
      ".ok{color:#24d16f}.warn{color:#ffd34f}.danger{color:#ff6f6f}"
      ".wrap{padding:14px;display:grid;gap:10px;grid-template-columns:repeat(auto-fit,minmax(250px,1fr))}"
      ".card{background:#121b2e;border-radius:10px;padding:12px;border:1px solid #203052}"
      ".big{font-size:36px;font-weight:700}.label{font-size:12px;color:#8ea6ce}"
      "meter{width:100%;height:12px}button,input,select{font-size:14px;padding:8px;border-radius:8px;border:1px solid #32466e;background:#0e1628;color:#fff}"
      "table{width:100%;font-size:13px}td{padding:4px 2px;border-bottom:1px solid #22365a}"
      "</style></head><body>"
      "<div class='top'><div><b>SHIP INCLINOMETER</b><div id='upd' class='label'></div></div><div id='statusDot' class='ok'>ONLINE</div></div>"
      "<div class='wrap'>"
      "<div class='card'><div class='label'>ROLL</div><div id='rollVal' class='big'>0.00°</div><div id='rollDir'>LEVEL</div><meter id='rollGauge' min='-45' max='45' value='0'></meter></div>"
      "<div class='card'><div class='label'>PITCH</div><div id='pitchVal' class='big'>0.00°</div><div id='pitchDir'>LEVEL</div><meter id='pitchGauge' min='-45' max='45' value='0'></meter></div>"
      "<div class='card'><h3>MPU6050</h3><table>"
      "<tr><td>ACC X/Y/Z</td><td id='mpuAcc'></td></tr><tr><td>GYRO X/Y/Z</td><td id='mpuGyro'></td></tr>"
      "<tr><td>ROLL/PITCH</td><td id='mpuAng'></td></tr><tr><td>Status</td><td id='mpuStatus'></td></tr></table></div>"
      "<div class='card'><h3>ADXL345</h3><table>"
      "<tr><td>ACC X/Y/Z</td><td id='adxlAcc'></td></tr><tr><td>ROLL/PITCH</td><td id='adxlAng'></td></tr>"
      "<tr><td>Status</td><td id='adxlStatus'></td></tr><tr><td>Alignment</td><td id='alignInfo'></td></tr></table></div>"
      "<div class='card'><h3>SENSOR DIFFERENCE</h3><table><tr><td>Roll diff</td><td id='diffRoll'></td></tr><tr><td>Pitch diff</td><td id='diffPitch'></td></tr><tr><td>Accel confidence</td><td id='accelConf'></td></tr></table></div>"
      "<div class='card'><h3>SYSTEM / NETWORK</h3><table><tr><td>Status</td><td id='statusTxt'></td></tr><tr><td>Calibration</td><td id='calibTxt'></td></tr><tr><td>SSID</td><td id='ssidTxt'></td></tr><tr><td>IP</td><td id='ipTxt'></td></tr><tr><td>Uptime</td><td id='uptimeTxt'></td></tr><tr><td>Clients</td><td id='clientsTxt'></td></tr></table></div>");

  if (adminMode) {
    h += F("<div class='card' style='grid-column:1/-1'><h3>ADMIN SETTINGS</h3><div class='label'>Token authenticated</div><table>"
           "<tr><td>Roll warning</td><td><input id='rollWarning'></td><td>Roll danger</td><td><input id='rollDanger'></td></tr>"
           "<tr><td>Pitch warning</td><td><input id='pitchWarning'></td><td>Pitch danger</td><td><input id='pitchDanger'></td></tr>"
           "<tr><td>Sensor diff warn</td><td><input id='sensorDiffWarning'></td><td>Comp gain</td><td><input id='compBaseGain'></td></tr>"
           "<tr><td>Accel LPF alpha</td><td><input id='accelLpfAlpha'></td><td>Reject window (g)</td><td><input id='linearAccelRejectG'></td></tr>"
           "<tr><td>ADXL axis x/y/z</td><td><input id='adxlAxis' placeholder='0,1,2'></td><td>sign sx/sy/sz</td><td><input id='adxlSign' placeholder='1,1,1'></td></tr></table>"
           "<p><button onclick='saveSettings()'>SAVE SETTINGS</button> <button onclick='recalibrate()'>CALIBRATE ZERO</button> <button onclick='resetDefaults()'>RESET DEFAULT</button></p>"
           "<div id='adminMsg' class='label'></div></div>");
  } else {
    h += F("<div class='card' style='grid-column:1/-1'><h3>ADMIN</h3><div class='label'>Settings panel is protected. Use /admin?token=YOUR_TOKEN</div></div>");
  }

  h += F("</div><script>"
         "const admin=" );
  h += adminMode ? "true" : "false";
  h += F(";const token=(new URLSearchParams(window.location.search)).get('token')||'';"
         "const q=(i)=>document.getElementById(i);"
         "const fmt=(v,p=2)=>Number.isFinite(v)?v.toFixed(p):'n/a';"
         "async function updateData(){try{const r=await fetch('/api/data');const d=await r.json();"
         "q('rollVal').textContent=`${fmt(d.roll)}°`;q('pitchVal').textContent=`${fmt(d.pitch)}°`;"
         "q('rollDir').textContent=d.roll_direction;q('pitchDir').textContent=d.pitch_direction;"
         "q('rollGauge').value=Math.max(-45,Math.min(45,d.roll));q('pitchGauge').value=Math.max(-45,Math.min(45,d.pitch));"
         "q('mpuAcc').textContent=`${fmt(d.mpu.ax,3)}, ${fmt(d.mpu.ay,3)}, ${fmt(d.mpu.az,3)}`;"
         "q('mpuGyro').textContent=`${fmt(d.mpu.gx,3)}, ${fmt(d.mpu.gy,3)}, ${fmt(d.mpu.gz,3)}`;"
         "q('mpuAng').textContent=`${fmt(d.mpu.roll)} / ${fmt(d.mpu.pitch)}`;q('mpuStatus').textContent=d.mpu.status;"
         "q('adxlAcc').textContent=`${fmt(d.adxl.ax,3)}, ${fmt(d.adxl.ay,3)}, ${fmt(d.adxl.az,3)}`;"
         "q('adxlAng').textContent=`${fmt(d.adxl.roll)} / ${fmt(d.adxl.pitch)}`;q('adxlStatus').textContent=d.adxl.status;"
         "q('alignInfo').textContent=`R ${fmt(d.adxl_align.roll)}°, P ${fmt(d.adxl_align.pitch)}°`;"
         "q('diffRoll').textContent=fmt(d.diff.roll);q('diffPitch').textContent=fmt(d.diff.pitch);q('accelConf').textContent=fmt(d.accel_confidence,2);"
         "q('statusTxt').textContent=d.system.status;q('calibTxt').textContent=d.system.calibration;"
         "q('ssidTxt').textContent=d.network.ssid;q('ipTxt').textContent=d.network.ip;"
         "q('uptimeTxt').textContent=d.system.uptime;q('clientsTxt').textContent=d.network.clients;"
         "q('upd').textContent='Updated '+new Date(d.system.updated_ms).toLocaleTimeString();"
         "const dot=q('statusDot');dot.className=(d.system.status.includes('DANGER')||d.system.status.includes('ERROR'))?'danger':(d.system.status.includes('WARNING')?'warn':'ok');"
         "dot.textContent=d.system.status;}catch(e){q('upd').textContent='update failed';}}"
         "async function loadSettings(){if(!admin||!token) return;const r=await fetch('/api/settings',{headers:{'X-Admin-Token':token}});if(!r.ok) return;const d=await r.json();"
         "for(const k of ['rollWarning','rollDanger','pitchWarning','pitchDanger','sensorDiffWarning','compBaseGain','accelLpfAlpha','linearAccelRejectG']){if(q(k)) q(k).value=d[k];}"
         "q('adxlAxis').value=[d.adxlAxis.x,d.adxlAxis.y,d.adxlAxis.z].join(',');q('adxlSign').value=[d.adxlAxis.sx,d.adxlAxis.sy,d.adxlAxis.sz].join(',');}"
         "async function saveSettings(){if(!admin) return;const p=new URLSearchParams({token,rollWarning:q('rollWarning').value,rollDanger:q('rollDanger').value,pitchWarning:q('pitchWarning').value,pitchDanger:q('pitchDanger').value,sensorDiffWarning:q('sensorDiffWarning').value,compBaseGain:q('compBaseGain').value,accelLpfAlpha:q('accelLpfAlpha').value,linearAccelRejectG:q('linearAccelRejectG').value,adxlAxis:q('adxlAxis').value,adxlSign:q('adxlSign').value});"
         "const r=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Admin-Token':token},body:p});q('adminMsg').textContent=await r.text();}"
         "async function recalibrate(){if(!admin||!token) return;q('adminMsg').textContent='Calibration in progress... keep vessel still';const r=await fetch('/api/calibrate',{method:'POST',headers:{'X-Admin-Token':token}});q('adminMsg').textContent=await r.text();}"
         "async function resetDefaults(){if(!admin||!token) return;const r=await fetch('/api/reset',{method:'POST',headers:{'X-Admin-Token':token}});q('adminMsg').textContent=await r.text();loadSettings();}"
         "updateData();setInterval(updateData," );
  h += String(cfg.webMs);
  h += F(");loadSettings();</script></body></html>");
  return h;
}

String sensorStatusString(const SensorState &s) {
  if (!s.present) return "ERROR";
  if (!s.healthy) return "WARNING";
  return "OK";
}

String buildDataJson() {
  float diffRoll = NAN;
  float diffPitch = NAN;
  if (mpuState.healthy && adxlState.healthy) {
    diffRoll = fabsf(mpuState.roll - adxlRollAligned());
    diffPitch = fabsf(mpuState.pitch - adxlPitchAligned());
  }

  String j;
  j.reserve(1400);
  j = "{";
  j += "\"roll\":" + fmtf(outputRollDeg, 3);
  j += ",\"pitch\":" + fmtf(outputPitchDeg, 3);
  j += ",\"roll_direction\":\"" + rollDirection(outputRollDeg) + "\"";
  j += ",\"pitch_direction\":\"" + pitchDirection(outputPitchDeg) + "\"";

  j += ",\"mpu\":{";
  j += "\"ax\":" + fmtf(mpuState.ax, 4) + ",\"ay\":" + fmtf(mpuState.ay, 4) + ",\"az\":" + fmtf(mpuState.az, 4);
  j += ",\"gx\":" + fmtf(mpuState.gx, 4) + ",\"gy\":" + fmtf(mpuState.gy, 4) + ",\"gz\":" + fmtf(mpuState.gz, 4);
  j += ",\"roll\":" + fmtf(mpuState.roll, 3) + ",\"pitch\":" + fmtf(mpuState.pitch, 3);
  j += ",\"status\":\"" + sensorStatusString(mpuState) + "\"}";

  j += ",\"adxl\":{";
  j += "\"ax\":" + fmtf(adxlState.ax, 4) + ",\"ay\":" + fmtf(adxlState.ay, 4) + ",\"az\":" + fmtf(adxlState.az, 4);
  j += ",\"roll\":" + fmtf(adxlRollAligned(), 3) + ",\"pitch\":" + fmtf(adxlPitchAligned(), 3);
  j += ",\"status\":\"" + sensorStatusString(adxlState) + "\"}";

  j += ",\"diff\":{\"roll\":" + fmtf(diffRoll, 3) + ",\"pitch\":" + fmtf(diffPitch, 3) + "}";
  j += ",\"adxl_align\":{\"roll\":" + fmtf(cal.adxlRollAlign, 3) + ",\"pitch\":" + fmtf(cal.adxlPitchAlign, 3) + "}";
  j += ",\"accel_confidence\":" + fmtf(accelConfidence, 3);

  j += ",\"system\":{";
  j += "\"status\":\"" + sysState.systemStatus + "\"";
  j += ",\"calibration\":\"" + sysState.calibrationMessage + "\"";
  j += ",\"uptime\":\"" + uptimeString() + "\"";
  j += ",\"updated_ms\":" + String(millis()) + "}";

  j += ",\"network\":{";
  j += "\"ssid\":\"" + String(cfg.apSsid) + "\"";
  j += ",\"ip\":\"" + WiFi.softAPIP().toString() + "\"";
  j += ",\"clients\":" + String(WiFi.softAPgetStationNum()) + "}";

  j += "}";
  return j;
}

void handleRoot() {
  server.send(200, "text/html", basePageHtml(false));
}

void handleAdmin() {
  if (!tokenOkAdminPage()) {
    server.send(401, "text/plain", "Admin token required: /admin?token=...");
    return;
  }
  server.send(200, "text/html", basePageHtml(true));
}

void handleData() {
  server.send(200, "application/json", buildDataJson());
}

void handleSettingsGet() {
  if (!originAllowed()) {
    server.send(403, "application/json", "{\"error\":\"origin_forbidden\"}");
    return;
  }
  if (!tokenOkApi()) {
    server.send(401, "application/json", "{\"error\":\"unauthorized\"}");
    return;
  }

  String j = "{";
  j += "\"rollWarning\":" + fmtf(cfg.rollWarning, 3);
  j += ",\"rollDanger\":" + fmtf(cfg.rollDanger, 3);
  j += ",\"pitchWarning\":" + fmtf(cfg.pitchWarning, 3);
  j += ",\"pitchDanger\":" + fmtf(cfg.pitchDanger, 3);
  j += ",\"sensorDiffWarning\":" + fmtf(cfg.sensorDiffWarning, 3);
  j += ",\"compBaseGain\":" + fmtf(cfg.compBaseGain, 4);
  j += ",\"accelLpfAlpha\":" + fmtf(cfg.accelLpfAlpha, 4);
  j += ",\"linearAccelRejectG\":" + fmtf(cfg.linearAccelRejectG, 4);
  j += ",\"adxlAxis\":{\"x\":" + String(cfg.adxlAxis.x) + ",\"y\":" + String(cfg.adxlAxis.y) + ",\"z\":" + String(cfg.adxlAxis.z) + ",\"sx\":" + String(cfg.adxlAxis.sx) + ",\"sy\":" + String(cfg.adxlAxis.sy) + ",\"sz\":" + String(cfg.adxlAxis.sz) + "}";
  j += "}";
  server.send(200, "application/json", j);
}

float argFloat(const char *k, float fallback) {
  if (!server.hasArg(k)) return fallback;
  return server.arg(k).toFloat();
}

void parseAxisArgs() {
  if (server.hasArg("adxlAxis")) {
    String a = server.arg("adxlAxis");
    int p1 = a.indexOf(',');
    int p2 = a.indexOf(',', p1 + 1);
    if (p1 > 0 && p2 > p1) {
      cfg.adxlAxis.x = a.substring(0, p1).toInt();
      cfg.adxlAxis.y = a.substring(p1 + 1, p2).toInt();
      cfg.adxlAxis.z = a.substring(p2 + 1).toInt();
    }
  }
  if (server.hasArg("adxlSign")) {
    String a = server.arg("adxlSign");
    int p1 = a.indexOf(',');
    int p2 = a.indexOf(',', p1 + 1);
    if (p1 > 0 && p2 > p1) {
      cfg.adxlAxis.sx = a.substring(0, p1).toInt();
      cfg.adxlAxis.sy = a.substring(p1 + 1, p2).toInt();
      cfg.adxlAxis.sz = a.substring(p2 + 1).toInt();
    }
  }
}

void handleSettingsPost() {
  if (!originAllowed()) {
    server.send(403, "text/plain", "origin forbidden");
    return;
  }
  if (!tokenOkApi()) {
    server.send(401, "text/plain", "unauthorized");
    return;
  }

  cfg.rollWarning = argFloat("rollWarning", cfg.rollWarning);
  cfg.rollDanger = argFloat("rollDanger", cfg.rollDanger);
  cfg.pitchWarning = argFloat("pitchWarning", cfg.pitchWarning);
  cfg.pitchDanger = argFloat("pitchDanger", cfg.pitchDanger);
  cfg.sensorDiffWarning = argFloat("sensorDiffWarning", cfg.sensorDiffWarning);
  cfg.compBaseGain = argFloat("compBaseGain", cfg.compBaseGain);
  cfg.accelLpfAlpha = argFloat("accelLpfAlpha", cfg.accelLpfAlpha);
  cfg.linearAccelRejectG = argFloat("linearAccelRejectG", cfg.linearAccelRejectG);
  parseAxisArgs();
  sanitizeConfig();
  saveConfig();

  server.send(200, "text/plain", "settings saved");
}

void handleCalibrate() {
  if (!originAllowed()) {
    server.send(403, "text/plain", "origin forbidden");
    return;
  }
  if (!tokenOkApi()) {
    server.send(401, "text/plain", "unauthorized");
    return;
  }
  if (calJob.active) {
    server.send(202, "text/plain", "calibration already in progress");
    return;
  }
  if (!beginCalibrationAsync()) {
    server.send(409, "text/plain", "calibration failed: no active sensor");
    return;
  }
  server.send(202, "text/plain", "calibration started");
}

void handleReset() {
  if (!originAllowed()) {
    server.send(403, "text/plain", "origin forbidden");
    return;
  }
  if (!tokenOkApi()) {
    server.send(401, "text/plain", "unauthorized");
    return;
  }
  resetDefaults();
  server.send(200, "text/plain", "defaults restored");
}

void initWebServer() {
  const char *headerKeys[] = {"X-Admin-Token", "Origin"};
  server.collectHeaders(headerKeys, 2);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/admin", HTTP_GET, handleAdmin);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/settings", HTTP_GET, handleSettingsGet);
  server.on("/api/settings", HTTP_POST, handleSettingsPost);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/api/reset", HTTP_POST, handleReset);

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });

  server.begin();
}

void printBootInfo() {
  Serial.println(F("=== SHIP INCLINOMETER BOOT ==="));
  Serial.printf("I2C pins SDA=%d SCL=%d\n", pins.i2cSda, pins.i2cScl);
  Serial.printf("TFT SPI pins SCK=%d MISO=%d MOSI=%d CS=%d DC=%d RST=%d\n", pins.tftSck, pins.tftMiso, pins.tftMosi, pins.tftCs, pins.tftDc, pins.tftRst);
  Serial.printf("MPU6050: %s | ADXL345: %s | TFT: %s\n", mpuState.present ? "detected" : "missing", adxlState.present ? "detected" : "missing", sysState.tftOk ? "ok" : "error");
  Serial.printf("AP SSID: %s\n", cfg.apSsid);
  Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
  if (showCommissioningSecrets) {
    Serial.println(F("Commissioning credentials (shown once):"));
    Serial.printf("AP PASS: %s\n", cfg.apPassword);
    Serial.printf("Admin URL: http://%s/admin?token=%s\n", WiFi.softAPIP().toString().c_str(), adminToken.c_str());
    prefs.begin(PREF_NS, false);
    prefs.putBool("commissioned", true);
    prefs.end();
  } else {
    Serial.println(F("Commissioning credentials hidden after first boot. Reset NVS to show again."));
  }
}

String randomToken() {
  const size_t hexLen = ADMIN_TOKEN_RANDOM_BYTES * 2;
  char b[(ADMIN_TOKEN_RANDOM_BYTES * 2) + 1];
  for (uint8_t i = 0; i < ADMIN_TOKEN_RANDOM_BYTES; i++) {
    uint8_t r = (uint8_t)(esp_random() & 0xFF);
    snprintf(&b[i * 2], 3, "%02X", r);
  }
  b[hexLen] = '\0';
  return String(ADMIN_TOKEN_PREFIX) + String(b);
}

bool isValidAdminTokenFormat(const String &v) {
  const size_t expectedLen = strlen(ADMIN_TOKEN_PREFIX) + (ADMIN_TOKEN_RANDOM_BYTES * 2);
  if (!v.startsWith(ADMIN_TOKEN_PREFIX)) return false;
  if (v.length() != expectedLen) return false;
  for (size_t i = strlen(ADMIN_TOKEN_PREFIX); i < v.length(); i++) {
    char c = v[i];
    bool hexDigit = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F');
    if (!hexDigit) return false;
  }
  return true;
}

void loadOrCreateAdminToken() {
  prefs.begin(PREF_NS, false);
  String stored = prefs.getString("adminToken", "");
  if (!isValidAdminTokenFormat(stored)) {
    stored = randomToken();
    prefs.putString("adminToken", stored);
  }
  adminToken = stored;
  showCommissioningSecrets = !prefs.getBool("commissioned", false);
  prefs.end();
}

void setup() {
  Serial.begin(115200);
  delay(100);

  sysState.bootMs = millis();
  loadConfig();
  loadOrCreateAdminToken();

  initSensors();
  initDisplay();
  initWiFiAP();
  initWebServer();

  lastSensorMs = millis();
  lastFilterMs = millis();
  lastTftMs = millis();
  lastSerialMs = millis();
  lastWebFrameMs = millis();
  lastFilterMicros = micros();

  if (cfg.startupCalibration && (mpuState.present || adxlState.present)) {
    beginCalibrationAsync();
  } else {
    sysState.calibrationMessage = "skipped";
  }

  updateOutputAngles();
  sysState.systemStatus = statusLevel();
  printBootInfo();
}

void loop() {
  uint32_t now = millis();

  server.handleClient();
  processCalibrationTask();

  if (now - lastSensorMs >= cfg.sensorMs) {
    lastSensorMs = now;
    readSensorsTask();
  }

  if (now - lastFilterMs >= cfg.filterMs) {
    lastFilterMs = now;
    filterTask();
  }

  if (now - lastTftMs >= cfg.tftMs) {
    lastTftMs = now;
    updateTftTask();
  }

  if (now - lastSerialMs >= cfg.serialMs) {
    lastSerialMs = now;
    serialTask();
  }

  if (now - lastWebFrameMs >= cfg.webMs) {
    lastWebFrameMs = now;
    sysState.dataFrame++;
  }
}
