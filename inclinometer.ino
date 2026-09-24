#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ========================= CONFIGURATION =========================
constexpr uint8_t DEFAULT_I2C_SDA = 21;
constexpr uint8_t DEFAULT_I2C_SCL = 22;
constexpr uint8_t DEFAULT_TFT_SCK = 18;
constexpr uint8_t DEFAULT_TFT_MISO = 19;
constexpr uint8_t DEFAULT_TFT_MOSI = 23;
constexpr uint8_t DEFAULT_TFT_CS = 5;
constexpr uint8_t DEFAULT_TFT_DC = 2;
constexpr uint8_t DEFAULT_TFT_RST = 4;

struct PinConfig {
  uint8_t i2cSDA = DEFAULT_I2C_SDA;
  uint8_t i2cSCL = DEFAULT_I2C_SCL;

  uint8_t tftSCK = DEFAULT_TFT_SCK;
  uint8_t tftMISO = DEFAULT_TFT_MISO;
  uint8_t tftMOSI = DEFAULT_TFT_MOSI;
  uint8_t tftCS = DEFAULT_TFT_CS;
  uint8_t tftDC = DEFAULT_TFT_DC;
  uint8_t tftRST = DEFAULT_TFT_RST;
};

struct RuntimeConfig {
  float rollWarning = 8.0f;
  float rollDanger = 15.0f;
  float pitchWarning = 8.0f;
  float pitchDanger = 15.0f;
  float diffThreshold = 3.0f;
  float complementaryAlpha = 0.96f;
  uint16_t webUpdateIntervalMs = 250;
  bool startupCalibration = true;
};

struct TaskInterval {
  uint16_t sensorMs = 20;   // 50 Hz
  uint16_t filterMs = 20;   // 50 Hz
  uint16_t tftMs = 100;     // 10 Hz
  uint16_t serialMs = 500;  // 2 Hz
};

struct SensorData {
  bool detected = false;
  bool healthy = false;

  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;

  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;

  float roll = 0.0f;
  float pitch = 0.0f;

  float axOffset = 0.0f;
  float ayOffset = 0.0f;
  float azOffset = 0.0f;

  float gxOffset = 0.0f;
  float gyOffset = 0.0f;
  float gzOffset = 0.0f;
};

enum SystemState {
  STATE_NORMAL,
  STATE_WARNING,
  STATE_DANGER,
  STATE_SENSOR_WARNING,
  STATE_SENSOR_ERROR
};

struct SystemData {
  float rollDiff = 0.0f;
  float pitchDiff = 0.0f;
  String rollDirection = "LEVEL";
  String pitchDirection = "LEVEL";
  SystemState state = STATE_SENSOR_ERROR;
  bool calibrating = false;

  uint32_t loopCount = 0;
  uint32_t lastLoopRateTs = 0;
  float loopRateHz = 0.0f;

  uint32_t bootMs = 0;
  uint32_t lastSensorMs = 0;
  uint32_t lastFilterMs = 0;
  uint32_t lastTftMs = 0;
  uint32_t lastSerialMs = 0;
  uint32_t lastWebMs = 0;
};

PinConfig pins;
RuntimeConfig cfg;
const RuntimeConfig cfgDefaults;
TaskInterval intervals;
SensorData mpu;
SensorData adxl;
SystemData sysData;

Adafruit_MPU6050 mpu6050;
Adafruit_ADXL345_Unified adxl345(12345);
SPIClass tftSPI(VSPI);
Adafruit_ILI9341 *tft = nullptr;
WebServer server(80);
Preferences preferences;
String webDataCache = "{}";

const char *AP_SSID = "SHIP-INCLINOMETER";
String apPassword = "";
String defaultApPassword = "";
String adminToken = "";
String defaultAdminToken = "";
IPAddress apIP(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

String statusText(SystemState state) {
  switch (state) {
    case STATE_NORMAL:
      return "NORMAL";
    case STATE_WARNING:
      return "WARNING";
    case STATE_DANGER:
      return "DANGER";
    case STATE_SENSOR_WARNING:
      return "SENSOR WARNING";
    case STATE_SENSOR_ERROR:
    default:
      return "SENSOR ERROR";
  }
}

uint16_t statusColor(SystemState state) {
  switch (state) {
    case STATE_NORMAL:
      return ILI9341_GREEN;
    case STATE_WARNING:
      return ILI9341_YELLOW;
    case STATE_DANGER:
      return ILI9341_RED;
    case STATE_SENSOR_WARNING:
      return ILI9341_ORANGE;
    case STATE_SENSOR_ERROR:
    default:
      return ILI9341_MAGENTA;
  }
}

String formatSigned(float value, uint8_t decimals = 2) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%+.*f", decimals, value);
  return String(buf);
}

String uptimeString() {
  uint32_t sec = (millis() - sysData.bootMs) / 1000;
  uint32_t h = sec / 3600;
  uint32_t m = (sec % 3600) / 60;
  uint32_t s = sec % 60;
  char out[16];
  snprintf(out, sizeof(out), "%02lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m, (unsigned long)s);
  return String(out);
}

void loadSettings();
void saveSettings();
void resetDefaultSettings();
void initCredentialDefaults();

void initSensors();
void initDisplay();
void initWiFiAP();
void applyAccessPointConfig();
void initWebServer();

bool calibrateSensors();
bool sensorsAreStill(uint16_t samples = 80);

void readMPU6050();
void readADXL345();
void calculateAngles(float dtSec);
void filterAngles(float dtSec);
void validateSensors();

void updateTFT(bool force = false);
void updateWebData();
void updateSerial();
void checkAlarm();

String webHtml();
String buildDataJson();
String jsonEscape(const String &input);
bool isAuthorized();
bool tryParseFloatArg(const char *name, float &outValue);
bool tryParseUIntArg(const char *name, uint16_t &outValue);
void handleRoot();
void handleData();
void handleSettingsGet();
void handleSettingsPost();
void handleCalibrate();
void handleReset();

void setup() {
  Serial.begin(115200);
  sysData.bootMs = millis();

  initCredentialDefaults();
  loadSettings();
  Wire.begin(pins.i2cSDA, pins.i2cSCL);

  initSensors();
  initDisplay();
  initWiFiAP();
  initWebServer();

  if (cfg.startupCalibration) {
    calibrateSensors();
  }

  updateTFT(true);
  Serial.println("System ready.");
}

void loop() {
  const uint32_t now = millis();
  sysData.loopCount++;

  server.handleClient();

  if (now - sysData.lastSensorMs >= intervals.sensorMs) {
    sysData.lastSensorMs = now;
    readMPU6050();
    readADXL345();
  }

  if (now - sysData.lastFilterMs >= intervals.filterMs) {
    float dtSec = (intervals.filterMs / 1000.0f);
    sysData.lastFilterMs = now;
    calculateAngles(dtSec);
    filterAngles(dtSec);
    validateSensors();
    checkAlarm();
  }

  if (now - sysData.lastTftMs >= intervals.tftMs) {
    sysData.lastTftMs = now;
    updateTFT();
  }

  if (now - sysData.lastSerialMs >= intervals.serialMs) {
    sysData.lastSerialMs = now;
    updateSerial();
  }

  if (now - sysData.lastWebMs >= cfg.webUpdateIntervalMs) {
    sysData.lastWebMs = now;
    updateWebData();
  }

  if (now - sysData.lastLoopRateTs >= 1000) {
    float dt = (now - sysData.lastLoopRateTs) / 1000.0f;
    if (dt > 0) {
      sysData.loopRateHz = sysData.loopCount / dt;
    }
    sysData.loopCount = 0;
    sysData.lastLoopRateTs = now;
  }
}

void initSensors() {
  mpu.detected = mpu6050.begin(0x68, &Wire);
  mpu.healthy = mpu.detected;
  if (mpu.detected) {
    mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  adxl.detected = adxl345.begin(0x53);
  adxl.healthy = adxl.detected;
  if (adxl.detected) {
    adxl345.setRange(ADXL345_RANGE_16_G);
  }
}

void initDisplay() {
  if (tft == nullptr) {
    tft = new Adafruit_ILI9341(&tftSPI, pins.tftDC, pins.tftCS, pins.tftRST);
  }
  if (tft == nullptr) {
    return;
  }

  tftSPI.begin(pins.tftSCK, pins.tftMISO, pins.tftMOSI, pins.tftCS);
  tft->begin();
  tft->setRotation(1);
  tft->fillScreen(ILI9341_BLACK);

  tft->setTextColor(ILI9341_CYAN);
  tft->setTextSize(2);
  tft->setCursor(10, 8);
  tft->print("SHIP INCLINOMETER");

  tft->drawFastHLine(0, 28, 320, ILI9341_DARKCYAN);
  tft->setTextSize(1);
  tft->setTextColor(ILI9341_WHITE);

  tft->setCursor(10, 36);
  tft->print("ROLL");
  tft->setCursor(170, 36);
  tft->print("PITCH");

  tft->setCursor(10, 102);
  tft->print("ADXL ROLL:");
  tft->setCursor(170, 102);
  tft->print("ADXL PITCH:");

  tft->setCursor(10, 118);
  tft->print("DIFF R/P:");
  tft->setCursor(10, 134);
  tft->print("STATUS:");
}

void initWiFiAP() {
  WiFi.mode(WIFI_AP);
  applyAccessPointConfig();
}

void applyAccessPointConfig() {
  WiFi.softAPdisconnect(true);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  if (apPassword.length() == 0) {
    WiFi.softAP(AP_SSID);
  } else {
    WiFi.softAP(AP_SSID, apPassword.c_str());
  }
}

void initWebServer() {
  const char *headerKeys[] = {"X-Admin-Token"};
  server.collectHeaders(headerKeys, 1);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/settings", HTTP_GET, handleSettingsGet);
  server.on("/api/settings", HTTP_POST, handleSettingsPost);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/api/reset", HTTP_POST, handleReset);
  server.begin();
}

bool sensorsAreStill(uint16_t samples) {
  if (!mpu.detected) {
    return false;
  }

  float gyroPeak = 0.0f;
  float accNormPeak = 0.0f;

  for (uint16_t i = 0; i < samples; i++) {
    sensors_event_t accEvent;
    sensors_event_t gyroEvent;
    sensors_event_t tempEvent;
    mpu6050.getEvent(&accEvent, &gyroEvent, &tempEvent);

    float gxDps = gyroEvent.gyro.x * 57.29578f;
    float gyDps = gyroEvent.gyro.y * 57.29578f;
    float gzDps = gyroEvent.gyro.z * 57.29578f;
    float gPeakLocal = fmaxf(fabsf(gxDps), fmaxf(fabsf(gyDps), fabsf(gzDps)));
    gyroPeak = fmaxf(gyroPeak, gPeakLocal);

    float norm = sqrtf(accEvent.acceleration.x * accEvent.acceleration.x +
                       accEvent.acceleration.y * accEvent.acceleration.y +
                       accEvent.acceleration.z * accEvent.acceleration.z);
    float gNorm = norm / 9.80665f;
    float dev = fabsf(gNorm - 1.0f);
    accNormPeak = fmaxf(accNormPeak, dev);

    yield();
  }

  return gyroPeak < 2.0f && accNormPeak < 0.08f;
}

bool calibrateSensors() {
  if (!mpu.detected && !adxl.detected) {
    return false;
  }

  sysData.calibrating = true;
  updateTFT(true);

  if (mpu.detected && !sensorsAreStill(120)) {
    sysData.calibrating = false;
    return false;
  }

  const uint16_t samples = 250;

  float mAx = 0, mAy = 0, mAz = 0, mGx = 0, mGy = 0, mGz = 0;
  float aAx = 0, aAy = 0, aAz = 0;

  for (uint16_t i = 0; i < samples; i++) {
    if (mpu.detected) {
      sensors_event_t accEvent;
      sensors_event_t gyroEvent;
      sensors_event_t tempEvent;
      mpu6050.getEvent(&accEvent, &gyroEvent, &tempEvent);
      mAx += accEvent.acceleration.x;
      mAy += accEvent.acceleration.y;
      mAz += accEvent.acceleration.z;
      mGx += gyroEvent.gyro.x * 57.29578f;
      mGy += gyroEvent.gyro.y * 57.29578f;
      mGz += gyroEvent.gyro.z * 57.29578f;
    }

    if (adxl.detected) {
      sensors_event_t e;
      adxl345.getEvent(&e);
      aAx += e.acceleration.x;
      aAy += e.acceleration.y;
      aAz += e.acceleration.z;
    }

    yield();
  }

  if (mpu.detected) {
    mpu.axOffset = mAx / samples;
    mpu.ayOffset = mAy / samples;
    mpu.azOffset = (mAz / samples) - 9.80665f;

    mpu.gxOffset = mGx / samples;
    mpu.gyOffset = mGy / samples;
    mpu.gzOffset = mGz / samples;

    mpu.roll = 0.0f;
    mpu.pitch = 0.0f;
  }

  if (adxl.detected) {
    adxl.axOffset = aAx / samples;
    adxl.ayOffset = aAy / samples;
    adxl.azOffset = (aAz / samples) - 9.80665f;

    adxl.roll = 0.0f;
    adxl.pitch = 0.0f;
  }

  sysData.calibrating = false;
  return true;
}

void readMPU6050() {
  if (!mpu.detected) {
    mpu.healthy = false;
    return;
  }

  sensors_event_t accEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;
  mpu6050.getEvent(&accEvent, &gyroEvent, &tempEvent);

  mpu.ax = accEvent.acceleration.x - mpu.axOffset;
  mpu.ay = accEvent.acceleration.y - mpu.ayOffset;
  mpu.az = accEvent.acceleration.z - mpu.azOffset;

  mpu.gx = (gyroEvent.gyro.x * 57.29578f) - mpu.gxOffset;
  mpu.gy = (gyroEvent.gyro.y * 57.29578f) - mpu.gyOffset;
  mpu.gz = (gyroEvent.gyro.z * 57.29578f) - mpu.gzOffset;

  mpu.healthy = isfinite(mpu.ax) && isfinite(mpu.ay) && isfinite(mpu.az) && isfinite(mpu.gx) && isfinite(mpu.gy) && isfinite(mpu.gz);
}

void readADXL345() {
  if (!adxl.detected) {
    adxl.healthy = false;
    return;
  }

  sensors_event_t e;
  adxl345.getEvent(&e);

  adxl.ax = e.acceleration.x - adxl.axOffset;
  adxl.ay = e.acceleration.y - adxl.ayOffset;
  adxl.az = e.acceleration.z - adxl.azOffset;

  adxl.healthy = isfinite(adxl.ax) && isfinite(adxl.ay) && isfinite(adxl.az);
}

void calculateAngles(float dtSec) {
  if (mpu.healthy) {
    float accRoll = atan2f(mpu.ay, mpu.az) * 57.29578f;
    float accPitch = atan2f(-mpu.ax, sqrtf((mpu.ay * mpu.ay) + (mpu.az * mpu.az))) * 57.29578f;

    float gyroRoll = mpu.roll + mpu.gx * dtSec;
    float gyroPitch = mpu.pitch + mpu.gy * dtSec;

    mpu.roll = cfg.complementaryAlpha * gyroRoll + (1.0f - cfg.complementaryAlpha) * accRoll;
    mpu.pitch = cfg.complementaryAlpha * gyroPitch + (1.0f - cfg.complementaryAlpha) * accPitch;
  }

  if (adxl.healthy) {
    adxl.roll = atan2f(adxl.ay, adxl.az) * 57.29578f;
    adxl.pitch = atan2f(-adxl.ax, sqrtf((adxl.ay * adxl.ay) + (adxl.az * adxl.az))) * 57.29578f;
  }
}

void filterAngles(float dtSec) {
  (void)dtSec;
  // complementary filter already applied in calculateAngles; place retained for architecture expansion
}

void validateSensors() {
  if (!mpu.detected && !adxl.detected) {
    sysData.rollDiff = 0.0f;
    sysData.pitchDiff = 0.0f;
    sysData.state = STATE_SENSOR_ERROR;
    return;
  }

  if (!mpu.healthy) {
    sysData.rollDiff = 0.0f;
    sysData.pitchDiff = 0.0f;
    sysData.state = STATE_SENSOR_ERROR;
    return;
  }
  float absRoll = fabsf(mpu.roll);
  float absPitch = fabsf(mpu.pitch);
  bool danger = absRoll >= cfg.rollDanger || absPitch >= cfg.pitchDanger;
  bool warning = absRoll >= cfg.rollWarning || absPitch >= cfg.pitchWarning;

  SystemState tiltState = STATE_NORMAL;
  if (danger) {
    tiltState = STATE_DANGER;
  } else if (warning) {
    tiltState = STATE_WARNING;
  }

  if (!adxl.healthy) {
    sysData.rollDiff = 0.0f;
    sysData.pitchDiff = 0.0f;
    if (tiltState == STATE_DANGER || tiltState == STATE_WARNING) {
      sysData.state = tiltState;
    } else {
      sysData.state = STATE_SENSOR_WARNING;
    }
    return;
  }

  sysData.rollDiff = fabsf(mpu.roll - adxl.roll);
  sysData.pitchDiff = fabsf(mpu.pitch - adxl.pitch);
  if (sysData.rollDiff > cfg.diffThreshold || sysData.pitchDiff > cfg.diffThreshold) {
    sysData.state = STATE_SENSOR_WARNING;
    return;
  }

  sysData.state = tiltState;
}

void checkAlarm() {
  const float levelDeadband = 0.35f;

  if (mpu.roll > levelDeadband) {
    sysData.rollDirection = "RIGHT";
  } else if (mpu.roll < -levelDeadband) {
    sysData.rollDirection = "LEFT";
  } else {
    sysData.rollDirection = "LEVEL";
  }

  if (mpu.pitch > levelDeadband) {
    sysData.pitchDirection = "FORWARD";
  } else if (mpu.pitch < -levelDeadband) {
    sysData.pitchDirection = "BACKWARD";
  } else {
    sysData.pitchDirection = "LEVEL";
  }
}

void drawGauge(int16_t x, int16_t y, int16_t w, int16_t h, float value, float warn, float danger, bool force = false) {
  if (force) {
    tft->drawRect(x, y, w, h, ILI9341_DARKGREY);
  }

  int16_t center = x + (w / 2);
  int16_t markerHalf = 2;
  int16_t markerH = h - 4;

  tft->fillRect(x + 1, y + 1, w - 2, h - 2, ILI9341_BLACK);

  int16_t warnPx = (int16_t)((warn / danger) * (w / 2));
  if (warnPx < 0) warnPx = 0;
  if (warnPx > w / 2) warnPx = w / 2;

  tft->fillRect(center - (w / 2) + 1, y + 1, (w / 2) - warnPx, h - 2, ILI9341_DARKGREEN);
  tft->fillRect(center - warnPx, y + 1, warnPx * 2, h - 2, tft->color565(140, 110, 0));
  tft->fillRect(center + warnPx, y + 1, (w / 2) - warnPx - 1, h - 2, tft->color565(110, 20, 20));

  float clamped = value;
  if (clamped > danger) clamped = danger;
  if (clamped < -danger) clamped = -danger;

  int16_t marker = center + (int16_t)((clamped / danger) * (w / 2));
  if (marker < x + 2) marker = x + 2;
  if (marker > x + w - 3) marker = x + w - 3;

  tft->fillRect(marker - markerHalf, y + 2, markerHalf * 2, markerH, ILI9341_WHITE);
  tft->drawFastVLine(center, y + 1, h - 2, ILI9341_LIGHTGREY);
}

void updateTFT(bool force) {
  if (tft == nullptr) {
    return;
  }

  static float pRoll = NAN, pPitch = NAN, pAdxlRoll = NAN, pAdxlPitch = NAN;
  static float pDR = NAN, pDP = NAN;
  static String pRS, pPS, pStatus;
  static bool pCal = false;

  auto changedF = [](float a, float b) {
    return !isfinite(a) || fabsf(a - b) > 0.03f;
  };

  if (force || changedF(mpu.roll, pRoll) || pCal != sysData.calibrating) {
    tft->fillRect(10, 50, 140, 24, ILI9341_BLACK);
    tft->setTextSize(3);
    tft->setTextColor(ILI9341_WHITE);
    tft->setCursor(10, 52);
    tft->print(formatSigned(mpu.roll));
    tft->print((char)247);
    pRoll = mpu.roll;
  }

  if (force || changedF(mpu.pitch, pPitch) || pCal != sysData.calibrating) {
    tft->fillRect(170, 50, 140, 24, ILI9341_BLACK);
    tft->setTextSize(3);
    tft->setTextColor(ILI9341_WHITE);
    tft->setCursor(170, 52);
    tft->print(formatSigned(mpu.pitch));
    tft->print((char)247);
    pPitch = mpu.pitch;
  }

  if (force || pRS != sysData.rollDirection) {
    tft->fillRect(10, 78, 140, 14, ILI9341_BLACK);
    tft->setTextSize(1);
    tft->setTextColor(ILI9341_CYAN);
    tft->setCursor(10, 80);
    tft->print(sysData.rollDirection);
    pRS = sysData.rollDirection;
  }

  if (force || pPS != sysData.pitchDirection) {
    tft->fillRect(170, 78, 140, 14, ILI9341_BLACK);
    tft->setTextSize(1);
    tft->setTextColor(ILI9341_CYAN);
    tft->setCursor(170, 80);
    tft->print(sysData.pitchDirection);
    pPS = sysData.pitchDirection;
  }

  if (force || changedF(adxl.roll, pAdxlRoll)) {
    tft->fillRect(85, 102, 70, 8, ILI9341_BLACK);
    tft->setTextColor(ILI9341_WHITE);
    tft->setCursor(85, 102);
    tft->print(formatSigned(adxl.roll));
    pAdxlRoll = adxl.roll;
  }

  if (force || changedF(adxl.pitch, pAdxlPitch)) {
    tft->fillRect(245, 102, 70, 8, ILI9341_BLACK);
    tft->setTextColor(ILI9341_WHITE);
    tft->setCursor(245, 102);
    tft->print(formatSigned(adxl.pitch));
    pAdxlPitch = adxl.pitch;
  }

  if (force || changedF(sysData.rollDiff, pDR) || changedF(sysData.pitchDiff, pDP)) {
    tft->fillRect(65, 118, 150, 8, ILI9341_BLACK);
    tft->setTextColor(ILI9341_WHITE);
    tft->setCursor(65, 118);
    tft->print(formatSigned(sysData.rollDiff));
    tft->print("/");
    tft->print(formatSigned(sysData.pitchDiff));
    pDR = sysData.rollDiff;
    pDP = sysData.pitchDiff;
  }

  String s = sysData.calibrating ? "CALIBRATING" : statusText(sysData.state);
  if (force || pStatus != s || pCal != sysData.calibrating) {
    uint16_t color = sysData.calibrating ? ILI9341_CYAN : statusColor(sysData.state);
    tft->fillRect(60, 134, 250, 10, ILI9341_BLACK);
    tft->setTextColor(color);
    tft->setCursor(60, 134);
    tft->print(s);
    pStatus = s;
    pCal = sysData.calibrating;
  }

  drawGauge(10, 160, 300, 24, mpu.roll, cfg.rollWarning, cfg.rollDanger, force);
  drawGauge(10, 198, 300, 24, mpu.pitch, cfg.pitchWarning, cfg.pitchDanger, force);

  tft->setTextSize(1);
  tft->setTextColor(ILI9341_WHITE);
  tft->fillRect(10, 224, 310, 12, ILI9341_BLACK);
  tft->setCursor(10, 226);
  tft->print("AP ");
  tft->print(AP_SSID);
  tft->print(" ");
  tft->print(WiFi.softAPIP());
}

void updateWebData() {
  webDataCache = buildDataJson();
}

void updateSerial() {
  Serial.println("================ SHIP INCLINOMETER ================");
  Serial.printf("MPU6050: %s\n", mpu.detected ? (mpu.healthy ? "OK" : "ERROR") : "NOT FOUND");
  Serial.printf("  ACC  X:%+.3f Y:%+.3f Z:%+.3f m/s2\n", mpu.ax, mpu.ay, mpu.az);
  Serial.printf("  GYRO X:%+.3f Y:%+.3f Z:%+.3f dps\n", mpu.gx, mpu.gy, mpu.gz);
  Serial.printf("  ROLL:%+.2f PITCH:%+.2f\n", mpu.roll, mpu.pitch);

  Serial.printf("ADXL345: %s\n", adxl.detected ? (adxl.healthy ? "OK" : "ERROR") : "NOT FOUND");
  Serial.printf("  ACC  X:%+.3f Y:%+.3f Z:%+.3f m/s2\n", adxl.ax, adxl.ay, adxl.az);
  Serial.printf("  ROLL:%+.2f PITCH:%+.2f\n", adxl.roll, adxl.pitch);

  Serial.printf("Difference: ROLL:%+.2f PITCH:%+.2f\n", sysData.rollDiff, sysData.pitchDiff);
  Serial.printf("System: STATUS:%s WIFI:%s IP:%s LOOP:%.1fHz UPTIME:%s\n",
                statusText(sysData.state).c_str(),
                AP_SSID,
                WiFi.softAPIP().toString().c_str(),
                sysData.loopRateHz,
                uptimeString().c_str());
}

void loadSettings() {
  preferences.begin("inclino", true);
  cfg.rollWarning = preferences.getFloat("rollWarn", cfg.rollWarning);
  cfg.rollDanger = preferences.getFloat("rollDanger", cfg.rollDanger);
  cfg.pitchWarning = preferences.getFloat("pitWarn", cfg.pitchWarning);
  cfg.pitchDanger = preferences.getFloat("pitDanger", cfg.pitchDanger);
  cfg.diffThreshold = preferences.getFloat("diffTh", cfg.diffThreshold);
  cfg.complementaryAlpha = preferences.getFloat("alpha", cfg.complementaryAlpha);
  cfg.webUpdateIntervalMs = preferences.getUShort("webInt", cfg.webUpdateIntervalMs);
  cfg.startupCalibration = preferences.getBool("startCal", cfg.startupCalibration);
  String pwd = preferences.getString("apPass", apPassword);
  String token = preferences.getString("admTok", adminToken);
  preferences.end();

  if (pwd.length() >= 8) {
    apPassword = pwd;
  } else {
    apPassword = defaultApPassword;
  }
  if (token.length() >= 8) {
    adminToken = token;
  } else {
    adminToken = defaultAdminToken;
  }

  cfg.complementaryAlpha = constrain(cfg.complementaryAlpha, 0.70f, 0.995f);
  cfg.webUpdateIntervalMs = constrain(cfg.webUpdateIntervalMs, (uint16_t)100, (uint16_t)1000);
}

void saveSettings() {
  preferences.begin("inclino", false);
  preferences.putFloat("rollWarn", cfg.rollWarning);
  preferences.putFloat("rollDanger", cfg.rollDanger);
  preferences.putFloat("pitWarn", cfg.pitchWarning);
  preferences.putFloat("pitDanger", cfg.pitchDanger);
  preferences.putFloat("diffTh", cfg.diffThreshold);
  preferences.putFloat("alpha", cfg.complementaryAlpha);
  preferences.putUShort("webInt", cfg.webUpdateIntervalMs);
  preferences.putBool("startCal", cfg.startupCalibration);
  preferences.putString("apPass", apPassword);
  preferences.putString("admTok", adminToken);
  preferences.end();
}

void resetDefaultSettings() {
  cfg = cfgDefaults;
  apPassword = defaultApPassword;
  adminToken = defaultAdminToken;
}

void initCredentialDefaults() {
  uint64_t mac = ESP.getEfuseMac();
  char suffix[9];
  snprintf(suffix, sizeof(suffix), "%08llX", (unsigned long long)(mac & 0xFFFFFFFFULL));

  defaultApPassword = String("Ship-") + suffix;
  defaultAdminToken = String("Admin-") + suffix;
  apPassword = defaultApPassword;
  adminToken = defaultAdminToken;
}

String jsonEscape(const String &input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); i++) {
    char c = input[i];
    if (c == '\"') out += "\\\"";
    else if (c == '\\\\') out += "\\\\";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else out += c;
  }
  return out;
}

bool isAuthorized() {
  return server.hasHeader("X-Admin-Token") && server.header("X-Admin-Token") == adminToken;
}

bool tryParseFloatArg(const char *name, float &outValue) {
  if (!server.hasArg(name)) {
    return true;
  }
  String raw = server.arg(name);
  char *endPtr = nullptr;
  double value = strtod(raw.c_str(), &endPtr);
  if (endPtr == raw.c_str() || *endPtr != '\0' || !isfinite((float)value)) {
    return false;
  }
  outValue = (float)value;
  return true;
}

bool tryParseUIntArg(const char *name, uint16_t &outValue) {
  if (!server.hasArg(name)) {
    return true;
  }
  String raw = server.arg(name);
  char *endPtr = nullptr;
  long value = strtol(raw.c_str(), &endPtr, 10);
  if (endPtr == raw.c_str() || *endPtr != '\0' || value < 0 || value > 65535) {
    return false;
  }
  outValue = (uint16_t)value;
  return true;
}

void handleRoot() {
  server.send(200, "text/html", webHtml());
}

void handleData() {
  if (webDataCache == "{}") {
    updateWebData();
  }
  server.send(200, "application/json", webDataCache);
}

String buildDataJson() {
  String status = jsonEscape(statusText(sysData.state));
  String rollDir = jsonEscape(sysData.rollDirection);
  String pitchDir = jsonEscape(sysData.pitchDirection);
  String uptime = jsonEscape(uptimeString());
  String ssid = jsonEscape(String(AP_SSID));
  String ipStr = jsonEscape(WiFi.softAPIP().toString());

  String json = "{";
  json += "\"online\":true,";
  json += "\"timestamp\":" + String(millis()) + ",";
  json += "\"uptime\":\"" + uptime + "\",";
  json += "\"status\":\"" + status + "\",";
  json += "\"rollDir\":\"" + rollDir + "\",";
  json += "\"pitchDir\":\"" + pitchDir + "\",";

  json += "\"mpu\":{";
  json += "\"detected\":" + String(mpu.detected ? "true" : "false") + ",";
  json += "\"healthy\":" + String(mpu.healthy ? "true" : "false") + ",";
  json += "\"ax\":" + String(mpu.ax, 3) + ",\"ay\":" + String(mpu.ay, 3) + ",\"az\":" + String(mpu.az, 3) + ",";
  json += "\"gx\":" + String(mpu.gx, 3) + ",\"gy\":" + String(mpu.gy, 3) + ",\"gz\":" + String(mpu.gz, 3) + ",";
  json += "\"roll\":" + String(mpu.roll, 2) + ",\"pitch\":" + String(mpu.pitch, 2);
  json += "},";

  json += "\"adxl\":{";
  json += "\"detected\":" + String(adxl.detected ? "true" : "false") + ",";
  json += "\"healthy\":" + String(adxl.healthy ? "true" : "false") + ",";
  json += "\"ax\":" + String(adxl.ax, 3) + ",\"ay\":" + String(adxl.ay, 3) + ",\"az\":" + String(adxl.az, 3) + ",";
  json += "\"roll\":" + String(adxl.roll, 2) + ",\"pitch\":" + String(adxl.pitch, 2);
  json += "},";

  json += "\"diff\":{";
  json += "\"roll\":" + String(sysData.rollDiff, 2) + ",";
  json += "\"pitch\":" + String(sysData.pitchDiff, 2);
  json += "},";

  json += "\"wifi\":{";
  json += "\"ssid\":\"" + ssid + "\",";
  json += "\"ip\":\"" + ipStr + "\",";
  json += "\"clients\":" + String(WiFi.softAPgetStationNum()) + "},";

  json += "\"rates\":{";
  json += "\"loopHz\":" + String(sysData.loopRateHz, 1) + ",";
  json += "\"sensorMs\":" + String(intervals.sensorMs) + ",";
  json += "\"filterMs\":" + String(intervals.filterMs) + ",";
  json += "\"tftMs\":" + String(intervals.tftMs) + ",";
  json += "\"webMs\":" + String(cfg.webUpdateIntervalMs);
  json += "},";

  json += "\"thresholds\":{";
  json += "\"rollWarning\":" + String(cfg.rollWarning, 2) + ",";
  json += "\"rollDanger\":" + String(cfg.rollDanger, 2) + ",";
  json += "\"pitchWarning\":" + String(cfg.pitchWarning, 2) + ",";
  json += "\"pitchDanger\":" + String(cfg.pitchDanger, 2) + ",";
  json += "\"diffThreshold\":" + String(cfg.diffThreshold, 2) + "}";

  json += "}";

  return json;
}

void handleSettingsGet() {
  String json = "{";
  json += "\"rollWarning\":" + String(cfg.rollWarning, 2) + ",";
  json += "\"rollDanger\":" + String(cfg.rollDanger, 2) + ",";
  json += "\"pitchWarning\":" + String(cfg.pitchWarning, 2) + ",";
  json += "\"pitchDanger\":" + String(cfg.pitchDanger, 2) + ",";
  json += "\"diffThreshold\":" + String(cfg.diffThreshold, 2) + ",";
  json += "\"alpha\":" + String(cfg.complementaryAlpha, 3) + ",";
  json += "\"webUpdateMs\":" + String(cfg.webUpdateIntervalMs) + ",";
  json += "\"startupCalibration\":" + String(cfg.startupCalibration ? "true" : "false") + "";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSettingsPost() {
  if (!isAuthorized()) {
    server.send(401, "application/json", "{\"ok\":false,\"reason\":\"unauthorized\"}");
    return;
  }

  float rollWarning = cfg.rollWarning;
  float rollDanger = cfg.rollDanger;
  float pitchWarning = cfg.pitchWarning;
  float pitchDanger = cfg.pitchDanger;
  float diffThreshold = cfg.diffThreshold;
  float alpha = cfg.complementaryAlpha;
  uint16_t webUpdateMs = cfg.webUpdateIntervalMs;

  if (!tryParseFloatArg("rollWarning", rollWarning) ||
      !tryParseFloatArg("rollDanger", rollDanger) ||
      !tryParseFloatArg("pitchWarning", pitchWarning) ||
      !tryParseFloatArg("pitchDanger", pitchDanger) ||
      !tryParseFloatArg("diffThreshold", diffThreshold) ||
      !tryParseFloatArg("alpha", alpha) ||
      !tryParseUIntArg("webUpdateMs", webUpdateMs)) {
    server.send(400, "application/json", "{\"ok\":false,\"reason\":\"invalid_numeric_input\"}");
    return;
  }

  cfg.rollWarning = rollWarning;
  cfg.rollDanger = rollDanger;
  cfg.pitchWarning = pitchWarning;
  cfg.pitchDanger = pitchDanger;
  cfg.diffThreshold = diffThreshold;
  cfg.complementaryAlpha = alpha;
  cfg.webUpdateIntervalMs = webUpdateMs;
  cfg.startupCalibration = server.hasArg("startupCalibration") ? (server.arg("startupCalibration") == "1") : cfg.startupCalibration;

  String oldApPassword = apPassword;
  if (server.hasArg("apPassword")) {
    String p = server.arg("apPassword");
    if (p.length() >= 8 && p.length() <= 63) {
      apPassword = p;
    }
  }
  if (server.hasArg("newAdminToken")) {
    String p = server.arg("newAdminToken");
    if (p.length() >= 8 && p.length() <= 63) {
      adminToken = p;
    }
  }

  cfg.rollWarning = fmaxf(0.1f, cfg.rollWarning);
  cfg.rollDanger = fmaxf(cfg.rollWarning + 0.1f, cfg.rollDanger);
  cfg.pitchWarning = fmaxf(0.1f, cfg.pitchWarning);
  cfg.pitchDanger = fmaxf(cfg.pitchWarning + 0.1f, cfg.pitchDanger);
  cfg.diffThreshold = fmaxf(0.1f, cfg.diffThreshold);
  cfg.complementaryAlpha = constrain(cfg.complementaryAlpha, 0.70f, 0.995f);
  cfg.webUpdateIntervalMs = constrain(cfg.webUpdateIntervalMs, (uint16_t)100, (uint16_t)1000);

  if (apPassword != oldApPassword) {
    applyAccessPointConfig();
  }

  saveSettings();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCalibrate() {
  if (!isAuthorized()) {
    server.send(401, "application/json", "{\"ok\":false,\"reason\":\"unauthorized\"}");
    return;
  }
  bool ok = calibrateSensors();
  server.send(ok ? 200 : 409, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false,\"reason\":\"sensor_moving_or_missing\"}");
}

void handleReset() {
  if (!isAuthorized()) {
    server.send(401, "application/json", "{\"ok\":false,\"reason\":\"unauthorized\"}");
    return;
  }
  resetDefaultSettings();
  applyAccessPointConfig();
  saveSettings();
  server.send(200, "application/json", "{\"ok\":true}");
}

String webHtml() {
  return R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>SHIP INCLINOMETER</title>
<style>
:root{--bg:#09111d;--panel:#121f32;--line:#234063;--text:#cde3ff;--ok:#11d97b;--warn:#ffd447;--danger:#ff4f4f;--muted:#84a7d2}
*{box-sizing:border-box}body{margin:0;font-family:Arial,Helvetica,sans-serif;background:var(--bg);color:var(--text)}
.wrap{max-width:1160px;margin:0 auto;padding:12px}.row{display:grid;grid-template-columns:1fr;gap:12px}
@media(min-width:900px){.row.two{grid-template-columns:1fr 1fr}.row.three{grid-template-columns:repeat(3,1fr)}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px}
.head{display:flex;justify-content:space-between;align-items:center;gap:8px;flex-wrap:wrap}
.title{font-weight:700;font-size:22px;letter-spacing:.6px}.online{color:var(--ok);font-weight:700}
.big{font-size:56px;font-weight:700;line-height:1;margin:8px 0}.dir{font-size:18px;color:var(--muted)}
.g{position:relative;height:22px;border:1px solid #3f5f89;border-radius:999px;background:linear-gradient(90deg,#115f35 0,#115f35 33%,#8e6c00 50%,#681717 100%);overflow:hidden}
.g .c{position:absolute;left:50%;top:0;bottom:0;width:1px;background:#fff8}
.g .m{position:absolute;top:2px;bottom:2px;width:10px;border-radius:8px;background:#fff;transform:translateX(-50%)}
.meta{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:6px;font-size:14px}
.statusDot{font-size:24px;vertical-align:middle}
.kv{display:flex;justify-content:space-between;gap:8px;padding:4px 0;border-bottom:1px dashed #345}
.kv:last-child{border-bottom:none}input,button{width:100%;padding:8px;border-radius:8px;border:1px solid #456;background:#0e1a2a;color:#d7e8ff}
button{background:#173455;cursor:pointer}button:hover{filter:brightness(1.1)}
.small{font-size:12px;color:#89a8cf}
</style>
</head>
<body>
<div class="wrap">
  <div class="card head">
    <div class="title">SHIP INCLINOMETER</div>
    <div><span class="online">ONLINE</span> <span class="small" id="stamp">-</span></div>
  </div>

  <div class="row two" style="margin-top:12px">
    <div class="card">
      <div>ROLL</div>
      <div class="big" id="rollVal">+0.00°</div>
      <div class="dir" id="rollDir">LEVEL</div>
      <div class="g" id="rollGauge"><span class="c"></span><span class="m" id="rollMarker"></span></div>
    </div>
    <div class="card">
      <div>PITCH</div>
      <div class="big" id="pitchVal">+0.00°</div>
      <div class="dir" id="pitchDir">LEVEL</div>
      <div class="g" id="pitchGauge"><span class="c"></span><span class="m" id="pitchMarker"></span></div>
    </div>
  </div>

  <div class="row two" style="margin-top:12px">
    <div class="card">
      <h3>MPU6050</h3>
      <div class="meta" id="mpuBox"></div>
    </div>
    <div class="card">
      <h3>ADXL345</h3>
      <div class="meta" id="adxlBox"></div>
    </div>
  </div>

  <div class="row three" style="margin-top:12px">
    <div class="card">
      <h3>SENSOR DIFFERENCE</h3>
      <div class="kv"><span>ROLL</span><b id="diffR">0.00°</b></div>
      <div class="kv"><span>PITCH</span><b id="diffP">0.00°</b></div>
    </div>
    <div class="card">
      <h3>SYSTEM STATUS</h3>
      <div class="kv"><span>Status</span><b id="statusTxt">-</b></div>
      <div class="kv"><span>Loop Rate</span><b id="loopHz">-</b></div>
      <div class="kv"><span>Uptime</span><b id="uptime">-</b></div>
    </div>
    <div class="card">
      <h3>NETWORK</h3>
      <div class="kv"><span>WiFi SSID</span><b id="ssid">-</b></div>
      <div class="kv"><span>IP</span><b id="ip">-</b></div>
      <div class="kv"><span>Clients</span><b id="clients">-</b></div>
    </div>
  </div>

  <div class="row two" style="margin-top:12px">
    <div class="card">
      <h3>SETTINGS</h3>
      <div class="meta">
        <label>ADMIN TOKEN<input id="adminToken" type="password" minlength="8" maxlength="63"></label>
        <label>ROLL WARNING<input id="rollWarning" type="number" step="0.1"></label>
        <label>ROLL DANGER<input id="rollDanger" type="number" step="0.1"></label>
        <label>PITCH WARNING<input id="pitchWarning" type="number" step="0.1"></label>
        <label>PITCH DANGER<input id="pitchDanger" type="number" step="0.1"></label>
        <label>DIFF THRESHOLD<input id="diffThreshold" type="number" step="0.1"></label>
        <label>FILTER ALPHA<input id="alpha" type="number" step="0.001" min="0.70" max="0.995"></label>
        <label>WEB UPDATE MS<input id="webUpdateMs" type="number" step="10" min="100" max="1000"></label>
        <label>AP PASSWORD<input id="apPassword" type="text" minlength="8" maxlength="63"></label>
        <label>NEW ADMIN TOKEN<input id="newAdminToken" type="password" minlength="8" maxlength="63"></label>
        <label style="display:flex;align-items:center;gap:8px;margin-top:24px">STARTUP CALIBRATION<input id="startupCalibration" type="checkbox" style="width:auto"></label>
      </div>
      <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px">
        <button id="btnCal">CALIBRATE ZERO</button>
        <button id="btnReset">RESET DEFAULT</button>
        <button id="btnSave">SAVE SETTINGS</button>
      </div>
    </div>
    <div class="card">
      <h3>SENSOR / SYSTEM FLAGS</h3>
      <div id="flags" class="small"></div>
    </div>
  </div>
</div>
<script>
let pollMs=250;
let danger=15;
const byId=(id)=>document.getElementById(id);
const fmt=(n)=>`${n>=0?'+':''}${Number(n).toFixed(2)}°`;
const toBar=(val,max)=>Math.max(0,Math.min(100,50+(val/max)*50));

function setGauge(marker,val,max){marker.style.left=`${toBar(val,max)}%`;}
function setStatusColor(text){
  const el=byId('statusTxt');
  let c='#11d97b';
  if(text.includes('DANGER')) c='#ff4f4f';
  else if(text.includes('WARNING')) c='#ffd447';
  else if(text.includes('ERROR')) c='#ff73ff';
  el.style.color=c;
}
function kv(entries){return entries.map(([k,v])=>`<div class="kv"><span>${k}</span><b>${v}</b></div>`).join('');}

async function loadSettings(){
  const r=await fetch('/api/settings'); const j=await r.json();
  for(const k of Object.keys(j)){
    const el=byId(k);
    if(!el) continue;
    if(el.type==='checkbox') el.checked=Boolean(j[k]);
    else el.value=j[k];
  }
  pollMs=Number(j.webUpdateMs)||250;
}

async function updateData(){
  try{
    const r=await fetch('/api/data');
    const d=await r.json();
    byId('stamp').textContent=`update ${new Date().toLocaleTimeString()}`;
    byId('rollVal').textContent=fmt(d.mpu.roll); byId('pitchVal').textContent=fmt(d.mpu.pitch);
    byId('rollDir').textContent=d.rollDir; byId('pitchDir').textContent=d.pitchDir;
    byId('diffR').textContent=fmt(d.diff.roll); byId('diffP').textContent=fmt(d.diff.pitch);
    byId('statusTxt').textContent=d.status; setStatusColor(d.status);
    byId('loopHz').textContent=`${Number(d.rates.loopHz).toFixed(1)} Hz`; byId('uptime').textContent=d.uptime;
    byId('ssid').textContent=d.wifi.ssid; byId('ip').textContent=d.wifi.ip; byId('clients').textContent=`${d.wifi.clients}`;

    byId('mpuBox').innerHTML=kv([
      ['Status', d.mpu.detected?(d.mpu.healthy?'OK':'ERROR'):'NOT FOUND'],
      ['ACC X/Y/Z', `${d.mpu.ax.toFixed(2)} / ${d.mpu.ay.toFixed(2)} / ${d.mpu.az.toFixed(2)}`],
      ['GYRO X/Y/Z', `${d.mpu.gx.toFixed(2)} / ${d.mpu.gy.toFixed(2)} / ${d.mpu.gz.toFixed(2)}`],
      ['ROLL', fmt(d.mpu.roll)], ['PITCH', fmt(d.mpu.pitch)]
    ]);

    byId('adxlBox').innerHTML=kv([
      ['Status', d.adxl.detected?(d.adxl.healthy?'OK':'ERROR'):'NOT FOUND'],
      ['ACC X/Y/Z', `${d.adxl.ax.toFixed(2)} / ${d.adxl.ay.toFixed(2)} / ${d.adxl.az.toFixed(2)}`],
      ['ROLL', fmt(d.adxl.roll)], ['PITCH', fmt(d.adxl.pitch)]
    ]);

    danger=Math.max(d.thresholds.rollDanger,d.thresholds.pitchDanger,1);
    setGauge(byId('rollMarker'),d.mpu.roll,danger);
    setGauge(byId('pitchMarker'),d.mpu.pitch,danger);

    byId('flags').innerHTML=kv([
      ['MPU6050', d.mpu.detected?(d.mpu.healthy?'OK':'ERROR'):'NOT FOUND'],
      ['ADXL345', d.adxl.detected?(d.adxl.healthy?'OK':'ERROR'):'NOT FOUND'],
      ['TFT', 'OK'], ['WiFi', 'OK'],
      ['Sensor Rate', `${d.rates.sensorMs} ms`],
      ['Filter Rate', `${d.rates.filterMs} ms`],
      ['TFT Rate', `${d.rates.tftMs} ms`],
      ['Web Rate', `${d.rates.webMs} ms`]
    ]);

    pollMs=Number(d.rates.webMs)||pollMs;
  }catch(e){byId('stamp').textContent='update failed';}
  setTimeout(updateData,pollMs);
}

async function post(path,params){
  const body=new URLSearchParams(params||{});
  const r=await fetch(path,{method:'POST',headers:{'X-Admin-Token':byId('adminToken').value||''},body});
  return r;
}

byId('btnSave').onclick=async()=>{
  const p={
    rollWarning:byId('rollWarning').value,rollDanger:byId('rollDanger').value,
    pitchWarning:byId('pitchWarning').value,pitchDanger:byId('pitchDanger').value,
    diffThreshold:byId('diffThreshold').value,alpha:byId('alpha').value,
    webUpdateMs:byId('webUpdateMs').value,apPassword:byId('apPassword').value,
    newAdminToken:byId('newAdminToken').value,
    startupCalibration:byId('startupCalibration').checked?'1':'0'
  };
  await post('/api/settings',p); await loadSettings();
};
byId('btnCal').onclick=()=>post('/api/calibrate');
byId('btnReset').onclick=async()=>{await post('/api/reset'); await loadSettings();};

loadSettings().then(updateData);
</script>
</body>
</html>
)HTML";
}
