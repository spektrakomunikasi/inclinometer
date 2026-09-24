# ESP32 Digital Ship Inclinometer

Project Arduino IDE untuk **ESP32** dengan:
- MPU6050 (sensor utama + gyro/accel)
- ADXL345 (validasi/redundansi)
- TFT ILI9341 320x240 (dashboard lokal)
- Web Dashboard lokal via **ESP32 Access Point**

## 1) Library yang diperlukan
Install dari **Library Manager** Arduino IDE:
- `Adafruit MPU6050`
- `Adafruit ADXL345`
- `Adafruit Unified Sensor`
- `Adafruit GFX Library`
- `Adafruit ILI9341`

Library bawaan ESP32 core:
- `WiFi`
- `WebServer`
- `Preferences`
- `Wire`
- `SPI`

## 2) Struktur file
- `inclinometer.ino`

## 3) Konfigurasi pin
Di `inclinometer.ino` bagian `PinConfig`:
- I2C default:
  - SDA = GPIO21
  - SCL = GPIO22
- TFT SPI default:
  - SCK  = GPIO18
  - MISO = GPIO19
  - MOSI = GPIO23
  - CS   = GPIO5
  - DC   = GPIO2
  - RST  = GPIO4

Semua pin dibuat sebagai variabel konfigurasi agar mudah diubah.

## 4) Wiring
### I2C bus (MPU6050 + ADXL345)
- ESP32 GPIO21 (SDA) -> SDA MPU6050 & SDA ADXL345
- ESP32 GPIO22 (SCL) -> SCL MPU6050 & SCL ADXL345
- ESP32 3V3 -> VCC MPU6050 & VCC ADXL345
- ESP32 GND -> GND MPU6050 & GND ADXL345

### TFT ILI9341 (SPI)
- ESP32 GPIO18 -> SCK
- ESP32 GPIO19 -> MISO
- ESP32 GPIO23 -> MOSI
- ESP32 GPIO5  -> CS
- ESP32 GPIO2  -> DC
- ESP32 GPIO4  -> RST
- ESP32 3V3 -> VCC
- ESP32 GND -> GND

## 5) I2C address sensor
- MPU6050: `0x68`
- ADXL345: `0x53`

## 6) Konfigurasi WiFi AP
Default:
- SSID: `SHIP-INCLINOMETER`
- IP AP: `192.168.4.1`
- Password AP default otomatis unik per device: `Ship-XXXXXXXX` (XXXXXXXX = 8 digit hex dari MAC chip).
- Admin token manajemen default otomatis acak saat boot pertama: `Admin-<16 HEX acak>` (langsung disimpan ke NVS).
- Keduanya bisa diubah dari web settings.

## 7) Cara compile & upload
1. Buka folder project di Arduino IDE.
2. Pilih board ESP32 (contoh: ESP32 Dev Module).
3. Pastikan semua library terpasang.
4. Compile lalu upload ke ESP32.

## 8) Cara membuka web dashboard
1. Hubungkan HP/laptop ke Wi-Fi AP: `SHIP-INCLINOMETER`.
2. Buka browser ke: `http://192.168.4.1` untuk dashboard telemetry (read-only).
3. Untuk konfigurasi, buka: `http://192.168.4.1/admin` lalu login Basic Auth (username: `admin`, password: admin token aktif **atau** AP password aktif).
4. Dashboard update realtime (polling JSON) tanpa internet/cloud.

## 9) Kalibrasi
- Saat startup, jika `startupCalibration=true`, sistem mencoba kalibrasi otomatis saat sensor diam.
- Kalibrasi manual dari web: tombol **CALIBRATE ZERO**.
- Jika sensor bergerak signifikan, kalibrasi ditolak (proteksi agar offset valid).

## 10) Ubah threshold/settings
Di panel web settings tersedia:
- Roll warning/danger
- Pitch warning/danger
- Sensor difference threshold
- Filter alpha (complementary)
- Web update interval
- AP password
- Admin token (untuk autentikasi aksi manajemen)
- New admin token (opsional untuk mengganti token lama)
- Catatan: perubahan password AP dan token admin valid jika panjang 8-63 karakter.
- Nilai token admin aktif **tidak ditampilkan kembali** dari API; masukkan token saat ini secara manual pada halaman `/admin`.

Tombol:
- **SAVE SETTINGS** -> simpan ke NVS (Preferences)
- **RESET DEFAULT** -> kembali default dan simpan

## 11) Arsitektur program
Fungsi modular utama yang diimplementasikan:
- `initSensors()`
- `initDisplay()`
- `initWiFiAP()`
- `initWebServer()`
- `calibrateSensors()`
- `readMPU6050()`
- `readADXL345()`
- `calculateAngles()`
- `filterAngles()`
- `validateSensors()`
- `updateTFT()`
- `updateWebData()`
- `updateSerial()`
- `checkAlarm()`
- `saveSettings()`
- `loadSettings()`

Arsitektur non-blocking berbasis `millis()` dengan task terpisah:
- sensor sampling rate
- filter calculation rate
- TFT refresh rate
- serial update rate
- web update rate (via polling endpoint)

Catatan: proses **kalibrasi** (startup/manual) melakukan sampling beruntun dan bersifat blocking sementara sampai selesai, sehingga selama kalibrasi request HTTP/dashboard dan task periodik utama akan tertunda sesaat.

## 12) Status sistem
Status utama:
- `NORMAL`
- `WARNING`
- `DANGER`

Status validasi/error tambahan:
- `SENSOR WARNING` (difference antar sensor melebihi threshold)
- `SENSOR ERROR` (sensor gagal/invalid)

Arah:
- Roll: `LEFT` / `RIGHT` / `LEVEL`
- Pitch: `FORWARD` / `BACKWARD` / `LEVEL`

## 13) Catatan reliability
- Tidak memakai `delay()` di loop utama.
- Web server tetap berjalan meskipun ada error sensor.
- TFT update parsial untuk mengurangi flicker.
- Setting tersimpan di Preferences/NVS.
