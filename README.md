# ESP32 Digital Ship Inclinometer

Sketch Arduino IDE untuk ESP32 dengan:
- MPU6050 (sensor utama: accelerometer + gyroscope)
- ADXL345 (sensor validasi/redundansi accelerometer)
- TFT ILI9341 320x240 (SPI)
- Dashboard web lokal via ESP32 Access Point (`192.168.4.1`)

File utama: `inclinometer.ino`

## Fitur utama

- Dashboard **publik** di `/` (tanpa token):
  - Roll/pitch, arah, gauge
  - Data MPU6050 dan ADXL345
  - Sensor difference
  - Status sistem/sensor
  - Informasi jaringan, uptime, update indicator
- Panel **admin/settings** hanya di `/admin?token=...`
- API data publik: `GET /api/data`
- API settings aman (wajib token), dan **token admin tidak pernah dikembalikan** dari API settings
- AP statik: `192.168.4.1`
- Preferences/NVS untuk penyimpanan konfigurasi + data kalibrasi

## Library yang diperlukan (Arduino IDE)

- **ESP32 Arduino Core**
- **Adafruit MPU6050**
- **Adafruit ADXL345 Unified**
- **Adafruit Unified Sensor**
- **Adafruit GFX Library**
- **Adafruit ILI9341**

## Wiring default

### I2C (MPU6050 + ADXL345)
- SDA: GPIO21
- SCL: GPIO22

### TFT ILI9341 (SPI)
- SCK: GPIO18
- MISO: GPIO19
- MOSI: GPIO23
- CS: GPIO5
- DC: GPIO2
- RST: GPIO4

Semua pin ada di struct `PinConfig` dan mudah diubah.

## Address I2C umum

- MPU6050: `0x68` (default)
- ADXL345: `0x53` (default)

## Kalibrasi (zero + alignment)

Kalibrasi dirancang agar **tidak** salah menganggap komponen gravitasi X/Y sebagai bias accelerometer:

1. **Stillness check** dengan jendela waktu terukur (sampling + variance/RMS) untuk mendeteksi sensor benar-benar diam.
2. Hitung **gyro bias** MPU6050.
3. Simpan **gravity reference vector** (bukan menghapus gravitasi).
4. Simpan **zero-angle reference** roll/pitch agar posisi awal menjadi 0°.
5. Hitung **ADXL alignment offset** terhadap MPU saat zero calibration.
6. Aman bila hanya satu sensor tersedia.

Recalibration web: tombol **CALIBRATE ZERO** pada `/admin?token=...`.

Kalibrasi dijalankan sebagai state-machine asynchronous agar dashboard publik/API tidak freeze selama proses berlangsung. Progress/status muncul pada field calibration.

## Filtering untuk kapal (vessel-oriented)

Complementary filter menggunakan:
- Integrasi gyro dengan **dt aktual** berbasis `micros()`
- Clamping `dt` untuk menghindari lonjakan setelah blocking operation
- Low-pass/smoothing accelerometer
- Adaptive accelerometer correction berdasarkan kedekatan norm percepatan terhadap 1g
- Koreksi accelerometer dikurangi saat surge/heave/getaran/percepatan tinggi
- Guard NaN dan nilai ekstrem

## ADXL345 mapping dan validasi

- Axis/sign mapping konfigurabel (admin setting)
- Perbandingan dilakukan setelah alignment offset diterapkan
- Status dipisah jelas:
  - `SENSOR_ERROR`
  - `SENSOR_WARNING`
  - `WARNING_TILT`
  - `DANGER_TILT`
- Warning perbedaan sensor dibuat tidak terlalu agresif saat dynamic acceleration tinggi (adaptive scale)

## Sensor health

Masing-masing sensor menyimpan:
- `lastGoodReadMs`
- `consecutiveFails`
- plausibility checks
- timeout health check

Jika sensor hilang atau tidak sehat, sistem tetap berjalan (tidak crash), dashboard tetap dapat diakses.

## Konfigurasi Wi-Fi AP & admin

- SSID default: `SHIP-INCLINOMETER`
- Password default: `ShipInclinometer`
- AP IP: `192.168.4.1`
- Token admin dibangkitkan random, disimpan di NVS, dan dipakai ulang setelah reboot
- Serial boot mencetak commissioning info sekali:
  - SSID
  - Password AP
  - AP IP
  - URL admin lengkap

Untuk endpoint admin API:
- wajib kirim header `X-Admin-Token` yang valid
- jika request menyertakan header `Origin`, nilainya harus origin device (`http://192.168.4.1`, `http://192.168.4.1:80`, atau alias host `http://ship-inclinometer.local`)

Token query URL hanya dipakai untuk akses awal ke halaman `/admin?token=...`.

## Cara compile & upload

1. Buka `inclinometer.ino` di Arduino IDE.
2. Pilih board ESP32 yang sesuai.
3. Install library yang dibutuhkan.
4. Compile lalu upload.
5. Buka Serial Monitor 115200 untuk melihat info commissioning.
6. Koneksi ke AP ESP32.
7. Buka:
   - Public dashboard: `http://192.168.4.1/`
   - Admin/settings: `http://192.168.4.1/admin?token=...`

## Arsitektur loop non-blocking

Task timing terpisah:
- sensor read
- filter
- TFT
- serial
- web update frame marker

Loop normal tanpa `delay()`, termasuk saat kalibrasi (dikelola asynchronous state machine).

## Validasi/testing pada repository ini

Repository ini tidak menyediakan infrastruktur unit test otomatis saat ini. Validasi yang dilakukan difokuskan pada:
- pemeriksaan struktur sketch dan API endpoint
- pemeriksaan arsitektur task timing non-blocking
- verifikasi sintaks/struktur untuk kompatibilitas ESP32 Arduino core

Pengujian hardware langsung (sensor/TFT fisik) **belum dilakukan** di lingkungan ini.
