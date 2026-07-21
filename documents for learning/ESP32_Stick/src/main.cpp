/*
 * ESP32 STICK angle + GPS sensor  --  RPi-centric excavation depth system
 * -----------------------------------------------------------------------------
 * Hardware: LilyGO T-SIM7000G (ESP32-WROVER + SIM7000G LTE/GNSS, 4 MB flash).
 *
 * Dual-core split (FreeRTOS, pinned tasks):
 *   - Core 1 (APP_CPU)  angleTask : MPU6050 DMP roll (EMA-filtered) -> POST /ingest @ 1 Hz
 *   - Core 0 (PRO_CPU)  gpsTask   : SIM7000G GNSS fix  -> POST /location (5 s until first fix, then 5 min)
 *
 * Transport for BOTH streams is the field WiFi (the site's 4G router that the
 * Raspberry Pi is also on). The SIM7000G is used ONLY as a GNSS receiver -- no
 * SIM, no cellular data. Each task owns its own WiFiClient + HTTPClient, so the
 * two cores never share a socket (separate LwIP sockets are thread-safe -> no
 * mutex needed). WiFi connect/reconnect is owned solely by angleTask to avoid a
 * two-core reconnect race.
 *
 * The Raspberry Pi still owns ALL calibration + geometry and computes depth;
 * this firmware sends smoothed (uncalibrated) degrees + RAW lat/lon only.
 *
 * NOTE: the Pi's sensor_receiver.py must expose a /location route to accept the
 * new GPS payload (the existing /ingest angle route is unchanged).
 *
 * Wiring:
 *   MPU6050 (I2C) : SDA=GPIO21  SCL=GPIO22  VCC=3V3  GND=GND  (21/22 free here)
 *   SIM7000G      : on-board UART  ESP_TX=27 -> modemRX, ESP_RX=26 <- modemTX,
 *                   PWRKEY=GPIO4, DTR=GPIO25  (board-fixed, do not reassign)
 *   LED           : GPIO12 (on-board) -- solid when WiFi is up
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// SIM7000G modem (GNSS only). The modem type is normally set via build_flags
// (-D TINY_GSM_MODEM_SIM7000); this guard keeps a direct compile working too.
#ifndef TINY_GSM_MODEM_SIM7000
#define TINY_GSM_MODEM_SIM7000
#endif
#include <TinyGsmClient.h>

// ------------------------------------------------------------------ config ---
const char *WIFI_SSID = "A 3";          // site 4G-router SSID (note the space)
const char *WIFI_PASS = "98832988";
// const char *WIFI_SSID = "AMAN 2";          // site 4G-router SSID (note the space)
// const char *WIFI_PASS = "AMAN2018";
// const char    *RPI_HOST      = "10.118.136.50";

const char    *RPI_HOST      = "192.168.0.110";
const uint16_t RPI_PORT      = 5000;
const char    *INGEST_PATH   = "/ingest";    // angle stream (existing)
const char    *LOCATION_PATH = "/location";  // NEW gps stream (add Pi-side route)

const char *DEVICE_ID = "stick";

// Sample the DMP fast (feeds the noise filter) but POST at 1 Hz. The Pi flags a
// unit "stale" after 1.5 s (sensor_receiver.py stale_ms=1500), so 1 s posting is
// reliable; 2 s would trip that alarm. Boom/stick move slowly -> 1 Hz is plenty
// for the depth readout while cutting network load ~10x vs the old 10 Hz.
const uint32_t SAMPLE_INTERVAL_MS = 10;            // DMP read + filter @ 100 Hz
const uint32_t POST_INTERVAL_MS   = 1000;          // angle POST @ 1 Hz
const uint32_t LOC_INTERVAL_MS    = 5UL * 60 * 1000; // location: every 5 minutes (steady)
const uint32_t GPS_SEARCH_MS      = 5000;            // ...but poll @5 s until a fix

// --- LilyGO T-SIM7000G fixed pin map -----------------------------------------
#define LED_PIN      12
#define MODEM_TX     27   // ESP -> modem RX
#define MODEM_RX     26   // ESP <- modem TX
#define MODEM_PWRKEY 4
#define MODEM_DTR    25
#define MODEM_BAUD   115200

HardwareSerial SerialAT(1);   // UART1 to the SIM7000G
TinyGsm        modem(SerialAT);

// ------------------------------------------------------------------- state ---
MPU6050 mpu(0x68);
bool        dmpReady   = false;
uint16_t    packetSize = 0;
uint8_t     fifoBuffer[64];
Quaternion  q;
VectorFloat gravity;
float       ypr[3];

float    rollDeg = 0, pitchDeg = 0, yawDeg = 0;   // raw DMP angles (deg)

// --- Noise filter: exponential moving average (EMA) low-pass ----------------
// One-pole IIR run at the DMP sample rate to smooth jitter before we POST:
//   filt += ALPHA * (raw - filt);   ALPHA in (0,1]: smaller = smoother + laggier
// Yaw stays RAW: it wraps at +/-180 deg and an EMA across that wrap would glitch.
const float EMA_ALPHA = 0.1f;
float    rollFilt = 0, pitchFilt = 0;
bool     filtInit = false;

uint32_t angleSeq = 0, locSeq = 0;
unsigned long lastWifiTry = 0;

// --------------------------------------------------------------- wifi / http --
// Non-blocking WiFi keep-alive. Called ONLY from angleTask so the two cores
// never fight over WiFi.begin()/disconnect().
static void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  digitalWrite(LED_PIN, LOW);
  unsigned long now = millis();
  if (now - lastWifiTry < 2000) return;
  lastWifiTry = now;
  Serial.println("[wifi] (re)connecting...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// Generic JSON POST. Each caller builds its own body and uses a LOCAL client,
// so angleTask (core 1) and gpsTask (core 0) post on independent sockets.
static int httpPostJson(const char *path, const String &body) {
  if (WiFi.status() != WL_CONNECTED) return -1000;
  WiFiClient client;
  HTTPClient http;
  String url = String("http://") + RPI_HOST + ":" + RPI_PORT + path;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  http.setConnectTimeout(800);   // don't let a missing Pi stall the loop
  http.setTimeout(800);
  int code = http.POST(body);
  http.end();
  return code;
}

// ------------------------------------------------------------------ imu ------
static void initMPU() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.println("[mpu] init...");
  mpu.initialize();
  if (!mpu.testConnection())
    Serial.println("[mpu] connection failed (will report mpu_ok=false)");

  Serial.println("[mpu] auto-calibrating, keep STILL...");
  delay(2000);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("[mpu] DMP ready");
  } else {
    Serial.printf("[mpu] DMP init failed: %d (angle disabled)\n", devStatus);
  }
}

// Pull the latest DMP sample when one is available. Returns true on update.
static bool readAngle() {
  if (!dmpReady) return false;
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return false;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  yawDeg   = ypr[0] * 180.0f / PI;
  pitchDeg = ypr[1] * 180.0f / PI;
  rollDeg  = ypr[2] * 180.0f / PI;   // arm tilt the depth model uses

  // EMA low-pass: seed on the first packet, then smooth every fresh sample.
  if (!filtInit) {
    rollFilt = rollDeg; pitchFilt = pitchDeg; filtInit = true;
  } else {
    rollFilt  += EMA_ALPHA * (rollDeg  - rollFilt);
    pitchFilt += EMA_ALPHA * (pitchDeg - pitchFilt);
  }
  return true;
}

// ----------------------------------------------------------- angle task (1) --
static void postAngle() {
  JsonDocument doc;
  doc["id"]        = DEVICE_ID;
  doc["angle_deg"] = rollFilt;    // EMA-filtered; Pi applies sign/offset + geometry
  doc["pitch_deg"] = pitchFilt;   // EMA-filtered
  doc["yaw_deg"]   = yawDeg;      // raw (yaw wraps; not used by the depth model)
  doc["mpu_ok"]    = dmpReady;
  doc["seq"]       = angleSeq;
  doc["uptime_ms"] = (uint32_t)millis();
  String body; serializeJson(doc, body);

  if (httpPostJson(INGEST_PATH, body) > 0) angleSeq++;
  // 1 Hz -- intentionally quiet (no per-post Serial spam)
}

static void angleTask(void *pv) {
  initMPU();
  TickType_t last = xTaskGetTickCount();
  unsigned long lastPostMs = 0;
  for (;;) {
    ensureWifi();
    if (WiFi.status() == WL_CONNECTED) digitalWrite(LED_PIN, HIGH);
    readAngle();                       // fast sampling feeds the EMA filter

    unsigned long now = millis();
    if (now - lastPostMs >= POST_INTERVAL_MS) {   // but only POST at 1 Hz
      lastPostMs = now;
      postAngle();
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
  }
}

// ------------------------------------------------------------- gps task (0) --
static void modemPowerOn() {
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW);   // PWRKEY is active-low: pulse >= 1 s
  delay(1100);
  digitalWrite(MODEM_PWRKEY, HIGH);
}

static void initModemGps() {
  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, LOW);      // hold DTR low: keep modem out of sleep
  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  modemPowerOn();
  delay(3000);                       // let the modem boot

  Serial.println("[gps] modem init...");
  if (!modem.testAT(10000)) {
    Serial.println("[gps] no AT response, restarting modem...");
    modem.restart();
  }
  // T-SIM7000G: enable the GNSS LNA power rail (modem's own GPIO4) before GPS.
  modem.sendAT("+SGPIO=0,4,1,1");
  modem.waitResponse(10000L);
  modem.enableGPS();
  Serial.println("[gps] GNSS enabled (cold fix can take 30-60 s outdoors)");
}

static void postLocation(bool fix, float lat, float lon, float alt,
                         float speed, int sats, float acc) {
  JsonDocument doc;
  doc["id"]     = DEVICE_ID;
  doc["gps_ok"] = fix;
  if (fix) {
    doc["lat"]       = lat;        // RAW WGS84 degrees
    doc["lon"]       = lon;
    doc["alt_m"]     = alt;
    doc["speed_kph"] = speed;
    doc["sats"]      = sats;
    doc["hacc_m"]    = acc;        // horizontal accuracy estimate
  }
  doc["seq"]       = locSeq;
  doc["uptime_ms"] = (uint32_t)millis();
  String body; serializeJson(doc, body);

  int code = httpPostJson(LOCATION_PATH, body);
  if (code > 0) {
    Serial.printf("[loc] fix=%d lat=%.6f lon=%.6f sats=%d seq=%lu -> HTTP %d\n",
                  fix, lat, lon, sats, (unsigned long)locSeq, code);
    locSeq++;
  } else {
    Serial.printf("[loc] POST failed (code %d)\n", code);
  }
}

static void gpsTask(void *pv) {
  initModemGps();
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    float lat = 0, lon = 0, speed = 0, alt = 0, acc = 0;
    int   vsat = 0, usat = 0;
    bool  fix = modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &acc);
    postLocation(fix, lat, lon, alt, speed, usat, acc);
    // Cold TTFF is 30-60 s; poll fast while there is NO fix so the first (and any
    // re-acquired) fix is reported within ~5 s of becoming available. Once fixed,
    // fall back to the slow 5-min cadence. (No fix ever -> keeps searching @5 s.)
    vTaskDelayUntil(&last, pdMS_TO_TICKS(fix ? LOC_INTERVAL_MS : GPS_SEARCH_MS));
  }
}

// --------------------------------------------------------------- setup/loop --
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[boot] WiFi '%s' | angle->%s:%u%s (1Hz) | loc->%s (5s->5min)\n",
                WIFI_SSID, RPI_HOST, RPI_PORT, INGEST_PATH, LOCATION_PATH);

  // Pin the two streams to opposite cores.
  xTaskCreatePinnedToCore(angleTask, "angle", 8192, NULL, 2, NULL, 1); // APP_CPU
  xTaskCreatePinnedToCore(gpsTask,   "gps",   8192, NULL, 1, NULL, 0); // PRO_CPU
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));   // all work runs in the pinned tasks
}
