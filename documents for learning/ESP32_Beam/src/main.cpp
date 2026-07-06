/*
 * ESP32 BEAM (boom) angle sensor  --  RPi-centric excavation depth system
 * -----------------------------------------------------------------------------
 * Role of this unit:
 *   - Mounted on the excavator BEAM / BOOM (the first, longer arm = L1).
 *   - Reads its tilt angle from an MPU6050 (DMP-fused roll).
 *   - Joins the FIELD WiFi as a STATION and PUSHES the RAW angle to the
 *     Raspberry Pi every POST_INTERVAL_MS:
 *
 *         POST  http://<RPI_HOST>:<RPI_PORT>/ingest
 *         Content-Type: application/json
 *         {"id":"beam","angle_deg":34.56,"pitch_deg":..,"yaw_deg":..,
 *          "mpu_ok":true,"seq":123,"uptime_ms":456789}
 *
 *   - The Raspberry Pi owns ALL calibration + geometry and computes depth:
 *         depth = L1*sin(boom_angle) + L2*sin(stick_angle)
 *     so this firmware sends RAW degrees only (no offsets / no sign flip here).
 *
 * This file is the TWIN of ESP32_Stick/src/main.cpp; the ONLY functional
 * difference is DEVICE_ID = "beam". Keep the two in sync when editing.
 *
 * Wiring (MPU6050 over I2C):  SDA=GPIO21  SCL=GPIO22  VCC=3V3  GND=GND
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ------------------------------------------------------------------ config ---
// The field WiFi that the Raspberry Pi is ALSO joined to (a 4G router / phone
// hotspot, or the Pi's own AP). This is NOT the old "ESP32_Server" SoftAP.
   

const char *WIFI_SSID = "AMAN 2";        // 4G modem SSID (note the space)
const char *WIFI_PASS = "AMAN2018";
// Raspberry Pi address on that WiFi + the receiver endpoint (sensor_receiver.py).
// Reserve a static lease for the Pi on the router so this never changes.
const char     *RPI_HOST    = "192.168.100.60";
// const char     *RPI_HOST    = "192.168.0.110";
const uint16_t  RPI_PORT    = 5000;
const char     *INGEST_PATH = "/ingest";

// Identity of THIS unit.
const char *DEVICE_ID = "beam";

const uint32_t POST_INTERVAL_MS = 500;   // telemetry rate (100 ms = 10 Hz)
#define LED_BUILTIN 2                     // onboard LED: solid when WiFi is up

// ------------------------------------------------------------------- state ---
MPU6050 mpu(0x68);
bool      dmpReady   = false;
uint16_t  packetSize = 0;
uint8_t   fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

float rollDeg = 0, pitchDeg = 0, yawDeg = 0;
uint32_t seq = 0;
unsigned long lastPostMs   = 0;
unsigned long lastWifiTry  = 0;

// --------------------------------------------------------------- wifi / imu --
// Non-blocking WiFi keep-alive: retry at most every 2 s so loop() never stalls.
static void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  digitalWrite(LED_BUILTIN, LOW);
  unsigned long now = millis();
  if (now - lastWifiTry < 2000) return;
  lastWifiTry = now;
  Serial.println("[wifi] (re)connecting...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static void initMPU() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection())
    Serial.println("MPU6050 connection failed (will report mpu_ok=false)");

  Serial.println("Auto calibrating... keep sensor STILL");
  delay(2000);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("DMP ready!");
  } else {
    Serial.printf("DMP init failed: %d (angle disabled)\n", devStatus);
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
  rollDeg  = ypr[2] * 180.0f / PI;   // arm tilt; same axis the depth model uses
  return true;
}

// Build + POST the telemetry JSON. Returns true on HTTP 200.
static bool postData() {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClient client;
  HTTPClient http;
  String url = String("http://") + RPI_HOST + ":" + RPI_PORT + INGEST_PATH;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  http.setConnectTimeout(800);   // don't let a missing Pi stall the 10 Hz loop
  http.setTimeout(800);

  JsonDocument doc;
  doc["id"]        = DEVICE_ID;
  doc["angle_deg"] = rollDeg;     // RAW tilt; the Pi applies sign/offset+geometry
  doc["pitch_deg"] = pitchDeg;
  doc["yaw_deg"]   = yawDeg;
  doc["mpu_ok"]    = dmpReady;
  doc["seq"]       = seq;
  doc["uptime_ms"] = (uint32_t)millis();
  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  String err = (code <= 0) ? http.errorToString(code) : String();
  http.end();

  if (code > 0) {
    Serial.printf("[post] %s angle=%.2f seq=%lu -> HTTP %d\n",
                  DEVICE_ID, rollDeg, (unsigned long)seq, code);
    seq++;
    return code == 200;
  }
  Serial.printf("[post] failed: %s\n", err.c_str());
  return false;
}

// --------------------------------------------------------------- setup/loop --
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Joining WiFi '%s' ... posting to http://%s:%u%s as '%s'\n",
                WIFI_SSID, RPI_HOST, RPI_PORT, INGEST_PATH, DEVICE_ID);

  initMPU();
}

void loop() {
  ensureWifi();
  if (WiFi.status() == WL_CONNECTED) digitalWrite(LED_BUILTIN, HIGH);

  readAngle();   // refresh the latest angle whenever the DMP has a fresh packet

  unsigned long now = millis();
  if (now - lastPostMs >= POST_INTERVAL_MS) {
    lastPostMs = now;
    postData();
  }
}
