/*
 * ESP32 Excavation Server
 * ------------------------
 * Role in the system:
 *   - Creates a SoftAP  "ESP32_Server"  (IP 192.168.4.1).
 *   - Has its OWN MPU6050  -> measures the BOOM angle  (theta1).
 *   - Receives the STICK angle (theta2) from the ExcavatorClient via
 *         GET /data?value=<deg>
 *   - Receives alarms + target depth from a Raspberry Pi 4 via
 *         POST /alarm   {"alarm":bool,"message":str,"target_depth":float}
 *   - Computes live excavation depth and serves it on GET /status (JSON)
 *     and GET / (human page).
 *
 * Excavator geometry (first-order model):
 *
 *        pivot
 *          o------ L1 ------o
 *          \ )theta1         \ )theta2
 *           \                 \
 *            \                 +---- L2 ----> bucket tip
 *
 *   depth = L1*sin(theta1) + L2*sin(theta2)
 *
 *   Angles are measured BELOW the horizontal (digging down => positive).
 *   L1/L2 and the angle sign/offset MUST be calibrated on the real machine
 *   (see the tunables below).
 *
 * Wiring (MPU6050 over I2C):  SDA=GPIO21  SCL=GPIO22  (ESP32 default)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ---------------------------------------------------------------- config ----
#define LED_BUILTIN 2               // onboard LED, lit while alarm is active

const char *AP_SSID = "ESP32_Server";
const char *AP_PASS = "12345678";   // >= 8 chars required by SoftAP

// --- Arm geometry (CALIBRATE these for your machine) ---
float L1 = 1.1f;                    // boom length  (m)
float L2 = 1.0f;                    // stick length (m)

// Sign / zero-offset calibration. depth uses sin(angle).
// Flip *_SIGN if "digging down" reads as a negative angle on your mount.
#define BOOM_SIGN   (+1.0f)
#define STICK_SIGN  (+1.0f)
float boomOffsetDeg  = 0.0f;        // subtracted from the raw boom reading
float stickOffsetDeg = 0.0f;        // subtracted from the raw stick reading

// ---------------------------------------------------------------- state -----
WebServer server(80);

MPU6050 mpu(0x68);
bool dmpReady = false;
uint16_t packetSize = 0;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Live values shared between loop() and the HTTP handlers.
float boomDeg     = 0.0f;           // theta1, from local MPU6050
float stickDeg    = 0.0f;           // theta2, last value from ExcavatorClient
float depth       = 0.0f;           // computed excavation depth (m)
float targetDepth = 0.0f;           // set by the Raspberry Pi
bool  piAlarm     = false;          // alarm flag raised by the Pi
bool  depthAlarm  = false;          // depth >= targetDepth
String piMessage  = "";             // last message from the Pi

unsigned long lastStickMs = 0;      // millis() of last /data update
unsigned long lastPiMs    = 0;      // millis() of last /alarm update

// ------------------------------------------------------------- helpers ------
static float deg2rad(float d) { return d * PI / 180.0f; }

// Recompute depth from the current boom + stick angles.
static void recomputeDepth() {
  float t1 = BOOM_SIGN  * (boomDeg  - boomOffsetDeg);
  float t2 = STICK_SIGN * (stickDeg - stickOffsetDeg);
  depth = L1 * sin(deg2rad(t1)) + L2 * sin(deg2rad(t2));

  // depthAlarm only meaningful once the Pi has supplied a target.
  depthAlarm = (targetDepth > 0.0f) && (depth >= targetDepth);
  digitalWrite(LED_BUILTIN, (piAlarm || depthAlarm) ? HIGH : LOW);
}

// Build the status JSON document used by /status and /.
static void fillStatus(JsonDocument &doc) {
  doc["boom_deg"]     = boomDeg;
  doc["stick_deg"]    = stickDeg;
  doc["depth_m"]      = depth;
  doc["target_depth"] = targetDepth;
  doc["depth_alarm"]  = depthAlarm;
  doc["pi_alarm"]     = piAlarm;
  doc["message"]      = piMessage;
  doc["dmp_ready"]    = dmpReady;
  doc["stick_age_ms"] = lastStickMs ? (millis() - lastStickMs) : -1;
  doc["pi_age_ms"]    = lastPiMs    ? (millis() - lastPiMs)    : -1;
}

// ------------------------------------------------------------- handlers -----

// ExcavatorClient sends the stick angle:  GET /data?value=<deg>
void handleData() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "missing 'value'");
    return;
  }
  stickDeg = server.arg("value").toFloat();
  lastStickMs = millis();
  recomputeDepth();
  server.send(200, "text/plain", "OK");
  Serial.printf("[/data] stick=%.2f deg -> depth=%.3f m\n", stickDeg, depth);
}

// Raspberry Pi sends alarms + target depth:
//   POST /alarm  {"alarm":bool,"message":str,"target_depth":float}
void handleAlarm() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "use POST");
    return;
  }
  String body = server.arg("plain");
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    server.send(400, "application/json", "{\"error\":\"bad json\"}");
    Serial.printf("[/alarm] JSON parse failed: %s\n", err.c_str());
    return;
  }

  if (!doc["alarm"].isNull())        piAlarm     = doc["alarm"].as<bool>();
  if (!doc["message"].isNull())      piMessage   = doc["message"].as<String>();
  if (!doc["target_depth"].isNull()) targetDepth = doc["target_depth"].as<float>();
  lastPiMs = millis();
  recomputeDepth();

  JsonDocument out;
  fillStatus(out);
  String resp;
  serializeJson(out, resp);
  server.send(200, "application/json", resp);
  Serial.printf("[/alarm] piAlarm=%d target=%.3f msg=%s\n",
                piAlarm, targetDepth, piMessage.c_str());
}

// Live status for the Pi:  GET /status -> JSON
void handleStatus() {
  JsonDocument doc;
  fillStatus(doc);
  String resp;
  serializeJson(doc, resp);
  server.send(200, "application/json", resp);
}

// Human-readable status page:  GET /
void handleRoot() {
  String html = "<html><head><meta http-equiv='refresh' content='1'>"
                "<title>Excavation Server</title></head><body>"
                "<h2>ESP32 Excavation Server</h2><pre>";
  html += "boom (theta1) : " + String(boomDeg, 2)  + " deg\n";
  html += "stick(theta2) : " + String(stickDeg, 2) + " deg\n";
  html += "depth         : " + String(depth, 3)    + " m\n";
  html += "target depth  : " + String(targetDepth, 3) + " m\n";
  html += "depth alarm   : " + String(depthAlarm ? "YES" : "no") + "\n";
  html += "pi alarm      : " + String(piAlarm ? "YES" : "no") + "\n";
  html += "pi message    : " + piMessage + "\n";
  html += "</pre></body></html>";
  server.send(200, "text/html", html);
}

// --------------------------------------------------------------- IMU --------
static void initMPU() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed (continuing without boom sensor)");
    // return;
  }

  Serial.println("Auto calibrating... keep sensor STILL");
  delay(2000);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println("Calibration done");

  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("DMP ready!");
  } else {
    Serial.printf("DMP init failed: %d (boom angle disabled)\n", devStatus);
  }
}

// Read the boom angle if a fresh DMP packet is available. Returns true on update.
static bool readBoom() {
  if (!dmpReady) return false;
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return false;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  boomDeg = ypr[2] * 180.0f / PI;   // roll == arm tilt, same axis as the client
  return true;
}

// -------------------------------------------------------------- setup -------
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("SoftAP \"");
  Serial.print(AP_SSID);
  Serial.print("\" started, IP: ");
  Serial.println(WiFi.softAPIP());   // 192.168.4.1

  initMPU();

  server.on("/",       HTTP_GET,  handleRoot);
  server.on("/data",   HTTP_GET,  handleData);
  server.on("/status", HTTP_GET,  handleStatus);
  server.on("/alarm",  HTTP_POST, handleAlarm);
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  server.begin();
  Serial.println("HTTP server started");
}

// -------------------------------------------------------------- loop --------
void loop() {
  server.handleClient();            // non-blocking

  // Refresh boom angle + depth at ~20 Hz without blocking the web server.
  static unsigned long lastImu = 0;
  if (millis() - lastImu >= 50) {
    lastImu = millis();
    if (readBoom()) recomputeDepth();
  }
}
