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
 *          "mpu_ok":true,"seq":123,"uptime_ms":456789,
 *          "ip":"192.168.0.61","len_seq":3}
 *
 *   - The Raspberry Pi owns ALL calibration + geometry and computes depth:
 *         depth = L1*sin(boom_angle) + L2*sin(stick_angle)
 *     so this firmware sends RAW degrees only (no offsets / no sign flip here).
 *
 * ON-BOARD CONFIG PORTAL (see web_config.h/.cpp -- identical file in ESP32_Stick):
 * the radio runs AP+STA, so besides joining the field WiFi this unit ALWAYS
 * publishes its own access point:
 *
 *     SSID "EXCAV-BEAM"  password "excav1234"  ->  http://192.168.4.1/
 *
 * That page shows the live angle and takes the field WiFi SSID + password, the
 * Raspberry Pi IP/port and the LENGTH of the beam this unit is bolted to.
 * Everything is stored in NVS. A new length is handed to the Pi ONCE
 * (POST /length) and retried only until the Pi confirms it. Because the AP never
 * goes down, a mistyped WiFi password is always fixable from the same page --
 * no USB cable, no reflash. The values below are only the factory defaults used
 * until the operator saves something. This unit has no GNSS module, so the page
 * hides the location card (the stick's SIM7000G is what reports position).
 *
 * This file is the TWIN of ESP32_Stick/src/main.cpp; the differences are
 * DEVICE_ID = "beam", the FIELD_ID block, and the stick's GPS task. web_config.h
 * and web_config.cpp are byte-identical in both projects -- keep them that way.
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

#include "web_config.h"   // AP + web config portal (identical file in ESP32_Stick)

// ------------------------------------------------------------------ config ---
// FACTORY DEFAULTS ONLY. Whatever the operator saves on http://192.168.4.1/ is
// kept in NVS and wins over these from the next boot on; erase the nvs partition
// (pio run -t erase) to fall back to them.
//
// def_ssid/def_pass are the field WiFi the Raspberry Pi is ALSO joined to (a 4G
// router / phone hotspot, or the Pi's own AP) -- NOT the config AP above.
// constexpr char  DEF_WIFI_SSID[] = "AMAN 2";      // 4G modem SSID (note the space)
// constexpr char  DEF_WIFI_PASS[] = "AMAN2018";
constexpr char     DEF_WIFI_SSID[] = "A 3";         // 4G modem SSID (note the space)
constexpr char     DEF_WIFI_PASS[] = "98832988";
// Raspberry Pi address on that WiFi. Reserve a static lease for the Pi on the
// router so this never changes.
constexpr char     DEF_RPI_HOST[]  = "192.168.0.110";
constexpr uint16_t DEF_RPI_PORT    = 5000;
constexpr uint32_t DEF_BEAM_MM     = 1100;          // beam length L1 (mm)

const char *INGEST_PATH = "/ingest";

// Identity of THIS unit.
constexpr char DEVICE_ID[] = "beam";

// Identity handed to the config portal. Only these lines differ from the STICK
// copy -- keep the AP SSID / IP distinct so there is never any doubt which unit's
// page you have open.
static const FieldCfgIdentity FIELD_ID = {
    DEVICE_ID,                    // device_id -- the id the Pi keys on
    "Excavator BEAM (boom / L1)",
    "EXCAV-BEAM",                 // AP SSID
    "excav1234",                  // AP password (>= 8 chars)
    {192, 168, 4, 1},             // fixed AP IP  (stick uses 192.168.5.1)
    "beam",                       // http://beam.local on the field WiFi
    false,                        // no GNSS on this unit -> hide the location card
    DEF_WIFI_SSID, DEF_WIFI_PASS, DEF_RPI_HOST, DEF_RPI_PORT, DEF_BEAM_MM,
};

const uint32_t POST_INTERVAL_MS = 500;   // telemetry rate (500 ms = 2 Hz)
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
bool wifiWasUp = false;    // rising-edge latch: print the station IP once per join

// --------------------------------------------------------------- wifi / imu --
// Non-blocking WiFi keep-alive: retry at most every 2 s so loop() never stalls.
// Credentials come from NVS (set on the config page), falling back to DEF_WIFI_*
// on a never-provisioned unit.
static void ensureWifi() {
  if (fieldcfg_take_wifi_reapply()) {       // operator just saved new credentials
    Serial.printf("[wifi] applying new credentials from the web page: '%s'\n",
                  fieldcfg_ssid());
    WiFi.disconnect();
    WiFi.begin(fieldcfg_ssid(), fieldcfg_pass());
    lastWifiTry = millis();
    return;
  }
  if (WiFi.status() == WL_CONNECTED) return;
  digitalWrite(LED_BUILTIN, LOW);
  unsigned long now = millis();
  if (now - lastWifiTry < 2000) return;
  lastWifiTry = now;
  Serial.println("[wifi] (re)connecting...");
  WiFi.disconnect();
  WiFi.begin(fieldcfg_ssid(), fieldcfg_pass());
}

// delay() that keeps the config page answering -- a plain delay() during the MPU
// calibration would make the portal look dead for the first seconds after
// power-up (nothing else services the web server on this single-task firmware).
static void servicedDelay(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    fieldcfg_handle();
    delay(10);
  }
}

static void initMPU() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection())
    Serial.println("MPU6050 connection failed (will report mpu_ok=false)");

  Serial.println("Auto calibrating... keep sensor STILL");
  servicedDelay(2000);
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
  // Publish the same numbers we POST so the config page shows the live angle.
  fieldcfg_set_angles(rollDeg, pitchDeg, yawDeg, dmpReady);
  return true;
}

// Build + POST the telemetry JSON. Returns true on HTTP 200.
static bool postData() {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClient client;
  HTTPClient http;
  // Pi address comes from NVS (set on the config page).
  String url = String("http://") + fieldcfg_host() + ":" + fieldcfg_port()
             + INGEST_PATH;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  http.setConnectTimeout(800);   // don't let a missing Pi stall the loop
  http.setTimeout(800);

  JsonDocument doc;
  doc["id"]        = DEVICE_ID;
  doc["angle_deg"] = rollDeg;     // RAW tilt; the Pi applies sign/offset+geometry
  doc["pitch_deg"] = pitchDeg;
  doc["yaw_deg"]   = yawDeg;
  doc["mpu_ok"]    = dmpReady;
  doc["seq"]       = seq;
  doc["uptime_ms"] = (uint32_t)millis();
  // Two extras for the Pi: our IP (so its log can print a link to this unit's
  // config page) and the length edit counter. The Pi answers need_length=true
  // when that counter is not the one it has stored -- e.g. its state file was
  // wiped -- and web_config then re-runs the one-shot /length hand-off.
  doc["ip"]        = WiFi.localIP().toString();
  doc["len_seq"]   = fieldcfg_len_seq();
  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  String err   = (code <= 0) ? http.errorToString(code) : String();
  String reply = (code > 0) ? http.getString() : String();
  http.end();

  if (code > 0) {
    Serial.printf("[post] %s angle=%.2f seq=%lu -> HTTP %d\n",
                  DEVICE_ID, rollDeg, (unsigned long)seq, code);
    seq++;
    fieldcfg_check_ingest_reply(reply);
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

  // Brings up AP+STA, the SoftAP on its fixed IP and the web server, and loads
  // the saved SSID / Pi address / beam length out of NVS. Must come BEFORE
  // WiFi.begin(): it is what selects WIFI_AP_STA mode.
  fieldcfg_begin(FIELD_ID);
  WiFi.begin(fieldcfg_ssid(), fieldcfg_pass());
  Serial.printf("Joining WiFi '%s' ... posting to http://%s:%u%s as '%s'\n",
                fieldcfg_ssid(), fieldcfg_host(), fieldcfg_port(),
                INGEST_PATH, DEVICE_ID);

  // Wait (up to 15 s, still serving the config page) for DHCP so the station IP
  // -- the address this ESP has ON THE FIELD WIFI, i.e. what you point a browser
  // or ping at from the Pi -- lands in the boot log. 192.168.4.1 is the SoftAP
  // and is NOT this address. A timeout is not fatal: loop() keeps retrying and
  // prints the IP as soon as the join succeeds.
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    fieldcfg_handle();
    delay(100);
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiWasUp = true;
    Serial.printf("[wifi] connected to '%s' -- ESP station IP %s  gw %s  RSSI %d dBm\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(),
                  WiFi.gatewayIP().toString().c_str(), (int)WiFi.RSSI());
  } else {
    Serial.printf("[wifi] not joined yet (status %d) -- no station IP; retrying in loop\n",
                  (int)WiFi.status());
  }

  initMPU();
}

void loop() {
  ensureWifi();
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  if (wifiUp) digitalWrite(LED_BUILTIN, HIGH);

  // Reprint on every (re)join: the router can hand out a different lease.
  if (wifiUp && !wifiWasUp) {
    Serial.printf("[wifi] connected to '%s' -- ESP station IP %s  gw %s  RSSI %d dBm\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(),
                  WiFi.gatewayIP().toString().c_str(), (int)WiFi.RSSI());
  }
  wifiWasUp = wifiUp;

  readAngle();       // refresh the latest angle whenever the DMP has a fresh packet
  fieldcfg_handle(); // serve the config page (every loop pass)

  unsigned long now = millis();
  if (now - lastPostMs >= POST_INTERVAL_MS) {
    lastPostMs = now;
    postData();
    // loop() owns WiFi here -> also the right place for the one-shot length push.
    fieldcfg_sync(now);
  }
}
