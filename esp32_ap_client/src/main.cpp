// ESP32 client for the Raspberry Pi concurrent AP+STA setup.
//
// Joins the Pi's own access point (raspberry_pi/ap_sta_setup.py) and, every few
// seconds, POSTs a JSON sample to the Pi then GETs back what the Pi stored --
// demonstrating both HTTP verbs against raspberry_pi/ap_demo_server.py.
//
// Watch it work:  pio run -t upload -t monitor   (Serial @115200)

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ---- Must match the Raspberry Pi AP (raspberry_pi/ap_sta_setup.py) ----------
static const char*    WIFI_SSID      = "RPi_AP";
static const char*    WIFI_PASS      = "raspberry123";
static const char*    BASE_URL       = "http://192.168.50.1:8080";  // Pi fixed AP IP
static const char*    DEVICE_ID      = "esp32-01";
static const uint32_t SEND_PERIOD_MS = 5000;

static uint32_t g_seq = 0;

// Join the Pi's access point as a station. Blocks up to ~20 s per attempt.
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Joining AP \"%s\" ", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400);
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf(" ok, IP=%s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println(" FAILED (will retry)");
}

// POST a JSON sample to the Pi. Returns HTTP status (<0 on transport error).
int postSample() {
  HTTPClient http;
  http.begin(String(BASE_URL) + "/ingest");
  http.addHeader("Content-Type", "application/json");

  JsonDocument doc;                                    // ArduinoJson v7
  doc["device"]    = DEVICE_ID;
  doc["seq"]       = ++g_seq;
  doc["uptime_ms"] = millis();
  doc["temp_c"]    = 20.0 + (float)(esp_random() % 1000) / 100.0;  // fake sensor
  doc["heap"]      = ESP.getFreeHeap();
  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  Serial.printf("POST /ingest  seq=%lu  -> %d", (unsigned long)g_seq, code);
  if (code > 0) Serial.printf("  resp=%s", http.getString().c_str());
  Serial.println();
  http.end();
  return code;
}

// GET the value the Pi last stored, parse it, print a couple of fields.
int getLatest() {
  HTTPClient http;
  http.begin(String(BASE_URL) + "/latest");

  int code = http.GET();
  Serial.printf("GET  /latest            -> %d", code);
  if (code > 0) {
    String payload = http.getString();
    JsonDocument doc;
    if (deserializeJson(doc, payload) == DeserializationError::Ok)
      Serial.printf("  server_seq=%d  temp_c=%.2f",
                    (int)(doc["_seq_server"] | -1),
                    (float)(doc["temp_c"] | 0.0f));
    else
      Serial.printf("  raw=%s", payload.c_str());
  }
  Serial.println();
  http.end();
  return code;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nESP32 -> Raspberry Pi AP client");
  connectWiFi();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {   // AP went away / boot race -> reconnect
    connectWiFi();
    delay(1000);
    return;
  }
  postSample();
  getLatest();
  Serial.println("----");
  delay(SEND_PERIOD_MS);
}
