#include <Arduino.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define LED_BUILTIN 2
const char* ssid = "ESP32_Server";
const char* pass = "12345678";

String serverIP = "192.168.4.1";

MPU6050 mpu(0x68);
float L1 = 1.1;
float L2 = 1.0;
bool dmpReady = false;
uint8_t fifoBuffer[64];


Quaternion q;
VectorFloat gravity;
float ypr[3];
float ypr2[3];

uint16_t packetSize;

void setup()
{
  Serial.begin(115200);

  WiFi.begin(ssid, pass);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  // Serial.println("Connected to ESP32");
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Initializing MPU6050...");

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    // while (1);
  }
  Serial.println("Auto calibrating... Keep sensor STILL");

  delay(2000);

  // Auto calibration
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  Serial.println("Calibration Done");

  Serial.println("Initializing DMP...");
  uint8_t devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {

    mpu.setDMPEnabled(true);

    packetSize = mpu.dmpGetFIFOPacketSize();

    dmpReady = true;

    Serial.println("DMP Ready!");

  } else {

    Serial.print("DMP Init Failed: ");
    Serial.println(devStatus);

    while (1);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFiClient client;
    HTTPClient http;
    if (!dmpReady) return;

    // roll1 is the STICK angle (theta2) the server uses for depth.
    // Only send when a fresh DMP packet arrived this loop, otherwise we would
    // transmit a stale/uninitialized value.
    bool haveFresh = false;
    float roll1 = 0.0f;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

      mpu.dmpGetQuaternion(&q, fifoBuffer);

      mpu.dmpGetGravity(&gravity, &q);

      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      float yaw   = ypr[0] * 180 / PI;
      float pitch = ypr[1] * 180 / PI;
      roll1  = ypr[2] * 180 / PI;
      haveFresh = true;

      // Serial.print("Yaw: ");
      // Serial.print(yaw);

      // Serial.print(" | Pitch: ");
      // Serial.print(pitch);

      Serial.print("Roll1: ");
      Serial.println(roll1);
      Serial.print("yaw: ");
      Serial.println(yaw);
      Serial.print("pitch: ");
      Serial.println(pitch);
      // float altitude = L1 * sin(-roll * PI / 180);
      // Serial.print("Altitude: ");
      // Serial.print(altitude);
      delay(50);
    }

    // Skip the HTTP round-trip if we have no new angle to report.
    if (!haveFresh) {
      delay(250);
      return;
    }

    String url =
      "http://" + serverIP +
      "/data?value=" + String(roll1, 2);

    http.begin(client, url);

    int httpCode = http.GET();

    if (httpCode > 0) {
      Serial.printf("POST stick=%.2f -> HTTP %d\n", roll1, httpCode);
    } else {
      Serial.printf("HTTP GET failed: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  }

  delay(250);
}