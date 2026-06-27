/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-https-requests-sim-card-sim7000g/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. Based on the library example: github.com/vshymanskyy/TinyGSM/blob/master/examples/HttpsClient/HttpsClient.ino
*/

// Select your modem
// #define TINY_GSM_MODEM_SIM7000
// #define TINY_GSM_RX_BUFFER 1024


#include "GeoMindgsm.h"
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <SSLClient.h>


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
// #define LOGGING  // <- Logging is for the HTTP library

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
// #define TINY_GSM_YIELD() { delay(2); }

// set GSM PIN, if any
#define GSM_PIN ""

// flag to force SSL client authentication, if needed
// #define TINY_GSM_SSL_CLIENT_AUTHENTICATION

// Set your APN Details / GPRS credentials
const char apn[]      = "taif";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Geomind API details
const char baseUrl[] = "app.geo-mind.ai";
const char GEO_USER[] = "mohammad.arab";
const char GEO_PASS[] = "Rakeeza@2025";
String accessToken;


// Server details
const char server[]   = "www.asciiart.eu";
const char resource[] = "/art/8d5b0e86c79a0afb";
const int  port       = 443;

TinyGsm        modem(SerialAT);
TinyGsmClient client(modem);
SSLClient sslClient(&client);

GeoMind geoMind(baseUrl, &sslClient);
Vector vector(&geoMind);

// LilyGO T-SIM7000G Pinout
#define UART_BAUD           9600
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13
#define LED_PIN             12

void modemPowerOn(){
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff(){
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500);
  digitalWrite(PWR_PIN, HIGH);
}

void modemRestart(){
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

void setup() {
  // Set Serial Monitor baud rate
  SerialMon.begin(115200);
  delay(10);

  // Set LED OFF
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  modemPowerOn();

  SerialMon.println("Wait...");

  // Set GSM module baud rate and Pins
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(3000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }
  Serial.println("Connecting to APN...");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println("GPRS failed");
    while (true);
  }


  Serial.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println("Network failed");
    while (true);
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }
  SerialMon.print(F("Performing HTTPS GET request... "));
  sslClient.setInsecure();  // Use with SSL, otherwise leave out


  Serial.println("Make sure your LTE antenna has been connected to the SIM interface on the board.");
  delay(5000);
  if (geoMind.getToken(GEO_USER, GEO_PASS)) {
    Serial.println("Login Success");
  } else {
    Serial.println("Login Failed");
  }
  // vector.getVectors();
  // delay(500);
  vector.getVectorByID(2061); // Example: Get vector with ID 1
  delay(500);
  String vectorUUID = "806205de-d9b4-4cb1-a7fc-43aeffc79535";
  String Name = "Altitude ex";
  String displayName = "IoT";
  String description = "Measuring altitude from SIM7000G";
  // vector.UpdateVector(vectorUUID, Name, displayName, description);
  // client.setInsecure();  // Use with SSL, otherwise leave out
  int featureID = 0;
  vector.createFeature(featureID, 10.123, 20.456, "Point", "Test Feature");
}

void loop() {
}
