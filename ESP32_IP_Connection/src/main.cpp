#include <Arduino.h>
#include <WiFi.h>

// put function declarations here:

// const char* ssid     = "AMAN";
// const char* password = "aman2024";
const char* ssid     = "TP-Link_ECA4";
const char* password = "50867387";
IPAddress local_IP(192, 168, 1, 184);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);   //optional

WiFiServer server(5000);
const char* PIP = "192.168.1.185";
const int PORT = 5001;
WiFiClient client;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(2, OUTPUT); // Onboard LED
  pinMode(4, INPUT); // External LED

  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(!client){
    Serial.println("No client connected");
    WiFiClient newClient = server.accept();
    if (newClient) {
    client = newClient;
    Serial.println("New Client Connected");
    Serial.println("Client IP: ");
    Serial.println(client.remoteIP());
    while (client.connected()) {
      if (client.available()) {
        String request = client.readStringUntil('\r');
        Serial.print("Received: ");
        Serial.println(request);
        client.flush();

        // Echo back the received message
        client.print("Echo: ");
        client.println(request);
        }
      }
    }
  }
  else {
    Serial.println("No new client");
    if(client && client.available()) {
      String request = client.readStringUntil('\r');
      Serial.print("Received from existing client: ");
      Serial.println(request);
      client.flush();

      // Echo back the received message
      client.print("Echo: ");
      client.println(request);
    }
  }
  // client.stop();
  // Serial.println("Client Disconnected");

  WiFiClient sender;
  if (sender.connect(PIP, PORT)) {
    sender.println("Hello from ESP32");
    sender.stop();
    Serial.println("Message sent to server");
  } else {
    Serial.println("Connection to server failed");
  }
  delay(1000); // Wait for 5 seconds before next loop
  client = server.available();
}

