#include <Arduino.h>
#include <ESP8266WiFi.h>

// put function declarations here:
// const char* ssid     = "AMAN";
// const char* password = "aman2024";
const char* ssid     = "TP-Link_ECA4";
const char* password = "50867387";

IPAddress local_IP(192, 168, 1, 185);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);   //optional

WiFiServer server(5001);
const char* PIP = "192.168.1.184";
const int PORT = 5000;

WiFiClient client; // persistent client

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10);
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(!client){
    WiFiClient newClient = server.accept(); // ✅ works more reliably than accept() on ESP8266
    if (newClient) {
      Serial.println("New client connected");
      client = newClient; // assign to persistent client
      Serial.print("Client IP: ");
      Serial.println(client.remoteIP());
    }
  }
  else {
  if (client && client.available()) {
    Serial.println("Client data available");
    String msg = client.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(msg);
    client.flush();
    }
  }

  WiFiClient senderClient;
  if (senderClient.connect(PIP, PORT)) {
    Serial.println("Connected to server");
    senderClient.println("Hello from ESP8266 client");
    senderClient.stop();
  } else {
    Serial.println("Connection to server failed");
  }
  delay(1000); // wait for 10 seconds before next loop
  client = server.accept();
}

// put function definitions here:
