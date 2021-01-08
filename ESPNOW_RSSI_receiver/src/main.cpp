// Load Wi-Fi library
#include <Arduino.h>
#include <WiFi.h>

// Replace with your network credentials
const char* ssid     = "ESP32-Gerwin";
const char* password = "gerwin12345";

// Set web server port number to 80
WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_AP);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.begin();
}

void loop(){
 
}