#include <Arduino.h>
#include <WiFi.h>

const char* ssid = "ZiggoA52DB1B"; // Fill SSID of network to scan

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
}

void loop() {
  int n = WiFi.scanNetworks();
  if (n != 0 ){
    for(int i=0; i<n; i++){
      if(WiFi.SSID(i).equals(ssid)){
        Serial.println("Network found!");
        Serial.print("RSSI: ");
        Serial.println(WiFi.RSSI(i));
        break;
      }
    }
  } else {
    Serial.println("No networks found!");
  }

  delay(1000);
}