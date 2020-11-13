#include <WiFi.h>
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <utils.h>

const char *ssid = userWiFiSSID;         //ssid internet access point, configure in utils.h
const char *password = userWiFipassword; //password internet access point, configure in utils.h

unsigned long lastTimeSended = 0; //to save what time (relative to millis) the last message to thingspeak was sent.

#define sendInterval 10000

//input:  requested state of WiFi communication
//output: true if succeed, otherwise false
//effect: sets WiFi communication in requested state
boolean WiFiconfig(boolean requestedState)
{
  if (requestedState)
  {

    Serial.println("Connecting");
    unsigned int thresholdCounter = 0;
#define thresholdValue 2000
    while (WiFi.status() != WL_CONNECTED && thresholdCounter < thresholdValue)
    {
      delay(1);
      thresholdCounter++;
      //Serial.print(".");
    }
    // Serial.println("");
    // Serial.print("Connected to WiFi network with IP Address: ");
    // Serial.println(WiFi.localIP());
    if (WiFi.status() == WL_CONNECTED)
    {
      // Serial.print("Wi-Fi Channel: ");
      // Serial.println(WiFi.channel());
      return true;
    }
  }
  else
  {
    if (WiFi.disconnect(true, true) == 1)
    {
      return true;
    }
  }
  return false;
}

void setup()
{

  Serial.begin(115200);
  Serial.println("Serial active");
  //WiFi.mode(WIFI_STA); //Set device as a Wi-Fi Station
}
boolean buttonState1 = false;
void loop()
{

  if (((millis() - lastTimeSended) > sendInterval))
  {
      WiFiconfig(true);
      if (WiFi.status() == WL_CONNECTED)
      {
        // if (sendToThingSpeak())
        // {
        lastTimeSended = millis();
        // }
      }
      else
      {
        Serial.println("WiFi Disconnected");
      }
    }
}