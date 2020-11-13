#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <utils.h>

#define macArrayLength 6 //number of position in a mac address

#define testDevices 6 //maximum nodes

#define sendInterval 1000 //sendinterval thingspeak 30 seconds

#define thresholdDifference 60

#define measureTimePin1 12 //for debugging with scope
#define measureTimePin2 27 //for debugging with scope
#define button1 14         //for debugging with button on breadboard

byte lastAddedNode = 0; //last node which registered at this gateway

unsigned long lastTimeSended = 0; //to save what time (relative to millis) the last message to thingspeak was sent.

const char *ssid = userWiFiSSID;         //ssid internet access point, configure in utils.h
const char *password = userWiFipassword; //password internet access point, configure in utils.h

const char *serverName = userServerName; // Domain Name with full URL Path for HTTP POST Request, configure in utils.h
String apiKey = userAPIkey;              // Service API Key, configure in utils.h

//input:  requested state of WiFi communication
//output: true if succeed, otherwise false
//effect: sets WiFi communication in requested state
boolean WiFiconfig(boolean requestedState)
{
  if (requestedState)
  {
    // Serial.print("Wi-Fi Channel: ");
    // Serial.println(WiFi.channel());
    digitalWrite(measureTimePin1, HIGH);

    if (WiFi.status() != WL_CONNECTED)
    {
      digitalWrite(measureTimePin2, HIGH);
      WiFi.begin(ssid, password);
      digitalWrite(measureTimePin2, LOW);
    }

    Serial.println("Connecting");
    unsigned int thresholdCounter = 0;
#define thresholdValue 2000
    while (WiFi.status() != WL_CONNECTED && thresholdCounter < thresholdValue)
    {
      delay(1);
      thresholdCounter++;
      //Serial.print(".");
    }
    digitalWrite(measureTimePin1, LOW);
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

//input:
//output: false if something wrong, true if succeed
//effect: sends defined parameters to ThingSpeak API
boolean sendToThingSpeak()
{
  HTTPClient http;
  if (!http.begin(serverName))
  {
    return false;
  }
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String httpRequestData = "api_key=" + apiKey +
                           "&field1=" + String(random(30)) +
                           "&field2=" + String(random(30)) +
                           "&field3=" + String(random(30)) +
                           "&field4=" + String(random(30)) +
                           "&field5=" + String(random(30)) +
                           "&field6=" + String(random(30)) +
                           "&field7=" + String(random(30)) +
                           "&field8=" + String(random(30));

  int httpResponseCode = http.POST(httpRequestData);
  // Serial.print("httpRequestData : ");
  // Serial.println(httpRequestData);

  // Serial.print("HTTP Response code: ");
  // Serial.println(httpResponseCode);

  http.end();
  if (httpResponseCode == 200)
  {
    Serial.println("send succeed");
    return true;
  }
  return false;
}

void setup()
{
  pinMode(measureTimePin1, OUTPUT);
  pinMode(measureTimePin2, OUTPUT);
  pinMode(button1, INPUT);

  Serial.begin(115200);
  Serial.println("Serial active");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  //WiFi.mode(WIFI_STA); //Set device as a Wi-Fi Station
  //ESPnowconfig(true);
}
boolean buttonState1 = false;
void loop()
{

  if (((millis() - lastTimeSended) > sendInterval) && buttonState1) //((millis() - lastTimeSended) > sendInterval) &&
  {
    //printStatics();
    if (buttonState1)
    {
      //ESPnowconfig(false);
      WiFiconfig(true); //Serial.printf("WiFiconfig = %x", WiFiconfig(true));
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
      //printStatics();
       WiFiconfig(false);
      //ESPnowconfig(true);
    }
  }
  if ((digitalRead(button1) == LOW))
  {
    byte deBounce = 0;
#define deBounceThreshold 50
    while (deBounce < deBounceThreshold)
    {
      if (digitalRead(button1) == LOW)
      {
        deBounce = 0;
      }
      else
      {
        deBounce++;
      }
      delay(1);
    }
    lastTimeSended = 0;
    buttonState1 = !buttonState1;
    Serial.printf("toggle buttonState1 to %x\n", buttonState1);
  }
}