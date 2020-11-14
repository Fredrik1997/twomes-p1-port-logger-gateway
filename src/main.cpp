// String key1="s97931a53932490f48a34c524c8f6ca22db80674869b3eaa42b7fdc25065aec86";
// ArduinoJWT giv= ArduinoJWT("sc0b52aa215ded9c92e0db5dcedcc4843e4fb565552e9910a5398d08f3a492f43");
// String msg="encode and decode this";
// String payload;

// void setup() {
//  Serial.begin(9600);

//  jwt.setPSK(key1);

// }

// void loop() {
//   // put your main code here, to run repeatedly:
//    Serial.println(msg);
//    payload = giv.encodeJWT(msg);
//    Serial.println(payload);
//    delay(2000);
//    payload = giv.decodeJWT(key1, payload);
//    Serial.println(payload);
//    delay(2000);

// }

#include <WiFi.h>
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <utils.h>
#include <ArduinoJWT.h>
#include <ArduinoJson.h>

const char *ssid = userWiFiSSID;         //ssid internet access point, configure in utils.h
const char *password = userWiFipassword; //password internet access point, configure in utils.h

unsigned long lastTimeSended = 0; //to save what time (relative to millis) the last message to thingspeak was sent.

#define sendInterval 10000

//JWT things
String key1 = "s97931a53932490f48a34c524c8f6ca22db80674869b3eaa42b7fdc25065aec86";
ArduinoJWT jwt = ArduinoJWT(key1);
String msg = "{'A'idA:AFF:FF:FF:FF:FFA,AdataSpecA:{AlastTimeA:A2020-10-13 14:42:24A,AintervalA:10,AtotalA:6},AdataA:{AroomTempA:[20,21,22,23,24,25]}}";
String payload;

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

#define requiredMemorySpaceMessages 250
char sendMessage[requiredMemorySpaceMessages];

void jsonEncoder()
{
  StaticJsonDocument<requiredMemorySpaceMessages> jsonDocEncoder;

  jsonDocEncoder["id"] = "FF:FF:FF:FF:FF";

  JsonObject dataSpec = jsonDocEncoder.createNestedObject("dataSpec");
  JsonObject data = jsonDocEncoder.createNestedObject("data");
  //dataSpec.
  dataSpec.getOrAddMember("lastTime");
  dataSpec["lastTime"] = "2020-10-13 14:42:24";
  dataSpec.getOrAddMember("interval");
  dataSpec["interval"] = 10;
  dataSpec.getOrAddMember("total");
#define testTotal 6
  dataSpec["total"] = testTotal;

  JsonArray data_roomTemp = data.createNestedArray("roomTemp");
  float testValues[10] = {20, 21, 22, 23, 24, 25};
  for (byte testCount = 0; testCount < testTotal; testCount++)
  {
    data_roomTemp.add(testValues[testCount]);
  }

  // Serial.print("Result: ");
  // serializeJson(jsonDocEncoder, Serial);
  //char jsonCharTemp[requiredMemorySpaceMessages];
  serializeJson(jsonDocEncoder, sendMessage);
  //strcpy(sendMessage, "packet:");    //sendMessage starten met packet:
  //strcpy(sendMessage, jsonCharTemp); //daarna jsonChar invoegen
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial active");
  jsonEncoder();
  jwt.setPSK(key1);

  //WiFi.mode(WIFI_STA); //Set device as a Wi-Fi Station
  lastTimeSended = sendInterval; //to execute direct after boot
}
boolean once = true;
void loop()
{
  if (((millis() - lastTimeSended) > sendInterval))
  {
    jsonEncoder();
    Serial.print("encoded JSON: ");
    Serial.println(sendMessage);
    unsigned int count = 0;
    while (sendMessage[count] != 0)
    {
      unsigned int temp = sendMessage[count];
      if (temp == 34)
      {
        temp = 39;
        sendMessage[count] = temp;
      }
      count++;
    }
    Serial.print("converted encoded JSON: ");
    Serial.println(sendMessage);
    // msg = sendMessage;
    // Serial.print("message: ");
    // Serial.println(msg);
    // payload = jwt.encodeJWT(msg);
    // Serial.print("encoded payload: ");
    // Serial.println(payload);
    //strcpy((char)msg, sendMessage);
    //strcpy(sendMessage, "{"id":piet)");
    //char testDing[20] = "testDing";
    char testPayload[requiredMemorySpaceMessages];
    jwt.encodeJWT(sendMessage, testPayload);

    Serial.print("encoded payload2: ");
    Serial.println(testPayload);
    Serial.println("end");
    // delay(1000);
    // String decoded;
    // decoded = jwt.decodeJWT(key1, payload);
    // Serial.print("decoded payload: ");
    // Serial.println(decoded);
    lastTimeSended = millis();

    // WiFiconfig(true);
    // if (WiFi.status() == WL_CONNECTED)
    // {
    //   // if (sendToThingSpeak())
    //   // {
    //   lastTimeSended = millis();
    //   // }
    // }
    // else
    // {
    //   Serial.println("WiFi Disconnected");
    // }
  }
}