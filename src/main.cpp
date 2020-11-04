#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <utils.h>

#define macArrayLength 6      //number of position in a mac address

#define testDevices 6         //maximum nodes

#define sendInterval 30000    //sendinterval thingspeak 30 seconds

byte lastAddedNode = 0; //last node which registered at this gateway

unsigned long lastTimeSended = 0;   //to save what time (relative to millis) the last message to thingspeak was sent.

const char *ssid = userWiFiSSID;        //ssid internet access point, configure in utils.h
const char *password = userWiFipassword;//password internet access point, configure in utils.h

const char *serverName = userServerName; // Domain Name with full URL Path for HTTP POST Request, configure in utils.h
String apiKey = userAPIkey;              // Service API Key, configure in utils.h

typedef struct esp_now_message    //this is how data from a node is saved
{
  byte macID[macArrayLength];
  int temperature1;
  int temperature2;
} esp_now_message;

esp_now_message node[testDevices];  //allocate memory for 'testDevices'

// esp_now_message node0, node1, node2, node3, node4, node5;

// esp_now_message *nodes[testDevices] = {&node0, &node1, &node2, &node3, &node4, &node5};

struct communicationTests       //save this parameters for statics
{
  unsigned int missedCalls;
  unsigned long lastContact;
  unsigned long firstRegisterTime;
  unsigned long averageInterval;
  unsigned long totalCalls;
};

struct communicationTests nodeTests[testDevices];

typedef struct struct_message   //this is how data is sended, REVIEW: this is possible on a different way
{
  int temperature1;
  int temperature2;
} struct_message;

struct_message myData;

//prototype functions
byte checkExistingMac(byte *pntToMac);
void putMac(byte *pntToMac, byte selectedRow);
boolean ESPnowconfig(boolean requestedState);

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  digitalWrite(LED_BUILTIN, HIGH);
  // char macStr[18];
  // Serial.print("Packet received from: ");
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.println(macStr);
  byte tempMac[macArrayLength] = {mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]};

  //check if this mac matches with a existing macID
  byte nodeNumber = checkExistingMac(tempMac);
  if (nodeNumber == 255)
  {
    Serial.println("mac not in database yet");
    putMac(tempMac, lastAddedNode);
    nodeTests[lastAddedNode].firstRegisterTime = millis();
    lastAddedNode++;
  }
  else
  {
    long logTime = millis();

    // Serial.printf("Node: %x is known\n", nodeNumber);
    memcpy(&myData, incomingData, sizeof(myData));
    // Serial.print("temperature1 = ");
    // Serial.println(myData.temperature1);
    // Serial.print("temperature2 = ");
    // Serial.println(myData.temperature2);
    if (myData.temperature1 != (node[nodeNumber].temperature1 + 1)) //if we missed a call
    {
      int difference = myData.temperature1 - node[nodeNumber].temperature1;
#define thresholdDifference 60
      if ((difference > thresholdDifference) || ((difference * -1) > thresholdDifference))
      {
        Serial.printf("Differene = %i\n", difference);
        Serial.println("difference to big, so not saved");
      }
      else
      {
        Serial.printf("missed calls: %i\n", difference);
        nodeTests[nodeNumber].missedCalls += difference;
      }
    }
    else
    {
      long interval = logTime - nodeTests[nodeNumber].lastContact;
      nodeTests[nodeNumber].averageInterval += interval;
      nodeTests[nodeNumber].averageInterval = nodeTests[nodeNumber].averageInterval / 2; //average not calculated on a right way
      nodeTests[nodeNumber].totalCalls++;
    }
    nodeTests[nodeNumber].lastContact = logTime;
    node[nodeNumber].temperature1 = myData.temperature1;
    node[nodeNumber].temperature2 = myData.temperature2;
  }
  digitalWrite(LED_BUILTIN, LOW);
}

//input:  requested state of WiFi communication
//output: true if succeed, otherwise false
//effect: sets WiFi communication in requested state
boolean WiFiconfig(boolean requestedState)
{
  if (requestedState)
  {
    // Serial.print("Wi-Fi Channel: ");
    // Serial.println(WiFi.channel());
    WiFi.begin(ssid, password, 3);
    //Serial.println("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(1);
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

//input:  requested state of ESP-NOW communication
//output: true if succeed, otherwise false
//effect: sets ESP-NOW communication in requested state
boolean ESPnowconfig(boolean requestedState)
{
  if (requestedState)
  {
    WiFi.softAP("bullshit", "bulllshit", 1); //REVIEW: use this to change wifi radio channel, but this should be an other function
    WiFi.mode(WIFI_STA);                     //REVIEW: check if this is necessarly
    if (esp_now_init() == ESP_OK)
    {
      if (esp_now_register_recv_cb(OnDataRecv) == ESP_OK) // Once ESPNow is successfully Init, we will register for recv CB to, get recv packer info
      {
        return true;
      }
    }
  }
  else
  {
    if (esp_now_unregister_recv_cb() == ESP_OK)
    {
      if (esp_now_deinit() == ESP_OK)
      {
        return true;
      }
    }
  }
  Serial.println("ESPNOW config fault");
  return false;
}

//input:  pointer to mac address, byte to free row/position
//output: nothing
//effect: puts new device into 'database'
void putMac(byte *pntToMac, byte selectedRow)
{
  for (byte macTestFill = 0; macTestFill < macArrayLength; macTestFill++)
  {
    node[selectedRow].macID[macTestFill] = (*pntToMac + macTestFill);
  }
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
                           "&field1=" + String(nodeTests[0].missedCalls) +
                           "&field2=" + String(nodeTests[0].totalCalls) +
                           "&field3=" + String(nodeTests[1].missedCalls) +
                           "&field4=" + String(nodeTests[1].totalCalls) +
                           "&field5=" + String(nodeTests[2].missedCalls) +
                           "&field6=" + String(nodeTests[2].totalCalls) +
                           "&field7=" + String(nodeTests[3].missedCalls) +
                           "&field8=" + String(nodeTests[3].totalCalls);

  int httpResponseCode = http.POST(httpRequestData);
  // Serial.print("httpRequestData : ");
  // Serial.println(httpRequestData);

  // Serial.print("HTTP Response code: ");
  // Serial.println(httpResponseCode);

  http.end();
  if (httpResponseCode == 200)
  {
    return true;
  }
  return false;
}

//input:  nothing
//output: nothing
//effect: returns statics from active nodes to serial monitor
void printStatics()
{
  for (byte testCount = 0; testCount < lastAddedNode; testCount++)
  {
    Serial.printf("Node %i:\n", testCount);
    Serial.printf("Last contact %lu microseconds ago\n", (millis() - nodeTests[testCount].lastContact));
    Serial.printf("Missed calls %i\n", nodeTests[testCount].missedCalls);
    Serial.printf("Registered %lu seconds ago\n", (((millis() - nodeTests[testCount].firstRegisterTime) / 1000) / 60));
    Serial.printf("Average interval time is %lu microseconds\n", nodeTests[testCount].averageInterval);
    Serial.printf("Total calls received: %lu\n", nodeTests[testCount].totalCalls);
    Serial.printf("----\n");
  }
  Serial.println("-----------------------------");
}


//input:  pointer to mac address, byte array with 'macArrayLength' positions
//output: 255: did not find existing node, 0-254 found existing node
//effect: return existing node number if this exist, if not return 255
byte checkExistingMac(byte *pntToMac)
{
  for (byte currentNode = 0; currentNode < testDevices; currentNode++)
  {
    //Serial.printf("currentNode = %i\n", currentNode);
    for (byte macTestFill = 0; macTestFill < macArrayLength; macTestFill++)
    {
      // Serial.printf("macTestFill = %i\n", macTestFill);
      // Serial.print("incoming: ");
      // Serial.println(*pntToMac + macTestFill);
      // Serial.printf("database: %i\n", nodes[currentNode]->macID[macTestFill]);
      if (node[currentNode].macID[macTestFill] != (*pntToMac + macTestFill))
      {
        //Serial.println("wrong");
        break;
      }
      else if (macTestFill == 5)
      {
        //Serial.println("good in last test");
        return currentNode;
      }
    }
  }
  return 255;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial active");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  WiFi.mode(WIFI_STA); //Set device as a Wi-Fi Station
  ESPnowconfig(true);
}

void loop()
{
  if ((millis() - lastTimeSended) > sendInterval)
  {
    ESPnowconfig(false);
    WiFiconfig(true); //Serial.printf("WiFiconfig = %x", WiFiconfig(true));
    if (WiFi.status() == WL_CONNECTED)
    {
      if (sendToThingSpeak())
      {
        lastTimeSended = millis();
      }
    }
    else
    {
      Serial.println("WiFi Disconnected");
    }
    //printStatics();
    WiFiconfig(false);
    ESPnowconfig(true);
  }
}