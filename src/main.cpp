#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <utils.h>
#include <espnow_settings.h>

#define macArrayLength 6 //number of position in a mac address

#define maximumExternalSensors 6 //maximum nodes

#define sendInterval 10000 //sendinterval thingspeak 30 seconds

#define thresholdDifference 60

byte lastAddedNode = 0; //last node which registered at this gateway

unsigned long lastTimeSended = 0; //to save what time (relative to millis) the last message to thingspeak was sent.

typedef struct ESP_message
{
  uint8_t numberofMeasurements;
  uint8_t intervalTime = defaultIntervalTime; //(INTERVAL_US / 1000000); //intervalTime in milliSeconds
  float pipeTemps1[maximum_samples_espnow];
  float pipeTemps2[maximum_samples_espnow];
} ESP_message;

ESP_message receivedMessage[maximumExternalSensors];

typedef struct esp_now_message //this is how data from a node is saved
{
  byte macID[macArrayLength];
  int temperature1;
  int temperature2;
} esp_now_message;

esp_now_message node[maximumExternalSensors]; //allocate memory for 'maximumExternalSensors'

// esp_now_message node0, node1, node2, node3, node4, node5;

// esp_now_message *nodes[maximumExternalSensors] = {&node0, &node1, &node2, &node3, &node4, &node5};

struct communicationStatics //save this parameters for statics
{
  unsigned int missedCalls;
  unsigned long lastContact;
  unsigned long firstRegisterTime;
  unsigned long averageInterval;
  unsigned long totalCalls;
};

struct communicationStatics nodeStatics[maximumExternalSensors];

typedef struct struct_message //this is how data is sended, REVIEW: this is possible on a different way
{
  int temperature1;
  int temperature2;
} struct_message;

struct_message myData;

//prototype functions
void RollingAverage(unsigned long *avg, unsigned long *new_sample, unsigned long *numberofSamples);
byte checkExistingMac(byte *pntToMac);
void putMac(byte *pntToMac, byte selectedRow);
boolean ESPnowconfig(boolean requestedState);

#define mac_does_not_exist 255

#define debug_sent_ESPNOW_message 1
#define temp_select_device 0
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);

  memcpy(&receivedMessage[temp_select_device], incomingData, sizeof(receivedMessage[temp_select_device]));

#if debug_sent_ESPNOW_message
  printf("[ESPNOW]: receivedMessage[temp_select_device].numberofMeasurements = %u\n", receivedMessage[temp_select_device].numberofMeasurements);
  printf("[ESPNOW]: receivedMessage[temp_select_device].intervalTime = %u\n", receivedMessage[temp_select_device].intervalTime);
  for (uint8_t counter1 = 0; counter1 < receivedMessage[temp_select_device].numberofMeasurements; counter1++)
  {
    printf("[ESPNOW]: receivedMessage[temp_select_device].pipeTemps1[%u] = %f\n", counter1, receivedMessage[temp_select_device].pipeTemps1[counter1]);
    printf("[ESPNOW]: receivedMessage[temp_select_device].pipeTemps2[%u] = %f\n", counter1, receivedMessage[temp_select_device].pipeTemps2[counter1]);
  }
  printf("[ESPNOW]: This should be the message\n");
#endif

  // byte tempMac[macArrayLength] = {mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]};

  // //check if this mac matches with a existing macID
  // byte nodeNumber = checkExistingMac(tempMac);
  // if (nodeNumber == mac_does_not_exist)
  // {
  //   Serial.println("mac not in database yet");
  //   putMac(tempMac, lastAddedNode);
  //   nodeStatics[lastAddedNode].firstRegisterTime = millis();
  //   lastAddedNode++;
  // }
  // else
  // {
  //   long logTime = millis();

  //   // Serial.printf("Node: %x is known\n", nodeNumber);
  //   memcpy(&myData, incomingData, sizeof(myData));
  //   // Serial.print("temperature1 = ");
  //   // Serial.println(myData.temperature1);
  //   // Serial.print("temperature2 = ");
  //   // Serial.println(myData.temperature2);
  //   if (myData.temperature1 != (node[nodeNumber].temperature1 + 1)) //if we missed a call
  //   {
  //     int difference = myData.temperature1 - node[nodeNumber].temperature1;
  //     if ((difference > thresholdDifference) || ((difference * -1) > thresholdDifference))
  //     {
  //       Serial.printf("Differene : %i for node %x\n", difference, nodeNumber);
  //     }
  //     else
  //     {
  //       Serial.printf("missed calls: %i for node %x\n", difference, nodeNumber);
  //       if (difference < 0)
  //       {
  //         difference *= -1;
  //       }
  //       nodeStatics[nodeNumber].missedCalls += difference;
  //     }
  //   }
  //   else
  //   {
  //     unsigned long interval = logTime - nodeStatics[nodeNumber].lastContact;
  //     RollingAverage(&nodeStatics[nodeNumber].averageInterval, &interval, &nodeStatics[nodeNumber].totalCalls);
  //     Serial.printf("Result average: %lu\n", nodeStatics[nodeNumber].averageInterval);
  //     nodeStatics[nodeNumber].totalCalls++;
  //   }
  //   nodeStatics[nodeNumber].lastContact = logTime;
  //   node[nodeNumber].temperature1 = myData.temperature1;
  //   node[nodeNumber].temperature2 = myData.temperature2;
  // }
}

void RollingAverage(unsigned long *avg, unsigned long *new_sample, unsigned long *numberofSamples)
{
  if (*numberofSamples == 0)
  {
    return;
  }
  float tempAverage;
  tempAverage = *avg - (*avg / *numberofSamples);
  tempAverage += *new_sample / *numberofSamples;
  *avg = (unsigned long)tempAverage;
}

//input:  requested state of WiFi communication
//output: true if succeed, otherwise false
//effect: sets WiFi communication in requested state
// boolean WiFiconfig(boolean requestedState)
// {
//   if (requestedState)
//   {
//     // Serial.print("Wi-Fi Channel: ");
//     // Serial.println(WiFi.channel());
//     WiFi.begin(ssid, password, 3);
//     //Serial.println("Connecting");
//     while (WiFi.status() != WL_CONNECTED)
//     {
//       delay(1);
//       //Serial.print(".");
//     }
//     // Serial.println("");
//     // Serial.print("Connected to WiFi network with IP Address: ");
//     // Serial.println(WiFi.localIP());
//     if (WiFi.status() == WL_CONNECTED)
//     {
//       // Serial.print("Wi-Fi Channel: ");
//       // Serial.println(WiFi.channel());
//       return true;
//     }
//   }
//   else
//   {
//     if (WiFi.disconnect(true, true) == 1)
//     {
//       return true;
//     }
//   }
//   return false;
// }

//input:  requested state of ESP-NOW communication
//output: true if succeed, otherwise false
//effect: sets ESP-NOW communication in requested state
boolean ESPnowconfig(boolean requestedState)
{
  if (requestedState)
  {
    //digitalWrite(measureTimePin1, LOW);
    //WiFi.softAP("bullshit", "bulllshit", espnow_channel); //REVIEW: use this to change wifi radio channel, but this should be an other function
    WiFi.mode(WIFI_STA); //REVIEW: check if this is necessary
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

//input:  nothing
//output: nothing
//effect: returns statics from active nodes to serial monitor
void printStatics()
{
  for (byte testCount = 0; testCount < lastAddedNode; testCount++)
  {
    Serial.printf("Node %i:\n", testCount);
    Serial.printf("Last contact %lu microseconds ago\n", (millis() - nodeStatics[testCount].lastContact));
    Serial.printf("Missed calls %i\n", nodeStatics[testCount].missedCalls);
    Serial.printf("Registered %lu seconds ago\n", (((millis() - nodeStatics[testCount].firstRegisterTime) / 1000) / 60));
    Serial.printf("Average interval time is %lu microseconds\n", nodeStatics[testCount].averageInterval);
    Serial.printf("Total calls received: %lu\n", nodeStatics[testCount].totalCalls);
    Serial.printf("----\n");
  }
  Serial.println("-----------------------------");
}

//input:  pointer to mac address, byte array with 'macArrayLength' positions
//output: "mac_does_not_exist": did not find existing node, 0-254 found existing node
//effect: return existing node number if this exist, if not return "mac_does_not_exist"
byte checkExistingMac(byte *pntToMac)
{
  for (byte currentNode = 0; currentNode < maximumExternalSensors; currentNode++)
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
  return mac_does_not_exist;
}

boolean startRemember = false;

void setup()
{
  Serial.begin(115200);
  Serial.printf("Device is started\n");
  //WiFi.mode(WIFI_MODE_STA);
  //Serial.print("MAC address: ");
  //Serial.println(WiFi.macAddress());
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  while (1)
  {
    delay(1000);
    //esp_deep_sleep_start();
  }
  printf("ESPnowconfig return: %d\n", ESPnowconfig(true));
  lastTimeSended = 1; //to execute function below direct
}
void loop()
{
  if (((millis() - lastTimeSended) > sendInterval)) // && buttonState1)
  {
    printStatics();
    //ESPnowconfig(false);
    //WiFiconfig(true); //Serial.printf("WiFiconfig = %x", WiFiconfig(true));
    if (WiFi.status() == WL_CONNECTED)
    {
      if (true) //sendAction here
      {
        lastTimeSended = millis();
      }
    }
    else
    {
      Serial.println("WiFi Disconnected");
    }
    //printStatics();
    //WiFiconfig(false);
    //ESPnowconfig(true);
  }
}